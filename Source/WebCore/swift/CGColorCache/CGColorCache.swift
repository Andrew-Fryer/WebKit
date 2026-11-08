// Copyright (C) 2026 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

// The recency order of cachedCGColor()'s 32-entry cache (Source/WebCore/platform/graphics/cg/ColorCG.cpp).
//
// C++ owns the entries (a Color and its RetainPtr<CGColorRef>, stored in slots that never move) and two POD arrays
// that this code keeps in least- to most-recently-used order:
//   keys[i]  = the bits of the i-th entry's Color (a Color is a single uint64_t, m_colorAndFlags);
//   order[i] = that entry's slot | outOfLineFlag if its Color is out of line.
// The policy is TinyLRUCache's (Source/WTF/wtf/TinyLRUCache.h): scan from the most recently used end, make a hit the
// most recently used entry, and on a miss append while there is room, else replace the least recently used entry.
//
// Equality is Color's operator==. An inline Color equals another Color exactly when their bits are equal (the bits
// include the out-of-line flag, so an inline key never matches an out-of-line entry). Two out-of-line Colors are equal
// when their color spaces, flags and components are (with NaN equal to NaN and -0 to +0), which their bits don't
// decide because equal Colors can have different OutOfLineComponents, so for an out-of-line key C++ compares each
// out-of-line entry.

enum CGColorCache {
    // These must match CGColorCacheSwiftBoundary.h.
    @inline(__always) static var slotMask: UInt8 { 0x7f }
    @inline(__always) static var outOfLineFlag: UInt8 { 0x80 }
    @inline(__always) static var missFlag: UInt32 { 0x100 }

    // Both lookups return the slot of the entry equal to the key, or with missFlag the slot C++ must fill with the key
    // (a new slot, or the least recently used entry's). Either way, that entry is now the most recently used one.

    // An inline key: equal exactly to the entry with the same bits.
    @inline(__always)
    static func lookup(keys: inout MutableSpan<UInt64>, order: inout MutableSpan<UInt8>, size: inout Int, keyBits: UInt64)
        -> UInt32
    {
        let count = checkedCount(keys: keys, order: order, size: size)
        var i = count
        while i > 0 {
            i &-= 1
            if keys[i] == keyBits {
                let entry = order[i]
                if i != count &- 1 {
                    makeMostRecentlyUsed(keys: &keys, order: &order, index: i, count: count, key: keyBits, entry: entry)
                }
                return UInt32(entry & slotMask)
            }
        }
        return insert(keys: &keys, order: &order, size: &size, count: count, keyBits: keyBits, flags: 0)
    }

    // An out-of-line key: only out-of-line entries can be equal, and C++ decides which (Color::operator==).
    @inline(__always)
    static func lookupOutOfLine(keys: inout MutableSpan<UInt64>, order: inout MutableSpan<UInt8>, size: inout Int,
        keyBits: UInt64, outOfLineEntryEquals: (UInt8) -> Bool) -> UInt32
    {
        let count = checkedCount(keys: keys, order: order, size: size)
        var i = count
        while i > 0 {
            i &-= 1
            let entry = order[i]
            if entry & outOfLineFlag != 0 && outOfLineEntryEquals(entry & slotMask) {
                if i != count &- 1 {
                    // The entry's own bits, not the key's: they hold a different OutOfLineComponents pointer.
                    makeMostRecentlyUsed(keys: &keys, order: &order, index: i, count: count, key: keys[i], entry: entry)
                }
                return UInt32(entry & slotMask)
            }
        }
        return insert(keys: &keys, order: &order, size: &size, count: count, keyBits: keyBits, flags: outOfLineFlag)
    }

    @inline(__always)
    static func checkedCount(keys: borrowing MutableSpan<UInt64>, order: borrowing MutableSpan<UInt8>, size: Int) -> Int {
        let capacity = keys.count
        precondition(order.count == capacity && capacity > 0 && capacity <= Int(slotMask) + 1)
        precondition(size >= 0 && size <= capacity)
        return size
    }

    // A miss: the key becomes the most recently used entry, in a new slot while there is room, else in the least
    // recently used entry's slot.
    @inline(__always)
    static func insert(keys: inout MutableSpan<UInt64>, order: inout MutableSpan<UInt8>, size: inout Int, count: Int,
        keyBits: UInt64, flags: UInt8) -> UInt32
    {
        if count == keys.count {
            let slot = order[0] & slotMask
            makeMostRecentlyUsed(keys: &keys, order: &order, index: 0, count: count, key: keyBits, entry: slot | flags)
            return UInt32(slot) | missFlag
        }
        let slot = UInt8(truncatingIfNeeded: count)
        keys[count] = keyBits
        order[count] = slot | flags
        size = count &+ 1
        return UInt32(slot) | missFlag
    }

    // Drops position `index`, moves the more recently used entries down by one, and stores (key, entry) at the end.
    @inline(__always)
    static func makeMostRecentlyUsed(keys: inout MutableSpan<UInt64>, order: inout MutableSpan<UInt8>, index: Int,
        count: Int, key: UInt64, entry: UInt8)
    {
        let last = count &- 1
        var j = index
        while j < last {
            keys[j] = keys[j &+ 1]
            order[j] = order[j &+ 1]
            j &+= 1
        }
        keys[last] = key
        order[last] = entry
    }
}
