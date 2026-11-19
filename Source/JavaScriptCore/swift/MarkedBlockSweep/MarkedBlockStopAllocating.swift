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

// The Resumable path of MarkedBlock::Handle::stopAllocating (heap/MarkedBlock.cpp): rebuild the block's
// newlyAllocated bitmap as "every cell except those still on the allocator's free list", zapping the free cells
// of a block with destructors so that the next sweep does not destroy them.
//
// Here the payload is a span of atoms as four 32-bit words, the widths HeapCell::zap writes: [0] the header word,
// [2] the low half of the cell's second 8 bytes (FreeCell::scrambledBits in a free cell).
typealias MarkedBlockAtomWords = InlineArray<4, UInt32>

/// HeapCell::ZapReason::StopAllocating (static_asserted in heap/MarkedBlock.cpp).
enum HeapCellZapReason {
    static var stopAllocating: UInt32 { 2 }
}

// The free list is the FreeList's raw state, as byte offsets into the payload:
// the current interval [intervalStart, intervalEnd), partly consumed or empty, and nextInterval, the first
// FreeCell not yet entered, where an odd offset is the sentinel (FreeList::isSentinel: the payload is 16-byte
// aligned, so the pointer is odd iff the offset is).

/// The bits 0, n, 2n, ... of a 64-bit word (n > 0): a word's share of {start, start + n, ...} is this mask shifted
/// left by the offset of the first such bit in the word.
@inline(__always)
func everyNthBit(_ n: Int) -> UInt64 {
    var mask: UInt64 = 1
    var shift = n
    while shift < 64 {
        mask |= mask &<< UInt64(shift)
        shift &*= 2
    }
    return mask
}

/// `newlyAllocated.clearAll(); newlyAllocated.setEachNthBit(n, start, end)`: every word is assigned the bits of
/// {start, start + n, ...} below `end` that fall in it (WTF::BitSet::setEachNthBit sets them one at a time).
/// `pattern` is everyNthBit(n).
@inline(__always)
func assignEachNthBit(_ words: inout MutableSpan<UInt64>, _ n: Int, _ pattern: UInt64, _ start: Int, _ end: Int) {
    var next = start // the first bit of the sequence not yet placed; never below the current word's first bit
    for w in words.indices {
        let base = w &* 64
        var mask: UInt64 = 0
        if next < end && next < base &+ 64 {
            mask = pattern &<< UInt64(truncatingIfNeeded: next &- base)
            next &+= mask.nonzeroBitCount &* n
            if end < base &+ 64 {
                mask &= (1 &<< UInt64(truncatingIfNeeded: end &- base)) &- 1
            }
        }
        words[w] = mask
    }
}

/// WTF::BitSet::clearEachNthBit(n, start, end): clears the bits of {start, start + n, ...} below `end`, a word at a
/// time (0 <= start). `pattern` is everyNthBit(n).
@inline(__always)
func clearEachNthBit(_ words: inout MutableSpan<UInt64>, _ n: Int, _ pattern: UInt64, _ start: Int, _ end: Int) {
    var next = start
    while next < end {
        let w = next &>> 6
        let base = w &<< 6
        var mask = pattern &<< UInt64(truncatingIfNeeded: next & 63)
        if end &- base < 64 {
            // The last word: only the bits below `end`.
            mask &= (1 &<< UInt64(truncatingIfNeeded: end &- base)) &- 1
            words[w] &= ~mask
            return
        }
        words[w] &= ~mask
        next &+= mask.nonzeroBitCount &* n
    }
}

/// `newlyAllocated.clearAll(); newlyAllocated.setEachNthBit(atomsPerCell, startAtom, endAtom)`, then for each
/// free-list interval: zap its cells with HeapCell::StopAllocating if `needsZapping`, and clear their bits.
func stopAllocatingMarkedBlock(_ atoms: inout MutableSpan<MarkedBlockAtomWords>, newlyAllocated: inout MutableSpan<UInt64>,
                               startAtom: UInt32, atomsPerCell: UInt32, endAtom: UInt32,
                               intervalStart: Int, intervalEnd: Int, nextInterval: Int, secret: UInt64, needsZapping: Bool) {
    let startAtom = Int(startAtom)
    let atomsPerCell = Int(atomsPerCell)
    let endAtom = Int(endAtom)
    precondition(atomsPerCell > 0 && startAtom <= endAtom && endAtom &- startAtom == atoms.count)
    precondition(endAtom <= newlyAllocated.count &* 64) // BitSet's ASSERT(end <= bitSetSize)
    let pattern = everyNthBit(atomsPerCell)

    assignEachNthBit(&newlyAllocated, atomsPerCell, pattern, startAtom, endAtom)

    // FreeList::forEachInterval. A well-formed list visits each dead cell once, so more intervals than atoms
    // means a cycle, where the C++ would loop forever.
    var start = intervalStart
    var end = intervalEnd
    var next = nextInterval
    var intervals = 0
    while true {
        if start < end {
            // Byte offsets in the payload; the cells are atomsPerCell atoms apart from `start`.
            precondition(start >= 0 && start & 15 == 0 && end <= atoms.count &* 16)
            let firstAtom = start &>> 4
            if needsZapping {
                let endAtomOfInterval = (end &+ 15) &>> 4 // the first atom not below `end`: `cell < intervalEnd`
                var a = firstAtom
                while a < endAtomOfInterval {
                    atoms[a][0] = 0
                    atoms[a][2] = HeapCellZapReason.stopAllocating
                    a &+= atomsPerCell
                }
            }
            // candidateAtomNumber(p) = (p - block) / atomSize, where the payload starts at atom startAtom.
            clearEachNthBit(&newlyAllocated, atomsPerCell, pattern, startAtom &+ firstAtom, startAtom &+ end &>> 4)
        }
        if next & 1 != 0 {
            break
        }
        intervals &+= 1
        precondition(intervals <= atoms.count)
        // FreeCell::advance: decode the FreeCell at `next`.
        precondition(next >= 0 && next & 15 == 0)
        let cell = next &>> 4
        let bits = ((UInt64(atoms[cell][3]) &<< 32) | UInt64(atoms[cell][2])) ^ secret
        start = next
        end = next &+ Int(UInt32(truncatingIfNeeded: bits &>> 32))
        next = next &+ Int(Int32(truncatingIfNeeded: bits))
    }
}
