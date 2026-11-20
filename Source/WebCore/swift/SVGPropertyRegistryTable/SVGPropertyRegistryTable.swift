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

// Memory-safe port of the attribute-name table of SVGPropertyOwnerRegistryBase<OwnerType, BaseTypes...>
// (svg/properties/SVGPropertyOwnerRegistry.h). Every OwnerType has one table mapping an attribute's QualifiedName to the
// SVGMemberAccessor<OwnerType> singleton of the property it names; it replaces the original
// HashMap<QualifiedName, const SVGMemberAccessor<OwnerType>*, SVGAttributeHashTranslator>. Swift defines the table's
// representation and every operation that builds or searches it (find, add, grow); C++ only allocates its words and,
// to enumerate, reads the entry array whose layout the boundary header publishes (see below).
//
// Keys. SVGAttributeHashTranslator::equal is QualifiedName::matches: the same QualifiedNameImpl, or the same localName
// and namespaceURI AtomStringImpls. Equal impls imply equal components, so two names are equal exactly when their
// (localName impl, namespaceURI impl) pairs are; the translator's hash ignores the prefix for the same reason. The table
// is therefore keyed on that pair of opaque handles (AtomStringImpl addresses; namespaceURI is 0 for nullAtom). Every
// handle is opaque: Swift compares and stores them, never dereferences them. The registered names are immortal statics,
// so their AtomStringImpls stay alive and their addresses can't be reused by another string.
//
// Values. An entry keeps the accessor handle (the address of the SVGMemberAccessor singleton, never 0) and the key
// handle (the address of the immortal static QualifiedName it was registered with, which C++ gives back to
// enumerateRecursively()'s functor).
//
// Layout, in 64-bit words (an empty span is the empty table):
//   word 0: capacity, the number of hash slots, a power of two >= minimumCapacity;
//   word 1: count, the number of entries, <= capacity / 2;                 (published: CountWord)
//   words 2 ..< 2 + 4 * (capacity / 2): the entries in registration order, (localName, namespaceURI, key, accessor);
//                                 the first count are used.               (published: EntriesWord, EntryWords,
//                                                                          EntryKeyWord, EntryAccessorWord)
//   then 3 * capacity words of hash slots, open addressing with linear probing. A slot is (localName, namespaceURI,
//   accessor) and is empty when accessor == 0.
// The load factor is at most 1/2, so a probe always reaches an empty slot. A find reads one slot per probe, all from
// one cache line in the common case. C++ enumerates the entry array directly (a boundary call per OwnerType level cost
// more than the original HashMap walk); that part of the layout is a published contract, mirrored by the constants
// below and checked by static_asserts in the header. The original HashMap enumerated in hash order, which follows
// AtomStringImpl addresses and so was unspecified; registration order is one of the orders it could produce.
//
// Duplicates. add() keeps the existing entry, like HashMap::add.
// Growth. When an add doesn't fit, nothing is changed and the number of words needed is returned. C++ allocates that
// many words, rehash() fills them with the old table's entries (in registration order), and C++ adds again.
// Every word index is bounds-checked, so damaged storage traps instead of escaping.

enum SVGPropertyRegistryTable {
    static var headerWords: Int { 2 }
    static var countWord: Int { 1 } // WebCoreSwift_SVGPropertyRegistryTable_CountWord
    static var entriesWord: Int { 2 } // WebCoreSwift_SVGPropertyRegistryTable_EntriesWord
    static var entryWords: Int { 4 } // WebCoreSwift_SVGPropertyRegistryTable_EntryWords
    static var entryKeyWord: Int { 2 } // WebCoreSwift_SVGPropertyRegistryTable_EntryKeyWord
    static var entryAccessorWord: Int { 3 } // WebCoreSwift_SVGPropertyRegistryTable_EntryAccessorWord
    static var slotWords: Int { 3 }
    static var minimumCapacity: Int { 8 }

    @inline(__always)
    static func slotsStart(capacity: Int) -> Int {
        entriesWord &+ (capacity / 2) &* entryWords
    }

    @inline(__always)
    static func wordCount(capacity: Int) -> Int {
        slotsStart(capacity: capacity) &+ capacity &* slotWords
    }

    // Fibonacci hashing: multiply, then take the slot from the top log2(capacity) bits of the product. The handles are
    // addresses of names allocated at a small fixed stride (the generated static name tables, or consecutive heap
    // atoms), and only the top bits of the product spread such a progression evenly; its middle bits barely move from
    // one name to the next (with bits 32 and up, names 32 bytes apart all fell into one or two slots, so every table was
    // one long cluster). capacity is a power of two >= minimumCapacity, so the shift is in 1 ..< 64 and the slot is
    // < capacity.
    @inline(__always)
    static func slotIndex(_ localName: UInt64, _ namespaceURI: UInt64, _ capacity: Int) -> Int {
        let hash = (localName &* 0x9E37_79B9_7F4A_7C15) ^ (namespaceURI &* 0xC2B2_AE3D_27D4_EB4F)
        return Int(truncatingIfNeeded: hash &>> UInt64(truncatingIfNeeded: capacity.leadingZeroBitCount &+ 1))
    }

    // The accessor registered for (localName, namespaceURI), or 0 if there is none.
    @inline(__always)
    static func find(_ table: Span<UInt64>, _ localName: UInt64, _ namespaceURI: UInt64) -> UInt64 {
        if table.isEmpty {
            return 0
        }
        let capacity = Int(truncatingIfNeeded: table[0])
        let mask = capacity &- 1
        let slots = slotsStart(capacity: capacity)
        var index = slotIndex(localName, namespaceURI, capacity)
        for _ in 0 ..< capacity {
            let base = slots &+ index &* slotWords
            let accessor = table[base &+ 2]
            if accessor == 0 {
                return 0
            }
            if table[base] == localName && table[base &+ 1] == namespaceURI {
                return accessor
            }
            index = (index &+ 1) & mask
        }
        return 0
    }

    // Adds (localName, namespaceURI) -> (key, accessor) unless the name is already there (then the table is unchanged,
    // like HashMap::add). Returns 0, or, if the entry doesn't fit, the number of words the grown table needs.
    static func add(_ table: inout MutableSpan<UInt64>, _ localName: UInt64, _ namespaceURI: UInt64, _ key: UInt64, _ accessor: UInt64) -> Int {
        precondition(localName != 0 && key != 0 && accessor != 0)
        if table.isEmpty {
            return wordCount(capacity: minimumCapacity)
        }
        let capacity = Int(truncatingIfNeeded: table[0])
        let count = Int(truncatingIfNeeded: table[countWord])
        precondition(capacity >= minimumCapacity && capacity & (capacity &- 1) == 0 && table.count == wordCount(capacity: capacity))
        precondition(count >= 0 && count <= capacity / 2)
        let mask = capacity &- 1
        let slots = slotsStart(capacity: capacity)
        var index = slotIndex(localName, namespaceURI, capacity)
        while true {
            let base = slots &+ index &* slotWords
            if table[base &+ 2] == 0 {
                break
            }
            if table[base] == localName && table[base &+ 1] == namespaceURI {
                return 0
            }
            index = (index &+ 1) & mask
        }
        if (count &+ 1) &* 2 > capacity {
            return wordCount(capacity: capacity &* 2)
        }
        let base = slots &+ index &* slotWords
        table[base] = localName
        table[base &+ 1] = namespaceURI
        table[base &+ 2] = accessor
        let entry = entriesWord &+ count &* entryWords
        table[entry] = localName
        table[entry &+ 1] = namespaceURI
        table[entry &+ entryKeyWord] = key
        table[entry &+ entryAccessorWord] = accessor
        table[countWord] = UInt64(count &+ 1)
        return 0
    }

    // Initializes every word of `grown` (whose count add() returned) as a table holding the entries of `table`, in the
    // same registration order.
    static func rehash(_ table: Span<UInt64>, _ grown: inout MutableSpan<UInt64>) {
        let capacity = (grown.count &- headerWords) / (slotWords &+ entryWords / 2)
        precondition(capacity >= minimumCapacity && capacity & (capacity &- 1) == 0 && grown.count == wordCount(capacity: capacity))
        for i in 0 ..< grown.count {
            grown[i] = 0
        }
        grown[0] = UInt64(capacity)
        if table.isEmpty {
            return
        }
        let count = Int(truncatingIfNeeded: table[countWord])
        precondition(count >= 0 && count <= capacity / 2)
        for i in 0 ..< count {
            let entry = entriesWord &+ i &* entryWords
            let status = add(&grown, table[entry], table[entry &+ 1], table[entry &+ entryKeyWord], table[entry &+ entryAccessorWord])
            precondition(status == 0)
        }
    }
}
