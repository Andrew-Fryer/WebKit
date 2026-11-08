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


// Memory-safe port of the CSS value keyword lookup of css/parser/CSSPropertyParser.cpp: cssValueKeywordID(StringView)
// (via its cssValueKeywordID<CharacterType>(std::span) template and isAppleLegacyCSSValueKeyword) and of the gperf
// lookup it ends in, findCSSValueKeyword() / CSSValueKeywordsHash::in_word_set() in the generated CSSValueKeywords.cpp.
//
// Semantics of the original: a NUL or non-ASCII code unit anywhere gives CSSValueID::Invalid (0). Otherwise the
// characters are ASCII-lowercased; if the result starts with "-apple-" and the rest starts with none of "system",
// "pay" and "wireless", "-apple-" becomes "-webkit-" (one character longer). The result is looked up exactly in the
// lowercase keyword names (the generator lowercases them for gperf); no match gives 0.
//
// Instead of gperf, a keyword table built once from the keyword names: open addressing with linear probing over 4096
// UInt16 slots holding CSSValueIDs (0 = empty). C++ owns the storage: the names as one byte pool with offsets (name of
// id i = pool[offsets[i] ..< offsets[i + 1]], id 0 is the empty Invalid name) and the slots; buildTable() fills them.
// Membership and ids are exactly those of the names because every candidate is compared in full.
//
// Keys are processed 8 code units at a time, as one 64-bit word of bytes: unaligned loads (the last partial word from
// overlapping loads inside the key), a SWAR check that every unit is in 1...0x7F, UTF-16 units narrowed to bytes,
// SWAR ASCII lowercasing, and a multiplicative hash per word. Candidates are compared word by word against the pool.
// The -apple- rewrite (rare) builds the rewritten key in a fixed-size local buffer, as the C++ does, and looks it up.
// No allocation, no reference counting, no traps beyond Span bounds checks and the table-shape preconditions.

// A key, read as words of 8 code units narrowed to bytes (little-endian: unit i in byte i).
protocol CSSValueKeywordKey: ~Escapable {
    var count: Int { get }
    // Units [offset, offset + 8) (offset + 8 <= count), and whether every one of them is in 1...0x7F.
    func word(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool)
    // Units [offset, count) (1 to 8 of them), zero above, and whether every one of them is in 1...0x7F.
    // Only reads units of the key (overlapping reads when the key is short).
    func tailWord(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool)
    // The unit at `index`, zero-extended.
    func unit(_ index: Int) -> UInt32
}

struct CSSValueKeywordLatin1Key: CSSValueKeywordKey, ~Escapable {
    let bytes: RawSpan

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    var count: Int {
        @inline(__always) get { bytes.byteCount }
    }

    @inline(__always)
    func word(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool) {
        let word = bytes.load(fromByteOffset: offset, as: UInt64.self)
        return (word, CSSValueKeywordTable.isASCIIWithoutNUL(word, remaining: 8))
    }

    @inline(__always)
    func tailWord(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool) {
        let word = CSSValueKeywordTable.tailWord(bytes, start: 0, count: count, offset: offset)
        return (word, CSSValueKeywordTable.isASCIIWithoutNUL(word, remaining: count &- offset))
    }

    @inline(__always)
    func unit(_ index: Int) -> UInt32 {
        UInt32(bytes.load(fromByteOffset: index, as: UInt8.self))
    }
}

struct CSSValueKeywordUTF16Key: CSSValueKeywordKey, ~Escapable {
    let bytes: RawSpan // 2 bytes per code unit, any alignment.

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    var count: Int {
        @inline(__always) get { bytes.byteCount &>> 1 }
    }

    // Four units (16-bit lanes) in 1...0x7F.
    @inline(__always)
    static func isASCIIWithoutNUL(_ lanes: UInt64) -> Bool {
        let hasZeroLane = (lanes &- 0x0001_0001_0001_0001) & ~lanes & 0x8000_8000_8000_8000
        return (lanes & 0xFF80_FF80_FF80_FF80) | hasZeroLane == 0
    }

    // Four units below 0x100 (high bytes zero) as four bytes in the low half.
    @inline(__always)
    static func narrow(_ lanes: UInt64) -> UInt64 {
        let pairs = (lanes | (lanes &>> 8)) & 0x0000_FFFF_0000_FFFF
        return (pairs | (pairs &>> 16)) & 0xFFFF_FFFF
    }

    @inline(__always)
    func eightUnits(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool) {
        let low = bytes.load(fromByteOffset: offset &* 2, as: UInt64.self)
        let high = bytes.load(fromByteOffset: offset &* 2 &+ 8, as: UInt64.self)
        return (Self.narrow(low) | (Self.narrow(high) &<< 32), Self.isASCIIWithoutNUL(low) && Self.isASCIIWithoutNUL(high))
    }

    @inline(__always)
    func word(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool) {
        eightUnits(at: offset)
    }

    @inline(__always)
    func tailWord(at offset: Int) -> (bytes: UInt64, isASCIIWithoutNUL: Bool) {
        let count = count
        let remaining = count &- offset
        if count >= 8 {
            let last = eightUnits(at: count &- 8)
            return (last.bytes &>> UInt64(truncatingIfNeeded: 64 &- 8 &* remaining), last.isASCIIWithoutNUL)
        }
        if remaining >= 4 {
            let low = bytes.load(fromByteOffset: 0, as: UInt64.self)
            let high = bytes.load(fromByteOffset: (count &- 4) &* 2, as: UInt64.self)
            let word = Self.narrow(low) | (Self.narrow(high) &<< UInt64(truncatingIfNeeded: 8 &* (remaining &- 4)))
            return (word, Self.isASCIIWithoutNUL(low) && Self.isASCIIWithoutNUL(high))
        }
        if remaining >= 2 {
            // Two units in each 32-bit load, widened to four lanes with two units of 1 (valid, narrowed above the key).
            let low = UInt64(bytes.load(fromByteOffset: 0, as: UInt32.self)) | 0x0001_0001_0000_0000
            let high = UInt64(bytes.load(fromByteOffset: (count &- 2) &* 2, as: UInt32.self)) | 0x0001_0001_0000_0000
            let word = (Self.narrow(low) & 0xFFFF) | ((Self.narrow(high) & 0xFFFF) &<< UInt64(truncatingIfNeeded: 8 &* (remaining &- 2)))
            return (word, Self.isASCIIWithoutNUL(low) && Self.isASCIIWithoutNUL(high))
        }
        let unit = bytes.load(fromByteOffset: 0, as: UInt16.self)
        return (UInt64(unit), unit &- 1 < 0x7F)
    }

    @inline(__always)
    func unit(_ index: Int) -> UInt32 {
        UInt32(bytes.load(fromByteOffset: index &* 2, as: UInt16.self))
    }
}

enum CSSValueKeywordTable {
    static var slotCount: Int { 4096 }
    static var slotMask: Int { 4095 }
    // The rewrite buffer. Longer keys cannot be keywords (the glue static_asserts maxCSSValueKeywordLength < 64).
    static var rewriteBufferCount: Int { 64 }

    // MARK: Words

    static var highBits: UInt64 { 0x8080_8080_8080_8080 }
    static var lowBits: UInt64 { 0x0101_0101_0101_0101 }

    // Bytes [start + offset, start + count) of `bytes` as a little-endian word, zero above, for
    // 1 <= count - offset <= 8. Every load stays inside [start, start + count) (overlapping loads for short keys).
    @inline(__always)
    static func tailWord(_ bytes: RawSpan, start: Int, count: Int, offset: Int) -> UInt64 {
        let remaining = count &- offset
        if count >= 8 {
            let last = bytes.load(fromByteOffset: start &+ count &- 8, as: UInt64.self)
            return last &>> UInt64(truncatingIfNeeded: 64 &- 8 &* remaining)
        }
        if remaining >= 4 {
            let low = UInt64(bytes.load(fromByteOffset: start, as: UInt32.self))
            let high = UInt64(bytes.load(fromByteOffset: start &+ count &- 4, as: UInt32.self))
            return low | (high &<< UInt64(truncatingIfNeeded: 8 &* (remaining &- 4)))
        }
        if remaining >= 2 {
            let low = UInt64(bytes.load(fromByteOffset: start, as: UInt16.self))
            let high = UInt64(bytes.load(fromByteOffset: start &+ count &- 2, as: UInt16.self))
            return low | (high &<< UInt64(truncatingIfNeeded: 8 &* (remaining &- 2)))
        }
        return UInt64(bytes.load(fromByteOffset: start, as: UInt8.self))
    }

    // The low `remaining` (>= 1) bytes of `word` are all in 1...0x7F (the bytes above are zero).
    @inline(__always)
    static func isASCIIWithoutNUL(_ word: UInt64, remaining: Int) -> Bool {
        let filled = remaining >= 8 ? word : word | (lowBits &<< UInt64(truncatingIfNeeded: 8 &* remaining))
        let hasZeroByte = (filled &- lowBits) & ~filled & highBits
        return (word & highBits) | hasZeroByte == 0
    }

    // toASCIILower on every byte of a word of ASCII bytes: +0x20 on 'A'...'Z'. No byte carries into the next.
    @inline(__always)
    static func toASCIILower(word: UInt64) -> UInt64 {
        let atLeastA = word &+ 0x3F3F_3F3F_3F3F_3F3F // high bit set iff byte >= 0x41
        let aboveZ = word &+ 0x2525_2525_2525_2525 // high bit set iff byte >= 0x5B
        return word | ((atLeastA & ~aboveZ & highBits) &>> 2)
    }

    @inline(__always)
    static func toASCIILower(_ character: UInt32) -> UInt32 {
        character | (character &- 0x41 < 26 ? 0x20 : 0)
    }

    // MARK: Hash

    @inline(__always)
    static func mix(_ hash: UInt64, _ word: UInt64) -> UInt64 {
        (hash ^ word) &* 0x9E37_79B9_7F4A_7C15
    }

    @inline(__always)
    static func slot(_ hash: UInt64, count: Int) -> Int {
        let mixed = (hash ^ UInt64(truncatingIfNeeded: count)) &* 0xC2B2_AE3D_27D4_EB4F
        return Int(truncatingIfNeeded: mixed &>> 52) & slotMask
    }

    // The hash of the lowercased key if every unit is in 1...0x7F, else nil. 1 <= key.count.
    @inline(__always)
    static func hash<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key) -> UInt64? {
        let count = key.count
        var hash: UInt64 = 0
        var offset = 0
        while count &- offset > 8 {
            let word = key.word(at: offset)
            if !word.isASCIIWithoutNUL {
                return nil
            }
            hash = mix(hash, toASCIILower(word: word.bytes))
            offset &+= 8
        }
        let word = key.tailWord(at: offset)
        if !word.isASCIIWithoutNUL {
            return nil
        }
        return mix(hash, toASCIILower(word: word.bytes))
    }

    // MARK: Build

    // Inserts every non-empty name, by increasing id; a name already present keeps its first id.
    static func buildTable(pool: RawSpan, offsets: Span<UInt32>, slots: inout MutableSpan<UInt16>) {
        precondition(slots.count == slotCount && offsets.count >= 1 && offsets.count &- 1 <= slotCount / 2)
        for index in slots.indices {
            slots[index] = 0
        }
        var id = 1
        while id < offsets.count &- 1 {
            let start = Int(offsets[id])
            let end = Int(offsets[id &+ 1])
            precondition(start <= end && end <= pool.byteCount)
            let name = CSSValueKeywordLatin1Key(pool.extracting(start..<end))
            if name.count > 0, let hash = hash(name) {
                var slot = slot(hash, count: name.count)
                while true {
                    let existing = slots[slot]
                    if existing == 0 {
                        slots[slot] = UInt16(id)
                        break
                    }
                    if matches(name, pool: pool, offsets: offsets, id: existing) {
                        break
                    }
                    slot = (slot &+ 1) & slotMask
                }
            }
            id &+= 1
        }
    }

    // MARK: Lookup

    // cssValueKeywordID<CharacterType>(std::span) and findCSSValueKeyword(). Returns the CSSValueID, 0 for Invalid.
    @inline(__always)
    static func lookup<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key, pool: RawSpan, offsets: Span<UInt32>, slots: Span<UInt16>) -> UInt16 {
        precondition(slots.count == slotCount)
        if key.count == 0 {
            return 0
        }
        guard let hash = hash(key) else {
            return 0
        }
        if key.unit(0) == 0x2D, startsWithLowercasedApplePrefix(key) { // '-'
            return lookupStartingWithApplePrefix(key, pool: pool, offsets: offsets, slots: slots)
        }
        return probe(key, hash: hash, pool: pool, offsets: offsets, slots: slots)
    }

    @inline(__always)
    static func probe<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key, hash: UInt64, pool: RawSpan, offsets: Span<UInt32>, slots: Span<UInt16>) -> UInt16 {
        var slot = slot(hash, count: key.count)
        for _ in 0..<slotCount {
            let id = slots[slot]
            if id == 0 {
                return 0
            }
            if matches(key, pool: pool, offsets: offsets, id: id) {
                return id
            }
            slot = (slot &+ 1) & slotMask
        }
        return 0
    }

    // The lowercased key (known to be ASCII) equals the name of `id`.
    @inline(__always)
    static func matches<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key, pool: RawSpan, offsets: Span<UInt32>, id: UInt16) -> Bool {
        let start = Int(offsets[Int(id)])
        let count = key.count
        if Int(offsets[Int(id) &+ 1]) &- start != count {
            return false
        }
        var offset = 0
        while count &- offset > 8 {
            if toASCIILower(word: key.word(at: offset).bytes) != pool.load(fromByteOffset: start &+ offset, as: UInt64.self) {
                return false
            }
            offset &+= 8
        }
        return toASCIILower(word: key.tailWord(at: offset).bytes) == tailWord(pool, start: start, count: count, offset: offset)
    }

    // MARK: -apple- keywords

    // The lowercased key (known to be ASCII) starts with "-apple-". Only such keys can be rewritten.
    @inline(__always)
    static func startsWithLowercasedApplePrefix<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key) -> Bool {
        let count = key.count
        if count < 7 {
            return false
        }
        let head = count >= 8 ? key.word(at: 0).bytes : key.tailWord(at: 0).bytes
        return toASCIILower(word: head) & 0x00FF_FFFF_FFFF_FFFF == 0x002D_656C_7070_612D // "-apple-"
    }

    // The rest of cssValueKeywordID<CharacterType>() for a key (known to be ASCII) that starts with "-apple-" (rare):
    // lowercased into a local buffer, the -apple- to -webkit- rewrite in place if isAppleLegacyCSSValueKeyword(), and
    // the lookup of the result.
    @inline(never)
    static func lookupStartingWithApplePrefix<Key: CSSValueKeywordKey & ~Escapable>(_ key: borrowing Key, pool: RawSpan, offsets: Span<UInt32>, slots: Span<UInt16>) -> UInt16 {
        let count = key.count
        if count >= rewriteBufferCount {
            return 0 // Longer than any keyword, before or after the rewrite.
        }
        var buffer = InlineArray<64, UInt8>(repeating: 0)
        for index in 0..<count {
            buffer[index] = UInt8(truncatingIfNeeded: toASCIILower(key.unit(index)))
        }
        var length = count
        if isAppleLegacyKeyword(buffer.span.extracting(0..<count)) {
            // memmoveSpan(bufferSpan.subspan(7), bufferSpan.subspan(6, length - 6)); memcpySpan(bufferSpan.subspan(1), "webkit"_span);
            var index = count
            while index > 6 {
                buffer[index] = buffer[index &- 1]
                index &-= 1
            }
            let webkit: InlineArray<6, UInt8> = [0x77, 0x65, 0x62, 0x6B, 0x69, 0x74] // "webkit"
            for webkitIndex in 0..<6 {
                buffer[1 &+ webkitIndex] = webkit[webkitIndex]
            }
            length &+= 1
        }
        let lowercased = CSSValueKeywordLatin1Key(buffer.span.bytes.extracting(0..<length))
        guard let hash = hash(lowercased) else {
            return 0
        }
        return probe(lowercased, hash: hash, pool: pool, offsets: offsets, slots: slots)
    }

    // isAppleLegacyCSSValueKeyword() on the lowercased characters.
    static func isAppleLegacyKeyword(_ characters: Span<UInt8>) -> Bool {
        let apple: InlineArray<6, UInt8> = [0x61, 0x70, 0x70, 0x6C, 0x65, 0x2D] // "apple-"
        let system: InlineArray<6, UInt8> = [0x73, 0x79, 0x73, 0x74, 0x65, 0x6D] // "system"
        let pay: InlineArray<3, UInt8> = [0x70, 0x61, 0x79] // "pay"
        let wireless: InlineArray<8, UInt8> = [0x77, 0x69, 0x72, 0x65, 0x6C, 0x65, 0x73, 0x73] // "wireless"
        return hasPrefix(characters, at: 1, apple.span)
            && !hasPrefix(characters, at: 7, system.span)
            && !hasPrefix(characters, at: 7, pay.span)
            && !hasPrefix(characters, at: 7, wireless.span)
    }

    // spanHasPrefix(characters.subspan(offset), prefix), offset <= characters.count.
    static func hasPrefix(_ characters: Span<UInt8>, at offset: Int, _ prefix: Span<UInt8>) -> Bool {
        if characters.count &- offset < prefix.count {
            return false
        }
        for index in prefix.indices where characters[offset &+ index] != prefix[index] {
            return false
        }
        return true
    }
}
