// Copyright (C) 2024-2026 Apple Inc. All rights reserved.
// Copyright (C) 2011 the V8 project authors. All rights reserved.
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

// The adaptive string search used by String.prototype.indexOf/includes/split/replace/replaceAll, atom RegExps and the
// DFG string operations (formerly WTF::AdaptiveStringSearcher and StringView::find(AdaptiveStringSearcherTables&, ...),
// which were derived from V8's StringSearch). The result is always the first occurrence of the pattern at or after the
// start index; the strategy only decides how fast it is found:
// - a one-unit pattern: a SIMD search for that unit;
// - otherwise a SIMD search for positions whose first and last units match, then a scalar compare of the units in
//   between. After 17 false candidates it bails out to Boyer-Moore-Horspool, which bails out to Boyer-Moore when
//   it reads too many units per shift;
// - a 16-bit pattern with a unit above 0xFF never occurs in an 8-bit subject.
// The Boyer-Moore tables live on the stack of the (cold) functions that use them. A multiple-match search (findAll)
// keeps its strategy and tables from one match to the next, as the old searcher object did.

enum StringSearch {
    // Boyer-Moore tables only cover the last bmMaxShift + 1 positions of the pattern; the bad-character table maps
    // each 16-bit unit to its low byte.
    static let bmMaxShift = 250
    static let initialSearchFailureThreshold = 16
}

// The code units of a subject or a pattern: Latin-1 bytes or UTF-16 code units (any alignment), both read from raw
// bytes.
protocol StringSearchUnits: ~Escapable {
    static var is8Bit: Bool { get }
    // Units per SIMD block.
    static var stride: Int { get }
    var count: Int { get }
    subscript(_ index: Int) -> UInt16 { get }
    // The lanes k of the block at index (index + stride <= count) whose unit equals `unit`, as two mask words in which
    // lane k is the k-th group of 8 (Latin-1) or 16 (UTF-16) bits.
    func equalLanes(at index: Int, _ unit: UInt16) -> SIMD2<UInt64>
    // The units as bytes (one per Latin-1 unit, two per UTF-16 unit, little-endian).
    var bytes: RawSpan { @_lifetime(copy self) get }
    // Units range of self.
    @_lifetime(copy self)
    func extracting(_ range: Range<Int>) -> Self
    // Whether a whole block starts at index (0 <= index <= count), in the form of the block load's bounds check.
    func hasBlock(at index: Int) -> Bool
    // The same for two consecutive blocks, and their lanes equal to `unit` (one load).
    func hasTwoBlocks(at index: Int) -> Bool
    func equalLanesOfTwoBlocks(at index: Int, _ unit: UInt16) -> (low: SIMD2<UInt64>, high: SIMD2<UInt64>)
    // Whether four consecutive blocks start at index, and whether any of their units equals `unit` (two loads).
    func hasFourBlocks(at index: Int) -> Bool
    func anyEqualInFourBlocks(at index: Int, _ unit: UInt16) -> Bool
}

struct StringSearchLatin1: StringSearchUnits, ~Escapable {
    let bytes: RawSpan

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    static var is8Bit: Bool { true }
    static var stride: Int { 16 }

    var count: Int {
        @inline(__always) get { bytes.byteCount }
    }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { UInt16(bytes.load(fromByteOffset: index, as: UInt8.self)) }
    }

    @inline(__always)
    func block(_ index: Int) -> SIMD16<UInt8> {
        bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)
    }

    @inline(__always)
    func equalLanes(at index: Int, _ unit: UInt16) -> SIMD2<UInt64> {
        StringSearch.maskWords(block(index) .== SIMD16(repeating: UInt8(truncatingIfNeeded: unit)))
    }

    @inline(__always)
    func hasBlock(at index: Int) -> Bool {
        UInt(bitPattern: index) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index >= 16
    }

    @inline(__always)
    func hasTwoBlocks(at index: Int) -> Bool {
        UInt(bitPattern: index) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index >= 32
    }

    @inline(__always)
    func equalLanesOfTwoBlocks(at index: Int, _ unit: UInt16) -> (low: SIMD2<UInt64>, high: SIMD2<UInt64>) {
        // One 32-byte load (as 16 UInt16s: SIMD32 is not lowered well), compared as two 16-byte blocks.
        let blocks = bitCast(bytes.load(fromByteOffset: index, as: SIMD16<UInt16>.self), to: SIMD4<UInt64>.self)
        let splat = SIMD16<UInt8>(repeating: UInt8(truncatingIfNeeded: unit))
        let low = StringSearch.maskWords(bitCast(blocks.lowHalf, to: SIMD16<UInt8>.self) .== splat)
        let high = StringSearch.maskWords(bitCast(blocks.highHalf, to: SIMD16<UInt8>.self) .== splat)
        return (low, high)
    }

    @inline(__always)
    func hasFourBlocks(at index: Int) -> Bool {
        UInt(bitPattern: index) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index >= 64
    }

    @inline(__always)
    func anyEqualInFourBlocks(at index: Int, _ unit: UInt16) -> Bool {
        let window = bytes.extracting(index ..< index &+ 64)
        let front = bitCast(window.load(fromByteOffset: 0, as: SIMD16<UInt16>.self), to: SIMD4<UInt64>.self)
        let back = bitCast(window.load(fromByteOffset: 32, as: SIMD16<UInt16>.self), to: SIMD4<UInt64>.self)
        let splat = SIMD16<UInt8>(repeating: UInt8(truncatingIfNeeded: unit))
        let matches = ((bitCast(front.lowHalf, to: SIMD16<UInt8>.self) .== splat) .| (bitCast(front.highHalf, to: SIMD16<UInt8>.self) .== splat))
            .| ((bitCast(back.lowHalf, to: SIMD16<UInt8>.self) .== splat) .| (bitCast(back.highHalf, to: SIMD16<UInt8>.self) .== splat))
        return !StringSearch.isZero(StringSearch.maskWords(matches))
    }

    @inline(__always)
    @_lifetime(copy self)
    func extracting(_ range: Range<Int>) -> StringSearchLatin1 {
        StringSearchLatin1(bytes.extracting(range))
    }
}

struct StringSearchUTF16: StringSearchUnits, ~Escapable {
    let bytes: RawSpan
    let count: Int

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
        self.count = bytes.byteCount &>> 1
    }

    static var is8Bit: Bool { false }
    static var stride: Int { 8 }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { UInt16(littleEndian: bytes.load(fromByteOffset: index &* 2, as: UInt16.self)) }
    }

    @inline(__always)
    func block(_ index: Int) -> SIMD8<UInt16> {
        bytes.load(fromByteOffset: index &* 2, as: SIMD8<UInt16>.self)
    }

    @inline(__always)
    func equalLanes(at index: Int, _ unit: UInt16) -> SIMD2<UInt64> {
        StringSearch.maskWords(block(index) .== SIMD8(repeating: unit))
    }

    @inline(__always)
    func hasBlock(at index: Int) -> Bool {
        UInt(bitPattern: index &* 2) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index &* 2 >= 16
    }

    @inline(__always)
    func hasTwoBlocks(at index: Int) -> Bool {
        UInt(bitPattern: index &* 2) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index &* 2 >= 32
    }

    @inline(__always)
    func equalLanesOfTwoBlocks(at index: Int, _ unit: UInt16) -> (low: SIMD2<UInt64>, high: SIMD2<UInt64>) {
        let matches = bytes.load(fromByteOffset: index &* 2, as: SIMD16<UInt16>.self) .== SIMD16(repeating: unit)
        let words = bitCast(SIMD16<UInt16>(repeating: 0).replacing(with: 0xFFFF, where: matches), to: SIMD4<UInt64>.self)
        return (words.lowHalf, words.highHalf)
    }

    @inline(__always)
    func hasFourBlocks(at index: Int) -> Bool {
        UInt(bitPattern: index &* 2) <= UInt(bitPattern: bytes.byteCount) && bytes.byteCount &- index &* 2 >= 64
    }

    @inline(__always)
    func anyEqualInFourBlocks(at index: Int, _ unit: UInt16) -> Bool {
        let window = bytes.extracting(index &* 2 ..< index &* 2 &+ 64)
        let splat = SIMD16<UInt16>(repeating: unit)
        let matches = (window.load(fromByteOffset: 0, as: SIMD16<UInt16>.self) .== splat) .| (window.load(fromByteOffset: 32, as: SIMD16<UInt16>.self) .== splat)
        let words = bitCast(SIMD16<UInt16>(repeating: 0).replacing(with: 0xFFFF, where: matches), to: SIMD4<UInt64>.self)
        return !StringSearch.isZero(words.lowHalf | words.highHalf)
    }

    @inline(__always)
    @_lifetime(copy self)
    func extracting(_ range: Range<Int>) -> StringSearchUTF16 {
        StringSearchUTF16(bytes.extracting(range.lowerBound &* 2 ..< range.upperBound &* 2))
    }
}

extension StringSearch {
    // A lane mask as its two little-endian 64-bit halves (bitCast, no lane shuffles).
    @inline(__always)
    static func maskWords(_ mask: SIMDMask<SIMD16<Int8>>) -> SIMD2<UInt64> {
        bitCast(SIMD16<UInt8>(repeating: 0).replacing(with: 0xFF, where: mask), to: SIMD2<UInt64>.self)
    }

    @inline(__always)
    static func maskWords(_ mask: SIMDMask<SIMD8<Int16>>) -> SIMD2<UInt64> {
        bitCast(SIMD8<UInt16>(repeating: 0).replacing(with: 0xFFFF, where: mask), to: SIMD2<UInt64>.self)
    }

    @inline(__always)
    static func isZero(_ words: SIMD2<UInt64>) -> Bool { (words[0] | words[1]) == 0 }

    // The first set lane of non-zero mask words of `Units`.
    @inline(__always)
    static func firstLane<Units: StringSearchUnits & ~Escapable>(_ words: SIMD2<UInt64>, _: borrowing Units) -> Int {
        let low = words[0]
        let byte = low != 0 ? low.trailingZeroBitCount &>> 3 : 8 &+ (words[1].trailingZeroBitCount &>> 3)
        return Units.is8Bit ? byte : byte &>> 1
    }

    // Whether a 16-bit pattern can occur in an 8-bit subject (all its units are Latin-1), or the widths allow it.
    @inline(__always)
    static func canOccur<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ pattern: borrowing Pattern, in _: borrowing Subject) -> Bool {
        if Pattern.is8Bit || !Subject.is8Bit {
            return true
        }
        var any: UInt16 = 0
        for index in 0..<pattern.count {
            any |= pattern[index]
        }
        return any <= 0xFF
    }

    // MARK: Single unit

    // The first index >= start of `unit` in subject, or -1 (0 <= start <= subject.count).
    @inline(always)
    static func singleUnitSearch<Subject: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ unit: UInt16, from start: Int) -> Int {
        let count = subject.count
        let stride = Subject.stride
        var index = start
        if count &- index < stride {
            while index < count {
                if subject[index] == unit {
                    return index
                }
                index &+= 1
            }
            return -1
        }
        let first = subject.equalLanes(at: index, unit)
        if !isZero(first) {
            return index &+ firstLane(first, subject)
        }
        index &+= stride
        // Long runs go four blocks per step; the step that has a match is redone two blocks at a time.
        let quadrupleStride = stride &* 4
        while subject.hasFourBlocks(at: index) {
            if subject.anyEqualInFourBlocks(at: index, unit) {
                break
            }
            index &+= quadrupleStride
        }
        let doubleStride = stride &* 2
        while subject.hasTwoBlocks(at: index) {
            let (low, high) = subject.equalLanesOfTwoBlocks(at: index, unit)
            if !isZero(low | high) {
                if !isZero(low) {
                    return index &+ firstLane(low, subject)
                }
                return index &+ stride &+ firstLane(high, subject)
            }
            index &+= doubleStride
        }
        if count &- index >= stride {
            let words = subject.equalLanes(at: index, unit)
            if !isZero(words) {
                return index &+ firstLane(words, subject)
            }
            index &+= stride
        }
        if index < count {
            // The last block overlaps units already known not to match.
            let last = count &- stride
            let words = subject.equalLanes(at: last, unit)
            if !isZero(words) {
                return last &+ firstLane(words, subject)
            }
        }
        return -1
    }

    // MARK: Initial search (SIMD first/last pair)

    // Whether pattern units 1 ..< count - 1 equal the subject units at position + 1 ...
    @inline(always)
    static func middleMatches<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, at position: Int) -> Bool {
        let length = pattern.count
        if length <= 2 {
            return true
        }
        if Subject.is8Bit == Pattern.is8Bit {
            // Same width: the code units are equal exactly when their bytes are.
            let shift = Subject.is8Bit ? 0 : 1
            let middle = (length &- 2) &<< shift
            let patternBytes = pattern.bytes.extracting(1 &<< shift ..< (1 &<< shift) &+ middle)
            let subjectBytes = subject.bytes.extracting((position &+ 1) &<< shift ..< ((position &+ 1) &<< shift) &+ middle)
            return bytesEqual(patternBytes, subjectBytes)
        }
        let window = subject.extracting(position ..< position &+ length)
        var j = 1
        while j < length &- 1 {
            if pattern[j] != window[j] {
                return false
            }
            j &+= 1
        }
        return true
    }

    // Whether two byte spans of the same length are equal: 8 bytes per step, the last step overlapping.
    @inline(always)
    static func bytesEqual(_ a: RawSpan, _ b: RawSpan) -> Bool {
        let count = a.byteCount
        if count >= 8 {
            var i = 0
            while count &- i > 8 {
                if a.load(fromByteOffset: i, as: UInt64.self) != b.load(fromByteOffset: i, as: UInt64.self) {
                    return false
                }
                i &+= 8
            }
            return a.load(fromByteOffset: count &- 8, as: UInt64.self) == b.load(fromByteOffset: count &- 8, as: UInt64.self)
        }
        if count >= 4 {
            if a.load(fromByteOffset: 0, as: UInt32.self) != b.load(fromByteOffset: 0, as: UInt32.self) {
                return false
            }
            return a.load(fromByteOffset: count &- 4, as: UInt32.self) == b.load(fromByteOffset: count &- 4, as: UInt32.self)
        }
        var i = 0
        while i < count {
            if a.load(fromByteOffset: i, as: UInt8.self) != b.load(fromByteOffset: i, as: UInt8.self) {
                return false
            }
            i &+= 1
        }
        return true
    }

    // The first match at or after startIndex (pattern.count >= 2), -1, or -2 - i: after too many false candidates,
    // continue with Boyer-Moore-Horspool from i (no match before i).
    @inline(always)
    static func initialSearch<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, from startIndex: Int) -> Int {
        if startIndex > subject.count &- pattern.count {
            return -1
        }
        let pair = StringSearchPair(pattern)
        return initialSearch(subject, pattern, pair, subject.extracting(0 ..< pair.positions(subject)), subject.extracting(pair.lastOffset ..< pair.lastOffset &+ pair.positions(subject)), from: startIndex)
    }

    // initialSearch with its pattern-dependent setup done by the caller (startIndex <= subject.count - pattern.count):
    // firsts and lasts are the units at candidate positions 0 ..< pair.positions(subject) and at those positions +
    // pair.lastOffset. The two views have the same length, so one bounds check covers both blocks.
    @inline(always)
    static func initialSearch<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, _ pair: StringSearchPair, _ firsts: borrowing Subject, _ lasts: borrowing Subject, from startIndex: Int) -> Int {
        let stride = Subject.stride
        let first = pair.first
        let last = pair.last
        // Two blocks per step while there are two (the first candidate of the pair is the one the block-by-block
        // search finds first), then one.
        var k = startIndex
        var failureCount = 0
        while true {
            var lane: Int
            if firsts.hasTwoBlocks(at: k) {
                let firstLanes = firsts.equalLanesOfTwoBlocks(at: k, first)
                let lastLanes = lasts.equalLanesOfTwoBlocks(at: k, last)
                let low = firstLanes.low & lastLanes.low
                let high = firstLanes.high & lastLanes.high
                if isZero(low | high) {
                    k &+= stride &* 2
                    continue
                }
                lane = isZero(low) ? stride &+ firstLane(high, subject) : firstLane(low, subject)
            } else if firsts.hasBlock(at: k) {
                let candidates = firsts.equalLanes(at: k, first) & lasts.equalLanes(at: k, last)
                if isZero(candidates) {
                    k &+= stride
                    continue
                }
                lane = firstLane(candidates, subject)
            } else {
                break
            }
            let position = k &+ lane
            if middleMatches(subject, pattern, at: position) {
                return position
            }
            k = position &+ 1
            failureCount &+= 1
            if failureCount > initialSearchFailureThreshold {
                return -2 &- k
            }
        }
        let maxIndex = firsts.count &- 1
        while k <= maxIndex {
            if firsts[k] == first {
                if lasts[k] == last {
                    if middleMatches(subject, pattern, at: k) {
                        return k
                    }
                }
            }
            k &+= 1
        }
        return -1
    }

    // MARK: Boyer-Moore-Horspool and Boyer-Moore

    // The bad-character table entry of a subject unit: the last pattern index < patternLength - 1 (at or after
    // max(0, patternLength - bmMaxShift)) holding a unit of the same class, else max(0, patternLength - bmMaxShift) - 1.
    // A 16-bit unit above 0xFF never occurs in an 8-bit pattern (-1); two 16-bit units are in the same class when their
    // low bytes are equal.
    @inline(__always)
    static func charOccurrence<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ badCharacters: inout InlineArray<256, Int32>, _ unit: UInt16, _: borrowing Subject, _: borrowing Pattern) -> Int {
        if !Subject.is8Bit && Pattern.is8Bit && unit > 0xFF {
            return -1
        }
        return Int(badCharacters[Int(unit & 0xFF)])
    }

    // The bad-character table of a pattern; every entry not set by the pattern is
    // max(0, patternLength - bmMaxShift) - 1.
    @inline(__always)
    static func populateBoyerMooreHorspoolTable<Pattern: StringSearchUnits & ~Escapable>(_ badCharacters: inout InlineArray<256, Int32>, _ pattern: borrowing Pattern) {
        let patternLength = pattern.count
        let start = max(0, patternLength &- bmMaxShift)
        var i = start
        while i < patternLength &- 1 {
            badCharacters[Int(pattern[i] & 0xFF)] = Int32(truncatingIfNeeded: i)
            i &+= 1
        }
    }

    // A match at or after startIndex, -1, or -2 - index: too many units read per shift, continue with Boyer-Moore
    // from index (no match before index).
    @inline(__always)
    static func boyerMooreHorspoolSearch<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, _ badCharacters: inout InlineArray<256, Int32>, from startIndex: Int) -> Int {
        let subjectLength = subject.count
        let patternLength = pattern.count
        var badness = -patternLength
        let lastChar = pattern[patternLength &- 1]
        let lastCharShift = patternLength &- 1 &- charOccurrence(&badCharacters, lastChar, subject, pattern)
        var index = startIndex
        while index <= subjectLength &- patternLength {
            var j = patternLength &- 1
            var subjectChar = subject[index &+ j]
            while lastChar != subjectChar {
                let shift = j &- charOccurrence(&badCharacters, subjectChar, subject, pattern)
                index &+= shift
                badness &+= 1 &- shift
                if index > subjectLength &- patternLength {
                    return -1
                }
                subjectChar = subject[index &+ j]
            }
            j &-= 1
            while j >= 0 {
                if pattern[j] != subject[index &+ j] {
                    break
                }
                j &-= 1
            }
            if j < 0 {
                return index
            }
            index &+= lastCharShift
            badness &+= (patternLength &- j) &- lastCharShift
            if badness > 0 {
                return -2 &- index
            }
        }
        return -1
    }

    // Good-suffix shift and suffix tables indexed by pattern index - max(0, patternLength - bmMaxShift).
    @inline(__always)
    static func populateBoyerMooreTable<Pattern: StringSearchUnits & ~Escapable>(_ shiftTable: inout InlineArray<251, Int>, _ suffixTable: inout InlineArray<251, Int>, _ pattern: borrowing Pattern) {
        let patternLength = pattern.count
        let start = max(0, patternLength &- bmMaxShift)
        let length = patternLength &- start

        for i in start..<patternLength {
            shiftTable[i &- start] = length
        }
        shiftTable[patternLength &- start] = 1
        suffixTable[patternLength &- start] = patternLength &+ 1

        if patternLength <= start {
            return
        }

        let lastChar = pattern[patternLength &- 1]
        var suffix = patternLength &+ 1
        var i = patternLength
        while i > start {
            let c = pattern[i &- 1]
            while suffix <= patternLength {
                if c == pattern[suffix &- 1] {
                    break
                }
                if shiftTable[suffix &- start] == length {
                    shiftTable[suffix &- start] = suffix &- i
                }
                suffix = suffixTable[suffix &- start]
            }
            i &-= 1
            suffix &-= 1
            suffixTable[i &- start] = suffix
            if suffix == patternLength {
                while i > start {
                    if pattern[i &- 1] == lastChar {
                        break
                    }
                    if shiftTable[patternLength &- start] == length {
                        shiftTable[patternLength &- start] = patternLength &- i
                    }
                    i &-= 1
                    suffixTable[i &- start] = patternLength
                }
                if i > start {
                    i &-= 1
                    suffix &-= 1
                    suffixTable[i &- start] = suffix
                }
            }
        }
        if suffix < patternLength {
            var k = start
            while k <= patternLength {
                if shiftTable[k &- start] == length {
                    shiftTable[k &- start] = suffix &- start
                }
                if k == suffix {
                    suffix = suffixTable[suffix &- start]
                }
                k &+= 1
            }
        }
    }

    @inline(__always)
    static func boyerMooreSearch<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, _ badCharacters: inout InlineArray<256, Int32>, _ goodSuffixShift: inout InlineArray<251, Int>, from startIndex: Int) -> Int {
        let subjectLength = subject.count
        let patternLength = pattern.count
        let start = max(0, patternLength &- bmMaxShift)
        let lastChar = pattern[patternLength &- 1]
        var index = startIndex
        while index <= subjectLength &- patternLength {
            var j = patternLength &- 1
            var c = subject[index &+ j]
            while lastChar != c {
                let shift = j &- charOccurrence(&badCharacters, c, subject, pattern)
                index &+= shift
                if index > subjectLength &- patternLength {
                    return -1
                }
                c = subject[index &+ j]
            }
            while j >= 0 {
                c = subject[index &+ j]
                if pattern[j] != c {
                    break
                }
                j &-= 1
            }
            if j < 0 {
                return index
            }
            if j < start {
                // More of the pattern matched than the tables cover: shift as Boyer-Moore-Horspool does.
                index &+= patternLength &- 1 &- charOccurrence(&badCharacters, lastChar, subject, pattern)
            } else {
                let goodSuffix = goodSuffixShift[j &+ 1 &- start]
                var shift = j &- charOccurrence(&badCharacters, c, subject, pattern)
                if goodSuffix > shift {
                    shift = goodSuffix
                }
                index &+= shift
            }
        }
        return -1
    }

    // MARK: Searches

    // StringView::find(AdaptiveStringSearcherTables&, ...): the first index >= start at which pattern occurs in
    // subject, or -1. start > subject.count gives -1, an empty pattern gives start.
    @inline(always)
    static func find<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, from start: Int) -> Int {
        let subjectLength = subject.count
        let patternLength = pattern.count
        if start > subjectLength {
            return -1
        }
        if patternLength == 0 {
            return start
        }
        if patternLength > subjectLength &- start {
            return -1
        }
        if !canOccur(pattern, in: subject) {
            return -1
        }
        if patternLength == 1 {
            return singleUnitSearch(subject, pattern[0], from: start)
        }
        let result = initialSearch(subject, pattern, from: start)
        if result >= -1 {
            return result
        }
        return findWithBoyerMoore(subject, pattern, from: -2 &- result)
    }

    @inline(never)
    static func findWithBoyerMoore<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, from index: Int) -> Int {
        var badCharacters = InlineArray<256, Int32>(repeating: Int32(truncatingIfNeeded: max(0, pattern.count &- bmMaxShift) &- 1))
        populateBoyerMooreHorspoolTable(&badCharacters, pattern)
        let result = boyerMooreHorspoolSearch(subject, pattern, &badCharacters, from: index)
        if result >= -1 {
            return result
        }
        return findWithGoodSuffix(subject, pattern, &badCharacters, from: -2 &- result)
    }

    @inline(never)
    static func findWithGoodSuffix<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, _ badCharacters: inout InlineArray<256, Int32>, from index: Int) -> Int {
        var shiftTable = InlineArray<251, Int>(repeating: 0)
        var suffixTable = InlineArray<251, Int>(repeating: 0)
        populateBoyerMooreTable(&shiftTable, &suffixTable, pattern)
        return boyerMooreSearch(subject, pattern, &badCharacters, &shiftTable, from: index)
    }

    // Successive non-overlapping matches: the first at or after start, each next one at or after the end of the
    // previous one (match + pattern.count). For each match, the gap since the previous end (since start for the first)
    // is written to gaps as an Int32 pair (begin, end) when it is not empty. Stops after the match that writes the last
    // pair gaps has room for (gaps.count / 2 pairs, at least one), or when there are no more matches.
    @inline(always)
    static func findAll<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, from start: Int, _ gaps: inout MutableSpan<Int32>) -> StringSearchMatches {
        let patternLength = pattern.count
        precondition(patternLength > 0 && gaps.count >= 2)
        var matches = StringSearchMatches(end: start)
        if start > subject.count {
            return matches
        }
        if !canOccur(pattern, in: subject) {
            return matches
        }
        if patternLength == 1 {
            let unit = pattern[0]
            while true {
                let found = singleUnitSearch(subject, unit, from: matches.end)
                if found < 0 || !matches.record(found, patternLength, &gaps) {
                    return matches
                }
            }
        }
        if start > subject.count &- patternLength {
            return matches
        }
        let pair = StringSearchPair(pattern)
        let firsts = subject.extracting(0 ..< pair.positions(subject))
        let lasts = subject.extracting(pair.lastOffset ..< pair.lastOffset &+ pair.positions(subject))
        while true {
            if matches.end > firsts.count &- 1 {
                return matches
            }
            let found = initialSearch(subject, pattern, pair, firsts, lasts, from: matches.end)
            if found == -1 {
                return matches
            }
            if found < -1 {
                findAllWithBoyerMoore(subject, pattern, from: -2 &- found, &gaps, &matches)
                return matches
            }
            if !matches.record(found, patternLength, &gaps) {
                return matches
            }
        }
    }

    @inline(never)
    static func findAllWithBoyerMoore<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, from start: Int, _ gaps: inout MutableSpan<Int32>, _ matches: inout StringSearchMatches) {
        var badCharacters = InlineArray<256, Int32>(repeating: Int32(truncatingIfNeeded: max(0, pattern.count &- bmMaxShift) &- 1))
        populateBoyerMooreHorspoolTable(&badCharacters, pattern)
        let patternLength = pattern.count
        var index = start
        while true {
            let found = boyerMooreHorspoolSearch(subject, pattern, &badCharacters, from: index)
            if found == -1 {
                return
            }
            if found < -1 {
                findAllWithGoodSuffix(subject, pattern, &badCharacters, from: -2 &- found, &gaps, &matches)
                return
            }
            if !matches.record(found, patternLength, &gaps) {
                return
            }
            index = matches.end
        }
    }

    @inline(never)
    static func findAllWithGoodSuffix<Subject: StringSearchUnits & ~Escapable, Pattern: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject, _ pattern: borrowing Pattern, _ badCharacters: inout InlineArray<256, Int32>, from start: Int, _ gaps: inout MutableSpan<Int32>, _ matches: inout StringSearchMatches) {
        var shiftTable = InlineArray<251, Int>(repeating: 0)
        var suffixTable = InlineArray<251, Int>(repeating: 0)
        populateBoyerMooreTable(&shiftTable, &suffixTable, pattern)
        let patternLength = pattern.count
        var index = start
        while true {
            let found = boyerMooreSearch(subject, pattern, &badCharacters, &shiftTable, from: index)
            if found < 0 || !matches.record(found, patternLength, &gaps) {
                return
            }
            index = matches.end
        }
    }
}

// The result of StringSearch.findAll.
struct StringSearchMatches {
    var count = 0
    var last = -1
    // The end of the last match (where the next search starts), or the start index if there was none.
    var end: Int
    // Gap pairs written.
    var gapCount = 0

    init(end: Int) {
        self.end = end
    }

    // Records a match; false when the gap buffer is full.
    @inline(__always)
    mutating func record(_ found: Int, _ patternLength: Int, _ gaps: inout MutableSpan<Int32>) -> Bool {
        if found > end {
            gaps[gapCount &* 2] = Int32(truncatingIfNeeded: end)
            gaps[gapCount &* 2 &+ 1] = Int32(truncatingIfNeeded: found)
            gapCount &+= 1
        }
        count &+= 1
        last = found
        end = found &+ patternLength
        return gapCount &* 2 &+ 2 <= gaps.count
    }
}

// The first and last units of a pattern of at least 2 units, for the SIMD pair search.
struct StringSearchPair {
    let first: UInt16
    let last: UInt16
    let lastOffset: Int

    @inline(__always)
    init<Pattern: StringSearchUnits & ~Escapable>(_ pattern: borrowing Pattern) {
        lastOffset = pattern.count &- 1
        first = pattern[0]
        last = pattern[lastOffset]
    }

    // The number of candidate positions in subject (subject.count - pattern.count + 1).
    @inline(__always)
    func positions<Subject: StringSearchUnits & ~Escapable>(_ subject: borrowing Subject) -> Int {
        subject.count &- lastOffset
    }
}
