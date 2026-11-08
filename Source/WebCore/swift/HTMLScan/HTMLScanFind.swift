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

// Shared pieces of the HTML parser scanning kernels (HTMLTokenizer.cpp batching scans, HTMLDocumentParserFastPath.cpp
// scanners): the "index of the first code unit in a small set" search, and read access to the parser's input as either
// Latin-1 bytes (Span<UInt8>) or UTF-16 code units (RawSpan, 2 bytes per unit, any alignment).
//
// The search tests 16 bytes (or 8 UTF-16 units) per step with SIMD compares; the last step overlaps the previous one
// rather than reading past the end, which is correct because the overlapped units were already found not to match.
// Below one vector it tests 8 bytes twice (Latin-1), or loops. The result equals the first matching index exactly.

enum HTMLScan {
}

// A small set of ASCII code units, tested on whole vectors, single bytes and single UTF-16 code units. A UTF-16 unit
// matches only if it equals the ASCII value exactly (its high byte is zero).
protocol HTMLScanCharacterSet {
    func matches(_ v: SIMD16<UInt8>) -> SIMDMask<SIMD16<Int8>>
    func matches(_ v: SIMD8<UInt8>) -> SIMDMask<SIMD8<Int8>>
    func matches(_ v: SIMD8<UInt16>) -> SIMDMask<SIMD8<Int16>>
    func matches(_ c: UInt16) -> Bool
}

struct HTMLScanFourCharacters: HTMLScanCharacterSet {
    let a: UInt8
    let b: UInt8
    let c: UInt8
    let d: UInt8

    @inline(__always)
    func matches(_ v: SIMD16<UInt8>) -> SIMDMask<SIMD16<Int8>> {
        ((v .== SIMD16(repeating: a)) .| (v .== SIMD16(repeating: b))) .| ((v .== SIMD16(repeating: c)) .| (v .== SIMD16(repeating: d)))
    }

    @inline(__always)
    func matches(_ v: SIMD8<UInt8>) -> SIMDMask<SIMD8<Int8>> {
        ((v .== SIMD8(repeating: a)) .| (v .== SIMD8(repeating: b))) .| ((v .== SIMD8(repeating: c)) .| (v .== SIMD8(repeating: d)))
    }

    @inline(__always)
    func matches(_ v: SIMD8<UInt16>) -> SIMDMask<SIMD8<Int16>> {
        let a = SIMD8<UInt16>(repeating: UInt16(a)), b = SIMD8<UInt16>(repeating: UInt16(b))
        let c = SIMD8<UInt16>(repeating: UInt16(c)), d = SIMD8<UInt16>(repeating: UInt16(d))
        return ((v .== a) .| (v .== b)) .| ((v .== c) .| (v .== d))
    }

    @inline(__always)
    func matches(_ character: UInt16) -> Bool {
        character == UInt16(a) || character == UInt16(b) || character == UInt16(c) || character == UInt16(d)
    }
}

struct HTMLScanFiveCharacters: HTMLScanCharacterSet {
    let a: UInt8
    let b: UInt8
    let c: UInt8
    let d: UInt8
    let e: UInt8

    @inline(__always)
    func matches(_ v: SIMD16<UInt8>) -> SIMDMask<SIMD16<Int8>> {
        let ab = (v .== SIMD16(repeating: a)) .| (v .== SIMD16(repeating: b))
        let cd = (v .== SIMD16(repeating: c)) .| (v .== SIMD16(repeating: d))
        return (ab .| cd) .| (v .== SIMD16(repeating: e))
    }

    @inline(__always)
    func matches(_ v: SIMD8<UInt8>) -> SIMDMask<SIMD8<Int8>> {
        let ab = (v .== SIMD8(repeating: a)) .| (v .== SIMD8(repeating: b))
        let cd = (v .== SIMD8(repeating: c)) .| (v .== SIMD8(repeating: d))
        return (ab .| cd) .| (v .== SIMD8(repeating: e))
    }

    @inline(__always)
    func matches(_ v: SIMD8<UInt16>) -> SIMDMask<SIMD8<Int16>> {
        let ab = (v .== SIMD8(repeating: UInt16(a))) .| (v .== SIMD8(repeating: UInt16(b)))
        let cd = (v .== SIMD8(repeating: UInt16(c))) .| (v .== SIMD8(repeating: UInt16(d)))
        return (ab .| cd) .| (v .== SIMD8(repeating: UInt16(e)))
    }

    @inline(__always)
    func matches(_ character: UInt16) -> Bool {
        character == UInt16(a) || character == UInt16(b) || character == UInt16(c) || character == UInt16(d) || character == UInt16(e)
    }
}

extension HTMLScan {
    // A 16-lane byte mask as its two little-endian 64-bit halves (bitCast, no lane shuffles: a narrowing conversion
    // here is not reliably inlined by the optimizer and becomes a call).
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

    // Index of the first non-zero byte of non-zero mask words.
    @inline(__always)
    static func firstNonZeroByte(_ words: SIMD2<UInt64>) -> Int {
        let low = words[0]
        if low != 0 {
            return low.trailingZeroBitCount &>> 3
        }
        return 8 &+ (words[1].trailingZeroBitCount &>> 3)
    }

    // An 8-lane byte mask as one word: trailing zero count / 8 is the first set lane.
    @inline(__always)
    static func maskWord(_ mask: SIMDMask<SIMD8<Int8>>) -> UInt64 {
        bitCast(SIMD8<UInt8>(repeating: 0).replacing(with: 0xFF, where: mask), to: UInt64.self)
    }

    // Index of the first byte at or after `start` (0 <= start <= count) that is in `set`, or characters.count.
    @inline(__always)
    static func firstIndex<Set: HTMLScanCharacterSet>(_ characters: Span<UInt8>, from start: Int, _ set: Set) -> Int {
        let count = characters.count
        let bytes = characters.bytes
        var index = start
        if count &- index >= 16 {
            // Most searches end in the first 16 bytes; longer runs go 32 bytes per step.
            let first = maskWords(set.matches(bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)))
            if !isZero(first) {
                return index &+ firstNonZeroByte(first)
            }
            index &+= 16
            while index &+ 32 <= count {
                let low = set.matches(bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self))
                let high = set.matches(bytes.load(fromByteOffset: index &+ 16, as: SIMD16<UInt8>.self))
                if !isZero(maskWords(low .| high)) {
                    let lowWords = maskWords(low)
                    if !isZero(lowWords) {
                        return index &+ firstNonZeroByte(lowWords)
                    }
                    return index &+ 16 &+ firstNonZeroByte(maskWords(high))
                }
                index &+= 32
            }
            if index &+ 16 <= count {
                let words = maskWords(set.matches(bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)))
                if !isZero(words) {
                    return index &+ firstNonZeroByte(words)
                }
                index &+= 16
            }
            if index < count {
                let last = count &- 16
                let words = maskWords(set.matches(bytes.load(fromByteOffset: last, as: SIMD16<UInt8>.self)))
                if !isZero(words) {
                    return last &+ firstNonZeroByte(words)
                }
            }
            return count
        }
        if count &- index >= 8 {
            let word = maskWord(set.matches(bytes.load(fromByteOffset: index, as: SIMD8<UInt8>.self)))
            if word != 0 {
                return index &+ (word.trailingZeroBitCount &>> 3)
            }
            let last = count &- 8
            let lastWord = maskWord(set.matches(bytes.load(fromByteOffset: last, as: SIMD8<UInt8>.self)))
            if lastWord != 0 {
                return last &+ (lastWord.trailingZeroBitCount &>> 3)
            }
            return count
        }
        while index < count {
            if set.matches(UInt16(characters[index])) {
                return index
            }
            index &+= 1
        }
        return count
    }

    // Index of the first UTF-16 code unit at or after `start` (0 <= start <= count) that is in `set`, or count.
    // `bytes` holds `count` code units.
    @inline(__always)
    static func firstIndex<Set: HTMLScanCharacterSet>(utf16 bytes: RawSpan, count: Int, from start: Int, _ set: Set) -> Int {
        var index = start
        if count &- index >= 8 {
            while index &+ 8 <= count {
                let words = maskWords(set.matches(bytes.load(fromByteOffset: index &* 2, as: SIMD8<UInt16>.self)))
                if !isZero(words) {
                    return index &+ (firstNonZeroByte(words) &>> 1)
                }
                index &+= 8
            }
            if index < count {
                let last = count &- 8
                let words = maskWords(set.matches(bytes.load(fromByteOffset: last &* 2, as: SIMD8<UInt16>.self)))
                if !isZero(words) {
                    return last &+ (firstNonZeroByte(words) &>> 1)
                }
            }
            return count
        }
        while index < count {
            if set.matches(bytes.load(fromByteOffset: index &* 2, as: UInt16.self)) {
                return index
            }
            index &+= 1
        }
        return count
    }
}
