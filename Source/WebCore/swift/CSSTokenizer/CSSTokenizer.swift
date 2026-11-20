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


// The whole CSS tokenizer (css/parser/CSSTokenizer.cpp and CSSTokenizerInputStream.cpp before): the input preprocessing
// check and rewrite, and every token of https://drafts.csswg.org/css-syntax/#tokenization exactly as WebKit's
// CSSTokenizer produced them (its nextToken() code-point dispatch, consumeNumber/NumericToken/IdentLikeToken/
// StringTokenUntil/URLToken/BadUrlRemnants/Name/Escape, comments, the block stack, convertToDimensionWithUnit's view
// merge). The C++ glue in CSSTokenizer.cpp only allocates: it gives this code the input, CSSTokenizer's own token
// storage to fill, the block stack storage and (once a token needs one) a pool for the code units of the strings the
// original built with a StringBuilder and registered in m_stringPool; it then turns the pool registrations into Strings.
//
// Token records are three words in CSSParserToken's own layout (CSSParserToken.h; the glue static_asserts and, in debug
// builds, checks it): word 0 = m_type | m_blockType << 6 | m_numericValueType << 8 | m_numericSign << 9 | m_unit << 11 |
// m_nonUnitPrefixLength << 18 | m_valueIs8Bit << 22 | m_valueLength << 32, word 1 = m_valueDataCharRaw (the input's
// address, an opaque integer this code never dereferences, plus the value's offset), word 2 = the union (m_id = -1 for
// the StringView constructor, the delimiter, the whitespace count, the HashTokenType or the double). A value held in the
// pool has bit 31 of word 0 set and its registration index in word 1; the glue points it at the String.
// Strictly memory-safe: no unsafe constructs in this file.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

// The tokenizer's input: StringView's characters, Latin-1 or UTF-16.
protocol CSSTokenizerCharacters: ~Escapable {
    var count: Int { get }
    subscript(_ index: Int) -> UInt16 { get }
    // The first index at or after `start` of a code unit that is not a name code point, of `quote`, '\\' or a
    // newline, of a URL token's end, or of '*': fast skips (the original's skipUntilMatch8Bit) that the scalar
    // loops after them re-check, so they may stop early; they return `start` when they don't apply.
    func skipNameCodePoints(from start: Int) -> Int
    func skipStringCharacters(from start: Int, quote: UInt8) -> Int
    func skipURLCharacters(from start: Int) -> Int
    func skipToAsterisk(from start: Int) -> Int
    // CSSTokenizer::preprocessString's test.
    func needsPreprocessing() -> Bool
    // Whether the whole input is one identifier with no escapes (a name start code point other than '-', then name code
    // points): its only token is that IdentToken (and it needs no preprocessing). Only Latin-1 input checks.
    func isPlainIdentifier() -> Bool
}

struct CSSTokenizerLatin1: CSSTokenizerCharacters, ~Escapable {
    let characters: Span<UInt8>

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(copy characters)
    init(_ characters: Span<UInt8>) {
        self.characters = characters
    }

    var count: Int {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { characters.count }
    }

    subscript(_ index: Int) -> UInt16 {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { UInt16(characters[index]) }
    }

    // Index of the first lane set in a 16-lane byte mask, or 16.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func firstLane(_ mask: SIMDMask<SIMD16<Int8>>) -> Int {
        let words = bitCast(SIMD16<UInt8>(repeating: 0).replacing(with: 0xFF, where: mask), to: SIMD2<UInt64>.self)
        if words[0] != 0 {
            return words[0].trailingZeroBitCount &>> 3
        }
        if words[1] != 0 {
            return 8 &+ (words[1].trailingZeroBitCount &>> 3)
        }
        return 16
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func needsPreprocessing() -> Bool {
        CSSTokenizerKernel.needsPreprocessing(characters)
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func isPlainIdentifier() -> Bool {
        let count = characters.count
        if count == 0 || !CSSTokenizerKernel.isNameStartCodePoint(UInt16(characters[0])) {
            return false
        }
        for index in 1..<count {
            if !CSSTokenizerKernel.isNameCodePoint(UInt16(characters[index])) {
                return false
            }
        }
        return true
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipNameCodePoints(from start: Int) -> Int {
        let bytes = characters.bytes
        var index = start
        while index >= 0 && index &+ 16 <= characters.count {
            let v = bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)
            let folded = v | SIMD16(repeating: 0x20)
            let isAlpha = (folded .>= SIMD16(repeating: 0x61)) .& (folded .<= SIMD16(repeating: 0x7A))
            let isDigit = (v .>= SIMD16(repeating: 0x30)) .& (v .<= SIMD16(repeating: 0x39))
            let isOther = (v .== SIMD16(repeating: 0x5F)) .| (v .== SIMD16(repeating: 0x2D)) .| (v .>= SIMD16(repeating: 0x80))
            let lane = Self.firstLane(.!(isAlpha .| isDigit .| isOther))
            index &+= lane
            if lane < 16 {
                break
            }
        }
        return index
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipStringCharacters(from start: Int, quote: UInt8) -> Int {
        let bytes = characters.bytes
        var index = start
        while index >= 0 && index &+ 16 <= characters.count {
            let v = bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)
            let ends = (v .== SIMD16(repeating: quote)) .| (v .== SIMD16(repeating: 0x5C)) .| (v .== SIMD16(repeating: 0x0A))
                .| (v .== SIMD16(repeating: 0x0D)) .| (v .== SIMD16(repeating: 0x0C))
            let lane = Self.firstLane(ends)
            index &+= lane
            if lane < 16 {
                break
            }
        }
        return index
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipURLCharacters(from start: Int) -> Int {
        let bytes = characters.bytes
        var index = start
        while index >= 0 && index &+ 16 <= characters.count {
            let v = bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self)
            let ends = (v .<= SIMD16(repeating: 0x20)) .| (v .== SIMD16(repeating: 0x29)) .| (v .== SIMD16(repeating: 0x5C))
                .| (v .== SIMD16(repeating: 0x22)) .| (v .== SIMD16(repeating: 0x27)) .| (v .== SIMD16(repeating: 0x28))
                .| (v .== SIMD16(repeating: 0x7F))
            let lane = Self.firstLane(ends)
            index &+= lane
            if lane < 16 {
                break
            }
        }
        return index
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipToAsterisk(from start: Int) -> Int {
        let bytes = characters.bytes
        var index = start
        while index >= 0 && index &+ 16 <= characters.count {
            let lane = Self.firstLane(bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self) .== SIMD16(repeating: 0x2A))
            index &+= lane
            if lane < 16 {
                break
            }
        }
        return index
    }
}

// UTF-16 input as bytes: StringView's 16-bit characters need not be 2-byte aligned.
struct CSSTokenizerUTF16: CSSTokenizerCharacters, ~Escapable {
    let bytes: RawSpan

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    var count: Int {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { bytes.byteCount &>> 1 }
    }

    subscript(_ index: Int) -> UInt16 {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { UInt16(littleEndian: bytes.load(fromByteOffset: index &* 2, as: UInt16.self)) }
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipNameCodePoints(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipStringCharacters(from start: Int, quote: UInt8) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipURLCharacters(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipToAsterisk(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func needsPreprocessing() -> Bool { CSSTokenizerKernel.needsPreprocessing(self) }
    func isPlainIdentifier() -> Bool { false }
}

// The code units of a value being built in the pool (for the unit and "url" checks of escaped names).
struct CSSTokenizerPoolCharacters: CSSTokenizerCharacters, ~Escapable {
    let units: Span<UInt16>

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(copy units)
    init(_ units: Span<UInt16>) {
        self.units = units
    }

    var count: Int {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { units.count }
    }

    subscript(_ index: Int) -> UInt16 {
#if compiler(>=6.4)
        @inline(always)
#else
        @inline(__always)
#endif
        get { units[index] }
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipNameCodePoints(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipStringCharacters(from start: Int, quote: UInt8) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipURLCharacters(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func skipToAsterisk(from start: Int) -> Int { start }
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func needsPreprocessing() -> Bool { false }
    func isPlainIdentifier() -> Bool { false }
}

enum CSSTokenizerKernel {
    // CSSParserTokenType.
    static let identToken: UInt64 = 0
    static let functionToken: UInt64 = 1
    static let atKeywordToken: UInt64 = 2
    static let hashToken: UInt64 = 3
    static let urlToken: UInt64 = 4
    static let badUrlToken: UInt64 = 5
    static let delimiterToken: UInt64 = 6
    static let numberToken: UInt64 = 7
    static let percentageToken: UInt64 = 8
    static let dimensionToken: UInt64 = 9
    static let includeMatchToken: UInt64 = 10
    static let dashMatchToken: UInt64 = 11
    static let prefixMatchToken: UInt64 = 12
    static let suffixMatchToken: UInt64 = 13
    static let substringMatchToken: UInt64 = 14
    static let columnToken: UInt64 = 15
    static let nonNewlineWhitespaceToken: UInt64 = 16
    static let newlineToken: UInt64 = 17
    static let cdoToken: UInt64 = 18
    static let cdcToken: UInt64 = 19
    static let colonToken: UInt64 = 20
    static let semicolonToken: UInt64 = 21
    static let commaToken: UInt64 = 22
    static let leftParenthesisToken: UInt64 = 23
    static let rightParenthesisToken: UInt64 = 24
    static let leftBracketToken: UInt64 = 25
    static let rightBracketToken: UInt64 = 26
    static let leftBraceToken: UInt64 = 27
    static let rightBraceToken: UInt64 = 28
    static let stringToken: UInt64 = 29
    static let badStringToken: UInt64 = 30
    static let eofToken: UInt64 = 31
    static let commentToken: UInt64 = 32

    // CSSParserToken::BlockType, at bit 6.
    static let blockStart: UInt64 = 1 << 6
    static let blockEnd: UInt64 = 2 << 6

    // Word 0 fields.
    static let numberValueTypeBit: UInt64 = 1 << 8 // NumberValueType (IntegerValueType is 0)
    static let signShift: UInt64 = 9
    static let unitShift: UInt64 = 11
    static let prefixShift: UInt64 = 18
    static let is8BitBit: UInt64 = 1 << 22
    static let registrationBit: UInt64 = 1 << 31
    static let lengthShift: UInt64 = 32
    // m_id = -1 (the StringView constructor), in the union's low 32 bits.
    static let unresolvedID: UInt64 = 0xFFFF_FFFF

    // Statuses (the high 32 bits of the entry point's result).
    static let statusDone: UInt64 = 0
    static let statusTokensFull: UInt64 = 1
    static let statusNeedBlockStack: UInt64 = 2
    static let statusNeedPool: UInt64 = 3
    static let statusNeedPreprocessing: UInt64 = 4

    // The state words (CSSTokenizerSwiftBoundary.h).
    static let statePosition = 0
    static let stateBlockDepth = 1
    static let statePoolUsed = 2
    static let stateRegistrationCount = 3
    static let stateFlags = 11
    static let stateCount = 12
    static let flagPreprocessed: UInt64 = 1
    static let flagObserving: UInt64 = 2

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isASCIIDigit(_ c: UInt16) -> Bool {
        c &- 0x30 < 10
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isASCIIAlpha(_ c: UInt16) -> Bool {
        (c | 0x20) &- 0x61 < 26
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isASCIIHexDigit(_ c: UInt16) -> Bool {
        isASCIIDigit(c) || (c | 0x20) &- 0x61 < 6
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func hexValue(_ c: UInt16) -> UInt32 {
        isASCIIDigit(c) ? UInt32(c &- 0x30) : UInt32((c | 0x20) &- 0x61 &+ 10)
    }

    // isCSSNewline
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isNewline(_ c: UInt16) -> Bool {
        c == 0x0A || c == 0x0D || c == 0x0C
    }

    // isASCIIWhitespace: ' ', '\n', '\t', '\r', '\f'.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isASCIIWhitespace(_ c: UInt16) -> Bool {
        c == 0x20 || c == 0x0A || c == 0x09 || c == 0x0D || c == 0x0C
    }

    // isNameStartCodePoint: ASCII letters, '_' and non-ASCII. isNameCodePoint adds digits and '-'.
    static let nameStartMask: UInt64 = 0x07FF_FFFE_87FF_FFFE // 'A'-'Z', '_', 'a'-'z' as bits 0x41 - 64 ...
    static let nameDigitHyphenMask: UInt64 = 0x03FF_2000_0000_0000 // '-' and '0'-'9' as bits 0x2D ...

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isNameStartCodePoint(_ c: UInt16) -> Bool {
        c >= 0x80 || (c >= 0x40 && (nameStartMask &>> UInt64(c & 63)) & 1 != 0)
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isNameCodePoint(_ c: UInt16) -> Bool {
        if c >= 0x80 {
            return true
        }
        let mask = c >= 0x40 ? nameStartMask : nameDigitHyphenMask
        return (mask &>> UInt64(c & 63)) & 1 != 0
    }

    // The code units that end consumeURLToken's fast loop: ')' and, for its slow case, those <= ' ', '\\', '"', '\'',
    // '(' and DEL.
    static let urlSpecialLowMask: UInt64 = 0x0000_0385_FFFF_FFFF // 0x00-0x20, '"', '\'', '(', ')'
    static let urlSpecialHighMask: UInt64 = 0x8000_0000_1000_0000 // '\\' (0x5C), DEL (0x7F)

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isURLSpecial(_ c: UInt16) -> Bool {
        if c >= 0x80 {
            return false
        }
        let mask = c >= 0x40 ? urlSpecialHighMask : urlSpecialLowMask
        return (mask &>> UInt64(c & 63)) & 1 != 0
    }

    // twoCharsAreValidEscape
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isValidEscape(_ first: UInt16, _ second: UInt16) -> Bool {
        first == 0x5C && !isNewline(second)
    }

    // isNonPrintableCodePoint
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isNonPrintable(_ c: UInt16) -> Bool {
        c <= 0x08 || c == 0x0B || (c &- 0x0E) <= (0x1F &- 0x0E) || c == 0x7F
    }

    // CSSTokenizer::preprocessString's test: Latin-1 input needs it when it has a NUL.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    static func needsPreprocessing(_ characters: Span<UInt8>) -> Bool {
        let count = characters.count
        let bytes = characters.bytes
        if count < 8 {
            for index in 0..<count {
                if characters[index] == 0 {
                    return true
                }
            }
            return false
        }
        if count <= 16 {
            // Two overlapping words: a word has a zero byte when (w - 0x01..01) & ~w & 0x80..80 is not zero.
            let first = bytes.load(fromByteOffset: 0, as: UInt64.self)
            let last = bytes.load(fromByteOffset: count &- 8, as: UInt64.self)
            let low: UInt64 = 0x0101_0101_0101_0101
            let high: UInt64 = 0x8080_8080_8080_8080
            return (((first &- low) & ~first) | ((last &- low) & ~last)) & high != 0
        }
        var index = 0
        while index &+ 16 <= count {
            if CSSTokenizerLatin1.firstLane(bytes.load(fromByteOffset: index, as: SIMD16<UInt8>.self) .== SIMD16(repeating: 0)) < 16 {
                return true
            }
            index &+= 16
        }
        if index < count {
            return CSSTokenizerLatin1.firstLane(bytes.load(fromByteOffset: count &- 16, as: SIMD16<UInt8>.self) .== SIMD16(repeating: 0)) < 16
        }
        return false
    }

    // UTF-16 input needs it when it has a NUL or an unpaired surrogate (WTF::CodePointIterator's pairing).
    static func needsPreprocessing(_ characters: CSSTokenizerUTF16) -> Bool {
        let count = characters.count
        let bytes = characters.bytes
        var index = 0
        var candidate = false
        while index &+ 8 <= count {
            let v = bytes.load(fromByteOffset: index &* 2, as: SIMD8<UInt16>.self)
            let hit = (v .== SIMD8(repeating: 0)) .| ((v & SIMD8(repeating: 0xF800)) .== SIMD8(repeating: 0xD800))
            if any(hit) {
                candidate = true
                break
            }
            index &+= 8
        }
        if !candidate {
            while index < count {
                let c = characters[index]
                if c == 0 || (c & 0xF800) == 0xD800 {
                    candidate = true
                    break
                }
                index &+= 1
            }
            if !candidate {
                return false
            }
        }
        index = 0
        while index < count {
            let c = characters[index]
            if c == 0 {
                return true
            }
            if (c & 0xF800) == 0xD800 {
                if c <= 0xDBFF && index &+ 1 < count && (characters[index &+ 1] & 0xFC00) == 0xDC00 {
                    index &+= 2
                    continue
                }
                return true
            }
            index &+= 1
        }
        return false
    }

    // The preprocessed (always 16-bit, same length) string: NUL and, in UTF-16 input, every unpaired surrogate
    // become U+FFFD.
    static func preprocess(_ characters: Span<UInt8>, into output: inout MutableSpan<UInt16>) {
        precondition(output.count == characters.count)
        for index in 0..<characters.count {
            let c = characters[index]
            output[index] = c == 0 ? 0xFFFD : UInt16(c)
        }
    }

    static func preprocess(_ characters: CSSTokenizerUTF16, into output: inout MutableSpan<UInt16>) {
        let count = characters.count
        precondition(output.count == count)
        var index = 0
        while index < count {
            let c = characters[index]
            if (c & 0xF800) == 0xD800 {
                if c <= 0xDBFF && index &+ 1 < count && (characters[index &+ 1] & 0xFC00) == 0xDC00 {
                    output[index] = c
                    output[index &+ 1] = characters[index &+ 1]
                    index &+= 2
                    continue
                }
                output[index] = 0xFFFD
            } else {
                output[index] = c == 0 ? 0xFFFD : c
            }
            index &+= 1
        }
    }
}

extension CSSTokenizerKernel {
    // One call of WebCoreSwift_CSSTokenizer_tokenize: resumes the tokenization the state words describe.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    static func tokenize<C: CSSTokenizerCharacters & ~Escapable>(_ input: C, is8Bit: Bool, inputBase: UInt64, tokens: consuming MutableRawSpan,
        blockStack: consuming MutableSpan<UInt8>, pool: consuming MutableSpan<UInt16>, registrations: consuming MutableSpan<UInt64>,
        offsets: consuming MutableSpan<UInt32>, observing: Bool, state: inout MutableSpan<UInt64>) -> UInt64
    {
        let position = Int(truncatingIfNeeded: state[statePosition])
        if position == 0 && (state[stateFlags] & flagPreprocessed) == 0 {
            if !observing && tokens.byteCount >= 24 && input.isPlainIdentifier() {
                // The common one-identifier input ("auto", "red"): its IdentToken, then EOF at the end.
                tokens.storeBytes(of: identToken | is8BitBit | UInt64(truncatingIfNeeded: input.count) &<< lengthShift, toByteOffset: 0, as: UInt64.self)
                tokens.storeBytes(of: inputBase, toByteOffset: 8, as: UInt64.self)
                tokens.storeBytes(of: unresolvedID, toByteOffset: 16, as: UInt64.self)
                state[statePosition] = UInt64(truncatingIfNeeded: input.count)
                state[stateFlags] |= flagPreprocessed
                return 1 | statusDone &<< 32
            }
            if input.needsPreprocessing() {
                return statusNeedPreprocessing &<< 32
            }
        }
        var run = CSSTokenizerRun(input: input, is8Bit: is8Bit, inputBase: inputBase, observing: observing, position: position, tokens: tokens,
            blockStack: blockStack, depth: Int(truncatingIfNeeded: state[stateBlockDepth]), offsets: offsets)
        var tokenizerPool = CSSTokenizerPool(units: pool, registrations: registrations, used: Int(truncatingIfNeeded: state[statePoolUsed]),
            registrationCount: Int(truncatingIfNeeded: state[stateRegistrationCount]))
        let status = run.run(&tokenizerPool)
        state[statePosition] = UInt64(truncatingIfNeeded: run.position)
        state[stateBlockDepth] = UInt64(truncatingIfNeeded: run.depth)
        state[statePoolUsed] = UInt64(truncatingIfNeeded: tokenizerPool.used)
        state[stateRegistrationCount] = UInt64(truncatingIfNeeded: tokenizerPool.registrationCount)
        // The preprocessing check is done once, even if the input needs no records yet.
        state[stateFlags] |= flagPreprocessed
        return UInt64(truncatingIfNeeded: run.tokenCount) | status &<< 32
    }
}

// Double's bits (Double.bitPattern stays a call into the standard library in the tokenizer loop).
#if compiler(>=6.4)
@inline(always)
#else
@inline(__always)
#endif
func cssTokenizerBits(_ value: Double) -> UInt64 {
    bitCast(value, to: UInt64.self)
}

struct CSSTokenizerToken {
    var word0: UInt64
    var word1: UInt64 = 0
    var word2: UInt64 = 0

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    init(_ word0: UInt64, _ word1: UInt64 = 0, _ word2: UInt64 = 0) {
        self.word0 = word0
        self.word1 = word1
        self.word2 = word2
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func delimiter(_ c: UInt16) -> CSSTokenizerToken {
        CSSTokenizerToken(CSSTokenizerKernel.delimiterToken, 0, UInt64(c))
    }
}

// A token's string value: in the input (offset in `start`) or a pool registration (index in `start`).
struct CSSTokenizerValue {
    var start: Int
    var length: Int
    var isRegistration: Bool
    var is8Bit: Bool
}

// The pool: code units of the values the original built with a StringBuilder and registered in m_stringPool. Only the
// slow cases (escapes, unterminated strings, URLs with spaces) touch it. A token that needs it before the glue has
// provided it (or with too few registrations left) is abandoned and redone after the glue provides it.
struct CSSTokenizerPool: ~Copyable, ~Escapable {
    var units: MutableSpan<UInt16>
    var registrations: MutableSpan<UInt64>
    var used: Int
    var registrationCount: Int
    // used and registrationCount when the current token first reserved the pool.
    var tokenUsed = 0
    var tokenRegistrationCount = 0
    // 0: the current token has not used the pool; 1: it has; 2: it needs a pool the glue has not provided.
    var status = 0

    @_lifetime(copy units, copy registrations)
    init(units: consuming MutableSpan<UInt16>, registrations: consuming MutableSpan<UInt64>, used: Int, registrationCount: Int) {
        self.units = units
        self.registrations = registrations
        self.used = used
        self.registrationCount = registrationCount
    }

    // Whether the pool can take every value of this token (a token registers at most two, and the values of the
    // whole input never have more code units than the input; the glue's pool has length + 2).
    @_lifetime(self: copy self)
    mutating func reserve(inputCount: Int) -> Bool {
        if status == 0 {
            tokenUsed = used
            tokenRegistrationCount = registrationCount
        }
        if units.count >= inputCount &+ 2 && registrations.count &- tokenRegistrationCount >= 2 {
            status = 1
            return true
        }
        status = 2
        return false
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func append(_ unit: UInt16, _ allUnits: inout UInt16) {
        units[used] = unit
        used &+= 1
        allUnits |= unit
    }

    // StringBuilder::append(char32_t).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func append(codePoint: UInt32, _ allUnits: inout UInt16) {
        if codePoint <= 0xFFFF {
            append(UInt16(truncatingIfNeeded: codePoint), &allUnits)
        } else {
            append(UInt16(truncatingIfNeeded: 0xD7C0 &+ (codePoint &>> 10)), &allUnits)
            append(UInt16(truncatingIfNeeded: 0xDC00 | (codePoint & 0x3FF)), &allUnits)
        }
    }

    // registerString(builder.toString()): the code units from `start`; 8-bit when they all are Latin-1.
    @_lifetime(self: copy self)
    mutating func register(_ start: Int, _ allUnits: UInt16) -> CSSTokenizerValue {
        let length = used &- start
        let is8Bit = allUnits <= 0xFF
        registrations[registrationCount] = UInt64(truncatingIfNeeded: start) | UInt64(truncatingIfNeeded: length) &<< 32 | (is8Bit ? 1 &<< 63 : 0)
        let index = registrationCount
        registrationCount &+= 1
        return CSSTokenizerValue(start: index, length: length, isRegistration: true, is8Bit: is8Bit)
    }

    // The pool offset of a registered value.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func start(of value: CSSTokenizerValue) -> Int {
        Int(truncatingIfNeeded: registrations[value.start] & 0xFFFF_FFFF)
    }
}

// One tokenization (resumable at token boundaries: the position and the block stack are its only state). Everything
// but the slow cases is inlined into the entry point, so the position and counts live in registers.
struct CSSTokenizerRun<C: CSSTokenizerCharacters & ~Escapable>: ~Copyable, ~Escapable {
    typealias K = CSSTokenizerKernel

    let input: C
    let inputBase: UInt64
    let characterShift: UInt64
    let inputIs8Bit: UInt64
    let observing: Bool
    var position: Int
    var tokens: MutableRawSpan
    var tokenCount = 0
    var blockStack: MutableSpan<UInt8>
    var depth: Int
    var offsets: MutableSpan<UInt32>

    @_lifetime(copy input, copy tokens, copy blockStack, copy offsets)
    init(input: C, is8Bit: Bool, inputBase: UInt64, observing: Bool, position: Int, tokens: consuming MutableRawSpan,
        blockStack: consuming MutableSpan<UInt8>, depth: Int, offsets: consuming MutableSpan<UInt32>)
    {
        self.input = input
        self.inputBase = inputBase
        self.characterShift = is8Bit ? 0 : 1
        self.inputIs8Bit = is8Bit ? K.is8BitBit : 0
        self.observing = observing
        self.position = position
        self.tokens = tokens
        self.blockStack = blockStack
        self.depth = depth
        self.offsets = offsets
    }

    // m_input.peek / nextInputChar: the code unit at `index`, NUL (kEndOfFileMarker) at or past the end.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func at(_ input: C, _ index: Int) -> UInt16 {
        UInt(bitPattern: index) < UInt(bitPattern: input.count) ? input[index] : 0
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func at(_ index: Int) -> UInt16 {
        Self.at(input, index)
    }

    // Tokenizes from `position` until EOF, a full token chunk, a full block stack or a needed pool; returns the
    // status. Record i < tokenCount is the 24 bytes at 24 * i (three words).
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func run(_ pool: inout CSSTokenizerPool) -> UInt64 {
        let tokenCapacity = tokens.byteCount / 24
        let blockStackCapacity = blockStack.count
        while true {
            if tokenCount >= tokenCapacity {
                return K.statusTokensFull
            }
            if depth >= blockStackCapacity {
                return K.statusNeedBlockStack
            }
            let start = position
            let token = nextToken(&pool)
            if pool.status != 0 {
                if pool.status == 2 {
                    position = start
                    pool.used = pool.tokenUsed
                    pool.registrationCount = pool.tokenRegistrationCount
                    pool.status = 0
                    return K.statusNeedPool
                }
                pool.status = 0
            }
            let type = token.word0 & 0x3F
            if type == K.eofToken {
                position = min(start, input.count)
                return K.statusDone
            }
            if type == K.commentToken && !observing {
                continue
            }
            let record = 24 &* tokenCount
            tokens.storeBytes(of: token.word0, toByteOffset: record, as: UInt64.self)
            tokens.storeBytes(of: token.word1, toByteOffset: record &+ 8, as: UInt64.self)
            tokens.storeBytes(of: token.word2, toByteOffset: record &+ 16, as: UInt64.self)
            if observing {
                offsets[tokenCount] = UInt32(truncatingIfNeeded: min(start, input.count))
            }
            tokenCount &+= 1
        }
    }

    // CSSTokenizer::nextToken().
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func nextToken(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let cc = at(position)
        position &+= 1
        switch cc {
        case 0x00:
            return CSSTokenizerToken(K.eofToken)
        case 0x09, 0x20:
            return whitespace()
        case 0x0A, 0x0C, 0x0D:
            return CSSTokenizerToken(K.newlineToken)
        case 0x22, 0x27:
            return consumeStringToken(until: cc, &pool)
        case 0x23:
            return hash(&pool)
        case 0x24:
            return consumeIfNext(0x3D) ? CSSTokenizerToken(K.suffixMatchToken) : .delimiter(cc)
        case 0x28:
            return blockStart(K.leftParenthesisToken)
        case 0x29:
            return blockEnd(K.rightParenthesisToken, K.leftParenthesisToken)
        case 0x2A:
            return consumeIfNext(0x3D) ? CSSTokenizerToken(K.substringMatchToken) : .delimiter(cc)
        case 0x2B, 0x2E:
            if nextCharsAreNumber(cc) {
                position &-= 1
                return consumeNumericToken(&pool)
            }
            return .delimiter(cc)
        case 0x2C:
            return CSSTokenizerToken(K.commaToken)
        case 0x2D:
            return hyphenMinus(&pool)
        case 0x2F:
            if consumeIfNext(0x2A) {
                consumeUntilCommentEndFound()
                return CSSTokenizerToken(K.commentToken)
            }
            return .delimiter(cc)
        case 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39:
            position &-= 1
            return consumeNumericToken(&pool)
        case 0x3A:
            return CSSTokenizerToken(K.colonToken)
        case 0x3B:
            return CSSTokenizerToken(K.semicolonToken)
        case 0x3C:
            if at(position) == 0x21 && at(position &+ 1) == 0x2D && at(position &+ 2) == 0x2D {
                position &+= 3
                return CSSTokenizerToken(K.cdoToken)
            }
            return .delimiter(cc)
        case 0x40:
            if wouldStartIdentifier(position) {
                return named(K.atKeywordToken, consumeName(&pool))
            }
            return .delimiter(cc)
        case 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D,
            0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5F,
            0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D,
            0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A:
            position &-= 1
            return consumeIdentLikeToken(&pool)
        case 0x5B:
            return blockStart(K.leftBracketToken)
        case 0x5C:
            if K.isValidEscape(cc, at(position)) {
                position &-= 1
                return consumeIdentLikeToken(&pool)
            }
            return .delimiter(cc)
        case 0x5D:
            return blockEnd(K.rightBracketToken, K.leftBracketToken)
        case 0x5E:
            return consumeIfNext(0x3D) ? CSSTokenizerToken(K.prefixMatchToken) : .delimiter(cc)
        case 0x7B:
            return blockStart(K.leftBraceToken)
        case 0x7C:
            if consumeIfNext(0x3D) {
                return CSSTokenizerToken(K.dashMatchToken)
            }
            if consumeIfNext(0x7C) {
                return CSSTokenizerToken(K.columnToken)
            }
            return .delimiter(cc)
        case 0x7D:
            return blockEnd(K.rightBraceToken, K.leftBraceToken)
        case 0x7E:
            return consumeIfNext(0x3D) ? CSSTokenizerToken(K.includeMatchToken) : .delimiter(cc)
        default:
            if cc >= 0x80 {
                position &-= 1
                return consumeIdentLikeToken(&pool)
            }
            return .delimiter(cc)
        }
    }

    // CSSTokenizer::whitespace: the run of ' ' and '\t' (advanceUntilNewlineOrNonWhitespace).
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func whitespace() -> CSSTokenizerToken {
        let startOffset = position
        while position < input.count {
            let c = input[position]
            if c != 0x20 && c != 0x09 {
                break
            }
            position &+= 1
        }
        return CSSTokenizerToken(K.nonNewlineWhitespaceToken, 0, UInt64(1 &+ (position &- startOffset)))
    }

#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeIfNext(_ c: UInt16) -> Bool {
        if at(position) == c {
            position &+= 1
            return true
        }
        return false
    }

#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func blockStart(_ type: UInt64) -> CSSTokenizerToken {
        blockStack[depth] = UInt8(truncatingIfNeeded: type)
        depth &+= 1
        return CSSTokenizerToken(type | K.blockStart)
    }

#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func blockEnd(_ type: UInt64, _ startType: UInt64) -> CSSTokenizerToken {
        if depth > 0 && UInt64(blockStack[depth &- 1]) == startType {
            depth &-= 1
            return CSSTokenizerToken(type | K.blockEnd)
        }
        return CSSTokenizerToken(type)
    }

    // A token whose value is `value` (the StringView constructor: m_id = -1).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func named(_ typeAndBlock: UInt64, _ value: CSSTokenizerValue) -> CSSTokenizerToken {
        CSSTokenizerToken(typeAndBlock | valueWord0(value), valueWord1(value), K.unresolvedID)
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func valueWord0(_ value: CSSTokenizerValue) -> UInt64 {
        var word = UInt64(truncatingIfNeeded: value.length) &<< K.lengthShift
        if value.isRegistration {
            word |= K.registrationBit | (value.is8Bit ? K.is8BitBit : 0)
        } else {
            word |= inputIs8Bit
        }
        return word
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func valueWord1(_ value: CSSTokenizerValue) -> UInt64 {
        if value.isRegistration {
            return UInt64(truncatingIfNeeded: value.start)
        }
        return inputBase &+ (UInt64(truncatingIfNeeded: value.start) &<< characterShift)
    }

    // nextCharsAreNumber(first), `first` consumed.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func nextCharsAreNumber(_ first: UInt16) -> Bool {
        let second = at(position)
        if K.isASCIIDigit(first) {
            return true
        }
        if first == 0x2B || first == 0x2D {
            return K.isASCIIDigit(second) || (second == 0x2E && K.isASCIIDigit(at(position &+ 1)))
        }
        if first == 0x2E {
            return K.isASCIIDigit(second)
        }
        return false
    }

    // nextCharsAreIdentifier() at `index` (https://drafts.csswg.org/css-syntax/#would-start-an-identifier).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func wouldStartIdentifier(_ index: Int) -> Bool {
        let first = at(index)
        let second = at(index &+ 1)
        if K.isNameStartCodePoint(first) || K.isValidEscape(first, second) {
            return true
        }
        if first == 0x2D {
            return K.isNameStartCodePoint(second) || second == 0x2D || K.isValidEscape(second, at(index &+ 2))
        }
        return false
    }

    // CSSTokenizer::hyphenMinus, '-' consumed.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func hyphenMinus(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        if nextCharsAreNumber(0x2D) {
            position &-= 1
            return consumeNumericToken(&pool)
        }
        if at(position) == 0x2D && at(position &+ 1) == 0x3E {
            position &+= 2
            return CSSTokenizerToken(K.cdcToken)
        }
        if wouldStartIdentifier(position &- 1) {
            position &-= 1
            return consumeIdentLikeToken(&pool)
        }
        return .delimiter(0x2D)
    }

    // CSSTokenizer::hash, '#' consumed.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func hash(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let next = at(position)
        if K.isNameCodePoint(next) || K.isValidEscape(next, at(position &+ 1)) {
            let hashType: UInt64 = wouldStartIdentifier(position) ? 0 : 1 // HashTokenId, HashTokenUnrestricted
            let name = consumeName(&pool)
            return CSSTokenizerToken(K.hashToken | valueWord0(name), valueWord1(name), hashType)
        }
        return .delimiter(0x23)
    }

    // consumeNumber + consumeNumericToken: the number (its value is fast_float's), then a unit or '%'.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeNumericToken(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let start = position
        var p = start
        var c = at(p)
        var sign: UInt64 = 0 // NoSign
        if c == 0x2B {
            p &+= 1
            sign = 1 // PlusSign
        } else if c == 0x2D {
            p &+= 1
            sign = 2 // MinusSign
        }
        var mantissa: UInt64 = 0
        let integerStart = p
        while true {
            c = at(p)
            if !K.isASCIIDigit(c) {
                break
            }
            mantissa = mantissa &* 10 &+ UInt64(c &- 0x30)
            p &+= 1
        }
        var digitCount = p &- integerStart
        var exponent: Int64 = 0
        var valueType: UInt64 = 0 // IntegerValueType
        if c == 0x2E && K.isASCIIDigit(at(p &+ 1)) {
            valueType = K.numberValueTypeBit
            p &+= 1
            let fractionStart = p
            while true {
                c = at(p)
                if !K.isASCIIDigit(c) {
                    break
                }
                mantissa = mantissa &* 10 &+ UInt64(c &- 0x30)
                p &+= 1
            }
            digitCount &+= p &- fractionStart
            exponent = Int64(fractionStart &- p)
        }
        if c == 0x45 || c == 0x65 {
            let next = at(p &+ 1)
            var digitsStart = -1
            var negativeExponent = false
            if K.isASCIIDigit(next) {
                digitsStart = p &+ 1
            } else if (next == 0x2B || next == 0x2D) && K.isASCIIDigit(at(p &+ 2)) {
                digitsStart = p &+ 2
                negativeExponent = next == 0x2D
            }
            if digitsStart >= 0 {
                valueType = K.numberValueTypeBit
                p = digitsStart
                var exponentNumber: Int64 = 0
                while true {
                    c = at(p)
                    if !K.isASCIIDigit(c) {
                        break
                    }
                    if exponentNumber < 0x10000000 {
                        exponentNumber = 10 &* exponentNumber &+ Int64(c &- 0x30)
                    }
                    p &+= 1
                }
                exponent &+= negativeExponent ? 0 &- exponentNumber : exponentNumber
            }
        }
        let numberLength = p &- start
        let value = CSSTokenizerNumber.value(input, start, p, mantissa: mantissa, exponent: exponent, digitCount: digitCount, negative: sign == 2)
        position = p

        let numberFields = valueType | sign &<< K.signShift
        if wouldStartIdentifier(position) {
            // convertToDimensionWithUnit(consumeName()): the number text and the unit share one view when the
            // number text is 1-15 characters and the unit is in the input right after it (mergeIfAdjacent).
            let unit = consumeName(&pool)
            let unitType: UInt8
            if unit.isRegistration {
                let unitStart = pool.start(of: unit)
                unitType = CSSTokenizerUnits.unitType(CSSTokenizerPoolCharacters(pool.units.span.extracting(unitStart..<unitStart &+ unit.length)), 0, unit.length)
            } else if pool.status == 2 {
                return CSSTokenizerToken(K.eofToken)
            } else {
                unitType = CSSTokenizerUnits.unitType(input, unit.start, unit.length)
            }
            var word0 = K.dimensionToken | numberFields | UInt64(unitType) &<< K.unitShift
            if !unit.isRegistration && numberLength < 16 {
                let merged = CSSTokenizerValue(start: start, length: numberLength &+ unit.length, isRegistration: false, is8Bit: false)
                word0 |= UInt64(numberLength) &<< K.prefixShift | valueWord0(merged)
                return CSSTokenizerToken(word0, valueWord1(merged), cssTokenizerBits(value))
            }
            return CSSTokenizerToken(word0 | valueWord0(unit), valueWord1(unit), cssTokenizerBits(value))
        }
        let text = CSSTokenizerValue(start: start, length: numberLength, isRegistration: false, is8Bit: false)
        if consumeIfNext(0x25) {
            return CSSTokenizerToken(K.percentageToken | numberFields | UInt64(CSSTokenizerUnits.percentage) &<< K.unitShift | valueWord0(text), valueWord1(text), cssTokenizerBits(value))
        }
        return CSSTokenizerToken(K.numberToken | numberFields | UInt64(CSSTokenizerUnits.number) &<< K.unitShift | valueWord0(text), valueWord1(text), cssTokenizerBits(value))
    }

    // The i-th code unit of a value.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    func valueCharacter(_ value: CSSTokenizerValue, _ index: Int, _ pool: borrowing CSSTokenizerPool) -> UInt16 {
        if value.isRegistration {
            return pool.units[pool.start(of: value) &+ index]
        }
        return input[value.start &+ index]
    }

    // CSSTokenizer::consumeIdentLikeToken.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeIdentLikeToken(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let name = consumeName(&pool)
        if at(position) == 0x28 {
            position &+= 1
            if name.length == 3 && (valueCharacter(name, 0, pool) | 0x20) == 0x75 && (valueCharacter(name, 1, pool) | 0x20) == 0x72
                && (valueCharacter(name, 2, pool) | 0x20) == 0x6C {
                // url(: the spec's whitespace tokens would be dropped anyway.
                while position < input.count && K.isASCIIWhitespace(input[position]) {
                    position &+= 1
                }
                let next = at(position)
                if next != 0x22 && next != 0x27 {
                    return consumeURLToken(&pool)
                }
            }
            if pool.status == 2 {
                return CSSTokenizerToken(K.eofToken)
            }
            blockStack[depth] = UInt8(truncatingIfNeeded: K.leftParenthesisToken)
            depth &+= 1
            return named(K.functionToken | K.blockStart, name)
        }
        return named(K.identToken, name)
    }

    // CSSTokenizer::consumeName: an input view, or a pool registration when it has escapes (or ends at a '\\').
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeName(_ pool: inout CSSTokenizerPool) -> CSSTokenizerValue {
        let start = position
        var end = input.skipNameCodePoints(from: start)
        while true {
            let cc = at(end)
            if K.isNameCodePoint(cc) {
                end &+= 1
                continue
            }
            if cc == 0x5C || (cc == 0 && end < input.count) {
                break
            }
            position = end
            return CSSTokenizerValue(start: start, length: end &- start, isRegistration: false, is8Bit: false)
        }
        let (newPosition, value) = Self.consumeNameSlowCase(input, start, &pool)
        position = newPosition
        return value
    }

    // CSSTokenizer::consumeStringTokenUntil, the quote consumed.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeStringToken(until ending: UInt16, _ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let start = position
        var end = input.skipStringCharacters(from: start, quote: UInt8(truncatingIfNeeded: ending))
        while true {
            let cc = at(end)
            if cc == ending {
                position = end &+ 1
                return named(K.stringToken, CSSTokenizerValue(start: start, length: end &- start, isRegistration: false, is8Bit: false))
            }
            if K.isNewline(cc) {
                position = end
                return CSSTokenizerToken(K.badStringToken)
            }
            if cc == 0 || cc == 0x5C {
                break
            }
            end &+= 1
        }
        let (newPosition, value) = Self.consumeStringTokenSlowCase(input, start, ending, &pool)
        position = newPosition
        if let value {
            return named(K.stringToken, value)
        }
        return CSSTokenizerToken(K.badStringToken)
    }

    // CSSTokenizer::consumeURLToken, after "url(" and its whitespace.
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeURLToken(_ pool: inout CSSTokenizerPool) -> CSSTokenizerToken {
        let start = position
        var end = input.skipURLCharacters(from: start)
        while true {
            let cc = at(end)
            if K.isURLSpecial(cc) {
                if cc == 0x29 {
                    position = end &+ 1
                    return named(K.urlToken, CSSTokenizerValue(start: start, length: end &- start, isRegistration: false, is8Bit: false))
                }
                break
            }
            end &+= 1
        }
        let (newPosition, value) = Self.consumeURLTokenSlowCase(input, start, &pool)
        position = newPosition
        if let value {
            return named(K.urlToken, value)
        }
        return CSSTokenizerToken(K.badUrlToken)
    }

    // CSSTokenizer::consumeUntilCommentEndFound, after "/*".
#if compiler(>=6.4)
    @inline(always)
#else
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
#endif
    @_lifetime(self: copy self)
    mutating func consumeUntilCommentEndFound() {
        var c = at(position)
        position &+= 1
        while true {
            if c == 0 {
                return
            }
            if c != 0x2A {
                position = input.skipToAsterisk(from: position)
                c = at(position)
                position &+= 1
                continue
            }
            c = at(position)
            position &+= 1
            if c == 0x2F {
                return
            }
        }
    }

    // The slow cases (the C++ StringBuilder paths): static, so the hot state above never leaves registers.

    // consumeName's slow case, from `start`: returns the position after the name and its registration.
    @inline(never)
    static func consumeNameSlowCase(_ input: C, _ start: Int, _ pool: inout CSSTokenizerPool) -> (Int, CSSTokenizerValue) {
        guard pool.reserve(inputCount: input.count) else {
            return (start, CSSTokenizerValue(start: start, length: 0, isRegistration: false, is8Bit: false))
        }
        var position = start
        let poolStart = pool.used
        var allUnits: UInt16 = 0
        while true {
            let cc = at(input, position)
            position &+= 1
            if K.isNameCodePoint(cc) {
                pool.append(cc, &allUnits)
                continue
            }
            if K.isValidEscape(cc, at(input, position)) {
                pool.append(codePoint: consumeEscape(input, &position), &allUnits)
                continue
            }
            position &-= 1
            return (position, pool.register(poolStart, allUnits))
        }
    }

    // CSSTokenizer::consumeEscape, the '\\' consumed.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func consumeEscape(_ input: C, _ position: inout Int) -> UInt32 {
        let cc = at(input, position)
        position &+= 1
        if K.isASCIIHexDigit(cc) {
            var codePoint = K.hexValue(cc)
            var consumedHexDigits = 1
            while consumedHexDigits < 6 && K.isASCIIHexDigit(at(input, position)) {
                codePoint = codePoint &* 16 &+ K.hexValue(at(input, position))
                position &+= 1
                consumedHexDigits &+= 1
            }
            consumeSingleWhitespaceIfNext(input, &position)
            if codePoint == 0 || (codePoint & 0xFFFF_F800) == 0xD800 || codePoint > 0x10FFFF {
                return 0xFFFD
            }
            return codePoint
        }
        if cc == 0 {
            return 0xFFFD
        }
        return UInt32(cc)
    }

    // CSSTokenizer::consumeSingleWhitespaceIfNext ("\r\n" counts as one).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func consumeSingleWhitespaceIfNext(_ input: C, _ position: inout Int) {
        let next = at(input, position)
        if next == 0x0D && at(input, position &+ 1) == 0x0A {
            position &+= 2
        } else if K.isASCIIWhitespace(next) {
            position &+= 1
        }
    }

    // consumeStringTokenUntil's slow case, from `start` (after the quote): the string's registration, or nil for a
    // BadStringToken.
    @inline(never)
    static func consumeStringTokenSlowCase(_ input: C, _ start: Int, _ ending: UInt16, _ pool: inout CSSTokenizerPool) -> (Int, CSSTokenizerValue?) {
        guard pool.reserve(inputCount: input.count) else {
            return (start, nil)
        }
        var position = start
        let poolStart = pool.used
        var allUnits: UInt16 = 0
        while true {
            let cc = at(input, position)
            position &+= 1
            if cc == ending || cc == 0 {
                return (position, pool.register(poolStart, allUnits))
            }
            if K.isNewline(cc) {
                position &-= 1
                pool.used = poolStart
                return (position, nil)
            }
            if cc == 0x5C {
                let next = at(input, position)
                if next == 0 {
                    continue
                }
                if K.isNewline(next) {
                    consumeSingleWhitespaceIfNext(input, &position)
                } else {
                    pool.append(codePoint: consumeEscape(input, &position), &allUnits)
                }
            } else {
                pool.append(cc, &allUnits)
            }
        }
    }

    // consumeURLToken's slow case, from `start`: the URL's registration, or nil for a BadUrlToken (after
    // consumeBadUrlRemnants).
    @inline(never)
    static func consumeURLTokenSlowCase(_ input: C, _ start: Int, _ pool: inout CSSTokenizerPool) -> (Int, CSSTokenizerValue?) {
        guard pool.reserve(inputCount: input.count) else {
            return (start, nil)
        }
        var position = start
        let poolStart = pool.used
        var allUnits: UInt16 = 0
        while true {
            let cc = at(input, position)
            position &+= 1
            if cc == 0x29 || cc == 0 {
                return (position, pool.register(poolStart, allUnits))
            }
            if K.isASCIIWhitespace(cc) {
                while position < input.count && K.isASCIIWhitespace(input[position]) {
                    position &+= 1
                }
                if at(input, position) == 0x29 {
                    position &+= 1
                    return (position, pool.register(poolStart, allUnits))
                }
                if at(input, position) == 0 {
                    return (position, pool.register(poolStart, allUnits))
                }
                break
            }
            if cc == 0x22 || cc == 0x27 || cc == 0x28 || K.isNonPrintable(cc) {
                break
            }
            if cc == 0x5C {
                if K.isValidEscape(cc, at(input, position)) {
                    pool.append(codePoint: consumeEscape(input, &position), &allUnits)
                    continue
                }
                break
            }
            pool.append(cc, &allUnits)
        }
        pool.used = poolStart
        // consumeBadUrlRemnants
        while true {
            let cc = at(input, position)
            position &+= 1
            if cc == 0x29 || cc == 0 {
                break
            }
            if K.isValidEscape(cc, at(input, position)) {
                _ = consumeEscape(input, &position)
            }
        }
        return (position, nil)
    }
}

#endif
