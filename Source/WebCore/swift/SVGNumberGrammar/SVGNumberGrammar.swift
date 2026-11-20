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

// WebCore's SVG <number> grammar, for Latin-1 and UTF-16 input: the only implementation (the C++ in
// svg/SVGParserUtilities.cpp calls this through SVGNumberGrammarSwiftBoundary.h, and the SVG transform lexer in
// swift/SVGTransform uses it directly). It mirrors, statement by statement and including where the cursor stops on
// failure, the original C++ it replaced (both character types ran the same templates):
//   - skipOptionalSVGSpaces, skipOptionalSVGSpacesOrDelimiter(',')   (svg/SVGParserUtilities.h)
//   - genericParseNumber<CharacterType, float>(.., Skip / DontSkip)   (svg/SVGParserUtilities.cpp)
//   - genericParseArcFlag<CharacterType>                              (svg/SVGParserUtilities.cpp)
//   - genericParseFloatPoint and the number sequences of the SVG path segment parsers (a "program", below)
//   - the item scanning of SVGLengthList::parse and parseGlyphName   (svg/SVGLengthList.cpp, SVGParserUtilities.cpp)

// The characters being parsed, as code units zero-extended to 32 bits.
protocol SVGCharacters: ~Escapable {
    var count: Int { get }
    subscript(_ index: Int) -> UInt32 { get }
}

struct SVGLatin1Characters: SVGCharacters, ~Escapable {
    let units: Span<UInt8>

    @_lifetime(copy units)
    init(_ units: Span<UInt8>) {
        self.units = units
    }

    var count: Int {
        @inline(__always) get { units.count }
    }

    subscript(_ index: Int) -> UInt32 {
        @inline(__always) get { UInt32(units[index]) }
    }
}

struct SVGUTF16Characters: SVGCharacters, ~Escapable {
    let units: Span<UInt16>

    @_lifetime(copy units)
    init(_ units: Span<UInt16>) {
        self.units = units
    }

    var count: Int {
        @inline(__always) get { units.count }
    }

    subscript(_ index: Int) -> UInt32 {
        @inline(__always) get { UInt32(units[index]) }
    }
}

// UTF-16 code units at an odd address (C++ strings can be odd-aligned), 2 bytes each.
struct SVGUnalignedUTF16Characters: SVGCharacters, ~Escapable {
    let bytes: RawSpan

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    var count: Int {
        @inline(__always) get { bytes.byteCount &>> 1 }
    }

    subscript(_ index: Int) -> UInt32 {
        @inline(__always) get { UInt32(bytes.load(fromByteOffset: index &<< 1, as: UInt16.self)) }
    }
}

enum SVGNumberGrammar {
    // MARK: - Programs

    // A program is a sequence of up to `maximumProgramLength` items parsed one after the other; keep in sync with
    // SVGNumberGrammarSwiftBoundary.h. Bits 0-3: the number of items. Bits 4 + 2k ..< 6 + 2k: item k's kind.
    static var itemNumber: UInt32 { 0 } // genericParseNumber(.., SuffixSkippingPolicy::Skip)
    static var itemNumberWithoutSuffix: UInt32 { 1 } // genericParseNumber(.., SuffixSkippingPolicy::DontSkip)
    static var itemArcFlag: UInt32 { 2 } // genericParseArcFlag (0 or 1)
    static var maximumProgramLength: Int { 8 }
    // The arc segment program: three numbers, two arc flags, two numbers.
    static var arcProgram: UInt32 { 7 | itemArcFlag << (4 + 2 * 3) | itemArcFlag << (4 + 2 * 4) }

    // parseItems from the start of `characters`, packed for the C entry point: the cursor advance << 8 | the number of
    // items parsed. Not inlined, so that each character type gets its own register allocation and the entry point
    // tail-calls it.
    @inline(never)
    static func scan<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, program: UInt32, _ values: inout MutableSpan<Float>) -> UInt64 {
        var index = 0
        let parsed = parseItems(characters, &index, program: program, &values)
        return UInt64(truncatingIfNeeded: index) &<< 8 | UInt64(truncatingIfNeeded: parsed)
    }

    // Parses the items of `program` from `index` until one fails or all are parsed. Item k's value goes to values[k]
    // (an arc flag as 0 or 1). Returns the number of items parsed; `index` is where the C++ would have left its
    // StringParsingBuffer, also when an item failed part way.
    @inline(always)
    static func parseItems<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int, program: UInt32, _ values: inout MutableSpan<Float>) -> Int {
        let count = Int(program & 0xF)
        precondition(count <= maximumProgramLength && count <= values.count)
        var parsed = 0
        if program == arcProgram {
            // rx ry angle largeArc sweep x y, straight-line.
            for _ in 0..<3 {
                guard let number = parseNumber(characters, &index, skipSuffix: true) else {
                    return parsed
                }
                values[parsed] = number
                parsed &+= 1
            }
            for _ in 0..<2 {
                guard let flag = parseArcFlag(characters, &index) else {
                    return parsed
                }
                _ = skipSpacesOrDelimiter(characters, &index)
                values[parsed] = flag
                parsed &+= 1
            }
            for _ in 0..<2 {
                guard let number = parseNumber(characters, &index, skipSuffix: true) else {
                    return parsed
                }
                values[parsed] = number
                parsed &+= 1
            }
            return parsed
        }
        let lastKind = count > 0 ? (program &>> (4 &+ 2 &* (count &- 1))) & 3 : itemNumber
        if count > 0 && program & ~(0xF | 3 &<< (4 &+ 2 &* (count &- 1))) == 0 && lastKind != itemArcFlag {
            // Numbers, all followed by the spaces / ',' after them except perhaps the last (path segments, points, rects).
            while parsed < count &- 1 {
                guard let number = parseNumber(characters, &index, skipSuffix: true) else {
                    return parsed
                }
                values[parsed] = number
                parsed &+= 1
            }
            guard let number = parseNumber(characters, &index, skipSuffix: lastKind != itemNumberWithoutSuffix) else {
                return parsed
            }
            values[parsed] = number
            return count
        }
        var kinds = program &>> 4
        while parsed < count {
            let kind = kinds & 3
            let value: Float
            if kind == itemArcFlag {
                guard let flag = parseArcFlag(characters, &index) else {
                    break
                }
                value = flag
            } else {
                guard let number = parseNumber(characters, &index) else {
                    break
                }
                value = number
            }
            // Numbers with SuffixSkippingPolicy::Skip and arc flags are followed by skipOptionalSVGSpacesOrDelimiter.
            // (Kind 3 is not a valid kind; it parses like itemNumber.)
            if kind != itemNumberWithoutSuffix {
                _ = skipSpacesOrDelimiter(characters, &index)
            }
            values[parsed] = value
            parsed &+= 1
            kinds = kinds &>> 2
        }
        return parsed
    }

    // MARK: - List items

    // The kinds of list item scanListItem finds; keep in sync with SVGNumberGrammarSwiftBoundary.h.
    static var listItemLength: UInt32 { 0 } // SVGLengthList::parse: up to a space or ','.
    static var listItemGlyphName: UInt32 { 1 } // parseGlyphName: up to ',', without trailing spaces.

    // The item-scanning half of one iteration of the list parser loop for `kind`: the item at the start of `characters`
    // and the skipOptionalSVGSpacesOrDelimiter(',') after it. Returns the item's length and how far the C++ loop
    // advances past both; an advance of 0 means there is no item (the loop stops there).
    @inline(always)
    static func scanListItem<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, kind: UInt32) -> (length: Int, advance: Int) {
        precondition(kind <= listItemGlyphName)
        var index = 0
        let length: Int
        if kind == listItemGlyphName {
            // skipUntil(buffer, ',')
            while index < characters.count && characters[index] != comma {
                index &+= 1
            }
            if index == 0 {
                return (0, 0)
            }
            // Walk backwards from the ',' to ignore any whitespace.
            var end = index
            while end > 0 && isSpace(characters[end &- 1]) {
                end &-= 1
            }
            length = end
        } else {
            // skipUntil<isSVGSpaceOrComma>(buffer)
            while index < characters.count && !isSpace(characters[index]) && characters[index] != comma {
                index &+= 1
            }
            if index == 0 {
                return (0, 0)
            }
            length = index
        }
        _ = skipSpacesOrDelimiter(characters, &index)
        return (length, index)
    }

    // MARK: - Characters

    static var comma: UInt32 { 0x2C } // ','
    static var plus: UInt32 { 0x2B } // '+'
    static var minus: UInt32 { 0x2D } // '-'
    static var period: UInt32 { 0x2E } // '.'

    // isASCIIWhitespace: ' ', '\n', '\t', '\r', '\f' (not '\v').
    @inline(__always)
    static func isSpace(_ character: UInt32) -> Bool {
        character == 0x20 || character == 0x0A || character == 0x09 || character == 0x0D || character == 0x0C
    }

    @inline(__always)
    static func isDigit(_ character: UInt32) -> Bool {
        character &- 0x30 < 10
    }

    // skipOptionalSVGSpaces: returns whether characters remain.
    @inline(__always)
    static func skipSpaces<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int) -> Bool {
        while index < characters.count && isSpace(characters[index]) {
            index &+= 1
        }
        return index < characters.count
    }

    // skipOptionalSVGSpacesOrDelimiter(buffer, ',').
    @inline(always)
    static func skipSpacesOrDelimiter<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int) -> Bool {
        if index >= characters.count {
            return false
        }
        let character = characters[index]
        if !isSpace(character) && character != comma {
            return false
        }
        // There are only spaces in the remaining characters.
        if !skipSpaces(characters, &index) {
            return false
        }
        if characters[index] != comma {
            return true
        }
        // A delimiter is hit. Skip the following spaces also e.g. " , ".
        index &+= 1
        return skipSpaces(characters, &index)
    }

    // MARK: - genericParseArcFlag<CharacterType>

    // Only used to parse largeArcFlag and sweepFlag, which must be a "0" or "1" and might not have any
    // whitespace/comma after it. Consumes the character even when it is not a flag. Returns the flag as 0 or 1;
    // genericParseArcFlag then skips the spaces / ',' after it (parseItems does that).
    @inline(always)
    static func parseArcFlag<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int) -> Float? {
        if index >= characters.count {
            return nil
        }
        let character = characters[index]
        index &+= 1
        if character == 0x30 { // '0'
            return 0
        }
        if character == 0x31 { // '1'
            return 1
        }
        return nil
    }

    // MARK: - genericParseNumber<CharacterType, float>

    // isValidRange: rejects infinities and NaN.
    @inline(__always)
    static func isValidRange(_ x: Float) -> Bool {
        x >= -Float.greatestFiniteMagnitude && x <= Float.greatestFiniteMagnitude
    }

    // `accumulator += a * b` as clang compiles it in WebKit: contracted to one fused multiply-add on
    // arm64, two roundings on x86_64 (no FMA in the baseline ISA; addingProduct would call fmaf).
    @inline(__always)
    static func multiplyAdd(_ accumulator: Float, _ a: Float, _ b: Float) -> Float {
#if arch(arm64)
        accumulator.addingProduct(a, b)
#else
        accumulator + a * b
#endif
    }

    // With `skipSuffix` (SuffixSkippingPolicy::Skip), a successfully parsed number is followed by
    // skipOptionalSVGSpacesOrDelimiter(buffer).
    @inline(always)
    static func parseNumber<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int, skipSuffix: Bool) -> Float? {
        guard let number = parseNumber(characters, &index) else {
            return nil
        }
        if skipSuffix {
            _ = skipSpacesOrDelimiter(characters, &index)
        }
        return number
    }

    // SuffixSkippingPolicy::DontSkip.
    @inline(always)
    static func parseNumber<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int) -> Float? {
        let count = characters.count

        // read the sign
        var sign: Float = 1
        if index < count && characters[index] == plus {
            index &+= 1
        } else if index < count && characters[index] == minus {
            index &+= 1
            sign = -1
        }

        if index >= count {
            return nil
        }
        let first = characters[index]
        if !isDigit(first) && first != period {
            return nil
        }

        // read the integer part, build right-to-left
        let digitsStart = index
        while index < count && isDigit(characters[index]) {
            index &+= 1
        }

        var integer: Float = 0
        if index > digitsStart {
            var multiplier: Float = 1
            var position = index
            while position > digitsStart {
                position &-= 1
                integer = multiplyAdd(integer, multiplier, Float(characters[position] &- 0x30))
                multiplier = multiplier * 10
            }
            // Bail out early if this overflows.
            if !isValidRange(integer) {
                return nil
            }
        }

        // read the decimals
        var decimal: Float = 0
        if index < count && characters[index] == period {
            index &+= 1

            // There must be a least one digit following the .
            if index >= count || !isDigit(characters[index]) {
                return nil
            }

            var fraction: Float = 1
            while index < count && isDigit(characters[index]) {
                fraction = fraction * Float(bitPattern: 0x3DCC_CCCD) // static_cast<float>(0.1)
                decimal = multiplyAdd(decimal, Float(characters[index] &- 0x30), fraction)
                index &+= 1
            }
        }

        var number = integer + decimal
        number = number * sign

        // read the exponent part
        if count &- index > 1 {
            let e = characters[index]
            let next = characters[index &+ 1]
            if (e == 0x65 || e == 0x45) && next != 0x78 && next != 0x6D { // 'e' or 'E', not followed by 'x' or 'm'
                index &+= 1

                // read the sign of the exponent
                var exponentIsNegative = false
                if characters[index] == plus {
                    index &+= 1
                } else if characters[index] == minus {
                    index &+= 1
                    exponentIsNegative = true
                }

                // There must be an exponent
                if index >= count || !isDigit(characters[index]) {
                    return nil
                }

                // Two statements in C++, so never contracted.
                var exponent: Float = 0
                while index < count && isDigit(characters[index]) {
                    exponent = exponent * 10
                    exponent = exponent + Float(characters[index] &- 0x30)
                    index &+= 1
                }

                if exponentIsNegative {
                    exponent = -exponent
                }
                // Fail if the exponent is greater than the largest positive power of ten
                // (that would yield a representable float).
                if exponent > 38 {
                    return nil
                }

                if exponent != 0 {
                    number = number * powerOfTen(exponent)
                }
            }
        }

        // Don't return Infinity() or NaN().
        if !isValidRange(number) {
            return nil
        }

        return number
    }

    // static_cast<float>(std::pow(10.0, exponent)) for an integral exponent <= 38. The exponent is a sum
    // of integer-valued products, so it is integral whenever it is in [-45, 38]; anything smaller
    // (including -infinity) gives a double below half the smallest float subnormal, i.e. 0. The values
    // were generated from libm's pow on arm64e and x86_64 (identical; see units/SVGTransform/gen).
    @inline(always)
    static func powerOfTen(_ exponent: Float) -> Float {
        if exponent < -45 {
            return 0
        }
        switch Int(exponent) {
        case -45: return Float(bitPattern: 0x0000_0001)
        case -44: return Float(bitPattern: 0x0000_0007)
        case -43: return Float(bitPattern: 0x0000_0047)
        case -42: return Float(bitPattern: 0x0000_02CA)
        case -41: return Float(bitPattern: 0x0000_1BE0)
        case -40: return Float(bitPattern: 0x0001_16C2)
        case -39: return Float(bitPattern: 0x000A_E398)
        case -38: return Float(bitPattern: 0x006C_E3EE)
        case -37: return Float(bitPattern: 0x0208_1CEA)
        case -36: return Float(bitPattern: 0x03AA_2425)
        case -35: return Float(bitPattern: 0x0554_AD2E)
        case -34: return Float(bitPattern: 0x0704_EC3D)
        case -33: return Float(bitPattern: 0x08A6_274C)
        case -32: return Float(bitPattern: 0x0A4F_B11F)
        case -31: return Float(bitPattern: 0x0C01_CEB3)
        case -30: return Float(bitPattern: 0x0DA2_4260)
        case -29: return Float(bitPattern: 0x0F4A_D2F8)
        case -28: return Float(bitPattern: 0x10FD_87B6)
        case -27: return Float(bitPattern: 0x129E_74D2)
        case -26: return Float(bitPattern: 0x1446_1206)
        case -25: return Float(bitPattern: 0x15F7_9688)
        case -24: return Float(bitPattern: 0x179A_BE15)
        case -23: return Float(bitPattern: 0x1941_6D9A)
        case -22: return Float(bitPattern: 0x1AF1_C901)
        case -21: return Float(bitPattern: 0x1C97_1DA0)
        case -20: return Float(bitPattern: 0x1E3C_E508)
        case -19: return Float(bitPattern: 0x1FEC_1E4A)
        case -18: return Float(bitPattern: 0x2193_92EF)
        case -17: return Float(bitPattern: 0x2338_77AA)
        case -16: return Float(bitPattern: 0x24E6_9595)
        case -15: return Float(bitPattern: 0x2690_1D7D)
        case -14: return Float(bitPattern: 0x2834_24DC)
        case -13: return Float(bitPattern: 0x29E1_2E13)
        case -12: return Float(bitPattern: 0x2B8C_BCCC)
        case -11: return Float(bitPattern: 0x2D2F_EBFF)
        case -10: return Float(bitPattern: 0x2EDB_E6FF)
        case -9: return Float(bitPattern: 0x3089_705F)
        case -8: return Float(bitPattern: 0x322B_CC77)
        case -7: return Float(bitPattern: 0x33D6_BF95)
        case -6: return Float(bitPattern: 0x3586_37BD)
        case -5: return Float(bitPattern: 0x3727_C5AC)
        case -4: return Float(bitPattern: 0x38D1_B717)
        case -3: return Float(bitPattern: 0x3A83_126F)
        case -2: return Float(bitPattern: 0x3C23_D70A)
        case -1: return Float(bitPattern: 0x3DCC_CCCD)
        case 1: return Float(bitPattern: 0x4120_0000)
        case 2: return Float(bitPattern: 0x42C8_0000)
        case 3: return Float(bitPattern: 0x447A_0000)
        case 4: return Float(bitPattern: 0x461C_4000)
        case 5: return Float(bitPattern: 0x47C3_5000)
        case 6: return Float(bitPattern: 0x4974_2400)
        case 7: return Float(bitPattern: 0x4B18_9680)
        case 8: return Float(bitPattern: 0x4CBE_BC20)
        case 9: return Float(bitPattern: 0x4E6E_6B28)
        case 10: return Float(bitPattern: 0x5015_02F9)
        case 11: return Float(bitPattern: 0x51BA_43B7)
        case 12: return Float(bitPattern: 0x5368_D4A5)
        case 13: return Float(bitPattern: 0x5511_84E7)
        case 14: return Float(bitPattern: 0x56B5_E621)
        case 15: return Float(bitPattern: 0x5863_5FA9)
        case 16: return Float(bitPattern: 0x5A0E_1BCA)
        case 17: return Float(bitPattern: 0x5BB1_A2BC)
        case 18: return Float(bitPattern: 0x5D5E_0B6B)
        case 19: return Float(bitPattern: 0x5F0A_C723)
        case 20: return Float(bitPattern: 0x60AD_78EC)
        case 21: return Float(bitPattern: 0x6258_D727)
        case 22: return Float(bitPattern: 0x6407_8678)
        case 23: return Float(bitPattern: 0x65A9_6816)
        case 24: return Float(bitPattern: 0x6753_C21C)
        case 25: return Float(bitPattern: 0x6904_5951)
        case 26: return Float(bitPattern: 0x6AA5_6FA6)
        case 27: return Float(bitPattern: 0x6C4E_CB8F)
        case 28: return Float(bitPattern: 0x6E01_3F39)
        case 29: return Float(bitPattern: 0x6FA1_8F08)
        case 30: return Float(bitPattern: 0x7149_F2CA)
        case 31: return Float(bitPattern: 0x72FC_6F7C)
        case 32: return Float(bitPattern: 0x749D_C5AE)
        case 33: return Float(bitPattern: 0x7645_3719)
        case 34: return Float(bitPattern: 0x77F6_84DF)
        case 35: return Float(bitPattern: 0x799A_130C)
        case 36: return Float(bitPattern: 0x7B40_97CE)
        case 37: return Float(bitPattern: 0x7CF0_BDC2)
        case 38: return Float(bitPattern: 0x7E96_7699)
        default: return 0 // Unreachable: exponent is integral, nonzero and in [-45, 38].
        }
    }
}
