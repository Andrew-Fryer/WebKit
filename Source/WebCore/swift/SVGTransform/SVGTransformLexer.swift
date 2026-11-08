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

// The lexing half of WebCore::SVGTransformList::parseGeneric for Latin-1 input. An item is: leading
// whitespace, transform keyword, parenthesized parameter list, then the whitespace / ',' / whitespace
// that separates it from the next item; one call lexes consecutive items (up to the caller's capacity,
// stopping after the first that fails). It mirrors, statement by statement and including where the
// cursor stops on failure:
//   - skipOptionalSVGSpaces, skipOptionalSVGSpacesOrDelimiter(',')   (svg/SVGParserUtilities.h)
//   - genericParseNumber<Latin1Character, float>(.., DontSkip)        (svg/SVGParserUtilities.cpp)
//   - parseTransformTypeGeneric                                       (svg/SVGTransformList.cpp)
//   - parseTransformParamList + required/optionalValuesForType        (svg/SVGTransformListInlines.h)
// The C++ glue in SVGTransformList.cpp applies the result to the list.
//
// Result layout (SVGTransformSwiftBoundary.h): bits 0-7 status, bits 8-15 SVGTransformType,
// bits 16-23 number of values, bit 24 set if a ',' followed the item.

enum SVGTransformLexer {
    // Status values; keep in sync with SVGTransformSwiftBoundary.h.
    static var statusNoTransformType: UInt32 { 0 }
    static var statusInvalidParameters: UInt32 { 1 }
    static var statusParsed: UInt32 { 2 }
    static var statusOnlyWhitespace: UInt32 { 3 }

    // SVGTransformValue::SVGTransformType.
    static var typeMatrix: UInt32 { 1 }
    static var typeTranslate: UInt32 { 2 }
    static var typeScale: UInt32 { 3 }
    static var typeRotate: UInt32 { 4 }
    static var typeSkewX: UInt32 { 5 }
    static var typeSkewY: UInt32 { 6 }

    static var maximumValueCount: Int { 6 }

    // Lexes consecutive items from the start of `characters` until one is not `statusParsed`, the input
    // ends, or `results` is full. Item k's result goes to results[k] and its values to
    // values[6 * k ..< 6 * k + count]. `end` is how far the C++ lexer would have advanced its
    // StringParsingBuffer after the last lexed item, also when that item failed (SVGViewSpec keeps
    // parsing from there). At least one item is lexed if `characters` and `results` are not empty.
    @inline(__always)
    static func lexItems(_ characters: Span<UInt8>, _ results: inout MutableSpan<UInt32>, _ values: inout MutableSpan<Float>) -> (count: Int, end: Int) {
        var index = 0
        var count = 0
        while count < results.count {
            let item = lexItem(characters, index, &values, valueBase: count &* maximumValueCount)
            results[count] = item.result
            count &+= 1
            index = item.end
            if item.result & 0xFF != statusParsed || index >= characters.count {
                break
            }
        }
        return (count, index)
    }

    // Lexes one item at `start`; its values go to values[valueBase...].
    @inline(__always)
    static func lexItem(_ characters: Span<UInt8>, _ start: Int, _ values: inout MutableSpan<Float>, valueBase: Int) -> (result: UInt32, end: Int) {
        var index = start

        // parseGeneric skips whitespace before the first item; between items its trailing skip has
        // already consumed all whitespace, so this is a no-op there.
        if !skipSpaces(characters, &index) {
            return (statusOnlyWhitespace, index)
        }

        guard let type = parseTransformType(characters, &index) else {
            return (statusNoTransformType, index)
        }

        let required: Int
        let optional: Int
        switch type {
        case typeMatrix:
            required = 6
            optional = 0
        case typeTranslate, typeScale:
            required = 1
            optional = 1
        case typeRotate:
            required = 1
            optional = 2
        default: // typeSkewX, typeSkewY
            required = 1
            optional = 0
        }

        let count = parseParameterList(characters, &index, &values, valueBase: valueBase, required: required, optional: optional)
        if count < 0 {
            return (statusInvalidParameters | type << 8, index)
        }

        // parseGeneric: skipOptionalSVGSpaces; skipExactly(','); skipOptionalSVGSpaces.
        _ = skipSpaces(characters, &index)
        var commaBit: UInt32 = 0
        if index < characters.count && characters[index] == comma {
            index &+= 1
            commaBit = 1 << 24
        }
        _ = skipSpaces(characters, &index)

        return (statusParsed | type << 8 | UInt32(truncatingIfNeeded: count) << 16 | commaBit, index)
    }

    // MARK: - Characters

    private static var comma: UInt8 { 0x2C } // ','
    private static var plus: UInt8 { 0x2B } // '+'
    private static var minus: UInt8 { 0x2D } // '-'
    private static var period: UInt8 { 0x2E } // '.'
    private static var leftParenthesis: UInt8 { 0x28 } // '('
    private static var rightParenthesis: UInt8 { 0x29 } // ')'

    // isASCIIWhitespace: ' ', '\n', '\t', '\r', '\f' (not '\v').
    @inline(__always)
    private static func isSpace(_ character: UInt8) -> Bool {
        character == 0x20 || character == 0x0A || character == 0x09 || character == 0x0D || character == 0x0C
    }

    @inline(__always)
    private static func isDigit(_ character: UInt8) -> Bool {
        character &- 0x30 < 10
    }

    // skipOptionalSVGSpaces: returns whether characters remain.
    @inline(__always)
    private static func skipSpaces(_ characters: Span<UInt8>, _ index: inout Int) -> Bool {
        while index < characters.count && isSpace(characters[index]) {
            index &+= 1
        }
        return index < characters.count
    }

    // skipOptionalSVGSpacesOrDelimiter(buffer, ',').
    @inline(__always)
    private static func skipSpacesOrComma(_ characters: Span<UInt8>, _ index: inout Int) -> Bool {
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

    // skipCharactersExactly: advances only on a full match.
    @inline(__always)
    private static func skipKeyword<let length: Int>(_ characters: Span<UInt8>, _ index: inout Int, _ keyword: InlineArray<length, UInt8>) -> Bool {
        if characters.count &- index < length {
            return false
        }
        let candidate = characters.extracting(index ..< index &+ length)
        for offset in 0..<length {
            if candidate[offset] != keyword[offset] {
                return false
            }
        }
        index &+= length
        return true
    }

    // MARK: - parseTransformTypeGeneric

    @inline(__always)
    private static func parseTransformType(_ characters: Span<UInt8>, _ index: inout Int) -> UInt32? {
        if index >= characters.count {
            return nil
        }

        if characters[index] == 0x73 { // 's'
            if skipKeyword(characters, &index, [0x73, 0x6B, 0x65, 0x77, 0x58]) { // "skewX"
                return typeSkewX
            }
            if skipKeyword(characters, &index, [0x73, 0x6B, 0x65, 0x77, 0x59]) { // "skewY"
                return typeSkewY
            }
            if skipKeyword(characters, &index, [0x73, 0x63, 0x61, 0x6C, 0x65]) { // "scale"
                return typeScale
            }
            return nil
        }

        if skipKeyword(characters, &index, [0x74, 0x72, 0x61, 0x6E, 0x73, 0x6C, 0x61, 0x74, 0x65]) { // "translate"
            return typeTranslate
        }
        if skipKeyword(characters, &index, [0x72, 0x6F, 0x74, 0x61, 0x74, 0x65]) { // "rotate"
            return typeRotate
        }
        if skipKeyword(characters, &index, [0x6D, 0x61, 0x74, 0x72, 0x69, 0x78]) { // "matrix"
            return typeMatrix
        }
        return nil
    }

    // MARK: - parseTransformParamList

    // Returns the number of values parsed, or -1. The C++ has one loop for the required values and one for
    // the optional ones, each followed by the same whitespace/delimiter handling; this runs that code
    // twice with the second pass only reached when an optional value follows (one inlined number parser).
    @inline(__always)
    private static func parseParameterList(_ characters: Span<UInt8>, _ index: inout Int, _ values: inout MutableSpan<Float>, valueBase: Int, required: Int, optional: Int) -> Int {
        if !skipSpaces(characters, &index) || characters[index] != leftParenthesis {
            return -1
        }

        index &+= 1

        _ = skipSpaces(characters, &index)

        var count = 0
        var limit = required
        var parsingOptionalValues = false
        while true {
            while count < limit {
                if index >= characters.count {
                    return -1
                }
                guard let number = parseNumber(characters, &index) else {
                    return -1
                }
                values[valueBase &+ count] = number
                count &+= 1
                if count < limit {
                    _ = skipSpacesOrComma(characters, &index)
                }
            }

            if !skipSpaces(characters, &index) {
                return -1
            }

            let delimiterParsed = skipSpacesOrComma(characters, &index)

            if parsingOptionalValues {
                if index >= characters.count || characters[index] != rightParenthesis || delimiterParsed {
                    return -1
                }
                index &+= 1
                return count
            }

            if index >= characters.count {
                return -1
            }

            if characters[index] == rightParenthesis {
                // skip optionals
                index &+= 1
                if delimiterParsed {
                    return -1
                }
                return count
            }

            parsingOptionalValues = true
            limit = required &+ optional
        }
    }

    // MARK: - genericParseNumber<Latin1Character, float>, SuffixSkippingPolicy::DontSkip

    // isValidRange: rejects infinities and NaN.
    @inline(__always)
    private static func isValidRange(_ x: Float) -> Bool {
        x >= -Float.greatestFiniteMagnitude && x <= Float.greatestFiniteMagnitude
    }

    // `accumulator += a * b` as clang compiles it in WebKit: contracted to one fused multiply-add on
    // arm64, two roundings on x86_64 (no FMA in the baseline ISA; addingProduct would call fmaf).
    @inline(__always)
    private static func multiplyAdd(_ accumulator: Float, _ a: Float, _ b: Float) -> Float {
#if arch(arm64)
        accumulator.addingProduct(a, b)
#else
        accumulator + a * b
#endif
    }

    @inline(__always)
    private static func parseNumber(_ characters: Span<UInt8>, _ index: inout Int) -> Float? {
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
    @inline(__always)
    private static func powerOfTen(_ exponent: Float) -> Float {
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
