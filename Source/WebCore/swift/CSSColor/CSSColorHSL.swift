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

// Memory-safe port of parseHSL<CharacterType> (CSSParserFastPaths.cpp at 05fbeeaa85b0), including the inlined
// parseSimpleAngle(RequireUnits::No) / parseCSSNumber hue parsing and the typed-color tail that turns
// the parsed components into SRGBA<uint8_t>:
//   StyleColorParseType<HSLFunctionModern> -> convertToTypedColor<HSLFunctionModern>   (CSSColorConversion+ToTypedColor.h)
//   -> ConvertToColor<HSLFunctionModern, Absolute>::convertToColor(color, 0)            (CSSColorConversion+ToColor.h)
//   -> convertColor<SRGBA<uint8_t>>(HSLA<float>): ColorConversion<ExtendedSRGBA<float>, HSLA<float>>::convert
//      (ColorConversion.cpp), clipToGamut (ColorTypes.h), handleToByteConversion (ColorConversion.h)
//   -> Color::tryGetAsSRGBABytes(), which is std::nullopt whenever convertToColor keeps the HSLA (out of gamut / none).
// The hue number is parsed with WTF::charactersToDouble() semantics by charactersToDouble() below.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

extension CSSColor {
    // parseHSL(characters[start...]); `start` is just past the '('.
    static func parseHSL<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int) -> UInt64 {
        let end = characters.count
        guard start >= 0, start <= end else {
            return notAColor
        }

        // Find the end of the hue token (terminated by comma or whitespace).
        // A comma means legacy syntax (comma-separated), whitespace means modern (space-separated).
        // The scan stops at the first ',' or isCSSSpace() character; isLegacy records which one. All four
        // terminators are <= ',', so most characters (digits, '.', '-', letters) take one well-predicted
        // compare per character, as in clang's code for the C++ loop.
        let afterParen = characters.extracting(start..<end)
        var hueLength = 0
        while hueLength < afterParen.count {
            let character = afterParen[hueLength]
            if character > asciiComma {
                hueLength &+= 1
                continue
            }
            if character == asciiComma || isCSSSpace(character) {
                break
            }
            hueLength &+= 1
        }
        let hueEnd = start &+ hueLength
        if hueEnd == start || hueEnd == end {
            return notAColor
        }
        let isLegacy = characters[hueEnd] == asciiComma

        guard let angle = parseSimpleAngle(characters, start, hueEnd) else {
            return notAColor
        }

        var current = hueEnd

        // In legacy syntax, components are separated by commas.
        // (The hue scan stopped at a ',' when isLegacy is set.)
        if isLegacy {
            current &+= 1
        }

        // The parsePercentage lambda, written out at both call sites: skip CSS spaces, parseDouble up to '%',
        // then consume the '%' (a non-zero parseDouble result means the '%' is at current + numCharactersParsed).
        skipCSSSpaces(characters, &current)
        var saturation: Double = 0
        let saturationLength = parseDouble(characters, current, end, asciiPercent, &saturation)
        if saturationLength == 0 {
            return notAColor
        }
        current &+= saturationLength &+ 1

        if isLegacy {
            if current >= end || characters[current] != asciiComma {
                return notAColor
            }
            current &+= 1
        }

        skipCSSSpaces(characters, &current)
        var lightness: Double = 0
        let lightnessLength = parseDouble(characters, current, end, asciiPercent, &lightness)
        if lightnessLength == 0 {
            return notAColor
        }
        current &+= lightnessLength &+ 1

        skipCSSSpaces(characters, &current)

        // Alpha is optional. Legacy uses comma separator, modern uses slash.
        var alpha: Double = 1.0
        let alphaSeparator = isLegacy ? asciiComma : asciiSlash
        if current < end && characters[current] == alphaSeparator {
            current &+= 1
            skipCSSSpaces(characters, &current)

            var numCharactersParsed = parseDouble(characters, current, end, asciiCloseParen, &alpha)
            if numCharactersParsed != 0 {
                current &+= numCharactersParsed
            } else {
                numCharactersParsed = parseDouble(characters, current, end, asciiPercent, &alpha)
                if numCharactersParsed == 0 {
                    return notAColor
                }
                // The '%' is at current + numCharactersParsed.
                current &+= numCharactersParsed &+ 1
                alpha /= 100.0
            }

            skipCSSSpaces(characters, &current)
        }

        if current >= end || characters[current] != asciiCloseParen {
            return notAColor
        }

        return hslToSRGBABytes(hue: angle.number, hueIsRadians: angle.isRadians, saturation: saturation, lightness: lightness, alpha: alpha)
    }

    @inline(__always)
    private static func skipCSSSpaces<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ current: inout Int) {
        var index = current
        while index >= 0 && index < characters.count && isCSSSpace(characters[index]) {
            index &+= 1
        }
        current = index
    }

    // parseSimpleAngle(characters[start..<end], RequireUnits::No, unit, number) followed by
    // parseCSSNumber. Returns the number and whether the unit is radians; nil is `return false`.
    // The caller guarantees end > start.
    @inline(__always)
    private static func parseSimpleAngle<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int) -> (number: Double, isRadians: Bool)? {
        guard start >= 0, end <= characters.count, end > start else {
            return nil
        }

        // "0deg" or "1rad"
        var numberEnd = end
        var isRadians = false
        if end &- start >= 4 {
            let unit0 = characters[end &- 3]
            let unit1 = characters[end &- 2]
            let unit2 = characters[end &- 1]
            if isASCIIAlphaCaselessEqual(unit0, 0x64) && isASCIIAlphaCaselessEqual(unit1, 0x65) && isASCIIAlphaCaselessEqual(unit2, 0x67) {
                numberEnd = end &- 3
            } else if isASCIIAlphaCaselessEqual(unit0, 0x72) && isASCIIAlphaCaselessEqual(unit1, 0x61) && isASCIIAlphaCaselessEqual(unit2, 0x64) {
                numberEnd = end &- 3
                isRadians = true
            }
        }

        // parseCSSNumber: charactersToDouble() allows a trailing '.' but that is not allowed in CSS number values.
        if characters[numberEnd &- 1] == asciiPeriod {
            return nil
        }
        guard let number = charactersToDouble(characters, start, numberEnd) else {
            return nil
        }
        return (number: number, isRadians: isRadians)
    }

    // MARK: - Numbers

    // WTF::charactersToDouble(characters[start..<end], &ok); nil is ok = false. That is toDoubleType<..., TrailingJunkPolicy::
    // Disallow, WhitespacePolicy::Skip, ParseMode::General> (WTFString.cpp): skip isUnicodeCompatibleASCIIWhitespace, then
    // WTF::parseDouble (FastFloat.cpp), fast_float::from_chars with chars_format::general | no_infnan | allow_leading_plus,
    // which must consume everything. fast_float rounds correctly (overflow gives +-infinity, underflow +-0 or a
    // subnormal), so every correctly rounded conversion of the same decimal gives the same double: the exact Clinger
    // fast path below, fast_float's own Eisel-Lemire step (eiselLemireDouble) for up to 19 digits and |exponent| <= 27,
    // or else the standard library's conversion of the validated text.
    static func charactersToDouble<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int) -> Double? {
        guard start >= 0, start <= end, end <= characters.count else {
            return nil
        }
        let text = characters.extracting(start..<end)
        let count = text.count
        var position = 0
        while position < count && isUnicodeCompatibleASCIIWhitespace(text[position]) {
            position &+= 1
        }
        if position == count {
            return nil
        }
        let numberStart = position

        // [+-]? (digits ('.' digits?)? | '.' digits) ([eE] [+-]? digits)?, where a sign must be followed by a digit or
        // '.', and an 'e' without exponent digits is not part of the number (and then remains as trailing junk).
        let negative = text[position] == asciiMinus
        if negative || text[position] == asciiPlus {
            position &+= 1
            if position == count || !(isASCIIDigit(text[position]) || text[position] == asciiPeriod) {
                return nil
            }
        }

        // mantissa * 10^exponent; exact while there are at most 19 digits (mantissa < 10^19 < 2^64), otherwise left to
        // correctlyRoundedDouble().
        var mantissa: UInt64 = 0
        let integerStart = position
        while position < count && isASCIIDigit(text[position]) {
            mantissa = mantissa &* 10 &+ UInt64(text[position] &- asciiZero)
            position &+= 1
        }
        var digitCount = position &- integerStart
        var exponent = 0
        if position < count && text[position] == asciiPeriod {
            position &+= 1
            let fractionStart = position
            // Eight digits at a time, as fast_float does.
            while count &- position >= 8 {
                let word = text.eightCharacters(at: position)
                if !isMadeOfEightDigits(word) {
                    break
                }
                mantissa = mantissa &* 100_000_000 &+ UInt64(parseEightDigits(word))
                position &+= 8
            }
            while position < count && isASCIIDigit(text[position]) {
                mantissa = mantissa &* 10 &+ UInt64(text[position] &- asciiZero)
                position &+= 1
            }
            exponent = fractionStart &- position
            digitCount &+= position &- fractionStart
        }
        let isExact = digitCount <= 19
        if digitCount == 0 {
            return nil
        }
        if position < count && text[position] | 0x20 == 0x65 { // 'e' or 'E'
            var exponentPosition = position &+ 1
            var negativeExponent = false
            if exponentPosition < count && text[exponentPosition] == asciiMinus {
                negativeExponent = true
                exponentPosition &+= 1
            } else if exponentPosition < count && text[exponentPosition] == asciiPlus {
                exponentPosition &+= 1
            }
            if exponentPosition < count && isASCIIDigit(text[exponentPosition]) {
                var exponentNumber = 0
                while exponentPosition < count && isASCIIDigit(text[exponentPosition]) {
                    if exponentNumber < 0x1000_0000 {
                        exponentNumber = exponentNumber &* 10 &+ Int(text[exponentPosition] &- asciiZero)
                    }
                    exponentPosition &+= 1
                }
                exponent &+= negativeExponent ? -exponentNumber : exponentNumber
                position = exponentPosition
            }
        }
        if position != count {
            return nil
        }

        if isExact && mantissa == 0 {
            return negative ? -0.0 : 0.0
        }
        if isExact && exponent >= -27 && exponent <= 27 {
            let magnitude: Double
            if exponent >= -22 && exponent <= 22 && mantissa <= 1 << 53 {
                // Clinger's fast path: the mantissa and the power of ten are exact doubles, so one IEEE operation
                // rounds once.
                magnitude = exponent < 0 ? Double(mantissa) / powerOfTen(-exponent) : Double(mantissa) * powerOfTen(exponent)
            } else {
                magnitude = eiselLemireDouble(mantissa, exponent)
            }
            return negative ? -magnitude : magnitude
        }
        return correctlyRoundedDouble(text, from: numberStart)
    }

    // fast_float's is_made_of_eight_digits_fast and parse_eight_digits_unrolled (little-endian bytes).
    @inline(__always)
    private static func isMadeOfEightDigits(_ word: UInt64) -> Bool {
        ((word &+ 0x4646_4646_4646_4646) | (word &- 0x3030_3030_3030_3030)) & 0x8080_8080_8080_8080 == 0
    }

    @inline(__always)
    private static func parseEightDigits(_ word: UInt64) -> UInt32 {
        var value = word &- 0x3030_3030_3030_3030
        value = value &* 10 &+ value &>> 8
        value = ((value & 0x0000_00FF_0000_00FF) &* 0x000F_4240_0000_0064 &+ (value &>> 16 & 0x0000_00FF_0000_00FF) &* 0x0000_2710_0000_0001) &>> 32
        return UInt32(truncatingIfNeeded: value)
    }

    // 10^e for 0 <= e <= 22, all exact doubles.
    @inline(__always)
    private static func powerOfTen(_ exponent: Int) -> Double {
        switch exponent {
        case 0: 1e0
        case 1: 1e1
        case 2: 1e2
        case 3: 1e3
        case 4: 1e4
        case 5: 1e5
        case 6: 1e6
        case 7: 1e7
        case 8: 1e8
        case 9: 1e9
        case 10: 1e10
        case 11: 1e11
        case 12: 1e12
        case 13: 1e13
        case 14: 1e14
        case 15: 1e15
        case 16: 1e16
        case 17: 1e17
        case 18: 1e18
        case 19: 1e19
        case 20: 1e20
        case 21: 1e21
        case 22: 1e22
        default: preconditionFailure("power of ten out of range")
        }
    }

    // fast_float's power_of_five_128 entries (WTF/wtf/fast_float/fast_float.h) for 5^-27 ... 5^27: each power of five
    // as a 128-bit significand (high word first), normalized so its top bit is set, rounded toward one.
    static let powerOfFive128: InlineArray<110, UInt64> = [
        0x9E74D1B791E07E48, 0x775EA264CF55347E, // 5^-27
        0xC612062576589DDA, 0x95364AFE032A819E, // 5^-26
        0xF79687AED3EEC551, 0x3A83DDBD83F52205, // 5^-25
        0x9ABE14CD44753B52, 0xC4926A9672793543, // 5^-24
        0xC16D9A0095928A27, 0x75B7053C0F178294, // 5^-23
        0xF1C90080BAF72CB1, 0x5324C68B12DD6339, // 5^-22
        0x971DA05074DA7BEE, 0xD3F6FC16EBCA5E04, // 5^-21
        0xBCE5086492111AEA, 0x88F4BB1CA6BCF585, // 5^-20
        0xEC1E4A7DB69561A5, 0x2B31E9E3D06C32E6, // 5^-19
        0x9392EE8E921D5D07, 0x3AFF322E62439FD0, // 5^-18
        0xB877AA3236A4B449, 0x09BEFEB9FAD487C3, // 5^-17
        0xE69594BEC44DE15B, 0x4C2EBE687989A9B4, // 5^-16
        0x901D7CF73AB0ACD9, 0x0F9D37014BF60A11, // 5^-15
        0xB424DC35095CD80F, 0x538484C19EF38C95, // 5^-14
        0xE12E13424BB40E13, 0x2865A5F206B06FBA, // 5^-13
        0x8CBCCC096F5088CB, 0xF93F87B7442E45D4, // 5^-12
        0xAFEBFF0BCB24AAFE, 0xF78F69A51539D749, // 5^-11
        0xDBE6FECEBDEDD5BE, 0xB573440E5A884D1C, // 5^-10
        0x89705F4136B4A597, 0x31680A88F8953031, // 5^-9
        0xABCC77118461CEFC, 0xFDC20D2B36BA7C3E, // 5^-8
        0xD6BF94D5E57A42BC, 0x3D32907604691B4D, // 5^-7
        0x8637BD05AF6C69B5, 0xA63F9A49C2C1B110, // 5^-6
        0xA7C5AC471B478423, 0x0FCF80DC33721D54, // 5^-5
        0xD1B71758E219652B, 0xD3C36113404EA4A9, // 5^-4
        0x83126E978D4FDF3B, 0x645A1CAC083126EA, // 5^-3
        0xA3D70A3D70A3D70A, 0x3D70A3D70A3D70A4, // 5^-2
        0xCCCCCCCCCCCCCCCC, 0xCCCCCCCCCCCCCCCD, // 5^-1
        0x8000000000000000, 0x0000000000000000, // 5^0
        0xA000000000000000, 0x0000000000000000, // 5^1
        0xC800000000000000, 0x0000000000000000, // 5^2
        0xFA00000000000000, 0x0000000000000000, // 5^3
        0x9C40000000000000, 0x0000000000000000, // 5^4
        0xC350000000000000, 0x0000000000000000, // 5^5
        0xF424000000000000, 0x0000000000000000, // 5^6
        0x9896800000000000, 0x0000000000000000, // 5^7
        0xBEBC200000000000, 0x0000000000000000, // 5^8
        0xEE6B280000000000, 0x0000000000000000, // 5^9
        0x9502F90000000000, 0x0000000000000000, // 5^10
        0xBA43B74000000000, 0x0000000000000000, // 5^11
        0xE8D4A51000000000, 0x0000000000000000, // 5^12
        0x9184E72A00000000, 0x0000000000000000, // 5^13
        0xB5E620F480000000, 0x0000000000000000, // 5^14
        0xE35FA931A0000000, 0x0000000000000000, // 5^15
        0x8E1BC9BF04000000, 0x0000000000000000, // 5^16
        0xB1A2BC2EC5000000, 0x0000000000000000, // 5^17
        0xDE0B6B3A76400000, 0x0000000000000000, // 5^18
        0x8AC7230489E80000, 0x0000000000000000, // 5^19
        0xAD78EBC5AC620000, 0x0000000000000000, // 5^20
        0xD8D726B7177A8000, 0x0000000000000000, // 5^21
        0x878678326EAC9000, 0x0000000000000000, // 5^22
        0xA968163F0A57B400, 0x0000000000000000, // 5^23
        0xD3C21BCECCEDA100, 0x0000000000000000, // 5^24
        0x84595161401484A0, 0x0000000000000000, // 5^25
        0xA56FA5B99019A5C8, 0x0000000000000000, // 5^26
        0xCECB8F27F4200F3A, 0x0000000000000000, // 5^27
    ]

    // mantissa * 10^exponent rounded to nearest (ties to even), for 0 < mantissa < 2^64 and |exponent| <= 27: fast_float's
    // compute_float<binary_format<double>> (Eisel-Lemire), which fast_float::from_chars uses for these inputs when the
    // Clinger fast path doesn't apply. The 128-bit product approximation is always sufficient for a 64-bit mantissa
    // (Mushtak and Lemire, "Fast Number Parsing Without Fallback"), and in this exponent range the result is a normal,
    // finite double (no zero, subnormal or infinity cases).
    @inline(__always)
    private static func eiselLemireDouble(_ mantissa: UInt64, _ exponent: Int) -> Double {
        let leadingZeros = mantissa.leadingZeroBitCount
        let normalized = mantissa << leadingZeros
        let index = 2 &* (exponent &+ 27)

        // compute_product_approximation<mantissa_explicit_bits + 3 = 55>: precision_mask = ~0 >> 55.
        let first = normalized.multipliedFullWidth(by: powerOfFive128[index])
        var high = first.high
        var low = first.low
        if high & 0x1FF == 0x1FF {
            let second = normalized.multipliedFullWidth(by: powerOfFive128[index &+ 1])
            low &+= second.high
            if second.high > low {
                high &+= 1
            }
        }

        let upperBit = Int(truncatingIfNeeded: high >> 63)
        let shift = upperBit &+ 64 &- 52 &- 3
        var significand = high >> UInt64(truncatingIfNeeded: shift)
        // detail::power(q) + upperbit - lz - minimum_exponent, with power(q) = ((152170 + 65536) * q >> 16) + 63.
        var power2 = ((152170 &+ 65536) &* exponent) >> 16 &+ 63 &+ upperBit &- leadingZeros &+ 1023

        // Usually round up, but round to even when exactly halfway (only possible when 5^q fits in 64 bits).
        if low <= 1 && exponent >= -4 && exponent <= 23 && significand & 3 == 1 && significand << UInt64(truncatingIfNeeded: shift) == high {
            significand &= ~1
        }
        significand &+= significand & 1
        significand >>= 1
        if significand >= 2 << 52 {
            significand = 1 << 52
            power2 &+= 1
        }
        significand &= ~(1 << 52)
        return Double(bitPattern: UInt64(truncatingIfNeeded: power2) << 52 | significand)
    }

    // The standard library's correctly rounded decimal conversion (strtod-based) of text[start...], which
    // charactersToDouble() has validated as a decimal number. Rare: more than 19 digits, or a decimal exponent outside
    // [-27, 27].
    @inline(never)
    private static func correctlyRoundedDouble<C: CSSColorCharacters & ~Escapable>(_ text: C, from start: Int) -> Double? {
        var string = ""
        for index in start..<text.count {
            string.unicodeScalars.append(Unicode.Scalar(text[index]))
        }
        return Double(string)
    }

    @inline(__always)
    static func isUnicodeCompatibleASCIIWhitespace(_ character: UInt8) -> Bool {
        isASCIIWhitespace(character) || character == asciiVerticalTab
    }

    // MARK: - HSL to SRGBA<uint8_t>

    // degreesPerRadianDouble = 180.0 / std::numbers::pi.
    private static let degreesPerRadian: Double = 180.0 / Double.pi

    // `lightness - a * m` from ColorConversion<ExtendedSRGBA<float>, HSLA<float>>::convert. clang contracts it
    // into one fused multiply-add wherever the target has FMA: always on arm64, never on the x86_64 baseline.
    @inline(__always)
    private static func lightnessMinusProduct(_ lightness: Float, _ a: Float, _ m: Float) -> Float {
#if arch(arm64) || arch(arm64_32)
        return lightness.addingProduct(-a, m)
#else
        return lightness - a * m
#endif
    }

    // std::min(a, b) / std::max(a, b) / std::clamp(v, lo, hi), which differ from Swift's min/max in which
    // operand they return on ties and unordered (NaN) comparisons.
    @inline(__always)
    private static func stdMin<T: BinaryFloatingPoint>(_ a: T, _ b: T) -> T {
        b < a ? b : a
    }

    @inline(__always)
    private static func stdMax<T: BinaryFloatingPoint>(_ a: T, _ b: T) -> T {
        a < b ? b : a
    }

    @inline(__always)
    private static func stdClamp<T: BinaryFloatingPoint>(_ value: T, _ low: T, _ high: T) -> T {
        value < low ? low : (high < value ? high : value)
    }

    // std::min({ a, b, c }): the first smallest element.
    @inline(__always)
    private static func stdMin3(_ a: Float, _ b: Float, _ c: Float) -> Float {
        let ab = b < a ? b : a
        return c < ab ? c : ab
    }

    // std::clamp(std::lround(value * 255.0f), 0l, 255l) for value in [0, 1] (the callers clamp it): value * 255 is then
    // in [0, 255], so the clamp is a no-op and the rounded integer is read out of the mantissa of rounded + 2^23.
    @inline(__always)
    private static func unitIntervalToByte(_ value: Float) -> UInt8 {
        UInt8(truncatingIfNeeded: ((value * 255.0).rounded(.toNearestOrAwayFromZero) + 8388608.0).bitPattern)
    }

    // normalizeHue<double>(ColorNormalization.h).
    @inline(__always)
    private static func normalizeHue(_ hue: Double) -> Double {
        (hue.truncatingRemainder(dividingBy: 360.0) + 360.0).truncatingRemainder(dividingBy: 360.0)
    }

    @inline(__always)
    private static func hueToRGB(_ n: Float, _ scaledHue: Float, _ scaledLightness: Float, _ a: Float) -> Float {
        let k = (n + scaledHue).truncatingRemainder(dividingBy: 12.0)
        return lightnessMinusProduct(scaledLightness, a, stdMax(-1.0, stdMin3(k - 3.0, 9.0 - k, 1.0)))
    }

    // The tail of parseHSL, from `auto parsedColor = StyleColorParseType<HSLFunctionModern> { ... }` to
    // `.tryGetAsSRGBABytes()`.
    private static func hslToSRGBABytes(hue: Double, hueIsRadians: Bool, saturation: Double, lightness: Double, alpha: Double) -> UInt64 {
        // StyleColorParseType<HSLFunctionModern>: each component is narrowPrecisionToFloat()ed and stored as a double;
        // CSS::convertAngle<CSS::AngleUnit::Deg> is rad2deg(), `r * degreesPerRadianDouble`, for radians.
        let styleHue = Double(Float(hueIsRadians ? hue * degreesPerRadian : hue))
        let styleSaturation = Double(Float(saturation))
        let styleLightness = Double(Float(lightness))
        let styleAlpha = Double(Float(alpha))

        // convertToTypedColor<HSLFunctionModern>: H is an angle (normalizeHue), S is clamped to [0, inf), L is
        // unclamped, alpha is clamped to [0, 1]; percentMultiplier and numberMultiplier are both 1. Each is narrowed to float.
        let typedHue = Float(normalizeHue(styleHue))
        let typedSaturation = Float(stdMax(styleSaturation * 1.0, 0.0))
        let typedLightness = Float(styleLightness * 1.0)
        let typedAlpha = Float(stdClamp(styleAlpha * 1.0, 0.0, 1.0))

        // ConvertToColor<HSLFunctionModern, Absolute>::convertToColor: "none" components (NaN) and colors outside the
        // sRGB gamut stay HSLA, which tryGetAsSRGBABytes() reports as std::nullopt.
        if typedHue.isNaN || typedSaturation.isNaN || typedLightness.isNaN || typedAlpha.isNaN {
            return notAColor
        }
        if Double(typedSaturation) > 100.0 || Double(typedLightness) < 0.0 || Double(typedLightness) > 100.0 {
            return notAColor
        }

        // ColorConversion<ExtendedSRGBA<float>, HSLA<float>>::convert (resolved() is the identity without NaNs).
        let onePercent: Float = 1.0 / 100.0
        let scaledHue = typedHue / 30.0
        let scaledSaturation = typedSaturation * onePercent
        let scaledLightness = typedLightness * onePercent

        let a = scaledSaturation * stdMin(scaledLightness, 1.0 - scaledLightness)

        let red = hueToRGB(0, scaledHue, scaledLightness, a)
        let green = hueToRGB(8, scaledHue, scaledLightness, a)
        let blue = hueToRGB(4, scaledHue, scaledLightness, a)

        // clipToGamut<SRGBA<float>> clamps red, green and blue to [0, 1]; handleToByteConversion then rounds every
        // component (alpha included) with std::clamp(std::lround(value * 255.0f), 0l, 255l).
        return packedColor(
            unitIntervalToByte(stdClamp(red, 0.0, 1.0)),
            unitIntervalToByte(stdClamp(green, 0.0, 1.0)),
            unitIntervalToByte(stdClamp(blue, 0.0, 1.0)),
            unitIntervalToByte(typedAlpha))
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
