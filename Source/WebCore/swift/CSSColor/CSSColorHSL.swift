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
    // fast path below, or else the standard library's conversion of the validated text.
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
            } else if exponent < 0 && exponent >= -22 {
                magnitude = verifiedQuotient(mantissa, -exponent)
            } else {
                magnitude = exactlyRoundedDouble(mantissa, exponent)
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

    // 5^e for 0 <= e <= 27 (5^27 < 2^63).
    @inline(__always)
    private static func powerOfFive(_ exponent: Int) -> UInt64 {
        switch exponent {
        case 0: 1
        case 1: 5
        case 2: 25
        case 3: 125
        case 4: 625
        case 5: 3125
        case 6: 15625
        case 7: 78125
        case 8: 390625
        case 9: 1953125
        case 10: 9765625
        case 11: 48828125
        case 12: 244140625
        case 13: 1220703125
        case 14: 6103515625
        case 15: 30517578125
        case 16: 152587890625
        case 17: 762939453125
        case 18: 3814697265625
        case 19: 19073486328125
        case 20: 95367431640625
        case 21: 476837158203125
        case 22: 2384185791015625
        case 23: 11920928955078125
        case 24: 59604644775390625
        case 25: 298023223876953125
        case 26: 1490116119384765625
        case 27: 7450580596923828125
        default: preconditionFailure("power of five out of range")
        }
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

    // mantissa / 10^j rounded to nearest (ties to even), for 2^53 < mantissa < 2^64 and 1 <= j <= 22: the double
    // quotient is within a few ulps; an exact 128-bit comparison of the value with the candidate and its midpoints picks
    // the correctly rounded double. A candidate at a power of two (asymmetric neighbors) goes to exactlyRoundedDouble().
    //
    // With candidate = s * 2^p and value = mantissa / 10^j, scaling by 2 * 10^j * 2^-p gives value -> mantissa * 2^(1 - p),
    // candidate -> 2s * 10^j, and the midpoints -> 2s * 10^j +- 10^j. Both sides stay below 2^128 (2s < 2^54 and
    // 10^j < 2^74, with 2s * 10^j < 2^127.2) and within about 2^74 of each other.
    private static func verifiedQuotient(_ mantissa: UInt64, _ j: Int) -> Double {
        let tenPower = UInt128(powerOfFive(j)) << j // 10^j < 2^74.
        var candidate = Double(mantissa) / powerOfTen(j)
        for _ in 0..<4 {
            let significand = candidate.significandBitPattern | 1 << 52
            if significand == 1 << 52 {
                break
            }
            var value = UInt128(mantissa)
            var scaledCandidate = UInt128(2 &* significand) &* tenPower
            var halfStep = tenPower
            let shift = 1 &- (Int(candidate.exponent) &- 52)
            if shift >= 0 {
                value <<= shift
            } else {
                scaledCandidate <<= -shift
                halfStep <<= -shift
            }
            let difference = Int128(truncatingIfNeeded: value &- scaledCandidate)
            let half = Int128(truncatingIfNeeded: halfStep)
            let isOdd = significand & 1 == 1
            if difference > half || (difference == half && isOdd) {
                candidate = candidate.nextUp
                continue
            }
            if difference < -half || (difference == -half && isOdd) {
                candidate = candidate.nextDown
                continue
            }
            return candidate
        }
        return exactlyRoundedDouble(mantissa, -j)
    }

    // mantissa * 10^exponent rounded to nearest (ties to even), for 0 < mantissa < 2^64 and |exponent| <= 27, with
    // exact integer arithmetic: 10^e = 5^e * 2^e and 5^27 < 2^63, so mantissa * 5^e (e >= 0) fits in 128 bits, and
    // for e < 0 the 128-bit quotient (mantissa << shift) / 5^-e has at least 55 bits, plus a sticky remainder.
    private static func exactlyRoundedDouble(_ mantissa: UInt64, _ exponent: Int) -> Double {
        let power = powerOfFive(exponent < 0 ? -exponent : exponent)
        var value: UInt128
        var binaryExponent: Int
        var isInexact = false
        if exponent >= 0 {
            value = UInt128(mantissa) &* UInt128(power)
            binaryExponent = exponent
        } else {
            let shift = max(0, 56 &+ (64 &- power.leadingZeroBitCount) &- (64 &- mantissa.leadingZeroBitCount))
            let (quotient, remainder) = (UInt128(mantissa) << shift).quotientAndRemainder(dividingBy: UInt128(power))
            value = quotient
            isInexact = remainder != 0
            binaryExponent = exponent &- shift
        }
        let bitCount = 128 &- value.leadingZeroBitCount
        if bitCount > 53 {
            let dropped = bitCount &- 53
            let droppedBits = value & ((1 << dropped) &- 1)
            let half: UInt128 = 1 << (dropped &- 1)
            value >>= dropped
            binaryExponent &+= dropped
            if droppedBits > half || (droppedBits == half && (isInexact || value & 1 == 1)) {
                value &+= 1 // At most 2^53, still exact.
            }
        }
        return Double(sign: .plus, exponent: binaryExponent, significand: Double(UInt64(truncatingIfNeeded: value)))
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
