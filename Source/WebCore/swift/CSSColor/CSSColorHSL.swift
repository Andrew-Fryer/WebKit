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

// Memory-safe port of parseHSL<Latin1Character> (CSSParserFastPaths.cpp), including the inlined
// parseSimpleAngle(RequireUnits::No) / parseCSSNumber hue parsing and the typed-color tail that turns
// the parsed components into SRGBA<uint8_t>:
//   StyleColorParseType<HSLFunctionModern> -> convertToTypedColor<HSLFunctionModern>   (CSSColorConversion+ToTypedColor.h)
//   -> ConvertToColor<HSLFunctionModern, Absolute>::convertToColor(color, 0)            (CSSColorConversion+ToColor.h)
//   -> convertColor<SRGBA<uint8_t>>(HSLA<float>): ColorConversion<ExtendedSRGBA<float>, HSLA<float>>::convert
//      (ColorConversion.cpp), clipToGamut (ColorTypes.h), handleToByteConversion (ColorConversion.h)
//   -> Color::tryGetAsSRGBABytes(), which is std::nullopt whenever convertToColor keeps the HSLA (out of gamut / none).
// Number text for the hue goes through the caller's charactersToDouble() (NumberParser), so it is exact by construction.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

extension CSSColor {
    // parseHSL(characters[start...]); `start` is just past the '('.
    static func parseHSL<Parser: NumberParser>(_ characters: Span<UInt8>, _ start: Int, _ parser: Parser) -> UInt64 {
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

        guard let angle = parseSimpleAngle(characters, start, hueEnd, parser) else {
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
    private static func skipCSSSpaces(_ characters: Span<UInt8>, _ current: inout Int) {
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
    private static func parseSimpleAngle<Parser: NumberParser>(_ characters: Span<UInt8>, _ start: Int, _ end: Int, _ parser: Parser) -> (number: Double, isRadians: Bool)? {
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
        guard let number = parser.parseNumber(start, numberEnd &- start) else {
            return nil
        }
        return (number: number, isRadians: isRadians)
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
