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

// Memory-safe port of the Latin-1 numeric color fast path of CSSParserFastPaths.cpp:
// parseNumericColor (#hex, quirks-mode bare hex, rgb()/rgba(), hsl()/hsla() dispatch),
// parseHexColorInternal, finishParsingHexColor, parseColorIntOrPercentage, parseRGBAlphaValue,
// isTenthAlpha, checkForValidDouble and parseDouble. hsl()/hsla() is in CSSColorHSL.swift.
// Every function mirrors its C++ twin statement by statement; the C++ names are kept.
//
// Results are packed into a UInt64: 0 means "not a numeric color" (std::nullopt), otherwise
// bit 32 is set and the low 32 bits are PackedColor::RGBA (r << 24 | g << 16 | b << 8 | a).
//
// Index discipline: each helper validates its [start, end) range against the span once up front,
// and the scanning loops run over a sub-span (`extracting`) whose count is the loop bound, so the
// optimizer can drop the per-character bounds checks. No allocation, no reference counting.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum CSSColor {
}

extension CSSColor {
    // Parses a number with WTF::charactersToDouble() semantics. The arguments are (offset, length) into the
    // characters being parsed; nil is charactersToDouble() setting ok = false. The production conformance
    // (CSSColorBoundary.swift) calls back into the C++ glue; generic so it is specialized and inlined.
    protocol NumberParser {
        func parseNumber(_ offset: Int, _ length: Int) -> Double?
    }

    static func parseNumericColor<Parser: NumberParser>(_ characters: Span<UInt8>, quirksMode: Bool, parser: Parser) -> UInt64 {
        let count = characters.count
        if count >= 4 && characters[0] == asciiHash {
            let hexColor = parseHexColorInternal(characters, 1, count)
            if hexColor != notAColor {
                return hexColor
            }
        }

        if quirksMode && (count == 3 || count == 6) {
            let hexColor = parseHexColorInternal(characters, 0, count)
            if hexColor != notAColor {
                return hexColor
            }
        }

        let isRGBA = mightBeRGBA(characters)
        if isRGBA || mightBeRGB(characters) {
            return parseRGB(characters, isRGBA ? 5 : 4)
        }

        // hsl() and hsla() are synonyms.
        if mightBeHSLA(characters) {
            return parseHSL(characters, 5, parser)
        }

        if mightBeHSL(characters) {
            return parseHSL(characters, 4, parser)
        }

        return notAColor
    }
}

extension CSSColor {
    // MARK: - Result packing

    static let notAColor: UInt64 = 0

    @inline(__always)
    static func packedColor(_ red: UInt8, _ green: UInt8, _ blue: UInt8, _ alpha: UInt8) -> UInt64 {
        UInt64(1) << 32 | UInt64(red) << 24 | UInt64(green) << 16 | UInt64(blue) << 8 | UInt64(alpha)
    }

    // MARK: - Characters (WTF ASCIICType.h, CSSParserIdioms.h)

    static let asciiTab: UInt8 = 0x09
    static let asciiNewline: UInt8 = 0x0A
    static let asciiFormFeed: UInt8 = 0x0C
    static let asciiCarriageReturn: UInt8 = 0x0D
    static let asciiSpace: UInt8 = 0x20
    static let asciiHash: UInt8 = 0x23
    static let asciiPercent: UInt8 = 0x25
    static let asciiOpenParen: UInt8 = 0x28
    static let asciiCloseParen: UInt8 = 0x29
    static let asciiComma: UInt8 = 0x2C
    static let asciiMinus: UInt8 = 0x2D
    static let asciiPeriod: UInt8 = 0x2E
    static let asciiSlash: UInt8 = 0x2F
    static let asciiZero: UInt8 = 0x30
    static let asciiOne: UInt8 = 0x31

    @inline(__always)
    static func isASCIIDigit(_ character: UInt8) -> Bool {
        character &- asciiZero < 10
    }

    @inline(__always)
    private static func isASCIIHexDigit(_ character: UInt8) -> Bool {
        // isASCIIDigit(character) || (toASCIILowerUnchecked(character) >= 'a' && toASCIILowerUnchecked(character) <= 'f'),
        // where toASCIILowerUnchecked(character) is character | 0x20. Evaluated without short-circuiting so the
        // per-digit check is a single, well-predicted branch (clang emits cmp + ccmp for the C++); with `||`
        // the digit/letter test is a data-dependent branch that mispredicts on random hex strings.
        let isDigit: UInt8 = character &- asciiZero < 10 ? 1 : 0
        let isLetter: UInt8 = (character | 0x20) &- 0x61 < 6 ? 1 : 0
        return isDigit | isLetter != 0
    }

    @inline(__always)
    private static func toASCIIHexValue(_ character: UInt8) -> UInt8 {
        character < 0x41 ? character &- asciiZero : (character &- 0x41 &+ 10) & 0xF
    }

    @inline(__always)
    static func isASCIIWhitespace(_ character: UInt8) -> Bool {
        character == asciiSpace || character == asciiNewline || character == asciiTab || character == asciiCarriageReturn || character == asciiFormFeed
    }

    @inline(__always)
    static func isCSSSpace(_ character: UInt8) -> Bool {
        character == asciiSpace || character == asciiTab || character == asciiNewline
    }

    @inline(__always)
    static func isASCIIAlphaCaselessEqual(_ character: UInt8, _ expectedASCIILowercaseLetter: UInt8) -> Bool {
        character | 0x20 == expectedASCIILowercaseLetter
    }

    // MARK: - Function names

    @inline(__always)
    private static func mightBeFunction(_ characters: Span<UInt8>, _ name0: UInt8, _ name1: UInt8, _ name2: UInt8) -> Bool {
        if characters.count < 4 {
            return false
        }
        return characters[3] == asciiOpenParen
            && isASCIIAlphaCaselessEqual(characters[0], name0)
            && isASCIIAlphaCaselessEqual(characters[1], name1)
            && isASCIIAlphaCaselessEqual(characters[2], name2)
    }

    @inline(__always)
    private static func mightBeFunction(_ characters: Span<UInt8>, _ name0: UInt8, _ name1: UInt8, _ name2: UInt8, _ name3: UInt8) -> Bool {
        if characters.count < 5 {
            return false
        }
        return characters[4] == asciiOpenParen
            && isASCIIAlphaCaselessEqual(characters[0], name0)
            && isASCIIAlphaCaselessEqual(characters[1], name1)
            && isASCIIAlphaCaselessEqual(characters[2], name2)
            && isASCIIAlphaCaselessEqual(characters[3], name3)
    }

    @inline(__always)
    private static func mightBeRGBA(_ characters: Span<UInt8>) -> Bool {
        mightBeFunction(characters, 0x72, 0x67, 0x62, 0x61) // "rgba("
    }

    @inline(__always)
    private static func mightBeRGB(_ characters: Span<UInt8>) -> Bool {
        mightBeFunction(characters, 0x72, 0x67, 0x62) // "rgb("
    }

    @inline(__always)
    private static func mightBeHSLA(_ characters: Span<UInt8>) -> Bool {
        mightBeFunction(characters, 0x68, 0x73, 0x6C, 0x61) // "hsla("
    }

    @inline(__always)
    private static func mightBeHSL(_ characters: Span<UInt8>) -> Bool {
        mightBeFunction(characters, 0x68, 0x73, 0x6C) // "hsl("
    }

    // MARK: - Hex colors

    @inline(__always)
    private static func finishParsingHexColor(_ value: UInt32, _ length: Int) -> UInt64 {
        switch length {
        case 3:
            // #234 converts to #223344.
            let r = UInt8(truncatingIfNeeded: (value & 0x0F00) >> 8)
            let g = UInt8(truncatingIfNeeded: (value & 0x00F0) >> 4)
            let b = UInt8(truncatingIfNeeded: value & 0x000F)
            return packedColor(r << 4 | r, g << 4 | g, b << 4 | b, 255)
        case 4:
            // #234a converts to #223344aa.
            let r = UInt8(truncatingIfNeeded: (value & 0xF000) >> 12)
            let g = UInt8(truncatingIfNeeded: (value & 0x0F00) >> 8)
            let b = UInt8(truncatingIfNeeded: (value & 0x00F0) >> 4)
            let a = UInt8(truncatingIfNeeded: value & 0x000F)
            return packedColor(r << 4 | r, g << 4 | g, b << 4 | b, a << 4 | a)
        case 6:
            // asSRGBA(PackedColor::ARGB { 0xFF000000 | value }).
            return UInt64(1) << 32 | UInt64(value << 8 | 0xFF)
        case 8:
            // asSRGBA(PackedColor::RGBA { value }).
            return UInt64(1) << 32 | UInt64(value)
        default:
            return notAColor
        }
    }

    // parseHexColorInternal(characters[start..<end]).
    @inline(__always)
    private static func parseHexColorInternal(_ characters: Span<UInt8>, _ start: Int, _ end: Int) -> UInt64 {
        let length = end &- start
        guard start >= 0, end <= characters.count, length == 3 || length == 4 || length == 6 || length == 8 else {
            return notAColor
        }

        var value: UInt32 = 0
        var index = start
        while index < end {
            let digit = characters[index]
            if !isASCIIHexDigit(digit) {
                return notAColor
            }
            value <<= 4
            value |= UInt32(toASCIIHexValue(digit))
            index &+= 1
        }
        return finishParsingHexColor(value, length)
    }

    // MARK: - Numbers

    // C++ `value * 10 + character - '0'`. clang contracts `value * 10 + character` into one fused
    // multiply-add wherever the target has FMA (-ffp-contract=on is the default and WebKit doesn't
    // override it): always on arm64, never on the x86_64 baseline CPU (no FMA). swiftc never contracts.
    @inline(__always)
    static func appendDigit(_ value: Double, _ character: UInt8) -> Double {
    #if arch(arm64) || arch(arm64_32)
        return Double(character).addingProduct(value, 10) - 48
    #else
        return value * 10 + Double(character) - 48
    #endif
    }

    // Returns the number of characters which form a valid double
    // and are terminated by the given terminator character (checkForValidDouble).
    // The terminator is never a digit (callers pass '%' or ')'), so testing for a digit first, which keeps
    // the common case to one well-predicted branch per character, is equivalent to the C++ order.
    @inline(__always)
    static func checkForValidDouble(_ string: Span<UInt8>, _ terminator: UInt8) -> Int {
        let length = string.count
        if length < 1 {
            return 0
        }

        var decimalMarkOffset = -1
        var processedLength = 0

        for i in string.indices {
            let character = string[i]
            if isASCIIDigit(character) {
                continue
            }
            if character == terminator {
                processedLength = i
                break
            }
            if decimalMarkOffset < 0 && character == asciiPeriod {
                decimalMarkOffset = i
            } else {
                return 0
            }
        }

        // CSS disallows a period without a subsequent digit.
        if decimalMarkOffset >= 0 && decimalMarkOffset == processedLength &- 1 {
            return 0
        }

        return processedLength
    }

    // checkForValidDouble(characters[start..<end], terminator).
    @inline(__always)
    static func checkForValidDouble(_ characters: Span<UInt8>, _ start: Int, _ end: Int, _ terminator: UInt8) -> Int {
        guard start >= 0, start <= end, end <= characters.count else {
            return 0
        }
        return checkForValidDouble(characters.extracting(start..<end), terminator)
    }

    // Returns the number of characters consumed for parsing a valid double
    // terminated by the given terminator character (parseDouble). `value` is only written on success.
    @inline(__always)
    static func parseDouble(_ string: Span<UInt8>, _ terminator: UInt8, _ value: inout Double) -> Int {
        let length = checkForValidDouble(string, terminator)
        // A non-zero result is the index of the terminator, so it is < string.count.
        guard length > 0, length < string.count else {
            return 0
        }

        // The consumed characters here are guaranteed to be
        // ASCII digits with or without a decimal mark
        let digits = string.extracting(0..<length)
        var position = 0
        var localValue: Double = 0

        while position < digits.count {
            let character = digits[position]
            if character == asciiPeriod {
                break
            }
            localValue = appendDigit(localValue, character)
            position &+= 1
        }

        position &+= 1
        if position == length {
            value = localValue
            return length
        }

        var fraction: Double = 0
        var scale: Double = 1

        let maxScale: Double = 1000000
        while position < digits.count && scale < maxScale {
            fraction = appendDigit(fraction, digits[position])
            position &+= 1
            scale *= 10
        }

        value = localValue + fraction / scale
        return length
    }

    // parseDouble(characters[start..<end], terminator, value).
    @inline(__always)
    static func parseDouble(_ characters: Span<UInt8>, _ start: Int, _ end: Int, _ terminator: UInt8, _ value: inout Double) -> Int {
        guard start >= 0, start <= end, end <= characters.count else {
            return 0
        }
        return parseDouble(characters.extracting(start..<end), terminator, &value)
    }

    // MARK: - Byte conversion (ColorUtilities.h)

    // std::clamp(std::lround(value), 0l, 255l) for a float. lround() rounds half away from zero; out of
    // range values saturate (arm64 fcvtas, x86_64 libm lroundf) and then clamp, so +inf gives 255 and NaN 0.
    // Branch-free: clamp the rounded value to [0, 255] (NaN -> 0), then read the integer out of the low
    // mantissa bits of clamped + 2^23 (exact for integers below 2^23). Checked against the C++ expression
    // for all 2^32 floats on arm64e and x86_64.
    @inline(__always)
    static func clampedRoundedByte(_ value: Float) -> UInt8 {
        let rounded = value.rounded(.toNearestOrAwayFromZero)
        let clamped: Float = rounded >= 255 ? 255 : (rounded > 0 ? rounded : 0)
        return UInt8(truncatingIfNeeded: (clamped + 8388608.0).bitPattern)
    }

    // convertPrescaledSRGBAFloatToSRGBAByte(float).
    @inline(__always)
    private static func convertPrescaledSRGBAFloatToSRGBAByte(_ value: Float) -> UInt8 {
        clampedRoundedByte(value)
    }

    // convertFloatAlphaTo<uint8_t>(float).
    @inline(__always)
    private static func convertFloatAlphaToByte(_ value: Float) -> UInt8 {
        clampedRoundedByte(value * 255)
    }

    // MARK: - rgb() / rgba()

    private enum CSSUnitType {
        case unknown
        case number
        case percentage
    }

    // parseColorIntOrPercentage. `position` is the start of the remaining characters; on success it is
    // advanced past the component (and the terminator, if any) and the byte is returned; nil is std::nullopt.
    private static func parseColorIntOrPercentage(_ characters: Span<UInt8>, _ position: inout Int, _ consumableTerminator: UInt8?, _ expectedUnitType: inout CSSUnitType) -> UInt8? {
        let end = characters.count
        var current = position
        guard current >= 0, current <= end else {
            return nil
        }
        var localValue: Double = 0
        var negative = false
        while current < end && isASCIIWhitespace(characters[current]) {
            current &+= 1
        }

        if current < end && characters[current] == asciiMinus {
            negative = true
            current &+= 1
        }

        if current == end || !isASCIIDigit(characters[current]) {
            return nil
        }

        while current < end && isASCIIDigit(characters[current]) {
            let newValue = appendDigit(localValue, characters[current])
            current &+= 1
            if newValue >= 255 {
                // Clamp values at 255.
                localValue = 255
                while current < end && isASCIIDigit(characters[current]) {
                    current &+= 1
                }
                break
            }
            localValue = newValue
        }

        if current == end {
            return nil
        }

        var front = characters[current]
        if expectedUnitType == .number && (front == asciiPeriod || front == asciiPercent) {
            return nil
        }

        if front == asciiPeriod {
            // We already parsed the integral part, try to parse
            // the fraction part of the percentage value.
            var percentage: Double = 0
            let numCharactersParsed = parseDouble(characters, current, end, asciiPercent, &percentage)
            if numCharactersParsed == 0 {
                return nil
            }
            current &+= numCharactersParsed
            // A non-zero result means the terminator is at `current`, so `current < end`.
            guard current < end else {
                return nil
            }
            front = characters[current]
            if front != asciiPercent {
                return nil
            }
            localValue += percentage
        }

        if expectedUnitType == .percentage && front != asciiPercent {
            return nil
        }

        if front == asciiPercent {
            current &+= 1
            expectedUnitType = .percentage
            localValue = localValue / 100.0 * 255.0
            // Clamp values at 255 for percentages over 100%
            if localValue > 255 {
                localValue = 255
            }
        } else {
            expectedUnitType = .number
        }

        while current < end && isASCIIWhitespace(characters[current]) {
            current &+= 1
        }

        if let consumableTerminator {
            if current == end || characters[current] != consumableTerminator {
                return nil
            }
            current &+= 1
        }

        position = current

        // Clamp negative values at zero.
        return negative ? 0 : convertPrescaledSRGBAFloatToSRGBAByte(Float(localValue))
    }

    // isTenthAlpha(characters[start..<end]).
    @inline(__always)
    private static func isTenthAlpha(_ characters: Span<UInt8>, _ start: Int, _ end: Int) -> Bool {
        guard start >= 0, end <= characters.count else {
            return false
        }
        let length = end &- start

        // "0.X"
        if length == 3 && characters[start] == asciiZero && characters[start &+ 1] == asciiPeriod && isASCIIDigit(characters[start &+ 2]) {
            return true
        }

        // ".X"
        if length == 2 && characters[start] == asciiPeriod && isASCIIDigit(characters[start &+ 1]) {
            return true
        }

        return false
    }

    // tenthAlphaValues { 0, 26, 51, 77, 102, 128, 153, 179, 204, 230 }[digit - '0'].
    @inline(__always)
    private static func tenthAlphaValue(_ digit: UInt8) -> UInt8 {
        switch digit {
        case 0x30: 0
        case 0x31: 26
        case 0x32: 51
        case 0x33: 77
        case 0x34: 102
        case 0x35: 128
        case 0x36: 153
        case 0x37: 179
        case 0x38: 204
        default: 230
        }
    }

    // parseRGBAlphaValue on characters[position...]. On success the remaining characters are
    // consumed (position = end) and the alpha byte is returned; nil is std::nullopt.
    private static func parseRGBAlphaValue(_ characters: Span<UInt8>, _ position: inout Int, _ terminator: UInt8) -> UInt8? {
        let end = characters.count
        var start = position
        guard start >= 0, start <= end else {
            return nil
        }
        while start < end && isASCIIWhitespace(characters[start]) {
            start &+= 1
        }

        var negative = false

        if start < end && characters[start] == asciiMinus {
            negative = true
            start &+= 1
        }

        let length = end &- start
        if length < 2 {
            return nil
        }

        if characters[end &- 1] != terminator || !isASCIIDigit(characters[end &- 2]) {
            return nil
        }

        let front = characters[start]
        if front != asciiZero && front != asciiOne && front != asciiPeriod {
            if checkForValidDouble(characters, start, end, terminator) != 0 {
                position = end
                return negative ? 0 : 255
            }
            return nil
        }

        if length == 2 && front != asciiPeriod {
            let result: UInt8 = !negative && front == asciiOne ? 255 : 0
            position = end
            return result
        }

        if isTenthAlpha(characters, start, end &- 1) {
            let result = negative ? 0 : tenthAlphaValue(characters[end &- 2])
            position = end
            return result
        }

        var alpha: Double = 0
        if parseDouble(characters, start, end, terminator, &alpha) == 0 {
            return nil
        }

        position = end
        return negative ? 0 : convertFloatAlphaToByte(Float(alpha))
    }

    // The rgb()/rgba() branch of parseNumericColor; `start` is just past the '('.
    private static func parseRGB(_ characters: Span<UInt8>, _ start: Int) -> UInt64 {
        let end = characters.count
        var expectedUnitType = CSSUnitType.unknown
        var current = start

        // Red and green will both terminate with ','.
        guard let red = parseColorIntOrPercentage(characters, &current, asciiComma, &expectedUnitType) else {
            return notAColor
        }
        guard let green = parseColorIntOrPercentage(characters, &current, asciiComma, &expectedUnitType) else {
            return notAColor
        }

        // Blue may terminate with ',' or ')', but do not consume either terminator.
        guard let blue = parseColorIntOrPercentage(characters, &current, nil, &expectedUnitType) else {
            return notAColor
        }
        guard current >= 0, current < end else {
            return notAColor
        }

        // Finish parsing rgb if no alpha value.
        if characters[current] == asciiCloseParen {
            current &+= 1
            if current != end {
                return notAColor
            }
            return packedColor(red, green, blue, 255)
        }

        // Parse alpha value if present.
        if characters[current] == asciiComma {
            current &+= 1
            guard let alpha = parseRGBAlphaValue(characters, &current, asciiCloseParen) else {
                return notAColor
            }
            if current != end {
                return notAColor
            }
            return packedColor(red, green, blue, alpha)
        }

        return notAColor
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
