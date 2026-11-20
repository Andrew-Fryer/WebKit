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

// Memory-safe port of the color fast paths of CSSParserFastPaths.cpp (at 05fbeeaa85b0) for Latin-1 and UTF-16
// strings: CSSParserFastPaths::parseSimpleColor (parseSimpleColorInternal), parseNumericColor (#hex, quirks-mode bare
// hex, rgb()/rgba(), hsl()/hsla() dispatch), CSSParserFastPaths::parseHexColor, parseHexColorInternal,
// finishParsingHexColor, parseColorIntOrPercentage, parseRGBAlphaValue, isTenthAlpha, checkForValidDouble, parseDouble,
// and CSSParserFastPaths::parseNamedColor (parseNamedColorInternal, finishParsingNamedColor and its findColor() lookup
// of platform/ColorData.gperf, whose table is CSSColorNamedColors.swift). hsl()/hsla() is in CSSColorHSL.swift.
// Every function mirrors its C++ twin statement by statement (the named-color lookup is a different hash with the
// same answers); the C++ names are kept.
//
// Results are packed into a UInt64: 0 means "not a color" (std::nullopt), otherwise bit 32 is set and the low 32
// bits are PackedColor::RGBA (r << 24 | g << 16 | b << 8 | a).
//
// Characters: the C++ templates only ever compare characters with ASCII constants or ASCII ranges, so every
// non-ASCII character behaves like every other one. UTF-16 code units are therefore read as bytes, with any unit
// >= 0x80 replaced by 0x80 (CSSColorUTF16Characters), and both widths run the same code.
//
// Index discipline: each helper validates its [start, end) range against the characters once up front, and the
// scanning loops run over a sub-range (`extracting`) whose count is the loop bound, so the optimizer can drop the
// per-character bounds checks. No allocation, no reference counting (outside the cold number fallback in
// CSSColorHSL.swift).

#if compiler(>=6.4) || WTF_PLATFORM_MAC

/// The characters being parsed, as bytes: Latin-1 characters as they are; UTF-16 code units below 0x80 as they are,
/// the others as 0x80.
protocol CSSColorCharacters: ~Escapable {
    var count: Int { get }
    var indices: Range<Int> { get }
    subscript(_ position: Int) -> UInt8 { get }
    @_lifetime(copy self)
    func extracting(_ bounds: Range<Int>) -> Self
    /// Characters [position, position + 8) as the bytes of a little-endian word, or a word with a non-digit byte
    /// when one of them is not ASCII.
    func eightCharacters(at position: Int) -> UInt64
}

/// Latin-1 characters. A wrapper rather than a conformance of Span to the internal protocol: such a conformance on a
/// standard-library type prints an unparseable .swiftinterface when the module is built with library evolution.
struct CSSColorLatin1Characters: CSSColorCharacters, ~Escapable {
    let span: Span<UInt8>

    @_lifetime(copy span)
    init(_ span: Span<UInt8>) {
        self.span = span
    }

    var count: Int {
        @inline(__always) get { span.count }
    }

    var indices: Range<Int> {
        @inline(__always) get { span.indices }
    }

    subscript(_ position: Int) -> UInt8 {
        @inline(__always) get { span[position] }
    }

    @_lifetime(copy self)
    func extracting(_ bounds: Range<Int>) -> CSSColorLatin1Characters {
        CSSColorLatin1Characters(span.extracting(bounds))
    }

    @inline(__always)
    func eightCharacters(at position: Int) -> UInt64 {
        span.bytes.load(fromByteOffset: position, as: UInt64.self)
    }
}

/// UTF-16 code units in any alignment (2 bytes each, little-endian).
struct CSSColorUTF16Characters: CSSColorCharacters, ~Escapable {
    let bytes: RawSpan

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    var count: Int {
        @inline(__always) get { bytes.byteCount &>> 1 }
    }

    var indices: Range<Int> {
        @inline(__always) get { 0..<count }
    }

    subscript(_ position: Int) -> UInt8 {
        @inline(__always) get {
            precondition(position >= 0 && position < count)
            let unit = bytes.load(fromByteOffset: position &* 2, as: UInt16.self)
            return unit < 0x80 ? UInt8(truncatingIfNeeded: unit) : 0x80
        }
    }

    @inline(__always)
    func eightCharacters(at position: Int) -> UInt64 {
        let low = bytes.load(fromByteOffset: position &* 2, as: UInt64.self)
        let high = bytes.load(fromByteOffset: position &* 2 &+ 8, as: UInt64.self)
        if (low | high) & 0xFF80_FF80_FF80_FF80 != 0 {
            return 0x8080_8080_8080_8080
        }
        let lowBytes = (low | low &>> 8) & 0x0000_FFFF_0000_FFFF
        let highBytes = (high | high &>> 8) & 0x0000_FFFF_0000_FFFF
        return (lowBytes | lowBytes &>> 16) & 0xFFFF_FFFF | ((highBytes | highBytes &>> 16) & 0xFFFF_FFFF) &<< 32
    }

    @inline(__always)
    @_lifetime(copy self)
    func extracting(_ bounds: Range<Int>) -> CSSColorUTF16Characters {
        precondition(bounds.lowerBound >= 0 && bounds.upperBound <= count)
        return CSSColorUTF16Characters(bytes.extracting((bounds.lowerBound &* 2)..<(bounds.upperBound &* 2)))
    }
}

enum CSSColor {
    // What to parse; CSSColorSwiftBoundary.h's WebCoreSwift_CSSColor_* mode values.
    static var modeNumeric: UInt8 { 0 }
    static var modeNumericQuirks: UInt8 { 1 }
    static var modeHex: UInt8 { 2 }
    static var modeSimple: UInt8 { 3 }
    static var modeSimpleQuirks: UInt8 { 4 }
    static var modeNamed: UInt8 { 5 }

    // parseNumericColor (modeNumeric, or modeNumericQuirks for isQuirksModeBehavior()),
    // CSSParserFastPaths::parseHexColor (modeHex), CSSParserFastPaths::parseSimpleColor (modeSimple, or
    // modeSimpleQuirks for isQuirksModeBehavior()) or CSSParserFastPaths::parseNamedColor (modeNamed).
    // Any other mode traps.
    @inline(__always)
    static func parseColor<C: CSSColorCharacters & ~Escapable>(_ characters: C, mode: UInt8) -> UInt64 {
        // The hot case (canvas fillStyle/strokeStyle "#rrggbb"), decided here so that it makes no further call.
        // In the numeric and simple modes, a '#' followed by at least 3 characters is parsed as exactly
        // parseHexColorInternal(characters[1...]): parseNumericColor tries that first and, when it fails, nothing
        // else can match a string that starts with '#' (the quirks-mode bare hex needs a hex digit there, the rgb(),
        // rgba(), hsl() and hsla() prefixes need 'r' or 'h', and parseNamedColor needs a letter).
        // The C entry point inlines this check and tail-calls parseColorOtherwise, so the hot case needs no stack frame.
        if mode != modeHex && mode <= modeSimpleQuirks && characters.count >= 4 && characters[0] == asciiHash {
            return parseHexColorInternal(characters, 1, characters.count)
        }
        return parseColorOtherwise(characters, mode: mode)
    }

    // One copy of parseNumericColor serves the numeric and simple modes (no further call level before parseRGB/parseHSL).
    @inline(never)
    static func parseColorOtherwise<C: CSSColorCharacters & ~Escapable>(_ characters: C, mode: UInt8) -> UInt64 {
        switch mode {
        case modeNumeric, modeNumericQuirks, modeSimple, modeSimpleQuirks:
            // parseSimpleColorInternal: parseNumericColor, then (simple modes only) parseNamedColorInternal.
            let color = parseNumericColor(characters, quirksMode: mode == modeNumericQuirks || mode == modeSimpleQuirks)
            if color != notAColor || mode == modeNumeric || mode == modeNumericQuirks {
                return color
            }
            return parseNamedColor(characters)
        case modeHex:
            return parseHexColorInternal(characters, 0, characters.count)
        case modeNamed:
            return parseNamedColor(characters)
        default:
            preconditionFailure("unknown CSSColor mode")
        }
    }
}

extension CSSColor {
    @inline(__always)
    static func parseNumericColor<C: CSSColorCharacters & ~Escapable>(_ characters: C, quirksMode: Bool) -> UInt64 {
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
            return parseHSL(characters, 5)
        }

        if mightBeHSL(characters) {
            return parseHSL(characters, 4)
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
    static let asciiVerticalTab: UInt8 = 0x0B
    static let asciiFormFeed: UInt8 = 0x0C
    static let asciiCarriageReturn: UInt8 = 0x0D
    static let asciiSpace: UInt8 = 0x20
    static let asciiHash: UInt8 = 0x23
    static let asciiPercent: UInt8 = 0x25
    static let asciiPlus: UInt8 = 0x2B
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

    // isASCIIHexDigit(character) ? toASCIIHexValue(character) : 0xFF, for every byte (a statically initialized constant
    // table). One load per digit replaces the C++ digit/letter tests; the values are the same.
    static let hexDigitValues: InlineArray<256, UInt8> = [
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x00
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x10
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x20
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x30
        0xFF, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x40
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x50
        0xFF, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x60
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x70
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x80
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x90
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xA0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xB0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xC0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xD0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xE0
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xF0
    ]

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
    private static func mightBeFunction<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ name0: UInt8, _ name1: UInt8, _ name2: UInt8) -> Bool {
        if characters.count < 4 {
            return false
        }
        return characters[3] == asciiOpenParen
            && isASCIIAlphaCaselessEqual(characters[0], name0)
            && isASCIIAlphaCaselessEqual(characters[1], name1)
            && isASCIIAlphaCaselessEqual(characters[2], name2)
    }

    @inline(__always)
    private static func mightBeFunction<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ name0: UInt8, _ name1: UInt8, _ name2: UInt8, _ name3: UInt8) -> Bool {
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
    private static func mightBeRGBA<C: CSSColorCharacters & ~Escapable>(_ characters: C) -> Bool {
        mightBeFunction(characters, 0x72, 0x67, 0x62, 0x61) // "rgba("
    }

    @inline(__always)
    private static func mightBeRGB<C: CSSColorCharacters & ~Escapable>(_ characters: C) -> Bool {
        mightBeFunction(characters, 0x72, 0x67, 0x62) // "rgb("
    }

    @inline(__always)
    private static func mightBeHSLA<C: CSSColorCharacters & ~Escapable>(_ characters: C) -> Bool {
        mightBeFunction(characters, 0x68, 0x73, 0x6C, 0x61) // "hsla("
    }

    @inline(__always)
    private static func mightBeHSL<C: CSSColorCharacters & ~Escapable>(_ characters: C) -> Bool {
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
    static func parseHexColorInternal<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int) -> UInt64 {
        let length = end &- start
        guard start >= 0, end <= characters.count, length == 3 || length == 4 || length == 6 || length == 8 else {
            return notAColor
        }

        var value: UInt32 = 0
        let digits = characters.extracting(start..<end)
        for index in digits.indices {
            let digitValue = hexDigitValues[Int(digits[index])]
            if digitValue > 0xF {
                return notAColor
            }
            value = value << 4 | UInt32(digitValue)
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
    static func checkForValidDouble<C: CSSColorCharacters & ~Escapable>(_ string: C, _ terminator: UInt8) -> Int {
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
    static func checkForValidDouble<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int, _ terminator: UInt8) -> Int {
        guard start >= 0, start <= end, end <= characters.count else {
            return 0
        }
        return checkForValidDouble(characters.extracting(start..<end), terminator)
    }

    // Returns the number of characters consumed for parsing a valid double
    // terminated by the given terminator character (parseDouble). `value` is only written on success.
    @inline(__always)
    static func parseDouble<C: CSSColorCharacters & ~Escapable>(_ string: C, _ terminator: UInt8, _ value: inout Double) -> Int {
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
    static func parseDouble<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int, _ terminator: UInt8, _ value: inout Double) -> Int {
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
    private static func parseColorIntOrPercentage<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ position: inout Int, _ consumableTerminator: UInt8?, _ expectedUnitType: inout CSSUnitType) -> UInt8? {
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
    private static func isTenthAlpha<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int, _ end: Int) -> Bool {
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
    private static func parseRGBAlphaValue<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ position: inout Int, _ terminator: UInt8) -> UInt8? {
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
    private static func parseRGB<C: CSSColorCharacters & ~Escapable>(_ characters: C, _ start: Int) -> UInt64 {
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

extension CSSColor {
    // MARK: - Named colors

    // Every character ORed with 0x20, as the bytes of a little-endian word. OR-ing 0x20 maps 'A'-'Z' to 'a'-'z',
    // keeps 'a'-'z', and maps every other byte (NUL, the rest of ASCII, Latin-1 0x80-0xFF, and the 0x80 that stands for
    // every non-ASCII UTF-16 code unit) to a nonzero byte that is not a letter.
    static var namedColorLowercaseBits: UInt64 { 0x2020_2020_2020_2020 }

    // CSSParserFastPaths::parseNamedColor: parseNamedColorInternal, finishParsingNamedColor and findColor().
    // The C++ gives nullopt for more than 63 characters or for a NUL or non-ASCII character, and otherwise looks the
    // ASCII-lowercased characters up, exactly, among the names of ColorData.gperf, which are 3 to 20 lowercase letters.
    // Here the characters, ORed with 0x20 (namedColorLowercaseBits), form the key words of CSSColorNamedColors.swift,
    // zero-filled. A key matches a name only if each of its first `count` bytes is the name's letter there (a non-letter
    // character can't become a letter) and the name has no letter beyond them (the key's bytes there are zero, and its
    // first `count` bytes are nonzero), so exactly the inputs the C++ accepts are found, with the same colors.
    static func parseNamedColor<C: CSSColorCharacters & ~Escapable>(_ characters: C) -> UInt64 {
        let count = characters.count
        guard count >= 3, count <= 20 else {
            return notAColor
        }

        var word0: UInt64 = 0
        var word1: UInt64 = 0
        var word2: UInt64 = 0
        if count >= 8 {
            word0 = characters.eightCharacters(at: 0) | namedColorLowercaseBits
            if count >= 16 {
                word1 = characters.eightCharacters(at: 8) | namedColorLowercaseBits
                if count > 16 {
                    // Characters [16, count), from the eight that end the string.
                    word2 = (characters.eightCharacters(at: count &- 8) | namedColorLowercaseBits) >> (UInt64(truncatingIfNeeded: 24 &- count) &* 8)
                }
            } else if count > 8 {
                // Characters [8, count), from the eight that end the string.
                word1 = (characters.eightCharacters(at: count &- 8) | namedColorLowercaseBits) >> (UInt64(truncatingIfNeeded: 16 &- count) &* 8)
            }
        } else {
            for index in 0..<count {
                word0 |= UInt64(characters[index] | 0x20) << (UInt64(truncatingIfNeeded: index) &* 8)
            }
        }

        // The table lookup (CSSColorNamedColors.swift). bucket < 64 and slot < 256, so the indices are in bounds.
        let hash = (word0 &+ word1 &* namedColorMultiplier1 &+ word2 &* namedColorMultiplier2) &* namedColorHashMultiplier
        let bucket = Int(truncatingIfNeeded: hash >> namedColorBucketShift)
        let slot = Int(truncatingIfNeeded: (hash >> 32) & 0xFF) ^ Int(namedColorDisplacements[bucket])
        let entry = slot &* 3
        let entryWord2 = namedColorEntries[entry &+ 2]
        guard namedColorEntries[entry] == word0, namedColorEntries[entry &+ 1] == word1, entryWord2 & 0xFFFF_FFFF == word2 else {
            return notAColor
        }
        // asSRGBA(PackedColor::ARGB { namedColor->ARGBValue }); the table holds the color as PackedColor::RGBA.
        return UInt64(1) << 32 | entryWord2 >> 32
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
