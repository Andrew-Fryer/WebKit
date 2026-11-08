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

// String and template literals with escapes, from parser/Lexer.cpp: Lexer<T>::parseStringSlowCase<shouldBuildStrings>
// and Lexer<T>::parseTemplateLiteral (the body of scanTemplateString), with the parseComplexEscape, parseUnicodeEscape,
// shiftLineTerminator, record16/append16 and recordUnicodeCodePoint they use. Each statement mirrors the C++.
//
// Correspondence: `source` is the text from the lexer's m_code to m_codeEnd and `i` is m_code - start; current(i) is
// m_current (0 at the end, see Lexer::shift); atEnd() is i == count (a NUL inside the source is an ordinary character).
// The C++ appends the cooked value to m_buffer16 and the raw value of a template to m_bufferForRawTemplateString16;
// here they are written to `cooked` and `raw`, C++-owned scratch buffers. A buffer that is too small is not written
// past its end: its length keeps counting and the scan reports needsCapacity with the lengths it needs, and the glue
// calls again with larger buffers. On every return the lengths are those of m_buffer16 and
// m_bufferForRawTemplateString16 at the corresponding return of the C++ (the glue keeps partial values on errors).

enum LexerEscapedLiterals {
    // StringParseResult, plus needsCapacity.
    static let parsedSuccessfully: UInt32 = 0
    static let unterminated: UInt32 = 1
    static let cannotBeParsed: UInt32 = 2
    static let needsCapacity: UInt32 = 3

    // The m_lexErrorMessage values the C++ assigns (see LexerEscapedLiteralsSwiftBoundary.h); 0 leaves it unchanged.
    static let noMessage: UInt32 = 0
    static let hexEscapeMessage: UInt32 = 1 // "\\x can only be followed by a hex character sequence"
    static let unicodeEscapeMessage: UInt32 = 2 // "\\u can only be followed by a Unicode character sequence"
    static let strictNumericEscapeMessage: UInt32 = 3 // "The only valid numeric escape in strict mode is '\\0'"
    static let unterminatedStringMessage: UInt32 = 4 // "Unterminated string constant"
    static let unexpectedEOFMessage: UInt32 = 5 // "Unexpected EOF"

    struct Result {
        var status: UInt32 = 0
        var message: UInt32 = 0
        var end = 0 // m_code - start at return
        var cookedLength = 0
        var rawLength = 0
        var cookedFailed = false // parseCookedFailed (templates)
        var isTail = false // tokenData->isTail (templates)
    }

    // MARK: Character helpers (SourceCharacters.h, WTF ASCIICType.h, Lexer.cpp)

    @inline(__always)
    static func current<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ i: Int) -> UInt32 {
        i < source.count ? UInt32(source[i]) : 0
    }

    // isLineTerminator<T>: LS and PS only in a UTF-16 source.
    @inline(__always)
    static func isLineTerminator(_ character: UInt32, wide: Bool) -> Bool {
        character == 0x0D || character == 0x0A || (wide && (character & ~1) == 0x2028)
    }

    @inline(__always)
    static func isASCIIHexDigit(_ character: UInt32) -> Bool {
        (character >= 0x30 && character <= 0x39) || ((character | 0x20) >= 0x61 && (character | 0x20) <= 0x66)
    }

    // toASCIIHexValue, for an ASCII hex digit.
    @inline(__always)
    static func hexValue(_ character: UInt32) -> UInt32 {
        character < 0x41 ? character &- 0x30 : ((character &- 0x41) & 0xF) &+ 10
    }

    @inline(__always)
    static func isASCIIDigit(_ character: UInt32) -> Bool {
        character >= 0x30 && character <= 0x39
    }

    @inline(__always)
    static func isASCIIOctalDigit(_ character: UInt32) -> Bool {
        character >= 0x30 && character <= 0x37
    }

    // singleEscape: singleCharacterEscapeValuesForASCII[c] for c < 128, else 0. The table maps the printable ASCII
    // characters to themselves except b f n r t v (control characters) and 0-9 u x (0: more processing); the rest is 0.
    @inline(__always)
    static func singleEscape(_ character: UInt32) -> UInt32 {
        if character < 0x20 || character >= 0x7F {
            return 0
        }
        switch character {
        case 0x30...0x39, 0x75, 0x78: return 0 // 0-9 u x
        case 0x62: return 0x08 // b
        case 0x66: return 0x0C // f
        case 0x6E: return 0x0A // n
        case 0x72: return 0x0D // r
        case 0x74: return 0x09 // t
        case 0x76: return 0x0B // v
        default: return character
        }
    }

    // MARK: Skipping ordinary characters

    // The index of the first character at or after `start` that is `stop1`, `stop2` or `stop3` (ASCII), below 0xE, or
    // (UTF-16) LS / PS; `count` if there is none. These are the only characters for which the C++ loops do anything but
    // shift(): the closing quote or backquote, '\\', '$' (templates), and what characterNeedsLiteralSpecialHandling
    // lets through to the end / CR / LF / LS / PS checks. 16 bytes at a time.
    @inline(__always)
    static func skipOrdinary<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ start: Int, _ stop1: UInt8, _ stop2: UInt8, _ stop3: UInt8) -> Int {
        let count = source.count
        let bytes = source.bytes
        var i = start
        if Character.bitWidth == 8 {
            while i &+ 16 <= count {
                let v = bytes.load(fromByteOffset: i, as: SIMD16<UInt8>.self)
                let hit = (v .== SIMD16(repeating: stop1)) .| (v .== SIMD16(repeating: stop2)) .| (v .== SIMD16(repeating: stop3)) .| (v .< SIMD16(repeating: 0xE))
                if any(hit) {
                    let words = bitCast(SIMD16<UInt8>(repeating: 0).replacing(with: 0xFF, where: hit), to: SIMD2<UInt64>.self)
                    return i &+ (words[0] != 0 ? words[0].trailingZeroBitCount &>> 3 : 8 &+ (words[1].trailingZeroBitCount &>> 3))
                }
                i &+= 16
            }
        } else {
            while i &+ 8 <= count {
                let v = bytes.load(fromByteOffset: i &* 2, as: SIMD8<UInt16>.self)
                let hit = (v .== SIMD8(repeating: UInt16(stop1))) .| (v .== SIMD8(repeating: UInt16(stop2))) .| (v .== SIMD8(repeating: UInt16(stop3)))
                    .| (v .< SIMD8(repeating: 0xE)) .| ((v & SIMD8(repeating: 0xFFFE)) .== SIMD8(repeating: 0x2028))
                if any(hit) {
                    let words = bitCast(SIMD8<UInt16>(repeating: 0).replacing(with: 0xFFFF, where: hit), to: SIMD2<UInt64>.self)
                    return i &+ (words[0] != 0 ? words[0].trailingZeroBitCount &>> 4 : 4 &+ (words[1].trailingZeroBitCount &>> 4))
                }
                i &+= 8
            }
        }
        while i < count {
            let character = UInt32(source[i])
            if character == UInt32(stop1) || character == UInt32(stop2) || character == UInt32(stop3) || character < 0xE
                || (Character.bitWidth == 16 && (character & ~1) == 0x2028) {
                return i
            }
            i &+= 1
        }
        return count
    }

    // MARK: Output (record16 / append16)

    @inline(__always)
    static func record(_ unit: UInt32, _ buffer: inout MutableSpan<UInt16>, _ length: inout Int) {
        if length < buffer.count {
            buffer[length] = UInt16(truncatingIfNeeded: unit)
        }
        length &+= 1
    }

    // append16(std::span { source + from, source + to }) (the Latin-1 overload widens).
    @inline(__always)
    static func append<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ from: Int, _ to: Int, _ buffer: inout MutableSpan<UInt16>, _ length: inout Int) {
        let chunkLength = to &- from
        if chunkLength <= buffer.count &- length {
            copy(source, from, chunkLength, &buffer, length)
        }
        length &+= chunkLength
    }

    // buffer[at ..< at + n] = source[from ..< from + n], widening Latin-1. 16 source bytes per step; the last step
    // overlaps the previous one rather than running a scalar remainder (the chunks between escapes are short).
    @inline(__always)
    static func copy<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ from: Int, _ n: Int, _ buffer: inout MutableSpan<UInt16>, _ at: Int) {
        let bytes = source.bytes
        if Character.bitWidth == 8 {
            if n >= 16 {
                var output = buffer.mutableBytes
                var k = 0
                while true {
                    let step = min(k, n &- 16)
                    let v = bytes.load(fromByteOffset: from &+ step, as: SIMD16<UInt8>.self)
                    output.storeBytes(of: SIMD16<UInt16>(truncatingIfNeeded: v), toByteOffset: (at &+ step) &* 2, as: SIMD16<UInt16>.self)
                    if step == n &- 16 {
                        break
                    }
                    k &+= 16
                }
                return
            }
            if n >= 8 {
                var output = buffer.mutableBytes
                let first = bytes.load(fromByteOffset: from, as: SIMD8<UInt8>.self)
                let last = bytes.load(fromByteOffset: from &+ n &- 8, as: SIMD8<UInt8>.self)
                output.storeBytes(of: SIMD8<UInt16>(truncatingIfNeeded: first), toByteOffset: at &* 2, as: SIMD8<UInt16>.self)
                output.storeBytes(of: SIMD8<UInt16>(truncatingIfNeeded: last), toByteOffset: (at &+ n &- 8) &* 2, as: SIMD8<UInt16>.self)
                return
            }
        } else {
            if n >= 8 {
                var output = buffer.mutableBytes
                var k = 0
                while true {
                    let step = min(k, n &- 8)
                    let v = bytes.load(fromByteOffset: (from &+ step) &* 2, as: SIMD8<UInt16>.self)
                    output.storeBytes(of: v, toByteOffset: (at &+ step) &* 2, as: SIMD8<UInt16>.self)
                    if step == n &- 8 {
                        break
                    }
                    k &+= 8
                }
                return
            }
        }
        for k in 0..<n {
            buffer[at + k] = UInt16(source[from + k])
        }
    }

    // recordUnicodeCodePoint: U16_LEAD / U16_TRAIL for a supplementary code point.
    @inline(__always)
    static func recordUnicodeCodePoint(_ codePoint: UInt32, _ buffer: inout MutableSpan<UInt16>, _ length: inout Int) {
        if codePoint <= 0xFFFF {
            record(codePoint, &buffer, &length)
        } else {
            record((codePoint &>> 10) &+ 0xD7C0, &buffer, &length)
            record((codePoint & 0x3FF) | 0xDC00, &buffer, &length)
        }
    }

    // shiftLineTerminator: m_current is a line terminator; a CR LF pair is consumed as one.
    @inline(__always)
    static func shiftLineTerminator<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ i: inout Int) {
        let previous = current(source, i)
        i &+= 1
        if previous == 0x0D && current(source, i) == 0x0A {
            i &+= 1
        }
    }

    // MARK: parseUnicodeEscape

    // Returns the code point, or nil for ParsedUnicodeEscapeValue::Incomplete / Invalid (which parseComplexEscape,
    // its only caller here, does not distinguish). Consumes exactly what the C++ consumes.
    @inline(__always)
    static func parseUnicodeEscape<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ i: inout Int) -> UInt32? {
        if current(source, i) == 0x7B { // '{'
            i &+= 1
            var codePoint: UInt32 = 0
            repeat {
                let character = current(source, i)
                if !isASCIIHexDigit(character) {
                    return nil
                }
                codePoint = (codePoint &<< 4) | hexValue(character)
                if codePoint > 0x10FFFF { // UCHAR_MAX_VALUE
                    // NotCodePoint: consume the remaining hex digits.
                    i &+= 1
                    while isASCIIHexDigit(current(source, i)) {
                        i &+= 1
                    }
                    return nil
                }
                i &+= 1
            } while current(source, i) != 0x7D // '}'
            i &+= 1
            return codePoint
        }

        let character1 = current(source, i)
        let character2 = current(source, i &+ 1) // peek(1)
        let character3 = current(source, i &+ 2)
        let character4 = current(source, i &+ 3)
        if !isASCIIHexDigit(character1) || !isASCIIHexDigit(character2) || !isASCIIHexDigit(character3) || !isASCIIHexDigit(character4) {
            // NotEscapeSequence: consume the hex digits there are.
            while isASCIIHexDigit(current(source, i)) {
                i &+= 1
            }
            return nil
        }
        i &+= 4
        return (hexValue(character1) &<< 12) | (hexValue(character2) &<< 8) | (hexValue(character3) &<< 4) | hexValue(character4)
    }

    // MARK: parseComplexEscape

    // m_current (source[i]) is the character after the backslash: not a single-character escape, not a line terminator.
    // Returns (StringParseResult, message).
    @inline(__always)
    static func parseComplexEscape<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, _ i: inout Int, build: Bool, strictMode: Bool, _ cooked: inout MutableSpan<UInt16>, _ cookedLength: inout Int) -> (UInt32, UInt32) {
        let count = source.count
        let first = current(source, i)
        if first == 0x78 { // 'x'
            i &+= 1
            let high = current(source, i)
            if !isASCIIHexDigit(high) || !isASCIIHexDigit(current(source, i &+ 1)) {
                // NotEscapeSequence: x [HexDigit]
                if isASCIIHexDigit(high) {
                    i &+= 1
                }
                return (i == count ? unterminated : cannotBeParsed, hexEscapeMessage)
            }
            i &+= 1
            let low = current(source, i)
            if build {
                record((hexValue(high) &<< 4) | hexValue(low), &cooked, &cookedLength)
            }
            i &+= 1
            return (parsedSuccessfully, noMessage)
        }

        if first == 0x75 { // 'u'
            i &+= 1
            if let codePoint = parseUnicodeEscape(source, &i) {
                if build {
                    recordUnicodeCodePoint(codePoint, &cooked, &cookedLength)
                }
                return (parsedSuccessfully, noMessage)
            }
            return (i == count ? unterminated : cannotBeParsed, unicodeEscapeMessage)
        }

        if strictMode {
            if isASCIIDigit(first) {
                // The only valid numeric escape in strict mode is '\0', and this must not be followed by a decimal digit.
                i &+= 1
                if first != 0x30 || isASCIIDigit(current(source, i)) {
                    // NotEscapeSequence: 0 DecimalDigit, or DecimalDigit but not 0
                    if first == 0x30 {
                        i &+= 1
                    }
                    return (i == count ? unterminated : cannotBeParsed, strictNumericEscapeMessage)
                }
                if build {
                    record(0, &cooked, &cookedLength)
                }
                return (parsedSuccessfully, noMessage)
            }
        } else {
            if isASCIIOctalDigit(first) {
                // Octal character sequences
                i &+= 1
                let second = current(source, i)
                if isASCIIOctalDigit(second) {
                    // Two octal characters
                    i &+= 1
                    let third = current(source, i)
                    if first >= 0x30 && first <= 0x33 && isASCIIOctalDigit(third) {
                        if build {
                            record((first &- 0x30) &* 64 &+ (second &- 0x30) &* 8 &+ third &- 0x30, &cooked, &cookedLength)
                        }
                        i &+= 1
                    } else {
                        if build {
                            record((first &- 0x30) &* 8 &+ second &- 0x30, &cooked, &cookedLength)
                        }
                    }
                } else {
                    if build {
                        record(first &- 0x30, &cooked, &cookedLength)
                    }
                }
                return (parsedSuccessfully, noMessage)
            }
        }

        if i != count { // !atEnd()
            if build {
                record(first, &cooked, &cookedLength)
            }
            i &+= 1
            return (parsedSuccessfully, noMessage)
        }

        return (unterminated, unterminatedStringMessage)
    }

    // MARK: parseStringSlowCase

    // source[0] is the opening quote. On success `end` is the offset of the closing quote (not consumed).
    @inline(__always)
    static func scanString<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, build: Bool, strictMode: Bool, _ cooked: inout MutableSpan<UInt16>) -> Result {
        let wide = Character.bitWidth == 16
        let count = source.count
        var result = Result()
        var cookedLength = 0
        let stringQuoteCharacter = current(source, 0)
        let quote = UInt8(truncatingIfNeeded: stringQuoteCharacter) // ' or "
        var i = 1
        var stringStart = i

        while true {
            i = skipOrdinary(source, i, quote, 0x5C, quote)
            let character = current(source, i)
            if character == stringQuoteCharacter {
                break
            }
            if character == 0x5C { // '\\'
                if build && stringStart != i {
                    append(source, stringStart, i, &cooked, &cookedLength)
                }
                i &+= 1

                let next = current(source, i)
                let escape = singleEscape(next)

                // Most common escape sequences first
                if escape != 0 {
                    if build {
                        record(escape, &cooked, &cookedLength)
                    }
                    i &+= 1
                } else if isLineTerminator(next, wide: wide) {
                    shiftLineTerminator(source, &i)
                } else {
                    let (status, message) = parseComplexEscape(source, &i, build: build, strictMode: strictMode, &cooked, &cookedLength)
                    if status != parsedSuccessfully {
                        result.status = status
                        result.message = message
                        result.end = i
                        result.cookedLength = cookedLength
                        return finish(result, cooked.count, 0)
                    }
                }

                stringStart = i
                continue
            }
            // characterNeedsLiteralSpecialHandling(c) (c < 0xE, and for UTF-16 also 0x200E-0x3FFF etc.) filters for
            // the end, CR, LF, LS and PS: only those change the outcome below.
            if character < 0xE || (wide && (character & ~1) == 0x2028) {
                // New-line or end of input is not allowed
                if i == count || character == 0x0D || character == 0x0A {
                    result.status = i == count ? unterminated : cannotBeParsed
                    result.message = unexpectedEOFMessage
                    result.end = i
                    result.cookedLength = cookedLength
                    return finish(result, cooked.count, 0)
                }
                // An unescaped LS or PS stays part of the string's value and ends the line.
                if isLineTerminator(character, wide: wide) {
                    shiftLineTerminator(source, &i)
                    continue
                }
                // Anything else is just a normal character
            }
            i &+= 1
        }

        if build && i != stringStart {
            append(source, stringStart, i, &cooked, &cookedLength)
        }
        result.status = parsedSuccessfully
        result.end = i
        result.cookedLength = cookedLength
        return finish(result, cooked.count, 0)
    }

    // needsCapacity if either value did not fit.
    @inline(__always)
    static func finish(_ result: Result, _ cookedCapacity: Int, _ rawCapacity: Int) -> Result {
        var result = result
        if result.cookedLength > cookedCapacity || result.rawLength > rawCapacity {
            result.status = needsCapacity
        }
        return result
    }

    // MARK: parseTemplateLiteral

    // source[0] is the first character after the '`' or the '}'. On success `end` is after the closing '`' or '${'.
    @inline(__always)
    static func scanTemplate<Character: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ source: Span<Character>, buildRaw: Bool, _ cooked: inout MutableSpan<UInt16>, _ raw: inout MutableSpan<UInt16>) -> Result {
        let wide = Character.bitWidth == 16
        let count = source.count
        var result = Result()
        var cookedLength = 0
        var rawLength = 0
        var parseCookedFailed = false
        var i = 0
        var stringStart = i
        var rawStringStart = i

        while true {
            i = skipOrdinary(source, i, 0x60, 0x5C, 0x24) // '`' '\\' '$'
            let character = current(source, i)
            if character == 0x60 { // '`'
                break
            }
            if character == 0x5C { // '\\'
                if stringStart != i {
                    append(source, stringStart, i, &cooked, &cookedLength)
                }
                i &+= 1

                let next = current(source, i)
                let escape = singleEscape(next)

                // Most common escape sequences first.
                if escape != 0 {
                    record(escape, &cooked, &cookedLength)
                    i &+= 1
                } else if isLineTerminator(next, wide: wide) {
                    // Normalize <CR>, <CR><LF> to <LF>.
                    if next == 0x0D {
                        if buildRaw {
                            append(source, rawStringStart, i, &raw, &rawLength)
                            record(0x0A, &raw, &rawLength)
                        }
                        shiftLineTerminator(source, &i)
                        rawStringStart = i
                    } else {
                        shiftLineTerminator(source, &i)
                    }
                } else {
                    let (status, message) = parseComplexEscape(source, &i, build: true, strictMode: true, &cooked, &cookedLength)
                    if message != noMessage {
                        result.message = message
                    }
                    if status != parsedSuccessfully {
                        if buildRaw && status == cannotBeParsed {
                            parseCookedFailed = true
                        } else {
                            result.status = status
                            result.end = i
                            result.cookedLength = cookedLength
                            result.rawLength = rawLength
                            return finish(result, cooked.count, raw.count)
                        }
                    }
                }

                stringStart = i
                continue
            }

            if character == 0x24 && current(source, i &+ 1) == 0x7B { // '$' '{'
                break
            }

            // characterNeedsLiteralSpecialHandling: only the end, CR, LF, LS and PS change the outcome below.
            if character < 0xE || (wide && (character & ~1) == 0x2028) {
                // End of input is not allowed. Unlike String, line terminator is allowed.
                if i == count {
                    result.status = unterminated
                    result.message = unexpectedEOFMessage
                    result.end = i
                    result.cookedLength = cookedLength
                    result.rawLength = rawLength
                    return finish(result, cooked.count, raw.count)
                }

                if isLineTerminator(character, wide: wide) {
                    if character == 0x0D {
                        // Normalize <CR>, <CR><LF> to <LF>.
                        if stringStart != i {
                            append(source, stringStart, i, &cooked, &cookedLength)
                        }
                        if rawStringStart != i && buildRaw {
                            append(source, rawStringStart, i, &raw, &rawLength)
                        }

                        record(0x0A, &cooked, &cookedLength)
                        if buildRaw {
                            record(0x0A, &raw, &rawLength)
                        }
                        shiftLineTerminator(source, &i)
                        stringStart = i
                        rawStringStart = i
                    } else {
                        shiftLineTerminator(source, &i)
                    }
                    continue
                }
                // Anything else is just a normal character
            }

            i &+= 1
        }

        let isTail = current(source, i) == 0x60

        if i != stringStart {
            append(source, stringStart, i, &cooked, &cookedLength)
        }
        if rawStringStart != i && buildRaw {
            append(source, rawStringStart, i, &raw, &rawLength)
        }

        result.status = parsedSuccessfully
        result.cookedFailed = parseCookedFailed
        result.isTail = isTail
        // Skip ` or ${.
        result.end = isTail ? i &+ 1 : i &+ 2
        result.cookedLength = cookedLength
        result.rawLength = rawLength
        return finish(result, cooked.count, raw.count)
    }
}
