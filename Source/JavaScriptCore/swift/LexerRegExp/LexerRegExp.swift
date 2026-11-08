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

// The scanning part of Lexer<T>::scanRegExp (parser/Lexer.cpp): the body of a regular expression literal up to its
// closing '/', then its flags. The C++ glue (Lexer<T>::scanRegExp) turns the offsets into the pattern and flags
// identifiers and the error tokens, exactly where the original builds them. The source is the text from the
// character after the opening '/' (m_code) to m_codeEnd; offsets are relative to it.
//
// Correspondence with the C++: index i is m_code - start, and m_current is source[i] when i < count and 0 at the end
// (Lexer::shift). atEnd() is i == count: a NUL inside the source is an ordinary character.

enum LexerRegExp {
    enum Status: UInt32 {
        case ok = 0
        case unterminated = 1 // UNTERMINATED_REGEXP_LITERAL_ERRORTOK at `end`.
        case invalidFlagCharacter = 2 // INVALID_IDENTIFIER_UNICODE_ERRORTOK at `end` (UTF-16 only).
    }

    struct Result {
        var status: Status
        var bodyEnd: Int // Offset of the closing '/'; the pattern is source[0 ..< bodyEnd].
        var end: Int // m_code at return: after the flags, or at the error.
        var charactersOredTogether: UInt16 // OR of the pattern's characters (UTF-16; 0 for Latin-1, as orCharacter<Latin1Character>).
    }

    // isLineTerminator<T> (SourceCharacters.h): LS and PS only in a UTF-16 source.
    @inline(__always)
    static func isLineTerminator(_ character: UInt32, wide: Bool) -> Bool {
        character == 0x0D || character == 0x0A || (wide && (character & ~1) == 0x2028)
    }

    // isIdentPart(Latin1Character): typesOfLatin1Characters[c] <= CharacterOtherIdentifierPart, i.e. '$', '0'-'9',
    // 'A'-'Z', '_', 'a'-'z', U+00AA, U+00B5, U+00B7, U+00BA, U+00C0-U+00FF except U+00D7 and U+00F7.
    @inline(__always)
    static func isLatin1IdentPart(_ character: UInt32) -> Bool {
        let word: UInt64
        switch character &>> 6 {
        case 0: word = 0x03ff_0010_0000_0000
        case 1: word = 0x07ff_fffe_87ff_fffe
        case 2: word = 0x04a0_0400_0000_0000
        default: word = 0xff7f_ffff_ff7f_ffff
        }
        return (word &>> UInt64(character & 63)) & 1 != 0
    }

    // isNonLatin1WhiteSpace (generated LexerUnicodePropertyTables.h): non-Latin-1 Zs characters and the BOM.
    @inline(__always)
    static func isNonLatin1WhiteSpace(_ character: UInt32) -> Bool {
        character == 0xFEFF || character == 0x1680 || (character >= 0x2000 && character <= 0x200A)
            || character == 0x202F || character == 0x205F || character == 0x3000
    }

    @inline(__always)
    static func scan<Character: FixedWidthInteger & UnsignedInteger>(_ source: Span<Character>) -> Result {
        let wide = Character.bitWidth == 16
        let count = source.count
        var lastWasEscape = false
        var inBrackets = false
        var charactersOredTogether: UInt32 = 0
        var i = 0

        // while (true): stop at a line terminator or the end, else shift; an unescaped '/' outside brackets ends the body.
        while true {
            if i == count {
                return Result(status: .unterminated, bodyEnd: 0, end: i, charactersOredTogether: 0)
            }
            let previous = UInt32(source[i])
            if isLineTerminator(previous, wide: wide) {
                return Result(status: .unterminated, bodyEnd: 0, end: i, charactersOredTogether: 0)
            }
            i &+= 1

            if previous == 0x2F && !lastWasEscape && !inBrackets { // '/'
                break
            }

            charactersOredTogether |= previous

            if lastWasEscape {
                lastWasEscape = false
                continue
            }

            switch previous {
            case 0x5B: // '['
                inBrackets = true
            case 0x5D: // ']'
                inBrackets = false
            case 0x5C: // '\\'
                lastWasEscape = true
            default:
                break
            }
        }
        let bodyEnd = i &- 1

        // Flags: while (isLatin1(m_current) && isIdentPart(m_current)). The 0 of the end is Latin-1 and not an IdentPart.
        while i < count {
            let character = UInt32(source[i])
            if character > 0xFF || !isLatin1IdentPart(character) {
                break
            }
            i &+= 1
        }

        // !isLatin1(m_current) && !isWhiteSpace(m_current) && !isLineTerminator(m_current): only a UTF-16 source has a
        // non-Latin-1 m_current, and for it isWhiteSpace is isNonLatin1WhiteSpace and isLineTerminator is LS/PS.
        var status = Status.ok
        if wide && i < count {
            let character = UInt32(source[i])
            if character > 0xFF && !isNonLatin1WhiteSpace(character) && !isLineTerminator(character, wide: true) {
                status = .invalidFlagCharacter
            }
        }
        return Result(status: status, bodyEnd: bodyEnd, end: i, charactersOredTogether: wide ? UInt16(truncatingIfNeeded: charactersOredTogether) : 0)
    }
}
