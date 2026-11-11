/*
 * Copyright (C) 2026 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

// The literal scans of the JavaScript lexer, implemented in Swift (swift/Lexer): the body and flags of a regular
// expression literal (Lexer<T>::scanRegExp), string literals with escapes (Lexer<T>::parseStringSlowCase) and template
// literal parts (Lexer<T>::scanTemplateString), all in parser/Lexer.cpp. The symbol is hidden in JavaScriptCore: only
// parser/Lexer.cpp calls it.
//
// `source[0 ..< length]` is the text from m_code to m_codeEnd, Latin-1 bytes or (with JSCSwift_Lexer_Wide) UTF-16 code
// units at any alignment; it may be null only if length is 0. For a regular expression, source[0] is the character
// after the opening '/'; for a string, the opening quote; for a template, the character after '`' or '}'. A string's
// cooked value is written to cooked[0 ..< cookedCapacity], a template's cooked and (only with JSCSwift_Lexer_Build)
// raw values to cooked and raw, in UTF-16 code units, as the original appended them to m_buffer16 and
// m_bufferForRawTemplateString16. Swift never writes past a buffer's capacity. results[JSCSwift_Lexer_ResultCount]:
//   regular expression:
//     [0] status | charactersOredTogether << 16 (the OR of the pattern's code units; 0 for Latin-1)
//         status: 0 REGEXP, 1 UNTERMINATED_REGEXP_LITERAL_ERRORTOK, 2 INVALID_IDENTIFIER_UNICODE_ERRORTOK (UTF-16 only)
//     [1] offset of m_code at return: after the flags, or at the error character; at most length
//     [2] offset of the closing '/' (when the status is not 1): the pattern is source[0 ..< [2]], the flags
//         source[[2] + 1 ..< [1]]
//   string or template:
//     [0] status | message << 8 | cookedFailed << 16 | isTail << 17
//         status: 0 StringParsedSuccessfully, 1 StringUnterminated, 2 StringCannotBeParsed,
//                 3 (JSCSwift_Lexer_NeedsCapacity) a buffer was too small: nothing else is valid except [2] and [3];
//                   call again with buffers at least that large.
//         message: the m_lexErrorMessage the C++ assigns (also on success, for a template whose cooked value failed):
//                 0 none, 1 "\\x can only be followed by a hex character sequence",
//                 2 "\\u can only be followed by a Unicode character sequence",
//                 3 "The only valid numeric escape in strict mode is '\\0'", 4 "Unterminated string constant",
//                 5 "Unexpected EOF".
//         cookedFailed: parseCookedFailed (tokenData->cooked = nullptr); isTail: tokenData->isTail.
//     [1] offset of m_code at return (a string: the closing quote, not consumed; a template: after '`' or '${')
//     [2] cooked length, [3] raw length: the lengths of m_buffer16 / m_bufferForRawTemplateString16 at return.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    // The scan (mode & 3).
    JSCSwift_Lexer_RegExp = 0,
    JSCSwift_Lexer_String = 1,
    JSCSwift_Lexer_Template = 2,
    // UTF-16 source (else Latin-1).
    JSCSwift_Lexer_Wide = 4,
    // String: shouldBuildStrings. Template: RawStringsBuildMode::BuildRawStrings.
    JSCSwift_Lexer_Build = 8,
    // String: strict mode.
    JSCSwift_Lexer_Strict = 16,

    JSCSwift_Lexer_ResultCount = 4,
    JSCSwift_Lexer_NeedsCapacity = 3,
};

__attribute__((nothrow)) void JSCSwift_Lexer_scan(uint32_t mode, const void* source, size_t length, char16_t* cooked, size_t cookedCapacity, char16_t* raw, size_t rawCapacity, size_t* results);

#ifdef __cplusplus
}
#endif
