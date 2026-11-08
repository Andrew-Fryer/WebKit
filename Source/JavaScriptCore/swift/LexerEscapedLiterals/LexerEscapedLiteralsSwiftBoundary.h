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

// String and template literals with escapes (Lexer<T>::parseStringSlowCase and the parseTemplateLiteral body of
// Lexer<T>::scanTemplateString, parser/Lexer.cpp) implemented in Swift (LexerEscapedLiterals.swift). The symbols are
// hidden in JavaScriptCore: only parser/Lexer.cpp calls them.
//
// `source[0 ..< length]` is the text from m_code to m_codeEnd (for a string, source[0] is the opening quote; for a
// template, the character after '`' or '}'); it may be null only if length is 0, and UTF-16 sources must be 2-byte
// aligned. The cooked value is written to cooked[0 ..< cookedCapacity] and a template's raw value (only if buildRaw)
// to raw[0 ..< rawCapacity], in UTF-16 code units, as the C++ appends them to m_buffer16 and
// m_bufferForRawTemplateString16. results[4]:
//   [0] status | message << 8 | cookedFailed << 16 | isTail << 17
//       status: 0 StringParsedSuccessfully, 1 StringUnterminated, 2 StringCannotBeParsed,
//               3 a buffer was too small: nothing else is valid except [2] and [3], call again with buffers at least
//                 that large.
//       message: the m_lexErrorMessage the C++ assigns (also on success, for a template whose cooked value failed):
//               0 none, 1 "\\x can only be followed by a hex character sequence",
//               2 "\\u can only be followed by a Unicode character sequence",
//               3 "The only valid numeric escape in strict mode is '\\0'", 4 "Unterminated string constant",
//               5 "Unexpected EOF".
//       cookedFailed: parseCookedFailed (tokenData->cooked = nullptr); isTail: tokenData->isTail.
//   [1] offset of m_code at return (a string: the closing quote, not consumed; a template: after '`' or '${').
//   [2] cooked length, [3] raw length: the lengths of m_buffer16 / m_bufferForRawTemplateString16 at return.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define JSCSwift_LexerEscapedLiterals_ResultCount 4
#define JSCSwift_LexerEscapedLiterals_NeedsCapacity 3

__attribute__((nothrow)) void JSCSwift_LexerEscapedLiterals_string8(const uint8_t* source, size_t length, bool build, bool strictMode, char16_t* cooked, size_t cookedCapacity, size_t* results);
__attribute__((nothrow)) void JSCSwift_LexerEscapedLiterals_string16(const char16_t* source, size_t length, bool build, bool strictMode, char16_t* cooked, size_t cookedCapacity, size_t* results);
__attribute__((nothrow)) void JSCSwift_LexerEscapedLiterals_template8(const uint8_t* source, size_t length, bool buildRaw, char16_t* cooked, size_t cookedCapacity, char16_t* raw, size_t rawCapacity, size_t* results);
__attribute__((nothrow)) void JSCSwift_LexerEscapedLiterals_template16(const char16_t* source, size_t length, bool buildRaw, char16_t* cooked, size_t cookedCapacity, char16_t* raw, size_t rawCapacity, size_t* results);

#ifdef __cplusplus
}
#endif
