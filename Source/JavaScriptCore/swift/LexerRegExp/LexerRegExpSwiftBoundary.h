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

// The scan of a regular expression literal's body and flags (Lexer<T>::scanRegExp, parser/Lexer.cpp) implemented in
// Swift (LexerRegExp.swift). `characters[0 ..< length]` is the source from the character after the opening '/'
// (m_code) to m_codeEnd; it may be null only if length is 0, and UTF-16 characters must be 2-byte aligned. On return
// *end is the offset of m_code where the C++ scan stops (after the flags, or at the error character), *bodyEnd the
// offset of the closing '/' (only meaningful when the status is not 1). Status (low 16 bits of the result):
// 0 = REGEXP, 1 = UNTERMINATED_REGEXP_LITERAL_ERRORTOK, 2 = INVALID_IDENTIFIER_UNICODE_ERRORTOK (UTF-16 only).
// scan16 also returns, in the high 16 bits, the OR of the pattern's characters (charactersOredTogether).
// The symbols are hidden in JavaScriptCore: only parser/Lexer.cpp calls them.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((nothrow)) uint32_t JSCSwift_LexerRegExp_scan8(const uint8_t* characters, size_t length, size_t* bodyEnd, size_t* end);
__attribute__((nothrow)) uint32_t JSCSwift_LexerRegExp_scan16(const char16_t* characters, size_t length, size_t* bodyEnd, size_t* end);

#ifdef __cplusplus
}
#endif
