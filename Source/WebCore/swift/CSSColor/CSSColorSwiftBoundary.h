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

#pragma once

// C boundary of the CSSColor Swift kernels (Source/WebCore/swift/CSSColor). The symbols are internal to
// WebCore (hidden); only WebCore .cpp files may call them, never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Parses the `length` characters at `offset` within the characters described by `context` with
// WTF::charactersToDouble() semantics. Sets *ok as charactersToDouble() does.
typedef double (*WebCoreSwift_CSSColor_ParseNumber)(const void* context, size_t offset, size_t length, bool* ok);

// parseNumericColor<Latin1Character>: #hex, quirks-mode bare hex (quirksMode = isQuirksModeBehavior(mode)),
// rgb()/rgba() and hsl()/hsla(). Returns 0 for std::nullopt, otherwise (1ull << 32) | PackedColor::RGBA,
// i.e. r << 24 | g << 16 | b << 8 | a in the low 32 bits. Hue numbers are parsed by calling
// parseNumber(context, offset, length, &ok) with offset + length <= `length` of the characters.
uint64_t WebCoreSwift_CSSColor_parseNumericColor8(const uint8_t* characters, size_t length, bool quirksMode, const void* context, WebCoreSwift_CSSColor_ParseNumber parseNumber);

#ifdef __cplusplus
}
#endif
