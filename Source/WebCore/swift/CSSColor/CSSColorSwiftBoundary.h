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

// C boundary of the CSSColor Swift kernel (Source/WebCore/swift/CSSColor). The symbol is internal to
// WebCore (hidden); only WebCore .cpp files may call it, never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WebCoreSwift_CSSColor_Numeric = 0, // parseNumericColor: #hex, rgb()/rgba() and hsl()/hsla().
    WebCoreSwift_CSSColor_NumericQuirks = 1, // The same, plus bare hex (isQuirksModeBehavior(mode)).
    WebCoreSwift_CSSColor_Hex = 2, // CSSParserFastPaths::parseHexColor: 3, 4, 6 or 8 hex digits, no '#'.
};

// Parses `length` Latin-1 characters (is8Bit) or UTF-16 code units in any alignment as `mode` (one of the values
// above; anything else traps). Returns 0 for std::nullopt, otherwise (1ull << 32) | PackedColor::RGBA, i.e.
// r << 24 | g << 16 | b << 8 | a in the low 32 bits.
uint64_t WebCoreSwift_CSSColor_parseColor(const void* characters, size_t length, bool is8Bit, uint8_t mode);

#ifdef __cplusplus
}
#endif
