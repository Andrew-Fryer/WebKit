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

// C boundary of the CSSKeywords Swift kernels (Source/WebCore/swift/CSSKeywords): cssValueKeywordID(StringView) of
// css/parser/CSSPropertyParser.cpp, including the lookup that the generated gperf code (findCSSValueKeyword) does in
// C++. The symbols are internal to WebCore (hidden); only WebCore .cpp files may call them, never header-inline code.
//
// The keyword table is owned by C++ and built once: `pool` holds the lowercase names of all CSSValueIDs back to back
// (nameLiteralForSerialization), the name of id i is pool[offsets[i] ..< offsets[i + 1]] (offsetCount =
// numCSSValueKeywords + 1, at most 2049), and `slots` is 4096 uint16_t slots that buildTable() fills.

#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_CSS_KEYWORDS_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_CSS_KEYWORDS_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// The slot count, and the longest keyword the lookup supports (maxCSSValueKeywordLength must be below it).
#define WEBCORE_SWIFT_CSS_KEYWORDS_SLOT_COUNT 4096
#define WEBCORE_SWIFT_CSS_KEYWORDS_MAXIMUM_LENGTH 63

// Fills `slots` from the names.
void WebCoreSwift_CSSKeywords_buildTable(const uint8_t* pool, size_t poolSize, const uint32_t* offsets, size_t offsetCount, uint16_t* slots, size_t slotCount);

// cssValueKeywordID(StringView) for 1 <= length characters: Latin-1 (lookup8) or UTF-16 code units, any alignment
// (lookup16). Returns the CSSValueID, 0 (CSSValueID::Invalid) when there is no keyword.
WEBCORE_SWIFT_CSS_KEYWORDS_NODELETE uint16_t WebCoreSwift_CSSKeywords_lookup8(const uint8_t* characters, size_t length, const uint8_t* pool, size_t poolSize, const uint32_t* offsets, size_t offsetCount, const uint16_t* slots, size_t slotCount);
WEBCORE_SWIFT_CSS_KEYWORDS_NODELETE uint16_t WebCoreSwift_CSSKeywords_lookup16(const void* characters, size_t length, const uint8_t* pool, size_t poolSize, const uint32_t* offsets, size_t offsetCount, const uint16_t* slots, size_t slotCount);

#ifdef __cplusplus
}
#endif
