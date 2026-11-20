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

// C boundary of the MatchResultCacheDiff Swift kernel (Source/WebCore/swift/MatchResultCacheDiff), called by
// Style::MatchResultCache::resultWithCurrentInlineStyle() in style/MatchResultCache.cpp. The symbol is internal to
// WebCore (hidden); only WebCore .cpp files may call it, never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The kernel's results.
enum {
    WebCoreSwiftMatchResultCacheDiffUnusable = 0, // isUsableAfterInlineStyleChange() is false: drop the entry.
    WebCoreSwiftMatchResultCacheDiffAllNormalProperties = 1, // PropertyCascade::normalProperties().
    WebCoreSwiftMatchResultCacheDiffChangedIDs = 2, // IncludedProperties { .ids = out[0 ..< state[2]] } (no ids: unchanged).
};

// isUsableAfterInlineStyleChange() and then computeAndUpdateChangedProperties() of one cache entry, in one pass.
//
// state (stateCount = 4): in, [0] the address of Entry::inlineStyle, [1] the address of the element's current inline
//   style (0 if none), [2] firstLowPriorityProperty | CSSPropertyID::LineHeight << 16 | CSSPropertyID::Custom << 32;
//   out, [2] the number of ids and [3] the number of null indices.
// original: Entry::originalInlineProperties, 2 words per OriginalInlineProperty: word 0 = propertyID (bits 0-15) |
//   wasImportant (bits 16-23, 0 or 1), the other bits padding; word 1 = the valueIfUnchanged address (0 if null).
// current: the CSSProperty records of Entry::inlineStyle (MutableStyleProperties::propertySpan()), 2 words each:
//   word 0 = StylePropertyMetadata (m_propertyID bits 0-9, m_important bit 13), bits 32-63 padding; word 1 = the
//   value address.
// aliases[cssPropertyIDEnumValueCount]: cascadeAliasProperty(id).
// out (outCount >= 2 * the original property count n): out[0 ..< state[2]] = the ids (cascade aliases) of the changed
//   low-priority properties, in order; out[n ..< n + state[3]] = the indices, in increasing order, of the changed
//   properties whose valueIfUnchanged is not null yet. The caller nulls those (computeAndUpdateChangedProperties'
//   update) whatever the result.
//
// The kernel compares the value addresses only as integers; it never dereferences them.
uint32_t WebCoreSwift_MatchResultCacheDiff_changedProperties(uint64_t* state, size_t stateCount,
    const uint64_t* original, size_t originalWordCount, const uint64_t* current, size_t currentWordCount,
    const uint16_t* aliases, size_t aliasCount, uint32_t* out, size_t outCount);

#ifdef __cplusplus
}
#endif
