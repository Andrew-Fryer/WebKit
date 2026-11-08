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

// C boundary of the TextRunScanners Swift kernels (Source/WebCore/swift/TextRunScanners). The symbols are internal to
// WebCore (hidden); only platform/graphics/FontCascade.cpp calls them, never header-inline code. Pointers may be null
// only when their count is 0; char16_t pointers must be 2-byte aligned.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// FontCascade::characterRangeCodePath(std::span<const char16_t>): returns the FontCascadeCodePath raw value
// (0 Simple, 1 Complex, 2 SimpleWithGlyphOverflow).
uint8_t WebCoreSwift_TextRunScanners_characterRangeCodePath(const char16_t* characters, size_t length);

// Scans characters[0 ..< length] with FontCascade::canUseSimplifiedTextMeasuring(character, FontVariant::Auto,
// whitespaceIsCollapsed, primaryFont) as far as `cache` (m_canUseSimplifiedTextMeasuringForAutoVariantCache.storage(),
// cacheWordCount = 8 64-bit words) answers it. Returns index << 1 | isFalse: if isFalse, characters[index] answers false;
// otherwise index == length (every character answered true) or characters[index] needs the C++ function (not cached,
// or, in the 16-bit scan, not Latin-1).
size_t WebCoreSwift_TextRunScanners_scanSimplifiedMeasuring8(const uint8_t* characters, size_t length, const uint64_t* cache, size_t cacheWordCount, bool whitespaceIsCollapsed);
size_t WebCoreSwift_TextRunScanners_scanSimplifiedMeasuring16(const char16_t* characters, size_t length, const uint64_t* cache, size_t cacheWordCount, bool whitespaceIsCollapsed);

#ifdef __cplusplus
}
#endif
