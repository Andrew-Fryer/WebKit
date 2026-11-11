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

// Swift implementation: Source/WebCore/swift/SVGTransform/SVGTransformLexer.swift (hidden in WebCore).
// Only include from WebCore .cpp files.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_SVG_TRANSFORM_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_SVG_TRANSFORM_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Lexes SVG transform-list items from the start of characters[0..<length] (Latin-1 code units, or UTF-16 code units
// at any alignment if is16Bit) until one does not parse, the input ends, or resultCapacity (1...255) items were lexed.
// An item is: leading whitespace, the transform keyword, its parenthesized parameter list, then the whitespace,
// optional ',' and whitespace that SVGTransformList::parseGeneric skips after each item. Lexes at least one item.
// results[k] holds item k's WebCoreSwift_SVGTransform_status() and, for InvalidParameters and Parsed, its
// SVGTransformValue::SVGTransformType; for Parsed also its number of values, written to values[6 * k ..< 6 * k + count]
// (values has valueCapacity floats; too few traps), and whether a ',' followed it. Returns the number of items lexed
// and how far the cursor advanced after the last of them, also when it failed (callers may keep parsing after a
// failure); see WebCoreSwift_SVGTransform_itemCount() and WebCoreSwift_SVGTransform_consumed().
WEBCORE_SWIFT_SVG_TRANSFORM_NODELETE uint64_t WebCoreSwift_SVGTransform_lexItems(const void* characters, size_t length, bool is16Bit, uint32_t* results, size_t resultCapacity, float* values, size_t valueCapacity);

static inline size_t WebCoreSwift_SVGTransform_itemCount(uint64_t lexed) { return lexed & 0xff; }
static inline size_t WebCoreSwift_SVGTransform_consumed(uint64_t lexed) { return lexed >> 8; }

enum {
    WebCoreSwift_SVGTransform_NoTransformType = 0, // No transform keyword at the cursor.
    WebCoreSwift_SVGTransform_InvalidParameters = 1, // Keyword lexed, parameter list invalid.
    WebCoreSwift_SVGTransform_Parsed = 2,
    WebCoreSwift_SVGTransform_OnlyWhitespace = 3, // Nothing but whitespace remained.
};

static inline unsigned WebCoreSwift_SVGTransform_status(uint32_t result) { return result & 0xff; }
static inline unsigned WebCoreSwift_SVGTransform_type(uint32_t result) { return (result >> 8) & 0xff; }
static inline unsigned WebCoreSwift_SVGTransform_valueCount(uint32_t result) { return (result >> 16) & 0xff; }
static inline bool WebCoreSwift_SVGTransform_followedByComma(uint32_t result) { return (result >> 24) & 1; }

#ifdef __cplusplus
}
#endif
