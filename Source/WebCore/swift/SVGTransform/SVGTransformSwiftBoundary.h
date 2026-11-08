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

#ifdef __cplusplus
extern "C" {
#endif

// Lexes SVG transform-list items from the start of characters[0..<length] (length > 0) until one does
// not parse, the input ends, or `capacity` (> 0) items were lexed. An item is: leading whitespace, the
// transform keyword, its parenthesized parameter list, then the whitespace, optional ',' and whitespace
// that SVGTransformList::parseGeneric skips after each item. Returns the number of items lexed (>= 1).
// results[k] holds item k's WebCoreSwift_SVGTransform_status() and, for InvalidParameters and Parsed,
// its SVGTransformValue::SVGTransformType; for Parsed also its number of values, written to
// values[6 * k ..< 6 * k + count], and whether a ',' followed it. *consumed receives how far the C++
// lexer would have advanced the StringParsingBuffer after the last lexed item, also when that item
// failed (callers may keep parsing after a failure).
size_t WebCoreSwift_SVGTransform_lexItems8(const uint8_t* characters, size_t length, uint32_t* results, float* values /* [6 * capacity] */, size_t capacity, size_t* consumed);

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
