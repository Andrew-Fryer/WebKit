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

// Swift implementation: Source/WebCore/swift/SVGNumberGrammar/SVGNumberGrammar.swift (hidden in WebCore): the SVG
// <number> grammar. Only include from WebCore .cpp files.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_NODELETE
#endif

#ifdef __cplusplus
#define WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_CONSTEXPR constexpr
#else
#define WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_CONSTEXPR
#endif

#ifdef __cplusplus
extern "C" {
#endif

// The kinds of item a program parses.
enum {
    WebCoreSwift_SVGNumberGrammar_Number = 0, // A number, then the spaces and ',' after it (SuffixSkippingPolicy::Skip).
    WebCoreSwift_SVGNumberGrammar_NumberWithoutSuffix = 1, // Just a number (SuffixSkippingPolicy::DontSkip).
    WebCoreSwift_SVGNumberGrammar_ArcFlag = 2, // '0' or '1', then the spaces and ',' after it; parsed as 0 or 1.
};

// A program: bits 0-3 the number of items (at most 8), bits 4 + 2k and 5 + 2k item k's kind.
// WebCoreSwift_SVGNumberGrammar_numbers(count): `count` items of kind WebCoreSwift_SVGNumberGrammar_Number.
static inline WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_CONSTEXPR uint32_t WebCoreSwift_SVGNumberGrammar_numbers(unsigned count) { return count; }
static inline WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_CONSTEXPR uint32_t WebCoreSwift_SVGNumberGrammar_withItemKind(uint32_t program, unsigned item, unsigned kind) { return program | kind << (4 + 2 * item); }

// Parses the items of `program` from the start of characters[0..<length] (Latin-1 code units, or UTF-16 code units at
// any alignment if is16Bit) until one does not parse or all were parsed. Item k's value is written to values[k]
// (valueCapacity must be at least the item count; too few traps). Returns the number of items parsed and how far the
// cursor advanced, also after a failure part way through an item (where the C++ StringParsingBuffer would be); see
// WebCoreSwift_SVGNumberGrammar_parsedCount() and WebCoreSwift_SVGNumberGrammar_consumed().
WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_NODELETE uint64_t WebCoreSwift_SVGNumberGrammar_parseNumbers(const void* characters, size_t length, bool is16Bit, uint32_t program, float* values, size_t valueCapacity);

static inline size_t WebCoreSwift_SVGNumberGrammar_parsedCount(uint64_t parsed) { return parsed & 0xff; }
static inline size_t WebCoreSwift_SVGNumberGrammar_consumed(uint64_t parsed) { return parsed >> 8; }

// One item of kind WebCoreSwift_SVGNumberGrammar_Number (skipSuffix) or WebCoreSwift_SVGNumberGrammar_NumberWithoutSuffix
// from the start of characters[0..<length]. On success writes *value. Returns whether it parsed and how far the cursor
// advanced (also on failure); see WebCoreSwift_SVGNumberGrammar_parsedNumber() and WebCoreSwift_SVGNumberGrammar_numberConsumed().
WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_NODELETE uint64_t WebCoreSwift_SVGNumberGrammar_parseNumber(const void* characters, size_t length, bool is16Bit, bool skipSuffix, float* value);

static inline bool WebCoreSwift_SVGNumberGrammar_parsedNumber(uint64_t parsed) { return parsed & 1; }
static inline size_t WebCoreSwift_SVGNumberGrammar_numberConsumed(uint64_t parsed) { return parsed >> 1; }

// The kinds of list item WebCoreSwift_SVGNumberGrammar_scanListItem() finds.
enum {
    WebCoreSwift_SVGNumberGrammar_LengthListItem = 0, // SVGLengthList::parse: up to a space or ','.
    WebCoreSwift_SVGNumberGrammar_GlyphNameItem = 1, // parseGlyphName: up to ',', without trailing spaces.
};

// One list item of `kind` from the start of characters[0..<length] and the spaces / ',' / spaces after it. Writes the
// item's length to *itemLength and returns how far to advance past the item and its delimiter; 0 if there is no item
// (the list parser stops there).
WEBCORE_SWIFT_SVG_NUMBER_GRAMMAR_NODELETE size_t WebCoreSwift_SVGNumberGrammar_scanListItem(const void* characters, size_t length, bool is16Bit, uint32_t kind, size_t* itemLength);

#ifdef __cplusplus
}
#endif
