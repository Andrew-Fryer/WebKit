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

// C boundary of the CSSTokenizer Swift kernel (Source/WebCore/swift/CSSTokenizer), called by css/parser/CSSTokenizer.cpp
// and CSSParserToken.cpp. The symbols are internal to WebCore (hidden); only WebCore .cpp files may call them, never
// header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#include <uchar.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// The words of `state` (WebCoreSwiftCSSTokenizerStateCount of them).
enum {
    WebCoreSwiftCSSTokenizerPosition = 0, // in/out: the input offset to resume at (m_input's m_offset; may exceed the length). Done: the final offset() (the EOF token's start).
    WebCoreSwiftCSSTokenizerBlockDepth = 1, // in/out: the number of block stack entries.
    WebCoreSwiftCSSTokenizerPoolUsed = 2, // in/out: pool code units in use.
    WebCoreSwiftCSSTokenizerRegistrationCount = 3, // in/out: registrations made.
    WebCoreSwiftCSSTokenizerBlockStack = 4, // the address of the block stack storage (uint8_t; a CSSParserTokenType per entry).
    WebCoreSwiftCSSTokenizerBlockStackCapacity = 5,
    WebCoreSwiftCSSTokenizerPool = 6, // the address of the pool (char16_t), or 0.
    WebCoreSwiftCSSTokenizerPoolCapacity = 7,
    WebCoreSwiftCSSTokenizerRegistrations = 8, // the address of the registrations (uint64_t), or 0.
    WebCoreSwiftCSSTokenizerRegistrationCapacity = 9,
    WebCoreSwiftCSSTokenizerOffsets = 10, // observer mode: the address of a uint32_t per token record (its start offset), else 0.
    WebCoreSwiftCSSTokenizerFlags = 11, // WebCoreSwiftCSSTokenizerFlag* bits.
    WebCoreSwiftCSSTokenizerStateCount = 12,
};

enum {
    WebCoreSwiftCSSTokenizerFlagPreprocessed = 1, // the input is preprocessString()'s result: don't check it again.
    WebCoreSwiftCSSTokenizerFlagObserving = 2, // CSSParserObserverWrapper mode: comments get records (CommentToken) too.
};

// The status in the high 32 bits of WebCoreSwift_CSSTokenizer_tokenize's result.
enum {
    WebCoreSwiftCSSTokenizerDone = 0, // reached the end of the input.
    WebCoreSwiftCSSTokenizerTokensFull = 1, // the record chunk is full: consume it and call again.
    WebCoreSwiftCSSTokenizerNeedBlockStack = 2, // the block stack is full: give a larger one (same contents) and call again.
    WebCoreSwiftCSSTokenizerNeedPool = 3, // a token needs the pool: give a pool of at least length + 2 code units with the
                                          // same contents and room for at least 2 more registrations, then call again.
    WebCoreSwiftCSSTokenizerNeedPreprocessing = 4, // (first call only) the input has a NUL or an unpaired surrogate:
                                                   // tokenize WebCoreSwift_CSSTokenizer_preprocess's result instead.
};

// Tokenizes `characters` (length code units; Latin-1 when is8Bit, else UTF-16 at any alignment) from the state's
// position (CSSTokenizer's constructor loop over nextToken()). Writes up to tokenCapacity 24-byte token records to
// `tokens` (CSSTokenizer's token storage) and returns their count | status << 32. A record is a CSSParserToken in its
// own layout (see CSSTokenizer.swift), m_valueDataCharRaw = characters + offset, except that bit 31 of word 0 marks a
// value held in the pool, whose word 1 is the registration index. A registration is pool offset | length << 32 |
// is8Bit << 63: the code units the original built with a StringBuilder and registered in m_stringPool, in order (some,
// like an escaped "url" function name, have no record). Records of CommentToken (observer mode only) have no other
// fields; the glue drops them.
uint64_t WebCoreSwift_CSSTokenizer_tokenize(const void* characters, size_t length, bool is8Bit, uint64_t* tokens, size_t tokenCapacity, uint64_t* state);

// CSSTokenizer::preprocessString's result for an input that needs it: length char16_t code units, NUL and (in UTF-16
// input) unpaired surrogates replaced with U+FFFD.
void WebCoreSwift_CSSTokenizer_preprocess(const void* characters, size_t length, bool is8Bit, char16_t* output);

// CSSParserToken::stringToUnitType: the CSSUnitType raw value of a unit name.
uint8_t WebCoreSwift_CSSTokenizer_unitType(const void* characters, size_t length, bool is8Bit);

#ifdef __cplusplus
}
#endif
