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

// C boundary of the HTMLScan Swift kernels (Source/WebCore/swift/HTMLScan): the batching scan of
// HTMLTokenizer::processToken (HTMLTokenizer.cpp) and the scanners of HTMLFastPathParser
// (HTMLDocumentParserFastPath.cpp). The symbols are internal to WebCore (hidden); only WebCore .cpp files may call
// them, never header-inline code.
//
// Inputs are the parser's remaining characters: `length` Latin-1 bytes (is8Bit, or the …8 function) or `length`
// UTF-16 code units (!is8Bit) at any alignment. Pointers may be null only when the length is 0. Offsets and lengths
// are in code units, relative to the start of the input.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_HTML_SCAN_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_HTML_SCAN_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// HTMLTokenizer: the length of the leading run free of stopCharacter, '&', '\r', '\n' and '\0', i.e.
// findPlainTextInDataState (stopCharacter '<') and findPlainTextInQuotedAttributeValue<quote> (stopCharacter quote).
WEBCORE_SWIFT_HTML_SCAN_NODELETE size_t WebCoreSwift_HTMLScan_plainTextLength8(const uint8_t* characters, size_t length, uint8_t stopCharacter);

// HTMLFastPathParser (HTMLDocumentParserFastPath.cpp). `characters` is Latin-1 when is8Bit, else UTF-16.

// scanText(): kind | length << 8 (see WebCoreSwiftHTMLScanText). `length` is the text length for Plain and
// WhitespacePattern, where scanText() leaves the position for the failures, and 0 for Escaped.
enum WebCoreSwiftHTMLScanText {
    WebCoreSwiftHTMLScanTextPlain = 0,
    WebCoreSwiftHTMLScanTextWhitespacePattern = 1,
    WebCoreSwiftHTMLScanTextEscaped = 2,
    WebCoreSwiftHTMLScanTextFailedContainsNull = 3,
    WebCoreSwiftHTMLScanTextFailedBigText = 4,
};
WEBCORE_SWIFT_HTML_SCAN_NODELETE uint64_t WebCoreSwift_HTMLScan_fastPathScanText(const void* characters, size_t length, bool is8Bit, size_t textLengthLimit);

// One attribute, offsets and lengths relative to the scanned input. The value is emptyAtom() when valueLength is 0
// (no '=', an empty value, or a failed value).
typedef struct {
    uint32_t nameOffset;
    uint32_t nameLength; // | WebCoreSwiftHTMLScanAttributeNameNeedsLowering when the name has ASCII upper case
    uint32_t valueOffset; // for the escaped value (the last record of ...EscapedValue), the opening quote
    uint32_t valueLength;
} WebCoreSwiftHTMLScanAttribute;
#define WebCoreSwiftHTMLScanAttributeNameNeedsLowering 0x80000000u
#define WebCoreSwiftHTMLScanAttributeCapacity 16

// What a start tag scan writes besides its result: where the attribute list scan stopped (see
// WebCoreSwiftHTMLScanAttributes) and attributes[0 ..< count], one record per attribute that parseAttributes()
// appends or drops as a duplicate.
typedef struct {
    uint64_t consumed;
    WebCoreSwiftHTMLScanAttribute attributes[WebCoreSwiftHTMLScanAttributeCapacity];
} WebCoreSwiftHTMLScanStartTag;

// Tag classes: the supported tags in FOR_EACH_SUPPORTED_TAG order, then these two.
#define WebCoreSwiftHTMLScanTagClassOther 18
#define WebCoreSwiftHTMLScanTagClassFailed 19

// How the attribute list ended.
enum WebCoreSwiftHTMLScanAttributes {
    WebCoreSwiftHTMLScanAttributesDone = 0, // '>' or '/' whitespace* '>'; consumed is past the '>'
    WebCoreSwiftHTMLScanAttributesMore = 1, // the records are full; call again (scanTagName false) at consumed
    WebCoreSwiftHTMLScanAttributesEscapedValue = 2, // scanEscapedAttributeValue() from the last record's valueOffset (== consumed), skip whitespace, call again
    WebCoreSwiftHTMLScanAttributesFailed = 3, // parseAttributes() returns at consumed without setting attributes
};

// The first failure parseAttributes() records with didFail().
enum WebCoreSwiftHTMLScanFailure {
    WebCoreSwiftHTMLScanFailureNone = 0,
    WebCoreSwiftHTMLScanFailureParsingQuotedAttributeValue = 1,
    WebCoreSwiftHTMLScanFailureParsingUnquotedAttributeValue = 2,
    WebCoreSwiftHTMLScanFailureEndOfInputReached = 3,
    WebCoreSwiftHTMLScanFailureParsingAttributes = 4,
};

// scanTagName() (from just after the '<') and the scanning of parseAttributes(); with scanTagName false, the rest
// of an attribute list from the start of an attribute name. Fills *startTag. Returns
//   tag class (0 when scanTagName is false; for ...TagClassFailed nothing else is set)
//   | attributes status << 8 | first failure << 12 | attribute count << 16
//   | where scanTagName() leaves the position (after the name and the whitespace after it) << 32.
WEBCORE_SWIFT_HTML_SCAN_NODELETE uint64_t WebCoreSwift_HTMLScan_fastPathScanStartTag(const void* characters, size_t length, bool is8Bit, bool scanTagName, WebCoreSwiftHTMLScanStartTag* startTag);

#ifdef __cplusplus
}

// The Swift side writes WebCoreSwiftHTMLScanStartTag by these offsets (HTMLScan.startTag* in HTMLScanFastPath.swift).
static_assert(offsetof(WebCoreSwiftHTMLScanStartTag, consumed) == 0 && offsetof(WebCoreSwiftHTMLScanStartTag, attributes) == 8);
static_assert(sizeof(WebCoreSwiftHTMLScanAttribute) == 16 && sizeof(WebCoreSwiftHTMLScanStartTag) == 8 + 16 * WebCoreSwiftHTMLScanAttributeCapacity);
#endif
