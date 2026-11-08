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

// C boundary of the HTMLScan Swift kernels (Source/WebCore/swift/HTMLScan): the batching scans of
// HTMLTokenizer::processToken (HTMLTokenizer.cpp) and the scanners of HTMLFastPathParser
// (HTMLDocumentParserFastPath.cpp). The symbols are internal to WebCore (hidden); only WebCore .cpp files may call
// them, never header-inline code.
//
// Inputs are the parser's remaining characters: `length` Latin-1 bytes (…8 functions) or `length` UTF-16 code units
// read with unaligned loads, so any alignment is fine (…16 functions). Pointers may be null only when the length is 0.
// Offsets and lengths are in code units, relative to the start of the input.

#include <stddef.h>
#include <stdint.h>

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

#ifdef __cplusplus
}
#endif
