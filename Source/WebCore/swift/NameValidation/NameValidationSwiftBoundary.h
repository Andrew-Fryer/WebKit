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

// C boundary of the NameValidation Swift kernels (Source/WebCore/swift/NameValidation), the Swift port of
// isValidAttributeName(StringView) and isValidElementName(StringView) of dom/NameValidation.cpp. The symbols are
// internal to WebCore (hidden); only WebCore .cpp files may call them, never header-inline code.
//
// The characters are a StringView's: rawCharacters() (null only when length is 0), length() and is8Bit(). When is8Bit
// is false they are `length` UTF-16 code units, read with unaligned loads, so any alignment is fine.

#include <stddef.h>
#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_NAME_VALIDATION_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_NAME_VALIDATION_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// NameValidation::isValidAttributeName(StringView).
WEBCORE_SWIFT_NAME_VALIDATION_NODELETE bool WebCoreSwift_NameValidation_isValidAttributeName(const void* characters, size_t length, bool is8Bit);

// NameValidation::isValidElementName(StringView).
WEBCORE_SWIFT_NAME_VALIDATION_NODELETE bool WebCoreSwift_NameValidation_isValidElementName(const void* characters, size_t length, bool is8Bit);

#ifdef __cplusplus
}
#endif
