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

// C boundary of the SelectorFilter Swift kernels (Source/WebCore/swift/SelectorFilter): the per-hash loops of
// SelectorFilter::pushParent and SelectorFilter::popParent (css/SelectorFilter.cpp) on the ancestor filter, a
// WTF::CountingBloomFilter<12>. The symbols are internal to WebCore (hidden); only WebCore .cpp files may call them,
// never header-inline code.
//
// `buckets` is the filter's whole bucket array (asMutableByteSpan of the CountingBloomFilter<12>, bucketCount = 4096,
// checked by the Swift side). `hashes` are the parent frame's identifierHashes, null only when hashCount is 0.

#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// for (auto hash : hashes) filter.add(hash);
WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE void WebCoreSwift_SelectorFilter_addHashes(uint8_t* buckets, size_t bucketCount, const uint32_t* hashes, size_t hashCount);

// for (auto hash : hashes) filter.remove(hash);
WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE void WebCoreSwift_SelectorFilter_removeHashes(uint8_t* buckets, size_t bucketCount, const uint32_t* hashes, size_t hashCount);

// filter.clear();
WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE void WebCoreSwift_SelectorFilter_clear(uint8_t* buckets, size_t bucketCount);

#ifdef __cplusplus
}
#endif
