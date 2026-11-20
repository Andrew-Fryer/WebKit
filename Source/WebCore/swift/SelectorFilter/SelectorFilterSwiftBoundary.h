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

// C boundary of the SelectorFilter Swift kernel (Source/WebCore/swift/SelectorFilter): the ancestor stack of
// SelectorFilter::pushParent and SelectorFilter::popParent (css/SelectorFilter.cpp). Swift owns the identifier hashes
// of every element on the parent stack and does every update of the ancestor filter, a WTF::CountingBloomFilter<12>.
// The symbols are internal to WebCore (hidden); only WebCore .cpp files may call them, never header-inline code.
//
// `buckets` is the filter's whole bucket array: asMutableByteSpan(CountingBloomFilter<12>).data(), always
// WebCoreSwift_SelectorFilter_BucketCount bytes (static_assert it). `stack` is the hash stack: `stackWords` 32-bit words that C++ allocates and keeps for
// Swift, starting with WebCoreSwift_SelectorFilter_StackHeaderWords words that must be zero for an empty stack. C++
// never reads or writes the words otherwise, except to copy all of them when it grows the storage. `hashes` are the
// pushed element's identifier hashes, null only when hashCount is 0.

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

enum {
    WebCoreSwift_SelectorFilter_BucketCount = 4096, // CountingBloomFilter<12>::tableSize.
    WebCoreSwift_SelectorFilter_StackHeaderWords = 2, // The minimum stackWords; zero-initialized for an empty stack.
};

// Pushes a frame with `hashes` and adds them to the filter: for (auto hash : hashes) filter.add(hash);
// Returns 0, or, if the frame doesn't fit in `stackWords`, the number of words needed, changing nothing. Then grow the
// storage to at least that many words, keeping its contents, and push again.
WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE size_t WebCoreSwift_SelectorFilter_push(uint8_t* buckets, uint32_t* stack, size_t stackWords, const uint32_t* hashes, size_t hashCount);

// Pops the top frame and removes its hashes from the filter: for (auto hash : hashes) filter.remove(hash);
// If the stack is then empty: ASSERT(filter.likelyEmpty()); filter.clear(); Traps if the stack is empty.
WEBCORE_SWIFT_SELECTOR_FILTER_NODELETE void WebCoreSwift_SelectorFilter_pop(uint8_t* buckets, uint32_t* stack, size_t stackWords);

#ifdef __cplusplus
}
#endif
