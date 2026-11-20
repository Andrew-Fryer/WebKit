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

// WTF::FastBitVector::clearRange, moved into JavaScriptCore's Swift (BitRangeClear.swift) because only DFG code used it.
// JSCSwift_BitRangeClear_clearRange(words, wordCount, numBits, begin, end) is vector.clearRange(begin, end) for the
// FastBitVector whose size is numBits and whose words are words[0 ..< wordCount] (vector.unsafeWords().words(), all
// fastBitVectorArrayLength(numBits) of them): it clears bits [begin, end) and leaves every other bit alone, including
// the unused high bits of the last word (a reversed begin > end clears only the partial words at either end, as the
// C++ did). It traps where the C++ RELEASE_ASSERTed (a bit-wise cleared index >= numBits) or overran the words. words
// may be null only if wordCount is 0. The symbol is hidden in JavaScriptCore: only dfg/DFGInPlaceAbstractState.cpp and
// dfg/DFGStoreBarrierClusteringPhase.cpp call it.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

__attribute__((nothrow)) void JSCSwift_BitRangeClear_clearRange(uint32_t* words, size_t wordCount, size_t numBits, size_t begin, size_t end);

#ifdef __cplusplus
}
#endif
