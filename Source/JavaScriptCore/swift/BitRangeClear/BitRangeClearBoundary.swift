// Copyright (C) 2026 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

// C entry point for BitRangeClear.swift, declared to C++ in BitRangeClearSwiftBoundary.h and called only from
// dfg/DFGInPlaceAbstractState.cpp and dfg/DFGStoreBarrierClusteringPhase.cpp. It takes a FastBitVector's words as a
// pointer and a word count, and the only unsafe operation is turning that pair into a MutableSpan. The pointer may be
// null only when the count is 0.

@_cdecl("JSCSwift_BitRangeClear_clearRange")
@inline(__always)
func JSCSwift_BitRangeClear_clearRange(_ words: UnsafeMutablePointer<UInt32>?, _ wordCount: Int, _ numBits: Int, _ begin: Int, _ end: Int) {
    var span = unsafe UnsafeMutableBufferPointer(start: words, count: wordCount).mutableSpan
    BitRangeClear.clearRange(&span, numBits: UInt(bitPattern: numBits), UInt(bitPattern: begin), UInt(bitPattern: end))
}
