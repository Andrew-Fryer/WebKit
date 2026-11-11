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

// Boundary: the only RegionShape file allowed to use `unsafe`. Declared to C++ in RegionShapeSwiftBoundary.h. Each
// entry point only wraps the (pointer, count) pairs that C++ passes into spans and calls the safe core in
// RegionShape.swift. Pointers are optional because an empty WTF::Vector's data() can be null. Region::Span arrays
// arrive as their UInt64 words (two per span), which are naturally aligned: Region::Span has 8-byte alignment.

@_cdecl("WebCoreSwift_RegionShape_combine")
func WebCoreSwift_RegionShape_combine(_ operation: Int,
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int,
    _ outSegments: UnsafeMutablePointer<Int32>?, _ outSegmentCapacity: Int, _ outSpanWords: UnsafeMutablePointer<UInt64>?,
    _ outSpanWordCapacity: Int, _ state: UnsafeMutablePointer<Int>?, _ stateCount: Int) -> Int
{
    let s1 = unsafe UnsafeBufferPointer(start: segments1, count: segmentCount1).span
    let w1 = unsafe UnsafeBufferPointer(start: spanWords1, count: spanWordCount1).span
    let s2 = unsafe UnsafeBufferPointer(start: segments2, count: segmentCount2).span
    let w2 = unsafe UnsafeBufferPointer(start: spanWords2, count: spanWordCount2).span
    var out = unsafe UnsafeMutableBufferPointer(start: outSegments, count: outSegmentCapacity).mutableSpan
    var outWords = unsafe UnsafeMutableBufferPointer(start: outSpanWords, count: outSpanWordCapacity).mutableSpan
    var stateWords = unsafe UnsafeMutableBufferPointer(start: state, count: stateCount).mutableSpan
    return RegionShape.combine(operation, s1, w1, s2, w2, &out, &outWords, &stateWords)
}

@_cdecl("WebCoreSwift_RegionShape_query")
func WebCoreSwift_RegionShape_query(_ operation: Int,
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int) -> Bool
{
    let s1 = unsafe UnsafeBufferPointer(start: segments1, count: segmentCount1).span
    let w1 = unsafe UnsafeBufferPointer(start: spanWords1, count: spanWordCount1).span
    let s2 = unsafe UnsafeBufferPointer(start: segments2, count: segmentCount2).span
    let w2 = unsafe UnsafeBufferPointer(start: spanWords2, count: spanWordCount2).span
    return RegionShape.query(operation, s1, w1, s2, w2)
}
