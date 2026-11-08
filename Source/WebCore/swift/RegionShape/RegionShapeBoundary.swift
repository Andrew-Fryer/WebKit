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

// Shared by the three shape operations; inlined into each, so `operation` is a constant there.
@inline(__always)
func regionShapeOperation(_ operation: RegionShapeOperation,
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int,
    _ outSegments: UnsafeMutablePointer<Int32>?, _ outSegmentCapacity: Int,
    _ outSpanWords: UnsafeMutablePointer<UInt64>?, _ outSpanWordCapacity: Int, _ state: UnsafeMutablePointer<Int>?) -> Int
{
    let s1 = unsafe UnsafeBufferPointer(start: segments1, count: segmentCount1)
    let w1 = unsafe UnsafeBufferPointer(start: spanWords1, count: spanWordCount1)
    let s2 = unsafe UnsafeBufferPointer(start: segments2, count: segmentCount2)
    let w2 = unsafe UnsafeBufferPointer(start: spanWords2, count: spanWordCount2)
    var out = unsafe UnsafeMutableBufferPointer(start: outSegments, count: outSegmentCapacity).mutableSpan
    var outWords = unsafe UnsafeMutableBufferPointer(start: outSpanWords, count: outSpanWordCapacity).mutableSpan
    var stateWords = unsafe UnsafeMutableBufferPointer(start: state, count: RegionShape.stateWordCount).mutableSpan
    return unsafe RegionShape.shapeOperation(operation, s1.span, w1.span, s2.span, w2.span, &out, &outWords, &stateWords)
}

@_cdecl("WebCoreSwift_RegionShape_unite")
func WebCoreSwift_RegionShape_unite(
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int,
    _ outSegments: UnsafeMutablePointer<Int32>?, _ outSegmentCapacity: Int,
    _ outSpanWords: UnsafeMutablePointer<UInt64>?, _ outSpanWordCapacity: Int, _ state: UnsafeMutablePointer<Int>?) -> Int
{
    unsafe regionShapeOperation(.union, segments1, segmentCount1, spanWords1, spanWordCount1, segments2, segmentCount2, spanWords2, spanWordCount2, outSegments, outSegmentCapacity, outSpanWords, outSpanWordCapacity, state)
}

@_cdecl("WebCoreSwift_RegionShape_intersect")
func WebCoreSwift_RegionShape_intersect(
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int,
    _ outSegments: UnsafeMutablePointer<Int32>?, _ outSegmentCapacity: Int,
    _ outSpanWords: UnsafeMutablePointer<UInt64>?, _ outSpanWordCapacity: Int, _ state: UnsafeMutablePointer<Int>?) -> Int
{
    unsafe regionShapeOperation(.intersect, segments1, segmentCount1, spanWords1, spanWordCount1, segments2, segmentCount2, spanWords2, spanWordCount2, outSegments, outSegmentCapacity, outSpanWords, outSpanWordCapacity, state)
}

@_cdecl("WebCoreSwift_RegionShape_subtract")
func WebCoreSwift_RegionShape_subtract(
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int,
    _ outSegments: UnsafeMutablePointer<Int32>?, _ outSegmentCapacity: Int,
    _ outSpanWords: UnsafeMutablePointer<UInt64>?, _ outSpanWordCapacity: Int, _ state: UnsafeMutablePointer<Int>?) -> Int
{
    unsafe regionShapeOperation(.subtract, segments1, segmentCount1, spanWords1, spanWordCount1, segments2, segmentCount2, spanWords2, spanWordCount2, outSegments, outSegmentCapacity, outSpanWords, outSpanWordCapacity, state)
}

// Shared by contains and intersects; inlined into each, so `contains` is a constant there.
@inline(__always)
func regionCompareShapes(contains: Bool,
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int) -> Bool
{
    let s1 = unsafe UnsafeBufferPointer(start: segments1, count: segmentCount1)
    let w1 = unsafe UnsafeBufferPointer(start: spanWords1, count: spanWordCount1)
    let s2 = unsafe UnsafeBufferPointer(start: segments2, count: segmentCount2)
    let w2 = unsafe UnsafeBufferPointer(start: spanWords2, count: spanWordCount2)
    return unsafe RegionShape.compareShapes(contains: contains, s1.span, w1.span, s2.span, w2.span)
}

@_cdecl("WebCoreSwift_RegionShape_contains")
func WebCoreSwift_RegionShape_contains(
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int) -> Bool
{
    unsafe regionCompareShapes(contains: true, segments1, segmentCount1, spanWords1, spanWordCount1, segments2, segmentCount2, spanWords2, spanWordCount2)
}

@_cdecl("WebCoreSwift_RegionShape_intersects")
func WebCoreSwift_RegionShape_intersects(
    _ segments1: UnsafePointer<Int32>?, _ segmentCount1: Int, _ spanWords1: UnsafePointer<UInt64>?, _ spanWordCount1: Int,
    _ segments2: UnsafePointer<Int32>?, _ segmentCount2: Int, _ spanWords2: UnsafePointer<UInt64>?, _ spanWordCount2: Int) -> Bool
{
    unsafe regionCompareShapes(contains: false, segments1, segmentCount1, spanWords1, spanWordCount1, segments2, segmentCount2, spanWords2, spanWordCount2)
}

@_cdecl("WebCoreSwift_RegionShape_bounds")
func WebCoreSwift_RegionShape_bounds(_ segments: UnsafePointer<Int32>?, _ segmentCount: Int,
    _ spanWords: UnsafePointer<UInt64>?, _ spanWordCount: Int, _ outRect: UnsafeMutablePointer<Int32>?)
{
    let s = unsafe UnsafeBufferPointer(start: segments, count: segmentCount)
    let w = unsafe UnsafeBufferPointer(start: spanWords, count: spanWordCount)
    let bounds = unsafe RegionShape.bounds(s.span, w.span)
    var out = unsafe UnsafeMutableBufferPointer(start: outRect, count: 4).mutableSpan
    out[0] = bounds.x
    out[1] = bounds.y
    out[2] = bounds.width
    out[3] = bounds.height
}

@_cdecl("WebCoreSwift_RegionShape_isValidShape")
func WebCoreSwift_RegionShape_isValidShape(_ segments: UnsafePointer<Int32>?, _ segmentCount: Int,
    _ spanWords: UnsafePointer<UInt64>?, _ spanWordCount: Int) -> Bool
{
    let s = unsafe UnsafeBufferPointer(start: segments, count: segmentCount)
    let w = unsafe UnsafeBufferPointer(start: spanWords, count: spanWordCount)
    return unsafe RegionShape.isValidShape(s.span, w.span)
}
