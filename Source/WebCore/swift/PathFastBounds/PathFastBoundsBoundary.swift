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

// Boundary for PathStream::computeFastBoundingRect in Source/WebCore/platform/graphics/PathStream.cpp, declared in
// PathFastBoundsSwiftBoundary.h: wraps the PathSegment array (4-byte aligned; PathStream.cpp static_asserts it) and the
// state struct in spans, and forwards the endPoint and extendRect callbacks.

@_cdecl("WebCoreSwift_PathFastBounds_computeFastBoundingRect")
func WebCoreSwift_PathFastBounds_computeFastBoundingRect(_ segments: UnsafeRawPointer?, _ wordCount: Int,
    _ segmentSizeInWords: Int, _ tagOffset: Int, _ context: UnsafeRawPointer?,
    _ endPoint: @convention(c) (UnsafeRawPointer?, Int, UInt64, UnsafeMutablePointer<UInt64>?) -> UInt64,
    _ extendRect: @convention(c) (UnsafeRawPointer?, Int, UnsafeMutableRawPointer?) -> Void, _ state: UnsafeMutableRawPointer)
{
    let words = unsafe UnsafeBufferPointer(start: segments?.assumingMemoryBound(to: UInt32.self), count: wordCount)
    var stateFields = unsafe UnsafeMutableBufferPointer(start: state.assumingMemoryBound(to: Float.self), count: PathFastBounds.State.count).mutableSpan
    unsafe PathFastBounds.computeFastBoundingRect(segments: words.span, segmentSizeInWords: segmentSizeInWords, tagOffset: tagOffset,
        state: &stateFields,
        endPoint: { segmentIndex, currentPoint, lastMoveToPoint in
            var lastMoveToPoint = lastMoveToPoint
            let currentPoint = unsafe endPoint(context, segmentIndex, currentPoint, &lastMoveToPoint)
            return (currentPoint, lastMoveToPoint)
        },
        extendRect: { segmentIndex, state in
            state.withFields { fields in
                fields.withUnsafeMutableBufferPointer { unsafe extendRect(context, segmentIndex, UnsafeMutableRawPointer($0.baseAddress)) }
            }
        })
}
