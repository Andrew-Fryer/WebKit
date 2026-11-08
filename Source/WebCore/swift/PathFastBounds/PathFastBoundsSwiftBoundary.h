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

// The segment walk of PathStream::computeFastBoundingRect (platform/graphics/PathStream.cpp), implemented in Swift in
// PathFastBounds.swift. Only include this from WebCore .cpp files: the Swift symbols are hidden in WebCore.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The PathSegment layout the Swift code reads (PathStream.cpp static_asserts it; PathFastBounds.swift has the same
// values and traps if the arguments below differ from them). A PathSegment is a Variant whose alternative is stored at
// offset 0 and whose index is the byte at WebCoreSwiftPathFastBoundsTagOffset.
enum {
    WebCoreSwiftPathFastBoundsSegmentSizeInWords = 14, // sizeof(PathSegment) / sizeof(uint32_t)
    WebCoreSwiftPathFastBoundsTagOffset = 52, // Byte offset of the Variant index in a PathSegment.
};

// Variant indices of the PathSegment::Data alternatives (PathStream.cpp static_asserts them).
enum {
    WebCoreSwiftPathFastBoundsMoveTo = 0,
    WebCoreSwiftPathFastBoundsLineTo = 1,
    WebCoreSwiftPathFastBoundsQuadCurveTo = 2,
    WebCoreSwiftPathFastBoundsBezierCurveTo = 3,
    WebCoreSwiftPathFastBoundsArcTo = 4,
    WebCoreSwiftPathFastBoundsArc = 5,
    WebCoreSwiftPathFastBoundsClosedArc = 6,
    WebCoreSwiftPathFastBoundsEllipse = 7,
    WebCoreSwiftPathFastBoundsEllipseInRect = 8,
    WebCoreSwiftPathFastBoundsRect = 9,
    WebCoreSwiftPathFastBoundsRoundedRect = 10,
    WebCoreSwiftPathFastBoundsContinuousRoundedRect = 11,
    WebCoreSwiftPathFastBoundsDataLine = 12,
    WebCoreSwiftPathFastBoundsDataQuadCurve = 13,
    WebCoreSwiftPathFastBoundsDataBezierCurve = 14,
    WebCoreSwiftPathFastBoundsDataArc = 15,
    WebCoreSwiftPathFastBoundsCloseSubpath = 16,
};

// The loop state of computeFastBoundingRect: the current point, the last move-to point and the bounding rect.
typedef struct {
    float currentPointX;
    float currentPointY;
    float lastMoveToPointX;
    float lastMoveToPointY;
    float x;
    float y;
    float width;
    float height;
} WebCoreSwiftPathFastBoundsState;

// Runs `currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint)` for segment segmentIndex of the span
// that context points to. Points are the bit patterns of FloatPoints (x in the low 32 bits): currentPoint is in,
// *lastMoveToPoint is in/out, and the result is the new current point. Swift calls it for the segment types whose end
// point needs trigonometry (PathArcTo, PathArc, PathClosedArc, PathEllipse, PathDataArc) and unknown indices, after the
// segment's rect extension, and only once a later segment needs the end point (so possibly never for a segment).
typedef uint64_t (*WebCoreSwiftPathFastBoundsEndPoint)(const void* context, size_t segmentIndex, uint64_t currentPoint, uint64_t* lastMoveToPoint);

// Runs segment.extendFastBoundingRect(currentPoint, lastMoveToPoint, boundingRect) for segment segmentIndex of the span
// that context points to, updating the rect of *state. Swift calls it for the segments whose rect extension it does not
// do itself (a rotated PathEllipse, and any unknown index), then treats the end point as for the types above.
typedef void (*WebCoreSwiftPathFastBoundsExtendRect)(const void* context, size_t segmentIndex, WebCoreSwiftPathFastBoundsState*);

// segments points to wordCount 32-bit words: the PathSegment array (may be null if wordCount is 0). segmentSizeInWords
// and tagOffset must be WebCoreSwiftPathFastBoundsSegmentSizeInWords and WebCoreSwiftPathFastBoundsTagOffset, and
// wordCount a multiple of segmentSizeInWords. *state is in/out: on entry the initial state (both points (0, 0), the rect
// FloatRect::smallestRect(); the rect counts as still "smallest" at the end if it equals this initial rect), on return
// x, y, width and height are the bounding rect (the points are unspecified).
void WebCoreSwift_PathFastBounds_computeFastBoundingRect(const void* segments, size_t wordCount, size_t segmentSizeInWords,
    size_t tagOffset, const void* context, WebCoreSwiftPathFastBoundsEndPoint, WebCoreSwiftPathFastBoundsExtendRect,
    WebCoreSwiftPathFastBoundsState* state);

#ifdef __cplusplus
}
#endif
