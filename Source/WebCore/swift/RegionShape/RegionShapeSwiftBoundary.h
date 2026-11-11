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

// C entry points of the Swift port of Region::Shape's set algebra (RegionShape.swift, RegionShapeBoundary.swift).
// They are hidden symbols of WebCore: call them only from WebCore .cpp files (Region.cpp), never from headers.
//
// A shape crosses as its two arrays: the segments (int32_t), and the Region::Span array as uint64_t words, two per
// span: y in the low 32 bits of the first word, segmentIndex in the second (Region.cpp static_asserts this layout).
// Counts are element counts of these arrays (so a span count is passed as 2 * spans.size() words).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_REGION_SHAPE_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_REGION_SHAPE_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Region::Shape::shapeOperation<UnionOperation / IntersectOperation / SubtractOperation>(shape1, shape2), minus
// trySimpleOperation, into outSegments / outSpanWords, then the result's Region::Shape::bounds(). outSpanWords must
// have room for all the spans of both shapes (outSpanWordCapacity >= spanWordCount1 + spanWordCount2). state is
// size_t[WebCoreSwiftRegionShapeStateSize] (stateCount), zeroed before the first call. Returns 0 when done: the state
// then holds the result's segment and span counts and bounds at the indices below. Otherwise outSegments is too small:
// grow it to at least the returned capacity, keeping its contents, and call again with the same arguments and state.
// WebCoreSwiftRegionShapeBoundsOnly only stores Region::Shape::bounds() of shape1 in the state (and returns 0); the
// other arguments may be null / 0.
enum {
    WebCoreSwiftRegionShapeUnion = 0,
    WebCoreSwiftRegionShapeIntersect = 1,
    WebCoreSwiftRegionShapeSubtract = 2,
    WebCoreSwiftRegionShapeBoundsOnly = 3,
};
enum {
    WebCoreSwiftRegionShapeStateSize = 14,
    WebCoreSwiftRegionShapeStateSegmentCount = 6,
    WebCoreSwiftRegionShapeStateSpanCount = 7,
    WebCoreSwiftRegionShapeStateBoundsX = 10, // Then y, width, height (int32_t values).
};
WEBCORE_SWIFT_REGION_SHAPE_NODELETE size_t WebCoreSwift_RegionShape_combine(size_t operation,
    const int32_t* segments1, size_t segmentCount1, const uint64_t* spanWords1, size_t spanWordCount1,
    const int32_t* segments2, size_t segmentCount2, const uint64_t* spanWords2, size_t spanWordCount2,
    int32_t* outSegments, size_t outSegmentCapacity, uint64_t* outSpanWords, size_t outSpanWordCapacity, size_t* state, size_t stateCount);

// Region::Shape::compareShapes<CompareContainsOperation / CompareIntersectsOperation>(shape1, shape2), or
// isValidShape(shape1) (shape2 unused; may be null / 0).
enum {
    WebCoreSwiftRegionShapeContains = 0,
    WebCoreSwiftRegionShapeIntersects = 1,
    WebCoreSwiftRegionShapeIsValid = 2,
};
WEBCORE_SWIFT_REGION_SHAPE_NODELETE bool WebCoreSwift_RegionShape_query(size_t operation,
    const int32_t* segments1, size_t segmentCount1, const uint64_t* spanWords1, size_t spanWordCount1,
    const int32_t* segments2, size_t segmentCount2, const uint64_t* spanWords2, size_t spanWordCount2);

#ifdef __cplusplus
} // extern "C"
#endif
