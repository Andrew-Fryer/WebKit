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

// Resume state of the shape operations: size_t[WebCoreSwiftRegionShapeStateSize], zeroed before the first call. When
// the operation is done, it holds the result's segment and span counts at these indices.
enum {
    WebCoreSwiftRegionShapeStateSize = 10,
    WebCoreSwiftRegionShapeStateSegmentCount = 6,
    WebCoreSwiftRegionShapeStateSpanCount = 7,
};

// Region::Shape::shapeOperation<UnionOperation / IntersectOperation / SubtractOperation>(shape1, shape2), minus
// trySimpleOperation, into outSegments / outSpanWords. outSpanWords must have room for all the spans of both shapes
// (outSpanWordCapacity >= spanWordCount1 + spanWordCount2). Returns 0 when done (see the state). Otherwise outSegments
// is too small: grow it to at least the returned capacity, keeping its contents, and call again with the same
// arguments and state.
#define WEBCORE_SWIFT_REGION_SHAPE_OPERATION(name) \
    size_t name(const int32_t* segments1, size_t segmentCount1, const uint64_t* spanWords1, size_t spanWordCount1, \
        const int32_t* segments2, size_t segmentCount2, const uint64_t* spanWords2, size_t spanWordCount2, \
        int32_t* outSegments, size_t outSegmentCapacity, uint64_t* outSpanWords, size_t outSpanWordCapacity, size_t* state)
WEBCORE_SWIFT_REGION_SHAPE_OPERATION(WebCoreSwift_RegionShape_unite);
WEBCORE_SWIFT_REGION_SHAPE_OPERATION(WebCoreSwift_RegionShape_intersect);
WEBCORE_SWIFT_REGION_SHAPE_OPERATION(WebCoreSwift_RegionShape_subtract);
#undef WEBCORE_SWIFT_REGION_SHAPE_OPERATION

// Region::Shape::compareShapes<CompareContainsOperation>(shape1, shape2).
WEBCORE_SWIFT_REGION_SHAPE_NODELETE bool WebCoreSwift_RegionShape_contains(
    const int32_t* segments1, size_t segmentCount1, const uint64_t* spanWords1, size_t spanWordCount1,
    const int32_t* segments2, size_t segmentCount2, const uint64_t* spanWords2, size_t spanWordCount2);

// Region::Shape::compareShapes<CompareIntersectsOperation>(shape1, shape2).
WEBCORE_SWIFT_REGION_SHAPE_NODELETE bool WebCoreSwift_RegionShape_intersects(
    const int32_t* segments1, size_t segmentCount1, const uint64_t* spanWords1, size_t spanWordCount1,
    const int32_t* segments2, size_t segmentCount2, const uint64_t* spanWords2, size_t spanWordCount2);

// Region::Shape::bounds(), as outRect[4] = { x, y, width, height }.
WEBCORE_SWIFT_REGION_SHAPE_NODELETE void WebCoreSwift_RegionShape_bounds(const int32_t* segments, size_t segmentCount,
    const uint64_t* spanWords, size_t spanWordCount, int32_t* outRect);

// Region::Shape::isValidShape(segments, spans).
WEBCORE_SWIFT_REGION_SHAPE_NODELETE bool WebCoreSwift_RegionShape_isValidShape(const int32_t* segments, size_t segmentCount,
    const uint64_t* spanWords, size_t spanWordCount);

#ifdef __cplusplus
} // extern "C"
#endif
