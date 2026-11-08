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

// Safe Swift port of the Region::Shape set algebra in Source/WebCore/platform/graphics/Region.cpp
// (shapeOperation<Union/Intersect/Subtract>, compareShapes<Contains/Intersects>, bounds, isValidShape).
//
// A shape is two flat arrays owned by C++:
// - segments: x coordinates. Each span's segments are [x, maxX) pairs in increasing x order.
// - spans: Region::Span { int y; uint64_t segmentIndex; }, seen here as two UInt64 words per span: y is the low 32 bits
//   of word 2 * i and segmentIndex is word 2 * i + 1. Span i covers [y(i), y(i + 1)) and owns the segments
//   [segmentIndex(i), segmentIndex(i + 1)). The last span has none (segmentIndex == segments.count).
//
// Results are identical to the C++ (the output arrays, not only the area they describe: Shape's operator== compares
// them and IPC encodes them). Where the C++ reads a malformed shape out of bounds, libc++ hardening traps; the same
// reads trap here.

/// The compile-time parameters of Region::Shape::shapeOperation<Operation> (UnionOperation, IntersectOperation,
/// SubtractOperation in Region.cpp).
struct RegionShapeOperation {
    let opCode: Int
    let addRemainingSegmentsFromSpan1: Bool
    let addRemainingSegmentsFromSpan2: Bool
    let addRemainingSpansFromShape1: Bool
    let addRemainingSpansFromShape2: Bool

    static var union: RegionShapeOperation {
        RegionShapeOperation(opCode: 0, addRemainingSegmentsFromSpan1: true, addRemainingSegmentsFromSpan2: true,
            addRemainingSpansFromShape1: true, addRemainingSpansFromShape2: true)
    }

    static var intersect: RegionShapeOperation {
        RegionShapeOperation(opCode: 3, addRemainingSegmentsFromSpan1: false, addRemainingSegmentsFromSpan2: false,
            addRemainingSpansFromShape1: false, addRemainingSpansFromShape2: false)
    }

    static var subtract: RegionShapeOperation {
        RegionShapeOperation(opCode: 1, addRemainingSegmentsFromSpan1: true, addRemainingSegmentsFromSpan2: false,
            addRemainingSpansFromShape1: true, addRemainingSpansFromShape2: false)
    }
}

/// Bounds of a shape, as the IntRect(x, y, width, height) that Region::Shape::bounds() returns.
struct RegionShapeBounds {
    var x: Int32
    var y: Int32
    var width: Int32
    var height: Int32
}

enum RegionShape {
    // Layout of the shapeOperation resume state (size_t[8] owned by the C++ glue, zero before the first call).
    // [0] p1, [1] p2: next span of each shape. [2] lo1, [3] hi1, [4] lo2, [5] hi2: the current segments of each shape.
    // [6] segment count, [7] span count of the result so far.
    static var stateWordCount: Int { 10 }

    // Spans are addressed by the index of their first word: span i is at word 2 * i (its y) and 2 * i + 1 (its
    // segmentIndex). "Span i + 1 exists" is `2 * i + 3 < words.count`, for an even or odd word count alike.

    @inline(__always)
    static func y(_ words: Span<UInt64>, _ index: Int) -> Int32 {
        Int32(truncatingIfNeeded: words[2 &* index])
    }

    @inline(__always)
    static func segmentIndex(_ words: Span<UInt64>, _ index: Int) -> UInt64 {
        words[2 &* index &+ 1]
    }

    /// Region::Shape::segments(spans.subspan(index)), as the range lo..<hi of segment indices.
    @inline(__always)
    static func segments(_ segmentCount: Int, _ words: Span<UInt64>, _ index: Int) -> (lo: Int, hi: Int) {
        segments(segmentCount, segmentIndex(words, index), words, nextSegmentIndexWord: 2 &* index &+ 3)
    }

    /// Region::Shape::segments() for a span whose segmentIndex is `start`; the next span's segmentIndex is
    /// words[nextSegmentIndexWord], read only if needed (as the C++ reads span[1]).
    @inline(__always)
    static func segments(_ segmentCount: Int, _ start: UInt64, _ words: Span<UInt64>, nextSegmentIndexWord: Int)
        -> (lo: Int, hi: Int)
    {
        let size = UInt64(truncatingIfNeeded: segmentCount)

        // Check if this span has any segments.
        if start == size {
            return (segmentCount, segmentCount)
        }

        // m_segments.subspan(span[0].segmentIndex, span[1].segmentIndex - span[0].segmentIndex): libc++ hardening traps
        // when span[1] doesn't exist (the subscript below) or the subspan isn't inside m_segments (the preconditions).
        // A count of std::dynamic_extent (UInt64.max) means the rest of m_segments.
        precondition(start < size)
        var count = words[nextSegmentIndexWord] &- start
        if count == UInt64.max {
            count = size &- start
        }
        precondition(count <= size &- start)
        let lo = Int(truncatingIfNeeded: start)
        return (lo, lo &+ Int(truncatingIfNeeded: count))
    }

    // MARK: - compareShapes

    /// Region::Shape::segments(spans) for spans = the remaining spans as words (spans[0] = y, spans[1] = segmentIndex,
    /// spans[3] = the next span's segmentIndex), as a span of segments. Every trap matches libc++ hardening in the C++:
    /// reading span[1] when there is none, and m_segments.subspan(offset, count) outside m_segments (here `extracting`).
    @inline(__always)
    @_lifetime(copy segments)
    static func segmentsSpan(_ segments: Span<Int32>, _ spans: Span<UInt64>) -> Span<Int32> {
        let size = UInt64(truncatingIfNeeded: segments.count)
        let start = spans[1]

        // Check if this span has any segments.
        if start == size {
            return segments.extracting(last: 0)
        }

        var count = spans[3] &- start
        if count == UInt64.max {
            // std::dynamic_extent: the rest of m_segments.
            count = size &- start
        }
        let lo = Int(truncatingIfNeeded: start)
        let hi = lo &+ Int(truncatingIfNeeded: count)
        return segments.extracting(lo..<hi)
    }

    /// Region::Shape::compareShapes<CompareContainsOperation> (contains == true) or
    /// <CompareIntersectsOperation> (contains == false).
    ///
    /// Contains: defaultResult true; bOutsideA sets false and returns; aOutsideB and aOverlapsB never return.
    /// Intersects: defaultResult false; aOverlapsB sets true and returns; aOutsideB and bOutsideA never return.
    @inline(__always)
    static func compareShapes(contains: Bool, _ aSegmentsAll: Span<Int32>, _ aWords: Span<UInt64>,
        _ bSegmentsAll: Span<Int32>, _ bWords: Span<UInt64>) -> Bool
    {
        // The remaining spans, as words; "spans.size() > 1" is "count > 3" (for an even or odd word count alike).
        var aSpans = aWords
        var bSpans = bWords

        // bHadSegmentInPreviousSpan only gates aOutsideB, which never returns for either operation.
        var aHadSegmentInPreviousSpan = false
        while aSpans.count > 3 && bSpans.count > 3 {
            let aY = Int32(truncatingIfNeeded: aSpans[0])
            let aMaxY = Int32(truncatingIfNeeded: aSpans[2])
            let bY = Int32(truncatingIfNeeded: bSpans[0])
            let bMaxY = Int32(truncatingIfNeeded: bSpans[2])

            var aSegments = segmentsSpan(aSegmentsAll, aSpans)
            var bSegments = segmentsSpan(bSegmentsAll, bSpans)

            // Look for a non-overlapping part of the spans. If B had a segment in its previous span, then we already
            // tested A against B within that span.
            let aHasSegmentInSpan = !aSegments.isEmpty
            let bHasSegmentInSpan = !bSegments.isEmpty
            if contains && bY < aY && !aHadSegmentInPreviousSpan && bHasSegmentInSpan {
                return false
            }

            aHadSegmentInPreviousSpan = aHasSegmentInSpan

            let spansOverlap = bMaxY > aY && bY < aMaxY
            if spansOverlap {
                while !aSegments.isEmpty && !bSegments.isEmpty {
                    // [1] traps like libc++ hardening when a span has an odd segment count.
                    let aX = aSegments[0]
                    let aMaxX = aSegments[1]
                    let bX = bSegments[0]
                    let bMaxX = bSegments[1]

                    let segmentsOverlap = bMaxX > aX && bX < aMaxX
                    if !contains && segmentsOverlap {
                        return true
                    }
                    if contains && bX < aX {
                        return false
                    }

                    if aMaxX < bMaxX {
                        aSegments = aSegments.extracting(droppingFirst: 2)
                    } else if bMaxX < aMaxX {
                        bSegments = bSegments.extracting(droppingFirst: 2)
                    } else {
                        aSegments = aSegments.extracting(droppingFirst: 2)
                        bSegments = bSegments.extracting(droppingFirst: 2)
                    }
                }

                if contains && !bSegments.isEmpty {
                    return false
                }
            }

            if aMaxY < bMaxY {
                aSpans = aSpans.extracting(droppingFirst: 2)
            } else if bMaxY < aMaxY {
                bSpans = bSpans.extracting(droppingFirst: 2)
            } else {
                aSpans = aSpans.extracting(droppingFirst: 2)
                bSpans = bSpans.extracting(droppingFirst: 2)
            }
        }

        if contains && bSpans.count > 3 {
            return false
        }

        return contains
    }

    // MARK: - bounds

    /// Region::Shape::bounds().
    @inline(__always)
    static func bounds(_ segments: Span<Int32>, _ words: Span<UInt64>) -> RegionShapeBounds {
        if words.count < 2 {
            return RegionShapeBounds(x: 0, y: 0, width: 0, height: 0)
        }

        let minY = Int32(truncatingIfNeeded: words[0])
        let maxY = Int32(truncatingIfNeeded: words[(words.count & ~1) &- 2])

        var minX = Int32.max
        var maxX = Int32.min

        // w is the segmentIndex word of the current span; its end is the next span's segmentIndex, which is the
        // next iteration's start.
        var start = words[1]
        var w = 1
        while w &+ 2 < words.count {
            let next = words[w &+ 2]
            let (lo, hi) = RegionShape.segments(segments.count, start, words, nextSegmentIndexWord: w &+ 2)
            if lo < hi {
                let firstSegment = segments[lo]
                let lastSegment = segments[hi &- 1]

                if firstSegment < minX {
                    minX = firstSegment
                }

                if lastSegment > maxX {
                    maxX = lastSegment
                }
            }

            start = next
            w &+= 2
        }

        // CheckedInt32 differences, saturated to INT32_MAX on overflow.
        let (width, widthOverflowed) = maxX.subtractingReportingOverflow(minX)
        let (height, heightOverflowed) = maxY.subtractingReportingOverflow(minY)
        return RegionShapeBounds(x: minX, y: minY, width: widthOverflowed ? Int32.max : width,
            height: heightOverflowed ? Int32.max : height)
    }

    // MARK: - isValidShape

    /// Region::Shape::isValidShape(segments, spans). Every check is a pure comparison, so only the conjunction matters.
    @inline(__always)
    static func isValidShape(_ segments: Span<Int32>, _ words: Span<UInt64>) -> Bool {
        let spansSize = words.count / 2
        let segmentsSize = segments.count
        if spansSize == 0 {
            return segmentsSize == 0
        }
        if segmentsSize == 0 {
            return false
        }
        if spansSize == 1 {
            return false
        }
        if segmentsSize % 2 != 0 {
            return false
        }
        let size = UInt64(truncatingIfNeeded: segmentsSize)
        var index = 0
        while index < spansSize {
            let start = segmentIndex(words, index)
            if start > size {
                return false
            }
            if start % 2 != 0 {
                return false
            }

            if index < spansSize &- 1 {
                let end = segmentIndex(words, index &+ 1)

                if y(words, index) >= y(words, index &+ 1) {
                    return false
                }
                if start > end {
                    return false
                }

                // segmentsForSpanSegmentIndices: empty unless end < segments.size().
                if end < size {
                    var lastX = Int32.min
                    var k = Int(truncatingIfNeeded: start)
                    let kEnd = Int(truncatingIfNeeded: end)
                    while k < kEnd {
                        let segment = segments[k]
                        if lastX > segment {
                            return false
                        }
                        lastX = segment
                        k &+= 1
                    }
                }
            } else if start != size {
                return false
            }
            index &+= 1
        }
        return true
    }

    // MARK: - shapeOperation

    /// Region::Shape::appendSpan(y, segments) where the new span's segments were already written to
    /// outSegments[segmentCount..<end]: they become the result's new last span, unless canCoalesce (they equal the
    /// segments of the current last span), in which case they're dropped.
    @inline(__always)
    static func appendSpan(_ y: Int32, end: Int, _ outSegments: inout MutableSpan<Int32>, _ outWords: inout MutableSpan<UInt64>,
        _ segmentCount: inout Int, _ spanCount: inout Int, _ lastSpanStart: inout Int)
    {
        let newCount = end &- segmentCount
        if spanCount != 0 && newCount == segmentCount &- lastSpanStart {
            var equal = true
            var i = 0
            while i < newCount {
                if outSegments[lastSpanStart &+ i] != outSegments[segmentCount &+ i] {
                    equal = false
                    break
                }
                i &+= 1
            }
            if equal {
                return
            }
        }

        outWords[2 &* spanCount] = UInt64(truncatingIfNeeded: UInt32(bitPattern: y))
        outWords[2 &* spanCount &+ 1] = UInt64(truncatingIfNeeded: segmentCount)
        spanCount &+= 1
        lastSpanStart = segmentCount
        segmentCount = end
    }

    /// Copies source[lo..<hi] to outSegments[at...]. Returns the end, or nil if outSegments is too small.
    @inline(__always)
    static func appendSegments(_ source: Span<Int32>, _ lo: Int, _ hi: Int, _ outSegments: inout MutableSpan<Int32>,
        at start: Int) -> Int?
    {
        if hi &- lo > outSegments.count &- start {
            return nil
        }
        var end = start
        var i = lo
        while i < hi {
            outSegments[end] = source[i]
            end &+= 1
            i &+= 1
        }
        return end
    }

    /// The inner loop of shapeOperation: merges segments1[lo1..<hi1] and segments2[lo2..<hi2] into
    /// outSegments[start...], then adds the remaining segments. Returns the end, or nil if outSegments is too small.
    @inline(__always)
    static func mergeSegments(_ operation: RegionShapeOperation, _ segments1: Span<Int32>, _ lo1: Int, _ hi1: Int,
        _ segments2: Span<Int32>, _ lo2: Int, _ hi2: Int, _ outSegments: inout MutableSpan<Int32>, at start: Int) -> Int?
    {
        var flag = 0
        var oldFlag = 0

        var i1 = lo1
        var i2 = lo2
        var end = start

        // Now iterate over the segments in each span and construct a new vector of segments.
        while i1 < hi1 && i2 < hi2 {
            let x1 = segments1[i1]
            let x2 = segments2[i2]
            var x: Int32 = 0

            if x1 <= x2 {
                x = x1
                flag ^= 1
                i1 &+= 1
            }
            if x1 >= x2 {
                x = x2
                flag ^= 2
                i2 &+= 1
            }

            if flag == operation.opCode || oldFlag == operation.opCode {
                if end == outSegments.count {
                    return nil
                }
                outSegments[end] = x
                end &+= 1
            }

            oldFlag = flag
        }

        // Add any remaining segments.
        if operation.addRemainingSegmentsFromSpan1 && i1 < hi1 {
            return appendSegments(segments1, i1, hi1, &outSegments, at: end)
        }
        if operation.addRemainingSegmentsFromSpan2 && i2 < hi2 {
            return appendSegments(segments2, i2, hi2, &outSegments, at: end)
        }
        return end
    }

    /// Region::Shape::appendSpans(shape, spans.subspan(index)), resumable: returns the segment capacity needed to
    /// continue (with index at the span that didn't fit), or nil when done.
    @inline(__always)
    static func appendSpans(_ segments: Span<Int32>, _ words: Span<UInt64>, _ index: inout Int,
        _ outSegments: inout MutableSpan<Int32>, _ outWords: inout MutableSpan<UInt64>,
        _ segmentCount: inout Int, _ spanCount: inout Int, _ lastSpanStart: inout Int) -> Int?
    {
        let count = words.count / 2
        while index < count {
            let (lo, hi) = RegionShape.segments(segments.count, words, index)
            guard let end = appendSegments(segments, lo, hi, &outSegments, at: segmentCount) else {
                return segmentCount &+ (hi &- lo)
            }
            appendSpan(y(words, index), end: end, &outSegments, &outWords, &segmentCount, &spanCount, &lastSpanStart)
            index &+= 1
        }
        return nil
    }

    /// Region::Shape::shapeOperation<operation>(shape1, shape2) after trySimpleOperation, writing the result to
    /// outSegments / outWords (2 words per span; the C++ guarantees room for spanCount1 + spanCount2 spans, which is
    /// an upper bound: every loop iteration consumes at least one input span and appends at most one).
    ///
    /// The result's segment count has no such bound, so this is resumable: when outSegments is full it saves its
    /// position in `state` (with the current span pending: its input spans are consumed, its segments not yet
    /// merged) and returns the capacity needed to merge that span. The caller then grows outSegments (keeping its
    /// contents) and calls again with the same state. Returns 0 when done; state[6] and state[7] are then the
    /// result's segment and span counts.
    @inline(__always)
    static func shapeOperation(_ operation: RegionShapeOperation, _ segments1: Span<Int32>, _ words1: Span<UInt64>,
        _ segments2: Span<Int32>, _ words2: Span<UInt64>, _ outSegments: inout MutableSpan<Int32>,
        _ outWords: inout MutableSpan<UInt64>, _ state: inout MutableSpan<Int>) -> Int
    {
        let spanCount1 = words1.count / 2
        let spanCount2 = words2.count / 2

        var p1 = state[0]
        var p2 = state[1]
        var lo1 = state[2]
        var hi1 = state[3]
        var lo2 = state[4]
        var hi2 = state[5]
        var segmentCount = state[6]
        var spanCount = state[7]
        // state[8]: 1 if the span at y = state[9] is pending.
        var pending = state[8] != 0
        var y = Int32(truncatingIfNeeded: state[9])
        var lastSpanStart = spanCount > 0 ? Int(truncatingIfNeeded: outWords[2 &* spanCount &- 1]) : 0

        // Iterate over all spans.
        while pending || (p1 < spanCount1 && p2 < spanCount2) {
            if !pending {
                let y1 = RegionShape.y(words1, p1)
                let y2 = RegionShape.y(words2, p2)

                if y1 <= y2 {
                    y = y1

                    (lo1, hi1) = segments(segments1.count, words1, p1)
                    p1 &+= 1
                }
                if y1 >= y2 {
                    y = y2

                    (lo2, hi2) = segments(segments2.count, words2, p2)
                    p2 &+= 1
                }
            }
            pending = false

            guard let end = mergeSegments(operation, segments1, lo1, hi1, segments2, lo2, hi2, &outSegments,
                at: segmentCount) else {
                saveState(&state, p1, p2, lo1, hi1, lo2, hi2, segmentCount, spanCount)
                state[8] = 1
                state[9] = Int(y)
                return segmentCount &+ (hi1 &- lo1) &+ (hi2 &- lo2)
            }

            // Add the span.
            if end != segmentCount || spanCount != 0 {
                appendSpan(y, end: end, &outSegments, &outWords, &segmentCount, &spanCount, &lastSpanStart)
            }
        }

        // Add any remaining spans.
        if operation.addRemainingSpansFromShape1 && p1 < spanCount1 {
            if let needed = appendSpans(segments1, words1, &p1, &outSegments, &outWords, &segmentCount, &spanCount,
                &lastSpanStart) {
                saveState(&state, p1, p2, lo1, hi1, lo2, hi2, segmentCount, spanCount)
                return needed
            }
        } else if operation.addRemainingSpansFromShape2 && p2 < spanCount2 {
            if let needed = appendSpans(segments2, words2, &p2, &outSegments, &outWords, &segmentCount, &spanCount,
                &lastSpanStart) {
                saveState(&state, p1, p2, lo1, hi1, lo2, hi2, segmentCount, spanCount)
                return needed
            }
        }

        state[6] = segmentCount
        state[7] = spanCount
        return 0
    }

    @inline(__always)
    static func saveState(_ state: inout MutableSpan<Int>, _ p1: Int, _ p2: Int, _ lo1: Int, _ hi1: Int, _ lo2: Int,
        _ hi2: Int, _ segmentCount: Int, _ spanCount: Int)
    {
        state[0] = p1
        state[1] = p2
        state[2] = lo1
        state[3] = hi1
        state[4] = lo2
        state[5] = hi2
        state[6] = segmentCount
        state[7] = spanCount
        state[8] = 0
    }
}
