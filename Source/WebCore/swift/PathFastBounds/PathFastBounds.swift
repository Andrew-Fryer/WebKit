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

// PathStream::computeFastBoundingRect (Source/WebCore/platform/graphics/PathStream.cpp): walks a path's segments,
// extending the bounding rect by each segment's control points and tracking the current point and the last move-to
// point. extend(bySegment:) below is each segment type's extendFastBoundingRect() followed by its calculateEndPoint()
// from PathSegmentData.cpp, with FloatRect's extend()/uniteEvenIfEmpty() (FloatRect.cpp) spelled out. Swift has no libm,
// so for the segment types whose end point needs trigonometry (PathArcTo, PathArc, PathClosedArc, PathEllipse,
// PathDataArc) Swift extends the rect and the endPoint callback runs the original C++ calculateEndPoint() for that
// segment, and only once a later segment needs that end point (see walk()). A rotated PathEllipse (whose rect extension
// needs trigonometry too) and unknown indices also extend the rect in C++, through the extendRect callback (the original
// segment.extendFastBoundingRect()).
//
// Float semantics follow the C++ exactly: std::min(a, b) is `b < a ? b : a` and std::max(a, b) is `a < b ? b : a`
// (Swift.max returns its second argument on ties, so it differs on +0/-0), the rect is stored as x, y, width, height with
// maxX = x + width, every extend recomputes width = maxX - minX, and there are no multiplications that clang could
// contract into an FMA.
//
// The initial state, including FloatRect::smallestRect(), comes from C++, and "the rect is still smallestRect()" is
// a comparison with that initial rect, so the optimizer never knows the rect's values. If it did (here, in the final
// extend, or in a peeled first iteration) it would turn `b < a ? b : a` with a constant `a` into fminnm/fmaxnm, which
// return NaN for a signaling NaN `b` where the C++ returns `a`.

#if !_endian(little)
#error("PathFastBounds reads the PathSegment variant index as the low byte of a 32-bit word")
#endif

enum PathFastBounds {
    // The PathSegment layout (PathFastBoundsSwiftBoundary.h; PathStream.cpp static_asserts it). The alternative is
    // stored at word 0; the variant index is the byte at tagOffset.
    static var segmentSizeInWords: Int { 14 }
    static var tagOffset: Int { 52 }

    // PathSegment::Data alternative indices.
    static var moveTo: UInt8 { 0 }
    static var lineTo: UInt8 { 1 }
    static var quadCurveTo: UInt8 { 2 }
    static var bezierCurveTo: UInt8 { 3 }
    static var arcTo: UInt8 { 4 }
    static var arc: UInt8 { 5 }
    static var closedArc: UInt8 { 6 }
    static var ellipse: UInt8 { 7 }
    static var ellipseInRect: UInt8 { 8 }
    static var rect: UInt8 { 9 }
    static var roundedRect: UInt8 { 10 }
    static var continuousRoundedRect: UInt8 { 11 }
    static var dataLine: UInt8 { 12 }
    static var dataQuadCurve: UInt8 { 13 }
    static var dataBezierCurve: UInt8 { 14 }
    static var dataArc: UInt8 { 15 }
    static var closeSubpath: UInt8 { 16 }

    // The loop state; mirrors WebCoreSwiftPathFastBoundsState.
    struct State {
        var currentPointX: Float
        var currentPointY: Float
        var lastMoveToPointX: Float
        var lastMoveToPointY: Float
        var x: Float
        var y: Float
        var width: Float
        var height: Float

        static var count: Int { 8 }
    }

    // What C++ still has to do for a segment after extend(bySegment:).
    enum CxxWork {
        case none
        case endPoint // currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint).
        case extendRectAndEndPoint // segment.extendFastBoundingRect(currentPoint, lastMoveToPoint, boundingRect), then the end point.
    }

    // Runs the segment walk on *state (WebCoreSwiftPathFastBoundsState: on entry both points (0, 0) and the rect
    // FloatRect::smallestRect(); on return the final rect in x, y, width and height; the points are then unspecified).
    // endPoint(i, currentPoint, lastMoveToPoint) runs `currentPoint = segment.calculateEndPoint(currentPoint,
    // lastMoveToPoint)` in C++ for segment i; points are FloatPoint bit patterns (see State.pack) and it returns the new
    // current point and last move-to point. extendRect(i, &state) runs `segment.extendFastBoundingRect(currentPoint,
    // lastMoveToPoint, boundingRect)` in C++ for segment i (it updates the rect of the state).
    @inline(__always)
    static func computeFastBoundingRect(segments: Span<UInt32>, segmentSizeInWords: Int, tagOffset: Int, state stateFields: inout MutableSpan<Float>,
        endPoint: (_ segmentIndex: Int, _ currentPoint: UInt64, _ lastMoveToPoint: UInt64) -> (UInt64, UInt64),
        extendRect: (_ segmentIndex: Int, _ state: inout State) -> Void)
    {
        precondition(segmentSizeInWords == Self.segmentSizeInWords && tagOffset == Self.tagOffset, "PathSegment layout mismatch")
        precondition(segments.count % Self.segmentSizeInWords == 0)
        walk(segments: segments, state: &stateFields, endPoint: endPoint, extendRect: extendRect)
    }

    // The loop of computeFastBoundingRect().
    //
    // calculateEndPoint() has no side effects, and every segment type whose rect extension and end point do not read the
    // current point or the last move-to point (see State.readsPoints) sets both. So the end point of a segment that needs
    // C++ for it is only computed once a later segment reads the points, or at the end if the rect is still
    // FloatRect::smallestRect(); if a later segment sets both points first, it is never computed. Until then the state's
    // points are those from before that segment, pendingEndPoint is its index (-1 if there is none) and pendingTag its
    // type.
    //
    // Keep this a separate function: inlined into the @_cdecl entry point, its rect arithmetic stops being vectorized
    // (scalar fcmp/fcsel instead of fcmgt/bsl on arm64, measured 1.2-1.6x slower).
    @inline(never)
    static func walk(segments: Span<UInt32>, state stateFields: inout MutableSpan<Float>,
        endPoint: (_ segmentIndex: Int, _ currentPoint: UInt64, _ lastMoveToPoint: UInt64) -> (UInt64, UInt64),
        extendRect: (_ segmentIndex: Int, _ state: inout State) -> Void)
    {
        let initialState = State(stateFields.span)
        var state = initialState
        var pendingEndPoint = -1
        var pendingTag: UInt8 = 0
        for segmentIndex in 0..<(segments.count / Self.segmentSizeInWords) {
            let words = segments.extracting((segmentIndex * Self.segmentSizeInWords)..<((segmentIndex + 1) * Self.segmentSizeInWords))
            let tag = State.tag(words)
            if pendingEndPoint >= 0 && State.readsPoints(tag: tag, pendingTag: pendingTag) {
                state.computeEndPoint(of: &pendingEndPoint, endPoint)
            }
            switch state.extend(bySegment: words) {
            case .none:
                // The segment either read the points (after computeEndPoint()) or set them.
                pendingEndPoint = -1
            case .endPoint:
                pendingEndPoint = segmentIndex
                pendingTag = tag
            case .extendRectAndEndPoint:
                extendRect(segmentIndex, &state)
                pendingEndPoint = segmentIndex
                pendingTag = tag
            }
        }

        // `if (boundingRect.isSmallest()) boundingRect.extend(currentPoint)`.
        if state.hasSameRect(as: initialState) {
            state.computeEndPoint(of: &pendingEndPoint, endPoint)
            state.extend(state.currentPointX, state.currentPointY)
        }
        state.store(into: &stateFields)
    }
}

extension PathFastBounds.State {
    typealias CxxWork = PathFastBounds.CxxWork

    // One segment: `words` are its PathSegment words. Returns what C++ still has to do for it.
    @inline(always) mutating func extend(bySegment words: Span<UInt32>) -> CxxWork {
        @inline(__always) func value(_ i: Int) -> Float { Float(bitPattern: words[i]) }

        switch Self.tag(words) {
        case PathFastBounds.moveTo: // PathMoveTo { point }: no extension.
            let x = value(0), y = value(1)
            lastMoveToPointX = x
            lastMoveToPointY = y
            currentPointX = x
            currentPointY = y
        case PathFastBounds.lineTo: // PathLineTo { point }
            let x = value(0), y = value(1)
            extend(currentPointX, currentPointY)
            extend(x, y)
            currentPointX = x
            currentPointY = y
        case PathFastBounds.quadCurveTo: // PathQuadCurveTo { controlPoint, endPoint }
            let endX = value(2), endY = value(3)
            extend(currentPointX, currentPointY, value(0), value(1), endX, endY)
            currentPointX = endX
            currentPointY = endY
        case PathFastBounds.bezierCurveTo: // PathBezierCurveTo { controlPoint1, controlPoint2, endPoint }
            let endX = value(4), endY = value(5)
            extend(currentPointX, currentPointY, value(0), value(1), value(2), value(3), endX, endY)
            currentPointX = endX
            currentPointY = endY
        case PathFastBounds.arcTo: // PathArcTo { controlPoint1, controlPoint2, radius }: extendRect(currentPoint, cp1, cp2).
            extend(currentPointX, currentPointY, value(0), value(1), value(2), value(3))
            return .endPoint
        case PathFastBounds.arc, PathFastBounds.closedArc: // PathArc { center, radius, ... }, PathClosedArc { arc }
            let centerX = value(0), centerY = value(1), radius = value(2)
            // extend(center - FloatSize { radius, radius }, center + FloatSize { radius, radius }).
            extend(minX: centerX - radius, minY: centerY - radius, maxX: centerX + radius, maxY: centerY + radius)
            return .endPoint
        case PathFastBounds.ellipse: // PathEllipse { center, radiusX, radiusY, rotation, ... }
            // `if (!rotation)`: true for +0 and -0, false for NaN. A rotated ellipse maps its rect through a rotation.
            guard value(4) == 0 else {
                return .extendRectAndEndPoint
            }
            let centerX = value(0), centerY = value(1), radiusX = value(2), radiusY = value(3)
            // extend(center - FloatSize { radiusX, radiusY }, center + FloatSize { radiusX, radiusY }).
            extend(minX: centerX - radiusX, minY: centerY - radiusY, maxX: centerX + radiusX, maxY: centerY + radiusY)
            return .endPoint
        case PathFastBounds.ellipseInRect: // PathEllipseInRect { rect }
            let x = value(0), y = value(1), width = value(2), height = value(3)
            unite(x, y, width, height)
            // rect.center() + FloatSize { rect.width() / 2, 0 }, where center() = location() + size() / 2.
            let moveToX = (x + width / 2) + width / 2
            let moveToY = (y + height / 2) + 0
            lastMoveToPointX = moveToX
            lastMoveToPointY = moveToY
            currentPointX = moveToX
            currentPointY = moveToY
        case PathFastBounds.rect, PathFastBounds.roundedRect, PathFastBounds.continuousRoundedRect:
            // PathRect { rect }, PathRoundedRect { FloatRoundedRect { rect, radii }, ... }, PathContinuousRoundedRect { rect, ... }
            let x = value(0), y = value(1)
            unite(x, y, value(2), value(3))
            lastMoveToPointX = x
            lastMoveToPointY = y
            currentPointX = x
            currentPointY = y
        case PathFastBounds.dataLine: // PathDataLine { start, end }
            let startX = value(0), startY = value(1), endX = value(2), endY = value(3)
            extend(startX, startY)
            extend(endX, endY)
            lastMoveToPointX = startX
            lastMoveToPointY = startY
            currentPointX = endX
            currentPointY = endY
        case PathFastBounds.dataQuadCurve: // PathDataQuadCurve { start, controlPoint, endPoint }
            let startX = value(0), startY = value(1), endX = value(4), endY = value(5)
            extend(startX, startY, value(2), value(3), endX, endY)
            lastMoveToPointX = startX
            lastMoveToPointY = startY
            currentPointX = endX
            currentPointY = endY
        case PathFastBounds.dataBezierCurve: // PathDataBezierCurve { start, controlPoint1, controlPoint2, endPoint }
            let startX = value(0), startY = value(1), endX = value(6), endY = value(7)
            extend(startX, startY, value(2), value(3), value(4), value(5), endX, endY)
            lastMoveToPointX = startX
            lastMoveToPointY = startY
            currentPointX = endX
            currentPointY = endY
        case PathFastBounds.dataArc: // PathDataArc { start, controlPoint1, controlPoint2, radius }: extendRect(start, cp1, cp2).
            extend(value(0), value(1), value(2), value(3), value(4), value(5))
            return .endPoint
        case PathFastBounds.closeSubpath: // PathCloseSubpath
            extend(lastMoveToPointX, lastMoveToPointY)
            currentPointX = lastMoveToPointX
            currentPointY = lastMoveToPointY
        default:
            return .extendRectAndEndPoint
        }
        return .none
    }

    // Runs the endPoint callback for segment `pending` if it is not -1, then sets it to -1.
    @inline(__always) mutating func computeEndPoint(of pending: inout Int, _ endPoint: (_ segmentIndex: Int, _ currentPoint: UInt64, _ lastMoveToPoint: UInt64) -> (UInt64, UInt64)) {
        if pending >= 0 {
            (packedCurrentPoint, packedLastMoveToPoint) = endPoint(pending, packedCurrentPoint, packedLastMoveToPoint)
            pending = -1
        }
    }

    // The variant index of the segment whose words are `words`.
    @inline(__always) static func tag(_ words: Span<UInt32>) -> UInt8 {
        UInt8(truncatingIfNeeded: words[PathFastBounds.tagOffset / 4] >> (UInt32(PathFastBounds.tagOffset % 4) * 8))
    }

    // Whether the rect extension or end point (in extend(bySegment:) or the callbacks) of a segment of type `tag` reads
    // the points that the end point of the segment of type `pendingTag` sets. PathLineTo, PathQuadCurveTo,
    // PathBezierCurveTo and PathArcTo read the current point, PathCloseSubpath reads the last move-to point (which a
    // PathArcTo's end point does not change; PathCloseSubpath then sets the current point), unknown indices may read
    // both. Every other type sets both points without reading them.
    @inline(__always) static func readsPoints(tag: UInt8, pendingTag: UInt8) -> Bool {
        if tag == PathFastBounds.closeSubpath {
            return pendingTag != PathFastBounds.arcTo
        }
        let mask: UInt32 = 1 << PathFastBounds.moveTo | 1 << PathFastBounds.arc | 1 << PathFastBounds.closedArc | 1 << PathFastBounds.ellipse
            | 1 << PathFastBounds.ellipseInRect | 1 << PathFastBounds.rect | 1 << PathFastBounds.roundedRect
            | 1 << PathFastBounds.continuousRoundedRect | 1 << PathFastBounds.dataLine | 1 << PathFastBounds.dataQuadCurve
            | 1 << PathFastBounds.dataBezierCurve | 1 << PathFastBounds.dataArc
        return tag >= 32 || (mask >> tag) & 1 == 0
    }

    // std::min and std::max.
    @inline(__always) static func min(_ a: Float, _ b: Float) -> Float { b < a ? b : a }
    @inline(__always) static func max(_ a: Float, _ b: Float) -> Float { a < b ? b : a }

    // FloatRect::operator==.
    func hasSameRect(as other: Self) -> Bool {
        x == other.x && y == other.y && width == other.width && height == other.height
    }

    // FloatRect::shiftEdgesTo().
    @inline(__always) mutating func shiftEdges(left: Float, top: Float, right: Float, bottom: Float) {
        x = left
        y = top
        width = right - left
        height = bottom - top
    }

    // FloatRect::extend(FloatPoint).
    @inline(__always) mutating func extend(_ pointX: Float, _ pointY: Float) {
        extend(minX: pointX, minY: pointY, maxX: pointX, maxY: pointY)
    }

    // FloatRect::extend(FloatPoint minPoint, FloatPoint maxPoint).
    @inline(__always) mutating func extend(minX: Float, minY: Float, maxX: Float, maxY: Float) {
        shiftEdges(left: Self.min(x, minX), top: Self.min(y, minY), right: Self.max(x + width, maxX), bottom: Self.max(y + height, maxY))
    }

    // FloatRect::uniteEvenIfEmpty(FloatRect).
    @inline(__always) mutating func unite(_ otherX: Float, _ otherY: Float, _ otherWidth: Float, _ otherHeight: Float) {
        shiftEdges(left: Self.min(x, otherX), top: Self.min(y, otherY), right: Self.max(x + width, otherX + otherWidth),
            bottom: Self.max(y + height, otherY + otherHeight))
    }

    // extendRect(rect, a, b, c) in PathSegmentData.cpp: min3/max3 (GeometryUtilities.h) per axis.
    @inline(__always) mutating func extend(_ aX: Float, _ aY: Float, _ bX: Float, _ bY: Float, _ cX: Float, _ cY: Float) {
        extend(minX: Self.min(Self.min(aX, bX), cX), minY: Self.min(Self.min(aY, bY), cY),
            maxX: Self.max(Self.max(aX, bX), cX), maxY: Self.max(Self.max(aY, bY), cY))
    }

    // extendRect(rect, a, b, c, d) in PathSegmentData.cpp: min4/max4 per axis, min(min(a, b), min(c, d)).
    @inline(__always) mutating func extend(_ aX: Float, _ aY: Float, _ bX: Float, _ bY: Float, _ cX: Float, _ cY: Float,
        _ dX: Float, _ dY: Float)
    {
        extend(minX: Self.min(Self.min(aX, bX), Self.min(cX, dX)), minY: Self.min(Self.min(aY, bY), Self.min(cY, dY)),
            maxX: Self.max(Self.max(aX, bX), Self.max(cX, dX)), maxY: Self.max(Self.max(aY, bY), Self.max(cY, dY)))
    }

    // The current point and the last move-to point as the bit patterns of FloatPoints (x in the low 32 bits).
    var packedCurrentPoint: UInt64 {
        get { Self.pack(currentPointX, currentPointY) }
        set { (currentPointX, currentPointY) = Self.unpack(newValue) }
    }
    var packedLastMoveToPoint: UInt64 {
        get { Self.pack(lastMoveToPointX, lastMoveToPointY) }
        set { (lastMoveToPointX, lastMoveToPointY) = Self.unpack(newValue) }
    }
    @inline(__always) static func pack(_ x: Float, _ y: Float) -> UInt64 { UInt64(y.bitPattern) << 32 | UInt64(x.bitPattern) }
    @inline(__always) static func unpack(_ point: UInt64) -> (Float, Float) {
        (Float(bitPattern: UInt32(truncatingIfNeeded: point)), Float(bitPattern: UInt32(truncatingIfNeeded: point >> 32)))
    }

    // The WebCoreSwiftPathFastBoundsState field order.
    init(_ fields: Span<Float>) {
        precondition(fields.count == Self.count)
        self.init(currentPointX: fields[0], currentPointY: fields[1], lastMoveToPointX: fields[2], lastMoveToPointY: fields[3],
            x: fields[4], y: fields[5], width: fields[6], height: fields[7])
    }

    func store(into fields: inout MutableSpan<Float>) {
        precondition(fields.count == Self.count)
        fields[0] = currentPointX
        fields[1] = currentPointY
        fields[2] = lastMoveToPointX
        fields[3] = lastMoveToPointY
        fields[4] = x
        fields[5] = y
        fields[6] = width
        fields[7] = height
    }

    // Calls body with the state laid out as a WebCoreSwiftPathFastBoundsState, then reads it back.
    @inline(__always) mutating func withFields(_ body: (inout MutableSpan<Float>) -> Void) {
        var fields = InlineArray<8, Float>(repeating: 0)
        var span = fields.mutableSpan
        store(into: &span)
        body(&span)
        self = Self(span.span)
    }
}
