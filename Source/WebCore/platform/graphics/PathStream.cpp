/*
 * Copyright (C) 2023 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "PathStream.h"

#include "GeometryUtilities.h"
#include <wtf/TZoneMallocInlines.h>

#if USE(SWIFT_KERNELS)
#include "PathFastBoundsSwiftBoundary.h"
#include <array>
#include <bit>
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

namespace WebCore {

WTF_MAKE_TZONE_ALLOCATED_IMPL(PathStream);

#if USE(SWIFT_KERNELS)
// The Swift port of computeFastBoundingRect (Source/WebCore/swift/PathFastBounds) reads the PathSegment array as 32-bit
// words: the variant index from the byte at WebCoreSwiftPathFastBoundsTagOffset, and the floats of the alternatives it
// handles from word 0 on. These assertions pin that layout; validation mode also checks it for every segment.
static_assert(std::endian::native == std::endian::little);
static_assert(std::is_trivially_copyable_v<PathSegment>);
static_assert(alignof(PathSegment) == alignof(uint32_t));
static_assert(sizeof(PathSegment) == WebCoreSwiftPathFastBoundsSegmentSizeInWords * sizeof(uint32_t));
static_assert(sizeof(PathSegment) == sizeof(PathSegment::Data));

static consteval bool variantIndexIsAtTagOffset()
{
    // PathCloseSubpath is the only empty alternative, so the only one that can be constructed in a constant expression.
    PathSegment::Data data { PathCloseSubpath { } };
    return std::bit_cast<std::array<uint8_t, sizeof(data)>>(data)[WebCoreSwiftPathFastBoundsTagOffset] == WTF::alternativeIndexV<PathCloseSubpath, PathSegment::Data>;
}
static_assert(variantIndexIsAtTagOffset());

static_assert(WTF::alternativeIndexV<PathMoveTo, PathSegment::Data> == WebCoreSwiftPathFastBoundsMoveTo);
static_assert(WTF::alternativeIndexV<PathLineTo, PathSegment::Data> == WebCoreSwiftPathFastBoundsLineTo);
static_assert(WTF::alternativeIndexV<PathQuadCurveTo, PathSegment::Data> == WebCoreSwiftPathFastBoundsQuadCurveTo);
static_assert(WTF::alternativeIndexV<PathBezierCurveTo, PathSegment::Data> == WebCoreSwiftPathFastBoundsBezierCurveTo);
static_assert(WTF::alternativeIndexV<PathArcTo, PathSegment::Data> == WebCoreSwiftPathFastBoundsArcTo);
static_assert(WTF::alternativeIndexV<PathArc, PathSegment::Data> == WebCoreSwiftPathFastBoundsArc);
static_assert(WTF::alternativeIndexV<PathClosedArc, PathSegment::Data> == WebCoreSwiftPathFastBoundsClosedArc);
static_assert(WTF::alternativeIndexV<PathEllipse, PathSegment::Data> == WebCoreSwiftPathFastBoundsEllipse);
static_assert(WTF::alternativeIndexV<PathEllipseInRect, PathSegment::Data> == WebCoreSwiftPathFastBoundsEllipseInRect);
static_assert(WTF::alternativeIndexV<PathRect, PathSegment::Data> == WebCoreSwiftPathFastBoundsRect);
static_assert(WTF::alternativeIndexV<PathRoundedRect, PathSegment::Data> == WebCoreSwiftPathFastBoundsRoundedRect);
static_assert(WTF::alternativeIndexV<PathContinuousRoundedRect, PathSegment::Data> == WebCoreSwiftPathFastBoundsContinuousRoundedRect);
static_assert(WTF::alternativeIndexV<PathDataLine, PathSegment::Data> == WebCoreSwiftPathFastBoundsDataLine);
static_assert(WTF::alternativeIndexV<PathDataQuadCurve, PathSegment::Data> == WebCoreSwiftPathFastBoundsDataQuadCurve);
static_assert(WTF::alternativeIndexV<PathDataBezierCurve, PathSegment::Data> == WebCoreSwiftPathFastBoundsDataBezierCurve);
static_assert(WTF::alternativeIndexV<PathDataArc, PathSegment::Data> == WebCoreSwiftPathFastBoundsDataArc);
static_assert(WTF::alternativeIndexV<PathCloseSubpath, PathSegment::Data> == WebCoreSwiftPathFastBoundsCloseSubpath);
static_assert(WTF::VariantSizeV<PathSegment::Data> == WebCoreSwiftPathFastBoundsCloseSubpath + 1);

// The fields Swift reads: FloatPoint is { x, y } and FloatRect is { x, y, width, height }.
static_assert(std::bit_cast<std::array<float, 2>>(FloatPoint(1, 2)) == std::array<float, 2> { 1, 2 });
static_assert(std::bit_cast<std::array<float, 4>>(FloatRect(1, 2, 3, 4)) == std::array<float, 4> { 1, 2, 3, 4 });
static_assert(sizeof(FloatPoint) == sizeof(uint64_t) && std::is_trivially_copyable_v<FloatPoint>); // The endPoint callback's encoding.
static_assert(!offsetof(PathMoveTo, point));
static_assert(!offsetof(PathLineTo, point));
static_assert(!offsetof(PathQuadCurveTo, controlPoint) && offsetof(PathQuadCurveTo, endPoint) == 2 * sizeof(float));
static_assert(!offsetof(PathBezierCurveTo, controlPoint1) && offsetof(PathBezierCurveTo, controlPoint2) == 2 * sizeof(float) && offsetof(PathBezierCurveTo, endPoint) == 4 * sizeof(float));
static_assert(!offsetof(PathArcTo, controlPoint1) && offsetof(PathArcTo, controlPoint2) == 2 * sizeof(float));
static_assert(!offsetof(PathArc, center) && offsetof(PathArc, radius) == 2 * sizeof(float));
static_assert(!offsetof(PathClosedArc, arc));
static_assert(!offsetof(PathEllipse, center) && offsetof(PathEllipse, radiusX) == 2 * sizeof(float) && offsetof(PathEllipse, radiusY) == 3 * sizeof(float) && offsetof(PathEllipse, rotation) == 4 * sizeof(float));
static_assert(!offsetof(PathEllipseInRect, rect));
static_assert(!offsetof(PathRect, rect));
// FloatRoundedRect is { FloatRect m_rect; CornerRadii m_radii; } with private members, so that m_rect comes first (Swift
// reads roundedRect.rect() from word 0) is checked in validation mode (validateFastBoundingRect) rather than here.
static_assert(!offsetof(PathRoundedRect, roundedRect));
static_assert(std::is_standard_layout_v<FloatRoundedRect> && sizeof(FloatRoundedRect) == sizeof(FloatRect) + sizeof(CornerRadii));
static_assert(!offsetof(PathContinuousRoundedRect, rect));
static_assert(std::is_standard_layout_v<PathDataLine> && sizeof(PathDataLine) == 4 * sizeof(float)); // m_values: start, end.
static_assert(!offsetof(PathDataQuadCurve, start) && offsetof(PathDataQuadCurve, controlPoint) == 2 * sizeof(float) && offsetof(PathDataQuadCurve, endPoint) == 4 * sizeof(float));
static_assert(!offsetof(PathDataBezierCurve, start) && offsetof(PathDataBezierCurve, controlPoint1) == 2 * sizeof(float) && offsetof(PathDataBezierCurve, controlPoint2) == 4 * sizeof(float) && offsetof(PathDataBezierCurve, endPoint) == 6 * sizeof(float));
static_assert(!offsetof(PathDataArc, start) && offsetof(PathDataArc, controlPoint1) == 2 * sizeof(float) && offsetof(PathDataArc, controlPoint2) == 4 * sizeof(float));
#endif

Ref<PathStream> PathStream::create()
{
    return adoptRef(*new PathStream);
}

Ref<PathStream> PathStream::create(PathSegment&& segment)
{
    return adoptRef(*new PathStream(WTF::move(segment)));
}

Ref<PathStream> PathStream::create(Vector<PathSegment>&& segments, IsTransient isTransient)
{
    return adoptRef(*new PathStream(WTF::move(segments), isTransient));
}

Ref<PathStream> PathStream::create(const Vector<PathSegment>& segments)
{
    return adoptRef(*new PathStream(segments));
}

Ref<PathStream> PathStream::create(const Vector<FloatPoint>& points)
{
    auto stream = PathStream::create();
    if (points.size() < 2)
        return stream;

    stream->add(PathMoveTo { points[0] });
    for (size_t i = 1; i < points.size(); ++i)
        stream->add(PathLineTo { points[i] });

    stream->add(PathCloseSubpath { });
    return stream;
}

PathStream::PathStream(Vector<PathSegment>&& segments, IsTransient isTransient)
    : m_segments(WTF::move(segments))
    , m_isTransient(isTransient)
{
}

PathStream::PathStream(const Vector<PathSegment>& segments)
    : m_segments(segments)
{
}

PathStream::PathStream(PathSegment&& segment)
{
    m_segments.reserveCapacity(16); // 16 is Vector's minCapacity, and we know we're going to append a second segment.
    m_segments.append(WTF::move(segment));
}

bool PathStream::definitelyEqual(const PathImpl& other) const
{
    RefPtr otherAsPathStream = dynamicDowncast<PathStream>(other);
    if (!otherAsPathStream)
        return false;

    if (otherAsPathStream.get() == this)
        return true;

    return segments() == otherAsPathStream->segments();
}

Ref<PathImpl> PathStream::copy() const
{
    return create(m_segments);
}

const PathMoveTo* PathStream::lastIfMoveTo() const
{
    if (m_segments.isEmpty())
        return nullptr;

    return std::get_if<PathMoveTo>(&m_segments.last().data());
}

void PathStream::add(PathMoveTo moveTo)
{
    segments().append(moveTo);
}

void PathStream::add(PathLineTo lineTo)
{
    if (const auto* moveTo = lastIfMoveTo())
        segments().last() = { PathDataLine { moveTo->point, lineTo.point } };
    else
        segments().append(lineTo);
}

void PathStream::add(PathQuadCurveTo quadTo)
{
    if (const auto* moveTo = lastIfMoveTo())
        segments().last() = { PathDataQuadCurve { moveTo->point, quadTo.controlPoint, quadTo.endPoint } };
    else
        segments().append(quadTo);
}

void PathStream::add(PathBezierCurveTo bezierTo)
{
    if (const auto* moveTo = lastIfMoveTo())
        segments().last() = { PathDataBezierCurve { moveTo->point, bezierTo.controlPoint1, bezierTo.controlPoint2, bezierTo.endPoint } };
    else
        segments().append(bezierTo);
}

void PathStream::add(PathArcTo arcTo)
{
    if (const auto* moveTo = lastIfMoveTo())
        segments().last() = { PathDataArc { moveTo->point, arcTo.controlPoint1, arcTo.controlPoint2, arcTo.radius } };
    else
        segments().append(arcTo);
}

void PathStream::add(PathArc arc)
{
    segments().append(arc);
}

void PathStream::add(PathClosedArc closedArc)
{
    segments().append(closedArc);
}

void PathStream::add(PathRect rect)
{
    segments().append(rect);
}

void PathStream::add(PathEllipse ellipse)
{
    segments().append(ellipse);
}

void PathStream::add(PathEllipseInRect ellipseInRect)
{
    segments().append(ellipseInRect);
}

void PathStream::add(PathRoundedRect roundedRect)
{
    segments().append(roundedRect);
}

void PathStream::add(PathContinuousRoundedRect continuousRoundedRect)
{
    segments().append(continuousRoundedRect);
}

void PathStream::add(PathCloseSubpath)
{
    segments().append(PathCloseSubpath { });
}

void PathStream::applySegments(NOESCAPE const PathSegmentApplier& applier) const
{
    for (auto& segment : m_segments)
        applier(segment);
}

bool PathStream::applyElements(NOESCAPE const PathElementApplier& applier) const
{
    for (auto& segment : m_segments) {
        if (!segment.canApplyElements())
            return false;
    }

    for (auto& segment : m_segments)
        segment.applyElements(applier);

    return true;
}

bool PathStream::transform(const AffineTransform& transform)
{
    for (auto& segment : m_segments) {
        if (!segment.canTransform())
            return false;
    }

    for (auto& segment : m_segments)
        segment.transform(transform);

    return true;
}

std::optional<PathSegment> PathStream::singleSegment() const
{
    if (m_segments.size() != 1)
        return std::nullopt;

    return m_segments.first();
}


bool PathStream::isClosed() const
{
    if (m_segments.isEmpty())
        return false;

    return m_segments.last().closesSubpath();
}

FloatPoint PathStream::currentPoint() const
{
    if (m_segments.isEmpty())
        return { };

    if (auto result = m_segments.last().tryGetEndPointWithoutContext())
        return result.value();

    FloatPoint lastMoveToPoint;
    FloatPoint currentPoint;

    for (auto& segment : m_segments)
        currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint);

    return currentPoint;
}

#if !USE(SWIFT_KERNELS) || ENABLE(SWIFT_KERNELS_VALIDATION)
static FloatRect computeFastBoundingRectCxx(std::span<const PathSegment> segments)
{
    FloatPoint lastMoveToPoint;
    FloatPoint currentPoint;
    FloatRect boundingRect = FloatRect::smallestRect();

    for (auto& segment : segments) {
        segment.extendFastBoundingRect(currentPoint, lastMoveToPoint, boundingRect);
        currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint);
    }

    if (boundingRect.isSmallest())
        boundingRect.extend(currentPoint);

    return boundingRect;
}
#endif

#if USE(SWIFT_KERNELS)
// The end point part of the loop body of computeFastBoundingRectCxx(), for the segment types whose end point needs
// trigonometry (Swift extends the rect). context points to the std::span<const PathSegment>.
static uint64_t computeFastBoundingRectEndPoint(const void* context, size_t segmentIndex, uint64_t currentPoint, uint64_t* lastMoveToPointBits)
{
    auto& segment = (*static_cast<const std::span<const PathSegment>*>(context))[segmentIndex];
    auto lastMoveToPoint = std::bit_cast<FloatPoint>(*lastMoveToPointBits);

    auto endPoint = segment.calculateEndPoint(std::bit_cast<FloatPoint>(currentPoint), lastMoveToPoint);

    *lastMoveToPointBits = std::bit_cast<uint64_t>(lastMoveToPoint);
    return std::bit_cast<uint64_t>(endPoint);
}

// The rect part of the loop body of computeFastBoundingRectCxx(), for the segments whose rect extension the Swift code
// leaves to C++ (a rotated PathEllipse). context points to the std::span<const PathSegment>.
static void computeFastBoundingRectExtendRect(const void* context, size_t segmentIndex, WebCoreSwiftPathFastBoundsState* state)
{
    auto& segment = (*static_cast<const std::span<const PathSegment>*>(context))[segmentIndex];
    FloatPoint lastMoveToPoint { state->lastMoveToPointX, state->lastMoveToPointY };
    FloatPoint currentPoint { state->currentPointX, state->currentPointY };
    FloatRect boundingRect { state->x, state->y, state->width, state->height };

    segment.extendFastBoundingRect(currentPoint, lastMoveToPoint, boundingRect);

    state->x = boundingRect.x();
    state->y = boundingRect.y();
    state->width = boundingRect.width();
    state->height = boundingRect.height();
}

#if ENABLE(SWIFT_KERNELS_VALIDATION)
static void validateFastBoundingRect(std::span<const PathSegment> segments, const FloatRect& result)
{
    for (auto& segment : segments) {
        RELEASE_ASSERT(asByteSpan(segment)[WebCoreSwiftPathFastBoundsTagOffset] == segment.data().index());
        WTF::switchOn(segment.data(), [&](auto& alternative) {
            RELEASE_ASSERT(static_cast<const void*>(&alternative) == static_cast<const void*>(&segment));
            if constexpr (std::is_same_v<std::remove_cvref_t<decltype(alternative)>, PathRoundedRect>)
                RELEASE_ASSERT(static_cast<const void*>(&alternative.roundedRect.rect()) == static_cast<const void*>(&segment));
        });
    }
    // Bitwise, so that NaN results compare equal to themselves.
    using Bits = std::array<uint32_t, 4>;
    RELEASE_ASSERT(std::bit_cast<Bits>(result) == std::bit_cast<Bits>(computeFastBoundingRectCxx(segments)));
}
#endif
#endif

FloatRect PathStream::computeFastBoundingRect(std::span<const PathSegment> segments)
{
#if USE(SWIFT_KERNELS)
    auto words = spanReinterpretCast<const uint32_t>(segments);
    auto smallestRect = FloatRect::smallestRect();
    WebCoreSwiftPathFastBoundsState state { 0, 0, 0, 0, smallestRect.x(), smallestRect.y(), smallestRect.width(), smallestRect.height() };
    WebCoreSwift_PathFastBounds_computeFastBoundingRect(words.data(), words.size(), sizeof(PathSegment) / sizeof(uint32_t), WebCoreSwiftPathFastBoundsTagOffset, &segments, computeFastBoundingRectEndPoint, computeFastBoundingRectExtendRect, &state);
    FloatRect result { state.x, state.y, state.width, state.height };
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]]
        validateFastBoundingRect(segments, result);
#endif
    return result;
#else
    return computeFastBoundingRectCxx(segments);
#endif
}

bool PathStream::computeHasSubpaths(std::span<const PathSegment> segments)
{
    FloatPoint lastMoveToPoint;
    FloatPoint currentPoint;
    FloatRect boundingRect = FloatRect::smallestRect();

    for (auto& segment : segments) {
        segment.extendFastBoundingRect(currentPoint, lastMoveToPoint, boundingRect);
        if (!boundingRect.isSmallest() && (boundingRect.height() || boundingRect.width()))
            return true;
        currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint);
    }

    if (boundingRect.isSmallest())
        boundingRect.extend(currentPoint);

    return boundingRect.height() || boundingRect.width();
}

bool PathStream::hasSubpaths() const
{
    return computeHasSubpaths(m_segments.span());
}

FloatRect PathStream::fastBoundingRect() const
{
    return computeFastBoundingRect(m_segments.span());
}

FloatRect PathStream::computeBoundingRect(std::span<const PathSegment> segments)
{
    FloatPoint lastMoveToPoint;
    FloatPoint currentPoint;
    FloatRect boundingRect = FloatRect::smallestRect();

    for (auto& segment : segments) {
        segment.extendBoundingRect(currentPoint, lastMoveToPoint, boundingRect);
        currentPoint = segment.calculateEndPoint(currentPoint, lastMoveToPoint);
    }

    if (boundingRect.isSmallest())
        boundingRect.extend(currentPoint);

    return boundingRect;
}

FloatRect PathStream::boundingRect() const
{
    return computeBoundingRect(m_segments.span());
}

} // namespace WebCore
