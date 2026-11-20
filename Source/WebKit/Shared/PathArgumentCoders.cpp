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

#include "config.h"

#include "ArgumentCoders.h"
#include "Encoder.h"
#include "IPCPathEncodeSwiftBoundary.h"
#include "StreamConnectionEncoder.h"
#include <WebCore/FloatRoundedRect.h>
#include <WebCore/Path.h>
#include <WebCore/PathImpl.h>
#include <WebCore/PathSegment.h>
#include <WebCore/PathSegmentData.h>
#include <WebCore/RotationDirection.h>
#include <array>
#include <bit>
#include <span>
#include <wtf/StdLibExtras.h>
#include <wtf/Vector.h>

// The encoders of WebCore::Path, WebCore::PathImpl and WebCore::PathSegment, for IPC::Encoder and
// IPC::StreamConnectionEncoder. Their serialization.in entries have CustomEncoder, and so do the alternatives of
// PathSegment::Data that are only encoded as part of a PathSegment: the encoding is in Swift
// (Source/WebKit/swift/IPCPathEncode/IPCPathEncodeSwiftBoundary.h documents the wire format, which the generated
// decoders and IPCPathDecode.swift decode). The C++ here only gets the segments, in place where the path stores them,
// and does the encoder's bookkeeping.

namespace IPC {

// In a namespace of its own, so that the using-directive stays out of the other files of a unified source bundle.
namespace PathArgumentCoders {

using namespace WebCore;

// The layout that the Swift encoder reads each PathSegment as: 14 32-bit words, the variant index in the low byte of
// word 13, an alternative's floats from word 0 on in the order its generated coder encoded them, and its 1-byte enum
// right after them.
static_assert(std::endian::native == std::endian::little);
static_assert(std::is_trivially_copyable_v<PathSegment>);
static_assert(alignof(PathSegment) == alignof(uint32_t));
static_assert(sizeof(PathSegment) == 14 * sizeof(uint32_t));
static_assert(sizeof(PathSegment) == sizeof(PathSegment::Data));
static_assert(sizeof(EncodedVariantIndex) == 1);

static consteval bool variantIndexIsInByte52()
{
    // PathCloseSubpath is the only empty alternative, so the only one that can be constructed in a constant expression.
    PathSegment::Data data { PathCloseSubpath { } };
    return std::bit_cast<std::array<uint8_t, sizeof(data)>>(data)[13 * sizeof(uint32_t)] == WTF::alternativeIndexV<PathCloseSubpath, PathSegment::Data>;
}
static_assert(variantIndexIsInByte52());

// The alternative indices, which are also the wire values (IPCPathEncode.swift's shape() has one case per shape).
static_assert(WTF::alternativeIndexV<PathMoveTo, PathSegment::Data> == 0);
static_assert(WTF::alternativeIndexV<PathLineTo, PathSegment::Data> == 1);
static_assert(WTF::alternativeIndexV<PathQuadCurveTo, PathSegment::Data> == 2);
static_assert(WTF::alternativeIndexV<PathBezierCurveTo, PathSegment::Data> == 3);
static_assert(WTF::alternativeIndexV<PathArcTo, PathSegment::Data> == 4);
static_assert(WTF::alternativeIndexV<PathArc, PathSegment::Data> == 5);
static_assert(WTF::alternativeIndexV<PathClosedArc, PathSegment::Data> == 6);
static_assert(WTF::alternativeIndexV<PathEllipse, PathSegment::Data> == 7);
static_assert(WTF::alternativeIndexV<PathEllipseInRect, PathSegment::Data> == 8);
static_assert(WTF::alternativeIndexV<PathRect, PathSegment::Data> == 9);
static_assert(WTF::alternativeIndexV<PathRoundedRect, PathSegment::Data> == 10);
static_assert(WTF::alternativeIndexV<PathContinuousRoundedRect, PathSegment::Data> == 11);
static_assert(WTF::alternativeIndexV<PathDataLine, PathSegment::Data> == 12);
static_assert(WTF::alternativeIndexV<PathDataQuadCurve, PathSegment::Data> == 13);
static_assert(WTF::alternativeIndexV<PathDataBezierCurve, PathSegment::Data> == 14);
static_assert(WTF::alternativeIndexV<PathDataArc, PathSegment::Data> == 15);
static_assert(WTF::alternativeIndexV<PathCloseSubpath, PathSegment::Data> == 16);
static_assert(WTF::VariantSizeV<PathSegment::Data> == 17);

// The geometry types are their floats in coder order: FloatPoint { x, y }, FloatSize { width, height }, FloatRect
// { location, size }.
static_assert(std::bit_cast<std::array<float, 2>>(FloatPoint(1, 2)) == std::array<float, 2> { 1, 2 });
static_assert(std::bit_cast<std::array<float, 2>>(FloatSize(1, 2)) == std::array<float, 2> { 1, 2 });
static_assert(std::bit_cast<std::array<float, 4>>(FloatRect(1, 2, 3, 4)) == std::array<float, 4> { 1, 2, 3, 4 });

// Each alternative's members, in coder order, from offset 0, with no gaps, then the 1-byte enums.
static constexpr size_t floatOffset(size_t index) { return index * sizeof(float); }
static_assert(!offsetof(PathMoveTo, point) && sizeof(PathMoveTo) == floatOffset(2));
static_assert(!offsetof(PathLineTo, point) && sizeof(PathLineTo) == floatOffset(2));
static_assert(!offsetof(PathQuadCurveTo, controlPoint) && offsetof(PathQuadCurveTo, endPoint) == floatOffset(2) && sizeof(PathQuadCurveTo) == floatOffset(4));
static_assert(!offsetof(PathBezierCurveTo, controlPoint1) && offsetof(PathBezierCurveTo, controlPoint2) == floatOffset(2) && offsetof(PathBezierCurveTo, endPoint) == floatOffset(4) && sizeof(PathBezierCurveTo) == floatOffset(6));
static_assert(!offsetof(PathArcTo, controlPoint1) && offsetof(PathArcTo, controlPoint2) == floatOffset(2) && offsetof(PathArcTo, radius) == floatOffset(4) && sizeof(PathArcTo) == floatOffset(5));
static_assert(!offsetof(PathArc, center) && offsetof(PathArc, radius) == floatOffset(2) && offsetof(PathArc, startAngle) == floatOffset(3) && offsetof(PathArc, endAngle) == floatOffset(4) && offsetof(PathArc, direction) == floatOffset(5));
static_assert(!offsetof(PathClosedArc, arc) && sizeof(PathClosedArc) == sizeof(PathArc));
static_assert(!offsetof(PathEllipse, center) && offsetof(PathEllipse, radiusX) == floatOffset(2) && offsetof(PathEllipse, radiusY) == floatOffset(3) && offsetof(PathEllipse, rotation) == floatOffset(4) && offsetof(PathEllipse, startAngle) == floatOffset(5) && offsetof(PathEllipse, endAngle) == floatOffset(6) && offsetof(PathEllipse, direction) == floatOffset(7));
static_assert(!offsetof(PathEllipseInRect, rect) && sizeof(PathEllipseInRect) == floatOffset(4));
static_assert(!offsetof(PathRect, rect) && sizeof(PathRect) == floatOffset(4));
// FloatRoundedRect is { FloatRect m_rect; CornerRadii m_radii; } and CornerRadii is { m_topLeft, m_topRight,
// m_bottomLeft, m_bottomRight }, the coder's order. The members are private, so that is checked at run time
// (floatRoundedRectHasCoderLayout(), in debug builds).
static_assert(std::is_standard_layout_v<FloatRoundedRect> && sizeof(FloatRoundedRect) == floatOffset(12));
static_assert(!offsetof(PathRoundedRect, roundedRect) && offsetof(PathRoundedRect, strategy) == floatOffset(12));
static_assert(!offsetof(PathContinuousRoundedRect, rect) && offsetof(PathContinuousRoundedRect, cornerWidth) == floatOffset(4) && offsetof(PathContinuousRoundedRect, cornerHeight) == floatOffset(5) && sizeof(PathContinuousRoundedRect) == floatOffset(6));
// PathDataLine is its std::array<float, 4> m_values (start, end), which span() returns.
static_assert(std::is_standard_layout_v<PathDataLine> && sizeof(PathDataLine) == floatOffset(4));
static_assert(!offsetof(PathDataQuadCurve, start) && offsetof(PathDataQuadCurve, controlPoint) == floatOffset(2) && offsetof(PathDataQuadCurve, endPoint) == floatOffset(4) && sizeof(PathDataQuadCurve) == floatOffset(6));
static_assert(!offsetof(PathDataBezierCurve, start) && offsetof(PathDataBezierCurve, controlPoint1) == floatOffset(2) && offsetof(PathDataBezierCurve, controlPoint2) == floatOffset(4) && offsetof(PathDataBezierCurve, endPoint) == floatOffset(6) && sizeof(PathDataBezierCurve) == floatOffset(8));
static_assert(!offsetof(PathDataArc, start) && offsetof(PathDataArc, controlPoint1) == floatOffset(2) && offsetof(PathDataArc, controlPoint2) == floatOffset(4) && offsetof(PathDataArc, radius) == floatOffset(6) && sizeof(PathDataArc) == floatOffset(7));
// The enums, which the coders encode as a uint8_t: RotationDirection (0 or 1) and PathRoundedRect::Strategy (its byte).
static_assert(std::is_same_v<std::underlying_type_t<RotationDirection>, bool> && sizeof(RotationDirection) == 1);
static_assert(std::is_same_v<std::underlying_type_t<PathRoundedRect::Strategy>, uint8_t>);

#if ASSERT_ENABLED
static bool floatRoundedRectHasCoderLayout()
{
    FloatRoundedRect roundedRect { FloatRect { 1, 2, 3, 4 }, FloatSize { 5, 6 }, FloatSize { 7, 8 }, FloatSize { 9, 10 }, FloatSize { 11, 12 } };
    return std::bit_cast<std::array<float, 12>>(roundedRect) == std::array<float, 12> { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
}
#endif

// The segments of a Path or PathImpl, as their segments() returns them, but without a copy where they are stored:
// a single segment, or a PathStream's. Only a platform path (PathCG) is copied, from its CGPath.
class PathSegmentsForEncoding {
public:
    explicit PathSegmentsForEncoding(const Path& path)
    {
        if (const auto* segment = path.singleSegmentIfExists())
            m_segments = singleElementSpan(*segment);
        else if (const auto* segments = path.segmentsIfExists())
            m_segments = segments->span();
        else {
            m_copy = path.segments();
            m_segments = m_copy.span();
        }
    }

    explicit PathSegmentsForEncoding(const PathImpl& impl)
    {
        if (const auto* segments = impl.segmentsIfExists())
            m_segments = segments->span();
        else {
            m_copy = impl.segments();
            m_segments = m_copy.span();
        }
    }

    std::span<const PathSegment> span() const LIFETIME_BOUND { return m_segments; }

private:
    std::span<const PathSegment> m_segments;
    Vector<PathSegment> m_copy;
};

// withCount: a Vector<PathSegment> (a Path or PathImpl), whose size is encoded first; otherwise a single PathSegment.
static void encodeSegments(StreamConnectionEncoder& encoder, std::span<const PathSegment> segments, bool withCount)
{
    ASSERT(floatRoundedRectHasCoderLayout());
    encoder.encodeInPlace([&](std::span<uint8_t> buffer) {
        return WebKitSwift_IPCPathEncode_encodeToStream(segments.data(), segments.size(), withCount, buffer.data(), buffer.size());
    });
}

static void encodeSegments(Encoder& encoder, std::span<const PathSegment> segments, bool withCount)
{
    ASSERT(floatRoundedRectHasCoderLayout());
    size_t bodySize = encoder.bodySize();
    size_t size = WebKitSwift_IPCPathEncode_encodedSize(segments.data(), segments.size(), withCount, bodySize);
    auto bytes = encoder.appendForInPlaceEncoding(size);
    WebKitSwift_IPCPathEncode_encodeToBody(segments.data(), segments.size(), withCount, bodySize, bytes.data(), bytes.size());
}

} // namespace PathArgumentCoders

void ArgumentCoder<WebCore::Path>::encode(Encoder& encoder, const WebCore::Path& path)
{
    PathArgumentCoders::PathSegmentsForEncoding segments { path };
    PathArgumentCoders::encodeSegments(encoder, segments.span(), true);
}

void ArgumentCoder<WebCore::Path>::encode(StreamConnectionEncoder& encoder, const WebCore::Path& path)
{
    PathArgumentCoders::PathSegmentsForEncoding segments { path };
    PathArgumentCoders::encodeSegments(encoder, segments.span(), true);
}

void ArgumentCoder<WebCore::PathImpl>::encode(Encoder& encoder, const WebCore::PathImpl& impl)
{
    PathArgumentCoders::PathSegmentsForEncoding segments { impl };
    PathArgumentCoders::encodeSegments(encoder, segments.span(), true);
}

void ArgumentCoder<WebCore::PathImpl>::encode(StreamConnectionEncoder& encoder, const WebCore::PathImpl& impl)
{
    PathArgumentCoders::PathSegmentsForEncoding segments { impl };
    PathArgumentCoders::encodeSegments(encoder, segments.span(), true);
}

void ArgumentCoder<WebCore::PathSegment>::encode(Encoder& encoder, const WebCore::PathSegment& segment)
{
    PathArgumentCoders::encodeSegments(encoder, singleElementSpan(segment), false);
}

void ArgumentCoder<WebCore::PathSegment>::encode(StreamConnectionEncoder& encoder, const WebCore::PathSegment& segment)
{
    PathArgumentCoders::encodeSegments(encoder, singleElementSpan(segment), false);
}

} // namespace IPC
