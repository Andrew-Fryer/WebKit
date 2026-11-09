/*
 * Copyright (C) 2010-2024 Apple Inc. All rights reserved.
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
#include "Region.h"

#include <stdio.h>
#include <wtf/NeverDestroyed.h>
#include <wtf/TZoneMallocInlines.h>
#include <wtf/text/ParsingUtilities.h>
#include <wtf/text/TextStream.h>

#include "RegionShapeSwiftBoundary.h"
#include <bit>

// A region class based on the paper "Scanline Coherent Shape Algebra"
// by Jonathan E. Steinhart from the book "Graphics Gems II".
//
// This implementation uses two vectors instead of linked list, and
// also compresses regions when possible.

namespace WebCore {

DEFINE_ALLOCATOR_WITH_HEAP_IDENTIFIER(Region);

WTF_MAKE_TZONE_ALLOCATED_IMPL(Region::Shape);

// The Swift port of Region::Shape's algebra (Source/WebCore/swift/RegionShape) reads and writes Region::Span arrays as
// uint64_t words: y in the low 32 bits of the first word of each span, segmentIndex in the second.
static_assert(sizeof(Region::Span) == 2 * sizeof(uint64_t));
static_assert(!offsetof(Region::Span, y) && offsetof(Region::Span, segmentIndex) == sizeof(uint64_t));
static_assert(sizeof(int) == sizeof(int32_t));
static_assert(std::endian::native == std::endian::little);

static std::span<const uint64_t> regionSpanWords(std::span<const Region::Span> spans)
{
    return spanReinterpretCast<const uint64_t>(spans);
}

Region::Region() = default;

Region::Region(const IntRect& rect)
    : m_bounds(rect)
{
}

Region::Region(const Region& other)
    : m_bounds(other.m_bounds)
    , m_shape(other.copyShape())
{
}

Region::Region(Region&&) = default;

Region::~Region() = default;

Region& Region::operator=(const Region& other)
{
    m_bounds = other.m_bounds;
    m_shape = other.copyShape();
    return *this;
}

Region& Region::operator=(Region&&) = default;

Vector<IntRect, 1> Region::rects() const
{
    Vector<IntRect, 1> rects;

    if (!m_shape) {
        if (!m_bounds.isEmpty())
            rects.append(m_bounds);
        return rects;
    }

    for (auto spans = m_shape->spans(); spans.size() > 1; skip(spans, 1)) {
        int y = spans[0].y;
        int height = spans[1].y - y; // Ok since isValidShape ensures increasing Span::y.

        for (auto segments = m_shape->segments(spans); segments.size() > 1; skip(segments, 2)) {
            int x = segments[0];
            int width = segments[1] - x; // Ok since isValidShape ensures increasing segments.

            rects.append(IntRect(x, y, width, height));
        }
    }

    return rects;
}

bool Region::contains(const Region& region) const
{
    if (!m_bounds.contains(region.m_bounds))
        return false;

    if (!m_shape)
        return true;

    return Shape::compareShapes<Shape::CompareContainsOperation>(*m_shape, region.m_shape ? *region.m_shape : Shape(region.m_bounds));
}

bool Region::contains(const IntPoint& point) const
{
    if (!m_bounds.contains(point))
        return false;

    if (!m_shape)
        return true;

    for (auto spans = m_shape->spans(); spans.size() > 1; skip(spans, 1)) {
        int y = spans[0].y;
        int maxY = spans[1].y;

        if (y > point.y())
            break;
        if (maxY <= point.y())
            continue;

        for (auto segments = m_shape->segments(spans); segments.size() > 1; skip(segments, 2)) {
            int x = segments[0];
            int maxX = segments[1];

            if (x > point.x())
                break;
            if (maxX > point.x())
                return true;
        }
    }

    return false;
}

bool Region::intersects(const Region& region) const
{
    if (!m_bounds.intersects(region.m_bounds))
        return false;

    if (!m_shape && !region.m_shape)
        return true;

    return Shape::compareShapes<Shape::CompareIntersectsOperation>(m_shape ? *m_shape : m_bounds, region.m_shape ? *region.m_shape : region.m_bounds);
}

uint64_t Region::totalArea() const
{
    uint64_t totalArea = 0;

    for (auto& rect : rects())
        totalArea += static_cast<uint64_t>(rect.width()) * rect.height();

    return totalArea;
}

template<typename CompareOperation>
bool Region::Shape::compareShapes(const Shape& aShape, const Shape& bShape)
{
    static_assert(std::is_same_v<CompareOperation, CompareContainsOperation> || std::is_same_v<CompareOperation, CompareIntersectsOperation>);
    constexpr auto compare = std::is_same_v<CompareOperation, CompareContainsOperation> ? WebCoreSwift_RegionShape_contains : WebCoreSwift_RegionShape_intersects;
    auto aWords = regionSpanWords(aShape.spans());
    auto bWords = regionSpanWords(bShape.spans());
    return compare(aShape.m_segments.span().data(), aShape.m_segments.size(), aWords.data(), aWords.size(), bShape.m_segments.span().data(), bShape.m_segments.size(), bWords.data(), bWords.size());
}

Region::Shape::Shape(const IntRect& rect)
{
    if (rect.isEmpty())
        return;
    m_segments.append(rect.x());
    m_segments.append(rect.maxX());
    m_spans.append({ rect.y(), 0 });
    m_spans.append({ rect.maxY(), 2 });
}

Region::Shape::Shape(Vector<int, 32>&& segments, Vector<Span, 16>&& spans)
    : m_segments(WTF::move(segments))
    , m_spans(WTF::move(spans))
{
    ASSERT(isValidShape(m_segments.span(), m_spans.span()));
}

std::span<const int> Region::Shape::segments(std::span<const Span> span) const LIFETIME_BOUND
{
    ASSERT(span.data() >= std::to_address(m_spans.begin()));
    ASSERT(span.data() < std::to_address(m_spans.end()));
    ASSERT(std::to_address(span.end()) <= std::to_address(m_spans.end()));

    // Check if this span has any segments.
    if (span[0].segmentIndex == m_segments.size())
        return { };

    return m_segments.subspan(span[0].segmentIndex, span[1].segmentIndex - span[0].segmentIndex);
}

WTF::TextStream& operator<<(WTF::TextStream& ts, const Region::Shape& value)
{
    ts << '\n';
    TextStream::IndentScope indentScope(ts);
    ts << indent;
    for (auto spans = value.spans(); !spans.empty(); skip(spans, 1)) {
        ts << "y: "_s << spans[0].y << " spans: ("_s;
        int comma = 0;
        for (auto& segment : value.segments(spans))
            ts << (comma++ > 0 ? ", "_s : ""_s) << segment;
        ts << ")\n"_s;
    }
    ts << "spans: ("_s;
    for (size_t i = 0; i < value.m_spans.size(); ++i)
        ts << (i > 0 ? ", "_s : ""_s) << "y: "_s << value.m_spans[i].y << " si: "_s << value.m_spans[i].segmentIndex;
    ts << ")\n"_s << "segments: ("_s;
    for (size_t i = 0; i < value.m_segments.size(); ++i)
        ts << (i > 0 ? ", "_s : ""_s) << value.m_segments[i];
    ts << ")\n"_s;
    return ts;
}

IntRect Region::Shape::bounds() const
{
    if (isEmpty())
        return IntRect();

    auto words = regionSpanWords(spans());
    std::array<int32_t, 4> rect;
    WebCoreSwift_RegionShape_bounds(m_segments.span().data(), m_segments.size(), words.data(), words.size(), rect.data());
    return IntRect(rect[0], rect[1], rect[2], rect[3]);
}

void Region::Shape::translate(const IntSize& offset)
{
    for (size_t i = 0; i < m_segments.size(); ++i)
        m_segments[i] += offset.width();
    for (size_t i = 0; i < m_spans.size(); ++i)
        m_spans[i].y += offset.height();
}

// The scanline merge of unionShapes/intersectShapes/subtractShapes (formerly the C++ shapeOperation<Operation>), in Swift.
// resultSegments and resultSpans are empty.
using ShapeOperationSwift = size_t(const int32_t*, size_t, const uint64_t*, size_t, const int32_t*, size_t, const uint64_t*, size_t, int32_t*, size_t, uint64_t*, size_t, size_t*);

ALWAYS_INLINE static void shapeOperationSwift(ShapeOperationSwift& operation, std::span<const int> segments1, std::span<const Region::Span> spans1, std::span<const int> segments2, std::span<const Region::Span> spans2, Vector<int, 32>& resultSegments, Vector<Region::Span, 16>& resultSpans)
{
    // Each iteration of shapeOperation consumes at least one span of either shape and appends at most one span, so the
    // result has at most spans1.size() + spans2.size() spans. There is no such bound on segments (a span's segments are
    // merged again with every span of the other shape that it overlaps): start with the capacity that shapeOperation
    // reserves and grow it whenever Swift asks for more. Reserving first keeps the capacities those of shapeOperation.
    resultSpans.reserveInitialCapacity(spans1.size() + spans2.size());
    resultSpans.grow(spans1.size() + spans2.size());
    resultSegments.reserveInitialCapacity(segments1.size() + segments2.size());
    resultSegments.grow(resultSegments.capacity());

    auto words1 = regionSpanWords(spans1);
    auto words2 = regionSpanWords(spans2);
    auto resultWords = spanReinterpretCast<uint64_t>(resultSpans.mutableSpan());
    std::array<size_t, WebCoreSwiftRegionShapeStateSize> state { };
    while (size_t segmentsNeeded = operation(segments1.data(), segments1.size(), words1.data(), words1.size(), segments2.data(), segments2.size(), words2.data(), words2.size(), resultSegments.mutableSpan().data(), resultSegments.size(), resultWords.data(), resultWords.size(), state.data())) {
        resultSegments.grow(segmentsNeeded);
        resultSegments.grow(resultSegments.capacity());
    }

    size_t segmentCount = state[WebCoreSwiftRegionShapeStateSegmentCount];
    size_t spanCount = state[WebCoreSwiftRegionShapeStateSpanCount];
    RELEASE_ASSERT(segmentCount <= resultSegments.size() && spanCount <= resultSpans.size());
    resultSegments.shrink(segmentCount);
    resultSpans.shrink(spanCount);
}

Region::Shape Region::Shape::unionShapes(const Shape& shape1, const Shape& shape2)
{
    if (shape1.isEmpty())
        return shape2;

    Shape result;
    shapeOperationSwift(WebCoreSwift_RegionShape_unite, shape1.m_segments.span(), shape1.spans(), shape2.m_segments.span(), shape2.spans(), result.m_segments, result.m_spans);
    return result;
}

Region::Shape Region::Shape::intersectShapes(const Shape& shape1, const Shape& shape2)
{
    Shape result;
    shapeOperationSwift(WebCoreSwift_RegionShape_intersect, shape1.m_segments.span(), shape1.spans(), shape2.m_segments.span(), shape2.spans(), result.m_segments, result.m_spans);
    return result;
}

Region::Shape Region::Shape::subtractShapes(const Shape& shape1, const Shape& shape2)
{
    Shape result;
    shapeOperationSwift(WebCoreSwift_RegionShape_subtract, shape1.m_segments.span(), shape1.spans(), shape2.m_segments.span(), shape2.spans(), result.m_segments, result.m_spans);
    return result;
}

void Region::intersect(const Region& region)
{
    if (m_bounds.isEmpty())
        return;
    if (!m_bounds.intersects(region.m_bounds)) {
        m_shape = nullptr;
        m_bounds = IntRect();
        return;
    }
    if (!m_shape && !region.m_shape) {
        m_bounds = intersection(m_bounds, region.m_bounds);
        return;
    }

    setShape(Shape::intersectShapes(m_shape ? *m_shape : m_bounds, region.m_shape ? *region.m_shape : region.m_bounds));
}

void Region::unite(const Region& region)
{
    if (region.isEmpty())
        return;
    if (isEmpty()) {
        m_bounds = region.m_bounds;
        m_shape = region.copyShape();
        return;
    }
    if (region.isRect() && region.m_bounds.contains(m_bounds)) {
        m_bounds = region.m_bounds;
        m_shape = nullptr;
        return;
    }
    if (contains(region))
        return;

    setShape(Shape::unionShapes(m_shape ? *m_shape : m_bounds, region.m_shape ? *region.m_shape : region.m_bounds));
}

void Region::subtract(const Region& region)
{
    if (isEmpty())
        return;
    if (region.isEmpty())
        return;
    if (!m_bounds.intersects(region.m_bounds))
        return;

    setShape(Shape::subtractShapes(m_shape ? *m_shape : m_bounds, region.m_shape ? *region.m_shape : region.m_bounds));
}

void Region::translate(const IntSize& offset)
{
    m_bounds.move(offset);
    if (m_shape)
        m_shape->translate(offset);
}

void Region::setShape(Shape&& shape)
{
    m_bounds = shape.bounds();

    if (shape.isRect()) {
        m_shape = nullptr;
        return;
    }

    if (!m_shape)
        m_shape = makeUnique<Shape>(WTF::move(shape));
    else
        *m_shape = WTF::move(shape);
}

bool Region::Shape::isValidShape(std::span<const int> segments, std::span<const Span> spans)
{
    auto words = regionSpanWords(spans);
    return WebCoreSwift_RegionShape_isValidShape(segments.data(), segments.size(), words.data(), words.size());
}

TextStream& operator<<(TextStream& ts, const Region& region)
{
    ts << '\n';
    {
        TextStream::IndentScope indentScope(ts);
        for (auto& rect : region.rects())
            ts << indent << "(rect "_s << rect << ")\n"_s;
    }

    return ts;
}

} // namespace WebCore
