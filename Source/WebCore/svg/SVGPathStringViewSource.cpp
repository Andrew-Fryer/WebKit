/*
 * Copyright (C) Research In Motion Limited 2010. All rights reserved.
 * Copyright (C) 2013 Apple Inc. All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public License
 * along with this library; see the file COPYING.LIB.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#include "config.h"
#include "SVGPathStringViewSource.h"

#include "SVGNumberGrammarSwiftBoundary.h"
#include "SVGParserUtilities.h"

namespace WebCore {

// The numbers and arc flags of a segment are parsed by the Swift SVG number grammar (SVGNumberGrammarSwiftBoundary.h),
// in one call per segment: each is followed by the spaces and ',' after it. On failure the buffer is left where the
// item that did not parse stopped.
template<size_t count, typename CharacterType> static std::optional<std::array<float, count>> parsePathSegmentNumbers(StringParsingBuffer<CharacterType>& buffer, uint32_t program = WebCoreSwift_SVGNumberGrammar_numbers(count))
{
    std::array<float, count> values;
    auto characters = buffer.span();
    if constexpr (count == 1) {
        auto parsed = WebCoreSwift_SVGNumberGrammar_parseNumber(characters.data(), characters.size(), std::is_same_v<CharacterType, char16_t>, true, values.data());
        buffer.advanceBy(WebCoreSwift_SVGNumberGrammar_numberConsumed(parsed));
        if (!WebCoreSwift_SVGNumberGrammar_parsedNumber(parsed))
            return std::nullopt;
        return values;
    }
    auto parsed = WebCoreSwift_SVGNumberGrammar_parseNumbers(characters.data(), characters.size(), std::is_same_v<CharacterType, char16_t>, program, values.data(), values.size());
    buffer.advanceBy(WebCoreSwift_SVGNumberGrammar_consumed(parsed));
    if (WebCoreSwift_SVGNumberGrammar_parsedCount(parsed) != count)
        return std::nullopt;
    return values;
}

SVGPathStringViewSource::SVGPathStringViewSource(StringView view)
    : m_is8BitSource(view.is8Bit())
{
    ASSERT(!view.isEmpty());

    if (m_is8BitSource)
        m_buffer8 = view.span8();
    else
        m_buffer16 = view.span16();
}

bool SVGPathStringViewSource::hasMoreData() const
{
    if (m_is8BitSource)
        return m_buffer8.hasCharactersRemaining();
    return m_buffer16.hasCharactersRemaining();
}

bool SVGPathStringViewSource::moveToNextToken()
{
    if (m_is8BitSource)
        return skipOptionalSVGSpaces(m_buffer8);
    return skipOptionalSVGSpaces(m_buffer16);
}

template <typename CharacterType> NODELETE static std::optional<SVGPathSegType> nextCommandHelper(StringParsingBuffer<CharacterType>& buffer, SVGPathSegType previousCommand)
{
    // Check for remaining coordinates in the current command.
    if ((*buffer == '+' || *buffer == '-' || *buffer == '.' || isASCIIDigit(*buffer))
        && previousCommand != SVGPathSegType::ClosePath) {
        if (previousCommand == SVGPathSegType::MoveToAbs)
            return SVGPathSegType::LineToAbs;
        if (previousCommand == SVGPathSegType::MoveToRel)
            return SVGPathSegType::LineToRel;
        return previousCommand;
    }

    return std::nullopt;
}

SVGPathSegType SVGPathStringViewSource::nextCommand(SVGPathSegType previousCommand)
{
    if (m_is8BitSource) {
        if (auto nextCommand = nextCommandHelper(m_buffer8, previousCommand))
            return *nextCommand;
    } else {
        if (auto nextCommand = nextCommandHelper(m_buffer16, previousCommand))
            return *nextCommand;
    }

    return *parseSVGSegmentType();
}

template<typename F> NODELETE decltype(auto) SVGPathStringViewSource::parse(NOESCAPE const F& functor)
{
    if (m_is8BitSource)
        return functor(m_buffer8);
    return functor(m_buffer16);
}

std::optional<SVGPathSegType> SVGPathStringViewSource::parseSVGSegmentType()
{
    return parse([](auto& buffer) -> SVGPathSegType {
        auto character = *buffer;
        buffer++;
        switch (character) {
        case 'Z':
        case 'z':
            return SVGPathSegType::ClosePath;
        case 'M':
            return SVGPathSegType::MoveToAbs;
        case 'm':
            return SVGPathSegType::MoveToRel;
        case 'L':
            return SVGPathSegType::LineToAbs;
        case 'l':
            return SVGPathSegType::LineToRel;
        case 'C':
            return SVGPathSegType::CurveToCubicAbs;
        case 'c':
            return SVGPathSegType::CurveToCubicRel;
        case 'Q':
            return SVGPathSegType::CurveToQuadraticAbs;
        case 'q':
            return SVGPathSegType::CurveToQuadraticRel;
        case 'A':
            return SVGPathSegType::ArcAbs;
        case 'a':
            return SVGPathSegType::ArcRel;
        case 'H':
            return SVGPathSegType::LineToHorizontalAbs;
        case 'h':
            return SVGPathSegType::LineToHorizontalRel;
        case 'V':
            return SVGPathSegType::LineToVerticalAbs;
        case 'v':
            return SVGPathSegType::LineToVerticalRel;
        case 'S':
            return SVGPathSegType::CurveToCubicSmoothAbs;
        case 's':
            return SVGPathSegType::CurveToCubicSmoothRel;
        case 'T':
            return SVGPathSegType::CurveToQuadraticSmoothAbs;
        case 't':
            return SVGPathSegType::CurveToQuadraticSmoothRel;
        default:
            return SVGPathSegType::Unknown;
        }
    });
}

std::optional<SVGPathSource::MoveToSegment> SVGPathStringViewSource::parseMoveToSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<MoveToSegment> {
        auto values = parsePathSegmentNumbers<2>(buffer);
        if (!values)
            return std::nullopt;

        MoveToSegment segment;
        segment.targetPoint = { (*values)[0], (*values)[1] };
        return segment;
    });
}

std::optional<SVGPathSource::LineToSegment> SVGPathStringViewSource::parseLineToSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<LineToSegment> {
        auto values = parsePathSegmentNumbers<2>(buffer);
        if (!values)
            return std::nullopt;

        LineToSegment segment;
        segment.targetPoint = { (*values)[0], (*values)[1] };
        return segment;
    });
}

std::optional<SVGPathSource::LineToHorizontalSegment> SVGPathStringViewSource::parseLineToHorizontalSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<LineToHorizontalSegment> {
        auto values = parsePathSegmentNumbers<1>(buffer);
        if (!values)
            return std::nullopt;

        LineToHorizontalSegment segment;
        segment.x = (*values)[0];
        return segment;
    });
}

std::optional<SVGPathSource::LineToVerticalSegment> SVGPathStringViewSource::parseLineToVerticalSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<LineToVerticalSegment> {
        auto values = parsePathSegmentNumbers<1>(buffer);
        if (!values)
            return std::nullopt;

        LineToVerticalSegment segment;
        segment.y = (*values)[0];
        return segment;
    });
}

std::optional<SVGPathSource::CurveToCubicSegment> SVGPathStringViewSource::parseCurveToCubicSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<CurveToCubicSegment> {
        auto values = parsePathSegmentNumbers<6>(buffer);
        if (!values)
            return std::nullopt;

        CurveToCubicSegment segment;
        segment.point1 = { (*values)[0], (*values)[1] };
        segment.point2 = { (*values)[2], (*values)[3] };
        segment.targetPoint = { (*values)[4], (*values)[5] };
        return segment;
    });
}

std::optional<SVGPathSource::CurveToCubicSmoothSegment> SVGPathStringViewSource::parseCurveToCubicSmoothSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<CurveToCubicSmoothSegment> {
        auto values = parsePathSegmentNumbers<4>(buffer);
        if (!values)
            return std::nullopt;

        CurveToCubicSmoothSegment segment;
        segment.point2 = { (*values)[0], (*values)[1] };
        segment.targetPoint = { (*values)[2], (*values)[3] };
        return segment;
    });
}

std::optional<SVGPathSource::CurveToQuadraticSegment> SVGPathStringViewSource::parseCurveToQuadraticSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<CurveToQuadraticSegment> {
        auto values = parsePathSegmentNumbers<4>(buffer);
        if (!values)
            return std::nullopt;

        CurveToQuadraticSegment segment;
        segment.point1 = { (*values)[0], (*values)[1] };
        segment.targetPoint = { (*values)[2], (*values)[3] };
        return segment;
    });
}

std::optional<SVGPathSource::CurveToQuadraticSmoothSegment> SVGPathStringViewSource::parseCurveToQuadraticSmoothSegment(FloatPoint)
{
    return parse([](auto& buffer) -> std::optional<CurveToQuadraticSmoothSegment> {
        auto values = parsePathSegmentNumbers<2>(buffer);
        if (!values)
            return std::nullopt;

        CurveToQuadraticSmoothSegment segment;
        segment.targetPoint = { (*values)[0], (*values)[1] };
        return segment;
    });
}

std::optional<SVGPathSource::ArcToSegment> SVGPathStringViewSource::parseArcToSegment(FloatPoint)
{
    // rx, ry, angle, largeArc, sweep, targetPoint.
    static constexpr uint32_t arcProgram = WebCoreSwift_SVGNumberGrammar_withItemKind(WebCoreSwift_SVGNumberGrammar_withItemKind(WebCoreSwift_SVGNumberGrammar_numbers(7), 3, WebCoreSwift_SVGNumberGrammar_ArcFlag), 4, WebCoreSwift_SVGNumberGrammar_ArcFlag);
    return parse([](auto& buffer) -> std::optional<ArcToSegment> {
        auto values = parsePathSegmentNumbers<7>(buffer, arcProgram);
        if (!values)
            return std::nullopt;

        ArcToSegment segment;
        segment.rx = (*values)[0];
        segment.ry = (*values)[1];
        segment.angle = (*values)[2];
        segment.largeArc = (*values)[3] != 0;
        segment.sweep = (*values)[4] != 0;
        segment.targetPoint = { (*values)[5], (*values)[6] };
        return segment;
    });
}

} // namespace WebKit
