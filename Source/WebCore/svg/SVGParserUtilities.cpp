/*
 * Copyright (C) 2002, 2003 The Karbon Developers
 * Copyright (C) 2006 Alexander Kellett <lypanov@kde.org>
 * Copyright (C) 2006, 2007 Rob Buis <buis@kde.org>
 * Copyright (C) 2007-2024 Apple Inc. All rights reserved.
 * Copyright (C) 2016-2018 Google Inc. All rights reserved.
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
#include "SVGParserUtilities.h"

#include "Document.h"
#include "FloatRect.h"
#include "SVGNumberGrammarSwiftBoundary.h"
#include <wtf/ASCIICType.h>
#include <wtf/text/StringParsingBuffer.h>
#include <wtf/text/StringView.h>

namespace WebCore {

// The SVG number grammar is in Swift (Source/WebCore/swift/SVGNumberGrammar/SVGNumberGrammar.swift). Parses the items
// of `program` (see SVGNumberGrammarSwiftBoundary.h) into `values` and advances `buffer` past them, or to where the
// item that did not parse stopped. Returns the number of items parsed.
template<typename CharacterType> static size_t parseSVGNumberGrammarItems(StringParsingBuffer<CharacterType>& buffer, uint32_t program, std::span<float> values)
{
    auto characters = buffer.span();
    auto parsed = WebCoreSwift_SVGNumberGrammar_parseNumbers(characters.data(), characters.size(), std::is_same_v<CharacterType, char16_t>, program, values.data(), values.size());
    buffer.advanceBy(WebCoreSwift_SVGNumberGrammar_consumed(parsed));
    return WebCoreSwift_SVGNumberGrammar_parsedCount(parsed);
}

template<typename CharacterType> static std::optional<float> parseSVGNumber(StringParsingBuffer<CharacterType>& buffer, SuffixSkippingPolicy skip)
{
    float value = 0;
    auto characters = buffer.span();
    auto parsed = WebCoreSwift_SVGNumberGrammar_parseNumber(characters.data(), characters.size(), std::is_same_v<CharacterType, char16_t>, skip == SuffixSkippingPolicy::Skip, &value);
    buffer.advanceBy(WebCoreSwift_SVGNumberGrammar_numberConsumed(parsed));
    if (!WebCoreSwift_SVGNumberGrammar_parsedNumber(parsed))
        return std::nullopt;
    return value;
}

std::optional<float> parseNumber(StringParsingBuffer<Latin1Character>& buffer, SuffixSkippingPolicy skip)
{
    return parseSVGNumber(buffer, skip);
}

std::optional<float> parseNumber(StringParsingBuffer<char16_t>& buffer, SuffixSkippingPolicy skip)
{
    return parseSVGNumber(buffer, skip);
}

std::optional<float> parseNumber(StringView string, SuffixSkippingPolicy skip)
{
    return readCharactersForParsing(string, [skip](auto buffer) -> std::optional<float> {
        // <number> uses CSS syntax, which ignores whitespace around the value.
        skipOptionalSVGSpaces(buffer);
        auto result = parseSVGNumber(buffer, skip);
        if (!buffer.atEnd())
            return std::nullopt;
        return result;
    });
}

std::optional<std::pair<float, float>> parseNumberOptionalNumber(StringView string)
{
    if (string.isEmpty())
        return std::nullopt;

    return readCharactersForParsing(string, [](auto buffer) -> std::optional<std::pair<float, float>> {
        skipOptionalSVGSpaces(buffer);

        auto x = parseNumber(buffer);
        if (!x)
            return std::nullopt;

        if (buffer.atEnd())
            return std::make_pair(*x, *x);

        auto y = parseNumber(buffer, SuffixSkippingPolicy::DontSkip);
        if (!y)
            return std::nullopt;

        skipOptionalSVGSpaces(buffer);
        if (!buffer.atEnd())
            return std::nullopt;

        return std::make_pair(*x, *y);
    });
}

std::optional<FloatPoint> parsePoint(StringView string)
{
    if (string.isEmpty())
        return std::nullopt;

    return readCharactersForParsing(string, [](auto buffer) -> std::optional<FloatPoint> {
        if (!skipOptionalSVGSpaces(buffer))
            return std::nullopt;

        std::array<float, 2> point;
        if (parseSVGNumberGrammarItems(buffer, WebCoreSwift_SVGNumberGrammar_numbers(2), point) != point.size())
            return std::nullopt;

        // Disallow anything except spaces at the end.
        skipOptionalSVGSpaces(buffer);
        if (!buffer.atEnd())
            return std::nullopt;

        return FloatPoint { point[0], point[1] };
    });
}

std::optional<FloatRect> parseRect(StringView string)
{
    return readCharactersForParsing(string, [](auto buffer) -> std::optional<FloatRect> {
        skipOptionalSVGSpaces(buffer);
        
        // x, y, width, height.
        std::array<float, 4> rect;
        if (parseSVGNumberGrammarItems(buffer, WebCoreSwift_SVGNumberGrammar_withItemKind(WebCoreSwift_SVGNumberGrammar_numbers(4), 3, WebCoreSwift_SVGNumberGrammar_NumberWithoutSuffix), rect) != rect.size())
            return std::nullopt;

        return FloatRect { rect[0], rect[1], rect[2], rect[3] };
    });
}

// Returns how far to advance past the glyph name at the start of `characters` and the ',' after it (0 if there is none),
// and its length without trailing spaces.
template<typename CharacterType> static size_t scanGlyphName(std::span<const CharacterType> characters, size_t& length)
{
    return WebCoreSwift_SVGNumberGrammar_scanListItem(characters.data(), characters.size(), std::is_same_v<CharacterType, char16_t>, WebCoreSwift_SVGNumberGrammar_GlyphNameItem, &length);
}

std::optional<HashSet<String>> parseGlyphName(StringView string)
{
    // FIXME: Parsing error detection is missing.

    return readCharactersForParsing(string, [](auto buffer) {
        skipOptionalSVGSpaces(buffer);

        HashSet<String> values;

        while (buffer.hasCharactersRemaining()) {
            // Leading and trailing white space, and white space before and after separators, will be ignored.
            auto inputStart = buffer.span();
            size_t length = 0;
            auto advance = scanGlyphName(inputStart, length);
            if (!advance)
                break;

            values.add(inputStart.first(length));
            buffer.advanceBy(advance);
        }
        return values;
    });
}

template<typename CharacterType> NODELETE static std::optional<UnicodeRange> parseUnicodeRange(std::span<const CharacterType> span)
{
    StringParsingBuffer buffer { span };
    unsigned length = buffer.lengthRemaining();
    if (length < 2 || buffer[0] != 'U' || buffer[1] != '+')
        return std::nullopt;

    buffer += 2;

    // Parse the starting hex number (or its prefix).
    unsigned startRange = 0;
    unsigned startLength = 0;

    while (buffer.hasCharactersRemaining()) {
        if (!isASCIIHexDigit(*buffer))
            break;
        ++startLength;
        if (startLength > 6)
            return std::nullopt;
        startRange = (startRange << 4) | toASCIIHexValue(*buffer);
        ++buffer;
    }

    // Handle the case of ranges separated by "-" sign.
    if (2 + startLength < length && *buffer == '-') {
        if (!startLength)
            return std::nullopt;
        
        // Parse the ending hex number (or its prefix).
        unsigned endRange = 0;
        unsigned endLength = 0;
        ++buffer;
        while (buffer.hasCharactersRemaining()) {
            if (!isASCIIHexDigit(*buffer))
                break;
            ++endLength;
            if (endLength > 6)
                return std::nullopt;
            endRange = (endRange << 4) | toASCIIHexValue(*buffer);
            ++buffer;
        }
        
        if (!endLength)
            return std::nullopt;
        
        UnicodeRange range;
        range.first = startRange;
        range.second = endRange;
        return range;
    }
    
    // Handle the case of a number with some optional trailing question marks.
    unsigned endRange = startRange;
    while (buffer.hasCharactersRemaining()) {
        if (*buffer != '?')
            break;
        ++startLength;
        if (startLength > 6)
            return std::nullopt;
        startRange <<= 4;
        endRange = (endRange << 4) | 0xF;
        ++buffer;
    }
    
    if (!startLength)
        return std::nullopt;
    
    UnicodeRange range;
    range.first = startRange;
    range.second = endRange;
    return range;
}

std::optional<std::pair<UnicodeRanges, HashSet<String>>> parseKerningUnicodeString(StringView string)
{
    // FIXME: Parsing error detection is missing.

    return readCharactersForParsing(string, [](auto buffer) -> std::pair<UnicodeRanges, HashSet<String>> {
        UnicodeRanges rangeList;
        HashSet<String> stringList;

        while (1) {
            auto inputStart = buffer.position();

            skipUntil(buffer, ',');

            if (buffer.position() == inputStart)
                break;

            // Try to parse unicode range first
            if (auto range = parseUnicodeRange(std::span { inputStart, buffer.position() }))
                rangeList.append(WTF::move(*range));
            else
                stringList.add(String({ inputStart, buffer.position() }));

            if (buffer.atEnd())
                break;

            ++buffer;
        }

        return std::make_pair(WTF::move(rangeList), WTF::move(stringList));
    });
}

}
