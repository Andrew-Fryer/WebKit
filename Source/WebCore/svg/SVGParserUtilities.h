/*
 * Copyright (C) 2002, 2003 The Karbon Developers
 * Copyright (C) 2006, 2007 Rob Buis <buis@kde.org>
 * Copyright (C) 2013-2024 Apple Inc. All rights reserved.
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

#pragma once

#include <wtf/Forward.h>
#include <wtf/text/ParsingUtilities.h>

typedef std::pair<char32_t, char32_t> UnicodeRange;
typedef Vector<UnicodeRange> UnicodeRanges;

namespace WebCore {

class FloatPoint;
class FloatRect;

enum class SuffixSkippingPolicy {
    DontSkip,
    Skip
};

// The SVG <number> grammar (numbers, path arc flags, the spaces / ',' between them, and list item scanning) is
// implemented in Swift: Source/WebCore/swift/SVGNumberGrammar. These functions are its C++ interface.

NODELETE std::optional<float> parseNumber(StringParsingBuffer<Latin1Character>&, SuffixSkippingPolicy = SuffixSkippingPolicy::Skip);
NODELETE std::optional<float> parseNumber(StringParsingBuffer<char16_t>&, SuffixSkippingPolicy = SuffixSkippingPolicy::Skip);
std::optional<float> parseNumber(StringView, SuffixSkippingPolicy = SuffixSkippingPolicy::Skip);

std::optional<std::pair<float, float>> parseNumberOptionalNumber(StringView);

std::optional<FloatPoint> parsePoint(StringView);
std::optional<FloatRect> parseRect(StringView);

std::optional<std::pair<UnicodeRanges, HashSet<String>>> parseKerningUnicodeString(StringView);
std::optional<HashSet<String>> parseGlyphName(StringView);

template<typename CharacterType> constexpr bool skipOptionalSVGSpaces(const CharacterType*& ptr, const CharacterType* end)
{
    skipWhile<isASCIIWhitespace>(ptr, end);
    return ptr < end;
}

template<typename CharacterType> constexpr bool skipOptionalSVGSpaces(StringParsingBuffer<CharacterType>& characters)
{
    skipWhile<isASCIIWhitespace>(characters);
    return characters.hasCharactersRemaining();
}

} // namespace WebCore
