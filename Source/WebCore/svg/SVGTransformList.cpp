/*
 * Copyright (C) 2004, 2005, 2008 Nikolas Zimmermann <zimmermann@kde.org>
 * Copyright (C) 2004, 2005, 2006, 2007 Rob Buis <buis@kde.org>
 * Copyright (C) 2007 Eric Seidel <eric@webkit.org>
 * Copyright (C) 2020 Apple Inc. All rights reserved.
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
#include "SVGTransformList.h"

#include "ExceptionOr.h"
#include "SVGParserUtilities.h"
#include "SVGTransform.h"
#include <wtf/text/StringBuilder.h>
#include <wtf/text/StringParsingBuffer.h>
#include <wtf/text/StringView.h>

#include "SVGTransformSwiftBoundary.h"

namespace WebCore {

SVGTransformList::SVGTransformList() = default;

SVGTransformList::SVGTransformList(SVGPropertyOwner* owner, SVGPropertyAccess access)
    : Base(owner, access)
{
}

SVGTransformList::SVGTransformList(const SVGTransformList& other, SVGPropertyAccess access)
    : Base(other, access)
{
}

SVGTransformList::~SVGTransformList() = default;

// SVG transform lists are lexed in Swift (Source/WebCore/swift/SVGTransform/SVGTransformLexer.swift); this file applies
// the lexed items to the list.

// Lexes up to items.size() items from the start of buffer, advances buffer past the last one (also if it failed) and
// returns how many were lexed (at least one). Lexing stops after an item that did not parse. Item i's values go to
// values[6 * i ..< 6 * i + 6].
template<typename CharacterType> NODELETE static size_t lexSVGTransformListItems(StringParsingBuffer<CharacterType>& buffer, std::span<uint32_t> items, std::span<float> values)
{
    auto characters = buffer.span();
    auto lexed = WebCoreSwift_SVGTransform_lexItems(characters.data(), characters.size(), sizeof(CharacterType) == sizeof(char16_t), items.data(), items.size(), values.data(), values.size());
    buffer.advanceBy(WebCoreSwift_SVGTransform_consumed(lexed));
    return WebCoreSwift_SVGTransform_itemCount(lexed);
}

template<typename CharacterType> NODELETE static std::optional<SVGTransformValue::SVGTransformType> parseTransformTypeGeneric(StringParsingBuffer<CharacterType>& buffer)
{
    // The keyword must start at the cursor (the lexer skips whitespace before an item).
    if (buffer.atEnd() || isASCIIWhitespace(*buffer))
        return std::nullopt;

    // Lex the item that starts with the keyword, but only advance past the keyword.
    auto start = buffer;
    std::array<uint32_t, 1> item { };
    std::array<float, 6> values { };
    lexSVGTransformListItems(start, std::span { item }, std::span { values });
    if (WebCoreSwift_SVGTransform_status(item[0]) == WebCoreSwift_SVGTransform_NoTransformType)
        return std::nullopt;

    // The lengths of "matrix", "translate", "scale", "rotate", "skewX" and "skewY", indexed by SVGTransformType.
    static constexpr std::array<uint8_t, 7> keywordLengths { 0, 6, 9, 5, 6, 5, 5 };
    auto type = static_cast<SVGTransformValue::SVGTransformType>(WebCoreSwift_SVGTransform_type(item[0]));
    buffer.advanceBy(keywordLengths[type]);
    return type;
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformList::parseTransformType(StringView string)
{
    return readCharactersForParsing(string, [](auto buffer) {
        return parseTransformType(buffer);
    });
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformList::parseTransformType(StringParsingBuffer<Latin1Character>& buffer)
{
    return parseTransformTypeGeneric(buffer);
}

std::optional<SVGTransformValue::SVGTransformType> SVGTransformList::parseTransformType(StringParsingBuffer<char16_t>& buffer)
{
    return parseTransformTypeGeneric(buffer);
}

ExceptionOr<RefPtr<SVGTransform>> SVGTransformList::consolidate()
{
    auto result = canAlterList();
    if (result.hasException())
        return result.releaseException();
    ASSERT(result.releaseReturnValue());

    // Spec: If the list was empty, then a value of null is returned.
    if (m_items.isEmpty())
        return nullptr;

    if (m_items.size() == 1)
        return RefPtr { at(0).ptr() };

    auto newItem = SVGTransform::create(*concatenate());
    clearItems();

    auto item = append(WTF::move(newItem));
    commitChange();
    return RefPtr { item.ptr() };
}

std::optional<AffineTransform> SVGTransformList::concatenate() const
{
    if (m_items.isEmpty())
        return std::nullopt;
    auto items = m_items.span();
    AffineTransform result = items.front()->matrix().value();
    for (auto& transform : items.subspan(1))
        result *= transform->matrix().value();
    return result;
}

// Sets transform (whose type is type) to the lexed item.
static bool setSVGTransformFromLexedValues(SVGTransformValue::SVGTransformType type, std::span<const float, 6> values, int valueCount, SVGTransform& transform)
{
    ASSERT(type == transform.value().type());

    switch (type) {
    case SVGTransformValue::SVG_TRANSFORM_UNKNOWN:
        ASSERT_NOT_REACHED();
        return false;

    case SVGTransformValue::SVG_TRANSFORM_SKEWX:
        transform.value().setSkewX(values[0]);
        return true;

    case SVGTransformValue::SVG_TRANSFORM_SKEWY:
        transform.value().setSkewY(values[0]);
        return true;

    case SVGTransformValue::SVG_TRANSFORM_SCALE:
        if (valueCount == 1)
            transform.value().setScale(values[0], values[0]);
        else
            transform.value().setScale(values[0], values[1]);
        return true;

    case SVGTransformValue::SVG_TRANSFORM_TRANSLATE:
        if (valueCount == 1)
            transform.value().setTranslate(values[0], 0);
        else
            transform.value().setTranslate(values[0], values[1]);
        return true;

    case SVGTransformValue::SVG_TRANSFORM_ROTATE:
        if (valueCount == 1)
            transform.value().setRotate(values[0], 0, 0);
        else
            transform.value().setRotate(values[0], values[1], values[2]);
        return true;

    case SVGTransformValue::SVG_TRANSFORM_MATRIX:
        transform.value().setMatrix(AffineTransform(values[0], values[1], values[2], values[3], values[4], values[5]));
        return true;
    }

    return false;
}

// A new SVGTransform for the lexed item.
static RefPtr<SVGTransform> createSVGTransformFromLexedValues(SVGTransformValue::SVGTransformType type, std::span<const float, 6> values, int valueCount)
{
    switch (type) {
    case SVGTransformValue::SVG_TRANSFORM_UNKNOWN:
        ASSERT_NOT_REACHED();
        return nullptr;

    case SVGTransformValue::SVG_TRANSFORM_SKEWX: {
        SVGTransformValue transform;
        transform.setSkewX(values[0]);
        return SVGTransform::create(WTF::move(transform));
    }
    case SVGTransformValue::SVG_TRANSFORM_SKEWY: {
        SVGTransformValue transform;
        transform.setSkewY(values[0]);
        return SVGTransform::create(WTF::move(transform));
    }
    case SVGTransformValue::SVG_TRANSFORM_SCALE: {
        auto resultValue = [&]() {
            if (valueCount == 1) // Spec: if only one param given, assume uniform scaling
                return SVGTransformValue::scaleTransformValue({ values[0], values[0] });

            return SVGTransformValue::scaleTransformValue({ values[0], values[1] });
        };

        return SVGTransform::create(resultValue());
    }
    case SVGTransformValue::SVG_TRANSFORM_TRANSLATE: {
        if (valueCount == 1) // Spec: if only one param given, assume 2nd param to be 0
            return SVGTransform::create(SVGTransformValue::translateTransformValue({ values[0], 0 }));

        return SVGTransform::create(SVGTransformValue::translateTransformValue({ values[0], values[1] }));
    }
    case SVGTransformValue::SVG_TRANSFORM_ROTATE: {
        auto resultValue = [&]() {
            if (valueCount == 1)
                return SVGTransformValue::rotateTransformValue(values[0], { });

            return SVGTransformValue::rotateTransformValue(values[0], { values[1], values[2] });
        };
        return SVGTransform::create(resultValue());
    }
    case SVGTransformValue::SVG_TRANSFORM_MATRIX: {
        SVGTransformValue transform;
        transform.setMatrix(AffineTransform(values[0], values[1], values[2], values[3], values[4], values[5]));
        return SVGTransform::create(transform);
    }
    }

    return nullptr;
}

template<typename CharacterType> bool SVGTransformList::parseGeneric(StringParsingBuffer<CharacterType>& buffer, ListReplacement listReplacement)
{
    bool delimParsed = false;

    size_t itemIndex = 0;
    auto currentListReplacement = listReplacement;

    while (buffer.hasCharactersRemaining()) {
        std::array<uint32_t, 8> items { };
        std::array<float, 6 * 8> values { };
        size_t itemCount = lexSVGTransformListItems(buffer, std::span { items }, std::span { values });
        for (size_t i = 0; i < itemCount; ++i) {
            auto itemValues = std::span { values }.subspan(6 * i).first<6>();
            auto status = WebCoreSwift_SVGTransform_status(items[i]);

            // Only the first item can be whitespace: every item skips the whitespace after it.
            if (status == WebCoreSwift_SVGTransform_OnlyWhitespace)
                break;

            delimParsed = false;

            if (status == WebCoreSwift_SVGTransform_NoTransformType)
                return false;

            auto parsedTransformType = static_cast<SVGTransformValue::SVGTransformType>(WebCoreSwift_SVGTransform_type(items[i]));
            int valueCount = WebCoreSwift_SVGTransform_valueCount(items[i]);

            if (currentListReplacement == ListReplacement::Replace && itemIndex < m_items.size() && parsedTransformType == m_items[itemIndex]->type()) {
                if (status != WebCoreSwift_SVGTransform_Parsed || !setSVGTransformFromLexedValues(parsedTransformType, itemValues, valueCount, m_items[itemIndex]))
                    return false;
            } else {
                // Switch to `Append` mode and remove the existing SVGTransforms starting from `itemIndex`.
                if (currentListReplacement == ListReplacement::Replace) {
                    currentListReplacement = ListReplacement::Append;
                    resize(itemIndex);
                }

                if (status != WebCoreSwift_SVGTransform_Parsed)
                    return false;

                RefPtr parsedTransform = createSVGTransformFromLexedValues(parsedTransformType, itemValues, valueCount);
                if (!parsedTransform)
                    return false;

                append(parsedTransform.releaseNonNull());
            }

            if (WebCoreSwift_SVGTransform_followedByComma(items[i]))
                delimParsed = true;

            ++itemIndex;
        }
    }

    if (itemIndex < m_items.size()) {
        ASSERT(currentListReplacement == ListReplacement::Replace);
        resize(itemIndex);
    }

    return !delimParsed;
}

bool SVGTransformList::parse(StringView value)
{
    bool parsingSucceeded = readCharactersForParsing(value, [&](auto buffer) {
        return parseGeneric(buffer, ListReplacement::Replace);
    });

    if (!parsingSucceeded)
        clearItems();
    return parsingSucceeded;
}

bool SVGTransformList::parse(StringParsingBuffer<Latin1Character>& buffer)
{
    return parseGeneric(buffer, ListReplacement::Append);
}

bool SVGTransformList::parse(StringParsingBuffer<char16_t>& buffer)
{
    return parseGeneric(buffer, ListReplacement::Append);
}

String SVGTransformList::valueAsString() const
{
    StringBuilder builder;
    for (const auto& transform : m_items) {
        if (builder.length())
            builder.append(' ');

        builder.append(transform->value().valueAsString());
    }
    return builder.toString();
}

}
