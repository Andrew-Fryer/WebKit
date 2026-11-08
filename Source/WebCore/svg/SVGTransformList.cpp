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
#include "SVGTransformListInlines.h"
#include <wtf/text/StringBuilder.h>
#include <wtf/text/StringParsingBuffer.h>
#include <wtf/text/StringView.h>

#if USE(SWIFT_KERNELS)
#include "SVGTransformSwiftBoundary.h"
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

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

template<typename CharacterType> static constexpr std::array<CharacterType, 5> skewXDesc  { 's', 'k', 'e', 'w', 'X' };
template<typename CharacterType> static constexpr std::array<CharacterType, 5> skewYDesc  { 's', 'k', 'e', 'w', 'Y' };
template<typename CharacterType> static constexpr std::array<CharacterType, 5> scaleDesc  { 's', 'c', 'a', 'l', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 9> translateDesc  { 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 6> rotateDesc  { 'r', 'o', 't', 'a', 't', 'e' };
template<typename CharacterType> static constexpr std::array<CharacterType, 6> matrixDesc  { 'm', 'a', 't', 'r', 'i', 'x' };

template<typename CharacterType> NODELETE static std::optional<SVGTransformValue::SVGTransformType> parseTransformTypeGeneric(StringParsingBuffer<CharacterType>& buffer)
{
    if (buffer.atEnd())
        return std::nullopt;

    if (*buffer == 's') {
        if (skipCharactersExactly(buffer, std::span { skewXDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SKEWX;
        if (skipCharactersExactly(buffer, std::span { skewYDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SKEWY;
        if (skipCharactersExactly(buffer, std::span { scaleDesc<CharacterType> }))
            return SVGTransformValue::SVG_TRANSFORM_SCALE;
        return std::nullopt;
    }

    if (skipCharactersExactly(buffer, std::span { translateDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_TRANSLATE;
    if (skipCharactersExactly(buffer, std::span { rotateDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_ROTATE;
    if (skipCharactersExactly(buffer, std::span { matrixDesc<CharacterType> }))
        return SVGTransformValue::SVG_TRANSFORM_MATRIX;

    return std::nullopt;
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

template<typename CharacterType> bool SVGTransformList::parseGeneric(StringParsingBuffer<CharacterType>& buffer, ListReplacement listReplacement)
{
    bool delimParsed = false;
    skipOptionalSVGSpaces(buffer);

    size_t itemIndex = 0;
    auto currentListReplacement = listReplacement;

    while (buffer.hasCharactersRemaining()) {
        delimParsed = false;

        auto parsedTransformType = parseTransformType(buffer);
        if (!parsedTransformType)
            return false;

        if (currentListReplacement == ListReplacement::Replace && itemIndex < m_items.size() && parsedTransformType == m_items[itemIndex]->type()) {
            if (!parseAndReplaceTransform(*parsedTransformType, buffer, m_items[itemIndex]))
                return false;
        } else {
            // Switch to `Append` mode and remove the existing SVGTransforms starting from `itemIndex`.
            if (currentListReplacement == ListReplacement::Replace) {
                currentListReplacement = ListReplacement::Append;
                resize(itemIndex);
            }

            RefPtr parsedTransform = parseTransform(*parsedTransformType, buffer);
            if (!parsedTransform)
                return false;

            append(parsedTransform.releaseNonNull());
        }

        skipOptionalSVGSpaces(buffer);

        if (skipExactly(buffer, ','))
            delimParsed = true;

        skipOptionalSVGSpaces(buffer);

        ++itemIndex;
    }

    if (itemIndex < m_items.size()) {
        ASSERT(currentListReplacement == ListReplacement::Replace);
        resize(itemIndex);
    }

    return !delimParsed;
}

#if USE(SWIFT_KERNELS)

// Latin-1 transform lists are lexed in Swift (Source/WebCore/swift/SVGTransform/SVGTransformLexer.swift),
// up to 8 items per call; the specialization of parseGeneric below applies each item to the list
// exactly like the primary template does. char16_t lists (rare) still use the primary template.

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// The C++ lexing the Swift item lexer replaces, assembled from what the primary template calls:
// its whitespace skipping, parseTransformType, parseTransformParamList (via parseAndReplaceTransform
// or parseTransform) and the separator handling after each item.
template<typename CharacterType> static uint32_t lexSVGTransformListItemCxx(StringParsingBuffer<CharacterType>& buffer, std::span<float, 6> values)
{
    if (!skipOptionalSVGSpaces(buffer))
        return WebCoreSwift_SVGTransform_OnlyWhitespace;

    auto parsedTransformType = SVGTransformList::parseTransformType(buffer);
    if (!parsedTransformType)
        return WebCoreSwift_SVGTransform_NoTransformType;

    uint32_t typeBits = static_cast<uint32_t>(*parsedTransformType) << 8;
    int valueCount = parseTransformParamList(buffer, values, requiredValuesForType[*parsedTransformType], optionalValuesForType[*parsedTransformType]);
    if (valueCount < 0)
        return WebCoreSwift_SVGTransform_InvalidParameters | typeBits;

    skipOptionalSVGSpaces(buffer);

    bool delimParsed = skipExactly(buffer, ',');

    skipOptionalSVGSpaces(buffer);

    return WebCoreSwift_SVGTransform_Parsed | typeBits | static_cast<uint32_t>(valueCount) << 16 | static_cast<uint32_t>(delimParsed) << 24;
}
#endif

// Lexes items from the start of buffer in Swift, advances buffer past the last one (also if it failed)
// and returns how many were lexed (at least one). Lexing stops after an item that did not parse.
static size_t lexSVGTransformListItems(StringParsingBuffer<Latin1Character>& buffer, std::array<uint32_t, 8>& items, std::array<std::array<float, 6>, 8>& values)
{
    static_assert(sizeof(values) == sizeof(float) * 6 * 8);
    auto characters = byteCast<uint8_t>(buffer.span());
    size_t consumed = 0;
    size_t itemCount = WebCoreSwift_SVGTransform_lexItems8(characters.data(), characters.size(), items.data(), values[0].data(), items.size(), &consumed);
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]] {
        RELEASE_ASSERT(itemCount >= 1 && itemCount <= items.size());
        auto expectedBuffer = buffer;
        for (size_t i = 0; i < itemCount; ++i) {
            std::array<float, 6> expectedValues { 0, 0, 0, 0, 0, 0 };
            uint32_t expectedResult = lexSVGTransformListItemCxx(expectedBuffer, expectedValues);
            RELEASE_ASSERT(items[i] == expectedResult);
            if (WebCoreSwift_SVGTransform_status(expectedResult) != WebCoreSwift_SVGTransform_Parsed) {
                RELEASE_ASSERT(i + 1 == itemCount);
                break;
            }
            for (unsigned j = 0; j < WebCoreSwift_SVGTransform_valueCount(expectedResult); ++j)
                RELEASE_ASSERT(std::bit_cast<uint32_t>(values[i][j]) == std::bit_cast<uint32_t>(expectedValues[j]));
        }
        RELEASE_ASSERT(consumed == characters.size() - expectedBuffer.lengthRemaining());
    }
#endif
    buffer.advanceBy(consumed);
    return itemCount;
}

// The tail of parseAndReplaceTransform (SVGTransformListInlines.h), after the parameter list is parsed.
static bool setSVGTransformFromLexedValues(SVGTransformValue::SVGTransformType type, const std::array<float, 6>& values, int valueCount, SVGTransform& transform)
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

// The tail of parseTransform (SVGTransformListInlines.h), after the parameter list is parsed.
static RefPtr<SVGTransform> createSVGTransformFromLexedValues(SVGTransformValue::SVGTransformType type, const std::array<float, 6>& values, int valueCount)
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

template<> bool SVGTransformList::parseGeneric(StringParsingBuffer<Latin1Character>& buffer, ListReplacement listReplacement)
{
    bool delimParsed = false;

    size_t itemIndex = 0;
    auto currentListReplacement = listReplacement;

    while (buffer.hasCharactersRemaining()) {
        std::array<uint32_t, 8> items { };
        std::array<std::array<float, 6>, 8> values { };
        size_t itemCount = lexSVGTransformListItems(buffer, items, values);
        for (size_t i = 0; i < itemCount; ++i) {
            auto status = WebCoreSwift_SVGTransform_status(items[i]);

            // The primary template skips whitespace before the loop; only the first item can see it.
            if (status == WebCoreSwift_SVGTransform_OnlyWhitespace)
                break;

            delimParsed = false;

            if (status == WebCoreSwift_SVGTransform_NoTransformType)
                return false;

            auto parsedTransformType = static_cast<SVGTransformValue::SVGTransformType>(WebCoreSwift_SVGTransform_type(items[i]));
            int valueCount = WebCoreSwift_SVGTransform_valueCount(items[i]);

            if (currentListReplacement == ListReplacement::Replace && itemIndex < m_items.size() && parsedTransformType == m_items[itemIndex]->type()) {
                if (status != WebCoreSwift_SVGTransform_Parsed || !setSVGTransformFromLexedValues(parsedTransformType, values[i], valueCount, m_items[itemIndex]))
                    return false;
            } else {
                // Switch to `Append` mode and remove the existing SVGTransforms starting from `itemIndex`.
                if (currentListReplacement == ListReplacement::Replace) {
                    currentListReplacement = ListReplacement::Append;
                    resize(itemIndex);
                }

                if (status != WebCoreSwift_SVGTransform_Parsed)
                    return false;

                RefPtr parsedTransform = createSVGTransformFromLexedValues(parsedTransformType, values[i], valueCount);
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

#endif // USE(SWIFT_KERNELS)

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
