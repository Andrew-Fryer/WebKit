// Copyright 2014 The Chromium Authors. All rights reserved.
// Copyright (C) 2016-2022 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "config.h"
#include "CSSParserFastPaths.h"

#include "CSSAbsoluteColorResolver.h"
#include "CSSFunctionValue.h"
#include "CSSKeywordColor.h"
#include "CSSParserContext.h"
#include "CSSParserIdioms.h"
#include "CSSPrimitiveNumericTypes.h"
#include "CSSPrimitiveValue.h"
#include "CSSProperty.h"
#include "CSSPropertyNames.h"
#include "CSSPropertyParser.h"
#include "CSSPropertyParserConsumer+Color.h"
#include "CSSPropertyParserState.h"
#include "CSSPropertyParsing.h"
#include "CSSTransformListValue.h"
#include "CSSValueList.h"
#include "CSSValuePool.h"
#include "ColorConversion.h"
#include "FloatConversion.h"
#include "HashTools.h"
#include "StylePropertyShorthand.h"
#include <wtf/text/ParsingUtilities.h>
#include <wtf/text/StringParsingBuffer.h>

#include "CSSColorSwiftBoundary.h"

namespace WebCore {

std::optional<CSS::Range> CSSParserFastPaths::lengthValueRangeForPropertiesSupportingSimpleLengths(CSSPropertyID propertyId)
{
    switch (propertyId) {
    case CSSPropertyID::FontSize:
    case CSSPropertyID::Height:
    case CSSPropertyID::Width:
    case CSSPropertyID::MinHeight:
    case CSSPropertyID::MinWidth:
    case CSSPropertyID::PaddingBottom:
    case CSSPropertyID::PaddingLeft:
    case CSSPropertyID::PaddingRight:
    case CSSPropertyID::PaddingTop:
    case CSSPropertyID::InlineSize:
    case CSSPropertyID::BlockSize:
    case CSSPropertyID::MinInlineSize:
    case CSSPropertyID::MinBlockSize:
    case CSSPropertyID::PaddingBlockEnd:
    case CSSPropertyID::PaddingBlockStart:
    case CSSPropertyID::PaddingInlineEnd:
    case CSSPropertyID::PaddingInlineStart:
    case CSSPropertyID::R:
    case CSSPropertyID::Rx:
    case CSSPropertyID::Ry:
    case CSSPropertyID::ShapeMargin:
        return CSS::Nonnegative;
    case CSSPropertyID::Bottom:
    case CSSPropertyID::Cx:
    case CSSPropertyID::Cy:
    case CSSPropertyID::Left:
    case CSSPropertyID::InsetBlockEnd:
    case CSSPropertyID::InsetBlockStart:
    case CSSPropertyID::InsetInlineEnd:
    case CSSPropertyID::InsetInlineStart:
    case CSSPropertyID::MarginBottom:
    case CSSPropertyID::MarginLeft:
    case CSSPropertyID::MarginRight:
    case CSSPropertyID::MarginTop:
    case CSSPropertyID::Right:
    case CSSPropertyID::Top:
    case CSSPropertyID::MarginBlockEnd:
    case CSSPropertyID::MarginBlockStart:
    case CSSPropertyID::MarginInlineEnd:
    case CSSPropertyID::MarginInlineStart:
    case CSSPropertyID::X:
    case CSSPropertyID::Y:
        return CSS::All;
    default:
        return { };
    }
}

template<typename CharacterType> static inline std::optional<double> parseCSSNumber(std::span<const CharacterType> characters)
{
    // The charactersToDouble() function allows a trailing '.' but that is not allowed in CSS number values.
    if (!characters.empty() && characters.back() == '.')
        return std::nullopt;
    // FIXME: If we don't want to skip over leading spaces, we should use parseDouble, not charactersToDouble.
    bool ok;
    auto number = charactersToDouble(characters, &ok);
    if (!ok)
        return std::nullopt;
    return number;
}

template <typename CharacterType>
static inline bool parseSimpleLength(std::span<const CharacterType> characters, CSSUnitType& unit, double& number)
{
    if (characters.size() > 2 && isASCIIAlphaCaselessEqual(characters[characters.size() - 2], 'p') && isASCIIAlphaCaselessEqual(characters[characters.size() - 1], 'x')) {
        dropLast(characters, 2);
        unit = CSSUnitType::Px;
    } else if (!characters.empty() && characters.back() == '%') {
        dropLast(characters);
        unit = CSSUnitType::Percentage;
    }

    auto parsedNumber = parseCSSNumber(characters);
    number = parsedNumber.value_or(0);
    return parsedNumber.has_value();
}

enum class RequireUnits : bool { No, Yes };

template <typename CharacterType>
static inline bool parseSimpleAngle(std::span<const CharacterType> characters, RequireUnits requireUnits, CSS::AngleUnit& unit, double& number)
{
    // "0deg" or "1rad"
    if (characters.size() >= 4) {
        if (isASCIIAlphaCaselessEqual(characters[characters.size() - 3], 'd') && isASCIIAlphaCaselessEqual(characters[characters.size() - 2], 'e') && isASCIIAlphaCaselessEqual(characters[characters.size() - 1], 'g')) {
            dropLast(characters, 3);
            unit = CSS::AngleUnit::Deg;
        } else if (isASCIIAlphaCaselessEqual(characters[characters.size() - 3], 'r') && isASCIIAlphaCaselessEqual(characters[characters.size() - 2], 'a') && isASCIIAlphaCaselessEqual(characters[characters.size() - 1], 'd')) {
            dropLast(characters, 3);
            unit = CSS::AngleUnit::Rad;
        } else if (requireUnits == RequireUnits::Yes)
            return false;
    } else {
        if (requireUnits == RequireUnits::Yes || !characters.size())
            return false;

        unit = CSS::AngleUnit::Deg;
    }

    auto parsedNumber = parseCSSNumber(characters);
    number = parsedNumber.value_or(0);
    return parsedNumber.has_value();
}

template <typename CharacterType>
static inline bool parseSimpleNumberOrPercentageDividedBy100(std::span<const CharacterType> characters, double& number)
{
    bool isPercentage = false;
    if (!characters.empty() && characters.back() == '%') {
        dropLast(characters);
        isPercentage = true;
    }

    auto parsedNumber = parseCSSNumber(characters);
    if (!parsedNumber)
        return false;

    number = *parsedNumber;
    if (std::isinf(number))
        return false;

    if (isPercentage)
        number /= 100.0;

    return true;
}

static RefPtr<CSSValue> parseSimpleLengthValue(StringView string, CSSParserMode cssParserMode, CSS::Range valueRange)
{
    ASSERT(!string.isEmpty());

    double number;
    auto unit = CSSUnitType::Number;

    if (string.is8Bit()) {
        if (!parseSimpleLength(string.span8(), unit, number))
            return nullptr;
    } else {
        if (!parseSimpleLength(string.span16(), unit, number))
            return nullptr;
    }

    if (unit == CSSUnitType::Number) {
        if (number && cssParserMode != SVGAttributeMode)
            return nullptr;
        unit = CSSUnitType::Px;
    }

    if (number < valueRange.min || number > valueRange.max)
        return nullptr;

    if (std::isinf(number))
        return nullptr;

    return CSSPrimitiveValue::create(number, unit);
}

// Numeric and hex colors are parsed in Swift (CSSColorSwiftBoundary.h).
static std::optional<SRGBA<uint8_t>> parseColorWithSwift(StringView string, uint8_t mode)
{
    auto packed = WebCoreSwift_CSSColor_parseColor(string.rawCharacters(), string.length(), string.is8Bit(), mode);
    if (!packed)
        return std::nullopt;
    return asSRGBA(PackedColor::RGBA { static_cast<uint32_t>(packed) });
}

static std::optional<SRGBA<uint8_t>> parseNumericColor(StringView string, const CSSParserContext& context)
{
    return parseColorWithSwift(string, isQuirksModeBehavior(context.mode) ? WebCoreSwift_CSSColor_NumericQuirks : WebCoreSwift_CSSColor_Numeric);
}

static RefPtr<CSSValue> parseColor(StringView string, const CSSParserContext& context)
{
    ASSERT(!string.isEmpty());
    auto valueID = cssValueKeywordID(string);
    if (CSS::isColorKeyword(valueID)) {
        if (!CSSPropertyParserHelpers::isColorKeywordAllowed(valueID, context))
            return nullptr;
        return CSSKeywordValue::create(valueID);
    }
    if (auto color = parseNumericColor(string, context))
        return CSSValuePool::singleton().createColorValue(*color);
    return nullptr;
}

static std::optional<SRGBA<uint8_t>> finishParsingNamedColor(std::span<char> buffer)
{
    buffer.back() = '\0';
    auto namedColor = findColor(buffer.data(), buffer.size() - 1);
    if (!namedColor)
        return std::nullopt;
    return asSRGBA(PackedColor::ARGB { namedColor->ARGBValue });
}

template<typename CharacterType> static std::optional<SRGBA<uint8_t>> parseNamedColorInternal(std::span<const CharacterType> characters)
{
    std::array<char, 64> buffer; // Easily big enough for the longest color name.
    if (characters.size() > buffer.size() - 1)
        return std::nullopt;
    for (size_t i = 0; i < characters.size(); ++i) {
        auto character = characters[i];
        if (!character || !isASCII(character))
            return std::nullopt;
        buffer[i] = toASCIILower(static_cast<char>(character));
    }
    return finishParsingNamedColor(std::span { buffer }.first(characters.size() + 1));
}

std::optional<SRGBA<uint8_t>> CSSParserFastPaths::parseSimpleColor(StringView string, const CSSParserContext& context)
{
    if (auto color = parseNumericColor(string, context))
        return color;
    return parseNamedColor(string);
}

std::optional<SRGBA<uint8_t>> CSSParserFastPaths::parseHexColor(StringView string)
{
    return parseColorWithSwift(string, WebCoreSwift_CSSColor_Hex);
}

std::optional<SRGBA<uint8_t>> CSSParserFastPaths::parseNamedColor(StringView string)
{
    if (string.is8Bit())
        return parseNamedColorInternal(string.span8());
    return parseNamedColorInternal(string.span16());
}

static bool isUniversalKeyword(StringView string)
{
    // These keywords can be used for all properties.
    return equalLettersIgnoringASCIICase(string, "initial"_s)
        || equalLettersIgnoringASCIICase(string, "inherit"_s)
        || equalLettersIgnoringASCIICase(string, "unset"_s)
        || equalLettersIgnoringASCIICase(string, "revert"_s)
        || equalLettersIgnoringASCIICase(string, "revert-layer"_s)
        || equalLettersIgnoringASCIICase(string, "revert-rule"_s);
}

static RefPtr<CSSValue> parseKeywordValue(CSSPropertyID property, StringView string, CSS::PropertyParserState& state)
{
    ASSERT(!string.isEmpty());

    if (!CSSPropertyParsing::isKeywordFastPathEligibleStyleProperty(property)) {
        // All properties, including non-keyword properties, accept the CSS-wide keywords.
        if (!isUniversalKeyword(string))
            return nullptr;

        // Leave shorthands to parse CSS-wide keywords using CSSPropertyParser.
        if (shorthandForProperty(property).length())
            return nullptr;
    }

    auto valueID = cssValueKeywordID(string);
    if (valueID == CSSValueID::Invalid)
        return nullptr;

    if (isCSSWideKeyword(valueID))
        return CSSKeywordValue::create(valueID);

    if (CSSPropertyParsing::isKeywordValidForStyleProperty(property, valueID, state))
        return CSSKeywordValue::create(valueID);
    return nullptr;
}

template <typename CharType>
static bool parseTransformTranslateArguments(StringParsingBuffer<CharType>& buffer, unsigned expectedCount, CSSValueID transformType, CSSValueListBuilder& arguments)
{
    while (expectedCount) {
        size_t delimiter = find(buffer.span(), expectedCount == 1 ? ')' : ',');
        if (delimiter == notFound)
            return false;
        unsigned argumentLength = static_cast<unsigned>(delimiter);
        CSSUnitType unit = CSSUnitType::Number;
        double number;
        if (!parseSimpleLength(buffer.span().first(argumentLength), unit, number))
            return false;
        if (!number && unit == CSSUnitType::Number)
            unit = CSSUnitType::Px;
        if (unit == CSSUnitType::Number || (unit == CSSUnitType::Percentage && (transformType == CSSValueID::TranslateZ || (transformType == CSSValueID::Translate3d && expectedCount == 1))))
            return false;
        arguments.append(CSSPrimitiveValue::create(number, unit));
        buffer.advanceBy(argumentLength + 1);
        --expectedCount;
    }
    return true;
}

template <typename CharType>
static RefPtr<CSSValue> parseTransformAngleArgument(StringParsingBuffer<CharType>& buffer)
{
    size_t delimiter = find(buffer.span(), ')');
    if (delimiter == notFound)
        return nullptr;

    unsigned argumentLength = static_cast<unsigned>(delimiter);
    auto angleUnit = CSS::AngleUnit::Deg;
    double number;
    if (!parseSimpleAngle(buffer.span().first(argumentLength), RequireUnits::Yes, angleUnit, number))
        return nullptr;

    buffer.advanceBy(argumentLength + 1);

    return CSSPrimitiveValue::create(number, CSS::toCSSUnitType(angleUnit));
}

template <typename CharType>
static bool parseTransformNumberArguments(StringParsingBuffer<CharType>& buffer, unsigned expectedCount, CSSValueListBuilder& arguments)
{
    while (expectedCount) {
        size_t delimiter = find(buffer.span(), expectedCount == 1 ? ')' : ',');
        if (delimiter == notFound)
            return false;
        unsigned argumentLength = static_cast<unsigned>(delimiter);
        auto number = parseCSSNumber(buffer.span().first(argumentLength));
        if (!number)
            return false;
        arguments.append(CSSPrimitiveValue::create(*number, CSSUnitType::Number));
        buffer.advanceBy(argumentLength + 1);
        --expectedCount;
    }
    return true;
}

template <typename CharType>
static RefPtr<CSSFunctionValue> parseSimpleTransformValue(StringParsingBuffer<CharType>& buffer)
{
    // Also guarantees indexes up to buffer[8] are safe to access below.
    constexpr auto shortestValidTransformStringLength = 9; // "rotate(0)"
    if (buffer.lengthRemaining() < shortestValidTransformStringLength)
        return nullptr;

    bool isTranslate = toASCIILower(buffer[0]) == 't'
        && toASCIILower(buffer[1]) == 'r'
        && toASCIILower(buffer[2]) == 'a'
        && toASCIILower(buffer[3]) == 'n'
        && toASCIILower(buffer[4]) == 's'
        && toASCIILower(buffer[5]) == 'l'
        && toASCIILower(buffer[6]) == 'a'
        && toASCIILower(buffer[7]) == 't'
        && toASCIILower(buffer[8]) == 'e';

    if (isTranslate) {
        // Also guarantees indexes up to buffer[11] are safe to access below.
        constexpr auto shortestValidTranslateStringLength = 12; // "translate(0)"
        if (buffer.lengthRemaining() < shortestValidTranslateStringLength)
            return nullptr;

        CSSValueID transformType;
        unsigned expectedArgumentCount = 1;
        unsigned argumentStart = 11;
        CharType c9 = toASCIILower(buffer[9]);
        if (c9 == 'x' && buffer[10] == '(') {
            transformType = CSSValueID::TranslateX;
        } else if (c9 == 'y' && buffer[10] == '(') {
            transformType = CSSValueID::TranslateY;
        } else if (c9 == 'z' && buffer[10] == '(') {
            transformType = CSSValueID::TranslateZ;
        } else if (c9 == '(') {
            transformType = CSSValueID::Translate;
            expectedArgumentCount = 2;
            argumentStart = 10;
        } else if (c9 == '3' && toASCIILower(buffer[10]) == 'd' && buffer[11] == '(') {
            transformType = CSSValueID::Translate3d;
            expectedArgumentCount = 3;
            argumentStart = 12;
        } else
            return nullptr;

        CSSValueListBuilder arguments;
        buffer.advanceBy(argumentStart);
        if (!parseTransformTranslateArguments(buffer, expectedArgumentCount, transformType, arguments))
            return nullptr;
        return CSSFunctionValue::create(transformType, WTF::move(arguments));
    }

    bool isMatrix3d = toASCIILower(buffer[0]) == 'm'
        && toASCIILower(buffer[1]) == 'a'
        && toASCIILower(buffer[2]) == 't'
        && toASCIILower(buffer[3]) == 'r'
        && toASCIILower(buffer[4]) == 'i'
        && toASCIILower(buffer[5]) == 'x'
        && buffer[6] == '3'
        && toASCIILower(buffer[7]) == 'd'
        && buffer[8] == '(';

    if (isMatrix3d) {
        buffer.advanceBy(9);
        CSSValueListBuilder arguments;
        if (!parseTransformNumberArguments(buffer, 16, arguments))
            return nullptr;
        return CSSFunctionValue::create(CSSValueID::Matrix3d, WTF::move(arguments));
    }

    bool isScale3d = toASCIILower(buffer[0]) == 's'
        && toASCIILower(buffer[1]) == 'c'
        && toASCIILower(buffer[2]) == 'a'
        && toASCIILower(buffer[3]) == 'l'
        && toASCIILower(buffer[4]) == 'e'
        && buffer[5] == '3'
        && toASCIILower(buffer[6]) == 'd'
        && buffer[7] == '(';

    if (isScale3d) {
        buffer.advanceBy(8);
        CSSValueListBuilder arguments;
        if (!parseTransformNumberArguments(buffer, 3, arguments))
            return nullptr;
        return CSSFunctionValue::create(CSSValueID::Scale3d, WTF::move(arguments));
    }

    bool isRotate = toASCIILower(buffer[0]) == 'r'
        && toASCIILower(buffer[1]) == 'o'
        && toASCIILower(buffer[2]) == 't'
        && toASCIILower(buffer[3]) == 'a'
        && toASCIILower(buffer[4]) == 't'
        && toASCIILower(buffer[5]) == 'e';

    if (isRotate) {
        CSSValueID transformType;
        unsigned argumentStart = 7;
        CharType c6 = toASCIILower(buffer[6]);
        if (c6 == '(') {
            transformType = CSSValueID::Rotate;
        } else if (c6 == 'z' && buffer[7] == '(') {
            transformType = CSSValueID::RotateZ;
            argumentStart = 8;
        } else
            return nullptr;

        buffer.advanceBy(argumentStart);
        RefPtr angle = parseTransformAngleArgument(buffer);
        if (!angle)
            return nullptr;
        return CSSFunctionValue::create(transformType, angle.releaseNonNull());
    }

    return nullptr;
}

template<typename CharacterType>
static RefPtr<CSSValue> parseSimpleTransformList(std::span<const CharacterType> characters)
{
    StringParsingBuffer<CharacterType> buffer(characters);
    CSSValueListBuilder builder;
    while (buffer.hasCharactersRemaining()) {
        while (buffer.hasCharactersRemaining() && isCSSSpace(*buffer))
            buffer.consume();
        if (buffer.atEnd())
            break;
        auto transformValue = parseSimpleTransformValue(buffer);
        if (!transformValue)
            return nullptr;
        builder.append(transformValue.releaseNonNull());
    }
    if (builder.isEmpty())
        return nullptr;
    return CSSTransformListValue::create(WTF::move(builder));
}

static RefPtr<CSSValue> parseSimpleTransform(StringView string)
{
    ASSERT(!string.isEmpty());
    if (string.is8Bit())
        return parseSimpleTransformList(string.span8());
    return parseSimpleTransformList(string.span16());
}

static RefPtr<CSSValue> parseDisplay(StringView string)
{
    ASSERT(!string.isEmpty());
    auto valueID = cssValueKeywordID(string);

    switch (valueID) {
    case CSSValueID::None:
    // <display-outside>
    case CSSValueID::Block:
    case CSSValueID::Inline:
    // <display-inside> (except for CSSValueID::Flow since it becomes "block")
    case CSSValueID::Flex:
    case CSSValueID::FlowRoot:
    case CSSValueID::Grid:
    case CSSValueID::Table:
    // <display-internal>
    case CSSValueID::TableCaption:
    case CSSValueID::TableCell:
    case CSSValueID::TableColumnGroup:
    case CSSValueID::TableColumn:
    case CSSValueID::TableHeaderGroup:
    case CSSValueID::TableFooterGroup:
    case CSSValueID::TableRow:
    case CSSValueID::TableRowGroup:
    // <display-legacy>
    case CSSValueID::InlineBlock:
    case CSSValueID::InlineFlex:
    case CSSValueID::InlineGrid:
    case CSSValueID::InlineTable:
    // Prefixed values
    case CSSValueID::WebkitInlineBox:
    case CSSValueID::WebkitBox:
    // No layout support for the full <display-listitem> syntax, so treat it as <display-legacy>
    case CSSValueID::ListItem:
        return CSSKeywordValue::create(valueID);
    default:
        if (isCSSWideKeyword(valueID))
            return CSSKeywordValue::create(valueID);
        return nullptr;
    }
}

static RefPtr<CSSValue> parseOpacity(StringView string)
{
    double number;

    if (string.is8Bit()) {
        if (!parseSimpleNumberOrPercentageDividedBy100(string.span8(), number))
            return nullptr;
    } else {
        if (!parseSimpleNumberOrPercentageDividedBy100(string.span16(), number))
            return nullptr;
    }

    return CSSPrimitiveValue::create(number, CSSUnitType::Number);
}

static RefPtr<CSSValue> parseColorWithAuto(StringView string, const CSSParserContext& context)
{
    ASSERT(!string.isEmpty());
    if (cssValueKeywordID(string) == CSSValueID::Auto)
        return CSSKeywordValue::create(CSSValueID::Auto);
    return parseColor(string, context);
}

RefPtr<CSSValue> CSSParserFastPaths::maybeParseValue(CSSPropertyID property, StringView string, CSS::PropertyParserState& state)
{
    // Some at-rules like @keyframes, @position-try restrict which properties
    // are allowed inside the rule. Fallback to slow path for at-rules since
    // the restriction logic is in the slow-path parser (CSSPropertyParser).
    if (state.currentRule != StyleRuleType::Style)
        return nullptr;

    switch (property) {
    case CSSPropertyID::Display:
        return parseDisplay(string);
    case CSSPropertyID::Opacity:
        return parseOpacity(string);
    case CSSPropertyID::Transform:
        return parseSimpleTransform(string);
    case CSSPropertyID::CaretColor:
    case CSSPropertyID::AccentColor:
        if (isExposed(property, &state.context.propertySettings))
            return parseColorWithAuto(string, state.context);
        break;
    default:
        break;
    }

    if (CSSProperty::isColorProperty(property)) {
        auto context = state.context;
        if (!CSSProperty::acceptsQuirkyColor(property))
            context.mode = HTMLStandardMode;
        return parseColor(string, context);
    }

    if (auto valueRange = lengthValueRangeForPropertiesSupportingSimpleLengths(property)) {
        if (auto result = parseSimpleLengthValue(string, state.context.mode, *valueRange))
            return result;
    }

    return parseKeywordValue(property, string, state);
}

} // namespace WebCore
