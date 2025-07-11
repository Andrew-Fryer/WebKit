/**
 * Copyright (C) 2023 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1.  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "ShorthandSerializer.h"

#include "CSSBorderImageWidthValue.h"
#include "CSSGridLineNamesValue.h"
#include "CSSGridTemplateAreasValue.h"
#include "CSSParserIdioms.h"
#include "CSSPendingSubstitutionValue.h"
#include "CSSPropertyInitialValues.h"
#include "CSSPropertyNames.h"
#include "CSSPropertyParser.h"
#include "CSSPropertyParserConsumer+Font.h"
#include "CSSPropertyParserConsumer+Grid.h"
#include "CSSPropertyParserConsumer+Ident.h"
#include "CSSSerializationContext.h"
#include "CSSValueKeywords.h"
#include "CSSValueList.h"
#include "CSSValuePair.h"
#include "CSSVariableReferenceValue.h"
#include "FontSelectionValueInlines.h"
#include "Quad.h"
#include "StyleExtractor.h"
#include "StylePropertiesInlines.h"
#include "StylePropertyShorthand.h"
#include "TimelineRange.h"
#include <algorithm>
#include <wtf/IndexedRange.h>
#include <wtf/text/MakeString.h>

namespace WebCore {

constexpr unsigned maxShorthandLength = 18; // FIXME: Generate this from CSSProperties.json.

class ShorthandSerializer {
public:
    template<typename PropertiesType> explicit ShorthandSerializer(const CSS::SerializationContext&, const PropertiesType&, CSSPropertyID shorthandID);
    String serialize();

private:
    struct Longhand {
        CSSPropertyID property;
        CSSValue& value;
    };
    struct LonghandIteratorBase {
        void operator++() { ++index; }
        bool operator==(std::nullptr_t) const { return index >= serializer.length(); }
        const ShorthandSerializer& serializer;
        unsigned index { 0 };
    };
    struct LonghandIterator : LonghandIteratorBase {
        Longhand operator*() const { return { serializer.longhand(index) }; }
    };
    struct LonghandValueIterator : LonghandIteratorBase {
        CSSValue& operator*() const { return { serializer.longhandValue(index) }; }
    };
    template<typename IteratorType> struct LonghandRange {
        IteratorType begin() const { return { { serializer } }; }
        static constexpr std::nullptr_t end() { return nullptr; }
        unsigned size() const { return serializer.length(); }
        const ShorthandSerializer& serializer;
    };

    static bool isInitialValue(Longhand);
    String serializeValue(Longhand) const;

    unsigned length() const { return m_shorthand.length(); }
    Longhand longhand(unsigned index) const { return { longhandProperty(index), longhandValue(index) }; }
    CSSPropertyID longhandProperty(unsigned index) const;
    CSSValue& longhandValue(unsigned index) const;

    unsigned longhandIndex(unsigned index, CSSPropertyID) const;

    LonghandRange<LonghandIterator> longhands() const { return { *this }; }
    LonghandRange<LonghandValueIterator> longhandValues() const { return { *this }; }

    CSSValueID longhandValueID(unsigned index) const;
    bool isLonghandValueID(unsigned index, CSSValueID valueID) const { return longhandValueID(index) == valueID; }
    bool isLonghandValueNone(unsigned index) const { return isLonghandValueID(index, CSSValueNone); }
    bool isLonghandInitialValue(unsigned index) const { return isInitialValue(longhand(index)); }
    String serializeLonghandValue(unsigned index) const;

    bool subsequentLonghandsHaveInitialValues(unsigned index) const;

    bool commonSerializationChecks(const Style::Extractor&);
    bool commonSerializationChecks(const StyleProperties&);

    String serializeLonghands() const;
    String serializeLonghands(unsigned lengthLimit, ASCIILiteral separator = " "_s) const;
    String serializeLonghandsOmittingInitialValues() const;
    String serializeLonghandsOmittingTrailingInitialValue(ASCIILiteral separator = " "_s) const;

    String serializeCommonValue() const;
    String serializeCommonValue(unsigned startIndex, unsigned count) const;
    String serializePair() const;
    String serializeQuad() const;

    String serializeLayered() const;
    String serializeCoordinatingListPropertyGroup() const;

    String serializeBorder(unsigned sectionLength) const;
    String serializeBorderImage() const;
    String serializeBorderRadius() const;
    String serializeBreakInside() const;
    String serializeColumnBreak() const;
    String serializeFont() const;
    String serializeFontSynthesis() const;
    String serializeFontVariant() const;
    String serializeGrid() const;
    String serializeGridArea() const;
    String serializeGridRowColumn() const;
    String serializeGridTemplate() const;
    String serializeOffset() const;
    String serializePageBreak() const;
    String serializePositionTry() const;
    String serializeLineClamp() const;
    String serializeTextBox() const;
    String serializeTextWrap() const;
    String serializeWhiteSpace() const;
    String serializeAnimationRange() const;
    String serializeSingleAnimationRange(const CSSValue&, SingleTimelineRange::Type, CSSValueID = CSSValueInvalid) const;

    StylePropertyShorthand m_shorthand;
    std::array<RefPtr<CSSValue>, maxShorthandLength> m_longhandValues;
    String m_result;
    bool m_commonSerializationChecksSuppliedResult { false };
    const CSS::SerializationContext& m_serializationContext;
};

template<typename PropertiesType>
inline ShorthandSerializer::ShorthandSerializer(const CSS::SerializationContext& serializationContext, const PropertiesType& properties, CSSPropertyID shorthandID)
    : m_shorthand(shorthandForProperty(shorthandID))
    , m_commonSerializationChecksSuppliedResult(commonSerializationChecks(properties))
    , m_serializationContext(serializationContext)
{
}

inline CSSPropertyID ShorthandSerializer::longhandProperty(unsigned index) const
{
    return m_shorthand.properties()[index];
}

inline CSSValue& ShorthandSerializer::longhandValue(unsigned index) const
{
    return *m_longhandValues[index];
}

inline String ShorthandSerializer::serializeValue(Longhand longhand) const
{
    return WebCore::serializeLonghandValue(m_serializationContext, longhand.property, longhand.value);
}

inline bool ShorthandSerializer::isInitialValue(Longhand longhand)
{
    return isInitialValueForLonghand(longhand.property, longhand.value);
}

inline unsigned ShorthandSerializer::longhandIndex(unsigned index, CSSPropertyID longhand) const
{
    ASSERT_UNUSED(longhand, longhandProperty(index) == longhand);
    return index;
}

inline CSSValueID ShorthandSerializer::longhandValueID(unsigned index) const
{
    return WebCore::longhandValueID(longhandProperty(index), longhandValue(index));
}

inline String ShorthandSerializer::serializeLonghandValue(unsigned index) const
{
    return serializeValue(longhand(index));
}

bool ShorthandSerializer::subsequentLonghandsHaveInitialValues(unsigned startIndex) const
{
    for (unsigned i = startIndex; i < length(); ++i) {
        if (!isLonghandInitialValue(i))
            return false;
    }
    return true;
}

bool ShorthandSerializer::commonSerializationChecks(const Style::Extractor& properties)
{
    ASSERT(length() && length() <= maxShorthandLength);

    ASSERT(m_shorthand.id() != CSSPropertyAll);

    for (unsigned i = 0; i < length(); ++i) {
        auto longhandValue = properties.propertyValue(longhandProperty(i));
        if (!longhandValue) {
            m_result = emptyString();
            return true;
        }
        m_longhandValues[i] = longhandValue;
    }

    return false;
}

bool ShorthandSerializer::commonSerializationChecks(const StyleProperties& properties)
{
    ASSERT(length());
    ASSERT(length() <= maxShorthandLength || m_shorthand.id() == CSSPropertyAll);

    std::optional<CSSValueID> specialKeyword;
    bool allSpecialKeywords = true;
    std::optional<bool> importance;
    std::optional<CSSPendingSubstitutionValue*> firstValueFromShorthand;
    String commonValue;
    for (unsigned i = 0; i < length(); ++i) {
        auto longhand = longhandProperty(i);

        int propertyIndex = properties.findPropertyIndex(longhand);
        if (propertyIndex == -1)
            return true;
        auto property = properties.propertyAt(propertyIndex);

        // Don't serialize if longhands have different importance.
        bool isImportant = property.isImportant();
        if (importance.value_or(isImportant) != isImportant)
            return true;
        importance = isImportant;

        auto value = property.value();

        // Don't serialize if longhands have different CSS-wide keywords.
        if (!isCSSWideKeyword(valueID(*value)) || value->isImplicitInitialValue()) {
            if (specialKeyword)
                return true;
            allSpecialKeywords = false;
        } else {
            if (!allSpecialKeywords)
                return true;
            auto keyword = valueID(value);
            if (specialKeyword.value_or(keyword) != keyword)
                return true;
            specialKeyword = keyword;
            continue;
        }

        // Don't serialize if any longhand was set to a variable.
        if (is<CSSVariableReferenceValue>(value))
            return true;

        // Don't serialize if any longhand was set by a different shorthand.
        auto* valueFromShorthand = dynamicDowncast<CSSPendingSubstitutionValue>(value);
        if (valueFromShorthand && valueFromShorthand->shorthandPropertyId() != m_shorthand.id())
            return true;

        // Don't serialize if longhands are inconsistent about whether they were set by the shorthand.
        if (!firstValueFromShorthand)
            firstValueFromShorthand = valueFromShorthand;
        else {
            bool wasSetByShorthand = valueFromShorthand;
            bool firstWasSetByShorthand = *firstValueFromShorthand;
            if (firstWasSetByShorthand != wasSetByShorthand)
                return true;
        }

        if (m_shorthand.id() != CSSPropertyAll)
            m_longhandValues[i] = WTFMove(value);
    }
    if (specialKeyword) {
        m_result = nameString(*specialKeyword);
        return true;
    }
    if (*firstValueFromShorthand) {
        m_result = (*firstValueFromShorthand)->shorthandValue().cssText(m_serializationContext);
        return true;
    }
    return false;
}

String ShorthandSerializer::serialize()
{
    if (m_commonSerializationChecksSuppliedResult)
        return WTFMove(m_result);

    switch (m_shorthand.id()) {
    case CSSPropertyAll:
        return String();
    case CSSPropertyAnimation:
    case CSSPropertyBackground:
    case CSSPropertyBackgroundPosition:
    case CSSPropertyMask:
    case CSSPropertyMaskPosition:
    case CSSPropertyTransition:
    case CSSPropertyWebkitMask:
    case CSSPropertyWebkitMaskPosition:
        return serializeLayered();
    case CSSPropertyBorder:
        return serializeBorder(4);
    case CSSPropertyBorderBlock:
    case CSSPropertyBorderInline:
        return serializeBorder(2);
    case CSSPropertyBorderBlockColor:
    case CSSPropertyBorderBlockStyle:
    case CSSPropertyBorderBlockWidth:
    case CSSPropertyBorderInlineColor:
    case CSSPropertyBorderInlineStyle:
    case CSSPropertyBorderInlineWidth:
    case CSSPropertyBorderSpacing:
    case CSSPropertyContainIntrinsicSize:
    case CSSPropertyGap:
    case CSSPropertyInsetBlock:
    case CSSPropertyInsetInline:
    case CSSPropertyMarginBlock:
    case CSSPropertyMarginInline:
    case CSSPropertyOverflow:
    case CSSPropertyOverscrollBehavior:
    case CSSPropertyPaddingBlock:
    case CSSPropertyPaddingInline:
    case CSSPropertyPlaceContent:
    case CSSPropertyPlaceItems:
    case CSSPropertyPlaceSelf:
    case CSSPropertyScrollMarginBlock:
    case CSSPropertyScrollMarginInline:
    case CSSPropertyScrollPaddingBlock:
    case CSSPropertyScrollPaddingInline:
        return serializePair();
    case CSSPropertyBlockStep:
    case CSSPropertyBorderBlockEnd:
    case CSSPropertyBorderBlockStart:
    case CSSPropertyBorderBottom:
    case CSSPropertyBorderInlineEnd:
    case CSSPropertyBorderInlineStart:
    case CSSPropertyBorderLeft:
    case CSSPropertyBorderRight:
    case CSSPropertyBorderTop:
    case CSSPropertyColumnRule:
    case CSSPropertyColumns:
    case CSSPropertyFlexFlow:
    case CSSPropertyListStyle:
    case CSSPropertyOutline:
    case CSSPropertyTextEmphasis:
    case CSSPropertyWebkitTextDecoration:
    case CSSPropertyWebkitTextStroke:
        return serializeLonghandsOmittingInitialValues();
    case CSSPropertyBorderColor:
    case CSSPropertyBorderStyle:
    case CSSPropertyBorderWidth:
    case CSSPropertyCornerShape:
    case CSSPropertyInset:
    case CSSPropertyMargin:
    case CSSPropertyPadding:
    case CSSPropertyScrollMargin:
    case CSSPropertyScrollPadding:
        return serializeQuad();
    case CSSPropertyBorderImage:
    case CSSPropertyWebkitBorderImage:
    case CSSPropertyWebkitMaskBoxImage:
    case CSSPropertyMaskBorder:
        return serializeBorderImage();
    case CSSPropertyBorderRadius:
    case CSSPropertyWebkitBorderRadius:
        return serializeBorderRadius();
    case CSSPropertyContainer:
        return serializeLonghandsOmittingTrailingInitialValue(" / "_s);
    case CSSPropertyFlex:
    case CSSPropertyPerspectiveOrigin:
        return serializeLonghands();
    case CSSPropertyFont:
        return serializeFont();
    case CSSPropertyFontVariant:
        return serializeFontVariant();
    case CSSPropertyFontSynthesis:
        return serializeFontSynthesis();
    case CSSPropertyGrid:
        return serializeGrid();
    case CSSPropertyGridArea:
        return serializeGridArea();
    case CSSPropertyGridColumn:
    case CSSPropertyGridRow:
        return serializeGridRowColumn();
    case CSSPropertyGridTemplate:
        return serializeGridTemplate();
    case CSSPropertyLineClamp:
        return serializeLineClamp();
    case CSSPropertyMarker:
        return serializeCommonValue();
    case CSSPropertyOffset:
        return serializeOffset();
    case CSSPropertyPageBreakAfter:
    case CSSPropertyPageBreakBefore:
        return serializePageBreak();
    case CSSPropertyPageBreakInside:
    case CSSPropertyWebkitColumnBreakInside:
        return serializeBreakInside();
    case CSSPropertyPositionTry:
        return serializePositionTry();
    case CSSPropertyTextDecorationSkip:
    case CSSPropertyTextDecoration:
    case CSSPropertyWebkitBackgroundSize:
    case CSSPropertyWebkitPerspective:
    case CSSPropertyWebkitTextOrientation:
        return serializeLonghandValue(0);
    case CSSPropertyTransformOrigin:
        return serializeLonghandsOmittingTrailingInitialValue();
    case CSSPropertyTextWrap:
        return serializeTextWrap();
    case CSSPropertyTextBox:
        return serializeTextBox();
    case CSSPropertyWebkitColumnBreakAfter:
    case CSSPropertyWebkitColumnBreakBefore:
        return serializeColumnBreak();
    case CSSPropertyWhiteSpace:
        return serializeWhiteSpace();
    case CSSPropertyScrollTimeline:
    case CSSPropertyViewTimeline:
        return serializeCoordinatingListPropertyGroup();
    case CSSPropertyAnimationRange:
        return serializeAnimationRange();
    default:
        ASSERT_NOT_REACHED();
        return String();
    }
}

String ShorthandSerializer::serializeLonghands() const
{
    return serializeLonghands(length());
}

String ShorthandSerializer::serializeLonghands(unsigned lengthLimit, ASCIILiteral separator) const
{
    ASSERT(lengthLimit <= length());
    switch (lengthLimit) {
    case 1:
        return serializeLonghandValue(0);
    case 2:
        return makeString(serializeLonghandValue(0), separator, serializeLonghandValue(1));
    case 3:
        return makeString(serializeLonghandValue(0), separator, serializeLonghandValue(1), separator, serializeLonghandValue(2));
    case 4:
        return makeString(serializeLonghandValue(0), separator, serializeLonghandValue(1), separator, serializeLonghandValue(2), separator, serializeLonghandValue(3));
    default:
        StringBuilder result;
        auto prefix = ""_s;
        for (unsigned i = 0; i < lengthLimit; ++i)
            result.append(std::exchange(prefix, separator), serializeLonghandValue(i));
        return result.toString();
    }
}

String ShorthandSerializer::serializeLonghandsOmittingInitialValues() const
{
    StringBuilder result;
    auto prefix = ""_s;
    for (auto longhand : longhands()) {
        if (!isInitialValue(longhand))
            result.append(std::exchange(prefix, " "_s), serializeValue(longhand));
    }
    return result.isEmpty() ? serializeLonghandValue(0) : result.toString();
}

String ShorthandSerializer::serializeLonghandsOmittingTrailingInitialValue(ASCIILiteral separator) const
{
    ASSERT(length() >= 2);
    return serializeLonghands(length() - isLonghandInitialValue(length() - 1), separator);
}

inline String ShorthandSerializer::serializeCommonValue() const
{
    return serializeCommonValue(0, length());
}

String ShorthandSerializer::serializeCommonValue(unsigned startIndex, unsigned count) const
{
    String result;
    for (unsigned i = 0; i < count; ++i) {
        String text = serializeLonghandValue(startIndex + i);
        if (result.isNull())
            result = text;
        else if (result != text)
            return String();
    }
    return result;
}

String ShorthandSerializer::serializePair() const
{
    ASSERT(length() == 2);
    auto first = serializeLonghandValue(0);
    auto second = serializeLonghandValue(1);
    if (first != second)
        return makeString(first, ' ', second);
    return first;
}

String ShorthandSerializer::serializeQuad() const
{
    ASSERT(length() == 4);
    return Quad::serialize(serializeLonghandValue(0), serializeLonghandValue(1), serializeLonghandValue(2), serializeLonghandValue(3));
}

class LayerValues {
public:
    explicit LayerValues(const StylePropertyShorthand& shorthand)
        : m_shorthand(shorthand)
    {
        ASSERT(m_shorthand.length() <= maxShorthandLength);
    }

    void set(unsigned index, const CSSValue* value, bool skipSerializing = false)
    {
        ASSERT(index < m_shorthand.length());
        m_skipSerializing[index] = skipSerializing
            || !value || isInitialValueForLonghand(m_shorthand.properties()[index], *value);
        m_values[index] = value;
    }

    bool& skip(unsigned index)
    {
        ASSERT(index < m_shorthand.length());
        return m_skipSerializing[index];
    }

    std::optional<CSSValueID> valueID(unsigned index) const
    {
        return longhandValueID(m_shorthand.properties()[index], m_values[index].get());
    }

    CSSValueID valueIDIncludingCustomIdent(unsigned index) const
    {
        auto* value = dynamicDowncast<CSSPrimitiveValue>(m_values[index].get());
        if (value && value->isCustomIdent())
            return cssValueKeywordID(value->stringValue());
        return valueID(index).value_or(CSSValueInvalid);
    }

    bool equalValueIDs(unsigned indexA, unsigned indexB) const
    {
        auto valueA = valueID(indexA);
        auto valueB = valueID(indexB);
        return valueA && valueB && *valueA == *valueB;
    }

    bool isValueID(unsigned index) const
    {
        auto result = valueID(index);
        return result && *result != CSSValueInvalid;
    }

    bool isPair(unsigned index) const
    {
        // This returns false for implicit initial values that are pairs, which is OK for now.
        ASSERT(index < m_shorthand.length());
        auto* value = m_values[index].get();
        return value && value->isPair();
    }

    void serialize(StringBuilder& builder, const CSS::SerializationContext& context) const
    {
        // If all are skipped, then serialize the first.
        auto range = std::span { m_skipSerializing }.first(m_shorthand.length());
        bool allSkipped = std::ranges::find(range, false) == range.end();

        auto separator = builder.isEmpty() ? ""_s : ", "_s;
        auto longhands = m_shorthand.properties();
        for (auto [j, longhand] : indexedRange(longhands)) {
            if (allSkipped ? j : m_skipSerializing[j])
                continue;
            if (longhand == CSSPropertyBackgroundSize || longhand == CSSPropertyMaskSize)
                separator = " / "_s;
            if (auto& value = m_values[j])
                builder.append(separator, serializeLonghandValue(context, longhand, *value));
            else
                builder.append(separator, initialValueTextForLonghand(longhand));
            separator = " "_s;
        }
    }

private:
    const StylePropertyShorthand& m_shorthand;
    std::array<bool, maxShorthandLength> m_skipSerializing = { };
    std::array<RefPtr<const CSSValue>, maxShorthandLength> m_values;
};

String ShorthandSerializer::serializeCoordinatingListPropertyGroup() const
{
    ASSERT(length() > 1);

    // https://drafts.csswg.org/css-values-4/#linked-properties

    // First, figure out the number of items in the coordinating list base property,
    // which we'll need to match for all coordinated longhands, thus possibly trimming
    // or expanding.
    unsigned numberOfItemsForCoordinatingListBaseProperty = 1;
    if (auto* valueList = dynamicDowncast<CSSValueList>(longhandValue(0)))
        numberOfItemsForCoordinatingListBaseProperty = std::max(valueList->length(), numberOfItemsForCoordinatingListBaseProperty);

    // Now go through all longhands and ensure we repeat items earlier in the list
    // should there not be a specified value.
    StringBuilder result;
    for (unsigned listItemIndex = 0; listItemIndex < numberOfItemsForCoordinatingListBaseProperty; ++listItemIndex) {
        LayerValues layerValues { m_shorthand };
        for (unsigned longhandIndex = 0; longhandIndex < length(); ++longhandIndex) {
            auto& value = longhandValue(longhandIndex);
            if (auto* valueList = dynamicDowncast<CSSValueList>(value)) {
                auto* valueInList = [&]() -> const CSSValue* {
                    if (auto* specifiedValue = valueList->item(listItemIndex))
                        return specifiedValue;
                    if (auto numberOfItemsInList = valueList->size())
                        return valueList->item(listItemIndex % numberOfItemsInList);
                    return nullptr;
                }();
                layerValues.set(longhandIndex, valueInList);
            } else
                layerValues.set(longhandIndex, &value);
        }
        // The coordinating list base property must never be skipped.
        layerValues.skip(0) = false;
        layerValues.serialize(result, m_serializationContext);
    }
    return result.toString();
}

String ShorthandSerializer::serializeLayered() const
{
    unsigned numLayers = 1;
    for (auto& value : longhandValues()) {
        if (auto* valueList = dynamicDowncast<CSSValueList>(value))
            numLayers = std::max(valueList->length(), numLayers);
    }

    StringBuilder result;
    for (unsigned i = 0; i < numLayers; i++) {
        LayerValues layerValues { m_shorthand };

        for (unsigned j = 0; j < length(); j++) {
            auto& value = longhandValue(j);
            if (auto* valueList = dynamicDowncast<CSSValueList>(value))
                layerValues.set(j, valueList->item(i));
            else {
                // Color is only in the last layer. Other singletons are only in the first.
                auto singletonLayer = longhandProperty(j) == CSSPropertyBackgroundColor ? numLayers - 1 : 0;
                layerValues.set(j, &value, i != singletonLayer);
            }
        }

        bool hasUnskippedValue = false;
        for (unsigned j = 0; j < length(); j++) {
            auto longhand = longhandProperty(j);

            // A single box value sets both background-origin and background-clip.
            // A single geometry-box value sets both mask-origin and mask-clip.
            // A single geometry-box value sets both mask-origin and -webkit-mask-clip.
            if (longhand == CSSPropertyBackgroundClip || longhand == CSSPropertyMaskClip || longhand == CSSPropertyWebkitMaskClip) {
                // The previous property is origin.
                ASSERT(j >= 1);
                ASSERT(longhandProperty(j - 1) == CSSPropertyBackgroundOrigin
                    || longhandProperty(j - 1) == CSSPropertyMaskOrigin);
                if (layerValues.equalValueIDs(j - 1, j)) {
                    // If the two are the same, one value sets both.
                    if (!layerValues.skip(j - 1) && !layerValues.skip(j))
                        layerValues.skip(j) = true;
                } else if (!layerValues.skip(j - 1) || !layerValues.skip(j)) {
                    // If the two are different, both need to be serialized, unless clip is invalid as origin
                    if (layerValues.valueID(j) == CSSValueNoClip)
                        continue;
                    if (layerValues.valueID(j) == CSSValueBorderArea) {
                        layerValues.skip(j - 1) = layerValues.valueID(j - 1) == CSSValueBorderBox;
                        continue;
                    }
                    layerValues.skip(j - 1) = false;
                    layerValues.skip(j) = false;
                }
            }

            // A single background-position value (identifier or numeric) sets the other value to center.
            // A single mask-position value (identifier or numeric) sets the other value to center.
            // Order matters when one is numeric, but not when both are identifiers.
            if (longhand == CSSPropertyBackgroundPositionY || longhand == CSSPropertyWebkitMaskPositionY) {
                // The previous property is X.
                ASSERT(j >= 1);
                ASSERT(longhandProperty(j - 1) == CSSPropertyBackgroundPositionX || longhandProperty(j - 1) == CSSPropertyWebkitMaskPositionX);
                if (length() == 2) {
                    ASSERT(j == 1);
                    layerValues.skip(0) = false;
                    layerValues.skip(1) = false;
                } else {
                    // Always serialize positions to at least 2 values.
                    // https://drafts.csswg.org/css-values-4/#position-serialization
                    if (!layerValues.skip(j - 1))
                        layerValues.skip(j) = false;
                }
            }

            // If we get to "animation-timeline" and yet haven't encountered an unskipped value,
            // this means that this "animation" shorthand only has initial values for the non
            // reset-only longhands and so we cannot serialize it. We only do this when we deal
            // with multiple layers since the single "none" case would be caught otherwise.
            if (numLayers > 1 && longhand == CSSPropertyAnimationTimeline && !hasUnskippedValue)
                return String();

            if (layerValues.skip(j))
                continue;

            hasUnskippedValue = true;

            // If we encounter one of the reset-only "animation" longhands and the value was not skipped,
            // then it was set to a non-initial value and we cannot serialize it.
            if (longhand == CSSPropertyAnimationTimeline || longhand == CSSPropertyAnimationRangeStart || longhand == CSSPropertyAnimationRangeEnd)
                return String();

            // The syntax for background-size means that if it is present, background-position must be too.
            // The syntax for mask-size means that if it is present, mask-position must be too.
            if (longhand == CSSPropertyBackgroundSize || longhand == CSSPropertyMaskSize) {
                // The previous properties are X and Y.
                ASSERT(j >= 2);
                ASSERT(longhandProperty(j - 2) == CSSPropertyBackgroundPositionX
                    || longhandProperty(j - 2) == CSSPropertyWebkitMaskPositionX);
                ASSERT(longhandProperty(j - 1) == CSSPropertyBackgroundPositionY
                    || longhandProperty(j - 1) == CSSPropertyWebkitMaskPositionY);
                layerValues.skip(j - 2) = false;
                layerValues.skip(j - 1) = false;
            }

            // The first value in each animation shorthand that can be parsed as a time is assigned to
            // animation-duration, and the second is assigned to animation-delay, so we must serialize
            // both if we are serializing animation-delay.
            if (longhand == CSSPropertyAnimationDelay)
                layerValues.skip(longhandIndex(0, CSSPropertyAnimationDuration)) = false;
        }

        // In the animation shorthand, if the value of animation-name could be parsed as one of
        // the other longhands that longhand must be serialized to avoid ambiguity.
        if (m_shorthand.id() == CSSPropertyAnimation) {
            auto animationTimingFunctionIndex = longhandIndex(1, CSSPropertyAnimationTimingFunction);
            auto animationIterationCountIndex = longhandIndex(3, CSSPropertyAnimationIterationCount);
            auto animationDirectionIndex = longhandIndex(4, CSSPropertyAnimationDirection);
            auto animationFillModeIndex = longhandIndex(5, CSSPropertyAnimationFillMode);
            auto animationPlayStateIndex = longhandIndex(6, CSSPropertyAnimationPlayState);
            auto animationNameIndex = longhandIndex(7, CSSPropertyAnimationName);
            if (!layerValues.skip(animationNameIndex)) {
                switch (layerValues.valueIDIncludingCustomIdent(animationNameIndex)) {
                case CSSValueAlternate:
                case CSSValueAlternateReverse:
                case CSSValueNormal:
                case CSSValueReverse:
                    layerValues.skip(animationDirectionIndex) = false;
                    break;
                case CSSValueBackwards:
                case CSSValueBoth:
                case CSSValueForwards:
                    layerValues.skip(animationFillModeIndex) = false;
                    break;
                case CSSValueEase:
                case CSSValueEaseIn:
                case CSSValueEaseInOut:
                case CSSValueEaseOut:
                case CSSValueLinear:
                case CSSValueStepEnd:
                case CSSValueStepStart:
                    layerValues.skip(animationTimingFunctionIndex) = false;
                    break;
                case CSSValueInfinite:
                    layerValues.skip(animationIterationCountIndex) = false;
                    break;
                case CSSValuePaused:
                case CSSValueRunning:
                    layerValues.skip(animationPlayStateIndex) = false;
                    break;
                default:
                    break;
                }
            }
        }

        layerValues.serialize(result, m_serializationContext);
    }

    return result.toString();
}

String ShorthandSerializer::serializeBorder(unsigned sectionLength) const
{
    ASSERT(3 * sectionLength <= length());

    bool mustSerializeAsEmptyString = false;
    auto serializeSection = [&](unsigned index, CSSValueID defaultValue) {
        auto value = serializeCommonValue(index * sectionLength, sectionLength);
        if (value.isNull())
            mustSerializeAsEmptyString = true;
        else if (value == nameLiteral(defaultValue))
            value = String();
        return value;
    };
    auto width = serializeSection(0, CSSValueMedium); // widths
    auto style = serializeSection(1, CSSValueNone); // styles
    auto color = serializeSection(2, CSSValueCurrentcolor); // colors
    if (mustSerializeAsEmptyString || !subsequentLonghandsHaveInitialValues(3 * sectionLength))
        return String();

    unsigned bits = !width.isNull() << 2 | !style.isNull() << 1 | !color.isNull();
    switch (bits) {
    case 0b000: return nameString(CSSValueMedium);
    case 0b001: return color;
    case 0b010: return style;
    case 0b011: return makeString(style, ' ', color);
    case 0b100: return width;
    case 0b101: return makeString(width, ' ', color);
    case 0b110: return makeString(width, ' ', style);
    case 0b111: return makeString(width, ' ', style, ' ', color);
    }

    ASSERT_NOT_REACHED();
    return String();
}

String ShorthandSerializer::serializeBorderImage() const
{
    auto isLength = [](const CSSValue& value) {
        RefPtr primitive = dynamicDowncast<CSSPrimitiveValue>(value);
        return primitive && primitive->isLength();
    };

    ASSERT(length() == 5);
    StringBuilder result;
    bool omittedSlice = false;
    bool omittedWidth = false;
    auto separator = ""_s;
    for (auto longhand : longhands()) {
        if (isInitialValue(longhand)) {
            if (longhand.property == CSSPropertyBorderImageSlice || longhand.property == CSSPropertyMaskBorderSlice)
                omittedSlice = true;
            else if (longhand.property == CSSPropertyBorderImageWidth || longhand.property == CSSPropertyMaskBorderWidth)
                omittedWidth = true;
            continue;
        }
        if (omittedSlice && (longhand.property == CSSPropertyBorderImageWidth || longhand.property == CSSPropertyBorderImageOutset || longhand.property == CSSPropertyMaskBorderWidth || longhand.property == CSSPropertyMaskBorderOutset))
            return String();

        String valueText;

        // -webkit-border-image has a legacy behavior that makes fixed border slices also set the border widths.
        if (auto* width = dynamicDowncast<CSSBorderImageWidthValue>(longhand.value)) {
            auto& widths = width->widths();
            bool overridesBorderWidths = m_shorthand.id() == CSSPropertyWebkitBorderImage && (isLength(widths.top()) || isLength(widths.right()) || isLength(widths.bottom()) || isLength(widths.left()));
            if (overridesBorderWidths != width->overridesBorderWidths())
                return String();
            valueText = widths.cssText(m_serializationContext);
        } else
            valueText = serializeValue(longhand);

        // Append separator and text.
        if (longhand.property == CSSPropertyBorderImageWidth || longhand.property == CSSPropertyMaskBorderWidth)
            separator = " / "_s;
        else if (longhand.property == CSSPropertyBorderImageOutset || longhand.property == CSSPropertyMaskBorderOutset)
            separator = omittedWidth ? " / / "_s : " / "_s;
        result.append(separator, valueText);
        separator = " "_s;
    }
    if (result.isEmpty())
        return nameString(CSSValueNone);
    return result.toString();
}

String ShorthandSerializer::serializeBorderRadius() const
{
    ASSERT(length() == 4);
    std::array<RefPtr<const CSSValue>, 4> horizontalRadii;
    std::array<RefPtr<const CSSValue>, 4> verticalRadii;
    for (unsigned i = 0; i < 4; ++i) {
        auto& value = longhandValue(i);
        horizontalRadii[i] = value.first();
        verticalRadii[i] = value.second();
    }

    bool serializeBoth = false;
    for (unsigned i = 0; i < 4; ++i) {
        if (!horizontalRadii[i]->equals(*verticalRadii[i])) {
            serializeBoth = true;
            break;
        }
    }

    StringBuilder result;
    auto serializeRadii = [&](const std::array<RefPtr<const CSSValue>, 4>& r) {
        if (!r[3]->equals(*r[1]))
            result.append(r[0]->cssText(m_serializationContext), ' ', r[1]->cssText(m_serializationContext), ' ', r[2]->cssText(m_serializationContext), ' ', r[3]->cssText(m_serializationContext));
        else if (!r[2]->equals(*r[0]) || (m_shorthand.id() == CSSPropertyWebkitBorderRadius && !serializeBoth && !r[1]->equals(*r[0])))
            result.append(r[0]->cssText(m_serializationContext), ' ', r[1]->cssText(m_serializationContext), ' ', r[2]->cssText(m_serializationContext));
        else if (!r[1]->equals(*r[0]))
            result.append(r[0]->cssText(m_serializationContext), ' ', r[1]->cssText(m_serializationContext));
        else
            result.append(r[0]->cssText(m_serializationContext));
    };
    serializeRadii(horizontalRadii);
    if (serializeBoth) {
        result.append(" / "_s);
        serializeRadii(verticalRadii);
    }
    return result.toString();
}

String ShorthandSerializer::serializeBreakInside() const
{
    auto keyword = longhandValueID(0);
    switch (keyword) {
    case CSSValueAuto:
    case CSSValueAvoid:
        return nameString(keyword);
    default:
        return String();
    }
}

String ShorthandSerializer::serializeColumnBreak() const
{
    switch (longhandValueID(0)) {
    case CSSValueColumn:
        return nameString(CSSValueAlways);
    case CSSValueAvoidColumn:
        return nameString(CSSValueAvoid);
    case CSSValueAuto:
        return nameString(CSSValueAuto);
    default:
        return String();
    }
}

static std::optional<CSSValueID> fontWidthKeyword(double value)
{
    // If the numeric value does not fit in the fixed point FontSelectionValue, don't convert it to a keyword even if it rounds to a keyword value.
    float valueAsFloat = value;
    FontSelectionValue valueAsFontSelectionValue { valueAsFloat };
    float valueAsFloatAfterRoundTrip = valueAsFontSelectionValue;
    if (value != valueAsFloatAfterRoundTrip)
        return std::nullopt;
    return fontWidthKeyword(valueAsFontSelectionValue);
}

String ShorthandSerializer::serializeFont() const
{
    // If all properties are set to the same system font shorthand, serialize as that.
    // If some but not all properties are, the font shorthand can't represent that, serialize as empty string.
    std::optional<CSSValueID> specialKeyword;
    bool allSpecialKeywords = true;
    for (auto& longhandValue : longhandValues()) {
        auto keyword = valueID(longhandValue);
        if (!CSSPropertyParserHelpers::isSystemFontShorthand(keyword))
            allSpecialKeywords = false;
        else {
            if (specialKeyword.value_or(keyword) != keyword)
                return String();
            specialKeyword = keyword;
        }
    }
    if (specialKeyword)
        return allSpecialKeywords ? nameString(*specialKeyword) : String();

    auto styleIndex = longhandIndex(0, CSSPropertyFontStyle);
    auto capsIndex = longhandIndex(1, CSSPropertyFontVariantCaps);
    auto weightIndex = longhandIndex(2, CSSPropertyFontWeight);
    auto widthIndex = longhandIndex(3, CSSPropertyFontWidth);
    auto sizeIndex = longhandIndex(4, CSSPropertyFontSize);
    auto lineHeightIndex = longhandIndex(5, CSSPropertyLineHeight);
    auto familyIndex = longhandIndex(6, CSSPropertyFontFamily);

    // Properties after font-family are reset but not represented by the shorthand.
    // If any is not the initial value, serialize as empty string.
    if (!subsequentLonghandsHaveInitialValues(familyIndex + 1))
        return String();

    // Only two values of variant-caps can be serialized in the font shorthand.
    // If the value is anything else, serialize as empty string.
    auto capsKeyword = longhandValueID(capsIndex);
    if (capsKeyword != CSSValueNormal && capsKeyword != CSSValueSmallCaps)
        return String();

    // Font width values can only be serialized in the font shorthand as keywords, since percentages are also valid font sizes.
    // If a font width percentage can be expressed as a keyword, then do that.
    auto widthKeyword = longhandValueID(widthIndex);
    if (widthKeyword == CSSValueInvalid) {
        auto& widthValue = downcast<CSSPrimitiveValue>(longhandValue(widthIndex));
        if (widthValue.isCalculated() || !widthValue.isPercentage())
            return String();
        auto keyword = fontWidthKeyword(widthValue.resolveAsPercentageNoConversionDataRequired());
        if (!keyword)
            return String();
        widthKeyword = *keyword;
    }

    bool includeStyle = !isLonghandInitialValue(styleIndex);
    bool includeCaps = capsKeyword != CSSValueNormal;
    bool includeWeight = !isLonghandInitialValue(weightIndex);
    bool includeWidth = widthKeyword != CSSValueNormal;
    bool includeLineHeight = !isLonghandInitialValue(lineHeightIndex);

    auto style = includeStyle ? serializeLonghandValue(styleIndex) : String();
    auto capsSeparator = includeCaps && includeStyle ? " "_s : ""_s;
    auto caps = includeCaps ? nameLiteral(capsKeyword) : ""_s;
    auto weightSeparator = includeWeight && (includeStyle || includeCaps) ? " "_s : ""_s;
    auto weight = includeWeight ? serializeLonghandValue(weightIndex) : String();
    auto widthSeparator = includeWidth && (includeStyle || includeCaps || includeWeight) ? " "_s : ""_s;
    auto width = includeWidth ? nameLiteral(widthKeyword) : ""_s;
    auto sizeSeparator = includeStyle || includeCaps || includeWeight || includeWidth ? " "_s : ""_s;
    auto lineHeightSeparator = includeLineHeight ? " / "_s : ""_s;
    auto lineHeight = includeLineHeight ? serializeLonghandValue(lineHeightIndex) : String();

    return makeString(style,
        capsSeparator, caps,
        weightSeparator, weight,
        widthSeparator, width,
        sizeSeparator, serializeLonghandValue(sizeIndex),
        lineHeightSeparator, lineHeight,
        ' ', serializeLonghandValue(familyIndex));
}

String ShorthandSerializer::serializeFontSynthesis() const
{
    // font-synthesis: none | [ weight || style || small-caps ]
    ASSERT(length() == 3);

    unsigned bits = !isLonghandValueNone(longhandIndex(0, CSSPropertyFontSynthesisWeight)) << 2
        | !isLonghandValueNone(longhandIndex(1, CSSPropertyFontSynthesisStyle)) << 1
        | !isLonghandValueNone(longhandIndex(2, CSSPropertyFontSynthesisSmallCaps));

    switch (bits) {
    case 0b000: return nameString(CSSValueNone);
    case 0b001: return nameString(CSSValueSmallCaps);
    case 0b010: return nameString(CSSValueStyle);
    case 0b011: return "style small-caps"_s;
    case 0b100: return nameString(CSSValueWeight);
    case 0b101: return "weight small-caps"_s;
    case 0b110: return "weight style"_s;
    case 0b111: return "weight style small-caps"_s;
    }

    ASSERT_NOT_REACHED();
    return String();
}

String ShorthandSerializer::serializeFontVariant() const
{
    for (auto& value : longhandValues()) {
        if (CSSPropertyParserHelpers::isSystemFontShorthand(valueID(value)))
            return String();
    }
    if (isLonghandValueNone(longhandIndex(0, CSSPropertyFontVariantLigatures))) {
        for (auto longhand : longhands()) {
            // font-variant cannot represent "font-variant-ligatures: none" along with any other non-normal longhands.
            if (longhand.property != CSSPropertyFontVariantLigatures && !isInitialValue(longhand))
                return String();
        }
    }
    return serializeLonghandsOmittingInitialValues();
}

static bool isValueIDIncludingList(const CSSValue& value, CSSValueID id)
{
    if (auto* valueList = dynamicDowncast<CSSValueList>(value)) {
        if (valueList->size() != 1)
            return false;
        auto* item = valueList->item(0);
        return item && isValueID(*item, id);
    }
    return isValueID(value, id);
}

static bool gridAutoFlowContains(CSSValue& autoFlow, CSSValueID id)
{
    if (auto* valueList = dynamicDowncast<CSSValueList>(autoFlow)) {
        for (auto& currentValue : *valueList) {
            if (isValueID(currentValue, id))
                return true;
        }
        return false;
    }
    return isValueID(autoFlow, id);
}

String ShorthandSerializer::serializeGrid() const
{
    ASSERT(length() == 6);

    auto rowsIndex = longhandIndex(0, CSSPropertyGridTemplateRows);
    auto columnsIndex = longhandIndex(1, CSSPropertyGridTemplateColumns);
    auto areasIndex = longhandIndex(2, CSSPropertyGridTemplateAreas);
    auto autoFlowIndex = longhandIndex(3, CSSPropertyGridAutoFlow);
    auto autoRowsIndex = longhandIndex(4, CSSPropertyGridAutoRows);
    auto autoColumnsIndex = longhandIndex(5, CSSPropertyGridAutoColumns);

    auto& autoColumns = longhandValue(autoColumnsIndex);
    auto& autoRows = longhandValue(autoRowsIndex);
    auto& autoFlow = longhandValue(autoFlowIndex);

    if (isValueIDIncludingList(autoColumns, CSSValueAuto) && isValueIDIncludingList(autoRows, CSSValueAuto) && isValueIDIncludingList(autoFlow, CSSValueRow))
        return serializeGridTemplate();

    if (!isLonghandValueNone(areasIndex))
        return String();

    auto& rows = longhandValue(rowsIndex);
    auto& columns = longhandValue(columnsIndex);

    bool autoFlowContainsDense = gridAutoFlowContains(autoFlow, CSSValueDense);
    auto dense = autoFlowContainsDense ? " dense"_s : ""_s;

    if (gridAutoFlowContains(autoFlow, CSSValueColumn)) {
        if (!isValueIDIncludingList(autoRows, CSSValueAuto) || !isValueIDIncludingList(columns, CSSValueNone))
            return String();

        if (isValueIDIncludingList(autoColumns, CSSValueAuto))
            return makeString(serializeLonghandValue(rowsIndex), " / auto-flow"_s, dense);
        return makeString(serializeLonghandValue(rowsIndex), " / auto-flow"_s, dense, ' ', serializeLonghandValue(autoColumnsIndex));
    }

    if (!gridAutoFlowContains(autoFlow, CSSValueRow) && !autoFlowContainsDense)
        return String();
    if (!isValueIDIncludingList(autoColumns, CSSValueAuto) || !isValueIDIncludingList(rows, CSSValueNone))
        return String();

    if (isValueIDIncludingList(autoRows, CSSValueAuto))
        return makeString("auto-flow"_s, dense, " / "_s, serializeLonghandValue(columnsIndex));
    return makeString("auto-flow"_s, dense, ' ', serializeLonghandValue(autoRowsIndex), " / "_s, serializeLonghandValue(columnsIndex));
}

static bool canOmitTrailingGridAreaValue(CSSValue& value, CSSValue& trailing, const CSS::SerializationContext& context)
{
    if (isCustomIdentValue(value))
        return isCustomIdentValue(trailing) && value.cssText(context) == trailing.cssText(context);
    return isValueID(trailing, CSSValueAuto);
}

String ShorthandSerializer::serializeGridArea() const
{
    ASSERT(length() == 4);
    unsigned longhandsToSerialize = 4;
    if (canOmitTrailingGridAreaValue(longhandValue(1), longhandValue(3), m_serializationContext)) {
        --longhandsToSerialize;
        if (canOmitTrailingGridAreaValue(longhandValue(0), longhandValue(2), m_serializationContext)) {
            --longhandsToSerialize;
            if (canOmitTrailingGridAreaValue(longhandValue(0), longhandValue(1), m_serializationContext))
                --longhandsToSerialize;
        }
    }
    return serializeLonghands(longhandsToSerialize, " / "_s);
}

String ShorthandSerializer::serializeGridRowColumn() const
{
    ASSERT(length() == 2);
    return serializeLonghands(canOmitTrailingGridAreaValue(longhandValue(0), longhandValue(1), m_serializationContext) ? 1 : 2, " / "_s);
}

String ShorthandSerializer::serializeGridTemplate() const
{
    ASSERT(length() >= 3);

    auto rowsIndex = longhandIndex(0, CSSPropertyGridTemplateRows);
    auto columnsIndex = longhandIndex(1, CSSPropertyGridTemplateColumns);
    auto areasIndex = longhandIndex(2, CSSPropertyGridTemplateAreas);

    auto* areasValue = dynamicDowncast<CSSGridTemplateAreasValue>(longhandValue(areasIndex));
    if (!areasValue) {
        if (isLonghandValueNone(rowsIndex) && isLonghandValueNone(columnsIndex))
            return nameString(CSSValueNone);
        return serializeLonghands(2, " / "_s);
    }

    // Depending on the values of grid-template-rows and grid-template-columns, we may not
    // be able to completely represent them in this version of the grid-template shorthand.
    // We need to make sure that those values map to a value the syntax supports
    auto isValidTrackSize = [&] (const CSSValue& value) {
        auto valueID = value.valueID();
        if (CSSPropertyParserHelpers::identMatches<CSSValueFitContent, CSSValueMinmax>(valueID) || CSSPropertyParserHelpers::isGridBreadthIdent(valueID))
            return true;
        if (const auto* primitiveValue = dynamicDowncast<CSSPrimitiveValue>(value))
            return primitiveValue->isLength() || primitiveValue->isPercentage() || primitiveValue->isCalculated() || primitiveValue->isFlex();
        return false;
    };
    auto isValidExplicitTrackList = [&] (const CSSValue& value) {
        const auto* values = dynamicDowncast<CSSValueList>(value);
        if (!values)
            return isValidTrackSize(value);

        auto hasAtLeastOneTrackSize = false;
        for (const auto& value : *values) {
            if (isValidTrackSize(value))
                hasAtLeastOneTrackSize = true;
            else if (!value.isGridLineNamesValue())
                return false;
        }
        return hasAtLeastOneTrackSize;
    };

    Ref rowTrackSizes = longhandValue(rowsIndex);

    // Make sure the longhands can be expressed in this version of the shorthand.
    if (!rowTrackSizes->isValueList() || (!isLonghandValueNone(columnsIndex) && !isValidExplicitTrackList(longhandValue(columnsIndex))))
        return String();

    StringBuilder result;
    unsigned row = 0;
    for (auto& currentValue : downcast<CSSValueList>(rowTrackSizes).get()) {
        if (!result.isEmpty())
            result.append(' ');
        if (auto lineNames = dynamicDowncast<CSSGridLineNamesValue>(currentValue))
            result.append(lineNames->customCSSText(m_serializationContext));
        else {
            result.append('"', areasValue->stringForRow(row), '"');
            if (!isValidTrackSize(currentValue))
                return String();
            if (!isValueID(currentValue, CSSValueAuto))
                result.append(' ', currentValue.cssText(m_serializationContext));
            row++;
        }
    }
    if (!isLonghandValueNone(columnsIndex))
        result.append(" / "_s, serializeLonghandValue(columnsIndex));
    return result.toString();
}

String ShorthandSerializer::serializeOffset() const
{
    auto positionIndex = longhandIndex(0, CSSPropertyOffsetPosition);
    auto pathIndex = longhandIndex(1, CSSPropertyOffsetPath);
    auto distanceIndex = longhandIndex(2, CSSPropertyOffsetDistance);
    auto rotateIndex = longhandIndex(3, CSSPropertyOffsetRotate);
    auto anchorIndex = longhandIndex(4, CSSPropertyOffsetAnchor);

    bool includeDistance = !isLonghandInitialValue(distanceIndex);
    bool includeRotate = !isLonghandInitialValue(rotateIndex);
    bool includePath = includeDistance || includeRotate || !isLonghandInitialValue(pathIndex);
    bool includePosition = !includePath || !isLonghandInitialValue(positionIndex);
    bool includeAnchor = !isLonghandInitialValue(anchorIndex);

    if (!includeDistance && !includeRotate && !includeAnchor) {
        if (includePosition && includePath)
            return serializeLonghands(2);
        if (includePosition)
            return serializeLonghandValue(positionIndex);
        ASSERT(includePath);
        return serializeLonghandValue(pathIndex);
    }

    auto position = includePosition ? serializeLonghandValue(positionIndex) : String();
    auto pathSeparator = includePosition && includePath ? " "_s : ""_s;
    auto path = includePath ? serializeLonghandValue(pathIndex) : String();
    auto distanceSeparator = includeDistance ? " "_s : ""_s;
    auto distance = includeDistance ? serializeLonghandValue(distanceIndex) : String();
    auto rotateSeparator = includeRotate ? " "_s : ""_s;
    auto rotate = includeRotate ? serializeLonghandValue(rotateIndex) : String();
    auto anchorSeparator = includeAnchor ? " / "_s : ""_s;
    auto anchor = includeAnchor ? serializeLonghandValue(anchorIndex) : String();

    return makeString(position,
        pathSeparator, path,
        distanceSeparator, distance,
        rotateSeparator, rotate,
        anchorSeparator, anchor);
}

String ShorthandSerializer::serializePageBreak() const
{
    auto keyword = longhandValueID(0);
    switch (keyword) {
    case CSSValuePage:
        return nameString(CSSValueAlways);
    case CSSValueAuto:
    case CSSValueAvoid:
    case CSSValueLeft:
    case CSSValueRight:
        return nameString(keyword);
    default:
        return String();
    }
}

String ShorthandSerializer::serializePositionTry() const
{
    auto positionTryOrderIndex = longhandIndex(0, CSSPropertyPositionTryOrder);
    auto positionTryFallbacksIndex = longhandIndex(1, CSSPropertyPositionTryFallbacks);

    auto positionTryFallbacksSerialization = serializeLonghandValue(positionTryFallbacksIndex);
    if (isLonghandInitialValue(positionTryOrderIndex))
        return positionTryFallbacksSerialization;

    return makeString(serializeLonghandValue(positionTryOrderIndex), " "_s, positionTryFallbacksSerialization);
}

String ShorthandSerializer::serializeLineClamp() const
{
    auto isMaxLinesInitial = isLonghandInitialValue(0);
    auto isBlockEllipsisInitial = isLonghandInitialValue(1);
    if (isMaxLinesInitial && isBlockEllipsisInitial)
        return nameString(CSSValueNone);

    if (isMaxLinesInitial != isBlockEllipsisInitial)
        return { };

    auto blockEllipsis = longhandValueID(1);
    if (isBlockEllipsisInitial || (!isMaxLinesInitial && blockEllipsis == CSSValueAuto))
        return serializeLonghands(1);

    // FIXME: Add check for correct order.
    return serializeLonghands(2);
}

String ShorthandSerializer::serializeTextBox() const
{
    auto textBoxTrim = longhandValueID(0);
    auto& textBoxEdge = longhandValue(longhandIndex(1, CSSPropertyTextBoxEdge));
    auto textBoxEdgeIsAuto = [&]() {
        if (auto* primitiveValue = dynamicDowncast<CSSPrimitiveValue>(textBoxEdge))
            return primitiveValue->valueID() == CSSValueAuto;
        return false;
    }();

    if (textBoxTrim == CSSValueNone && textBoxEdgeIsAuto)
        return nameString(CSSValueNormal);

    if (textBoxEdgeIsAuto)
        return nameLiteral(textBoxTrim);

    if (textBoxTrim == CSSValueTrimBoth)
        return textBoxEdge.cssText(m_serializationContext);

    return makeString(nameLiteral(textBoxTrim), ' ', textBoxEdge.cssText(m_serializationContext));
}

String ShorthandSerializer::serializeTextWrap() const
{
    auto mode = longhandValueID(0);
    auto style = longhandValueID(1);

    if (style == CSSValueAuto)
        return nameString(mode);
    if (mode == CSSValueWrap)
        return nameString(style);

    return makeString(nameLiteral(mode), ' ', nameLiteral(style));
}

String ShorthandSerializer::serializeSingleAnimationRange(const CSSValue& value, SingleTimelineRange::Type type, CSSValueID startValueID) const
{
    if (RefPtr pair = dynamicDowncast<CSSValuePair>(value)) {
        bool isSameNameAsStart = pair->first().valueID() == startValueID;
        bool isStartValue = type == SingleTimelineRange::Type::Start;
        bool isDefaultValue = SingleTimelineRange::isDefault(downcast<CSSPrimitiveValue>(pair->second()), SingleTimelineRange::Type::Start);
        if (isDefaultValue && (isStartValue || !isSameNameAsStart))
            return nameLiteral(pair->first().valueID());
        return pair->cssText(m_serializationContext);
    }
    if (RefPtr primitiveValue = dynamicDowncast<CSSPrimitiveValue>(value)) {
        if (SingleTimelineRange::isOffsetValue(*primitiveValue))
            return primitiveValue->cssText(m_serializationContext);
        bool isNormal = primitiveValue->valueID() == CSSValueNormal;
        bool isSameNameAsStart = primitiveValue->valueID() == startValueID;
        bool isStartValue = type == SingleTimelineRange::Type::Start;
        if (isStartValue || (!isNormal && !isSameNameAsStart))
            return nameLiteral(primitiveValue->valueID());
    }
    return emptyString();
}

String ShorthandSerializer::serializeAnimationRange() const
{
    StringBuilder builder;
    auto& startValue = longhandValue(0);
    auto& endValue = longhandValue(1);
    auto* startList = dynamicDowncast<CSSValueList>(startValue);
    auto* endList = dynamicDowncast<CSSValueList>(endValue);
    if (startList && endList) {
        if (startList->size() != endList->size())
            return emptyString();

        for (unsigned i = 0; i < startList->size(); i++) {
            auto start = startList->item(i);
            RefPtr startPair = dynamicDowncast<CSSValuePair>(start);
            auto startID = startPair ? startPair->first().valueID() : start->valueID();

            auto serializedStart = serializeSingleAnimationRange(*start, SingleTimelineRange::Type::Start);
            auto serializedEnd = serializeSingleAnimationRange(*endList->item(i), SingleTimelineRange::Type::End, startID);
            builder.append(
                serializedEnd.isEmpty() ? serializedStart : makeString(serializedStart, ' ', serializedEnd),
                (i < startList->size() - 1) ? ", "_s : emptyString()
            );
        }
        return builder.toString();
    }

    RefPtr startPair = dynamicDowncast<CSSValuePair>(startValue);
    auto startID = startPair ? startPair->first().valueID() : startValue.valueID();

    auto serializedStart = serializeSingleAnimationRange(startValue, SingleTimelineRange::Type::Start);
    auto serializedEnd = serializeSingleAnimationRange(endValue, SingleTimelineRange::Type::End, startID);
    if (serializedEnd.isEmpty())
        return serializedStart;
    return makeString(serializedStart, ' ', serializedEnd);
}

String ShorthandSerializer::serializeWhiteSpace() const
{
    auto whiteSpaceCollapse = longhandValueID(0);
    auto textWrapMode = longhandValueID(1);

    // Convert to backwards-compatible keywords if possible.
    if (whiteSpaceCollapse == CSSValueCollapse && textWrapMode == CSSValueWrap)
        return nameString(CSSValueNormal);
    if (whiteSpaceCollapse == CSSValuePreserve && textWrapMode == CSSValueNowrap)
        return nameString(CSSValuePre);
    if (whiteSpaceCollapse == CSSValuePreserve && textWrapMode == CSSValueWrap)
        return nameString(CSSValuePreWrap);
    if (whiteSpaceCollapse == CSSValuePreserveBreaks && textWrapMode == CSSValueWrap)
        return nameString(CSSValuePreLine);

    // Omit default longhand values.
    if (whiteSpaceCollapse == CSSValueCollapse)
        return nameString(textWrapMode);
    if (textWrapMode == CSSValueWrap)
        return nameString(whiteSpaceCollapse);

    return makeString(nameLiteral(whiteSpaceCollapse), ' ', nameLiteral(textWrapMode));
}

String serializeShorthandValue(const CSS::SerializationContext& context, const StyleProperties& properties, CSSPropertyID shorthand)
{
    return ShorthandSerializer(context, properties, shorthand).serialize();
}

String serializeShorthandValue(const CSS::SerializationContext& context, const Style::Extractor& extractor, CSSPropertyID shorthand)
{
    return ShorthandSerializer(context, extractor, shorthand).serialize();
}

}
