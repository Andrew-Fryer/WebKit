/*
 * Copyright (C) 2019-2026 Apple Inc. All rights reserved.
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
#include "PropertyCascade.h"

#include "CSSCustomPropertyValue.h"
#include "CSSKeywordValueInlines.h"
#include "CSSPaintImageValue.h"
#include "CSSShorthandSubstitutionValue.h"
#include "CSSSubstitutionValue.h"
#include "CSSValuePool.h"
#include "ComputedStyleDependencies.h"
#include "PaintWorkletGlobalScope.h"
#include "PropertyAllowlist.h"
#include "StyleBuilderGenerated.h"
#include "StyleKeyword+Mappings.h"
#include "StylePropertyShorthand.h"
#include <ranges>
#include <wtf/TZoneMallocInlines.h>

#if USE(SWIFT_KERNELS)
#include "ImmutableStyleProperties.h"
#include "MutableStyleProperties.h"
#include "PropertyCascadeSwiftBoundary.h"
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

namespace WebCore {
namespace Style {

WTF_MAKE_TZONE_ALLOCATED_IMPL(PropertyCascade);

PropertyCascade::PropertyCascade(const MatchResult& matchResult, IncludedProperties&& includedProperties, const HashMap<AnimatableCSSProperty, EnumSet<AnimationSource>>* animatedProperties, const StyleProperties* positionTryFallbackProperties)
    : m_matchResult(matchResult)
    , m_includedProperties(WTF::move(includedProperties))
    , m_maximumOrigin(positionTryFallbackProperties ? PropertyCascade::Origin::PositionFallback : PropertyCascade::Origin::Author)
    , m_animationLayer(animatedProperties ? std::optional { AnimationLayer { *animatedProperties } } : std::nullopt)
{
    ASSERT(!m_includedProperties.isEmpty());

    if (positionTryFallbackProperties)
        m_positionTryFallbackProperties = MatchedProperties { *positionTryFallbackProperties };

    buildCascade();
}

PropertyCascade::IncludedProperties PropertyCascade::rollbackProperties(const PropertyCascade& parent)
{
    auto types = normalPropertyTypes();
    // Preserve starting-style resolution so that a revert inside @starting-style can roll back to
    // @starting-style declarations from lower priority layers or origins.
    if (parent.m_includedProperties.types.contains(PropertyType::StartingStyle))
        types.add(PropertyType::StartingStyle);
    return { types };
}

PropertyCascade::PropertyCascade(const PropertyCascade& parent, Origin maximumOrigin, std::optional<ScopeOrdinal> rollbackScope, std::optional<CascadeLayerPriority> maximumCascadeLayerPriorityForRollback)
    : m_matchResult(parent.m_matchResult)
    , m_includedProperties(rollbackProperties(parent)) // Include all properties to the rollback cascade, lower prority layers may not get included otherwise.
    , m_maximumOrigin(maximumOrigin)
    , m_rollbackScope(rollbackScope)
    , m_maximumCascadeLayerPriorityForRollback(maximumCascadeLayerPriorityForRollback)
    , m_animationLayer(parent.m_animationLayer)
    , m_positionTryFallbackProperties(parent.m_positionTryFallbackProperties)
{
    buildCascade();
}

// Constructs a cascade where all properties in the parent cascade have been reverted.
PropertyCascade::PropertyCascade(const PropertyCascade& parent, RevertRuleTag)
    : m_matchResult(parent.m_matchResult)
    , m_includedProperties(rollbackProperties(parent))
    , m_maximumOrigin(parent.m_maximumOrigin)
    , m_rollbackScope(parent.m_rollbackScope)
    , m_maximumCascadeLayerPriorityForRollback(parent.m_maximumCascadeLayerPriorityForRollback)
    , m_ruleRollbackDepth(parent.m_ruleRollbackDepth + 1) // Increment rollback depth.
    , m_animationLayer(parent.m_animationLayer)
    , m_positionTryFallbackProperties(parent.m_positionTryFallbackProperties)
{
    buildCascade();
}

PropertyCascade::~PropertyCascade() = default;

PropertyCascade::AnimationLayer::AnimationLayer(const HashMap<AnimatableCSSProperty, EnumSet<AnimationSource>>& properties)
    : properties(properties)
{
    for (auto& key : properties.keys()) {
        if (std::holds_alternative<AtomString>(key)) {
            hasCustomProperties = true;
            break;
        }
    }

    hasFontSize = properties.contains(CSSPropertyID::FontSize);
    hasLineHeight = properties.contains(CSSPropertyID::LineHeight);
}

#if USE(SWIFT_KERNELS)

// The layouts PropertyCascadeSwiftBoundary.h documents for the Swift kernel.
static_assert(sizeof(PropertyCascade::Property) == 40);
static_assert(offsetof(PropertyCascade::Property, id) == 0 && offsetof(PropertyCascade::Property, origin) == 2);
static_assert(offsetof(PropertyCascade::Property, styleScopeOrdinal) == 3 && offsetof(PropertyCascade::Property, cascadeLayerPriority) == 4);
static_assert(offsetof(PropertyCascade::Property, fromStyleAttribute) == 6 && offsetof(PropertyCascade::Property, cssValue) == 8);
static_assert(offsetof(PropertyCascade::Property, origins) == 32 && sizeof(CSSValue*) == 8);
static_assert(std::is_trivially_copyable_v<PropertyCascade::Property>);
static_assert(sizeof(StylePropertyMetadata) == 4 && sizeof(CSSProperty) == 16 && alignof(CSSProperty) == 8 && sizeof(Ref<CSSValue>) == 8);
static_assert(std::is_same_v<PropertyCascade::PropertyBitSet::WordType, uint64_t> && sizeof(CSSPropertyID) == 2);
static_assert(std::to_underlying(CSSPropertyID::Custom) == 1 && cssPropertyIDEnumValueCount <= 0x8000);
static_assert(SelectorChecker::MatchAll == 3 && std::to_underlying(IsCacheable::Partially) == 1);
static_assert(std::to_underlying(ScopeOrdinal::Element) == 0 && RuleSet::cascadeLayerPriorityForUnlayered == 0xFFFF);
static_assert(std::to_underlying(PropertyCascade::PropertyType::NonInherited) == 1 && std::to_underlying(PropertyCascade::PropertyType::Inherited) == 2);
static_assert(std::to_underlying(PropertyCascade::PropertyType::ExplicitlyInherited) == 4 && std::to_underlying(PropertyCascade::PropertyType::AfterAnimation) == 8);
static_assert(std::to_underlying(PropertyCascade::PropertyType::StartingStyle) == 16 && std::to_underlying(PropertyCascade::PropertyType::NonCacheable) == 32);
static_assert(sizeof(MatchedProperties) == 16 && offsetof(MatchedProperties, linkMatchType) == 8 && offsetof(MatchedProperties, allowlistType) == 9);
static_assert(offsetof(MatchedProperties, styleScopeOrdinal) == 10 && offsetof(MatchedProperties, fromStyleAttribute) == 11);
static_assert(offsetof(MatchedProperties, cascadeLayerPriority) == 12 && offsetof(MatchedProperties, isStartingStyle) == 14 && offsetof(MatchedProperties, isCacheable) == 15);
static_assert(sizeof(PropertyAllowlist) == 1 && sizeof(ScopeOrdinal) == 1 && sizeof(FromStyleAttribute) == 1 && sizeof(CascadeLayerPriority) == 2 && sizeof(IsStartingStyle) == 1 && sizeof(IsCacheable) == 1);
static_assert(!std::to_underlying(PropertyAllowlist::None));

// For each CSSPropertyID: cascadeAliasProperty(id) | CSSProperty::isInheritedProperty(id) << 15.
static std::span<const uint16_t> propertyTableForSwift()
{
    static const std::array<uint16_t, cssPropertyIDEnumValueCount> table = [] {
        std::array<uint16_t, cssPropertyIDEnumValueCount> table { };
        for (uint16_t id = std::to_underlying(CSSPropertyID::Custom); id < cssPropertyIDEnumValueCount; ++id) {
            auto propertyID = static_cast<CSSPropertyID>(id);
            table[id] = std::to_underlying(cascadeAliasProperty(propertyID)) | (CSSProperty::isInheritedProperty(propertyID) ? 0x8000 : 0);
        }
        return table;
    }();
    return table;
}

#endif // USE(SWIFT_KERNELS)

void PropertyCascade::buildCascade()
{
#if USE(SWIFT_KERNELS)
    if (buildCascadeWithSwift()) {
#if ENABLE(SWIFT_KERNELS_VALIDATION)
        if (swiftKernelsValidationEnabled()) [[unlikely]]
            validateSwiftCascade();
#endif
        return;
    }
#endif
    buildCascadeCxx();
}

#if USE(SWIFT_KERNELS)

// Builds the cascade with the Swift kernel. Returns false for the cascades the kernel leaves to
// buildCascadeCxx(), with the cascade still in the fresh state buildCascadeCxx() starts from (the kernel
// may have written Property records, which the C++ treats as uninitialized).
bool PropertyCascade::buildCascadeWithSwift()
{
    if (m_ruleRollbackDepth || m_rollbackScope || m_maximumCascadeLayerPriorityForRollback || m_positionTryFallbackProperties || m_maximumOrigin != Origin::Author)
        return false;
    auto types = m_includedProperties.types;
    if (types.contains(PropertyType::AfterAnimation) || (types.contains(PropertyType::ExplicitlyInherited) && !types.containsAll(normalPropertyTypes())))
        return false;
    ASSERT(m_propertyIsPresent.isEmpty() && !m_lastIndexForLogicalGroup && m_customProperties.isEmpty());

    auto& userAgentDeclarations = m_matchResult.userAgentDeclarations;
    auto& userDeclarations = m_matchResult.userDeclarations;
    auto& authorDeclarations = m_matchResult.authorDeclarations;
    size_t userAgentCount = userAgentDeclarations.size();
    size_t userCount = userDeclarations.size();
    size_t authorCount = authorDeclarations.size();

    Vector<uint64_t, 3 * 32> matchWords(3 * (userAgentCount + userCount + authorCount));
    auto matchWordSpan = matchWords.mutableSpan();
    size_t wordIndex = 0;
    size_t propertyTotal = 0;
    for (auto* declarations : { &userAgentDeclarations, &userDeclarations, &authorDeclarations }) {
        for (auto& matchedProperties : *declarations) {
            if (matchedProperties.allowlistType != PropertyAllowlist::None)
                return false;
            auto& properties = matchedProperties.properties.get();
            auto words = matchWordSpan.subspan(wordIndex).first<3>();
            if (properties.isMutable()) {
                auto storage = asByteSpan(uncheckedDowncast<MutableStyleProperties>(properties).propertySpan());
                size_t count = storage.size() / sizeof(CSSProperty);
                if (count >= 1u << 28)
                    return false;
                words[0] = reinterpret_cast<uintptr_t>(storage.data());
                words[2] = storage.size() | uint64_t { count } << 32 | 1ull << 63;
                propertyTotal += count;
            } else {
                auto& immutableProperties = uncheckedDowncast<ImmutableStyleProperties>(properties);
                size_t count = immutableProperties.propertyCount();
                words[0] = reinterpret_cast<uintptr_t>(&immutableProperties.m_storage);
                words[2] = count * (sizeof(StylePropertyMetadata) + sizeof(PackedPtr<const CSSValue>)) | uint64_t { count } << 32;
                propertyTotal += count;
            }
            // linkMatchType through isCacheable, in their byte order (static_asserted above).
            memcpySpan(asMutableByteSpan(words[1]), asByteSpan(matchedProperties).subspan(offsetof(MatchedProperties, linkMatchType), sizeof(uint64_t)));
            wordIndex += 3;
        }
    }

    size_t scratchCount = std::max<size_t>({ logicalGroupPropertyCount, userAgentCount, userCount, authorCount });
    Vector<uint64_t, 256> work(scratchCount + propertyTotal);
    std::array<uint32_t, 7> config { static_cast<uint32_t>(userAgentCount), static_cast<uint32_t>(userCount), static_cast<uint32_t>(authorCount),
        types.toRaw(), std::to_underlying(firstLogicalGroupProperty), sizeof(PackedPtr<const CSSValue>), static_cast<uint32_t>(scratchCount) };
    std::array<uint32_t, 4> scalars { m_lastIndexForLogicalGroup, m_seenLogicalGroupPropertyCount,
        std::to_underlying(m_lowestSeenLogicalGroupProperty), std::to_underlying(m_highestSeenLogicalGroupProperty) };
    auto includedIDs = spanReinterpretCast<const uint16_t>(m_includedProperties.ids.span());
    auto table = propertyTableForSwift();
    auto properties = asMutableByteSpan(std::span { m_properties });
    auto propertyIsPresent = m_propertyIsPresent.storage();
    auto logicalGroupIDs = spanReinterpretCast<uint16_t>(std::span { m_logicalGroupPropertyIDs });

    auto customCount = WebCoreSwift_PropertyCascade_build(matchWords.span().data(), matchWords.size(), config.data(), config.size(),
        includedIDs.data(), includedIDs.size(), table.data(), table.size(), properties.data(), properties.size(),
        propertyIsPresent.data(), propertyIsPresent.size(), m_logicalGroupPropertyIndices.data(), logicalGroupIDs.data(), logicalGroupPropertyCount,
        scalars.data(), work.mutableSpan().data(), work.size());
    if (customCount < 0)
        return false;

    m_lastIndexForLogicalGroup = scalars[0];
    m_seenLogicalGroupPropertyCount = scalars[1];
    m_lowestSeenLogicalGroupProperty = static_cast<CSSPropertyID>(scalars[2]);
    m_highestSeenLogicalGroupProperty = static_cast<CSSPropertyID>(scalars[3]);

    // The custom property sets, in cascade order.
    for (auto entry : work.span().subspan(scratchCount, customCount)) {
        size_t matchIndex = entry >> 32;
        unsigned propertyIndex = static_cast<uint32_t>(entry);
        auto origin = Origin::UserAgent;
        auto* declarations = &userAgentDeclarations;
        if (matchIndex >= userAgentCount + userCount) {
            origin = Origin::Author;
            declarations = &authorDeclarations;
            matchIndex -= userAgentCount + userCount;
        } else if (matchIndex >= userAgentCount) {
            origin = Origin::User;
            declarations = &userDeclarations;
            matchIndex -= userAgentCount;
        }
        auto& matchedProperties = (*declarations)[matchIndex];
        auto current = matchedProperties.properties->propertyAt(propertyIndex);
        SUPPRESS_UNCOUNTED_ARG set(CSSPropertyID::Custom, *current.value(), matchedProperties, origin);
    }

    m_delayedRollbackProperties.clear();
    return true;
}

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// Checks the cascade the Swift kernel built against buildCascadeCxx() on the same, freshly reset, state.
void PropertyCascade::validateSwiftCascade()
{
    Vector<Property> swiftProperties(std::span<const Property> { m_properties });
    auto swiftPropertyIsPresent = m_propertyIsPresent;
    auto swiftLogicalGroupPropertyIndices = m_logicalGroupPropertyIndices;
    auto swiftLastIndexForLogicalGroup = m_lastIndexForLogicalGroup;
    auto swiftLogicalGroupPropertyIDs = m_logicalGroupPropertyIDs;
    auto swiftSeenLogicalGroupPropertyCount = m_seenLogicalGroupPropertyCount;
    auto swiftLowestSeenLogicalGroupProperty = m_lowestSeenLogicalGroupProperty;
    auto swiftHighestSeenLogicalGroupProperty = m_highestSeenLogicalGroupProperty;
    auto swiftCustomProperties = std::exchange(m_customProperties, { });

    m_propertyIsPresent = { };
    m_logicalGroupPropertyIndices = { };
    m_lastIndexForLogicalGroup = 0;
    m_logicalGroupPropertyIDs = { };
    m_seenLogicalGroupPropertyCount = 0;
    m_lowestSeenLogicalGroupProperty = lastLogicalGroupProperty;
    m_highestSeenLogicalGroupProperty = firstLogicalGroupProperty;
    buildCascadeCxx();

    // Values are only meaningful where the cascade set them: origins[i] only where cssValue[i] is set.
    auto sameProperty = [](const Property& a, const Property& b) {
        if (a.id != b.id || a.origin != b.origin || a.styleScopeOrdinal != b.styleScopeOrdinal || a.cascadeLayerPriority != b.cascadeLayerPriority || a.fromStyleAttribute != b.fromStyleAttribute)
            return false;
        for (size_t i = 0; i < a.cssValue.size(); ++i) {
            if (a.cssValue[i] != b.cssValue[i] || (a.cssValue[i] && a.origins[i] != b.origins[i]))
                return false;
        }
        return true;
    };
    RELEASE_ASSERT(m_propertyIsPresent == swiftPropertyIsPresent);
    RELEASE_ASSERT(m_logicalGroupPropertyIndices == swiftLogicalGroupPropertyIndices);
    RELEASE_ASSERT(m_lastIndexForLogicalGroup == swiftLastIndexForLogicalGroup);
    RELEASE_ASSERT(m_logicalGroupPropertyIDs == swiftLogicalGroupPropertyIDs);
    RELEASE_ASSERT(m_seenLogicalGroupPropertyCount == swiftSeenLogicalGroupPropertyCount);
    RELEASE_ASSERT(m_lowestSeenLogicalGroupProperty == swiftLowestSeenLogicalGroupProperty);
    RELEASE_ASSERT(m_highestSeenLogicalGroupProperty == swiftHighestSeenLogicalGroupProperty);
    for (size_t id = 0; id < m_properties.size(); ++id) {
        bool isSet = id < std::to_underlying(firstLogicalGroupProperty) ? m_propertyIsPresent.get(id) && id != std::to_underlying(CSSPropertyID::Custom) : m_logicalGroupPropertyIndices[id - std::to_underlying(firstLogicalGroupProperty)];
        if (isSet)
            RELEASE_ASSERT(sameProperty(m_properties[id], swiftProperties[id]));
    }
    auto sameCustomProperties = [&] {
        if (m_customProperties.size() != swiftCustomProperties.size())
            return false;
        auto swiftCustomProperty = swiftCustomProperties.begin();
        for (auto& customProperty : m_customProperties) {
            if (customProperty.key != swiftCustomProperty->key || !sameProperty(customProperty.value, swiftCustomProperty->value))
                return false;
            ++swiftCustomProperty;
        }
        return true;
    };
    RELEASE_ASSERT(sameCustomProperties());
}
#endif // ENABLE(SWIFT_KERNELS_VALIDATION)

#endif // USE(SWIFT_KERNELS)

void PropertyCascade::buildCascadeCxx()
{
    std::array<bool, 3> originsWithImportant = { };

    for (auto origin : { Origin::UserAgent, Origin::User, Origin::Author }) {
        if (origin > m_maximumOrigin)
            break;
        bool hasImportant = addNormalMatches(origin);
        if (hasImportant)
            originsWithImportant[std::to_underlying(origin)] = true;
    }

    if (m_positionTryFallbackProperties)
        addPositionTryFallbackProperties();

    for (auto origin : { Origin::Author, Origin::User, Origin::UserAgent }) {
        if (!originsWithImportant[std::to_underlying(origin)])
            continue;
        addImportantMatches(origin);
    }

    sortLogicalGroupPropertyIDs();

    m_delayedRollbackProperties.clear();
}

void PropertyCascade::setPropertyInternal(Property& property, CSSPropertyID id, CSSValue& cssValue, const MatchedProperties& matchedProperties, Origin origin)
{
    ASSERT(matchedProperties.linkMatchType <= SelectorChecker::MatchAll);
    property.id = id;
    property.origin = origin;
    property.styleScopeOrdinal = matchedProperties.styleScopeOrdinal;
    property.cascadeLayerPriority = matchedProperties.cascadeLayerPriority;
    property.fromStyleAttribute = matchedProperties.fromStyleAttribute;

    if (matchedProperties.linkMatchType == SelectorChecker::MatchAll) {
        property.origins[SelectorChecker::MatchDefault] = origin;
        property.cssValue[SelectorChecker::MatchDefault] = &cssValue;

        property.origins[SelectorChecker::MatchLink] = origin;
        property.cssValue[SelectorChecker::MatchLink] = &cssValue;

        property.origins[SelectorChecker::MatchVisited] = origin;
        property.cssValue[SelectorChecker::MatchVisited] = &cssValue;
    } else {
        property.origins[matchedProperties.linkMatchType] = origin;
        property.cssValue[matchedProperties.linkMatchType] = &cssValue;
    }
}

void PropertyCascade::set(CSSPropertyID id, CSSValue& cssValue, const MatchedProperties& matchedProperties, Origin origin)
{
    if (id >= firstLogicalGroupProperty) {
        setLogicalGroupProperty(id, cssValue, matchedProperties, origin);
        return;
    }

    ASSERT(std::to_underlying(id) < m_propertyIsPresent.size());
    if (id == CSSPropertyID::Custom) {
        m_propertyIsPresent.set(std::to_underlying(id));
        const auto& customValue = downcast<CSSCustomPropertyValue>(cssValue);
        auto result = m_customProperties.ensure(customValue.name(), [&]() {
            Property property;
            property.cssValue = { };
            setPropertyInternal(property, id, cssValue, matchedProperties, origin);
            return property;
        });
        if (!result.isNewEntry)
            setPropertyInternal(result.iterator->value, id, cssValue, matchedProperties, origin);
        return;
    }

    auto& property = m_properties[std::to_underlying(id)];
    if (!m_propertyIsPresent.testAndSet(std::to_underlying(id)))
        property.cssValue = { };
    setPropertyInternal(property, id, cssValue, matchedProperties, origin);
}

void PropertyCascade::setLogicalGroupProperty(CSSPropertyID id, CSSValue& cssValue, const MatchedProperties& matchedProperties, Origin origin)
{
    ASSERT(id >= firstLogicalGroupProperty);
    ASSERT(id <= lastLogicalGroupProperty);

    auto& property = m_properties[std::to_underlying(id)];
    if (!hasLogicalGroupProperty(id)) {
        property.cssValue = { };
        m_lowestSeenLogicalGroupProperty = std::min(m_lowestSeenLogicalGroupProperty, id);
        m_highestSeenLogicalGroupProperty = std::max(m_highestSeenLogicalGroupProperty, id);
    }
    setLogicalGroupPropertyIndex(id, ++m_lastIndexForLogicalGroup);
    setPropertyInternal(property, id, cssValue, matchedProperties, origin);
}

void PropertyCascade::setDelayingForRuleRollback(CSSPropertyID propertyID, CSSValue& cssValue, const MatchedProperties& matchedProperties, Origin origin)
{
    ASSERT(m_ruleRollbackDepth);

    auto key = [&] -> std::pair<unsigned, AtomString>  {
        if (propertyID == CSSPropertyID::Custom)
            return { std::to_underlying(propertyID), downcast<CSSCustomPropertyValue>(cssValue).name() };
        return { std::to_underlying(propertyID), emptyAtom() };
    }();
    auto& delayedValues = m_delayedRollbackProperties.ensure(key, [&] {
        return Deque<DelayedRollbackProperty>();
    }).iterator->value;

    delayedValues.prepend(DelayedRollbackProperty { cssValue, matchedProperties, origin });

    // Ignore the last m_ruleRollbackDepth values for each property.
    if (delayedValues.size() <= m_ruleRollbackDepth)
        return;

    auto last = delayedValues.takeLast();
    SUPPRESS_UNCOUNTED_ARG set(propertyID, last.value, last.properties, last.origin);
}

bool PropertyCascade::hasProperty(CSSPropertyID propertyID, const CSSValue& value)
{
    if (propertyID == CSSPropertyID::Custom)
        return hasCustomProperty(downcast<CSSCustomPropertyValue>(value).name());
    return propertyID < firstLogicalGroupProperty ? hasNormalProperty(propertyID) : hasLogicalGroupProperty(propertyID);
}

bool PropertyCascade::mayOverrideExistingProperty(CSSPropertyID propertyID, const CSSValue& value)
{
    if (propertyID == CSSPropertyID::Custom)
        return hasCustomProperty(downcast<CSSCustomPropertyValue>(value).name());
    if (propertyID < firstLogicalGroupProperty)
        return hasNormalProperty(propertyID);

    // Apply all logical group properties if we have applied any. They may override the ones we already applied.
    // FIXME: This could check if any existing properties are actually in the same group.
    return !!m_lastIndexForLogicalGroup;
}

const PropertyCascade::Property& PropertyCascade::functionResultProperty() const
{
    ASSERT(hasNormalProperty(CSSPropertyID::Result));
    return normalProperty(CSSPropertyID::Result);
}

const PropertyCascade::Property* PropertyCascade::lastPropertyResolvingLogicalPropertyPair(CSSPropertyID propertyID, WritingMode writingMode) const
{
    ASSERT(CSSProperty::isInLogicalPropertyGroup(propertyID));

    auto pairID = [&] {
        if (CSSProperty::isDirectionAwareProperty(propertyID))
            return CSSProperty::resolveDirectionAwareProperty(propertyID, writingMode);
        return CSSProperty::unresolvePhysicalProperty(propertyID, writingMode);
    }();
    ASSERT(pairID != CSSPropertyID::Invalid);

    auto indexForPropertyID = logicalGroupPropertyIndex(propertyID);
    auto indexForPairID = logicalGroupPropertyIndex(pairID);
    if (indexForPropertyID > indexForPairID)
        return &logicalGroupProperty(propertyID);
    if (indexForPropertyID < indexForPairID)
        return &logicalGroupProperty(pairID);
    ASSERT(!hasLogicalGroupProperty(propertyID));
    ASSERT(!hasLogicalGroupProperty(pairID));
    return nullptr;
}

// inherit() reads a value from the parent that may be a custom property which does not itself
// inherit, so like the inherit keyword it has to be re-applied when the parent's non-inherited
// style changes. https://drafts.csswg.org/css-values-5/#funcdef-inherit
static bool explicitlyInheritsFromParent(const CSSValue& value)
{
    if (isValueID(value, CSSValueID::Inherit))
        return true;
    if (RefPtr shorthandValue = dynamicDowncast<CSSShorthandSubstitutionValue>(value))
        return shorthandValue->shorthandValue().containsInheritFunction();
    if (RefPtr substitutionValue = dynamicDowncast<CSSSubstitutionValue>(value))
        return substitutionValue->containsInheritFunction();
    if (RefPtr customPropertyValue = dynamicDowncast<CSSCustomPropertyValue>(value)) {
        return WTF::switchOn(customPropertyValue->value(),
            [](const Ref<CSSSubstitutionValue>& substitution) { return substitution->containsInheritFunction(); },
            [](const Ref<CSSVariableData>&) { return false; },
            [](CSSWideKeyword keyword) {
                return keyword == CSSWideKeyword::Inherit;
            });
    }
    return false;
}

bool PropertyCascade::addMatch(const MatchedProperties& matchedProperties, Origin origin, IsImportant important)
{
    auto includePropertiesForRollback = [&] {
        if (m_rollbackScope && matchedProperties.styleScopeOrdinal > *m_rollbackScope)
            return true;
        if (origin < m_maximumOrigin)
            return true;
        if (matchedProperties.fromStyleAttribute == FromStyleAttribute::Yes)
            return false;
        return matchedProperties.cascadeLayerPriority <= *m_maximumCascadeLayerPriorityForRollback;
    };
    if (m_maximumCascadeLayerPriorityForRollback && !includePropertiesForRollback())
        return false;

    if (matchedProperties.isStartingStyle == IsStartingStyle::Yes && !m_includedProperties.types.contains(PropertyType::StartingStyle))
        return false;

    auto propertyAllowlist = matchedProperties.allowlistType;
    bool hasImportantProperties = false;

    for (auto current : matchedProperties.properties.get()) {
        if (current.isImportant())
            hasImportantProperties = true;
        if ((important == IsImportant::No && current.isImportant()) || (important == IsImportant::Yes && !current.isImportant()))
            continue;

        auto propertyID = cascadeAliasProperty(current.id());

        auto shouldIncludeProperty = [&] {
#if ENABLE(VIDEO)
            if (propertyAllowlist == PropertyAllowlist::Cue && !isValidCueStyleProperty(propertyID))
                return false;
            if (propertyAllowlist == PropertyAllowlist::CueSelector && !isValidCueSelectorStyleProperty(propertyID))
                return false;
            if (propertyAllowlist == PropertyAllowlist::CueBackground && !isValidCueBackgroundStyleProperty(propertyID))
                return false;
#endif
            if (propertyAllowlist == PropertyAllowlist::Marker && !isValidMarkerStyleProperty(propertyID))
                return false;
            if (propertyAllowlist == PropertyAllowlist::Highlight && !isValidHighlightStyleProperty(propertyID))
                return false;

            if (propertyAllowlist == PropertyAllowlist::FirstLetter && !isValidFirstLetterStyleProperty(propertyID))
                return false;

            if (propertyAllowlist == PropertyAllowlist::FirstLine && !isValidFirstLineStyleProperty(propertyID))
                return false;

            if (m_includedProperties.types.containsAll(normalPropertyTypes()))
                return true;

            if (m_includedProperties.ids.contains(propertyID))
                return true;

            if (matchedProperties.isCacheable == IsCacheable::Partially && m_includedProperties.types.contains(PropertyType::NonCacheable))
                return true;

            // If we have applied this property for some reason already we must apply anything that overrides it.
            if (mayOverrideExistingProperty(propertyID, *current.value()))
                return true;

            if (m_includedProperties.types.contains(PropertyType::AfterAnimation)) {
                if (shouldApplyAfterAnimation(current)) {
                    m_animationLayer->overriddenProperties.add(propertyID);
                    return true;
                }
                return false;
            }

            bool currentIsInherited = CSSProperty::isInheritedProperty(current.id());
            if (m_includedProperties.types.contains(PropertyType::Inherited) && currentIsInherited)
                return true;
            if (m_includedProperties.types.contains(PropertyType::ExplicitlyInherited)) {
                RefPtr value = current.value();
                if (explicitlyInheritsFromParent(*value))
                    return true;
            }
            if (m_includedProperties.types.contains(PropertyType::NonInherited) && !currentIsInherited)
                return true;

            return false;
        }();

        if (!shouldIncludeProperty)
            continue;

        if (m_ruleRollbackDepth) {
            setDelayingForRuleRollback(propertyID, protect(*current.value()), matchedProperties, origin);
            continue;
        }

        SUPPRESS_UNCOUNTED_ARG set(propertyID, *current.value(), matchedProperties, origin);
    }

    return hasImportantProperties;
}

bool PropertyCascade::shouldApplyAfterAnimation(const StyleProperties::PropertyReference& property)
{
    ASSERT(m_animationLayer);

    auto id = property.id();
    RefPtr customProperty = dynamicDowncast<CSSCustomPropertyValue>(*property.value());

    auto animationSource = m_animationLayer->properties.getOptional(customProperty ? AnimatableCSSProperty { customProperty->name() } : id);
    if (animationSource) {
        // "Important declarations from all origins take precedence over animations."
        // https://drafts.csswg.org/css-cascade-5/#importance
        // But transitions take precedence over !important, so only allow the
        // override for properties not animated by a transition.
        return !animationSource->contains(AnimationSource::CSSTransition) && property.isImportant();
    }

    // If we are animating custom properties they may affect other properties so we need to re-resolve them.
    if (m_animationLayer->hasCustomProperties) {
        // We could check if the we are actually animating the referenced variable. Indirect cases would need to be taken into account.
        if (customProperty && customProperty->isVariableReference())
            return true;
        if (property.value()->hasSubstitutionFunctions())
            return true;
    }

    // Check for 'em' units and similar property dependencies.
    if (m_animationLayer->hasFontSize || m_animationLayer->hasLineHeight) {
        auto dependencies = protect(property.value())->computedStyleDependencies();
        if (m_animationLayer->hasFontSize && dependencies.properties.contains(CSSPropertyID::FontSize))
            return true;
        if (m_animationLayer->hasLineHeight && dependencies.properties.contains(CSSPropertyID::LineHeight))
            return true;
    }

    return false;
}

void PropertyCascade::addPositionTryFallbackProperties()
{
    ASSERT(m_positionTryFallbackProperties);

    // "All of the properties in a @position-try are applied to the box as part of the Position Fallback Origin,
    // a new cascade origin that lies between the Author Origin and the Animation Origin"
    // https://drafts.csswg.org/css-anchor-position-1/#fallback-rule
    if (m_maximumOrigin < Origin::PositionFallback)
        return;

    addMatch(*m_positionTryFallbackProperties, Origin::PositionFallback, IsImportant::No);
}

NODELETE static auto& declarationsForOrigin(const MatchResult& matchResult, PropertyCascade::Origin origin)
{
    switch (origin) {
    case PropertyCascade::Origin::UserAgent: return matchResult.userAgentDeclarations;
    case PropertyCascade::Origin::User: return matchResult.userDeclarations;
    case PropertyCascade::Origin::Author: return matchResult.authorDeclarations;
    case PropertyCascade::Origin::PositionFallback: break;
    }
    ASSERT_NOT_REACHED();
    return matchResult.authorDeclarations;
}

bool PropertyCascade::addNormalMatches(Origin origin)
{
    bool hasImportant = false;
    for (auto& matchedDeclarations : declarationsForOrigin(m_matchResult, origin))
        hasImportant |= addMatch(matchedDeclarations, origin, IsImportant::No);

    return hasImportant;
}

NODELETE static bool hasImportantProperties(const StyleProperties& properties)
{
    for (auto property : properties) {
        if (property.isImportant())
            return true;
    }
    return false;
}

void PropertyCascade::addImportantMatches(Origin origin)
{
    struct ImportantMatch {
        unsigned index;
        ScopeOrdinal ordinal;
        CascadeLayerPriority layerPriority;
        FromStyleAttribute fromStyleAttribute;
    };
    Vector<ImportantMatch> importantMatches;
    bool hasMatchesFromOtherScopesOrLayers = false;

    auto& matchedDeclarations = declarationsForOrigin(m_matchResult, origin);

    for (unsigned i = 0; i < matchedDeclarations.size(); ++i) {
        const MatchedProperties& matchedProperties = matchedDeclarations[i];

        if (!hasImportantProperties(matchedProperties.properties))
            continue;

        importantMatches.append({ i, matchedProperties.styleScopeOrdinal, matchedProperties.cascadeLayerPriority, matchedProperties.fromStyleAttribute });

        if (matchedProperties.styleScopeOrdinal != ScopeOrdinal::Element || matchedProperties.cascadeLayerPriority != RuleSet::cascadeLayerPriorityForUnlayered)
            hasMatchesFromOtherScopesOrLayers = true;
    }

    if (importantMatches.isEmpty())
        return;

    if (hasMatchesFromOtherScopesOrLayers) {
        // Match results are sorted in reverse tree context order so this is not needed for normal properties.
        std::ranges::stable_sort(importantMatches, [](auto& a, auto& b) {
            // For !important properties a later shadow tree wins.
            if (a.ordinal != b.ordinal)
                return a.ordinal < b.ordinal;
            // Lower priority layer wins, except if style attribute is involved.
            if (a.fromStyleAttribute != b.fromStyleAttribute)
                return a.fromStyleAttribute == FromStyleAttribute::No;
            return a.layerPriority > b.layerPriority;
        });
    }

    for (auto& match : importantMatches)
        addMatch(matchedDeclarations[match.index], origin, IsImportant::Yes);
}

void PropertyCascade::sortLogicalGroupPropertyIDs()
{
    size_t endIndex = 0;
    for (uint16_t id = std::to_underlying(m_lowestSeenLogicalGroupProperty); id <= std::to_underlying(m_highestSeenLogicalGroupProperty); ++id) {
        auto propertyID = static_cast<CSSPropertyID>(id);
        if (hasLogicalGroupProperty(propertyID))
            m_logicalGroupPropertyIDs[endIndex++] = propertyID;
    }
    m_seenLogicalGroupPropertyCount = endIndex;
    auto logicalGroupPropertyIDs = std::span { m_logicalGroupPropertyIDs }.first(endIndex);
    std::ranges::sort(logicalGroupPropertyIDs, [&](auto id1, auto id2) {
        return logicalGroupPropertyIndex(id1) < logicalGroupPropertyIndex(id2);
    });
}

ValueOrReference<HashSet<AnimatableCSSProperty>> PropertyCascade::overriddenAnimatedProperties() const
{
    if (m_animationLayer)
        return m_animationLayer->overriddenProperties;
    return { };
}

}
}
