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

#include "ImmutableStyleProperties.h"
#include "MutableStyleProperties.h"
#include "PropertyCascadeSwiftBoundary.h"

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
static_assert(SelectorChecker::MatchAll == 3 && std::to_underlying(IsCacheable::Partially) == 1 && std::to_underlying(FromStyleAttribute::Yes) == 1);
static_assert(std::to_underlying(ScopeOrdinal::Element) == 0 && RuleSet::cascadeLayerPriorityForUnlayered == 0xFFFF);
static_assert(std::to_underlying(PropertyCascade::PropertyType::NonInherited) == 1 && std::to_underlying(PropertyCascade::PropertyType::Inherited) == 2);
static_assert(std::to_underlying(PropertyCascade::PropertyType::ExplicitlyInherited) == 4 && std::to_underlying(PropertyCascade::PropertyType::AfterAnimation) == 8);
static_assert(std::to_underlying(PropertyCascade::PropertyType::StartingStyle) == 16 && std::to_underlying(PropertyCascade::PropertyType::NonCacheable) == 32);
static_assert(!std::to_underlying(PropertyCascade::Origin::UserAgent) && std::to_underlying(PropertyCascade::Origin::User) == 1);
static_assert(std::to_underlying(PropertyCascade::Origin::Author) == 2 && std::to_underlying(PropertyCascade::Origin::PositionFallback) == 3);
static_assert(sizeof(MatchedProperties) == 16 && offsetof(MatchedProperties, linkMatchType) == 8 && offsetof(MatchedProperties, allowlistType) == 9);
static_assert(offsetof(MatchedProperties, styleScopeOrdinal) == 10 && offsetof(MatchedProperties, fromStyleAttribute) == 11);
static_assert(offsetof(MatchedProperties, cascadeLayerPriority) == 12 && offsetof(MatchedProperties, isStartingStyle) == 14 && offsetof(MatchedProperties, isCacheable) == 15);
static_assert(sizeof(PropertyAllowlist) == 1 && sizeof(ScopeOrdinal) == 1 && sizeof(FromStyleAttribute) == 1 && sizeof(CascadeLayerPriority) == 2 && sizeof(IsStartingStyle) == 1 && sizeof(IsCacheable) == 1);
static_assert(!std::to_underlying(PropertyAllowlist::None));
// The match storage the kernel reads: CSSProperty records (StylePropertyMetadata, then the Ref<CSSValue>) for
// MutableStyleProperties; for ImmutableStyleProperties, m_storage followed by StylePropertyMetadata[n], then
// PackedPtr<const CSSValue>[n], each pointer stored as its low storageSize bytes, little-endian. The parts that
// cannot be static_asserted from here (CSSProperty's private members, the order of ImmutableStyleProperties' arrays,
// StylePropertyMetadata's bit-fields) are checked against the public API on every cascade in debug builds
// (assertStorageLayoutForSwift()).
static_assert(std::endian::native == std::endian::little);
static_assert(!PackedPtr<const CSSValue>::alignmentShiftSize && sizeof(PackedPtr<const CSSValue>) == PackedPtr<const CSSValue>::storageSize);
static_assert(ImmutableStyleProperties::objectSize(1) - ImmutableStyleProperties::objectSize(0) == sizeof(StylePropertyMetadata) + sizeof(PackedPtr<const CSSValue>));

#if ASSERT_ENABLED
// Decodes every property of one match's storage as the kernel does (StylePropertyMetadata: m_propertyID in bits 0-9,
// m_important in bit 13) and compares it with the public API.
static void assertStorageLayoutForSwift(const StyleProperties& properties, std::span<const uint8_t> storage)
{
    unsigned count = properties.propertyCount();
    bool isMutable = properties.isMutable();
    for (unsigned i = 0; i < count; ++i) {
        auto property = properties.propertyAt(i);
        size_t metadataOffset = isMutable ? i * sizeof(CSSProperty) : i * sizeof(StylePropertyMetadata);
        size_t valueOffset = isMutable ? i * sizeof(CSSProperty) + sizeof(uintptr_t) : count * sizeof(StylePropertyMetadata) + i * sizeof(PackedPtr<const CSSValue>);
        size_t valueSize = isMutable ? sizeof(uintptr_t) : sizeof(PackedPtr<const CSSValue>);
        uint32_t metadata = 0;
        memcpySpan(asMutableByteSpan(metadata), storage.subspan(metadataOffset, sizeof(metadata)));
        uintptr_t value = 0;
        memcpySpan(asMutableByteSpan(value).first(valueSize), storage.subspan(valueOffset, valueSize));
        ASSERT((metadata & 0x3FF) == std::to_underlying(property.id()));
        ASSERT(!!(metadata & (1 << 13)) == property.isImportant());
        ASSERT(value == reinterpret_cast<uintptr_t>(property.value()));
    }
}
#endif

// The property allowlist check of a match (the start of the original addMatch's shouldIncludeProperty).
static bool isAllowedByPropertyAllowlist(PropertyAllowlist propertyAllowlist, CSSPropertyID propertyID)
{
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

    return true;
}

// For each CSSPropertyID id, with alias = cascadeAliasProperty(id): alias (bits 0-14) | CSSProperty::isInheritedProperty(id) << 15
// | isAllowedByPropertyAllowlist(k, alias) << (16 + k) for each PropertyAllowlist value k < 8 (the kernel admits every property
// of a match whose allowlistType is 8 or more, as the allowlist check does).
static std::span<const uint32_t> propertyTableForSwift()
{
    static const std::array<uint32_t, cssPropertyIDEnumValueCount> table = [] {
        std::array<uint32_t, cssPropertyIDEnumValueCount> table { };
        for (uint16_t id = 0; id < cssPropertyIDEnumValueCount; ++id) {
            auto propertyID = static_cast<CSSPropertyID>(id);
            auto alias = cascadeAliasProperty(propertyID);
            // The kernel and the glue's name gathering tell custom properties apart by the alias.
            ASSERT((alias == CSSPropertyID::Custom) == (propertyID == CSSPropertyID::Custom));
            uint32_t entry = std::to_underlying(alias) | (CSSProperty::isInheritedProperty(propertyID) ? 0x8000 : 0);
            for (uint8_t allowlist = 0; allowlist < 8; ++allowlist) {
                if (isAllowedByPropertyAllowlist(static_cast<PropertyAllowlist>(allowlist), alias))
                    entry |= 1u << (16 + allowlist);
            }
            table[id] = entry;
        }
        return table;
    }();
    return table;
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

// Builds the cascade with the Swift kernel: buildCascade's normal and !important passes over the UserAgent, User and
// Author matches up to m_maximumOrigin, the position-try fallback match, the rollback limits (m_rollbackScope,
// m_maximumCascadeLayerPriorityForRollback), revert-rule delaying (m_ruleRollbackDepth) and sortLogicalGroupPropertyIDs.
// The glue only gathers inputs and stores results: the match storage, the per-property facts the kernel can't read
// from CSSValues (gathered below for the cascades that can need them), and, from the kernel, the custom-property sets
// (into m_customProperties) and the overridden animated properties.
void PropertyCascade::buildCascade()
{
    ASSERT(m_propertyIsPresent.isEmpty() && !m_lastIndexForLogicalGroup && m_customProperties.isEmpty());
    auto types = m_includedProperties.types;

    // The kernel's matches, in order: UserAgent, User, Author, then the position-try fallback (Origin::PositionFallback).
    auto positionTryFallback = m_positionTryFallbackProperties ? std::span<const MatchedProperties> { singleElementSpan(*m_positionTryFallbackProperties) } : std::span<const MatchedProperties> { };
    std::array<std::span<const MatchedProperties>, 4> groups { m_matchResult.userAgentDeclarations.span(), m_matchResult.userDeclarations.span(), m_matchResult.authorDeclarations.span(), positionTryFallback };
    size_t matchCount = groups[0].size() + groups[1].size() + groups[2].size() + groups[3].size();

    Vector<uint64_t, 3 * 32> matchWords(3 * matchCount);
    auto matchWordSpan = matchWords.mutableSpan();
    size_t wordIndex = 0;
    size_t propertyTotal = 0;
    for (auto group : groups) {
        for (auto& matchedProperties : group) {
            auto& properties = matchedProperties.properties.get();
            auto words = matchWordSpan.subspan(wordIndex).first<3>();
            size_t count;
            if (properties.isMutable()) {
                auto storage = uncheckedDowncast<MutableStyleProperties>(properties).propertySpan();
                count = storage.size();
                words[0] = reinterpret_cast<uintptr_t>(storage.data());
                words[2] = count | 1ull << 63;
#if ASSERT_ENABLED
                assertStorageLayoutForSwift(properties, asByteSpan(storage));
#endif
            } else {
                auto& immutableProperties = uncheckedDowncast<ImmutableStyleProperties>(properties);
                count = immutableProperties.propertyCount();
                words[0] = reinterpret_cast<uintptr_t>(&immutableProperties.m_storage);
                words[2] = count;
#if ASSERT_ENABLED
                assertStorageLayoutForSwift(properties, unsafeMakeSpan(reinterpret_cast<const uint8_t*>(&immutableProperties.m_storage), count * (sizeof(StylePropertyMetadata) + sizeof(PackedPtr<const CSSValue>))));
#endif
            }
            // linkMatchType through isCacheable, in their byte order (static_asserted above).
            memcpySpan(asMutableByteSpan(words[1]), asByteSpan(matchedProperties).subspan(offsetof(MatchedProperties, linkMatchType), sizeof(uint64_t)));
            propertyTotal += count;
            wordIndex += 3;
        }
    }

    // Per property, in match order: bit 32 = shouldApplyAfterAnimation(), bit 33 = explicitlyInheritsFromParent(), and for
    // a custom property bits 0-31 = an id per distinct name. Gathered only where the inclusion rules can read them (the
    // kernel traps on a missing input): AfterAnimation and ExplicitlyInherited without both normal types (with Inherited
    // included, the ExplicitlyInherited check is never reached for an inherited property); the names with rule rollback
    // (they key the delayed properties) and when whether a custom property is included can depend on hasCustomProperty()
    // (some type is included, but AfterAnimation or not the custom properties' own one).
    auto table = propertyTableForSwift();
    bool includesAllNormal = types.containsAll(normalPropertyTypes());
    bool needsAfterAnimation = !includesAllNormal && types.contains(PropertyType::AfterAnimation);
    bool needsExplicitlyInherited = !includesAllNormal && types.contains(PropertyType::ExplicitlyInherited);
    bool includesInherited = types.contains(PropertyType::Inherited);
    bool needsCustomNames = m_ruleRollbackDepth || (types && !includesAllNormal && (types.contains(PropertyType::AfterAnimation)
        || !types.contains(table[std::to_underlying(CSSPropertyID::Custom)] & 0x8000 ? PropertyType::Inherited : PropertyType::NonInherited)));
    Vector<uint64_t, 64> propertyInputs;
    unsigned customNameCount = 0;
    if (needsAfterAnimation || needsExplicitlyInherited || needsCustomNames) {
        propertyInputs.grow(propertyTotal);
        HashMap<AtomString, unsigned> customNameIDs;
        size_t index = 0;
        for (auto group : groups) {
            for (auto& matchedProperties : group) {
                for (auto property : matchedProperties.properties.get()) {
                    uint64_t input = 0;
                    if (needsAfterAnimation && shouldApplyAfterAnimation(property))
                        input |= 1ull << 32;
                    if (needsExplicitlyInherited && !(includesInherited && (table[std::to_underlying(property.id())] & 0x8000))) {
                        RefPtr value = property.value();
                        if (explicitlyInheritsFromParent(*value))
                            input |= 1ull << 33;
                    }
                    if (needsCustomNames && (table[std::to_underlying(property.id())] & 0x7FFF) == std::to_underlying(CSSPropertyID::Custom)) {
                        auto& name = downcast<CSSCustomPropertyValue>(*property.value()).name();
                        input |= customNameIDs.ensure(name, [&] { return customNameCount++; }).iterator->value;
                    }
                    propertyInputs[index++] = input;
                }
            }
        }
    }

    // The kernel's work regions (PropertyCascadeSwiftBoundary.h): sort scratch, custom-property sets, overridden animated
    // properties, per-match property offsets and seen custom names, revert-rule keys and delayed properties.
    size_t scratchCount = std::max<size_t>({ logicalGroupPropertyCount, groups[0].size(), groups[1].size(), groups[2].size() });
    size_t workCount = scratchCount + propertyTotal;
    if (types.contains(PropertyType::AfterAnimation))
        workCount += propertyTotal;
    if (!propertyInputs.isEmpty())
        workCount += matchCount + (customNameCount + 63) / 64;
    if (m_ruleRollbackDepth)
        workCount += 2 * (table.size() + customNameCount) + 2 * propertyTotal;
    Vector<uint64_t, 256> work(workCount);

    std::array<uint32_t, 19> state { static_cast<uint32_t>(groups[0].size()), static_cast<uint32_t>(groups[1].size()), static_cast<uint32_t>(groups[2].size()),
        static_cast<uint32_t>(groups[3].size()), types.toRaw(), std::to_underlying(firstLogicalGroupProperty), sizeof(PackedPtr<const CSSValue>),
        static_cast<uint32_t>(scratchCount), std::to_underlying(m_maximumOrigin),
        m_rollbackScope ? 0x100u | static_cast<uint8_t>(std::to_underlying(*m_rollbackScope)) : 0u,
        m_maximumCascadeLayerPriorityForRollback ? 0x10000u | *m_maximumCascadeLayerPriorityForRollback : 0u,
        m_ruleRollbackDepth, static_cast<uint32_t>(propertyTotal), customNameCount,
        m_lastIndexForLogicalGroup, m_seenLogicalGroupPropertyCount, std::to_underlying(m_lowestSeenLogicalGroupProperty), std::to_underlying(m_highestSeenLogicalGroupProperty), 0 };
    auto includedIDs = spanReinterpretCast<const uint16_t>(m_includedProperties.ids.span());
    auto properties = asMutableByteSpan(std::span { m_properties });
    auto propertyIsPresent = m_propertyIsPresent.storage();
    auto logicalGroupIDs = spanReinterpretCast<uint16_t>(std::span { m_logicalGroupPropertyIDs });

    size_t customCount = WebCoreSwift_PropertyCascade_build(matchWords.span().data(), matchWords.size(), state.data(), state.size(),
        includedIDs.data(), includedIDs.size(), table.data(), table.size(), propertyInputs.span().data(), propertyInputs.size(),
        properties.data(), properties.size(), propertyIsPresent.data(), propertyIsPresent.size(),
        m_logicalGroupPropertyIndices.data(), logicalGroupIDs.data(), logicalGroupPropertyCount, work.mutableSpan().data(), work.size());

    m_lastIndexForLogicalGroup = state[14];
    m_seenLogicalGroupPropertyCount = state[15];
    m_lowestSeenLogicalGroupProperty = static_cast<CSSPropertyID>(state[16]);
    m_highestSeenLogicalGroupProperty = static_cast<CSSPropertyID>(state[17]);

    // The custom property sets, in cascade order (work[S + i] = match << 32 | property index).
    for (auto entry : work.span().subspan(scratchCount, customCount)) {
        size_t matchIndex = entry >> 32;
        size_t groupIndex = 0;
        while (matchIndex >= groups[groupIndex].size()) {
            matchIndex -= groups[groupIndex].size();
            ++groupIndex;
        }
        auto& matchedProperties = groups[groupIndex][matchIndex];
        auto current = matchedProperties.properties->propertyAt(static_cast<uint32_t>(entry));
        auto& property = m_customProperties.ensure(downcast<CSSCustomPropertyValue>(*current.value()).name(), [] {
            Property property;
            property.cssValue = { };
            return property;
        }).iterator->value;
        SUPPRESS_UNCOUNTED_ARG setPropertyInternal(property, CSSPropertyID::Custom, *current.value(), matchedProperties, static_cast<Origin>(groupIndex));
    }

    // The properties AfterAnimation included because they override an animation, in order (work[S + T + i]).
    if (unsigned overriddenCount = state[18]) {
        for (auto id : work.span().subspan(scratchCount + propertyTotal, overriddenCount))
            m_animationLayer->overriddenProperties.add(static_cast<CSSPropertyID>(id));
    }
}

ValueOrReference<HashSet<AnimatableCSSProperty>> PropertyCascade::overriddenAnimatedProperties() const
{
    if (m_animationLayer)
        return m_animationLayer->overriddenProperties;
    return { };
}

}
}
