/*
 * Copyright (C) 2025 Apple Inc. All rights reserved.
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
#include "MatchResultCache.h"

#include "MatchResult.h"
#include "MatchResultCacheDiffSwiftBoundary.h"
#include "MutableStyleProperties.h"
#include "ResolvedStyle.h"
#include "StyleComputedStyle+SettersInlines.h"
#include "StyleProperties.h"
#include "StyledElement.h"
#include <bit>
#include <wtf/BitSet.h>

namespace WebCore {
namespace Style {

// Cache entries contain a MatchResult object that references element's mutable inline style.
// As a result the cache entry mutates when element's inline style mutates.
// We save the original properties and values so we can check which properties have changed.
// If a property changes we null the value and assume it will change in the future too.
//
// It would be nicer if the cache entries were immutable but doing that in a sufficiently
// performant way is tricky.

struct OriginalInlineProperty {
    CSSPropertyID propertyID;
    bool wasImportant { false };
    RefPtr<const CSSValue> valueIfUnchanged;
};

struct MatchResultCache::Entry : CanMakeCheckedPtr<MatchResultCache::Entry> {
    UnadjustedStyle unadjustedStyle;
    Ref<const MutableStyleProperties> inlineStyle;
    Vector<OriginalInlineProperty> originalInlineProperties;

    Entry(UnadjustedStyle&& unadjustedStyle, const MutableStyleProperties& inlineStyle)
        : unadjustedStyle(WTF::move(unadjustedStyle))
        , inlineStyle(inlineStyle)
    {
        originalInlineProperties.reserveInitialCapacity(inlineStyle.size());
        for (auto property : inlineStyle) {
            originalInlineProperties.append({
                .propertyID = property.id(),
                .wasImportant = property.isImportant(),
                .valueIfUnchanged = property.value()
            });
        }
    }

    WTF_STRUCT_OVERRIDE_DELETE_FOR_CHECKED_PTR(Entry);
    WTF_DEPRECATED_MAKE_STRUCT_FAST_ALLOCATED(MatchResultCache);
};

MatchResultCache::MatchResultCache() = default;
MatchResultCache::~MatchResultCache() = default;

inline UnadjustedStyle copy(const UnadjustedStyle& other)
{
    // Relations hold raw Element pointers valid only for a single style resolution. They should not be cached.
    return {
        .style = ComputedStyle::clonePtr(*other.style),
        .relations = { },
        .matchResult = other.matchResult
    };
}

// The layouts MatchResultCacheDiffSwiftBoundary.h documents for the Swift kernel. CSSProperty's members are private;
// its layout (StylePropertyMetadata, then the Ref<CSSValue>) is checked against the public API in debug builds
// (assertLayoutForSwift()).
static_assert(std::endian::native == std::endian::little);
static_assert(sizeof(OriginalInlineProperty) == 16 && alignof(OriginalInlineProperty) == 8 && sizeof(CSSPropertyID) == 2 && sizeof(bool) == 1);
static_assert(offsetof(OriginalInlineProperty, propertyID) == 0 && offsetof(OriginalInlineProperty, wasImportant) == 2);
static_assert(offsetof(OriginalInlineProperty, valueIfUnchanged) == 8 && sizeof(RefPtr<const CSSValue>) == sizeof(uintptr_t));
static_assert(sizeof(CSSProperty) == 16 && alignof(CSSProperty) == 8 && sizeof(StylePropertyMetadata) == 4 && sizeof(uintptr_t) == 8);
static_assert(std::to_underlying(firstLowPriorityProperty) <= 0xFFFF && std::to_underlying(CSSPropertyID::LineHeight) <= 0xFFFF);
static_assert(std::to_underlying(CSSPropertyID::Custom) <= 0xFFFF && cssPropertyIDEnumValueCount <= 0x400);

#if ASSERT_ENABLED
// Decodes the snapshot and the current properties as the kernel does and compares them with the public API.
static void assertLayoutForSwift(std::span<const OriginalInlineProperty> originalProperties, const MutableStyleProperties& inlineStyle)
{
    for (auto& property : originalProperties) {
        auto words = spanReinterpretCast<const uint64_t>(asByteSpan(property));
        ASSERT((words[0] & 0xFFFF) == std::to_underlying(property.propertyID));
        // The kernel compares wasImportant as a byte, as `bool ==` compiles.
        ASSERT(((words[0] >> 16) & 0xFF) == static_cast<uint64_t>(property.wasImportant));
        ASSERT(words[1] == reinterpret_cast<uintptr_t>(property.valueIfUnchanged.get()));
    }
    auto currentProperties = inlineStyle.propertySpan();
    for (size_t index = 0; index < currentProperties.size(); ++index) {
        auto words = spanReinterpretCast<const uint64_t>(asByteSpan(currentProperties[index]));
        auto property = inlineStyle.propertyAt(index);
        ASSERT((words[0] & 0x3FF) == std::to_underlying(property.id()));
        ASSERT(!!(words[0] & (1 << 13)) == property.isImportant());
        ASSERT(words[1] == reinterpret_cast<uintptr_t>(property.value()));
    }
}
#endif

// cascadeAliasProperty() for every CSSPropertyID, for the Swift kernel.
static std::span<const uint16_t> cascadeAliasTableForSwift()
{
    static const std::array<uint16_t, cssPropertyIDEnumValueCount> table = [] {
        std::array<uint16_t, cssPropertyIDEnumValueCount> table { };
        for (uint16_t id = 0; id < cssPropertyIDEnumValueCount; ++id)
            table[id] = std::to_underlying(cascadeAliasProperty(static_cast<CSSPropertyID>(id)));
        return table;
    }();
    return table;
}

// isUsableAfterInlineStyleChange() and computeAndUpdateChangedProperties() in one Swift pass
// (MatchResultCacheDiffSwiftBoundary.h). Returns false if the entry can't be used with the element's current inline
// style; otherwise sets `changedProperties`. Only the RefPtr resets (the update of the entry's snapshot) stay in C++.
ALWAYS_INLINE bool MatchResultCache::changedPropertiesAfterInlineStyleChange(MatchResultCache::Entry& entry, const StyleProperties* currentInlineStyle, PropertyCascade::IncludedProperties& changedProperties)
{
    auto& originalProperties = entry.originalInlineProperties;
    auto currentProperties = entry.inlineStyle->propertySpan();
#if ASSERT_ENABLED
    assertLayoutForSwift(originalProperties.span(), entry.inlineStyle);
#endif

    // The kernel's output: the changed ids, then the indices of the snapshot values to null. On the stack unless the
    // inline style has more than 32 properties.
    size_t count = originalProperties.size();
    std::array<uint32_t, 64> inlineOut;
    Vector<uint32_t> outOfLineOut;
    std::span<uint32_t> out = inlineOut;
    if (2 * count > inlineOut.size()) [[unlikely]] {
        outOfLineOut.grow(2 * count);
        out = outOfLineOut.mutableSpan();
    }
    // The kernel compares the two inline style addresses as integers, both as StyleProperties pointers.
    std::array<uint64_t, 4> state { reinterpret_cast<uintptr_t>(static_cast<const StyleProperties*>(entry.inlineStyle.ptr())), reinterpret_cast<uintptr_t>(currentInlineStyle),
        std::to_underlying(firstLowPriorityProperty) | static_cast<uint64_t>(std::to_underlying(CSSPropertyID::LineHeight)) << 16
            | static_cast<uint64_t>(std::to_underlying(CSSPropertyID::Custom)) << 32, 0 };
    auto aliases = cascadeAliasTableForSwift();
    auto result = WebCoreSwift_MatchResultCacheDiff_changedProperties(state.data(), state.size(),
        reinterpret_cast<const uint64_t*>(originalProperties.span().data()), 2 * count,
        reinterpret_cast<const uint64_t*>(currentProperties.data()), 2 * currentProperties.size(), aliases.data(), aliases.size(), out.data(), out.size());

    // Assume that if a value or priority changes once then it will change more. Don't track changes anymore.
    if (state[3]) [[unlikely]] {
        for (auto index : out.subspan(count, state[3]))
            originalProperties[index].valueIfUnchanged = nullptr;
    }

    switch (result) {
    case WebCoreSwiftMatchResultCacheDiffUnusable:
        return false;
    case WebCoreSwiftMatchResultCacheDiffAllNormalProperties:
        // PropertyCascade::normalProperties().
        ASSERT(!changedProperties.types && changedProperties.ids.isEmpty());
        changedProperties.types = PropertyCascade::normalPropertyTypes();
        return true;
    default:
        ASSERT(result == WebCoreSwiftMatchResultCacheDiffChangedIDs);
        if (state[2]) {
            changedProperties.ids.appendUsingFunctor(state[2], [&](size_t index) {
                return static_cast<CSSPropertyID>(out[index]);
            });
        }
        return true;
    }
}

const std::optional<CachedMatchResult> MatchResultCache::resultWithCurrentInlineStyle(const Element& element)
{
    auto it = m_entries.find(element);
    if (it == m_entries.end())
        return { };

    auto& entry = it->value;

    auto* styledElement = dynamicDowncast<StyledElement>(element);
    RefPtr inlineStyle = styledElement ? styledElement->inlineStyle() : nullptr;

    PropertyCascade::IncludedProperties changedProperties;
    if (!changedPropertiesAfterInlineStyleChange(entry, inlineStyle.get(), changedProperties)) {
        m_entries.remove(it);
        return { };
    }

    return CachedMatchResult {
        .unadjustedStyle = copy(entry->unadjustedStyle),
        .changedProperties = WTF::move(changedProperties),
        .styleToUpdate = *entry->unadjustedStyle.style
    };
}

void MatchResultCache::update(CachedMatchResult& result, const Style::ComputedStyle& style)
{
    result.styleToUpdate.get() = ComputedStyle::clone(style);
}

void MatchResultCache::updateForFastPathInherit(const Element& element, const Style::ComputedStyle& parentStyle)
{
    CheckedPtr entry = m_entries.get(element);
    if (!entry)
        return;
    entry->unadjustedStyle.style->fastPathInheritFrom(parentStyle);
}

void MatchResultCache::set(const Element& element, const UnadjustedStyle& unadjustedStyle)
{
    // For now we cache match results if there is mutable inline style. This way we can avoid
    // selector matching when it gets mutated again.
    auto* styledElement = dynamicDowncast<StyledElement>(element);
    RefPtr inlineStyle = styledElement ? dynamicDowncast<MutableStyleProperties>(styledElement->inlineStyle()) : nullptr;

    if (inlineStyle)
        m_entries.set(element, makeUniqueRef<Entry>(copy(unadjustedStyle), *inlineStyle));
    else
        m_entries.remove(element);
}

}
}
