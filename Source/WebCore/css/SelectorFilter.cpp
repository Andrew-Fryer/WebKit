/*
 * Copyright (C) 1999 Lars Knoll (knoll@kde.org)
 *           (C) 2004-2005 Allan Sandfeld Jensen (kde@carewolf.com)
 * Copyright (C) 2006, 2007 Nicholas Shanks (webkit@nickshanks.com)
 * Copyright (C) 2005, 2006, 2007, 2008, 2009, 2010, 2011 Apple Inc. All rights reserved.
 * Copyright (C) 2007 Alexey Proskuryakov <ap@webkit.org>
 * Copyright (C) 2007, 2008 Eric Seidel <eric@webkit.org>
 * Copyright (C) 2008, 2009 Torch Mobile Inc. All rights reserved. (http://www.torchmobile.com/)
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 * Copyright (C) Research In Motion Limited 2011. All rights reserved.
 * Copyright (C) 2012 Google Inc. All rights reserved.
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
#include "SelectorFilter.h"

#include "CSSSelectorList.h"
#include "CommonAtomStrings.h"
#include "ElementInlines.h"
#include "HTMLNames.h"
#include "ShadowRoot.h"
#include "StyledElement.h"

#if USE(SWIFT_KERNELS)
#include "SelectorFilterSwiftBoundary.h"
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

namespace WebCore {

// Salt to separate otherwise identical string hashes so a class-selector like .article won't match <article> elements.
enum { TagNameSalt = 13, IdSalt = 17, ClassSalt = 19, AttributeSalt = 23 };

NODELETE static bool isExcludedAttribute(const AtomString& name)
{
    return name == HTMLNames::classAttr->localName() || name == HTMLNames::idAttr->localName() || name == HTMLNames::styleAttr->localName();
}

void SelectorFilter::collectElementIdentifierHashes(const Element& element, Vector<unsigned, 4>& identifierHashes)
{
    identifierHashes.append(element.localNameLowercase().impl()->existingHash() * TagNameSalt);

    auto& id = element.idForStyleResolution();
    if (!id.isNull())
        identifierHashes.append(id.impl()->existingHash() * IdSalt);

    if (element.hasClass()) {
        identifierHashes.appendContainerWithMapping(element.classNames(), [](auto& className) {
            return className.impl()->existingHash() * ClassSalt;
        });
    }
    
    if (element.hasAttributesWithoutUpdate()) {
        for (auto& attribute : element.attributes()) {
            auto& attributeName = attribute.localNameLowercase();
            if (isExcludedAttribute(attributeName))
                continue;
            identifierHashes.append(attributeName.impl()->existingHash() * AttributeSalt);
        }
    }
}

bool SelectorFilter::parentStackIsConsistent(const ContainerNode* parentNode) const
{
    if (!parentNode || isAnyOf<Document, ShadowRoot>(*parentNode))
        return m_parentStack.isEmpty();

    return !m_parentStack.isEmpty() && m_parentStack.last().element == parentNode;
}

void SelectorFilter::initializeParentStack(Element& parent)
{
    Vector<Element*, 20> ancestors;
    for (CheckedPtr ancestor = &parent; ancestor; ancestor = ancestor->parentElement())
        ancestors.append(ancestor.get());
    m_parentStack.reserveCapacity(ancestors.size());
    for (unsigned i = ancestors.size(); i--;)
        pushParent(protect(ancestors[i]));
}

void SelectorFilter::pushParentCxx(Element* parent)
{
    ASSERT(m_parentStack.isEmpty() || m_parentStack.last().element == parent->parentElement());
    ASSERT(!m_parentStack.isEmpty() || !parent->parentElement());
    m_parentStack.append(ParentStackFrame(parent));
    ParentStackFrame& parentFrame = m_parentStack.last();
    // Mix tags, class names and ids into some sort of weird bouillabaisse.
    // The filter is used for fast rejection of child and descendant selectors.
    collectElementIdentifierHashes(*parent, parentFrame.identifierHashes);
    for (auto hash : parentFrame.identifierHashes.span())
        m_ancestorIdentifierFilter.add(hash);
}

void SelectorFilter::pushParentInitializingIfNeeded(Element& parent)
{
    if (m_parentStack.isEmpty()) [[unlikely]] {
        initializeParentStack(parent);
        return;
    }
    pushParent(&parent);
}

void SelectorFilter::popParentCxx()
{
    ASSERT(!m_parentStack.isEmpty());
    const ParentStackFrame& parentFrame = m_parentStack.last();
    for (auto hash : parentFrame.identifierHashes.span())
        m_ancestorIdentifierFilter.remove(hash);
    m_parentStack.removeLast();
    if (m_parentStack.isEmpty()) {
        ASSERT(m_ancestorIdentifierFilter.likelyEmpty());
        m_ancestorIdentifierFilter.clear();
    }
}

#if USE(SWIFT_KERNELS)
// The Swift kernels update the CountingBloomFilter's bucket array in place (SelectorFilterSwiftBoundary.h).
template<unsigned keyBits> static std::span<uint8_t> bucketsForSwift(CountingBloomFilter<keyBits>& filter)
{
    static_assert(sizeof(CountingBloomFilter<keyBits>) == CountingBloomFilter<keyBits>::tableSize);
    return asMutableByteSpan(filter);
}

template<unsigned keyBits> static void addHashesWithSwift(CountingBloomFilter<keyBits>& filter, std::span<const unsigned> hashes)
{
    static_assert(sizeof(unsigned) == sizeof(uint32_t));
    auto buckets = bucketsForSwift(filter);
    WebCoreSwift_SelectorFilter_addHashes(buckets.data(), buckets.size(), hashes.data(), hashes.size());
}

template<unsigned keyBits> NODELETE static void removeHashesWithSwift(CountingBloomFilter<keyBits>& filter, std::span<const unsigned> hashes)
{
    auto buckets = bucketsForSwift(filter);
    WebCoreSwift_SelectorFilter_removeHashes(buckets.data(), buckets.size(), hashes.data(), hashes.size());
}

template<unsigned keyBits> NODELETE static void clearWithSwift(CountingBloomFilter<keyBits>& filter)
{
    auto buckets = bucketsForSwift(filter);
    WebCoreSwift_SelectorFilter_clear(buckets.data(), buckets.size());
}

#if ENABLE(SWIFT_KERNELS_VALIDATION)
template<unsigned keyBits> NODELETE static bool bucketsAreEqual(const CountingBloomFilter<keyBits>& a, const CountingBloomFilter<keyBits>& b)
{
    return equalSpans(asByteSpan(a), asByteSpan(b));
}
#endif
#endif // USE(SWIFT_KERNELS)

void SelectorFilter::pushParent(Element* parent)
{
#if USE(SWIFT_KERNELS)
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]] {
        // The original updates the real filter; Swift updates a copy taken before, with the hashes the original collected.
        auto filter = m_ancestorIdentifierFilter;
        pushParentCxx(parent);
        addHashesWithSwift(filter, m_parentStack.last().identifierHashes.span());
        RELEASE_ASSERT(bucketsAreEqual(filter, m_ancestorIdentifierFilter));
        return;
    }
#endif
    ASSERT(m_parentStack.isEmpty() || m_parentStack.last().element == parent->parentElement());
    ASSERT(!m_parentStack.isEmpty() || !parent->parentElement());
    m_parentStack.append(ParentStackFrame(parent));
    ParentStackFrame& parentFrame = m_parentStack.last();
    // Mix tags, class names and ids into some sort of weird bouillabaisse.
    // The filter is used for fast rejection of child and descendant selectors.
    collectElementIdentifierHashes(*parent, parentFrame.identifierHashes);
    addHashesWithSwift(m_ancestorIdentifierFilter, parentFrame.identifierHashes.span());
#else
    pushParentCxx(parent);
#endif
}

void SelectorFilter::popParent()
{
#if USE(SWIFT_KERNELS)
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]] {
        // The original updates the real filter; Swift updates a copy taken before.
        auto filter = m_ancestorIdentifierFilter;
        removeHashesWithSwift(filter, m_parentStack.last().identifierHashes.span());
        if (m_parentStack.size() == 1)
            clearWithSwift(filter);
        popParentCxx();
        RELEASE_ASSERT(bucketsAreEqual(filter, m_ancestorIdentifierFilter));
        return;
    }
#endif
    ASSERT(!m_parentStack.isEmpty());
    const ParentStackFrame& parentFrame = m_parentStack.last();
    removeHashesWithSwift(m_ancestorIdentifierFilter, parentFrame.identifierHashes.span());
    m_parentStack.removeLast();
    if (m_parentStack.isEmpty()) {
        ASSERT(m_ancestorIdentifierFilter.likelyEmpty());
        clearWithSwift(m_ancestorIdentifierFilter);
    }
#else
    popParentCxx();
#endif
}

void SelectorFilter::popParentsUntil(Element* parent)
{
    while (!m_parentStack.isEmpty()) {
        if (parent && m_parentStack.last().element == parent)
            return;
        popParent();
    }
}

void SelectorFilter::collectSimpleSelectorHash(CollectedSelectorHashes& collectedHashes, const CSSSelector& selector)
{
    switch (selector.match()) {
    case CSSSelector::Match::Id:
        if (!selector.value().isEmpty())
            collectedHashes.ids.append(selector.value().impl()->existingHash() * IdSalt);
        break;
    case CSSSelector::Match::Class:
        if (!selector.value().isEmpty())
            collectedHashes.classes.append(selector.value().impl()->existingHash() * ClassSalt);
        break;
    case CSSSelector::Match::Tag: {
        auto& tagLowercaseLocalName = selector.tagLowercaseLocalName();
        if (tagLowercaseLocalName != starAtom())
            collectedHashes.tags.append(tagLowercaseLocalName.impl()->existingHash() * TagNameSalt);
        break;
    }
    case CSSSelector::Match::Exact:
    case CSSSelector::Match::Set:
    case CSSSelector::Match::List:
    case CSSSelector::Match::Hyphen:
    case CSSSelector::Match::Contain:
    case CSSSelector::Match::Begin:
    case CSSSelector::Match::End: {
        if (selector.isEquivalentToClassSelector()) {
            collectedHashes.classes.append(selector.value().impl()->existingHash() * ClassSalt);
            break;
        }
        auto attributeName = selector.attribute().localNameLowercase();
        if (!isExcludedAttribute(attributeName))
            collectedHashes.attributes.append(attributeName.impl()->existingHash() * AttributeSalt);
        break;
    }
    case CSSSelector::Match::PseudoClass:
        switch (selector.pseudoClass()) {
        case CSSSelector::PseudoClass::Is:
        case CSSSelector::PseudoClass::Where:
            // We can use the filter in the trivial case of single argument :is()/:where().
            // Supporting the multiargument case would require more than one hash.
            if (selector.selectorList()->size() == 1)
                collectSelectorHashes(collectedHashes, selector.selectorList()->first(), IncludeRightmost::Yes);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void SelectorFilter::collectSelectorHashes(CollectedSelectorHashes& collectedHashes, const CSSSelector& rightmostSelector, IncludeRightmost includeRightmost)
{
    auto [selector, relation, skipOverSubselectors] = [&] {
        if (includeRightmost == IncludeRightmost::No)
            return std::tuple { rightmostSelector.precedingInComplexSelector(), rightmostSelector.relation(), true };

        return std::tuple { &rightmostSelector, CSSSelector::Relation::Subselector, false };
    }();

    for (; selector; selector = selector->precedingInComplexSelector()) {
        // Only collect identifiers that match ancestors.
        switch (relation) {
        case CSSSelector::Relation::Subselector:
            if (!skipOverSubselectors)
                collectSimpleSelectorHash(collectedHashes, *selector);
            break;
        case CSSSelector::Relation::DirectAdjacent:
        case CSSSelector::Relation::IndirectAdjacent:
        case CSSSelector::Relation::ShadowDescendant:
        case CSSSelector::Relation::ShadowPartDescendant:
        case CSSSelector::Relation::ShadowSlotted:
            skipOverSubselectors = true;
            break;
        case CSSSelector::Relation::DescendantSpace:
        case CSSSelector::Relation::Child:
            skipOverSubselectors = false;
            collectSimpleSelectorHash(collectedHashes, *selector);
            break;
        }
        relation = selector->relation();
    }
}

auto SelectorFilter::chooseSelectorHashesForFilter(const CollectedSelectorHashes& collectedSelectorHashes) -> Hashes
{
    Hashes resultHashes;
    unsigned index = 0;

    auto addIfNew = [&] (unsigned hash) {
        for (unsigned i = 0; i < index; ++i) {
            if (resultHashes[i] == hash)
                return;
        }
        resultHashes[index++] = hash;
    };

    auto copyHashes = [&] (auto& hashes) {
        for (auto& hash : hashes) {
            addIfNew(hash);
            if (index == resultHashes.size())
                return true;
        }
        return false;
    };

    // There is a limited number of slots. Prefer more specific selector types.
    if (copyHashes(collectedSelectorHashes.ids))
        return resultHashes;
    if (copyHashes(collectedSelectorHashes.attributes))
        return resultHashes;
    if (copyHashes(collectedSelectorHashes.classes))
        return resultHashes;
    if (copyHashes(collectedSelectorHashes.tags))
        return resultHashes;

    // Null-terminate if not full.
    resultHashes[index] = 0;
    return resultHashes;
}

SelectorFilter::Hashes SelectorFilter::collectHashes(const CSSSelector& selector)
{
    CollectedSelectorHashes collectedHashes;
    collectSelectorHashes(collectedHashes, selector, IncludeRightmost::No);
    return chooseSelectorHashesForFilter(collectedHashes);
}

SelectorFilter::CollectedSelectorHashes SelectorFilter::collectHashesForTesting(const CSSSelector& selector)
{
    CollectedSelectorHashes collectedHashes;
    collectSelectorHashes(collectedHashes, selector, IncludeRightmost::No);
    return collectedHashes;
}

}

