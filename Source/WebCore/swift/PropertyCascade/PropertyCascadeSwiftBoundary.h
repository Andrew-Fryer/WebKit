/*
 * Copyright (C) 2026 Apple Inc. All rights reserved.
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

#pragma once

// C boundary of the PropertyCascade Swift kernel (Source/WebCore/swift/PropertyCascade), called by
// Style::PropertyCascade::buildCascade() in style/PropertyCascade.cpp. The symbol is internal to WebCore
// (hidden); only WebCore .cpp files may call it, never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Builds a whole PropertyCascade (buildCascade with its normal and !important passes up to the maximum origin, the
// position-try fallback, the rollback limits, revert-rule delaying, the !important stable sort and
// sortLogicalGroupPropertyIDs) into the cascade's own state, for a fresh cascade (no property present, no
// logical-group index set, no custom property).
//
// matches: 3 words per match (matchWordCount = 3 * match count), UserAgent, User, Author, then the position-try
//   fallback match if any:
//   [0] the address of the match's property storage: ImmutableStyleProperties::m_storage
//       (StylePropertyMetadata[n] then PackedPtr<const CSSValue>[n]) or the MutableStyleProperties
//       CSSProperty[n] array;
//   [1] the MatchedProperties scalars in their byte order: linkMatchType | allowlistType << 8 |
//       uint8(styleScopeOrdinal) << 16 | fromStyleAttribute << 24 | cascadeLayerPriority << 32 |
//       isStartingStyle << 48 | isCacheable << 56;
//   [2] the property count n | isMutable << 63.
// state[19] (stateCount = 19): [0-3] the UserAgent, User, Author and position-try (0 or 1) match counts,
//   [4] includedTypes (OptionSet<PropertyType> bits), [5] firstLogicalGroupProperty, [6] sizeof(PackedPtr<const CSSValue>),
//   [7] the scratch count S, [8] m_maximumOrigin, [9] m_rollbackScope (0x100 | uint8 bit pattern, or 0),
//   [10] m_maximumCascadeLayerPriorityForRollback (0x10000 | priority, or 0), [11] m_ruleRollbackDepth,
//   [12] T = the property count of all matches, [13] N = the number of custom-property name ids in `inputs`;
//   then, in/out, [14] m_lastIndexForLogicalGroup, [15] m_seenLogicalGroupPropertyCount (out),
//   [16] m_lowestSeenLogicalGroupProperty, [17] m_highestSeenLogicalGroupProperty, and [18] the overridden count (out).
// includedIDs: IncludedProperties::ids. table[cssPropertyIDEnumValueCount]: cascadeAliasProperty(id) |
//   CSSProperty::isInheritedProperty(id) << 15 | (the allowlist check admits cascadeAliasProperty(id) for PropertyAllowlist k) << (16 + k).
// inputs: empty, or T entries, one per property of the matches in order: for a custom property, its name id (< N) in
//   bits 0-31; shouldApplyAfterAnimation() in bit 32; explicitlyInheritsFromParent() in bit 33. The glue gathers the
//   ones the cascade's included types can reach (see PropertyCascade::buildCascade()); the kernel traps on a missing one.
// properties: m_properties (40-byte Property records: id u16 @0, origin u8 @2, styleScopeOrdinal @3,
//   cascadeLayerPriority u16 @4, fromStyleAttribute @6, cssValue[3] @8, origins[3] @32).
// propertyIsPresent: m_propertyIsPresent words. logicalGroupIndices / logicalGroupIDs: the
//   m_logicalGroupPropertyIndices / m_logicalGroupPropertyIDs arrays, logicalGroupCount entries each.
// work (no initial contents needed), in order: S >= max(logicalGroupCount, the UserAgent, User and Author match counts)
//   scratch words; T for the custom-property sets; T more if includedTypes has AfterAnimation (the overridden
//   properties); if inputs is not empty, one per match plus (N + 63) / 64; if m_ruleRollbackDepth, 2 * (tableCount + N)
//   plus 2 * T.
//
// Returns the number n of custom-property sets the caller must apply, in order, to m_customProperties (presence bit
// CSSPropertyID::Custom is already set): work[S + i] is match << 32 | property index. The state[18] properties overridden
// for AfterAnimation are work[S + T + i], in the order the cascade adds them to overriddenProperties.
size_t WebCoreSwift_PropertyCascade_build(const uint64_t* matches, size_t matchWordCount, uint32_t* state, size_t stateCount,
    const uint16_t* includedIDs, size_t includedIDCount, const uint32_t* table, size_t tableCount, const uint64_t* inputs, size_t inputCount,
    void* properties, size_t propertyBytes, uint64_t* propertyIsPresent, size_t propertyIsPresentWordCount,
    uint32_t* logicalGroupIndices, uint16_t* logicalGroupIDs, size_t logicalGroupCount,
    uint64_t* work, size_t workCount);

#ifdef __cplusplus
}
#endif
