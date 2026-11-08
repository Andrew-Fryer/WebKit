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

// Builds a whole PropertyCascade (buildCascade with its normal and !important passes, the !important
// stable sort and sortLogicalGroupPropertyIDs) into the cascade's own state, for a fresh cascade (no
// property present, no logical-group index set, no custom property) without rollback, position-try
// fallback, AfterAnimation, property allowlists, or ExplicitlyInherited without both normal types.
//
// matches: 3 words per match (matchWordCount = 3 * match count), UserAgent then User then Author:
//   [0] the address of the match's property storage: ImmutableStyleProperties::m_storage
//       (StylePropertyMetadata[n] then PackedPtr<const CSSValue>[n]) or the MutableStyleProperties
//       CSSProperty[n] array;
//   [1] the MatchedProperties scalars in their byte order: linkMatchType | allowlistType << 8 |
//       uint8(styleScopeOrdinal) << 16 | fromStyleAttribute << 24 | cascadeLayerPriority << 32 |
//       isStartingStyle << 48 | isCacheable << 56;
//   [2] the storage byte count | isMutable << 63.
// config[7]: UserAgent, User and Author match counts, includedTypes (OptionSet<PropertyType> bits),
//   firstLogicalGroupProperty, sizeof(PackedPtr<const CSSValue>), and the scratch count S.
// includedIDs: IncludedProperties::ids. table[cssPropertyIDEnumValueCount]: cascadeAliasProperty(id) |
//   CSSProperty::isInheritedProperty(id) << 15.
// properties: m_properties (40-byte Property records: id u16 @0, origin u8 @2, styleScopeOrdinal @3,
//   cascadeLayerPriority u16 @4, fromStyleAttribute @6, cssValue[3] @8, origins[3] @32).
// propertyIsPresent: m_propertyIsPresent words. logicalGroupIndices / logicalGroupIDs: the
//   m_logicalGroupPropertyIndices / m_logicalGroupPropertyIDs arrays, logicalGroupCount entries each.
// scalars[4] in/out: m_lastIndexForLogicalGroup, m_seenLogicalGroupPropertyCount,
//   m_lowestSeenLogicalGroupProperty, m_highestSeenLogicalGroupProperty.
// work: S >= max(logicalGroupCount, largest origin match count) scratch words, then room for one
//   entry per property of all matches.
//
// Returns the number n of CSSPropertyID::Custom sets the caller must apply, in order, with set(): work[S + i]
// is match << 32 | property index. Returns -1 when the caller must run the C++ instead; the presence words
// and logical-group indices have then been zeroed again and nothing else the C++ reads was changed.
ptrdiff_t WebCoreSwift_PropertyCascade_build(const uint64_t* matches, size_t matchWordCount, const uint32_t* config, size_t configCount,
    const uint16_t* includedIDs, size_t includedIDCount, const uint16_t* table, size_t tableCount,
    void* properties, size_t propertyBytes, uint64_t* propertyIsPresent, size_t propertyIsPresentWordCount,
    uint32_t* logicalGroupIndices, uint16_t* logicalGroupIDs, size_t logicalGroupCount,
    uint32_t* scalars, uint64_t* work, size_t workCount);

#ifdef __cplusplus
}
#endif
