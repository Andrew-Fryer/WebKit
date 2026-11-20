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

// C boundary of the SVGPropertyRegistryTable Swift kernel (Source/WebCore/swift/SVGPropertyRegistryTable): the
// attribute-name -> SVGMemberAccessor table of every SVGPropertyOwnerRegistryBase<OwnerType, BaseTypes...>
// (svg/properties/SVGPropertyOwnerRegistry.h). Swift owns the table's layout and every operation that builds or searches
// it; C++ allocates its words and interprets only the published entry array below, to enumerate it without a call per
// OwnerType. An empty span (null, 0) is the empty table.
//
// The symbols are internal to WebCore (hidden). Only WebCore may call them. SVGPropertyOwnerRegistry.h calls them from
// header-inline template code, which is fine only because that header is included by WebCore sources alone (approved
// for this unit: no include, direct or transitive, from outside Source/WebCore, and it isn't a private or exported header).
//
// Handles are opaque 64-bit values that Swift stores and compares and never dereferences: `localName` and
// `namespaceURI` are a QualifiedName's localName().impl() and namespaceURI().impl() (0 for nullAtom), `key` is the
// address of the immortal static QualifiedName the property was registered with, `accessor` the address of its
// SVGMemberAccessor<OwnerType> singleton (never 0).

#include <stddef.h>
#include <stdint.h>

#if defined(__cplusplus) && defined(NODELETE)
#define WEBCORE_SWIFT_SVG_PROPERTY_REGISTRY_TABLE_NODELETE NODELETE
#else
#define WEBCORE_SWIFT_SVG_PROPERTY_REGISTRY_TABLE_NODELETE
#endif

#ifdef __cplusplus
extern "C" {
#endif

// The published part of the layout (mirrored in SVGPropertyRegistryTable.swift): word CountWord of a non-empty table is
// the number of entries; entry i, in registration order, is the EntryWords words from EntriesWord + i * EntryWords, and
// holds the `key` handle at EntryKeyWord and the `accessor` handle at EntryAccessorWord. Every entry lies inside the
// table's words. C++ only reads them.
enum {
    WebCoreSwift_SVGPropertyRegistryTable_CountWord = 1,
    WebCoreSwift_SVGPropertyRegistryTable_EntriesWord = 2,
    WebCoreSwift_SVGPropertyRegistryTable_EntryWords = 4,
    WebCoreSwift_SVGPropertyRegistryTable_EntryKeyWord = 2,
    WebCoreSwift_SVGPropertyRegistryTable_EntryAccessorWord = 3,
};

// The accessor registered under (localName, namespaceURI), i.e. under any QualifiedName that matches() the registered
// one, or 0. A lookup of nullQName() (0, 0) finds nothing.
WEBCORE_SWIFT_SVG_PROPERTY_REGISTRY_TABLE_NODELETE uint64_t WebCoreSwift_SVGPropertyRegistryTable_find(const uint64_t* table, size_t tableWords, uint64_t localName, uint64_t namespaceURI);

// HashMap::add: registers (localName, namespaceURI) -> (key, accessor) unless the name is already there, in which case
// the table is unchanged. Returns 0, or, if the entry doesn't fit, the number of words the grown table needs, changing
// nothing. Then allocate that many words, call rehash() into them, make them the table, and add again.
WEBCORE_SWIFT_SVG_PROPERTY_REGISTRY_TABLE_NODELETE size_t WebCoreSwift_SVGPropertyRegistryTable_add(uint64_t* table, size_t tableWords, uint64_t localName, uint64_t namespaceURI, uint64_t key, uint64_t accessor);

// Initializes every one of the `grownWords` words (the count add() returned) as a table with the entries of `table`.
WEBCORE_SWIFT_SVG_PROPERTY_REGISTRY_TABLE_NODELETE void WebCoreSwift_SVGPropertyRegistryTable_rehash(const uint64_t* table, size_t tableWords, uint64_t* grown, size_t grownWords);


#ifdef __cplusplus
}
#endif
