/*
 * Copyright (C) 2019 Apple Inc. All rights reserved.
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

#include "MatchResult.h"
#include "RenderStyle.h"
#include "Timer.h"
#include <wtf/TZoneMalloc.h>
#include <wtf/WeakRef.h>

namespace WebCore {

namespace Style {

class Resolver;

class MatchedDeclarationsCache {
    WTF_MAKE_TZONE_ALLOCATED(MatchedDeclarationsCache);
public:
    explicit MatchedDeclarationsCache(const Resolver&);
    ~MatchedDeclarationsCache();

    // static bool isCacheable(const Element&, const RenderStyle&, const RenderStyle& parentStyle);
    // static unsigned computeHash(const MatchResult&, const StyleCustomPropertyData& inheritedCustomProperties);

    class Key {
    public: // make this a struct?
        MatchResult m_matchResult;
        std::unique_ptr<const RenderStyle> m_parentRenderStyle;
        size_t m_hash;
        
        Key Key(MatchResult, StyleCustomPropertyData, std::unique_ptr<const RenderStyle>); // remove this?
        size_t hash();
        bool equals(const Key&);
    };

    // struct MatchedDeclarationsCacheHash {
    //     size_t operator()(const MatchedDeclarationsCacheKey& s) const {
    //         return asdf;
    //     }
    // };
    
    // struct MatchedDeclarationsCacheEqual {
    //     bool operator()(const MatchedDeclarationsCacheKey& a, const MatchedDeclarationsCacheKey& b) const {
    //     }
    // };

    struct Entry {
        MatchResult matchResult; // this definitely shouldn't be here
        std::unique_ptr<const RenderStyle> renderStyle;
        std::unique_ptr<const RenderStyle> parentRenderStyle;
        std::unique_ptr<const RenderStyle> userAgentAppearanceStyle;

        bool isUsableAfterHighPriorityProperties(const RenderStyle&) const;
    };

    std::optional<Key> createKeyIfCacheable(const MatchResult& matchResult, const Element& element, const RenderStyle& style, const RenderStyle& parentStyle);
    const Entry& find(const Key& key);
    void add(const Key& key);
    void remove(const Key& key);

    // Every N additions to the matched declaration cache trigger a sweep where entries holding
    // the last reference to a style declaration are garbage collected.
    void invalidate();
    void clearEntriesAffectedByViewportUnits();

    void ref() const;
    void deref() const;

private:
    void sweep();

    SingleThreadWeakRef<const Resolver> m_owner;
    UncheckedKeyHashMap<unsigned, Entry, AlreadyHashed> m_entries;
    std::unordered_multimap<Key, 
    Timer m_sweepTimer;
    unsigned m_additionsSinceLastSweep { 0 };
};

}
}
