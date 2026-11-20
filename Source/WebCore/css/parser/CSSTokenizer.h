// Copyright 2014 The Chromium Authors. All rights reserved.
// Copyright (C) 2016-2024 Apple Inc. All rights reserved.
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

#pragma once

#include <WebCore/CSSParserToken.h>
#include <WebCore/CSSTokenizerInputStream.h>
#include <climits>
#include <wtf/MallocSpan.h>
#include <wtf/text/StringView.h>

namespace WebCore {

class CSSParserObserverWrapper;
class CSSParserTokenRange;

DECLARE_ALLOCATOR_WITH_HEAP_IDENTIFIER(CSSTokenizer);
class CSSTokenizer {
    WTF_MAKE_NONCOPYABLE(CSSTokenizer);
    WTF_DEPRECATED_MAKE_FAST_ALLOCATED_WITH_HEAP_IDENTIFIER(CSSTokenizer, CSSTokenizer);
public:
    static std::unique_ptr<CSSTokenizer> tryCreate(StringView string LIFETIME_BOUND);
    static std::unique_ptr<CSSTokenizer> tryCreate(StringView string LIFETIME_BOUND, CSSParserObserverWrapper&); // For the inspector

    WEBCORE_EXPORT explicit CSSTokenizer(StringView string LIFETIME_BOUND);
    CSSTokenizer(StringView string LIFETIME_BOUND, CSSParserObserverWrapper&); // For the inspector

    NODELETE WEBCORE_EXPORT CSSParserTokenRange tokenRange() const LIFETIME_BOUND;
    NODELETE unsigned tokenCount() const;

    NODELETE static bool isWhitespace(CSSParserTokenType);

    Vector<String>&& escapedStringsForAdoption() { return WTF::move(m_stringPool); }

private:
    CSSTokenizer(StringView string LIFETIME_BOUND, CSSParserObserverWrapper*, bool* constructionSuccess);

    // The tokenization itself (the input stream, the block stack, every token) is Swift: swift/CSSTokenizer. It writes
    // the tokens, in CSSParserToken's layout, straight into m_tokenStorage: m_inlineTokenStorage, or m_tokenBuffer once
    // the input has more tokens than fit inline.
    static constexpr size_t inlineTokenCapacity = 32;
    std::span<const CSSParserToken> tokens() const LIFETIME_BOUND { return m_tokenStorage.first(m_tokenCount); }

    std::span<CSSParserToken> m_tokenStorage;
    unsigned m_tokenCount { 0 };
    std::optional<MallocSpan<CSSParserToken>> m_tokenBuffer; // Engaged only for inputs with more tokens than fit inline.
    alignas(CSSParserToken) std::array<std::byte, inlineTokenCapacity * sizeof(CSSParserToken)> m_inlineTokenStorage;
    // We only allocate strings when escapes are used.
    Vector<String> m_stringPool;
};

} // namespace WebCore
