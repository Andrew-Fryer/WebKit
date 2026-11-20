// Copyright 2015 The Chromium Authors. All rights reserved.
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

#include "config.h"
#include "CSSTokenizer.h"

#include "CSSParserObserverWrapper.h"
#include "CSSParserTokenRange.h"
#include "CSSTokenizerSwiftBoundary.h"
#include <bit>
#include <wtf/text/StringImpl.h>

namespace WebCore {
DEFINE_ALLOCATOR_WITH_HEAP_IDENTIFIER(CSSTokenizer);

// The Swift tokenizer writes each token as the three words of a CSSParserToken (see swift/CSSTokenizer/CSSTokenizer.swift).
static_assert(sizeof(CSSParserToken) == 3 * sizeof(uint64_t));
static_assert(alignof(CSSParserToken) == alignof(uint64_t));
static_assert(std::is_trivially_copyable_v<CSSParserToken>);
static_assert(std::is_trivially_destructible_v<CSSParserToken>);
static_assert(EOFToken == 31 && CommentToken == 32 && LeftParenthesisToken == 23 && StringToken == 29);

static constexpr uint64_t tokenRecordRegistrationBit = 1ull << 31;
static constexpr size_t inlineBlockStackCapacity = 32;

#if ASSERT_ENABLED
// The record's fields, as CSSParserToken reads them, must be the ones the Swift encoded.
static void checkTokenRecord(const CSSParserToken& token, std::array<uint64_t, 3> words)
{
    ASSERT(static_cast<uint64_t>(token.type()) == (words[0] & 0x3f));
    ASSERT(static_cast<uint64_t>(token.getBlockType()) == ((words[0] >> 6) & 3));
    ASSERT(token.value().length() == (words[0] >> 32));
    ASSERT(token.value().is8Bit() == !!(words[0] & (1u << 22)));
    ASSERT(std::bit_cast<uintptr_t>(token.value().rawCharacters()) == words[1]);
    ASSERT(!token.isBackedByStringLiteral());
    if (token.type() == NumberToken || token.type() == PercentageToken || token.type() == DimensionToken) {
        ASSERT(static_cast<uint64_t>(token.numericValueType()) == ((words[0] >> 8) & 1));
        ASSERT(static_cast<uint64_t>(std::to_underlying(token.unitType())) == ((words[0] >> 11) & 0x7f));
        ASSERT(std::bit_cast<uint64_t>(token.numericValue()) == words[2]);
    }
    if (token.type() == NumberToken)
        ASSERT(static_cast<uint64_t>(token.numericSign()) == ((words[0] >> 9) & 3));
    if (token.type() == DimensionToken)
        ASSERT(token.value().length() - token.unitString().length() == ((words[0] >> 18) & 0xf));
    if (token.type() == DelimiterToken)
        ASSERT(token.delimiter() == static_cast<char16_t>(words[2]));
}
#endif

std::unique_ptr<CSSTokenizer> CSSTokenizer::tryCreate(StringView string)
{
    bool success = true;
    // We can't use makeUnique here because it does not have access to this private constructor.
    auto tokenizer = std::unique_ptr<CSSTokenizer>(new CSSTokenizer(string, nullptr, &success));
    if (!success) [[unlikely]]
        return nullptr;
    return tokenizer;
}

std::unique_ptr<CSSTokenizer> CSSTokenizer::tryCreate(StringView string, CSSParserObserverWrapper& wrapper)
{
    bool success = true;
    // We can't use makeUnique here because it does not have access to this private constructor.
    auto tokenizer = std::unique_ptr<CSSTokenizer>(new CSSTokenizer(string, &wrapper, &success));
    if (!success) [[unlikely]]
        return nullptr;
    return tokenizer;
}

CSSTokenizer::CSSTokenizer(StringView string)
    : CSSTokenizer(string, nullptr, nullptr)
{
}

CSSTokenizer::CSSTokenizer(StringView string, CSSParserObserverWrapper& wrapper)
    : CSSTokenizer(string, &wrapper, nullptr)
{
}

// Glue around the Swift tokenizer: it allocates the token storage, the block stack and the string pool, and turns the
// pool registrations into the m_stringPool Strings the original registered (and points their tokens at them).
CSSTokenizer::CSSTokenizer(StringView string, CSSParserObserverWrapper* wrapper, bool* constructionSuccessPtr)
{
    m_tokenStorage = spanReinterpretCast<CSSParserToken>(std::span { m_inlineTokenStorage });
    if (constructionSuccessPtr)
        *constructionSuccessPtr = true;

    if (string.isEmpty())
        return;

    // To avoid resizing we err on the side of reserving too much space.
    // Most strings we tokenize have about 3.5 to 5 characters per token.
    auto failed = [&] {
        // When constructionSuccessPtr is null, our policy is to crash on failure.
        RELEASE_ASSERT(constructionSuccessPtr);
        *constructionSuccessPtr = false;
        m_tokenCount = 0;
    };
    auto allocateTokens = [&](size_t capacity) -> bool {
        if (capacity > std::numeric_limits<unsigned>::max() / sizeof(CSSParserToken))
            return false;
        auto buffer = MallocSpan<CSSParserToken>::tryMalloc(capacity * sizeof(CSSParserToken));
        if (!buffer)
            return false;
        memcpySpan(buffer.mutableSpan(), tokens());
        m_tokenBuffer = WTF::move(buffer);
        m_tokenStorage = m_tokenBuffer->mutableSpan();
        return true;
    };
    if (string.length() / 3 > inlineTokenCapacity && !allocateTokens(string.length() / 3)) [[unlikely]] {
        failed();
        return;
    }

    // The storage the Swift tokenizer needs only for deep nesting, escapes and the like.
    struct SlowCaseStorage {
        Vector<uint8_t> blockStack;
        Vector<char16_t> pool;
        Vector<uint64_t> registrations;
        Vector<uint32_t> offsets;
    };
    std::optional<SlowCaseStorage> slowCaseStorage;
    std::array<uint64_t, WebCoreSwiftCSSTokenizerStateCount> state { };
    std::array<uint8_t, inlineBlockStackCapacity> inlineBlockStack;
    state[WebCoreSwiftCSSTokenizerBlockStack] = std::bit_cast<uintptr_t>(inlineBlockStack.data());
    state[WebCoreSwiftCSSTokenizerBlockStackCapacity] = inlineBlockStack.size();
    if (wrapper) [[unlikely]] {
        slowCaseStorage.emplace();
        state[WebCoreSwiftCSSTokenizerFlags] = WebCoreSwiftCSSTokenizerFlagObserving;
    }

    StringView input = string;
    size_t registrationBase = 0;
    size_t materializedRegistrations = 0;
    bool hasPendingComment = false;
    unsigned pendingCommentStart = 0;
    unsigned pendingCommentTokensBefore = 0;
    while (true) {
        auto room = m_tokenStorage.subspan(m_tokenCount);
#if ASSERT_ENABLED
        size_t tokenCountBefore = m_tokenCount;
#endif
        if (wrapper) [[unlikely]] {
            if (slowCaseStorage->offsets.size() < room.size())
                slowCaseStorage->offsets.grow(room.size());
            state[WebCoreSwiftCSSTokenizerOffsets] = std::bit_cast<uintptr_t>(slowCaseStorage->offsets.mutableSpan().data());
        }
        auto result = WebCoreSwift_CSSTokenizer_tokenize(input.rawCharacters(), input.length(), input.is8Bit(), reinterpret_cast<uint64_t*>(room.data()), room.size(), state.data());
        size_t recordCount = static_cast<uint32_t>(result);
        auto status = result >> 32;
        RELEASE_ASSERT(recordCount <= room.size());

        if (status == WebCoreSwiftCSSTokenizerNeedPreprocessing) [[unlikely]] {
            // https://drafts.csswg.org/css-syntax/#input-preprocessing (preprocessString): tokenize a copy with NULs
            // and unpaired surrogates replaced by U+FFFD instead. We don't replace '\r' and '\f' with '\n' as the
            // specification suggests, instead the tokenizer treats them all the same.
            std::span<char16_t> characters;
            auto preprocessed = String::createUninitialized(input.length(), characters);
            WebCoreSwift_CSSTokenizer_preprocess(input.rawCharacters(), input.length(), input.is8Bit(), characters.data());
            m_stringPool.append(WTF::move(preprocessed));
            input = m_stringPool.last();
            registrationBase = m_stringPool.size();
            state[WebCoreSwiftCSSTokenizerFlags] |= WebCoreSwiftCSSTokenizerFlagPreprocessed;
            continue;
        }

        size_t registrationCount = state[WebCoreSwiftCSSTokenizerRegistrationCount];
        if (registrationCount > materializedRegistrations || wrapper) [[unlikely]] {
            // registerString(builder.toString()) for each value the new tokens built.
            for (; materializedRegistrations < registrationCount; ++materializedRegistrations) {
                auto registration = slowCaseStorage->registrations[materializedRegistrations];
                auto units = slowCaseStorage->pool.span().subspan(static_cast<uint32_t>(registration), (registration >> 32) & 0x7fffffff);
                if (units.empty())
                    m_stringPool.append(emptyString());
                else if (registration >> 63) {
                    std::span<Latin1Character> characters;
                    auto value = String::createUninitialized(units.size(), characters);
                    for (size_t i = 0; i < units.size(); ++i)
                        characters[i] = static_cast<Latin1Character>(units[i]);
                    m_stringPool.append(WTF::move(value));
                } else
                    m_stringPool.append(String { units });
            }
            // Point the new tokens whose values are registered at their Strings, and (observer mode) report the
            // tokens and comments, dropping the comment records.
            auto newTokens = room.first(recordCount);
            size_t kept = m_tokenCount;
            for (size_t i = 0; i < recordCount; ++i) {
                auto words = std::bit_cast<std::array<uint64_t, 3>>(newTokens[i]);
                if (words[0] & tokenRecordRegistrationBit) {
                    words[0] &= ~tokenRecordRegistrationBit;
                    words[1] = std::bit_cast<uintptr_t>(StringView { m_stringPool[registrationBase + words[1]] }.rawCharacters());
                }
                if (wrapper) {
                    unsigned start = slowCaseStorage->offsets[i];
                    if (hasPendingComment) {
                        wrapper->addComment(pendingCommentStart, start, pendingCommentTokensBefore);
                        hasPendingComment = false;
                    }
                    if ((words[0] & 0x3f) == CommentToken) {
                        hasPendingComment = true;
                        pendingCommentStart = start;
                        pendingCommentTokensBefore = kept;
                        continue;
                    }
                    wrapper->addToken(start);
                }
                m_tokenStorage[kept++] = std::bit_cast<CSSParserToken>(words);
            }
            m_tokenCount = kept;
        } else
            m_tokenCount += recordCount;
#if ASSERT_ENABLED
        for (auto& token : tokens().subspan(tokenCountBefore))
            checkTokenRecord(token, std::bit_cast<std::array<uint64_t, 3>>(token));
#endif

        if (status == WebCoreSwiftCSSTokenizerDone)
            break;
        if (status == WebCoreSwiftCSSTokenizerTokensFull) {
            // Vector's growth policy (expandCapacity).
            size_t capacity = m_tokenStorage.size();
            if (!allocateTokens(std::max<size_t>(m_tokenCount + 1, std::max<size_t>(16, capacity + capacity / 4 + 1)))) [[unlikely]] {
                failed();
                return;
            }
            continue;
        }
        if (!slowCaseStorage)
            slowCaseStorage.emplace();
        auto& blockStack = slowCaseStorage->blockStack;
        auto& pool = slowCaseStorage->pool;
        auto& registrations = slowCaseStorage->registrations;
        if (status == WebCoreSwiftCSSTokenizerNeedBlockStack) {
            size_t capacity = state[WebCoreSwiftCSSTokenizerBlockStackCapacity];
            if (blockStack.isEmpty()) {
                blockStack.grow(2 * capacity);
                memcpySpan(blockStack.mutableSpan(), std::span { inlineBlockStack });
            } else
                blockStack.grow(2 * capacity);
            state[WebCoreSwiftCSSTokenizerBlockStack] = std::bit_cast<uintptr_t>(blockStack.mutableSpan().data());
            state[WebCoreSwiftCSSTokenizerBlockStackCapacity] = blockStack.size();
        } else {
            RELEASE_ASSERT(status == WebCoreSwiftCSSTokenizerNeedPool);
            // The pool never needs more code units than the input has; only the registrations grow.
            RELEASE_ASSERT(pool.isEmpty() || registrations.size() - registrationCount < 2);
            if (pool.isEmpty()) {
                pool.grow(input.length() + 2);
                state[WebCoreSwiftCSSTokenizerPool] = std::bit_cast<uintptr_t>(pool.mutableSpan().data());
                state[WebCoreSwiftCSSTokenizerPoolCapacity] = pool.size();
            }
            if (registrations.size() - registrationCount < 2) {
                registrations.grow(std::max<size_t>(8, 2 * registrations.size()));
                state[WebCoreSwiftCSSTokenizerRegistrations] = std::bit_cast<uintptr_t>(registrations.mutableSpan().data());
                state[WebCoreSwiftCSSTokenizerRegistrationCapacity] = registrations.size();
            }
        }
    }

    if (wrapper) {
        unsigned offset = static_cast<unsigned>(state[WebCoreSwiftCSSTokenizerPosition]);
        if (hasPendingComment)
            wrapper->addComment(pendingCommentStart, offset, pendingCommentTokensBefore);
        wrapper->addToken(offset);
        wrapper->finalizeConstruction(m_tokenStorage.data());
    }
}

CSSParserTokenRange CSSTokenizer::tokenRange() const LIFETIME_BOUND
{
    return tokens();
}

unsigned CSSTokenizer::tokenCount() const
{
    return m_tokenCount;
}

bool CSSTokenizer::isWhitespace(CSSParserTokenType type)
{
    return type == NonNewlineWhitespaceToken || type == NewlineToken;
}

} // namespace WebCore
