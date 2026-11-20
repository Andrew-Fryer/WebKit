// Copyright 2014 The Chromium Authors. All rights reserved.
// Copyright (C) 2016-2021 Apple Inc. All rights reserved.
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
#include "CSSParserToken.h"

#include "CSSMarkup.h"
#include "CSSPropertyParser.h"
#include "CSSTokenizerSwiftBoundary.h"
#include <wtf/HexNumber.h>
#include <wtf/NeverDestroyed.h>
#include <wtf/text/StringBuilder.h>

namespace WebCore {
DEFINE_ALLOCATOR_WITH_HEAP_IDENTIFIER(CSSParserToken);

// The unit names are matched in Swift (swift/CSSTokenizer/CSSTokenizerUnits.swift), which returns CSSUnitType raw values.
static_assert(std::to_underlying(CSSUnitType::Unknown) == 0);
static_assert(std::to_underlying(CSSUnitType::Number) == 1);
static_assert(std::to_underlying(CSSUnitType::Percentage) == 3);
static_assert(std::to_underlying(CSSUnitType::Em) == 4);
static_assert(std::to_underlying(CSSUnitType::Ex) == 5);
static_assert(std::to_underlying(CSSUnitType::Px) == 6);
static_assert(std::to_underlying(CSSUnitType::Cm) == 7);
static_assert(std::to_underlying(CSSUnitType::Mm) == 8);
static_assert(std::to_underlying(CSSUnitType::In) == 9);
static_assert(std::to_underlying(CSSUnitType::Pt) == 10);
static_assert(std::to_underlying(CSSUnitType::Pc) == 11);
static_assert(std::to_underlying(CSSUnitType::Deg) == 12);
static_assert(std::to_underlying(CSSUnitType::Rad) == 13);
static_assert(std::to_underlying(CSSUnitType::Grad) == 14);
static_assert(std::to_underlying(CSSUnitType::Ms) == 15);
static_assert(std::to_underlying(CSSUnitType::S) == 16);
static_assert(std::to_underlying(CSSUnitType::Hz) == 17);
static_assert(std::to_underlying(CSSUnitType::Khz) == 18);
static_assert(std::to_underlying(CSSUnitType::Vw) == 19);
static_assert(std::to_underlying(CSSUnitType::Vh) == 20);
static_assert(std::to_underlying(CSSUnitType::Vmin) == 21);
static_assert(std::to_underlying(CSSUnitType::Vmax) == 22);
static_assert(std::to_underlying(CSSUnitType::Vb) == 23);
static_assert(std::to_underlying(CSSUnitType::Vi) == 24);
static_assert(std::to_underlying(CSSUnitType::Svw) == 25);
static_assert(std::to_underlying(CSSUnitType::Svh) == 26);
static_assert(std::to_underlying(CSSUnitType::Svmin) == 27);
static_assert(std::to_underlying(CSSUnitType::Svmax) == 28);
static_assert(std::to_underlying(CSSUnitType::Svb) == 29);
static_assert(std::to_underlying(CSSUnitType::Svi) == 30);
static_assert(std::to_underlying(CSSUnitType::Lvw) == 31);
static_assert(std::to_underlying(CSSUnitType::Lvh) == 32);
static_assert(std::to_underlying(CSSUnitType::Lvmin) == 33);
static_assert(std::to_underlying(CSSUnitType::Lvmax) == 34);
static_assert(std::to_underlying(CSSUnitType::Lvb) == 35);
static_assert(std::to_underlying(CSSUnitType::Lvi) == 36);
static_assert(std::to_underlying(CSSUnitType::Dvw) == 37);
static_assert(std::to_underlying(CSSUnitType::Dvh) == 38);
static_assert(std::to_underlying(CSSUnitType::Dvmin) == 39);
static_assert(std::to_underlying(CSSUnitType::Dvmax) == 40);
static_assert(std::to_underlying(CSSUnitType::Dvb) == 41);
static_assert(std::to_underlying(CSSUnitType::Dvi) == 42);
static_assert(std::to_underlying(CSSUnitType::Cqw) == 43);
static_assert(std::to_underlying(CSSUnitType::Cqh) == 44);
static_assert(std::to_underlying(CSSUnitType::Cqi) == 45);
static_assert(std::to_underlying(CSSUnitType::Cqb) == 46);
static_assert(std::to_underlying(CSSUnitType::Cqmin) == 47);
static_assert(std::to_underlying(CSSUnitType::Cqmax) == 48);
static_assert(std::to_underlying(CSSUnitType::Dppx) == 49);
static_assert(std::to_underlying(CSSUnitType::X) == 50);
static_assert(std::to_underlying(CSSUnitType::Dpi) == 51);
static_assert(std::to_underlying(CSSUnitType::Dpcm) == 52);
static_assert(std::to_underlying(CSSUnitType::Fr) == 53);
static_assert(std::to_underlying(CSSUnitType::Q) == 54);
static_assert(std::to_underlying(CSSUnitType::Lh) == 55);
static_assert(std::to_underlying(CSSUnitType::Rlh) == 56);
static_assert(std::to_underlying(CSSUnitType::Turn) == 57);
static_assert(std::to_underlying(CSSUnitType::Rem) == 58);
static_assert(std::to_underlying(CSSUnitType::Rex) == 59);
static_assert(std::to_underlying(CSSUnitType::Cap) == 60);
static_assert(std::to_underlying(CSSUnitType::Rcap) == 61);
static_assert(std::to_underlying(CSSUnitType::Ch) == 62);
static_assert(std::to_underlying(CSSUnitType::Rch) == 63);
static_assert(std::to_underlying(CSSUnitType::Ic) == 64);
static_assert(std::to_underlying(CSSUnitType::Ric) == 65);
static_assert(std::to_underlying(CSSUnitType::QuirkyEm) == 69);

CSSUnitType CSSParserToken::stringToUnitType(StringView stringView)
{
    return static_cast<CSSUnitType>(WebCoreSwift_CSSTokenizer_unitType(stringView.rawCharacters(), stringView.length(), stringView.is8Bit()));
}

CSSParserToken::CSSParserToken(CSSParserTokenType type, BlockType blockType)
    : m_type(type)
    , m_blockType(blockType)
{
}

CSSParserToken::CSSParserToken(CSSParserTokenType type, StringView value, BlockType blockType)
    : m_type(type)
    , m_blockType(blockType)
    , m_id(-1)
{
    initValueFromStringView(value);
}

CSSParserToken::CSSParserToken(double numericValue, NumericValueType numericValueType, NumericSign sign, StringView originalText)
    : m_type(NumberToken)
    , m_blockType(NotBlock)
    , m_numericValueType(numericValueType)
    , m_numericSign(sign)
    , m_unit(static_cast<unsigned>(CSSUnitType::Number))
    , m_numericValue(numericValue)
{
    initValueFromStringView(originalText);
}

NODELETE static StringView mergeIfAdjacent(StringView a, StringView b)
{
    if (a.is8Bit() && b.is8Bit()) {
        auto characters = a.span8();
        if (std::to_address(characters.end()) == std::to_address(b.span8().begin()))
            return unsafeMakeSpan(characters.data(), a.length() + b.length());
    } else if (!a.is8Bit() && !b.is8Bit()) {
        auto characters = a.span16();
        if (std::to_address(characters.end()) == std::to_address(b.span16().begin()))
            return unsafeMakeSpan(characters.data(), a.length() + b.length());
    }
    return { };
}

void CSSParserToken::convertToDimensionWithUnit(CSSUnitType unit)
{
    ASSERT(m_type == NumberToken);
    auto originalNumberText = originalText();
    auto originalNumberTextLength = originalNumberText.length();
    auto unitString = unitTypeString(unit);
    auto string = StringView { unitString };
    if (originalNumberTextLength && originalNumberTextLength < 16) {
        if (auto merged = mergeIfAdjacent(originalNumberText, unitString))
            string = merged;
    }
    m_type = DimensionToken;
    m_unit = static_cast<unsigned>(unit);
    m_nonUnitPrefixLength = string == unitString ? 0 : originalNumberTextLength;
    initValueFromStringView(string);
}

void CSSParserToken::convertToPercentage()
{
    ASSERT(m_type == NumberToken);
    m_type = PercentageToken;
    m_unit = static_cast<unsigned>(CSSUnitType::Percentage);
}

StringView CSSParserToken::originalText() const
{
    ASSERT(m_type == NumberToken || m_type == DimensionToken || m_type == PercentageToken);
    return value();
}

StringView CSSParserToken::unitString() const
{
    ASSERT(m_type == DimensionToken);
    return value().substring(m_nonUnitPrefixLength);
}

char16_t CSSParserToken::delimiter() const
{
    ASSERT(m_type == DelimiterToken);
    return m_delimiter;
}

NumericSign CSSParserToken::numericSign() const
{
    // This is valid for DimensionToken and PercentageToken, but only used
    // in <an+b> parsing on NumberTokens.
    ASSERT(m_type == NumberToken);
    return static_cast<NumericSign>(m_numericSign);
}

NumericValueType CSSParserToken::numericValueType() const
{
    ASSERT(m_type == NumberToken || m_type == PercentageToken || m_type == DimensionToken);
    return static_cast<NumericValueType>(m_numericValueType);
}

double CSSParserToken::numericValue() const
{
    ASSERT(m_type == NumberToken || m_type == PercentageToken || m_type == DimensionToken);
    return m_numericValue;
}

CSSPropertyID CSSParserToken::parseAsCSSPropertyID() const
{
    ASSERT(m_type == IdentToken);
    return cssPropertyID(value());
}

CSSValueID CSSParserToken::id() const
{
    if (m_type != IdentToken)
        return CSSValueID::Invalid;
    return identOrFunctionId();
}

CSSValueID CSSParserToken::functionId() const
{
    if (m_type != FunctionToken)
        return CSSValueID::Invalid;
    return identOrFunctionId();
}

CSSValueID CSSParserToken::identOrFunctionId() const
{
    ASSERT(m_type == IdentToken || m_type == FunctionToken);
    if (m_id < 0)
        m_id = std::to_underlying(cssValueKeywordID(value()));
    return static_cast<CSSValueID>(m_id);
}

bool CSSParserToken::hasStringBacking() const
{
    switch (type()) {
    case AtKeywordToken:
    case DimensionToken:
    case FunctionToken:
    case HashToken:
    case IdentToken:
    case NumberToken:
    case PercentageToken:
    case StringToken:
    case UrlToken:
        return true;
    case BadStringToken:
    case BadUrlToken:
    case CDCToken:
    case CDOToken:
    case ColonToken:
    case ColumnToken:
    case CommaToken:
    case CommentToken:
    case DashMatchToken:
    case DelimiterToken:
    case EOFToken:
    case IncludeMatchToken:
    case LeftBraceToken:
    case LeftBracketToken:
    case LeftParenthesisToken:
    case NewlineToken:
    case NonNewlineWhitespaceToken:
    case PrefixMatchToken:
    case RightBraceToken:
    case RightBracketToken:
    case RightParenthesisToken:
    case SemicolonToken:
    case SubstringMatchToken:
    case SuffixMatchToken:
        return false;
    }
    ASSERT_NOT_REACHED();
    return false;
}

bool CSSParserToken::tryUseStringLiteralBacking()
{
    if (m_type != IdentToken && m_type != FunctionToken)
        return false;

    if (!m_isBackedByStringLiteral) {
        auto valueId = identOrFunctionId();
        if (valueId == CSSValueID::Invalid)
            return false;

        auto literal = nameLiteral(valueId);

        // Typically all lowercase but we need to keep the original for correct serialization if they differ.
        if (value() != literal)
            return false;

        updateCharacters(literal.span8());

        m_isBackedByStringLiteral = true;
    }
    return true;
}

bool CSSParserToken::operator==(const CSSParserToken& other) const
{
    if (m_type != other.m_type)
        return false;
    switch (m_type) {
    case DelimiterToken:
        return delimiter() == other.delimiter();
    case HashToken:
        if (m_hashTokenType != other.m_hashTokenType)
            return false;
        [[fallthrough]];
    case IdentToken:
    case FunctionToken:
    case StringToken:
    case UrlToken:
        return value() == other.value();
    case DimensionToken:
        if (!m_nonUnitPrefixLength || !other.m_nonUnitPrefixLength) {
            // The spec wants equality comparison of the original text but in some rare dimension cases we don't have it. Fall back to parsed values.
            // This must be decided by both tokens so that the comparison is symmetric.
            if (unitString() != other.unitString())
                return false;
            return m_numericSign == other.m_numericSign && m_numericValue == other.m_numericValue && m_numericValueType == other.m_numericValueType;
        }
        [[fallthrough]];
    case NumberToken:
    case PercentageToken:
        return originalText() == other.originalText();
    case NonNewlineWhitespaceToken:
        return m_whitespaceCount == other.m_whitespaceCount;
    default:
        return true;
    }
}

struct NextTokenNeedsCommentBuilder {
    constexpr NextTokenNeedsCommentBuilder(std::initializer_list<CSSParserTokenType> tokens)
    {
        for (auto token : tokens)
            buffer[token] = true;
    }

    std::array<bool, numberOfCSSParserTokenTypes> buffer { false };
};

void CSSParserToken::serialize(StringBuilder& builder, const CSSParserToken* nextToken, SerializationMode mode) const
{
    // This is currently only used for @supports CSSOM. To keep our implementation
    // simple we handle some of the edge cases incorrectly (see comments below).
    auto appendCommentIfNeeded = [&] (const NextTokenNeedsCommentBuilder& tokensNeedingComment, auto... delimitersNeedingComment) {
        if (!nextToken)
            return;

        CSSParserTokenType nextType = nextToken->type();
        if (tokensNeedingComment.buffer[nextType]) {
            builder.append("/**/"_s);
            return;
        }

        if (nextType == DelimiterToken && ((delimitersNeedingComment == nextToken->delimiter()) || ... || false)) {
            builder.append("/**/"_s);
            return;
        }
    };

    switch (type()) {
    case IdentToken:
        serializeIdentifier(builder, value());
        appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken, CDCToken, LeftParenthesisToken }, '-');
        break;
    case FunctionToken:
        serializeIdentifier(builder, value());
        builder.append('(');
        break;
    case AtKeywordToken:
        builder.append('@');
        serializeIdentifier(builder, value());
        appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken, CDCToken }, '-');
        break;
    case HashToken:
        builder.append('#');
        serializeIdentifier(builder, value(), (getHashTokenType() == HashTokenUnrestricted) ? ShouldSkipStartChecks::Yes : ShouldSkipStartChecks::No);
        appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken, CDCToken }, '-');
        break;
    case UrlToken:
        builder.append("url("_s);
        serializeURLTokenValue(builder, value());
        builder.append(')');
        break;
    case DelimiterToken:
        switch (delimiter()) {
        case '\\':
            builder.append("\\\n"_s);
            break;

        case '#':
        case '-':
            builder.append(delimiter());
            appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken }, '-');
            break;

        case '@':
            builder.append('@');
            appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken }, '-');
            break;

        case '.':
        case '+':
            builder.append(delimiter());
            appendCommentIfNeeded({ NumberToken, PercentageToken, DimensionToken });
            break;

        case '/':
            builder.append('/');
            // Weirdly Clang errors if you try to use the fold expression in buildNextTokenNeedsCommentTable() because the true value is unused.
            // So we just build the table by hand here instead. See: rdar://69710661
            appendCommentIfNeeded({ }, '*');
            break;

        default:
            builder.append(delimiter());
            break;
        }
        break;
    case NumberToken:
        if (mode == SerializationMode::CustomProperty)
            builder.append(originalText());
        else {
            if (m_numericSign == PlusSign)
                builder.append('+');
            builder.append(numericValue());
        }
        appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken }, '%');
        break;
    case PercentageToken:
        if (mode == SerializationMode::CustomProperty)
            builder.append(originalText(), '%');
        else
            builder.append(numericValue(), '%');
        break;
    case DimensionToken:
        if (mode == SerializationMode::CustomProperty && m_nonUnitPrefixLength)
            builder.append(originalText());
        else {
            builder.append(numericValue());
            serializeIdentifier(builder, unitString());
        }
        appendCommentIfNeeded({ IdentToken, FunctionToken, UrlToken, BadUrlToken, NumberToken, PercentageToken, DimensionToken, CDCToken }, '-');
        break;
    case StringToken:
        serializeString(builder, value());
        break;

    case IncludeMatchToken:
        builder.append("~="_s);
        break;
    case DashMatchToken:
        builder.append("|="_s);
        break;
    case PrefixMatchToken:
        builder.append("^="_s);
        break;
    case SuffixMatchToken:
        builder.append("$="_s);
        break;
    case SubstringMatchToken:
        builder.append("*="_s);
        break;
    case ColumnToken:
        builder.append("||"_s);
        break;
    case CDOToken:
        builder.append("<!--"_s);
        break;
    case CDCToken:
        builder.append("-->"_s);
        break;
    case BadStringToken:
        builder.append("'\n"_s);
        break;
    case BadUrlToken:
        builder.append("url(()"_s);
        break;
    case NonNewlineWhitespaceToken: {
        auto count = mode == SerializationMode::CustomProperty ? m_whitespaceCount : 1;
        for (decltype(count) i = 0; i < count; ++i)
            builder.append(' ');
        break;
    }
    case NewlineToken:
        builder.append(mode == SerializationMode::CustomProperty ? '\n' : ' ');
        break;
    case ColonToken:
        builder.append(':');
        break;
    case SemicolonToken:
        builder.append(';');
        break;
    case CommaToken:
        builder.append(',');
        break;
    case LeftParenthesisToken:
        builder.append('(');
        break;
    case RightParenthesisToken:
        builder.append(')');
        break;
    case LeftBracketToken:
        builder.append('[');
        break;
    case RightBracketToken:
        builder.append(']');
        break;
    case LeftBraceToken:
        builder.append('{');
        break;
    case RightBraceToken:
        builder.append('}');
        break;

    case EOFToken:
    case CommentToken:
        ASSERT_NOT_REACHED();
        break;
    }
}

} // namespace WebCore
