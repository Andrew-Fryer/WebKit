/*
 * Copyright (C) 2022 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include <wtf/FastFloat.h>

#include "NumberParseSwiftBoundary.h"

// The parsing itself (fast_float's algorithm) is Swift: wtf/swift/NumberParse.

namespace WTF {

double parseDouble(std::span<const Latin1Character> string, size_t& parsedLength)
{
    return WTFSwift_NumberParse_parseDouble8(byteCast<uint8_t>(string).data(), string.size(), &parsedLength);
}

double parseDouble(std::span<const char16_t> string, size_t& parsedLength)
{
    return WTFSwift_NumberParse_parse16(string.data(), string.size(), 0, &parsedLength);
}

double parseFixedDouble(std::span<const Latin1Character> string, size_t& parsedLength)
{
    return WTFSwift_NumberParse_parse8(byteCast<uint8_t>(string).data(), string.size(), WTFSwiftNumberParseModeFixed, &parsedLength);
}

double parseFixedDouble(std::span<const char16_t> string, size_t& parsedLength)
{
    return WTFSwift_NumberParse_parse16(string.data(), string.size(), WTFSwiftNumberParseModeFixed, &parsedLength);
}

std::optional<double> parseJSONDouble(std::span<const Latin1Character> string, size_t& parsedLength)
{
    size_t length;
    double doubleValue = WTFSwift_NumberParse_parseJSONDouble8(byteCast<uint8_t>(string).data(), string.size(), &length);
    if (length == SIZE_MAX) [[unlikely]]
        return std::nullopt;
    parsedLength = length;
    return doubleValue;
}

std::optional<double> parseJSONDouble(std::span<const char16_t> string, size_t& parsedLength)
{
    size_t length;
    double doubleValue = WTFSwift_NumberParse_parse16(string.data(), string.size(), WTFSwiftNumberParseModeJSON, &length);
    if (length == SIZE_MAX) [[unlikely]]
        return std::nullopt;
    parsedLength = length;
    return doubleValue;
}

double parseJSONDoubleFromDigitsSlow(uint64_t mantissa, int64_t exponent)
{
    return WTFSwift_NumberParse_doubleFromDigits(mantissa, exponent);
}

} // namespace WTF
