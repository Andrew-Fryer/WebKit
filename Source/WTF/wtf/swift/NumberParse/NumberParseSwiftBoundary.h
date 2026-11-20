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

// C boundary of the NumberParse Swift unit (Source/WTF/wtf/swift/NumberParse, module WTFSwift): decimal string
// to double conversion (a port of fast_float 8.2.10 as WTF uses it). Called only by wtf/FastFloat.cpp and
// wtf/text/WTFString.cpp. The symbols are internal to libWTF.a (hidden): only WTF .cpp files may call them,
// never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Every function returns exactly what the C++ it replaced returned (fast_float 8.2.10's from_chars plus the WTF
// wrappers), including the parsed length, the sign of zero and the out-of-range results (0 or infinity).
//
// WTFSwift_NumberParse_parse8/16 modes (fast_float::from_chars formats):
//   0                                  parseDouble: chars_format::general | no_infnan | allow_leading_plus
//   WTFSwiftNumberParseModeFixed       parseFixedDouble: chars_format::fixed | no_infnan
//   WTFSwiftNumberParseModeJSON        parseJSONDouble: chars_format::json; *parsedLength = SIZE_MAX for std::nullopt
// WTFSwift_NumberParse_charactersTo8/16 modes (WTFString.cpp's toDoubleType): 0 (parseDouble) or
// WTFSwiftNumberParseModeFixed (parseFixedDouble), plus
//   WTFSwiftNumberParseModeSkipWhitespace       skip leading isUnicodeCompatibleASCIIWhitespace first
//   WTFSwiftNumberParseModeAllowTrailingJunk    *ok does not require every character to be parsed
//   WTFSwiftNumberParseModeFloat                round to float; clear *ok when a finite double overflows float
//                                               (doubleToFloatCheckingOverflow)
// When nothing parses they return 0 with *parsedLength = 0 and *ok = false; otherwise *parsedLength counts the
// skipped whitespace. For charactersTo8/16 parsedLength and ok may be null; for the other functions parsedLength
// must not be. UTF-16 characters must be 2-byte aligned, like any char16_t*.
enum : uint32_t {
    WTFSwiftNumberParseModeFixed = 1,
    WTFSwiftNumberParseModeJSON = 2,
    WTFSwiftNumberParseModeSkipWhitespace = 4,
    WTFSwiftNumberParseModeAllowTrailingJunk = 8,
    WTFSwiftNumberParseModeFloat = 16,
};

// parseDouble(std::span<const Latin1Character>, size_t&): returns the value and stores the parsed length.
double WTFSwift_NumberParse_parseDouble8(const uint8_t* characters, size_t length, size_t* parsedLength);

// parseJSONDouble(std::span<const Latin1Character>, size_t&): *parsedLength = SIZE_MAX for std::nullopt.
double WTFSwift_NumberParse_parseJSONDouble8(const uint8_t* characters, size_t length, size_t* parsedLength);

double WTFSwift_NumberParse_parse8(const uint8_t* characters, size_t length, uint32_t mode, size_t* parsedLength);
double WTFSwift_NumberParse_parse16(const char16_t* characters, size_t length, uint32_t mode, size_t* parsedLength);

// charactersToDouble(std::span<const Latin1Character>, bool*): toDoubleType with WTFSwiftNumberParseModeSkipWhitespace.
double WTFSwift_NumberParse_charactersToDouble8(const uint8_t* characters, size_t length, bool* ok);

double WTFSwift_NumberParse_charactersTo8(const uint8_t* characters, size_t length, uint32_t mode, size_t* parsedLength, bool* ok);
double WTFSwift_NumberParse_charactersTo16(const char16_t* characters, size_t length, uint32_t mode, size_t* parsedLength, bool* ok);

// parseJSONDoubleFromDigitsSlow: mantissa * 10^exponent for a mantissa of at most 19 digits (traps otherwise,
// like the C++ RELEASE_ASSERT).
double WTFSwift_NumberParse_doubleFromDigits(uint64_t mantissa, int64_t exponent);

#ifdef __cplusplus
}
#endif
