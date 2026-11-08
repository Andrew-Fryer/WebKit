// Copyright (C) 2026 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

// Memory-safe port of the batching scans of HTMLTokenizer::processToken (html/parser/HTMLTokenizer.cpp):
// findPlainTextInDataState and findPlainTextInQuotedAttributeValue<'"'> / <'\''>, i.e. std::ranges::find_if with
// isDataStateSpecialCharacter / isQuotedAttributeValueSpecialCharacter<quote> over 8-bit input.

extension HTMLScan {
    // The length of the leading run of `characters` free of `stopCharacter`, '&', '\r', '\n' and '\0'
    // (stopCharacter is '<' in the data state, the quote in the quoted attribute value states).
    @inline(__always)
    static func plainTextLength(_ characters: Span<UInt8>, stopCharacter: UInt8) -> Int {
        firstIndex(characters, from: 0, HTMLScanFiveCharacters(a: stopCharacter, b: 0x26, c: 0x0D, d: 0x0A, e: 0x00))
    }
}
