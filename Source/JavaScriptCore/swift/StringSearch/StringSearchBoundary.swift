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

// C entry points for StringSearch.swift, declared to C++ in StringSearchSwiftBoundary.h and called only from
// runtime/VM.cpp. Subject and pattern arrive as a pointer and a length in code units plus a width bit each; they are
// read as raw bytes (a UTF-16 pointer need not be 2-byte aligned) and may be null only when their length is 0.
// `unsafe`: each such line wraps a C pointer and its length in a span and calls the safe core.

@inline(__always)
func stringSearchBytes(_ characters: UnsafeRawPointer?, _ length: Int, is8Bit: Bool) -> UnsafeRawBufferPointer {
    unsafe UnsafeRawBufferPointer(start: characters, count: is8Bit ? length : length &* 2)
}

@_cdecl("JSCSwift_StringSearch_find")
func JSCSwift_StringSearch_find(_ subject: UnsafeRawPointer?, _ subjectLength: Int, _ pattern: UnsafeRawPointer?, _ patternLength: Int, _ widths: UInt32, _ start: Int) -> Int {
    let subjectBytes = unsafe stringSearchBytes(subject, subjectLength, is8Bit: widths & 1 != 0).bytes
    let patternBytes = unsafe stringSearchBytes(pattern, patternLength, is8Bit: widths & 2 != 0).bytes
    switch widths & 3 {
    case 3:
        return StringSearch.find(StringSearchLatin1(subjectBytes), StringSearchLatin1(patternBytes), from: start)
    case 1:
        return StringSearch.find(StringSearchLatin1(subjectBytes), StringSearchUTF16(patternBytes), from: start)
    case 2:
        return StringSearch.find(StringSearchUTF16(subjectBytes), StringSearchLatin1(patternBytes), from: start)
    default:
        return StringSearch.find(StringSearchUTF16(subjectBytes), StringSearchUTF16(patternBytes), from: start)
    }
}

@_cdecl("JSCSwift_StringSearch_findAll")
func JSCSwift_StringSearch_findAll(_ subject: UnsafeRawPointer?, _ subjectLength: Int, _ pattern: UnsafeRawPointer?, _ patternLength: Int, _ widths: UInt32, _ start: Int, _ gaps: UnsafeMutablePointer<Int32>?, _ gapCapacity: Int, _ result: UnsafeMutablePointer<Int>?) {
    let subjectBytes = unsafe stringSearchBytes(subject, subjectLength, is8Bit: widths & 1 != 0).bytes
    let patternBytes = unsafe stringSearchBytes(pattern, patternLength, is8Bit: widths & 2 != 0).bytes
    var gapWords = unsafe UnsafeMutableBufferPointer(start: gaps, count: gapCapacity &* 2).mutableSpan
    var resultWords = unsafe UnsafeMutableBufferPointer(start: result, count: 4).mutableSpan
    let matches: StringSearchMatches
    switch widths & 3 {
    case 3:
        matches = StringSearch.findAll(StringSearchLatin1(subjectBytes), StringSearchLatin1(patternBytes), from: start, &gapWords)
    case 1:
        matches = StringSearch.findAll(StringSearchLatin1(subjectBytes), StringSearchUTF16(patternBytes), from: start, &gapWords)
    case 2:
        matches = StringSearch.findAll(StringSearchUTF16(subjectBytes), StringSearchLatin1(patternBytes), from: start, &gapWords)
    default:
        matches = StringSearch.findAll(StringSearchUTF16(subjectBytes), StringSearchUTF16(patternBytes), from: start, &gapWords)
    }
    resultWords[0] = matches.count
    resultWords[1] = matches.last
    resultWords[2] = matches.end
    resultWords[3] = matches.gapCount
}
