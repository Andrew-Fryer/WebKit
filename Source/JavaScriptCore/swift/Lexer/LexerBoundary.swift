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

// The C entry point of the lexer scans (LexerScan.swift), declared to C++ in LexerSwiftBoundary.h and called only from
// parser/Lexer.cpp. The only unsafe operations wrap the source, the glue's two scratch buffers and the four result
// words in spans. A pointer may be null only when its count is 0. UTF-16 text at an odd address (possible: a String
// can point into a buffer at any alignment) is read through a RawSpan, since a Span<UInt16> cannot view it.

@_cdecl("JSCSwift_Lexer_scan")
func JSCSwift_Lexer_scan(_ mode: UInt32, _ source: UnsafeRawPointer?, _ length: Int, _ cooked: UnsafeMutablePointer<UInt16>?, _ cookedCapacity: Int,
                         _ raw: UnsafeMutablePointer<UInt16>?, _ rawCapacity: Int, _ results: UnsafeMutablePointer<Int>) {
    var cookedSpan = unsafe UnsafeMutableBufferPointer(start: cooked, count: cookedCapacity).mutableSpan
    var rawSpan = unsafe UnsafeMutableBufferPointer(start: raw, count: rawCapacity).mutableSpan
    var resultSpan = unsafe UnsafeMutableBufferPointer(start: results, count: LexerScan.resultCount).mutableSpan
    if mode & LexerScan.wideFlag == 0 {
        let latin1 = unsafe UnsafeBufferPointer(start: source?.assumingMemoryBound(to: UInt8.self), count: length).span
        LexerScan.scan(mode, Latin1Characters(latin1), &cookedSpan, &rawSpan, &resultSpan)
    } else if UInt(bitPattern: source) & 1 == 0 {
        let utf16 = unsafe UnsafeBufferPointer(start: source?.assumingMemoryBound(to: UInt16.self), count: length).span
        LexerScan.scan(mode, UTF16Characters(utf16), &cookedSpan, &rawSpan, &resultSpan)
    } else {
        let bytes = unsafe UnsafeRawBufferPointer(start: source, count: 2 * length).bytes
        LexerScan.scan(mode, UnalignedUTF16Characters(bytes), &cookedSpan, &rawSpan, &resultSpan)
    }
}
