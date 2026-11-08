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

// C entry points of LexerEscapedLiterals.swift, declared to C++ in LexerEscapedLiteralsSwiftBoundary.h and called only
// from parser/Lexer.cpp (parseStringSlowCase, scanTemplateString). The only unsafe operations are wrapping the source
// and the glue's scratch buffers in spans and writing the four result words. A pointer may be null only when its
// count is 0; UTF-16 sources are 2-byte aligned (the glue checks).

@inline(__always)
func storeLiteralResult(_ result: LexerEscapedLiterals.Result, _ results: UnsafeMutablePointer<Int>) {
    var flags = Int(result.status) | (Int(result.message) &<< 8)
    if result.cookedFailed {
        flags |= 1 &<< 16
    }
    if result.isTail {
        flags |= 1 &<< 17
    }
    unsafe results[0] = flags
    unsafe results[1] = result.end
    unsafe results[2] = result.cookedLength
    unsafe results[3] = result.rawLength
}

@_cdecl("JSCSwift_LexerEscapedLiterals_string8")
func JSCSwift_LexerEscapedLiterals_string8(_ source: UnsafePointer<UInt8>?, _ length: Int, _ build: Bool, _ strictMode: Bool, _ cooked: UnsafeMutablePointer<UInt16>?, _ cookedCapacity: Int, _ results: UnsafeMutablePointer<Int>) {
    let sourceSpan = unsafe UnsafeBufferPointer(start: source, count: length).span
    var cookedSpan = unsafe UnsafeMutableBufferPointer(start: cooked, count: cookedCapacity).mutableSpan
    let result = build ? LexerEscapedLiterals.scanString(sourceSpan, build: true, strictMode: strictMode, &cookedSpan)
        : LexerEscapedLiterals.scanString(sourceSpan, build: false, strictMode: strictMode, &cookedSpan)
    unsafe storeLiteralResult(result, results)
}

@_cdecl("JSCSwift_LexerEscapedLiterals_string16")
func JSCSwift_LexerEscapedLiterals_string16(_ source: UnsafePointer<UInt16>?, _ length: Int, _ build: Bool, _ strictMode: Bool, _ cooked: UnsafeMutablePointer<UInt16>?, _ cookedCapacity: Int, _ results: UnsafeMutablePointer<Int>) {
    let sourceSpan = unsafe UnsafeBufferPointer(start: source, count: length).span
    var cookedSpan = unsafe UnsafeMutableBufferPointer(start: cooked, count: cookedCapacity).mutableSpan
    let result = build ? LexerEscapedLiterals.scanString(sourceSpan, build: true, strictMode: strictMode, &cookedSpan)
        : LexerEscapedLiterals.scanString(sourceSpan, build: false, strictMode: strictMode, &cookedSpan)
    unsafe storeLiteralResult(result, results)
}

@_cdecl("JSCSwift_LexerEscapedLiterals_template8")
func JSCSwift_LexerEscapedLiterals_template8(_ source: UnsafePointer<UInt8>?, _ length: Int, _ buildRaw: Bool, _ cooked: UnsafeMutablePointer<UInt16>?, _ cookedCapacity: Int, _ raw: UnsafeMutablePointer<UInt16>?, _ rawCapacity: Int, _ results: UnsafeMutablePointer<Int>) {
    let sourceSpan = unsafe UnsafeBufferPointer(start: source, count: length).span
    var cookedSpan = unsafe UnsafeMutableBufferPointer(start: cooked, count: cookedCapacity).mutableSpan
    var rawSpan = unsafe UnsafeMutableBufferPointer(start: raw, count: rawCapacity).mutableSpan
    let result = buildRaw ? LexerEscapedLiterals.scanTemplate(sourceSpan, buildRaw: true, &cookedSpan, &rawSpan)
        : LexerEscapedLiterals.scanTemplate(sourceSpan, buildRaw: false, &cookedSpan, &rawSpan)
    unsafe storeLiteralResult(result, results)
}

@_cdecl("JSCSwift_LexerEscapedLiterals_template16")
func JSCSwift_LexerEscapedLiterals_template16(_ source: UnsafePointer<UInt16>?, _ length: Int, _ buildRaw: Bool, _ cooked: UnsafeMutablePointer<UInt16>?, _ cookedCapacity: Int, _ raw: UnsafeMutablePointer<UInt16>?, _ rawCapacity: Int, _ results: UnsafeMutablePointer<Int>) {
    let sourceSpan = unsafe UnsafeBufferPointer(start: source, count: length).span
    var cookedSpan = unsafe UnsafeMutableBufferPointer(start: cooked, count: cookedCapacity).mutableSpan
    var rawSpan = unsafe UnsafeMutableBufferPointer(start: raw, count: rawCapacity).mutableSpan
    let result = buildRaw ? LexerEscapedLiterals.scanTemplate(sourceSpan, buildRaw: true, &cookedSpan, &rawSpan)
        : LexerEscapedLiterals.scanTemplate(sourceSpan, buildRaw: false, &cookedSpan, &rawSpan)
    unsafe storeLiteralResult(result, results)
}
