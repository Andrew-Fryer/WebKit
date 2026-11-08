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

// C entry points of LexerRegExp.swift, declared to C++ in LexerRegExpSwiftBoundary.h and called only from
// parser/Lexer.cpp (Lexer<T>::scanRegExp). The only unsafe operations are wrapping the source pointer and length in
// a Span and writing the scalar out-params. `characters` may be null only when `length` is 0.

@_cdecl("JSCSwift_LexerRegExp_scan8")
func JSCSwift_LexerRegExp_scan8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ bodyEnd: UnsafeMutablePointer<Int>, _ end: UnsafeMutablePointer<Int>) -> UInt32 {
    let source = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe LexerRegExp.scan(source.span)
    unsafe bodyEnd.pointee = result.bodyEnd
    unsafe end.pointee = result.end
    return result.status.rawValue
}

@_cdecl("JSCSwift_LexerRegExp_scan16")
func JSCSwift_LexerRegExp_scan16(_ characters: UnsafePointer<UInt16>?, _ length: Int, _ bodyEnd: UnsafeMutablePointer<Int>, _ end: UnsafeMutablePointer<Int>) -> UInt32 {
    let source = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe LexerRegExp.scan(source.span)
    unsafe bodyEnd.pointee = result.bodyEnd
    unsafe end.pointee = result.end
    return result.status.rawValue | (UInt32(result.charactersOredTogether) &<< 16)
}
