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

// C entry point for SVGTransformLexer, declared in SVGTransformSwiftBoundary.h. The only unsafe code
// in this unit: wrapping the C++ pointers into spans and storing the cursor advance.

// `characters`/`length`: the Latin-1 characters remaining in the C++ StringParsingBuffer.
// `results`: `capacity` item results. `values`: 6 * `capacity` floats. `consumed`: receives the cursor advance.
@_cdecl("WebCoreSwift_SVGTransform_lexItems8")
func WebCoreSwift_SVGTransform_lexItems8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ results: UnsafeMutablePointer<UInt32>, _ values: UnsafeMutablePointer<Float>, _ capacity: Int, _ consumed: UnsafeMutablePointer<Int>) -> Int {
    let input = unsafe UnsafeBufferPointer(start: characters, count: length)
    let resultBuffer = unsafe UnsafeMutableBufferPointer(start: results, count: capacity)
    let valueBuffer = unsafe UnsafeMutableBufferPointer(start: values, count: capacity * SVGTransformLexer.maximumValueCount)
    var resultSpan = unsafe resultBuffer.mutableSpan
    var valueSpan = unsafe valueBuffer.mutableSpan
    let lexed = unsafe SVGTransformLexer.lexItems(input.span, &resultSpan, &valueSpan)
    unsafe consumed.pointee = lexed.end
    return lexed.count
}
