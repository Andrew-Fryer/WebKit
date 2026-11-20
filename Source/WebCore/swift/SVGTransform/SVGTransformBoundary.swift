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

// C entry point for SVGTransformLexer, declared in SVGTransformSwiftBoundary.h. The only unsafe code in this unit:
// wrapping the C++ pointers into spans. Pointers may be null only when their count is 0.

// `characters`/`length`: the code units (Latin-1, or UTF-16 at any alignment if `is16Bit`) remaining in the C++
// StringParsingBuffer. `results`: `resultCapacity` item results. `values`: `valueCapacity` floats, 6 per result.
@_cdecl("WebCoreSwift_SVGTransform_lexItems")
func WebCoreSwift_SVGTransform_lexItems(_ characters: UnsafeRawPointer?, _ length: Int, _ is16Bit: Bool, _ results: UnsafeMutablePointer<UInt32>?, _ resultCapacity: Int, _ values: UnsafeMutablePointer<Float>?, _ valueCapacity: Int) -> UInt64 {
    var resultSpan = unsafe UnsafeMutableBufferPointer(start: results, count: resultCapacity).mutableSpan
    var valueSpan = unsafe UnsafeMutableBufferPointer(start: values, count: valueCapacity).mutableSpan
    if !is16Bit {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        return SVGTransformLexer.lex(SVGLatin1Characters(units), &resultSpan, &valueSpan)
    }
    if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        return SVGTransformLexer.lex(SVGUTF16Characters(units), &resultSpan, &valueSpan)
    }
    let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
    return SVGTransformLexer.lex(SVGUnalignedUTF16Characters(bytes), &resultSpan, &valueSpan)
}
