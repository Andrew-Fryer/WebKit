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

// C entry points for SVGNumberGrammar, declared in SVGNumberGrammarSwiftBoundary.h. The only unsafe code in this
// unit: wrapping the C++ pointers into spans. Pointers may be null only when their count is 0.

// `characters`/`length`: the code units (Latin-1, or UTF-16 at any alignment if `is16Bit`) remaining in the C++
// StringParsingBuffer. `values`: `valueCapacity` floats, at least the program's item count.
@_cdecl("WebCoreSwift_SVGNumberGrammar_parseNumbers")
func WebCoreSwift_SVGNumberGrammar_parseNumbers(_ characters: UnsafeRawPointer?, _ length: Int, _ is16Bit: Bool, _ program: UInt32, _ values: UnsafeMutablePointer<Float>?, _ valueCapacity: Int) -> UInt64 {
    var valueSpan = unsafe UnsafeMutableBufferPointer(start: values, count: valueCapacity).mutableSpan
    if !is16Bit {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        return SVGNumberGrammar.scan(SVGLatin1Characters(units), program: program, &valueSpan)
    }
    if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        return SVGNumberGrammar.scan(SVGUTF16Characters(units), program: program, &valueSpan)
    }
    let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
    return SVGNumberGrammar.scan(SVGUnalignedUTF16Characters(bytes), program: program, &valueSpan)
}

// One number (the common case: number lists, single-number attributes, H/V path segments), without a values span.
@_cdecl("WebCoreSwift_SVGNumberGrammar_parseNumber")
func WebCoreSwift_SVGNumberGrammar_parseNumber(_ characters: UnsafeRawPointer?, _ length: Int, _ is16Bit: Bool, _ skipSuffix: Bool, _ value: UnsafeMutablePointer<Float>) -> UInt64 {
    var index = 0
    let number: Float?
    if !is16Bit {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        number = SVGNumberGrammar.parseNumber(SVGLatin1Characters(units), &index, skipSuffix: skipSuffix)
    } else if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        number = SVGNumberGrammar.parseNumber(SVGUTF16Characters(units), &index, skipSuffix: skipSuffix)
    } else {
        let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
        number = SVGNumberGrammar.parseNumber(SVGUnalignedUTF16Characters(bytes), &index, skipSuffix: skipSuffix)
    }
    guard let number else {
        return UInt64(truncatingIfNeeded: index) &<< 1
    }
    unsafe value.pointee = number
    return UInt64(truncatingIfNeeded: index) &<< 1 | 1
}

// `kind`: see SVGNumberGrammar.scanListItem. Writes the item's length to `itemLength`; returns the advance.
@_cdecl("WebCoreSwift_SVGNumberGrammar_scanListItem")
func WebCoreSwift_SVGNumberGrammar_scanListItem(_ characters: UnsafeRawPointer?, _ length: Int, _ is16Bit: Bool, _ kind: UInt32, _ itemLength: UnsafeMutablePointer<Int>) -> Int {
    let item: (length: Int, advance: Int)
    if !is16Bit {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        item = SVGNumberGrammar.scanListItem(SVGLatin1Characters(units), kind: kind)
    } else if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        item = SVGNumberGrammar.scanListItem(SVGUTF16Characters(units), kind: kind)
    } else {
        let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
        item = SVGNumberGrammar.scanListItem(SVGUnalignedUTF16Characters(bytes), kind: kind)
    }
    unsafe itemLength.pointee = item.length
    return item.advance
}
