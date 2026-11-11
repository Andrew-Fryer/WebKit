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

// C entry points of HTMLScan (declared to C++ in HTMLScanSwiftBoundary.h). This is the only file of the unit with
// `unsafe`: each such line wraps a C pointer and length in a span (the parser input as Latin-1 bytes, as UTF-16 code
// units at an even address or as raw bytes at an odd one; the glue's start tag scan buffer) and calls the safe core in
// HTMLScanTokenizer.swift / HTMLScanFastPath.swift. Input pointers may be null only when the length is 0.

@_cdecl("WebCoreSwift_HTMLScan_plainTextLength8")
@inline(__always)
func WebCoreSwift_HTMLScan_plainTextLength8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ stopCharacter: UInt8) -> Int {
    unsafe HTMLScan.plainTextLength(UnsafeBufferPointer(start: characters, count: length).span, stopCharacter: stopCharacter)
}

@_cdecl("WebCoreSwift_HTMLScan_fastPathScanText")
func WebCoreSwift_HTMLScan_fastPathScanText(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ textLengthLimit: Int) -> UInt64 {
    if is8Bit {
        return unsafe HTMLScan.scanText(HTMLScanLatin1Input(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span), textLengthLimit: textLengthLimit)
    }
    if Int(bitPattern: characters) & 1 == 0 {
        return unsafe HTMLScan.scanText(HTMLScanUTF16Input(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span), textLengthLimit: textLengthLimit)
    }
    return unsafe HTMLScan.scanText(HTMLScanUnalignedUTF16Input(UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes), textLengthLimit: textLengthLimit)
}

@_cdecl("WebCoreSwift_HTMLScan_fastPathScanStartTag")
func WebCoreSwift_HTMLScan_fastPathScanStartTag(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ scanTagName: Bool, _ startTag: UnsafeMutableRawPointer) -> UInt64 {
    let startTagBuffer = unsafe UnsafeMutableRawBufferPointer(start: startTag, count: HTMLScan.startTagByteCount)
    var scan = unsafe startTagBuffer.mutableBytes
    if is8Bit {
        return unsafe HTMLScan.scanStartTag(HTMLScanLatin1Input(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span), scanTagName: scanTagName, startTag: &scan)
    }
    if Int(bitPattern: characters) & 1 == 0 {
        return unsafe HTMLScan.scanStartTag(HTMLScanUTF16Input(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span), scanTagName: scanTagName, startTag: &scan)
    }
    return unsafe HTMLScan.scanStartTag(HTMLScanUnalignedUTF16Input(UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes), scanTagName: scanTagName, startTag: &scan)
}
