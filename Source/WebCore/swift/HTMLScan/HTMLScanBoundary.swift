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
// `unsafe`: it wraps the parser input (Latin-1 bytes, or UTF-16 code units as raw bytes because they may be
// odd-aligned) and the glue's output buffers in spans, writes scalar out-params, and calls the safe core in
// HTMLScanTokenizer.swift / HTMLScanFastPath.swift. Input pointers may be null only when the length is 0.

// MARK: HTMLTokenizer

@_cdecl("WebCoreSwift_HTMLScan_plainTextLength8")
@inline(__always)
func WebCoreSwift_HTMLScan_plainTextLength8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ stopCharacter: UInt8) -> Int {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    return unsafe HTMLScan.plainTextLength(buffer.span, stopCharacter: stopCharacter)
}
