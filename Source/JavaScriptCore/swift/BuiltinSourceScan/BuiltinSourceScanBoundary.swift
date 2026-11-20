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

// C entry point for BuiltinSourceScan.swift, declared to C++ in BuiltinSourceScanSwiftBoundary.h and called only from
// builtins/BuiltinExecutables.cpp. The only unsafe operations turn the source's (pointer, length) and the result words
// into spans. The source pointer may be null only when the length is 0 (and then the scan's first precondition traps).
// Internal, so the symbol stays hidden inside JavaScriptCore. @inline(__always) folds the safe core into the C entry
// point itself rather than leaving a call to a Swift-native copy.

@_cdecl("JSCSwift_BuiltinSourceScan_computeMetadata")
@inline(__always)
func JSCSwift_BuiltinSourceScan_computeMetadata(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ words: UnsafeMutablePointer<UInt32>) {
    let source = unsafe UnsafeBufferPointer(start: characters, count: length).span
    var result = unsafe UnsafeMutableBufferPointer(start: words, count: BuiltinSourceScan.wordCount).mutableSpan
    BuiltinSourceScan.computeMetadata(source, into: &result)
}
