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

// C entry points of the IPC argument decoders, declared to C++ in IPCArgDecodersSwiftBoundary.h. The only unsafe
// operations: wrapping the decoder's span (pointer + size) as a RawSpan, taking its start address modulo 8, and storing
// scalar results through out-parameters.

/// The decoder's span. A std::span is never larger than PTRDIFF_MAX and the position never past its end, so anything
/// else (a negative Int) traps rather than forming a span with a negative byte count.
@inline(__always)
private func stream(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int) -> UnsafeRawBufferPointer {
    precondition(position >= 0 && position <= size, "IPC decoder position outside its span")
    return unsafe UnsafeRawBufferPointer(start: data, count: size)
}

/// The address of the span's start modulo 8: values are aligned in memory, not in the span.
@inline(__always)
private func baseMisalignment(_ data: UnsafeRawPointer?) -> Int {
    Int(truncatingIfNeeded: UInt(bitPattern: data) & 7)
}

@_cdecl("WebKitSwift_IPCArgDecoders_decodeWords")
func WebKitSwift_IPCArgDecoders_decodeWords(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int, _ wordCount: Int,
                                            _ wordsOffset: UnsafeMutablePointer<Int>) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var offset = 0
    let result = IPCArgumentDecoders.decodeWords(&reader, wordCount: wordCount, wordsOffset: &offset)
    unsafe wordsOffset.pointee = offset
    return result
}

@_cdecl("WebKitSwift_IPCArgDecoders_decodeWordsAndBoolEnum")
func WebKitSwift_IPCArgDecoders_decodeWordsAndBoolEnum(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int, _ wordCount: Int,
                                                       _ wordsOffset: UnsafeMutablePointer<Int>, _ flag: UnsafeMutablePointer<UInt8>) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var offset = 0
    var value: UInt8 = 0
    let result = IPCArgumentDecoders.decodeWordsAndBoolEnum(&reader, wordCount: wordCount, wordsOffset: &offset, flag: &value)
    unsafe wordsOffset.pointee = offset
    unsafe flag.pointee = value
    return result
}

@_cdecl("WebKitSwift_IPCArgDecoders_decodeIdentifierAndAffineTransform")
func WebKitSwift_IPCArgDecoders_decodeIdentifierAndAffineTransform(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int,
                                                                   _ identifier: UnsafeMutablePointer<UInt64>,
                                                                   _ matrixOffset: UnsafeMutablePointer<Int>) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var value: UInt64 = 0
    var offset = 0
    let result = IPCArgumentDecoders.decodeIdentifierAndAffineTransform(&reader, identifier: &value, matrixOffset: &offset)
    unsafe identifier.pointee = value
    unsafe matrixOffset.pointee = offset
    return result
}
