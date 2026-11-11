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

// The C entry points of the WebKit IPC decoders, declared to C++ in IPCStreamMessageSwiftBoundary.h. The only unsafe
// operations: wrapping the C++ spans (pointer + count) as Spans and MutableSpans with cxxSpan() and cxxMutableSpan(),
// and the C structs as MutableRawSpans, and storing the reply ID. Everything else is in the safe cores (IPCStreamMessage.swift, IPCArgDecoders.swift,
// IPCPathDecode.swift).

/// A C++ span (start, count). A std::span is never larger than PTRDIFF_MAX, so a negative count (a larger size_t) traps
/// rather than forming a buffer with a negative count.
@inline(__always)
private func cxxSpan<T>(_ start: UnsafePointer<T>?, _ count: Int) -> UnsafeBufferPointer<T> {
    precondition(count >= 0, "C++ span larger than PTRDIFF_MAX")
    return unsafe UnsafeBufferPointer(start: start, count: count)
}

@inline(__always)
private func cxxMutableSpan<T>(_ start: UnsafeMutablePointer<T>?, _ count: Int) -> UnsafeMutableBufferPointer<T> {
    precondition(count >= 0, "C++ span larger than PTRDIFF_MAX")
    return unsafe UnsafeMutableBufferPointer(start: start, count: count)
}

/// The address of a span's start modulo 8: values are aligned in memory, not in the span.
@inline(__always)
private func baseMisalignment(_ data: UnsafePointer<UInt8>?) -> Int {
    Int(truncatingIfNeeded: UInt(bitPattern: data) & 7)
}

@_cdecl("WebKitSwift_IPCStreamMessage_decode")
func WebKitSwift_IPCStreamMessage_decode(_ data: UnsafePointer<UInt8>?, _ size: Int, _ destinationID: UInt64, _ firstMessageNameWithReplyID: UInt16,
                                         _ argumentLayouts: UnsafePointer<UInt8>?, _ layoutCount: Int,
                                         _ replyID: UnsafeMutablePointer<UInt64>, _ arguments: UnsafeMutableRawPointer) -> UInt64 {
    var output = unsafe UnsafeMutableRawBufferPointer(start: arguments, count: IPCStreamArguments.byteCount).mutableBytes
    let header = unsafe decodeIPCStreamMessage(cxxSpan(data, size).span.bytes, baseMisalignment: baseMisalignment(data), destinationID: destinationID,
                                               firstMessageNameWithReplyID: firstMessageNameWithReplyID,
                                               layouts: cxxSpan(argumentLayouts, layoutCount).span, arguments: &output)
    // Only a successfully decoded reply ID is non-zero; the C++ leaves m_replyID untouched otherwise.
    if header.replyID != 0 {
        unsafe replyID.pointee = header.replyID
    }
    return header.packed
}

@_cdecl("WebKitSwift_IPCStreamMessage_locateArguments")
func WebKitSwift_IPCStreamMessage_locateArguments(_ data: UnsafePointer<UInt8>?, _ size: Int, _ position: Int, _ layout: UInt8,
                                                  _ arguments: UnsafeMutableRawPointer) {
    var output = unsafe UnsafeMutableRawBufferPointer(start: arguments, count: IPCStreamArguments.byteCount).mutableBytes
    unsafe locateIPCArguments(cxxSpan(data, size).span.bytes, baseMisalignment: baseMisalignment(data), position: position, layout: layout,
                              arguments: &output)
}

// `progress` is a WebKitSwift_IPCStreamMessage_PathProgress: four 64-bit words.
@_cdecl("WebKitSwift_IPCStreamMessage_decodePath")
func WebKitSwift_IPCStreamMessage_decodePath(_ data: UnsafePointer<UInt8>?, _ size: Int, _ position: Int, _ words: UnsafeMutablePointer<UInt32>?,
                                             _ wordCapacity: Int, _ trailingBoolEnum: Bool, _ progress: UnsafeMutablePointer<UInt64>) -> Int {
    var output = unsafe cxxMutableSpan(words, wordCapacity).mutableSpan
    var state = unsafe cxxMutableSpan(progress, IPCPathDecoder.progressWordCount).mutableSpan
    return unsafe IPCPathDecoder.decodePath(cxxSpan(data, size).span.bytes, baseMisalignment: baseMisalignment(data), position: position,
                                            into: &output, trailingBoolEnum: trailingBoolEnum, progress: &state)
}
