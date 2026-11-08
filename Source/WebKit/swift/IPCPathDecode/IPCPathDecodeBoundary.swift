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

// C entry points of the Path decoder, declared to C++ in IPCPathDecodeSwiftBoundary.h. The only unsafe operations:
// wrapping the decoder's span (pointer + size) as a RawSpan and the records (pointer + count) as a MutableSpan, taking
// the span's start address modulo 8, and storing scalar results through out-parameters.

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

/// `recordCount` records of the caller's.
@inline(__always)
private func records(_ records: UnsafeMutablePointer<UInt32>?, _ recordCount: Int) -> UnsafeMutableBufferPointer<UInt32> {
    precondition(recordCount >= 0 && recordCount <= Int.max / IPCPathDecoder.recordWords, "bad IPC path record count")
    return unsafe UnsafeMutableBufferPointer(start: records, count: recordCount &* IPCPathDecoder.recordWords)
}

@_cdecl("WebKitSwift_IPCPathDecode_decodePath")
func WebKitSwift_IPCPathDecode_decodePath(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int, _ recordsPointer: UnsafeMutablePointer<UInt32>?,
                                          _ recordCapacity: Int, _ segmentCount: UnsafeMutablePointer<UInt64>) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var output = unsafe records(recordsPointer, recordCapacity).mutableSpan
    var count: UInt64 = 0
    let result = IPCPathDecoder.decodePath(&reader, into: &output, segmentCount: &count)
    unsafe segmentCount.pointee = count
    return result
}

@_cdecl("WebKitSwift_IPCPathDecode_decodePathSegments")
func WebKitSwift_IPCPathDecode_decodePathSegments(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int,
                                                  _ recordsPointer: UnsafeMutablePointer<UInt32>?, _ recordCount: Int) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var output = unsafe records(recordsPointer, recordCount).mutableSpan
    return IPCPathDecoder.decodeSegments(&reader, into: &output, count: recordCount)
}

@_cdecl("WebKitSwift_IPCPathDecode_decodeBoolEnum")
func WebKitSwift_IPCPathDecode_decodeBoolEnum(_ data: UnsafeRawPointer?, _ size: Int, _ position: Int, _ value: UnsafeMutablePointer<UInt8>) -> Int {
    let bytes = unsafe stream(data, size, position)
    var reader = unsafe IPCArgumentDecoders.Reader(bytes.bytes, baseMisalignment: baseMisalignment(data), position: position)
    var decoded: UInt8 = 0
    let result = IPCPathDecoder.decodeBoolEnum(&reader, value: &decoded)
    unsafe value.pointee = decoded
    return result
}
