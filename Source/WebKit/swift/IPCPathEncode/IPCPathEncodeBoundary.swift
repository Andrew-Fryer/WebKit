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

// The C entry points of the Path encoder, declared to C++ in IPCPathEncodeSwiftBoundary.h. The only unsafe operations:
// wrapping the PathSegment array (4-byte aligned, which the glue static_asserts) as a Span of words and the output
// buffer as a MutableRawSpan, and reading the output buffer's address modulo 8. Everything else is in the safe core
// (IPCPathEncode.swift).

/// The PathSegment array (start, segmentCount) as 32-bit words. A count that a std::span can't have traps rather than
/// overflowing.
@inline(__always)
private func pathSegmentWords(_ segments: UnsafeRawPointer?, _ segmentCount: Int) -> UnsafeBufferPointer<UInt32> {
    precondition(segmentCount >= 0 && segmentCount <= Int.max / (4 &* IPCPathEncoder.segmentWordCount), "PathSegment span larger than PTRDIFF_MAX")
    return unsafe UnsafeBufferPointer(start: segments?.assumingMemoryBound(to: UInt32.self), count: segmentCount &* IPCPathEncoder.segmentWordCount)
}

/// A C++ span of bytes (data, size).
@inline(__always)
private func cxxMutableBytes(_ data: UnsafeMutablePointer<UInt8>?, _ size: Int) -> UnsafeMutableRawBufferPointer {
    precondition(size >= 0, "C++ span larger than PTRDIFF_MAX")
    return unsafe UnsafeMutableRawBufferPointer(start: data, count: size)
}

@_cdecl("WebKitSwift_IPCPathEncode_encodeToStream")
func WebKitSwift_IPCPathEncode_encodeToStream(_ segments: UnsafeRawPointer?, _ segmentCount: Int, _ withCount: Bool,
                                              _ data: UnsafeMutablePointer<UInt8>?, _ capacity: Int) -> Int {
    var out = unsafe cxxMutableBytes(data, capacity).mutableBytes
    // StreamConnectionEncoder aligns addresses, not offsets.
    let baseMisalignment = Int(truncatingIfNeeded: UInt(bitPattern: data) & 7)
    return unsafe IPCPathEncoder.encodeToStream(pathSegmentWords(segments, segmentCount).span, segmentCount: segmentCount, withCount: withCount,
                                                into: &out, baseMisalignment: baseMisalignment)
}

@_cdecl("WebKitSwift_IPCPathEncode_encodedSize")
func WebKitSwift_IPCPathEncode_encodedSize(_ segments: UnsafeRawPointer?, _ segmentCount: Int, _ withCount: Bool, _ bodySize: Int) -> Int {
    unsafe IPCPathEncoder.encodedSize(pathSegmentWords(segments, segmentCount).span, segmentCount: segmentCount, withCount: withCount,
                                      baseMisalignment: bodySize & 7)
}

@_cdecl("WebKitSwift_IPCPathEncode_encodeToBody")
func WebKitSwift_IPCPathEncode_encodeToBody(_ segments: UnsafeRawPointer?, _ segmentCount: Int, _ withCount: Bool, _ bodySize: Int,
                                            _ data: UnsafeMutablePointer<UInt8>?, _ size: Int) {
    var out = unsafe cxxMutableBytes(data, size).mutableBytes
    unsafe IPCPathEncoder.encodeToBody(pathSegmentWords(segments, segmentCount).span, segmentCount: segmentCount, withCount: withCount,
                                       into: &out, baseMisalignment: bodySize & 7)
}
