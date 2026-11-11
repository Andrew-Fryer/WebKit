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

// C entry point of the stream message decoder, declared to C++ in IPCStreamMessageSwiftBoundary.h. The only unsafe
// operations: wrapping the C++ spans (pointer + size) of the stream and of the layout table as a RawSpan and a Span,
// taking the stream's start address modulo 8, and storing the reply ID and the arguments through the out-parameters.

@_cdecl("WebKitSwift_IPCStreamMessage_decode")
func WebKitSwift_IPCStreamMessage_decode(_ data: UnsafeRawPointer?, _ size: Int, _ destinationID: UInt64,
                                         _ firstMessageNameWithReplyID: UInt16, _ argumentLayouts: UnsafePointer<UInt8>?,
                                         _ layoutCount: Int, _ replyID: UnsafeMutablePointer<UInt64>,
                                         _ arguments: UnsafeMutableRawPointer) -> UInt64 {
    // Both are size_t. A std::span can't be larger than PTRDIFF_MAX (no object is), so a negative Int means the caller's
    // span is already undefined behavior; trap rather than form a span with a negative count.
    precondition(size >= 0 && layoutCount >= 0, "IPC stream span larger than PTRDIFF_MAX")
    let stream = unsafe UnsafeRawBufferPointer(start: data, count: size)
    let layouts = unsafe UnsafeBufferPointer(start: argumentLayouts, count: layoutCount)
    let baseMisalignment = Int(truncatingIfNeeded: UInt(bitPattern: data) & 7)
    var located = IPCStreamArguments()
    let header = unsafe decodeIPCStreamMessage(stream.bytes, baseMisalignment: baseMisalignment, destinationID: destinationID,
                                               firstMessageNameWithReplyID: firstMessageNameWithReplyID, layouts: layouts.span,
                                               arguments: &located)
    // Only a successfully decoded reply ID is non-zero; the C++ leaves m_replyID untouched otherwise.
    if header.replyID != 0 {
        unsafe replyID.pointee = header.replyID
    }
    // WebKitSwift_IPCStreamMessage_Arguments: uint64_t, size_t, size_t, uint8_t, uint8_t.
    unsafe arguments.storeBytes(of: located.identifier, toByteOffset: 0, as: UInt64.self)
    unsafe arguments.storeBytes(of: located.valuesOffset, toByteOffset: 8, as: Int.self)
    unsafe arguments.storeBytes(of: located.endOffset, toByteOffset: 16, as: Int.self)
    unsafe arguments.storeBytes(of: located.status.rawValue, toByteOffset: 24, as: UInt8.self)
    unsafe arguments.storeBytes(of: located.flag, toByteOffset: 25, as: UInt8.self)
    return header.packed
}
