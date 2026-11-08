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

// C entry point of the stream message header decoder, declared to C++ in IPCStreamHeaderSwiftBoundary.h.
// The only unsafe operations: wrapping the C++ span (pointer + size) as a RawSpan, taking the start address
// modulo 8, and storing the reply ID through the out-parameter.

@_cdecl("WebKitSwift_IPCStreamHeader_decode")
func WebKitSwift_IPCStreamHeader_decode(_ data: UnsafeRawPointer?, _ size: Int, _ destinationID: UInt64,
                                        _ lastMessageName: UInt16, _ firstMessageNameWithReplyID: UInt16,
                                        _ replyID: UnsafeMutablePointer<UInt64>) -> UInt64 {
    // size is a size_t. A std::span can't be larger than PTRDIFF_MAX (no object is), so a negative Int means the
    // caller's span is already undefined behavior; trap rather than form a RawSpan with a negative byte count.
    precondition(size >= 0, "IPC stream span larger than PTRDIFF_MAX")
    let stream = unsafe UnsafeRawBufferPointer(start: data, count: size)
    let baseMisalignment = Int(truncatingIfNeeded: UInt(bitPattern: data) & 7)
    let header = unsafe decodeIPCStreamHeader(stream.bytes, baseMisalignment: baseMisalignment, destinationID: destinationID,
                                              lastMessageName: lastMessageName, firstMessageNameWithReplyID: firstMessageNameWithReplyID)
    // Only a successfully decoded reply ID is non-zero; the C++ leaves m_replyID untouched otherwise.
    if header.replyID != 0 {
        unsafe replyID.pointee = header.replyID
    }
    return header.packed
}
