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

// The header of an IPC stream message, read from the bytes that the sending process wrote into the stream
// connection's shared memory. This is the body of
//     IPC::Decoder::Decoder(std::span<const uint8_t> stream, uint64_t destinationID)
// in Source/WebKit/Platform/IPC/Decoder.cpp (the original C++ is kept there as Decoder::decodeStreamHeaderCxx()).
// The checks and the wire format are those of the C++ decoders that the constructor calls:
// - Decoder::decodeSpan<T>(1): a T is read at roundUpToMultipleOf<alignof(T)>(its address), and only if
//   alignedBufferIsLargeEnoughToContain(size, alignedPosition, sizeof(T)).
// - ArgumentCoder<MessageName> (the enum coder): a uint16_t that must satisfy isValidEnum<MessageName>, i.e. <= Last.
// - ArgumentCoder<AtomicObjectIdentifier<AsyncReplyIDType or SyncRequestIDType>>: a uint64_t that must satisfy
//   ObjectIdentifierGenericBase::isValidIdentifier(). Both coders are identical (same validator, same error string).

/// Bits 16-23 of the boundary result. The values are part of the C boundary (IPCStreamHeaderSwiftBoundary.h).
enum IPCStreamHeaderStatus: UInt8 {
    /// Header decoded; the arguments start at `argumentsOffset`.
    case success = 0
    /// The destination ID is the HashTable deleted value; nothing was read.
    case invalidDestinationID = 1
    /// decode<MessageName>() failed (truncated, or > MessageName::Last); the message name stays Invalid.
    case invalidMessageName = 2
    /// The message name was decoded, but the reply ID that follows it doesn't fit in the stream.
    case truncatedReplyID = 3
    /// The message name was decoded, but its reply ID is 0 or the HashTable deleted value. The C++ decoder sets
    /// the IPC_TESTING_API error string in this case.
    case invalidReplyID = 4
}

struct IPCStreamHeader {
    var status: IPCStreamHeaderStatus
    /// Valid for success, truncatedReplyID and invalidReplyID (the C++ decoder has already stored it then).
    var messageName: UInt16 = 0
    /// Valid for success: the offset of the first argument, from the start of the stream span.
    var argumentsOffset: Int = 0
    /// Valid for success: the AsyncReplyID or SyncRequestID, or 0 if the message has none.
    var replyID: UInt64 = 0

    /// The boundary result: bits 0-15 the message name, 16-23 the status, 24-63 the arguments offset.
    /// (argumentsOffset is at most 1 + 2 + 7 + 8 = 18, so truncation never drops a bit.)
    @inline(__always)
    var packed: UInt64 {
        UInt64(messageName) | UInt64(status.rawValue) &<< 16 | UInt64(truncatingIfNeeded: argumentsOffset) &<< 24
    }
}

/// WTF::ObjectIdentifierGenericBase::isValidIdentifier(): neither 0 nor the HashTable deleted value (-1).
@inline(__always)
func isValidIPCIdentifier(_ identifier: UInt64) -> Bool {
    identifier != 0 && identifier != UInt64.max
}

/// The offset, from the start of the stream, of roundUpToMultipleOf<alignment>(start + offset), given
/// start % 8 == baseMisalignment. Valid for the power-of-two alignments that divide 8 (all the header uses).
@inline(__always)
func ipcAlignedOffset(_ offset: Int, alignment: Int, baseMisalignment: Int) -> Int {
    let mask = alignment &- 1
    return ((baseMisalignment &+ offset &+ mask) & ~mask) &- baseMisalignment
}

/// IPC::alignedBufferIsLargeEnoughToContain().
@inline(__always)
func ipcBufferIsLargeEnoughToContain(_ bufferSize: Int, _ alignedBufferPosition: Int, _ bytesNeeded: Int) -> Bool {
    bufferSize >= alignedBufferPosition && bufferSize &- alignedBufferPosition >= bytesNeeded
}

/// Decodes a stream message header. `baseMisalignment` is the stream's start address modulo 8 (the C++ aligns
/// absolute addresses, not offsets). `firstMessageNameWithReplyID` is
/// min(MessageName::FirstAsyncWithReply, MessageName::FirstSynchronous): messageIsAsyncWithReply() and
/// messageIsSync() together are exactly the names >= that, and both decode the same uint64_t reply ID.
@inline(__always)
func decodeIPCStreamHeader(_ stream: RawSpan, baseMisalignment: Int, destinationID: UInt64,
                           lastMessageName: UInt16, firstMessageNameWithReplyID: UInt16) -> IPCStreamHeader {
    // 0 is a valid destinationID but we can at least reject -1 which is the HashTable deleted value.
    if destinationID != 0 && !isValidIPCIdentifier(destinationID) {
        return IPCStreamHeader(status: .invalidDestinationID)
    }

    let size = stream.byteCount

    // decode<MessageName>()
    let messageNameOffset = ipcAlignedOffset(0, alignment: 2, baseMisalignment: baseMisalignment)
    guard ipcBufferIsLargeEnoughToContain(size, messageNameOffset, 2) else {
        return IPCStreamHeader(status: .invalidMessageName)
    }
    let messageName = stream.load(fromByteOffset: messageNameOffset, as: UInt16.self)
    guard messageName <= lastMessageName else {
        return IPCStreamHeader(status: .invalidMessageName)
    }
    var position = messageNameOffset &+ 2

    // decode<AsyncReplyID>() or decode<SyncRequestID>()
    var replyID: UInt64 = 0
    if messageName >= firstMessageNameWithReplyID {
        let replyIDOffset = ipcAlignedOffset(position, alignment: 8, baseMisalignment: baseMisalignment)
        guard ipcBufferIsLargeEnoughToContain(size, replyIDOffset, 8) else {
            return IPCStreamHeader(status: .truncatedReplyID, messageName: messageName)
        }
        let decodedReplyID = stream.load(fromByteOffset: replyIDOffset, as: UInt64.self)
        guard isValidIPCIdentifier(decodedReplyID) else {
            return IPCStreamHeader(status: .invalidReplyID, messageName: messageName)
        }
        replyID = decodedReplyID
        position = replyIDOffset &+ 8
    }

    return IPCStreamHeader(status: .success, messageName: messageName, argumentsOffset: position, replyID: replyID)
}
