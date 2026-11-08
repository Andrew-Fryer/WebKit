/*
 * Copyright (C) 2026 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// The IPC stream message header decoder implemented in Swift (IPCStreamHeader.swift), called from
// IPC::Decoder::Decoder(std::span<const uint8_t>, uint64_t) in Platform/IPC/Decoder.cpp.
// Decoder.cpp is compiled into the WebKitPlatform static library (Platform/Sources.txt), which WebKit.framework and
// TestIPC both link, so the Swift files belong to the WebKitPlatform target too (not to WebKit or WebKitSwift).
// The symbol is hidden: only call it from WebKitPlatform .cpp files, never from a header.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bits 16-23 of the result of WebKitSwift_IPCStreamHeader_decode().
enum {
    WebKitSwift_IPCStreamHeader_Success = 0, // Bits 0-15: message name. Bits 24-63: arguments offset.
    WebKitSwift_IPCStreamHeader_InvalidDestinationID = 1, // destinationID is the HashTable deleted value.
    WebKitSwift_IPCStreamHeader_InvalidMessageName = 2, // Truncated or > lastMessageName.
    WebKitSwift_IPCStreamHeader_TruncatedReplyID = 3, // Bits 0-15: message name.
    WebKitSwift_IPCStreamHeader_InvalidReplyID = 4, // Bits 0-15: message name. Reply ID is 0 or the deleted value.
};

// Decodes the message name, and the reply ID of a message name >= firstMessageNameWithReplyID, from the start of
// the stream span (data, size). data's address matters: values are aligned to their size in memory, not in the
// span. Stores the reply ID in *replyID only on success and only if the message has one.
// size must be at most PTRDIFF_MAX, like the size of any std::span; a larger size traps.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
uint64_t WebKitSwift_IPCStreamHeader_decode(const uint8_t* data, size_t size, uint64_t destinationID,
    uint16_t lastMessageName, uint16_t firstMessageNameWithReplyID, uint64_t* replyID);

#ifdef __cplusplus
}
#endif
