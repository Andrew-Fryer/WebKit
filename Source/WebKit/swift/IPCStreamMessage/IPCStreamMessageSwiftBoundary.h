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

// The decoder of IPC stream messages implemented in Swift (IPCStreamMessage.swift, with the IPCStreamHeader and
// IPCArgDecoders safe cores), called from IPC::Decoder::Decoder(std::span<const uint8_t>, uint64_t) in
// Platform/IPC/Decoder.cpp. It decodes the message header and, for a message whose arguments have a layout it knows
// (IPC::swiftArgumentLayout(), generated into MessageNames.h by messages.py from the parameter types of the messages
// with the SwiftArgumentDecoder attribute), validates and locates the arguments in the same call. Their
// decodeArgumentsInSwift() glue (GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp) then only builds
// the C++ arguments.
// Decoder.cpp is compiled into the WebKitPlatform static library (Platform/Sources.txt), which WebKit.framework and
// TestIPC both link, so the Swift files belong to the WebKitPlatform target too. The symbols are hidden: only call them
// from WebKit and WebKitPlatform .cpp files, never from a header.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bits 16-23 of the result of WebKitSwift_IPCStreamMessage_decode().
enum {
    WebKitSwift_IPCStreamMessage_Success = 0, // Bits 0-15: message name. Bits 24-63: arguments offset.
    WebKitSwift_IPCStreamMessage_InvalidDestinationID = 1, // destinationID is the HashTable deleted value.
    WebKitSwift_IPCStreamMessage_InvalidMessageName = 2, // Truncated, or not less than layoutCount.
    WebKitSwift_IPCStreamMessage_TruncatedReplyID = 3, // Bits 0-15: message name.
    WebKitSwift_IPCStreamMessage_InvalidReplyID = 4, // Bits 0-15: message name. Reply ID is 0 or the deleted value.
};

// The argument layouts, the values of IPC::swiftArgumentLayout(). Every value is read at its alignment in memory (not
// in the span), as IPC::Decoder::decodeSpan<T>() reads it.
enum {
    // Not located: the arguments are decoded by the message's decodeArgumentsInSwift(), or with decode<Arguments>().
    WebKitSwift_IPCStreamMessage_LayoutNone = 0,
    // 1-15: that many 4-byte values that accept every bit pattern (float, uint32_t, and structs made of them, decoded
    // with decodeObject<T>() each), one aligned run at valuesOffset.
    WebKitSwift_IPCStreamMessage_LayoutWords = 0,
    // 17-31: (layout - 16) such values at valuesOffset, then an enum class : bool, in flag.
    WebKitSwift_IPCStreamMessage_LayoutWordsAndBoolEnum = 16,
    // An ObjectIdentifier, in identifier, then an AffineTransform (6 doubles, std::span<const double, 6>), at valuesOffset.
    WebKitSwift_IPCStreamMessage_LayoutIdentifierAndAffineTransform = 32,
    // An AffineTransform, at valuesOffset, an ObjectIdentifier, in identifier, then an enum class : bool, in flag.
    WebKitSwift_IPCStreamMessage_LayoutAffineTransformIdentifierAndBoolEnum = 33,
};
// (The other wire values of the RemoteGraphicsContext messages decode in about a nanosecond inline in C++, which is less
// than locating them costs; see Source/WebKit/GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp.)

// WebKitSwift_IPCStreamMessage_Arguments::status.
enum {
    WebKitSwift_IPCStreamMessage_ArgumentsNone = 0, // Not located: the header failed, or the layout is None.
    WebKitSwift_IPCStreamMessage_ArgumentsValid = 1,
    // A value didn't fit in the span or failed validation: decode<Arguments>() fails and marks the decoder invalid.
    WebKitSwift_IPCStreamMessage_ArgumentsInvalid = 2,
    // An ObjectIdentifier was 0 or the deleted value: also sets the IPC_TESTING_API error string of the generated
    // ObjectIdentifier coders.
    WebKitSwift_IPCStreamMessage_ArgumentsInvalidIdentifier = 3,
};

// Where the arguments are, and the values that the C++ coders validate (ObjectIdentifier, enum class : bool), read
// once. The values that accept every bit pattern (floats, doubles, uint32_t) are left in the span, at valuesOffset, for
// the caller to copy (read once) as Decoder::decodeObject<T>() does. All the offsets are from the start of the span, and
// only meaningful for status Valid: the values are then in bounds of the span, and endOffset, the decoder's position
// after the arguments, is at most its size. The fields that a layout doesn't have are 0.
typedef struct {
    uint64_t identifier;
    size_t valuesOffset;
    size_t endOffset;
    uint8_t status;
    uint8_t flag; // 0 or 1.
} WebKitSwift_IPCStreamMessage_Arguments;

// Decodes the message name, the reply ID of a message name >= firstMessageNameWithReplyID, and then, for a message name
// whose argumentLayouts entry is not None, the arguments, from the start of the stream span (data, size). data's
// address matters: values are aligned to their size in memory, not in the span. argumentLayouts has one entry per
// message name, so a message name is valid if and only if it is less than layoutCount (MessageName::Last + 1).
// Stores the reply ID in *replyID only on success and only if the message has one, and always stores all of *arguments
// (all 0, which is status None, if the header failed or the layout is None).
// size and layoutCount must be at most PTRDIFF_MAX, like the size of any std::span; larger ones trap.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
uint64_t WebKitSwift_IPCStreamMessage_decode(const uint8_t* data, size_t size, uint64_t destinationID,
    uint16_t firstMessageNameWithReplyID, const uint8_t* argumentLayouts, size_t layoutCount, uint64_t* replyID,
    WebKitSwift_IPCStreamMessage_Arguments* arguments);

#ifdef __cplusplus
}
#endif
