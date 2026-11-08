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

// Decoders for the arguments of hot IPC stream messages, implemented in Swift (IPCArgDecoders.swift) and called from
// GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp, which defines decodeArgumentsInSwift() for the
// messages that have the SwiftArgumentDecoder attribute in their messages.in file.
// The Swift files are compiled into the WebKitPlatform static library, like IPCStreamHeader's (they share its
// helpers), and WebKit.framework links it. The symbols are hidden: only call them from WebKit .cpp files.
//
// Every decoder reads from the span of an IPC::Decoder, (data, size), from position = Decoder::currentBufferOffset().
// data's address matters: values are aligned to their size in memory, not in the span. position must be at most size,
// and size at most PTRDIFF_MAX, as with any std::span and Decoder; anything else traps.
// A decoder returns the decoder's new position (>= 0), or a negative WebKitSwift_IPCArgDecoders_Failure. The output
// parameters are meaningful only on success. The decoders output the values that the C++ coders validate (enum class :
// bool, ObjectIdentifier); for floats, doubles and uint32_t, which accept any bit pattern, they output the offset in
// the span of each aligned run of them, from which the caller copies them, as Decoder::decodeObject<T>() does.
// The run's bytes are in bounds of the span; the caller reads each value once.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum WebKitSwift_IPCArgDecoders_Failure {
    // A value didn't fit in the span or failed validation: the caller marks the decoder invalid.
    WebKitSwift_IPCArgDecoders_Invalid = -1,
    // An ObjectIdentifier was 0 or the deleted value: the caller also sets the IPC_TESTING_API error string of the
    // generated ObjectIdentifier coders.
    WebKitSwift_IPCArgDecoders_InvalidIdentifier = -2,
};

// wordCount > 0 4-byte values (float, uint32_t, PackedColor::RGBA), each read with Decoder::decodeObject<T>(): they
// are the 4 * wordCount bytes at data + *wordsOffset.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCArgDecoders_decodeWords(const uint8_t* data, size_t size, size_t position, size_t wordCount,
    size_t* wordsOffset);

// wordCount > 0 4-byte values, at data + *wordsOffset, then an enum class : bool, stored in *flag as 0 or 1.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCArgDecoders_decodeWordsAndBoolEnum(const uint8_t* data, size_t size, size_t position, size_t wordCount,
    size_t* wordsOffset, uint8_t* flag);

// An ObjectIdentifier, then an AffineTransform (std::span<const double, 6>): the 48 bytes at data + *matrixOffset.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCArgDecoders_decodeIdentifierAndAffineTransform(const uint8_t* data, size_t size, size_t position,
    uint64_t* identifier, size_t* matrixOffset);

#ifdef __cplusplus
}
#endif
