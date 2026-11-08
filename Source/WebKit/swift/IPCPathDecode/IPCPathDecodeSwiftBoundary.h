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

// Decoder for the WebCore::Path arguments of IPC stream messages (StrokePath, FillPath, ...), implemented in Swift
// (IPCPathDecode.swift) and called from GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp, which builds
// the Path from what it decodes. Like IPCArgDecoders' decoders, it reads the span of an IPC::Decoder, (data, size), from
// position = Decoder::currentBufferOffset(), with the same rules (IPCArgDecodersSwiftBoundary.h): data's address matters,
// position must be at most size and size at most PTRDIFF_MAX (anything else traps), and a decoder returns the decoder's
// new position (>= 0) or WebKitSwift_IPCArgDecoders_Invalid (-1), after which the output is meaningless.
// The symbols are hidden: only call them from WebKit .cpp files.
//
// The wire format of a Path is that of ArgumentCoder<Path>: a Vector<PathSegment>, which is a uint64_t size and then
// that many PathSegment::Data variants, each a uint8_t alternative index followed by the alternative's members.
// The decoder outputs each segment as a record of WebKitSwift_IPCPathDecode_RecordWords uint32_t words:
//   word 0:       the alternative index (bits 0-7, always < WebKitSwift_IPCPathDecode_AlternativeCount), and the
//                 alternative's trailing 1-byte enum (bits 8-15, 0 or 1: the RotationDirection of PathArc,
//                 PathClosedArc and PathEllipse, the Strategy of PathRoundedRect; 0 for the other alternatives)
//   words 1...n:  the bit patterns of the alternative's n floats, in the order the coders read them (n is 0 to 12);
//                 the words after them are not written.
// The floats are copied from the span, so each byte of the span is read once.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    WebKitSwift_IPCPathDecode_RecordWords = 13,
    WebKitSwift_IPCPathDecode_AlternativeCount = 17,
};

// Decodes the size of the Vector<PathSegment>, *segmentCount, then its first min(*segmentCount, recordCapacity) segments
// into records (recordCapacity * WebKitSwift_IPCPathDecode_RecordWords words). A size larger than the bytes left in the
// span fails (each segment takes at least one byte), so on success *segmentCount <= size - position.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCPathDecode_decodePath(const uint8_t* data, size_t size, size_t position, uint32_t* records,
    size_t recordCapacity, uint64_t* segmentCount);

// Decodes the next recordCount segments into records (recordCount * WebKitSwift_IPCPathDecode_RecordWords words).
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCPathDecode_decodePathSegments(const uint8_t* data, size_t size, size_t position, uint32_t* records,
    size_t recordCount);

// Decodes an enum class : bool (the WindRule after the Path of ClipPath), stored in *value as 0 or 1.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCPathDecode_decodeBoolEnum(const uint8_t* data, size_t size, size_t position, uint8_t* value);

#ifdef __cplusplus
}
#endif
