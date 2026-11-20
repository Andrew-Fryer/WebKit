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

// The IPC encoder of WebCore::Path, WebCore::PathImpl and WebCore::PathSegment, implemented in Swift
// (IPCPathEncode.swift), which is the only encoder of them: ArgumentCoder<Path>, ArgumentCoder<PathImpl> and
// ArgumentCoder<PathSegment>::encode() (Shared/PathArgumentCoders.cpp, for both IPC::Encoder and
// IPC::StreamConnectionEncoder) only gather the segments and do the encoder's bookkeeping. The decoders are the generated
// C++ ones and, for the GPU process's stream messages, IPCPathDecode.swift (IPCStreamMessageSwiftBoundary.h).
// The Swift files are compiled into the WebKitPlatform static library, with the other IPC Swift, and WebKit.framework
// links it. The symbols are hidden: only call them from WebKit and WebKitPlatform .cpp files, never from a header.
//
// `segments` is an array of `segmentCount` WebCore::PathSegments, read as 14 32-bit words each (the glue static_asserts
// the layout): an alternative's floats from word 0 on, in the order the coders encode them, then its 1-byte enum, if it
// has one, in the low byte of the next word (PathArc, PathClosedArc, PathEllipse and PathRoundedRect), and the
// PathSegment::Data variant index in the low byte of word 13. An index that isn't an alternative (>= 17) traps.
// The wire format is the C++ coders': with `withCount` (a Path or PathImpl: VectorArgumentCoder<false, PathSegment>),
// the segment count as a uint64_t at 8-byte alignment; then, for each segment, the variant index as a uint8_t, the
// alternative's floats at 4-byte alignment (none for PathCloseSubpath), and its 1-byte enum. Without `withCount`
// (a PathSegment), `segmentCount` must be 1.
// Sizes must be at most PTRDIFF_MAX, like the size of any std::span; larger ones trap.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// IPC::StreamConnectionEncoder: encodes the segments at the start of the encoder's buffer (data, capacity), aligning
// each value to its size in memory (the address of data modulo 8 matters), as StreamConnectionEncoder::encodeSpan()
// does, without writing the alignment gaps. Returns the number of bytes encoded, or -1 if they don't fit, in which case
// the bytes it wrote are meaningless and the encoder must become invalid.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
intptr_t WebKitSwift_IPCPathEncode_encodeToStream(const void* segments, size_t segmentCount, bool withCount, uint8_t* data,
    size_t capacity);

// IPC::Encoder: the number of bytes that encoding the segments appends to a body of `bodySize` bytes. IPC::Encoder
// aligns each value to its size in the body (not in memory), and zeroes the alignment gaps.
__attribute__((visibility("hidden"))) __attribute__((nothrow)) __attribute__((warn_unused_result))
size_t WebKitSwift_IPCPathEncode_encodedSize(const void* segments, size_t segmentCount, bool withCount, size_t bodySize);

// IPC::Encoder: encodes the segments into the `size` bytes (data, size) appended to a body of `bodySize` bytes, writing
// every one of them (the alignment gaps as 0). `size` must be what WebKitSwift_IPCPathEncode_encodedSize() returned for
// the same arguments; any other size traps.
__attribute__((visibility("hidden"))) __attribute__((nothrow))
void WebKitSwift_IPCPathEncode_encodeToBody(const void* segments, size_t segmentCount, bool withCount, size_t bodySize,
    uint8_t* data, size_t size);

#ifdef __cplusplus
}
#endif
