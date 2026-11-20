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

// The GPU process's batched line strokes (RemoteGraphicsContext::strokeLinesWithColorAndThickness(), the
// StrokeLinesWithColorAndThickness stream message), implemented in Swift: GPUStrokeLines.swift decides, line by line,
// which stroke state changes to push to the context and calls back into C++ for them and for each line.
// The only caller is RemoteGraphicsContext.cpp, which is compiled into WebKit.framework, so the Swift files belong to the
// WebKit target. The symbol is hidden: only call it from WebKit .cpp files, never from a header.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// A WebCore::PathDataLineColorThickness is 6 32-bit words, at 4-byte alignment (static_asserted by the caller):
// words 0-3 the bit patterns of the line's floats (start x, start y, end x, end y), word 4 the PackedColor::RGBA value,
// word 5 the bit pattern of the float thickness.
enum {
    WebKitSwift_GPUStrokeLines_WordsPerLine = 6,
    WebKitSwift_GPUStrokeLines_ColorWord = 4,
    WebKitSwift_GPUStrokeLines_ThicknessWord = 5,
};

// The callbacks, which do what the C++ loop called for a line. `context` is the opaque handle the caller passed in
// (Swift never dereferences it).
typedef void (*WebKitSwift_GPUStrokeLines_SetStrokeColor)(uintptr_t context, uint32_t rgba);
typedef void (*WebKitSwift_GPUStrokeLines_SetStrokeThickness)(uintptr_t context, float thickness);
typedef void (*WebKitSwift_GPUStrokeLines_StrokeLine)(uintptr_t context, float startX, float startY, float endX, float endY);

// Strokes the `lineCount` lines at `lines` (lineCount * WordsPerLine words, read once each), in order. Before a line, it
// calls setStrokeColor if the line's color differs from the previous line's (or it is the first line), then
// setStrokeThickness, with the line's thickness clamped to be at least 0 (a NaN or -0 is passed as is, as std::max(t, 0.f)
// does), if the line's thickness compares (IEEE) not equal to the previous line's (or it is the first line, so a NaN
// thickness is set for every line), and then strokeLine with the line. `lines` may be null when lineCount is 0.
// lineCount * WordsPerLine must be at most PTRDIFF_MAX, like the size of any std::span; a larger one traps.
__attribute__((visibility("hidden"))) __attribute__((nothrow))
void WebKitSwift_GPUStrokeLines_strokeLines(const uint32_t* lines, size_t lineCount, uintptr_t context,
    WebKitSwift_GPUStrokeLines_SetStrokeColor setStrokeColor, WebKitSwift_GPUStrokeLines_SetStrokeThickness setStrokeThickness,
    WebKitSwift_GPUStrokeLines_StrokeLine strokeLine);

#ifdef __cplusplus
}
#endif
