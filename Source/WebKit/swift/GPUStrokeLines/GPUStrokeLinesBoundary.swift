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

// The C entry point of the batched line strokes, declared to C++ in GPUStrokeLinesSwiftBoundary.h. The only unsafe
// operation: wrapping the C++ span of lines (pointer + count) as a Span. The context is an opaque integer handle that
// Swift only passes back to the C++ callbacks. Everything else is in the safe core (GPUStrokeLines.swift).

/// The C++ callbacks of RemoteGraphicsContext::strokeLinesWithColorAndThickness(), with its handle.
struct GPUStrokeLinesCallbacks: GPUStrokeLinesSink {
    let context: UInt
    let setStrokeColorCallback: @convention(c) (UInt, UInt32) -> Void
    let setStrokeThicknessCallback: @convention(c) (UInt, Float) -> Void
    let strokeLineCallback: @convention(c) (UInt, Float, Float, Float, Float) -> Void

    @inline(__always)
    func setStrokeColor(_ rgba: UInt32) {
        setStrokeColorCallback(context, rgba)
    }

    @inline(__always)
    func setStrokeThickness(_ thickness: Float) {
        setStrokeThicknessCallback(context, thickness)
    }

    @inline(__always)
    func strokeLine(_ startX: Float, _ startY: Float, _ endX: Float, _ endY: Float) {
        strokeLineCallback(context, startX, startY, endX, endY)
    }
}

@_cdecl("WebKitSwift_GPUStrokeLines_strokeLines")
func WebKitSwift_GPUStrokeLines_strokeLines(_ lines: UnsafePointer<UInt32>?, _ lineCount: Int, _ context: UInt,
                                            _ setStrokeColor: @convention(c) (UInt, UInt32) -> Void,
                                            _ setStrokeThickness: @convention(c) (UInt, Float) -> Void,
                                            _ strokeLine: @convention(c) (UInt, Float, Float, Float, Float) -> Void) {
    // A std::span is never larger than PTRDIFF_MAX bytes, so a negative count (a larger size_t), or one whose word
    // count overflows, traps rather than forming a buffer with a wrong count.
    precondition(lineCount >= 0, "C++ span larger than PTRDIFF_MAX")
    let (wordCount, overflow) = lineCount.multipliedReportingOverflow(by: GPUStrokeLines.wordsPerLine)
    precondition(!overflow, "C++ span larger than PTRDIFF_MAX")
    let words = unsafe UnsafeBufferPointer(start: lines, count: wordCount)
    let sink = GPUStrokeLinesCallbacks(context: context, setStrokeColorCallback: setStrokeColor,
                                       setStrokeThicknessCallback: setStrokeThickness, strokeLineCallback: strokeLine)
    unsafe GPUStrokeLines.strokeLines(words.span, sink: sink)
}
