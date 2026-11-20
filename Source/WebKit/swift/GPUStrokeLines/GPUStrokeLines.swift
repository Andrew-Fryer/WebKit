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

// The GPU process's batched line strokes: the body of RemoteGraphicsContext::strokeLinesWithColorAndThickness()
// (Source/WebKit/GPUProcess/graphics/RemoteGraphicsContext.cpp), which receives the lines of a
// StrokeLinesWithColorAndThickness stream message in place, in the stream connection's shared memory. Each line carries
// its stroke color and thickness, so the batch is self-describing; a state change is pushed to the context only when it
// differs from the previous line's, to avoid redundant CG state changes. Pushing the state and stroking the line are the
// context's work, done by the sink (GPUStrokeLinesBoundary.swift calls back into C++ for them).
//
// The lines are in memory that the sending process can write while this runs, so each word of a line is read exactly
// once, and the value read is the one used: the comparison with the previous line and the value pushed are the same.

/// What the loop asks the context to do, in order.
protocol GPUStrokeLinesSink {
    /// GraphicsContext::setStrokeColor() with the color of a PackedColor::RGBA value.
    func setStrokeColor(_ rgba: UInt32)
    /// GraphicsContext::setStrokeThickness().
    func setStrokeThickness(_ thickness: Float)
    /// GraphicsContext::strokeLine() of the line from (startX, startY) to (endX, endY).
    func strokeLine(_ startX: Float, _ startY: Float, _ endX: Float, _ endY: Float)
}

enum GPUStrokeLines {
    /// The layout of a WebCore::PathDataLineColorThickness in 32-bit words (GPUStrokeLinesSwiftBoundary.h):
    /// the four floats of the PathDataLine, the PackedColor::RGBA, the float thickness.
    static let wordsPerLine = 6
    static let colorWord = 4
    static let thicknessWord = 5

    /// The thickness pushed for a line: std::max(thickness, 0.f), which is `thickness < 0 ? 0 : thickness`, so a NaN
    /// and -0 are pushed as they are (Swift's max() would not keep -0).
    @inline(__always)
    static func clampedThickness(_ thickness: Float) -> Float {
        thickness < 0 ? 0 : thickness
    }

    /// Strokes the lines in `lines` (`wordsPerLine` words each; a partial line at the end is not a line, and the caller
    /// passes whole lines), in order.
    @inline(__always)
    static func strokeLines<Sink: GPUStrokeLinesSink>(_ lines: Span<UInt32>, sink: Sink) {
        // The last applied values. A std::optional compares not equal to every value when it is empty, and an engaged
        // std::optional<float> compares with the IEEE !=, so a NaN thickness is pushed again for every line.
        var hasColor = false
        var lastColor: UInt32 = 0
        var hasThickness = false
        var lastThickness: Float = 0
        var base = 0
        while lines.count &- base >= wordsPerLine {
            let startX = Float(bitPattern: lines[base])
            let startY = Float(bitPattern: lines[base &+ 1])
            let endX = Float(bitPattern: lines[base &+ 2])
            let endY = Float(bitPattern: lines[base &+ 3])
            let color = lines[base &+ colorWord]
            let thickness = Float(bitPattern: lines[base &+ thicknessWord])
            if !hasColor || lastColor != color {
                sink.setStrokeColor(color)
                hasColor = true
                lastColor = color
            }
            if !hasThickness || lastThickness != thickness {
                sink.setStrokeThickness(clampedThickness(thickness))
                hasThickness = true
                lastThickness = thickness
            }
            sink.strokeLine(startX, startY, endX, endY)
            base &+= wordsPerLine
        }
    }
}
