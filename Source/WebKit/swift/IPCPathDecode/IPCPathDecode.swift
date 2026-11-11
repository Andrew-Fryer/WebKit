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

// Decoder for the WebCore::Path arguments of the GPU process's stream messages (StrokePath, FillPath, ClipPath, ...). It
// does what IPC::Decoder::decode<WebCore::Path>() does up to building the C++ objects, which the glue
// (GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp) does from the words this writes
// (WebKitSwift_IPCStreamMessage_decodePath() in IPCStreamMessageSwiftBoundary.h documents them).
//
// The wire format is that of the C++ coders (IPCArgDecoders.swift describes the primitives, which use its Reader):
// - ArgumentCoder<Path> (generated): Vector<PathSegment>.
// - VectorArgumentCoder<false, PathSegment>: a uint64_t size, then that many PathSegments, each decoded with
//   decode<PathSegment>(); the first that fails makes the whole vector fail.
// - ArgumentCoder<PathSegment> (generated): its PathSegment::Data.
// - ArgumentCoder<Variant<...>>: a uint8_t index (EncodedVariantIndex), which must be < 17, then the alternative.
// - The 17 alternatives (generated coders, PathSegment.serialization.in): their members in order, all of which are
//   floats (FloatPoint, FloatSize, FloatRect and FloatRoundedRect are floats too), except for one trailing 1-byte enum in
//   PathArc, PathClosedArc and PathEllipse (RotationDirection, an enum class : bool) and in PathRoundedRect
//   (PathRoundedRect::Strategy : uint8_t, whose isValidEnum<>() accepts exactly PreferNative = 0 and PreferBezier = 1).
//   PathDataLine is a std::span<const float, 4>, decodeSpan<const float>(4), which reads the same 16 bytes at 4-byte
//   alignment as four floats. Each float is decodeObject<float>(), 4 bytes at 4-byte alignment, so after the first
//   one, an alternative's floats are one aligned run.
// A failure anywhere fails the Path and leaves the decoder invalid, so the decoder stops at the first one.
// ClipPath's Path is followed by an enum class : bool (WindRule), which the call that decodes the last segments also
// decodes, so that a message takes one call into Swift (for up to as many segments as fit in the caller's buffer).

enum IPCPathDecoder {
    /// The number of 64-bit words of WebKitSwift_IPCStreamMessage_PathProgress: segmentCount, segmentsDecoded,
    /// wordCount, boolEnum.
    @inline(__always)
    static var progressWordCount: Int { 4 }

    /// The most words that one segment takes: its header word and the 12 floats of PathRoundedRect. A call decodes
    /// segments while that many words are left in the caller's buffer.
    @inline(__always)
    static var maximumSegmentWords: Int { 13 }

    /// decode<PathSegment>(): one segment into `words[start...]`, which has room for maximumSegmentWords: its header word
    /// (index | trailing enum << 8), then its floats. Returns the number of words written, or nil if decoding failed
    /// (an index that isn't an alternative fails the variant coder). Each alternative's floats are copied with a
    /// constant count, so that the copy is unrolled.
    @inline(__always)
    static func decodeSegment(_ reader: inout IPCArgumentDecoders.Reader, into words: inout MutableSpan<UInt32>, at start: Int) -> Int? {
        guard let indexOffset = reader.reserve(byteCount: 1, alignment: 1) else {
            return nil
        }
        let index = reader.stream.load(fromByteOffset: indexOffset, as: UInt8.self)
        switch index {
        case 0, 1: // PathMoveTo, PathLineTo: point
            return decodeAlternative(index, floats: 2, trailingEnum: false, &reader, into: &words, at: start)
        case 2, 8, 9, 12: // PathQuadCurveTo: controlPoint, endPoint; PathEllipseInRect, PathRect: rect; PathDataLine: std::span<const float, 4>
            return decodeAlternative(index, floats: 4, trailingEnum: false, &reader, into: &words, at: start)
        case 3, 11, 13: // PathBezierCurveTo: 3 points; PathContinuousRoundedRect: rect, cornerWidth, cornerHeight; PathDataQuadCurve: 3 points
            return decodeAlternative(index, floats: 6, trailingEnum: false, &reader, into: &words, at: start)
        case 4: // PathArcTo: controlPoint1, controlPoint2, radius
            return decodeAlternative(index, floats: 5, trailingEnum: false, &reader, into: &words, at: start)
        case 5, 6: // PathArc: center, radius, startAngle, endAngle, direction; PathClosedArc: arc (a PathArc)
            return decodeAlternative(index, floats: 5, trailingEnum: true, &reader, into: &words, at: start)
        case 7: // PathEllipse: center, radiusX, radiusY, rotation, startAngle, endAngle, direction
            return decodeAlternative(index, floats: 7, trailingEnum: true, &reader, into: &words, at: start)
        case 10: // PathRoundedRect: roundedRect (rect, then 4 radii FloatSizes), strategy
            return decodeAlternative(index, floats: 12, trailingEnum: true, &reader, into: &words, at: start)
        case 14: // PathDataBezierCurve: start, controlPoint1, controlPoint2, endPoint
            return decodeAlternative(index, floats: 8, trailingEnum: false, &reader, into: &words, at: start)
        case 15: // PathDataArc: start, controlPoint1, controlPoint2, radius
            return decodeAlternative(index, floats: 7, trailingEnum: false, &reader, into: &words, at: start)
        case 16: // PathCloseSubpath
            return decodeAlternative(index, floats: 0, trailingEnum: false, &reader, into: &words, at: start)
        default:
            return nil
        }
    }

    /// The rest of decodeSegment() for alternative `index`, which has `floats` floats (at most 12), then a 1-byte enum
    /// that must be 0 or 1 if `trailingEnum`.
    @inline(__always)
    static func decodeAlternative(_ index: UInt8, floats: Int, trailingEnum: Bool, _ reader: inout IPCArgumentDecoders.Reader,
                                  into words: inout MutableSpan<UInt32>, at start: Int) -> Int? {
        if floats > 0 {
            guard let offset = reader.reserveValues(floats, size: 4) else {
                return nil
            }
            // One bounds check for the run and one for the words, rather than two per float.
            let values = reader.stream.extracting(offset..<offset &+ 4 &* floats)
            precondition(start >= 0 && start &+ 1 &+ floats <= words.count)
            for i in 0..<floats {
                words[start &+ 1 &+ i] = values.load(fromByteOffset: 4 &* i, as: UInt32.self)
            }
        }
        var enumValue: UInt8 = 0
        if trailingEnum {
            guard let value = reader.readBoolEnum() else {
                return nil
            }
            enumValue = value
        }
        words[start] = UInt32(index) | UInt32(enumValue) &<< 8
        return 1 &+ floats
    }

    /// WebKitSwift_IPCStreamMessage_decodePath(): decode<Path>() from `position` of `stream`, one bufferful of segments
    /// per call, with `progress` (WebKitSwift_IPCStreamMessage_PathProgress) carried from one call to the next.
    @inline(__always)
    static func decodePath(_ stream: RawSpan, baseMisalignment: Int, position: Int, into words: inout MutableSpan<UInt32>,
                           trailingBoolEnum: Bool, progress: inout MutableSpan<UInt64>) -> Int {
        // A decoder's position is never past the end of its span.
        precondition(position >= 0 && position <= stream.byteCount, "IPC decoder position outside its span")
        var reader = IPCArgumentDecoders.Reader(stream, baseMisalignment: baseMisalignment, position: position)
        var segmentCount = progress[0]
        var decoded = progress[1]
        if decoded == 0 {
            guard let sizeOffset = reader.reserveValues(1, size: 8) else {
                return IPCArgumentDecodeFailure.invalid.rawValue
            }
            segmentCount = reader.stream.load(fromByteOffset: sizeOffset, as: UInt64.self)
            // Each segment takes at least its index byte, so with more segments than bytes left the C++ fails at the
            // first segment past the end (after reserving up to 1 MB for the vector). Failing now also bounds what the
            // glue allocates by the bytes in the span.
            guard segmentCount <= UInt64(reader.stream.byteCount &- reader.position) else {
                return IPCArgumentDecodeFailure.invalid.rawValue
            }
            progress[0] = segmentCount
        }
        // The caller calls again while segments are left, so each call must have room for one.
        let lastStart = words.count &- maximumSegmentWords
        precondition(lastStart >= 0, "IPC path words buffer smaller than a segment")
        var wordCount = 0
        while decoded < segmentCount && wordCount <= lastStart {
            guard let segmentWords = decodeSegment(&reader, into: &words, at: wordCount) else {
                return IPCArgumentDecodeFailure.invalid.rawValue
            }
            wordCount &+= segmentWords
            decoded &+= 1
        }
        var boolEnum: UInt8 = 0
        if trailingBoolEnum && decoded == segmentCount {
            guard let value = reader.readBoolEnum() else {
                return IPCArgumentDecodeFailure.invalid.rawValue
            }
            boolEnum = value
        }
        progress[1] = decoded
        progress[2] = UInt64(truncatingIfNeeded: wordCount)
        progress[3] = UInt64(boolEnum)
        return reader.position
    }
}
