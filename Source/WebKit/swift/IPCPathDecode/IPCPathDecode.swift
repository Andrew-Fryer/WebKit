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

// Decoder for the WebCore::Path arguments of the GPU process's stream messages (StrokePath, FillPath, ClipPath, ...). It
// does what IPC::Decoder::decode<WebCore::Path>() does up to building the C++ objects, which the glue
// (GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp) does from the records this writes.
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

enum IPCPathDecoder {
    /// The UInt32 words of one segment's record (IPCPathDecodeSwiftBoundary.h): word 0 is the alternative index and
    /// the trailing enum (index | enum << 8), then the alternative's floats.
    @inline(__always)
    static var recordWords: Int { 13 }

    /// For each PathSegment::Data alternative index: how many floats it has, and whether a 1-byte enum that must be 0 or
    /// 1 follows them. nil for an index that isn't an alternative (the variant coder fails).
    @inline(__always)
    static func layout(ofAlternative index: UInt8) -> (floats: Int, trailingEnum: Bool)? {
        switch index {
        case 0: return (2, false) // PathMoveTo: point
        case 1: return (2, false) // PathLineTo: point
        case 2: return (4, false) // PathQuadCurveTo: controlPoint, endPoint
        case 3: return (6, false) // PathBezierCurveTo: controlPoint1, controlPoint2, endPoint
        case 4: return (5, false) // PathArcTo: controlPoint1, controlPoint2, radius
        case 5: return (5, true) // PathArc: center, radius, startAngle, endAngle, direction
        case 6: return (5, true) // PathClosedArc: arc (a PathArc)
        case 7: return (7, true) // PathEllipse: center, radiusX, radiusY, rotation, startAngle, endAngle, direction
        case 8: return (4, false) // PathEllipseInRect: rect
        case 9: return (4, false) // PathRect: rect
        case 10: return (12, true) // PathRoundedRect: roundedRect (rect, then 4 radii FloatSizes), strategy
        case 11: return (6, false) // PathContinuousRoundedRect: rect, cornerWidth, cornerHeight
        case 12: return (4, false) // PathDataLine: std::span<const float, 4>
        case 13: return (6, false) // PathDataQuadCurve: start, controlPoint, endPoint
        case 14: return (8, false) // PathDataBezierCurve: start, controlPoint1, controlPoint2, endPoint
        case 15: return (7, false) // PathDataArc: start, controlPoint1, controlPoint2, radius
        case 16: return (0, false) // PathCloseSubpath
        default: return nil
        }
    }

    /// decode<PathSegment>(): one segment into the record at `records[record...]`. Returns false if it fails.
    @inline(__always)
    static func decodeSegment(_ reader: inout IPCArgumentDecoders.Reader, into records: inout MutableSpan<UInt32>, record: Int) -> Bool {
        guard let indexOffset = reader.reserve(byteCount: 1, alignment: 1) else {
            return false
        }
        let index = reader.stream.load(fromByteOffset: indexOffset, as: UInt8.self)
        guard let layout = layout(ofAlternative: index) else {
            return false
        }
        if layout.floats > 0 {
            guard let offset = reader.reserveValues(layout.floats, size: 4) else {
                return false
            }
            // One bounds check for the run and one for the record's words, rather than two per float.
            let values = reader.stream.extracting(offset..<offset &+ 4 &* layout.floats)
            precondition(record &+ 1 &+ layout.floats <= records.count)
            for i in 0..<layout.floats {
                records[record &+ 1 &+ i] = values.load(fromByteOffset: 4 &* i, as: UInt32.self)
            }
        }
        var trailingEnum: UInt8 = 0
        if layout.trailingEnum {
            guard let value = reader.readBoolEnum() else {
                return false
            }
            trailingEnum = value
        }
        records[record] = UInt32(index) | UInt32(trailingEnum) << 8
        return true
    }

    /// The next `count` segments into the first `count` records.
    @inline(__always)
    static func decodeSegments(_ reader: inout IPCArgumentDecoders.Reader, into records: inout MutableSpan<UInt32>, count: Int) -> Int {
        precondition(count >= 0 && count <= records.count / recordWords)
        var record = 0
        for _ in 0..<count {
            guard decodeSegment(&reader, into: &records, record: record) else {
                return IPCArgumentDecodeFailure.invalid.rawValue
            }
            record &+= recordWords
        }
        return reader.result
    }

    /// decode<Path>() up to the first min(size, capacity) segments: the vector's size, stored in `segmentCount`, then
    /// those segments.
    @inline(__always)
    static func decodePath(_ reader: inout IPCArgumentDecoders.Reader, into records: inout MutableSpan<UInt32>, segmentCount: inout UInt64) -> Int {
        guard let sizeOffset = reader.reserveValues(1, size: 8) else {
            return IPCArgumentDecodeFailure.invalid.rawValue
        }
        let size = reader.stream.load(fromByteOffset: sizeOffset, as: UInt64.self)
        // Each segment takes at least its index byte, so with more segments than bytes left the C++ fails at the first
        // segment past the end (after reserving up to 1 MB for the vector). Failing now also bounds what the glue
        // allocates by the bytes in the span.
        let bytesLeft = reader.stream.byteCount &- reader.position
        guard size <= UInt64(bytesLeft) else {
            return IPCArgumentDecodeFailure.invalid.rawValue
        }
        segmentCount = size
        return decodeSegments(&reader, into: &records, count: min(Int(size), records.count / recordWords))
    }

    /// An enum class : bool (ClipPath's WindRule).
    @inline(__always)
    static func decodeBoolEnum(_ reader: inout IPCArgumentDecoders.Reader, value: inout UInt8) -> Int {
        guard let decoded = reader.readBoolEnum() else {
            return IPCArgumentDecodeFailure.invalid.rawValue
        }
        value = decoded
        return reader.result
    }
}
