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

// The IPC encoder of WebCore::Path, WebCore::PathImpl and WebCore::PathSegment (IPCPathEncodeSwiftBoundary.h): what
// ArgumentCoder<Path>::encode() and the coders under it did in C++, for both IPC::StreamConnectionEncoder (the web
// process's stream messages to the GPU process: StrokePath, FillPath, ClipPath, ...) and IPC::Encoder (out-of-stream
// messages, display lists, remote layer tree transactions, CachePathImpl). The glue (Shared/PathArgumentCoders.cpp)
// passes the PathSegments in place, as 32-bit words.
//
// The wire format is that of the C++ coders, which IPCPathDecode.swift decodes:
// - ArgumentCoder<Path> and ArgumentCoder<PathImpl> (generated): their segments(), a Vector<PathSegment>.
// - VectorArgumentCoder<false, PathSegment>: the size as a uint64_t, then each PathSegment.
// - ArgumentCoder<PathSegment> (generated): its PathSegment::Data.
// - ArgumentCoder<Variant<...>>: the index as a uint8_t (EncodedVariantIndex), then the alternative.
// - The 17 alternatives (generated coders, PathSegment.serialization.in): their members in order. All are floats
//   (FloatPoint, FloatSize, FloatRect and FloatRoundedRect are floats too, encoded member by member, in the order they
//   are in memory), except for one trailing 1-byte enum in PathArc, PathClosedArc (a PathArc) and PathEllipse
//   (RotationDirection, an enum class : bool, encoded as its uint8_t 0 or 1) and in PathRoundedRect
//   (PathRoundedRect::Strategy : uint8_t). PathDataLine is a std::span<const float, 4>, encodeSpan<const float>(),
//   the same 16 bytes at 4-byte alignment as four floats.
// - Each value is encodeObject<T>() (encodeSpan<T>() of one), aligned to its size: in memory for
//   StreamConnectionEncoder (baseMisalignment is the buffer's address modulo 8), in the body for IPC::Encoder
//   (baseMisalignment is the body size modulo 8). ipcAlignedOffset() (IPCStreamHeader.swift) is shared with the
//   decoders.
// - StreamConnectionEncoder doesn't write the alignment gaps, and becomes invalid at the first value that doesn't fit.
//   The position after each value only grows, so the values all fit if and only if the last one does, and stopping at
//   the first that doesn't has the same outcome. IPC::Encoder grows its body for each value and zeroes the gaps.
// Floats are copied as their bit patterns, as the C++ copies them (memcpySpan), so NaN payloads are kept.

enum IPCPathEncoder {
    /// sizeof(WebCore::PathSegment) in 32-bit words.
    @inline(__always)
    static var segmentWordCount: Int { 14 }

    /// The word of a PathSegment whose low byte is the PathSegment::Data variant index (byte 52).
    @inline(__always)
    static var indexWord: Int { 13 }

    /// The number of alternatives of PathSegment::Data: a valid index is less than this.
    @inline(__always)
    static var alternativeCount: UInt32 { 17 }

    /// The segments as words, checked against their count, which a non-Path (a single PathSegment) requires to be 1.
    @inline(__always)
    static func checkSegments(_ segments: Span<UInt32>, segmentCount: Int, withCount: Bool) {
        precondition(segmentCount >= 0 && segmentCount <= Int.max / (4 &* segmentWordCount)
            && segments.count == segmentCount &* segmentWordCount, "PathSegment words don't match the segment count")
        precondition(withCount || segmentCount == 1, "a PathSegment is encoded one at a time")
    }

    /// The variant index of the segment whose first word is `word`. Every PathSegment::Data holds an alternative.
    @inline(__always)
    static func variantIndex(_ segments: Span<UInt32>, _ word: Int) -> UInt32 {
        let index = segments[word &+ indexWord] & 0xff
        precondition(index < alternativeCount, "PathSegment::Data variant index out of range")
        return index
    }

    /// The number of floats of alternative `index` (< alternativeCount), and whether a 1-byte enum follows them. This
    /// is the table that encode()'s switch unrolls, one case per shape.
    @inline(__always)
    static func shape(_ index: UInt32) -> (floats: Int, trailingEnum: Bool) {
        switch index {
        case 0, 1: (2, false) // PathMoveTo, PathLineTo: point
        case 2, 8, 9, 12: (4, false) // PathQuadCurveTo: controlPoint, endPoint; PathEllipseInRect, PathRect: rect; PathDataLine
        case 3, 11, 13: (6, false) // PathBezierCurveTo: 3 points; PathContinuousRoundedRect: rect, cornerWidth, cornerHeight; PathDataQuadCurve
        case 4: (5, false) // PathArcTo: controlPoint1, controlPoint2, radius
        case 5, 6: (5, true) // PathArc: center, radius, startAngle, endAngle, direction; PathClosedArc: arc
        case 7: (7, true) // PathEllipse: center, radiusX, radiusY, rotation, startAngle, endAngle, direction
        case 10: (12, true) // PathRoundedRect: roundedRect (rect, then the topLeft, topRight, bottomLeft, bottomRight FloatSizes), strategy
        case 14: (8, false) // PathDataBezierCurve: start, controlPoint1, controlPoint2, endPoint
        case 15: (7, false) // PathDataArc: start, controlPoint1, controlPoint2, radius
        default: (0, false) // PathCloseSubpath
        }
    }

    /// The bytes that encoding the segments takes from a position whose misalignment is `baseMisalignment`.
    static func encodedSize(_ segments: Span<UInt32>, segmentCount: Int, withCount: Bool, baseMisalignment: Int) -> Int {
        checkSegments(segments, segmentCount: segmentCount, withCount: withCount)
        var position = 0
        if withCount {
            position = ipcAlignedOffset(0, alignment: 8, baseMisalignment: baseMisalignment) &+ 8
        }
        var word = 0
        for _ in 0..<segmentCount {
            let (floats, trailingEnum) = shape(variantIndex(segments, word))
            position &+= 1
            if floats > 0 {
                position = ipcAlignedOffset(position, alignment: 4, baseMisalignment: baseMisalignment) &+ 4 &* floats
            }
            if trailingEnum {
                position &+= 1
            }
            word &+= segmentWordCount
        }
        return position
    }

    /// Encodes the segments at the start of `out`, as the coders do: returns the number of bytes encoded, or nil if
    /// they don't fit in `out`. With `zeroGaps`, writes the alignment gaps as 0 (IPC::Encoder); otherwise leaves them
    /// alone (StreamConnectionEncoder).
    @inline(always)
    static func encode(_ segments: Span<UInt32>, segmentCount: Int, withCount: Bool, into out: inout MutableRawSpan,
                       baseMisalignment: Int, zeroGaps: Bool) -> Int? {
        checkSegments(segments, segmentCount: segmentCount, withCount: withCount)
        var position = 0
        if withCount {
            let offset = ipcAlignedOffset(0, alignment: 8, baseMisalignment: baseMisalignment)
            guard out.byteCount &- offset >= 8 else {
                return nil
            }
            if zeroGaps {
                zero(&out, from: 0, to: offset)
            }
            out.storeBytes(of: UInt64(truncatingIfNeeded: segmentCount), toByteOffset: offset, as: UInt64.self)
            position = offset &+ 8
        }
        var word = 0
        for _ in 0..<segmentCount {
            let index = variantIndex(segments, word)
            let encoded: Bool
            // One case per shape (see shape()), so that each copies a constant number of floats.
            switch index {
            case 0, 1:
                encoded = encodeAlternative(index, floats: 2, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 2, 8, 9, 12:
                encoded = encodeAlternative(index, floats: 4, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 3, 11, 13:
                encoded = encodeAlternative(index, floats: 6, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 4:
                encoded = encodeAlternative(index, floats: 5, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 5, 6:
                encoded = encodeAlternative(index, floats: 5, trailingEnum: true, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 7:
                encoded = encodeAlternative(index, floats: 7, trailingEnum: true, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 10:
                encoded = encodeAlternative(index, floats: 12, trailingEnum: true, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 14:
                encoded = encodeAlternative(index, floats: 8, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            case 15:
                encoded = encodeAlternative(index, floats: 7, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            default:
                encoded = encodeAlternative(index, floats: 0, trailingEnum: false, segments, word, &out, &position, baseMisalignment, zeroGaps)
            }
            guard encoded else {
                return nil
            }
            word &+= segmentWordCount
        }
        return position
    }

    /// One segment, whose first word is `word`, at `position` of `out`: its index byte, its `floats` floats (words 0
    /// on) at 4-byte alignment, and, if `trailingEnum`, the low byte of the word after them. Advances `position`, or
    /// returns false if the segment doesn't fit.
    @inline(always)
    static func encodeAlternative(_ index: UInt32, floats: Int, trailingEnum: Bool, _ segments: Span<UInt32>, _ word: Int,
                                  _ out: inout MutableRawSpan, _ position: inout Int, _ baseMisalignment: Int, _ zeroGaps: Bool) -> Bool {
        let capacity = out.byteCount
        var next = position
        guard next < capacity else {
            return false
        }
        out.storeBytes(of: UInt8(truncatingIfNeeded: index), toByteOffset: next, as: UInt8.self)
        next &+= 1
        if floats > 0 {
            let offset = ipcAlignedOffset(next, alignment: 4, baseMisalignment: baseMisalignment)
            guard capacity &- offset >= 4 &* floats else {
                return false
            }
            if zeroGaps {
                zero(&out, from: next, to: offset)
            }
            // One bounds check for the words rather than one per float.
            let values = segments.extracting(word..<word &+ floats)
            for i in 0..<floats {
                out.storeBytes(of: values[i], toByteOffset: offset &+ 4 &* i, as: UInt32.self)
            }
            next = offset &+ 4 &* floats
        }
        if trailingEnum {
            guard next < capacity else {
                return false
            }
            out.storeBytes(of: UInt8(truncatingIfNeeded: segments[word &+ floats]), toByteOffset: next, as: UInt8.self)
            next &+= 1
        }
        position = next
        return true
    }

    /// Writes 0 to the bytes start..<end (at most 7) of `out`.
    @inline(__always)
    static func zero(_ out: inout MutableRawSpan, from start: Int, to end: Int) {
        for i in start..<end {
            out.storeBytes(of: 0, toByteOffset: i, as: UInt8.self)
        }
    }

    /// WebKitSwift_IPCPathEncode_encodeToStream(): returns the bytes encoded, or -1.
    @inline(__always)
    static func encodeToStream(_ segments: Span<UInt32>, segmentCount: Int, withCount: Bool, into out: inout MutableRawSpan,
                               baseMisalignment: Int) -> Int {
        encode(segments, segmentCount: segmentCount, withCount: withCount, into: &out, baseMisalignment: baseMisalignment, zeroGaps: false) ?? -1
    }

    /// WebKitSwift_IPCPathEncode_encodeToBody(): fills all of `out`, which must be exactly encodedSize() bytes.
    static func encodeToBody(_ segments: Span<UInt32>, segmentCount: Int, withCount: Bool, into out: inout MutableRawSpan,
                             baseMisalignment: Int) {
        let encoded = encode(segments, segmentCount: segmentCount, withCount: withCount, into: &out, baseMisalignment: baseMisalignment, zeroGaps: true)
        precondition(encoded == out.byteCount, "IPC::Encoder grew by a size other than the Path's encoded size")
    }
}
