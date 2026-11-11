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

// Locates the arguments of the hottest IPC stream messages (RemoteGraphicsContext state changes and draws), which the
// GPU process reads from the shared memory that the web process writes. For each argument layout
// (IPCStreamMessageSwiftBoundary.h), locateArguments() does what IPC::Decoder::decode<Message::Arguments>() does for
// those argument types, except building the C++ objects, which the glue
// (GPUProcess/graphics/RemoteGraphicsContextSwiftArgumentDecoders.cpp) does. The stream message decoder
// (IPCStreamMessage.swift) calls it right after the header.
//
// The wire format is that of the C++ coders:
// - Decoder::decodeSpan<T>(count) reads count * sizeof(T) bytes at roundUpToMultipleOf<alignof(T)>(the current
//   address), if alignedBufferIsLargeEnoughToContain(); otherwise the decoder becomes invalid.
// - float, uint32_t (and PackedColor::RGBA, its only member), uint64_t: decodeObject<T>() = decodeSpan<T>(1).
// - FloatPoint, FloatSize, FloatRect, PathArc, ...: their members one after the other (generated coders).
// - std::span<const double, 6> (AffineTransform): decodeSpan<const double>(6), 48 bytes at 8-byte alignment.
// - enum class E : bool: a uint8_t that isValidEnum<E>() accepts, which for bool enums is exactly 0 or 1.
// - ObjectIdentifier: a uint64_t that must be neither 0 nor the HashTable deleted value.
// Every value is aligned in memory, not in the span, which is why the reader takes the address of the span's start
// modulo 8 (baseMisalignment) along with the span.
// The decoders validate and return what the C++ coders validate: the bool enums and the identifiers. Floats, doubles
// and uint32_t accept every bit pattern, so for those the decoders return where they are (the offset of each aligned run
// of them), and the glue copies them out, as Decoder::decodeObject<T>() and decodeSpan<T>() return data[0] and a span.
// The C++ coders decode all arguments before checking, but once a read fails the decoder is invalid and every later
// read fails, so stopping at the first failure has the same outcome.

/// The failures of a decoder. The raw values are part of the C boundary of the Path decoder (IPCPathDecodeSwiftBoundary.h).
enum IPCArgumentDecodeFailure: Int {
    /// A value didn't fit in the span or failed validation. The C++ glue marks the decoder invalid.
    case invalid = -1
    /// An ObjectIdentifier was 0 or the deleted value. The C++ glue also sets the IPC_TESTING_API error string that
    /// the generated ObjectIdentifier coders set, then marks the decoder invalid.
    case invalidIdentifier = -2
}

/// WebKitSwift_IPCStreamMessage_Arguments::status. The values are part of the C boundary (IPCStreamMessageSwiftBoundary.h).
enum IPCStreamArgumentsStatus: UInt8 {
    case none = 0
    case valid = 1
    case invalid = 2
    case invalidIdentifier = 3
}

/// WebKitSwift_IPCStreamMessage_Arguments: where the arguments are, and their validated values.
struct IPCStreamArguments {
    var identifier: UInt64 = 0
    var valuesOffset: Int = 0
    var endOffset: Int = 0
    var status: IPCStreamArgumentsStatus = .none
    var flag: UInt8 = 0

    @inline(__always)
    static func failed(_ failure: IPCArgumentDecodeFailure) -> IPCStreamArguments {
        IPCStreamArguments(status: failure == .invalid ? .invalid : .invalidIdentifier)
    }
}

enum IPCArgumentDecoders {
    /// A cursor over the arguments of one message: `stream` is the decoder's whole span, `position` its position.
    struct Reader: ~Escapable {
        let stream: RawSpan
        let baseMisalignment: Int
        var position: Int

        @_lifetime(copy stream)
        @inline(__always)
        init(_ stream: RawSpan, baseMisalignment: Int, position: Int) {
            self.stream = stream
            self.baseMisalignment = baseMisalignment
            self.position = position
        }

        /// Decoder::decodeSpan<T>() for count * sizeof(T) == byteCount and alignof(T) == alignment: the offset of the
        /// bytes, which are then consumed, or nil if they don't fit.
        @inline(__always)
        mutating func reserve(byteCount: Int, alignment: Int) -> Int? {
            let offset = ipcAlignedOffset(position, alignment: alignment, baseMisalignment: baseMisalignment)
            guard ipcBufferIsLargeEnoughToContain(stream.byteCount, offset, byteCount) else {
                return nil
            }
            position = offset &+ byteCount
            return offset
        }

        /// `count` > 0 consecutive values of `size` (4 or 8) bytes, each decoded with decodeObject<T>() (or all with
        /// decodeSpan<T>(count)), for types whose alignment is their size. After the first, the position stays
        /// aligned, so together they are one aligned run. Returns the run's offset, or nil if it doesn't fit.
        @inline(__always)
        mutating func reserveValues(_ count: Int, size: Int) -> Int? {
            precondition(count > 0 && count <= Int.max / 8 && (size == 4 || size == 8))
            return reserve(byteCount: count &* size, alignment: size)
        }

        /// ArgumentCoder<E> for an enum class E : bool: a uint8_t that must be 0 or 1.
        @inline(__always)
        mutating func readBoolEnum() -> UInt8? {
            guard let offset = reserve(byteCount: 1, alignment: 1) else {
                return nil
            }
            let value = stream.load(fromByteOffset: offset, as: UInt8.self)
            return value <= 1 ? value : nil
        }

        /// The generated ArgumentCoder<ObjectIdentifierGeneric<...>>: a uint64_t that isValidIdentifier() accepts.
        /// Returns nil and stores the identifier on success.
        @inline(__always)
        mutating func readIdentifier(into identifier: inout UInt64) -> IPCArgumentDecodeFailure? {
            guard let offset = reserve(byteCount: 8, alignment: 8) else {
                return .invalid
            }
            let value = stream.load(fromByteOffset: offset, as: UInt64.self)
            guard isValidIPCIdentifier(value) else {
                return .invalidIdentifier
            }
            identifier = value
            return nil
        }
    }

    /// decode<Message::Arguments>() for the arguments of `layout` (IPCStreamMessageSwiftBoundary.h), from the reader's
    /// position: their offsets and validated values, or the failure. A layout this doesn't know is None.
    @inline(__always)
    static func locateArguments(_ reader: inout Reader, layout: UInt8) -> IPCStreamArguments {
        var arguments = IPCStreamArguments()
        switch layout {
        case 1...15:
            // float, uint32_t, PackedColor::RGBA, and the FloatPoint, FloatSize, FloatRect, PathDataQuadCurve and
            // PathDataBezierCurve structs, which are all floats.
            guard let offset = reader.reserveValues(Int(layout), size: 4) else {
                return .failed(.invalid)
            }
            arguments.valuesOffset = offset
        case 17...31:
            // Those, then an enum class : bool (PathArc, PathClosedArc, tuple<FloatRect, RequiresClipToRect>).
            guard let offset = reader.reserveValues(Int(layout &- 16), size: 4), let flag = reader.readBoolEnum() else {
                return .failed(.invalid)
            }
            arguments.valuesOffset = offset
            arguments.flag = flag
        case 32:
            // tuple<RemoteGradientIdentifier, AffineTransform>: 7 aligned uint64_t-size values. When they all fit, one check
            // covers them; otherwise decode them one by one, which tells which value fails.
            var all = reader
            if let offset = all.reserve(byteCount: 56, alignment: 8) {
                let identifier = all.stream.load(fromByteOffset: offset, as: UInt64.self)
                guard isValidIPCIdentifier(identifier) else {
                    return .failed(.invalidIdentifier)
                }
                reader = all
                arguments.identifier = identifier
                arguments.valuesOffset = offset &+ 8
                break
            }
            if let failure = reader.readIdentifier(into: &arguments.identifier) {
                return .failed(failure)
            }
            guard let offset = reader.reserveValues(6, size: 8) else {
                return .failed(.invalid)
            }
            arguments.valuesOffset = offset
        case 33:
            // tuple<AffineTransform, RemotePathImplIdentifier, WindRule>.
            guard let offset = reader.reserveValues(6, size: 8) else {
                return .failed(.invalid)
            }
            if let failure = reader.readIdentifier(into: &arguments.identifier) {
                return .failed(failure)
            }
            guard let flag = reader.readBoolEnum() else {
                return .failed(.invalid)
            }
            arguments.valuesOffset = offset
            arguments.flag = flag
        default:
            return arguments
        }
        arguments.endOffset = reader.position
        arguments.status = .valid
        return arguments
    }
}
