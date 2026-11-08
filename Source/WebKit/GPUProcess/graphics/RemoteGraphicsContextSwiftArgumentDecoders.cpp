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

#include "config.h"

#if ENABLE(GPU_PROCESS) && USE(SWIFT_KERNELS)
#include "RemoteGraphicsContextMessages.h"

#include "Decoder.h"
#include "IPCArgDecodersSwiftBoundary.h"
#include "RemoteGradientIdentifier.h"
#include <WebCore/AffineTransform.h>
#include <WebCore/ColorTypes.h>
#include <WebCore/FloatRect.h>
#include <WebCore/GraphicsContext.h>
#include <WebCore/PathSegmentData.h>
#include <WebCore/RotationDirection.h>
#include <array>
#include <bit>
#include <span>
#include <wtf/StdLibExtras.h>

#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#include <wtf/Vector.h>
#endif

// The arguments of the RemoteGraphicsContext messages that have the SwiftArgumentDecoder attribute are decoded in Swift
// (Source/WebKit/swift/IPCArgDecoders), by the decodeArgumentsInSwift() functions defined at the end of this file, which
// IPC::handleMessage() calls instead of the original decoder.decode<Message::Arguments>() (HandleMessage.h). The C++
// here only reads the decoder's span and position, builds the C++ arguments from the decoded values, and moves the
// decoder past them or marks it invalid, as the original does.
// With ENABLE(SWIFT_KERNELS_VALIDATION) and WEBKIT_SWIFT_KERNELS_VALIDATE=1, every message is decoded both ways and
// compared (decodeArgumentsAndValidate()).
// The attribute is only on messages whose decoding is worth a call: the arguments of Translate and of the cached-gradient
// and curve messages are decoded out of line in Production anyway, and those of SetStrokePackedColorAndThickness and of
// the arc and rect draws decode faster in Swift than inline in C++, because the generated coders check every field
// separately. The messages with a single float, color or identifier, or with just an AffineTransform, decode in about a
// nanosecond inline in RemoteGraphicsContext::didReceiveStreamMessage(), which is less than the call into Swift costs.

namespace WebKit {
namespace SwiftArgumentDecoders {

using WebCore::AffineTransform;
using WebCore::FloatPoint;
using WebCore::FloatRect;
using WebCore::FloatSize;

// The Swift decoders (IPCArgDecodersSwiftBoundary.h), each with what it outputs, and the number of argument bytes it can
// read at most (from the decoder's position, alignment padding included). The values that need no validation (floats,
// doubles, uint32_t) are copied from the span at the offsets the decoder returns, as decodeObject<T>() copies data[0].
// rebase() moves those offsets to another span holding the same bytes at another offset (the validation snapshot).

template<size_t count, typename T>
std::array<T, count> copyValues(std::span<const uint8_t> stream, size_t offset)
{
    std::array<T, count> values;
    memcpySpan(asMutableByteSpan(values), stream.subspan(offset, sizeof(values)));
    return values;
}

template<size_t wordCount>
struct Words {
    static_assert(wordCount > 0);
    size_t wordsOffset;

    static constexpr size_t maximumReadSize = 3 + 4 * wordCount;
    static intptr_t decode(std::span<const uint8_t> stream, size_t position, Words& d)
    {
        return WebKitSwift_IPCArgDecoders_decodeWords(stream.data(), stream.size(), position, wordCount, &d.wordsOffset);
    }
    std::array<uint32_t, wordCount> words(std::span<const uint8_t> stream) const { return copyValues<wordCount, uint32_t>(stream, wordsOffset); }
    void rebase(size_t from, size_t to) { wordsOffset = wordsOffset - from + to; }
};

template<size_t wordCount>
struct WordsAndBoolEnum {
    static_assert(wordCount > 0);
    size_t wordsOffset;
    uint8_t flag;

    static constexpr size_t maximumReadSize = 3 + 4 * wordCount + 1;
    static intptr_t decode(std::span<const uint8_t> stream, size_t position, WordsAndBoolEnum& d)
    {
        return WebKitSwift_IPCArgDecoders_decodeWordsAndBoolEnum(stream.data(), stream.size(), position, wordCount, &d.wordsOffset, &d.flag);
    }
    std::array<uint32_t, wordCount> words(std::span<const uint8_t> stream) const { return copyValues<wordCount, uint32_t>(stream, wordsOffset); }
    void rebase(size_t from, size_t to) { wordsOffset = wordsOffset - from + to; }
};

struct IdentifierAndMatrix {
    uint64_t identifier;
    size_t matrixOffset;

    static constexpr size_t maximumReadSize = 7 + 8 + 48;
    static intptr_t decode(std::span<const uint8_t> stream, size_t position, IdentifierAndMatrix& d)
    {
        return WebKitSwift_IPCArgDecoders_decodeIdentifierAndAffineTransform(stream.data(), stream.size(), position, &d.identifier, &d.matrixOffset);
    }
    std::array<double, 6> matrix(std::span<const uint8_t> stream) const { return copyValues<6, double>(stream, matrixOffset); }
    void rebase(size_t from, size_t to) { matrixOffset = matrixOffset - from + to; }
};

// Arguments<std::tuple<...>>: Decoded is the Swift decoder, make() builds the C++ arguments from its results as the
// generated coders do, and bits() is their bit pattern (to validate). The bool enums are built from a 0 or 1, which is
// what isValidEnum() accepts for every enum class : bool.

struct Bits {
    std::array<uint64_t, 10> values { };
    size_t size { 0 };
    void append(uint64_t value) { values[size++] = value; }
    void append(float value) { append(static_cast<uint64_t>(std::bit_cast<uint32_t>(value))); }
    void append(double value) { append(std::bit_cast<uint64_t>(value)); }
    void append(const FloatPoint& point) { append(point.x()); append(point.y()); }
    void append(const AffineTransform& transform) { append(transform.a()); append(transform.b()); append(transform.c()); append(transform.d()); append(transform.e()); append(transform.f()); }
    void append(WebCore::PackedColor::RGBA color) { append(static_cast<uint64_t>(color.value)); }
    template<typename Enum> requires std::is_enum_v<Enum> void append(Enum value)
    {
        static_assert(std::is_same_v<std::underlying_type_t<Enum>, bool>);
        append(static_cast<uint64_t>(value));
    }
    void append(RemoteGradientIdentifier identifier) { append(identifier.toUInt64()); }
    bool operator==(const Bits&) const = default;
};

inline float toFloat(uint32_t bits) { return std::bit_cast<float>(bits); }
inline FloatPoint toFloatPoint(uint32_t x, uint32_t y) { return { toFloat(x), toFloat(y) }; }
template<typename Enum> Enum toBoolEnum(uint8_t value)
{
    static_assert(std::is_same_v<std::underlying_type_t<Enum>, bool>);
    return static_cast<Enum>(value);
}
inline AffineTransform toAffineTransform(const std::array<double, 6>& matrix) { return AffineTransform { std::span<const double, 6> { matrix } }; }

template<typename> struct Arguments;

template<> struct Arguments<std::tuple<float, float>> {
    using Decoded = Words<2>;
    static std::tuple<float, float> make(const Decoded& d, std::span<const uint8_t> s)
    {
        auto w = d.words(s);
        return { toFloat(w[0]), toFloat(w[1]) };
    }
    static void bits(Bits& b, const std::tuple<float, float>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PackedColor::RGBA, float>> {
    using Decoded = Words<2>;
    static std::tuple<WebCore::PackedColor::RGBA, float> make(const Decoded& d, std::span<const uint8_t> s)
    {
        auto w = d.words(s);
        return { WebCore::PackedColor::RGBA { w[0] }, toFloat(w[1]) };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PackedColor::RGBA, float>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PathDataQuadCurve>> {
    using Decoded = Words<6>;
    static std::tuple<WebCore::PathDataQuadCurve> make(const Decoded& d, std::span<const uint8_t> s)
    {
        auto w = d.words(s);
        return { WebCore::PathDataQuadCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]) } };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PathDataQuadCurve>& a, std::span<const uint8_t>)
    {
        auto& c = std::get<0>(a);
        b.append(c.start); b.append(c.controlPoint); b.append(c.endPoint);
    }
};

template<> struct Arguments<std::tuple<WebCore::PathDataBezierCurve>> {
    using Decoded = Words<8>;
    static std::tuple<WebCore::PathDataBezierCurve> make(const Decoded& d, std::span<const uint8_t> s)
    {
        auto w = d.words(s);
        return { WebCore::PathDataBezierCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]), toFloatPoint(w[6], w[7]) } };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PathDataBezierCurve>& a, std::span<const uint8_t>)
    {
        auto& c = std::get<0>(a);
        b.append(c.start); b.append(c.controlPoint1); b.append(c.controlPoint2); b.append(c.endPoint);
    }
};

inline WebCore::PathArc toPathArc(const WordsAndBoolEnum<5>& d, std::span<const uint8_t> s)
{
    auto w = d.words(s);
    return { toFloatPoint(w[0], w[1]), toFloat(w[2]), toFloat(w[3]), toFloat(w[4]), toBoolEnum<WebCore::RotationDirection>(d.flag) };
}
inline void appendPathArc(Bits& b, const WebCore::PathArc& arc)
{
    b.append(arc.center); b.append(arc.radius); b.append(arc.startAngle); b.append(arc.endAngle); b.append(arc.direction);
}

template<> struct Arguments<std::tuple<WebCore::PathArc>> {
    using Decoded = WordsAndBoolEnum<5>;
    static std::tuple<WebCore::PathArc> make(const Decoded& d, std::span<const uint8_t> s) { return { toPathArc(d, s) }; }
    static void bits(Bits& b, const std::tuple<WebCore::PathArc>& a, std::span<const uint8_t>) { appendPathArc(b, std::get<0>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PathClosedArc>> {
    using Decoded = WordsAndBoolEnum<5>;
    static std::tuple<WebCore::PathClosedArc> make(const Decoded& d, std::span<const uint8_t> s) { return { WebCore::PathClosedArc { toPathArc(d, s) } }; }
    static void bits(Bits& b, const std::tuple<WebCore::PathClosedArc>& a, std::span<const uint8_t>) { appendPathArc(b, std::get<0>(a).arc); }
};

template<> struct Arguments<std::tuple<FloatRect, WebCore::RequiresClipToRect>> {
    using Decoded = WordsAndBoolEnum<4>;
    static std::tuple<FloatRect, WebCore::RequiresClipToRect> make(const Decoded& d, std::span<const uint8_t> s)
    {
        auto w = d.words(s);
        return { FloatRect { toFloatPoint(w[0], w[1]), FloatSize { toFloat(w[2]), toFloat(w[3]) } }, toBoolEnum<WebCore::RequiresClipToRect>(d.flag) };
    }
    static void bits(Bits& b, const std::tuple<FloatRect, WebCore::RequiresClipToRect>& a, std::span<const uint8_t>)
    {
        auto& rect = std::get<0>(a);
        b.append(rect.x()); b.append(rect.y()); b.append(rect.width()); b.append(rect.height()); b.append(std::get<1>(a));
    }
};

template<> struct Arguments<std::tuple<RemoteGradientIdentifier, AffineTransform>> {
    using Decoded = IdentifierAndMatrix;
    static std::tuple<RemoteGradientIdentifier, AffineTransform> make(const Decoded& d, std::span<const uint8_t> s)
    {
        return { RemoteGradientIdentifier { d.identifier }, toAffineTransform(d.matrix(s)) };
    }
    static void bits(Bits& b, const std::tuple<RemoteGradientIdentifier, AffineTransform>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

ALWAYS_INLINE void markInvalid(IPC::Decoder& decoder, intptr_t failure)
{
#if ENABLE(IPC_TESTING_API)
    if (failure == WebKitSwift_IPCArgDecoders_InvalidIdentifier)
        decoder.setErrorString("Validation failed: WTF::ObjectIdentifierGenericBase::isValidIdentifier(*toUInt64)"_s);
#else
    UNUSED_PARAM(failure);
#endif
    decoder.markInvalid();
}

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// The stream is shared memory that the sending process can rewrite at any time, so a second read could see other bytes
// than the first. Both decoders read one private snapshot instead: a stream that holds the message name, then the
// first argument bytes at the same address modulo 8 as in the stream (values are aligned in memory, not in the span).
// It holds as many argument bytes as the decoder can read (all of them if fewer remain), so decoding it is decoding the
// stream at the time of the copy.
template<typename Message, typename Decoded>
NEVER_INLINE std::optional<typename Message::Arguments> decodeArgumentsAndValidate(IPC::Decoder& decoder)
{
    using MessageArguments = Arguments<typename Message::Arguments>;
    static constexpr size_t argumentsOffset = sizeof(IPC::MessageName);

    auto stream = decoder.span();
    size_t position = decoder.currentBufferOffset();
    auto remaining = stream.subspan(position);
    // The arguments of a message without a reply ID follow its uint16_t name, which is 2-byte aligned.
    auto argumentsAddress = reinterpret_cast<uintptr_t>(remaining.data());
    RELEASE_ASSERT(!(argumentsAddress % alignof(IPC::MessageName)));

    size_t size = std::min(remaining.size(), Decoded::maximumReadSize);

    Vector<uint8_t> storage(8 + argumentsOffset + size);
    size_t start = (argumentsAddress + 16 - argumentsOffset - reinterpret_cast<uintptr_t>(storage.span().data()) % 8) % 8;
    auto writableSnapshot = storage.mutableSpan().subspan(start, argumentsOffset + size);
    auto name = Message::name();
    memcpySpan(writableSnapshot.first(argumentsOffset), asByteSpan(name));
    memcpySpan(writableSnapshot.subspan(argumentsOffset), remaining.first(size));
    std::span<const uint8_t> snapshot { writableSnapshot };
    RELEASE_ASSERT((reinterpret_cast<uintptr_t>(snapshot.data()) + argumentsOffset) % 8 == argumentsAddress % 8);

    IPC::Decoder cxxDecoder { snapshot, 0 };
    RELEASE_ASSERT(cxxDecoder.isValid() && cxxDecoder.messageName() == Message::name() && cxxDecoder.currentBufferOffset() == argumentsOffset);
    auto cxxArguments = cxxDecoder.decode<typename Message::Arguments>();

    Decoded decoded;
    intptr_t result = Decoded::decode(snapshot, argumentsOffset, decoded);
    RELEASE_ASSERT((result >= 0) == !!cxxArguments);
    RELEASE_ASSERT(cxxDecoder.isValid() == !!cxxArguments);
    if (result < 0) {
        RELEASE_ASSERT(result == WebKitSwift_IPCArgDecoders_Invalid || result == WebKitSwift_IPCArgDecoders_InvalidIdentifier);
#if ENABLE(IPC_TESTING_API)
        RELEASE_ASSERT(cxxDecoder.hasErrorString() == (result == WebKitSwift_IPCArgDecoders_InvalidIdentifier));
#endif
        markInvalid(decoder, result);
        return std::nullopt;
    }
    RELEASE_ASSERT(static_cast<size_t>(result) == cxxDecoder.currentBufferOffset());
    Bits swiftBits;
    MessageArguments::bits(swiftBits, MessageArguments::make(decoded, snapshot), snapshot);
    Bits cxxBits;
    MessageArguments::bits(cxxBits, *cxxArguments, snapshot);
    RELEASE_ASSERT(swiftBits == cxxBits);

    // The same arguments, at the same offsets, in the stream. Only the values that need no validation (floats, doubles,
    // uint32_t) are read from it again.
    decoded.rebase(argumentsOffset, position);
    decoder.setCurrentBufferOffset(position + result - argumentsOffset);
    return MessageArguments::make(decoded, stream);
}
#endif

template<typename Message>
ALWAYS_INLINE std::optional<typename Message::Arguments> decodeArguments(IPC::Decoder& decoder)
{
    using MessageArguments = Arguments<typename Message::Arguments>;
    using Decoded = typename MessageArguments::Decoded;
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]]
        return decodeArgumentsAndValidate<Message, Decoded>(decoder);
#endif
    auto stream = decoder.span();
    Decoded decoded;
    intptr_t result = Decoded::decode(stream, decoder.currentBufferOffset(), decoded);
    if (result < 0) [[unlikely]] {
        markInvalid(decoder, result);
        return std::nullopt;
    }
    decoder.setCurrentBufferOffset(result);
    return MessageArguments::make(decoded, stream);
}

} // namespace SwiftArgumentDecoders
} // namespace WebKit

namespace Messages::RemoteGraphicsContext {

#define DEFINE_SWIFT_ARGUMENT_DECODER(Message) \
    std::optional<Message::Arguments> Message::decodeArgumentsInSwift(IPC::Decoder& decoder) \
    { \
        return WebKit::SwiftArgumentDecoders::decodeArguments<Message>(decoder); \
    }

DEFINE_SWIFT_ARGUMENT_DECODER(Translate)
DEFINE_SWIFT_ARGUMENT_DECODER(SetFillCachedGradient)
DEFINE_SWIFT_ARGUMENT_DECODER(SetStrokeCachedGradient)
DEFINE_SWIFT_ARGUMENT_DECODER(SetStrokePackedColorAndThickness)
DEFINE_SWIFT_ARGUMENT_DECODER(FillRect)
DEFINE_SWIFT_ARGUMENT_DECODER(FillArc)
DEFINE_SWIFT_ARGUMENT_DECODER(FillClosedArc)
DEFINE_SWIFT_ARGUMENT_DECODER(FillQuadCurve)
DEFINE_SWIFT_ARGUMENT_DECODER(FillBezierCurve)
DEFINE_SWIFT_ARGUMENT_DECODER(StrokeArc)
DEFINE_SWIFT_ARGUMENT_DECODER(StrokeClosedArc)
DEFINE_SWIFT_ARGUMENT_DECODER(StrokeQuadCurve)
DEFINE_SWIFT_ARGUMENT_DECODER(StrokeBezierCurve)

#undef DEFINE_SWIFT_ARGUMENT_DECODER

} // namespace Messages::RemoteGraphicsContext

#endif // ENABLE(GPU_PROCESS) && USE(SWIFT_KERNELS)
