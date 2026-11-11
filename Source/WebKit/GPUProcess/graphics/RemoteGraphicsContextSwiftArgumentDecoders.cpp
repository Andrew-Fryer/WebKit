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
#include "IPCPathDecodeSwiftBoundary.h"
#include "IPCStreamMessageSwiftBoundary.h"
#include "MessageNames.h"
#include "RemoteGradientIdentifier.h"
#include "RemotePathImplIdentifier.h"
#include <WebCore/AffineTransform.h>
#include <WebCore/ColorTypes.h>
#include <WebCore/FloatRect.h>
#include <WebCore/FloatRoundedRect.h>
#include <WebCore/GraphicsContext.h>
#include <WebCore/Path.h>
#include <WebCore/PathSegment.h>
#include <WebCore/PathSegmentData.h>
#include <WebCore/RotationDirection.h>
#include <WebCore/WindRule.h>
#include <array>
#include <bit>
#include <span>
#include <wtf/StdLibExtras.h>
#include <wtf/Vector.h>

#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif

// The arguments of the RemoteGraphicsContext messages that have the SwiftArgumentDecoder attribute are decoded in Swift,
// and IPC::handleMessage() gets them from the decodeArgumentsInSwift() functions defined at the end of this file instead
// of from the original decoder.decode<Message::Arguments>() (HandleMessage.h).
// - The messages with an argument layout (IPC::swiftArgumentLayout(), which messages.py derives from their parameter
//   types) have their arguments validated and located by the same call into Swift that decodes the message header, in
//   the IPC::Decoder constructor (Source/WebKit/swift/IPCStreamMessage). The C++ here only builds the C++ arguments from
//   what that stored in the decoder and from its span, and moves the decoder past them or marks it invalid, as the
//   original does. A decoder that isn't a stream message's (an out-of-stream message) has no located arguments, and
//   decodes them with the original.
//   The attribute is only on messages whose decoding is worth it: those of Translate, the cached-gradient and curve
//   messages, SetStrokePackedColorAndThickness, ClipTransformedCachedPath, and the arc and rect draws, which the generated
//   coders check field by field. The messages with a single float, color or identifier, an AffineTransform or a span of
//   lines decode in about a nanosecond inline in RemoteGraphicsContext::didReceiveStreamMessage(), which is less than
//   locating them in Swift and copying them out costs.
// - The messages with a WebCore::Path have no layout, and call into Swift for the Path (Source/WebKit/swift/IPCPathDecode).
// With ENABLE(SWIFT_KERNELS_VALIDATION) and WEBKIT_SWIFT_KERNELS_VALIDATE=1, every message is decoded both ways and
// compared (decodeArgumentsAndValidate()).

namespace WebKit {
namespace SwiftArgumentDecoders {

using WebCore::AffineTransform;
using WebCore::FloatPoint;
using WebCore::FloatRect;
using WebCore::FloatSize;
using Located = IPC::Decoder::SwiftStreamArguments;

// The argument layouts (IPCStreamMessageSwiftBoundary.h).
constexpr uint8_t wordsLayout(size_t count) { return static_cast<uint8_t>(WebKitSwift_IPCStreamMessage_LayoutWords + count); }
constexpr uint8_t wordsAndBoolEnumLayout(size_t count) { return static_cast<uint8_t>(WebKitSwift_IPCStreamMessage_LayoutWordsAndBoolEnum + count); }

// The values that need no validation (floats, doubles, uint32_t) are copied from the span at the offsets that the Swift
// decoder located, as decodeObject<T>() copies data[0].
template<size_t count, typename T>
std::array<T, count> copyValues(std::span<const uint8_t> stream, size_t offset)
{
    std::array<T, count> values;
    memcpySpan(asMutableByteSpan(values), stream.subspan(offset, sizeof(values)));
    return values;
}
template<size_t count> std::array<uint32_t, count> words(const Located& a, std::span<const uint8_t> s) { return copyValues<count, uint32_t>(s, a.valuesOffset); }

// Arguments<std::tuple<...>>: the layout of the arguments, make(), which builds the C++ arguments from the located ones as
// the generated coders do, and bits(), their bit pattern (to validate). The bool enums are built from a 0 or 1, which is
// what isValidEnum() accepts for every enum class : bool, and the identifiers from a validated value.

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
    template<typename IdentifierType> void append(AtomicObjectIdentifier<IdentifierType> identifier) { append(identifier.toUInt64()); }
    bool operator==(const Bits&) const = default;
};

inline float toFloat(uint32_t bits) { return std::bit_cast<float>(bits); }
inline FloatPoint toFloatPoint(uint32_t x, uint32_t y) { return { toFloat(x), toFloat(y) }; }
template<typename Enum> Enum toBoolEnum(uint8_t value)
{
    static_assert(std::is_same_v<std::underlying_type_t<Enum>, bool>);
    return static_cast<Enum>(value);
}
inline AffineTransform toAffineTransform(const Located& a, std::span<const uint8_t> s)
{
    auto matrix = copyValues<6, double>(s, a.valuesOffset);
    return AffineTransform { std::span<const double, 6> { matrix } };
}

template<typename> struct Arguments;

template<> struct Arguments<std::tuple<float, float>> {
    static constexpr uint8_t layout = wordsLayout(2);
    static std::tuple<float, float> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<2>(a, s);
        return { toFloat(w[0]), toFloat(w[1]) };
    }
    static void bits(Bits& b, const std::tuple<float, float>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PackedColor::RGBA, float>> {
    static constexpr uint8_t layout = wordsLayout(2);
    static std::tuple<WebCore::PackedColor::RGBA, float> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<2>(a, s);
        return { WebCore::PackedColor::RGBA { w[0] }, toFloat(w[1]) };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PackedColor::RGBA, float>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PathDataQuadCurve>> {
    static constexpr uint8_t layout = wordsLayout(6);
    static std::tuple<WebCore::PathDataQuadCurve> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<6>(a, s);
        return { WebCore::PathDataQuadCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]) } };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PathDataQuadCurve>& a, std::span<const uint8_t>)
    {
        auto& c = std::get<0>(a);
        b.append(c.start); b.append(c.controlPoint); b.append(c.endPoint);
    }
};

template<> struct Arguments<std::tuple<WebCore::PathDataBezierCurve>> {
    static constexpr uint8_t layout = wordsLayout(8);
    static std::tuple<WebCore::PathDataBezierCurve> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<8>(a, s);
        return { WebCore::PathDataBezierCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]), toFloatPoint(w[6], w[7]) } };
    }
    static void bits(Bits& b, const std::tuple<WebCore::PathDataBezierCurve>& a, std::span<const uint8_t>)
    {
        auto& c = std::get<0>(a);
        b.append(c.start); b.append(c.controlPoint1); b.append(c.controlPoint2); b.append(c.endPoint);
    }
};

inline WebCore::PathArc toPathArc(const Located& a, std::span<const uint8_t> s)
{
    auto w = words<5>(a, s);
    return { toFloatPoint(w[0], w[1]), toFloat(w[2]), toFloat(w[3]), toFloat(w[4]), toBoolEnum<WebCore::RotationDirection>(a.flag) };
}
inline void appendPathArc(Bits& b, const WebCore::PathArc& arc)
{
    b.append(arc.center); b.append(arc.radius); b.append(arc.startAngle); b.append(arc.endAngle); b.append(arc.direction);
}

template<> struct Arguments<std::tuple<WebCore::PathArc>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(5);
    static std::tuple<WebCore::PathArc> make(const Located& a, std::span<const uint8_t> s) { return { toPathArc(a, s) }; }
    static void bits(Bits& b, const std::tuple<WebCore::PathArc>& a, std::span<const uint8_t>) { appendPathArc(b, std::get<0>(a)); }
};

template<> struct Arguments<std::tuple<WebCore::PathClosedArc>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(5);
    static std::tuple<WebCore::PathClosedArc> make(const Located& a, std::span<const uint8_t> s) { return { WebCore::PathClosedArc { toPathArc(a, s) } }; }
    static void bits(Bits& b, const std::tuple<WebCore::PathClosedArc>& a, std::span<const uint8_t>) { appendPathArc(b, std::get<0>(a).arc); }
};

template<> struct Arguments<std::tuple<FloatRect, WebCore::RequiresClipToRect>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(4);
    static std::tuple<FloatRect, WebCore::RequiresClipToRect> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<4>(a, s);
        return { FloatRect { toFloatPoint(w[0], w[1]), FloatSize { toFloat(w[2]), toFloat(w[3]) } }, toBoolEnum<WebCore::RequiresClipToRect>(a.flag) };
    }
    static void bits(Bits& b, const std::tuple<FloatRect, WebCore::RequiresClipToRect>& a, std::span<const uint8_t>)
    {
        auto& rect = std::get<0>(a);
        b.append(rect.x()); b.append(rect.y()); b.append(rect.width()); b.append(rect.height()); b.append(std::get<1>(a));
    }
};

template<> struct Arguments<std::tuple<RemoteGradientIdentifier, AffineTransform>> {
    static constexpr uint8_t layout = WebKitSwift_IPCStreamMessage_LayoutIdentifierAndAffineTransform;
    static std::tuple<RemoteGradientIdentifier, AffineTransform> make(const Located& a, std::span<const uint8_t> s)
    {
        return { RemoteGradientIdentifier { a.identifier }, toAffineTransform(a, s) };
    }
    static void bits(Bits& b, const std::tuple<RemoteGradientIdentifier, AffineTransform>& a, std::span<const uint8_t>) { b.append(std::get<0>(a)); b.append(std::get<1>(a)); }
};

template<> struct Arguments<std::tuple<AffineTransform, RemotePathImplIdentifier, WebCore::WindRule>> {
    static constexpr uint8_t layout = WebKitSwift_IPCStreamMessage_LayoutAffineTransformIdentifierAndBoolEnum;
    static std::tuple<AffineTransform, RemotePathImplIdentifier, WebCore::WindRule> make(const Located& a, std::span<const uint8_t> s)
    {
        return { toAffineTransform(a, s), RemotePathImplIdentifier { a.identifier }, toBoolEnum<WebCore::WindRule>(a.flag) };
    }
    static void bits(Bits& b, const std::tuple<AffineTransform, RemotePathImplIdentifier, WebCore::WindRule>& a, std::span<const uint8_t>)
    {
        b.append(std::get<0>(a)); b.append(std::get<1>(a)); b.append(std::get<2>(a));
    }
};

// The number of argument bytes that the decoder of a layout reads at most, from the decoder's position, alignment
// padding included.
constexpr size_t maximumReadSize(uint8_t layout)
{
    if (layout >= wordsLayout(1) && layout <= wordsLayout(15))
        return 3 + 4 * layout;
    if (layout >= wordsAndBoolEnumLayout(1) && layout <= wordsAndBoolEnumLayout(15))
        return 3 + 4 * (layout - wordsAndBoolEnumLayout(0)) + 1;
    if (layout == WebKitSwift_IPCStreamMessage_LayoutIdentifierAndAffineTransform)
        return 7 + 8 + 48;
    if (layout == WebKitSwift_IPCStreamMessage_LayoutAffineTransformIdentifierAndBoolEnum)
        return 7 + 48 + 8 + 1;
    RELEASE_ASSERT_NOT_REACHED_UNDER_CONSTEXPR_CONTEXT();
}

ALWAYS_INLINE void markInvalid(IPC::Decoder& decoder, Located::Status status)
{
#if ENABLE(IPC_TESTING_API)
    if (status == Located::Status::InvalidIdentifier)
        decoder.setErrorString("Validation failed: WTF::ObjectIdentifierGenericBase::isValidIdentifier(*toUInt64)"_s);
#else
    UNUSED_PARAM(status);
#endif
    decoder.markInvalid();
}

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// The stream is shared memory that the sending process can rewrite at any time, so a second read could see other bytes
// than the first. Both decoders read one private snapshot instead: a stream that holds the message name, then the first
// argument bytes at the same address modulo 8 as in the stream (values are aligned in memory, not in the span). It holds
// as many argument bytes as the decoder can read (all of them if fewer remain), so decoding it is decoding the stream at
// the time of the copy.
// The Swift decoder is the one that the stream decoder constructor calls (StreamMessageSwiftTag), header included; the
// C++ is the original header decoder (StreamHeaderCxxTag), then decode<Arguments>().
template<typename Message>
NEVER_INLINE std::optional<typename Message::Arguments> decodeArgumentsAndValidate(IPC::Decoder& decoder)
{
    using MessageArguments = Arguments<typename Message::Arguments>;
    static constexpr size_t argumentsOffset = sizeof(IPC::MessageName);
    static constexpr uint8_t layout = MessageArguments::layout;

    // The located arguments are where the header ends, so the decoder must still be there, as in IPC::handleMessage().
    RELEASE_ASSERT(decoder.isAtArguments());
    auto stream = decoder.span();
    size_t position = decoder.currentBufferOffset();
    auto remaining = stream.subspan(position);
    // The arguments of a message without a reply ID follow its uint16_t name, which is 2-byte aligned.
    auto argumentsAddress = reinterpret_cast<uintptr_t>(remaining.data());
    RELEASE_ASSERT(!(argumentsAddress % alignof(IPC::MessageName)));

    size_t size = std::min(remaining.size(), maximumReadSize(layout));

    Vector<uint8_t> storage(8 + argumentsOffset + size);
    size_t start = (argumentsAddress + 16 - argumentsOffset - reinterpret_cast<uintptr_t>(storage.span().data()) % 8) % 8;
    auto writableSnapshot = storage.mutableSpan().subspan(start, argumentsOffset + size);
    auto name = Message::name();
    memcpySpan(writableSnapshot.first(argumentsOffset), asByteSpan(name));
    memcpySpan(writableSnapshot.subspan(argumentsOffset), remaining.first(size));
    std::span<const uint8_t> snapshot { writableSnapshot };
    RELEASE_ASSERT((reinterpret_cast<uintptr_t>(snapshot.data()) + argumentsOffset) % 8 == argumentsAddress % 8);

    IPC::Decoder cxxDecoder { snapshot, 0, IPC::Decoder::StreamHeaderCxxTag { } };
    RELEASE_ASSERT(cxxDecoder.isValid() && cxxDecoder.messageName() == Message::name() && cxxDecoder.currentBufferOffset() == argumentsOffset);
    auto cxxArguments = cxxDecoder.decode<typename Message::Arguments>();

    IPC::Decoder swiftDecoder { snapshot, 0, IPC::Decoder::StreamMessageSwiftTag { } };
    RELEASE_ASSERT(swiftDecoder.isValid() && swiftDecoder.messageName() == Message::name() && swiftDecoder.currentBufferOffset() == argumentsOffset);
    auto located = swiftDecoder.swiftStreamArguments();
    RELEASE_ASSERT(located.status != Located::Status::None);
    RELEASE_ASSERT((located.status == Located::Status::Valid) == !!cxxArguments);
    RELEASE_ASSERT(cxxDecoder.isValid() == !!cxxArguments);
    if (located.status != Located::Status::Valid) {
        RELEASE_ASSERT(located.status == Located::Status::Invalid || located.status == Located::Status::InvalidIdentifier);
#if ENABLE(IPC_TESTING_API)
        RELEASE_ASSERT(cxxDecoder.hasErrorString() == (located.status == Located::Status::InvalidIdentifier));
#endif
        markInvalid(decoder, located.status);
        return std::nullopt;
    }
    RELEASE_ASSERT(located.endOffset == cxxDecoder.currentBufferOffset());
    RELEASE_ASSERT(located.flag <= 1);
    Bits swiftBits;
    MessageArguments::bits(swiftBits, MessageArguments::make(located, snapshot), snapshot);
    Bits cxxBits;
    MessageArguments::bits(cxxBits, *cxxArguments, snapshot);
    RELEASE_ASSERT(swiftBits == cxxBits);

    // The same arguments, at the same offsets, in the stream. Only the values that need no validation (floats, doubles,
    // uint32_t) are read from it again.
    located.valuesOffset = located.valuesOffset - argumentsOffset + position;
    decoder.setCurrentBufferOffset(located.endOffset - argumentsOffset + position);
    return MessageArguments::make(located, stream);
}
#endif

template<typename Message>
ALWAYS_INLINE std::optional<typename Message::Arguments> decodeArguments(IPC::Decoder& decoder)
{
    using MessageArguments = Arguments<typename Message::Arguments>;
    static_assert(IPC::swiftArgumentLayout(Message::name()) == MessageArguments::layout, "messages.py and the glue disagree on the layout");
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]]
        return decodeArgumentsAndValidate<Message>(decoder);
#endif
    auto& located = decoder.swiftStreamArguments();
    if (located.status == Located::Status::Valid) [[likely]] {
        decoder.setCurrentBufferOffset(located.endOffset);
        return MessageArguments::make(located, decoder.span());
    }
    if (located.status == Located::Status::None)
        return decoder.decode<typename Message::Arguments>();
    markInvalid(decoder, located.status);
    return std::nullopt;
}

} // namespace SwiftArgumentDecoders
} // namespace WebKit

// The WebCore::Path arguments of StrokePath, FillPath, DrawPath, ClipOutToPath and ClipPath (the messages with a Path and
// the SwiftArgumentDecoder attribute, which have no argument layout) are decoded in Swift too
// (Source/WebKit/swift/IPCPathDecode), which validates the wire format and copies each segment's floats into a record,
// and also decodes ClipPath's trailing WindRule; the C++ here builds the PathSegments from the records, the
// Vector<PathSegment> with the capacity that VectorArgumentCoder gives it, and the Path with Path(Vector<PathSegment>&&),
// as the generated coders do.

namespace WebKit {
namespace SwiftPathDecoders {

using WebCore::FloatPoint;
using WebCore::FloatRect;
using WebCore::FloatSize;
using WebCore::Path;
using WebCore::PathSegment;

static constexpr size_t recordWords = WebKitSwift_IPCPathDecode_RecordWords;
using Record = std::span<const uint32_t, recordWords>;
// The segments decoded per call into Swift. Most paths have fewer (those of MotionMark's canvas tests have about 3).
static constexpr size_t recordCapacity = 16;
using Records = std::array<uint32_t, recordCapacity * recordWords>;

static_assert(WebKitSwift_IPCPathDecode_AlternativeCount == WTF::VariantSizeV<PathSegment::Data>);

// A PathSegment holding alternative `index` of PathSegment::Data, built from `arguments` as its generated coder builds it,
// and emplaced at that index as ArgumentCoder<Variant> does.
template<size_t index, typename... Arguments>
ALWAYS_INLINE PathSegment makeSegment(Arguments&&... arguments)
{
    using Alternative = WTF::VariantAlternativeT<index, PathSegment::Data>;
    return PathSegment { PathSegment::Data { WTF::InPlaceIndex<index>, Alternative { std::forward<Arguments>(arguments)... } } };
}

// The floats of a record (IPCPathDecodeSwiftBoundary.h) of an alternative that has `count` of them. Swift writes only
// those words (the rest of the record is not initialized), and records how many it wrote. Each read of float i is
// checked against count at compile time, and floats<count>() checks the count that Swift recorded, so that the C++ never
// reads a word that Swift didn't write, even if the two disagreed on an alternative's layout. The check is a
// RELEASE_ASSERT in every build that has the validation (all but Production, so every test run checks every segment),
// and an ASSERT in Production, where it costs 2-5% of decoding a short path (as would zeroing the records first, 12-18%).
template<size_t count>
struct RecordFloats {
    Record record;
    template<size_t i> float value() const
    {
        static_assert(i < count);
        return std::bit_cast<float>(record[1 + i]);
    }
    template<size_t i> FloatPoint point() const { return { value<i>(), value<i + 1>() }; }
    template<size_t i> FloatSize size() const { return { value<i>(), value<i + 1>() }; }
    template<size_t i> FloatRect rect() const { return { point<i>(), size<i + 2>() }; }
};
template<size_t count>
ALWAYS_INLINE RecordFloats<count> floats(Record record)
{
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    RELEASE_ASSERT(((record[0] >> 16) & 0xff) == count);
#else
    ASSERT(((record[0] >> 16) & 0xff) == count);
#endif
    return { record };
}

// The segment of a record. The trailing enums are built from a 0 or 1, which is what isValidEnum() accepts for
// RotationDirection and PathRoundedRect::Strategy.
ALWAYS_INLINE PathSegment toPathSegment(Record record)
{
    uint8_t trailingEnum = static_cast<uint8_t>(record[0] >> 8);
    auto direction = [trailingEnum] { return static_cast<WebCore::RotationDirection>(trailingEnum); };

    switch (record[0] & 0xff) {
    case 0: { // PathMoveTo
        auto f = floats<2>(record);
        return makeSegment<0>(f.point<0>());
    }
    case 1: { // PathLineTo
        auto f = floats<2>(record);
        return makeSegment<1>(f.point<0>());
    }
    case 2: { // PathQuadCurveTo
        auto f = floats<4>(record);
        return makeSegment<2>(f.point<0>(), f.point<2>());
    }
    case 3: { // PathBezierCurveTo
        auto f = floats<6>(record);
        return makeSegment<3>(f.point<0>(), f.point<2>(), f.point<4>());
    }
    case 4: { // PathArcTo
        auto f = floats<5>(record);
        return makeSegment<4>(f.point<0>(), f.point<2>(), f.value<4>());
    }
    case 5: { // PathArc
        auto f = floats<5>(record);
        return makeSegment<5>(f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), direction());
    }
    case 6: { // PathClosedArc
        auto f = floats<5>(record);
        return makeSegment<6>(WebCore::PathArc { f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), direction() });
    }
    case 7: { // PathEllipse
        auto f = floats<7>(record);
        return makeSegment<7>(f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), f.value<5>(), f.value<6>(), direction());
    }
    case 8: { // PathEllipseInRect
        auto f = floats<4>(record);
        return makeSegment<8>(f.rect<0>());
    }
    case 9: { // PathRect
        auto f = floats<4>(record);
        return makeSegment<9>(f.rect<0>());
    }
    case 10: { // PathRoundedRect
        auto f = floats<12>(record);
        return makeSegment<10>(WebCore::FloatRoundedRect { f.rect<0>(), f.size<4>(), f.size<6>(), f.size<8>(), f.size<10>() }, static_cast<WebCore::PathRoundedRect::Strategy>(trailingEnum));
    }
    case 11: { // PathContinuousRoundedRect
        auto f = floats<6>(record);
        return makeSegment<11>(f.rect<0>(), f.value<4>(), f.value<5>());
    }
    case 12: { // PathDataLine
        auto f = floats<4>(record);
        std::array<float, 4> values { f.value<0>(), f.value<1>(), f.value<2>(), f.value<3>() };
        return makeSegment<12>(std::span<const float, 4> { values });
    }
    case 13: { // PathDataQuadCurve
        auto f = floats<6>(record);
        return makeSegment<13>(f.point<0>(), f.point<2>(), f.point<4>());
    }
    case 14: { // PathDataBezierCurve
        auto f = floats<8>(record);
        return makeSegment<14>(f.point<0>(), f.point<2>(), f.point<4>(), f.point<6>());
    }
    case 15: { // PathDataArc
        auto f = floats<7>(record);
        return makeSegment<15>(f.point<0>(), f.point<2>(), f.point<4>(), f.value<6>());
    }
    case 16: // PathCloseSubpath
        floats<0>(record);
        return makeSegment<16>();
    default: RELEASE_ASSERT_NOT_REACHED();
    }
}

ALWAYS_INLINE void appendSegments(Vector<PathSegment>& segments, const Records& records, size_t count)
{
    std::span<const uint32_t> all { records };
    for (size_t i = 0; i < count; ++i)
        segments.append(toPathSegment(all.subspan(i * recordWords).first<recordWords>()));
}

// decode<Path>() from `position` of `stream`, then if trailingBoolEnum, decode<E>() for an enum class E : bool: sets `path`
// and `boolEnum` and returns the new position, or returns WebKitSwift_IPCPathDecode_Invalid.
inline intptr_t decodePath(std::span<const uint8_t> stream, size_t position, bool trailingBoolEnum, std::optional<Path>& path, uint8_t& boolEnum)
{
    // Not initialized (an 832-byte memset costs more than decoding the path): toPathSegment() only reads the words written.
    Records records;
    uint64_t segmentCount = 0;
    intptr_t result = WebKitSwift_IPCPathDecode_decodePath(stream.data(), stream.size(), position, records.data(), records.size(), trailingBoolEnum, &segmentCount, &boolEnum);
    if (result < 0) [[unlikely]]
        return result;
    // As VectorArgumentCoder<false, PathSegment>::decode(), which reserves the capacity of vectors under 1 MB and grows
    // and then shrinks larger ones. (Swift fails a segmentCount larger than the bytes left in the stream.)
    Vector<PathSegment> segments;
    bool reserve = segmentCount < 1024 * 1024 / sizeof(PathSegment);
    if (reserve) [[likely]]
        segments.reserveInitialCapacity(segmentCount);
    size_t decoded = 0;
    size_t count = std::min<uint64_t>(segmentCount, recordCapacity);
    while (true) {
        appendSegments(segments, records, count);
        decoded += count;
        if (decoded == segmentCount) [[likely]]
            break;
        count = std::min<uint64_t>(segmentCount - decoded, recordCapacity);
        result = WebKitSwift_IPCPathDecode_decodePathSegments(stream.data(), stream.size(), result, records.data(), records.size(), segmentCount - decoded, trailingBoolEnum, &boolEnum);
        if (result < 0) [[unlikely]]
            return result;
    }
    if (!reserve)
        segments.shrinkToFit();
    path.emplace(WTF::move(segments));
    return result;
}

// Arguments<std::tuple<...>>::decode(): the arguments of a message from `position` of `stream`, as decodePath().
template<typename> struct Arguments;

template<> struct Arguments<std::tuple<Path>> {
    ALWAYS_INLINE static intptr_t decode(std::span<const uint8_t> stream, size_t position, std::optional<std::tuple<Path>>& arguments)
    {
        std::optional<Path> path;
        uint8_t unused = 0;
        intptr_t result = decodePath(stream, position, false, path, unused);
        if (result >= 0) [[likely]]
            arguments.emplace(WTF::move(*path));
        return result;
    }
};

template<> struct Arguments<std::tuple<Path, WebCore::WindRule>> {
    ALWAYS_INLINE static intptr_t decode(std::span<const uint8_t> stream, size_t position, std::optional<std::tuple<Path, WebCore::WindRule>>& arguments)
    {
        std::optional<Path> path;
        uint8_t windRule = 0;
        intptr_t result = decodePath(stream, position, true, path, windRule);
        if (result >= 0) [[likely]]
            arguments.emplace(WTF::move(*path), static_cast<WebCore::WindRule>(windRule));
        return result;
    }
};

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// The bit patterns of decoded arguments, member by member.
struct Bits {
    Vector<uint32_t> values;
    void append(uint32_t value) { values.append(value); }
    void append(float value) { append(std::bit_cast<uint32_t>(value)); }
    void append(const FloatPoint& point) { append(point.x()); append(point.y()); }
    void append(const FloatSize& size) { append(size.width()); append(size.height()); }
    void append(const FloatRect& rect) { append(rect.location()); append(rect.size()); }
    void append(const WebCore::FloatRoundedRect& rect)
    {
        append(rect.rect());
        append(rect.radii().topLeft()); append(rect.radii().topRight()); append(rect.radii().bottomLeft()); append(rect.radii().bottomRight());
    }
    template<typename Enum> requires std::is_enum_v<Enum> void append(Enum value)
    {
        static_assert(sizeof(Enum) == 1);
        append(static_cast<uint32_t>(value));
    }
    void append(const WebCore::PathMoveTo& s) { append(s.point); }
    void append(const WebCore::PathLineTo& s) { append(s.point); }
    void append(const WebCore::PathQuadCurveTo& s) { append(s.controlPoint); append(s.endPoint); }
    void append(const WebCore::PathBezierCurveTo& s) { append(s.controlPoint1); append(s.controlPoint2); append(s.endPoint); }
    void append(const WebCore::PathArcTo& s) { append(s.controlPoint1); append(s.controlPoint2); append(s.radius); }
    void append(const WebCore::PathArc& s) { append(s.center); append(s.radius); append(s.startAngle); append(s.endAngle); append(s.direction); }
    void append(const WebCore::PathClosedArc& s) { append(s.arc); }
    void append(const WebCore::PathEllipse& s)
    {
        append(s.center); append(s.radiusX); append(s.radiusY); append(s.rotation); append(s.startAngle); append(s.endAngle); append(s.direction);
    }
    void append(const WebCore::PathEllipseInRect& s) { append(s.rect); }
    void append(const WebCore::PathRect& s) { append(s.rect); }
    void append(const WebCore::PathRoundedRect& s) { append(s.roundedRect); append(s.strategy); }
    void append(const WebCore::PathContinuousRoundedRect& s) { append(s.rect); append(s.cornerWidth); append(s.cornerHeight); }
    void append(const WebCore::PathDataLine& s)
    {
        for (float value : s.span())
            append(value);
    }
    void append(const WebCore::PathDataQuadCurve& s) { append(s.start); append(s.controlPoint); append(s.endPoint); }
    void append(const WebCore::PathDataBezierCurve& s) { append(s.start); append(s.controlPoint1); append(s.controlPoint2); append(s.endPoint); }
    void append(const WebCore::PathDataArc& s) { append(s.start); append(s.controlPoint1); append(s.controlPoint2); append(s.radius); }
    void append(const WebCore::PathCloseSubpath&) { }
    void append(const PathSegment& segment)
    {
        append(static_cast<uint32_t>(segment.data().index()));
        WTF::switchOn(segment.data(), [&](const auto& alternative) { append(alternative); });
    }
    void append(const Path& path)
    {
        // Path(Vector<PathSegment>&&) is empty, a single segment, or a PathImpl (a PathStream) holding the vector.
        auto* segments = path.segmentsIfExists();
        append(static_cast<uint32_t>(segments ? 2 : path.singleSegmentIfExists() ? 1 : 0));
        if (segments) {
            append(static_cast<uint32_t>(segments->size()));
            append(static_cast<uint32_t>(segments->capacity()));
        }
        for (auto& segment : path.segments())
            append(segment);
    }
    template<typename... Elements> void append(const std::tuple<Elements...>& arguments)
    {
        std::apply([&](const auto&... element) { (append(element), ...); }, arguments);
    }
    bool operator==(const Bits&) const = default;
};

// As SwiftArgumentDecoders::decodeArgumentsAndValidate(): both decoders read one private snapshot of the stream, with
// the message name, then the argument bytes at the same address modulo 8 as in the stream. A Path has no size bound, so
// the snapshot holds the first 4 KB of what is left in the stream, and all of it if the arguments don't decode from
// that. Both decoders must agree on every snapshot (they read nothing past its end); the last one is what is decoded.
template<typename Message>
NEVER_INLINE std::optional<typename Message::Arguments> decodeArgumentsAndValidate(IPC::Decoder& decoder)
{
    static constexpr size_t argumentsOffset = sizeof(IPC::MessageName);
    static constexpr size_t firstSnapshotSize = 4096;

    auto stream = decoder.span();
    size_t position = decoder.currentBufferOffset();
    auto remaining = stream.subspan(position);
    // The arguments of a message without a reply ID follow its uint16_t name, which is 2-byte aligned.
    auto argumentsAddress = reinterpret_cast<uintptr_t>(remaining.data());
    RELEASE_ASSERT(!(argumentsAddress % alignof(IPC::MessageName)));

    for (size_t size = std::min(remaining.size(), firstSnapshotSize); ; size = remaining.size()) {
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

        std::optional<typename Message::Arguments> swiftArguments;
        intptr_t result = Arguments<typename Message::Arguments>::decode(snapshot, argumentsOffset, swiftArguments);
        RELEASE_ASSERT((result >= 0) == !!cxxArguments && (result >= 0) == !!swiftArguments);
        RELEASE_ASSERT(cxxDecoder.isValid() == !!cxxArguments);
#if ENABLE(IPC_TESTING_API)
        RELEASE_ASSERT(!cxxDecoder.hasErrorString());
#endif
        if (result < 0) {
            RELEASE_ASSERT(result == WebKitSwift_IPCPathDecode_Invalid);
            if (size < remaining.size())
                continue;
            decoder.markInvalid();
            return std::nullopt;
        }
        RELEASE_ASSERT(static_cast<size_t>(result) == cxxDecoder.currentBufferOffset());
        Bits swiftBits;
        swiftBits.append(*swiftArguments);
        Bits cxxBits;
        cxxBits.append(*cxxArguments);
        RELEASE_ASSERT(swiftBits == cxxBits);

        decoder.setCurrentBufferOffset(position + result - argumentsOffset);
        return swiftArguments;
    }
}
#endif

template<typename Message>
ALWAYS_INLINE std::optional<typename Message::Arguments> decodeArguments(IPC::Decoder& decoder)
{
    static_assert(IPC::swiftArgumentLayout(Message::name()) == WebKitSwift_IPCStreamMessage_LayoutNone, "messages.py and the glue disagree on the layout");
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]]
        return decodeArgumentsAndValidate<Message>(decoder);
#endif
    std::optional<typename Message::Arguments> arguments;
    intptr_t result = Arguments<typename Message::Arguments>::decode(decoder.span(), decoder.currentBufferOffset(), arguments);
    if (result < 0) [[unlikely]] {
        decoder.markInvalid();
        return std::nullopt;
    }
    decoder.setCurrentBufferOffset(result);
    return arguments;
}

} // namespace SwiftPathDecoders
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
DEFINE_SWIFT_ARGUMENT_DECODER(ClipTransformedCachedPath)
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

#define DEFINE_SWIFT_PATH_ARGUMENT_DECODER(Message) \
    std::optional<Message::Arguments> Message::decodeArgumentsInSwift(IPC::Decoder& decoder) \
    { \
        return WebKit::SwiftPathDecoders::decodeArguments<Message>(decoder); \
    }

DEFINE_SWIFT_PATH_ARGUMENT_DECODER(ClipOutToPath)
DEFINE_SWIFT_PATH_ARGUMENT_DECODER(ClipPath)
DEFINE_SWIFT_PATH_ARGUMENT_DECODER(DrawPath)
DEFINE_SWIFT_PATH_ARGUMENT_DECODER(FillPath)
DEFINE_SWIFT_PATH_ARGUMENT_DECODER(StrokePath)

#undef DEFINE_SWIFT_PATH_ARGUMENT_DECODER

} // namespace Messages::RemoteGraphicsContext

#endif // ENABLE(GPU_PROCESS) && USE(SWIFT_KERNELS)
