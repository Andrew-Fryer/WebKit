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

#if ENABLE(GPU_PROCESS)
#include "RemoteGraphicsContextMessages.h"

#include "Decoder.h"
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

// The arguments of the RemoteGraphicsContext messages that have the SwiftArgumentDecoder attribute are decoded in Swift
// (Source/WebKit/swift/IPCStreamMessage/IPCStreamMessageSwiftBoundary.h), and IPC::handleMessage() gets them from the
// decodeArgumentsInSwift() functions defined at the end of this file instead of from decoder.decode<Message::Arguments>()
// (HandleMessage.h).
// - The messages with an argument layout (IPC::swiftArgumentLayout(), which messages.py derives from their parameter
//   types) have their arguments validated and located by the same call into Swift that decodes the message header, in
//   the IPC::Decoder constructor, or, for a decoder that isn't a stream message's (an out-of-stream message), by a call
//   from here. The C++ here only builds the C++ arguments from what Swift located and from the decoder's span, and moves
//   the decoder past them or marks it invalid, as decode<Arguments>() does.
//   The attribute is only on messages whose decoding is worth it: those of Translate, the cached-gradient and curve
//   messages, SetStrokePackedColorAndThickness, ClipTransformedCachedPath, and the arc and rect draws, which the generated
//   coders check field by field. The messages with a single float, color or identifier, an AffineTransform or a span of
//   lines decode in about a nanosecond inline in RemoteGraphicsContext::didReceiveStreamMessage(), which is less than
//   locating them in Swift and copying them out costs.
// - The messages with a WebCore::Path have no layout; Swift decodes the Path into words that the C++ here builds the
//   PathSegments from.

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

// Arguments<std::tuple<...>>: the layout of the arguments, and make(), which builds the C++ arguments from the located
// ones as the generated coders do. The bool enums are built from a 0 or 1, which is what isValidEnum() accepts for every
// enum class : bool, and the identifiers from a validated value.

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
};

template<> struct Arguments<std::tuple<WebCore::PackedColor::RGBA, float>> {
    static constexpr uint8_t layout = wordsLayout(2);
    static std::tuple<WebCore::PackedColor::RGBA, float> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<2>(a, s);
        return { WebCore::PackedColor::RGBA { w[0] }, toFloat(w[1]) };
    }
};

template<> struct Arguments<std::tuple<WebCore::PathDataQuadCurve>> {
    static constexpr uint8_t layout = wordsLayout(6);
    static std::tuple<WebCore::PathDataQuadCurve> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<6>(a, s);
        return { WebCore::PathDataQuadCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]) } };
    }
};

template<> struct Arguments<std::tuple<WebCore::PathDataBezierCurve>> {
    static constexpr uint8_t layout = wordsLayout(8);
    static std::tuple<WebCore::PathDataBezierCurve> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<8>(a, s);
        return { WebCore::PathDataBezierCurve { toFloatPoint(w[0], w[1]), toFloatPoint(w[2], w[3]), toFloatPoint(w[4], w[5]), toFloatPoint(w[6], w[7]) } };
    }
};

inline WebCore::PathArc toPathArc(const Located& a, std::span<const uint8_t> s)
{
    auto w = words<5>(a, s);
    return { toFloatPoint(w[0], w[1]), toFloat(w[2]), toFloat(w[3]), toFloat(w[4]), toBoolEnum<WebCore::RotationDirection>(a.flag) };
}

template<> struct Arguments<std::tuple<WebCore::PathArc>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(5);
    static std::tuple<WebCore::PathArc> make(const Located& a, std::span<const uint8_t> s) { return { toPathArc(a, s) }; }
};

template<> struct Arguments<std::tuple<WebCore::PathClosedArc>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(5);
    static std::tuple<WebCore::PathClosedArc> make(const Located& a, std::span<const uint8_t> s) { return { WebCore::PathClosedArc { toPathArc(a, s) } }; }
};

template<> struct Arguments<std::tuple<FloatRect, WebCore::RequiresClipToRect>> {
    static constexpr uint8_t layout = wordsAndBoolEnumLayout(4);
    static std::tuple<FloatRect, WebCore::RequiresClipToRect> make(const Located& a, std::span<const uint8_t> s)
    {
        auto w = words<4>(a, s);
        return { FloatRect { toFloatPoint(w[0], w[1]), FloatSize { toFloat(w[2]), toFloat(w[3]) } }, toBoolEnum<WebCore::RequiresClipToRect>(a.flag) };
    }
};

template<> struct Arguments<std::tuple<RemoteGradientIdentifier, AffineTransform>> {
    static constexpr uint8_t layout = WebKitSwift_IPCStreamMessage_LayoutIdentifierAndAffineTransform;
    static std::tuple<RemoteGradientIdentifier, AffineTransform> make(const Located& a, std::span<const uint8_t> s)
    {
        return { RemoteGradientIdentifier { a.identifier }, toAffineTransform(a, s) };
    }
};

template<> struct Arguments<std::tuple<AffineTransform, RemotePathImplIdentifier, WebCore::WindRule>> {
    static constexpr uint8_t layout = WebKitSwift_IPCStreamMessage_LayoutAffineTransformIdentifierAndBoolEnum;
    static std::tuple<AffineTransform, RemotePathImplIdentifier, WebCore::WindRule> make(const Located& a, std::span<const uint8_t> s)
    {
        return { toAffineTransform(a, s), RemotePathImplIdentifier { a.identifier }, toBoolEnum<WebCore::WindRule>(a.flag) };
    }
};

// What decode<Arguments>() does with arguments that failed to locate: marks the decoder invalid, and, for an invalid
// identifier, sets the IPC_TESTING_API error string of the generated coders.
inline void markInvalid(IPC::Decoder& decoder, Located::Status status)
{
#if ENABLE(IPC_TESTING_API)
    if (status == Located::Status::InvalidIdentifier)
        decoder.setErrorString("Validation failed: WTF::ObjectIdentifierGenericBase::isValidIdentifier(*toUInt64)"_s);
#else
    UNUSED_PARAM(status);
#endif
    decoder.markInvalid();
}

// A decoder that isn't a stream message's has no located arguments: Swift locates them from its position.
template<typename Tuple>
NEVER_INLINE std::optional<Tuple> decodeOutOfStreamArguments(IPC::Decoder& decoder)
{
    Located located;
    auto span = decoder.span();
    WebKitSwift_IPCStreamMessage_locateArguments(span.data(), span.size(), decoder.currentBufferOffset(), Arguments<Tuple>::layout,
        reinterpret_cast<WebKitSwift_IPCStreamMessage_Arguments*>(&located));
    if (located.status != Located::Status::Valid) {
        markInvalid(decoder, located.status);
        return std::nullopt;
    }
    decoder.setCurrentBufferOffset(located.endOffset);
    return Arguments<Tuple>::make(located, span);
}

template<typename Message>
ALWAYS_INLINE std::optional<typename Message::Arguments> decodeArguments(IPC::Decoder& decoder)
{
    using Tuple = typename Message::Arguments;
    static_assert(IPC::swiftArgumentLayout(Message::name()) == Arguments<Tuple>::layout, "messages.py and the glue disagree on the layout");
    auto& located = decoder.swiftStreamArguments();
    if (located.status == Located::Status::Valid) [[likely]] {
        decoder.setCurrentBufferOffset(located.endOffset);
        return Arguments<Tuple>::make(located, decoder.span());
    }
    if (located.status == Located::Status::None)
        return decodeOutOfStreamArguments<Tuple>(decoder);
    markInvalid(decoder, located.status);
    return std::nullopt;
}

} // namespace SwiftArgumentDecoders
} // namespace WebKit

// The WebCore::Path arguments of StrokePath, FillPath, DrawPath, ClipOutToPath and ClipPath (the messages with a Path and
// the SwiftArgumentDecoder attribute, which have no argument layout) are decoded in Swift too
// (WebKitSwift_IPCStreamMessage_decodePath()), which validates the wire format and copies each segment's alternative
// index, trailing enum and floats into words, and also decodes ClipPath's trailing WindRule; the C++ here builds the
// PathSegments from the words, the Vector<PathSegment> with the capacity that VectorArgumentCoder gives it, and the Path
// with Path(Vector<PathSegment>&&), as the generated coders do.

namespace WebKit {
namespace SwiftPathDecoders {

using WebCore::FloatPoint;
using WebCore::FloatRect;
using WebCore::FloatSize;
using WebCore::Path;
using WebCore::PathSegment;

// The buffer of one call into Swift: 21 segments of one point (MoveTo, LineTo), at least 4 of the largest (RoundedRect,
// 13 words). Most paths have fewer segments (those of MotionMark's canvas tests have about 3); longer ones take a call per
// bufferful, which measured faster than a larger buffer for paths of 64 and 100 segments.
using Words = std::array<uint32_t, 64>;

// A PathSegment holding alternative `index` of PathSegment::Data, built from `arguments` as its generated coder builds it,
// and emplaced at that index as ArgumentCoder<Variant> does.
template<size_t index, typename... Arguments>
ALWAYS_INLINE PathSegment makeSegment(Arguments&&... arguments)
{
    using Alternative = WTF::VariantAlternativeT<index, PathSegment::Data>;
    return PathSegment { PathSegment::Data { WTF::InPlaceIndex<index>, Alternative { std::forward<Arguments>(arguments)... } } };
}

// The `count` floats of a segment, which follow its first word.
template<size_t count>
struct SegmentFloats {
    std::span<const uint32_t, count> words;
    template<size_t i> float value() const { return std::bit_cast<float>(words[i]); }
    template<size_t i> FloatPoint point() const { return { value<i>(), value<i + 1>() }; }
    template<size_t i> FloatSize size() const { return { value<i>(), value<i + 1>() }; }
    template<size_t i> FloatRect rect() const { return { point<i>(), size<i + 2>() }; }
};

// Takes the floats of a segment that has `count` of them from the start of `words`. Every word that Swift wrote is
// read as what Swift wrote it as, if both agree on the alternatives' float counts; if they didn't, the bounds checks of
// std::span would still keep every read within the words that Swift wrote.
template<size_t count>
ALWAYS_INLINE SegmentFloats<count> takeFloats(std::span<const uint32_t>& words)
{
    SegmentFloats<count> floats { words.template first<count>() };
    words = words.subspan(count);
    return floats;
}

// Takes the segment at the start of `words`. The trailing enums are built from a 0 or 1, which is what isValidEnum()
// accepts for RotationDirection and PathRoundedRect::Strategy.
ALWAYS_INLINE PathSegment takeSegment(std::span<const uint32_t>& words)
{
    uint32_t header = words.front();
    words = words.subspan(1);
    uint8_t trailingEnum = static_cast<uint8_t>(header >> 8);
    auto direction = [trailingEnum] { return static_cast<WebCore::RotationDirection>(trailingEnum); };

    switch (header & 0xff) {
    case 0: { // PathMoveTo
        auto f = takeFloats<2>(words);
        return makeSegment<0>(f.point<0>());
    }
    case 1: { // PathLineTo
        auto f = takeFloats<2>(words);
        return makeSegment<1>(f.point<0>());
    }
    case 2: { // PathQuadCurveTo
        auto f = takeFloats<4>(words);
        return makeSegment<2>(f.point<0>(), f.point<2>());
    }
    case 3: { // PathBezierCurveTo
        auto f = takeFloats<6>(words);
        return makeSegment<3>(f.point<0>(), f.point<2>(), f.point<4>());
    }
    case 4: { // PathArcTo
        auto f = takeFloats<5>(words);
        return makeSegment<4>(f.point<0>(), f.point<2>(), f.value<4>());
    }
    case 5: { // PathArc
        auto f = takeFloats<5>(words);
        return makeSegment<5>(f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), direction());
    }
    case 6: { // PathClosedArc
        auto f = takeFloats<5>(words);
        return makeSegment<6>(WebCore::PathArc { f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), direction() });
    }
    case 7: { // PathEllipse
        auto f = takeFloats<7>(words);
        return makeSegment<7>(f.point<0>(), f.value<2>(), f.value<3>(), f.value<4>(), f.value<5>(), f.value<6>(), direction());
    }
    case 8: { // PathEllipseInRect
        auto f = takeFloats<4>(words);
        return makeSegment<8>(f.rect<0>());
    }
    case 9: { // PathRect
        auto f = takeFloats<4>(words);
        return makeSegment<9>(f.rect<0>());
    }
    case 10: { // PathRoundedRect
        auto f = takeFloats<12>(words);
        return makeSegment<10>(WebCore::FloatRoundedRect { f.rect<0>(), f.size<4>(), f.size<6>(), f.size<8>(), f.size<10>() }, static_cast<WebCore::PathRoundedRect::Strategy>(trailingEnum));
    }
    case 11: { // PathContinuousRoundedRect
        auto f = takeFloats<6>(words);
        return makeSegment<11>(f.rect<0>(), f.value<4>(), f.value<5>());
    }
    case 12: { // PathDataLine
        auto f = takeFloats<4>(words);
        std::array<float, 4> values { f.value<0>(), f.value<1>(), f.value<2>(), f.value<3>() };
        return makeSegment<12>(std::span<const float, 4> { values });
    }
    case 13: { // PathDataQuadCurve
        auto f = takeFloats<6>(words);
        return makeSegment<13>(f.point<0>(), f.point<2>(), f.point<4>());
    }
    case 14: { // PathDataBezierCurve
        auto f = takeFloats<8>(words);
        return makeSegment<14>(f.point<0>(), f.point<2>(), f.point<4>(), f.point<6>());
    }
    case 15: { // PathDataArc
        auto f = takeFloats<7>(words);
        return makeSegment<15>(f.point<0>(), f.point<2>(), f.point<4>(), f.value<6>());
    }
    case 16: // PathCloseSubpath
        return makeSegment<16>();
    default: RELEASE_ASSERT_NOT_REACHED();
    }
}

static_assert(sizeof(WebKitSwift_IPCStreamMessage_PathProgress) == 4 * sizeof(uint64_t));

// decode<Path>() from `position` of `stream`, then if trailingBoolEnum, decode<E>() for an enum class E : bool: sets `path`
// and `boolEnum` and returns the new position, or returns -1.
inline intptr_t decodePath(std::span<const uint8_t> stream, size_t position, bool trailingBoolEnum, std::optional<Path>& path, uint8_t& boolEnum)
{
    // Not initialized (a memset costs more than decoding a short path): only the words that Swift wrote are read.
    Words words;
    WebKitSwift_IPCStreamMessage_PathProgress progress { };
    intptr_t result = WebKitSwift_IPCStreamMessage_decodePath(stream.data(), stream.size(), position, words.data(), words.size(), trailingBoolEnum, &progress);
    if (result < 0) [[unlikely]]
        return result;
    // As VectorArgumentCoder<false, PathSegment>::decode(), which reserves the capacity of vectors under 1 MB and grows
    // and then shrinks larger ones. (Swift fails a segmentCount larger than the bytes left in the stream.)
    Vector<PathSegment> segments;
    bool reserve = progress.segmentCount < 1024 * 1024 / sizeof(PathSegment);
    if (reserve) [[likely]]
        segments.reserveInitialCapacity(progress.segmentCount);
    while (true) {
        for (auto written = std::span<const uint32_t> { words }.first(progress.wordCount); !written.empty();)
            segments.append(takeSegment(written));
        ASSERT(segments.size() == progress.segmentsDecoded);
        if (progress.segmentsDecoded == progress.segmentCount) [[likely]]
            break;
        result = WebKitSwift_IPCStreamMessage_decodePath(stream.data(), stream.size(), result, words.data(), words.size(), trailingBoolEnum, &progress);
        if (result < 0) [[unlikely]]
            return result;
    }
    if (!reserve)
        segments.shrinkToFit();
    boolEnum = static_cast<uint8_t>(progress.boolEnum);
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

template<typename Message>
ALWAYS_INLINE std::optional<typename Message::Arguments> decodeArguments(IPC::Decoder& decoder)
{
    static_assert(IPC::swiftArgumentLayout(Message::name()) == WebKitSwift_IPCStreamMessage_LayoutNone, "messages.py and the glue disagree on the layout");
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

#endif // ENABLE(GPU_PROCESS)
