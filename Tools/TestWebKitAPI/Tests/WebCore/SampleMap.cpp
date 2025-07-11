/*
 * Copyright (C) 2017 Apple Inc. All rights reserved.
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

#if ENABLE(MEDIA_SOURCE)

#include "Test.h"
#include <WebCore/MediaSample.h>
#include <WebCore/SampleMap.h>

namespace WTF {
inline std::ostream& operator<<(std::ostream& os, const MediaTime& time)
{
    if (time.hasDoubleValue())
        os << "{ " << time.toDouble() << " }";
    else
        os << "{ " << time.timeValue() << " / " << time.timeScale() << ", " << time.toDouble() << " }";
    return os;
}
}

using namespace WebCore;

namespace TestWebKitAPI {

class TestSample : public MediaSample {
public:
    static Ref<TestSample> create(const MediaTime& presentationTime, const MediaTime& decodeTime, const MediaTime& duration, SampleFlags flags)
    {
        return adoptRef(*new TestSample(presentationTime, decodeTime, duration, flags));
    }

    MediaTime presentationTime() const final { return m_presentationTime; }
    MediaTime decodeTime() const final { return m_decodeTime; }
    MediaTime duration() const final { return m_duration; }
    TrackID trackID() const final { return m_trackID; }
    size_t sizeInBytes() const final { return m_sizeInBytes; }
    FloatSize presentationSize() const final { return m_presentationSize; }
    void offsetTimestampsBy(const MediaTime& offset) final { m_presentationTime += offset; m_decodeTime += offset; }
    void setTimestamps(const MediaTime& presentationTime, const MediaTime& decodeTime) final {
        m_presentationTime = presentationTime;
        m_decodeTime = decodeTime;
    };
    Ref<MediaSample> createNonDisplayingCopy() const final {
        return create(m_presentationTime, m_decodeTime, m_duration, static_cast<SampleFlags>(m_flags | IsNonDisplaying));
    }
    SampleFlags flags() const final { return m_flags; }
    PlatformSample platformSample() const final { return { PlatformSample::None, { nullptr } }; }
    PlatformSample::Type platformSampleType() const final { return PlatformSample::None; }

    void dump(PrintStream&) const final { }

private:
    TestSample(const MediaTime& presentationTime, const MediaTime& decodeTime, const MediaTime& duration, SampleFlags flags)
        : m_presentationTime(presentationTime)
        , m_decodeTime(decodeTime)
        , m_duration(duration)
        , m_flags(flags)
    {
    }

    MediaTime m_presentationTime;
    MediaTime m_decodeTime;
    MediaTime m_duration;
    FloatSize m_presentationSize;
    TrackID m_trackID;
    size_t m_sizeInBytes { 0 };
    SampleFlags m_flags { None };
};

class SampleMapTest : public testing::Test {
public:
    void SetUp() final {
        map.addSample(TestSample::create(MediaTime(0, 1), MediaTime(0, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(1, 1), MediaTime(1, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(2, 1), MediaTime(2, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(3, 1), MediaTime(3, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(4, 1), MediaTime(4, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(5, 1), MediaTime(5, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(6, 1), MediaTime(6, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(7, 1), MediaTime(7, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(8, 1), MediaTime(8, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(9, 1), MediaTime(9, 1), MediaTime(1, 1), MediaSample::None));
        // Gap at MediaTime(10, 1) -> MediaTime(11, 1);
        map.addSample(TestSample::create(MediaTime(11, 1), MediaTime(11, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(12, 1), MediaTime(12, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(13, 1), MediaTime(13, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(14, 1), MediaTime(14, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(15, 1), MediaTime(15, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(16, 1), MediaTime(16, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(17, 1), MediaTime(17, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(18, 1), MediaTime(18, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(19, 1), MediaTime(19, 1), MediaTime(1, 1), MediaSample::None));
        // Some B-frames
        map.addSample(TestSample::create(MediaTime(20, 1), MediaTime(20, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(23, 1), MediaTime(21, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(22, 1), MediaTime(22, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(21, 1), MediaTime(23, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(24, 1), MediaTime(24, 1), MediaTime(1, 1), MediaSample::IsSync));
        map.addSample(TestSample::create(MediaTime(28, 1), MediaTime(25, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(27, 1), MediaTime(26, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(26, 1), MediaTime(27, 1), MediaTime(1, 1), MediaSample::None));
        map.addSample(TestSample::create(MediaTime(25, 1), MediaTime(28, 1), MediaTime(1, 1), MediaSample::None));
    }

    SampleMap map;
};

TEST_F(SampleMapTest, findSampleWithPresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleWithPresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(19, 1), presentationMap.findSampleWithPresentationTime(MediaTime(19, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(26, 1), presentationMap.findSampleWithPresentationTime(MediaTime(26, 1))->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleWithPresentationTime(MediaTime(-1, 1)));
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleWithPresentationTime(MediaTime(10, 1)));
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleWithPresentationTime(MediaTime(29, 1)));
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleWithPresentationTime(MediaTime(1, 2)));
}

TEST_F(SampleMapTest, findSampleContainingPresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleContainingPresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(19, 1), presentationMap.findSampleContainingPresentationTime(MediaTime(19, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleContainingPresentationTime(MediaTime(1, 2))->second->presentationTime());
    EXPECT_EQ(MediaTime(26, 1), presentationMap.findSampleContainingPresentationTime(MediaTime(26, 1) + MediaTime(1, 2))->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleContainingPresentationTime(MediaTime(-1, 1)));
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleContainingPresentationTime(MediaTime(29, 1)));
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleContainingPresentationTime(MediaTime(30, 1)));
}

TEST_F(SampleMapTest, findSampleStartingOnOrAfterPresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(19, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(19, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(1, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(1, 2))->second->presentationTime());
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(-1, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(11, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(10, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(25, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(25, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(26, 1), presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(25, 1) + MediaTime(1, 2))->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleStartingOnOrAfterPresentationTime(MediaTime(30, 1)));
}

TEST_F(SampleMapTest, findSampleContainingOrAfterPresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(19, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(19, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(1, 2))->second->presentationTime());
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(-1, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(11, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(10, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(26, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(26, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(25, 1), presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(25, 1) + MediaTime(1, 2))->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleContainingOrAfterPresentationTime(MediaTime(30, 1)));
}

TEST_F(SampleMapTest, findSampleStartingAfterPresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(1, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(1, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(1, 2))->second->presentationTime());
    EXPECT_EQ(MediaTime(0, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(-1, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(11, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(10, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(23, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(22, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(26, 1), presentationMap.findSampleStartingAfterPresentationTime(MediaTime(25, 1))->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == presentationMap.findSampleStartingAfterPresentationTime(MediaTime(30, 1)));
}

TEST_F(SampleMapTest, findSamplesBetweenPresentationTimes)
{
    auto& presentationMap = map.presentationOrder();
    auto iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(0, 1), MediaTime(1, 1));
    EXPECT_EQ(MediaTime(0, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(1, 1), iterator_range.second->second->presentationTime());

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(1, 2), MediaTime(3, 2));
    EXPECT_EQ(MediaTime(1, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(2, 1), iterator_range.second->second->presentationTime());

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(19, 1), MediaTime(25, 1));
    EXPECT_EQ(MediaTime(19, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(25, 1), iterator_range.second->second->presentationTime());
    MediaTime currentTime = iterator_range.first->second->presentationTime();
    for (auto it = iterator_range.first; it != iterator_range.second; ++it) {
        EXPECT_EQ(it->second->presentationTime(), currentTime);
        currentTime += MediaTime { 1, 1 };
    }

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(9, 1), MediaTime(31, 1));
    EXPECT_EQ(MediaTime(9, 1), iterator_range.first->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(-1, 1), MediaTime(0, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(19, 2), MediaTime(10, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimes(MediaTime(30, 1), MediaTime(31, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);
}

TEST_F(SampleMapTest, findSamplesBetweenPresentationTimesFromEnd)
{
    auto& presentationMap = map.presentationOrder();
    auto iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(0, 1), MediaTime(1, 1));
    EXPECT_EQ(MediaTime(0, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(1, 1), iterator_range.second->second->presentationTime());

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(1, 2), MediaTime(3, 2));
    EXPECT_EQ(MediaTime(1, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(2, 1), iterator_range.second->second->presentationTime());

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(19, 1), MediaTime(25, 1));
    EXPECT_EQ(MediaTime(19, 1), iterator_range.first->second->presentationTime());
    EXPECT_EQ(MediaTime(25, 1), iterator_range.second->second->presentationTime());
    MediaTime currentTime = iterator_range.first->second->presentationTime();
    for (auto it = iterator_range.first; it != iterator_range.second; ++it) {
        EXPECT_EQ(it->second->presentationTime(), currentTime);
        currentTime += MediaTime { 1, 1 };
    }

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(9, 1), MediaTime(31, 1));
    EXPECT_EQ(MediaTime(9, 1), iterator_range.first->second->presentationTime());
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(-1, 1), MediaTime(0, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(19, 2), MediaTime(10, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);

    iterator_range = presentationMap.findSamplesBetweenPresentationTimesFromEnd(MediaTime(30, 1), MediaTime(31, 1));
    EXPECT_TRUE(presentationMap.end() == iterator_range.first);
    EXPECT_TRUE(presentationMap.end() == iterator_range.second);
}

TEST_F(SampleMapTest, reverseFindSampleBeforePresentationTime)
{
    auto& presentationMap = map.presentationOrder();
    EXPECT_EQ(MediaTime(0, 1), presentationMap.reverseFindSampleBeforePresentationTime(MediaTime(0, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(9, 1), presentationMap.reverseFindSampleBeforePresentationTime(MediaTime(10, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(19, 1), presentationMap.reverseFindSampleBeforePresentationTime(MediaTime(19, 1))->second->presentationTime());
    EXPECT_EQ(MediaTime(28, 1), presentationMap.reverseFindSampleBeforePresentationTime(MediaTime(31, 1))->second->presentationTime());
    EXPECT_TRUE(presentationMap.rend() == presentationMap.reverseFindSampleBeforePresentationTime(MediaTime(-1, 1)));
}

TEST_F(SampleMapTest, findSamplesBetweenDecodeKeys)
{
    auto& decodeMap = map.decodeOrder();
    DecodeOrderSampleMap::MapType dependentSamples;
    DecodeOrderSampleMap::KeyType decodeKeyStart(MediaTime(0, 1), MediaTime(0, 1));
    DecodeOrderSampleMap::KeyType decodeKeyEnd(MediaTime(28, 1), MediaTime(25, 1));

    auto samplesWithHigherDecodeTimes = decodeMap.findSamplesBetweenDecodeKeys(decodeKeyStart, decodeKeyEnd);
    EXPECT_FALSE(samplesWithHigherDecodeTimes.first == samplesWithHigherDecodeTimes.second);
    EXPECT_EQ(MediaTime(0, 1), samplesWithHigherDecodeTimes.first->second->presentationTime());
    EXPECT_EQ(MediaTime(25, 1), samplesWithHigherDecodeTimes.second->second->presentationTime());

    decodeKeyEnd = { MediaTime(25, 1), MediaTime(28, 1) };
    samplesWithHigherDecodeTimes = decodeMap.findSamplesBetweenDecodeKeys(decodeKeyStart, decodeKeyEnd);
    EXPECT_FALSE(samplesWithHigherDecodeTimes.first == samplesWithHigherDecodeTimes.second);
    EXPECT_EQ(MediaTime(28, 1), samplesWithHigherDecodeTimes.second->second->presentationTime());

    decodeKeyEnd = { MediaTime(28, 1), MediaTime(24, 1) };
    samplesWithHigherDecodeTimes = decodeMap.findSamplesBetweenDecodeKeys(decodeKeyStart, decodeKeyEnd);
    EXPECT_EQ(MediaTime(25, 1), samplesWithHigherDecodeTimes.second->second->presentationTime());

    decodeKeyEnd = { MediaTime(28, 1), MediaTime(28, 1) };
    samplesWithHigherDecodeTimes = decodeMap.findSamplesBetweenDecodeKeys(decodeKeyStart, decodeKeyEnd);
    EXPECT_FALSE(samplesWithHigherDecodeTimes.first == samplesWithHigherDecodeTimes.second);
    EXPECT_EQ(decodeMap.end(), samplesWithHigherDecodeTimes.second);

    decodeKeyEnd = { MediaTime(29, 1), MediaTime(30, 1) };
    samplesWithHigherDecodeTimes = decodeMap.findSamplesBetweenDecodeKeys(decodeKeyStart, decodeKeyEnd);
    EXPECT_FALSE(samplesWithHigherDecodeTimes.first == samplesWithHigherDecodeTimes.second);
    EXPECT_EQ(decodeMap.end(), samplesWithHigherDecodeTimes.second);
}

}
#endif // ENABLE(MEDIA_SOURCE)
