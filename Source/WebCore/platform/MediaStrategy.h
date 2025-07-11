/*
 * Copyright (C) 2020-2025 Apple Inc. All rights reserved.
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

#pragma once

#if PLATFORM(COCOA) && ENABLE(MEDIA_RECORDER)
#include "MediaRecorderPrivateWriter.h"
#endif
#include "NativeImage.h"
#include "NowPlayingManager.h"
#include <wtf/CompletionHandler.h>
#include <wtf/Forward.h>

namespace WebCore {

class AudioDestination;
class AudioIOCallback;
class CDMFactory;
class NowPlayingManager;
class VideoFrame;

struct AudioDestinationCreationOptions;

class WEBCORE_EXPORT MediaStrategy {
public:
#if ENABLE(WEB_AUDIO)
    virtual Ref<AudioDestination> createAudioDestination(const AudioDestinationCreationOptions&) = 0;
#endif
    virtual std::unique_ptr<NowPlayingManager> createNowPlayingManager() const;
    void resetMediaEngines();
    virtual bool hasThreadSafeMediaSourceSupport() const;
#if ENABLE(MEDIA_SOURCE)
    virtual void enableMockMediaSource();
    bool mockMediaSourceEnabled() const;
    static void addMockMediaSourceEngine();
#endif

#if ENABLE(VIDEO)
    virtual void nativeImageFromVideoFrame(const VideoFrame&, CompletionHandler<void(std::optional<RefPtr<NativeImage>>&&)>&&);
#endif

    virtual bool enableWebMMediaPlayer() const { return true; }
    virtual bool isWebMediaStrategy() const { return false; }

protected:
    MediaStrategy();
    virtual ~MediaStrategy();
    bool m_mockMediaSourceEnabled { false };
};

inline void MediaStrategy::nativeImageFromVideoFrame(const VideoFrame&, CompletionHandler<void(std::optional<RefPtr<NativeImage>>&&)>&& completionHandler)
{
    completionHandler(std::nullopt);
}

} // namespace WebCore
