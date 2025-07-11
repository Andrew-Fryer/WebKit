/*
 * Copyright (C) 2013-2025 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#if ENABLE(SPEECH_SYNTHESIS)

#include "PlatformSpeechSynthesisVoice.h"
#include <wtf/RefCounted.h>
#include <wtf/text/WTFString.h>

namespace WebCore {

class SpeechSynthesisVoice : public RefCounted<SpeechSynthesisVoice> {
public:
    virtual ~SpeechSynthesisVoice() = default;
    static Ref<SpeechSynthesisVoice> create(PlatformSpeechSynthesisVoice&);

    const String& voiceURI() const { return m_platformVoice->voiceURI(); }
    const String& name() const { return m_platformVoice->name(); }
    const String& lang() const { return m_platformVoice->lang(); }
    bool localService() const { return m_platformVoice->localService(); }
    bool isDefault() const { return m_platformVoice->isDefault(); }

    PlatformSpeechSynthesisVoice& platformVoice() { return m_platformVoice; }

private:
    explicit SpeechSynthesisVoice(PlatformSpeechSynthesisVoice&);

    const Ref<PlatformSpeechSynthesisVoice> m_platformVoice;
};

} // namespace WebCore

#endif // ENABLE(SPEECH_SYNTHESIS)
