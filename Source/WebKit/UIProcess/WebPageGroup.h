/*
 * Copyright (C) 2010-2025 Apple Inc. All rights reserved.
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

#include "APIObject.h"
#include "IdentifierTypes.h"
#include "WebPageGroupData.h"
#include "WebProcessProxy.h"
#include <WebCore/UserStyleSheetTypes.h>
#include <wtf/CheckedRef.h>
#include <wtf/WeakHashSet.h>
#include <wtf/text/WTFString.h>

namespace WebKit {

class WebPreferences;
class WebPageProxy;

class WebPageGroup : public API::ObjectImpl<API::Object::Type::PageGroup>, public CanMakeWeakPtr<WebPageGroup> {
public:
    explicit WebPageGroup(const String& identifier = { });
    static Ref<WebPageGroup> create(const String& identifier = { });

    virtual ~WebPageGroup();

    PageGroupIdentifier pageGroupID() const { return m_data.pageGroupID; }

    const WebPageGroupData& data() const { return m_data; }

    WebPreferences& preferences() const { return m_preferences; }

private:
    WebPageGroupData m_data;
    const Ref<WebPreferences> m_preferences;
};

} // namespace WebKit
