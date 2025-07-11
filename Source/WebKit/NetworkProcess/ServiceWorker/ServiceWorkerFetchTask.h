/*
 * Copyright (C) 2019 Apple Inc. All rights reserved.
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

#include "DownloadID.h"
#include <WebCore/FetchIdentifier.h>
#include <WebCore/ResourceRequest.h>
#include <WebCore/ScriptExecutionContextIdentifier.h>
#include <WebCore/ServiceWorkerTypes.h>
#include <WebCore/Timer.h>
#include <pal/SessionID.h>
#include <wtf/RefCountedAndCanMakeWeakPtr.h>
#include <wtf/TZoneMalloc.h>

namespace WebCore {
class ResourceError;
class ResourceRequest;
class ResourceResponse;
class SWServerRegistration;
}

namespace IPC {
class Connection;
class Decoder;
class FormDataReference;
class SharedBufferReference;
}

namespace WebCore {
class NetworkLoadMetrics;
}

namespace WebKit {
class DownloadManager;
class NetworkResourceLoader;
class NetworkSession;
class ServiceWorkerNavigationPreloader;
class WebSWServerConnection;
class WebSWServerToContextConnection;

class ServiceWorkerFetchTask : public RefCountedAndCanMakeWeakPtr<ServiceWorkerFetchTask> {
    WTF_MAKE_TZONE_ALLOCATED(ServiceWorkerFetchTask);
public:
    static RefPtr<ServiceWorkerFetchTask> fromNavigationPreloader(WebSWServerConnection&, NetworkResourceLoader&, const WebCore::ResourceRequest&, NetworkSession*);

    static Ref<ServiceWorkerFetchTask> create(WebSWServerConnection&, NetworkResourceLoader&, WebCore::ResourceRequest&&, WebCore::SWServerConnectionIdentifier, WebCore::ServiceWorkerIdentifier, WebCore::SWServerRegistration&, NetworkSession*, bool isWorkerReady);
    static Ref<ServiceWorkerFetchTask> create(WebSWServerConnection&, NetworkResourceLoader&, std::unique_ptr<ServiceWorkerNavigationPreloader>&&);

    ~ServiceWorkerFetchTask();

    void start(WebSWServerToContextConnection&);
    void cancelFromClient();
    void didReceiveMessage(IPC::Connection&, IPC::Decoder&);

    void continueDidReceiveFetchResponse();
    void continueFetchTaskWith(WebCore::ResourceRequest&&);

    WebCore::FetchIdentifier fetchIdentifier() const { return m_fetchIdentifier; }
    std::optional<WebCore::ServiceWorkerIdentifier> serviceWorkerIdentifier() const { return m_serviceWorkerIdentifier; }

    WebCore::ResourceRequest takeRequest() { return WTFMove(m_currentRequest); }

    void cannotHandle();
    void contextClosed();

    bool convertToDownload(DownloadManager&, DownloadID, const WebCore::ResourceRequest&, const WebCore::ResourceResponse&);

    MonotonicTime startTime() const;

private:
    ServiceWorkerFetchTask(WebSWServerConnection&, NetworkResourceLoader&, WebCore::ResourceRequest&&, WebCore::SWServerConnectionIdentifier, WebCore::ServiceWorkerIdentifier, WebCore::SWServerRegistration&, NetworkSession*, bool isWorkerReady);
    ServiceWorkerFetchTask(WebSWServerConnection&, NetworkResourceLoader&, std::unique_ptr<ServiceWorkerNavigationPreloader>&&);

    enum class ShouldSetSource : bool { No, Yes };
    void didReceiveRedirectResponse(WebCore::ResourceResponse&&);
    void didReceiveResponse(WebCore::ResourceResponse&&, bool needsContinueDidReceiveResponseMessage);
    void didReceiveData(const IPC::SharedBufferReference&);
    void didReceiveDataFromPreloader(const WebCore::FragmentedSharedBuffer&);
    void didReceiveFormData(const IPC::FormDataReference&);
    void didFinish(const WebCore::NetworkLoadMetrics&);
    void didFail(const WebCore::ResourceError&);
    void didNotHandle();
    void usePreload();

    void processRedirectResponse(WebCore::ResourceResponse&&, ShouldSetSource);
    void processResponse(WebCore::ResourceResponse&&, bool needsContinueDidReceiveResponseMessage, ShouldSetSource);

    void startFetch();

    void timeoutTimerFired();
    void softUpdateIfNeeded();
    void loadResponseFromPreloader();
    void loadBodyFromPreloader();
    void cancelPreloadIfNecessary();
    NetworkSession* session();
    void preloadResponseIsReady();

    void workerClosed();

    RefPtr<IPC::Connection> serviceWorkerConnection();
    template<typename Message> bool sendToClient(Message&&);

    RefPtr<NetworkResourceLoader> protectedLoader() const;
    void sendNavigationPreloadUpdate();

    CheckedPtr<ServiceWorkerNavigationPreloader> checkedPreloader();

    WeakPtr<WebSWServerConnection> m_swServerConnection;
    WeakPtr<NetworkResourceLoader> m_loader;
    WeakPtr<WebSWServerToContextConnection> m_serviceWorkerConnection;
    WebCore::FetchIdentifier m_fetchIdentifier;
    Markable<WebCore::SWServerConnectionIdentifier> m_serverConnectionIdentifier;
    Markable<WebCore::ServiceWorkerIdentifier> m_serviceWorkerIdentifier;
    WebCore::ResourceRequest m_currentRequest;
    std::unique_ptr<WebCore::Timer> m_timeoutTimer;
    Markable<WebCore::ServiceWorkerRegistrationIdentifier> m_serviceWorkerRegistrationIdentifier;
    std::unique_ptr<ServiceWorkerNavigationPreloader> m_preloader;
    bool m_wasHandled { false };
    bool m_isDone { false };
    bool m_shouldSoftUpdate { false };
    bool m_isLoadingFromPreloader { false };
};

}
