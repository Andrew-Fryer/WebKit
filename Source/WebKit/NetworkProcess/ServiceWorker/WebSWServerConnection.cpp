/*
 * Copyright (C) 2017-2025 Apple Inc. All rights reserved.
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
#include "WebSWServerConnection.h"

#include "FormDataReference.h"
#include "Logging.h"
#include "MessageSenderInlines.h"
#include "NetworkConnectionToWebProcessMessages.h"
#include "NetworkNotificationManager.h"
#include "NetworkProcess.h"
#include "NetworkProcessProxyMessages.h"
#include "NetworkResourceLoader.h"
#include "NetworkSession.h"
#include "RemoteWorkerType.h"
#include "SharedBufferReference.h"
#include "SharedPreferencesForWebProcess.h"
#include "WebProcess.h"
#include "WebProcessMessages.h"
#include "WebResourceLoaderMessages.h"
#include "WebSWClientConnectionMessages.h"
#include "WebSWContextManagerConnectionMessages.h"
#include "WebSWServerConnectionMessages.h"
#include "WebSWServerToContextConnection.h"
#include <WebCore/CookieChangeSubscription.h>
#include <WebCore/ExceptionData.h>
#include <WebCore/LegacySchemeRegistry.h>
#include <WebCore/NotImplemented.h>
#include <WebCore/NotificationData.h>
#include <WebCore/SWServerRegistration.h>
#include <WebCore/ScriptExecutionContextIdentifier.h>
#include <WebCore/SecurityOrigin.h>
#include <WebCore/ServiceWorkerClientData.h>
#include <WebCore/ServiceWorkerContextData.h>
#include <WebCore/ServiceWorkerJobData.h>
#include <WebCore/ServiceWorkerUpdateViaCache.h>
#include <algorithm>
#include <cstdint>
#include <wtf/MainThread.h>
#include <wtf/TZoneMallocInlines.h>
#include <wtf/Vector.h>

namespace WebKit {
using namespace PAL;
using namespace WebCore;

#define SWSERVERCONNECTION_RELEASE_LOG_WITH_THIS(thisPtr, fmt, ...) RELEASE_LOG(ServiceWorker, "%p - WebSWServerConnection::" fmt, WTF::getPtr(thisPtr), ##__VA_ARGS__)
#define SWSERVERCONNECTION_RELEASE_LOG_ERROR_WITH_THIS(thisPtr, fmt, ...) RELEASE_LOG_ERROR(ServiceWorker, "%p - WebSWServerConnection::" fmt, WTF::getPtr(thisPtr), ##__VA_ARGS__)
#define SWSERVERCONNECTION_RELEASE_LOG(fmt, ...) RELEASE_LOG(ServiceWorker, "%p - WebSWServerConnection::" fmt, this, ##__VA_ARGS__)
#define SWSERVERCONNECTION_RELEASE_LOG_ERROR(fmt, ...) RELEASE_LOG_ERROR(ServiceWorker, "%p - WebSWServerConnection::" fmt, this, ##__VA_ARGS__)

#define MESSAGE_CHECK(assertion) MESSAGE_CHECK_BASE(assertion, m_contentConnection.get())

WTF_MAKE_TZONE_ALLOCATED_IMPL(WebSWServerConnection);

Ref<WebSWServerConnection> WebSWServerConnection::create(NetworkConnectionToWebProcess& networkConnectionToWebProcess, WebCore::SWServer& server, IPC::Connection& connection, WebCore::ProcessIdentifier processIdentifier)
{
    return adoptRef(*new WebSWServerConnection(networkConnectionToWebProcess, server, connection, processIdentifier));
}

WebSWServerConnection::WebSWServerConnection(NetworkConnectionToWebProcess& networkConnectionToWebProcess, SWServer& server, IPC::Connection& connection, ProcessIdentifier processIdentifier)
    : SWServer::Connection(server, processIdentifier)
    , m_networkConnectionToWebProcess(networkConnectionToWebProcess)
    , m_contentConnection(connection)
{
    if (CheckedPtr session = this->session())
        session->registerSWServerConnection(*this);
}

WebSWServerConnection::~WebSWServerConnection()
{
    if (CheckedPtr session = this->session())
        session->unregisterSWServerConnection(*this);
    if (RefPtr server = this->server()) {
        for (const auto& keyValue : m_clientOrigins)
            server->unregisterServiceWorkerClient(keyValue.value, keyValue.key);
    }
    for (auto& completionHandler : m_unregisterJobs.values())
        completionHandler(false);
}

NetworkProcess& WebSWServerConnection::networkProcess()
{
    return m_networkConnectionToWebProcess->networkProcess();
}

Ref<NetworkProcess> WebSWServerConnection::protectedNetworkProcess()
{
    return networkProcess();
}

std::optional<SharedPreferencesForWebProcess> WebSWServerConnection::sharedPreferencesForWebProcess() const
{
    if (!m_networkConnectionToWebProcess)
        return std::nullopt;

    return m_networkConnectionToWebProcess->sharedPreferencesForWebProcess();
}

void WebSWServerConnection::rejectJobInClient(ServiceWorkerJobIdentifier jobIdentifier, const ExceptionData& exceptionData)
{
    if (auto completionHandler = m_unregisterJobs.take(jobIdentifier))
        return completionHandler(makeUnexpected(exceptionData));
    send(Messages::WebSWClientConnection::JobRejectedInServer(jobIdentifier, exceptionData));
}

void WebSWServerConnection::resolveRegistrationJobInClient(ServiceWorkerJobIdentifier jobIdentifier, const ServiceWorkerRegistrationData& registrationData, ShouldNotifyWhenResolved shouldNotifyWhenResolved)
{
    send(Messages::WebSWClientConnection::RegistrationJobResolvedInServer(jobIdentifier, registrationData, shouldNotifyWhenResolved));
}

void WebSWServerConnection::resolveUnregistrationJobInClient(ServiceWorkerJobIdentifier jobIdentifier, const ServiceWorkerRegistrationKey& registrationKey, bool unregistrationResult)
{
    ASSERT(m_unregisterJobs.contains(jobIdentifier));
    if (auto completionHandler = m_unregisterJobs.take(jobIdentifier)) {
#if ENABLE(WEB_PUSH_NOTIFICATIONS)
        CheckedPtr checkedSession = session();
#if ENABLE(DECLARATIVE_WEB_PUSH)
        if (!checkedSession || checkedSession->isDeclarativeWebPushEnabled()) {
#else
        if (!checkedSession) {
#endif
            completionHandler(unregistrationResult);
            return;
        }

        auto scopeURL = registrationKey.scope();
        checkedSession->protectedNotificationManager()->unsubscribeFromPushService(WTFMove(scopeURL), std::nullopt, [completionHandler = WTFMove(completionHandler), unregistrationResult](auto&&) mutable {
            completionHandler(unregistrationResult);
        });

#else
        completionHandler(unregistrationResult);
#endif
    }
}

void WebSWServerConnection::startScriptFetchInClient(ServiceWorkerJobIdentifier jobIdentifier, const ServiceWorkerRegistrationKey& registrationKey, FetchOptions::Cache cachePolicy)
{
    send(Messages::WebSWClientConnection::StartScriptFetchForServer(jobIdentifier, registrationKey, cachePolicy));
}

void WebSWServerConnection::updateRegistrationStateInClient(ServiceWorkerRegistrationIdentifier identifier, ServiceWorkerRegistrationState state, const std::optional<ServiceWorkerData>& serviceWorkerData)
{
    send(Messages::WebSWClientConnection::UpdateRegistrationState(identifier, state, serviceWorkerData));
    if (RefPtr contextConnection = m_networkConnectionToWebProcess->swContextConnection())
        sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::UpdateRegistrationState(identifier, state, serviceWorkerData));
}

void WebSWServerConnection::fireUpdateFoundEvent(ServiceWorkerRegistrationIdentifier identifier)
{
    send(Messages::WebSWClientConnection::FireUpdateFoundEvent(identifier));
    if (RefPtr contextConnection = m_networkConnectionToWebProcess->swContextConnection())
        sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::FireUpdateFoundEvent(identifier));
}

void WebSWServerConnection::setRegistrationLastUpdateTime(ServiceWorkerRegistrationIdentifier identifier, WallTime lastUpdateTime)
{
    send(Messages::WebSWClientConnection::SetRegistrationLastUpdateTime(identifier, lastUpdateTime));
    if (RefPtr contextConnection = m_networkConnectionToWebProcess->swContextConnection())
        sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::SetRegistrationLastUpdateTime(identifier, lastUpdateTime));
}

void WebSWServerConnection::setRegistrationUpdateViaCache(ServiceWorkerRegistrationIdentifier identifier, ServiceWorkerUpdateViaCache updateViaCache)
{
    send(Messages::WebSWClientConnection::SetRegistrationUpdateViaCache(identifier, updateViaCache));
    if (RefPtr contextConnection = m_networkConnectionToWebProcess->swContextConnection())
        sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::SetRegistrationUpdateViaCache(identifier, updateViaCache));
}

void WebSWServerConnection::notifyClientsOfControllerChange(const HashSet<ScriptExecutionContextIdentifier>& contextIdentifiers, const std::optional<ServiceWorkerData>& newController)
{
    send(Messages::WebSWClientConnection::NotifyClientsOfControllerChange(contextIdentifiers, newController));
}

void WebSWServerConnection::updateWorkerStateInClient(ServiceWorkerIdentifier worker, ServiceWorkerState state)
{
    send(Messages::WebSWClientConnection::UpdateWorkerState(worker, state));
    if (RefPtr contextConnection = m_networkConnectionToWebProcess->swContextConnection())
        sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::UpdateWorkerState(worker, state));
}

void WebSWServerConnection::controlClient(const NetworkResourceLoadParameters& parameters, SWServerRegistration& registration, const ResourceRequest& request, WebCore::ProcessIdentifier webProcessIdentifier)
{
    ServiceWorkerClientType clientType;
    if (parameters.options.destination  == FetchOptions::Destination::Worker)
        clientType = ServiceWorkerClientType::Worker;
    else if (parameters.options.destination  == FetchOptions::Destination::Sharedworker)
        clientType = ServiceWorkerClientType::Sharedworker;
    else
        clientType = ServiceWorkerClientType::Window;

    ASSERT(parameters.options.resultingClientIdentifier);
    ScriptExecutionContextIdentifier clientIdentifier { *parameters.options.resultingClientIdentifier, webProcessIdentifier };

    // As per step 12 of https://w3c.github.io/ServiceWorker/#on-fetch-request-algorithm, the active service worker should be controlling the document.
    // We register the service worker client using the identifier provided by DocumentLoader and notify DocumentLoader about it.
    // If notification is successful, DocumentLoader is responsible to unregister the service worker client as needed.
    sendWithAsyncReply(Messages::WebSWClientConnection::SetServiceWorkerClientIsControlled { clientIdentifier, registration.data() }, [weakThis = WeakPtr { *this }, clientIdentifier](bool isSuccess) {
        RefPtr protectedThis = weakThis.get();
        if (!protectedThis || isSuccess)
            return;

        protectedThis->unregisterServiceWorkerClient(clientIdentifier);
    });

    RefPtr server = this->server();
    if (!server)
        return;

    auto ancestorOrigins = map(parameters.frameAncestorOrigins, [](auto& origin) { return origin->toString(); });
    auto advancedPrivacyProtections = server->advancedPrivacyProtectionsFromClient(registration.key().clientOrigin());
    ServiceWorkerClientData data { clientIdentifier, clientType, ServiceWorkerClientFrameType::None, request.url(), URL(), parameters.webPageID, parameters.webFrameID, request.isAppInitiated() ? WebCore::LastNavigationWasAppInitiated::Yes : WebCore::LastNavigationWasAppInitiated::No, advancedPrivacyProtections, false, false, 0, WTFMove(ancestorOrigins) };

    registerServiceWorkerClientInternal(ClientOrigin { registration.key().topOrigin(), SecurityOriginData::fromURLWithoutStrictOpaqueness(request.url()) }, WTFMove(data), registration.identifier(), request.httpUserAgent(), WebCore::SWServer::IsBeingCreatedClient::Yes);
}

RefPtr<ServiceWorkerFetchTask> WebSWServerConnection::createFetchTask(NetworkResourceLoader& loader, const ResourceRequest& request)
{
    if (loader.parameters().serviceWorkersMode == ServiceWorkersMode::None) {
        if (loader.parameters().request.requester() == ResourceRequestRequester::Fetch && isNavigationRequest(loader.parameters().options.destination)) {
            if (auto task = ServiceWorkerFetchTask::fromNavigationPreloader(*this, loader, request, session()))
                return task;
        }
        return nullptr;
    }

    RefPtr server = this->server();
    if (!server)
        return nullptr;

    if (!server->canHandleScheme(loader.originalRequest().url().protocol()))
        return nullptr;

    std::optional<ServiceWorkerRegistrationIdentifier> serviceWorkerRegistrationIdentifier;
    if (auto resultingClientIdentifier = loader.parameters().options.resultingClientIdentifier) {
        auto topOrigin = loader.parameters().isMainFrameNavigation ? SecurityOriginData::fromURLWithoutStrictOpaqueness(request.url()) : loader.parameters().topOrigin->data();
        RefPtr registration = doRegistrationMatching(topOrigin, request.url());
        if (!registration)
            return nullptr;

        serviceWorkerRegistrationIdentifier = registration->identifier();
        controlClient(loader.parameters(), *registration, request, loader.connectionToWebProcess().webProcessIdentifier());
        loader.setServiceWorkerRegistration(*registration);
    } else {
        if (!loader.parameters().serviceWorkerRegistrationIdentifier)
            return nullptr;

        if (isPotentialNavigationOrSubresourceRequest(loader.parameters().options.destination))
            return nullptr;

        serviceWorkerRegistrationIdentifier = *loader.parameters().serviceWorkerRegistrationIdentifier;
    }

    RefPtr registration = server->getRegistration(*serviceWorkerRegistrationIdentifier);
    RefPtr worker = registration ? registration->activeWorker() : nullptr;
    if (!worker) {
        SWSERVERCONNECTION_RELEASE_LOG_ERROR("startFetch: DidNotHandle because no active worker for registration %" PRIu64, serviceWorkerRegistrationIdentifier->toUInt64());
        return nullptr;
    }

    // FIXME: Add support for cache route w/o cacheName, for now we go to fetch event.
    auto routerSource = worker->getRouterSource(loader.parameters().options, request);
    if (std::holds_alternative<RouterSourceEnum>(routerSource)) {
        switch (std::get<RouterSourceEnum>(routerSource)) {
        case RouterSourceEnum::Cache:
        case RouterSourceEnum::FetchEvent:
            break;
        case RouterSourceEnum::Network:
            if (registration->shouldSoftUpdate(loader.parameters().options))
                registration->scheduleSoftUpdate(loader.isAppInitiated() ? WebCore::IsAppInitiated::Yes : WebCore::IsAppInitiated::No);
            return nullptr;
        }
    }

    if (worker->hasTimedOutAnyFetchTasks()) {
        SWSERVERCONNECTION_RELEASE_LOG_ERROR("startFetch: DidNotHandle because worker %" PRIu64 " has some timeouts", worker->identifier().toUInt64());
        return nullptr;
    }

    bool isWorkerReady = worker->isRunning() && worker->state() == ServiceWorkerState::Activated;
    Ref task = ServiceWorkerFetchTask::create(*this, loader, ResourceRequest { request }, identifier(), worker->identifier(), *registration, session(), isWorkerReady);
    startFetch(task, *worker);
    return task;
}

void WebSWServerConnection::startFetch(ServiceWorkerFetchTask& task, SWServerWorker& worker)
{
    auto runServerWorkerAndStartFetch = [weakThis = WeakPtr { *this }, task = WeakPtr { task }](bool success) mutable {
        if (!task)
            return;

        RefPtr protectedThis = weakThis.get();
        if (!protectedThis) {
            task->cannotHandle();
            return;
        }

        if (!success) {
            SWSERVERCONNECTION_RELEASE_LOG_ERROR_WITH_THIS(protectedThis, "startFetch: fetchIdentifier=%" PRIu64 " DidNotHandle because worker did not become activated", task->fetchIdentifier().toUInt64());
            task->cannotHandle();
            return;
        }

        RefPtr server = protectedThis->server();
        if (!server)
            return;

        RefPtr worker = server->workerByID(*task->serviceWorkerIdentifier());
        if (!worker || worker->hasTimedOutAnyFetchTasks()) {
            task->cannotHandle();
            return;
        }

        if (!worker->contextConnection())
            server->createContextConnection(worker->topSite(), worker->serviceWorkerPageIdentifier());

        auto identifier = *task->serviceWorkerIdentifier();
        server->runServiceWorkerIfNecessary(identifier, [weakThis = WTFMove(weakThis), task = WTFMove(task)](auto* contextConnection) mutable {
            if (!task)
                return;

            RefPtr protectedThis = weakThis.get();
            if (!protectedThis) {
                task->cannotHandle();
                return;
            }

            if (!contextConnection) {
                SWSERVERCONNECTION_RELEASE_LOG_ERROR_WITH_THIS(protectedThis, "startFetch: fetchIdentifier=%s DidNotHandle because failed to run service worker", task->fetchIdentifier().loggingString().utf8().data());
                task->cannotHandle();
                return;
            }
            SWSERVERCONNECTION_RELEASE_LOG_WITH_THIS(protectedThis, "startFetch: Starting fetch %" PRIu64 " via service worker %" PRIu64, task->fetchIdentifier().toUInt64(), task->serviceWorkerIdentifier()->toUInt64());
            downcast<WebSWServerToContextConnection>(*contextConnection).startFetch(*task);
        });
    };

    worker.whenActivated(WTFMove(runServerWorkerAndStartFetch));
}

void WebSWServerConnection::postMessageToServiceWorker(ServiceWorkerIdentifier destinationIdentifier, MessageWithMessagePorts&& message, const ServiceWorkerOrClientIdentifier& sourceIdentifier)
{
    RefPtr server = this->server();
    if (!server)
        return;

    RefPtr destinationWorker = server->workerByID(destinationIdentifier);
    if (!destinationWorker)
        return;

    std::optional<ServiceWorkerOrClientData> sourceData;
    WTF::switchOn(sourceIdentifier, [&](ServiceWorkerIdentifier identifier) {
        if (RefPtr sourceWorker = server->workerByID(identifier))
            sourceData = ServiceWorkerOrClientData { sourceWorker->data() };
    }, [&](ScriptExecutionContextIdentifier identifier) {
        if (auto clientData = destinationWorker->findClientByIdentifier(identifier))
            sourceData = ServiceWorkerOrClientData { *clientData };
    });

    if (!sourceData)
        return;

    // It's possible this specific worker cannot be re-run (e.g. its registration has been removed)
    server->runServiceWorkerIfNecessary(destinationIdentifier, [destinationIdentifier, message = WTFMove(message), sourceData = WTFMove(*sourceData)](auto* contextConnection) mutable {
        if (contextConnection)
            sendToContextProcess(*contextConnection, Messages::WebSWContextManagerConnection::PostMessageToServiceWorker { destinationIdentifier, WTFMove(message), WTFMove(sourceData) });
    });
}

void WebSWServerConnection::scheduleJobInServer(ServiceWorkerJobData&& jobData)
{
    checkTopOrigin(jobData.topOrigin);

    ASSERT(!jobData.scopeURL.isNull());
    if (jobData.scopeURL.isNull()) {
        rejectJobInClient(jobData.identifier().jobIdentifier, ExceptionData { ExceptionCode::InvalidStateError, "Scope URL is empty"_s });
        return;
    }

    SWSERVERCONNECTION_RELEASE_LOG("Scheduling ServiceWorker job %s in server", jobData.identifier().loggingString().utf8().data());
    ASSERT(identifier() == jobData.connectionIdentifier());

    if (RefPtr server = this->server())
        server->scheduleJob(WTFMove(jobData));
}

URL WebSWServerConnection::clientURLFromIdentifier(ServiceWorkerOrClientIdentifier contextIdentifier)
{
    RefPtr server = this->server();
    if (!server)
        return { };

    return WTF::switchOn(contextIdentifier, [&](ScriptExecutionContextIdentifier clientIdentifier) -> URL {
        auto iterator = m_clientOrigins.find(clientIdentifier);
        if (iterator == m_clientOrigins.end())
            return { };

        auto clientData = server->serviceWorkerClientWithOriginByID(iterator->value, clientIdentifier);
        if (!clientData)
            return { };

        return clientData->url;
    }, [&](ServiceWorkerIdentifier serviceWorkerIdentifier) -> URL {
        RefPtr worker = server->workerByID(serviceWorkerIdentifier);
        if (!worker)
            return { };
        return worker->data().scriptURL;
    });
}

void WebSWServerConnection::scheduleUnregisterJobInServer(ServiceWorkerJobIdentifier jobIdentifier, ServiceWorkerRegistrationIdentifier registrationIdentifier, ServiceWorkerOrClientIdentifier contextIdentifier, CompletionHandler<void(UnregisterJobResult&&)>&& completionHandler)
{
    SWSERVERCONNECTION_RELEASE_LOG("Scheduling unregister ServiceWorker job in server");

    RefPtr server = this->server();
    if (!server)
        return completionHandler(false);

    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration)
        return completionHandler(false);

    auto clientURL = clientURLFromIdentifier(contextIdentifier);
    if (!clientURL.isValid())
        return completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Client is unknown"_s }));

    ASSERT(!m_unregisterJobs.contains(jobIdentifier));
    m_unregisterJobs.add(jobIdentifier, WTFMove(completionHandler));

    server->scheduleUnregisterJob(ServiceWorkerJobDataIdentifier { identifier(), jobIdentifier }, *registration, contextIdentifier, WTFMove(clientURL));
}

void WebSWServerConnection::postMessageToServiceWorkerClient(ScriptExecutionContextIdentifier destinationContextIdentifier, const MessageWithMessagePorts& message, ServiceWorkerIdentifier sourceIdentifier, const String& sourceOrigin)
{
    RefPtr server = this->server();
    if (!server)
        return;

    server->postMessageToServiceWorkerClient(destinationContextIdentifier, message, sourceIdentifier, sourceOrigin, [protectedThis = Ref { *this }] (auto destinationContextIdentifier, auto& message, auto sourceServiceWorkerData, auto& sourceOrigin) {
        protectedThis->send(Messages::WebSWClientConnection::PostMessageToServiceWorkerClient { destinationContextIdentifier, message, sourceServiceWorkerData, sourceOrigin }, 0);
    });
}

void WebSWServerConnection::matchRegistration(const SecurityOriginData& topOrigin, const URL& clientURL, CompletionHandler<void(std::optional<ServiceWorkerRegistrationData>&&)>&& callback)
{
    checkTopOrigin(topOrigin);

    if (RefPtr registration = doRegistrationMatching(topOrigin, clientURL)) {
        callback(registration->data());
        return;
    }
    callback({ });
}

void WebSWServerConnection::whenRegistrationReady(const WebCore::SecurityOriginData& topOrigin, const URL& clientURL, CompletionHandler<void(std::optional<WebCore::ServiceWorkerRegistrationData>&&)>&& callback)
{
    checkTopOrigin(topOrigin);

    SWServer::Connection::whenRegistrationReady(topOrigin, clientURL, WTFMove(callback));
}

void WebSWServerConnection::getRegistrations(const SecurityOriginData& topOrigin, const URL& clientURL, CompletionHandler<void(const Vector<ServiceWorkerRegistrationData>&)>&& callback)
{
    checkTopOrigin(topOrigin);

    if (RefPtr server = this->server())
        callback(server->getRegistrations(topOrigin, clientURL));
    else
        callback({ });
}

void WebSWServerConnection::registerServiceWorkerClient(WebCore::ClientOrigin&& clientOrigin, ServiceWorkerClientData&& data, const std::optional<ServiceWorkerRegistrationIdentifier>& controllingServiceWorkerRegistrationIdentifier, String&& userAgent)
{
    MESSAGE_CHECK(data.identifier.processIdentifier() == identifier());
    checkTopOrigin(clientOrigin.topOrigin);

    registerServiceWorkerClientInternal(WTFMove(clientOrigin), WTFMove(data), controllingServiceWorkerRegistrationIdentifier, WTFMove(userAgent), SWServer::IsBeingCreatedClient::No);
}

void WebSWServerConnection::registerServiceWorkerClientInternal(WebCore::ClientOrigin&& clientOrigin, ServiceWorkerClientData&& data, const std::optional<ServiceWorkerRegistrationIdentifier>& controllingServiceWorkerRegistrationIdentifier, String&& userAgent, WebCore::SWServer::IsBeingCreatedClient isBeingCreatedClient)
{
    auto& contextOrigin = clientOrigin.clientOrigin;
    if (data.url.protocolIsInHTTPFamily()) {
        // We do not register any sandbox document.
        if (contextOrigin != SecurityOriginData::fromURLWithoutStrictOpaqueness(data.url))
            return;
    }

    MESSAGE_CHECK(!contextOrigin.isNull());

    bool isNewOrigin = std::ranges::all_of(m_clientOrigins.values(), [&contextOrigin](auto& origin) {
        return contextOrigin != origin.clientOrigin;
    });
    RefPtr server = this->server();
    if (!server)
        return;

    RefPtr contextConnection = isNewOrigin ? server->contextConnectionForRegistrableDomain(RegistrableDomain { contextOrigin }) : nullptr;

    m_clientOrigins.add(data.identifier, clientOrigin);

    if (isBeingCreatedClient == SWServer::IsBeingCreatedClient::No) {
        for (auto&& pendingMessage : server->releaseServiceWorkerClientPendingMessage(data.identifier))
            send(Messages::WebSWClientConnection::PostMessageToServiceWorkerClient { data.identifier, pendingMessage.message, pendingMessage.sourceData, pendingMessage.sourceOrigin }, 0);
    }

    server->registerServiceWorkerClient(WTFMove(clientOrigin), WTFMove(data), controllingServiceWorkerRegistrationIdentifier, WTFMove(userAgent), isBeingCreatedClient);

    if (!m_isThrottleable)
        updateThrottleState();

    if (contextConnection) {
        auto& connection = downcast<WebSWServerToContextConnection>(*contextConnection);
        protectedNetworkProcess()->protectedParentProcessConnection()->send(Messages::NetworkProcessProxy::RegisterRemoteWorkerClientProcess { RemoteWorkerType::ServiceWorker, identifier(), connection.webProcessIdentifier() }, 0);
    }
}

void WebSWServerConnection::unregisterServiceWorkerClient(const ScriptExecutionContextIdentifier& clientIdentifier)
{
    MESSAGE_CHECK(clientIdentifier.processIdentifier() == identifier());
    auto iterator = m_clientOrigins.find(clientIdentifier);
    if (iterator == m_clientOrigins.end())
        return;

    auto clientOrigin = iterator->value;

    RefPtr server = this->server();
    if (!server)
        return;

    server->unregisterServiceWorkerClient(clientOrigin, clientIdentifier);
    m_clientOrigins.remove(iterator);

    if (!m_isThrottleable)
        updateThrottleState();

    bool isDeletedOrigin = std::ranges::all_of(m_clientOrigins.values(), [&clientOrigin](auto& origin) {
        return clientOrigin.clientOrigin != origin.clientOrigin;
    });

    if (isDeletedOrigin) {
        RegistrableDomain potentiallyRemovedDomain { clientOrigin.clientOrigin };
        if (!hasMatchingClient(potentiallyRemovedDomain)) {
            if (RefPtr contextConnection = server->contextConnectionForRegistrableDomain(potentiallyRemovedDomain)) {
                auto& connection = downcast<WebSWServerToContextConnection>(*contextConnection);
                protectedNetworkProcess()->protectedParentProcessConnection()->send(Messages::NetworkProcessProxy::UnregisterRemoteWorkerClientProcess { RemoteWorkerType::ServiceWorker, identifier(), connection.webProcessIdentifier() }, 0);
            }
        }
    }
}

bool WebSWServerConnection::hasMatchingClient(const RegistrableDomain& domain) const
{
    return std::ranges::any_of(m_clientOrigins.values(), [&domain](auto& origin) {
        return domain.matches(origin.clientOrigin);
    });
}

bool WebSWServerConnection::computeThrottleState(const RegistrableDomain& domain) const
{
    RefPtr server = this->server();
    if (!server)
        return true;

    return std::ranges::all_of(server->connections().values(), [&domain](auto& serverConnection) {
        Ref connection = downcast<WebSWServerConnection>(serverConnection.get());
        return connection->isThrottleable() || !connection->hasMatchingClient(domain);
    });
}

void WebSWServerConnection::setThrottleState(bool isThrottleable)
{
    m_isThrottleable = isThrottleable;
    updateThrottleState();
}

void WebSWServerConnection::updateThrottleState()
{
    HashSet<SecurityOriginData> origins;
    for (auto& origin : m_clientOrigins.values())
        origins.add(origin.clientOrigin);

    RefPtr server = this->server();
    if (!server)
        return;

    for (auto& origin : origins) {
        if (RefPtr contextConnection = server->contextConnectionForRegistrableDomain(RegistrableDomain { origin })) {
            auto& connection = downcast<WebSWServerToContextConnection>(*contextConnection);

            if (connection.isThrottleable() == m_isThrottleable)
                continue;
            bool newThrottleState = computeThrottleState(connection.registrableDomain());
            if (connection.isThrottleable() == newThrottleState)
                continue;
            connection.setThrottleState(newThrottleState);
        }
    }
}

void WebSWServerConnection::subscribeToPushService(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, Vector<uint8_t>&& applicationServerKey, CompletionHandler<void(Expected<PushSubscriptionData, ExceptionData>&&)>&& completionHandler)
{
#if !ENABLE(WEB_PUSH_NOTIFICATIONS)
    UNUSED_PARAM(registrationIdentifier);
    UNUSED_PARAM(applicationServerKey);
    completionHandler(makeUnexpected(ExceptionData { ExceptionCode::AbortError, "Push service not implemented"_s }));
#else
    RefPtr server = this->server();
    if (!server) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Subscribing for push requires a server"_s }));
        return;
    }

    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Subscribing for push requires an active service worker"_s }));
        return;
    }

    if (!session()) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "No active network session"_s }));
        return;
    }

    session()->protectedNotificationManager()->subscribeToPushService(registration->scopeURLWithoutFragment(), WTFMove(applicationServerKey), [weakThis = WeakPtr { *this }, completionHandler = WTFMove(completionHandler), registrableDomain = RegistrableDomain(registration->data().scopeURL)] (Expected<PushSubscriptionData, ExceptionData>&& result) mutable {
        if (RefPtr resourceLoadStatistics = weakThis && weakThis->session() ? weakThis->session()->resourceLoadStatistics() : nullptr; result && resourceLoadStatistics) {
            return resourceLoadStatistics->setMostRecentWebPushInteractionTime(WTFMove(registrableDomain), [result = WTFMove(result), completionHandler = WTFMove(completionHandler)] () mutable {
                completionHandler(WTFMove(result));
            });
        }
        completionHandler(WTFMove(result));
    });
#endif
}

void WebSWServerConnection::unsubscribeFromPushService(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, WebCore::PushSubscriptionIdentifier subscriptionIdentifier, CompletionHandler<void(Expected<bool, ExceptionData>&&)>&& completionHandler)
{
#if !ENABLE(WEB_PUSH_NOTIFICATIONS)
    UNUSED_PARAM(registrationIdentifier);
    UNUSED_PARAM(subscriptionIdentifier);

    completionHandler(false);
#else
    RefPtr server = this->server();
    if (!server) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Unsubscribing from push requires a server"_s }));
        return;
    }

    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Unsubscribing from push requires a service worker"_s }));
        return;
    }

    if (!session()) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "No active network session"_s }));
        return;
    }

    session()->protectedNotificationManager()->unsubscribeFromPushService(registration->scopeURLWithoutFragment(), subscriptionIdentifier, WTFMove(completionHandler));
#endif
}

void WebSWServerConnection::getPushSubscription(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, CompletionHandler<void(Expected<std::optional<PushSubscriptionData>, ExceptionData>&&)>&& completionHandler)
{
#if !ENABLE(WEB_PUSH_NOTIFICATIONS)
    UNUSED_PARAM(registrationIdentifier);

    completionHandler(std::optional<PushSubscriptionData>(std::nullopt));
#else
    RefPtr server = this->server();
    if (!server) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Getting push subscription requires a server"_s }));
        return;
    }

    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Getting push subscription requires a service worker"_s }));
        return;
    }

    if (!session()) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "No active network session"_s }));
        return;
    }

    session()->protectedNotificationManager()->getPushSubscription(registration->scopeURLWithoutFragment(), WTFMove(completionHandler));
#endif
}

void WebSWServerConnection::getPushPermissionState(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, CompletionHandler<void(Expected<uint8_t, ExceptionData>&&)>&& completionHandler)
{
#if !ENABLE(WEB_PUSH_NOTIFICATIONS)
    UNUSED_PARAM(registrationIdentifier);

    completionHandler(static_cast<uint8_t>(PushPermissionState::Denied));
#else
    RefPtr server = this->server();
    if (!server) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Getting push permission state requires a server"_s }));
        return;
    }

    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "Getting push permission state requires a service worker"_s }));
        return;
    }

    if (!session()) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "No active network session"_s }));
        return;
    }

    session()->protectedNotificationManager()->getPermissionState(SecurityOriginData::fromURL( registration->scopeURLWithoutFragment()), [completionHandler = WTFMove(completionHandler)](WebCore::PushPermissionState state) mutable {
        completionHandler(static_cast<uint8_t>(state));
    });
#endif
}

void WebSWServerConnection::contextConnectionCreated(SWServerToContextConnection& contextConnection)
{
    auto& connection =  downcast<WebSWServerToContextConnection>(contextConnection);
    connection.setThrottleState(computeThrottleState(connection.registrableDomain()));

    if (hasMatchingClient(connection.registrableDomain()))
        networkProcess().protectedParentProcessConnection()->send(Messages::NetworkProcessProxy::RegisterRemoteWorkerClientProcess { RemoteWorkerType::ServiceWorker, identifier(), connection.webProcessIdentifier() }, 0);
}

void WebSWServerConnection::terminateWorkerFromClient(ServiceWorkerIdentifier serviceWorkerIdentifier, CompletionHandler<void()>&& callback)
{
    RefPtr server = this->server();
    if (!server)
        return callback();
    RefPtr worker = server->workerByID(serviceWorkerIdentifier);
    if (!worker)
        return callback();
    worker->terminate(WTFMove(callback));
}

void WebSWServerConnection::whenServiceWorkerIsTerminatedForTesting(WebCore::ServiceWorkerIdentifier identifier, CompletionHandler<void()>&& completionHandler)
{
    RefPtr worker = SWServerWorker::existingWorkerForIdentifier(identifier);
    if (!worker || worker->isNotRunning())
        return completionHandler();
    worker->whenTerminated(WTFMove(completionHandler));
}

PAL::SessionID WebSWServerConnection::sessionID() const
{
    return server()->sessionID();
}

NetworkSession* WebSWServerConnection::session()
{
    return protectedNetworkProcess()->networkSession(sessionID());
}

template<typename U> void WebSWServerConnection::sendToContextProcess(WebCore::SWServerToContextConnection& connection, U&& message)
{
    Ref { downcast<WebSWServerToContextConnection>(connection) }->send(WTFMove(message));
}

void WebSWServerConnection::fetchTaskTimedOut(ServiceWorkerIdentifier serviceWorkerIdentifier)
{
    RefPtr server = this->server();
    if (!server)
        return;

    RefPtr worker = server->workerByID(serviceWorkerIdentifier);
    if (!worker)
        return;

    worker->setHasTimedOutAnyFetchTasks();
    worker->terminate();
}

void WebSWServerConnection::enableNavigationPreload(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, ExceptionOrVoidCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No server"_s });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No registration"_s });
        return;
    }
    callback(registration->enableNavigationPreload());
}

void WebSWServerConnection::disableNavigationPreload(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, ExceptionOrVoidCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No server"_s });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No registration"_s });
        return;
    }
    callback(registration->disableNavigationPreload());
}

void WebSWServerConnection::setNavigationPreloadHeaderValue(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, String&& headerValue, ExceptionOrVoidCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No server"_s });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        callback(ExceptionData { ExceptionCode::InvalidStateError, "No registration"_s });
        return;
    }
    callback(registration->setNavigationPreloadHeaderValue(WTFMove(headerValue)));
}

void WebSWServerConnection::getNavigationPreloadState(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, ExceptionOrNavigationPreloadStateCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, { } }));
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        callback(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, { } }));
        return;
    }
    callback(registration->navigationPreloadState());
}

void WebSWServerConnection::focusServiceWorkerClient(WebCore::ScriptExecutionContextIdentifier clientIdentifier, CompletionHandler<void(std::optional<ServiceWorkerClientData>&&)>&& callback)
{
    sendWithAsyncReply(Messages::WebSWClientConnection::FocusServiceWorkerClient { clientIdentifier }, WTFMove(callback));
}

void WebSWServerConnection::transferServiceWorkerLoadToNewWebProcess(NetworkResourceLoader& loader, WebCore::SWServerRegistration& registration, const WebCore::ResourceRequest& request)
{
    controlClient(loader.parameters(), registration, request, loader.connectionToWebProcess().webProcessIdentifier());
}

std::optional<SWServer::GatheredClientData> WebSWServerConnection::gatherClientData(ScriptExecutionContextIdentifier clientIdentifier)
{
    ASSERT(clientIdentifier.processIdentifier() == identifier());
    auto iterator = m_clientOrigins.find(clientIdentifier);
    if (iterator == m_clientOrigins.end())
        return { };

    RefPtr server = this->server();
    if (!server)
        return { };

    return server->gatherClientData(iterator->value, clientIdentifier);
}

void WebSWServerConnection::updateBackgroundFetchRegistration(const WebCore::BackgroundFetchInformation& information)
{
    send(Messages::WebSWClientConnection::UpdateBackgroundFetchRegistration(information));
}

void WebSWServerConnection::retrieveRecordResponseBody(WebCore::BackgroundFetchRecordIdentifier recordIdentifier, RetrieveRecordResponseBodyCallbackIdentifier callbackIdentifier)
{
    SWServer::Connection::retrieveRecordResponseBody(recordIdentifier, [weakThis = WeakPtr { *this }, callbackIdentifier](auto&& result) {
        RefPtr protectedThis = weakThis.get();
        if (!protectedThis)
            return;
        if (!result.has_value()) {
            protectedThis->send(Messages::WebSWClientConnection::NotifyRecordResponseBodyEnd(callbackIdentifier, result.error()));
            return;
        }
        protectedThis->send(Messages::WebSWClientConnection::NotifyRecordResponseBodyChunk(callbackIdentifier, IPC::SharedBufferReference(WTFMove(result.value()))));
    });
}

void WebSWServerConnection::addCookieChangeSubscriptions(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, Vector<WebCore::CookieChangeSubscription>&& subscriptions, ExceptionOrVoidCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback({ });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        SWSERVERCONNECTION_RELEASE_LOG_ERROR("AddCookieChangeSubscriptions: Did not handle because no valid registration for registration identifier %" PRIu64, registrationIdentifier.toUInt64());
        // FIXME: Spec is not clear about this case at this point and wpt test expects no error (https://github.com/WICG/cookie-store/issues/237).
        callback({ });
        return;
    }

    registration->addCookieChangeSubscriptions(WTFMove(subscriptions));
    callback({ });
}

void WebSWServerConnection::removeCookieChangeSubscriptions(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, Vector<WebCore::CookieChangeSubscription>&& subscriptions, ExceptionOrVoidCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback({ });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        SWSERVERCONNECTION_RELEASE_LOG_ERROR("RemoveCookieChangeSubscriptions: Did not handle because no valid registration for registration identifier %" PRIu64, registrationIdentifier.toUInt64());
        // FIXME: Spec is not clear about this case at this point and wpt test expects no error (https://github.com/WICG/cookie-store/issues/237).
        callback({ });
        return;
    }

    registration->removeCookieChangeSubscriptions(WTFMove(subscriptions));
    callback({ });
}

void WebSWServerConnection::cookieChangeSubscriptions(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, ExceptionOrCookieChangeSubscriptionsCallback&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback({ });
        return;
    }
    RefPtr registration = server->getRegistration(registrationIdentifier);
    if (!registration) {
        SWSERVERCONNECTION_RELEASE_LOG_ERROR("CookieChangeSubscriptions: Did not handle because no valid registration for registration identifier %" PRIu64, registrationIdentifier.toUInt64());
        // FIXME: Spec is not clear about this case at this point and wpt test expects no error (https://github.com/WICG/cookie-store/issues/237).
        callback({ });
        return;
    }

    callback(registration->cookieChangeSubscriptions());
}

void WebSWServerConnection::addRoutes(WebCore::ServiceWorkerRegistrationIdentifier registrationIdentifier, Vector<WebCore::ServiceWorkerRoute>&& routes, CompletionHandler<void(Expected<void, WebCore::ExceptionData>&&)>&& callback)
{
    RefPtr server = this->server();
    if (!server) {
        callback(makeUnexpected(WebCore::ExceptionData { ExceptionCode::TypeError, "Internal error"_s }));
        return;
    }
    server->addRoutes(registrationIdentifier, WTFMove(routes), WTFMove(callback));
}

#if ENABLE(WEB_PUSH_NOTIFICATIONS)
void WebSWServerConnection::getNotifications(const URL& registrationURL, const String& tag, CompletionHandler<void(Expected<Vector<WebCore::NotificationData>, WebCore::ExceptionData>&&)>&& completionHandler)
{
    if (!session()) {
        completionHandler(makeUnexpected(ExceptionData { ExceptionCode::InvalidStateError, "No active network session"_s }));
        return;
    }

    session()->protectedNotificationManager()->getNotifications(registrationURL, tag, WTFMove(completionHandler));
}
#endif

#if ENABLE(CONTENT_EXTENSIONS)
void WebSWServerConnection::reportNetworkUsageToWorkerClient(WebCore::ScriptExecutionContextIdentifier identifier, uint64_t bytesTransferredOverNetworkDelta)
{
    send(Messages::WebSWClientConnection::ReportNetworkUsageToWorkerClient(identifier, bytesTransferredOverNetworkDelta));
}
#endif

void WebSWServerConnection::checkTopOrigin(const WebCore::SecurityOriginData& origin)
{
    MESSAGE_CHECK(!origin.isNull());
    RefPtr networkConnectionToWebProcess = m_networkConnectionToWebProcess.get();
    if (!networkConnectionToWebProcess)
        return;

    Ref networkProcess = networkConnectionToWebProcess->networkProcess();
    MESSAGE_CHECK(networkProcess->allowsFirstPartyForCookies(networkConnectionToWebProcess->webProcessIdentifier(), WebCore::RegistrableDomain::uncheckedCreateFromHost(origin.host())) != NetworkProcess::AllowCookieAccess::Terminate);
}

} // namespace WebKit

#undef MESSAGE_CHECK
#undef SWSERVERCONNECTION_RELEASE_LOG
#undef SWSERVERCONNECTION_RELEASE_LOG_ERROR
