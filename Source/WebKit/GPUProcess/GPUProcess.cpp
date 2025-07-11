/*
 * Copyright (C) 2019-2025 Apple Inc. All rights reserved.
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
#include "GPUProcess.h"

#if ENABLE(GPU_PROCESS)

#include "ArgumentCoders.h"
#include "Attachment.h"
#include "AuxiliaryProcessMessages.h"
#include "GPUConnectionToWebProcess.h"
#include "GPUProcessConnectionParameters.h"
#include "GPUProcessCreationParameters.h"
#include "GPUProcessPreferences.h"
#include "GPUProcessProxyMessages.h"
#include "GPUProcessSessionParameters.h"
#include "LogInitialization.h"
#include "Logging.h"
#include "RemoteMediaPlayerManagerProxy.h"
#include "SandboxExtension.h"
#include "WebPageProxyMessages.h"
#include "WebProcessPoolMessages.h"
#include <WebCore/CommonAtomStrings.h>
#include <WebCore/DeprecatedGlobalSettings.h>
#include <WebCore/LogInitialization.h>
#include <WebCore/MediaPlayer.h>
#include <WebCore/MemoryRelease.h>
#include <WebCore/NowPlayingManager.h>
#include <wtf/CallbackAggregator.h>
#include <wtf/Language.h>
#include <wtf/LogInitialization.h>
#include <wtf/MemoryPressureHandler.h>
#include <wtf/OptionSet.h>
#include <wtf/ProcessPrivilege.h>
#include <wtf/RunLoop.h>
#include <wtf/RuntimeApplicationChecks.h>
#include <wtf/Scope.h>
#include <wtf/UniqueRef.h>
#include <wtf/text/AtomString.h>

#if USE(AUDIO_SESSION)
#include "RemoteAudioSessionProxyManager.h"
#endif

#if ENABLE(MEDIA_STREAM)
#include <WebCore/MockRealtimeMediaSourceCenter.h>
#endif

#if PLATFORM(COCOA)
#include "ArgumentCodersCocoa.h"
#include <WebCore/CoreAudioSharedUnit.h>
#include <WebCore/UTIUtilities.h>
#include <WebCore/VP9UtilitiesCocoa.h>
#endif

#if HAVE(SCREEN_CAPTURE_KIT)
#include <WebCore/ScreenCaptureKitCaptureSource.h>
#endif

namespace WebKit {

// We wouldn't want the GPUProcess to repeatedly exit then relaunch when under memory pressure. In particular, we need to make sure the
// WebProcess has a change to schedule work after the GPUProcess get launched. For this reason, we make sure that the GPUProcess never
// idle-exits less than 5 seconds after getting launched. This amount of time should be sufficient for the WebProcess to schedule work
// work in the GPUProcess.
constexpr Seconds minimumLifetimeBeforeIdleExit { 5_s };

GPUProcess::GPUProcess()
    : m_idleExitTimer(*this, &GPUProcess::tryExitIfUnused)
{
    RELEASE_LOG(Process, "%p - GPUProcess::GPUProcess:", this);
#if ASSERT_ENABLED && PLATFORM(COCOA) && ENABLE(MEDIA_STREAM)
    CoreAudioSharedUnit::singleton().allowStarting();
#endif
}

GPUProcess::~GPUProcess()
{
}

GPUProcess& GPUProcess::singleton()
{
    static NeverDestroyed<GPUProcess> gpuProcess;
    return gpuProcess.get();
}

void GPUProcess::createGPUConnectionToWebProcess(WebCore::ProcessIdentifier identifier, PAL::SessionID sessionID, IPC::Connection::Handle&& connectionHandle, GPUProcessConnectionParameters&& parameters, CompletionHandler<void()>&& completionHandler)
{
    RELEASE_LOG(Process, "%p - GPUProcess::createGPUConnectionToWebProcess: processIdentifier=%" PRIu64, this, identifier.toUInt64());

    auto reply = makeScopeExit(WTFMove(completionHandler));
    // If sender exited before we received the handle, the handle may not be valid.
    if (!connectionHandle)
        return;

    auto newConnection = GPUConnectionToWebProcess::create(*this, identifier, sessionID, WTFMove(connectionHandle), WTFMove(parameters));

#if ENABLE(MEDIA_STREAM)
    // FIXME: We should refactor code to go from WebProcess -> GPUProcess -> UIProcess when getUserMedia is called instead of going from WebProcess -> UIProcess directly.
    auto access = m_mediaCaptureAccessMap.take(identifier);
    newConnection->updateCaptureAccess(access.allowAudioCapture, access.allowVideoCapture, access.allowDisplayCapture);
    newConnection->setOrientationForMediaCapture(m_orientation);
#endif

#if ENABLE(IPC_TESTING_API)
    if (parameters.ignoreInvalidMessageForTesting)
        newConnection->connection().setIgnoreInvalidMessageForTesting();
#endif

    ASSERT(!m_webProcessConnections.contains(identifier));
    m_webProcessConnections.add(identifier, WTFMove(newConnection));
}

void GPUProcess::sharedPreferencesForWebProcessDidChange(WebCore::ProcessIdentifier identifier, SharedPreferencesForWebProcess&& sharedPreferencesForWebProcess, CompletionHandler<void()>&& completionHandler)
{
    if (RefPtr connection = m_webProcessConnections.get(identifier))
        connection->updateSharedPreferencesForWebProcess(WTFMove(sharedPreferencesForWebProcess));
    completionHandler();
}

void GPUProcess::removeGPUConnectionToWebProcess(GPUConnectionToWebProcess& connection)
{
    RELEASE_LOG(Process, "%p - GPUProcess::removeGPUConnectionToWebProcess: processIdentifier=%" PRIu64, this, connection.webProcessIdentifier().toUInt64());
    ASSERT(m_webProcessConnections.contains(connection.webProcessIdentifier()));
    m_webProcessConnections.remove(connection.webProcessIdentifier());
    tryExitIfUnusedAndUnderMemoryPressure();
}

void GPUProcess::connectionToWebProcessClosed(IPC::Connection& connection)
{
}

bool GPUProcess::shouldTerminate()
{
    return m_webProcessConnections.isEmpty();
}

bool GPUProcess::canExitUnderMemoryPressure() const
{
    ASSERT(isMainRunLoop());
    for (auto& webProcessConnection : m_webProcessConnections.values()) {
        if (!webProcessConnection->allowsExitUnderMemoryPressure())
            return false;
    }
    return true;
}

void GPUProcess::tryExitIfUnusedAndUnderMemoryPressure()
{
    ASSERT(isMainRunLoop());
    if (!MemoryPressureHandler::singleton().isUnderMemoryPressure())
        return;

    tryExitIfUnused();
}

void GPUProcess::tryExitIfUnused()
{
    ASSERT(isMainRunLoop());
    if (!canExitUnderMemoryPressure()) {
        m_idleExitTimer.stop();
        return;
    }

    // To avoid exiting the GPUProcess too aggressively while under memory pressure and make sure the WebProcess gets a
    // change to schedule work, we don't exit if we've been running for less than |minimumLifetimeBeforeIdleExit|.
    // In case of simulated memory pressure, we ignore this rule to avoid flakiness in our benchmarks and tests.
    auto lifetime = MonotonicTime::now() - m_creationTime;
    if (lifetime < minimumLifetimeBeforeIdleExit && !MemoryPressureHandler::singleton().isSimulatingMemoryPressure()) {
        RELEASE_LOG(Process, "GPUProcess::tryExitIfUnused: GPUProcess is idle and under memory pressure but it is not exiting because it has just launched");
        // Check again after the process have lived long enough (minimumLifetimeBeforeIdleExit) to see if the GPUProcess
        // can idle-exit then.
        if (!m_idleExitTimer.isActive())
            m_idleExitTimer.startOneShot(minimumLifetimeBeforeIdleExit - lifetime);
        return;
    }
    m_idleExitTimer.stop();

    RELEASE_LOG(Process, "GPUProcess::tryExitIfUnused: GPUProcess is exiting because we are under memory pressure and the process is no longer useful.");
    protectedParentProcessConnection()->send(Messages::GPUProcessProxy::ProcessIsReadyToExit(), 0);
}

void GPUProcess::lowMemoryHandler(Critical critical, Synchronous synchronous)
{
    RELEASE_LOG(Process, "GPUProcess::lowMemoryHandler: critical=%d, synchronous=%d", critical == Critical::Yes, synchronous == Synchronous::Yes);
    tryExitIfUnused();

    for (auto& connection : m_webProcessConnections.values())
        connection->lowMemoryHandler(critical, synchronous);

    WebCore::releaseGraphicsMemory(critical, synchronous);
}

void GPUProcess::initializeGPUProcess(GPUProcessCreationParameters&& parameters, CompletionHandler<void()>&& completionHandler)
{
    CompletionHandlerCallingScope callCompletionHandler(WTFMove(completionHandler));

    applyProcessCreationParameters(WTFMove(parameters.auxiliaryProcessParameters));
    RELEASE_LOG(Process, "%p - GPUProcess::initializeGPUProcess:", this);
    WTF::Thread::setCurrentThreadIsUserInitiated();
    WebCore::initializeCommonAtomStrings();

    Ref memoryPressureHandler = MemoryPressureHandler::singleton();
    memoryPressureHandler->setLowMemoryHandler([weakThis = WeakPtr { *this }] (Critical critical, Synchronous synchronous) {
        if (RefPtr process = weakThis.get())
            process->lowMemoryHandler(critical, synchronous);
    });
    memoryPressureHandler->install();

#if PLATFORM(IOS_FAMILY) || ENABLE(ROUTING_ARBITRATION)
    DeprecatedGlobalSettings::setShouldManageAudioSessionCategory(true);
#endif

#if ENABLE(MEDIA_STREAM)
    setMockCaptureDevicesEnabled(parameters.useMockCaptureDevices);
#if PLATFORM(MAC)
    SandboxExtension::consumePermanently(parameters.microphoneSandboxExtensionHandle);
#endif
#if PLATFORM(IOS_FAMILY)
    CoreAudioSharedUnit::singleton().setStatusBarWasTappedCallback([weakProcess = WeakPtr { *this }] (auto completionHandler) {
        if (RefPtr process = weakProcess.get())
            process->parentProcessConnection()->sendWithAsyncReply(Messages::GPUProcessProxy::StatusBarWasTapped(), [] { }, 0);
        completionHandler();
    });
#endif
#endif // ENABLE(MEDIA_STREAM)

#if USE(SANDBOX_EXTENSIONS_FOR_CACHE_AND_TEMP_DIRECTORY_ACCESS)
    SandboxExtension::consumePermanently(parameters.containerCachesDirectoryExtensionHandle);
    grantAccessToContainerTempDirectory(parameters.containerTemporaryDirectoryExtensionHandle);
#endif

    populateMobileGestaltCache(WTFMove(parameters.mobileGestaltExtensionHandle));

#if PLATFORM(COCOA) && ENABLE(REMOTE_INSPECTOR)
    SandboxExtension::consumePermanently(parameters.gpuToolsExtensionHandles);
#endif

#if PLATFORM(COCOA)
    WebCore::setImageSourceAllowableTypes({ });
#endif

    m_applicationVisibleName = WTFMove(parameters.applicationVisibleName);

    // Match the QoS of the UIProcess since the GPU process is doing rendering on its behalf.
    WTF::Thread::setCurrentThreadIsUserInteractive(0);

    if (!parameters.overrideLanguages.isEmpty())
        overrideUserPreferredLanguages(parameters.overrideLanguages);

#if USE(OS_STATE)
    registerWithStateDumper("GPUProcess state"_s);
#endif

    platformInitializeGPUProcess(parameters);
}

void GPUProcess::updateGPUProcessPreferences(GPUProcessPreferences&& preferences)
{
#if USE(MODERN_AVCONTENTKEYSESSION)
    if (updatePreference(m_preferences.shouldUseModernAVContentKeySession, preferences.shouldUseModernAVContentKeySession))
        MediaSessionManagerCocoa::setShouldUseModernAVContentKeySession(*m_preferences.shouldUseModernAVContentKeySession);
#endif

#if ENABLE(VP9)
    if (updatePreference(m_preferences.vp9DecoderEnabled, preferences.vp9DecoderEnabled)) {
        VP9TestingOverrides::singleton().setShouldEnableVP9Decoder(*m_preferences.vp9DecoderEnabled);
#if PLATFORM(COCOA)
        if (!m_haveEnabledVP9Decoder && *m_preferences.vp9DecoderEnabled) {
            m_haveEnabledVP9Decoder = true;
            WebCore::registerSupplementalVP9Decoder();
        }
#endif
    }
#if PLATFORM(COCOA)
    if (preferences.swVPDecodersAlwaysEnabled != std::exchange(m_preferences.swVPDecodersAlwaysEnabled, preferences.swVPDecodersAlwaysEnabled))
        VP9TestingOverrides::singleton().setSWVPDecodersAlwaysEnabled(m_preferences.swVPDecodersAlwaysEnabled);

    if (!m_haveEnabledSWVP9Decoder && WebCore::shouldEnableSWVP9Decoder()) {
        WebCore::registerWebKitVP9Decoder();
        m_haveEnabledSWVP9Decoder = true;
    }
#endif
#endif
}

bool GPUProcess::updatePreference(std::optional<bool>& oldPreference, std::optional<bool>& newPreference)
{
    if (newPreference.has_value() && oldPreference != newPreference) {
        oldPreference = WTFMove(newPreference);
        return true;
    }
    
    if (!oldPreference.has_value()) {
        oldPreference = false;
        return true;
    }
    
    return false;
}

void GPUProcess::userPreferredLanguagesChanged(Vector<String>&& languages)
{
    overrideUserPreferredLanguages(languages);
}

void GPUProcess::prepareToSuspend(bool isSuspensionImminent, MonotonicTime, CompletionHandler<void()>&& completionHandler)
{
    RELEASE_LOG(ProcessSuspension, "%p - GPUProcess::prepareToSuspend(), isSuspensionImminent: %d", this, isSuspensionImminent);

    lowMemoryHandler(Critical::Yes, Synchronous::Yes);
    completionHandler();
}

void GPUProcess::processDidResume()
{
    RELEASE_LOG(ProcessSuspension, "%p - GPUProcess::processDidResume()", this);
    resume();
}

void GPUProcess::resume()
{
}

GPUConnectionToWebProcess* GPUProcess::webProcessConnection(WebCore::ProcessIdentifier identifier) const
{
    return m_webProcessConnections.get(identifier);
}

void GPUProcess::updateSandboxAccess(const Vector<SandboxExtension::Handle>& extensions)
{
    RELEASE_LOG(WebRTC, "GPUProcess::updateSandboxAccess: Adding %ld extensions", extensions.size());
    for (auto& extension : extensions)
        SandboxExtension::consumePermanently(extension);
}

#if PLATFORM(COCOA)
void GPUProcess::didDrawRemoteToPDF(PageIdentifier pageID, RefPtr<SharedBuffer>&& data, SnapshotIdentifier snapshotIdentifier)
{
    protectedParentProcessConnection()->send(Messages::GPUProcessProxy::DidDrawRemoteToPDF(pageID, WTFMove(data), snapshotIdentifier), 0);
}
#endif

#if ENABLE(MEDIA_STREAM)
void GPUProcess::setMockCaptureDevicesEnabled(bool isEnabled)
{
    WebCore::MockRealtimeMediaSourceCenter::setMockRealtimeMediaSourceCenterEnabled(isEnabled);
}

void GPUProcess::setOrientationForMediaCapture(WebCore::IntDegrees orientation)
{
    m_orientation = orientation;
    for (auto& connection : m_webProcessConnections.values())
        connection->setOrientationForMediaCapture(orientation);
}

void GPUProcess::enableMicrophoneMuteStatusAPI()
{
#if PLATFORM(COCOA)
    CoreAudioSharedUnit::singleton().setMuteStatusChangedCallback([weakProcess = WeakPtr { *this }] (bool isMuting) {
        if (RefPtr process = weakProcess.get())
            process->protectedParentProcessConnection()->send(Messages::GPUProcessProxy::MicrophoneMuteStatusChanged(isMuting), 0);
    });
#endif
}

void GPUProcess::rotationAngleForCaptureDeviceChanged(const String& persistentId, WebCore::VideoFrameRotation rotation)
{
    for (auto& connection : m_webProcessConnections.values())
        connection->rotationAngleForCaptureDeviceChanged(persistentId, rotation);
}

void GPUProcess::updateCaptureAccess(bool allowAudioCapture, bool allowVideoCapture, bool allowDisplayCapture, WebCore::ProcessIdentifier processID, CompletionHandler<void()>&& completionHandler)
{
    RELEASE_LOG(WebRTC, "GPUProcess::updateCaptureAccess: Entering (audio=%d, video=%d, display=%d)", allowAudioCapture, allowVideoCapture, allowDisplayCapture);

#if ENABLE(MEDIA_STREAM) && PLATFORM(COCOA)
    ensureAVCaptureServerConnection();
#endif

    if (RefPtr connection = webProcessConnection(processID)) {
        connection->updateCaptureAccess(allowAudioCapture, allowVideoCapture, allowDisplayCapture);
        return completionHandler();
    }

    auto& access = m_mediaCaptureAccessMap.add(processID, MediaCaptureAccess { allowAudioCapture, allowVideoCapture, allowDisplayCapture }).iterator->value;
    access.allowAudioCapture |= allowAudioCapture;
    access.allowVideoCapture |= allowVideoCapture;
    access.allowDisplayCapture |= allowDisplayCapture;

    completionHandler();
}

void GPUProcess::updateCaptureOrigin(const WebCore::SecurityOriginData& originData, WebCore::ProcessIdentifier processID)
{
    if (RefPtr connection = webProcessConnection(processID))
        connection->updateCaptureOrigin(originData);
}

void GPUProcess::addMockMediaDevice(const WebCore::MockMediaDevice& device)
{
    WebCore::MockRealtimeMediaSourceCenter::addDevice(device);
}

void GPUProcess::clearMockMediaDevices()
{
    WebCore::MockRealtimeMediaSourceCenter::setDevices({ });
}

void GPUProcess::removeMockMediaDevice(const String& persistentId)
{
    WebCore::MockRealtimeMediaSourceCenter::removeDevice(persistentId);
}

void GPUProcess::setMockMediaDeviceIsEphemeral(const String& persistentId, bool isEphemeral)
{
    WebCore::MockRealtimeMediaSourceCenter::setDeviceIsEphemeral(persistentId, isEphemeral);
}

void GPUProcess::resetMockMediaDevices()
{
    WebCore::MockRealtimeMediaSourceCenter::resetDevices();
}

void GPUProcess::setMockCaptureDevicesInterrupted(bool isCameraInterrupted, bool isMicrophoneInterrupted)
{
    WebCore::MockRealtimeMediaSourceCenter::setMockCaptureDevicesInterrupted(isCameraInterrupted, isMicrophoneInterrupted);
}

void GPUProcess::triggerMockCaptureConfigurationChange(bool forCamera, bool forMicrophone, bool forDisplay)
{
    WebCore::MockRealtimeMediaSourceCenter::singleton().triggerMockCaptureConfigurationChange(forCamera, forMicrophone, forDisplay);
}

void GPUProcess::setShouldListenToVoiceActivity(bool shouldListen)
{
#if PLATFORM(COCOA)
    if (!shouldListen) {
        RealtimeMediaSourceCenter::singleton().audioCaptureFactory().disableMutedSpeechActivityEventListener();
        return;
    }

    RealtimeMediaSourceCenter::singleton().audioCaptureFactory().enableMutedSpeechActivityEventListener([] {
        GPUProcess::singleton().protectedParentProcessConnection()->send(Messages::GPUProcessProxy::VoiceActivityDetected { }, 0);
    });
#endif
}
#endif // ENABLE(MEDIA_STREAM)

#if HAVE(SCREEN_CAPTURE_KIT)
void GPUProcess::promptForGetDisplayMedia(WebCore::DisplayCapturePromptType type, CompletionHandler<void(std::optional<WebCore::CaptureDevice>)>&& completionHandler)
{
    WebCore::ScreenCaptureKitSharingSessionManager::singleton().promptForGetDisplayMedia(type, WTFMove(completionHandler));
}

void GPUProcess::cancelGetDisplayMediaPrompt()
{
    WebCore::ScreenCaptureKitSharingSessionManager::singleton().cancelGetDisplayMediaPrompt();
}

#endif // HAVE(SCREEN_CAPTURE_KIT)

void GPUProcess::addSession(PAL::SessionID sessionID, GPUProcessSessionParameters&& parameters)
{
    ASSERT(!m_sessions.contains(sessionID));
    SandboxExtension::consumePermanently(parameters.mediaCacheDirectorySandboxExtensionHandle);
#if ENABLE(LEGACY_ENCRYPTED_MEDIA)
    SandboxExtension::consumePermanently(parameters.mediaKeysStorageDirectorySandboxExtensionHandle);
#endif

    m_sessions.add(sessionID, GPUSession {
        WTFMove(parameters.mediaCacheDirectory)
#if ENABLE(LEGACY_ENCRYPTED_MEDIA) || ENABLE(ENCRYPTED_MEDIA)
        , WTFMove(parameters.mediaKeysStorageDirectory)
#endif
    });
}

void GPUProcess::removeSession(PAL::SessionID sessionID)
{
    auto findResult = m_sessions.find(sessionID);
    if (findResult == m_sessions.end()) {
        ASSERT_NOT_REACHED("Invalid SessionID");
        return;
    }
    m_sessions.remove(findResult);
}

const String& GPUProcess::mediaCacheDirectory(PAL::SessionID sessionID) const
{
    auto findResult = m_sessions.find(sessionID);
    if (findResult == m_sessions.end()) {
        ASSERT_NOT_REACHED("Invalid SessionID");
        return nullString();
    }
    return findResult->value.mediaCacheDirectory;
}

#if ENABLE(LEGACY_ENCRYPTED_MEDIA) || ENABLE(ENCRYPTED_MEDIA)
const String& GPUProcess::mediaKeysStorageDirectory(PAL::SessionID sessionID) const
{
    auto findResult = m_sessions.find(sessionID);
    if (findResult == m_sessions.end()) {
        ASSERT_NOT_REACHED("Invalid SessionID");
        return nullString();
    }
    return findResult->value.mediaKeysStorageDirectory;
}
#endif

WebCore::NowPlayingManager& GPUProcess::nowPlayingManager()
{
    if (!m_nowPlayingManager)
        m_nowPlayingManager = makeUnique<WebCore::NowPlayingManager>();
    return *m_nowPlayingManager;
}

#if ENABLE(GPU_PROCESS) && USE(AUDIO_SESSION)
RemoteAudioSessionProxyManager& GPUProcess::audioSessionManager() const
{
    if (!m_audioSessionManager)
        m_audioSessionManager = RemoteAudioSessionProxyManager::create(const_cast<GPUProcess&>(*this));
    return *m_audioSessionManager;
}

Ref<RemoteAudioSessionProxyManager> GPUProcess::protectedAudioSessionManager() const
{
    return audioSessionManager();
}
#endif

#if ENABLE(MEDIA_STREAM) && PLATFORM(COCOA)
WorkQueue& GPUProcess::videoMediaStreamTrackRendererQueue()
{
    if (!m_videoMediaStreamTrackRendererQueue)
        m_videoMediaStreamTrackRendererQueue = WorkQueue::create("RemoteVideoMediaStreamTrackRenderer"_s, WorkQueue::QOS::UserInitiated);
    return *m_videoMediaStreamTrackRendererQueue;
}
#endif

#if USE(LIBWEBRTC) && PLATFORM(COCOA)
WorkQueue& GPUProcess::libWebRTCCodecsQueue()
{
    if (!m_libWebRTCCodecsQueue)
        m_libWebRTCCodecsQueue = WorkQueue::create("LibWebRTCCodecsQueue"_s, WorkQueue::QOS::UserInitiated);
    return *m_libWebRTCCodecsQueue;
}
#endif

void GPUProcess::webProcessConnectionCountForTesting(CompletionHandler<void(uint64_t)>&& completionHandler)
{
    completionHandler(GPUConnectionToWebProcess::objectCountForTesting());
}

#if PLATFORM(COCOA) && ENABLE(MEDIA_STREAM)
void GPUProcess::processIsStartingToCaptureAudio(GPUConnectionToWebProcess& process)
{
    for (auto& connection : m_webProcessConnections.values())
        connection->processIsStartingToCaptureAudio(process);
}
#endif

#if ENABLE(VIDEO)
void GPUProcess::requestBitmapImageForCurrentTime(WebCore::ProcessIdentifier processIdentifier, WebCore::MediaPlayerIdentifier playerIdentifier, CompletionHandler<void(std::optional<WebCore::ShareableBitmap::Handle>&&)>&& completion)
{
    RefPtr connection = m_webProcessConnections.get(processIdentifier);
    if (!connection) {
        completion(std::nullopt);
        return;
    }

    completion(connection->protectedRemoteMediaPlayerManagerProxy()->bitmapImageForCurrentTime(playerIdentifier));
}
#endif

#if ENABLE(WEBXR)
std::optional<WebCore::ProcessIdentity> GPUProcess::immersiveModeProcessIdentity() const
{
    return m_processIdentity;
}

void GPUProcess::webXRPromptAccepted(std::optional<WebCore::ProcessIdentity> processIdentity, CompletionHandler<void(bool)>&& completionHandler)
{
    m_processIdentity = processIdentity;
    completionHandler(true);
}
#endif

#if HAVE(AUDIT_TOKEN)
void GPUProcess::setPresentingApplicationAuditToken(WebCore::ProcessIdentifier processIdentifier, WebCore::PageIdentifier pageIdentifier, std::optional<WebKit::CoreIPCAuditToken>&& auditToken)
{
    if (RefPtr connection = m_webProcessConnections.get(processIdentifier))
        connection->setPresentingApplicationAuditToken(pageIdentifier, WTFMove(auditToken));
}
#endif

} // namespace WebKit

#endif // ENABLE(GPU_PROCESS)
