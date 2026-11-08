/*
 * Copyright (C) 2010-2023 Apple Inc. All rights reserved.
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
#include "Decoder.h"

#include "ArgumentCoders.h"
#include "Logging.h"
#include "MessageFlags.h"
#include <stdio.h>
#include <wtf/MallocSpan.h>
#include <wtf/ObjectIdentifier.h>
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

#if USE(SWIFT_KERNELS)
#include "IPCStreamHeaderSwiftBoundary.h"
#endif

#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif

namespace IPC {

static MallocSpan<uint8_t> copyBuffer(std::span<const uint8_t> buffer)
{
    auto bufferCopy = MallocSpan<uint8_t>::tryMalloc(buffer.size());
    if (!bufferCopy) {
        RELEASE_LOG_FAULT(IPC, "Decoder::copyBuffer: tryMalloc(%zu) failed", buffer.size());
        return { };
    }
    memcpySpan(bufferCopy.mutableSpan(), buffer);
    return bufferCopy;
}

WTF_MAKE_TZONE_ALLOCATED_IMPL(Decoder);

std::unique_ptr<Decoder> Decoder::create(std::span<const uint8_t> buffer, Vector<Attachment>&& attachments)
{
    auto bufferCopy = copyBuffer(buffer);
    auto bufferCopySpan = bufferCopy.span();
    return Decoder::create(bufferCopySpan, [bufferCopy = WTF::move(bufferCopy)](auto) { }, WTF::move(attachments)); // NOLINT
}

std::unique_ptr<Decoder> Decoder::create(std::span<const uint8_t> buffer, BufferDeallocator&& bufferDeallocator, Vector<Attachment>&& attachments)
{
    ASSERT(bufferDeallocator);
    ASSERT(!!buffer.data());
    if (!buffer.data()) [[unlikely]] {
        RELEASE_LOG_FAULT(IPC, "Decoder::create() called with a null buffer (buffer size: %zu)", buffer.size_bytes());
        return nullptr;
    }
    auto decoder = std::unique_ptr<Decoder>(new Decoder(buffer, WTF::move(bufferDeallocator), WTF::move(attachments)));
    if (!decoder->isValid())
        return nullptr;
    return decoder;
}

Decoder::Decoder(std::span<const uint8_t> buffer, BufferDeallocator&& bufferDeallocator, Vector<Attachment>&& attachments)
    : m_buffer { buffer }
    , m_bufferPosition { m_buffer.begin() }
    , m_bufferDeallocator { WTF::move(bufferDeallocator) }
    , m_attachments { WTF::move(attachments) }
{
    if (reinterpret_cast<uintptr_t>(m_buffer.data()) % alignof(uint64_t)) [[unlikely]] {
        markInvalid();
        return;
    }

    auto messageFlags = decode<OptionSet<MessageFlags>>();
    if (!messageFlags) [[unlikely]]
        return;
    m_messageFlags = WTF::move(*messageFlags);

    auto messageName = decode<MessageName>();
    if (!messageName) [[unlikely]]
        return;
    m_messageName = WTF::move(*messageName);

    auto destinationID = decode<uint64_t>();
    if (!destinationID) [[unlikely]]
        return;
    // 0 is a valid destinationID but we can at least reject -1 which is the HashTable deleted value.
    if (*destinationID && !WTF::ObjectIdentifierGenericBase::isValidIdentifier(*destinationID)) {
        markInvalid();
        return;
    }
    m_destinationID = WTF::move(*destinationID);
    if (messageIsAsyncWithReply(m_messageName)) {
        auto asyncReplyID = decode<AsyncReplyID>();
        if (!asyncReplyID) [[unlikely]]
            return;
        m_replyID = asyncReplyID->toUInt64();
    } else if (messageIsSync(m_messageName)) {
        auto syncRequestID = decode<SyncRequestID>();
        if (!syncRequestID) [[unlikely]]
            return;
        m_replyID = syncRequestID->toUInt64();
    }
    m_argumentsOffset = currentBufferOffset();
}

#if !USE(SWIFT_KERNELS) || ENABLE(SWIFT_KERNELS_VALIDATION)
ALWAYS_INLINE void Decoder::decodeStreamHeaderCxx(uint64_t destinationID)
{
    // 0 is a valid destinationID but we can at least reject -1 which is the HashTable deleted value.
    if (destinationID && !WTF::ObjectIdentifierGenericBase::isValidIdentifier(destinationID)) {
        markInvalid();
        return;
    }

    auto messageName = decode<MessageName>();
    if (!messageName) [[unlikely]]
        return;
    m_messageName = WTF::move(*messageName);
    if (messageIsAsyncWithReply(m_messageName)) {
        auto asyncReplyID = decode<AsyncReplyID>();
        if (!asyncReplyID) [[unlikely]]
            return;
        m_replyID = asyncReplyID->toUInt64();
    } else if (messageIsSync(m_messageName)) {
        auto syncRequestID = decode<SyncRequestID>();
        if (!syncRequestID) [[unlikely]]
            return;
        m_replyID = syncRequestID->toUInt64();
    }
    m_argumentsOffset = currentBufferOffset();
}
#endif

#if USE(SWIFT_KERNELS)
// The header is decoded in Swift (Source/WebKit/swift/IPCStreamHeader); decodeStreamHeaderCxx() is the original.
// header is m_buffer, or a private snapshot of its start (decodeStreamHeaderAndValidate()).
ALWAYS_INLINE void Decoder::decodeStreamHeader(std::span<const uint8_t> header, uint64_t destinationID)
{
    // messageIsAsyncWithReply() and messageIsSync() hold for exactly the names >= firstMessageNameWithReplyID, and
    // AsyncReplyID and SyncRequestID have the same wire format and validator.
    static constexpr auto firstMessageNameWithReplyID = std::min(MessageName::FirstAsyncWithReply, MessageName::FirstSynchronous);
    uint64_t result = WebKitSwift_IPCStreamHeader_decode(header.data(), header.size(), destinationID,
        std::to_underlying(MessageName::Last), std::to_underlying(firstMessageNameWithReplyID), &m_replyID);
    auto status = static_cast<uint8_t>(result >> 16);
    if (status == WebKitSwift_IPCStreamHeader_Success) [[likely]] {
        m_messageName = static_cast<MessageName>(static_cast<uint16_t>(result));
        m_argumentsOffset = static_cast<size_t>(result >> 24);
        m_bufferPosition = m_buffer.begin() + m_argumentsOffset;
    } else {
        // The C++ stores the message name before decoding the reply ID, and reports it for an invalid message.
        if (status == WebKitSwift_IPCStreamHeader_TruncatedReplyID || status == WebKitSwift_IPCStreamHeader_InvalidReplyID)
            m_messageName = static_cast<MessageName>(static_cast<uint16_t>(result));
#if ENABLE(IPC_TESTING_API)
        if (status == WebKitSwift_IPCStreamHeader_InvalidReplyID)
            setErrorString("Validation failed: WTF::ObjectIdentifierGenericBase::isValidIdentifier(*toUInt64)"_s);
#endif
        markInvalid();
    }
}
#endif

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// The stream is shared memory that the sending process can rewrite at any time, so a second read could see other bytes
// than the Swift read did. Both decoders read one private snapshot of the header instead. It keeps the stream's address
// modulo 8 (values are aligned in memory, not in the span) and at most 24 bytes (a header ends by offset 17, so every
// larger size decodes the same), so decoding it is decoding the stream at the time of the copy.
NEVER_INLINE void Decoder::decodeStreamHeaderAndValidate(uint64_t destinationID)
{
    auto stream = m_buffer;
    alignas(8) std::array<uint8_t, 8 + 24> snapshotStorage;
    auto snapshot = std::span { snapshotStorage }.subspan(reinterpret_cast<uintptr_t>(stream.data()) % 8, std::min<size_t>(stream.size(), 24));
    memcpySpan(snapshot, stream.first(snapshot.size()));
    decodeStreamHeader(snapshot, destinationID);

    Decoder cxx { snapshot, destinationID, StreamHeaderCxxTag { } };
    RELEASE_ASSERT(isValid() == cxx.isValid());
    RELEASE_ASSERT(!isValid() || (m_buffer.data() == stream.data() && m_buffer.size() == stream.size()));
    RELEASE_ASSERT(currentBufferOffset() == cxx.currentBufferOffset());
    RELEASE_ASSERT(m_messageName == cxx.m_messageName);
    RELEASE_ASSERT(m_replyID == cxx.m_replyID);
    RELEASE_ASSERT(m_argumentsOffset == cxx.m_argumentsOffset);
#if ENABLE(IPC_TESTING_API)
    RELEASE_ASSERT(m_errorString == cxx.m_errorString);
#endif
}

Decoder::Decoder(std::span<const uint8_t> stream, uint64_t destinationID, StreamHeaderCxxTag)
    : m_buffer { stream }
    , m_bufferPosition { m_buffer.begin() }
    , m_bufferDeallocator { nullptr }
    , m_destinationID { destinationID }
{
    decodeStreamHeaderCxx(destinationID);
}
#endif

Decoder::Decoder(std::span<const uint8_t> stream, uint64_t destinationID)
    : m_buffer { stream }
    , m_bufferPosition { m_buffer.begin() }
    , m_bufferDeallocator { nullptr }
    , m_destinationID { destinationID }
{
#if USE(SWIFT_KERNELS)
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (WebKit::swiftKernelsValidationEnabled()) [[unlikely]] {
        decodeStreamHeaderAndValidate(destinationID);
        return;
    }
#endif
    decodeStreamHeader(m_buffer, destinationID);
#else
    decodeStreamHeaderCxx(destinationID);
#endif
}

Decoder::~Decoder()
{
    if (isValid())
        markInvalid();
}

ShouldDispatchWhenWaitingForSyncReply Decoder::shouldDispatchMessageWhenWaitingForSyncReply() const
{
    if (m_messageFlags.contains(MessageFlags::DispatchMessageWhenWaitingForSyncReply))
        return ShouldDispatchWhenWaitingForSyncReply::Yes;
    if (m_messageFlags.contains(MessageFlags::DispatchMessageWhenWaitingForUnboundedSyncReply))
        return ShouldDispatchWhenWaitingForSyncReply::YesDuringUnboundedIPC;
    return ShouldDispatchWhenWaitingForSyncReply::No;
}

bool Decoder::shouldUseFullySynchronousModeForTesting() const
{
    return m_messageFlags.contains(MessageFlags::UseFullySynchronousModeForTesting);
}

bool Decoder::shouldMaintainOrderingWithAsyncMessages() const
{
    return m_messageFlags.contains(MessageFlags::MaintainOrderingWithAsyncMessages);
}

#if PLATFORM(MAC)
void Decoder::setImportanceAssertion(ImportanceAssertion&& assertion)
{
    m_importanceAssertion = WTF::move(assertion);
}
#endif

std::unique_ptr<Decoder> Decoder::unwrapForTesting(Decoder& decoder)
{
    ASSERT(decoder.isSyncMessage());

    auto attachments = std::exchange(decoder.m_attachments, { });

    auto wrappedMessage = decoder.decode<std::span<const uint8_t>>();
    if (!wrappedMessage)
        return nullptr;

    auto wrappedDecoder = Decoder::create(*wrappedMessage, WTF::move(attachments));
    if (!wrappedDecoder)
        return nullptr;
    wrappedDecoder->setIsAllowedWhenWaitingForSyncReplyOverride(true);
    return wrappedDecoder;
}

std::optional<Attachment> Decoder::takeLastAttachment()
{
    if (m_attachments.isEmpty()) {
        markInvalid();
        return std::nullopt;
    }
    return m_attachments.takeLast();
}

void Decoder::rewindToArguments()
{
    ASSERT(isValid());
    m_bufferPosition = m_buffer.begin() + m_argumentsOffset;
    m_indicesOfObjectsFailingDecoding.clear();
}

} // namespace IPC
