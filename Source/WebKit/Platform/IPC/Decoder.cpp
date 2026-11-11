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
#include "IPCStreamMessageSwiftBoundary.h"
#include "Logging.h"
#include "MessageFlags.h"
#include <stdio.h>
#include <wtf/MallocSpan.h>
#include <wtf/ObjectIdentifier.h>
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

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
    , m_swiftStreamArguments { }
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

// Swift stores the arguments straight into m_swiftStreamArguments (all the fields, so the stream constructor doesn't
// initialize them), which C++ reads only as a SwiftStreamArguments: a copy would read them right after Swift's stores.
static_assert(sizeof(Decoder::SwiftStreamArguments) == sizeof(WebKitSwift_IPCStreamMessage_Arguments));
static_assert(alignof(Decoder::SwiftStreamArguments) == alignof(WebKitSwift_IPCStreamMessage_Arguments));
static_assert(std::is_standard_layout_v<Decoder::SwiftStreamArguments> && std::is_trivially_copyable_v<Decoder::SwiftStreamArguments>);
static_assert(offsetof(Decoder::SwiftStreamArguments, identifier) == offsetof(WebKitSwift_IPCStreamMessage_Arguments, identifier));
static_assert(offsetof(Decoder::SwiftStreamArguments, valuesOffset) == offsetof(WebKitSwift_IPCStreamMessage_Arguments, valuesOffset));
static_assert(offsetof(Decoder::SwiftStreamArguments, endOffset) == offsetof(WebKitSwift_IPCStreamMessage_Arguments, endOffset));
static_assert(offsetof(Decoder::SwiftStreamArguments, status) == offsetof(WebKitSwift_IPCStreamMessage_Arguments, status));
static_assert(offsetof(Decoder::SwiftStreamArguments, flag) == offsetof(WebKitSwift_IPCStreamMessage_Arguments, flag));
static_assert(std::to_underlying(Decoder::SwiftStreamArguments::Status::None) == WebKitSwift_IPCStreamMessage_ArgumentsNone);
static_assert(std::to_underlying(Decoder::SwiftStreamArguments::Status::Valid) == WebKitSwift_IPCStreamMessage_ArgumentsValid);
static_assert(std::to_underlying(Decoder::SwiftStreamArguments::Status::Invalid) == WebKitSwift_IPCStreamMessage_ArgumentsInvalid);
static_assert(std::to_underlying(Decoder::SwiftStreamArguments::Status::InvalidIdentifier) == WebKitSwift_IPCStreamMessage_ArgumentsInvalidIdentifier);
// Swift stores the fields at these offsets (IPCStreamArguments.store(into:) in IPCArgDecoders.swift).
static_assert(sizeof(WebKitSwift_IPCStreamMessage_Arguments) == 32 && offsetof(WebKitSwift_IPCStreamMessage_Arguments, valuesOffset) == 8
    && offsetof(WebKitSwift_IPCStreamMessage_Arguments, endOffset) == 16 && offsetof(WebKitSwift_IPCStreamMessage_Arguments, status) == 24
    && offsetof(WebKitSwift_IPCStreamMessage_Arguments, flag) == 25);

// The argument layout of each message name, for the Swift decoder: a name is valid iff it indexes the table.
static constexpr auto swiftArgumentLayouts = [] {
    std::array<uint8_t, std::to_underlying(MessageName::Last) + 1> layouts { };
    for (size_t name = 0; name < layouts.size(); ++name)
        layouts[name] = swiftArgumentLayout(static_cast<MessageName>(name));
    return layouts;
}();

// The header is decoded, and the arguments of a message with a layout are located, in Swift
// (Source/WebKit/swift/IPCStreamMessage), which IPC::handleMessage() then builds through the message's
// decodeArgumentsInSwift() glue.
Decoder::Decoder(std::span<const uint8_t> stream, uint64_t destinationID)
    : m_buffer { stream }
    , m_bufferPosition { m_buffer.begin() }
    , m_bufferDeallocator { nullptr }
    , m_destinationID { destinationID }
{
    // messageIsAsyncWithReply() and messageIsSync() hold for exactly the names >= firstMessageNameWithReplyID, and
    // AsyncReplyID and SyncRequestID have the same wire format and validator.
    static constexpr auto firstMessageNameWithReplyID = std::min(MessageName::FirstAsyncWithReply, MessageName::FirstSynchronous);
    uint64_t result = WebKitSwift_IPCStreamMessage_decode(m_buffer.data(), m_buffer.size(), destinationID,
        std::to_underlying(firstMessageNameWithReplyID), swiftArgumentLayouts.data(), swiftArgumentLayouts.size(), &m_replyID,
        reinterpret_cast<WebKitSwift_IPCStreamMessage_Arguments*>(&m_swiftStreamArguments));
    auto status = static_cast<uint8_t>(result >> 16);
    if (status == WebKitSwift_IPCStreamMessage_Success) [[likely]] {
        m_messageName = static_cast<MessageName>(static_cast<uint16_t>(result));
        m_argumentsOffset = static_cast<size_t>(result >> 24);
        m_bufferPosition = m_buffer.begin() + m_argumentsOffset;
        return;
    }
    // The message name is stored before decoding the reply ID, and reported for an invalid message.
    if (status == WebKitSwift_IPCStreamMessage_TruncatedReplyID || status == WebKitSwift_IPCStreamMessage_InvalidReplyID)
        m_messageName = static_cast<MessageName>(static_cast<uint16_t>(result));
#if ENABLE(IPC_TESTING_API)
    if (status == WebKitSwift_IPCStreamMessage_InvalidReplyID)
        setErrorString("Validation failed: WTF::ObjectIdentifierGenericBase::isValidIdentifier(*toUInt64)"_s);
#endif
    markInvalid();
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
