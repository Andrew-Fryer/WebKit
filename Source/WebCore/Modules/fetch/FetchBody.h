/*
 * Copyright (C) 2016 Canon Inc.
 * Copyright (C) 2016-2024 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions
 * are required to be met:
 *
 * 1.  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 * 2.  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 3.  Neither the name of Canon Inc. nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CANON INC. AND ITS CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CANON INC. AND ITS CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "DOMFormData.h"
#include "FetchBodyConsumer.h"
#include "FormData.h"
#include "ReadableStream.h"
#include "URLSearchParams.h"

namespace WebCore {

class DeferredPromise;
class FetchBodyOwner;
class FetchBodySource;
class ScriptExecutionContext;

template<typename> class ExceptionOr;

class FetchBody {
public:
    void arrayBuffer(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void blob(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void bytes(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void json(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void text(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void formData(FetchBodyOwner&, Ref<DeferredPromise>&&);

    void consumeAsStream(FetchBodyOwner&, FetchBodySource&);

    using Init = Variant<RefPtr<Blob>, RefPtr<ArrayBufferView>, RefPtr<ArrayBuffer>, RefPtr<DOMFormData>, RefPtr<URLSearchParams>, RefPtr<ReadableStream>, String>;
    static ExceptionOr<FetchBody> extract(Init&&, String&);
    FetchBody() = default;
    FetchBody(FetchBody&&) = default;
    WEBCORE_EXPORT ~FetchBody();
    FetchBody& operator=(FetchBody&&) = default;

    explicit FetchBody(String&& data)
        : m_data(WTFMove(data))
    {
    }

    WEBCORE_EXPORT static std::optional<FetchBody> fromFormData(ScriptExecutionContext&, Ref<FormData>&&);

    void loadingFailed(const Exception&);
    void loadingSucceeded(const String& contentType);

    RefPtr<FormData> bodyAsFormData() const;

    using TakenData = Variant<std::nullptr_t, Ref<FormData>, Ref<SharedBuffer>>;
    TakenData take();

    void setAsFormData(Ref<FormData>&& data) { m_data = WTFMove(data); }
    FetchBodyConsumer& consumer() { return m_consumer; }
    CheckedRef<FetchBodyConsumer> checkedConsumer() { return consumer(); }

    void consumeOnceLoadingFinished(FetchBodyConsumer::Type, Ref<DeferredPromise>&&);
    void cleanConsumer() { m_consumer.clean(); }
    bool hasConsumerPendingActivity() const { return m_consumer.hasPendingActivity(); }

    FetchBody clone();

    bool hasReadableStream() const { return !!m_readableStream; }
    const ReadableStream* readableStream() const { return m_readableStream.get(); }
    ReadableStream* readableStream() { return m_readableStream.get(); }
    RefPtr<const ReadableStream> protectedReadableStream() const { return readableStream(); }
    RefPtr<ReadableStream> protectedReadableStream() { return readableStream(); }
    void setReadableStream(Ref<ReadableStream>&& stream)
    {
        ASSERT(!m_readableStream);
        m_readableStream = WTFMove(stream);
    }

    void convertReadableStreamToArrayBuffer(FetchBodyOwner&, CompletionHandler<void(std::optional<Exception>&&)>&&);

    bool isBlob() const { return std::holds_alternative<Ref<const Blob>>(m_data); }
    bool isFormData() const { return std::holds_alternative<Ref<FormData>>(m_data); }
    bool isReadableStream() const { return std::holds_alternative<Ref<ReadableStream>>(m_data); }

private:
    explicit FetchBody(Ref<const Blob>&& data) : m_data(WTFMove(data)) { }
    explicit FetchBody(Ref<const ArrayBuffer>&& data) : m_data(WTFMove(data)) { }
    explicit FetchBody(Ref<const ArrayBufferView>&& data) : m_data(WTFMove(data)) { }
    explicit FetchBody(Ref<FormData>&& data) : m_data(WTFMove(data)) { }
    explicit FetchBody(Ref<const URLSearchParams>&& data) : m_data(WTFMove(data)) { }
    explicit FetchBody(Ref<ReadableStream>&& stream) : m_data(stream), m_readableStream(WTFMove(stream)) { }
    explicit FetchBody(FetchBodyConsumer&& consumer) : m_consumer(WTFMove(consumer)) { }

    void consume(FetchBodyOwner&, Ref<DeferredPromise>&&);

    void consumeArrayBuffer(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void consumeArrayBufferView(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void consumeText(FetchBodyOwner&, Ref<DeferredPromise>&&, const String&);
    void consumeBlob(FetchBodyOwner&, Ref<DeferredPromise>&&);
    void consumeFormData(FetchBodyOwner&, Ref<DeferredPromise>&&);

    bool isArrayBuffer() const { return std::holds_alternative<Ref<const ArrayBuffer>>(m_data); }
    bool isArrayBufferView() const { return std::holds_alternative<Ref<const ArrayBufferView>>(m_data); }
    bool isURLSearchParams() const { return std::holds_alternative<Ref<const URLSearchParams>>(m_data); }
    bool isText() const { return std::holds_alternative<String>(m_data); }

    const Blob& blobBody() const { return std::get<Ref<const Blob>>(m_data).get(); }
    Ref<const Blob> protectedBlobBody() const { return blobBody(); }
    FormData& formDataBody() { return std::get<Ref<FormData>>(m_data).get(); }
    Ref<FormData> protectedFormDataBody() { return formDataBody(); }
    const FormData& formDataBody() const { return std::get<Ref<FormData>>(m_data).get(); }
    Ref<const FormData> protectedFormDataBody() const { return formDataBody(); }
    const ArrayBuffer& arrayBufferBody() const { return std::get<Ref<const ArrayBuffer>>(m_data).get(); }
    Ref<const ArrayBuffer> protectedArrayBufferBody() const { return arrayBufferBody(); }
    const ArrayBufferView& arrayBufferViewBody() const { return std::get<Ref<const ArrayBufferView>>(m_data).get(); }
    Ref<const ArrayBufferView> protectedArrayBufferViewBody() const { return arrayBufferViewBody(); }
    String& textBody() { return std::get<String>(m_data); }
    const String& textBody() const { return std::get<String>(m_data); }
    const URLSearchParams& urlSearchParamsBody() const { return std::get<Ref<const URLSearchParams>>(m_data).get(); }
    Ref<const URLSearchParams> protectedURLSearchParamsBody() const { return urlSearchParamsBody(); }

    using Data = Variant<std::nullptr_t, Ref<const Blob>, Ref<FormData>, Ref<const ArrayBuffer>, Ref<const ArrayBufferView>, Ref<const URLSearchParams>, String, Ref<ReadableStream>>;
    Data m_data { nullptr };

    FetchBodyConsumer m_consumer { FetchBodyConsumer::Type::None };
    RefPtr<ReadableStream> m_readableStream;
};

struct FetchBodyWithType {
    FetchBody body;
    String type;
};

} // namespace WebCore
