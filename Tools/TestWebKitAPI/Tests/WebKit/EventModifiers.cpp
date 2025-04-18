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

#if WK_HAVE_C_SPI

#include "JavaScriptTest.h"
#include "PlatformUtilities.h"
#include "PlatformWebView.h"

namespace TestWebKitAPI {

static bool didFinishLoad { false };
static bool mouseMoveCallbackFinished { false };

static void didFinishNavigation(WKPageRef, WKNavigationRef, WKTypeRef, const void*)
{
    didFinishLoad = true;
}

static void mouseDidMoveOverElement(WKPageRef, WKHitTestResultRef, WKEventModifiers modifiers, WKTypeRef, const void*)
{
    EXPECT_EQ(modifiers, kWKEventModifiersControlKey);
    mouseMoveCallbackFinished = true;
}

static void setClients(WKPageRef page)
{
    WKPageNavigationClientV0 loaderClient;
    zeroBytes(loaderClient);
    loaderClient.base.version = 0;
    loaderClient.didFinishNavigation = didFinishNavigation;
    WKPageSetPageNavigationClient(page, &loaderClient.base);
    
    WKPageUIClientV1 uiClient;
    zeroBytes(uiClient);
    uiClient.base.version = 1;
    uiClient.mouseDidMoveOverElement = mouseDidMoveOverElement;
    WKPageSetPageUIClient(page, &uiClient.base);
}

TEST(WebKit, EventModifiers)
{
    WKRetainPtr<WKContextRef> context = adoptWK(WKContextCreateWithConfiguration(nullptr));
    
    PlatformWebView webView(context.get());
    setClients(webView.page());
    
    WKRetainPtr<WKURLRef> url = adoptWK(Util::createURLForResource("simple", "html"));
    WKPageLoadURL(webView.page(), url.get());
    Util::run(&didFinishLoad);
    
    webView.simulateMouseMove(10, 10, kWKEventModifiersControlKey);
    Util::run(&mouseMoveCallbackFinished);
}

} // namespace TestWebKitAPI

#endif
