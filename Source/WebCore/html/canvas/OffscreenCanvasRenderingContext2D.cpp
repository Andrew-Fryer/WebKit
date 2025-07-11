/*
 * Copyright (C) 2004-2017 Apple Inc. All rights reserved.
 * Copyright (C) 2008, 2010 Nokia Corporation and/or its subsidiary(-ies)
 * Copyright (C) 2007 Alp Toker <alp@atoker.com>
 * Copyright (C) 2008 Eric Seidel <eric@webkit.org>
 * Copyright (C) 2008 Dirk Schulze <krit@webkit.org>
 * Copyright (C) 2010 Torch Mobile (Beijing) Co. Ltd. All rights reserved.
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 * Copyright (C) 2013, 2014 Adobe Systems Incorporated. All rights reserved.
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

#include "config.h"
#include "OffscreenCanvasRenderingContext2D.h"

#if ENABLE(OFFSCREEN_CANVAS)

#include "CSSFontSelector.h"
#include "CSSParserContext.h"
#include "CSSPropertyParserConsumer+Font.h"
#include "InspectorInstrumentation.h"
#include "RenderStyle.h"
#include "ScriptExecutionContext.h"
#include "StyleResolveForFont.h"
#include "TextMetrics.h"
#include <wtf/TZoneMallocInlines.h>

namespace WebCore {

WTF_MAKE_TZONE_OR_ISO_ALLOCATED_IMPL(OffscreenCanvasRenderingContext2D);

bool OffscreenCanvasRenderingContext2D::enabledForContext(ScriptExecutionContext& context)
{
    UNUSED_PARAM(context);
#if ENABLE(OFFSCREEN_CANVAS_IN_WORKERS)
    if (context.isWorkerGlobalScope())
        return context.settingsValues().offscreenCanvasInWorkersEnabled;
#endif

    ASSERT(context.isDocument());
    return true;
}


std::unique_ptr<OffscreenCanvasRenderingContext2D> OffscreenCanvasRenderingContext2D::create(CanvasBase& canvas, CanvasRenderingContext2DSettings&& settings)
{
    auto renderingContext = std::unique_ptr<OffscreenCanvasRenderingContext2D>(new OffscreenCanvasRenderingContext2D(canvas, WTFMove(settings)));

    InspectorInstrumentation::didCreateCanvasRenderingContext(*renderingContext);

    return renderingContext;
}

OffscreenCanvasRenderingContext2D::OffscreenCanvasRenderingContext2D(CanvasBase& canvas, CanvasRenderingContext2DSettings&& settings)
    : CanvasRenderingContext2DBase(canvas, Type::Offscreen2D, WTFMove(settings), false)
{
}

void OffscreenCanvasRenderingContext2D::drawText(const String& text, double x, double y, bool fill, std::optional<double> maxWidth)
{
    if (!canDrawText(x, y, fill, maxWidth))
        return;

    String normalizedText = normalizeSpaces(text);
    auto direction = (state().direction == Direction::Rtl) ? TextDirection::RTL : TextDirection::LTR;
    TextRun textRun(normalizedText, 0, 0, ExpansionBehavior::allowRightOnly(), direction, false, true);
    drawTextUnchecked(textRun, x, y, fill, maxWidth);
}

OffscreenCanvasRenderingContext2D::~OffscreenCanvasRenderingContext2D() = default;

void OffscreenCanvasRenderingContext2D::setFont(const String& newFont)
{
    Ref context = *canvasBase().scriptExecutionContext();

    if (newFont.isEmpty())
        return;

    if (newFont == state().unparsedFont && state().font.realized())
        return;

    // According to http://lists.w3.org/Archives/Public/public-html/2009Jul/0947.html,
    // the "inherit" and "initial" values must be ignored. CSSPropertyParserHelpers::parseFont() ignores these.
    auto unresolvedFont = CSSPropertyParserHelpers::parseUnresolvedFont(newFont, context.get(), strictToCSSParserMode(!usesCSSCompatibilityParseMode()));
    if (!unresolvedFont)
        return;

    // The parse succeeded.
    String newFontSafeCopy(newFont); // Create a string copy since newFont can be deleted inside realizeSaves.
    realizeSaves();
    modifiableState().unparsedFont = newFontSafeCopy;

    // Map the <canvas> font into the text style. If the font uses keywords like larger/smaller, these will work
    // relative to the default font.
    FontCascadeDescription fontDescription;
    fontDescription.setOneFamily(DefaultFontFamily);
    fontDescription.setSpecifiedSize(DefaultFontSize);
    fontDescription.setComputedSize(DefaultFontSize);

    if (auto fontCascade = Style::resolveForUnresolvedFont(*unresolvedFont, WTFMove(fontDescription), context.get())) {
        ASSERT(context->cssFontSelector());
        modifiableState().font.initialize(*context->cssFontSelector(), *fontCascade);

        String letterSpacing;
        setLetterSpacing(std::exchange(modifiableState().letterSpacing, letterSpacing));
        String wordSpacing;
        setWordSpacing(std::exchange(modifiableState().wordSpacing, wordSpacing));
    }
}

RefPtr<ImageBuffer> OffscreenCanvasRenderingContext2D::transferToImageBuffer()
{
    if (!canvasBase().hasCreatedImageBuffer())
        return canvasBase().allocateImageBuffer();
    RefPtr buffer = canvasBase().buffer();
    if (!buffer)
        return nullptr;
    // As the canvas context state is stored in GraphicsContext, which is owned
    // by buffer(), to avoid resetting the context state, we have to make a copy and
    // clear the original buffer rather than returning the original buffer.
    RefPtr result = buffer->clone();
    clearCanvas();
    return result;
}

CanvasDirection OffscreenCanvasRenderingContext2D::direction() const
{
    // FIXME: What should we do about inherit here?
    switch (state().direction) {
    case Direction::Inherit:
    case Direction::Ltr:
        return Direction::Ltr;
    case Direction::Rtl:
        return Direction::Rtl;
    }
    ASSERT_NOT_REACHED();
    return Direction::Ltr;
}

auto OffscreenCanvasRenderingContext2D::fontProxy() -> const FontProxy* {
    if (!state().font.realized())
        setFont(state().unparsedFont);
    return &state().font;
}

void OffscreenCanvasRenderingContext2D::fillText(const String& text, double x, double y, std::optional<double> maxWidth)
{
    drawText(text, x, y, true, maxWidth);
}

void OffscreenCanvasRenderingContext2D::strokeText(const String& text, double x, double y, std::optional<double> maxWidth)
{
    drawText(text, x, y, false, maxWidth);
}

Ref<TextMetrics> OffscreenCanvasRenderingContext2D::measureText(const String& text)
{
    return measureTextInternal(text);
}

} // namespace WebCore

#endif
