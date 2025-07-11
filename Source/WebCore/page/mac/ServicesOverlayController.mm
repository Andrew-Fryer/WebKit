/*
 * Copyright (C) 2014 Apple Inc. All rights reserved.
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

#import "config.h"
#import "ServicesOverlayController.h"

#if (ENABLE(SERVICE_CONTROLS) || ENABLE(TELEPHONE_NUMBER_DETECTION)) && PLATFORM(MAC)

#import "BoundaryPointInlines.h"
#import "Chrome.h"
#import "ChromeClient.h"
#import "Document.h"
#import "Editor.h"
#import "EventHandler.h"
#import "FloatQuad.h"
#import "FocusController.h"
#import "FrameSelection.h"
#import "GapRects.h"
#import "GraphicsContext.h"
#import "GraphicsLayer.h"
#import "GraphicsLayerCA.h"
#import "LocalFrame.h"
#import "LocalFrameView.h"
#import "Logging.h"
#import "MouseEventTypes.h"
#import "Page.h"
#import "PageOverlayController.h"
#import "Settings.h"
#import "TextIterator.h"
#import <QuartzCore/QuartzCore.h>
#import <wtf/TZoneMallocInlines.h>
#import <pal/mac/DataDetectorsSoftLink.h>

namespace WebCore {

WTF_MAKE_TZONE_ALLOCATED_IMPL(ServicesOverlayController);

ServicesOverlayController::ServicesOverlayController(Page& page)
    : m_page(page)
    , m_determineActiveHighlightTimer(*this, &ServicesOverlayController::determineActiveHighlightTimerFired)
    , m_buildHighlightsTimer(*this, &ServicesOverlayController::buildPotentialHighlightsIfNeeded)
{
}

ServicesOverlayController::~ServicesOverlayController() = default;

void ServicesOverlayController::ref() const
{
    m_page->ref();
}

void ServicesOverlayController::deref() const
{
    m_page->deref();
}

void ServicesOverlayController::willMoveToPage(PageOverlay&, Page* page)
{
    if (page)
        return;

    ASSERT(m_servicesOverlay);
    m_servicesOverlay = nullptr;
}

void ServicesOverlayController::didMoveToPage(PageOverlay&, Page*)
{
}

static const uint8_t AlignmentNone = 0;
static const uint8_t AlignmentLeft = 1 << 0;
static const uint8_t AlignmentRight = 1 << 1;

static void expandForGap(Vector<LayoutRect>& rects, std::span<uint8_t> alignments, const GapRects& gap)
{
    if (!gap.left().isEmpty()) {
        LayoutUnit leftEdge = gap.left().x();
        for (unsigned i = 0; i < rects.size(); ++i) {
            if (alignments[i] & AlignmentLeft)
                rects[i].shiftXEdgeTo(leftEdge);
        }
    }

    if (!gap.right().isEmpty()) {
        LayoutUnit rightEdge = gap.right().maxX();
        for (unsigned i = 0; i < rects.size(); ++i) {
            if (alignments[i] & AlignmentRight)
                rects[i].shiftMaxXEdgeTo(rightEdge);
        }
    }
}

static inline void stitchRects(Vector<LayoutRect>& rects)
{
    if (rects.size() <= 1)
        return;
    
    Vector<LayoutRect> newRects;
    
    // FIXME: Need to support vertical layout.
    // First stitch together all the rects on the first line of the selection.
    size_t indexFromStart = 0;
    LayoutUnit firstTop = rects[indexFromStart].y();
    LayoutRect& currentRect = rects[indexFromStart];
    while (indexFromStart < rects.size() && rects[indexFromStart].y() == firstTop)
        currentRect.unite(rects[indexFromStart++]);
    
    newRects.append(currentRect);
    if (indexFromStart == rects.size()) {
        // All the rects are on one line. There is nothing else to do.
        rects.swap(newRects);
        return;
    }
    
    // Next stitch together all the rects on the last line of the selection.
    size_t indexFromEnd = rects.size() - 1;
    LayoutUnit lastTop = rects[indexFromEnd].y();
    LayoutRect lastRect = rects[indexFromEnd];
    while (indexFromEnd >= indexFromStart && rects[indexFromEnd].y() == lastTop)
        lastRect.unite(rects[indexFromEnd--]);
    
    // indexFromStart is the index of the first rectangle on the second line.
    // indexFromEnd is the index of the last rectangle on the second to the last line.
    // if they are equal, there is one additional rectangle for the line in the middle.
    if (indexFromEnd == indexFromStart)
        newRects.append(rects[indexFromStart]);
    
    if (indexFromEnd <= indexFromStart) {
        // There are no more rects to stitch. Just append the last line.
        newRects.append(lastRect);
        rects.swap(newRects);
        return;
    }
    
    // Stitch together all the rects after the first line until the second to the last included.
    currentRect = rects[indexFromStart];
    while (indexFromStart != indexFromEnd)
        currentRect.unite(rects[++indexFromStart]);
    
    newRects.append(currentRect);
    newRects.append(lastRect);

    rects.swap(newRects);
}

static void compactRectsWithGapRects(Vector<LayoutRect>& rects, const Vector<GapRects>& gapRects)
{
    stitchRects(rects);
    
    // FIXME: The following alignments are correct for LTR text.
    // We should also account for RTL.
    std::array<uint8_t, 3> alignments;
    if (rects.size() == 1) {
        alignments[0] = AlignmentLeft | AlignmentRight;
        alignments[1] = AlignmentNone;
        alignments[2] = AlignmentNone;
    } else if (rects.size() == 2) {
        alignments[0] = AlignmentRight;
        alignments[1] = AlignmentLeft;
        alignments[2] = AlignmentNone;
    } else {
        alignments[0] = AlignmentRight;
        alignments[1] = AlignmentLeft | AlignmentRight;
        alignments[2] = AlignmentLeft;
    }

    // Account for each GapRects by extending the edge of certain LayoutRects to meet the gap.
    for (auto& gap : gapRects)
        expandForGap(rects, alignments, gap);

    // If we have 3 rects we might need one final GapRects to align the edges.
    if (rects.size() == 3) {
        LayoutRect left;
        LayoutRect right;
        for (unsigned i = 0; i < 3; ++i) {
            if (alignments[i] & AlignmentLeft) {
                if (left.isEmpty())
                    left = rects[i];
                else if (rects[i].x() < left.x())
                    left = rects[i];
            }
            if (alignments[i] & AlignmentRight) {
                if (right.isEmpty())
                    right = rects[i];
                else if ((rects[i].x() + rects[i].width()) > (right.x() + right.width()))
                    right = rects[i];
            }
        }

        if (!left.isEmpty() || !right.isEmpty()) {
            GapRects gap;
            gap.uniteLeft(left);
            gap.uniteRight(right);
            expandForGap(rects, alignments, gap);
        }
    }
}

void ServicesOverlayController::selectionRectsDidChange(const Vector<LayoutRect>& rects, const Vector<GapRects>& gapRects, bool isTextOnly)
{
    m_currentSelectionRects = rects;
    m_isTextOnly = isTextOnly;

    m_lastSelectionChangeTime = MonotonicTime::now();

    compactRectsWithGapRects(m_currentSelectionRects, gapRects);

    // DataDetectors needs these reversed in order to place the arrow in the right location.
    m_currentSelectionRects.reverse();

    LOG(Services, "ServicesOverlayController - Selection rects changed - Now have %lu\n", rects.size());
    invalidateHighlightsOfType(DataDetectorHighlight::Type::Selection);
}

void ServicesOverlayController::selectedTelephoneNumberRangesChanged()
{
    LOG(Services, "ServicesOverlayController - Telephone number ranges changed\n");
    invalidateHighlightsOfType(DataDetectorHighlight::Type::TelephoneNumber);
}

void ServicesOverlayController::invalidateHighlightsOfType(DataDetectorHighlight::Type type)
{
    if (!m_page->settings().serviceControlsEnabled())
        return;

    m_dirtyHighlightTypes.add(type);
    m_buildHighlightsTimer.startOneShot(0_s);
}

void ServicesOverlayController::buildPotentialHighlightsIfNeeded()
{
    if (m_dirtyHighlightTypes.isEmpty())
        return;

    if (m_dirtyHighlightTypes.contains(DataDetectorHighlight::Type::TelephoneNumber))
        buildPhoneNumberHighlights();

    if (m_dirtyHighlightTypes.contains(DataDetectorHighlight::Type::Selection))
        buildSelectionHighlight();

    m_dirtyHighlightTypes = { };

    if (m_potentialHighlights.isEmpty()) {
        if (RefPtr servicesOverlay = m_servicesOverlay.get())
            m_page->pageOverlayController().uninstallPageOverlay(*servicesOverlay, PageOverlay::FadeMode::DoNotFade);
        return;
    }

    if (telephoneNumberRangesForFocusedFrame().isEmpty() && !hasRelevantSelectionServices())
        return;

    createOverlayIfNeeded();

    bool mouseIsOverButton;
    determineActiveHighlight(mouseIsOverButton);
}

bool ServicesOverlayController::mouseIsOverHighlight(DataDetectorHighlight& highlight, bool& mouseIsOverButton) const
{
    if (!PAL::isDataDetectorsFrameworkAvailable())
        return false;

    Boolean onButton;
    bool hovered = PAL::softLink_DataDetectors_DDHighlightPointIsOnHighlight(highlight.highlight(), (CGPoint)m_mousePosition, &onButton);
    mouseIsOverButton = onButton;
    return hovered;
}

Seconds ServicesOverlayController::remainingTimeUntilHighlightShouldBeShown(DataDetectorHighlight* highlight) const
{
    if (!highlight)
        return 0_s;

    Ref page = m_page.get();
    RefPtr localMainFrame = page->localMainFrame();
    if (!localMainFrame)
        return 0_s;

    Seconds minimumTimeUntilHighlightShouldBeShown = 200_ms;
    RefPtr focusedOrMainFrame = page->focusController().focusedOrMainFrame();
    if (focusedOrMainFrame && focusedOrMainFrame->selection().selection().isContentEditable())
        minimumTimeUntilHighlightShouldBeShown = 1_s;

    bool mousePressed = localMainFrame->eventHandler().mousePressed();

    // Highlight hysteresis is only for selection services, because telephone number highlights are already much more stable
    // by virtue of being expanded to include the entire telephone number. However, we will still avoid highlighting
    // telephone numbers while the mouse is down.
    if (highlight->type() == DataDetectorHighlight::Type::TelephoneNumber)
        return mousePressed ? minimumTimeUntilHighlightShouldBeShown : 0_s;

    MonotonicTime now = MonotonicTime::now();
    Seconds timeSinceLastSelectionChange = now - m_lastSelectionChangeTime;
    Seconds timeSinceHighlightBecameActive = now - m_nextActiveHighlightChangeTime;
    Seconds timeSinceLastMouseUp = mousePressed ? 0_s : now - m_lastMouseUpTime;

    return minimumTimeUntilHighlightShouldBeShown - std::min(std::min(timeSinceLastSelectionChange, timeSinceHighlightBecameActive), timeSinceLastMouseUp);
}

void ServicesOverlayController::determineActiveHighlightTimerFired()
{
    bool mouseIsOverButton;
    determineActiveHighlight(mouseIsOverButton);
}

void ServicesOverlayController::drawRect(PageOverlay&, GraphicsContext&, const IntRect&)
{
}

void ServicesOverlayController::clearActiveHighlight()
{
    if (!m_activeHighlight)
        return;

    if (m_currentMouseDownOnButtonHighlight == m_activeHighlight)
        m_currentMouseDownOnButtonHighlight = nullptr;
    m_activeHighlight = nullptr;
}

void ServicesOverlayController::removeAllPotentialHighlightsOfType(DataDetectorHighlight::Type type)
{
    Vector<RefPtr<DataDetectorHighlight>> highlightsToRemove;
    for (auto& highlight : m_potentialHighlights) {
        if (highlight->type() == type)
            highlightsToRemove.append(highlight);
    }

    while (!highlightsToRemove.isEmpty())
        m_potentialHighlights.remove(highlightsToRemove.takeLast());
}

void ServicesOverlayController::buildPhoneNumberHighlights()
{
    Ref page = m_page.get();
    RefPtr localMainFrame = page->localMainFrame();
    if (!localMainFrame)
        return;

    Vector<SimpleRange> phoneNumberRanges;
    for (Frame* frame = &page->mainFrame(); frame; frame = frame->tree().traverseNext()) {
        auto* localFrame = dynamicDowncast<LocalFrame>(frame);
        if (!localFrame)
            continue;
        phoneNumberRanges.appendVector(localFrame->editor().detectedTelephoneNumberRanges());
    }

    if (phoneNumberRanges.isEmpty()) {
        removeAllPotentialHighlightsOfType(DataDetectorHighlight::Type::TelephoneNumber);
        return;
    }

    if (!PAL::isDataDetectorsFrameworkAvailable())
        return;

    HashSet<RefPtr<DataDetectorHighlight>> newPotentialHighlights;

    auto& mainFrameView = *localMainFrame->view();

    for (auto& range : phoneNumberRanges) {
        // FIXME: This makes a big rect if the range extends from the end of one line to the start of the next. Handle that case better?
        auto rect = enclosingIntRect(unitedBoundingBoxes(RenderObject::absoluteTextQuads(range)));

        // Convert to the main document's coordinate space.
        // FIXME: It's a little crazy to call contentsToWindow and then windowToContents in order to get the right coordinate space.
        // We should consider adding conversion functions to ScrollView for contentsToDocument(). Right now, contentsToRootView() is
        // not equivalent to what we need when you have a content inset or a header banner.
        auto* viewForRange = range.start.document().view();
        if (!viewForRange)
            continue;

        rect.setLocation(mainFrameView.windowToContents(viewForRange->contentsToWindow(rect.location())));

        CGRect cgRect = rect;
        auto ddHighlight = adoptCF(PAL::softLink_DataDetectors_DDHighlightCreateWithRectsInVisibleRectWithStyleScaleAndDirection(nullptr, &cgRect, 1, mainFrameView.visibleContentRect(), static_cast<DDHighlightStyle>(DDHighlightStyleBubbleStandard) | static_cast<DDHighlightStyle>(DDHighlightStyleStandardIconArrow), YES, NSWritingDirectionNatural, NO, YES, 0));
        auto highlight = DataDetectorHighlight::createForTelephoneNumber(*this, WTFMove(ddHighlight), WTFMove(range));
        m_highlights.add(highlight.get());
        newPotentialHighlights.add(WTFMove(highlight));
    }

    replaceHighlightsOfTypePreservingEquivalentHighlights(newPotentialHighlights, DataDetectorHighlight::Type::TelephoneNumber);
}

void ServicesOverlayController::buildSelectionHighlight()
{
    if (m_currentSelectionRects.isEmpty()) {
        removeAllPotentialHighlightsOfType(DataDetectorHighlight::Type::Selection);
        return;
    }

    if (!PAL::isDataDetectorsFrameworkAvailable())
        return;

    HashSet<RefPtr<DataDetectorHighlight>> newPotentialHighlights;

    Ref page = m_page.get();
    if (auto selectionRange = page->selection().firstRange()) {
        RefPtr mainFrameView = page->mainFrame().virtualView();
        if (!mainFrameView)
            return;

        RefPtr viewForRange = selectionRange->start.document().view();
        if (!viewForRange)
            return;

        auto cgRects = WTF::map(m_currentSelectionRects, [&](auto& rect) -> CGRect {
            IntRect currentRect = snappedIntRect(rect);
            currentRect.setLocation(mainFrameView->windowToContents(viewForRange->contentsToWindow(currentRect.location())));
            return currentRect;
        });

        if (!cgRects.isEmpty()) {
            CGRect visibleRect = mainFrameView->visibleContentRect();
            auto ddHighlight = adoptCF(PAL::softLink_DataDetectors_DDHighlightCreateWithRectsInVisibleRectWithStyleScaleAndDirection(nullptr, cgRects.begin(), cgRects.size(), visibleRect, static_cast<DDHighlightStyle>(DDHighlightStyleBubbleNone) | static_cast<DDHighlightStyle>(DDHighlightStyleStandardIconArrow) | static_cast<DDHighlightStyle>(DDHighlightStyleButtonShowAlways), YES, NSWritingDirectionNatural, NO, YES, 0));
            auto highlight = DataDetectorHighlight::createForSelection(*this, WTFMove(ddHighlight), WTFMove(*selectionRange));
            m_highlights.add(highlight.get());
            newPotentialHighlights.add(WTFMove(highlight));
        }
    }

    replaceHighlightsOfTypePreservingEquivalentHighlights(newPotentialHighlights, DataDetectorHighlight::Type::Selection);
}

void ServicesOverlayController::replaceHighlightsOfTypePreservingEquivalentHighlights(HashSet<RefPtr<DataDetectorHighlight>>& newPotentialHighlights, DataDetectorHighlight::Type type)
{
    // If any old Highlights are equivalent (by Range) to a new Highlight, reuse the old
    // one so that any metadata is retained.
    HashSet<RefPtr<DataDetectorHighlight>> reusedPotentialHighlights;

    for (auto& oldHighlight : m_potentialHighlights) {
        if (oldHighlight->type() != type)
            continue;

        for (auto& newHighlight : newPotentialHighlights) {
            if (areEquivalent(oldHighlight.get(), newHighlight.get())) {
                oldHighlight->setHighlight(newHighlight->highlight());

                reusedPotentialHighlights.add(oldHighlight);
                newPotentialHighlights.remove(newHighlight);

                break;
            }
        }
    }

    removeAllPotentialHighlightsOfType(type);

    m_potentialHighlights.add(newPotentialHighlights.begin(), newPotentialHighlights.end());
    m_potentialHighlights.add(reusedPotentialHighlights.begin(), reusedPotentialHighlights.end());
}

bool ServicesOverlayController::hasRelevantSelectionServices()
{
    return m_page->chrome().client().hasRelevantSelectionServices(m_isTextOnly);
}

void ServicesOverlayController::createOverlayIfNeeded()
{
    if (m_servicesOverlay)
        return;

    Ref page = m_page.get();
    if (!page->settings().serviceControlsEnabled())
        return;

    auto overlay = PageOverlay::create(*this, PageOverlay::OverlayType::Document);
    m_servicesOverlay = overlay.ptr();
    page->pageOverlayController().installPageOverlay(WTFMove(overlay), PageOverlay::FadeMode::DoNotFade);
}

Vector<SimpleRange> ServicesOverlayController::telephoneNumberRangesForFocusedFrame()
{
    RefPtr focusedOrMainFrame = m_page->focusController().focusedOrMainFrame();
    if (!focusedOrMainFrame)
        return { };
    return focusedOrMainFrame->editor().detectedTelephoneNumberRanges();
}

DataDetectorHighlight* ServicesOverlayController::findTelephoneNumberHighlightContainingSelectionHighlight(DataDetectorHighlight& selectionHighlight)
{
    if (selectionHighlight.type() != DataDetectorHighlight::Type::Selection)
        return nullptr;

    auto selectionRange = m_page->selection().toNormalizedRange();
    if (!selectionRange)
        return nullptr;

    for (auto& highlight : m_potentialHighlights) {
        if (highlight->type() == DataDetectorHighlight::Type::TelephoneNumber && contains<ComposedTree>(highlight->range(), *selectionRange))
            return highlight.get();
    }

    return nullptr;
}

void ServicesOverlayController::determineActiveHighlight(bool& mouseIsOverActiveHighlightButton)
{
    buildPotentialHighlightsIfNeeded();

    mouseIsOverActiveHighlightButton = false;

    RefPtr<DataDetectorHighlight> newActiveHighlight;

    for (auto& highlight : m_potentialHighlights) {
        if (highlight->type() == DataDetectorHighlight::Type::Selection) {
            // If we've already found a new active highlight, and it's
            // a telephone number highlight, prefer that over this selection highlight.
            if (newActiveHighlight && newActiveHighlight->type() == DataDetectorHighlight::Type::TelephoneNumber)
                continue;

            // If this highlight has no compatible services, it can't be active.
            if (!hasRelevantSelectionServices())
                continue;
        }

        // If this highlight isn't hovered, it can't be active.
        bool mouseIsOverButton;
        if (!mouseIsOverHighlight(*highlight, mouseIsOverButton))
            continue;

        newActiveHighlight = highlight;
        mouseIsOverActiveHighlightButton = mouseIsOverButton;
    }

    // If our new active highlight is a selection highlight that is completely contained
    // by one of the phone number highlights, we'll make the phone number highlight active even if it's not hovered.
    if (newActiveHighlight && newActiveHighlight->type() == DataDetectorHighlight::Type::Selection) {
        if (DataDetectorHighlight* containedTelephoneNumberHighlight = findTelephoneNumberHighlightContainingSelectionHighlight(*newActiveHighlight)) {
            newActiveHighlight = containedTelephoneNumberHighlight;

            // We will always initially choose the telephone number highlight over the selection highlight if the
            // mouse is over the telephone number highlight's button, so we know that it's not hovered if we got here.
            mouseIsOverActiveHighlightButton = false;
        }
    }

    if (!areEquivalent(m_activeHighlight.get(), newActiveHighlight.get())) {
        // When transitioning to a new highlight, we might end up in determineActiveHighlight multiple times
        // before the new highlight actually becomes active. Keep track of the last next-but-not-yet-active
        // highlight, and only reset the active highlight hysteresis when that changes.
        if (m_nextActiveHighlight != newActiveHighlight) {
            m_nextActiveHighlight = newActiveHighlight;
            m_nextActiveHighlightChangeTime = MonotonicTime::now();
        }

        m_currentMouseDownOnButtonHighlight = nullptr;

        if (m_activeHighlight) {
            m_activeHighlight->fadeOut();
            m_activeHighlight = nullptr;
        }

        auto remainingTimeUntilHighlightShouldBeShown = this->remainingTimeUntilHighlightShouldBeShown(newActiveHighlight.get());
        if (remainingTimeUntilHighlightShouldBeShown > 0_s) {
            m_determineActiveHighlightTimer.startOneShot(remainingTimeUntilHighlightShouldBeShown);
            return;
        }

        m_activeHighlight = WTFMove(m_nextActiveHighlight);

        if (m_activeHighlight) {
            Ref<GraphicsLayer> highlightLayer = m_activeHighlight->layer();
            m_servicesOverlay->layer().addChild(WTFMove(highlightLayer));
            m_activeHighlight->fadeIn();
        }
    }
}

bool ServicesOverlayController::mouseEvent(PageOverlay&, const PlatformMouseEvent& event)
{
    m_mousePosition = m_page->mainFrame().virtualView()->windowToContents(event.position());

    bool mouseIsOverActiveHighlightButton = false;
    determineActiveHighlight(mouseIsOverActiveHighlightButton);

    // Cancel the potential click if any button other than the left button changes state, and ignore the event.
    if (event.button() != MouseButton::Left) {
        m_currentMouseDownOnButtonHighlight = nullptr;
        return false;
    }

    // If the mouse lifted while still over the highlight button that it went down on, then that is a click.
    if (event.type() == PlatformEvent::Type::MouseReleased) {
        auto mouseDownHighlight = m_currentMouseDownOnButtonHighlight.copyRef();
        m_currentMouseDownOnButtonHighlight = nullptr;

        m_lastMouseUpTime = MonotonicTime::now();

        if (mouseIsOverActiveHighlightButton && mouseDownHighlight) {
            handleClick(m_mousePosition, *mouseDownHighlight);
            return true;
        }
        
        return false;
    }

    // If the mouse moved outside of the button tracking a potential click, stop tracking the click.
    if (event.type() == PlatformEvent::Type::MouseMoved) {
        if (m_currentMouseDownOnButtonHighlight && mouseIsOverActiveHighlightButton)
            return true;

        m_currentMouseDownOnButtonHighlight = nullptr;
        return false;
    }

    // If the mouse went down over the active highlight's button, track this as a potential click.
    if (event.type() == PlatformEvent::Type::MousePressed) {
        if (m_activeHighlight && mouseIsOverActiveHighlightButton) {
            m_currentMouseDownOnButtonHighlight = m_activeHighlight;
            return true;
        }

        return false;
    }

    return false;
}

void ServicesOverlayController::didScrollFrame(PageOverlay&, LocalFrame& frame)
{
    if (frame.isMainFrame())
        return;

    invalidateHighlightsOfType(DataDetectorHighlight::Type::TelephoneNumber);
    invalidateHighlightsOfType(DataDetectorHighlight::Type::Selection);
    buildPotentialHighlightsIfNeeded();

    bool mouseIsOverActiveHighlightButton;
    determineActiveHighlight(mouseIsOverActiveHighlightButton);
}

void ServicesOverlayController::handleClick(const IntPoint& clickPoint, DataDetectorHighlight& highlight)
{
    Ref page = m_page.get();
    RefPtr frameView = page->mainFrame().virtualView();
    if (!frameView)
        return;

    IntPoint windowPoint = frameView->contentsToWindow(clickPoint);

    RefPtr focusedOrMainFrame = page->focusController().focusedOrMainFrame();
    if (!focusedOrMainFrame)
        return;

    if (highlight.type() == DataDetectorHighlight::Type::Selection) {
        auto selectedTelephoneNumbers = telephoneNumberRangesForFocusedFrame().map([](auto& range) {
            return plainText(range);
        });

        page->chrome().client().handleSelectionServiceClick(focusedOrMainFrame->frameID(), focusedOrMainFrame->selection(), WTFMove(selectedTelephoneNumbers), windowPoint);
    } else if (highlight.type() == DataDetectorHighlight::Type::TelephoneNumber)
        page->chrome().client().handleTelephoneNumberClick(plainText(highlight.range()), windowPoint, frameView->contentsToWindow(focusedOrMainFrame->editor().firstRectForRange(highlight.range())));
}

#pragma mark - DataDetectorHighlightClient

#if ENABLE(DATA_DETECTION)

void ServicesOverlayController::scheduleRenderingUpdate(OptionSet<RenderingUpdateStep> requestedSteps)
{
    protectedPage()->scheduleRenderingUpdate(requestedSteps);
}

float ServicesOverlayController::deviceScaleFactor() const
{
    return protectedPage()->deviceScaleFactor();
}

RefPtr<GraphicsLayer> ServicesOverlayController::createGraphicsLayer(GraphicsLayerClient& client)
{
    return GraphicsLayer::create(protectedPage()->chrome().client().graphicsLayerFactory(), client);
}

#endif

} // namespace WebKit

#endif // (ENABLE(SERVICE_CONTROLS) || ENABLE(TELEPHONE_NUMBER_DETECTION)) && PLATFORM(MAC)
