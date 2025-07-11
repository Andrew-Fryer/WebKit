/*
 * Copyright (C) 2008, 2014 Apple Inc. All rights reserved.
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
 *
 */

#include "config.h"
#include "DOMTimer.h"

#include "HTMLPlugInElement.h"
#include "InspectorInstrumentation.h"
#include "Logging.h"
#include "OpportunisticTaskScheduler.h"
#include "Page.h"
#include "ScheduledAction.h"
#include "ScriptExecutionContextInlines.h"
#include "Settings.h"
#include <wtf/CryptographicallyRandomNumber.h>
#include <wtf/HashMap.h>
#include <wtf/MathExtras.h>
#include <wtf/NeverDestroyed.h>
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

#if ENABLE(CONTENT_CHANGE_OBSERVER)
#include "ContentChangeObserver.h"
#include "DOMTimerHoldingTank.h"
#endif

namespace WebCore {

static constexpr Seconds minIntervalForNonUserObservableChangeTimers { 1_s }; // Empirically determined to maximize battery life.
static constexpr Seconds minIntervalForOneShotTimers { 0_ms };
static constexpr Seconds minIntervalForRepeatingTimers { 1_ms };
static constexpr int maxTimerNestingLevel = 10;
static constexpr int maxTimerNestingLevelForOneShotTimers = 10;
static constexpr int maxTimerNestingLevelForRepeatingTimers = 5;

WTF_MAKE_TZONE_ALLOCATED_IMPL(DOMTimer);

class DOMTimerFireState {
public:
    DOMTimerFireState(ScriptExecutionContext& context, int nestingLevel)
        : m_context(context)
        , m_contextIsDocument(is<Document>(m_context))
        // For worker threads, don't update the current DOMTimerFireState.
        // Setting this from workers would not be thread-safe, and its not relevant to current uses.
        , m_initialDOMTreeVersion(m_contextIsDocument ? downcast<Document>(context).domTreeVersion() : 0)
        , m_previous(m_contextIsDocument ? std::exchange(current, this) : nullptr)
    {
        m_context->setTimerNestingLevel(nestingLevel);
    }

    ~DOMTimerFireState()
    {
        if (m_contextIsDocument)
            current = m_previous;
        m_context->setTimerNestingLevel(0);
    }

    const Document* contextDocument() const { return m_contextIsDocument ? downcast<Document>(m_context.ptr()) : nullptr; }

    void setScriptMadeUserObservableChanges() { m_scriptMadeUserObservableChanges = true; }
    void setScriptMadeNonUserObservableChanges() { m_scriptMadeNonUserObservableChanges = true; }

    bool scriptMadeNonUserObservableChanges() const { return m_scriptMadeNonUserObservableChanges; }
    bool scriptMadeUserObservableChanges() const
    {
        if (m_scriptMadeUserObservableChanges)
            return true;

        auto* document = contextDocument();
        // To be conservative, we also consider any DOM Tree change to be user observable.
        return document && document->domTreeVersion() != m_initialDOMTreeVersion;
    }

    static DOMTimerFireState* current;

private:
    const Ref<ScriptExecutionContext> m_context;
    bool m_contextIsDocument;
    bool m_scriptMadeNonUserObservableChanges { false };
    bool m_scriptMadeUserObservableChanges { false };
    uint64_t m_initialDOMTreeVersion;
    DOMTimerFireState* m_previous;
};

DOMTimerFireState* DOMTimerFireState::current = nullptr;

struct NestedTimersMap {
    typedef HashMap<int, Ref<DOMTimer>>::const_iterator const_iterator;

    static NestedTimersMap* instanceForContext(ScriptExecutionContext& context)
    {
        // For worker threads, we don't use NestedTimersMap as doing so would not
        // be thread safe.
        if (is<Document>(context))
            return &instance();
        return nullptr;
    }

    void startTracking()
    {
        // Make sure we start with an empty HashMap. In theory, it is possible the HashMap is not
        // empty if a timer fires during the execution of another timer (may happen with the
        // in-process Web Inspector).
        nestedTimers.clear();
        isTrackingNestedTimers = true;
    }

    void stopTracking()
    {
        isTrackingNestedTimers = false;
        nestedTimers.clear();
    }

    void add(int timeoutId, Ref<DOMTimer>&& timer)
    {
        if (isTrackingNestedTimers)
            nestedTimers.add(timeoutId, WTFMove(timer));
    }

    void remove(int timeoutId)
    {
        if (isTrackingNestedTimers)
            nestedTimers.remove(timeoutId);
    }

    const_iterator begin() const { return nestedTimers.begin(); }
    const_iterator end() const { return nestedTimers.end(); }

private:
    static NestedTimersMap& instance()
    {
        static NeverDestroyed<NestedTimersMap> map;
        return map;
    }

    static bool isTrackingNestedTimers;
    HashMap<int /* timeoutId */, Ref<DOMTimer>> nestedTimers;
};

bool NestedTimersMap::isTrackingNestedTimers = false;

DOMTimer::DOMTimer(ScriptExecutionContext& context, Function<void(ScriptExecutionContext&)>&& action, Seconds interval, Type type)
    : ActiveDOMObject(&context)
    , m_nestingLevel(context.timerNestingLevel())
    , m_action(WTFMove(action))
    , m_originalInterval(interval)
    , m_throttleState(Undetermined)
    , m_oneShot(type == Type::SingleShot)
    , m_currentTimerInterval(intervalClampedToMinimum())
    , m_userGestureTokenToForward(UserGestureIndicator::currentUserGesture())
{
    CheckedRef eventLoop = context.eventLoop();
    m_hasReachedMaxNestingLevel = m_nestingLevel >= (m_oneShot ? maxTimerNestingLevelForOneShotTimers : maxTimerNestingLevelForRepeatingTimers);
    if (m_oneShot) {
        m_timer = eventLoop->scheduleTask(m_currentTimerInterval, context, m_hasReachedMaxNestingLevel ? HasReachedMaxNestingLevel::Yes : HasReachedMaxNestingLevel::No, TaskSource::Timer, [weakThis = WeakPtr { *this }] {
            if (RefPtr protectedThis = weakThis.get())
                protectedThis->fired();
        });
    } else {
        m_timer = eventLoop->scheduleRepeatingTask(m_originalInterval, m_currentTimerInterval, context, m_hasReachedMaxNestingLevel ? HasReachedMaxNestingLevel::Yes : HasReachedMaxNestingLevel::No, TaskSource::Timer, [weakThis = WeakPtr { *this }] {
            if (RefPtr protectedThis = weakThis.get())
                protectedThis->fired();
        });
    }
}

DOMTimer::~DOMTimer() = default;

int DOMTimer::install(ScriptExecutionContext& context, std::unique_ptr<ScheduledAction> action, Seconds timeout, Type type)
{
    auto actionFunction = [action = WTFMove(action)](ScriptExecutionContext& context) mutable {
        action->execute(context);
    };
    return DOMTimer::install(context, WTFMove(actionFunction), timeout, type);
}

int DOMTimer::install(ScriptExecutionContext& context, Function<void(ScriptExecutionContext&)>&& action, Seconds timeout, Type type)
{
    Ref timer = adoptRef(*new DOMTimer(context, WTFMove(action), timeout, type));
    timer->suspendIfNeeded();
    timer->makeImminentlyScheduledWorkScopeIfPossible(context);

    // Keep asking for the next id until we're given one that we don't already have.
    do {
        timer->m_timeoutId = context.circularSequentialID();
    } while (!context.addTimeout(timer->m_timeoutId, timer.get()));

    InspectorInstrumentation::didInstallTimer(context, timer->m_timeoutId, timeout, type == Type::SingleShot);

    // Keep track of nested timer installs.
    if (NestedTimersMap* nestedTimers = NestedTimersMap::instanceForContext(context))
        nestedTimers->add(timer->m_timeoutId, timer.get());
#if ENABLE(CONTENT_CHANGE_OBSERVER)
    if (RefPtr document = dynamicDowncast<Document>(context)) {
        document->contentChangeObserver().didInstallDOMTimer(timer.get(), timeout, type == Type::SingleShot);
        if (DeferDOMTimersForScope::isDeferring())
            document->domTimerHoldingTank().add(timer.get());
    }
#endif
    return timer->m_timeoutId;
}

void DOMTimer::removeById(ScriptExecutionContext& context, int timeoutId)
{
    // timeout IDs have to be positive, and 0 and -1 are unsafe to
    // even look up since they are the empty and deleted value
    // respectively
    if (timeoutId <= 0)
        return;

#if ENABLE(CONTENT_CHANGE_OBSERVER)
    if (RefPtr document = dynamicDowncast<Document>(context)) {
        if (RefPtr timer = document->findTimeout(timeoutId)) {
            document->contentChangeObserver().didRemoveDOMTimer(*timer);
            if (auto* holdingTank = document->domTimerHoldingTankIfExists())
                holdingTank->remove(*timer);
        }
    }
#endif

    if (NestedTimersMap* nestedTimers = NestedTimersMap::instanceForContext(context))
        nestedTimers->remove(timeoutId);

    InspectorInstrumentation::didRemoveTimer(context, timeoutId);

    if (RefPtr timer = context.takeTimeout(timeoutId)) {
        timer->clearImminentlyScheduledWorkScope();
        timer->m_timer = nullptr;
    }
}

inline bool DOMTimer::isDOMTimersThrottlingEnabled(const Document& document) const
{
    auto* page = document.page();
    if (!page)
        return true;
    return page->settings().domTimersThrottlingEnabled();
}

void DOMTimer::updateThrottlingStateIfNecessary(const DOMTimerFireState& fireState)
{
    RefPtr contextDocument = fireState.contextDocument();
    // We don't throttle timers in worker threads.
    if (!contextDocument)
        return;

    if (!isDOMTimersThrottlingEnabled(*contextDocument)) [[unlikely]] {
        if (m_throttleState == ShouldThrottle) {
            // Unthrottle the timer in case it was throttled before the setting was updated.
            LOG(DOMTimers, "%p - Unthrottling DOM timer because throttling was disabled via settings.", this);
            m_throttleState = ShouldNotThrottle;
            updateTimerIntervalIfNecessary();
        }
        return;
    }

    if (fireState.scriptMadeUserObservableChanges()) {
        if (m_throttleState != ShouldNotThrottle) {
            m_throttleState = ShouldNotThrottle;
            updateTimerIntervalIfNecessary();
        }
    } else if (fireState.scriptMadeNonUserObservableChanges()) {
        if (m_throttleState != ShouldThrottle) {
            m_throttleState = ShouldThrottle;
            updateTimerIntervalIfNecessary();
        }
    }
}

void DOMTimer::scriptDidInteractWithPlugin()
{
    if (!DOMTimerFireState::current)
        return;

    DOMTimerFireState::current->setScriptMadeUserObservableChanges();
}

void DOMTimer::fired()
{
    // Retain this - if the timer is cancelled while this function is on the stack (implicitly and always
    // for one-shot timers, or if removeById is called on itself from within an interval timer fire) then
    // wait unit the end of this function to delete DOMTimer.
    Ref protectedThis { *this };

    ASSERT(scriptExecutionContext());
    Ref context = *scriptExecutionContext();

#if PLATFORM(IOS_FAMILY)
    if (RefPtr document = dynamicDowncast<Document>(context); document && m_oneShot) {
        if (auto* holdingTank = document->domTimerHoldingTankIfExists(); holdingTank && holdingTank->contains(*this)) {
            m_timer = document->checkedEventLoop()->scheduleTask(0_s, TaskSource::Timer, [weakThis = WeakPtr { *this }] {
                if (RefPtr protectedThis = weakThis.get())
                    protectedThis->fired();
            });
            return;
        }
    }
#endif

    DOMTimerFireState fireState(context, std::min(m_nestingLevel + 1, maxTimerNestingLevel));

    if (m_userGestureTokenToForward && m_userGestureTokenToForward->hasExpired(UserGestureToken::maximumIntervalForUserGestureForwarding))
        m_userGestureTokenToForward = nullptr;

    ASSERT(!context->activeDOMObjectsAreSuspended());
    UserGestureIndicator gestureIndicator(m_userGestureTokenToForward);
    // Only the first execution of a multi-shot timer should get an affirmative user gesture indicator.
    m_userGestureTokenToForward = nullptr;

    InspectorInstrumentation::willFireTimer(context, m_timeoutId, m_oneShot);

    // Simple case for non-one-shot timers.
    if (!m_oneShot) {
        if (m_nestingLevel < maxTimerNestingLevel) {
            m_nestingLevel++;
            m_hasReachedMaxNestingLevel = m_nestingLevel >= maxTimerNestingLevelForRepeatingTimers;
            context->checkedEventLoop()->setTimerHasReachedMaxNestingLevel(m_timer, m_hasReachedMaxNestingLevel);
            updateTimerIntervalIfNecessary();
        }

        m_action(context);

        InspectorInstrumentation::didFireTimer(context, m_timeoutId, m_oneShot);

        updateThrottlingStateIfNecessary(fireState);

        clearImminentlyScheduledWorkScope();
        return;
    }

    context->takeTimeout(m_timeoutId);

    // Keep track nested timer installs.
    NestedTimersMap* nestedTimers = NestedTimersMap::instanceForContext(context);
    if (nestedTimers)
        nestedTimers->startTracking();

#if ENABLE(CONTENT_CHANGE_OBSERVER)
    ContentChangeObserver::DOMTimerScope observingScope(dynamicDowncast<Document>(context.get()), *this);
#endif
    m_action(context);

    InspectorInstrumentation::didFireTimer(context, m_timeoutId, m_oneShot);

    // Check if we should throttle nested single-shot timers.
    if (nestedTimers) {
        for (auto& idAndTimer : *nestedTimers) {
            Ref timer = idAndTimer.value;
            if (timer->m_oneShot)
                timer->updateThrottlingStateIfNecessary(fireState);
        }
        nestedTimers->stopTracking();
    }

    clearImminentlyScheduledWorkScope();
}

void DOMTimer::stop()
{
    // Need to release JS objects potentially protected by ScheduledAction
    // because they can form circular references back to the ScriptExecutionContext
    // which will cause a memory leak.
    m_timer = nullptr;
    m_action = nullptr;

    clearImminentlyScheduledWorkScope();
}

void DOMTimer::updateTimerIntervalIfNecessary()
{
    ASSERT(m_nestingLevel <= maxTimerNestingLevel);

    if (!scriptExecutionContext())
        return;

    auto previousInterval = m_currentTimerInterval;
    m_currentTimerInterval = intervalClampedToMinimum();
    if (previousInterval == m_currentTimerInterval)
        return;

    Ref context = *scriptExecutionContext();
    if (m_oneShot) {
        LOG(DOMTimers, "%p - Updating DOMTimer's fire interval from %.2f ms to %.2f ms due to throttling.", this, previousInterval.milliseconds(), m_currentTimerInterval.milliseconds());
        context->checkedEventLoop()->adjustTimerNextFireTime(m_timer, m_currentTimerInterval - previousInterval);
    } else {
        LOG(DOMTimers, "%p - Updating DOMTimer's repeat interval from %.2f ms to %.2f ms due to throttling.", this, previousInterval.milliseconds(), m_currentTimerInterval.milliseconds());
        context->checkedEventLoop()->adjustTimerRepeatInterval(m_timer, m_currentTimerInterval - previousInterval);
    }
}

Seconds DOMTimer::intervalClampedToMinimum() const
{
    ASSERT(scriptExecutionContext());
    ASSERT(m_nestingLevel <= maxTimerNestingLevel);

    Seconds interval = std::max(m_oneShot ? minIntervalForOneShotTimers : minIntervalForRepeatingTimers, m_originalInterval);

    // Only apply throttling to repeating timers.
    if (m_nestingLevel < (m_oneShot ? maxTimerNestingLevelForOneShotTimers : maxTimerNestingLevelForRepeatingTimers))
        return interval;

    // Apply two throttles - the global (per Page) minimum, and also a per-timer throttle.
    interval = std::max(interval, scriptExecutionContext()->minimumDOMTimerInterval());
    if (m_throttleState == ShouldThrottle)
        interval = std::max(interval, minIntervalForNonUserObservableChangeTimers);
    return interval;
}

std::optional<MonotonicTime> ScriptExecutionContext::alignedFireTime(bool hasReachedMaxNestingLevel, MonotonicTime fireTime) const
{
    Seconds alignmentInterval = domTimerAlignmentInterval(hasReachedMaxNestingLevel);
    if (!alignmentInterval)
        return std::nullopt;
    
    static const double randomizedProportion = cryptographicallyRandomUnitInterval();

    // Force alignment to randomizedAlignment fraction of the way between alignemntIntervals, e.g.
    // if alignmentInterval is 10_ms and randomizedAlignment is 0.3 this will align to 3, 13, 23, ...
    Seconds randomizedOffset = alignmentInterval * randomizedProportion;
    MonotonicTime adjustedFireTime = fireTime - randomizedOffset;
    return adjustedFireTime - (adjustedFireTime % alignmentInterval) + alignmentInterval + randomizedOffset;
}

void DOMTimer::makeImminentlyScheduledWorkScopeIfPossible(ScriptExecutionContext& context)
{
    if (!m_oneShot || m_currentTimerInterval > 1_ms)
        return;

    RefPtr document = dynamicDowncast<Document>(context);
    if (!document)
        return;

    RefPtr page = document->page();
    if (!page)
        return;

    m_imminentlyScheduledWorkScope = page->opportunisticTaskScheduler().makeScheduledWorkScope();
}

void DOMTimer::clearImminentlyScheduledWorkScope()
{
    m_imminentlyScheduledWorkScope = nullptr;
}

} // namespace WebCore
