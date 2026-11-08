/*
 * Copyright (C) 2013, 2015 Apple Inc. All rights reserved.
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
#include "DFGAbstractHeap.h"

#if ENABLE(DFG_JIT)

#if USE(SWIFT_KERNELS)
#include "AbstractHeapLatticeSwiftBoundary.h"
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

namespace JSC { namespace DFG {

#if USE(SWIFT_KERNELS)

// AbstractHeapLattice.swift hard-codes the kind numbers and the hierarchy, so changing
// FOR_EACH_ABSTRACT_HEAP_KIND or superKind() has to fail here (the encoding is checked in the header).
#define DFG_ABSTRACT_HEAP_COUNT_KIND(name) + 1
static_assert(0 FOR_EACH_ABSTRACT_HEAP_KIND(DFG_ABSTRACT_HEAP_COUNT_KIND) == 44);
#undef DFG_ABSTRACT_HEAP_COUNT_KIND
static_assert(InvalidAbstractHeap == 0 && World == 1 && Stack == 2 && Heap == 3);
static_assert(NamedProperties == 15 && IndexedProperties == 16);
static_assert(IndexedInt32Properties == 17 && TypedArrayProperties == 23 && HeapObjectCount == 24);
static_assert(DOMState == 40 && MiscFields == 42 && SideState == 43);

// AbstractHeapLattice.superKind, for every 7-bit kind.
static constexpr unsigned swiftSuperKind(unsigned kind)
{
    if (kind - 17 < 7)
        return IndexedProperties;
    if (kind - 4 < 39)
        return Heap;
    if (kind == Stack || kind == Heap || kind == SideState)
        return World;
    return InvalidAbstractHeap;
}

static_assert([] {
    for (unsigned kind = 0; kind < 128; ++kind) {
        if (AbstractHeap::superKind(static_cast<AbstractHeapKind>(kind)) != swiftSuperKind(kind))
            return false;
    }
    return true;
}(), "AbstractHeapLattice.swift's hierarchy must match AbstractHeap::superKind()");

#if ENABLE(SWIFT_KERNELS_VALIDATION)
// A query is a few nanoseconds of work, so with validation off each one should stay a load, a compare and a tail
// call to Swift, with no stack frame: swiftKernelsValidationEnabled() is cached here and the checks are out of line.
enum class AbstractHeapSwiftValidation : uint8_t { Unknown, Off, On };
static std::atomic<AbstractHeapSwiftValidation> abstractHeapSwiftValidation;

static ALWAYS_INLINE bool abstractHeapSwiftValidationMaybeOn()
{
    return abstractHeapSwiftValidation.load(std::memory_order_relaxed) != AbstractHeapSwiftValidation::Off;
}

static bool abstractHeapSwiftValidationOn()
{
    auto state = abstractHeapSwiftValidation.load(std::memory_order_relaxed);
    if (state == AbstractHeapSwiftValidation::Unknown) {
        state = swiftKernelsValidationEnabled() ? AbstractHeapSwiftValidation::On : AbstractHeapSwiftValidation::Off;
        abstractHeapSwiftValidation.store(state, std::memory_order_relaxed);
    }
    return state == AbstractHeapSwiftValidation::On;
}

static NEVER_INLINE intptr_t isStrictSubtypeOfSwiftValidating(int64_t heap, int64_t other)
{
    intptr_t result = JSCSwift_AbstractHeapLattice_isStrictSubtypeOf(heap, other);
    if (abstractHeapSwiftValidationOn())
        RELEASE_ASSERT(!!result == std::bit_cast<AbstractHeap>(heap).isStrictSubtypeOfCxx(std::bit_cast<AbstractHeap>(other)));
    return result;
}

static NEVER_INLINE intptr_t overlapsSwiftValidating(int64_t heap, int64_t other)
{
    intptr_t result = JSCSwift_AbstractHeapLattice_overlaps(heap, other);
    if (abstractHeapSwiftValidationOn())
        RELEASE_ASSERT(!!result == std::bit_cast<AbstractHeap>(heap).overlapsCxx(std::bit_cast<AbstractHeap>(other)));
    return result;
}

static NEVER_INLINE int64_t supertypeSwiftValidating(int64_t heap)
{
    int64_t result = JSCSwift_AbstractHeapLattice_supertype(heap);
    if (abstractHeapSwiftValidationOn())
        RELEASE_ASSERT(std::bit_cast<AbstractHeap>(result) == std::bit_cast<AbstractHeap>(heap).supertypeCxx());
    return result;
}
#endif

intptr_t AbstractHeap::isStrictSubtypeOfSwift(int64_t heap, int64_t other)
{
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (abstractHeapSwiftValidationMaybeOn()) [[unlikely]]
        return isStrictSubtypeOfSwiftValidating(heap, other);
#endif
    return JSCSwift_AbstractHeapLattice_isStrictSubtypeOf(heap, other);
}

intptr_t AbstractHeap::overlapsSwift(int64_t heap, int64_t other)
{
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (abstractHeapSwiftValidationMaybeOn()) [[unlikely]]
        return overlapsSwiftValidating(heap, other);
#endif
    return JSCSwift_AbstractHeapLattice_overlaps(heap, other);
}

int64_t AbstractHeap::supertypeSwift(int64_t heap)
{
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (abstractHeapSwiftValidationMaybeOn()) [[unlikely]]
        return supertypeSwiftValidating(heap);
#endif
    return JSCSwift_AbstractHeapLattice_supertype(heap);
}

#endif // USE(SWIFT_KERNELS)

void AbstractHeap::Payload::dump(PrintStream& out) const
{
    if (isTop())
        out.print("TOP");
    else
        out.print(value());
}

void AbstractHeap::Payload::dumpAsOperand(PrintStream& out) const
{
    if (isTop())
        out.print("TOP");
    else
        out.print(Operand::fromBits(value()));
}

void AbstractHeap::dump(PrintStream& out) const
{
    out.print(kind());
    if (kind() == InvalidAbstractHeap || kind() == World || kind() == Heap || payload().isTop())
        return;
    if (kind() == DOMState) {
        out.print("(", DOMJIT::HeapRange::fromRaw(payload().value32()), ")");
        return;
    }
    if (kind() == Stack) {
        out.print("(");
        payload().dumpAsOperand(out);
        out.print(")");
        return;
    }

    out.print("(", payload(), ")");
}

} } // namespace JSC::DFG

namespace WTF {

using namespace JSC::DFG;

void printInternal(PrintStream& out, AbstractHeapKind kind)
{
    switch (kind) {
#define ABSTRACT_HEAP_DUMP(name)            \
    case AbstractHeapKind::name:            \
        out.print(#name);                   \
        return;
    FOR_EACH_ABSTRACT_HEAP_KIND(ABSTRACT_HEAP_DUMP)
#undef ABSTRACT_HEAP_DUMP
    }
    RELEASE_ASSERT_NOT_REACHED();
}

} // namespace WTF

#endif // ENABLE(DFG_JIT)

