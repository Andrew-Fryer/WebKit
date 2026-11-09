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

#include "AbstractHeapLatticeSwiftBoundary.h"

namespace JSC { namespace DFG {

// AbstractHeapLattice.swift hard-codes the kind numbers and the hierarchy, so changing
// FOR_EACH_ABSTRACT_HEAP_KIND has to fail here (the encoding is checked in the header).
#define DFG_ABSTRACT_HEAP_COUNT_KIND(name) + 1
static_assert(0 FOR_EACH_ABSTRACT_HEAP_KIND(DFG_ABSTRACT_HEAP_COUNT_KIND) == 44);
#undef DFG_ABSTRACT_HEAP_COUNT_KIND
static_assert(InvalidAbstractHeap == 0 && World == 1 && Stack == 2 && Heap == 3);
static_assert(NamedProperties == 15 && IndexedProperties == 16);
static_assert(IndexedInt32Properties == 17 && TypedArrayProperties == 23 && HeapObjectCount == 24);
static_assert(DOMState == 40 && MiscFields == 42 && SideState == 43);

intptr_t AbstractHeap::isStrictSubtypeOfSwift(int64_t heap, int64_t other)
{
    return JSCSwift_AbstractHeapLattice_isStrictSubtypeOf(heap, other);
}

intptr_t AbstractHeap::overlapsSwift(int64_t heap, int64_t other)
{
    return JSCSwift_AbstractHeapLattice_overlaps(heap, other);
}

int64_t AbstractHeap::supertypeSwift(int64_t heap)
{
    return JSCSwift_AbstractHeapLattice_supertype(heap);
}

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

