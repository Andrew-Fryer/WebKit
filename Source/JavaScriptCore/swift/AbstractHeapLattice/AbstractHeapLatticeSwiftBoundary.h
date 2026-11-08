/*
 * Copyright (C) 2026 Apple Inc. All rights reserved.
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

// DFG::AbstractHeap lattice queries implemented in Swift (AbstractHeapLattice.swift). An AbstractHeap is
// passed as its raw 64-bit encoding (AbstractHeap::m_value). The symbols are hidden in JavaScriptCore:
// only DFGAbstractHeap.cpp calls them.

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Pure functions of their arguments: C++ may CSE, hoist and drop calls. They return 0 or 1. They trap only
// where the C++ supertype walk would never terminate (heaps of InvalidAbstractHeap or unknown kinds).
__attribute__((const, nothrow, warn_unused_result)) intptr_t JSCSwift_AbstractHeapLattice_isStrictSubtypeOf(int64_t heap, int64_t other);
__attribute__((const, nothrow, warn_unused_result)) intptr_t JSCSwift_AbstractHeapLattice_overlaps(int64_t heap, int64_t other);

// The encoding of AbstractHeap::supertype() (0, the encoding of AbstractHeap(), for World).
__attribute__((const, nothrow, warn_unused_result)) int64_t JSCSwift_AbstractHeapLattice_supertype(int64_t heap);

#ifdef __cplusplus
}
#endif
