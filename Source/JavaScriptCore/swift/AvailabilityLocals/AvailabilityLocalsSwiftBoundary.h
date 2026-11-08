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

// The m_locals loops of JSC::DFG::AvailabilityMap's operator==, merge() and clear(), implemented in Swift
// (AvailabilityLocals.swift). An Operands<Availability> is passed as a pointer to its first element (null only if
// it is empty) and its element count; an Availability is read as two 64-bit words, m_node then m_flushedAt (the
// layout is static_asserted in DFGAvailability.h and DFGFlushedAt.h). Node pointers are never dereferenced.
// The symbols are hidden in JavaScriptCore: only dfg/DFGAvailabilityMap.cpp calls them.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Whether a[i] == b[i] (Availability::operator==, which ignores FlushedAt's padding) for every i < count.
__attribute__((pure, nothrow, warn_unused_result)) bool JSCSwift_AvailabilityLocals_equal(const uint64_t* a, const uint64_t* b, size_t count);

// target[i] = other[i].merge(target[i]) for every i < otherCount. Traps unless otherCount <= targetCount (C++ would
// write out of bounds). target and other must not overlap.
__attribute__((nothrow)) void JSCSwift_AvailabilityLocals_merge(uint64_t* target, size_t targetCount, const uint64_t* other, size_t otherCount);

// target[i] = Availability() (no node, FlushedAt(DeadFlush)) for every i < count.
__attribute__((nothrow)) void JSCSwift_AvailabilityLocals_fillWithDead(uint64_t* target, size_t count);

#ifdef __cplusplus
}
#endif
