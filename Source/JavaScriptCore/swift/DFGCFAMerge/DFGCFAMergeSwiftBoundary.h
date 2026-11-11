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

// The per-variable loops of JSC::DFG::InPlaceAbstractState::merge(from, to), implemented in Swift (DFGCFAMerge.swift).
// An Operands<AbstractValue> is passed as a pointer to its first element (null only if it is empty) and its element
// count; an AbstractValue is read and written as four 64-bit words (m_structure, m_type, m_arrayModes and
// m_effectEpoch, m_value; static_asserted in DFGAbstractValue.h and DFGStructureAbstractValue.h). Node pointers are
// only tested for null and structure pointers are never dereferenced. The symbol is hidden in JavaScriptCore: only
// dfg/DFGInPlaceAbstractState.cpp calls it.

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// For every i < count, and when checkNodes only where destinationNodes[i] is not null: destination[i].merge(source[i])
// (AbstractValue::merge). Returns whether any value changed. A merge that needs a structure list to be allocated or
// freed (a fat set on either side, or two different single structures) leaves destination[i] untouched and appends i
// to slowIndices instead; *slowCount is the number of such indices, which the caller must merge itself. Traps unless
// count <= destinationCount, sourceCount and slowCapacity (and destinationNodeCount when checkNodes). destination
// must not overlap source.
__attribute__((nothrow, warn_unused_result)) bool JSCSwift_DFGCFAMerge_mergeValues(uint64_t* destination, size_t destinationCount, const uint64_t* source, size_t sourceCount, const uint64_t* destinationNodes, size_t destinationNodeCount, bool checkNodes, size_t count, uint32_t* slowIndices, size_t slowCapacity, size_t* slowCount);

#ifdef __cplusplus
}
#endif
