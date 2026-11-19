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
// (AbstractValue::merge). Returns whether any value changed. A merge that needs a structure list (a fat set on either
// side, or two different single structures) leaves destination[i] untouched and appends i to slowIndices instead, for
// the caller to merge with JSCSwift_DFGCFAMerge_mergeSlowValue; except that a copy into a clear destination (*this =
// other) that allocates or frees a list merges words 1-3 and appends i | 1 << 31, for the caller to copy source[i]'s
// StructureAbstractValue. *slowCount is the number of entries. Traps unless count < 2^31 and
// count <= destinationCount, sourceCount and slowCapacity (and destinationNodeCount when checkNodes).
// destination must not overlap source.
__attribute__((nothrow, warn_unused_result)) bool JSCSwift_DFGCFAMerge_mergeValues(uint64_t* destination, size_t destinationCount, const uint64_t* source, size_t sourceCount, const uint64_t* destinationNodes, size_t destinationNodeCount, bool checkNodes, size_t count, uint32_t* slowIndices, size_t slowCapacity, size_t* slowCount);

// destination.merge(source) for one AbstractValue (four words each) whose merge needs a structure list (see
// DFGCFAMerge.swift). destinationList/sourceList: the fat sets' elements (null and 0 when thin). list: scratch for the
// result's elements, at least destinationLength + sourceLength + 2 words. result[0] = kind | capacity << 32,
// result[1] = word, result[2] = length, describing the destination's new StructureAbstractValue:
//   kind 0: unchanged. 1: free its list if fat; its word becomes `word` (thin or TOP).
//   2: list[0..<length] is appended to its fat list, which has room for them. 3: free its list if fat; a new list
//   of `capacity` holds list[0..<length]. For kinds 2 and 3 the clobbered flag becomes `word` (0 or 2).
//   4: it becomes a copy of the source's StructureAbstractValue (operator=).
__attribute__((nothrow, warn_unused_result)) bool JSCSwift_DFGCFAMerge_mergeSlowValue(uint64_t* destination, const uint64_t* source,
    const uint64_t* destinationList, size_t destinationLength, uint32_t destinationCapacity, const uint64_t* sourceList, size_t sourceLength,
    uint64_t* list, size_t listCapacity, uint64_t* result);

#ifdef __cplusplus
}
#endif
