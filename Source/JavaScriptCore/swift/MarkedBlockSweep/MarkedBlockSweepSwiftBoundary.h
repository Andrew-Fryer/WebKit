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

// MarkedBlock's per-cell sweep and stop-allocating loops implemented in Swift (MarkedBlockSweep.swift,
// MarkedBlockStopAllocating.swift). The symbols are hidden in JavaScriptCore: only heap/MarkedBlock.cpp calls
// them, from MarkedBlock::Handle::sweepWithSwiftKernel and MarkedBlock::Handle::stopAllocating.
//
// `payload` is a block's atoms from the handle's m_startAtom to the end of the block: payloadAtoms 16-byte atoms
// (at most 1024), 16-byte aligned. Swift writes only there and into the arrays passed to it.

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The loops of MarkedBlock::Handle::specializedSweep after setBits() and the unlock, for a sweep that destroys
// cells: destroy (JSString or JSRopeString destructor, then zap(Destruction)) every dead cell that is not zapped
// yet, and with SweepToFreeList thread the runs of dead cells into a free list.
//   liveWords: the live-cell bitmap (marks, or marks | newlyAllocated) as BitSet<atomsPerBlock> words, indexed
//     by absolute atom number; null for the all-dead (IsEmpty) path.
//   flags: bit 0 SweepToFreeList; bits 2-3 the destructor: 1 JSString, 2 JSRopeString.
//   derefs: exactly 1024 entries (one per atom of a block; Swift traps on any other capacity). Receives, in cell
//     order, the StringImpl* of every ~String run on a non-null string; the caller derefs them.
//   *outHeadOffset: payload byte offset of the free list's head FreeCell, or -1 for none (always -1 for SweepOnly).
//   *outFreedBytes: the free list's size in bytes, for FreeList::initialize.
// Returns the number of entries written to derefs.
__attribute__((nothrow)) size_t JSCSwift_MarkedBlockSweep_sweep(uint8_t* payload, size_t payloadAtoms, const uint64_t* liveWords,
    size_t liveWordCount, uint32_t startAtom, uint32_t atomsPerCell, uint64_t secret, uint32_t flags, uint64_t* derefs,
    size_t derefCapacity, int64_t* outHeadOffset, uint64_t* outFreedBytes);

// The Resumable path of MarkedBlock::Handle::stopAllocating: newlyAllocated = every cell, minus the cells still
// on the free list, which are zapped with HeapCell::StopAllocating if needsZapping.
//   newlyAllocatedWords: the block header's m_newlyAllocated words (wordCount * 64 >= endAtom).
//   intervalStart, intervalEnd, nextInterval: the FreeList's raw state as payload byte offsets (pointer - payload);
//     an odd nextInterval is the sentinel.
__attribute__((nothrow)) void JSCSwift_MarkedBlockSweep_stopAllocating(uint8_t* payload, size_t payloadAtoms,
    uint64_t* newlyAllocatedWords, size_t wordCount, uint32_t startAtom, uint32_t atomsPerCell, uint32_t endAtom,
    int64_t intervalStart, int64_t intervalEnd, int64_t nextInterval, uint64_t secret, bool needsZapping);

#ifdef __cplusplus
}
#endif
