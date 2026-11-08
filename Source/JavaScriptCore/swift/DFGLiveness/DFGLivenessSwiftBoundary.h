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

// The per-block work of JSC::DFG::LivenessAnalysisPhase<DenseTailStorage>, implemented in Swift (DFGLiveness.swift,
// which also documents the action stream format). Every array is owned by the C++ caller; a pointer may be null
// only when its count is 0, and no two arrays overlap. Out-of-range indices trap. The symbols are hidden in
// JavaScriptCore: only dfg/DFGLivenessAnalysisPhase.cpp calls them.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// processBlock() up to "Update live at head": clears the workset (worksetMap/worksetValues, numIndices entries
// each; the map's initial contents may be anything), adds the set bits of the block's live-at-tail row (wordsPerSet
// words) in ascending order, then runs the block's action stream. kill[k] receives 1 (DoesKill) or 0 (DoesNotKill)
// for the k-th use in the stream. Returns the workset size.
__attribute__((nothrow, warn_unused_result)) size_t JSCSwift_DFGLiveness_processBlock(const uint32_t* actions, size_t actionCount,
    const uint64_t* tailRow, size_t wordsPerSet,
    uint32_t* worksetMap, uint32_t* worksetValues, size_t numIndices,
    uint8_t* kill, size_t killCount);

// processBlock()'s "Update live at head", called right after JSCSwift_DFGLiveness_processBlock returned
// worksetCount for a block whose live-at-head list has a different size. head has headCapacity slots (at least
// oldHeadCount + worksetCount), the first oldHeadCount being the block's current list; the new entries are appended
// and *newHeadCount receives the new length. Each new entry is set in the live-at-tail row (of the
// tailMatrixWords-word matrix, wordsPerSet words per block) of each predecessor; a predecessor gaining a bit whose
// bit in dirtyWords is clear gets it set and is appended to worklist (worklistCount entries in use, capacity
// worklistCapacity). Returns the new worklist length.
__attribute__((nothrow, warn_unused_result)) size_t JSCSwift_DFGLiveness_updateHead(uint32_t* head, size_t oldHeadCount, size_t headCapacity,
    uint32_t* worksetMap, uint32_t* worksetValues, size_t worksetCount, size_t numIndices,
    uint64_t* tailMatrix, size_t tailMatrixWords, size_t wordsPerSet,
    const uint32_t* predecessors, size_t predecessorCount,
    uint64_t* dirtyWords, size_t dirtyWordCount,
    uint32_t* worklist, size_t worklistCapacity, size_t worklistCount,
    size_t* newHeadCount);

#ifdef __cplusplus
}
#endif
