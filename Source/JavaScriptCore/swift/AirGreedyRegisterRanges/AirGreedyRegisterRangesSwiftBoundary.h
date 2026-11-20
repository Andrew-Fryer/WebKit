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

// The register ranges of JSC::B3::Air's greedy register allocator (which Tmp occupies each register at each Point),
// a data structure owned by Swift (AirGreedyRegisterRanges.swift: a port of the WTF::IntervalSet B+ tree and of
// Greedy::RegisterRange). C++ owns only the storage: an arena of `arenaWords` 64-bit words, 64-byte aligned, zero-filled
// when created and when grown (the old words copied), at least JSCSwift_AirGreedyRegisterRanges_minimumWords long.
// Swift links its nodes by index, so the arena may move between calls. An operation that needs more words changes
// nothing past what it reports and returns the number of words to grow the arena to (0 means done).
//
// A register is its Reg::index() (below JSCSwift_AirGreedyRegisterRanges_maxRegisters); a Tmp its internalValue(),
// stored and returned, and interpreted only by cheapestEviction (against the register Tmp bounds it is given). A LiveRange's intervals are passed as a pointer to 8-byte aligned words
// begin | end << 32 (its WTF::Range<uint32_t> elements, on a little-endian CPU; null only when there are none) and
// their number. The symbols are hidden in JavaScriptCore:
// only b3/air/AirAllocateRegistersByGreedy.cpp calls them.

#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    JSCSwift_AirGreedyRegisterRanges_maxRegisters = 64,
    // The arena is made of nodes of this many words (and a header of 17 nodes).
    JSCSwift_AirGreedyRegisterRanges_nodeWords = 32,
    JSCSwift_AirGreedyRegisterRanges_minimumWords = 17 * 32,
};

// GreedyAllocator::tryAllocate's register loop: tries `preferred` (a register, or -1 for none), then each register of
// order[0 ..< orderCount] other than it; the first whose allocations (and, if checkHigh64, high-64-bit clobbers) overlap
// none of the intervals gets them added for `tmp` (RegisterRange::add) and is returned; -1 if there is none.
// *progress is the number of intervals added: if it is less than intervalCount, the caller must finish with
// JSCSwift_AirGreedyRegisterRanges_add after growing the arena.
__attribute__((nothrow, warn_unused_result)) int32_t JSCSwift_AirGreedyRegisterRanges_tryAllocate(uint64_t* arena, size_t arenaWords,
    const uint64_t* intervals, size_t intervalCount, int32_t preferred, const uint8_t* order, size_t orderCount, bool checkHigh64, uint32_t tmp, size_t* progress);

// RegisterRange::add: adds intervals[*progress ..< intervalCount] to the register's allocations for `tmp` (they must not
// overlap any; when *progress is 0 and the register has none, the set is built in one pass). Returns 0 when all are
// added, else the arena size it needs to continue; *progress is updated either way.
__attribute__((nothrow, warn_unused_result)) size_t JSCSwift_AirGreedyRegisterRanges_add(uint64_t* arena, size_t arenaWords, uint32_t reg, uint32_t tmp,
    const uint64_t* intervals, size_t intervalCount, size_t* progress);

// RegisterRange::evict: removes the intervals (which must all be present) from the register's allocations.
__attribute__((nothrow)) void JSCSwift_AirGreedyRegisterRanges_evict(uint64_t* arena, size_t arenaWords, uint32_t reg, const uint64_t* intervals, size_t intervalCount);

// RegisterRange::addClobberHigh64: adds [point, point + 1) for `tmp` (Tmp(reg)) to the register's high-64-bit clobbers.
// Returns 0, or (changing nothing) the arena size it needs.
__attribute__((nothrow, warn_unused_result)) size_t JSCSwift_AirGreedyRegisterRanges_addClobberHigh64(uint64_t* arena, size_t arenaWords, uint32_t reg, uint32_t point, uint32_t tmp);

// RegisterRange::forEachConflict's scan, resumable: writes the next (at least one, at most `capacity`, which must not be
// 0) entries of the register that overlap the intervals, as (tmp, begin, end) triples, in forEachConflict's order, and
// returns their number. sets: 1 scans the allocations, 3 the allocations then the high-64-bit clobbers, 2 only the
// clobbers. *cursor is 0 to start and is updated; it becomes JSCSwift_AirGreedyRegisterRanges_scanOver when the scan is
// over (only then can the result be 0). The arena is only read.
#define JSCSwift_AirGreedyRegisterRanges_scanOver (~(uint64_t)0)

// The first batch of JSCSwift_AirGreedyRegisterRanges_conflicts for each register of order[0 ..< orderCount] (Reg
// indices, at most 255), each from the start of its scan with room for `capacity` (1 ... 255) conflicts: register i's go
// to conflicts[3 * capacity * i ...], their number to counts[i] and its updated cursor to cursors[i].
__attribute__((nothrow)) void JSCSwift_AirGreedyRegisterRanges_firstConflicts(const uint64_t* arena, size_t arenaWords, const uint8_t* order, size_t orderCount,
    const uint64_t* intervals, size_t intervalCount, uint32_t sets, uint64_t* cursors, uint8_t* counts, uint32_t* conflicts, size_t capacity);
__attribute__((nothrow, warn_unused_result)) size_t JSCSwift_AirGreedyRegisterRanges_conflicts(const uint64_t* arena, size_t arenaWords, uint32_t reg,
    const uint64_t* intervals, size_t intervalCount, uint32_t sets, uint64_t* cursor, uint32_t* conflicts, size_t capacity);

// GreedyAllocator::tryEvict's cost loop. For each register order[i] (Reg indices) in turn, scans its conflicts with the
// intervals (`sets` as for JSCSwift_AirGreedyRegisterRanges_conflicts) in forEachConflict's order. A conflict with a
// register's own Tmp (Tmp::isReg(): its value as int32_t is in lastFPRTmp ... -1 or 1 ... lastGPRTmp, the values of
// Tmp(lastFPRegister) and Tmp(lastRegister)) is unspillable; for any other it calls cost(context, tmp, i), which returns
// the conflict's spill cost (a float's bits), or JSCSwift_AirGreedyRegisterRanges_skipConflict to skip it. A register's scan stops at an infinite (unspillable) cost
// or once its sum (saturating at FLT_MAX, which also absorbs FLT_MAX costs) is at least *minSpillCost; a register whose
// sum is below *minSpillCost makes it that sum. Returns the index in the order of the last such register, or -1.
// The arena is only read.
#define JSCSwift_AirGreedyRegisterRanges_skipConflict ((uint64_t)1 << 32)
typedef uint64_t (*JSCSwift_AirGreedyRegisterRanges_ConflictCostFunction)(void* context, uint32_t tmp, size_t orderIndex);
__attribute__((nothrow)) int32_t JSCSwift_AirGreedyRegisterRanges_cheapestEviction(const uint64_t* arena, size_t arenaWords, const uint8_t* order, size_t orderCount,
    const uint64_t* intervals, size_t intervalCount, uint32_t sets, int32_t lastGPRTmp, int32_t lastFPRTmp, float* minSpillCost,
    JSCSwift_AirGreedyRegisterRanges_ConflictCostFunction cost, void* context);

#ifdef __cplusplus
}
#endif
