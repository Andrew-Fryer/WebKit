/*
 * Copyright (C) 2016-2019 Apple Inc. All rights reserved.
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

#pragma once

#include <wtf/MathExtras.h>
#include <wtf/Noncopyable.h>
#include <wtf/PrintStream.h>

#include "MarkedBlock.h"
#include "wtf/StringPrintStream.h"

WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN

namespace JSC {

class HeapCell;

class FreeList {
public:
    FreeList(unsigned cellSize);
    ~FreeList();
    
    void clear();
    
    JS_EXPORT_PRIVATE void initialize(MarkedBlock::Handle* block, const WTF::BitSet<MarkedBlock::atomsPerBlock>& free, unsigned startIndex, unsigned bytes);
    JS_EXPORT_PRIVATE void initializeEmpty(MarkedBlock::Handle* block, char* intervalStart, char* intervalEnd);
    
    bool allocationWillFail() { return !allocationWillSucceed(); }
    bool allocationWillSucceed() { return m_intervalStart < m_intervalEnd || findNextInterval(); }

    unsigned currentAllocationIndex() {
        if (m_intervalStart >= m_intervalEnd)
            findNextInterval();
        return m_block->block().candidateAtomNumber(m_intervalStart);
    }
    
    template<typename Func>
    HeapCell* allocateWithCellSize(const Func& slowPath, size_t cellSize);
    
    WTF::BitSet<MarkedBlock::atomsPerBlock> live() { return m_live; }
    template<typename Func>
    void forEachRemaining(const Func&);
    
    unsigned originalSize() const { return m_originalSize; }

    static constexpr ptrdiff_t offsetOfIntervalStart() { return OBJECT_OFFSETOF(FreeList, m_intervalStart); }
    static constexpr ptrdiff_t offsetOfIntervalEnd() { return OBJECT_OFFSETOF(FreeList, m_intervalEnd); }
    static constexpr ptrdiff_t offsetOfOriginalSize() { return OBJECT_OFFSETOF(FreeList, m_originalSize); }
    static constexpr ptrdiff_t offsetOfCellSize() { return OBJECT_OFFSETOF(FreeList, m_cellSize); }
    
    JS_EXPORT_PRIVATE void dump(PrintStream&) const;

    unsigned cellSize() const { return m_cellSize; }
    unsigned atomsPerCell() const {
        ASSERT(m_cellSize % MarkedBlock::atomSize == 0);
        static_assert(MarkedBlock::atomSize);
        return m_cellSize / MarkedBlock::atomSize;
    }
    
private:
    template<bool value> // which value to find
    ALWAYS_INLINE bool findFreeCellFast() {
        unsigned atomsPerCell = m_cellSize >> 4;
        static_assert(MarkedBlock::atomsPerBlock == 1024);
        uint64_t* bits = std::bit_cast<uint64_t*>(&m_live);
        ASSERT(bits == &m_live.storage()[0]);
        // unsigned startIndexOriginal = m_startIndex;
        ASSERT((uint32_t*)bits == std::bit_cast<uint32_t*>(&m_live));
        // WTFLogAlways("afryer_findFreeCell %u : %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx\n", m_startIndex, bits[0], bits[1], bits[2], bits[3], bits[4], bits[5], bits[6], bits[7], bits[8], bits[9], bits[10], bits[11], bits[12], bits[13], bits[14], bits[15]);
        uint64_t startIndex = m_startIndex;
        while (startIndex < MarkedBlock::atomsPerBlock) {
            unsigned startIndexWordIndex = startIndex >> 6;
            unsigned startIndexBitIndex = startIndex & 0x3F;
            uint64_t bitmask = m_bitmask << startIndexBitIndex;
            uint64_t word = bits[startIndexWordIndex];
            uint64_t searchResults = (value ? word : ~word) & bitmask; // has a 1 on each bit we're looking for
            if (searchResults) {
                // find in word
                // WTFLogAlways("afryer_findFreeCell_ctzll %d %llx %llx %llx %llx %llu\n", value, word, m_bitmask, bitmask, searchResults, (uint64_t)__builtin_ctzll(searchResults));
                ASSERT(__builtin_ctzll(searchResults) >= (int)startIndexBitIndex);
                startIndex += __builtin_ctzll(searchResults) - startIndexBitIndex;
                // WTFLogAlways("afryer_findFreeCell %d %u %u %u %llx\n", value, startIndexOriginal, startIndex, atomsPerCell, bitmask);
                m_startIndex = startIndex;
                return true;
            }
            // m_startIndex += atomsPerCell; // slow
            unsigned skipSize = ((atomsPerCell + (64 - startIndexBitIndex) - 1) / atomsPerCell) * atomsPerCell;
            // WTFLogAlways("afryer_findFreeCell_skip %u %u %u\n", startIndex, atomsPerCell, skipSize);
            startIndex += skipSize;
        }
        m_startIndex = startIndex;
        return false;
    }

    ALWAYS_INLINE bool findNextIntervalFast() {
        if (m_startIndex >= MarkedBlock::atomsPerBlock)
            return false;
        ASSERT(m_intervalStart >= m_intervalEnd); // you should only advance to the next interval it the current interval is depleted
        if (findFreeCellFast<false>()) {
            m_intervalStart = (char*)m_block->atomAt(m_startIndex);
            findFreeCellFast<true>();
            ASSERT(m_startIndex <= 1024);
            m_intervalEnd = (char*)m_block->atomAt(m_startIndex);
            return true;
        }
        return false;
    }

    ALWAYS_INLINE bool findNextInterval() {
        return findNextIntervalFast();
        // // Todo: optimize this using word iteration and bitmask
        // ASSERT(m_block);
        // unsigned stride = cellSize >> 4; // cellSize / MarkedBlock::atomSize
        // for (unsigned i = 0; i < MarkedBlock::atomsPerBlock; i += stride) {
        //     if (!m_live.get(i)) {
        //         unsigned j;
        //         for (j = i + 1; j < MarkedBlock::atomsPerBlock; j += stride) {
        //             if (m_live.get(j))
        //                 break;
        //         }
        //         m_startIndex = i;
        //         m_intervalStart = (char*)m_block->atomAt(m_startIndex);
        //         // m_endIndex = j;
        //         m_intervalEnd = (char*)m_block->atomAt(j);
        //         return true;
        //     }
        // }
        // return false;
    }
    
    char* m_intervalStart { nullptr };
    char* m_intervalEnd { nullptr };
    unsigned m_cellSize { 0 };

    MarkedBlock::Handle* m_block { nullptr };
    uint64_t m_bitmask { 0 };
    unsigned m_startIndex { MarkedBlock::atomsPerBlock }; // could be just uin16_t // FreeList should fail allocation until it is initialized

    WTF::BitSet<MarkedBlock::atomsPerBlock> m_live;
    unsigned m_originalSize { 0 };
};

} // namespace JSC

WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
