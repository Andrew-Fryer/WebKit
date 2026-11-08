/*
 * Copyright (C) 2013-2017 Apple Inc. All rights reserved.
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
#include "DFGLivenessAnalysisPhase.h"

#if ENABLE(DFG_JIT)

#include "DFGFlowIndexing.h"
#include "DFGGraph.h"
#include "DFGPhase.h"
#include "JSCJSValueInlines.h"
#include <wtf/BitVector.h>
#include <wtf/IndexMap.h>
#include <wtf/IndexSparseSet.h>
#include <wtf/SparseBitVector.h>

#if USE(SWIFT_KERNELS)
#include "DFGLivenessSwiftBoundary.h"
#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif
#endif

namespace JSC { namespace DFG {

namespace {

using Workset = IndexSparseSet<unsigned, DefaultIndexSparseSetTraits<unsigned>, UnsafeVectorOverflow>;

class DenseTailStorage {
public:
    class Row {
    public:
        explicit Row(std::span<uint64_t> words)
            : m_words(words)
        { }

        bool add(unsigned bit)
        {
            uint64_t& word = m_words[bit / 64];
            uint64_t mask = static_cast<uint64_t>(1) << (bit % 64);
            bool wasSet = word & mask;
            word |= mask;
            return !wasSet;
        }

        template<typename Func>
        void forEachSetBit(NOESCAPE const Func& func) const
        {
            WTF::forEachSetBit(std::span<const uint64_t> { m_words }, func);
        }

    private:
        std::span<uint64_t> m_words;
    };

    DenseTailStorage(Graph&, unsigned numBlocks, unsigned numIndices)
        : m_wordsPerSet((numIndices + 63) / 64)
        , m_store(static_cast<size_t>(numBlocks) * m_wordsPerSet)
    {
        std::ranges::fill(m_store, 0);
    }

    Row operator[](BlockIndex blockIndex)
    {
        return Row(m_store.mutableSpan().subspan(static_cast<size_t>(blockIndex) * m_wordsPerSet, m_wordsPerSet));
    }
    Row operator[](BasicBlock* block) { return (*this)[block->index()]; }

#if USE(SWIFT_KERNELS)
    // For LivenessAnalysisPhase::runSwift(): the whole matrix, numBlocks rows of wordsPerSet() words.
    std::span<uint64_t> words() LIFETIME_BOUND { return m_store.mutableSpan(); }
    size_t wordsPerSet() const { return m_wordsPerSet; }
    void clear() { std::ranges::fill(m_store, 0); }
#endif

private:
    size_t m_wordsPerSet { 0 };
    Vector<uint64_t> m_store;
};

class SparseTailStorage {
public:
    SparseTailStorage(Graph& graph, unsigned, unsigned)
        : m_map(graph.numBlocks())
    { }

    SparseBitVector<>& operator[](BlockIndex blockIndex) LIFETIME_BOUND { return m_map[blockIndex]; }
    SparseBitVector<>& operator[](BasicBlock* block) LIFETIME_BOUND { return m_map[block]; }

private:
    IndexMap<BasicBlock*, SparseBitVector<>> m_map;
};

template<typename TailStorage>
class LivenessAnalysisPhase : public Phase {
public:
    LivenessAnalysisPhase(Graph& graph)
        : Phase(graph, "liveness analysis"_s)
        , m_dirtyBlocks(m_graph.numBlocks())
        , m_indexing(*m_graph.m_indexingCache)
        , m_liveAtHead(m_graph.numBlocks())
        , m_liveAtTail(m_graph, m_graph.numBlocks(), m_indexing.numIndices())
        , m_workset(m_indexing.numIndices())
    {
    }

    bool run()
    {
#if USE(SWIFT_KERNELS)
        if constexpr (std::is_same_v<TailStorage, DenseTailStorage>)
            return runSwift();
        else
            return runCxx();
#else
        return runCxx();
#endif
    }

    bool runCxx()
    {
        BlockIndex numBlock = m_graph.numBlocks();
        m_dirtyBlocks.ensureSize(numBlock);
        m_worklist.reserveInitialCapacity(numBlock);
        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            if (m_graph.block(blockIndex)) {
                m_dirtyBlocks.quickSet(blockIndex);
                m_worklist.append(blockIndex);
            }
        }

        // Fixpoint until we do not add any new live values at tail.
        while (!m_worklist.isEmpty()) {
            BlockIndex blockIndex = m_worklist.takeLast();
            bool cleared = m_dirtyBlocks.quickClear(blockIndex);
            ASSERT_UNUSED(cleared, cleared);
            processBlock(blockIndex);
        }

        // Update the per-block node list for live and tail.
        for (BlockIndex blockIndex = numBlock; blockIndex--;) {
            BasicBlock* block = m_graph.block(blockIndex);
            if (!block)
                continue;

            {
                block->ssa->liveAtHead = m_liveAtHead[blockIndex].map([this](auto index) {
                    return m_indexing.nodeProjection(index);
                });
            }
            {
                auto& liveAtTail = block->ssa->liveAtTail;
                liveAtTail.shrink(0);
                m_liveAtTail[blockIndex].forEachSetBit([&](size_t index) {
                    liveAtTail.append(m_indexing.nodeProjection(static_cast<unsigned>(index)));
                });
            }
        }

        return true;
    }

private:
#if USE(SWIFT_KERNELS)
    // Graphs with fewer flow indices (about nodes + Phis) than this run the original C++. On FTL graphs captured
    // from JetStream 3 the Swift path is slower below about 1500 nodes (flattening the graph costs about as much as
    // one pass of the original, and small graphs are walked only about 1.5 times) and 10-13% faster above 2048.
    static constexpr unsigned swiftMinimumIndexCount = 2048;

    // The DenseTailStorage instantiation of run(), with processBlock() in Swift (DFGLiveness.swift):
    // 1. One walk over the graph flattens every block into an action stream (nodes in reverse order, see
    //    DFGLiveness.swift), the Edges of its uses (in stream order) and its predecessors' indices.
    // 2. The original worklist loop, calling Swift for processBlock() and for its live-at-head update. It uses
    //    m_liveAtHead and m_liveAtTail; the workset, the dirty bits and the worklist are scratch arrays here.
    // 3. Each Edge gets the kill status computed by the last processing of its block, in the order of those last
    //    processings (so the last write wins as in the original even if an Edge were reached from two blocks).
    // 4. The original output loop, verbatim.
    bool runSwift()
    {
        static constexpr uint32_t phiTag = 1u << 30;

        unsigned numIndices = m_indexing.numIndices();
        bool takeSwiftPath = numIndices >= swiftMinimumIndexCount;
#if ENABLE(SWIFT_KERNELS_VALIDATION)
        // With validation on, every graph takes the Swift path (and is checked against runCxx()), for coverage.
        takeSwiftPath |= swiftKernelsValidationEnabled();
#endif
        if (!takeSwiftPath)
            return runCxx();

        BlockIndex numBlock = m_graph.numBlocks();
        size_t wordsPerSet = m_liveAtTail.wordsPerSet();

        size_t nodeCount = 0;
        size_t predecessorCount = 0;
        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            if (BasicBlock* block = m_graph.block(blockIndex)) {
                nodeCount += block->size();
                predecessorCount += block->predecessors.size();
            }
        }

        // 1. Flatten. A node takes at most 5 action words and 3 use Edges, except a varargs node with more than 3
        // children, whose excess comes out of `extra` (grown if the varargs ranges overlap).
        size_t extra = m_graph.m_varArgChildren.size();
        Vector<uint32_t> actions;
        actions.grow(5 * nodeCount + extra);
        Vector<Edge*> useEdges;
        useEdges.grow(3 * nodeCount + extra);
        Vector<uint32_t, 64> predecessors;
        predecessors.grow(predecessorCount);
        Vector<size_t, 64> actionBegin;
        actionBegin.grow(numBlock + 1);
        Vector<size_t, 64> useBegin;
        useBegin.grow(numBlock + 1);
        Vector<size_t, 64> predecessorBegin;
        predecessorBegin.grow(numBlock + 1);

        std::span<uint32_t> actionWords = actions.mutableSpan();
        std::span<Edge*> edges = useEdges.mutableSpan();
        size_t actionCount = 0;
        size_t useCount = 0;
        size_t predecessorIndex = 0;
        size_t excess = 0;
        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            actionBegin[blockIndex] = actionCount;
            useBegin[blockIndex] = useCount;
            predecessorBegin[blockIndex] = predecessorIndex;
            BasicBlock* block = m_graph.block(blockIndex);
            if (!block)
                continue;

            for (unsigned nodeIndex = block->size(); nodeIndex--;) {
                Node* node = block->at(nodeIndex);
                switch (node->op()) {
                case Upsilon: {
                    actionWords[actionCount] = 1;
                    actionWords[actionCount + 1] = m_indexing.shadowIndex(node->phi());
                    actionWords[actionCount + 2] = m_indexing.index(node->child1().node());
                    actionCount += 3;
                    edges[useCount++] = &node->child1();
                    break;
                }
                case Phi: {
                    actionWords[actionCount] = phiTag;
                    actionWords[actionCount + 1] = m_indexing.index(node);
                    actionWords[actionCount + 2] = m_indexing.shadowIndex(node);
                    actionCount += 3;
                    break;
                }
                default: {
                    if (node->flags() & NodeHasVarArgs) {
                        unsigned numChildren = node->numChildren();
                        RELEASE_ASSERT(numChildren < phiTag);
                        if (numChildren > 3) {
                            excess += numChildren - 3;
                            if (excess > extra) [[unlikely]] {
                                actions.grow(actions.size() + excess - extra);
                                useEdges.grow(useEdges.size() + excess - extra);
                                actionWords = actions.mutableSpan();
                                edges = useEdges.mutableSpan();
                                extra = excess;
                            }
                        }
                    }
                    size_t header = actionCount;
                    actionWords[actionCount + 1] = m_indexing.index(node);
                    actionCount += 2;
                    m_graph.doToChildren(node, [&](Edge& edge) {
                        actionWords[actionCount++] = m_indexing.index(edge.node());
                        edges[useCount++] = &edge;
                    });
                    actionWords[header] = static_cast<uint32_t>(actionCount - header - 2);
                    break;
                }
                }
            }

            for (BasicBlock* predecessor : block->predecessors)
                predecessors[predecessorIndex++] = predecessor->index();
        }
        actionBegin[numBlock] = actionCount;
        useBegin[numBlock] = useCount;
        predecessorBegin[numBlock] = predecessorIndex;

        // 2. Fixpoint until we do not add any new live values at tail.
        Vector<uint32_t> workset(FillWith { }, 2 * static_cast<size_t>(numIndices), 0);
        std::span<uint32_t> worksetMap = workset.mutableSpan().first(numIndices);
        std::span<uint32_t> worksetValues = workset.mutableSpan().subspan(numIndices);
        Vector<uint8_t> kill(FillWith { }, useCount, 0);
        Vector<uint64_t, 4> dirtyBlocks(FillWith { }, (numBlock + 63) / 64, 0);
        Vector<uint32_t, 64> worklist(FillWith { }, numBlock, 0);
        Vector<uint32_t, 128> processingOrder;
        std::span<uint64_t> tail = m_liveAtTail.words();

        size_t worklistCount = 0;
        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            if (m_graph.block(blockIndex)) {
                dirtyBlocks[blockIndex / 64] |= static_cast<uint64_t>(1) << (blockIndex % 64);
                worklist[worklistCount++] = blockIndex;
            }
        }

        while (worklistCount) {
            BlockIndex blockIndex = worklist[--worklistCount];
            dirtyBlocks[blockIndex / 64] &= ~(static_cast<uint64_t>(1) << (blockIndex % 64));
            processingOrder.append(blockIndex);

            auto blockActions = actions.span().subspan(actionBegin[blockIndex], actionBegin[blockIndex + 1] - actionBegin[blockIndex]);
            auto blockKills = kill.mutableSpan().subspan(useBegin[blockIndex], useBegin[blockIndex + 1] - useBegin[blockIndex]);
            auto tailRow = tail.subspan(static_cast<size_t>(blockIndex) * wordsPerSet, wordsPerSet);
            size_t worksetCount = JSCSwift_DFGLiveness_processBlock(blockActions.data(), blockActions.size(), tailRow.data(), tailRow.size(),
                worksetMap.data(), worksetValues.data(), numIndices, blockKills.data(), blockKills.size());

            // Update live at head.
            auto& liveAtHead = m_liveAtHead[blockIndex];
            if (worksetCount == liveAtHead.size())
                continue;

            size_t oldHeadCount = liveAtHead.size();
            liveAtHead.grow(oldHeadCount + worksetCount);
            auto blockPredecessors = predecessors.span().subspan(predecessorBegin[blockIndex], predecessorBegin[blockIndex + 1] - predecessorBegin[blockIndex]);
            size_t newHeadCount = 0;
            worklistCount = JSCSwift_DFGLiveness_updateHead(liveAtHead.mutableSpan().data(), oldHeadCount, liveAtHead.size(),
                worksetMap.data(), worksetValues.data(), worksetCount, numIndices,
                tail.data(), tail.size(), wordsPerSet,
                blockPredecessors.data(), blockPredecessors.size(),
                dirtyBlocks.mutableSpan().data(), dirtyBlocks.size(), worklist.mutableSpan().data(), worklist.size(), worklistCount,
                &newHeadCount);
            liveAtHead.shrink(newHeadCount);
        }

        // 3. Kill statuses from each block's last processing, in the order of those.
        Vector<uint32_t, 64> lastProcessing(FillWith { }, numBlock, 0);
        for (size_t i = 0; i < processingOrder.size(); ++i)
            lastProcessing[processingOrder[i]] = static_cast<uint32_t>(i);
        for (size_t i = 0; i < processingOrder.size(); ++i) {
            BlockIndex blockIndex = processingOrder[i];
            if (lastProcessing[blockIndex] != i)
                continue;
            for (size_t k = useBegin[blockIndex]; k < useBegin[blockIndex + 1]; ++k)
                edges[k]->setKillStatus(kill[k] ? DoesKill : DoesNotKill);
        }

        // 4. Update the per-block node list for live and tail.
        for (BlockIndex blockIndex = numBlock; blockIndex--;) {
            BasicBlock* block = m_graph.block(blockIndex);
            if (!block)
                continue;

            {
                block->ssa->liveAtHead = m_liveAtHead[blockIndex].map([this](auto index) {
                    return m_indexing.nodeProjection(index);
                });
            }
            {
                auto& liveAtTail = block->ssa->liveAtTail;
                liveAtTail.shrink(0);
                m_liveAtTail[blockIndex].forEachSetBit([&](size_t index) {
                    liveAtTail.append(m_indexing.nodeProjection(static_cast<unsigned>(index)));
                });
            }
        }

#if ENABLE(SWIFT_KERNELS_VALIDATION)
        if (swiftKernelsValidationEnabled()) [[unlikely]]
            validateAgainstCxx(edges.first(useCount));
#endif
        return true;
    }

#if ENABLE(SWIFT_KERNELS_VALIDATION)
    // Snapshots what runSwift() produced (every block's live-at-head and live-at-tail lists and the kill status of
    // every use Edge), reruns the original runCxx() from the state it starts in, and crashes on any difference.
    // runCxx() owns m_dirtyBlocks, m_worklist and m_workset, which runSwift() does not touch.
    void validateAgainstCxx(std::span<Edge* const> edges)
    {
        BlockIndex numBlock = m_graph.numBlocks();
        Vector<Vector<NodeFlowProjection>> liveAtHead(numBlock);
        Vector<Vector<NodeFlowProjection>> liveAtTail(numBlock);
        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            if (BasicBlock* block = m_graph.block(blockIndex)) {
                liveAtHead[blockIndex] = block->ssa->liveAtHead;
                liveAtTail[blockIndex] = block->ssa->liveAtTail;
            }
            m_liveAtHead[blockIndex].shrink(0);
        }
        Vector<KillStatus> killStatuses = WTF::map(edges, [](Edge* edge) {
            return edge->killStatusUnchecked();
        });
        m_liveAtTail.clear();

        runCxx();

        for (BlockIndex blockIndex = 0; blockIndex < numBlock; ++blockIndex) {
            if (BasicBlock* block = m_graph.block(blockIndex)) {
                RELEASE_ASSERT(liveAtHead[blockIndex] == block->ssa->liveAtHead);
                RELEASE_ASSERT(liveAtTail[blockIndex] == block->ssa->liveAtTail);
            }
        }
        for (size_t i = 0; i < edges.size(); ++i)
            RELEASE_ASSERT(killStatuses[i] == edges[i]->killStatusUnchecked());
    }
#endif
#endif // USE(SWIFT_KERNELS)

    void processBlock(BlockIndex blockIndex)
    {
        BasicBlock* block = m_graph.block(blockIndex);
        ASSERT_WITH_MESSAGE(block, "Only dirty blocks needs updates. A null block should never be dirty.");

        m_workset.clear();
        m_liveAtTail[blockIndex].forEachSetBit([&](size_t index) {
            m_workset.add(static_cast<unsigned>(index));
        });

        for (unsigned nodeIndex = block->size(); nodeIndex--;) {
            Node* node = block->at(nodeIndex);

            auto handleEdge = [&] (Edge& edge) {
                bool newEntry = m_workset.add(m_indexing.index(edge.node()));
                edge.setKillStatus(newEntry ? DoesKill : DoesNotKill);
            };
            
            switch (node->op()) {
            case Upsilon: {
                ASSERT_WITH_MESSAGE(!m_workset.contains(node->index()), "Upsilon should not be used as defs by other nodes.");

                Node* phi = node->phi();
                m_workset.remove(m_indexing.shadowIndex(phi));
                handleEdge(node->child1());
                break;
            }
            case Phi: {
                m_workset.remove(m_indexing.index(node));
                m_workset.add(m_indexing.shadowIndex(node));
                break;
            }
            default:
                m_workset.remove(m_indexing.index(node));
                m_graph.doToChildren(node, handleEdge);
                break;
            }
        }

        // Update live at head.
        auto& liveAtHead = m_liveAtHead[blockIndex];
        if (m_workset.size() == liveAtHead.size())
            return;

        for (unsigned liveIndexAtHead : liveAtHead)
            m_workset.remove(liveIndexAtHead);
        ASSERT(!m_workset.isEmpty());

        liveAtHead.appendRange(m_workset.begin(), m_workset.end());

        for (BasicBlock* predecessor : block->predecessors) {
            auto&& liveAtTail = m_liveAtTail[predecessor];
            for (unsigned newValue : m_workset) {
                if (liveAtTail.add(newValue)) {
                    if (!m_dirtyBlocks.quickSet(predecessor->index()))
                        m_worklist.append(predecessor->index());
                }
            }
        }
    }

    // Blocks with new live values at tail.
    BitVector m_dirtyBlocks;

    FlowIndexing& m_indexing;

    // Live values per block edge.
    IndexMap<BasicBlock*, Vector<unsigned, 0, UnsafeVectorOverflow, 1>> m_liveAtHead;
    TailStorage m_liveAtTail;

    // Single sparse set allocated once and used by every basic block.
    Workset m_workset;

    Vector<BlockIndex, 64> m_worklist;
};

} // anonymous namespace

bool performGraphPackingAndLivenessAnalysis(Graph& graph)
{
    graph.packNodeIndices();
#ifndef NDEBUG
    graph.clearAbstractValues();
#endif
    graph.m_indexingCache->recompute();

    // Pick the live-at-tail representation by the same memory budget WTF::Liveness uses. The dense
    // bit-matrix is numBlocks * numIndices bits; once that would exceed the budget, fall back to the
    // SparseBitVector representation whose memory is proportional to the live values instead.
    uint64_t denseMatrixBits = static_cast<uint64_t>(graph.numBlocks()) * graph.m_indexingCache->numIndices();
    constexpr uint64_t denseMatrixBitBudget = 32 * 1024 * 1024; // 4 MB.
    bool useSparse = denseMatrixBits > denseMatrixBitBudget;
#if ASSERT_ENABLED
    // Most DFG functions are small enough that the dense path would always win. Force the sparse
    // path on roughly half of them in debug builds so the sparse implementation gets meaningful
    // coverage from the regular test suite.
    if (!useSparse && (graph.numBlocks() & 1))
        useSparse = true;
#endif
    if (useSparse)
        return runPhase<LivenessAnalysisPhase<SparseTailStorage>>(graph);
    return runPhase<LivenessAnalysisPhase<DenseTailStorage>>(graph);
}

} } // namespace JSC::DFG

#endif // ENABLE(DFG_JIT)

