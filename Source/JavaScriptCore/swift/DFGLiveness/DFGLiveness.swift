// Copyright (C) 2026 Apple Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.

// The per-block work of JSC::DFG::LivenessAnalysisPhase<DenseTailStorage> (dfg/DFGLivenessAnalysisPhase.cpp):
// processBlock()'s backward walk with its kill statuses, and its "Update live at head" tail (the new live-at-head
// entries and their propagation into the predecessors' live-at-tail rows). The C++ glue keeps the worklist loop,
// flattens the graph into an action stream once per phase run, and afterwards writes the kill statuses into the
// Edges and maps flow indices to NodeFlowProjections. It runs on the DFG/FTL compiler threads.
//
// The workset mirrors WTF::IndexSparseSet<unsigned> exactly (map[value] = position, values[0..<count], add appends,
// remove swaps the last value into the hole), because the order of its values becomes the order of
// BasicBlock::ssa->liveAtHead. The map's contents are arbitrary where no value points back at them, as in C++.
//
// Action stream of one block: its nodes in reverse order, as UInt32 words.
//   Phi:           phiTag, index(node), shadowIndex(node)  -> remove(index); add(shadow)               (no kill)
//   Upsilon:       1, shadowIndex(phi), index(child1)      -> remove(shadow); kill[k++] = add(child)
//   any other:     n (< 2^30), index(node), index(child)*n -> remove(index); kill[k++] = add(child) for each child
// (children in DFG_NODE_DO_TO_CHILDREN order: fixed children up to the first empty one, varargs children that are
// set). kill[k] is 1 for DoesKill (the value was not live below this use) and 0 for DoesNotKill.

enum DFGLiveness {
    @inline(__always) static var phiTag: UInt32 { 1 &<< 30 }
    @inline(__always) static var useCountMask: UInt32 { (1 &<< 30) &- 1 }

    // IndexSparseSet::add: false if value is already in the set.
    @inline(__always)
    static func add(_ value: UInt32, _ map: inout MutableSpan<UInt32>, _ values: inout MutableSpan<UInt32>, _ count: inout Int) -> Bool {
        let key = Int(value)
        let position = Int(map[key])
        if position < count && values[position] == value {
            return false
        }
        values[count] = value
        map[key] = UInt32(truncatingIfNeeded: count)
        count &+= 1
        return true
    }

    // IndexSparseSet::remove.
    @inline(__always)
    static func remove(_ value: UInt32, _ map: inout MutableSpan<UInt32>, _ values: inout MutableSpan<UInt32>, _ count: inout Int) {
        let key = Int(value)
        let position = Int(map[key])
        if position >= count {
            return
        }
        if values[position] == value {
            let last = values[count &- 1]
            map[Int(last)] = UInt32(truncatingIfNeeded: position)
            values[position] = last
            count &-= 1
        }
    }

    // processBlock() up to "Update live at head": m_workset.clear(), add every live-at-tail bit in ascending order,
    // then run the block's action stream. Returns m_workset.size(); the workset is left in map/values.
    @inline(__always)
    static func processBlock(
        actions: Span<UInt32>, tailRow: Span<UInt64>,
        map: inout MutableSpan<UInt32>, values: inout MutableSpan<UInt32>,
        kill: inout MutableSpan<UInt8>
    ) -> Int {
        var count = 0
        var wordIndex = 0
        while wordIndex < tailRow.count {
            var word = tailRow[wordIndex]
            let base = UInt32(truncatingIfNeeded: wordIndex &* 64)
            // The set is empty and the row's bits are distinct, so each add() appends: do that directly (same map,
            // values and count as IndexSparseSet::add would leave).
            while word != 0 {
                let value = base | UInt32(truncatingIfNeeded: word.trailingZeroBitCount)
                values[count] = value
                map[Int(value)] = UInt32(truncatingIfNeeded: count)
                count &+= 1
                word &= word &- 1
            }
            wordIndex &+= 1
        }

        var i = 0
        var k = 0
        while i < actions.count {
            let header = actions[i]
            remove(actions[i &+ 1], &map, &values, &count)
            if header == phiTag {
                _ = add(actions[i &+ 2], &map, &values, &count)
                i &+= 3
                continue
            }
            let uses = Int(header & useCountMask)
            var u = 0
            while u < uses {
                kill[k] = add(actions[i &+ 2 &+ u], &map, &values, &count) ? 1 : 0
                k &+= 1
                u &+= 1
            }
            i &+= 2 &+ uses
        }
        return count
    }

    // processBlock()'s "Update live at head", for a block whose workset size differs from its live-at-head size.
    // head[0..<oldHeadCount] is the block's live-at-head list; the new entries are appended after it. Every new
    // entry is ORed into each predecessor's live-at-tail row; a predecessor that gains a bit and is not dirty is
    // marked dirty and pushed on the worklist. Returns (new head count, new worklist count).
    @inline(__always)
    static func updateHead(
        head: inout MutableSpan<UInt32>, oldHeadCount: Int,
        map: inout MutableSpan<UInt32>, values: inout MutableSpan<UInt32>, count worksetCount: Int,
        tail: inout MutableSpan<UInt64>, wordsPerSet: Int,
        predecessors: Span<UInt32>,
        dirty: inout MutableSpan<UInt64>, worklist: inout MutableSpan<UInt32>, worklistCount: Int
    ) -> (head: Int, worklist: Int) {
        // for (unsigned liveIndexAtHead : liveAtHead) m_workset.remove(liveIndexAtHead);
        var count = worksetCount
        var h = 0
        while h < oldHeadCount {
            remove(head[h], &map, &values, &count)
            h &+= 1
        }
        // liveAtHead.appendRange(m_workset.begin(), m_workset.end());
        var j = 0
        while j < count {
            head[oldHeadCount &+ j] = values[j]
            j &+= 1
        }

        var listCount = worklistCount
        var p = 0
        while p < predecessors.count {
            let predecessor = Int(predecessors[p])
            let rowBase = predecessor &* wordsPerSet
            let dirtyWord = predecessor &>> 6
            let dirtyMask: UInt64 = 1 &<< UInt64(predecessor & 63)
            j = 0
            while j < count {
                let value = Int(values[j])
                // DenseTailStorage::Row::add indexes a std::span of wordsPerSet words (a hardened trap in C++).
                precondition(value &>> 6 < wordsPerSet)
                let wordIndex = rowBase &+ (value &>> 6)
                let mask: UInt64 = 1 &<< UInt64(value & 63)
                let old = tail[wordIndex]
                if old & mask == 0 {
                    tail[wordIndex] = old | mask
                    let oldDirty = dirty[dirtyWord]
                    if oldDirty & dirtyMask == 0 {
                        dirty[dirtyWord] = oldDirty | dirtyMask
                        worklist[listCount] = UInt32(truncatingIfNeeded: predecessor)
                        listCount &+= 1
                    }
                }
                j &+= 1
            }
            p &+= 1
        }
        return (oldHeadCount &+ count, listCount)
    }
}
