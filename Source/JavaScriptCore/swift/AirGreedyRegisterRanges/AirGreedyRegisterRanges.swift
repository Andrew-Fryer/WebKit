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

// The register ranges of JSC::B3::Air's greedy register allocator (b3/air/AirAllocateRegistersByGreedy.cpp): for every
// register, which Tmp occupies it at each Point. They were Greedy::RegisterRange, two WTF::IntervalSet<Point, Tmp, 3>
// per register; this is that data structure, owned by Swift: a faithful port of the IntervalSet B+ tree (the same
// node orders, splits, redistribution with cousins and coverage maintenance, so every tree has the same shape as the
// C++ one built by the same operations) plus RegisterRange's operations and the register loop of
// GreedyAllocator::tryAllocate.
//
// Storage is one arena of 64-bit words that the C++ caller allocates (zero-filled, 64-byte aligned) and grows when an
// operation reports that it needs more; Swift never allocates. The arena is a sequence of 32-word (256-byte, four
// cache line) nodes; node indices, never pointers, link them, so the arena can move when it grows. Nodes
// 0 ..< headerNodes are the header:
//   word 0: the bump index, the first node never handed out (0 before the first allocation: headerNodes).
//   word 1: the head of the free-node list (0 if empty); a free node's word 0 links the next one.
//   word 2: the number of free nodes on that list.
//   words 8 + 4 * set ..< 8 + 4 * set + 3: set `set` = 2 * register + (0 for the allocations, 1 for the high-64-bit
//     clobbers of a vector register): its root NodeRef, its root interval (coverage) and its height.
// An interval [begin, end) of Points (UInt32, WTF::Range<Point>) is one word: begin | end << 32; the empty interval
// is 0 (WTF::Range canonicalizes an empty range to (0, 0)). A NodeRef is node index << 8 | size (0 is null). A Tmp is
// its 32-bit internal value, stored and returned, never interpreted.
//   Leaf (at depth == height): words 0 ..< 16 its intervals; words 16 ..< 32 its Tmps (zero-extended).
//   Inner: words 0 ..< 12 the coverage of each child; words 12 ..< 24 the children's NodeRefs.
// (The queries read only the intervals, the first two cache lines, and a leaf's one Tmp they return.)
// Sizes live in the parent's NodeRef (or the set's root NodeRef), as in the C++.

enum AirGreedyRegisterRanges {
    @inline(__always) static var leafOrder: Int { 16 }
    @inline(__always) static var innerOrder: Int { 12 }
    @inline(__always) static var nodeWords: Int { 32 }
    @inline(__always) static var maxRegisters: Int { 64 }
    @inline(__always) static var globalWords: Int { 8 }
    @inline(__always) static var setWords: Int { 4 }
    // (8 + 2 * 64 * 4 + 31) / 32
    @inline(__always) static var headerNodes: Int { 17 }
    @inline(__always) static var bumpWord: Int { 0 }
    @inline(__always) static var freeHeadWord: Int { 1 }
    @inline(__always) static var freeCountWord: Int { 2 }
    // Heights stay far below this: a height of h takes at least about 6^(h - 2) inserts.
    @inline(__always) static var maxPathLength: Int { 16 }

    // MARK: Words

    @inline(__always) static func begin(_ interval: UInt64) -> UInt32 { UInt32(truncatingIfNeeded: interval) }
    @inline(__always) static func end(_ interval: UInt64) -> UInt32 { UInt32(truncatingIfNeeded: interval &>> 32) }
    @inline(__always) static func pack(_ begin: UInt32, _ end: UInt32) -> UInt64 { UInt64(begin) | UInt64(end) &<< 32 }
    // WTF::Range(begin, end), which canonicalizes an empty range.
    @inline(__always) static func range(_ begin: UInt32, _ end: UInt32) -> UInt64 { begin == end ? 0 : pack(begin, end) }

    // WTF::Range::overlaps (rangesOverlap: empty ranges overlap nothing).
    @inline(__always) static func overlaps(_ a: UInt64, _ b: UInt64) -> Bool {
        let aBegin = begin(a), aEnd = end(a), bBegin = begin(b), bEnd = end(b)
        if aBegin == aEnd || bBegin == bEnd { return false }
        return aEnd > bBegin && bEnd > aBegin
    }

    @inline(__always) static func refNode(_ ref: UInt64) -> Int { Int(truncatingIfNeeded: ref &>> 8) }
    @inline(__always) static func refSize(_ ref: UInt64) -> Int { Int(truncatingIfNeeded: ref & 0xff) }
    @inline(__always) static func makeRef(_ node: Int, _ size: Int) -> UInt64 { UInt64(truncatingIfNeeded: node) &<< 8 | UInt64(truncatingIfNeeded: size) }
    // The first word of a node.
    @inline(__always) static func base(_ ref: UInt64) -> Int { refNode(ref) &* nodeWords }
    @inline(__always) static func withSize(_ ref: UInt64, _ size: Int) -> UInt64 { (ref & ~0xff) | UInt64(truncatingIfNeeded: size) }

    @inline(__always) static func setBase(_ set: Int) -> Int { globalWords &+ set &* setWords }
    @inline(__always) static func rootSlot(_ set: Int) -> Int { setBase(set) }
    @inline(__always) static func coverageSlot(_ set: Int) -> Int { setBase(set) &+ 1 }
    @inline(__always) static func heightSlot(_ set: Int) -> Int { setBase(set) &+ 2 }

    @inline(__always) static func allocationsSet(_ register: Int) -> Int {
        precondition(UInt(bitPattern: register) < UInt(maxRegisters))
        return register &* 2
    }

    // MARK: Read-only queries (IntervalSet::hasOverlap, find)

    // NodeImpl::firstIntervalEndAfter over one node's words: the first of its `size` intervals that ends after `point`,
    // else size (a node holds at most 16 intervals, so the scan needs no bounds checks).
    @inline(__always) static func firstIntervalEndAfter(node: borrowing Span<UInt64>, _ size: Int, _ point: UInt32) -> Int {
        let size = min(size, leafOrder)
        var i = 0
        while i < size {
            if point < end(node[i]) { return i }
            i &+= 1
        }
        return size
    }

    // The same, without branches for nodes of more than 8 intervals (whose early exit mispredicts): a node's intervals
    // are disjoint and sorted (RegisterRange never adds overlapping ones), so their ends ascend and the first one after
    // `point` is preceded by exactly those that end at or before it.
    @inline(__always) static func firstIntervalEndAfterNoBranches(node: borrowing Span<UInt64>, _ size: Int, _ point: UInt32) -> Int {
        if size <= 8 {
            // Small nodes (small sets): a short scan is cheaper.
            var i = 0
            while i < size {
                if point < end(node[i]) { return i }
                i &+= 1
            }
            return size
        }
        let intervals = SIMD16<UInt64>(node[0], node[1], node[2], node[3], node[4], node[5], node[6], node[7],
                                       node[8], node[9], node[10], node[11], node[12], node[13], node[14], node[15])
        let ends = SIMD16<UInt32>(truncatingIfNeeded: intervals &>> 32)
        let lanes = SIMD16<UInt32>(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15)
        let before = (ends .<= SIMD16(repeating: point)) .& (lanes .< SIMD16(repeating: UInt32(truncatingIfNeeded: min(size, leafOrder))))
        return Int(truncatingIfNeeded: SIMD16<UInt32>().replacing(with: 1, where: before).wrappedSum())
    }

    // IntervalSet::hasOverlap.
    @inline(__always) static func hasOverlap(_ words: borrowing Span<UInt64>, _ root: UInt64, _ height: Int, _ query: UInt64) -> Bool {
        let queryBegin = begin(query), queryEnd = end(query)
        var ref = root
        var depth = 0
        while depth < height {
            let start = base(ref)
            let inner = words.extracting(start ..< start &+ nodeWords)
            let size = min(refSize(ref), innerOrder)
            let index = firstIntervalEndAfterNoBranches(node: inner, size, queryBegin)
            if index == size { return false } // query starts after all intervals
            let coverage = inner[index]
            if queryEnd <= begin(coverage) { return false } // query is entirely in the gap before this subtree
            if end(coverage) <= queryEnd { return true } // query spans the subtree's end so it overlaps its last interval
            if queryBegin <= begin(coverage) { return true } // query spans the subtree's start
            ref = inner[innerOrder &+ index]
            depth &+= 1
        }
        let start = base(ref)
        let leaf = words.extracting(start ..< start &+ nodeWords)
        let index = firstIntervalEndAfterNoBranches(node: leaf, refSize(ref), queryBegin)
        return begin(leaf[index]) < queryEnd
    }

    // IntervalSet::find: the first interval overlapping the query; returned as where it is: the first word of its leaf,
    // its index there and the leaf's size.
    @inline(__always) static func find(_ words: borrowing Span<UInt64>, _ set: Int, _ query: UInt64) -> (leaf: Int, index: Int, size: Int)? {
        if !overlaps(query, words[coverageSlot(set)]) { return nil }
        let queryBegin = begin(query), queryEnd = end(query)
        var ref = words[rootSlot(set)]
        let height = Int(truncatingIfNeeded: words[heightSlot(set)])
        var depth = 0
        while depth < height {
            let start = base(ref)
            let inner = words.extracting(start ..< start &+ nodeWords)
            let size = min(refSize(ref), innerOrder)
            let index = firstIntervalEndAfter(node: inner, size, queryBegin)
            if index == size { return nil } // query is entirely after this subtree
            if queryEnd <= begin(inner[index]) { return nil } // query is entirely before this subtree
            ref = inner[innerOrder &+ index]
            depth &+= 1
        }
        let start = base(ref)
        let leaf = words.extracting(start ..< start &+ nodeWords)
        let index = firstIntervalEndAfter(node: leaf, refSize(ref), queryBegin)
        if queryEnd <= begin(leaf[index]) { return nil }
        return (start, index, refSize(ref))
    }

    // RegisterRange::hasConflict: does any of the intervals overlap the register's allocations (or, if checkHigh64,
    // its high-64-bit clobbers)?
    @inline(__always) static func hasConflict(_ words: borrowing Span<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>, _ checkHigh64: Bool) -> Bool {
        let set = allocationsSet(register)
        if anyOverlap(words, set, intervals) { return true }
        if !checkHigh64 { return false }
        return anyOverlap(words, set &+ 1, intervals)
    }

    // Whether hasOverlap() holds for any of the intervals.
    @inline(__always) static func anyOverlap(_ words: borrowing Span<UInt64>, _ set: Int, _ intervals: borrowing Span<UInt64>) -> Bool {
        let header = words.extracting(setBase(set) ..< setBase(set) &+ setWords)
        let coverage = header[1]
        if coverage == 0 { return false } // Empty: overlaps nothing.
        let coverageBegin = begin(coverage), coverageEnd = end(coverage)
        let root = header[0]
        let height = Int(truncatingIfNeeded: header[2])
        for i in intervals.indices {
            let query = intervals[i]
            // query.overlaps(coverage), knowing that the coverage is not empty.
            let queryBegin = begin(query), queryEnd = end(query)
            if queryBegin != queryEnd && queryEnd > coverageBegin && coverageEnd > queryBegin && hasOverlap(words, root, height, query) { return true }
        }
        return false
    }

    // The cursor of a finished conflict scan.
    @inline(__always) static var scanOver: UInt64 { ~0 }

    // RegisterRange::forEachConflict's scan, resumable: writes (tmp, begin, end) triples to `conflicts` for the next
    // conflicts (at most conflicts.count / 3, which must be at least 1) in the order forEachConflictImpl visits them:
    // for each interval of the range, find() the first overlapping entry, then continue after its end. `sets` bit 0
    // scans the allocations, bit 1 then the high-64 clobbers. The cursor (0 to start) is the position: low half the
    // interval index (counting the high-64 pass's intervals after the allocation pass's), high half the query's begin
    // once it was advanced past a conflict; scanOver when the scan is over, which is the only case in which this
    // returns 0. Returns the number of triples written.
    //
    // The next conflict after one in the same interval is that entry's successor, if the successor overlaps the
    // interval (the entries are disjoint and sorted); within one call (the set cannot change) it is read directly when it
    // is in the same leaf. A call does a root-to-leaf descent only until it has found a conflict, so that callers that
    // stop early (most do) waste no descents.
    @inline(__always)
    static func conflicts(_ words: borrowing Span<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>, _ sets: UInt32, _ cursor: inout UInt64, _ conflicts: inout MutableSpan<UInt32>) -> Int {
        Self.conflicts(words, register, intervals, sets, &cursor, &conflicts, at: 0, capacity: conflicts.count / 3)
    }

    // The same, writing the triples to conflicts[3 * at ..< 3 * (at + capacity)].
    @inline(__always)
    static func conflicts(_ words: borrowing Span<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>, _ sets: UInt32, _ cursor: inout UInt64, _ conflicts: inout MutableSpan<UInt32>, at first: Int, capacity: Int) -> Int {
        let set = allocationsSet(register)
        let count = intervals.count
        precondition(capacity > 0)
        if cursor == scanOver { return 0 }
        var position = Int(truncatingIfNeeded: cursor & 0xffff_ffff)
        var advancedBegin = UInt32(truncatingIfNeeded: cursor &>> 32)
        if (sets & 1) == 0 && position < count {
            position = count
            advancedBegin = 0
        }
        let limit = (sets & 2) != 0 ? 2 &* count : count
        var written = 0
        // The leaf of the last conflict found by this call (its first word, the conflict's index and the leaf's size;
        // size 0 when there is none or the query moved on to another interval).
        var leaf = 0, leafIndex = 0, leafSize = 0
        while position < limit && written < capacity {
            let isHigh64 = position >= count
            let index = isHigh64 ? position &- count : position
            let intervalBegin = begin(intervals[index]), intervalEnd = end(intervals[index])
            if leafIndex &+ 1 < leafSize {
                leafIndex &+= 1
                if intervalEnd <= begin(words[leaf &+ leafIndex]) {
                    position &+= 1 // The successor is after this interval: no other conflicts with it.
                    advancedBegin = 0
                    leafSize = 0
                    continue
                }
            } else {
                if written != 0 { break }
                guard let found = find(words, isHigh64 ? set &+ 1 : set, range(max(advancedBegin, intervalBegin), intervalEnd)) else {
                    position &+= 1
                    advancedBegin = 0
                    leafSize = 0
                    continue
                }
                (leaf, leafIndex, leafSize) = found
            }
            let conflict = words[leaf &+ leafIndex]
            let slot = 3 &* (first &+ written)
            conflicts[slot] = UInt32(truncatingIfNeeded: words[leaf &+ leafOrder &+ leafIndex])
            conflicts[slot &+ 1] = begin(conflict)
            conflicts[slot &+ 2] = end(conflict)
            written &+= 1
            if intervalEnd <= end(conflict) {
                position &+= 1 // There can't exist other conflicts with this interval.
                advancedBegin = 0
                leafSize = 0
            } else {
                advancedBegin = end(conflict) // Search for remaining conflicts with the interval.
            }
        }
        cursor = position >= limit ? scanOver : UInt64(truncatingIfNeeded: position) | UInt64(advancedBegin) &<< 32
        return written
    }

    // conflicts() for each register of an order, from the start of its scan, with room for `capacity` conflicts each:
    // register i's go to conflicts[3 * capacity * i ...], their number to counts[i] and its cursor to cursors[i].
    static func firstConflicts(_ words: borrowing Span<UInt64>, _ order: borrowing Span<UInt8>, _ intervals: borrowing Span<UInt64>, _ sets: UInt32, _ cursors: inout MutableSpan<UInt64>, _ counts: inout MutableSpan<UInt8>, _ conflicts: inout MutableSpan<UInt32>, capacity: Int) {
        precondition(capacity > 0 && capacity <= Int(UInt8.max))
        for i in order.indices {
            var cursor: UInt64 = 0
            counts[i] = UInt8(truncatingIfNeeded: Self.conflicts(words, Int(order[i] & 0x7f), intervals, sets, &cursor, &conflicts, at: capacity &* i, capacity: capacity))
            cursors[i] = cursor
        }
    }

    // RegisterRange::forEachConflict's scan, in the order conflicts() reports: calls body with each conflict's Tmp until
    // it returns false. (Within the scan the set cannot change, so the next conflict in the same interval is read as the
    // previous one's successor when that is in the same leaf.)
    @inline(__always)
    static func forEachConflict(_ words: borrowing Span<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>, _ sets: UInt32, _ body: (UInt32) -> Bool) {
        let set = allocationsSet(register)
        let count = intervals.count
        var position = (sets & 1) == 0 ? count : 0
        let limit = (sets & 2) != 0 ? 2 &* count : count
        var advancedBegin: UInt32 = 0
        // The leaf of the last conflict (its first word, the conflict's index and the leaf's size; size 0 when there
        // is none or the scan moved on to another interval).
        var leaf = 0, leafIndex = 0, leafSize = 0
        while position < limit {
            let isHigh64 = position >= count
            let index = isHigh64 ? position &- count : position
            let intervalBegin = begin(intervals[index]), intervalEnd = end(intervals[index])
            if leafIndex &+ 1 < leafSize {
                leafIndex &+= 1
                if intervalEnd <= begin(words[leaf &+ leafIndex]) {
                    position &+= 1 // The successor is after this interval: no other conflicts with it.
                    advancedBegin = 0
                    leafSize = 0
                    continue
                }
            } else {
                guard let found = find(words, isHigh64 ? set &+ 1 : set, range(max(advancedBegin, intervalBegin), intervalEnd)) else {
                    position &+= 1
                    advancedBegin = 0
                    leafSize = 0
                    continue
                }
                (leaf, leafIndex, leafSize) = found
            }
            let conflict = words[leaf &+ leafIndex]
            if !body(UInt32(truncatingIfNeeded: words[leaf &+ leafOrder &+ leafIndex])) { return }
            if intervalEnd <= end(conflict) {
                position &+= 1 // There can't exist other conflicts with this interval.
                advancedBegin = 0
                leafSize = 0
            } else {
                advancedBegin = end(conflict) // Search for remaining conflicts with the interval.
            }
        }
    }

    // GreedyAllocator::tryEvict's cost loop: for each register of the order, the sum of the spill costs of its conflicts
    // with the intervals (in forEachConflict's order, stopping at an unspillable one or once the sum reaches the lowest
    // so far); returns the index in the order of the first register whose sum is below minSpillCost, the lowest so far,
    // (which becomes that sum), or -1. A conflict whose Tmp is a register's (its Int32 value v is in lastFPR ... -1 or
    // 1 ... lastGPR, as Tmp::isReg() decides) is unspillable; for any other, cost(tmp, orderIndex) is its spill cost (Float
    // bits), or a value of 1 << 32 or more (JSCSwift_AirGreedyRegisterRanges_skipConflict) to skip it.
    @inline(__always)
    static func cheapestEviction(_ words: borrowing Span<UInt64>, _ order: borrowing Span<UInt8>, _ intervals: borrowing Span<UInt64>, _ sets: UInt32, lastGPR: Int32, lastFPR: Int32, _ minSpillCost: inout Float, _ cost: (UInt32, Int) -> UInt64) -> Int32 {
        let unspillableCost = Float.infinity
        let maxSpillableSpillCost = Float.greatestFiniteMagnitude
        var best: Int32 = -1
        var minimum = minSpillCost
        for i in order.indices {
            var conflictsSpillCost: Float = 0
            forEachConflict(words, Int(order[i] & 0x7f), intervals, sets) { tmp in
                // Tmp::isReg(): a register's own Tmp (a fixed use, def or clobber) cannot be evicted.
                let value = Int32(bitPattern: tmp)
                if value != 0 && value >= lastFPR && value <= lastGPR {
                    conflictsSpillCost = unspillableCost
                    return false
                }
                let bits = cost(tmp, i)
                if bits > UInt64(UInt32.max) { return true } // Skipped (already counted): continue.
                let spillCost = Float(bitPattern: UInt32(truncatingIfNeeded: bits))
                if spillCost == unspillableCost {
                    conflictsSpillCost = unspillableCost
                    return false
                }
                if spillCost == maxSpillableSpillCost || conflictsSpillCost == maxSpillableSpillCost {
                    conflictsSpillCost = maxSpillableSpillCost
                } else {
                    conflictsSpillCost += spillCost
                }
                return !(conflictsSpillCost >= minimum)
            }
            if conflictsSpillCost < minimum {
                minimum = conflictsSpillCost
                best = Int32(truncatingIfNeeded: i)
            }
        }
        minSpillCost = minimum
        return best
    }

    // MARK: Node allocation

    @inline(__always) static func bump(_ words: borrowing MutableSpan<UInt64>) -> Int {
        let bump = Int(truncatingIfNeeded: words[bumpWord])
        return bump == 0 ? headerNodes : bump
    }

    // Returns 0 if `nodes` more nodes can be allocated, else the number of words the arena needs for that.
    @inline(__always) static func wordsNeeded(_ words: borrowing MutableSpan<UInt64>, _ nodes: Int) -> Int {
        let free = Int(truncatingIfNeeded: words[freeCountWord])
        let unused = words.count / nodeWords &- bump(words)
        if nodes <= free &+ unused { return 0 }
        return (bump(words) &+ nodes &- free) &* nodeWords
    }

    // Returns the first word of a new node (its contents are garbage, as fastAlignedMalloc's would be).
    static func allocNode(_ words: inout MutableSpan<UInt64>) -> Int {
        let head = Int(truncatingIfNeeded: words[freeHeadWord])
        if head != 0 {
            words[freeHeadWord] = words[head &* nodeWords]
            words[freeCountWord] &-= 1
            return head &* nodeWords
        }
        let node = bump(words)
        precondition(node < words.count / nodeWords)
        words[bumpWord] = UInt64(truncatingIfNeeded: node &+ 1)
        return node &* nodeWords
    }

    static func freeNode(_ words: inout MutableSpan<UInt64>, _ node: Int) {
        words[node] = words[freeHeadWord]
        words[freeHeadWord] = UInt64(truncatingIfNeeded: node / nodeWords)
        words[freeCountWord] &+= 1
    }

    // IntervalSet::freeAllNodes for a subtree.
    static func freeSubtree(_ words: inout MutableSpan<UInt64>, _ ref: UInt64, _ distanceToLeaf: Int) {
        let node = base(ref)
        if distanceToLeaf != 0 {
            var i = 0
            while i < refSize(ref) {
                freeSubtree(&words, words[node &+ innerOrder &+ i], distanceToLeaf &- 1)
                i &+= 1
            }
        }
        freeNode(&words, node)
    }

    // MARK: Node contents

    // A leaf's Tmps follow its 16 intervals, an inner node's children its 12 coverages.
    @inline(__always) static func payloadOffset(leaf: Bool) -> Int { leaf ? leafOrder : innerOrder }

    @inline(__always) static func payload(_ words: borrowing MutableSpan<UInt64>, _ node: Int, _ index: Int, leaf: Bool) -> UInt64 {
        words[node &+ payloadOffset(leaf: leaf) &+ index]
    }

    @inline(__always) static func setPayload(_ words: inout MutableSpan<UInt64>, _ node: Int, _ index: Int, _ value: UInt64, leaf: Bool) {
        words[node &+ payloadOffset(leaf: leaf) &+ index] = value
    }

    @inline(__always) static func copyEntry(_ words: inout MutableSpan<UInt64>, from: Int, _ fromIndex: Int, to: Int, _ toIndex: Int, leaf: Bool) {
        words[to &+ toIndex] = words[from &+ fromIndex]
        setPayload(&words, to, toIndex, payload(words, from, fromIndex, leaf: leaf), leaf: leaf)
    }

    // NodeImpl::coverage (RELEASE_ASSERTs a non-empty node).
    @inline(__always) static func coverage(_ words: borrowing MutableSpan<UInt64>, _ node: Int, _ size: Int) -> UInt64 {
        precondition(size != 0)
        return range(begin(words[node]), end(words[node &+ size &- 1]))
    }

    @inline(__always) static func capacity(leaf: Bool) -> Int { leaf ? leafOrder : innerOrder }

    // NodeImpl::insertAt.
    @inline(__always) static func insertAt(_ words: inout MutableSpan<UInt64>, _ node: Int, _ size: inout Int, _ index: Int, _ interval: UInt64, _ value: UInt64, leaf: Bool) {
        var i = size
        while i > index {
            copyEntry(&words, from: node, i &- 1, to: node, i, leaf: leaf)
            i &-= 1
        }
        words[node &+ index] = interval
        setPayload(&words, node, index, value, leaf: leaf)
        size &+= 1
    }

    // NodeImpl::removeAt.
    static func removeAt(_ words: inout MutableSpan<UInt64>, _ node: Int, _ size: inout Int, _ index: Int, leaf: Bool) {
        var i = index
        while i < size &- 1 {
            copyEntry(&words, from: node, i &+ 1, to: node, i, leaf: leaf)
            i &+= 1
        }
        size &-= 1
    }

    // NodeImpl::shiftLeftFrom: moves the first `count` entries of `right` (this node's right cousin) to this node's end.
    static func shiftLeftFrom(_ words: inout MutableSpan<UInt64>, _ node: Int, _ size: inout Int, _ right: Int, _ rightSize: inout Int, _ count: Int, leaf: Bool) {
        var i = 0
        while i < count {
            copyEntry(&words, from: right, i, to: node, i &+ size, leaf: leaf)
            i &+= 1
        }
        i = 0
        while i < rightSize &- count {
            copyEntry(&words, from: right, i &+ count, to: right, i, leaf: leaf)
            i &+= 1
        }
        size &+= count
        rightSize &-= count
    }

    // NodeImpl::shiftRightTo: moves this node's last `count` entries to the start of `right`.
    static func shiftRightTo(_ words: inout MutableSpan<UInt64>, _ node: Int, _ size: inout Int, _ right: Int, _ rightSize: inout Int, _ count: Int, leaf: Bool) {
        var i = rightSize &+ count &- 1
        while i >= count {
            copyEntry(&words, from: right, i &- count, to: right, i, leaf: leaf)
            i &-= 1
        }
        i = 0
        while i < count {
            copyEntry(&words, from: node, size &- count &+ i, to: right, i, leaf: leaf)
            i &+= 1
        }
        size &-= count
        rightSize &+= count
    }

    // std::midpoint for size_t: rounds towards a.
    @inline(__always) static func midpoint(_ a: Int, _ b: Int) -> Int {
        a <= b ? a &+ (b &- a) / 2 : a &- (a &- b) / 2
    }

    // MARK: Paths

    // IntervalSet::Path: for each depth, the word holding the NodeRef of the node at that depth (a set's root slot or
    // a child slot of its parent) and the index within that node.
    struct Path {
        var slots: InlineArray<16, Int>
        var indices: InlineArray<16, Int>
        var count: Int

        @inline(__always) init() {
            slots = InlineArray(repeating: 0)
            indices = InlineArray(repeating: 0)
            count = 0
        }

        @inline(__always) mutating func append(_ slot: Int, _ index: Int) {
            slots[count] = slot
            indices[count] = index
            count &+= 1
        }
    }

    // Path::toLeftCousin / toRightCousin: makes the path that of the immediate left (right) cousin of its last node,
    // or empty if there is none.
    static func toCousin(_ words: borrowing MutableSpan<UInt64>, _ path: inout Path, left: Bool) {
        let initialDepth = path.count &- 1
        if initialDepth == 0 {
            path.count = 0 // Root has no cousins
            return
        }
        var depth = initialDepth &- 1
        while depth >= 0 {
            let index = path.indices[depth]
            if left ? index != 0 : index < refSize(words[path.slots[depth]]) &- 1 { break }
            depth &-= 1
        }
        if depth < 0 {
            path.count = 0
            return
        }
        path.indices[depth] = left ? path.indices[depth] &- 1 : path.indices[depth] &+ 1
        var childSlot = base(words[path.slots[depth]]) &+ innerOrder &+ path.indices[depth]
        depth &+= 1
        while depth < initialDepth {
            let childRef = words[childSlot]
            let childIndex = left ? refSize(childRef) &- 1 : 0
            path.slots[depth] = childSlot
            path.indices[depth] = childIndex
            childSlot = base(childRef) &+ innerOrder &+ childIndex
            depth &+= 1
        }
        path.slots[depth] = childSlot
        path.indices[depth] = left ? refSize(words[childSlot]) &- 1 : 0
    }

    @inline(__always) static func isFirstOrLastIndex(_ ref: UInt64, _ index: Int) -> Bool {
        index == 0 || index == refSize(ref) &- 1
    }

    @inline(__always) static func setSize(_ words: inout MutableSpan<UInt64>, _ slot: Int, _ size: Int) {
        words[slot] = withSize(words[slot], size)
    }

    // IntervalSet::updateCoverage: after the node at `depth` of the path changed, propagates its new coverage up.
    static func updateCoverage(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, _ coverage: UInt64) {
        var depth = depth &- 1
        var coverage = coverage
        while depth >= 0 {
            let ref = words[path.slots[depth]]
            let inner = base(ref)
            let index = path.indices[depth]
            words[inner &+ index] = coverage
            if !isFirstOrLastIndex(ref, index) { return }
            coverage = Self.coverage(words, inner, refSize(ref))
            depth &-= 1
        }
        words[coverageSlot(set)] = coverage
    }

    // MARK: Insert (IntervalSet::insert)

    static func insertInNodeSplitIfNeeded(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, _ interval: UInt64, _ value: UInt64, leaf: Bool) -> (ref: UInt64, coverage: UInt64) {
        let slot = path.slots[depth]
        var nodeSize = refSize(words[slot])
        if nodeSize < capacity(leaf: leaf) {
            let insertionIndex = path.indices[depth]
            let node = base(words[slot])
            insertAt(&words, node, &nodeSize, insertionIndex, interval, value, leaf: leaf)
            setSize(&words, slot, nodeSize)
            if isFirstOrLastIndex(words[slot], insertionIndex) {
                updateCoverage(&words, set, path, depth, coverage(words, node, nodeSize))
            }
            return (0, 0)
        }
        if tryRedistributeLeftAndInsert(&words, set, path, depth, interval, value, leaf: leaf) { return (0, 0) }
        if tryRedistributeRightAndInsert(&words, set, path, depth, interval, value, leaf: leaf) { return (0, 0) }
        return splitNodeAndInsert(&words, set, path, depth, interval, value, leaf: leaf)
    }

    static func tryRedistributeLeftAndInsert(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, _ interval: UInt64, _ value: UInt64, leaf: Bool) -> Bool {
        let slot = path.slots[depth]
        let insertionIndex = path.indices[depth]
        let node = base(words[slot])
        var nodeSize = refSize(words[slot])

        var leftPath = copy path
        leftPath.count = depth &+ 1
        toCousin(words, &leftPath, left: true)
        if leftPath.count == 0 { return false }

        let leftSlot = leftPath.slots[depth]
        var leftNodeSize = refSize(words[leftSlot])
        if leftNodeSize == capacity(leaf: leaf) { return false }

        let leftNode = base(words[leftSlot])
        let newSize = midpoint(leftNodeSize, nodeSize)
        let numToMove = nodeSize &- newSize
        shiftLeftFrom(&words, leftNode, &leftNodeSize, node, &nodeSize, numToMove, leaf: leaf)
        if insertionIndex < numToMove {
            insertAt(&words, leftNode, &leftNodeSize, leftNodeSize &+ insertionIndex &- numToMove, interval, value, leaf: leaf)
        } else {
            insertAt(&words, node, &nodeSize, insertionIndex &- numToMove, interval, value, leaf: leaf)
        }
        setSize(&words, leftSlot, leftNodeSize)
        updateCoverage(&words, set, leftPath, depth, coverage(words, leftNode, leftNodeSize))
        setSize(&words, slot, nodeSize)
        updateCoverage(&words, set, path, depth, coverage(words, node, nodeSize))
        return true
    }

    static func tryRedistributeRightAndInsert(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, _ interval: UInt64, _ value: UInt64, leaf: Bool) -> Bool {
        let slot = path.slots[depth]
        let insertionIndex = path.indices[depth]
        let node = base(words[slot])
        var nodeSize = refSize(words[slot])

        var rightPath = copy path
        rightPath.count = depth &+ 1
        toCousin(words, &rightPath, left: false)
        if rightPath.count == 0 { return false }

        let rightSlot = rightPath.slots[depth]
        var rightNodeSize = refSize(words[rightSlot])
        if rightNodeSize == capacity(leaf: leaf) { return false }

        let rightNode = base(words[rightSlot])
        // If the insertion index is after all items of the left node and we only have one empty slot
        // we need to insert into the head of the right node.
        if insertionIndex == capacity(leaf: leaf) {
            insertAt(&words, rightNode, &rightNodeSize, 0, interval, value, leaf: leaf)
            setSize(&words, rightSlot, rightNodeSize)
            updateCoverage(&words, set, rightPath, depth, coverage(words, rightNode, rightNodeSize))
            return true
        }
        let newSize = midpoint(rightNodeSize, nodeSize)
        let numToMove = nodeSize &- newSize
        shiftRightTo(&words, node, &nodeSize, rightNode, &rightNodeSize, numToMove, leaf: leaf)
        if insertionIndex <= nodeSize {
            insertAt(&words, node, &nodeSize, insertionIndex, interval, value, leaf: leaf)
        } else {
            insertAt(&words, rightNode, &rightNodeSize, insertionIndex &- nodeSize, interval, value, leaf: leaf)
        }
        setSize(&words, slot, nodeSize)
        updateCoverage(&words, set, path, depth, coverage(words, node, nodeSize))
        setSize(&words, rightSlot, rightNodeSize)
        updateCoverage(&words, set, rightPath, depth, coverage(words, rightNode, rightNodeSize))
        return true
    }

    static func splitNodeAndInsert(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, _ interval: UInt64, _ value: UInt64, leaf: Bool) -> (ref: UInt64, coverage: UInt64) {
        let slot = path.slots[depth]
        let insertionIndex = path.indices[depth]
        let node = base(words[slot])
        var nodeSize = refSize(words[slot])
        let splitPoint = (capacity(leaf: leaf) &+ 1) / 2
        let newNode = allocNode(&words)

        var i = splitPoint
        while i < nodeSize {
            copyEntry(&words, from: node, i, to: newNode, i &- splitPoint, leaf: leaf)
            i &+= 1
        }
        var newNodeSize = nodeSize &- splitPoint
        nodeSize = splitPoint

        if insertionIndex <= nodeSize {
            insertAt(&words, node, &nodeSize, insertionIndex, interval, value, leaf: leaf)
        } else {
            insertAt(&words, newNode, &newNodeSize, insertionIndex &- nodeSize, interval, value, leaf: leaf)
        }
        setSize(&words, slot, nodeSize)
        updateCoverage(&words, set, path, depth, coverage(words, node, nodeSize))
        return (makeRef(newNode / nodeWords, newNodeSize), coverage(words, newNode, newNodeSize))
    }

    // InnerNode::subtreeForInsert.
    @inline(__always) static func subtreeForInsert(_ words: borrowing MutableSpan<UInt64>, _ node: Int, _ size: Int, _ endPoint: UInt32) -> Int {
        var i = 0
        while i < size &- 1 {
            if endPoint <= begin(words[node &+ i &+ 1]) { return i }
            i &+= 1
        }
        return size &- 1
    }

    // The caller has checked that height + 2 nodes can be allocated (a split at every depth and a new root).
    static func insert(_ words: inout MutableSpan<UInt64>, _ set: Int, _ interval: UInt64, _ value: UInt32) {
        if words[rootSlot(set)] == 0 {
            words[rootSlot(set)] = makeRef(allocNode(&words) / nodeWords, 0)
            words[heightSlot(set)] = 0
        }
        let height = Int(truncatingIfNeeded: words[heightSlot(set)])
        precondition(height < maxPathLength)

        var path = Path()
        var slot = rootSlot(set)
        let intervalEnd = end(interval)
        var depth = 0
        while depth < height {
            let ref = words[slot]
            let index = subtreeForInsert(words, base(ref), refSize(ref), intervalEnd)
            path.append(slot, index)
            slot = base(ref) &+ innerOrder &+ index
            depth &+= 1
        }
        let leafRef = words[slot]
        let leafStart = base(leafRef)
        var leafSize = refSize(leafRef)
        let insertionIndex = firstIntervalEndAfter(node: words.span.extracting(leafStart ..< leafStart &+ nodeWords), leafSize, intervalEnd)
        path.append(slot, insertionIndex)

        if leafSize < leafOrder {
            // The common case, insertInNodeSplitIfNeeded's first: the leaf has room.
            insertAt(&words, leafStart, &leafSize, insertionIndex, interval, UInt64(value), leaf: true)
            setSize(&words, slot, leafSize)
            if isFirstOrLastIndex(words[slot], insertionIndex) {
                updateCoverage(&words, set, path, height, coverage(words, leafStart, leafSize))
            }
            return
        }
        var (newNode, newNodeCoverage) = insertInNodeSplitIfNeeded(&words, set, path, height, interval, UInt64(value), leaf: true)

        depth = height &- 1
        while depth >= 0 {
            if newNode == 0 { return }
            path.indices[depth] &+= 1 // Insert new parent immediately after the existing parent
            (newNode, newNodeCoverage) = insertInNodeSplitIfNeeded(&words, set, path, depth, newNodeCoverage, newNode, leaf: false)
            depth &-= 1
        }

        if newNode != 0 {
            // Need to add another level to the tree.
            let newRoot = allocNode(&words)
            words[newRoot] = words[coverageSlot(set)]
            words[newRoot &+ innerOrder] = words[rootSlot(set)]
            words[newRoot &+ 1] = newNodeCoverage
            words[newRoot &+ innerOrder &+ 1] = newNode
            words[heightSlot(set)] = UInt64(truncatingIfNeeded: height &+ 1)
            words[rootSlot(set)] = makeRef(newRoot / nodeWords, 2)
            words[coverageSlot(set)] = coverage(words, newRoot, 2)
        }
    }

    // MARK: Erase (IntervalSet::erase)

    static func eraseFromNode(_ words: inout MutableSpan<UInt64>, _ set: Int, _ path: borrowing Path, _ depth: Int, leaf: Bool) -> Bool {
        let slot = path.slots[depth]
        let eraseIndex = path.indices[depth]
        let ref = words[slot]
        let node = base(ref)
        var nodeSize = refSize(ref)
        if nodeSize == 1 {
            freeNode(&words, node)
            words[slot] = 0
            return true
        }
        removeAt(&words, node, &nodeSize, eraseIndex, leaf: leaf)
        if isFirstOrLastIndex(ref, eraseIndex) {
            updateCoverage(&words, set, path, depth, coverage(words, node, nodeSize))
        }
        setSize(&words, slot, nodeSize)
        return false
    }

    static func erase(_ words: inout MutableSpan<UInt64>, _ set: Int, _ interval: UInt64) {
        precondition(words[rootSlot(set)] != 0) // C++ would dereference a null root.
        let height = Int(truncatingIfNeeded: words[heightSlot(set)])
        precondition(height < maxPathLength)
        var path = Path()
        var slot = rootSlot(set)
        let intervalBegin = begin(interval)
        var depth = 0
        while depth < height {
            let ref = words[slot]
            let start = base(ref)
            let index = firstIntervalEndAfter(node: words.span.extracting(start ..< start &+ nodeWords), min(refSize(ref), innerOrder), intervalBegin)
            path.append(slot, index)
            slot = start &+ innerOrder &+ index
            depth &+= 1
        }
        let leafRef = words[slot]
        let leafStart = base(leafRef)
        path.append(slot, firstIntervalEndAfter(node: words.span.extracting(leafStart ..< leafStart &+ nodeWords), refSize(leafRef), intervalBegin))

        var removedNode = eraseFromNode(&words, set, path, height, leaf: true)
        depth = height &- 1
        while depth >= 0 {
            if !removedNode { return }
            removedNode = eraseFromNode(&words, set, path, depth, leaf: false)
            depth &-= 1
        }
        if removedNode {
            // The tree is now empty.
            words[coverageSlot(set)] = 0
            words[heightSlot(set)] = 0
        }
    }

    // MARK: Bulk build (IntervalSet(span, value))

    // The nodes the bulk build of `count` entries allocates.
    static func bulkNodes(_ count: Int) -> Int {
        if count == 0 { return 0 }
        var entries = count
        var perNode = leafOrder
        var total = 0
        while true {
            let nodes = (entries &+ perNode &- 1) / perNode
            total &+= nodes
            if nodes == 1 { return total }
            entries = nodes
            perNode = innerOrder
        }
    }

    // Replaces the set with one built from the ascending, non-overlapping intervals, all mapped to `value`: the same
    // tree IntervalSet's bulk constructor lays out level by level (each level cut into as few nodes as possible, sized
    // as evenly as possible, earlier nodes taking the remainder), built here in one left-to-right pass that closes
    // each node when it is full and adds it to its parent. The caller has checked that bulkNodes(count) nodes can be
    // allocated.
    static func build(_ words: inout MutableSpan<UInt64>, _ set: Int, _ intervals: borrowing Span<UInt64>, _ value: UInt32) {
        // IntervalSet's move assignment frees the old tree.
        let oldRoot = words[rootSlot(set)]
        if oldRoot != 0 {
            freeSubtree(&words, oldRoot, Int(truncatingIfNeeded: words[heightSlot(set)]))
        }
        words[rootSlot(set)] = 0
        words[coverageSlot(set)] = 0
        words[heightSlot(set)] = 0
        let count = intervals.count
        if count == 0 { return }

        // Per level: the number of nodes, the base node size and the number of nodes one larger (the first ones).
        var levelNodes = InlineArray<32, Int>(repeating: 0)
        var levelBase = InlineArray<32, Int>(repeating: 0)
        var levelRemainder = InlineArray<32, Int>(repeating: 0)
        var height = 0
        var entries = count
        while true {
            let perNode = height == 0 ? leafOrder : innerOrder
            let nodes = (entries &+ perNode &- 1) / perNode
            levelNodes[height] = nodes
            levelBase[height] = entries / nodes
            levelRemainder[height] = entries % nodes
            if nodes == 1 { break }
            entries = nodes
            height &+= 1
        }

        // Per level: the node being filled, how many entries it has and will have, and how many nodes were closed.
        var current = InlineArray<32, Int>(repeating: 0)
        var filled = InlineArray<32, Int>(repeating: 0)
        var target = InlineArray<32, Int>(repeating: 0)
        var closed = InlineArray<32, Int>(repeating: 0)
        var i = 0
        while i < count {
            var level = 0
            var entry = intervals[i]
            var payload = UInt64(value)
            while true {
                let leaf = level == 0
                if filled[level] == 0 {
                    current[level] = allocNode(&words)
                    target[level] = levelBase[level] &+ (closed[level] < levelRemainder[level] ? 1 : 0)
                }
                words[current[level] &+ filled[level]] = entry
                setPayload(&words, current[level], filled[level], payload, leaf: leaf)
                filled[level] &+= 1
                if filled[level] < target[level] { break }
                let ref = makeRef(current[level] / nodeWords, target[level])
                let nodeCoverage = coverage(words, current[level], target[level])
                filled[level] = 0
                closed[level] &+= 1
                if level == height {
                    words[rootSlot(set)] = ref
                    words[coverageSlot(set)] = nodeCoverage
                    words[heightSlot(set)] = UInt64(truncatingIfNeeded: height)
                    break
                }
                entry = nodeCoverage
                payload = ref
                level &+= 1
            }
            i &+= 1
        }
    }

    // MARK: RegisterRange operations

    @inline(__always) static func isEmpty(_ words: borrowing MutableSpan<UInt64>, _ set: Int) -> Bool {
        words[coverageSlot(set)] == 0
    }

    // RegisterRange::add from intervals[progress...]: the first range added to an empty register builds the tree in one
    // pass; otherwise each interval is inserted. Returns 0 when done (progress == count), or the number of words the
    // arena needs to continue from `progress` (nothing is changed past what `progress` reports).
    static func add(_ words: inout MutableSpan<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>, _ value: UInt32, _ progress: inout Int) -> Int {
        let set = allocationsSet(register)
        let count = intervals.count
        if progress == 0 && isEmpty(words, set) {
            let needed = wordsNeeded(words, bulkNodes(count))
            if needed != 0 { return needed }
            build(&words, set, intervals, value)
            progress = count
            return 0
        }
        precondition(progress <= count)
        while progress < count {
            let needed = wordsNeeded(words, Int(truncatingIfNeeded: words[heightSlot(set)]) &+ 2)
            if needed != 0 { return needed }
            insert(&words, set, intervals[progress], value)
            progress &+= 1
        }
        return 0
    }

    // RegisterRange::evict.
    static func evict(_ words: inout MutableSpan<UInt64>, _ register: Int, _ intervals: borrowing Span<UInt64>) {
        let set = allocationsSet(register)
        for i in intervals.indices {
            erase(&words, set, intervals[i])
        }
    }

    // RegisterRange::addClobberHigh64: inserts Interval(point) mapped to the register's Tmp. Returns 0, or (changing
    // nothing) the number of words the arena needs.
    static func addClobberHigh64(_ words: inout MutableSpan<UInt64>, _ register: Int, _ point: UInt32, _ value: UInt32) -> Int {
        let set = allocationsSet(register) &+ 1
        let needed = wordsNeeded(words, Int(truncatingIfNeeded: words[heightSlot(set)]) &+ 2)
        if needed != 0 { return needed }
        insert(&words, set, pack(point, point &+ 1), value) // WTF::Range(point)
        return 0
    }

    // GreedyAllocator::tryAllocate's register loop: the preferred register (if not negative), then the others in
    // priority order; the first with no conflict gets the range added (from progress 0; see add()). Returns that
    // register, or -1 if every one conflicts.
    @inline(__always)
    static func tryAllocate(_ words: inout MutableSpan<UInt64>, _ intervals: borrowing Span<UInt64>, _ preferred: Int32, _ order: borrowing Span<UInt8>, _ checkHigh64: Bool, _ value: UInt32, _ progress: inout Int) -> Int32 {
        // i == -1 is the preferred register.
        var i = preferred >= 0 ? -1 : 0
        while i < order.count {
            let register = i < 0 ? preferred : Int32(order[i] & 0x7f) // Reg::m_index is a 7-bit bit-field
            if (i < 0 || register != preferred) && !hasConflict(words.span, Int(register), intervals, checkHigh64) {
                _ = add(&words, Int(register), intervals, value, &progress)
                return register
            }
            i &+= 1
        }
        return -1
    }
}
