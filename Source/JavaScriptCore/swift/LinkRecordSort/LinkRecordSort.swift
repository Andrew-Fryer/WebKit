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

// Sorts ARM64Assembler::m_jumpsToLink by LinkRecord::from(), replacing
// std::ranges::sort(m_jumpsToLink, { }, &LinkRecord::from) in ARM64Assembler::jumpsToLinkCxx().
//
// Each LinkRecord is 24 bytes, viewed here as three UInt64 words. Word 0 is m_from (int64_t, compared
// signed like ranges::less on intptr_t). Words 1 and 2 (the PAC-tagged m_to and the packed bitfields) are
// moved as opaque data and never decoded.
//
// This is a step-for-step port of the libc++ algorithm the C++ build uses for this call
// (MacOSX27.0.Internal.sdk, <__algorithm/sort.h>: __introsort<_RangeAlgPolicy, ..., LinkRecord*, false>,
// which is not the branchless/bitset variant because LinkRecord is not arithmetic). std::sort is not stable,
// so a different correct sort would give a different order whenever two records have the same `from`.
// Following libc++ step for step means the output is identical byte for byte for every input,
// duplicates included. (An "already sorted" pre-check was measured and dropped: libc++'s
// __insertion_sort_incomplete path already handles sorted input, and the check made small sorts slower.)

enum LinkRecordSort {
    // One record held in registers (libc++'s `value_type __t(std::move(*it))` temporaries).
    struct Record {
        var w0: UInt64
        var w1: UInt64
        var w2: UInt64

        @inline(__always)
        var key: Int64 { Int64(bitPattern: w0) }
    }

    // Upper bound for using insertion sort (libc++ __introsort's __limit).
    static let insertionSortLimit = 24
    // Lower bound for using Tukey's ninther for pivot selection (libc++ __ninther_threshold).
    static let ninthersThreshold = 128

    @inline(__always)
    static func key(_ w: borrowing MutableSpan<UInt64>, _ i: Int) -> Int64 {
        Int64(bitPattern: w[3 &* i])
    }

    // __comp(*a, *b)
    @inline(__always)
    static func less(_ w: borrowing MutableSpan<UInt64>, _ a: Int, _ b: Int) -> Bool {
        key(w, a) < key(w, b)
    }

    @inline(__always)
    static func load(_ w: borrowing MutableSpan<UInt64>, _ i: Int) -> Record {
        let p = 3 &* i
        return Record(w0: w[p], w1: w[p &+ 1], w2: w[p &+ 2])
    }

    @inline(__always)
    static func store(_ w: inout MutableSpan<UInt64>, _ i: Int, _ r: Record) {
        let p = 3 &* i
        w[p] = r.w0
        w[p &+ 1] = r.w1
        w[p &+ 2] = r.w2
    }

    // *dst = std::move(*src)
    @inline(__always)
    static func copy(_ w: inout MutableSpan<UInt64>, to dst: Int, from src: Int) {
        store(&w, dst, load(w, src))
    }

    // _IterOps::iter_swap(a, b)
    @inline(__always)
    static func swapRecords(_ w: inout MutableSpan<UInt64>, _ a: Int, _ b: Int) {
        let ra = load(w, a)
        let rb = load(w, b)
        store(&w, a, rb)
        store(&w, b, ra)
    }

    // __sort3 (branchy variant).
    @inline(__always)
    static func sort3(_ w: inout MutableSpan<UInt64>, _ x: Int, _ y: Int, _ z: Int) {
        if !less(w, y, x) {
            if !less(w, z, y) {
                return
            }
            swapRecords(&w, y, z)
            if less(w, y, x) {
                swapRecords(&w, x, y)
            }
            return
        }
        if less(w, z, y) {
            swapRecords(&w, x, z)
            return
        }
        swapRecords(&w, x, y)
        if less(w, z, y) {
            swapRecords(&w, y, z)
        }
    }

    // __sort4 (branchy variant).
    @inline(__always)
    static func sort4(_ w: inout MutableSpan<UInt64>, _ x1: Int, _ x2: Int, _ x3: Int, _ x4: Int) {
        sort3(&w, x1, x2, x3)
        if less(w, x4, x3) {
            swapRecords(&w, x3, x4)
            if less(w, x3, x2) {
                swapRecords(&w, x2, x3)
                if less(w, x2, x1) {
                    swapRecords(&w, x1, x2)
                }
            }
        }
    }

    // __sort5 (branchy variant).
    @inline(__always)
    static func sort5(_ w: inout MutableSpan<UInt64>, _ x1: Int, _ x2: Int, _ x3: Int, _ x4: Int, _ x5: Int) {
        sort4(&w, x1, x2, x3, x4)
        if less(w, x5, x4) {
            swapRecords(&w, x4, x5)
            if less(w, x4, x3) {
                swapRecords(&w, x3, x4)
                if less(w, x3, x2) {
                    swapRecords(&w, x2, x3)
                    if less(w, x2, x1) {
                        swapRecords(&w, x1, x2)
                    }
                }
            }
        }
    }

    // __insertion_sort: guarded insertion sort of [first, last).
    static func insertionSort(_ w: inout MutableSpan<UInt64>, _ first: Int, _ last: Int) {
        if first == last {
            return
        }
        var i = first &+ 1
        while i != last {
            var j = i &- 1
            if less(w, i, j) {
                let t = load(w, i)
                var k = j
                j = i
                while true {
                    copy(&w, to: j, from: k)
                    j = k
                    if j == first {
                        break
                    }
                    k &-= 1
                    if !(t.key < key(w, k)) {
                        break
                    }
                }
                store(&w, j, t)
            }
            i &+= 1
        }
    }

    // __insertion_sort_unguarded: requires an element at first - 1 that is <= every element of
    // [first, last). If that ever failed, the span bounds check would trap where libc++'s hardened
    // _LIBCPP_ASSERT_VALID_ELEMENT_ACCESS traps.
    static func insertionSortUnguarded(_ w: inout MutableSpan<UInt64>, _ first: Int, _ last: Int) {
        if first == last {
            return
        }
        var i = first &+ 1
        while i != last {
            var j = i &- 1
            if less(w, i, j) {
                let t = load(w, i)
                var k = j
                j = i
                repeat {
                    copy(&w, to: j, from: k)
                    j = k
                    k &-= 1
                } while t.key < key(w, k)
                store(&w, j, t)
            }
            i &+= 1
        }
    }

    // __insertion_sort_incomplete: returns true if [first, last) ended up sorted, false if it gave up
    // after 8 out-of-place elements.
    static func insertionSortIncomplete(_ w: inout MutableSpan<UInt64>, _ first: Int, _ last: Int) -> Bool {
        switch last &- first {
        case 0, 1:
            return true
        case 2:
            if less(w, last &- 1, first) {
                swapRecords(&w, first, last &- 1)
            }
            return true
        case 3:
            sort3(&w, first, first &+ 1, last &- 1)
            return true
        case 4:
            sort4(&w, first, first &+ 1, first &+ 2, last &- 1)
            return true
        case 5:
            sort5(&w, first, first &+ 1, first &+ 2, first &+ 3, last &- 1)
            return true
        default:
            break
        }
        var j = first &+ 2
        sort3(&w, first, first &+ 1, j)
        let limit = 8
        var count = 0
        var i = j &+ 1
        while i != last {
            if less(w, i, j) {
                let t = load(w, i)
                var k = j
                j = i
                while true {
                    copy(&w, to: j, from: k)
                    j = k
                    if j == first {
                        break
                    }
                    k &-= 1
                    if !(t.key < key(w, k)) {
                        break
                    }
                }
                store(&w, j, t)
                count &+= 1
                if count == limit {
                    i &+= 1
                    return i == last
                }
            }
            j = i
            i &+= 1
        }
        return true
    }

    // __partition_with_equals_on_right: *first is the pivot. Returns the pivot's final position and
    // whether the range was already partitioned. Requires last - first >= 3 and a median-of-three
    // pivot (so both scans are guarded).
    static func partitionWithEqualsOnRight(_ w: inout MutableSpan<UInt64>, _ begin: Int, _ end: Int) -> (Int, Bool) {
        var first = begin
        var last = end
        let pivot = load(w, first)
        // Find the first element greater than or equal to the pivot.
        repeat {
            first &+= 1
        } while key(w, first) < pivot.key
        // Find the last element less than the pivot.
        if begin == first &- 1 {
            while first < last {
                last &-= 1
                if key(w, last) < pivot.key {
                    break
                }
            }
        } else {
            repeat {
                last &-= 1
            } while !(key(w, last) < pivot.key)
        }
        let alreadyPartitioned = first >= last
        while first < last {
            swapRecords(&w, first, last)
            repeat {
                first &+= 1
            } while key(w, first) < pivot.key
            repeat {
                last &-= 1
            } while !(key(w, last) < pivot.key)
        }
        let pivotPosition = first &- 1
        if begin != pivotPosition {
            copy(&w, to: begin, from: pivotPosition)
        }
        store(&w, pivotPosition, pivot)
        return (pivotPosition, alreadyPartitioned)
    }

    // __partition_with_equals_on_left: elements equal to the pivot go left. Returns the new first.
    static func partitionWithEqualsOnLeft(_ w: inout MutableSpan<UInt64>, _ begin: Int, _ end: Int) -> Int {
        var first = begin
        var last = end
        let pivot = load(w, first)
        if pivot.key < key(w, last &- 1) {
            repeat {
                first &+= 1
            } while !(pivot.key < key(w, first))
        } else {
            first &+= 1
            while first < last && !(pivot.key < key(w, first)) {
                first &+= 1
            }
        }
        if first < last {
            repeat {
                last &-= 1
            } while pivot.key < key(w, last)
        }
        while first < last {
            swapRecords(&w, first, last)
            repeat {
                first &+= 1
            } while !(pivot.key < key(w, first))
            repeat {
                last &-= 1
            } while pivot.key < key(w, last)
        }
        let pivotPosition = first &- 1
        if begin != pivotPosition {
            copy(&w, to: begin, from: pivotPosition)
        }
        store(&w, pivotPosition, pivot)
        return first
    }

    // __sift_down<_AlgPolicy, /* __assume_both_children */ false> on the heap at `first` of length `len`.
    static func siftDown(_ w: inout MutableSpan<UInt64>, _ first: Int, _ len: Int, _ start0: Int) {
        var start = start0
        var child = start
        if len < 2 || (len &- 2) / 2 < child {
            return
        }
        child = 2 &* child &+ 1
        if child &+ 1 < len && less(w, first &+ child, first &+ child &+ 1) {
            child &+= 1
        }
        if less(w, first &+ child, first &+ start) {
            return
        }
        let top = load(w, first &+ start)
        repeat {
            copy(&w, to: first &+ start, from: first &+ child)
            start = child
            if (len &- 2) / 2 < child {
                break
            }
            child = 2 &* child &+ 1
            if child &+ 1 < len && less(w, first &+ child, first &+ child &+ 1) {
                child &+= 1
            }
        } while !(key(w, first &+ child) < top.key)
        store(&w, first &+ start, top)
    }

    // __floyd_sift_down: returns the hole position. Requires len >= 2.
    static func floydSiftDown(_ w: inout MutableSpan<UInt64>, _ first: Int, _ len: Int) -> Int {
        var hole = first
        var childI = first
        var child = 0
        while true {
            childI &+= child &+ 1
            child = 2 &* child &+ 1
            if child &+ 1 < len && less(w, childI, childI &+ 1) {
                childI &+= 1
                child &+= 1
            }
            copy(&w, to: hole, from: childI)
            hole = childI
            if child > (len &- 2) / 2 {
                return hole
            }
        }
    }

    // __sift_up(first, last, comp, len).
    static func siftUp(_ w: inout MutableSpan<UInt64>, _ first: Int, _ last0: Int, _ len0: Int) {
        var len = len0
        var last = last0
        if len > 1 {
            len = (len &- 2) / 2
            var ptr = first &+ len
            last &-= 1
            if less(w, ptr, last) {
                let t = load(w, last)
                repeat {
                    copy(&w, to: last, from: ptr)
                    last = ptr
                    if len == 0 {
                        break
                    }
                    len = (len &- 1) / 2
                    ptr = first &+ len
                } while key(w, ptr) < t.key
                store(&w, last, t)
            }
        }
    }

    // __partial_sort(first, last, last): __make_heap then __sort_heap. (The middle-to-last loop of
    // __partial_sort_impl is empty when middle == last.)
    static func heapSort(_ w: inout MutableSpan<UInt64>, _ first: Int, _ last0: Int) {
        if first == last0 {
            return
        }
        let n = last0 &- first
        // __make_heap
        if n > 1 {
            var start = (n &- 2) / 2
            while start >= 0 {
                siftDown(&w, first, n, start)
                start &-= 1
            }
        }
        // __sort_heap: __pop_heap(first, last, comp, n) for n = len down to 2.
        var last = last0
        var len = n
        while len > 1 {
            let top = load(w, first)
            var hole = floydSiftDown(&w, first, len)
            let back = last &- 1
            if hole == back {
                store(&w, hole, top)
            } else {
                copy(&w, to: hole, from: back)
                hole &+= 1
                store(&w, back, top)
                siftUp(&w, first, hole, hole &- first)
            }
            last &-= 1
            len &-= 1
        }
    }

    // __introsort<..., /* _UseBitSetPartition */ false>.
    static func introsort(_ w: inout MutableSpan<UInt64>, _ first0: Int, _ last0: Int, _ depth0: Int, _ leftmost0: Bool) {
        var first = first0
        var last = last0
        var depth = depth0
        var leftmost = leftmost0
        while true {
            let len = last &- first
            switch len {
            case 0, 1:
                return
            case 2:
                if less(w, last &- 1, first) {
                    swapRecords(&w, first, last &- 1)
                }
                return
            case 3:
                sort3(&w, first, first &+ 1, last &- 1)
                return
            case 4:
                sort4(&w, first, first &+ 1, first &+ 2, last &- 1)
                return
            case 5:
                sort5(&w, first, first &+ 1, first &+ 2, first &+ 3, last &- 1)
                return
            default:
                break
            }
            if len < insertionSortLimit {
                if leftmost {
                    insertionSort(&w, first, last)
                } else {
                    insertionSortUnguarded(&w, first, last)
                }
                return
            }
            if depth == 0 {
                heapSort(&w, first, last)
                return
            }
            depth &-= 1
            let halfLen = len / 2
            if len > ninthersThreshold {
                sort3(&w, first, first &+ halfLen, last &- 1)
                sort3(&w, first &+ 1, first &+ (halfLen &- 1), last &- 2)
                sort3(&w, first &+ 2, first &+ (halfLen &+ 1), last &- 3)
                sort3(&w, first &+ (halfLen &- 1), first &+ halfLen, first &+ (halfLen &+ 1))
                swapRecords(&w, first, first &+ halfLen)
            } else {
                sort3(&w, first &+ halfLen, first, last &- 1)
            }
            if !leftmost && !less(w, first &- 1, first) {
                first = partitionWithEqualsOnLeft(&w, first, last)
                continue
            }
            let (pivot, alreadyPartitioned) = partitionWithEqualsOnRight(&w, first, last)
            if alreadyPartitioned {
                let leftSorted = insertionSortIncomplete(&w, first, pivot)
                if insertionSortIncomplete(&w, pivot &+ 1, last) {
                    if leftSorted {
                        return
                    }
                    last = pivot
                    continue
                }
                if leftSorted {
                    first = pivot &+ 1
                    continue
                }
            }
            introsort(&w, first, pivot, depth, leftmost)
            leftmost = false
            first = pivot &+ 1
        }
    }

    // ranges::sort(records, { }, &LinkRecord::from) over `count` records stored as 3 * count words.
    static func sortByFrom(_ w: inout MutableSpan<UInt64>, count: Int) {
        precondition(w.count == 3 * count)
        // __sort_dispatch returns early for an empty range and __introsort returns at once for one record.
        if count < 2 {
            return
        }
        // __sort_dispatch: depth limit 2 * floor(log2(count)).
        let depthLimit = 2 &* (Int.bitWidth &- 1 &- count.leadingZeroBitCount)
        introsort(&w, 0, count, depthLimit, true)
    }
}
