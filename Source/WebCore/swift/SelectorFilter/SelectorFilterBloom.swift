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


// Memory-safe port of the ancestor stack of css/SelectorFilter.cpp. Swift owns the identifier hashes of every element
// on the parent stack (SelectorFilter::pushParent's ParentStackFrame::identifierHashes in the original) and does every
// update of the ancestor filter, a WTF::CountingBloomFilter<12> (wtf/BloomFilter.h): pushParent adds a parent's hashes,
// popParent removes them again and clears the filter when the parent stack becomes empty. C++ keeps the element
// pointers (a Vector<CheckedPtr<Element>>, for CheckedPtr's lifetime check), gathers the hashes from the DOM, and
// reads the filter in SelectorFilter::fastRejectSelector.
//
// The filter is the CountingBloomFilter's 4096 one-byte buckets, owned by C++. Each hash touches two buckets:
// first = hash & 0xFFF and second = (hash * 0x9e3779b9) & 0xFFF (wrapping 32-bit multiply). A bucket at 255 is
// saturated and never changes until the filter is cleared. The two buckets may be the same one; it is then updated
// twice, the second update reading the first one's result, exactly as CountingBloomFilter::add/remove do through
// two references to the same byte. Removing from an empty bucket is an ASSERT in WTF and wraps to 255 in release
// builds; it does the same here (assert, then wrapping subtract).
//
// The hash stack lives in 32-bit words that C++ allocates and never interprets (SelectorFilterSwiftBoundary.h):
//   word 0: `used`, the number of words in use after the header (0 when the parent stack is empty);
//   word 1: the number of hashes in the top frame (0 when the parent stack is empty);
//   words 2 ..< 2 + used: the frames, bottom of the stack first. A frame is its element's hashes in order, followed by
//                         one word holding the hash count of the frame below it (0 for the bottom frame).
// The header locates the top frame without a dependent load, so pop's loop can start right away.
// When a push doesn't fit, nothing is changed and the number of words needed is returned; C++ grows the storage
// (keeping its contents) and pushes again. Words past `used` are never read, so C++ needn't initialize them.
// No allocation, no reference counting; bucket indices are masked to 12 bits and the bucket count is checked once, so
// those bounds checks fold away. Every word index is bounds-checked, so damaged storage traps instead of escaping.

enum SelectorFilterBloom {
    // CountingBloomFilter<12>::tableSize, keyMask, goldenRatioHash and maximumCount().
    static var tableSize: Int { 4096 }
    static var keyMask: UInt32 { 0xFFF }
    static var goldenRatioHash: UInt32 { 0x9e37_79b9 }
    static var maximumCount: UInt8 { 255 }

    // WebCoreSwift_SelectorFilter_StackHeaderWords.
    static var headerWords: Int { 2 }

    @inline(__always)
    static func firstBucket(_ hash: UInt32) -> Int {
        Int(hash & keyMask)
    }

    @inline(__always)
    static func secondBucket(_ hash: UInt32) -> Int {
        Int((hash &* goldenRatioHash) & keyMask)
    }

    // SelectorFilter::pushParent: pushes a frame holding `hashes` and adds them to the filter
    // (for (auto hash : hashes) filter.add(hash)). Returns 0, or, when the frame doesn't fit in `stack`, the number of
    // words needed, leaving everything unchanged.
    @inline(__always) // Lets the C entry point contain the whole loop (no frame, no further call).
    static func push(_ buckets: inout MutableSpan<UInt8>, _ stack: inout MutableSpan<UInt32>, _ hashes: Span<UInt32>) -> Int {
        precondition(buckets.count == tableSize)
        precondition(stack.count >= headerWords)
        let used = Int(stack[0])
        let start = headerWords + used
        let linkWord = start + hashes.count
        guard linkWord < stack.count else {
            return linkWord + 1
        }
        for index in hashes.indices {
            let hash = hashes[index]
            stack[start &+ index] = hash
            add(&buckets, hash)
        }
        stack[linkWord] = stack[1]
        stack[1] = UInt32(hashes.count)
        stack[0] = UInt32(used + hashes.count + 1)
        return 0
    }

    // SelectorFilter::popParent: pops the top frame and removes its hashes from the filter
    // (for (auto hash : hashes) filter.remove(hash)); when that empties the stack, clears the filter. Popping an empty
    // stack traps, as Vector::last() does. Returns nothing, so that the clear is a tail call and the common path needs
    // no stack frame.
    @inline(__always) // Lets the C entry point contain the whole loop (no frame, no further call).
    static func pop(_ buckets: inout MutableSpan<UInt8>, _ stack: inout MutableSpan<UInt32>) {
        precondition(buckets.count == tableSize)
        precondition(stack.count >= headerWords)
        let used = stack[0]
        let count = stack[1]
        precondition(count < used, "popParent on an empty or damaged parent stack")
        // The top frame: its hashes and then its link word (one bounds check for all of them).
        let linkWord = headerWords + Int(used) - 1
        let below: UInt32
        do {
            let frame = stack.span.extracting(linkWord - Int(count) ..< linkWord + 1)
            let hashCount = Int(count)
            for index in 0 ..< hashCount {
                remove(&buckets, frame[index])
            }
            below = frame[hashCount]
        }
        let remaining = used &- count &- 1
        stack[1] = below
        stack[0] = remaining
        if remaining == 0 {
            assert(likelyEmpty(buckets))
            clear(&buckets)
        }
    }

    // CountingBloomFilter::add(unsigned).
    @inline(__always)
    static func add(_ buckets: inout MutableSpan<UInt8>, _ hash: UInt32) {
        let first = firstBucket(hash)
        let second = secondBucket(hash)
        // Branch-free increment unless saturated (cmp + cinc); measured faster than the branch here, unlike remove.
        let firstCount = buckets[first]
        buckets[first] = firstCount &+ (firstCount < maximumCount ? 1 : 0)
        let secondCount = buckets[second]
        buckets[second] = secondCount &+ (secondCount < maximumCount ? 1 : 0)
    }

    // CountingBloomFilter::remove(unsigned).
    @inline(__always)
    static func remove(_ buckets: inout MutableSpan<UInt8>, _ hash: UInt32) {
        let first = firstBucket(hash)
        let second = secondBucket(hash)
        let firstCount = buckets[first]
        assert(firstCount != 0)
        if firstCount < maximumCount {
            buckets[first] = firstCount &- 1
        }
        let secondCount = buckets[second]
        assert(secondCount != 0)
        if secondCount < maximumCount {
            buckets[second] = secondCount &- 1
        }
    }

    // CountingBloomFilter::likelyEmpty() (every bucket empty or saturated); only used by assert().
    static func likelyEmpty(_ buckets: borrowing MutableSpan<UInt8>) -> Bool {
        for index in buckets.indices where buckets[index] != 0 && buckets[index] != maximumCount {
            return false
        }
        return true
    }

    // CountingBloomFilter::clear() (popParent, when the parent stack becomes empty).
    @inline(__always)
    static func clear(_ buckets: inout MutableSpan<UInt8>) {
        precondition(buckets.count == tableSize)
        for index in buckets.indices {
            buckets[index] = 0
        }
    }
}
