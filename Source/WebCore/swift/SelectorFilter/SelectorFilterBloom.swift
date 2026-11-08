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


// Memory-safe port of the per-hash loops of css/SelectorFilter.cpp: SelectorFilter::pushParent adds a parent's
// identifier hashes to the ancestor filter, a WTF::CountingBloomFilter<12> (wtf/BloomFilter.h), and
// SelectorFilter::popParent removes them again (and clears the filter when the parent stack becomes empty).
//
// The filter is the CountingBloomFilter's 4096 one-byte buckets, owned by C++. Each hash touches two buckets:
// first = hash & 0xFFF and second = (hash * 0x9e3779b9) & 0xFFF (wrapping 32-bit multiply). A bucket at 255 is
// saturated and never changes until the filter is cleared. The two buckets may be the same one; it is then updated
// twice, the second update reading the first one's result, exactly as CountingBloomFilter::add/remove do through
// two references to the same byte. Removing from an empty bucket is an ASSERT in WTF and wraps to 255 in release
// builds; it does the same here (assert, then wrapping subtract). No allocation, no reference counting; the bucket
// indices are masked to 12 bits and the bucket count is checked once, so the bounds checks fold away.

enum SelectorFilterBloom {
    // CountingBloomFilter<12>::tableSize, keyMask, goldenRatioHash and maximumCount().
    static var tableSize: Int { 4096 }
    static var keyMask: UInt32 { 0xFFF }
    static var goldenRatioHash: UInt32 { 0x9e37_79b9 }
    static var maximumCount: UInt8 { 255 }

    @inline(__always)
    static func firstBucket(_ hash: UInt32) -> Int {
        Int(hash & keyMask)
    }

    @inline(__always)
    static func secondBucket(_ hash: UInt32) -> Int {
        Int((hash &* goldenRatioHash) & keyMask)
    }

    // CountingBloomFilter::add(unsigned) for every hash, in order.
    @inline(__always)
    static func addHashes(_ buckets: inout MutableSpan<UInt8>, _ hashes: Span<UInt32>) {
        precondition(buckets.count == tableSize)
        for index in hashes.indices {
            let hash = hashes[index]
            let first = firstBucket(hash)
            let second = secondBucket(hash)
            // Branch-free increment unless saturated (cmp + cinc); measured faster than the branch here, unlike remove.
            let firstCount = buckets[first]
            buckets[first] = firstCount &+ (firstCount < maximumCount ? 1 : 0)
            let secondCount = buckets[second]
            buckets[second] = secondCount &+ (secondCount < maximumCount ? 1 : 0)
        }
    }

    // CountingBloomFilter::remove(unsigned) for every hash, in order.
    @inline(__always)
    static func removeHashes(_ buckets: inout MutableSpan<UInt8>, _ hashes: Span<UInt32>) {
        precondition(buckets.count == tableSize)
        for index in hashes.indices {
            let hash = hashes[index]
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
