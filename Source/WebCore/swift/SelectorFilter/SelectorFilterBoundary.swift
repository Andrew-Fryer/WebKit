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


// C entry points of SelectorFilter (declared to C++ in SelectorFilterSwiftBoundary.h). This is the only file of the
// unit with `unsafe`: it wraps the CountingBloomFilter's buckets and the parent frame's identifier hashes, both owned
// by C++, in spans and calls the safe core in SelectorFilterBloom.swift. `hashes` may be null only when `hashCount`
// is 0.

@_cdecl("WebCoreSwift_SelectorFilter_addHashes")
@inline(__always) // Lets the C entry point contain the whole loop (no frame, no further call).
func WebCoreSwift_SelectorFilter_addHashes(_ buckets: UnsafeMutablePointer<UInt8>?, _ bucketCount: Int, _ hashes: UnsafePointer<UInt32>?, _ hashCount: Int) {
    let bucketBuffer = unsafe UnsafeMutableBufferPointer(start: buckets, count: bucketCount)
    let hashBuffer = unsafe UnsafeBufferPointer(start: hashes, count: hashCount)
    var bucketSpan = unsafe bucketBuffer.mutableSpan
    unsafe SelectorFilterBloom.addHashes(&bucketSpan, hashBuffer.span)
}

@_cdecl("WebCoreSwift_SelectorFilter_removeHashes")
@inline(__always)
func WebCoreSwift_SelectorFilter_removeHashes(_ buckets: UnsafeMutablePointer<UInt8>?, _ bucketCount: Int, _ hashes: UnsafePointer<UInt32>?, _ hashCount: Int) {
    let bucketBuffer = unsafe UnsafeMutableBufferPointer(start: buckets, count: bucketCount)
    let hashBuffer = unsafe UnsafeBufferPointer(start: hashes, count: hashCount)
    var bucketSpan = unsafe bucketBuffer.mutableSpan
    unsafe SelectorFilterBloom.removeHashes(&bucketSpan, hashBuffer.span)
}

@_cdecl("WebCoreSwift_SelectorFilter_clear")
func WebCoreSwift_SelectorFilter_clear(_ buckets: UnsafeMutablePointer<UInt8>?, _ bucketCount: Int) {
    let bucketBuffer = unsafe UnsafeMutableBufferPointer(start: buckets, count: bucketCount)
    var bucketSpan = unsafe bucketBuffer.mutableSpan
    SelectorFilterBloom.clear(&bucketSpan)
}
