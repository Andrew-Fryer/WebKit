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

// C entry points for AvailabilityLocals.swift, declared to C++ in AvailabilityLocalsSwiftBoundary.h and called
// only from dfg/DFGAvailabilityMap.cpp. Each takes an Operands<Availability> as a pointer to its first element
// (null only when empty) and an element count; an element is two 64-bit words. The only unsafe operations turn
// those pairs into spans. The C++ glue never passes overlapping target and source arrays.

@_cdecl("JSCSwift_AvailabilityLocals_equal")
@inline(__always)
func JSCSwift_AvailabilityLocals_equal(_ a: UnsafePointer<UInt64>?, _ b: UnsafePointer<UInt64>?, _ count: Int) -> Bool {
    let aWords = unsafe UnsafeBufferPointer(start: a, count: 2 &* count).span
    let bWords = unsafe UnsafeBufferPointer(start: b, count: 2 &* count).span
    return AvailabilityLocals.equal(aWords, bWords)
}

@_cdecl("JSCSwift_AvailabilityLocals_merge")
@inline(__always)
func JSCSwift_AvailabilityLocals_merge(_ target: UnsafeMutablePointer<UInt64>?, _ targetCount: Int, _ other: UnsafePointer<UInt64>?, _ otherCount: Int) {
    var targetWords = unsafe UnsafeMutableBufferPointer(start: target, count: 2 &* targetCount).mutableSpan
    let otherWords = unsafe UnsafeBufferPointer(start: other, count: 2 &* otherCount).span
    AvailabilityLocals.merge(&targetWords, otherWords)
}

@_cdecl("JSCSwift_AvailabilityLocals_fillWithDead")
@inline(__always)
func JSCSwift_AvailabilityLocals_fillWithDead(_ target: UnsafeMutablePointer<UInt64>?, _ count: Int) {
    var targetWords = unsafe UnsafeMutableBufferPointer(start: target, count: 2 &* count).mutableSpan
    AvailabilityLocals.fillWithDead(&targetWords)
}
