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

// C entry point for AvailabilityLocals.swift, declared to C++ in AvailabilityLocalsSwiftBoundary.h and called only
// from dfg/DFGAvailabilityMap.cpp. It takes Operands<Availability> arrays as a pointer to the first element (null
// only when empty) and an element count; an element is two 64-bit words. The only unsafe operations turn those
// pairs into spans; the equality test views the target as a
// read-only Span, since the C++ passes a const map's storage. The C++ glue never passes overlapping target and
// source arrays to a merge.

@_cdecl("JSCSwift_AvailabilityLocals_apply")
@inline(__always)
func JSCSwift_AvailabilityLocals_apply(_ operation: UInt32, _ target: UnsafeMutablePointer<UInt64>?, _ targetCount: Int, _ source: UnsafePointer<UInt64>?, _ sourceCount: Int) -> Bool {
    // The element-to-word conversions trap on overflow (a precondition and then `&*`, which, unlike a checked `*`,
    // leaves fillWithDead's loop vectorizable).
    precondition(UInt(bitPattern: targetCount) <= UInt(Int.max / 2) && UInt(bitPattern: sourceCount) <= UInt(Int.max / 2))
    let sourceWords = unsafe UnsafeBufferPointer(start: source, count: 2 &* sourceCount).span
    if operation == AvailabilityLocals.equalOperation {
        let targetWords = unsafe UnsafeBufferPointer(start: UnsafePointer(target), count: 2 &* targetCount).span
        return AvailabilityLocals.equal(targetWords, sourceWords)
    }
    var targetWords = unsafe UnsafeMutableBufferPointer(start: target, count: 2 &* targetCount).mutableSpan
    AvailabilityLocals.update(operation, &targetWords, sourceWords)
    return false
}
