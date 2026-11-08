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

// C entry points for DFGLiveness.swift, declared to C++ in DFGLivenessSwiftBoundary.h and called only from
// dfg/DFGLivenessAnalysisPhase.cpp. Every array is C++-owned, passed as a pointer (null only when its count is 0)
// and an element count; the only unsafe operations turn those into spans and store the scalar out-param. The C++
// glue never passes overlapping arrays. The core is inlined here so that its inout spans stay in registers.

@_cdecl("JSCSwift_DFGLiveness_processBlock")
@inline(__always)
func JSCSwift_DFGLiveness_processBlock(
    _ actions: UnsafePointer<UInt32>?, _ actionCount: Int,
    _ tailRow: UnsafePointer<UInt64>?, _ wordsPerSet: Int,
    _ worksetMap: UnsafeMutablePointer<UInt32>?, _ worksetValues: UnsafeMutablePointer<UInt32>?, _ numIndices: Int,
    _ kill: UnsafeMutablePointer<UInt8>?, _ killCount: Int
) -> Int {
    let actionSpan = unsafe UnsafeBufferPointer(start: actions, count: actionCount).span
    let tailSpan = unsafe UnsafeBufferPointer(start: tailRow, count: wordsPerSet).span
    var map = unsafe UnsafeMutableBufferPointer(start: worksetMap, count: numIndices).mutableSpan
    var values = unsafe UnsafeMutableBufferPointer(start: worksetValues, count: numIndices).mutableSpan
    var killSpan = unsafe UnsafeMutableBufferPointer(start: kill, count: killCount).mutableSpan
    return DFGLiveness.processBlock(actions: actionSpan, tailRow: tailSpan, map: &map, values: &values, kill: &killSpan)
}

@_cdecl("JSCSwift_DFGLiveness_updateHead")
@inline(__always)
func JSCSwift_DFGLiveness_updateHead(
    _ head: UnsafeMutablePointer<UInt32>?, _ oldHeadCount: Int, _ headCapacity: Int,
    _ worksetMap: UnsafeMutablePointer<UInt32>?, _ worksetValues: UnsafeMutablePointer<UInt32>?, _ worksetCount: Int, _ numIndices: Int,
    _ tailMatrix: UnsafeMutablePointer<UInt64>?, _ tailMatrixWords: Int, _ wordsPerSet: Int,
    _ predecessors: UnsafePointer<UInt32>?, _ predecessorCount: Int,
    _ dirtyWords: UnsafeMutablePointer<UInt64>?, _ dirtyWordCount: Int,
    _ worklist: UnsafeMutablePointer<UInt32>?, _ worklistCapacity: Int, _ worklistCount: Int,
    _ newHeadCount: UnsafeMutablePointer<Int>
) -> Int {
    var headSpan = unsafe UnsafeMutableBufferPointer(start: head, count: headCapacity).mutableSpan
    var map = unsafe UnsafeMutableBufferPointer(start: worksetMap, count: numIndices).mutableSpan
    var values = unsafe UnsafeMutableBufferPointer(start: worksetValues, count: numIndices).mutableSpan
    var tail = unsafe UnsafeMutableBufferPointer(start: tailMatrix, count: tailMatrixWords).mutableSpan
    let predecessorSpan = unsafe UnsafeBufferPointer(start: predecessors, count: predecessorCount).span
    var dirty = unsafe UnsafeMutableBufferPointer(start: dirtyWords, count: dirtyWordCount).mutableSpan
    var list = unsafe UnsafeMutableBufferPointer(start: worklist, count: worklistCapacity).mutableSpan
    let result = DFGLiveness.updateHead(
        head: &headSpan, oldHeadCount: oldHeadCount,
        map: &map, values: &values, count: worksetCount,
        tail: &tail, wordsPerSet: wordsPerSet,
        predecessors: predecessorSpan,
        dirty: &dirty, worklist: &list, worklistCount: worklistCount)
    unsafe newHeadCount.pointee = result.head
    return result.worklist
}
