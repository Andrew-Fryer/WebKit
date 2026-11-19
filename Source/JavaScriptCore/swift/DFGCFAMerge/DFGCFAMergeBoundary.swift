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

// C entry points for DFGCFAMerge.swift, declared to C++ in DFGCFAMergeSwiftBoundary.h and called only from
// dfg/DFGInPlaceAbstractState.cpp. Operands<AbstractValue> arrays come as a pointer to the first element (null only
// when empty) and an element count, an element being four 64-bit words; Operands<Node*> as a pointer and a count of
// one-word elements. The only unsafe operations turn those, and the slow-index buffer, into spans and write the
// slow count. The C++ glue never passes overlapping arrays
// (to->valuesAtHead and from->valuesAtTail are distinct Operands even when from == to).

@_cdecl("JSCSwift_DFGCFAMerge_mergeValues")
@inline(__always)
func JSCSwift_DFGCFAMerge_mergeValues(_ destination: UnsafeMutablePointer<UInt64>?, _ destinationCount: Int, _ source: UnsafePointer<UInt64>?, _ sourceCount: Int, _ destinationNodes: UnsafePointer<UInt64>?, _ destinationNodeCount: Int, _ checkNodes: Bool, _ count: Int, _ slowIndices: UnsafeMutablePointer<UInt32>?, _ slowCapacity: Int, _ slowCount: UnsafeMutablePointer<Int>) -> Bool {
    // The element-to-word conversions trap on overflow (a precondition and then `&*`, which, unlike a checked `*`,
    // leaves the loop's bounds checks hoistable).
    precondition(UInt(bitPattern: destinationCount) <= UInt(Int.max / 4) && UInt(bitPattern: sourceCount) <= UInt(Int.max / 4))
    var destinationWords = unsafe UnsafeMutableBufferPointer(start: destination, count: 4 &* destinationCount).mutableSpan
    let sourceWords = unsafe UnsafeBufferPointer(start: source, count: 4 &* sourceCount).span
    let nodes = unsafe UnsafeBufferPointer(start: destinationNodes, count: destinationNodeCount).span
    var slow = unsafe UnsafeMutableBufferPointer(start: slowIndices, count: slowCapacity).mutableSpan
    var slowFound = 0
    let changed = DFGCFAMerge.mergeValues(&destinationWords, sourceWords, destinationNodes: nodes, checkNodes: checkNodes, count: count, slowIndices: &slow, slowCount: &slowFound)
    unsafe slowCount.pointee = slowFound
    return changed
}

// One value's merge that needs a structure list: the destination and source values as four words each, the elements
// of their fat sets' lists (null and 0 for a thin set), the destination list's capacity, a scratch list for the
// result's elements, and three result words. The unsafe operations turn those into spans.
@_cdecl("JSCSwift_DFGCFAMerge_mergeSlowValue")
func JSCSwift_DFGCFAMerge_mergeSlowValue(_ destination: UnsafeMutablePointer<UInt64>, _ source: UnsafePointer<UInt64>,
                                         _ destinationList: UnsafePointer<UInt64>?, _ destinationLength: Int, _ destinationCapacity: UInt32,
                                         _ sourceList: UnsafePointer<UInt64>?, _ sourceLength: Int,
                                         _ list: UnsafeMutablePointer<UInt64>?, _ listCapacity: Int,
                                         _ result: UnsafeMutablePointer<UInt64>) -> Bool {
    var destinationWords = unsafe UnsafeMutableBufferPointer(start: destination, count: 4).mutableSpan
    let sourceWords = unsafe UnsafeBufferPointer(start: source, count: 4).span
    let destinationElements = unsafe UnsafeBufferPointer(start: destinationList, count: destinationLength).span
    let sourceElements = unsafe UnsafeBufferPointer(start: sourceList, count: sourceLength).span
    var listSpan = unsafe UnsafeMutableBufferPointer(start: list, count: listCapacity).mutableSpan
    var resultWords = unsafe UnsafeMutableBufferPointer(start: result, count: 3).mutableSpan
    return DFGCFAMerge.mergeSlowValue(&destinationWords, sourceWords, destinationList: destinationElements, destinationCapacity: destinationCapacity,
                                      sourceList: sourceElements, list: &listSpan, result: &resultWords)
}
