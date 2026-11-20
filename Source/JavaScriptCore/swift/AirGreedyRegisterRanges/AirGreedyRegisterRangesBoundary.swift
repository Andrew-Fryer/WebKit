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

// C entry points for AirGreedyRegisterRanges.swift, declared to C++ in AirGreedyRegisterRangesSwiftBoundary.h and
// called only from b3/air/AirAllocateRegistersByGreedy.cpp. The arena is a pointer to its words and their count; a
// LiveRange's intervals are a pointer (null only when empty, 8-byte aligned) to words begin | end << 32 and their
// number; a register priority order is a pointer to Reg indices (one byte each) and their number. The only unsafe
// operations turn those into spans and read and write the scalar out-parameters.

@_cdecl("JSCSwift_AirGreedyRegisterRanges_tryAllocate")
func JSCSwift_AirGreedyRegisterRanges_tryAllocate(_ arena: UnsafeMutablePointer<UInt64>, _ arenaWords: Int, _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int,
                                                  _ preferred: Int32, _ order: UnsafePointer<UInt8>?, _ orderCount: Int, _ checkHigh64: Bool, _ tmp: UInt32,
                                                  _ progress: UnsafeMutablePointer<Int>) -> Int32 {
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    var words = unsafe UnsafeMutableBufferPointer(start: arena, count: arenaWords).mutableSpan
    let orderSpan = unsafe UnsafeBufferPointer(start: order, count: orderCount).span
    var done = 0
    let result = AirGreedyRegisterRanges.tryAllocate(&words, intervalSpan, preferred, orderSpan, checkHigh64, tmp, &done)
    unsafe progress.pointee = done
    return result
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_add")
func JSCSwift_AirGreedyRegisterRanges_add(_ arena: UnsafeMutablePointer<UInt64>, _ arenaWords: Int, _ register: UInt32, _ tmp: UInt32,
                                          _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int, _ progress: UnsafeMutablePointer<Int>) -> Int {
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    var words = unsafe UnsafeMutableBufferPointer(start: arena, count: arenaWords).mutableSpan
    var done = unsafe progress.pointee
    let needed = AirGreedyRegisterRanges.add(&words, Int(register), intervalSpan, tmp, &done)
    unsafe progress.pointee = done
    return needed
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_evict")
func JSCSwift_AirGreedyRegisterRanges_evict(_ arena: UnsafeMutablePointer<UInt64>, _ arenaWords: Int, _ register: UInt32, _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int) {
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    var words = unsafe UnsafeMutableBufferPointer(start: arena, count: arenaWords).mutableSpan
    AirGreedyRegisterRanges.evict(&words, Int(register), intervalSpan)
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_addClobberHigh64")
func JSCSwift_AirGreedyRegisterRanges_addClobberHigh64(_ arena: UnsafeMutablePointer<UInt64>, _ arenaWords: Int, _ register: UInt32, _ point: UInt32, _ tmp: UInt32) -> Int {
    var words = unsafe UnsafeMutableBufferPointer(start: arena, count: arenaWords).mutableSpan
    return AirGreedyRegisterRanges.addClobberHigh64(&words, Int(register), point, tmp)
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_conflicts")
func JSCSwift_AirGreedyRegisterRanges_conflicts(_ arena: UnsafePointer<UInt64>, _ arenaWords: Int, _ register: UInt32, _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int,
                                                _ sets: UInt32, _ cursor: UnsafeMutablePointer<UInt64>, _ conflicts: UnsafeMutablePointer<UInt32>, _ capacity: Int) -> Int {
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    precondition(UInt(bitPattern: capacity) <= UInt(Int.max / 3))
    let words = unsafe UnsafeBufferPointer(start: arena, count: arenaWords).span
    var out = unsafe UnsafeMutableBufferPointer(start: conflicts, count: 3 &* capacity).mutableSpan
    var position = unsafe cursor.pointee
    let written = AirGreedyRegisterRanges.conflicts(words, Int(register), intervalSpan, sets, &position, &out)
    unsafe cursor.pointee = position
    return written
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_firstConflicts")
func JSCSwift_AirGreedyRegisterRanges_firstConflicts(_ arena: UnsafePointer<UInt64>, _ arenaWords: Int, _ order: UnsafePointer<UInt8>?, _ orderCount: Int,
                                                     _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int, _ sets: UInt32,
                                                     _ cursors: UnsafeMutablePointer<UInt64>?, _ counts: UnsafeMutablePointer<UInt8>?, _ conflicts: UnsafeMutablePointer<UInt32>?, _ capacity: Int) {
    precondition(UInt(bitPattern: orderCount) <= UInt(UInt8.max) && UInt(bitPattern: capacity) <= UInt(UInt8.max))
    let words = unsafe UnsafeBufferPointer(start: arena, count: arenaWords).span
    let orderSpan = unsafe UnsafeBufferPointer(start: order, count: orderCount).span
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    var cursorSpan = unsafe UnsafeMutableBufferPointer(start: cursors, count: orderCount).mutableSpan
    var countSpan = unsafe UnsafeMutableBufferPointer(start: counts, count: orderCount).mutableSpan
    var out = unsafe UnsafeMutableBufferPointer(start: conflicts, count: 3 &* capacity &* orderCount).mutableSpan
    AirGreedyRegisterRanges.firstConflicts(words, orderSpan, intervalSpan, sets, &cursorSpan, &countSpan, &out, capacity: capacity)
}

@_cdecl("JSCSwift_AirGreedyRegisterRanges_cheapestEviction")
func JSCSwift_AirGreedyRegisterRanges_cheapestEviction(_ arena: UnsafePointer<UInt64>, _ arenaWords: Int, _ order: UnsafePointer<UInt8>?, _ orderCount: Int,
                                                       _ intervals: UnsafePointer<UInt64>?, _ intervalCount: Int, _ sets: UInt32, _ lastGPRTmp: Int32, _ lastFPRTmp: Int32, _ minSpillCost: UnsafeMutablePointer<Float>,
                                                       _ cost: @convention(c) (UnsafeMutableRawPointer?, UInt32, Int) -> UInt64, _ context: UnsafeMutableRawPointer?) -> Int32 {
    let words = unsafe UnsafeBufferPointer(start: arena, count: arenaWords).span
    let orderSpan = unsafe UnsafeBufferPointer(start: order, count: orderCount).span
    let intervalSpan = unsafe UnsafeBufferPointer(start: intervals, count: intervalCount).span
    var minimum = unsafe minSpillCost.pointee
    let best = AirGreedyRegisterRanges.cheapestEviction(words, orderSpan, intervalSpan, sets, lastGPR: lastGPRTmp, lastFPR: lastFPRTmp, &minimum) { tmp, orderIndex in
        unsafe cost(context, tmp, orderIndex)
    }
    unsafe minSpillCost.pointee = minimum
    return best
}
