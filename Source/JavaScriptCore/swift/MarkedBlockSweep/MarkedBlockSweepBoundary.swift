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

// C entry points for MarkedBlockSweep.swift and MarkedBlockStopAllocating.swift, declared to C++ in
// MarkedBlockSweepSwiftBoundary.h and called only from heap/MarkedBlock.cpp. The unsafe operations are turning
// (pointer, count) pairs into spans and writing the scalar results. The payload pointer is 16-byte aligned
// (MarkedBlock atoms): the sweep views it as atoms of two UInt64, stopAllocating as atoms of four UInt32. The
// bitmaps and the deref buffer are 8-byte aligned.

@_cdecl("JSCSwift_MarkedBlockSweep_sweep")
func JSCSwift_MarkedBlockSweep_sweep(_ payload: UnsafeMutableRawPointer?, _ payloadAtoms: Int,
                                     _ liveWords: UnsafePointer<UInt64>?, _ liveWordCount: Int,
                                     _ startAtom: UInt32, _ atomsPerCell: UInt32, _ secret: UInt64, _ flags: UInt32,
                                     _ derefs: UnsafeMutablePointer<UInt64>?, _ derefCapacity: Int,
                                     _ outHeadOffset: UnsafeMutablePointer<Int64>, _ outFreedBytes: UnsafeMutablePointer<UInt64>) -> Int {
    let atomBase = unsafe payload?.bindMemory(to: MarkedBlockAtom.self, capacity: payloadAtoms)
    let atoms = unsafe UnsafeMutableBufferPointer(start: atomBase, count: payloadAtoms).mutableSpan
    let live = unsafe UnsafeBufferPointer(start: liveWords, count: liveWordCount).span
    precondition(derefCapacity == 1024)
    let derefBase = unsafe derefs.map { unsafe UnsafeMutableRawPointer($0).bindMemory(to: MarkedBlockDerefs.self, capacity: 1) }
    let derefSpan = unsafe UnsafeMutableBufferPointer(start: derefBase, count: 1).mutableSpan
    let destroyKind: MarkedBlockSweepDestroyKind
    switch (flags >> 2) & 3 {
    case 1: destroyKind = .string
    case 2: destroyKind = .rope
    default: preconditionFailure("unknown MarkedBlockSweep destroy kind")
    }
    let result = sweepMarkedBlock(atoms, live: live, allDead: unsafe liveWords == nil, startAtom: startAtom,
                                  atomsPerCell: atomsPerCell, secret: secret, toFreeList: flags & 1 != 0, scribble: flags & 2 != 0,
                                  destroyKind: destroyKind, derefs: derefSpan)
    unsafe outHeadOffset.pointee = Int64(result.headOffset)
    unsafe outFreedBytes.pointee = UInt64(result.freedBytes)
    return result.derefCount
}

@_cdecl("JSCSwift_MarkedBlockSweep_stopAllocating")
func JSCSwift_MarkedBlockSweep_stopAllocating(_ payload: UnsafeMutableRawPointer?, _ payloadAtoms: Int,
                                              _ newlyAllocatedWords: UnsafeMutablePointer<UInt64>?, _ wordCount: Int,
                                              _ startAtom: UInt32, _ atomsPerCell: UInt32, _ endAtom: UInt32,
                                              _ intervalStart: Int64, _ intervalEnd: Int64, _ nextInterval: Int64,
                                              _ secret: UInt64, _ needsZapping: Bool) {
    let atomBase = unsafe payload?.bindMemory(to: MarkedBlockAtomWords.self, capacity: payloadAtoms)
    var atoms = unsafe UnsafeMutableBufferPointer(start: atomBase, count: payloadAtoms).mutableSpan
    var words = unsafe UnsafeMutableBufferPointer(start: newlyAllocatedWords, count: wordCount).mutableSpan
    stopAllocatingMarkedBlock(&atoms, newlyAllocated: &words, startAtom: startAtom, atomsPerCell: atomsPerCell,
                              endAtom: endAtom, intervalStart: Int(intervalStart), intervalEnd: Int(intervalEnd),
                              nextInterval: Int(nextInterval), secret: secret, needsZapping: needsZapping)
}
