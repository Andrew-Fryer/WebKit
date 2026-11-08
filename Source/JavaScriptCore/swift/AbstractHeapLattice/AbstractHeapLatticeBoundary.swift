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

// C entry points for AbstractHeapLattice.swift, declared to C++ in AbstractHeapLatticeSwiftBoundary.h and
// called only from DFGAbstractHeap.cpp. An AbstractHeap crosses the boundary as its raw 64-bit m_value, so
// there are no pointers and nothing here is unsafe.
//
// @inline(__always) puts the whole body in the C entry point (otherwise it is a thunk that calls the Swift
// function), which keeps the fast path free of a stack frame. The predicates return 0 or 1.

@_cdecl("JSCSwift_AbstractHeapLattice_isStrictSubtypeOf")
@inline(__always)
func JSCSwift_AbstractHeapLattice_isStrictSubtypeOf(_ heap: Int64, _ other: Int64) -> Int {
    AbstractHeapLattice.isStrictSubtypeOf(UInt64(bitPattern: heap), UInt64(bitPattern: other))
}

@_cdecl("JSCSwift_AbstractHeapLattice_overlaps")
@inline(__always)
func JSCSwift_AbstractHeapLattice_overlaps(_ heap: Int64, _ other: Int64) -> Int {
    AbstractHeapLattice.overlaps(UInt64(bitPattern: heap), UInt64(bitPattern: other))
}

@_cdecl("JSCSwift_AbstractHeapLattice_supertype")
@inline(__always)
func JSCSwift_AbstractHeapLattice_supertype(_ heap: Int64) -> Int64 {
    Int64(bitPattern: AbstractHeapLattice.supertype(UInt64(bitPattern: heap)))
}
