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

// C entry points for ButterflyFill.swift, declared to C++ in ButterflyFillSwiftBoundary.h and called only from
// runtime/JSObject.cpp. Each takes a butterfly's indexed payload (or a sub-range of it) as a pointer and an
// element count, and the only unsafe operation is turning that pair into a MutableSpan. The pointer may be null
// only when the count is 0.

@_cdecl("JSCSwift_ButterflyFill_fillWithPureNaN")
@inline(__always)
func JSCSwift_ButterflyFill_fillWithPureNaN(_ data: UnsafeMutablePointer<UInt64>?, _ count: Int) {
    var elements = unsafe UnsafeMutableBufferPointer(start: data, count: count).mutableSpan
    ButterflyFill.fillWithPureNaN(&elements)
}

@_cdecl("JSCSwift_ButterflyFill_fillWithEmptyValue")
@inline(__always)
func JSCSwift_ButterflyFill_fillWithEmptyValue(_ data: UnsafeMutablePointer<UInt64>?, _ count: Int) {
    var elements = unsafe UnsafeMutableBufferPointer(start: data, count: count).mutableSpan
    ButterflyFill.fillWithEmptyValue(&elements)
}

@_cdecl("JSCSwift_ButterflyFill_int32ToDouble")
@inline(__always)
func JSCSwift_ButterflyFill_int32ToDouble(_ data: UnsafeMutablePointer<UInt64>?, _ count: Int) {
    var elements = unsafe UnsafeMutableBufferPointer(start: data, count: count).mutableSpan
    ButterflyFill.int32ToDouble(&elements)
}

@_cdecl("JSCSwift_ButterflyFill_doubleToJSValue")
@inline(__always)
func JSCSwift_ButterflyFill_doubleToJSValue(_ data: UnsafeMutablePointer<UInt64>?, _ count: Int) {
    var elements = unsafe UnsafeMutableBufferPointer(start: data, count: count).mutableSpan
    ButterflyFill.doubleToJSValue(&elements)
}
