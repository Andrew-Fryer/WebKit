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


// C entry points of the CSSTokenizer kernel (declared to C++ in CSSTokenizerSwiftBoundary.h). This is the only file
// of the unit with `unsafe`: it wraps the input and the C++-owned buffers (the record chunk, and the block stack,
// pool, registrations and offsets whose addresses are in the state words) in spans, and calls the safe code.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

@_cdecl("WebCoreSwift_CSSTokenizer_tokenize")
func WebCoreSwift_CSSTokenizer_tokenize(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ tokens: UnsafeMutablePointer<UInt64>?,
    _ tokenCapacity: Int, _ state: UnsafeMutablePointer<UInt64>?) -> UInt64
{
    typealias K = CSSTokenizerKernel
    var stateSpan = unsafe UnsafeMutableBufferPointer(start: state, count: K.stateCount).mutableSpan
    let inputBase = UInt64(UInt(bitPattern: characters))
    let tokenSpan = unsafe UnsafeMutableRawBufferPointer(start: tokens, count: tokenCapacity &* 24).mutableBytes
    let blockStack = unsafe UnsafeMutableBufferPointer(start: UnsafeMutablePointer<UInt8>(bitPattern: UInt(stateSpan[4])), count: Int(stateSpan[5])).mutableSpan
    let pool = unsafe UnsafeMutableBufferPointer(start: UnsafeMutablePointer<UInt16>(bitPattern: UInt(stateSpan[6])), count: Int(stateSpan[7])).mutableSpan
    let registrations = unsafe UnsafeMutableBufferPointer(start: UnsafeMutablePointer<UInt64>(bitPattern: UInt(stateSpan[8])), count: Int(stateSpan[9])).mutableSpan
    let observing = (stateSpan[K.stateFlags] & K.flagObserving) != 0
    let offsets = unsafe UnsafeMutableBufferPointer(start: UnsafeMutablePointer<UInt32>(bitPattern: UInt(stateSpan[10])), count: observing ? tokenCapacity : 0).mutableSpan
    if is8Bit {
        let input = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        return CSSTokenizerKernel.tokenize(CSSTokenizerLatin1(input), is8Bit: true, inputBase: inputBase, tokens: tokenSpan, blockStack: blockStack,
            pool: pool, registrations: registrations, offsets: offsets, observing: observing, state: &stateSpan)
    }
    let input = unsafe UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes
    return CSSTokenizerKernel.tokenize(CSSTokenizerUTF16(input), is8Bit: false, inputBase: inputBase, tokens: tokenSpan, blockStack: blockStack,
        pool: pool, registrations: registrations, offsets: offsets, observing: observing, state: &stateSpan)
}

@_cdecl("WebCoreSwift_CSSTokenizer_preprocess")
func WebCoreSwift_CSSTokenizer_preprocess(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ output: UnsafeMutablePointer<UInt16>?) {
    var outputSpan = unsafe UnsafeMutableBufferPointer(start: output, count: length).mutableSpan
    if is8Bit {
        CSSTokenizerKernel.preprocess(unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span, into: &outputSpan)
        return
    }
    CSSTokenizerKernel.preprocess(CSSTokenizerUTF16(unsafe UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes), into: &outputSpan)
}

@_cdecl("WebCoreSwift_CSSTokenizer_unitType")
func WebCoreSwift_CSSTokenizer_unitType(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool) -> UInt8 {
    if is8Bit {
        let input = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        return CSSTokenizerUnits.unitType(CSSTokenizerLatin1(input), 0, length)
    }
    let input = unsafe UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes
    return CSSTokenizerUnits.unitType(CSSTokenizerUTF16(input), 0, length)
}

#endif
