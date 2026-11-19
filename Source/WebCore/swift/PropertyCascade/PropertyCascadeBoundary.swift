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

// C entry point for PropertyCascade (declared to C++ in PropertyCascadeSwiftBoundary.h). This is the only
// file of the unit with `unsafe`: it wraps the C++-owned buffers in spans and, per match, the storage
// address the glue gathered (word 0 of each match) and the byte count the kernel derives from the match's
// property count (word 2) in a RawSpan.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

// The property storage of the matches, read from the glue's match words. Only valid during the call.
@unsafe struct PropertyCascadeMatchStorageFromGlue: PropertyCascadeMatchStorage {
    let matches: UnsafeBufferPointer<UInt64>

    @_lifetime(borrow self)
    func bytes(_ match: Int, byteCount: Int) -> RawSpan {
        let word = PropertyCascade.matchWordCount * match
        let count = unsafe matches.count
        precondition(match >= 0 && word + 2 < count && byteCount >= 0)
        let start = unsafe UnsafeRawPointer(bitPattern: UInt(unsafe matches[word]))
        let buffer = unsafe UnsafeRawBufferPointer(start: start, count: byteCount)
        return unsafe _overrideLifetime(buffer.bytes, borrowing: self)
    }
}

@_cdecl("WebCoreSwift_PropertyCascade_build")
func WebCoreSwift_PropertyCascade_build(_ matches: UnsafePointer<UInt64>?, _ matchWordCount: Int, _ state: UnsafeMutablePointer<UInt32>?, _ stateCount: Int,
    _ includedIDs: UnsafePointer<UInt16>?, _ includedIDCount: Int, _ table: UnsafePointer<UInt32>?, _ tableCount: Int,
    _ inputs: UnsafePointer<UInt64>?, _ inputCount: Int,
    _ properties: UnsafeMutableRawPointer?, _ propertyBytes: Int, _ propertyIsPresent: UnsafeMutablePointer<UInt64>?, _ propertyIsPresentWordCount: Int,
    _ logicalGroupIndices: UnsafeMutablePointer<UInt32>?, _ logicalGroupIDs: UnsafeMutablePointer<UInt16>?, _ logicalGroupCount: Int,
    _ work: UnsafeMutablePointer<UInt64>?, _ workCount: Int) -> Int
{
    let matchBuffer = unsafe UnsafeBufferPointer(start: matches, count: matchWordCount)
    let storage = unsafe PropertyCascadeMatchStorageFromGlue(matches: matchBuffer)
    let includedIDBuffer = unsafe UnsafeBufferPointer(start: includedIDs, count: includedIDCount)
    let tableBuffer = unsafe UnsafeBufferPointer(start: table, count: tableCount)
    let inputBuffer = unsafe UnsafeBufferPointer(start: inputs, count: inputCount)
    let propertyBuffer = unsafe UnsafeMutableRawBufferPointer(start: properties, count: propertyBytes)
    let presentBuffer = unsafe UnsafeMutableBufferPointer(start: propertyIsPresent, count: propertyIsPresentWordCount)
    let indexBuffer = unsafe UnsafeMutableBufferPointer(start: logicalGroupIndices, count: logicalGroupCount)
    let idBuffer = unsafe UnsafeMutableBufferPointer(start: logicalGroupIDs, count: logicalGroupCount)
    let workBuffer = unsafe UnsafeMutableBufferPointer(start: work, count: workCount)
    var logicalIDSpan = unsafe idBuffer.mutableSpan
    var stateSpan = unsafe UnsafeMutableBufferPointer(start: state, count: stateCount).mutableSpan
    return unsafe PropertyCascade.build(storage: storage, matches: matchBuffer.span, state: &stateSpan, includedIDs: includedIDBuffer.span,
        table: tableBuffer.span, inputs: inputBuffer.span, records: propertyBuffer.mutableBytes, present: presentBuffer.mutableSpan,
        logicalIndices: indexBuffer.mutableSpan, logicalIDs: &logicalIDSpan, work: workBuffer.mutableSpan)
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
