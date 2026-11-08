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


// C entry points of CSSKeywords (declared to C++ in CSSKeywordsSwiftBoundary.h). This is the only file of the unit
// with `unsafe`: it wraps the C++-owned keyword table (name pool, name offsets, slots) and the characters (Latin-1
// bytes, or UTF-16 code units read as raw bytes because they may be odd-aligned) in spans and calls the safe core in
// CSSValueKeywordTable.swift. Pointers may be null only when their count is 0.

@_cdecl("WebCoreSwift_CSSKeywords_buildTable")
func WebCoreSwift_CSSKeywords_buildTable(_ pool: UnsafePointer<UInt8>?, _ poolSize: Int, _ offsets: UnsafePointer<UInt32>?, _ offsetCount: Int, _ slots: UnsafeMutablePointer<UInt16>?, _ slotCount: Int) {
    let poolBuffer = unsafe UnsafeRawBufferPointer(start: pool, count: poolSize)
    let offsetBuffer = unsafe UnsafeBufferPointer(start: offsets, count: offsetCount)
    let slotBuffer = unsafe UnsafeMutableBufferPointer(start: slots, count: slotCount)
    var slotSpan = unsafe slotBuffer.mutableSpan
    unsafe CSSValueKeywordTable.buildTable(pool: poolBuffer.bytes, offsets: offsetBuffer.span, slots: &slotSpan)
}

@_cdecl("WebCoreSwift_CSSKeywords_lookup8")
@inline(__always) // Lets the C entry point contain the whole lookup (no further call on the common path).
func WebCoreSwift_CSSKeywords_lookup8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ pool: UnsafePointer<UInt8>?, _ poolSize: Int, _ offsets: UnsafePointer<UInt32>?, _ offsetCount: Int, _ slots: UnsafePointer<UInt16>?, _ slotCount: Int) -> UInt16 {
    let characterBuffer = unsafe UnsafeRawBufferPointer(start: characters, count: length)
    let poolBuffer = unsafe UnsafeRawBufferPointer(start: pool, count: poolSize)
    let offsetBuffer = unsafe UnsafeBufferPointer(start: offsets, count: offsetCount)
    let slotBuffer = unsafe UnsafeBufferPointer(start: slots, count: slotCount)
    return unsafe CSSValueKeywordTable.lookup(CSSValueKeywordLatin1Key(characterBuffer.bytes), pool: poolBuffer.bytes, offsets: offsetBuffer.span, slots: slotBuffer.span)
}

@_cdecl("WebCoreSwift_CSSKeywords_lookup16")
@inline(__always)
func WebCoreSwift_CSSKeywords_lookup16(_ characters: UnsafeRawPointer?, _ length: Int, _ pool: UnsafePointer<UInt8>?, _ poolSize: Int, _ offsets: UnsafePointer<UInt32>?, _ offsetCount: Int, _ slots: UnsafePointer<UInt16>?, _ slotCount: Int) -> UInt16 {
    let characterBuffer = unsafe UnsafeRawBufferPointer(start: characters, count: length &* 2)
    let poolBuffer = unsafe UnsafeRawBufferPointer(start: pool, count: poolSize)
    let offsetBuffer = unsafe UnsafeBufferPointer(start: offsets, count: offsetCount)
    let slotBuffer = unsafe UnsafeBufferPointer(start: slots, count: slotCount)
    return unsafe CSSValueKeywordTable.lookup(CSSValueKeywordUTF16Key(characterBuffer.bytes), pool: poolBuffer.bytes, offsets: offsetBuffer.span, slots: slotBuffer.span)
}
