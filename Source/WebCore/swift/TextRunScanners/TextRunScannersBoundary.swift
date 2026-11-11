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

// C entry points of TextRunScanners.swift, declared to C++ in TextRunScannersSwiftBoundary.h and called only from
// platform/graphics/FontCascade.cpp. The only unsafe operations are wrapping the C++-owned characters and cache words
// in spans. A pointer may be null only when its count is 0. UTF-16 may be odd-aligned: it is then read as raw bytes.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

@_cdecl("WebCoreSwift_TextRunScanners_characterRangeCodePath")
func WebCoreSwift_TextRunScanners_characterRangeCodePath(_ characters: UnsafeRawPointer?, _ length: Int) -> UInt8 {
    if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        return TextRunScanners.characterRangeCodePath(TextRunUTF16Units(units))
    }
    let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
    return TextRunScanners.characterRangeCodePath(TextRunUnalignedUTF16Units(bytes))
}

@_cdecl("WebCoreSwift_TextRunScanners_scanSimplifiedMeasuring")
func WebCoreSwift_TextRunScanners_scanSimplifiedMeasuring(_ characters: UnsafeRawPointer?, _ length: Int, _ is16Bit: Bool, _ cache: UnsafePointer<UInt64>?, _ cacheWordCount: Int, _ whitespaceIsCollapsed: Bool) -> Int {
    let cacheWords = unsafe UnsafeBufferPointer(start: cache, count: cacheWordCount).span
    if !is16Bit {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span
        return TextRunScanners.scanSimplifiedMeasuring(TextRunLatin1Units(units), cache: cacheWords, whitespaceIsCollapsed: whitespaceIsCollapsed)
    }
    if Int(bitPattern: characters) & 1 == 0 {
        let units = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt16.self), count: length).span
        return TextRunScanners.scanSimplifiedMeasuring(TextRunUTF16Units(units), cache: cacheWords, whitespaceIsCollapsed: whitespaceIsCollapsed)
    }
    let bytes = unsafe UnsafeRawBufferPointer(start: characters, count: length * 2).bytes
    return TextRunScanners.scanSimplifiedMeasuring(TextRunUnalignedUTF16Units(bytes), cache: cacheWords, whitespaceIsCollapsed: whitespaceIsCollapsed)
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
