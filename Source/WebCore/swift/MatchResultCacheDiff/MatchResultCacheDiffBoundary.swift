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

// C entry point of MatchResultCacheDiff (declared to C++ in MatchResultCacheDiffSwiftBoundary.h). This is the only file
// of the unit with `unsafe`: it wraps the C++-owned buffers (the entry's snapshot, the inline style's CSSProperty
// records, the alias table and the glue's output words) in spans and calls the safe core in MatchResultCacheDiff.swift.
// A pointer may be null only when its count is 0.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

@_cdecl("WebCoreSwift_MatchResultCacheDiff_changedProperties")
func WebCoreSwift_MatchResultCacheDiff_changedProperties(_ state: UnsafeMutablePointer<UInt64>?, _ stateCount: Int,
    _ original: UnsafePointer<UInt64>?, _ originalWordCount: Int, _ current: UnsafePointer<UInt64>?, _ currentWordCount: Int,
    _ aliases: UnsafePointer<UInt16>?, _ aliasCount: Int, _ out: UnsafeMutablePointer<UInt32>?, _ outCount: Int) -> UInt32
{
    var stateSpan = unsafe UnsafeMutableBufferPointer(start: state, count: stateCount).mutableSpan
    var outSpan = unsafe UnsafeMutableBufferPointer(start: out, count: outCount).mutableSpan
    return unsafe MatchResultCacheDiff.changedProperties(state: &stateSpan, original: UnsafeBufferPointer(start: original, count: originalWordCount).span,
        current: UnsafeBufferPointer(start: current, count: currentWordCount).span, aliases: UnsafeBufferPointer(start: aliases, count: aliasCount).span,
        out: &outSpan)
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
