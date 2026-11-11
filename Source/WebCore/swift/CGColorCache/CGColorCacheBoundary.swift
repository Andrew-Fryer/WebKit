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

// Boundary for cachedCGColor() in Source/WebCore/platform/graphics/cg/ColorCG.cpp, declared in
// CGColorCacheSwiftBoundary.h: wraps C++'s arrays in spans and forwards the equality callback.

@_cdecl("WebCoreSwift_CGColorCache_lookup")
func WebCoreSwift_CGColorCache_lookup(_ keys: UnsafeMutablePointer<UInt64>?, _ keyCount: Int, _ order: UnsafeMutablePointer<UInt8>?,
    _ orderCount: Int, _ size: Int, _ keyBits: UInt64, _ outOfLineKey: UnsafeRawPointer?,
    _ outOfLineEntryEquals: @convention(c) (UnsafeRawPointer?, UInt8) -> Bool) -> UInt32
{
    var keySpan = unsafe UnsafeMutableBufferPointer(start: keys, count: keyCount).mutableSpan
    var orderSpan = unsafe UnsafeMutableBufferPointer(start: order, count: orderCount).mutableSpan
    guard unsafe outOfLineKey != nil else {
        return CGColorCache.lookup(keys: &keySpan, order: &orderSpan, size: size, keyBits: keyBits)
    }
    let equals = unsafe CGColorCacheEquals(key: outOfLineKey, equals: outOfLineEntryEquals)
    return unsafe CGColorCache.lookupOutOfLine(keys: &keySpan, order: &orderSpan, size: size, keyBits: keyBits, equals)
}

// The out-of-line equality callback from C++ and its context.
@unsafe struct CGColorCacheEquals: CGColorCache.OutOfLineEntryEquals {
    let key: UnsafeRawPointer?
    let equals: @convention(c) (UnsafeRawPointer?, UInt8) -> Bool

    func callAsFunction(_ slot: UInt8) -> Bool {
        unsafe equals(key, slot)
    }
}
