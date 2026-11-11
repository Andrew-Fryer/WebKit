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

// C entry point for CSSColor (declared to C++ in CSSColorSwiftBoundary.h). This is the only file of the unit with
// `unsafe`: it wraps the C++-owned characters (Latin-1 bytes, or UTF-16 code units read as raw bytes because they
// may be odd-aligned) in a span and calls the safe core in CSSColor.swift. `characters` may be null only when
// `length` is 0.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

@_cdecl("WebCoreSwift_CSSColor_parseColor")
func WebCoreSwift_CSSColor_parseColor(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ mode: UInt8) -> UInt64 {
    if is8Bit {
        return unsafe CSSColor.parseColor(CSSColorLatin1Characters(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span), mode: mode)
    }
    return unsafe CSSColor.parseColor(CSSColorUTF16Characters(UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes), mode: mode)
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
