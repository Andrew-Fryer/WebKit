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

// The C entry point of NameValidation (declared to C++ in NameValidationSwiftBoundary.h). This is the only file of the
// unit with `unsafe`: it wraps the StringView's characters (Latin-1 bytes, or UTF-16 code units read as raw bytes
// because they may be odd-aligned) in a span and calls the safe core in NameValidation.swift. `characters` may be null
// only when `length` is 0.

@_cdecl("WebCoreSwift_NameValidation_isValidName")
@inline(__always) // The Latin-1 attribute and element name checks run in the C entry point itself, with no frame.
func WebCoreSwift_NameValidation_isValidName(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ kind: UInt8) -> Bool {
    if is8Bit {
        let name = unsafe UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length)
        if kind == NameValidation.attributeName {
            return unsafe NameValidation.isValidAttributeName(name.span)
        }
        if kind == NameValidation.elementName {
            return unsafe NameValidation.isValidElementName(NameValidationLatin1(name.span))
        }
    }
    // One out-of-line call for the rest, so that only its path sets up a frame.
    return unsafe nameValidationIsValidName(characters, length, is8Bit, kind)
}

@inline(never)
func nameValidationIsValidName(_ characters: UnsafeRawPointer?, _ length: Int, _ is8Bit: Bool, _ kind: UInt8) -> Bool {
    if is8Bit {
        return unsafe NameValidation.isValidName(NameValidationLatin1(UnsafeBufferPointer(start: characters?.assumingMemoryBound(to: UInt8.self), count: length).span), kind: kind)
    }
    return unsafe NameValidation.isValidName(NameValidationUTF16(UnsafeRawBufferPointer(start: characters, count: length &* 2).bytes), kind: kind)
}
