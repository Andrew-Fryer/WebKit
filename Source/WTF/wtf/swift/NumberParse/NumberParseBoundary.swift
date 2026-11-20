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

// C entry points of the NumberParse unit (declared to C++ in NumberParseSwiftBoundary.h, called by
// wtf/FastFloat.cpp and wtf/text/WTFString.cpp). This is the only file of the unit with `unsafe`: each
// entry point wraps the caller's pointer + count in a span, calls the safe core and stores the scalar
// out-parameters.

@_cdecl("WTFSwift_NumberParse_parseDouble8")
@inline(__always)
func WTFSwift_NumberParse_parseDouble8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    guard let result = unsafe NumberParse.parseGeneralWithoutSlowPath(buffer.span) else {
        return unsafe parseDouble8SlowPath(characters, length, parsedLength)
    }
    unsafe parsedLength.pointee = result.length
    return result.value
}

// Tail-called with the entry point's own arguments, so that WTFSwift_NumberParse_parseDouble8 needs no frame.
@inline(never)
func parseDouble8SlowPath(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.parseGeneralSlowPath(buffer.span)
    unsafe parsedLength.pointee = result.length
    return result.value
}

@_cdecl("WTFSwift_NumberParse_parseJSONDouble8")
@inline(__always)
func WTFSwift_NumberParse_parseJSONDouble8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    guard let result = unsafe NumberParse.parseJSONWithoutSlowPath(buffer.span) else {
        return unsafe parseJSONDouble8SlowPath(characters, length, parsedLength)
    }
    unsafe parsedLength.pointee = result.length
    return result.value
}

// Tail-called with the entry point's own arguments, so that WTFSwift_NumberParse_parseJSONDouble8 needs no frame.
@inline(never)
func parseJSONDouble8SlowPath(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.parseJSONSlowPath(buffer.span)
    unsafe parsedLength.pointee = result.length
    return result.value
}

@_cdecl("WTFSwift_NumberParse_parse8")
func WTFSwift_NumberParse_parse8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ mode: UInt32, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.parse(buffer.span, mode: mode)
    unsafe parsedLength.pointee = result.length
    return result.value
}

// The characters are char16_t, so they are 2-byte aligned (UnsafeBufferPointer<UInt16>.span traps otherwise).
@_cdecl("WTFSwift_NumberParse_parse16")
func WTFSwift_NumberParse_parse16(_ characters: UnsafePointer<UInt16>?, _ length: Int, _ mode: UInt32, _ parsedLength: UnsafeMutablePointer<Int>) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.parse(buffer.span, mode: mode)
    unsafe parsedLength.pointee = result.length
    return result.value
}

@_cdecl("WTFSwift_NumberParse_charactersTo8")
@inline(__always)
func WTFSwift_NumberParse_charactersTo8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ mode: UInt32, _ parsedLength: UnsafeMutablePointer<Int>?, _ ok: UnsafeMutablePointer<Bool>?) -> Double {
    if (mode & NumberParse.modeFormatMask) == 0 {
        let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
        if let result = unsafe NumberParse.charactersToGeneralWithoutSlowPath(buffer.span, mode: mode) {
            if let parsedLength = unsafe parsedLength {
                unsafe parsedLength.pointee = result.length
            }
            if let ok = unsafe ok {
                unsafe ok.pointee = result.ok
            }
            return result.value
        }
    }
    return unsafe charactersTo8Complete(characters, length, mode, parsedLength, ok)
}

// charactersToDouble(std::span<const Latin1Character>, bool*), the hot one: the mode is fixed, so less is live
// across the parse than in WTFSwift_NumberParse_charactersTo8.
@_cdecl("WTFSwift_NumberParse_charactersToDouble8")
@inline(__always)
func WTFSwift_NumberParse_charactersToDouble8(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ ok: UnsafeMutablePointer<Bool>?) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    if let result = unsafe NumberParse.charactersToGeneralWithoutSlowPath(buffer.span, mode: NumberParse.modeSkipWhitespace) {
        if let ok = unsafe ok {
            unsafe ok.pointee = result.ok
        }
        return result.value
    }
    return unsafe charactersTo8Complete(characters, length, NumberParse.modeSkipWhitespace, nil, ok)
}

// charactersToFixedDouble, and the inputs that need fast_float's slow path. Tail-called with the entry point's own
// arguments, so that WTFSwift_NumberParse_charactersTo8 needs no frame.
@inline(never)
func charactersTo8Complete(_ characters: UnsafePointer<UInt8>?, _ length: Int, _ mode: UInt32, _ parsedLength: UnsafeMutablePointer<Int>?, _ ok: UnsafeMutablePointer<Bool>?) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.charactersTo(buffer.span, mode: mode)
    if let parsedLength = unsafe parsedLength {
        unsafe parsedLength.pointee = result.length
    }
    if let ok = unsafe ok {
        unsafe ok.pointee = result.ok
    }
    return result.value
}

@_cdecl("WTFSwift_NumberParse_charactersTo16")
func WTFSwift_NumberParse_charactersTo16(_ characters: UnsafePointer<UInt16>?, _ length: Int, _ mode: UInt32, _ parsedLength: UnsafeMutablePointer<Int>?, _ ok: UnsafeMutablePointer<Bool>?) -> Double {
    let buffer = unsafe UnsafeBufferPointer(start: characters, count: length)
    let result = unsafe NumberParse.charactersTo(buffer.span, mode: mode)
    if let parsedLength = unsafe parsedLength {
        unsafe parsedLength.pointee = result.length
    }
    if let ok = unsafe ok {
        unsafe ok.pointee = result.ok
    }
    return result.value
}

@_cdecl("WTFSwift_NumberParse_doubleFromDigits")
@inline(__always)
func WTFSwift_NumberParse_doubleFromDigits(_ mantissa: UInt64, _ exponent: Int64) -> Double {
    NumberParse.doubleFromDigits(mantissa, exponent)
}
