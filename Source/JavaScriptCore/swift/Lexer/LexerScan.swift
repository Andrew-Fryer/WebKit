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

// The one Swift entry into the lexer (LexerBoundary.swift): dispatch on the scan kind and its flags, and pack the
// result into the four words the C++ glue reads (see LexerSwiftBoundary.h).
enum LexerScan {
    // The mode bits of LexerSwiftBoundary.h.
    @inline(__always) static var kindMask: UInt32 { 3 }
    @inline(__always) static var regExpKind: UInt32 { 0 }
    @inline(__always) static var stringKind: UInt32 { 1 }
    @inline(__always) static var templateKind: UInt32 { 2 }
    @inline(__always) static var wideFlag: UInt32 { 4 }
    @inline(__always) static var buildFlag: UInt32 { 8 }
    @inline(__always) static var strictFlag: UInt32 { 16 }
    @inline(__always) static var resultCount: Int { 4 }

    // Each scan has its own function per kind of source, with its build and strict-mode flags folded in where the
    // C++ had them as template parameters (shouldBuildStrings), as the separate entry points used to have.
    @inline(__always)
    static func scan<Source: LexerCharacters & ~Escapable>(_ mode: UInt32, _ source: Source, _ cooked: inout MutableSpan<UInt16>, _ raw: inout MutableSpan<UInt16>, _ results: inout MutableSpan<Int>) {
        let build = mode & buildFlag != 0
        switch mode & kindMask {
        case regExpKind:
            let result = regExp(source)
            results[0] = Int(result.status.rawValue | (UInt32(result.charactersOredTogether) &<< 16))
            results[1] = result.end
            results[2] = result.bodyEnd
            results[3] = 0
        case stringKind:
            let strictMode = mode & strictFlag != 0
            store(build ? stringBuild(source, strictMode, &cooked) : stringNoBuild(source, strictMode, &cooked), &results)
        case templateKind:
            store(build ? templateRaw(source, &cooked, &raw) : templateNoRaw(source, &cooked, &raw), &results)
        default:
            preconditionFailure("unknown lexer scan kind")
        }
    }

    @inline(never)
    static func regExp<Source: LexerCharacters & ~Escapable>(_ source: Source) -> LexerRegExp.Result {
        LexerRegExp.scan(source)
    }

    @inline(never)
    static func stringBuild<Source: LexerCharacters & ~Escapable>(_ source: Source, _ strictMode: Bool, _ cooked: inout MutableSpan<UInt16>) -> LexerEscapedLiterals.Result {
        LexerEscapedLiterals.scanString(source, build: true, strictMode: strictMode, &cooked)
    }

    @inline(never)
    static func stringNoBuild<Source: LexerCharacters & ~Escapable>(_ source: Source, _ strictMode: Bool, _ cooked: inout MutableSpan<UInt16>) -> LexerEscapedLiterals.Result {
        LexerEscapedLiterals.scanString(source, build: false, strictMode: strictMode, &cooked)
    }

    @inline(never)
    static func templateRaw<Source: LexerCharacters & ~Escapable>(_ source: Source, _ cooked: inout MutableSpan<UInt16>, _ raw: inout MutableSpan<UInt16>) -> LexerEscapedLiterals.Result {
        LexerEscapedLiterals.scanTemplate(source, buildRaw: true, &cooked, &raw)
    }

    @inline(never)
    static func templateNoRaw<Source: LexerCharacters & ~Escapable>(_ source: Source, _ cooked: inout MutableSpan<UInt16>, _ raw: inout MutableSpan<UInt16>) -> LexerEscapedLiterals.Result {
        LexerEscapedLiterals.scanTemplate(source, buildRaw: false, &cooked, &raw)
    }

    @inline(__always)
    static func store(_ result: LexerEscapedLiterals.Result, _ results: inout MutableSpan<Int>) {
        var flags = Int(result.status) | (Int(result.message) &<< 8)
        if result.cookedFailed {
            flags |= 1 &<< 16
        }
        if result.isTail {
            flags |= 1 &<< 17
        }
        results[0] = flags
        results[1] = result.end
        results[2] = result.cookedLength
        results[3] = result.rawLength
    }
}
