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

// The hand-rolled scan of a builtin's source text that BuiltinExecutables::createExecutable
// (builtins/BuiltinExecutables.cpp) does for every builtin without generated metadata: WebCore's builtins and the two
// default class constructors. The source is a single anonymous function expression,
// "(function (<parameters>) { <body> })" or "(async function (<parameters>) { <body> })", in Latin-1. Scanning by hand
// rather than parsing keeps createExecutable from recursing into the parser, and hence from overflowing the stack.
//
// This is the original C++ computeBuiltinSourceMetadata step for step, and Scripts/wkbuiltins/builtins_generator.py's
// compute_builtin_source_metadata transliterates it for the JavaScriptCore builtins it scans at build time: change one
// and you must change the other. Two things differ from the C++, neither observably:
// - The "use strict" scan stops at the first match. Once the C++ had found it, the rest of its loop did nothing.
// - Where the C++ read out of bounds (no ')' after the parameters, no '}' at all), the span's bounds check traps.
//   The C++ trapped there too, in libc++'s hardened std::span::operator[].
// Its RELEASE_ASSERTs are preconditions here.
//
// Indices are Int; the C++ uses `unsigned` and `int`. They agree because a String is at most 2^31 - 1 characters long.

enum BuiltinSourceScan {
    // BuiltinSourceMetadata (builtins/BuiltinExecutables.h).
    struct Metadata {
        var sourceLength: UInt32
        var parametersStart: UInt32
        var parameterCount: UInt32
        var closeBraceOffsetFromEnd: Int32
        var isAsyncFunction: Bool
        var isInStrictContext: Bool
    }

    // strlen("(function (){})") and strlen("(async function (){})").
    static var minimumLength: Int { 15 }
    static var minimumAsyncLength: Int { 21 }

    // isWhiteSpace<Latin1Character> (parser/SourceCharacters.h): JavaScript white space in Latin-1, which does not
    // include the line terminators.
    @inline(__always)
    static func isWhiteSpace(_ character: UInt8) -> Bool {
        switch character {
        case 0x20, 0x09, 0x0B, 0x0C, 0xA0: true
        default: false
        }
    }

    // spanHasPrefix(characters, "(function ("_span).
    @inline(__always)
    static func hasRegularFunctionPrefix(_ characters: Span<UInt8>) -> Bool {
        characters.count >= 11
            && characters[0] == 0x28 && characters[1] == 0x66 && characters[2] == 0x75 && characters[3] == 0x6E // "(fun"
            && characters[4] == 0x63 && characters[5] == 0x74 && characters[6] == 0x69 && characters[7] == 0x6F // "ctio"
            && characters[8] == 0x6E && characters[9] == 0x20 && characters[10] == 0x28 // "n ("
    }

    // spanHasPrefix(characters, "(async function ("_span).
    @inline(__always)
    static func hasAsyncFunctionPrefix(_ characters: Span<UInt8>) -> Bool {
        characters.count >= 17
            && characters[0] == 0x28 && characters[1] == 0x61 && characters[2] == 0x73 && characters[3] == 0x79 // "(asy"
            && characters[4] == 0x6E && characters[5] == 0x63 && characters[6] == 0x20 && characters[7] == 0x66 // "nc f"
            && characters[8] == 0x75 && characters[9] == 0x6E && characters[10] == 0x63 && characters[11] == 0x74 // "unct"
            && characters[12] == 0x69 && characters[13] == 0x6F && characters[14] == 0x6E && characters[15] == 0x20 // "ion "
            && characters[16] == 0x28 // "("
    }

    // spanHasPrefix(characters.subspan(index), "use strict"_span), given index + 10 < characters.count.
    @inline(__always)
    static func isUseStrict(_ characters: Span<UInt8>, at index: Int) -> Bool {
        characters[index] == 0x75 && characters[index &+ 1] == 0x73 && characters[index &+ 2] == 0x65 // "use"
            && characters[index &+ 3] == 0x20 && characters[index &+ 4] == 0x73 && characters[index &+ 5] == 0x74 // " st"
            && characters[index &+ 6] == 0x72 && characters[index &+ 7] == 0x69 && characters[index &+ 8] == 0x63 // "ric"
            && characters[index &+ 9] == 0x74 // "t"
    }

    // The number of formal parameters in the list that starts after the '(' at characters[parametersStart]: one more
    // than the commas outside curly brackets, or 1 for a list that has anything but white space, or 0; less one for
    // a rest parameter. A ')' ends the list wherever it is, even inside curly brackets, and the brackets do not nest.
    @inline(__always)
    static func parameterCount(_ characters: Span<UInt8>, parametersStart: Int) -> UInt32 {
        let count = characters.count
        var i = parametersStart &+ 1
        var commas: UInt32 = 0
        var insideCurlyBrackets = false
        var sawOneParam = false
        var hasRestParam = false
        while true {
            let character = characters[i]
            if character == 0x29 { // ')'
                break
            }

            if character == 0x7D { // '}'
                insideCurlyBrackets = false
            } else if character == 0x7B || insideCurlyBrackets { // '{'
                insideCurlyBrackets = true
                i &+= 1
                continue
            } else if character == 0x2C { // ','
                commas &+= 1
            } else if !isWhiteSpace(character) {
                sawOneParam = true
            }

            if character == 0x2E && i &+ 2 < count && characters[i &+ 1] == 0x2E && characters[i &+ 2] == 0x2E { // "..."
                hasRestParam = true
                i &+= 2
            }

            i &+= 1
        }

        var parameterCount: UInt32
        if commas != 0 {
            parameterCount = commas &+ 1
        } else if sawOneParam {
            parameterCount = 1
        } else {
            parameterCount = 0
        }

        if hasRestParam {
            precondition(parameterCount != 0)
            parameterCount &-= 1
        }
        return parameterCount
    }

    // Whether a '"' or '\'' anywhere in the source is followed by "use strict" and at least one more character.
    // The quotes need not match, and what follows "use strict" is not looked at.
    @inline(__always)
    static func isInStrictContext(_ characters: Span<UInt8>) -> Bool {
        let count = characters.count
        var i = 0
        while i < count {
            let character = characters[i]
            if character == 0x22 || character == 0x27 { // '"' or '\''
                if i &+ 11 < count && isUseStrict(characters, at: i &+ 1) {
                    return true
                }
            }
            i &+= 1
        }
        return false
    }

    // How far the last '}' is from the end: 1 when it is the last character.
    @inline(__always)
    static func closeBraceOffsetFromEnd(_ characters: Span<UInt8>) -> Int32 {
        let count = characters.count
        var closeBraceOffsetFromEnd = 1
        while characters[count &- closeBraceOffsetFromEnd] != 0x7D { // '}'
            closeBraceOffsetFromEnd &+= 1
        }
        return Int32(truncatingIfNeeded: closeBraceOffsetFromEnd)
    }

    @inline(__always)
    static func computeMetadata(_ characters: Span<UInt8>) -> Metadata {
        precondition(characters.count >= minimumLength)
        let isAsyncFunction = characters.count >= minimumAsyncLength && hasAsyncFunctionPrefix(characters)
        precondition(isAsyncFunction || hasRegularFunctionPrefix(characters))

        // strlen("function (") + strlen("async "): the offset of the '(' that opens the parameters.
        let parametersStart = isAsyncFunction ? 16 : 10
        let parameterCount = parameterCount(characters, parametersStart: parametersStart)
        let isInStrictContext = isInStrictContext(characters)
        let closeBraceOffsetFromEnd = closeBraceOffsetFromEnd(characters)

        return Metadata(
            sourceLength: UInt32(truncatingIfNeeded: characters.count),
            parametersStart: UInt32(truncatingIfNeeded: parametersStart),
            parameterCount: parameterCount,
            closeBraceOffsetFromEnd: closeBraceOffsetFromEnd,
            isAsyncFunction: isAsyncFunction,
            isInStrictContext: isInStrictContext)
    }

    // JSCSwift_BuiltinSourceScan_WordCount: the metadata's fields, in order, one UInt32 each (the Int32 as its bit
    // pattern, the flags as 0 or 1).
    static var wordCount: Int { 6 }

    @inline(__always)
    static func computeMetadata(_ characters: Span<UInt8>, into words: inout MutableSpan<UInt32>) {
        let metadata = computeMetadata(characters)
        words[0] = metadata.sourceLength
        words[1] = metadata.parametersStart
        words[2] = metadata.parameterCount
        words[3] = UInt32(bitPattern: metadata.closeBraceOffsetFromEnd)
        words[4] = metadata.isAsyncFunction ? 1 : 0
        words[5] = metadata.isInStrictContext ? 1 : 0
    }
}
