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

// The lexing half of WebCore::SVGTransformList::parseGeneric and parseTransformType, for Latin-1 and UTF-16 input. An
// item is: leading whitespace, transform keyword, parenthesized parameter list, then the whitespace / ',' / whitespace
// that separates it from the next item; one call lexes consecutive items (up to the caller's capacity, stopping after
// the first that fails). It mirrors, statement by statement and including where the cursor stops on failure, the
// original C++ (which this replaces; both character types ran the same templates):
//   - parseTransformTypeGeneric                                       (svg/SVGTransformList.cpp)
//   - parseTransformParamList + required/optionalValuesForType        (formerly svg/SVGTransformListInlines.h)
// Whitespace, delimiters and numbers use WebCore's one SVG number grammar, SVGNumberGrammar (swift/SVGNumberGrammar).
// The C++ glue in SVGTransformList.cpp applies the result to the list.
//
// Result layout (SVGTransformSwiftBoundary.h): bits 0-7 status, bits 8-15 SVGTransformType,
// bits 16-23 number of values, bit 24 set if a ',' followed the item.

enum SVGTransformLexer {
    // Status values; keep in sync with SVGTransformSwiftBoundary.h.
    static var statusNoTransformType: UInt32 { 0 }
    static var statusInvalidParameters: UInt32 { 1 }
    static var statusParsed: UInt32 { 2 }
    static var statusOnlyWhitespace: UInt32 { 3 }

    // SVGTransformValue::SVGTransformType.
    static var typeMatrix: UInt32 { 1 }
    static var typeTranslate: UInt32 { 2 }
    static var typeScale: UInt32 { 3 }
    static var typeRotate: UInt32 { 4 }
    static var typeSkewX: UInt32 { 5 }
    static var typeSkewY: UInt32 { 6 }

    static var maximumValueCount: Int { 6 }

    // lexItems, packed for the C entry point: the cursor advance << 8 | the number of items (results.count <= 255).
    @inline(__always)
    static func lex<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ results: inout MutableSpan<UInt32>, _ values: inout MutableSpan<Float>) -> UInt64 {
        precondition(results.count <= 0xFF)
        let lexed = lexItems(characters, &results, &values)
        return UInt64(truncatingIfNeeded: lexed.end) &<< 8 | UInt64(truncatingIfNeeded: lexed.count)
    }

    // Lexes consecutive items from the start of `characters` until one is not `statusParsed`, the input
    // ends, or `results` is full. Item k's result goes to results[k] and its values to
    // values[6 * k ..< 6 * k + count] (`values` must have room for them). `end` is how far the C++ lexer would have advanced its
    // StringParsingBuffer after the last lexed item, also when that item failed (SVGViewSpec keeps
    // parsing from there). At least one item is lexed if `characters` and `results` are not empty.
    @inline(__always)
    static func lexItems<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ results: inout MutableSpan<UInt32>, _ values: inout MutableSpan<Float>) -> (count: Int, end: Int) {
        var index = 0
        var count = 0
        while count < results.count {
            let item = lexItem(characters, index, &values, valueBase: count &* maximumValueCount)
            results[count] = item.result
            count &+= 1
            index = item.end
            if item.result & 0xFF != statusParsed || index >= characters.count {
                break
            }
        }
        return (count, index)
    }

    // Lexes one item at `start`; its values go to values[valueBase...].
    @inline(__always)
    static func lexItem<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ start: Int, _ values: inout MutableSpan<Float>, valueBase: Int) -> (result: UInt32, end: Int) {
        var index = start

        // parseGeneric skips whitespace before the first item; between items its trailing skip has
        // already consumed all whitespace, so this is a no-op there.
        if !SVGNumberGrammar.skipSpaces(characters, &index) {
            return (statusOnlyWhitespace, index)
        }

        guard let type = parseTransformType(characters, &index) else {
            return (statusNoTransformType, index)
        }

        let required: Int
        let optional: Int
        switch type {
        case typeMatrix:
            required = 6
            optional = 0
        case typeTranslate, typeScale:
            required = 1
            optional = 1
        case typeRotate:
            required = 1
            optional = 2
        default: // typeSkewX, typeSkewY
            required = 1
            optional = 0
        }

        let count = parseParameterList(characters, &index, &values, valueBase: valueBase, required: required, optional: optional)
        if count < 0 {
            return (statusInvalidParameters | type << 8, index)
        }

        // parseGeneric: skipOptionalSVGSpaces; skipExactly(','); skipOptionalSVGSpaces.
        _ = SVGNumberGrammar.skipSpaces(characters, &index)
        var commaBit: UInt32 = 0
        if index < characters.count && characters[index] == SVGNumberGrammar.comma {
            index &+= 1
            commaBit = 1 << 24
        }
        _ = SVGNumberGrammar.skipSpaces(characters, &index)

        return (statusParsed | type << 8 | UInt32(truncatingIfNeeded: count) << 16 | commaBit, index)
    }

    // MARK: - Characters

    private static var leftParenthesis: UInt32 { 0x28 } // '('
    private static var rightParenthesis: UInt32 { 0x29 } // ')'

    // skipCharactersExactly: advances only on a full match.
    @inline(__always)
    private static func skipKeyword<Characters: SVGCharacters & ~Escapable, let length: Int>(_ characters: Characters, _ index: inout Int, _ keyword: InlineArray<length, UInt8>) -> Bool {
        if characters.count &- index < length {
            return false
        }
        for offset in 0..<length {
            if characters[index &+ offset] != UInt32(keyword[offset]) {
                return false
            }
        }
        index &+= length
        return true
    }

    // MARK: - parseTransformTypeGeneric

    @inline(__always)
    private static func parseTransformType<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int) -> UInt32? {
        if index >= characters.count {
            return nil
        }

        if characters[index] == 0x73 { // 's'
            if skipKeyword(characters, &index, [0x73, 0x6B, 0x65, 0x77, 0x58]) { // "skewX"
                return typeSkewX
            }
            if skipKeyword(characters, &index, [0x73, 0x6B, 0x65, 0x77, 0x59]) { // "skewY"
                return typeSkewY
            }
            if skipKeyword(characters, &index, [0x73, 0x63, 0x61, 0x6C, 0x65]) { // "scale"
                return typeScale
            }
            return nil
        }

        if skipKeyword(characters, &index, [0x74, 0x72, 0x61, 0x6E, 0x73, 0x6C, 0x61, 0x74, 0x65]) { // "translate"
            return typeTranslate
        }
        if skipKeyword(characters, &index, [0x72, 0x6F, 0x74, 0x61, 0x74, 0x65]) { // "rotate"
            return typeRotate
        }
        if skipKeyword(characters, &index, [0x6D, 0x61, 0x74, 0x72, 0x69, 0x78]) { // "matrix"
            return typeMatrix
        }
        return nil
    }

    // MARK: - parseTransformParamList

    // Returns the number of values parsed, or -1. The C++ has one loop for the required values and one for
    // the optional ones, each followed by the same whitespace/delimiter handling; this runs that code
    // twice with the second pass only reached when an optional value follows (one inlined number parser).
    @inline(__always)
    private static func parseParameterList<Characters: SVGCharacters & ~Escapable>(_ characters: Characters, _ index: inout Int, _ values: inout MutableSpan<Float>, valueBase: Int, required: Int, optional: Int) -> Int {
        if !SVGNumberGrammar.skipSpaces(characters, &index) || characters[index] != leftParenthesis {
            return -1
        }

        index &+= 1

        _ = SVGNumberGrammar.skipSpaces(characters, &index)

        var count = 0
        var limit = required
        var parsingOptionalValues = false
        while true {
            while count < limit {
                if index >= characters.count {
                    return -1
                }
                guard let number = SVGNumberGrammar.parseNumber(characters, &index) else {
                    return -1
                }
                values[valueBase &+ count] = number
                count &+= 1
                if count < limit {
                    _ = SVGNumberGrammar.skipSpacesOrDelimiter(characters, &index)
                }
            }

            if !SVGNumberGrammar.skipSpaces(characters, &index) {
                return -1
            }

            let delimiterParsed = SVGNumberGrammar.skipSpacesOrDelimiter(characters, &index)

            if parsingOptionalValues {
                if index >= characters.count || characters[index] != rightParenthesis || delimiterParsed {
                    return -1
                }
                index &+= 1
                return count
            }

            if index >= characters.count {
                return -1
            }

            if characters[index] == rightParenthesis {
                // skip optionals
                index &+= 1
                if delimiterParsed {
                    return -1
                }
                return count
            }

            parsingOptionalValues = true
            limit = required &+ optional
        }
    }
}
