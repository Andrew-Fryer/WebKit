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

// Memory-safe port of the scanners of HTMLFastPathParser<CharacterType> (html/parser/HTMLDocumentParserFastPath.cpp):
// - scanText() minus scanEscapedText();
// - one start tag in one call: scanTagName() with findHTMLElementName() narrowed to the 18 tags the fast path
//   supports, then the scanning half of parseAttributes() with scanAttributeName() and scanAttributeValue() inlined,
//   minus scanEscapedAttributeValue().
// The C++ glue keeps everything that touches WebCore objects: the parsing buffer position, didFail(), String /
// AtomString / QualifiedName creation through HTMLNameCache, the escaped (character reference, '\r') slow paths,
// which have no training counts, the attribute dedupe and parserSetAttributes(). Statement order and failure codes
// follow the C++ one to one; comments name the C++ statements. No allocation, no reference counting.

// Read access to the parser input, specialized for Latin-1 and UTF-16 by the generic scanners.
protocol HTMLScanInput: ~Escapable {
    var count: Int { get }
    subscript(_ index: Int) -> UInt16 { get }
    // Index of the first code unit at or after `start` that is in `set`, or count.
    func firstIndex<Set: HTMLScanCharacterSet>(from start: Int, _ set: Set) -> Int
    // Code units 0..<length (1 <= length <= 6, all ASCII letters), lowercased, one per byte, first unit lowest.
    func packedLowercaseName(length: Int) -> UInt64
    // How many units of a quoted attribute value are tested one by one before the vector search (as in the C++,
    // attribute names are always scanned one by one). A vector search makes the next position wait for a load ->
    // compare -> move-to-GPR -> count-trailing-zeros chain, which a short, well-predicted scalar loop avoids; the UTF-16
    // vectors (two loads and a deinterleave) only pay off for longer values.
    static var valueScalarPrefix: Int { get }
}

struct HTMLScanLatin1Input: HTMLScanInput, ~Escapable {
    let characters: Span<UInt8>

    static var valueScalarPrefix: Int { 0 }

    @_lifetime(copy characters)
    init(_ characters: Span<UInt8>) {
        self.characters = characters
    }

    var count: Int {
        @inline(__always) get { characters.count }
    }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { UInt16(characters[index]) }
    }

    @inline(__always)
    func firstIndex<Set: HTMLScanCharacterSet>(from start: Int, _ set: Set) -> Int {
        HTMLScan.firstIndex(characters, from: start, set)
    }

    @inline(__always)
    func packedLowercaseName(length: Int) -> UInt64 {
        let mask = (1 &<< (UInt64(truncatingIfNeeded: length) &* 8)) &- 1
        if characters.count >= 8 {
            let word = UInt64(littleEndian: characters.bytes.load(fromByteOffset: 0, as: UInt64.self))
            return (word | 0x2020_2020_2020_2020) & mask
        }
        var word: UInt64 = 0
        for index in 0..<min(length, characters.count) {
            word |= UInt64(characters[index] | 0x20) &<< (UInt64(truncatingIfNeeded: index) &* 8)
        }
        return word
    }
}

// UTF-16 code units at an even address (as WTF strings have them).
struct HTMLScanUTF16Input: HTMLScanInput, ~Escapable {
    let characters: Span<UInt16>

    static var valueScalarPrefix: Int { 16 }

    @_lifetime(copy characters)
    init(_ characters: Span<UInt16>) {
        self.characters = characters
    }

    var count: Int {
        @inline(__always) get { characters.count }
    }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { characters[index] }
    }

    @inline(__always)
    func firstIndex<Set: HTMLScanCharacterSet>(from start: Int, _ set: Set) -> Int {
        HTMLScan.firstIndex(utf16: characters.bytes, count: characters.count, from: start, set)
    }

    @inline(__always)
    func packedLowercaseName(length: Int) -> UInt64 {
        HTMLScan.packedLowercaseName(utf16: characters.bytes, count: characters.count, length: length) { characters[$0] }
    }
}

// UTF-16 code units at any address, read as raw bytes (an odd address is possible in principle).
struct HTMLScanUnalignedUTF16Input: HTMLScanInput, ~Escapable {
    let bytes: RawSpan
    let count: Int

    static var valueScalarPrefix: Int { 16 }

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
        self.count = bytes.byteCount &>> 1
    }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { UInt16(littleEndian: bytes.load(fromByteOffset: index &* 2, as: UInt16.self)) }
    }

    @inline(__always)
    func firstIndex<Set: HTMLScanCharacterSet>(from start: Int, _ set: Set) -> Int {
        HTMLScan.firstIndex(utf16: bytes, count: count, from: start, set)
    }

    @inline(__always)
    func packedLowercaseName(length: Int) -> UInt64 {
        HTMLScan.packedLowercaseName(utf16: bytes, count: count, length: length) { self[$0] }
    }
}

extension HTMLScan {
    // HTMLScanInput.packedLowercaseName for UTF-16: with 8 units available, their low bytes (the units are ASCII
    // letters) in one step (xtn); otherwise unit by unit.
    @inline(__always)
    static func packedLowercaseName(utf16 bytes: RawSpan, count: Int, length: Int, _ unit: (Int) -> UInt16) -> UInt64 {
        let mask = (1 &<< (UInt64(truncatingIfNeeded: length) &* 8)) &- 1
        if count >= 8 {
            let lowBytes = SIMD8<UInt8>(truncatingIfNeeded: bytes.load(fromByteOffset: 0, as: SIMD8<UInt16>.self))
            return (UInt64(littleEndian: bitCast(lowBytes, to: UInt64.self)) | 0x2020_2020_2020_2020) & mask
        }
        var word: UInt64 = 0
        for index in 0..<min(length, count) {
            word |= UInt64(truncatingIfNeeded: unit(index) | 0x20) &<< (UInt64(truncatingIfNeeded: index) &* 8)
        }
        return word
    }
}

extension HTMLScan {
    // MARK: Character predicates (WTF ASCIICType.h and the HTMLDocumentParserFastPath.cpp helpers), on code units.

    @inline(__always) static func isASCIILower(_ c: UInt16) -> Bool { c &- 0x61 < 26 }
    @inline(__always) static func isASCIIUpper(_ c: UInt16) -> Bool { c &- 0x41 < 26 }
    @inline(__always) static func isASCIIDigit(_ c: UInt16) -> Bool { c &- 0x30 < 10 }
    @inline(__always) static func isASCIIAlpha(_ c: UInt16) -> Bool { (c | 0x20) &- 0x61 < 26 }
    @inline(__always) static func isASCIIAlphanumeric(_ c: UInt16) -> Bool { isASCIIDigit(c) || isASCIIAlpha(c) }

    // isASCIIWhitespace: ' ', '\n', '\t', '\r', '\f'.
    @inline(__always)
    static func isASCIIWhitespace(_ c: UInt16) -> Bool {
        c == 0x20 || c == 0x0A || c == 0x09 || c == 0x0D || c == 0x0C
    }

    @inline(__always)
    static func isQuoteCharacter(_ c: UInt16) -> Bool { c == 0x22 || c == 0x27 }

    @inline(__always)
    static func isValidUnquotedAttributeValueChar(_ c: UInt16) -> Bool {
        isASCIIAlphanumeric(c) || c == 0x5F || c == 0x2D
    }

    @inline(__always)
    static func isValidAttributeNameChar(_ c: UInt16) -> Bool {
        if c == 0x3D {
            return false
        }
        return isASCIIAlphanumeric(c) || c == 0x2D || c == 0x5F
    }

    @inline(__always)
    static func isNonUppercaseAttributeNameChar(_ c: UInt16) -> Bool {
        isASCIILower(c) || isASCIIDigit(c) || c == 0x2D || c == 0x5F
    }

    @inline(__always)
    static func isCharAfterTagNameOrAttribute(_ c: UInt16) -> Bool {
        c == 0x20 || c == 0x3E || isASCIIWhitespace(c) || c == 0x2F
    }

    @inline(__always)
    static func isCharAfterUnquotedAttribute(_ c: UInt16) -> Bool {
        c == 0x20 || c == 0x3E || isASCIIWhitespace(c)
    }

    // skipWhile<isASCIIWhitespace>.
    @inline(__always)
    static func skipWhitespace<Input: HTMLScanInput & ~Escapable>(_ input: Input, _ start: Int) -> Int {
        let count = input.count
        var index = start
        while index < count && isASCIIWhitespace(input[index]) {
            index &+= 1
        }
        return index
    }

    // MARK: scanText

    // Result kinds of scanText (WebCoreSwiftHTMLScanText* in HTMLScanSwiftBoundary.h).
    static var textPlain: UInt64 { 0 }
    static var textWhitespacePattern: UInt64 { 1 }
    static var textEscaped: UInt64 { 2 }
    static var textFailedContainsNull: UInt64 { 3 }
    static var textFailedBigText: UInt64 { 4 }

    // HTMLFastPathParser::scanText(), packed as kind | length << 8. `length` is the text length for textPlain and
    // textWhitespacePattern, where the C++ leaves the position for the failures, and 0 for textEscaped (the glue then
    // runs scanEscapedText() from the start).
    @inline(__always)
    static func scanText<Input: HTMLScanInput & ~Escapable>(_ input: Input, textLengthLimit: Int) -> UInt64 {
        let count = input.count
        var cursor = 0
        if count > 0 {
            let first = input[0]
            if first == 0x0A {
                // Check if the text is a cached whitespace string.
                var index = 1
                while index < count && input[index] == 0x20 {
                    index &+= 1
                }
                if index == count || input[index] == 0x3C {
                    return textWhitespacePattern | (UInt64(truncatingIfNeeded: index) &<< 8)
                }
                // The C++ restarts the search at the '\n'; '\n' and ' ' are not special, so starting here finds the same
                // index.
                cursor = input.firstIndex(from: index, HTMLScanFourCharacters(a: 0x3C, b: 0x26, c: 0x0D, d: 0x00))
            } else if first != 0x3C {
                // '<', '&', '\r', '\0'. Most texts between tags are empty, so the '<' at 0 is tested first.
                cursor = input.firstIndex(from: 0, HTMLScanFourCharacters(a: 0x3C, b: 0x26, c: 0x0D, d: 0x00))
            }
        }

        if cursor < count {
            let c = input[cursor]
            if c == 0 {
                return textFailedContainsNull | (UInt64(truncatingIfNeeded: cursor) &<< 8)
            }
            if c == 0x26 || c == 0x0D {
                return textEscaped
            }
        }
        if cursor >= textLengthLimit {
            return textFailedBigText | (UInt64(truncatingIfNeeded: cursor) &<< 8)
        }
        return textPlain | (UInt64(truncatingIfNeeded: cursor) &<< 8)
    }

    // MARK: scanStartTag

    // skipWhile<isNonUppercaseAttributeNameChar>: index of the first unit at or after `start` that is not [a-z0-9_-], or
    // count.
    @inline(__always)
    static func nonUppercaseAttributeNameRunEnd<Input: HTMLScanInput & ~Escapable>(_ input: Input, from start: Int) -> Int {
        let count = input.count
        var index = start
        while index < count && isNonUppercaseAttributeNameChar(input[index]) {
            index &+= 1
        }
        return index
    }

    // Index of the first unit at or after `start` that is `quote`, '&', '\r' or '\0', or count.
    @inline(__always)
    static func quotedValueEnd<Input: HTMLScanInput & ~Escapable>(_ input: Input, from start: Int, quote: UInt16) -> Int {
        let count = input.count
        let scalarEnd = min(count, start &+ Input.valueScalarPrefix)
        // All four are below 64: one bit test per unit.
        let stops: UInt64 = (1 &<< UInt64(truncatingIfNeeded: quote)) | (1 &<< 0x26) | (1 &<< 0x0D) | 1
        var index = start
        while index < scalarEnd {
            let c = input[index]
            if c < 64 && (stops &>> UInt64(truncatingIfNeeded: c)) & 1 != 0 {
                return index
            }
            index &+= 1
        }
        return index == count ? count : input.firstIndex(from: index, HTMLScanFourCharacters(a: UInt8(truncatingIfNeeded: quote), b: 0x26, c: 0x0D, d: 0x00))
    }

    // Tag classes: 0 ..< 18 are the supported tags in FOR_EACH_SUPPORTED_TAG order (a b body br button div footer i
    // input li label option ol p select span strong ul); then any other name, and a failed tag name.
    static var tagClassOther: UInt64 { 18 }
    static var tagClassFailed: UInt64 { 19 }

    // How the attribute list ended (WebCoreSwiftHTMLScanAttributes*).
    static var attributesDone: UInt64 { 0 }
    static var attributesMore: UInt64 { 1 }
    static var attributesEscapedValue: UInt64 { 2 }
    static var attributesFailed: UInt64 { 3 }

    // The first failure the C++ would have recorded with didFail() (WebCoreSwiftHTMLScanFailure*).
    static var failureNone: UInt64 { 0 }
    static var failureParsingQuotedAttributeValue: UInt64 { 1 }
    static var failureParsingUnquotedAttributeValue: UInt64 { 2 }
    static var failureEndOfInputReached: UInt64 { 3 }
    static var failureParsingAttributes: UInt64 { 4 }

    // The start tag scan buffer (WebCoreSwiftHTMLScanStartTag): `consumed` (64 bits), then the attribute records,
    // 16 bytes each (WebCoreSwiftHTMLScanAttribute): nameOffset, nameLength | lowering flag, valueOffset, valueLength
    // (32 bits each).
    static var startTagConsumedOffset: Int { 0 }
    static var startTagAttributesOffset: Int { 8 }
    static var attributeRecordBytes: Int { 16 }
    static var attributeCapacity: Int { 16 }
    static var startTagByteCount: Int { startTagAttributesOffset + attributeCapacity * attributeRecordBytes }
    static var attributeNameNeedsLowering: UInt32 { 0x8000_0000 }

    // One record, as two little-endian 64-bit words (offset | length << 32).
    @inline(__always)
    static func storeAttribute(_ startTag: inout MutableRawSpan, at offset: Int, nameOffset: Int, nameLength: Int, nameFlags: UInt32, valueOffset: Int, valueLength: Int) {
        let name = UInt64(UInt32(truncatingIfNeeded: nameOffset)) | (UInt64(UInt32(truncatingIfNeeded: nameLength) | nameFlags) &<< 32)
        let value = UInt64(UInt32(truncatingIfNeeded: valueOffset)) | (UInt64(UInt32(truncatingIfNeeded: valueLength)) &<< 32)
        startTag.storeBytes(of: name.littleEndian, toByteOffset: offset, as: UInt64.self)
        startTag.storeBytes(of: value.littleEndian, toByteOffset: offset &+ 8, as: UInt64.self)
    }

    // The class of a lowercase tag name of `length` (1 ... 6) code units packed by packedLowercaseName.
    @inline(__always)
    static func tagClass(_ name: UInt64, length: Int) -> UInt64 {
        switch length {
        case 1:
            switch name {
            case 0x61: 0 // a
            case 0x62: 1 // b
            case 0x69: 7 // i
            case 0x70: 13 // p
            default: tagClassOther
            }
        case 2:
            switch name {
            case 0x7262: 3 // br
            case 0x696C: 9 // li
            case 0x6C6F: 12 // ol
            case 0x6C75: 17 // ul
            default: tagClassOther
            }
        case 3:
            name == 0x76_6964 ? 5 : tagClassOther // div
        case 4:
            switch name {
            case 0x7964_6F62: 2 // body
            case 0x6E61_7073: 15 // span
            default: tagClassOther
            }
        case 5:
            switch name {
            case 0x74_7570_6E69: 8 // input
            case 0x6C_6562_616C: 10 // label
            default: tagClassOther
            }
        case 6:
            switch name {
            case 0x6E6F_7474_7562: 4 // button
            case 0x7265_746F_6F66: 6 // footer
            case 0x6E6F_6974_706F: 11 // option
            case 0x7463_656C_6573: 14 // select
            case 0x676E_6F72_7473: 16 // strong
            default: tagClassOther
            }
        default:
            tagClassOther
        }
    }

    // One start tag from just after its '<', or (scanTagName false) the rest of an attribute list from the start of
    // an attribute name. Writes to the records of `startTag` one record per attribute that parseAttributes() would
    // append or drop as a duplicate (the dedupe is the glue's), and `consumed`. Packed result:
    //   bits 0-7   tag class (0 when scanTagName is false); for tagClassFailed nothing else is set
    //   bits 8-11  attributesDone: the tag ended ('>' or '/' whitespace* '>'), `consumed` is past the '>'.
    //              attributesMore: the records are full; call again at `consumed`.
    //              attributesEscapedValue: the last record's value needs scanEscapedAttributeValue() from the quote at
    //              its valueOffset (== `consumed`); the glue then skips ASCII whitespace and calls again.
    //              attributesFailed: parseAttributes() returns at `consumed` without parserSetAttributes().
    //   bits 12-15 the first failure, which the glue reports with didFail() before using the records
    //   bits 16-23 the record count
    //   bits 32-63 where scanTagName() leaves the position: after the name and the ASCII whitespace that follows it
    //              (for tagClassFailed, where the case-insensitive loop stopped)
    @inline(__always)
    static func scanStartTag<Input: HTMLScanInput & ~Escapable>(_ input: Input, scanTagName: Bool, startTag: inout MutableRawSpan) -> UInt64 {
        var consumed = 0
        let result = scanStartTag(input, scanTagName: scanTagName, startTag: &startTag, consumed: &consumed)
        startTag.storeBytes(of: UInt64(truncatingIfNeeded: consumed).littleEndian, toByteOffset: startTagConsumedOffset, as: UInt64.self)
        return result
    }

    @inline(__always)
    static func scanStartTag<Input: HTMLScanInput & ~Escapable>(_ input: Input, scanTagName: Bool, startTag: inout MutableRawSpan, consumed: inout Int) -> UInt64 {
        let count = input.count
        var index = 0
        var head: UInt64 = 0
        if scanTagName {
            // skipWhile<isASCIILower>.
            while index < count && isASCIILower(input[index]) {
                index &+= 1
            }
            if index == count || !isCharAfterTagNameOrAttribute(input[index]) {
                // Try parsing a case-insensitive tagName. The C++ restarts at 0; input[0..<index] is lower case, which
                // this loop accepts, so continuing at index ends at the same place.
                while index < count && isASCIIAlpha(input[index]) {
                    index &+= 1
                }
                if index == count || !isCharAfterTagNameOrAttribute(input[index]) {
                    consumed = index
                    return tagClassFailed | (UInt64(truncatingIfNeeded: index) &<< 32)
                }
            }
            // findHTMLElementName() on the (lowered) name, narrowed to the supported tags.
            let tag = UInt(bitPattern: index &- 1) < 6 ? tagClass(input.packedLowercaseName(length: index), length: index) : tagClassOther
            index = skipWhitespace(input, index)
            head = tag | (UInt64(truncatingIfNeeded: index) &<< 32)
            // No attributes: what the loop below does for an empty name followed by '>'.
            if index < count && input[index] == 0x3E {
                consumed = index &+ 1
                return head
            }
        }

        // parseAttributes().
        var recordCount = 0
        var failure = failureNone
        var status = attributesDone
        while true {
            let record = startTagAttributesOffset &+ recordCount &* attributeRecordBytes
            if record &+ attributeRecordBytes > startTag.byteCount {
                status = attributesMore
                break
            }

            // scanAttributeName(): first look for a name without upper case.
            let nameStart = index
            index = nonUppercaseAttributeNameRunEnd(input, from: index)
            if index == count {
                // didFail(FailedEndOfInputReached); parseAttributes() then fails on nullQName() at the end.
                if failure == failureNone {
                    failure = failureEndOfInputReached
                }
                status = attributesFailed
                break
            }
            var nameFlags: UInt32 = 0
            if isASCIIUpper(input[index]) {
                // do { consume() } while (hasCharactersRemaining() && isValidAttributeNameChar(*m_parsingBuffer)).
                nameFlags = attributeNameNeedsLowering
                index = nameStart
                repeat {
                    index &+= 1
                } while index < count && isValidAttributeNameChar(input[index])
            }
            let nameLength = index &- nameStart

            // nullQName(): an empty name, or an on* name (compared after lowering: c | 0x20 is 'o' exactly for 'o' and
            // 'O', 'n' exactly for 'n' and 'N').
            if nameLength == 0 || (nameLength > 2 && input[nameStart] | 0x20 == 0x6F && input[nameStart &+ 1] | 0x20 == 0x6E) {
                status = attributesFailed
                if index < count {
                    let c = input[index]
                    if c == 0x3E {
                        index &+= 1
                        status = attributesDone
                    } else if c == 0x2F {
                        index = skipWhitespace(input, index &+ 1)
                        // if (atEnd() || consume() != '>') fail.
                        if index < count {
                            status = input[index] == 0x3E ? attributesDone : attributesFailed
                            index &+= 1
                        }
                    }
                }
                if status == attributesFailed && failure == failureNone {
                    failure = failureParsingAttributes
                }
                break
            }

            index = skipWhitespace(input, index)
            var valueOffset = index
            var valueLength = 0
            // skipExactly(m_parsingBuffer, '=').
            if index < count && input[index] == 0x3D {
                index &+= 1
                // scanAttributeValue().
                index = skipWhitespace(input, index)
                if index < count && isQuoteCharacter(input[index]) {
                    let quoteStart = index
                    let quote = input[index]
                    index &+= 1
                    // quote, '&', '\r', '\0'.
                    let cursor = quotedValueEnd(input, from: index, quote: quote)
                    if cursor < count && input[cursor] == quote {
                        valueOffset = index
                        valueLength = cursor &- index
                        index = cursor &+ 1
                    } else if cursor < count && input[cursor] != 0 {
                        // '&' or '\r': scanEscapedAttributeValue() from the quote, in the glue.
                        storeAttribute(&startTag, at: record, nameOffset: nameStart, nameLength: nameLength, nameFlags: nameFlags, valueOffset: quoteStart, valueLength: 0)
                        recordCount &+= 1
                        index = quoteStart
                        status = attributesEscapedValue
                        break
                    } else if failure == failureNone {
                        // No closing quote, or '\0': emptyAtom(), the position stays after the quote.
                        failure = failureParsingQuotedAttributeValue
                    }
                } else {
                    let start = index
                    while index < count && isValidUnquotedAttributeValueChar(input[index]) {
                        index &+= 1
                    }
                    if index < count && isCharAfterUnquotedAttribute(input[index]) {
                        valueOffset = start
                        valueLength = index &- start
                    } else if failure == failureNone {
                        failure = failureParsingUnquotedAttributeValue
                    }
                }
                index = skipWhitespace(input, index)
            }
            storeAttribute(&startTag, at: record, nameOffset: nameStart, nameLength: nameLength, nameFlags: nameFlags, valueOffset: valueOffset, valueLength: valueLength)
            recordCount &+= 1
        }
        consumed = index
        return head | (status &<< 8) | (failure &<< 12) | (UInt64(truncatingIfNeeded: recordCount) &<< 16)
    }
}
