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

// Memory-safe port of the name checks of dom/NameValidation.cpp, with their character predicates:
// isValidAttributeName (isInvalidAttributeNameCharacter), isValidElementName (isInvalidElementNameCharacterAfterAlphaStart,
// isValidElementNameContinuationCharacter), isValidNamespacePrefix (isInvalidNamespacePrefixCharacter),
// isValidDoctypeName (isInvalidDoctypeNameCharacter) and isValidXMLName (isValidASCIIXMLName, isValidXMLNameStart,
// isValidXMLNamePart, over StringView::codePoints()).
//
// Latin-1 names arrive as a Span<UInt8>. UTF-16 names arrive as a RawSpan (2 bytes per code unit, any alignment) and
// every code unit is compared whole, so U+0120 (low byte ' ') or U+013D (low byte '=') stay valid.
//
// isValidAttributeName is hot (setAttribute); its Latin-1 path tests 8 or 16 bytes per step with SIMD compares and
// overlapping loads instead of the C++ per-character loop. The answer is an any-match over the bytes, so checking a
// byte twice is harmless. The other checks have no training counts and share one generic implementation over
// Latin-1 and UTF-16. No allocation, no reference counting, no traps beyond the Span bounds checks (which cannot
// fire: every load is in range by construction).

enum NameValidation {
}

// The code units of a name: Latin-1 bytes or UTF-16 code units (raw bytes, any alignment).
protocol NameValidationUnits: ~Escapable {
    var count: Int { get }
    subscript(_ index: Int) -> UInt16 { get }
}

struct NameValidationLatin1: NameValidationUnits, ~Escapable {
    let characters: Span<UInt8>

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
}

struct NameValidationUTF16: NameValidationUnits, ~Escapable {
    let bytes: RawSpan
    let count: Int

    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
        self.count = bytes.byteCount &>> 1
    }

    subscript(_ index: Int) -> UInt16 {
        @inline(__always) get { UInt16(littleEndian: bytes.load(fromByteOffset: index &* 2, as: UInt16.self)) }
    }
}

extension NameValidation {
    // The checks (WebCoreSwiftNameValidationKind in NameValidationSwiftBoundary.h).
    static var attributeName: UInt8 { 0 }
    static var elementName: UInt8 { 1 }
    static var namespacePrefix: UInt8 { 2 }
    static var doctypeName: UInt8 { 3 }
    static var xmlName: UInt8 { 4 }

    // The invalid ASCII code units as bit sets over c < 64 (every code unit >= 64 is valid in these sets):
    // isInvalidAttributeNameCharacter: !c || isASCIIWhitespace(c) || c == '/' || c == '>' || c == '='
    //   = { NUL, '\t', '\n', '\f', '\r', ' ', '/', '=', '>' }. '\v' (0x0B) is not ASCII whitespace.
    // isInvalidElementNameCharacterAfterAlphaStart and isInvalidNamespacePrefixCharacter: the same without '='.
    // isInvalidDoctypeNameCharacter: !c || isASCIIWhitespace(c) || c == '>'.
    static var invalidAttributeNameCharacters: UInt64 { 0x6000_8001_0000_3601 }
    static var invalidElementNameCharactersAfterAlphaStart: UInt64 { 0x4000_8001_0000_3601 }
    static var invalidDoctypeNameCharacters: UInt64 { 0x4000_0001_0000_3601 }

    @inline(__always)
    static func isInSet(_ character: UInt16, _ set: UInt64) -> Bool {
        character < 64 && (set &>> UInt64(character)) & 1 != 0
    }

    @inline(__always)
    static func isASCIIAlpha(_ character: UInt32) -> Bool {
        (character | 0x20) &- 0x61 < 26
    }

    @inline(__always)
    static func isASCIIDigit(_ character: UInt32) -> Bool {
        character &- 0x30 < 10
    }

    // isValidElementNameContinuationCharacter: isASCIIAlphanumeric(c) || c == '-' || c == '.' || c == ':' || c == '_'
    // || !isASCII(c).
    @inline(__always)
    static func isValidElementNameContinuationCharacter(_ character: UInt16) -> Bool {
        if character >= 0x80 {
            return true
        }
        let c = UInt32(character)
        return isASCIIAlpha(c) || isASCIIDigit(c) || c == 0x2D || c == 0x2E || c == 0x3A || c == 0x5F
    }

    // isValidXMLNameStart: https://www.w3.org/TR/xml/#NT-NameStartChar
    @inline(__always)
    static func isValidXMLNameStart(_ c: UInt32) -> Bool {
        c == 0x3A || isASCIIAlpha(c) || c == 0x5F || (c >= 0x00C0 && c <= 0x00D6)
            || (c >= 0x00D8 && c <= 0x00F6) || (c >= 0x00F8 && c <= 0x02FF) || (c >= 0x0370 && c <= 0x037D) || (c >= 0x037F && c <= 0x1FFF)
            || (c >= 0x200C && c <= 0x200D) || (c >= 0x2070 && c <= 0x218F) || (c >= 0x2C00 && c <= 0x2FEF) || (c >= 0x3001 && c <= 0xD7FF)
            || (c >= 0xF900 && c <= 0xFDCF) || (c >= 0xFDF0 && c <= 0xFFFD) || (c >= 0x10000 && c <= 0xEFFFF)
    }

    // isValidXMLNamePart: https://www.w3.org/TR/xml/#NT-NameChar
    @inline(__always)
    static func isValidXMLNamePart(_ c: UInt32) -> Bool {
        isValidXMLNameStart(c) || c == 0x2D || c == 0x2E || isASCIIDigit(c) || c == 0x00B7
            || (c >= 0x0300 && c <= 0x036F) || (c >= 0x203F && c <= 0x2040)
    }

    // !name.contains(c in set).
    @inline(__always)
    static func containsNone<Units: NameValidationUnits & ~Escapable>(_ name: Units, _ set: UInt64) -> Bool {
        for index in 0..<name.count where isInSet(name[index], set) {
            return false
        }
        return true
    }

    // MARK: The checks

    // !name.isEmpty() && !name.contains(isInvalidAttributeNameCharacter).
    @inline(__always)
    static func isValidAttributeName<Units: NameValidationUnits & ~Escapable>(_ name: Units) -> Bool {
        name.count > 0 && containsNone(name, invalidAttributeNameCharacters)
    }

    // https://dom.spec.whatwg.org/#valid-element-name
    @inline(__always)
    static func isValidElementName<Units: NameValidationUnits & ~Escapable>(_ name: Units) -> Bool {
        let count = name.count
        if count == 0 {
            return false
        }
        let firstCharacter = name[0]
        if isASCIIAlpha(UInt32(firstCharacter)) {
            return containsNone(name, invalidElementNameCharactersAfterAlphaStart)
        }
        if firstCharacter == 0x3A || firstCharacter == 0x5F || firstCharacter >= 0x80 {
            for index in 0..<count where !isValidElementNameContinuationCharacter(name[index]) {
                return false
            }
            return true
        }
        return false
    }

    // Every check but the Latin-1 attribute and element name checks, which the C entry point inlines.
    @inline(never)
    static func isValidName<Units: NameValidationUnits & ~Escapable>(_ name: Units, kind: UInt8) -> Bool {
        switch kind {
        case attributeName:
            return isValidAttributeName(name)
        case elementName:
            return isValidElementName(name)
        case namespacePrefix:
            // !prefix.isEmpty() && !prefix.contains(isInvalidNamespacePrefixCharacter).
            return name.count > 0 && containsNone(name, invalidElementNameCharactersAfterAlphaStart)
        case doctypeName:
            // !name.contains(isInvalidDoctypeNameCharacter).
            return containsNone(name, invalidDoctypeNameCharacters)
        default:
            return isValidXMLName(name)
        }
    }

    // isValidXMLName: the ASCII fast path of the C++ (isValidASCIIXMLName) accepts exactly the all-ASCII names the
    // code point loop accepts, so only the loop is ported. The code points are StringView::codePoints()'s: Latin-1
    // units as they are; for UTF-16 a lead surrogate followed by a trail surrogate is one supplementary code point,
    // any other unit is its own code point (lone surrogates fail both predicates).
    static func isValidXMLName<Units: NameValidationUnits & ~Escapable>(_ name: Units) -> Bool {
        let count = name.count
        var index = 0
        var isFirst = true
        while index < count {
            var codePoint = UInt32(name[index])
            index &+= 1
            if codePoint & 0xFC00 == 0xD800 && index < count {
                let trail = UInt32(name[index])
                if trail & 0xFC00 == 0xDC00 {
                    codePoint = 0x10000 &+ ((codePoint &- 0xD800) &<< 10) &+ (trail &- 0xDC00)
                    index &+= 1
                }
            }
            if !(isFirst ? isValidXMLNameStart(codePoint) : isValidXMLNamePart(codePoint)) {
                return false
            }
            isFirst = false
        }
        return !isFirst
    }

    // MARK: isValidAttributeName, Latin-1

    // The lanes holding one of the invalid attribute name characters.
    @inline(__always)
    static func invalidAttributeNameCharacters(_ v: SIMD8<UInt8>) -> SIMDMask<SIMD8<Int8>> {
        let whitespace = (v .== SIMD8(repeating: 0x09)) .| (v .== SIMD8(repeating: 0x0A)) .| (v .== SIMD8(repeating: 0x0C)) .| (v .== SIMD8(repeating: 0x0D)) .| (v .== SIMD8(repeating: 0x20))
        let other = (v .== SIMD8(repeating: 0x00)) .| (v .== SIMD8(repeating: 0x2F)) .| (v .== SIMD8(repeating: 0x3D)) .| (v .== SIMD8(repeating: 0x3E))
        return whitespace .| other
    }

    @inline(__always)
    static func hasAnyLane(_ mask: SIMDMask<SIMD8<Int8>>) -> Bool {
        bitCast(SIMD8<UInt8>(repeating: 0).replacing(with: 0xFF, where: mask), to: UInt64.self) != 0
    }

    @inline(__always)
    static func hasInvalidAttributeNameCharacter(_ v: SIMD16<UInt8>) -> Bool {
        let whitespace = (v .== SIMD16(repeating: 0x09)) .| (v .== SIMD16(repeating: 0x0A)) .| (v .== SIMD16(repeating: 0x0C)) .| (v .== SIMD16(repeating: 0x0D)) .| (v .== SIMD16(repeating: 0x20))
        let other = (v .== SIMD16(repeating: 0x00)) .| (v .== SIMD16(repeating: 0x2F)) .| (v .== SIMD16(repeating: 0x3D)) .| (v .== SIMD16(repeating: 0x3E))
        let bits = bitCast(SIMD16<UInt8>(repeating: 0).replacing(with: 0xFF, where: whitespace .| other), to: SIMD2<UInt64>.self)
        return (bits[0] | bits[1]) != 0
    }

    // isValidAttributeName(StringView) for Latin-1: !name.isEmpty() && !name.contains(isInvalidAttributeNameCharacter).
    // Inlined into the C entry point so that the call from C++ has no further call or stack frame.
    @inline(__always)
    static func isValidAttributeName(_ name: Span<UInt8>) -> Bool {
        let count = name.count
        let bytes = name.bytes
        if count > 16 {
            var offset = 0
            while offset &+ 16 < count {
                if hasInvalidAttributeNameCharacter(bytes.load(fromByteOffset: offset, as: SIMD16<UInt8>.self)) {
                    return false
                }
                offset &+= 16
            }
            // The last 16 bytes, overlapping bytes already checked.
            return !hasInvalidAttributeNameCharacter(bytes.load(fromByteOffset: count &- 16, as: SIMD16<UInt8>.self))
        }
        if count >= 8 {
            // Two possibly overlapping 8-byte words cover 8...16 bytes.
            let first = bytes.load(fromByteOffset: 0, as: SIMD8<UInt8>.self)
            let last = bytes.load(fromByteOffset: count &- 8, as: SIMD8<UInt8>.self)
            return !hasAnyLane(invalidAttributeNameCharacters(first) .| invalidAttributeNameCharacters(last))
        }
        let word: UInt64
        if count >= 4 {
            // Two possibly overlapping 4-byte words cover 4...8 bytes.
            let low = UInt64(bytes.load(fromByteOffset: 0, as: UInt32.self))
            let high = UInt64(bytes.load(fromByteOffset: count &- 4, as: UInt32.self))
            word = low | (high &<< 32)
        } else {
            if count == 0 {
                return false
            }
            // 1...3 bytes: positions 0, count / 2 and count - 1 cover them all; the other lanes get 'a' (valid).
            word = UInt64(name[0]) | (UInt64(name[count &>> 1]) &<< 8) | (UInt64(name[count &- 1]) &<< 16) | 0x6161_6161_6100_0000
        }
        return !hasAnyLane(invalidAttributeNameCharacters(bitCast(word, to: SIMD8<UInt8>.self)))
    }
}
