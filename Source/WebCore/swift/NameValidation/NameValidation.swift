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

// Memory-safe port of the name checks of dom/NameValidation.cpp: isValidAttributeName(StringView) and
// isValidElementName(StringView), including their character predicates (isInvalidAttributeNameCharacter,
// isInvalidElementNameCharacterAfterAlphaStart, isValidElementNameContinuationCharacter).
//
// Latin-1 names arrive as a Span<UInt8>. UTF-16 names arrive as a RawSpan (2 bytes per code unit, any alignment) and
// every code unit is compared whole, so U+0120 (low byte ' ') or U+013D (low byte '=') stay valid.
//
// isValidAttributeName is hot (setAttribute); its Latin-1 path tests 8 or 16 bytes per step with SIMD compares and
// overlapping loads instead of the C++ per-character loop. The answer is an any-match over the bytes, so checking a
// byte twice is harmless. No allocation, no reference counting, no traps beyond the Span bounds checks (which cannot
// fire: every load is in range by construction).

enum NameValidation {
}

extension NameValidation {
    // The invalid ASCII code units as bit sets over c < 64 (every code unit >= 64 is valid in both sets):
    // isInvalidAttributeNameCharacter: !c || isASCIIWhitespace(c) || c == '/' || c == '>' || c == '='
    //   = { NUL, '\t', '\n', '\f', '\r', ' ', '/', '=', '>' }. '\v' (0x0B) is not ASCII whitespace.
    // isInvalidElementNameCharacterAfterAlphaStart: the same without '='.
    static var invalidAttributeNameCharacters: UInt64 { 0x6000_8001_0000_3601 }
    static var invalidElementNameCharactersAfterAlphaStart: UInt64 { 0x4000_8001_0000_3601 }

    @inline(__always)
    static func isInSet(_ character: UInt16, _ set: UInt64) -> Bool {
        character < 64 && (set &>> UInt64(character)) & 1 != 0
    }

    @inline(__always)
    static func isASCIIAlpha(_ character: UInt16) -> Bool {
        (character | 0x20) &- 0x61 < 26
    }

    // isValidElementNameContinuationCharacter: isASCIIAlphanumeric(c) || c == '-' || c == '.' || c == ':' || c == '_'
    // || !isASCII(c).
    @inline(__always)
    static func isValidElementNameContinuationCharacter(_ character: UInt16) -> Bool {
        if character >= 0x80 {
            return true
        }
        return isASCIIAlpha(character) || character &- 0x30 < 10 || character == 0x2D || character == 0x2E || character == 0x3A || character == 0x5F
    }

    // MARK: isValidAttributeName

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
    // Inlined into its C entry point so that the call from C++ has no further call or stack frame.
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

    // isValidAttributeName(StringView) for UTF-16 (no UTF-16 names in the profiles; a plain loop).
    static func isValidAttributeName(utf16 name: RawSpan) -> Bool {
        let count = name.byteCount &>> 1
        if count == 0 {
            return false
        }
        let set = invalidAttributeNameCharacters
        for index in 0..<count where isInSet(name.load(fromByteOffset: index &* 2, as: UInt16.self), set) {
            return false
        }
        return true
    }

    // MARK: isValidElementName

    // isValidElementName(StringView) for Latin-1 (inlined into its C entry point).
    @inline(__always)
    static func isValidElementName(_ name: Span<UInt8>) -> Bool {
        let count = name.count
        if count == 0 {
            return false
        }
        let firstCharacter = UInt16(name[0])
        if isASCIIAlpha(firstCharacter) {
            let set = invalidElementNameCharactersAfterAlphaStart
            for index in 0..<count where isInSet(UInt16(name[index]), set) {
                return false
            }
            return true
        }
        if firstCharacter == 0x3A || firstCharacter == 0x5F || firstCharacter >= 0x80 {
            for index in 0..<count where !isValidElementNameContinuationCharacter(UInt16(name[index])) {
                return false
            }
            return true
        }
        return false
    }

    // isValidElementName(StringView) for UTF-16.
    static func isValidElementName(utf16 name: RawSpan) -> Bool {
        let count = name.byteCount &>> 1
        if count == 0 {
            return false
        }
        let firstCharacter = name.load(fromByteOffset: 0, as: UInt16.self)
        if isASCIIAlpha(firstCharacter) {
            let set = invalidElementNameCharactersAfterAlphaStart
            for index in 0..<count where isInSet(name.load(fromByteOffset: index &* 2, as: UInt16.self), set) {
                return false
            }
            return true
        }
        if firstCharacter == 0x3A || firstCharacter == 0x5F || firstCharacter >= 0x80 {
            for index in 0..<count where !isValidElementNameContinuationCharacter(name.load(fromByteOffset: index &* 2, as: UInt16.self)) {
                return false
            }
            return true
        }
        return false
    }
}
