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

// Two text-run scans of WebCore's text code:
//  * FontCascade::characterRangeCodePath(std::span<const char16_t>) (platform/graphics/FontCascade.cpp): which
//    FontCascade::CodePath a UTF-16 run needs, a cascade of code point range checks. Transcribed statement by statement.
//  * The per-character loop of Layout::TextUtil::canUseSimplifiedTextMeasuring (layout/formattingContexts/inline/text/
//    TextUtil.cpp), as far as FontCascade::canUseSimplifiedTextMeasuring(character, FontVariant::Auto, ...) answers
//    from its per-font cache: scanSimplifiedMeasuring stops at the first character the cache cannot answer, and the
//    C++ glue asks FontCascade (glyph lookup, which also fills the cache) about it and calls again after it.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum TextRunScanners {
    // FontCascadeCodePath raw values (FontCascadeEnums.h).
    static let simple: UInt8 = 0
    static let complex: UInt8 = 1
    static let simpleWithGlyphOverflow: UInt8 = 2

    // isEmojiGroupCandidate (wtf/text/CharacterProperties.h) for a supplementary code point: ublock_getCode() is one of
    // Miscellaneous Symbols and Pictographs, Emoticons, Transport and Map Symbols, Supplemental Symbols and Pictographs,
    // Symbols and Pictographs Extended-A. (The other two blocks it accepts, Miscellaneous Symbols and Dingbats, are in
    // the BMP.) Unicode block ranges never change once allocated.
    @inline(__always)
    static func isSupplementaryEmojiGroupCandidate(_ character: UInt32) -> Bool {
        (character >= 0x1F300 && character <= 0x1F64F) || (character >= 0x1F680 && character <= 0x1F6FF)
            || (character >= 0x1F900 && character <= 0x1F9FF) || (character >= 0x1FA70 && character <= 0x1FAFF)
    }

    // isEmojiFitzpatrickModifier (wtf/text/CharacterProperties.h).
    @inline(__always)
    static func isEmojiFitzpatrickModifier(_ character: UInt32) -> Bool {
        character >= 0x1F3FB && character <= 0x1F3FF
    }

    @inline(__always) // into its C entry point: one call, not two.
    static func characterRangeCodePath(_ span: Span<UInt16>) -> UInt8 {
        let zeroWidthJoiner: UInt16 = 0x200D
        var result = simple
        var previousCharacterIsEmojiGroupCandidate = false
        let size = span.count
        var i = 0
        // `i` is advanced on entry, so every C++ `continue` (which runs ++i) is a plain `continue` here.
        while i < size {
            let c = span[i]
            i &+= 1
            // The C++ first checks for a ZWJ after an emoji group candidate, then clears the flag, then skips
            // c < U+02E5. ZWJ (U+200D) is not below U+02E5, so testing c < U+02E5 first is the same and keeps the
            // loop for Latin text short.
            if c < 0x2E5 { // U+02E5 through U+02E9 (Modifier Letters : Tone letters)
                previousCharacterIsEmojiGroupCandidate = false
                continue
            }
            if c == zeroWidthJoiner && previousCharacterIsEmojiGroupCandidate {
                return complex
            }

            previousCharacterIsEmojiGroupCandidate = false
            if c <= 0x2E9 { return complex }

            if c < 0x300 { continue } // U+0300 through U+036F Combining diacritical marks
            if c <= 0x36F { return complex }

            if c < 0x0591 || c == 0x05BE { continue } // U+0591 through U+05CF excluding U+05BE Hebrew combining marks, ...
            if c <= 0x05CF { return complex }

            // U+0600 through U+109F Arabic, Syriac, Thaana, NKo, Samaritan, Mandaic, Devanagari, ..., Tibetan, Myanmar
            if c < 0x0600 { continue }
            if c <= 0x109F { return complex }

            // U+1100 through U+11FF Hangul Jamo
            if c < 0x1100 { continue }
            if c <= 0x11FF { return complex }

            if c < 0x135D { continue } // U+135D through U+135F Ethiopic combining marks
            if c <= 0x135F { return complex }

            if c < 0x1700 { continue } // U+1780 through U+18AF Tagalog, Hanunoo, Buhid, Taghanwa,Khmer, Mongolian
            if c <= 0x18AF { return complex }

            if c < 0x1900 { continue } // U+1900 through U+194F Limbu (Unicode 4.0)
            if c <= 0x194F { return complex }

            if c < 0x1980 { continue } // U+1980 through U+19DF New Tai Lue
            if c <= 0x19DF { return complex }

            if c < 0x1A00 { continue } // U+1A00 through U+1CFF Buginese, Tai Tham, Balinese, Batak, Lepcha, Vedic
            if c <= 0x1CFF { return complex }

            if c < 0x1DC0 { continue } // U+1DC0 through U+1DFF Comining diacritical mark supplement
            if c <= 0x1DFF { return complex }

            // U+1E00 through U+2000 characters with diacritics and stacked diacritics
            if c <= 0x2000 {
                result = simpleWithGlyphOverflow
                continue
            }

            if c < 0x20D0 { continue } // U+20D0 through U+20FF Combining marks for symbols
            if c <= 0x20FF { return complex }

            if c < 0x26F9 { continue }
            if c < 0x26FA { return complex }

            if c < 0x2CEF { continue } // U+2CEF through U+2CF1 Combining marks for Coptic
            if c <= 0x2CF1 { return complex }

            if c < 0x302A { continue } // U+302A through U+302F Ideographic and Hangul Tone marks
            if c <= 0x302F { return complex }

            if c < 0x3099 { continue }
            if c < 0x309D { return complex } // KATAKANA-HIRAGANA (SEMI-)VOICED SOUND MARKS require character composition

            if c < 0xA67C { continue } // U+A67C through U+A67D Combining marks for old Cyrillic
            if c <= 0xA67D { return complex }

            if c < 0xA6F0 { continue } // U+A6F0 through U+A6F1 Combining mark for Bamum
            if c <= 0xA6F1 { return complex }

            // U+A800 through U+ABFF Nagri, Phags-pa, Saurashtra, Devanagari Extended, Hangul Jamo Ext. A, ...
            if c < 0xA800 { continue }
            if c <= 0xABFF { return complex }

            if c < 0xD7B0 { continue } // U+D7B0 through U+D7FF Hangul Jamo Ext. B
            if c <= 0xD7FF { return complex }

            if c <= 0xDBFF {
                // High surrogate

                if i == size { continue }

                // char16_t next = span[++i]: the next code unit is consumed even when it is not a trail surrogate.
                let next = span[i]
                i &+= 1
                if next & 0xFC00 != 0xDC00 { continue } // !U16_IS_TRAIL(next)

                // U16_GET_SUPPLEMENTARY(c, next)
                let supplementaryCharacter = (UInt32(c) &<< 10) &+ UInt32(next) &- ((0xD800 &<< 10) &+ 0xDC00 &- 0x10000)

                if supplementaryCharacter < 0x10A00 { continue }
                if supplementaryCharacter < 0x10A60 { return complex } // Kharoshthi
                if supplementaryCharacter < 0x11000 { continue }
                if supplementaryCharacter < 0x11080 { return complex } // Brahmi
                if supplementaryCharacter < 0x110D0 { return complex } // Kaithi
                if supplementaryCharacter < 0x11100 { continue }
                if supplementaryCharacter < 0x11150 { return complex } // Chakma
                if supplementaryCharacter < 0x11180 { return complex } // Mahajani
                if supplementaryCharacter < 0x111E0 { return complex } // Sharada
                if supplementaryCharacter < 0x11200 { continue }
                if supplementaryCharacter < 0x11250 { return complex } // Khojki
                if supplementaryCharacter < 0x112B0 { continue }
                if supplementaryCharacter < 0x11300 { return complex } // Khudawadi
                if supplementaryCharacter < 0x11380 { return complex } // Grantha
                if supplementaryCharacter < 0x11400 { continue }
                if supplementaryCharacter < 0x11480 { return complex } // Newa
                if supplementaryCharacter < 0x114E0 { return complex } // Tirhuta
                if supplementaryCharacter < 0x11580 { continue }
                if supplementaryCharacter < 0x11600 { return complex } // Siddham
                if supplementaryCharacter < 0x11660 { return complex } // Modi
                if supplementaryCharacter < 0x11680 { continue }
                if supplementaryCharacter < 0x116D0 { return complex } // Takri
                if supplementaryCharacter < 0x11700 { continue }
                if supplementaryCharacter < 0x11C00 { return complex } // Ahom, Dogra, Dives Akuru, Nandinagari, ...
                if supplementaryCharacter < 0x11C70 { return complex } // Bhaiksuki
                if supplementaryCharacter < 0x11CC0 { return complex } // Marchen
                if supplementaryCharacter < 0x16B00 { continue }
                if supplementaryCharacter < 0x16B90 { return complex } // Pahawh Hmong
                if supplementaryCharacter < 0x1E900 { continue }
                if supplementaryCharacter < 0x1E960 { return complex } // Adlam
                if supplementaryCharacter < 0x1F1E6 { continue } // U+1F1E6 through U+1F1FF Regional Indicator Symbols
                if supplementaryCharacter <= 0x1F1FF { return complex }

                if isEmojiFitzpatrickModifier(supplementaryCharacter) { return complex }
                if isSupplementaryEmojiGroupCandidate(supplementaryCharacter) {
                    previousCharacterIsEmojiGroupCandidate = true
                    continue
                }

                if supplementaryCharacter < 0xE0000 { continue }
                if supplementaryCharacter < 0xE0080 { return complex } // Tags
                if supplementaryCharacter < 0xE0100 { continue } // U+E0100 through U+E01EF Unicode variation selectors.
                if supplementaryCharacter <= 0xE01EF { return complex }

                continue
            }

            if c < 0xFE00 { continue } // U+FE00 through U+FE0F Unicode variation selectors
            if c <= 0xFE0F { return complex }

            if c < 0xFE20 { continue } // U+FE20 through U+FE2F Combining half marks
            if c <= 0xFE2F { return complex }
        }
        return result
    }

    // The loop `for (character : characters) if (!fontCascade.canUseSimplifiedTextMeasuring(character, FontVariant::Auto,
    // whitespaceIsCollapsed, primaryFont)) return false;` as far as the answers need no glyph lookup.
    // `cache` is FontCascade::m_canUseSimplifiedTextMeasuringForAutoVariantCache (BitSet<512> words): for a Latin-1
    // character c, bit 2c says the answer (for whitespaceIsCollapsed = true) is cached and bit 2c + 1 is that answer.
    // Returns index << 1 | isFalse: isFalse = 1 when characters[index] is known to answer false (a tab when whitespace
    // is not collapsed, which is answered before the cache, or a cached false); otherwise characters[index] is the
    // first character the cache does not answer (not cached, or not Latin-1), or index == count.
    @inline(__always)
    static func scanSimplifiedMeasuring<Character: FixedWidthInteger & UnsignedInteger>(_ characters: Span<Character>, cache: Span<UInt64>, whitespaceIsCollapsed: Bool) -> Int {
        precondition(cache.count == 8)
        let count = characters.count
        var index = 0
        while index < count {
            let character = UInt32(characters[index])
            if character > 0xFF {
                return index &<< 1
            }
            if character == 0x09 && !whitespaceIsCollapsed { // tabCharacter
                return index &<< 1 | 1
            }
            let word = cache[Int(character &>> 5)] &>> UInt64((character &<< 1) & 63)
            if word & 1 == 0 {
                return index &<< 1
            }
            if word & 2 == 0 {
                return index &<< 1 | 1
            }
            index &+= 1
        }
        return count &<< 1
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
