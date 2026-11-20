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


// CSSParserToken::stringToUnitType (the cssPrimitiveValueUnitFromTrie switch of CSSParserToken.cpp): the
// CSSUnitType of a dimension's unit name, ASCII case-insensitively, or CSSUnitType::Unknown. The result is the
// CSSUnitType raw value; CSSParserToken.cpp static_asserts every one of them.
// Strictly memory-safe: no unsafe constructs in this file.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum CSSTokenizerUnits {
    static let unknown: UInt8 = 0
    static let number: UInt8 = 1
    static let percentage: UInt8 = 3

    // The unit of the name s[start..<start + length].
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func unitType<C: CSSTokenizerCharacters & ~Escapable>(_ s: C, _ start: Int, _ length: Int) -> UInt8 {
        if length < 1 || length > 5 {
            return unknown
        }
        // The ASCII-lowercased name, one byte per character, and its length in bits 40-47.
        var key = UInt64(length) &<< 40
        for i in 0..<length {
            let c = s[start &+ i]
            if c >= 0x80 {
                return unknown
            }
            let lowered = c &- 0x41 < 26 ? c | 0x20 : c
            key |= UInt64(lowered) &<< UInt64(8 &* i)
        }
        return unitType(key: key)
    }

    static func unitType(key: UInt64) -> UInt8 {
        switch key {
        case 0x010000000071: 54 // "q": CSSUnitType::Q
        case 0x010000000073: 16 // "s": CSSUnitType::S
        case 0x010000000078: 50 // "x": CSSUnitType::X
        case 0x020000006863: 62 // "ch": CSSUnitType::Ch
        case 0x020000006D63: 7 // "cm": CSSUnitType::Cm
        case 0x020000006D65: 4 // "em": CSSUnitType::Em
        case 0x020000007865: 5 // "ex": CSSUnitType::Ex
        case 0x020000007266: 53 // "fr": CSSUnitType::Fr
        case 0x020000007A68: 17 // "hz": CSSUnitType::Hz
        case 0x020000006369: 64 // "ic": CSSUnitType::Ic
        case 0x020000006E69: 9 // "in": CSSUnitType::In
        case 0x02000000686C: 55 // "lh": CSSUnitType::Lh
        case 0x020000006D6D: 8 // "mm": CSSUnitType::Mm
        case 0x02000000736D: 15 // "ms": CSSUnitType::Ms
        case 0x020000006370: 11 // "pc": CSSUnitType::Pc
        case 0x020000007470: 10 // "pt": CSSUnitType::Pt
        case 0x020000007870: 6 // "px": CSSUnitType::Px
        case 0x020000006276: 23 // "vb": CSSUnitType::Vb
        case 0x020000006876: 20 // "vh": CSSUnitType::Vh
        case 0x020000006976: 24 // "vi": CSSUnitType::Vi
        case 0x020000007776: 19 // "vw": CSSUnitType::Vw
        case 0x030000706163: 60 // "cap": CSSUnitType::Cap
        case 0x030000627163: 46 // "cqb": CSSUnitType::Cqb
        case 0x030000687163: 44 // "cqh": CSSUnitType::Cqh
        case 0x030000697163: 45 // "cqi": CSSUnitType::Cqi
        case 0x030000777163: 43 // "cqw": CSSUnitType::Cqw
        case 0x030000676564: 12 // "deg": CSSUnitType::Deg
        case 0x030000697064: 51 // "dpi": CSSUnitType::Dpi
        case 0x030000627664: 41 // "dvb": CSSUnitType::Dvb
        case 0x030000687664: 38 // "dvh": CSSUnitType::Dvh
        case 0x030000697664: 42 // "dvi": CSSUnitType::Dvi
        case 0x030000777664: 37 // "dvw": CSSUnitType::Dvw
        case 0x0300007A686B: 18 // "khz": CSSUnitType::Khz
        case 0x03000062766C: 35 // "lvb": CSSUnitType::Lvb
        case 0x03000068766C: 32 // "lvh": CSSUnitType::Lvh
        case 0x03000069766C: 36 // "lvi": CSSUnitType::Lvi
        case 0x03000077766C: 31 // "lvw": CSSUnitType::Lvw
        case 0x030000646172: 13 // "rad": CSSUnitType::Rad
        case 0x030000686372: 63 // "rch": CSSUnitType::Rch
        case 0x0300006D6572: 58 // "rem": CSSUnitType::Rem
        case 0x030000786572: 59 // "rex": CSSUnitType::Rex
        case 0x030000636972: 65 // "ric": CSSUnitType::Ric
        case 0x030000686C72: 56 // "rlh": CSSUnitType::Rlh
        case 0x030000627673: 29 // "svb": CSSUnitType::Svb
        case 0x030000687673: 26 // "svh": CSSUnitType::Svh
        case 0x030000697673: 30 // "svi": CSSUnitType::Svi
        case 0x030000777673: 25 // "svw": CSSUnitType::Svw
        case 0x04006D637064: 52 // "dpcm": CSSUnitType::Dpcm
        case 0x040078707064: 49 // "dppx": CSSUnitType::Dppx
        case 0x040064617267: 14 // "grad": CSSUnitType::Grad
        case 0x040070616372: 61 // "rcap": CSSUnitType::Rcap
        case 0x04006E727574: 57 // "turn": CSSUnitType::Turn
        case 0x040078616D76: 22 // "vmax": CSSUnitType::Vmax
        case 0x04006E696D76: 21 // "vmin": CSSUnitType::Vmin
        case 0x056D65715F5F: 69 // "__qem": CSSUnitType::QuirkyEm
        case 0x0578616D7163: 48 // "cqmax": CSSUnitType::Cqmax
        case 0x056E696D7163: 47 // "cqmin": CSSUnitType::Cqmin
        case 0x0578616D7664: 40 // "dvmax": CSSUnitType::Dvmax
        case 0x056E696D7664: 39 // "dvmin": CSSUnitType::Dvmin
        case 0x0578616D766C: 34 // "lvmax": CSSUnitType::Lvmax
        case 0x056E696D766C: 33 // "lvmin": CSSUnitType::Lvmin
        case 0x0578616D7673: 28 // "svmax": CSSUnitType::Svmax
        case 0x056E696D7673: 27 // "svmin": CSSUnitType::Svmin
        default: unknown
        }
    }
}

#endif
