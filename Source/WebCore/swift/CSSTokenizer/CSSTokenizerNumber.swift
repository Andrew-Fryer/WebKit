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


// The numeric value of a CSS <number-token> (CSSTokenizerInputStream::getDouble, i.e. WTF::charactersToDouble):
// fast_float 8.2.10 (Source/WTF/wtf/fast_float) from_chars with chars_format::general | no_infnan |
// allow_leading_plus, as WTF::parseDouble (FastFloat.cpp) calls it, on the number's text. The tokenizer's scan
// accumulates the parsed number string as it consumes the digits (fast_float::parse_number_string for the CSS
// number grammar, which is a subset of fast_float's); then Clinger's fast path, the Eisel-Lemire algorithm
// and (CSSTokenizerNumberBigint.swift) the big-integer digit comparison. Forward-ported from swift-wtf's
// fuzzed NumberConversionParse.swift. Results are bit-identical to WTF::charactersToDouble for every number
// text the tokenizer produces: the text always parses completely, so getDouble never takes its 0.0 failure path.
// Strictly memory-safe: no unsafe constructs in this file.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum CSSTokenizerNumber {
}

extension CSSTokenizerNumber {
    struct ParsedNumberString {
        var exponent: Int64 = 0
        var mantissa: UInt64 = 0
        var lastMatch: Int = 0
        var negative = false
        var valid = false
        var tooManyDigits = false
        var integerStart = 0
        var integerLength = 0
        // -1 when there is no decimal point (fast_float's null fraction span).
        var fractionStart = -1
        var fractionLength = 0
    }

    struct AdjustedMantissa: Equatable {
        var mantissa: UInt64 = 0
        var power2: Int32 = 0
    }

    static let fastFloatInvalidAmBias: Int32 = -0x8000

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isDigitCharacter(_ c: UInt16) -> Bool {
        c &- 0x30 <= 9
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func digitValue(_ c: UInt16) -> UInt64 {
        UInt64(c &- 0x30)
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func isCharacter(_ c: UInt16, _ ascii: UInt8) -> Bool {
        c == UInt16(ascii)
    }

    static let powersOfTenUInt64: InlineArray<20, UInt64> = [
        1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
        10000000000, 100000000000, 1000000000000, 10000000000000, 100000000000000,
        1000000000000000, 10000000000000000, 100000000000000000, 1000000000000000000,
        10000000000000000000,
    ]

    // High 64 bits of x * y.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func mulHigh(_ x: UInt64, _ y: UInt64) -> UInt64 {
        x.multipliedFullWidth(by: y).high
    }

    // The value of the number whose text is s[start..<end], given the parsed number string the tokenizer
    // accumulated while scanning it: `mantissa` folds every digit (wrapping), `exponent` is the explicit
    // exponent minus the fraction digit count and `digitCount` counts the integer and fraction digits
    // (fast_float::parse_number_string before its more-than-19-digits re-count).
    // fast_float::from_chars_float_advanced<double> for that text.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func value<C: CSSTokenizerCharacters & ~Escapable>(_ s: C, _ start: Int, _ end: Int, mantissa: UInt64, exponent: Int64, digitCount: Int, negative: Bool) -> Double {
        if digitCount <= 19 {
            if exponent >= -22 && exponent <= 22 && mantissa <= (1 << 53) {
                // Clinger's fast path (round-to-nearest mode).
                var value = Double(mantissa)
                if exponent < 0 {
                    value = value / powerOfTen(Int(0 &- exponent))
                } else {
                    value = value * powerOfTen(Int(exponent))
                }
                return negative ? -value : value
            }
            let am = computeFloat(exponent, mantissa)
            if am.power2 >= 0 {
                return toDouble(negative, am)
            }
        }
        return slowValue(s, start, end)
    }

    // 1e0 ... 1e22, exactly (fast_float's binary_format<double>::exact_power_of_ten), without a table.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func powerOfTen(_ exponent: Int) -> Double {
        switch exponent {
        case 0: 1e0
        case 1: 1e1
        case 2: 1e2
        case 3: 1e3
        case 4: 1e4
        case 5: 1e5
        case 6: 1e6
        case 7: 1e7
        case 8: 1e8
        case 9: 1e9
        case 10: 1e10
        case 11: 1e11
        case 12: 1e12
        case 13: 1e13
        case 14: 1e14
        case 15: 1e15
        case 16: 1e16
        case 17: 1e17
        case 18: 1e18
        case 19: 1e19
        case 20: 1e20
        case 21: 1e21
        default: 1e22
        }
    }

    // fast_float::parse_number_string<..., char> for chars_format::general | allow_leading_plus, on s[start..<end]
    // (which is never empty). Positions in the result are absolute indices into s.
    static func parseNumberString<C: CSSTokenizerCharacters & ~Escapable>(_ s: C, _ start: Int, _ end: Int) -> ParsedNumberString {
        var answer = ParsedNumberString()
        var p = start
        let first = s[p]
        answer.negative = isCharacter(first, 0x2D)
        if isCharacter(first, 0x2D) || isCharacter(first, 0x2B) {
            p &+= 1
            if p == end {
                answer.lastMatch = p
                return answer
            }
            if !isDigitCharacter(s[p]) && !isCharacter(s[p], 0x2E) {
                answer.lastMatch = p
                return answer
            }
        }
        let startDigits = p
        var i: UInt64 = 0
        while p < end && isDigitCharacter(s[p]) {
            i = 10 &* i &+ digitValue(s[p])
            p &+= 1
        }
        let endOfIntegerPart = p
        var digitCount = Int64(endOfIntegerPart &- startDigits)
        answer.integerStart = startDigits
        answer.integerLength = Int(digitCount)

        var exponent: Int64 = 0
        let hasDecimalPoint = p < end && isCharacter(s[p], 0x2E)
        if hasDecimalPoint {
            p &+= 1
            let before = p
            while p < end && isDigitCharacter(s[p]) {
                i = i &* 10 &+ digitValue(s[p])
                p &+= 1
            }
            exponent = Int64(before &- p)
            answer.fractionStart = before
            answer.fractionLength = p &- before
            digitCount &-= exponent
        }
        if digitCount == 0 {
            answer.lastMatch = p
            return answer
        }

        var expNumber: Int64 = 0
        if p < end && (isCharacter(s[p], 0x65) || isCharacter(s[p], 0x45)) {
            let locationOfE = p
            p &+= 1
            var negativeExponent = false
            if p < end && isCharacter(s[p], 0x2D) {
                negativeExponent = true
                p &+= 1
            } else if p < end && isCharacter(s[p], 0x2B) {
                p &+= 1
            }
            if p == end || !isDigitCharacter(s[p]) {
                p = locationOfE
            } else {
                while p < end && isDigitCharacter(s[p]) {
                    if expNumber < 0x10000000 {
                        expNumber = 10 &* expNumber &+ Int64(digitValue(s[p]))
                    }
                    p &+= 1
                }
                if negativeExponent {
                    expNumber = 0 &- expNumber
                }
                exponent &+= expNumber
            }
        }
        answer.lastMatch = p
        answer.valid = true

        if digitCount > 19 {
            var start = startDigits
            while start < end && (isCharacter(s[start], 0x30) || isCharacter(s[start], 0x2E)) {
                if isCharacter(s[start], 0x30) {
                    digitCount &-= 1
                }
                start &+= 1
            }
            if digitCount > 19 {
                answer.tooManyDigits = true
                i = 0
                p = answer.integerStart
                let integerEnd = p &+ answer.integerLength
                let minimalNineteenDigitInteger: UInt64 = 1000000000000000000
                while i < minimalNineteenDigitInteger && p != integerEnd {
                    i = i &* 10 &+ digitValue(s[p])
                    p &+= 1
                }
                if i >= minimalNineteenDigitInteger {
                    exponent = Int64(endOfIntegerPart &- p) &+ expNumber
                } else {
                    p = answer.fractionStart
                    let fractionEnd = p &+ answer.fractionLength
                    while i < minimalNineteenDigitInteger && p != fractionEnd {
                        i = i &* 10 &+ digitValue(s[p])
                        p &+= 1
                    }
                    exponent = Int64(answer.fractionStart &- p) &+ expNumber
                }
            }
        }
        answer.exponent = exponent
        answer.mantissa = i
        return answer
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func fastFloatPower(_ q: Int32) -> Int32 {
        (((152170 &+ 65536) &* q) >> 16) &+ 63
    }

    // fast_float::compute_product_approximation<55>.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func computeProductApproximation(_ q: Int64, _ w: UInt64) -> (high: UInt64, low: UInt64) {
        let index = 2 &* Int(q &+ 342)
        let first = w.multipliedFullWidth(by: fastFloatPowerOfFive128[index])
        var high = first.high
        var low = first.low
        let precisionMask: UInt64 = 0xFFFF_FFFF_FFFF_FFFF &>> 55
        if (high & precisionMask) == precisionMask {
            let secondHigh = mulHigh(w, fastFloatPowerOfFive128[index &+ 1])
            low &+= secondHigh
            if secondHigh > low {
                high &+= 1
            }
        }
        return (high, low)
    }

    // fast_float::compute_float<binary_format<double>> (Eisel-Lemire).
    static func computeFloat(_ q: Int64, _ wIn: UInt64) -> AdjustedMantissa {
        var answer = AdjustedMantissa()
        if wIn == 0 || q < -342 {
            return answer
        }
        if q > 308 {
            answer.power2 = 0x7FF
            return answer
        }
        let lz = wIn.leadingZeroBitCount
        let w = wIn &<< UInt64(lz)
        let product = computeProductApproximation(q, w)
        let upperBit = Int(truncatingIfNeeded: product.high &>> 63)
        let shift = upperBit &+ 64 &- 52 &- 3
        answer.mantissa = product.high &>> UInt64(shift)
        answer.power2 = fastFloatPower(Int32(truncatingIfNeeded: q)) &+ Int32(truncatingIfNeeded: upperBit &- lz) &+ 1023
        if answer.power2 <= 0 {
            if 0 &- answer.power2 &+ 1 >= 64 {
                answer.power2 = 0
                answer.mantissa = 0
                return answer
            }
            answer.mantissa &>>= UInt64(truncatingIfNeeded: 0 &- answer.power2 &+ 1)
            answer.mantissa &+= answer.mantissa & 1
            answer.mantissa &>>= 1
            answer.power2 = answer.mantissa < (1 << 52) ? 0 : 1
            return answer
        }
        if product.low <= 1 && q >= -4 && q <= 23 && (answer.mantissa & 3) == 1 {
            if (answer.mantissa &<< UInt64(shift)) == product.high {
                answer.mantissa &= ~1
            }
        }
        answer.mantissa &+= answer.mantissa & 1
        answer.mantissa &>>= 1
        if answer.mantissa >= (2 << 52) {
            answer.mantissa = 1 << 52
            answer.power2 &+= 1
        }
        answer.mantissa &= ~(1 << 52)
        if answer.power2 >= 0x7FF {
            answer.power2 = 0x7FF
            answer.mantissa = 0
        }
        return answer
    }

    // fast_float::compute_error<binary_format<double>>.
    static func computeError(_ q: Int64, _ wIn: UInt64) -> AdjustedMantissa {
        let lz = wIn.leadingZeroBitCount
        let w = wIn &<< UInt64(lz)
        let product = computeProductApproximation(q, w)
        let hilz = Int(truncatingIfNeeded: product.high &>> 63) ^ 1
        var answer = AdjustedMantissa()
        answer.mantissa = product.high &<< UInt64(hilz)
        let bias: Int32 = 52 &+ 1023
        answer.power2 = fastFloatPower(Int32(truncatingIfNeeded: q)) &+ bias &- Int32(truncatingIfNeeded: hilz &+ lz) &- 62 &+ fastFloatInvalidAmBias
        return answer
    }

#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    static func toDouble(_ negative: Bool, _ am: AdjustedMantissa) -> Double {
        var word = am.mantissa
        word |= UInt64(truncatingIfNeeded: Int64(am.power2)) &<< 52
        word |= (negative ? 1 : 0) &<< 63
        return Double(bitPattern: word)
    }

    // fast_float::parse_number_slow_path: re-parse with spans, then from_chars_advanced(pns, value).
    // Out-of-range magnitudes give +-infinity or +-0, as fast_float stores them (WTF::charactersToDouble keeps them).
    @inline(never)
    static func slowValue<C: CSSTokenizerCharacters & ~Escapable>(_ s: C, _ start: Int, _ end: Int) -> Double {
        let pns = parseNumberString(s, start, end)
        if !pns.tooManyDigits && pns.exponent >= -22 && pns.exponent <= 22 && pns.mantissa <= (1 << 53) {
            var value = Double(pns.mantissa)
            if pns.exponent < 0 {
                value = value / powerOfTen(Int(0 &- pns.exponent))
            } else {
                value = value * powerOfTen(Int(pns.exponent))
            }
            return pns.negative ? -value : value
        }
        var am = computeFloat(pns.exponent, pns.mantissa)
        if pns.tooManyDigits && am.power2 >= 0 {
            if am != computeFloat(pns.exponent, pns.mantissa &+ 1) {
                am = computeError(pns.exponent, pns.mantissa)
            }
        }
        if am.power2 < 0 {
            am = digitComp(s, pns, am)
        }
        return toDouble(pns.negative, am)
    }
}

#endif
