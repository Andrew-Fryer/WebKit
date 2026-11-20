// Derived from fast_float 8.2.10 by Daniel Lemire, João Paulo Magalhaes and contributors
// (https://github.com/fastfloat/fast_float), formerly vendored as Source/WTF/wtf/fast_float/fast_float.h,
// licensed under the Apache License 2.0 (LICENSE-fast_float in this directory), the MIT license or the
// Boost Software License 1.0, at your option.
//
// Port of fast_float 8.2.10 as used by WTF::parseDouble,
// WTF::parseFixedDouble and WTF::parseJSONDouble (FastFloat.cpp): the number scanner, Clinger's
// fast path, the Eisel-Lemire algorithm and the big-integer digit comparison slow path.
// Results (value, consumed length, error) are identical to fast_float::from_chars for every input.
// Strictly memory-safe: no unsafe constructs in this file. _slowPath/_fastPath are only branch-weight hints
// (llvm.expect) that give the code a block layout close to the PGO-optimized C++ when this module is built
// without a profile that covers it; they change no result.

enum NumberParse {}

extension NumberParse {
    // The fast_float chars_format of each WTF entry point, as a type so that every format gets its own
    // specialization of the parser with the format checks folded away.
    protocol ParseFormat {
        static var allowsLeadingPlus: Bool { get }
        static var isJSON: Bool { get }
        static var allowsScientific: Bool { get }
    }
    // Every format WTF uses has the fixed bit set (fast_float's chars_format::fixed).
    static let formatAllowsFixed = true

    // chars_format::general | no_infnan | allow_leading_plus (parseDouble)
    enum GeneralFormat: ParseFormat {
        static var allowsLeadingPlus: Bool { true }
        static var isJSON: Bool { false }
        static var allowsScientific: Bool { true }
    }

    // chars_format::fixed | no_infnan (parseFixedDouble)
    enum FixedFormat: ParseFormat {
        static var allowsLeadingPlus: Bool { false }
        static var isJSON: Bool { false }
        static var allowsScientific: Bool { false }
    }

    // chars_format::json (parseJSONDouble)
    enum JSONFormat: ParseFormat {
        static var allowsLeadingPlus: Bool { false }
        static var isJSON: Bool { true }
        static var allowsScientific: Bool { true }
    }

    enum ParseStatus {
        case ok
        case outOfRange
        case invalid
    }

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

    @inline(__always)
    static func isDigitCharacter<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ c: C) -> Bool {
        UInt32(truncatingIfNeeded: c) &- 0x30 <= 9
    }

    @inline(__always)
    static func digitValue<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ c: C) -> UInt64 {
        UInt64(truncatingIfNeeded: c) &- 0x30
    }

    @inline(__always)
    static func isCharacter<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ c: C, _ ascii: UInt8) -> Bool {
        c == C(truncatingIfNeeded: ascii)
    }

    // fast_float::parse_number_string.
    @inline(__always)
    static func parseNumberString<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, format: F.Type, storeSpans: Bool) -> ParsedNumberString {
        var answer = ParsedNumberString()
        let end = s.count
        var p = 0
        let first = s[0]
        answer.negative = isCharacter(first, 0x2D)
        // fast_float returns early when the sign is not followed by a digit (or, outside JSON, a decimal
        // point). Without those returns both the integer part and the fraction are empty, so the digit-count
        // checks below reject exactly the same inputs; an invalid parse reports no length, so where it stopped
        // does not matter. This keeps the common unsigned path free of the sign branches.
        let hasSign = isCharacter(first, 0x2D) || (F.allowsLeadingPlus && isCharacter(first, 0x2B))
        p = hasSign ? 1 : 0
        let startDigits = p
        var i: UInt64 = 0
        // Like fast_float, peel the first digits (most integer parts are short) before looping.
        if p < end && isDigitCharacter(s[p]) {
            i = digitValue(s[p])
            p &+= 1
            if p < end && isDigitCharacter(s[p]) {
                i = 10 &* i &+ digitValue(s[p])
                p &+= 1
                if p < end && isDigitCharacter(s[p]) {
                    i = 10 &* i &+ digitValue(s[p])
                    p &+= 1
                    if p < end && isDigitCharacter(s[p]) {
                        i = 10 &* i &+ digitValue(s[p])
                        p &+= 1
                        if p < end && isDigitCharacter(s[p]) {
                            i = 10 &* i &+ digitValue(s[p])
                            p &+= 1
                            while p < end && isDigitCharacter(s[p]) {
                                i = 10 &* i &+ digitValue(s[p])
                                p &+= 1
                            }
                        }
                    }
                }
            }
        }
        let endOfIntegerPart = p
        var digitCount = Int64(endOfIntegerPart &- startDigits)
        if storeSpans {
            answer.integerStart = startDigits
            answer.integerLength = Int(digitCount)
        }
        if F.isJSON {
            if digitCount == 0 {
                answer.lastMatch = p
                return answer
            }
            if isCharacter(s[startDigits], 0x30) && digitCount > 1 {
                answer.lastMatch = startDigits
                return answer
            }
        }

        var exponent: Int64 = 0
        let hasDecimalPoint = p < end && isCharacter(s[p], 0x2E)
        if hasDecimalPoint {
            p &+= 1
            let before = p
            parseEightDigitRuns(s, &p, &i)
            while p < end && isDigitCharacter(s[p]) {
                i = i &* 10 &+ digitValue(s[p])
                p &+= 1
            }
            exponent = Int64(before &- p)
            if storeSpans {
                answer.fractionStart = before
                answer.fractionLength = p &- before
            }
            digitCount &-= exponent
        }
        if F.isJSON {
            if hasDecimalPoint && exponent == 0 {
                answer.lastMatch = p
                return answer
            }
        } else if _slowPath(digitCount == 0) {
            answer.lastMatch = p
            return answer
        }

        var expNumber: Int64 = 0
        if _slowPath(F.allowsScientific && p < end && (isCharacter(s[p], 0x65) || isCharacter(s[p], 0x45))) {
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
                if !formatAllowsFixed {
                    answer.lastMatch = p
                    return answer
                }
                p = locationOfE
            } else {
                // The first exponent digit is known to be a digit (and expNumber is 0).
                expNumber = Int64(digitValue(s[p]))
                p &+= 1
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
        } else if F.allowsScientific && !formatAllowsFixed {
            answer.lastMatch = p
            return answer
        }
        answer.lastMatch = p
        answer.valid = true

        if _slowPath(digitCount > 19) {
            var start = startDigits
            while start < end && (isCharacter(s[start], 0x30) || isCharacter(s[start], 0x2E)) {
                if isCharacter(s[start], 0x30) {
                    digitCount &-= 1
                }
                start &+= 1
            }
            if digitCount > 19 {
                answer.tooManyDigits = true
                if storeSpans {
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
        }
        answer.exponent = exponent
        answer.mantissa = i
        return answer
    }


    // fast_float's SWAR digit parsing (credit @aqrit): `val` holds eight ASCII bytes.
    @inline(__always)
    static func isMadeOfEightDigits(_ val: UInt64) -> Bool {
        (((val &+ 0x4646_4646_4646_4646) | (val &- 0x3030_3030_3030_3030)) & 0x8080_8080_8080_8080) == 0
    }

    @inline(__always)
    static func parseEightDigitsUnrolled(_ valIn: UInt64) -> UInt64 {
        let mask: UInt64 = 0x0000_00FF_0000_00FF
        let mul1: UInt64 = 0x000F_4240_0000_0064
        let mul2: UInt64 = 0x0000_2710_0000_0001
        var val = valIn &- 0x3030_3030_3030_3030
        val = (val &* 10) &+ (val &>> 8)
        val = (((val & mask) &* mul1) &+ (((val &>> 16) & mask) &* mul2)) &>> 32
        return UInt64(UInt32(truncatingIfNeeded: val))
    }

    @inline(__always)
    static func isMadeOfFourDigits(_ val: UInt32) -> Bool {
        (((val &+ 0x4646_4646) | (val &- 0x3030_3030)) & 0x8080_8080) == 0
    }

    @inline(__always)
    static func parseFourDigitsUnrolled(_ valIn: UInt32) -> UInt64 {
        var val = valIn &- 0x3030_3030
        val = (val &* 10) &+ (val &>> 8)
        return UInt64(((val & 0x00FF_00FF) &* 0x0064_0001) &>> 16 & 0xFFFF)
    }

    // fast_float's loop_parse_if_eight_digits. Folding eight digits at a time gives the same
    // (wrapping) result as folding them one at a time.
    @inline(__always)
    static func parseEightDigitRuns<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, _ p: inout Int, _ i: inout UInt64) {
        let end = s.count
        if MemoryLayout<C>.size == 1 {
            let bytes = s.bytes
            while end &- p >= 8 {
                let val = bytes.load(fromByteOffset: p, as: UInt64.self, .littleEndian)
                if !isMadeOfEightDigits(val) { break }
                i = i &* 100_000_000 &+ parseEightDigitsUnrolled(val)
                p &+= 8
            }
            if end &- p >= 4 {
                let val = bytes.load(fromByteOffset: p, as: UInt32.self, .littleEndian)
                if isMadeOfFourDigits(val) {
                    i = i &* 10000 &+ parseFourDigitsUnrolled(val)
                    p &+= 4
                }
            }
        } else if MemoryLayout<C>.size == 2 {
            let bytes = s.bytes
            while end &- p >= 8 {
                let lo = bytes.load(fromByteOffset: p &* 2, as: UInt64.self, .littleEndian)
                let hi = bytes.load(fromByteOffset: p &* 2 &+ 8, as: UInt64.self, .littleEndian)
                // Every 16-bit lane must be in ['0', '9'].
                let bad = ((lo &+ 0x7FC6_7FC6_7FC6_7FC6) | (lo &- 0x0030_0030_0030_0030) | (hi &+ 0x7FC6_7FC6_7FC6_7FC6) | (hi &- 0x0030_0030_0030_0030)) & 0x8000_8000_8000_8000
                if bad != 0 || ((lo | hi) & 0xFF00_FF00_FF00_FF00) != 0 { break }
                let packedLo = (lo & 0xFF) | ((lo &>> 8) & 0xFF00) | ((lo &>> 16) & 0xFF_0000) | ((lo &>> 24) & 0xFF00_0000)
                let packedHi = (hi & 0xFF) | ((hi &>> 8) & 0xFF00) | ((hi &>> 16) & 0xFF_0000) | ((hi &>> 24) & 0xFF00_0000)
                i = i &* 100_000_000 &+ parseEightDigitsUnrolled(packedLo | (packedHi &<< 32))
                p &+= 8
            }
        }
    }

    static let fastFloatPowersOfTenDouble: InlineArray<23, Double> = [
        1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11,
        1e12, 1e13, 1e14, 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22,
    ]

    // fast_float::clinger_fast_path_impl<double> (round-to-nearest mode).
    @inline(__always)
    static func clingerFastPath(_ mantissa: UInt64, _ exponent: Int64, _ negative: Bool) -> Double? {
        guard _fastPath(exponent >= -22 && exponent <= 22 && mantissa <= (1 << 53)) else {
            return nil
        }
        // Both operations, then a select: the sign of the exponent is as unpredictable as the input.
        let mantissaValue = Double(mantissa)
        // (The unused one reads entry 0.)
        let quotient = mantissaValue / fastFloatPowersOfTenDouble[exponent < 0 ? Int(0 &- exponent) : 0]
        let product = mantissaValue * fastFloatPowersOfTenDouble[exponent < 0 ? 0 : Int(exponent)]
        let value = exponent < 0 ? quotient : product
        return negative ? -value : value
    }

    @inline(__always)
    static func fastFloatPower(_ q: Int32) -> Int32 {
        (((152170 &+ 65536) &* q) >> 16) &+ 63
    }

    // fast_float::compute_product_approximation<55>.
    @inline(__always)
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
    @inline(__always)
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
    @inline(__always)
    static func computeError(_ q: Int64, _ wIn: UInt64) -> AdjustedMantissa {
        let lz = wIn.leadingZeroBitCount
        let w = wIn &<< UInt64(lz)
        let product = computeProductApproximation(q, w)
        return computeErrorScaled(q, product.high, lz)
    }

    @inline(__always)
    static func computeErrorScaled(_ q: Int64, _ w: UInt64, _ lz: Int) -> AdjustedMantissa {
        let hilz = Int(truncatingIfNeeded: w &>> 63) ^ 1
        var answer = AdjustedMantissa()
        answer.mantissa = w &<< UInt64(hilz)
        let bias: Int32 = 52 &+ 1023
        answer.power2 = fastFloatPower(Int32(truncatingIfNeeded: q)) &+ bias &- Int32(truncatingIfNeeded: hilz &+ lz) &- 62 &+ fastFloatInvalidAmBias
        return answer
    }

    @inline(__always)
    static func toDouble(_ negative: Bool, _ am: AdjustedMantissa) -> Double {
        var word = am.mantissa
        word |= UInt64(truncatingIfNeeded: Int64(am.power2)) &<< 52
        word |= (negative ? 1 : 0) &<< 63
        return Double(bitPattern: word)
    }

    // fast_float::from_chars_float_advanced<double> for the formats WTF uses.
    // Returns the value, the number of characters consumed and the status. On `.invalid` the
    // value is 0 and the length is 0 (fast_float leaves the output untouched and points at `first`).
    @inline(__always)
    static func parseDouble<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, format: F.Type) -> (value: Double, length: Int, status: ParseStatus) {
        if let result = parseDoubleWithoutSlowPath(s, format: format) {
            return result
        }
        return parseNumberSlowPath(s, format: format)
    }

    // from_chars_float_advanced up to its calls to parse_number_slow_path (more than 19 significant digits, or an
    // Eisel-Lemire result that needs the digits): nil when parseNumberSlowPath has to finish the job. It makes
    // no calls, so an entry point built on it can tail-call the slow path and stay a leaf function.
    @inline(__always)
    static func parseDoubleWithoutSlowPath<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, format: F.Type) -> (value: Double, length: Int, status: ParseStatus)? {
        if _slowPath(s.isEmpty) {
            return (0, 0, .invalid)
        }
        let pns = parseNumberString(s, format: format, storeSpans: false)
        if _slowPath(!pns.valid) {
            return (0, 0, .invalid)
        }
        if _slowPath(pns.tooManyDigits) {
            return nil
        }
        if let value = clingerFastPath(pns.mantissa, pns.exponent, pns.negative) {
            return (value, pns.lastMatch, .ok)
        }
        let am = computeFloat(pns.exponent, pns.mantissa)
        if am.power2 < 0 {
            return nil
        }
        let value = toDouble(pns.negative, am)
        let outOfRange = (pns.mantissa != 0 && am.mantissa == 0 && am.power2 == 0) || am.power2 == 0x7FF
        return (value, pns.lastMatch, outOfRange ? .outOfRange : .ok)
    }

    // fast_float::parse_number_slow_path: re-parse with spans, then from_chars_advanced(pns, value).
    @inline(never)
    static func parseNumberSlowPath<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, format: F.Type) -> (value: Double, length: Int, status: ParseStatus) {
        let pns = parseNumberString(s, format: format, storeSpans: true)
        if !pns.tooManyDigits, let value = clingerFastPath(pns.mantissa, pns.exponent, pns.negative) {
            return (value, pns.lastMatch, .ok)
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
        let value = toDouble(pns.negative, am)
        let outOfRange = (pns.mantissa != 0 && am.mantissa == 0 && am.power2 == 0) || am.power2 == 0x7FF
        return (value, pns.lastMatch, outOfRange ? .outOfRange : .ok)
    }
}
