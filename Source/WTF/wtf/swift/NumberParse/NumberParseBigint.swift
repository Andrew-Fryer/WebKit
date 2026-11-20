// Derived from fast_float 8.2.10 by Daniel Lemire, João Paulo Magalhaes and contributors
// (https://github.com/fastfloat/fast_float), formerly vendored as Source/WTF/wtf/fast_float/fast_float.h,
// licensed under the Apache License 2.0 (LICENSE-fast_float in this directory), the MIT license or the
// Boost Software License 1.0, at your option.
//
// Port of fast_float's big-integer slow path (bigint.h + digit_comparison.h): used only when the
// Eisel-Lemire algorithm cannot decide the rounding (very rare inputs, e.g. near-halfway values with
// many digits). Strictly memory-safe: no unsafe constructs in this file.

extension NumberParse {
    // fast_float::stackvec<bigint_limbs> with 64-bit limbs; bigint_limbs = 4000 / 64 = 62.
    struct FastFloatBigint {
        static let capacity = 62
        var limbs = InlineArray<62, UInt64>(repeating: 0)
        var length = 0

        init() {}

        init(_ value: UInt64) {
            limbs[0] = value
            length = 1
            normalize()
        }

        @inline(__always)
        mutating func rindex(_ index: Int) -> UInt64 {
            limbs[length &- index &- 1]
        }

        @inline(__always)
        mutating func tryPush(_ value: UInt64) -> Bool {
            if length < Self.capacity {
                limbs[length] = value
                length &+= 1
                return true
            }
            return false
        }

        mutating func tryResize(_ newLength: Int, _ value: UInt64) -> Bool {
            if newLength > Self.capacity {
                return false
            }
            if newLength > length {
                for i in length..<newLength {
                    limbs[i] = value
                }
            }
            length = newLength
            return true
        }

        mutating func nonzero(_ start: Int) -> Bool {
            var index = start
            while index < length {
                if rindex(index) != 0 {
                    return true
                }
                index &+= 1
            }
            return false
        }

        mutating func normalize() {
            while length > 0 && rindex(0) == 0 {
                length &-= 1
            }
        }

        // small_add_from
        mutating func smallAdd(_ y: UInt64, from start: Int) -> Bool {
            var index = start
            var carry = y
            while carry != 0 && index < length {
                let (sum, overflow) = limbs[index].addingReportingOverflow(carry)
                limbs[index] = sum
                carry = overflow ? 1 : 0
                index &+= 1
            }
            if carry != 0 {
                return tryPush(carry)
            }
            return true
        }

        // small_mul
        mutating func smallMul(_ y: UInt64) -> Bool {
            var carry: UInt64 = 0
            for index in 0..<length {
                let product = limbs[index].multipliedFullWidth(by: y)
                let (low, overflow) = product.low.addingReportingOverflow(carry)
                limbs[index] = low
                carry = product.high &+ (overflow ? 1 : 0)
            }
            if carry != 0 {
                return tryPush(carry)
            }
            return true
        }

        // large_add_from
        mutating func largeAdd(_ y: inout FastFloatBigint, from start: Int) -> Bool {
            if length < start || y.length > length &- start {
                if !tryResize(y.length &+ start, 0) {
                    return false
                }
            }
            var carry = false
            for index in 0..<y.length {
                var xi = limbs[index &+ start]
                let yi = y.limbs[index]
                let (s1, c1) = xi.addingReportingOverflow(yi)
                xi = s1
                var c2 = false
                if carry {
                    let (s2, o2) = xi.addingReportingOverflow(1)
                    xi = s2
                    c2 = o2
                }
                limbs[index &+ start] = xi
                carry = c1 || c2
            }
            if carry {
                return smallAdd(1, from: y.length &+ start)
            }
            return true
        }

        // long_mul / large_mul
        mutating func largeMul(_ y: inout FastFloatBigint) -> Bool {
            if y.length == 1 {
                return smallMul(y.limbs[0])
            }
            let z = self
            if y.length != 0 {
                if !smallMul(y.limbs[0]) {
                    return false
                }
                for index in 1..<y.length {
                    let yi = y.limbs[index]
                    if yi != 0 {
                        var zi = FastFloatBigint()
                        zi.limbs = z.limbs
                        zi.length = z.length
                        if !zi.smallMul(yi) {
                            return false
                        }
                        if !largeAdd(&zi, from: index) {
                            return false
                        }
                    }
                }
            }
            normalize()
            return true
        }

        mutating func hi64() -> (value: UInt64, truncated: Bool) {
            if length == 0 {
                return (0, false)
            }
            if length == 1 {
                let r0 = rindex(0)
                return (r0 &<< UInt64(r0.leadingZeroBitCount), false)
            }
            let r0 = rindex(0)
            let r1 = rindex(1)
            let shl = r0.leadingZeroBitCount
            var value: UInt64
            var truncated: Bool
            if shl == 0 {
                truncated = r1 != 0
                value = r0
            } else {
                truncated = (r1 &<< UInt64(shl)) != 0
                value = (r0 &<< UInt64(shl)) | (r1 &>> UInt64(64 &- shl))
            }
            truncated = truncated || nonzero(2)
            return (value, truncated)
        }

        mutating func compare(_ other: inout FastFloatBigint) -> Int {
            if length > other.length {
                return 1
            }
            if length < other.length {
                return -1
            }
            var index = length
            while index > 0 {
                let xi = limbs[index &- 1]
                let yi = other.limbs[index &- 1]
                if xi > yi {
                    return 1
                }
                if xi < yi {
                    return -1
                }
                index &-= 1
            }
            return 0
        }

        mutating func shlBits(_ n: Int) -> Bool {
            let shl = UInt64(n)
            let shr = UInt64(64 &- n)
            var prev: UInt64 = 0
            for index in 0..<length {
                let xi = limbs[index]
                limbs[index] = (xi &<< shl) | (prev &>> shr)
                prev = xi
            }
            let carry = prev &>> shr
            if carry != 0 {
                return tryPush(carry)
            }
            return true
        }

        mutating func shlLimbs(_ n: Int) -> Bool {
            if n &+ length > Self.capacity {
                return false
            }
            if length != 0 {
                var index = length
                while index > 0 {
                    index &-= 1
                    limbs[index &+ n] = limbs[index]
                }
                for i in 0..<n {
                    limbs[i] = 0
                }
                length = n &+ length
            }
            return true
        }

        mutating func shl(_ n: Int) -> Bool {
            let rem = n % 64
            let div = n / 64
            if rem != 0 && !shlBits(rem) {
                return false
            }
            if div != 0 && !shlLimbs(div) {
                return false
            }
            return true
        }

        mutating func ctlz() -> Int {
            length == 0 ? 0 : rindex(0).leadingZeroBitCount
        }

        mutating func bitLength() -> Int {
            64 &* length &- ctlz()
        }

        mutating func pow2(_ exp: UInt32) -> Bool {
            shl(Int(exp))
        }

        mutating func pow5(_ expIn: UInt32) -> Bool {
            var exp = expIn
            if exp >= 135 {
                var large = FastFloatBigint()
                large.limbs[0] = 1414648277510068013
                large.limbs[1] = 9180637584431281687
                large.limbs[2] = 4539964771860779200
                large.limbs[3] = 10482974169319127550
                large.limbs[4] = 198276706040285095
                large.length = 5
                while exp >= 135 {
                    if !largeMul(&large) {
                        return false
                    }
                    exp &-= 135
                }
            }
            while exp >= 27 {
                if !smallMul(7450580596923828125) {
                    return false
                }
                exp &-= 27
            }
            if exp != 0 {
                if !smallMul(NumberParse.fastFloatSmallPowerOfFive(Int(exp))) {
                    return false
                }
            }
            return true
        }

        mutating func pow10(_ exp: UInt32) -> Bool {
            if !pow5(exp) {
                return false
            }
            return pow2(exp)
        }
    }

    // 5^n for n < 28.
    @inline(__always)
    static func fastFloatSmallPowerOfFive(_ n: Int) -> UInt64 {
        var result: UInt64 = 1
        for _ in 0..<n {
            result &*= 5
        }
        return result
    }

    static func scientificExponent(_ mantissaIn: UInt64, _ exponentIn: Int32) -> Int32 {
        var mantissa = mantissaIn
        var exponent = exponentIn
        while mantissa >= 10000 {
            mantissa /= 10000
            exponent &+= 4
        }
        while mantissa >= 100 {
            mantissa /= 100
            exponent &+= 2
        }
        while mantissa >= 10 {
            mantissa /= 10
            exponent &+= 1
        }
        return exponent
    }

    @inline(__always)
    static func toExtended(_ value: Double) -> AdjustedMantissa {
        let bits = value.bitPattern
        let bias: Int32 = 52 &+ 1023
        var am = AdjustedMantissa()
        if (bits & 0x7FF0_0000_0000_0000) == 0 {
            am.power2 = 1 &- bias
            am.mantissa = bits & 0x000F_FFFF_FFFF_FFFF
        } else {
            am.power2 = Int32(truncatingIfNeeded: (bits & 0x7FF0_0000_0000_0000) &>> 52) &- bias
            am.mantissa = (bits & 0x000F_FFFF_FFFF_FFFF) | 0x0010_0000_0000_0000
        }
        return am
    }

    enum FastFloatRounding {
        case down
        case nearestTruncated(Bool)
        case compared(Int)
    }

    @inline(__always)
    static func applyRounding(_ am: inout AdjustedMantissa, _ shift: Int32, _ mode: FastFloatRounding) {
        switch mode {
        case .down:
            if shift == 64 {
                am.mantissa = 0
            } else {
                am.mantissa &>>= UInt64(truncatingIfNeeded: shift)
            }
            am.power2 &+= shift
        case .nearestTruncated, .compared:
            let mask: UInt64 = shift == 64 ? UInt64.max : (1 &<< UInt64(truncatingIfNeeded: shift)) &- 1
            let halfway: UInt64 = shift == 0 ? 0 : 1 &<< UInt64(truncatingIfNeeded: shift &- 1)
            let truncatedBits = am.mantissa & mask
            let isAbove = truncatedBits > halfway
            let isHalfway = truncatedBits == halfway
            if shift == 64 {
                am.mantissa = 0
            } else {
                am.mantissa &>>= UInt64(truncatingIfNeeded: shift)
            }
            am.power2 &+= shift
            let isOdd = (am.mantissa & 1) == 1
            let roundUp: Bool
            switch mode {
            case .nearestTruncated(let truncated):
                roundUp = isAbove || (isHalfway && truncated) || (isOdd && isHalfway)
            case .compared(let ord):
                roundUp = ord > 0 ? true : (ord < 0 ? false : isOdd)
            case .down:
                roundUp = false
            }
            am.mantissa &+= roundUp ? 1 : 0
        }
    }

    // fast_float::round<double>.
    static func fastFloatRound(_ am: inout AdjustedMantissa, _ mode: FastFloatRounding) {
        let mantissaShift: Int32 = 64 &- 52 &- 1
        if 0 &- am.power2 >= mantissaShift {
            let shift = 0 &- am.power2 &+ 1
            applyRounding(&am, shift < 64 ? shift : 64, mode)
            am.power2 = am.mantissa < (1 << 52) ? 0 : 1
            return
        }
        applyRounding(&am, mantissaShift, mode)
        if am.mantissa >= (2 << 52) {
            am.mantissa = 1 << 52
            am.power2 &+= 1
        }
        am.mantissa &= ~(1 << 52)
        if am.power2 >= 0x7FF {
            am.power2 = 0x7FF
            am.mantissa = 0
        }
    }

    @inline(__always)
    static func skipZeros<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, _ p: inout Int, _ end: Int) {
        while p != end && isCharacter(s[p], 0x30) {
            p &+= 1
        }
    }

    @inline(__always)
    static func isTruncated<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, _ start: Int, _ end: Int) -> Bool {
        var p = start
        while p != end {
            if !isCharacter(s[p], 0x30) {
                return true
            }
            p &+= 1
        }
        return false
    }

    // fast_float::parse_mantissa (step = 19 for 64-bit limbs).
    static func parseMantissa<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, _ num: ParsedNumberString, _ result: inout FastFloatBigint, maxDigits: Int) -> Int {
        var counter = 0
        var digits = 0
        var value: UInt64 = 0
        let step = 19

        var p = num.integerStart
        var end = p &+ num.integerLength
        skipZeros(s, &p, end)
        while p != end {
            while counter < step && p != end && digits < maxDigits {
                value = value &* 10 &+ digitValue(s[p])
                p &+= 1
                counter &+= 1
                digits &+= 1
            }
            _ = result.smallMul(powersOfTenUInt64[counter])
            _ = result.smallAdd(value, from: 0)
            if digits == maxDigits {
                var truncated = isTruncated(s, p, end)
                if num.fractionStart >= 0 {
                    truncated = truncated || isTruncated(s, num.fractionStart, num.fractionStart &+ num.fractionLength)
                }
                if truncated {
                    _ = result.smallMul(10)
                    _ = result.smallAdd(1, from: 0)
                    digits &+= 1
                }
                return digits
            }
            counter = 0
            value = 0
        }

        if num.fractionStart >= 0 {
            p = num.fractionStart
            end = p &+ num.fractionLength
            if digits == 0 {
                skipZeros(s, &p, end)
            }
            while p != end {
                while counter < step && p != end && digits < maxDigits {
                    value = value &* 10 &+ digitValue(s[p])
                    p &+= 1
                    counter &+= 1
                    digits &+= 1
                }
                _ = result.smallMul(powersOfTenUInt64[counter])
                _ = result.smallAdd(value, from: 0)
                if digits == maxDigits {
                    if isTruncated(s, p, end) {
                        _ = result.smallMul(10)
                        _ = result.smallAdd(1, from: 0)
                        digits &+= 1
                    }
                    return digits
                }
                counter = 0
                value = 0
            }
        }

        if counter != 0 {
            _ = result.smallMul(powersOfTenUInt64[counter])
            _ = result.smallAdd(value, from: 0)
        }
        return digits
    }

    static func positiveDigitComp(_ bigmant: inout FastFloatBigint, _ exponent: Int32) -> AdjustedMantissa {
        _ = bigmant.pow10(UInt32(truncatingIfNeeded: exponent))
        let hi = bigmant.hi64()
        var answer = AdjustedMantissa()
        answer.mantissa = hi.value
        let bias: Int32 = 52 &+ 1023
        answer.power2 = Int32(truncatingIfNeeded: bigmant.bitLength() &- 64) &+ bias
        fastFloatRound(&answer, .nearestTruncated(hi.truncated))
        return answer
    }

    static func negativeDigitComp(_ bigmant: inout FastFloatBigint, _ am: AdjustedMantissa, _ exponent: Int32) -> AdjustedMantissa {
        let realExp = exponent
        var amB = am
        fastFloatRound(&amB, .down)
        let b = toDouble(false, amB)
        var theor = toExtended(b)
        theor.mantissa = theor.mantissa &<< 1
        theor.mantissa &+= 1
        theor.power2 &-= 1
        var theorDigits = FastFloatBigint(theor.mantissa)
        let theorExp = theor.power2

        let pow2Exp = theorExp &- realExp
        let pow5Exp = UInt32(truncatingIfNeeded: 0 &- realExp)
        if pow5Exp != 0 {
            _ = theorDigits.pow5(pow5Exp)
        }
        if pow2Exp > 0 {
            _ = theorDigits.pow2(UInt32(truncatingIfNeeded: pow2Exp))
        } else if pow2Exp < 0 {
            _ = bigmant.pow2(UInt32(truncatingIfNeeded: 0 &- pow2Exp))
        }

        let ord = bigmant.compare(&theorDigits)
        var answer = am
        fastFloatRound(&answer, .compared(ord))
        return answer
    }

    // fast_float::digit_comp<double>.
    @inline(never)
    static func digitComp<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, _ num: ParsedNumberString, _ amIn: AdjustedMantissa) -> AdjustedMantissa {
        var am = amIn
        am.power2 &-= fastFloatInvalidAmBias
        let sciExp = scientificExponent(num.mantissa, Int32(truncatingIfNeeded: num.exponent))
        var bigmant = FastFloatBigint()
        let digits = parseMantissa(s, num, &bigmant, maxDigits: 769)
        let exponent = sciExp &+ 1 &- Int32(truncatingIfNeeded: digits)
        if exponent >= 0 {
            return positiveDigitComp(&bigmant, exponent)
        }
        return negativeDigitComp(&bigmant, am, exponent)
    }
}
