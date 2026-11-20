// The WTF number-parsing functions built on the fast_float port (NumberParse.swift):
// FastFloat.cpp's parseDouble / parseFixedDouble / parseJSONDouble / parseJSONDoubleFromDigitsSlow and
// WTFString.cpp's toDoubleType (charactersToDouble, charactersToFixedDouble, charactersToFloat).
// Strictly memory-safe: no unsafe constructs in this file.

extension NumberParse {
    // Mode bits of NumberParseSwiftBoundary.h.
    static let modeFormatMask: UInt32 = 3
    static let modeFormatFixed: UInt32 = 1
    static let modeFormatJSON: UInt32 = 2
    static let modeSkipWhitespace: UInt32 = 4
    static let modeAllowTrailingJunk: UInt32 = 8
    static let modeFloat: UInt32 = 16

    struct EntryResult {
        var value: Double
        var length: Int
        // toDoubleType's *ok.
        var ok: Bool
    }

    // WTF::parseDouble: fast_float::from_chars(general | no_infnan | allow_leading_plus). Invalid input
    // parses nothing and yields 0, like the C++ (which initializes the value to 0 and ignores the error).
    // Split in two so that the hot entry point is a leaf function that tail-calls the slow path.
    @inline(__always)
    static func parseGeneralWithoutSlowPath(_ s: Span<UInt8>) -> (value: Double, length: Int)? {
        guard let result = parseDoubleWithoutSlowPath(s, format: GeneralFormat.self) else {
            return nil
        }
        return (result.value, result.length)
    }

    @inline(__always)
    static func parseGeneralSlowPath(_ s: Span<UInt8>) -> (value: Double, length: Int) {
        let result = parseNumberSlowPath(s, format: GeneralFormat.self)
        return (result.value, result.length)
    }

    // parseJSONDouble, split the same way. The length is -1 (SIZE_MAX) when parseJSONDouble returns std::nullopt.
    @inline(__always)
    static func parseJSONWithoutSlowPath(_ s: Span<UInt8>) -> (value: Double, length: Int)? {
        guard let result = parseDoubleWithoutSlowPath(s, format: JSONFormat.self) else {
            return nil
        }
        return jsonResult(result)
    }

    @inline(__always)
    static func parseJSONSlowPath(_ s: Span<UInt8>) -> (value: Double, length: Int) {
        jsonResult(parseNumberSlowPath(s, format: JSONFormat.self))
    }

    // parseJSONDouble returns std::nullopt for everything but success and result_out_of_range.
    @inline(__always)
    static func jsonResult(_ result: (value: Double, length: Int, status: ParseStatus)) -> (value: Double, length: Int) {
        if result.status == .invalid {
            return (0, -1)
        }
        return (result.value, result.length)
    }

    @inline(__always)
    static func parseWithFormat<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, _ format: F.Type) -> (value: Double, length: Int, status: ParseStatus) {
        parseDouble(s, format: format)
    }

    // isUnicodeCompatibleASCIIWhitespace: ' ', '\t', '\n', '\v', '\f', '\r'.
    @inline(__always)
    static func isUnicodeCompatibleASCIIWhitespace<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ c: C) -> Bool {
        // One test: bit c of {'\t', '\n', '\v', '\f', '\r', ' '}.
        let u = UInt64(truncatingIfNeeded: c)
        return u <= 0x20 && ((1 as UInt64) &<< u) & 0x1_0000_3E00 != 0
    }

    // parseFixedDouble (and parseJSONDouble and parseDouble for UTF-16), each with its own specialization of the parser.
    @inline(__always)
    static func parse<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, mode: UInt32) -> (value: Double, length: Int) {
        switch mode {
        case 0:
            let result = parseWithFormat(s, GeneralFormat.self)
            return (result.value, result.length)
        case modeFormatFixed:
            let result = parseWithFormat(s, FixedFormat.self)
            return (result.value, result.length)
        default:
            return jsonResult(parseWithFormat(s, JSONFormat.self))
        }
    }

    // toDoubleType for every mode (charactersToDouble, charactersToFixedDouble, charactersToFloat).
    @inline(__always)
    static func charactersTo<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes>(_ s: Span<C>, mode: UInt32) -> EntryResult {
        if (mode & modeFormatMask) == modeFormatFixed {
            return charactersTo(s, mode: mode, format: FixedFormat.self, slowPath: true)!
        }
        return charactersTo(s, mode: mode, format: GeneralFormat.self, slowPath: true)!
    }

    // charactersToDouble / charactersToFloat without fast_float's slow path: nil when the caller has to use
    // charactersTo(_:mode:) instead (see parseDoubleWithoutSlowPath).
    @inline(__always)
    static func charactersToGeneralWithoutSlowPath(_ s: Span<UInt8>, mode: UInt32) -> EntryResult? {
        charactersTo(s, mode: mode, format: GeneralFormat.self, slowPath: false)
    }

    // WTFString.cpp's toDoubleType<CharacterType, trailingJunkPolicy, whitespacePolicy, mode>, plus
    // doubleToFloatCheckingOverflow for charactersToFloat (modeFloat: the result is rounded to float).
    // Never nil when slowPath is true.
    @inline(__always)
    static func charactersTo<C: FixedWidthInteger & UnsignedInteger & ConvertibleToBytes, F: ParseFormat>(_ s: Span<C>, mode: UInt32, format: F.Type, slowPath: Bool) -> EntryResult? {
        var leadingSpacesLength = 0
        if (mode & modeSkipWhitespace) != 0 && _slowPath(!s.isEmpty && isUnicodeCompatibleASCIIWhitespace(s[0])) {
            leadingSpacesLength = 1
            while leadingSpacesLength < s.count && isUnicodeCompatibleASCIIWhitespace(s[leadingSpacesLength]) {
                leadingSpacesLength &+= 1
            }
        }
        let rest = s.extracting(droppingFirst: leadingSpacesLength)
        let result: (value: Double, length: Int, status: ParseStatus)
        if slowPath {
            result = parseDouble(rest, format: format)
        } else {
            guard let fastResult = parseDoubleWithoutSlowPath(rest, format: format) else {
                return nil
            }
            result = fastResult
        }
        if _slowPath(result.length == 0) {
            return EntryResult(value: 0, length: 0, ok: false)
        }
        let parsedLength = result.length &+ leadingSpacesLength
        var ok = (mode & modeAllowTrailingJunk) != 0 || parsedLength == s.count
        var value = result.value
        if (mode & modeFloat) != 0 {
            let rounded = Float(value)
            if ok && value.isFinite && !rounded.isFinite {
                ok = false
            }
            value = Double(rounded)
        }
        return EntryResult(value: value, length: parsedLength, ok: ok)
    }

    // WTF::parseJSONDoubleFromDigitsSlow: fast_float::compute_float + to_float for a mantissa of at most
    // 19 digits. compute_float only asks for the digits (power2 < 0) when the mantissa was truncated,
    // which the caller rules out; the C++ RELEASE_ASSERTs it.
    @inline(__always)
    static func doubleFromDigits(_ mantissa: UInt64, _ exponent: Int64) -> Double {
        let adjusted = computeFloat(exponent, mantissa)
        precondition(adjusted.power2 >= 0)
        return toDouble(false, adjusted)
    }

    // 10^0 ... 10^19.
    static let powersOfTenUInt64: InlineArray<20, UInt64> = [
        1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000,
        10000000000, 100000000000, 1000000000000, 10000000000000, 100000000000000,
        1000000000000000, 10000000000000000, 100000000000000000, 1000000000000000000,
        10000000000000000000,
    ]

    // High 64 bits of x * y.
    @inline(__always)
    static func mulHigh(_ x: UInt64, _ y: UInt64) -> UInt64 {
        x.multipliedFullWidth(by: y).high
    }
}
