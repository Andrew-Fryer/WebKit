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

// Swift implementation of JSC::ARM64LogicalImmediate::create32/create64 (assembler/AssemblerCommon.h):
// encode a 32- or 64-bit value as the immN:immr:imms field of an ARM64 logical (AND/ORR/EOR/TST)
// immediate, or report that it has no such encoding.
//
// This is the C++ algorithm step for step (same width cascade 64, 32, 16, 8, 4, 2, same two/three-range
// search, same packing), so each function here mirrors one C++ function of the same name. The one change
// is highestSetBit: the C++ does a six-step binary search; here it is 63 - leadingZeroBitCount (clz).
// Both agree on every non-zero value, and the C++ callers never pass zero (findBitRange's preconditions).
//
// All arithmetic is wrapping (&+ &- &<< &>>) so nothing here can trap. Every intermediate is a small
// non-negative number by construction (lsb <= hsb < width <= 64), as in the C++, which computes them in
// `unsigned`; the result is at most 0x1fff.

enum ARM64LogicalImmediateEncoder {
    // InvalidLogicalImmediate in the C++.
    @inline(__always)
    static var invalid: Int32 { -1 }

    // mask(hsb): bits hsb...0 set. hsb is in 0...63.
    @inline(__always)
    static func mask(_ hsb: Int) -> UInt64 {
        ~UInt64(0) &>> (63 &- hsb)
    }

    // Bit number of the highest set bit of a non-zero value.
    @inline(__always)
    static func highestSetBit(_ value: UInt64) -> Int {
        63 &- value.leadingZeroBitCount
    }

    // findBitRange<width>: value has no bits set at or above `width` and is neither 0 nor mask(width - 1).
    // If it is two or three contiguous ranges of bits, returns the middle range hsb...lsb, and whether that
    // range is clear (inverted) rather than set.
    @inline(__always)
    static func findBitRange(_ width: Int, _ value: UInt64) -> (hsb: Int, lsb: Int, inverted: Bool)? {
        var value = value

        // If the top bit is set, flip all the bits and remember that we did.
        let msb: UInt64 = 1 &<< (width &- 1)
        let inverted = (value & msb) != 0
        if inverted {
            value ^= mask(width &- 1)
        }

        // Flip everything at or below the highest set bit.
        let hsb = highestSetBit(value)
        value ^= mask(hsb)
        if value == 0 {
            // The range hsb...0 was all set.
            return (hsb, 0, inverted)
        }

        // One more flip. If that clears the value, lsb is the top of a third range, so the middle range
        // starts one above it.
        let lsb = highestSetBit(value)
        value ^= mask(lsb)
        if value == 0 {
            return (hsb, lsb &+ 1, inverted)
        }

        return nil
    }

    // encodeLogicalImmediate<width>: pack immN:immr:imms. width is a power of 2 in 2...64.
    @inline(__always)
    static func encode(_ width: Int, _ hsb: Int, _ lsb: Int, _ inverted: Bool) -> Int32 {
        var immN = 0
        var imms = 0
        let immr: Int

        // 64-bit elements set immN. Narrower elements put a run of leading ones (then a zero) in imms.
        if width == 64 {
            immN = 1
        } else {
            imms = 63 & ~(width &+ width &- 1)
        }

        if inverted {
            immr = (width &- 1) &- hsb
            imms |= (width &- ((hsb &- lsb) &+ 1)) &- 1
        } else {
            immr = (width &- lsb) & (width &- 1)
            imms |= hsb &- lsb
        }

        return Int32(truncatingIfNeeded: (immN &<< 12) | (immr &<< 6) | imms)
    }

    // findBitRange<width> followed by encodeLogicalImmediate<width>.
    @inline(__always)
    static func tryWidth(_ width: Int, _ value: UInt64) -> Int32? {
        guard let range = findBitRange(width, value) else {
            return nil
        }
        return encode(width, range.hsb, range.lsb, range.inverted)
    }

    @inline(__always)
    static func create32(_ value32: UInt32) -> Int32 {
        // 0 and all-ones cannot be encoded.
        if value32 == 0 || value32 == ~0 {
            return invalid
        }

        // First a 32-bit pattern, then repeating 16-, 8-, 4- and 2-bit patterns. Each narrower width is
        // only possible if the two halves of the current width are equal.
        var value = UInt64(value32)
        if let result = tryWidth(32, value) {
            return result
        }

        if (value & 0xffff) != (value &>> 16) {
            return invalid
        }
        value &= 0xffff
        if let result = tryWidth(16, value) {
            return result
        }

        if (value & 0xff) != (value &>> 8) {
            return invalid
        }
        value &= 0xff
        if let result = tryWidth(8, value) {
            return result
        }

        if (value & 0xf) != (value &>> 4) {
            return invalid
        }
        value &= 0xf
        if let result = tryWidth(4, value) {
            return result
        }

        if (value & 0x3) != (value &>> 2) {
            return invalid
        }
        value &= 0x3
        if let result = tryWidth(2, value) {
            return result
        }

        return invalid
    }

    @inline(__always)
    static func create64(_ value: UInt64) -> Int32 {
        // 0 and all-ones cannot be encoded.
        if value == 0 || value == ~0 {
            return invalid
        }

        // A contiguous 64-bit range.
        if let result = tryWidth(64, value) {
            return result
        }

        // If the two 32-bit halves are equal, try a 32-bit (or narrower) pattern.
        let low = UInt32(truncatingIfNeeded: value)
        if low == UInt32(truncatingIfNeeded: value &>> 32) {
            return create32(low)
        }
        return invalid
    }
}
