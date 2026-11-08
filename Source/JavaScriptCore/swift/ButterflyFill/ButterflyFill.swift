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

// The element loops of JSObject's indexed-storage allocation and shape conversion (runtime/JSObject.cpp:
// ensureLengthSlow, createInitial{Int32,Double,Contiguous}, convertUndecidedTo{Int32,Double,Contiguous},
// convertInt32ToDouble, convertDoubleToContiguous). An element is one 64-bit slot of a butterfly's indexed
// payload, holding either an EncodedJSValue (Int32 and Contiguous shapes) or the bits of a double (Double shape).
//
// As before, the C++ caller publishes what these loops write (a new vector length or structure, after a
// storeStoreFence) only after they return, and until then no other thread interprets these slots: so the width
// and order of the stores is no more observable than in the C++, which clang also compiles to bzero,
// memset_pattern16 or vector loops. The conversions store whole elements (64-bit or vector stores).

enum ButterflyFill {
    // PNaNAsBits (runtime/PureNaN.h): the hole value of a Double-shape butterfly.
    @inline(__always) static var pureNaN: UInt64 { 0x7ff8_0000_0000_0000 }
    // JSValue::NumberTag (runtime/JSCJSValue.h): a value is an int32 iff all of these bits are set.
    @inline(__always) static var numberTag: UInt64 { 0xfffe_0000_0000_0000 }
    // JSValue::DoubleEncodeOffset (1 << JSValueDoubleEncodeOffsetBit, 49).
    @inline(__always) static var doubleEncodeOffset: UInt64 { 1 &<< 49 }

    // Every element = PNaN (the C++ `data[i] = PNaN` loops, which clang turns into memset_pattern16 calls).
    // From 64 elements on, a plain loop, which LLVM vectorizes. Below that, its scalar remainder would make
    // lengths like 12 or 23 about 1.3x slower than memset_pattern16, so like memset_pattern16 this stores 8
    // elements at a time and finishes with 8 more that overlap the last chunk (below 8 elements, two overlapping
    // chunks of 4 or 2). A slot written twice gets the same value both times, and nobody reads these slots until
    // the caller publishes them.
    @inline(__always)
    static func fillWithPureNaN(_ elements: inout MutableSpan<UInt64>) {
        let count = elements.count
        if count >= 64 {
            var i = 0
            while i < elements.count {
                elements[i] = pureNaN
                i &+= 1
            }
        } else if count >= 8 {
            var i = 0
            repeat {
                store8(&elements, at: i, pureNaN)
                i += 8
            } while i <= count - 8
            if i < count {
                store8(&elements, at: count - 8, pureNaN)
            }
        } else if count >= 4 {
            store4(&elements, at: 0, pureNaN)
            store4(&elements, at: count - 4, pureNaN)
        } else if count >= 2 {
            store2(&elements, at: 0, pureNaN)
            store2(&elements, at: count - 2, pureNaN)
        } else if count == 1 {
            elements[0] = pureNaN
        }
    }

    // elements[start ..< start + 8] = value. Checking the first and the last index first lets the compiler drop the
    // other checks and merge the stores into vector stores.
    @inline(__always)
    static func store8(_ elements: inout MutableSpan<UInt64>, at start: Int, _ value: UInt64) {
        elements[start] = value
        elements[start + 7] = value
        elements[start + 1] = value
        elements[start + 2] = value
        elements[start + 3] = value
        elements[start + 4] = value
        elements[start + 5] = value
        elements[start + 6] = value
    }

    @inline(__always)
    static func store4(_ elements: inout MutableSpan<UInt64>, at start: Int, _ value: UInt64) {
        elements[start] = value
        elements[start + 3] = value
        elements[start + 1] = value
        elements[start + 2] = value
    }

    @inline(__always)
    static func store2(_ elements: inout MutableSpan<UInt64>, at start: Int, _ value: UInt64) {
        elements[start] = value
        elements[start + 1] = value
    }

    // Every element = JSValue::encode(JSValue()), which is 0 (`clear()` / `setWithoutWriteBarrier(JSValue())`).
    // LLVM turns this loop into a call to bzero, as clang does for the C++ loops (a vectorized loop, which LLVM
    // emits for the `while` form, is slower than bzero from about 100 elements on).
    @inline(__always)
    static func fillWithEmptyValue(_ elements: inout MutableSpan<UInt64>) {
        for i in elements.indices {
            elements[i] = 0
        }
    }

    // The two conversions are `while` loops over the span's indices, which LLVM vectorizes with the bounds checks
    // hoisted.

    // convertInt32ToDouble: an int32 JSValue becomes the double of its value; anything else (the slot may hold
    // garbage during initialization) becomes PNaN. The select is written as a mask so that the loop stays
    // branch-free and vectorizes: a branchy loop is about twice as slow.
    @inline(__always)
    static func int32ToDouble(_ elements: inout MutableSpan<UInt64>) {
        var i = 0
        while i < elements.count {
            let value = elements[i]
            let converted = Double(Int32(truncatingIfNeeded: value)).bitPattern
            let isInt32Mask = 0 &- UInt64((value & numberTag) == numberTag ? 1 : 0)
            elements[i] = (converted & isInt32Mask) | (pureNaN & ~isInt32Mask)
            i &+= 1
        }
    }

    // convertDoubleToContiguous: a NaN (any payload) becomes JSValue() (0); any other double becomes
    // JSValue(EncodeAsDouble, d), its bits plus DoubleEncodeOffset (so -0.0 stays distinct from 0.0).
    @inline(__always)
    static func doubleToJSValue(_ elements: inout MutableSpan<UInt64>) {
        var i = 0
        while i < elements.count {
            let bits = elements[i]
            let d = Double(bitPattern: bits)
            let notNaNMask = 0 &- UInt64(d == d ? 1 : 0)
            elements[i] = (bits &+ doubleEncodeOffset) & notNaNMask
            i &+= 1
        }
    }
}
