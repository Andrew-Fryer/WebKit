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

// FEColorMatrixSoftwareApplier::applyPlatformUnaccelerated (platform/graphics/filters/software/
// FEColorMatrixSoftwareApplier.cpp) with its inline helpers matrix(), saturateAndHueRotate() and luminance(), and the
// PixelBuffer::item()/set() it calls per channel (set() is JSC::Uint8ClampedAdaptor::toNativeFromDouble). The pixels
// are unpremultiplied RGBA8, 4 bytes per pixel, rewritten in place.
//
// Floating point: WebKit compiles the C++ with clang's default -ffp-contract=on, so each `a*b + c*d + ...` expression is
// a chain of llvm.fmuladd: fused multiply-adds on arm64, separate multiplies and adds on x86_64 (no FMA in the
// baseline ISA). mulAdd() reproduces that per architecture, in the C++ association order: for `x0*y0 + x1*y1 + ...`,
// fmuladd(x0, y0, x1*y1) first, then each further product fused into the running sum.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum FEColorMatrix {
    // ColorMatrixType raw values (FEColorMatrix.h).
    static let typeUnknown: UInt8 = 0
    static let typeMatrix: UInt8 = 1
    static let typeSaturate: UInt8 = 2
    static let typeHueRotate: UInt8 = 3
    static let typeLuminanceToAlpha: UInt8 = 4

    // llvm.fmuladd(a, b, sum) as clang lowers it for the lab slices.
    @inline(__always)
    static func mulAdd(_ sum: Float, _ a: Float, _ b: Float) -> Float {
#if arch(arm64)
        return sum.addingProduct(a, b)
#else
        return sum + a * b
#endif
    }

    @inline(__always)
    static func mulAdd(_ sum: Double, _ a: Double, _ b: Double) -> Double {
#if arch(arm64)
        return sum.addingProduct(a, b)
#else
        return sum + a * b
#endif
    }

    // PixelBuffer::set(index, double(value)): Uint8ClampedAdaptor::toNativeFromDouble. NaN and negative values give 0,
    // values above 255 give 255, others lrint (round half to even). value is a float, so its widening to double is
    // exact and doing this in Float gives the same results. Written as selects (`value > 0` is false for NaN, -0 and
    // negative values) so that it compiles without branches: on image data the C++ branches mispredict.
    @inline(__always)
    static func clampedByte(_ value: Float) -> UInt8 {
        let positive: Float = value > 0 ? value : 0
        let clamped: Float = positive < 255 ? positive : 255
        return UInt8(clamped.rounded(.toNearestOrEven))
    }

    // values[0 ..< 20] (5x4 matrix, row-major) and components[0 ..< 9] are read only for the types that use them,
    // as the C++ does (m_components is not initialized for the other types).
    static func applyUnaccelerated(_ pixels: inout MutableSpan<UInt8>, type: UInt8, values: Span<Float>, components: Span<Float>) {
        let byteCount = pixels.count
        if byteCount == 0 || type == typeUnknown || type > typeLuminanceToAlpha {
            return
        }
        // The C++ loop reads item(offset + 1 ... offset + 3) of the last pixel, a RELEASE_ASSERT failure if the
        // length is not a multiple of 4.
        precondition(byteCount & 3 == 0)

        switch type {
        case typeMatrix:
            // values[] is a bounds-checked Vector<float> in the C++: fewer than 20 values crash on the first pixel.
            let v0 = values[0], v1 = values[1], v2 = values[2], v3 = values[3], v4 = values[4]
            let v5 = values[5], v6 = values[6], v7 = values[7], v8 = values[8], v9 = values[9]
            let v10 = values[10], v11 = values[11], v12 = values[12], v13 = values[13], v14 = values[14]
            let v15 = values[15], v16 = values[16], v17 = values[17], v18 = values[18], v19 = values[19]
            var offset = 0
            while offset < byteCount {
                let r = Float(pixels[offset])
                let g = Float(pixels[offset &+ 1])
                let b = Float(pixels[offset &+ 2])
                let a = Float(pixels[offset &+ 3])
                let red = mulAdd(mulAdd(mulAdd(mulAdd(v1 * g, v0, r), v2, b), v3, a), v4, 255)
                let green = mulAdd(mulAdd(mulAdd(mulAdd(v6 * g, v5, r), v7, b), v8, a), v9, 255)
                let blue = mulAdd(mulAdd(mulAdd(mulAdd(v11 * g, v10, r), v12, b), v13, a), v14, 255)
                let alpha = mulAdd(mulAdd(mulAdd(mulAdd(v16 * g, v15, r), v17, b), v18, a), v19, 255)
                pixels[offset] = clampedByte(red)
                pixels[offset &+ 1] = clampedByte(green)
                pixels[offset &+ 2] = clampedByte(blue)
                pixels[offset &+ 3] = clampedByte(alpha)
                offset &+= 4
            }

        case typeSaturate, typeHueRotate:
            let c0 = components[0], c1 = components[1], c2 = components[2]
            let c3 = components[3], c4 = components[4], c5 = components[5]
            let c6 = components[6], c7 = components[7], c8 = components[8]
            var offset = 0
            while offset < byteCount {
                let r = Float(pixels[offset])
                let g = Float(pixels[offset &+ 1])
                let b = Float(pixels[offset &+ 2])
                // r * c0 + g * c1 + b * c2
                let red = mulAdd(mulAdd(g * c1, r, c0), b, c2)
                let green = mulAdd(mulAdd(g * c4, r, c3), b, c5)
                let blue = mulAdd(mulAdd(g * c7, r, c6), b, c8)
                pixels[offset] = clampedByte(red)
                pixels[offset &+ 1] = clampedByte(green)
                pixels[offset &+ 2] = clampedByte(blue)
                // alpha = item(offset + 3), stored back unchanged (set() of an integral value in 0...255).
                offset &+= 4
            }

        default: // typeLuminanceToAlpha
            var offset = 0
            while offset < byteCount {
                let r = Double(pixels[offset])
                let g = Double(pixels[offset &+ 1])
                let b = Double(pixels[offset &+ 2])
                // alpha = 0.2125 * red + 0.7154 * green + 0.0721 * blue, in double, then narrowed to float.
                let alpha = Float(mulAdd(mulAdd(0.7154 * g, 0.2125, r), 0.0721, b))
                pixels[offset] = 0
                pixels[offset &+ 1] = 0
                pixels[offset &+ 2] = 0
                pixels[offset &+ 3] = clampedByte(alpha)
                offset &+= 4
            }
        }
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
