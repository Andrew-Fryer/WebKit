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

// C entry point of FEColorMatrix.swift, declared to C++ in FEColorMatrixSwiftBoundary.h and called only from
// FEColorMatrixSoftwareApplier.cpp. The only unsafe operations are wrapping the C++-owned pixel bytes, matrix values
// and components in spans. A pointer may be null only when its count is 0.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

@_cdecl("WebCoreSwift_FEColorMatrix_applyUnaccelerated")
func WebCoreSwift_FEColorMatrix_applyUnaccelerated(_ pixels: UnsafeMutablePointer<UInt8>?, _ length: Int, _ type: UInt8, _ values: UnsafePointer<Float>?, _ valueCount: Int, _ components: UnsafePointer<Float>?, _ componentCount: Int) {
    var pixelSpan = unsafe UnsafeMutableBufferPointer(start: pixels, count: length).mutableSpan
    let valueSpan = unsafe UnsafeBufferPointer(start: values, count: valueCount).span
    let componentSpan = unsafe UnsafeBufferPointer(start: components, count: componentCount).span
    FEColorMatrix.applyUnaccelerated(&pixelSpan, type: type, values: valueSpan, components: componentSpan)
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
