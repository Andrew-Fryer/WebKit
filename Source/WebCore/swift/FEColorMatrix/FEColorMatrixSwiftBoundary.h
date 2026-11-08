/*
 * Copyright (C) 2026 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

// C boundary of FEColorMatrixSoftwareApplier::applyPlatformUnaccelerated implemented in Swift
// (Source/WebCore/swift/FEColorMatrix). The symbol is internal to WebCore (hidden); only
// FEColorMatrixSoftwareApplier.cpp calls it, never header-inline code.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Rewrites pixels[0 ..< length] (unpremultiplied RGBA8, length a multiple of 4) in place, exactly as
// applyPlatformUnaccelerated does for `type` (ColorMatrixType raw value: 0 unknown, 1 matrix, 2 saturate,
// 3 hueRotate, 4 luminanceToAlpha). `values` (FEColorMatrix::values(), at least 20 for the matrix type) and
// `components` (the applier's 9 saturate/hue-rotate components) are only read for the types that use them.
void WebCoreSwift_FEColorMatrix_applyUnaccelerated(uint8_t* pixels, size_t length, uint8_t type, const float* values, size_t valueCount, const float* components, size_t componentCount);

#ifdef __cplusplus
}
#endif
