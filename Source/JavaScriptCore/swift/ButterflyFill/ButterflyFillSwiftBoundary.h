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

// JSObject's indexed-storage fill and shape-conversion loops implemented in Swift (ButterflyFill.swift). Each
// function rewrites data[0 ..< count], a butterfly's indexed payload or a sub-range of it, viewed as 64-bit
// elements; data must be 8-byte aligned and may be null only if count is 0. The symbols are hidden in
// JavaScriptCore: only runtime/JSObject.cpp calls them.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Every element = PNaN (PNaNAsBits), the hole of a Double-shape butterfly.
__attribute__((nothrow)) void JSCSwift_ButterflyFill_fillWithPureNaN(uint64_t* data, size_t count);

// Every element = JSValue::encode(JSValue()) (0), the hole of an Int32- or Contiguous-shape butterfly.
__attribute__((nothrow)) void JSCSwift_ButterflyFill_fillWithEmptyValue(uint64_t* data, size_t count);

// In place, as JSObject::convertInt32ToDouble: an int32 JSValue becomes the bits of the double of its value, any
// other bit pattern becomes PNaN.
__attribute__((nothrow)) void JSCSwift_ButterflyFill_int32ToDouble(uint64_t* data, size_t count);

// In place, as JSObject::convertDoubleToContiguous: a NaN becomes JSValue() (0), any other double d becomes
// JSValue::encode(JSValue(JSValue::EncodeAsDouble, d)).
__attribute__((nothrow)) void JSCSwift_ButterflyFill_doubleToJSValue(uint64_t* data, size_t count);

#ifdef __cplusplus
}
#endif
