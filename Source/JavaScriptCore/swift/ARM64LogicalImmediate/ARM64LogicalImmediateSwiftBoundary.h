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

// Swift implementation of JSC::ARM64LogicalImmediate::create32/create64 (ARM64LogicalImmediate.swift).
// The symbols are hidden inside JavaScriptCore: include this only from JavaScriptCore .cpp files
// (assembler/AssemblerCommon.cpp), never from a header that WebCore or the test tools compile.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Both return the immN:immr:imms encoding of value (0...0x1fff) as an ARM64 logical immediate, or -1
// (ARM64LogicalImmediate::InvalidLogicalImmediate) if it has none. Pure functions of their argument.
__attribute__((const, nothrow, warn_unused_result)) int32_t JSCSwift_ARM64LogicalImmediate_create32(uint32_t value);
__attribute__((const, nothrow, warn_unused_result)) int32_t JSCSwift_ARM64LogicalImmediate_create64(uint64_t value);

#ifdef __cplusplus
} // extern "C"
#endif
