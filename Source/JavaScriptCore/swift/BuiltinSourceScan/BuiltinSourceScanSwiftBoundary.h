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

// Swift implementation of the hand-rolled scan of a builtin's source (BuiltinSourceScan.swift) that
// BuiltinExecutables::createExecutable does for builtins without generated metadata. The symbol is hidden inside
// JavaScriptCore: include this only from builtins/BuiltinExecutables.cpp, never from a header that WebCore or the
// test tools compile.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// The result words, in the order and with the meaning of the fields of JSC::BuiltinSourceMetadata
// (builtins/BuiltinExecutables.h). The two flags are 0 or 1; closeBraceOffsetFromEnd is a positive int.
enum {
    JSCSwift_BuiltinSourceScan_SourceLength = 0,
    JSCSwift_BuiltinSourceScan_ParametersStart = 1,
    JSCSwift_BuiltinSourceScan_ParameterCount = 2,
    JSCSwift_BuiltinSourceScan_CloseBraceOffsetFromEnd = 3,
    JSCSwift_BuiltinSourceScan_IsAsyncFunction = 4,
    JSCSwift_BuiltinSourceScan_IsInStrictContext = 5,
    JSCSwift_BuiltinSourceScan_WordCount = 6,
};

// Scans characters[0 ..< length], the Latin-1 source of a single anonymous function expression
// "(function (...) {...})" or "(async function (...) {...})", and writes its BuiltinSourceMetadata to
// words[0 ..< JSCSwift_BuiltinSourceScan_WordCount]. Traps where the original C++ RELEASE_ASSERTed (the source is
// shorter than "(function (){})" or does not start with one of the two prefixes) and where it read out of bounds
// (no ')' after the parameters' '(', no '}'). characters may be null only if length is 0.
__attribute__((nothrow)) void JSCSwift_BuiltinSourceScan_computeMetadata(const uint8_t* characters, size_t length, uint32_t* words);

#ifdef __cplusplus
} // extern "C"
#endif
