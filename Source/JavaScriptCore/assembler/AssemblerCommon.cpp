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

#include "config.h"
#include "AssemblerCommon.h"

#if USE(SWIFT_KERNELS)

#include "ARM64LogicalImmediateSwiftBoundary.h"

#if ENABLE(SWIFT_KERNELS_VALIDATION)
#include "SwiftKernelsValidation.h"
#endif

namespace JSC {

// Not CPU(ARM64)-only: AssemblerCommon.h declares these on every CPU, and Air (Arg::isValidBitImmForm)
// references them behind a non-constexpr isARM64() check.

ARM64LogicalImmediate ARM64LogicalImmediate::create32(uint32_t value)
{
    ARM64LogicalImmediate result(JSCSwift_ARM64LogicalImmediate_create32(value));
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]] {
        int expected = create32Cxx(value).m_value;
        RELEASE_ASSERT(result.m_value == expected, value, result.m_value, expected);
    }
#endif
    return result;
}

ARM64LogicalImmediate ARM64LogicalImmediate::create64(uint64_t value)
{
    ARM64LogicalImmediate result(JSCSwift_ARM64LogicalImmediate_create64(value));
#if ENABLE(SWIFT_KERNELS_VALIDATION)
    if (swiftKernelsValidationEnabled()) [[unlikely]] {
        int expected = create64Cxx(value).m_value;
        RELEASE_ASSERT(result.m_value == expected, value, result.m_value, expected);
    }
#endif
    return result;
}

} // namespace JSC

#endif // USE(SWIFT_KERNELS)
