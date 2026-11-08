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

// C entry points for ARM64LogicalImmediateSwiftBoundary.h, called only from assembler/AssemblerCommon.cpp.
// Scalars in, scalar out: there is nothing unsafe here. Internal, so the symbols stay hidden inside
// JavaScriptCore. @inline(__always) folds the safe core into the C entry point itself rather than leaving a
// thunk that branches to a Swift-native copy.

@_cdecl("JSCSwift_ARM64LogicalImmediate_create32")
@inline(__always)
func JSCSwift_ARM64LogicalImmediate_create32(_ value: UInt32) -> Int32 {
    ARM64LogicalImmediateEncoder.create32(value)
}

@_cdecl("JSCSwift_ARM64LogicalImmediate_create64")
@inline(__always)
func JSCSwift_ARM64LogicalImmediate_create64(_ value: UInt64) -> Int32 {
    ARM64LogicalImmediateEncoder.create64(value)
}
