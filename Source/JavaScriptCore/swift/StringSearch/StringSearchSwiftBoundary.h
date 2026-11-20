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

// The JS string search (StringSearch.swift): String.prototype.indexOf/includes/split/replace/replaceAll, atom RegExps
// and the DFG string operations. The symbols are hidden in JavaScriptCore; only runtime/VM.cpp (JSC::stringSearch and
// JSC::stringSearchAll) calls them, never header-inline code.
//
// Subject and pattern are a pointer and a length in code units each, Latin-1 bytes or UTF-16 code units as given by
// the width bits; a UTF-16 pointer need not be 2-byte aligned, and a pointer may be null only if its length is 0.

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    JSCSwift_StringSearch_Subject8Bit = 1,
    JSCSwift_StringSearch_Pattern8Bit = 2,
};

// The first index >= start at which the pattern occurs in the subject, or -1. start > subjectLength gives -1; an empty
// pattern gives start.
__attribute__((nothrow)) intptr_t JSCSwift_StringSearch_find(const void* subject, size_t subjectLength, const void* pattern, size_t patternLength, uint32_t widths, size_t start);

// The matches found by JSCSwift_StringSearch_findAll (4 words).
typedef struct {
    size_t count; // Matches found by this call.
    size_t last; // The last of them (if count).
    size_t end; // The end of the last match (last + patternLength), or start if count is 0: where to continue.
    size_t gapCount; // (begin, end) pairs written to gaps.
} JSCSwift_StringSearch_Matches;

// Successive non-overlapping matches of a non-empty pattern (traps on an empty one, or on gapCapacity 0): the first at
// or after start, each next one at or after the end of the previous one. For each match, the gap since the end of the
// previous one (since start for the first) is written to gaps as an int32_t pair (begin, end) when it is not empty.
// Stops after the match that fills the last of the gapCapacity pairs (more matches may follow from result->end), or
// when there are no more matches (start > subjectLength: none).
__attribute__((nothrow)) void JSCSwift_StringSearch_findAll(const void* subject, size_t subjectLength, const void* pattern, size_t patternLength, uint32_t widths, size_t start, int32_t* gaps, size_t gapCapacity, JSCSwift_StringSearch_Matches* result);

#ifdef __cplusplus
}
#endif
