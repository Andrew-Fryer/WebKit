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

// The recency order of cachedCGColor()'s cache (platform/graphics/cg/ColorCG.cpp), implemented in Swift in
// CGColorCache.swift. Only include this from WebCore .cpp files: the Swift symbols are hidden in WebCore.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Bits of order[i] and of the lookup result (CGColorCache.swift has the same values).
enum {
    WebCoreSwiftCGColorCacheSlotMask = 0x7f, // The entry's slot in the C++ storage.
    WebCoreSwiftCGColorCacheOutOfLine = 0x80, // order[i] only: the entry's Color is out of line.
    WebCoreSwiftCGColorCacheMiss = 0x100, // Result only: the key was not found; C++ must store it in that slot.
};

// keys and order have capacity (<= 128) elements, of which the first *size (in/out) are live, least recently used
// first: keys[i] is the bits of an entry's Color, order[i] its slot | WebCoreSwiftCGColorCacheOutOfLine if it is out
// of line. A lookup finds the entry equal to the key (keyBits are its bits) and makes it the most recently used one,
// or makes room for the key as the most recently used entry: a new slot (++*size) or the least recently used entry's
// slot. It returns that slot, | WebCoreSwiftCGColorCacheMiss on a miss.

// For an inline key (equal exactly to the entry with the same bits).
uint32_t WebCoreSwift_CGColorCache_lookup(uint64_t* keys, uint8_t* order, size_t capacity, size_t* size, uint64_t keyBits);

// Returns whether the Color in the given slot equals the out-of-line key described by context (Color::operator==).
typedef bool (*WebCoreSwiftCGColorCacheEqualsFunction)(const void* context, uint8_t slot);

// For an out-of-line key: outOfLineEntryEquals(context, slot) decides equality for each out-of-line entry scanned.
uint32_t WebCoreSwift_CGColorCache_lookupOutOfLine(uint64_t* keys, uint8_t* order, size_t capacity, size_t* size,
    uint64_t keyBits, const void* context, WebCoreSwiftCGColorCacheEqualsFunction outOfLineEntryEquals);

#ifdef __cplusplus
}
#endif
