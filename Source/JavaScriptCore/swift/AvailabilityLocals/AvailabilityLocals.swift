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

// The loops over AvailabilityMap::m_locals (dfg/DFGAvailabilityMap.cpp): operator==, merge and clear. They run on
// the DFG/FTL compiler threads, mostly in OSRAvailabilityAnalysisPhase.
//
// A JSC::DFG::Availability (dfg/DFGAvailability.h) is 16 bytes, seen here as two 64-bit words (the C++ glue
// static_asserts this layout):
//   word 0: m_node, a Node* used only as an opaque value: 0 = undecided, 1 = unavailableMarker(), else a node.
//   word 1: m_flushedAt, a FlushedAt (dfg/DFGFlushedAt.h), little-endian: byte 0 = FlushFormat (DeadFlush = 0 ...
//           ConflictingFlush = 7), bytes 1-3 = padding (never compared, unspecified), bytes 4-7 = the
//           VirtualRegister (int32; VirtualRegister::invalidVirtualRegister = 0x3fffffff).
// Swift never dereferences a node.

enum AvailabilityLocals {
    // The FlushedAt bits that operator== compares: the format byte and the virtual register, not the padding.
    @inline(__always) static var flushFieldsMask: UInt64 { 0xffff_ffff_0000_00ff }
    @inline(__always) static var formatMask: UInt64 { 0xff }
    // FlushedAt(DeadFlush): format DeadFlush (0), invalid virtual register.
    @inline(__always) static var deadFlush: UInt64 { 0x3fff_ffff &<< 32 }
    // FlushedAt(ConflictingFlush): format ConflictingFlush (7), invalid virtual register.
    @inline(__always) static var conflictingFlush: UInt64 { (0x3fff_ffff &<< 32) | 7 }
    // Availability::unavailableMarker().
    @inline(__always) static var unavailableNode: UInt64 { 1 }

    // `a == b` for two equally long Operands<Availability>, element by element in order (Availability's
    // defaulted operator== compares m_node, then FlushedAt's format and virtual register).
    @inline(__always)
    static func equal(_ a: Span<UInt64>, _ b: Span<UInt64>) -> Bool {
        precondition(a.count == b.count && a.count % 2 == 0)
        var i = 0
        while i < a.count {
            if a[i] != b[i] || (a[i &+ 1] ^ b[i &+ 1]) & flushFieldsMask != 0 {
                return false
            }
            i &+= 2
        }
        return true
    }

    // AvailabilityMap::merge's locals loop: `m_locals[i] = other.m_locals[i].merge(m_locals[i])` for every
    // i < other.m_locals.size(). With a = other[i] and b = self[i], Availability::merge is
    //   node:  a == 0 ? b : (b == 0 ? a : (a == b ? a : unavailableMarker))           (mergeNodes)
    //   flush: a dead ? b : (b dead ? a : (a == b ? a : FlushedAt(ConflictingFlush)))  (FlushedAt::merge)
    // where "dead" is format == DeadFlush and == compares format and virtual register. Every element is
    // independent, so the order of the loop is not observable (the C++ goes downwards). C++ indexes self out of
    // bounds when it is the shorter one (UnsafeVectorOverflow); here that traps.
    @inline(__always)
    static func merge(_ target: inout MutableSpan<UInt64>, _ other: Span<UInt64>) {
        precondition(other.count <= target.count && other.count % 2 == 0)
        var i = 0
        while i < other.count {
            let aNode = other[i]
            let bNode = target[i]
            let aFlush = other[i &+ 1]
            let bFlush = target[i &+ 1]
            target[i] = aNode == 0 ? bNode : (bNode == 0 || aNode == bNode ? aNode : unavailableNode)
            if aFlush & formatMask == 0 {
                target[i &+ 1] = bFlush
            } else if bFlush & formatMask == 0 || (aFlush ^ bFlush) & flushFieldsMask == 0 {
                target[i &+ 1] = aFlush
            } else {
                target[i &+ 1] = conflictingFlush
            }
            i &+= 2
        }
    }

    // AvailabilityMap::clear's `m_locals.fill(Availability())`: every element = {nullptr, FlushedAt(DeadFlush)}.
    @inline(__always)
    static func fillWithDead(_ target: inout MutableSpan<UInt64>) {
        precondition(target.count % 2 == 0)
        // i is the index of an element's second word: with i < count, both stores are provably in bounds, so
        // the bounds checks leave the loop and it vectorizes.
        var i = 1
        while i < target.count {
            target[i &- 1] = 0
            target[i] = deadFlush
            i &+= 2
        }
    }
}
