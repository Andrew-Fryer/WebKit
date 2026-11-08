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

// The lattice queries of JSC::DFG::AbstractHeap (dfg/DFGAbstractHeap.h): isStrictSubtypeOf, overlaps and
// supertype, computed directly on AbstractHeap's 64-bit encoding (its m_value).
//
// Encoding:
//   bits 0...47   payload value (Operand bits, a pointer, a DOMJIT::HeapRange, a small integer)
//   bits 48...54  AbstractHeapKind
//   bit  55       the payload is TOP
//   bits 56...63  also part of the payload value: always 0 for heaps DFG makes, but == compares them
//
// Kind numbers (FOR_EACH_ABSTRACT_HEAP_KIND; DFGAbstractHeap.cpp static_asserts every one used here):
//   0 InvalidAbstractHeap, 1 World, 2 Stack, 3 Heap, 4...14 object fields, 15 NamedProperties,
//   16 IndexedProperties, 17...23 its children (Indexed*Properties, DirectArgumentsProperties,
//   ScopeProperties, TypedArrayProperties), 24...39 other state, 40 DOMState, 41 Watchpoint_fire,
//   42 MiscFields, 43 SideState. 44...127 are not kinds.
//
// Hierarchy (AbstractHeap::superKind and supertype): a heap whose payload is not TOP is a direct subtype of
// TOP of its own kind; TOP(k) is a direct subtype of TOP(superKind(k)); TOP(World) is the root. superKind
// maps 17...23 to IndexedProperties, the other kinds 4...42 to Heap, Stack/Heap/SideState to World, and
// everything else to InvalidAbstractHeap. Two DOMState heaps compare by their DOMJIT::HeapRange instead.
//
// C++ answers isStrictSubtypeOf by walking supertype() up to World. Here the chain of a valid kind is a
// bitmask, so the common case (both kinds valid, not both DOMState) is branch-free. The rest (DOMState
// pairs, kinds that are not valid) goes through an exact model of the walk. For InvalidAbstractHeap and
// unknown kinds the C++ walk only stops if `other` is one of the heaps it visits; otherwise it is a
// side-effect-free infinite loop (undefined behavior in C++), and Swift traps instead.
//
// Predicates return 0 or 1 as Int rather than Bool: a Bool result has to be renormalized after every call,
// which would turn the tail calls to the slow paths into calls with a stack frame.

enum AbstractHeapLattice {
    @inline(__always) static var worldKind: UInt64 { 1 }
    @inline(__always) static var stackKind: UInt64 { 2 }
    @inline(__always) static var heapKind: UInt64 { 3 }
    @inline(__always) static var indexedPropertiesKind: UInt64 { 16 }
    @inline(__always) static var domStateKind: UInt64 { 40 }
    @inline(__always) static var sideStateKind: UInt64 { 43 }

    @inline(__always) static var kindShift: UInt64 { 48 }
    @inline(__always) static var kindMask: UInt64 { 0x7f &<< 48 }
    @inline(__always) static var topFlag: UInt64 { 1 &<< 55 }

    // 0 or 1 without a Bool: a Bool becomes a branch in SIL, and LLVM doesn't always turn a chain of those
    // back into conditional selects.
    @inline(__always)
    static func isZero(_ value: UInt64) -> UInt64 { ((value | (0 &- value)) &>> 63) ^ 1 }

    @inline(__always)
    static func isNonZero(_ value: UInt64) -> UInt64 { (value | (0 &- value)) &>> 63 }

    @inline(__always)
    static func kind(_ heap: UInt64) -> UInt64 { (heap &>> kindShift) & 0x7f }

    // 1 if the payload is TOP, else 0.
    @inline(__always)
    static func topBit(_ heap: UInt64) -> UInt64 { (heap &>> 55) & 1 }

    // AbstractHeap(kind): TOP of a kind, the only form the supertype walk produces.
    @inline(__always)
    static func top(ofKind kind: UInt64) -> UInt64 { (kind &<< kindShift) | topFlag }

    // 1 if `heap` is exactly TOP of its kind (no other payload bits), else 0.
    @inline(__always)
    static func isTopOfItsKind(_ heap: UInt64) -> UInt64 { isZero((heap & ~kindMask) ^ topFlag) }

    // World...SideState.
    @inline(__always)
    static func isValidKind(_ kind: UInt64) -> Bool { kind &- 1 < 43 }

    // AbstractHeap::superKind, for any 7-bit kind.
    @inline(__always)
    static func superKind(_ kind: UInt64) -> UInt64 {
        if kind &- 17 < 7 { return indexedPropertiesKind }
        if kind &- 4 < 39 { return heapKind }
        if kind == stackKind || kind == heapKind || kind == sideStateKind { return worldKind }
        return 0
    }

    // For a valid kind: bit j is set iff j is the kind itself or one of its superkinds (World included).
    // Heap (bit 3) is above every kind in 4...42: its children, and IndexedProperties' children 17...23.
    // IndexedProperties (bit 16) is above 17...23. World (bit 1) is above everything.
    @inline(__always)
    static func kindChain(_ kind: UInt64) -> UInt64 {
        let heapChildrenAndGrandchildren: UInt64 = 0x7ff_ffff_fff0 // bits 4...42
        let indexedPropertiesChildren: UInt64 = 0xfe_0000 // bits 17...23
        return (1 &<< worldKind) | (1 &<< kind)
            | (((heapChildrenAndGrandchildren &>> kind) & 1) &<< heapKind)
            | (((indexedPropertiesChildren &>> kind) & 1) &<< indexedPropertiesKind)
    }

    // Both kinds valid and not both DOMState: the pairs the bitmask form answers.
    @inline(__always)
    static func isFastPair(_ kindA: UInt64, _ kindB: UInt64) -> Bool {
        isValidKind(kindA) && isValidKind(kindB) && (kindA != domStateKind || kindB != domStateKind)
    }

    // For a fast pair: 1 if `a` is a strict subtype of `b`, else 0. The walk from `a` visits TOP(k) for each
    // k in the chain of a's kind (minus the kind itself when `a` is already TOP), and visits nothing from
    // World, so `b` has to be exactly TOP of one of those kinds.
    @inline(__always)
    static func strictSubtypeBit(_ a: UInt64, _ kindA: UInt64, _ b: UInt64, _ kindB: UInt64) -> UInt64 {
        let strictSuperkinds = kindChain(kindA) & ~(topBit(a) &<< kindA)
        return isTopOfItsKind(b) & isNonZero(kindA ^ worldKind) & ((strictSuperkinds &>> kindB) & 1)
    }

    // DOMJIT::HeapRange::fromRaw(payload.value32()).isStrictSubtypeOf(...): begin is the low 16 bits, end the
    // next 16.
    @inline(__always)
    static func domStateIsStrictSubtype(_ a: UInt64, _ b: UInt64) -> Bool {
        if topBit(a) != 0 { return false }
        if topBit(b) != 0 { return true }
        let aBegin = UInt16(truncatingIfNeeded: a)
        let aEnd = UInt16(truncatingIfNeeded: a &>> 16)
        let bBegin = UInt16(truncatingIfNeeded: b)
        let bEnd = UInt16(truncatingIfNeeded: b &>> 16)
        if aBegin == aEnd || bBegin == bEnd { return false }
        if aBegin == bBegin && aEnd == bEnd { return false }
        return bBegin <= aBegin && aEnd <= bEnd
    }

    // AbstractHeap::isStrictSubtypeOf step by step, for every input on which the C++ terminates.
    @inline(never)
    static func isStrictSubtypeOfSlow(_ a: UInt64, _ b: UInt64) -> Int {
        let kindA = kind(a)
        if kindA == domStateKind && kind(b) == domStateKind { return domStateIsStrictSubtype(a, b) ? 1 : 0 }
        if kindA == worldKind { return 0 }
        // The walk visits TOP(k), TOP(superKind(k)), ... and stops after the first World, where k is the kind
        // of `a` (or its superkind when `a` is already TOP). A valid kind reaches World within 4 steps; any
        // other kind reaches InvalidAbstractHeap within 2, which is its own superkind.
        var k = topBit(a) != 0 ? superKind(kindA) : kindA
        for _ in 0..<4 {
            if b == top(ofKind: k) { return 1 }
            if k == worldKind { return 0 }
            k = superKind(k)
        }
        // C++ loops forever here: the walk is stuck at TOP(InvalidAbstractHeap), which isn't `b`.
        preconditionFailure()
    }

    @inline(never)
    static func overlapsSlow(_ a: UInt64, _ b: UInt64) -> Int {
        // Same evaluation order as C++, which matters when one of the walks would not terminate.
        if a == b { return 1 }
        if isStrictSubtypeOfSlow(a, b) != 0 { return 1 }
        return isStrictSubtypeOfSlow(b, a)
    }

    @inline(__always)
    static func isStrictSubtypeOf(_ a: UInt64, _ b: UInt64) -> Int {
        let kindA = kind(a), kindB = kind(b)
        guard isFastPair(kindA, kindB) else { return isStrictSubtypeOfSlow(a, b) }
        return Int(truncatingIfNeeded: strictSubtypeBit(a, kindA, b, kindB))
    }

    @inline(__always)
    static func overlaps(_ a: UInt64, _ b: UInt64) -> Int {
        let kindA = kind(a), kindB = kind(b)
        guard isFastPair(kindA, kindB) else { return overlapsSlow(a, b) }
        return Int(truncatingIfNeeded: isZero(a ^ b) | strictSubtypeBit(a, kindA, b, kindB) | strictSubtypeBit(b, kindB, a, kindA))
    }

    // AbstractHeap::supertype. World's supertype is AbstractHeap(), which is 0.
    @inline(__always)
    static func supertype(_ heap: UInt64) -> UInt64 {
        let k = kind(heap)
        if k == worldKind { return 0 }
        return top(ofKind: topBit(heap) != 0 ? superKind(k) : k)
    }
}
