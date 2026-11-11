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

// The per-variable loops of JSC::DFG::InPlaceAbstractState::merge(from, to) (dfg/DFGInPlaceAbstractState.cpp): merge
// every from->valuesAtTail[i] into to->valuesAtHead[i] with AbstractValue::merge (dfg/DFGAbstractValue.h), skipping
// the variables whose to->variablesAtHead node is null in ThreadedCPS form (mergeVariableBetweenBlocks). They run on
// the DFG/FTL compiler threads, in the CFA phase.
//
// A JSC::DFG::AbstractValue is 32 bytes, seen here as four 64-bit words (the layout is static_asserted in
// dfg/DFGAbstractValue.h and dfg/DFGStructureAbstractValue.h):
//   word 0: m_structure, a StructureAbstractValue, which is one TinyPtrSet<RegisteredStructure> word (wtf/TinyPtrSet.h):
//           bit 0 = fat (the rest points to an out-of-line list), bit 1 = clobbered (TinyPtrSet's reserved flag),
//           4 = TOP (the reserved value), else thin: the bits above bit 1 are the set's only structure, 0 if empty.
//   word 1: m_type, a SpeculatedType; SpecNone (0) means the value is clear.
//   word 2: m_arrayModes (ArrayModes, 32 bits) in the low half, m_effectEpoch in the high half (little-endian).
//   word 3: m_value, an EncodedJSValue; JSValue's == and bool are bitwise.
// Swift never dereferences a pointer: it handles exactly the merges these words determine without allocating or
// freeing a structure list. Any other merge (a fat set on either side, or two different single structures, which
// need a new list) is left untouched and reported, and the C++ caller does it with the original AbstractValue::merge.

enum DFGCFAMerge {
    @inline(__always) static var fatFlag: UInt64 { 1 }
    @inline(__always) static var clobberedFlag: UInt64 { 2 }
    @inline(__always) static var flags: UInt64 { 3 }
    @inline(__always) static var topValue: UInt64 { 4 }
    @inline(__always) static var arrayModesMask: UInt64 { 0xffff_ffff }

    enum Outcome {
        case unchanged
        case changed
        // Needs a structure list: left untouched for the C++ AbstractValue::merge.
        case slow
    }

    // destination[base ..< base + 4].merge(source[base ..< base + 4]), AbstractValue::merge with
    // StructureAbstractValue::merge (and, for two thin sets, mergeSlow and TinyPtrSet::add) inlined.
    @inline(__always)
    static func mergeValue(_ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>, at base: Int) -> Outcome {
        let sourceType = source[base &+ 1]
        // if (other.isClear()) return false;
        if sourceType == 0 {
            return .unchanged
        }
        let destinationStructure = destination[base]
        let sourceStructure = source[base]
        let destinationType = destination[base &+ 1]
        if destinationType == 0 {
            // *this = other. Copying a thin (or TOP) set copies its word, clobbered bit included; a fat source would
            // need a new list and a fat destination would free its list.
            if (destinationStructure | sourceStructure) & fatFlag != 0 {
                return .slow
            }
            destination[base] = sourceStructure
            destination[base &+ 1] = sourceType
            destination[base &+ 2] = source[base &+ 2]
            destination[base &+ 3] = source[base &+ 3]
            return .changed
        }

        // m_structure.merge(other.m_structure).
        var structure = destinationStructure
        if sourceStructure & ~clobberedFlag == 0 {
            // other.isClear() (thin and empty; a fat set is never empty): no change.
        } else if destinationStructure == topValue {
            // isTop(): no change.
        } else if sourceStructure == topValue {
            // other.isTop(): makeTop(), which frees a fat destination's list.
            if destinationStructure & fatFlag != 0 {
                return .slow
            }
            structure = topValue
        } else if (destinationStructure | sourceStructure) & fatFlag != 0 {
            return .slow
        } else {
            // mergeSlow() for two thin sets: the clobbered bit is or-ed in, then m_set.merge(other.m_set) adds other's
            // only structure (TinyPtrSet::add keeps the clobbered bit). A thin set has at most one structure, so
            // mergeNotTop's polymorphismLimit can't be reached.
            let sourceEntry = sourceStructure & ~flags
            let destinationEntry = destinationStructure & ~flags
            if sourceEntry == destinationEntry {
                structure = destinationStructure | (sourceStructure & clobberedFlag)
            } else if destinationEntry == 0 {
                structure = sourceEntry | ((destinationStructure | sourceStructure) & clobberedFlag)
            } else {
                // Two different structures: TinyPtrSet::add allocates a list.
                return .slow
            }
        }

        // mergeSpeculation(m_type, other.m_type), mergeArrayModes(m_arrayModes, other.m_arrayModes) (the epoch half of
        // word 2 is not touched), and the m_value rule.
        let type = destinationType | sourceType
        let destinationModesAndEpoch = destination[base &+ 2]
        let modesAndEpoch = destinationModesAndEpoch | (source[base &+ 2] & arrayModesMask)
        var changed = (structure ^ destinationStructure) | (type ^ destinationType) | (modesAndEpoch ^ destinationModesAndEpoch) != 0
        destination[base] = structure
        destination[base &+ 1] = type
        destination[base &+ 2] = modesAndEpoch
        let destinationValue = destination[base &+ 3]
        if destinationValue != source[base &+ 3] {
            // result |= !!m_value; m_value = JSValue();
            changed = changed || destinationValue != 0
            destination[base &+ 3] = 0
        }
        return changed ? .changed : .unchanged
    }

    // Merges source value i into destination value i for every i < count, and in CPS form only where
    // destinationNodes[i] != 0. Returns whether a value changed. Each value that needs the C++ merge is left
    // untouched and its index appended to slowIndices; slowCount is how many were. Values are independent, so the
    // order is not observable (the C++ SSA loop goes downwards). C++ indexes out of bounds when count exceeds a
    // size (UnsafeVectorOverflow); here that traps.
    @inline(__always)
    static func mergeValues(_ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>, destinationNodes: Span<UInt64>, checkNodes: Bool, count: Int, slowIndices: inout MutableSpan<UInt32>, slowCount: inout Int) -> Bool {
        precondition(count >= 0 && count <= destination.count / 4 && count <= source.count / 4 && count <= slowIndices.count)
        if checkNodes {
            precondition(count <= destinationNodes.count)
            return mergeValuesLoop(&destination, source, destinationNodes, checkNodes: true, count, &slowIndices, &slowCount)
        }
        return mergeValuesLoop(&destination, source, destinationNodes, checkNodes: false, count, &slowIndices, &slowCount)
    }

    // The loop, specialized for a constant checkNodes once inlined.
    @inline(__always)
    static func mergeValuesLoop(_ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>, _ destinationNodes: Span<UInt64>, checkNodes: Bool, _ count: Int, _ slowIndices: inout MutableSpan<UInt32>, _ slowCount: inout Int) -> Bool {
        var changed = false
        var slow = 0
        for i in 0 ..< count {
            if checkNodes && destinationNodes[i] == 0 {
                continue
            }
            switch mergeValue(&destination, source, at: i &* 4) {
            case .unchanged:
                break
            case .changed:
                changed = true
            case .slow:
                slowIndices[slow] = UInt32(i)
                slow &+= 1
            }
        }
        slowCount = slow
        return changed
    }
}
