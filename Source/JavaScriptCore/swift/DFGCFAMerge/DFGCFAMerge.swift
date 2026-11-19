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
// Swift never dereferences a pointer. mergeValues does every merge these words determine without allocating or
// freeing a structure list (where only the copy of a set into a clear value does, it merges the other words and the
// C++ caller copies the set). Any other merge (a fat set on either side, or two different single structures) is left
// untouched and reported; for each of those the C++ caller passes the two sets' lists to mergeSlowValue, which does
// the whole AbstractValue::merge and says what the destination's set becomes (a new word, a copy of the source's
// set, or structures that the caller stores in the destination's list or a new one), since C++ owns the lists.

enum DFGCFAMerge {
    @inline(__always) static var fatFlag: UInt64 { 1 }
    @inline(__always) static var clobberedFlag: UInt64 { 2 }
    @inline(__always) static var flags: UInt64 { 3 }
    @inline(__always) static var topValue: UInt64 { 4 }
    @inline(__always) static var arrayModesMask: UInt64 { 0xffff_ffff }

    enum Outcome {
        case unchanged
        case changed
        // Needs a structure list: left untouched, for mergeSlowValue.
        case slow
        // *this = other where that allocates or frees a list: words 1-3 are copied, and the C++ caller copies the set.
        case copyStructure
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
            destination[base &+ 1] = sourceType
            destination[base &+ 2] = source[base &+ 2]
            destination[base &+ 3] = source[base &+ 3]
            if (destinationStructure | sourceStructure) & fatFlag != 0 {
                return .copyStructure
            }
            destination[base] = sourceStructure
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
    // destinationNodes[i] != 0. Returns whether a value changed. Each value that needs a structure list is left
    // untouched and its index appended to slowIndices (for mergeSlowValue), except a copyStructure one, whose index
    // is appended with bit 31 set (the caller copies the set); slowCount is how many were. Values are
    // independent, so the order is not observable (the C++ SSA loop goes downwards). C++ indexes out of bounds when count exceeds a
    // size (UnsafeVectorOverflow); here that traps.
    @inline(__always)
    static func mergeValues(_ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>, destinationNodes: Span<UInt64>, checkNodes: Bool, count: Int, slowIndices: inout MutableSpan<UInt32>, slowCount: inout Int) -> Bool {
        precondition(count >= 0 && count <= destination.count / 4 && count <= source.count / 4 && count <= slowIndices.count && count < 1 &<< 31)
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
            case .copyStructure:
                changed = true
                slowIndices[slow] = UInt32(i) | 1 &<< 31
                slow &+= 1
            }
        }
        slowCount = slow
        return changed
    }

    // What mergeSlowValue did to the destination's StructureAbstractValue, for the C++ caller (which owns the lists)
    // to apply: the kinds in DFGCFAMergeSwiftBoundary.h.
    @inline(__always) static var keepStructure: UInt32 { 0 } // unchanged
    @inline(__always) static var setWord: UInt32 { 1 } // free the list, if fat; then the word is `word` (thin or TOP)
    @inline(__always) static var appendToList: UInt32 { 2 } // the fat list keeps its storage; list[0 ..< length] is appended
    @inline(__always) static var newList: UInt32 { 3 } // free the list, if fat; a new list of `capacity` holds list[0 ..< length]
    @inline(__always) static var copySourceSet: UInt32 { 4 } // the source's set is copied (StructureAbstractValue's operator=)
    @inline(__always) static var polymorphismLimit: Int { 10 } // StructureAbstractValue::polymorphismLimit
    @inline(__always) static var defaultStartingSize: UInt32 { 4 } // TinyPtrSet::defaultStartingSize

    struct StructureResult {
        var kind: UInt32
        var capacity: UInt32 // newList: the out-of-line list's m_capacity
        var word: UInt64 // setWord: the new word; list kinds: the clobbered flag (0 or clobberedFlag)
        var length: Int // list kinds: how many elements of `list`
    }

    // The destination's TinyPtrSet<RegisteredStructure> while merging: thin (`single`, 0 if empty) or fat, with
    // `length` elements in an out-of-line list of `capacity`. While the fat destination keeps its original list
    // (`reallocated` false), its elements are destinationList[0 ..< prefix] followed by list[0 ..< length - prefix];
    // once it would be reallocated (or it becomes fat), they are list[0 ..< length] and prefix is 0.
    struct SetState {
        var fat: Bool
        var single: UInt64
        var length: Int
        var prefix: Int
        var capacity: UInt32
        var reallocated: Bool
    }

    // TinyPtrSet::addOutOfLine.
    @inline(__always)
    static func addOutOfLine(_ value: UInt64, _ set: inout SetState, _ destinationList: Span<UInt64>, _ list: inout MutableSpan<UInt64>) -> Bool {
        // (prefix is destinationList.count or 0.)
        if set.prefix != 0 {
            for i in destinationList.indices where destinationList[i] == value {
                return false
            }
        }
        let appended = set.length &- set.prefix
        for i in list.indices.prefix(appended) where list[i] == value {
            return false
        }
        if UInt32(truncatingIfNeeded: set.length) >= set.capacity {
            // OutOfLineList::create(m_capacity * 2), the old entries copied, then the value: move the list's
            // elements to `list` (the appended ones up behind the original ones).
            set.capacity = set.capacity &* 2
            set.reallocated = true
            if set.prefix != 0 {
                var k = appended
                while k > 0 {
                    k &-= 1
                    list[set.prefix &+ k] = list[k]
                }
                for i in 0 ..< set.prefix {
                    list[i] = destinationList[i]
                }
                set.prefix = 0
            }
        }
        list[set.length &- set.prefix] = value
        set.length &+= 1
        return true
    }

    // TinyPtrSet::add.
    @inline(__always)
    static func add(_ value: UInt64, _ set: inout SetState, _ destinationList: Span<UInt64>, _ list: inout MutableSpan<UInt64>) -> Bool {
        if set.fat {
            return addOutOfLine(value, &set, destinationList, &list)
        }
        if set.single == value {
            return false
        }
        if set.single == 0 {
            set.single = value
            return true
        }
        // A new list of defaultStartingSize: the single entry, then the value.
        list[0] = set.single
        list[1] = value
        set = SetState(fat: true, single: 0, length: 2, prefix: 0, capacity: defaultStartingSize, reallocated: true)
        return true
    }

    // StructureAbstractValue::merge(other) for a destination that is not clear (and so not *this = other).
    @inline(__always)
    static func mergeStructures(
        _ destinationStructure: UInt64, _ sourceStructure: UInt64,
        destinationList: Span<UInt64>, destinationCapacity: UInt32, sourceList: Span<UInt64>,
        list: inout MutableSpan<UInt64>, result: inout StructureResult
    ) -> Bool {
        // other.isClear(): thin and empty (a fat set is never empty, whatever its length).
        if sourceStructure & ~clobberedFlag == 0 {
            return false
        }
        if destinationStructure == topValue {
            return false
        }
        if sourceStructure == topValue {
            // makeTop(), which frees a fat destination's list.
            result = StructureResult(kind: setWord, capacity: 0, word: topValue, length: 0)
            return true
        }

        // mergeSlow(other): the clobbered bit, then mergeNotTop(other.m_set).
        var changed = false
        var clobbered = destinationStructure & clobberedFlag
        if clobbered == 0 && sourceStructure & clobberedFlag != 0 {
            clobbered = clobberedFlag
            changed = true
        }
        var set = SetState(fat: false, single: destinationStructure & ~flags, length: 0, prefix: 0, capacity: 0, reallocated: false)
        if destinationStructure & fatFlag != 0 {
            set = SetState(fat: true, single: 0, length: destinationList.count, prefix: destinationList.count, capacity: destinationCapacity, reallocated: false)
        }

        // TinyPtrSet::merge(other).
        var merged = false
        if sourceStructure & fatFlag == 0 {
            let entry = sourceStructure & ~flags
            if entry != 0 {
                merged = add(entry, &set, destinationList, &list)
            }
        } else if sourceList.count >= 2 {
            // mergeOtherOutOfLine: a thin destination first becomes a list with room for both sets.
            if !set.fat {
                let hasSingle = set.single != 0
                if hasSingle {
                    list[0] = set.single
                }
                set = SetState(fat: true, single: 0, length: hasSingle ? 1 : 0, prefix: 0,
                               capacity: UInt32(truncatingIfNeeded: sourceList.count &+ (hasSingle ? 1 : 0)), reallocated: true)
            }
            for item in sourceList.indices where addOutOfLine(sourceList[item], &set, destinationList, &list) {
                merged = true
            }
        } else {
            // A fat list of one entry. (The C++ reads lengthSpan()[0]; an empty list traps here.)
            merged = add(sourceList[0], &set, destinationList, &list)
        }

        if merged {
            changed = true
            // mergeNotTop: past polymorphismLimit structures, makeTop().
            if (set.fat ? set.length : (set.single != 0 ? 1 : 0)) > polymorphismLimit {
                result = StructureResult(kind: setWord, capacity: 0, word: topValue, length: 0)
                return true
            }
        }
        if set.fat {
            if set.reallocated {
                result = StructureResult(kind: newList, capacity: set.capacity, word: clobbered, length: set.length)
            } else if changed {
                result = StructureResult(kind: appendToList, capacity: set.capacity, word: clobbered, length: set.length &- set.prefix)
            }
        } else if changed {
            result = StructureResult(kind: setWord, capacity: 0, word: set.single | clobbered, length: 0)
        }
        return changed
    }

    // destination.merge(source), AbstractValue::merge, for one value of any kind (mergeValues reports the ones that
    // need it). destination and source are one value's four words. destinationList / sourceList are the elements of
    // a fat set's list (empty for a thin set), destinationCapacity the destination list's m_capacity. The
    // destination's words 1-3 are updated here; its set is described by `result`, with any list in list[0 ..< length].
    @inline(__always)
    static func mergeSlowValue(
        _ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>,
        destinationList: Span<UInt64>, destinationCapacity: UInt32, sourceList: Span<UInt64>,
        list: inout MutableSpan<UInt64>, result resultWords: inout MutableSpan<UInt64>
    ) -> Bool {
        precondition(destination.count == 4 && source.count == 4 && resultWords.count == 3)
        var result = StructureResult(kind: keepStructure, capacity: 0, word: 0, length: 0)
        let changed = mergeValueWithLists(&destination, source, destinationList: destinationList, destinationCapacity: destinationCapacity,
                                          sourceList: sourceList, list: &list, result: &result)
        resultWords[0] = UInt64(result.kind) | UInt64(result.capacity) &<< 32
        resultWords[1] = result.word
        resultWords[2] = UInt64(truncatingIfNeeded: result.length)
        return changed
    }

    @inline(__always)
    static func mergeValueWithLists(
        _ destination: inout MutableSpan<UInt64>, _ source: Span<UInt64>,
        destinationList: Span<UInt64>, destinationCapacity: UInt32, sourceList: Span<UInt64>,
        list: inout MutableSpan<UInt64>, result: inout StructureResult
    ) -> Bool {
        let destinationStructure = destination[0]
        let sourceStructure = source[0]
        precondition(destinationStructure & fatFlag != 0 || destinationList.isEmpty)
        precondition(sourceStructure & fatFlag != 0 || sourceList.isEmpty)
        // Room for both lists and a thin entry, and for the two-entry list that add() makes from two thin sets.
        precondition(list.count >= destinationList.count &+ sourceList.count &+ 2)
        result = StructureResult(kind: keepStructure, capacity: 0, word: 0, length: 0)

        let sourceType = source[1]
        // if (other.isClear()) return false;
        if sourceType == 0 {
            return false
        }
        let destinationType = destination[1]
        if destinationType == 0 {
            // *this = other. StructureAbstractValue's assignment copies other's set (a fat one into a new list) and
            // other's clobbered bit: the C++ caller copies the object.
            destination[1] = sourceType
            destination[2] = source[2]
            destination[3] = source[3]
            result = StructureResult(kind: copySourceSet, capacity: 0, word: 0, length: 0)
            return true
        }

        // mergeSpeculation, mergeArrayModes (the epoch half of word 2 is kept), m_structure.merge, the m_value rule.
        let type = destinationType | sourceType
        let destinationModesAndEpoch = destination[2]
        let modesAndEpoch = destinationModesAndEpoch | (source[2] & arrayModesMask)
        var changed = type != destinationType || modesAndEpoch != destinationModesAndEpoch
        destination[1] = type
        destination[2] = modesAndEpoch
        if mergeStructures(destinationStructure, sourceStructure, destinationList: destinationList, destinationCapacity: destinationCapacity,
                           sourceList: sourceList, list: &list, result: &result) {
            changed = true
        }
        let destinationValue = destination[3]
        if destinationValue != source[3] {
            changed = changed || destinationValue != 0
            destination[3] = 0
        }
        return changed
    }
}
