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

// Memory-safe port of Style::MatchResultCache's inline-style diff (style/MatchResultCache.cpp):
// isUsableAfterInlineStyleChange() followed by computeAndUpdateChangedProperties(), fused into one pass over the
// cache entry. The C++ names are kept. cascadeAliasProperty() comes in as a table the glue builds.
//
// The entry's snapshot (Vector<OriginalInlineProperty>) stays C++-owned: its RefPtrs keep the original values alive,
// so a freed value whose address is reused can never compare as unchanged. The kernel reads it, and the current
// CSSProperty records, as words; the value addresses are only compared. Instead of nulling a changed
// valueIfUnchanged (a deref), the kernel returns its index and the glue resets the RefPtr.
//
// The memory layouts are documented in MatchResultCacheDiffSwiftBoundary.h and checked on the C++ side.
// No allocation, no reference counting, no callbacks.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

enum MatchResultCacheDiff {
    // The results (MatchResultCacheDiffSwiftBoundary.h).
    @inline(__always) static var unusable: UInt32 { 0 }
    @inline(__always) static var allNormalProperties: UInt32 { 1 }
    @inline(__always) static var changedIDs: UInt32 { 2 }

    // Indices into `state`.
    @inline(__always) static var stateEntryInlineStyle: Int { 0 }
    @inline(__always) static var stateCurrentInlineStyle: Int { 1 }
    @inline(__always) static var stateConstantsAndIDCount: Int { 2 }
    @inline(__always) static var stateNullCount: Int { 3 }
    @inline(__always) static var stateCount: Int { 4 }

    // OriginalInlineProperty word 0: propertyID bits 0-15, wasImportant bits 16-23.
    @inline(__always) static func originalID(_ word: UInt64) -> UInt32 { UInt32(truncatingIfNeeded: word) & 0xFFFF }
    @inline(__always) static func originalWasImportantByte(_ word: UInt64) -> UInt64 { (word >> 16) & 0xFF }
    // StylePropertyMetadata (CSSProperty word 0): m_propertyID bits 0-9, m_important bit 13.
    @inline(__always) static func currentID(_ word: UInt64) -> UInt32 { UInt32(truncatingIfNeeded: word) & 0x3FF }
    @inline(__always) static func currentImportantBit(_ word: UInt64) -> UInt64 { (word >> 13) & 1 }

    /// isUsableAfterInlineStyleChange(entry, currentInlineStyle) and computeAndUpdateChangedProperties(entry), fused
    /// into one pass over the entry. Returns `unusable`, `allNormalProperties` or `changedIDs` and sets
    /// state[stateConstantsAndIDCount] / state[stateNullCount] (see MatchResultCacheDiffSwiftBoundary.h).
    ///
    /// The upstream order: isUsableAfterInlineStyleChange checks every index before computeAndUpdateChangedProperties
    /// runs, so an id mismatch anywhere makes the entry unusable with no snapshot update, even past a change that
    /// ends the diff early.
    @inline(always)
    static func changedProperties(state: inout MutableSpan<UInt64>, original: Span<UInt64>, current: Span<UInt64>,
        aliases: Span<UInt16>, out: inout MutableSpan<UInt32>) -> UInt32
    {
        precondition(state.count == stateCount)
        let constants = state[stateConstantsAndIDCount]
        state[stateConstantsAndIDCount] = 0
        state[stateNullCount] = 0

        // isUsableAfterInlineStyleChange: the entry was made for this inline style, with the same number of properties
        // and the same ids in the same order (checked in the loop below).
        // Only allow the same exact properties after a change. This way the previous values in ComputedStyle are guaranteed to get overwritten.
        // Adding properties could be allowed without other changes. Removal would require resetting the removed property to initial
        // value in the style builder.
        if state[stateEntryInlineStyle] != state[stateCurrentInlineStyle] || original.count != current.count {
            return unusable
        }

        let firstLowPriorityProperty = UInt32(truncatingIfNeeded: constants) & 0xFFFF
        let lineHeight = UInt32(truncatingIfNeeded: constants >> 16) & 0xFFFF
        let custom = UInt32(truncatingIfNeeded: constants >> 32) & 0xFFFF

        // Property i is words 2 * i (`low`) and 2 * i + 1 (`high`) of each span.
        let wordCount = original.count
        let propertyCount = wordCount / 2
        precondition(wordCount == 2 * propertyCount && out.count >= wordCount)
        var idCount = 0
        var nullCount = 0
        var high = 1
        while high < wordCount {
            let low = high &- 1
            let originalWord = original[low]
            let currentWord = current[low]
            let propertyID = currentID(currentWord)
            if originalID(originalWord) != propertyID {
                return unusable
            }

            // computeAndUpdateChangedProperties: the value (by address) or the priority changed. The priorities are
            // compared as upstream's compiled `bool ==` does, as bytes (wasImportant is 0 or 1).
            let originalValue = original[high]
            let importanceChange = (originalWasImportantByte(originalWord) ^ currentImportantBit(currentWord))
            if (originalValue ^ current[high]) | importanceChange == 0 {
                high &+= 2
                continue
            }

            // Assume that if a value or priority changes once then it will change more. Don't track changes anymore
            // (the glue nulls valueIfUnchanged).
            if originalValue != 0 {
                out[propertyCount &+ nullCount] = UInt32(truncatingIfNeeded: low >> 1)
                nullCount &+= 1
            }

            // FIXME: Support custom properties.
            // Only use partial applying with low-priority properties since we know changes to them can't
            // affect values of other properties.
            // FIXME: CSSPropertyID::LineHeight shouldn't be a low-priority property as others can depend on it via `lh` unit.
            if propertyID == custom || propertyID < firstLowPriorityProperty || propertyID == lineHeight {
                // isUsableAfterInlineStyleChange still has to hold for the rest.
                var mismatch: UInt32 = 0
                var idWord = high &+ 1
                while idWord < wordCount {
                    mismatch |= originalID(original[idWord]) ^ currentID(current[idWord])
                    idWord &+= 2
                }
                if mismatch != 0 {
                    return unusable
                }
                state[stateNullCount] = UInt64(nullCount)
                return allNormalProperties
            }

            out[idCount] = UInt32(aliases[Int(propertyID)])
            idCount &+= 1
            high &+= 2
        }

        state[stateConstantsAndIDCount] = UInt64(idCount)
        state[stateNullCount] = UInt64(nullCount)
        return changedIDs
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
