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

// Memory-safe port of Style::PropertyCascade's cascade building (style/PropertyCascade.cpp), for every cascade.
// One call builds the whole cascade: buildCascade, addNormalMatches, addMatch (with its rollback limits, the
// property allowlists, and cascadeAliasProperty / CSSProperty::isInheritedProperty / the allowlist checks through
// a table the glue builds), addPositionTryFallbackProperties, set, setLogicalGroupProperty, setPropertyInternal,
// setDelayingForRuleRollback, mayOverrideExistingProperty / hasCustomProperty, hasImportantProperties,
// addImportantMatches (with its stable sort) and sortLogicalGroupPropertyIDs. The C++ names are kept.
//
// What the kernel cannot read itself comes in as data, per property in match order (`inputs`, only for the
// cascades that can need it): shouldApplyAfterAnimation(), explicitlyInheritsFromParent(), and a dense id per
// custom-property name (custom properties are keyed by name). The custom-property sets and the AfterAnimation
// overrides go back as lists the glue stores into its hash containers.
//
// The memory layouts this code reads and writes are documented, and static_asserted on the C++ side, in
// PropertyCascadeSwiftBoundary.h. No allocation, no reference counting, no callbacks.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

/// The property storage of each matched declaration block, provided by the boundary.
protocol PropertyCascadeMatchStorage {
    /// The first `byteCount` storage bytes of match `match` (immutable: metadata[n] then packed pointers[n];
    /// mutable: CSSProperty[n]).
    @_lifetime(borrow self)
    func bytes(_ match: Int, byteCount: Int) -> RawSpan
}

enum PropertyCascade {
    // Indices into the in/out `state` span: the configuration, then the cascade's logical-group scalars (in/out)
    // and the overridden-property count (out).
    @inline(__always) static var stateUserAgentCount: Int { 0 }
    @inline(__always) static var stateUserCount: Int { 1 }
    @inline(__always) static var stateAuthorCount: Int { 2 }
    @inline(__always) static var statePositionTryCount: Int { 3 }
    @inline(__always) static var stateIncludedTypes: Int { 4 }
    @inline(__always) static var stateFirstLogicalGroupProperty: Int { 5 }
    @inline(__always) static var statePackedPointerSize: Int { 6 }
    @inline(__always) static var stateScratchCount: Int { 7 }
    @inline(__always) static var stateMaximumOrigin: Int { 8 }
    @inline(__always) static var stateRollbackScope: Int { 9 }
    @inline(__always) static var stateMaximumLayerForRollback: Int { 10 }
    @inline(__always) static var stateRuleRollbackDepth: Int { 11 }
    @inline(__always) static var statePropertyTotal: Int { 12 }
    @inline(__always) static var stateCustomNameCount: Int { 13 }
    @inline(__always) static var stateLastIndexForLogicalGroup: Int { 14 }
    @inline(__always) static var stateSeenLogicalGroupPropertyCount: Int { 15 }
    @inline(__always) static var stateLowestSeenLogicalGroupProperty: Int { 16 }
    @inline(__always) static var stateHighestSeenLogicalGroupProperty: Int { 17 }
    @inline(__always) static var stateOverriddenCount: Int { 18 }
    @inline(__always) static var stateCount: Int { 19 }

    // PropertyCascade::PropertyType.
    @inline(__always) static var typeNonInherited: UInt32 { 1 << 0 }
    @inline(__always) static var typeInherited: UInt32 { 1 << 1 }
    @inline(__always) static var typeExplicitlyInherited: UInt32 { 1 << 2 }
    @inline(__always) static var typeAfterAnimation: UInt32 { 1 << 3 }
    @inline(__always) static var typeStartingStyle: UInt32 { 1 << 4 }
    @inline(__always) static var typeNonCacheable: UInt32 { 1 << 5 }

    // PropertyCascade::Origin.
    @inline(__always) static var originAuthor: Int { 2 }
    @inline(__always) static var originPositionFallback: Int { 3 }

    // CSSPropertyID::Custom; SelectorChecker::MatchAll; IsCacheable::Partially.
    @inline(__always) static var customPropertyID: Int { 1 }
    @inline(__always) static var matchAll: Int { 3 }
    @inline(__always) static var isCacheablePartially: UInt64 { 1 }

    // PropertyCascade::Property: id u16 @0, origin u8 @2, styleScopeOrdinal i8 @3,
    // cascadeLayerPriority u16 @4, fromStyleAttribute u8 @6, cssValue[3] @8, origins[3] @32; 40 bytes.
    @inline(__always) static var propertyStride: Int { 40 }
    @inline(__always) static var propertyCSSValueOffset: Int { 8 }
    @inline(__always) static var propertyOriginsOffset: Int { 32 }

    // StylePropertyMetadata: m_propertyID bits 0-9, m_important bit 13.
    @inline(__always) static var metadataIDMask: UInt32 { 0x3FF }
    @inline(__always) static var metadataImportantBit: UInt32 { 1 << 13 }
    // Table entry: cascadeAliasProperty(id) | CSSProperty::isInheritedProperty(id) << 15 | allowed by allowlist k << (16 + k).
    @inline(__always) static var tableAliasMask: UInt32 { 0x7FFF }
    @inline(__always) static var tableInheritedBit: UInt32 { 0x8000 }
    @inline(__always) static var tableAllowlistShift: UInt32 { 16 }
    @inline(__always) static var allowlistCount: UInt64 { 8 }
    // Input entry: custom-property name id (bits 0-31), shouldApplyAfterAnimation (bit 32), explicitlyInheritsFromParent (bit 33).
    @inline(__always) static var inputNameMask: UInt64 { 0xFFFF_FFFF }
    @inline(__always) static var inputAfterAnimationBit: UInt64 { 1 << 32 }
    @inline(__always) static var inputExplicitlyInheritsBit: UInt64 { 1 << 33 }

    // Matches: 3 words each. Word 1 is the MatchedProperties scalars in their byte order: linkMatchType,
    // allowlistType, styleScopeOrdinal, fromStyleAttribute, cascadeLayerPriority (2 bytes), isStartingStyle,
    // isCacheable. Word 2 is the property count (bits 0-31) | isMutable << 63. (Word 0 is for the boundary.)
    @inline(__always) static var matchWordCount: Int { 3 }

    /// Builds the cascade. Returns the number of custom-property sets recorded at work[S...] (S =
    /// state[stateScratchCount]), in application order, each `match << 32 | propertyIndex`; the overridden
    /// animated properties (state[stateOverriddenCount] of them) follow at work[S + T...] (T = state[statePropertyTotal]).
    static func build<Storage: PropertyCascadeMatchStorage>(
        storage: Storage,
        matches: Span<UInt64>,
        state: inout MutableSpan<UInt32>,
        includedIDs: Span<UInt16>,
        table: Span<UInt32>,
        inputs: Span<UInt64>,
        records: consuming MutableRawSpan,
        present: consuming MutableSpan<UInt64>,
        logicalIndices: consuming MutableSpan<UInt32>,
        logicalIDs: inout MutableSpan<UInt16>,
        work: consuming MutableSpan<UInt64>) -> Int
    {
        precondition(state.count == stateCount)
        let userAgentCount = Int(state[stateUserAgentCount])
        let userCount = Int(state[stateUserCount])
        let authorCount = Int(state[stateAuthorCount])
        let positionTryCount = Int(state[statePositionTryCount])
        let matchCount = userAgentCount + userCount + authorCount + positionTryCount
        precondition(positionTryCount <= 1 && matchWordCount * matchCount == matches.count)
        let includedTypes = state[stateIncludedTypes]
        let propertyTotal = Int(state[statePropertyTotal])
        let customNameCount = Int(state[stateCustomNameCount])
        let ruleRollbackDepth = UInt64(state[stateRuleRollbackDepth])
        let scratchCount = Int(state[stateScratchCount])
        let packedPointerSize = Int(state[statePackedPointerSize])
        precondition(packedPointerSize >= 1 && packedPointerSize <= 8)
        precondition(logicalIndices.count <= scratchCount && logicalIDs.count == logicalIndices.count)
        precondition(userAgentCount <= scratchCount && userCount <= scratchCount && authorCount <= scratchCount)
        precondition(inputs.isEmpty || inputs.count == propertyTotal)

        // The work regions, in the glue's order (PropertyCascadeSwiftBoundary.h). (Wrapping arithmetic: every count
        // here is below 2^32, and the bases only index the bounds-checked `work`.)
        var next = scratchCount
        let customBase = next
        next &+= propertyTotal
        let overriddenBase = next
        if includedTypes & typeAfterAnimation != 0 {
            next &+= propertyTotal
        }
        let offsetBase = next
        let namePresentBase = next &+ matchCount
        if !inputs.isEmpty {
            next &+= matchCount &+ (customNameCount &+ 63) / 64
        }
        let rollbackKeyBase = next
        let rollbackKeyCount = table.count &+ customNameCount
        if ruleRollbackDepth != 0 {
            next &+= 2 &* rollbackKeyCount
        }
        let rollbackEntryBase = next
        if ruleRollbackDepth != 0 {
            next &+= 2 &* propertyTotal
        }
        precondition(next >= 0 && next <= work.count)

        // Zero the regions read before they are written; the property offsets of each match when there are inputs.
        if !inputs.isEmpty {
            var offset = 0
            for match in 0..<matchCount {
                work[offsetBase + match] = UInt64(offset)
                offset += PropertyCascadeMatch(matches, match).propertyCount
            }
            precondition(offset == propertyTotal)
            for index in namePresentBase..<(namePresentBase + (customNameCount + 63) / 64) {
                work[index] = 0
            }
        }
        if ruleRollbackDepth != 0 {
            for index in rollbackKeyBase..<(rollbackKeyBase + 2 * rollbackKeyCount) {
                work[index] = 0
            }
        }

        let rollbackScope = state[stateRollbackScope]
        let maximumLayer = state[stateMaximumLayerForRollback]
        let firstLogicalGroupProperty = Int(state[stateFirstLogicalGroupProperty])
        // Whether shouldIncludeProperty can include a property after the all-normal, ids and partially cacheable checks.
        // It cannot when no type is included that those later checks test (nor NonCacheable, which could set a property
        // mayOverrideExistingProperty would then find) and the ids hold no logical-group or custom property: by
        // induction only the ids' own properties are ever set, and the ids check already includes those.
        var reachesOtherChecks = includedTypes & (typeNonInherited | typeInherited | typeExplicitlyInherited | typeAfterAnimation | typeNonCacheable) != 0
        for index in includedIDs.indices where Int(includedIDs[index]) >= firstLogicalGroupProperty || Int(includedIDs[index]) == customPropertyID {
            reachesOtherChecks = true
        }
        let config = PropertyCascadeConfig(includedTypes: includedTypes, firstLogicalGroupProperty: UInt32(firstLogicalGroupProperty),
            packedPointerSize: UInt32(packedPointerSize), maximumOrigin: UInt32(Int(state[stateMaximumOrigin])),
            hasRollbackScope: rollbackScope & 0x100 != 0, rollbackScope: Int8(truncatingIfNeeded: rollbackScope),
            hasMaximumLayerForRollback: maximumLayer & 0x10000 != 0, maximumLayerForRollback: UInt16(truncatingIfNeeded: maximumLayer),
            ruleRollbackDepth: UInt32(ruleRollbackDepth), customNameCount: UInt32(customNameCount), userAgentCount: UInt32(userAgentCount), userCount: UInt32(userCount),
            authorCount: UInt32(authorCount), positionTryCount: UInt32(positionTryCount), customBase: UInt32(customBase), overriddenBase: UInt32(overriddenBase),
            offsetBase: UInt32(offsetBase), namePresentBase: UInt32(namePresentBase), rollbackKeyBase: UInt32(rollbackKeyBase), rollbackEntryBase: UInt32(rollbackEntryBase),
            includesAllNormal: includedTypes & (typeNonInherited | typeInherited) == typeNonInherited | typeInherited, reachesOtherChecks: reachesOtherChecks)

        // The cascades without rule rollback, rollback limits or per-property inputs (all the hot ones) run a copy of
        // the kernel without those paths. Without inputs, AfterAnimation and ExplicitlyInherited can only be included
        // with both normal types (whose cascades never reach those checks), or with no property at all.
        if ruleRollbackDepth == 0 && !config.hasMaximumLayerForRollback && inputs.isEmpty {
            let includesAllNormal = includedTypes & (typeNonInherited | typeInherited) == typeNonInherited | typeInherited
            precondition(includedTypes & (typeAfterAnimation | typeExplicitlyInherited) == 0 || includesAllNormal || propertyTotal == 0)
            var cascade = PropertyCascadeState<Storage, PropertyCascadeCommonMode>(config: config, storage: storage, matches: matches,
                includedIDs: includedIDs, table: table, inputs: inputs, records: records, present: present, logicalIndices: logicalIndices,
                work: work, lastIndexForLogicalGroup: state[stateLastIndexForLogicalGroup],
                lowestSeenLogicalGroupProperty: Int(state[stateLowestSeenLogicalGroupProperty]),
                highestSeenLogicalGroupProperty: Int(state[stateHighestSeenLogicalGroupProperty]))
            return cascade.buildCascade(state: &state, logicalIDs: &logicalIDs)
        }
        var cascade = PropertyCascadeState<Storage, PropertyCascadeGeneralMode>(config: config, storage: storage, matches: matches,
            includedIDs: includedIDs, table: table, inputs: inputs, records: records, present: present, logicalIndices: logicalIndices,
            work: work, lastIndexForLogicalGroup: state[stateLastIndexForLogicalGroup],
            lowestSeenLogicalGroupProperty: Int(state[stateLowestSeenLogicalGroupProperty]),
            highestSeenLogicalGroupProperty: Int(state[stateHighestSeenLogicalGroupProperty]))
        return cascade.buildCascade(state: &state, logicalIDs: &logicalIDs)
    }
}

// The scalars of one match (words 1 and 2), decoded on use.
struct PropertyCascadeMatch {
    let scalars: UInt64
    let storage: UInt64

    @inline(__always)
    init(_ matches: Span<UInt64>, _ match: Int) {
        scalars = matches[PropertyCascade.matchWordCount * match + 1]
        storage = matches[PropertyCascade.matchWordCount * match + 2]
    }

    @inline(__always) var linkMatchType: Int { Int(scalars & 0xFF) }
    @inline(__always) var allowlistType: UInt64 { (scalars >> 8) & 0xFF }
    @inline(__always) var styleScopeOrdinal: UInt8 { UInt8(truncatingIfNeeded: scalars >> 16) } // int8_t bit pattern
    @inline(__always) var fromStyleAttribute: UInt64 { (scalars >> 24) & 0xFF }
    @inline(__always) var cascadeLayerPriority: UInt16 { UInt16(truncatingIfNeeded: scalars >> 32) }
    @inline(__always) var isStartingStyle: Bool { (scalars >> 48) & 0xFF != 0 }
    @inline(__always) var isCacheable: UInt64 { scalars >> 56 }
    @inline(__always) var isMutable: Bool { storage >> 63 != 0 }
    @inline(__always) var propertyCount: Int { Int(storage & 0xFFFF_FFFF) }

    // PropertyCascade::Property bytes 0-7: id, origin, styleScopeOrdinal, cascadeLayerPriority,
    // fromStyleAttribute (byte 7 is padding).
    @inline(__always)
    func propertyHeader(_ propertyID: Int, origin: Int) -> UInt64 {
        UInt64(truncatingIfNeeded: propertyID) & 0xFFFF | UInt64(origin) << 16 | UInt64(styleScopeOrdinal) << 24
            | UInt64(cascadeLayerPriority) << 32 | fromStyleAttribute << 48
    }
}

// Where the metadata and the value pointer of property `index` of `count` are in the storage bytes.
protocol PropertyCascadeStorageLayout {
    static func byteCount(_ count: Int, packedPointerSize: Int) -> Int
    static func metadataOffset(_ index: Int) -> Int
    static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int
    static func valueSize(packedPointerSize: Int) -> Int
}

// ImmutableStyleProperties: StylePropertyMetadata[count], then PackedPtr<const CSSValue>[count].
enum PropertyCascadeImmutableStorage: PropertyCascadeStorageLayout {
    @inline(__always) static func byteCount(_ count: Int, packedPointerSize: Int) -> Int { (4 + packedPointerSize) * count }
    @inline(__always) static func metadataOffset(_ index: Int) -> Int { 4 * index }
    @inline(__always) static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int { 4 * count + packedPointerSize * index }
    @inline(__always) static func valueSize(packedPointerSize: Int) -> Int { packedPointerSize }
}

// MutableStyleProperties: CSSProperty[count] (StylePropertyMetadata @0, Ref<CSSValue> @8; 16 bytes).
enum PropertyCascadeMutableStorage: PropertyCascadeStorageLayout {
    @inline(__always) static func byteCount(_ count: Int, packedPointerSize: Int) -> Int { 16 * count }
    @inline(__always) static func metadataOffset(_ index: Int) -> Int { 16 * index }
    @inline(__always) static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int { 16 * index + 8 }
    @inline(__always) static func valueSize(packedPointerSize: Int) -> Int { 8 }
}

// Which copy of the kernel runs: the general one handles every cascade, the common one only those without rule
// rollback, rollback limits and per-property inputs, and is compiled without their paths.
protocol PropertyCascadeMode {
    static var isGeneral: Bool { get }
}

enum PropertyCascadeCommonMode: PropertyCascadeMode {
    @inline(__always) static var isGeneral: Bool { false }
}

enum PropertyCascadeGeneralMode: PropertyCascadeMode {
    @inline(__always) static var isGeneral: Bool { true }
}

// The cascade's configuration, from the state words, and where the work regions are.
struct PropertyCascadeConfig {
    let includedTypes: UInt32
    let firstLogicalGroupProperty: UInt32
    let packedPointerSize: UInt32
    let maximumOrigin: UInt32
    let hasRollbackScope: Bool
    let rollbackScope: Int8
    let hasMaximumLayerForRollback: Bool
    let maximumLayerForRollback: UInt16
    let ruleRollbackDepth: UInt32
    let customNameCount: UInt32
    let userAgentCount: UInt32
    let userCount: UInt32
    let authorCount: UInt32
    let positionTryCount: UInt32
    let customBase: UInt32          // the custom-property sets (match << 32 | index)
    let overriddenBase: UInt32      // the AfterAnimation overrides (property ids)
    let offsetBase: UInt32          // per match, the index of its first property in `inputs`
    let namePresentBase: UInt32     // bitmap over custom-property name ids: hasCustomProperty(name)
    let rollbackKeyBase: UInt32     // per delay key: head + 1 | (tail + 1) << 32, then the undelivered count
    let rollbackEntryBase: UInt32   // per delayed property: match << 32 | index, then next + 1 | input index << 32
    let includesAllNormal: Bool
    let reachesOtherChecks: Bool    // see build()
}

// The state of one PropertyCascade under construction, and its read-only inputs.
struct PropertyCascadeState<Storage: PropertyCascadeMatchStorage, Mode: PropertyCascadeMode>: ~Copyable, ~Escapable {
    let config: PropertyCascadeConfig
    let storage: Storage
    let matches: Span<UInt64>
    let includedIDs: Span<UInt16>
    let table: Span<UInt32>
    let inputs: Span<UInt64>
    var records: MutableRawSpan              // m_properties
    var present: MutableSpan<UInt64>         // m_propertyIsPresent
    var logicalIndices: MutableSpan<UInt32>  // m_logicalGroupPropertyIndices
    var work: MutableSpan<UInt64>            // sort scratch, then the regions at the config's bases
    var lastIndexForLogicalGroup: UInt32
    var lowestSeenLogicalGroupProperty: Int
    var highestSeenLogicalGroupProperty: Int
    var customCount: Int = 0
    var overriddenCount: Int = 0
    var delayedCount: Int = 0

    @_lifetime(copy matches, copy includedIDs, copy table, copy inputs, copy records, copy present, copy logicalIndices, copy work)
    init(config: PropertyCascadeConfig, storage: Storage, matches: Span<UInt64>, includedIDs: Span<UInt16>, table: Span<UInt32>,
        inputs: Span<UInt64>, records: consuming MutableRawSpan, present: consuming MutableSpan<UInt64>,
        logicalIndices: consuming MutableSpan<UInt32>, work: consuming MutableSpan<UInt64>, lastIndexForLogicalGroup: UInt32,
        lowestSeenLogicalGroupProperty: Int, highestSeenLogicalGroupProperty: Int)
    {
        self.config = config
        self.storage = storage
        self.matches = matches
        self.includedIDs = includedIDs
        self.table = table
        self.inputs = inputs
        self.records = records
        self.present = present
        self.logicalIndices = logicalIndices
        self.work = work
        self.lastIndexForLogicalGroup = lastIndexForLogicalGroup
        self.lowestSeenLogicalGroupProperty = lowestSeenLogicalGroupProperty
        self.highestSeenLogicalGroupProperty = highestSeenLogicalGroupProperty
    }

    @inline(__always) var includedTypes: UInt32 { config.includedTypes }
    @inline(__always) var firstLogicalGroupProperty: Int { Int(config.firstLogicalGroupProperty) }
    @inline(__always) var packedPointerSize: Int { Int(config.packedPointerSize) }
    @inline(__always) var maximumOrigin: Int { Int(config.maximumOrigin) }
    @inline(__always) var ruleRollbackDepth: UInt64 { Mode.isGeneral ? UInt64(config.ruleRollbackDepth) : 0 }
    @inline(__always) var hasMaximumLayerForRollback: Bool { Mode.isGeneral && config.hasMaximumLayerForRollback }
    @inline(__always) var hasInputs: Bool { Mode.isGeneral && !inputs.isEmpty }
    @inline(__always) var customNameCount: Int { Int(config.customNameCount) }
    // (The common copy has no AfterAnimation or ExplicitlyInherited checks: see build().)
    @inline(__always) var includesAfterAnimation: Bool { Mode.isGeneral && includes(PropertyCascade.typeAfterAnimation) }
    @inline(__always) var userAgentCount: Int { Int(config.userAgentCount) }
    @inline(__always) var userCount: Int { Int(config.userCount) }
    @inline(__always) var authorCount: Int { Int(config.authorCount) }
    @inline(__always) var customBase: Int { Int(config.customBase) }
    @inline(__always) var overriddenBase: Int { Int(config.overriddenBase) }
    @inline(__always) var offsetBase: Int { Int(config.offsetBase) }
    @inline(__always) var namePresentBase: Int { Int(config.namePresentBase) }
    @inline(__always) var rollbackKeyBase: Int { Int(config.rollbackKeyBase) }
    @inline(__always) var rollbackEntryBase: Int { Int(config.rollbackEntryBase) }

    // buildCascade: normal matches for UserAgent, User and Author up to the maximum origin, the position-try fallback,
    // then the important ones for Author, User and UserAgent (those whose normal pass found any), and
    // sortLogicalGroupPropertyIDs. Writes the logical-group scalars and the overridden count to `state`; returns the
    // custom-property set count. (One loop over the origins, so the inlined property loop exists once.)
    @_lifetime(self: copy self)
    mutating func buildCascade(state: inout MutableSpan<UInt32>, logicalIDs: inout MutableSpan<UInt16>) -> Int {
        var originsWithImportant: UInt32 = 0
        for origin in 0..<3 {
            if origin > maximumOrigin {
                break
            }
            if addNormalMatches(matchRange(origin: origin), origin: origin) {
                originsWithImportant |= 1 << UInt32(origin)
            }
        }
        if config.positionTryCount != 0 && maximumOrigin >= PropertyCascade.originPositionFallback {
            _ = addMatch(userAgentCount + userCount + authorCount, origin: PropertyCascade.originPositionFallback, important: false)
        }
        var origin = PropertyCascade.originAuthor
        while origin >= 0 {
            if originsWithImportant & (1 << UInt32(origin)) != 0 {
                addImportantMatches(matchRange(origin: origin), origin: origin)
            }
            origin -= 1
        }

        let seenCount = sortLogicalGroupPropertyIDs(&logicalIDs)

        state[PropertyCascade.stateLastIndexForLogicalGroup] = lastIndexForLogicalGroup
        state[PropertyCascade.stateSeenLogicalGroupPropertyCount] = UInt32(seenCount)
        state[PropertyCascade.stateLowestSeenLogicalGroupProperty] = UInt32(lowestSeenLogicalGroupProperty)
        state[PropertyCascade.stateHighestSeenLogicalGroupProperty] = UInt32(highestSeenLogicalGroupProperty)
        state[PropertyCascade.stateOverriddenCount] = UInt32(overriddenCount)
        return customCount
    }

    @inline(__always)
    func includes(_ type: UInt32) -> Bool { includedTypes & type != 0 }

    // The matches of origin UserAgent (0), User (1) or Author (2). (The small non-mutating helpers are inlined:
    // an out-of-line call would pass the whole state by a stack copy.)
    @inline(__always)
    func matchRange(origin: Int) -> Range<Int> {
        if origin == 0 {
            return 0..<userAgentCount
        }
        if origin == 1 {
            return userAgentCount..<(userAgentCount + userCount)
        }
        return (userAgentCount + userCount)..<(userAgentCount + userCount + authorCount)
    }

    // The origin addMatch was called with for `match`.
    @inline(__always)
    func origin(ofMatch match: Int) -> Int {
        if match < userAgentCount {
            return 0
        }
        if match < userAgentCount + userCount {
            return 1
        }
        if match < userAgentCount + userCount + authorCount {
            return PropertyCascade.originAuthor
        }
        return PropertyCascade.originPositionFallback
    }

    // addNormalMatches. Returns whether any match has important properties. Inlined into build() so the
    // cascade's scalars stay in registers across the property loop (guaranteed inlining, as for addMatch).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func addNormalMatches(_ range: Range<Int>, origin: Int) -> Bool {
        var hasImportant = false
        for match in range {
            let matchHasImportant = addMatch(match, origin: origin, important: false)
            hasImportant = hasImportant || matchHasImportant
        }
        return hasImportant
    }

    // addImportantMatches.
    @_lifetime(self: copy self)
    mutating func addImportantMatches(_ range: Range<Int>, origin: Int) {
        var count = 0
        var hasMatchesFromOtherScopesOrLayers = false
        for match in range {
            let matched = PropertyCascadeMatch(matches, match)
            if !hasImportantProperties(match, matched) {
                continue
            }
            // std::ranges::stable_sort key: ordinal ascending (signed), fromStyleAttribute No first, layer
            // priority descending; the match index makes every key distinct, so any sort is stable.
            let key = UInt64(matched.styleScopeOrdinal ^ 0x80) << 56 | (matched.fromStyleAttribute & 0xFF) << 48
                | UInt64(0xFFFF - matched.cascadeLayerPriority) << 32 | UInt64(UInt32(truncatingIfNeeded: match))
            work[count] = key
            count += 1
            if matched.styleScopeOrdinal != 0 || matched.cascadeLayerPriority != 0xFFFF {
                hasMatchesFromOtherScopesOrLayers = true
            }
        }
        if count == 0 {
            return
        }
        if hasMatchesFromOtherScopesOrLayers {
            PropertyCascadeState.sortDistinct(&work, count: count)
        }
        for index in 0..<count {
            let match = Int(UInt32(truncatingIfNeeded: work[index]))
            _ = addMatch(match, origin: origin, important: true)
        }
    }

    // hasImportantProperties(const StyleProperties&): every property, whatever the match's other scalars.
    @inline(__always)
    func hasImportantProperties(_ match: Int, _ matched: PropertyCascadeMatch) -> Bool {
        let storage = self.storage
        let count = matched.propertyCount
        if matched.isMutable {
            let bytes = storage.bytes(match, byteCount: PropertyCascadeMutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
            for index in 0..<count where bytes.load(fromByteOffset: PropertyCascadeMutableStorage.metadataOffset(index), as: UInt32.self) & PropertyCascade.metadataImportantBit != 0 {
                return true
            }
            return false
        }
        let bytes = storage.bytes(match, byteCount: PropertyCascadeImmutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
        for index in 0..<count where bytes.load(fromByteOffset: PropertyCascadeImmutableStorage.metadataOffset(index), as: UInt32.self) & PropertyCascade.metadataImportantBit != 0 {
            return true
        }
        return false
    }

    // addMatch's includePropertiesForRollback.
    @inline(__always)
    func includePropertiesForRollback(_ matched: PropertyCascadeMatch, origin: Int) -> Bool {
        if config.hasRollbackScope && Int8(bitPattern: matched.styleScopeOrdinal) > config.rollbackScope {
            return true
        }
        if origin < maximumOrigin {
            return true
        }
        if matched.fromStyleAttribute == 1 {
            return false
        }
        return matched.cascadeLayerPriority <= config.maximumLayerForRollback
    }

    // addMatch. Returns hasImportantProperties.
    // Guaranteed inlining (SE-0496) keeps both storage layouts' loops specialized and inlined; with
    // @inline(__always) the immutable one stays an out-of-line call per match (measured ~3-5% slower).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func addMatch(_ match: Int, origin: Int, important: Bool) -> Bool {
        let matched = PropertyCascadeMatch(matches, match)
        if hasMaximumLayerForRollback && !includePropertiesForRollback(matched, origin: origin) {
            return false
        }
        if matched.isStartingStyle && !includes(PropertyCascade.typeStartingStyle) {
            return false
        }
        let storage = self.storage
        let count = matched.propertyCount
        if matched.isMutable {
            let bytes = storage.bytes(match, byteCount: PropertyCascadeMutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
            return addProperties(PropertyCascadeMutableStorage.self, bytes, match: match, matched: matched, origin: origin, important: important)
        }
        let bytes = storage.bytes(match, byteCount: PropertyCascadeImmutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
        return addProperties(PropertyCascadeImmutableStorage.self, bytes, match: match, matched: matched, origin: origin, important: important)
    }

    // The property loop of addMatch.
    // Guaranteed inlining (SE-0496) keeps both storage layouts' loops specialized and inlined; with
    // @inline(__always) the immutable one stays an out-of-line call per match (measured ~3-5% slower).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func addProperties<Layout: PropertyCascadeStorageLayout>(_ layout: Layout.Type, _ bytes: RawSpan, match: Int, matched: PropertyCascadeMatch,
        origin: Int, important: Bool) -> Bool
    {
        let count = matched.propertyCount
        let includesAllNormal = config.includesAllNormal
        let reachesOtherChecks = config.reachesOtherChecks
        let includesPartiallyCacheable = matched.isCacheable == PropertyCascade.isCacheablePartially && includes(PropertyCascade.typeNonCacheable)
        // The allowlist bit of the table entry; an allowlistType no allowlist check knows admits every property.
        let allowlist = matched.allowlistType < PropertyCascade.allowlistCount ? UInt32(matched.allowlistType) : 0
        let allowedBit: UInt32 = 1 << (PropertyCascade.tableAllowlistShift + allowlist)
        let inputBase = hasInputs ? Int(work[offsetBase + match]) : 0
        var hasImportantProperties = false

        for index in 0..<count {
            let metadata = bytes.load(fromByteOffset: Layout.metadataOffset(index), as: UInt32.self)
            let isImportant = metadata & PropertyCascade.metadataImportantBit != 0
            if isImportant {
                hasImportantProperties = true
            }
            if isImportant != important {
                continue
            }

            let entry = table[Int(metadata & PropertyCascade.metadataIDMask)]
            if entry & allowedBit == 0 {
                continue
            }
            let propertyID = Int(entry & PropertyCascade.tableAliasMask)

            // shouldIncludeProperty.
            if !(includesAllNormal || PropertyCascadeState.contains(includedIDs, propertyID) || includesPartiallyCacheable) {
                if !reachesOtherChecks || !shouldIncludeByTypeOrOverride(propertyID, entry: entry, input: inputBase + index) {
                    continue
                }
            }

            let reference = UInt64(UInt32(truncatingIfNeeded: match)) << 32 | UInt64(UInt32(truncatingIfNeeded: index))
            if ruleRollbackDepth != 0 {
                setDelayingForRuleRollback(propertyID, reference: reference, input: inputBase + index)
                continue
            }
            let value = PropertyCascadeState.loadPointer(bytes, Layout.valueOffset(index, count: count, packedPointerSize: packedPointerSize),
                size: Layout.valueSize(packedPointerSize: packedPointerSize))
            set(propertyID, value: value, reference: reference, input: inputBase + index, matched: matched, origin: origin)
        }
        return hasImportantProperties
    }

    // The rest of shouldIncludeProperty, after the allowlist, all-normal, ids and partially cacheable checks.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func shouldIncludeByTypeOrOverride(_ propertyID: Int, entry: UInt32, input: Int) -> Bool {
        let currentIsInherited = entry & PropertyCascade.tableInheritedBit != 0
        // mayOverrideExistingProperty.
        if propertyID == PropertyCascade.customPropertyID {
            if !includesAfterAnimation {
                // Without AfterAnimation the checks below have no side effects, so they can go first; they include
                // the custom property without its name (hasCustomProperty) whenever its type is included.
                if includedByType(currentIsInherited, input: input) {
                    return true
                }
                return hasCustomProperty(input: input)
            }
            if hasCustomProperty(input: input) {
                return true
            }
        } else if propertyID < firstLogicalGroupProperty {
            if present[propertyID >> 6] & (1 << UInt64(propertyID & 63)) != 0 {
                return true
            }
        } else if lastIndexForLogicalGroup != 0 {
            // Apply all logical group properties if we have applied any. They may override the ones we already applied.
            return true
        }

        if includesAfterAnimation {
            if inputs[input] & PropertyCascade.inputAfterAnimationBit != 0 {
                work[overriddenBase + overriddenCount] = UInt64(propertyID)
                overriddenCount += 1
                return true
            }
            return false
        }
        return includedByType(currentIsInherited, input: input)
    }

    // The Inherited, ExplicitlyInherited and NonInherited checks of shouldIncludeProperty.
    @inline(__always)
    func includedByType(_ currentIsInherited: Bool, input: Int) -> Bool {
        if includes(PropertyCascade.typeInherited) && currentIsInherited {
            return true
        }
        if Mode.isGeneral && includes(PropertyCascade.typeExplicitlyInherited) && inputs[input] & PropertyCascade.inputExplicitlyInheritsBit != 0 {
            return true
        }
        return includes(PropertyCascade.typeNonInherited) && !currentIsInherited
    }

    // hasCustomProperty(name): m_customProperties is empty until a custom property has been set.
    @inline(__always)
    func hasCustomProperty(input: Int) -> Bool {
        if customCount == 0 {
            return false
        }
        let name = customName(input: input)
        return work[namePresentBase + (name >> 6)] & (1 << UInt64(name & 63)) != 0
    }

    // The custom-property name id of input `input` (the glue gathers them for every cascade that can need one).
    @inline(__always)
    func customName(input: Int) -> Int {
        let name = Int(inputs[input] & PropertyCascade.inputNameMask)
        precondition(name < customNameCount)
        return name
    }

    @inline(__always)
    static func contains(_ ids: Span<UInt16>, _ propertyID: Int) -> Bool {
        for index in ids.indices where Int(ids[index]) == propertyID {
            return true
        }
        return false
    }

    // PackedPtr<const CSSValue>::get() (little-endian, zero-extended) or the Ref<CSSValue> pointer.
    @inline(__always)
    static func loadPointer(_ bytes: RawSpan, _ offset: Int, size: Int) -> UInt64 {
        if size == 8 {
            return bytes.load(fromByteOffset: offset, as: UInt64.self)
        }
        if size == 6 {
            return UInt64(bytes.load(fromByteOffset: offset, as: UInt32.self)) | UInt64(bytes.load(fromByteOffset: offset + 4, as: UInt16.self)) << 32
        }
        var value: UInt64 = 0
        for byte in 0..<size {
            value |= UInt64(bytes.load(fromByteOffset: offset + byte, as: UInt8.self)) << UInt64(8 * byte)
        }
        return value
    }

    // set. Custom properties go to the glue's list (m_customProperties is keyed by name) and set presence bit 1.
    @inline(__always)
    @_lifetime(self: copy self)
    mutating func set(_ propertyID: Int, value: UInt64, reference: UInt64, input: Int, matched: PropertyCascadeMatch, origin: Int) {
        if propertyID >= firstLogicalGroupProperty {
            setLogicalGroupProperty(propertyID, value: value, matched: matched, origin: origin)
            return
        }
        let word = propertyID >> 6
        let bit: UInt64 = 1 << UInt64(propertyID & 63)
        let wasPresent = present[word] & bit != 0
        present[word] |= bit
        if propertyID == PropertyCascade.customPropertyID {
            setCustomProperty(reference: reference, input: input)
            return
        }
        setPropertyInternal(propertyID, value: value, matched: matched, origin: origin, clearValues: !wasPresent)
    }

    @_lifetime(self: copy self)
    mutating func setCustomProperty(reference: UInt64, input: Int) {
        work[customBase + customCount] = reference
        customCount += 1
        if Mode.isGeneral && customNameCount != 0 {
            // The name ids were gathered: hasCustomProperty(name) from now on.
            let name = customName(input: input)
            work[namePresentBase + (name >> 6)] |= 1 << UInt64(name & 63)
        }
    }

    // setDelayingForRuleRollback: per (property, custom name) key, the last ruleRollbackDepth values are dropped;
    // each value is set once ruleRollbackDepth newer ones have arrived.
    @_lifetime(self: copy self)
    mutating func setDelayingForRuleRollback(_ propertyID: Int, reference: UInt64, input: Int) {
        let key = propertyID == PropertyCascade.customPropertyID ? table.count + customName(input: input) : propertyID
        let keySlot = rollbackKeyBase + 2 * key
        let entry = delayedCount
        delayedCount += 1
        work[rollbackEntryBase + 2 * entry] = reference
        work[rollbackEntryBase + 2 * entry + 1] = UInt64(UInt32(truncatingIfNeeded: input)) << 32
        let links = work[keySlot]
        let tail = Int(links >> 32)
        var head = Int(links & 0xFFFF_FFFF)
        if tail == 0 {
            head = entry + 1
        } else {
            work[rollbackEntryBase + 2 * (tail - 1) + 1] |= UInt64(entry + 1)
        }
        let count = work[keySlot + 1] + 1
        if count <= ruleRollbackDepth {
            work[keySlot] = UInt64(head) | UInt64(entry + 1) << 32
            work[keySlot + 1] = count
            return
        }
        // Deliver the oldest one (Deque::takeLast after prepend).
        let oldest = head - 1
        let oldestLinks = work[rollbackEntryBase + 2 * oldest + 1]
        work[keySlot] = (oldestLinks & 0xFFFF_FFFF) | UInt64(entry + 1) << 32
        work[keySlot + 1] = count - 1
        let oldestReference = work[rollbackEntryBase + 2 * oldest]
        setFromReference(propertyID, reference: oldestReference, input: Int(oldestLinks >> 32))
    }

    // set() for a delayed property, from its match and index.
    @_lifetime(self: copy self)
    mutating func setFromReference(_ propertyID: Int, reference: UInt64, input: Int) {
        let match = Int(reference >> 32)
        let index = Int(UInt32(truncatingIfNeeded: reference))
        let matched = PropertyCascadeMatch(matches, match)
        let count = matched.propertyCount
        let storage = self.storage
        let value: UInt64
        if matched.isMutable {
            let bytes = storage.bytes(match, byteCount: PropertyCascadeMutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
            value = PropertyCascadeState.loadPointer(bytes, PropertyCascadeMutableStorage.valueOffset(index, count: count, packedPointerSize: packedPointerSize),
                size: PropertyCascadeMutableStorage.valueSize(packedPointerSize: packedPointerSize))
        } else {
            let bytes = storage.bytes(match, byteCount: PropertyCascadeImmutableStorage.byteCount(count, packedPointerSize: packedPointerSize))
            value = PropertyCascadeState.loadPointer(bytes, PropertyCascadeImmutableStorage.valueOffset(index, count: count, packedPointerSize: packedPointerSize),
                size: PropertyCascadeImmutableStorage.valueSize(packedPointerSize: packedPointerSize))
        }
        set(propertyID, value: value, reference: reference, input: input, matched: matched, origin: origin(ofMatch: match))
    }

    @inline(__always)
    @_lifetime(self: copy self)
    mutating func setLogicalGroupProperty(_ propertyID: Int, value: UInt64, matched: PropertyCascadeMatch, origin: Int) {
        let logicalIndex = propertyID - firstLogicalGroupProperty
        let isFirst = logicalIndices[logicalIndex] == 0
        if isFirst {
            lowestSeenLogicalGroupProperty = min(lowestSeenLogicalGroupProperty, propertyID)
            highestSeenLogicalGroupProperty = max(highestSeenLogicalGroupProperty, propertyID)
        }
        lastIndexForLogicalGroup &+= 1
        logicalIndices[logicalIndex] = lastIndexForLogicalGroup
        setPropertyInternal(propertyID, value: value, matched: matched, origin: origin, clearValues: isFirst)
    }

    // setPropertyInternal, preceded by `property.cssValue = { }` when `clearValues`.
    @inline(__always)
    @_lifetime(self: copy self)
    mutating func setPropertyInternal(_ propertyID: Int, value: UInt64, matched: PropertyCascadeMatch, origin: Int, clearValues: Bool) {
        let start = propertyID * PropertyCascade.propertyStride
        precondition(start >= 0 && start + PropertyCascade.propertyStride <= records.byteCount)
        records.storeBytes(of: matched.propertyHeader(propertyID, origin: origin), toByteOffset: start, as: UInt64.self)
        let values = start + PropertyCascade.propertyCSSValueOffset
        let origins = start + PropertyCascade.propertyOriginsOffset
        let link = matched.linkMatchType
        if link == PropertyCascade.matchAll {
            records.storeBytes(of: value, toByteOffset: values, as: UInt64.self)
            records.storeBytes(of: value, toByteOffset: values + 8, as: UInt64.self)
            records.storeBytes(of: value, toByteOffset: values + 16, as: UInt64.self)
            records.storeBytes(of: UInt8(origin), toByteOffset: origins, as: UInt8.self)
            records.storeBytes(of: UInt8(origin), toByteOffset: origins + 1, as: UInt8.self)
            records.storeBytes(of: UInt8(origin), toByteOffset: origins + 2, as: UInt8.self)
            return
        }
        precondition(link < 3)
        if clearValues {
            records.storeBytes(of: 0, toByteOffset: values, as: UInt64.self)
            records.storeBytes(of: 0, toByteOffset: values + 8, as: UInt64.self)
            records.storeBytes(of: 0, toByteOffset: values + 16, as: UInt64.self)
        }
        records.storeBytes(of: value, toByteOffset: values + 8 * link, as: UInt64.self)
        records.storeBytes(of: UInt8(origin), toByteOffset: origins + link, as: UInt8.self)
    }

    // sortLogicalGroupPropertyIDs. Returns m_seenLogicalGroupPropertyCount.
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func sortLogicalGroupPropertyIDs(_ logicalIDs: inout MutableSpan<UInt16>) -> Int {
        var endIndex = 0
        if lowestSeenLogicalGroupProperty <= highestSeenLogicalGroupProperty {
            for propertyID in lowestSeenLogicalGroupProperty...highestSeenLogicalGroupProperty {
                let index = logicalIndices[propertyID - firstLogicalGroupProperty]
                if index != 0 {
                    // Indices are distinct, so sorting (index, id) pairs equals std::ranges::sort by index.
                    work[endIndex] = UInt64(index) << 16 | UInt64(propertyID & 0xFFFF)
                    endIndex += 1
                }
            }
        }
        PropertyCascadeState.sortDistinct(&work, count: endIndex)
        for index in 0..<endIndex {
            logicalIDs[index] = UInt16(truncatingIfNeeded: work[index])
        }
        return endIndex
    }

    // Sorts work[0..<count] ascending. All keys are distinct, so the result is the unique sorted order.
    static func sortDistinct(_ work: inout MutableSpan<UInt64>, count: Int) {
        precondition(count <= work.count)
        if count <= 32 {
            var index = 1
            while index < count {
                let key = work[index]
                var hole = index
                while hole > 0 && work[hole - 1] > key {
                    work[hole] = work[hole - 1]
                    hole -= 1
                }
                work[hole] = key
                index += 1
            }
            return
        }
        // Heapsort: O(n log n) with no scratch.
        var start = count / 2
        while start > 0 {
            start -= 1
            siftDown(&work, start, count)
        }
        var end = count
        while end > 1 {
            end -= 1
            work.swapAt(0, end)
            siftDown(&work, 0, end)
        }
    }

    static func siftDown(_ work: inout MutableSpan<UInt64>, _ start: Int, _ end: Int) {
        var root = start
        while true {
            var child = 2 * root + 1
            if child >= end {
                return
            }
            if child + 1 < end && work[child] < work[child + 1] {
                child += 1
            }
            if work[root] >= work[child] {
                return
            }
            work.swapAt(root, child)
            root = child
        }
    }
}

#endif // compiler(>=6.4) || WTF_PLATFORM_MAC
