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

// Memory-safe port of Style::PropertyCascade::buildCascade (style/PropertyCascade.cpp) for the cascades
// the C++ glue routes here (no rollback, no position-try fallback, no AfterAnimation, no allowlist, and
// ExplicitlyInherited only together with both normal types). One call builds the whole cascade:
// buildCascade, addNormalMatches, addMatch (+ cascadeAliasProperty and CSSProperty::isInheritedProperty
// through a table the glue builds from the generated functions), set, setLogicalGroupProperty,
// setPropertyInternal, mayOverrideExistingProperty, hasImportantProperties, addImportantMatches (with its
// stable sort) and sortLogicalGroupPropertyIDs. The C++ names are kept.
//
// Sets of CSSPropertyID::Custom are not applied here: they only touch m_customProperties and presence bit
// 1, which nothing below reads, so they are recorded in application order and the glue replays them with
// the original set(). Whether a custom property is included can depend on m_customProperties
// (mayOverrideExistingProperty -> hasCustomProperty); while no custom property has been recorded the map
// is still empty, so that is exact. The one undecidable case returns `bail` (see build()).
//
// The memory layouts this code reads and writes are documented, and static_asserted on the C++ side, in
// PropertyCascadeSwiftBoundary.h. No allocation, no reference counting, no callbacks.

#if compiler(>=6.4) || WTF_PLATFORM_MAC

/// The property storage of each matched declaration block, provided by the boundary.
protocol PropertyCascadeMatchStorage {
    /// The storage bytes of match `match` (immutable: metadata[n] then packed pointers[n]; mutable: CSSProperty[n]).
    @_lifetime(borrow self)
    func bytes(_ match: Int) -> RawSpan
}

enum PropertyCascade {
    // Indices into the `config` span.
    @inline(__always) static var configUserAgentCount: Int { 0 }
    @inline(__always) static var configUserCount: Int { 1 }
    @inline(__always) static var configAuthorCount: Int { 2 }
    @inline(__always) static var configIncludedTypes: Int { 3 }
    @inline(__always) static var configFirstLogicalGroupProperty: Int { 4 }
    @inline(__always) static var configPackedPointerSize: Int { 5 }
    @inline(__always) static var configScratchCount: Int { 6 }
    @inline(__always) static var configCount: Int { 7 }

    // Indices into the in/out `scalars` span.
    @inline(__always) static var scalarLastIndexForLogicalGroup: Int { 0 }
    @inline(__always) static var scalarSeenLogicalGroupPropertyCount: Int { 1 }
    @inline(__always) static var scalarLowestSeenLogicalGroupProperty: Int { 2 }
    @inline(__always) static var scalarHighestSeenLogicalGroupProperty: Int { 3 }
    @inline(__always) static var scalarCount: Int { 4 }

    // PropertyCascade::PropertyType.
    @inline(__always) static var typeNonInherited: UInt32 { 1 << 0 }
    @inline(__always) static var typeInherited: UInt32 { 1 << 1 }
    @inline(__always) static var typeExplicitlyInherited: UInt32 { 1 << 2 }
    @inline(__always) static var typeAfterAnimation: UInt32 { 1 << 3 }
    @inline(__always) static var typeStartingStyle: UInt32 { 1 << 4 }
    @inline(__always) static var typeNonCacheable: UInt32 { 1 << 5 }

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
    // Table entry: cascadeAliasProperty(id) | CSSProperty::isInheritedProperty(id) << 15.
    @inline(__always) static var tableAliasMask: UInt16 { 0x7FFF }
    @inline(__always) static var tableInheritedBit: UInt16 { 0x8000 }

    // Matches: 3 words each. Word 1 is the MatchedProperties scalars in their byte order: linkMatchType,
    // allowlistType, styleScopeOrdinal, fromStyleAttribute, cascadeLayerPriority (2 bytes), isStartingStyle,
    // isCacheable. Word 2 is the storage byte count (bits 0-31) | property count << 32 (bits 32-59) |
    // isMutable << 63. (Word 0 is for the boundary.)
    @inline(__always) static var matchWordCount: Int { 3 }

    @inline(__always) static var bail: Int { -1 }

    /// Builds the cascade. Returns the number of custom-property sets recorded at the start of `work` +
    /// config[configScratchCount] (each `match << 32 | propertyIndex`), or `bail` (-1) when the glue must run
    /// the C++ instead; in that case the presence words and logical-group indices have been zeroed again
    /// (their state on entry) and nothing else C++ reads was changed.
    static func build<Storage: PropertyCascadeMatchStorage>(
        storage: Storage,
        matches: Span<UInt64>,
        config: Span<UInt32>,
        includedIDs: Span<UInt16>,
        table: Span<UInt16>,
        records: consuming MutableRawSpan,
        present: consuming MutableSpan<UInt64>,
        logicalIndices: consuming MutableSpan<UInt32>,
        logicalIDs: inout MutableSpan<UInt16>,
        scalars: inout MutableSpan<UInt32>,
        work: consuming MutableSpan<UInt64>) -> Int
    {
        // The glue routes these to the C++ (AfterAnimation needs shouldApplyAfterAnimation(), and
        // ExplicitlyInherited without both normal types needs explicitlyInheritsFromParent()).
        let includedTypes = config[configIncludedTypes]
        let includesAllNormal = includedTypes & (typeNonInherited | typeInherited) == typeNonInherited | typeInherited
        if includedTypes & typeAfterAnimation != 0 || (includedTypes & typeExplicitlyInherited != 0 && !includesAllNormal) {
            return bail
        }

        let userAgentCount = Int(config[configUserAgentCount])
        let userCount = Int(config[configUserCount])
        let authorCount = Int(config[configAuthorCount])
        precondition(matchWordCount * (userAgentCount + userCount + authorCount) <= matches.count)
        let scratchCount = Int(config[configScratchCount])
        precondition(scratchCount <= work.count && logicalIndices.count <= scratchCount && logicalIDs.count == logicalIndices.count)
        let userAgentRange = 0..<userAgentCount
        let userRange = userAgentCount..<(userAgentCount + userCount)
        let authorRange = (userAgentCount + userCount)..<(userAgentCount + userCount + authorCount)

        var cascade = PropertyCascadeState(
            storage: storage, matches: matches, includedIDs: includedIDs, table: table,
            records: records, present: present, logicalIndices: logicalIndices, work: work,
            includedTypes: includedTypes, firstLogicalGroupProperty: Int(config[configFirstLogicalGroupProperty]),
            packedPointerSize: Int(config[configPackedPointerSize]), scratchCount: scratchCount,
            lastIndexForLogicalGroup: scalars[scalarLastIndexForLogicalGroup],
            lowestSeenLogicalGroupProperty: Int(scalars[scalarLowestSeenLogicalGroupProperty]),
            highestSeenLogicalGroupProperty: Int(scalars[scalarHighestSeenLogicalGroupProperty]))

        // buildCascade: normal matches for UserAgent, User, Author; then important ones for Author, User, UserAgent.
        guard let userAgentHasImportant = cascade.addNormalMatches(userAgentRange, origin: 0) else { return cascade.bail() }
        guard let userHasImportant = cascade.addNormalMatches(userRange, origin: 1) else { return cascade.bail() }
        guard let authorHasImportant = cascade.addNormalMatches(authorRange, origin: 2) else { return cascade.bail() }
        if authorHasImportant {
            guard cascade.addImportantMatches(authorRange, origin: 2) else { return cascade.bail() }
        }
        if userHasImportant {
            guard cascade.addImportantMatches(userRange, origin: 1) else { return cascade.bail() }
        }
        if userAgentHasImportant {
            guard cascade.addImportantMatches(userAgentRange, origin: 0) else { return cascade.bail() }
        }

        let seenCount = cascade.sortLogicalGroupPropertyIDs(&logicalIDs)

        scalars[scalarLastIndexForLogicalGroup] = cascade.lastIndexForLogicalGroup
        scalars[scalarSeenLogicalGroupPropertyCount] = UInt32(seenCount)
        scalars[scalarLowestSeenLogicalGroupProperty] = UInt32(cascade.lowestSeenLogicalGroupProperty)
        scalars[scalarHighestSeenLogicalGroupProperty] = UInt32(cascade.highestSeenLogicalGroupProperty)
        return cascade.deferredCount
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
    @inline(__always) var propertyCount: Int { Int((storage >> 32) & 0x0FFF_FFFF) }

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
    static func metadataOffset(_ index: Int) -> Int
    static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int
    static func valueSize(packedPointerSize: Int) -> Int
}

// ImmutableStyleProperties: StylePropertyMetadata[count], then PackedPtr<const CSSValue>[count].
enum PropertyCascadeImmutableStorage: PropertyCascadeStorageLayout {
    @inline(__always) static func metadataOffset(_ index: Int) -> Int { 4 * index }
    @inline(__always) static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int { 4 * count + packedPointerSize * index }
    @inline(__always) static func valueSize(packedPointerSize: Int) -> Int { packedPointerSize }
}

// MutableStyleProperties: CSSProperty[count] (StylePropertyMetadata @0, Ref<CSSValue> @8; 16 bytes).
enum PropertyCascadeMutableStorage: PropertyCascadeStorageLayout {
    @inline(__always) static func metadataOffset(_ index: Int) -> Int { 16 * index }
    @inline(__always) static func valueOffset(_ index: Int, count: Int, packedPointerSize: Int) -> Int { 16 * index + 8 }
    @inline(__always) static func valueSize(packedPointerSize: Int) -> Int { 8 }
}

// The state of one PropertyCascade under construction, and its read-only inputs.
struct PropertyCascadeState<Storage: PropertyCascadeMatchStorage>: ~Copyable, ~Escapable {
    let storage: Storage
    let matches: Span<UInt64>
    let includedIDs: Span<UInt16>
    let table: Span<UInt16>
    var records: MutableRawSpan              // m_properties
    var present: MutableSpan<UInt64>         // m_propertyIsPresent
    var logicalIndices: MutableSpan<UInt32>  // m_logicalGroupPropertyIndices
    var work: MutableSpan<UInt64>            // [0, scratchCount): sort scratch; then the recorded custom sets
    let includedTypes: UInt32
    let firstLogicalGroupProperty: Int
    let packedPointerSize: Int
    let scratchCount: Int
    var lastIndexForLogicalGroup: UInt32
    var lowestSeenLogicalGroupProperty: Int
    var highestSeenLogicalGroupProperty: Int
    var deferredCount: Int = 0

    @_lifetime(copy matches, copy includedIDs, copy table, copy records, copy present, copy logicalIndices, copy work)
    init(storage: Storage, matches: Span<UInt64>, includedIDs: Span<UInt16>, table: Span<UInt16>, records: consuming MutableRawSpan,
        present: consuming MutableSpan<UInt64>, logicalIndices: consuming MutableSpan<UInt32>, work: consuming MutableSpan<UInt64>,
        includedTypes: UInt32, firstLogicalGroupProperty: Int, packedPointerSize: Int, scratchCount: Int, lastIndexForLogicalGroup: UInt32,
        lowestSeenLogicalGroupProperty: Int, highestSeenLogicalGroupProperty: Int)
    {
        self.storage = storage
        self.matches = matches
        self.includedIDs = includedIDs
        self.table = table
        self.records = records
        self.present = present
        self.logicalIndices = logicalIndices
        self.work = work
        self.includedTypes = includedTypes
        self.firstLogicalGroupProperty = firstLogicalGroupProperty
        self.packedPointerSize = packedPointerSize
        self.scratchCount = scratchCount
        self.lastIndexForLogicalGroup = lastIndexForLogicalGroup
        self.lowestSeenLogicalGroupProperty = lowestSeenLogicalGroupProperty
        self.highestSeenLogicalGroupProperty = highestSeenLogicalGroupProperty
    }

    @inline(__always)
    func includes(_ type: UInt32) -> Bool { includedTypes & type != 0 }

    // Undo what C++ could observe and report that the glue must run buildCascadeCxx().
    @_lifetime(self: copy self)
    mutating func bail() -> Int {
        for index in present.indices {
            present[index] = 0
        }
        for index in logicalIndices.indices {
            logicalIndices[index] = 0
        }
        return PropertyCascade.bail
    }

    // addNormalMatches. Returns whether any match has important properties, nil to bail.
    @_lifetime(self: copy self)
    mutating func addNormalMatches(_ range: Range<Int>, origin: Int) -> Bool? {
        var hasImportant = false
        for match in range {
            guard let matchHasImportant = addMatch(match, origin: origin, important: false) else { return nil }
            hasImportant = hasImportant || matchHasImportant
        }
        return hasImportant
    }

    // addImportantMatches. Returns false to bail.
    @_lifetime(self: copy self)
    mutating func addImportantMatches(_ range: Range<Int>, origin: Int) -> Bool {
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
            return true
        }
        if hasMatchesFromOtherScopesOrLayers {
            PropertyCascadeState.sortDistinct(&work, count: count)
        }
        for index in 0..<count {
            let match = Int(UInt32(truncatingIfNeeded: work[index]))
            if addMatch(match, origin: origin, important: true) == nil {
                return false
            }
        }
        return true
    }

    // hasImportantProperties(const StyleProperties&): every property, whatever the match's other scalars.
    func hasImportantProperties(_ match: Int, _ matched: PropertyCascadeMatch) -> Bool {
        let storage = self.storage
        let bytes = storage.bytes(match)
        for index in 0..<matched.propertyCount {
            let offset = matched.isMutable ? PropertyCascadeMutableStorage.metadataOffset(index) : PropertyCascadeImmutableStorage.metadataOffset(index)
            if bytes.load(fromByteOffset: offset, as: UInt32.self) & PropertyCascade.metadataImportantBit != 0 {
                return true
            }
        }
        return false
    }

    // addMatch. Returns hasImportantProperties, or nil to bail.
    // Guaranteed inlining (SE-0496) keeps both storage layouts' loops specialized and inlined; with
    // @inline(__always) the immutable one stays an out-of-line call per match (measured ~3-5% slower).
#if compiler(>=6.4)
    @inline(always)
#else
    @inline(__always)
#endif
    @_lifetime(self: copy self)
    mutating func addMatch(_ match: Int, origin: Int, important: Bool) -> Bool? {
        let matched = PropertyCascadeMatch(matches, match)
        if matched.isStartingStyle && !includes(PropertyCascade.typeStartingStyle) {
            return false
        }
        // The glue routes matches with a property allowlist to the C++.
        if matched.allowlistType != 0 {
            return nil
        }
        let storage = self.storage
        let bytes = storage.bytes(match)
        if matched.isMutable {
            return addProperties(PropertyCascadeMutableStorage.self, bytes, match: match, matched: matched, origin: origin, important: important)
        }
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
        origin: Int, important: Bool) -> Bool?
    {
        let count = matched.propertyCount
        let includesAllNormal = includes(PropertyCascade.typeNonInherited) && includes(PropertyCascade.typeInherited)
        let includesPartiallyCacheable = matched.isCacheable == PropertyCascade.isCacheablePartially && includes(PropertyCascade.typeNonCacheable)
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
            let propertyID = Int(entry & PropertyCascade.tableAliasMask)
            let currentIsInherited = entry & PropertyCascade.tableInheritedBit != 0

            // shouldIncludeProperty.
            var shouldInclude: Bool
            if includesAllNormal || includesPartiallyCacheable || PropertyCascadeState.contains(includedIDs, propertyID) {
                shouldInclude = true
            } else {
                let includedByType = (includes(PropertyCascade.typeInherited) && currentIsInherited)
                    || (includes(PropertyCascade.typeNonInherited) && !currentIsInherited)
                // mayOverrideExistingProperty.
                if propertyID == PropertyCascade.customPropertyID {
                    // hasCustomProperty(name): false while no custom property has been set; otherwise it
                    // needs the name, which only matters when the types alone would exclude the property.
                    if deferredCount != 0 && !includedByType {
                        return nil
                    }
                    shouldInclude = includedByType
                } else if propertyID < firstLogicalGroupProperty {
                    shouldInclude = present[propertyID >> 6] & (1 << UInt64(propertyID & 63)) != 0 || includedByType
                } else {
                    shouldInclude = lastIndexForLogicalGroup != 0 || includedByType
                }
            }
            if !shouldInclude {
                continue
            }

            // set.
            if propertyID == PropertyCascade.customPropertyID {
                let slot = scratchCount + deferredCount
                if slot >= work.count {
                    return nil
                }
                work[slot] = UInt64(UInt32(truncatingIfNeeded: match)) << 32 | UInt64(UInt32(truncatingIfNeeded: index))
                deferredCount += 1
                continue
            }
            let value = PropertyCascadeState.loadPointer(bytes, Layout.valueOffset(index, count: count, packedPointerSize: packedPointerSize),
                size: Layout.valueSize(packedPointerSize: packedPointerSize))
            if propertyID >= firstLogicalGroupProperty {
                setLogicalGroupProperty(propertyID, value: value, matched: matched, origin: origin)
            } else {
                let word = propertyID >> 6
                let bit: UInt64 = 1 << UInt64(propertyID & 63)
                let wasPresent = present[word] & bit != 0
                present[word] |= bit
                setPropertyInternal(propertyID, value: value, matched: matched, origin: origin, clearValues: !wasPresent)
            }
        }
        return hasImportantProperties
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
