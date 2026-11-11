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

// The per-cell loops of MarkedBlock::Handle::specializedSweep (heap/MarkedBlockInlines.h) for JSString and
// JSRopeString blocks: run the destructor of every dead cell that is not zapped yet, then zap it, and thread the
// runs of dead cells into a free list (scribbling over them first in Scribble mode). specializedSweep calls this for
// every sweep of such a block that destroys cells (the block was unswept); the sweeps that destroy nothing, and
// every other DestroyFunc, keep the C++ loops.
//
// Swift sees the block's payload, from the handle's first cell (m_startAtom) to the end of the block, as a span
// of 16-byte atoms. It never sees the header (marks, newly-allocated bits, lock). Atom indices below are relative
// to m_startAtom; mark-bit indices are absolute atom numbers, as in the C++.
//
// The JSString destructor is `valueInternal().~String()`: ~RefPtr nulls m_fiber and derefs the StringImpl. Swift
// does the nulling and appends the StringImpl pointer to `derefs`, in cell order; the C++ caller derefs them all
// once the sweep returns. StringImpl::destroy never touches GC blocks, so nothing can observe the delay.

/// One 16-byte MarkedBlock atom as two 64-bit words. In a JSCell, the low half of [0] is the header word that
/// HeapCell::zap() clears and isZapped() tests (its high half, cellWords[1], is never written: zap keeps it for
/// crash analysis), and [1] is JSString::m_fiber. In a FreeCell, [1] is scrambledBits. (All lab targets are
/// little-endian, so cellWords[0] and cellWords[2] are the low halves of [0] and [1].)
typealias MarkedBlockAtom = InlineArray<2, UInt64>

#if !_endian(little)
#error("MarkedBlockAtom assumes a little-endian target")
#endif

@inline(__always)
var lowHalfMask: UInt64 { 0x0000_0000_FFFF_FFFF }

/// HeapCell::ZapReason.
enum HeapCellZapReason {
    static var destruction: UInt32 { 1 }
    static var stopAllocating: UInt32 { 2 }
}

/// The DestroyFunc of the cells being swept.
enum MarkedBlockSweepDestroyKind {
    /// IsoInlinedHeapCellType<JSString>: JSString::destroy, `valueInternal().~String()`.
    case string
    /// IsoInlinedHeapCellType<JSRopeString>: JSRopeString::destroy, which returns early if isRope()
    /// (JSString::isRopeInPointer, bit 0 of m_fiber) and is JSString::destroy otherwise.
    case rope
}

struct MarkedBlockSweepResult {
    /// Payload byte offset of the free list's first FreeCell, or -1 if there is none (or SweepOnly).
    var headOffset: Int
    /// Bytes in the free list (SweepToFreeList only).
    var freedBytes: Int
    /// How many StringImpl pointers were appended to `derefs`.
    var derefCount: Int
}

/// FreeCell::scramble: `(uint64_t(lengthInBytes) << 32 | offsetToNext) ^ secret`, where the int32 offset is
/// sign-extended to 64 bits by the `|`. Callers pass 1 (makeLast) or the distance between two cells of one block
/// (setNext), which is within Int32 range, so the truncation to int32 and the sign extension change nothing.
@inline(__always)
func scrambleFreeCell(offsetToNext: Int, lengthInBytes: UInt32, secret: UInt64) -> UInt64 {
    ((UInt64(lengthInBytes) &<< 32) | UInt64(bitPattern: Int64(offsetToNext))) ^ secret
}

/// scribble(cell, bytes) (heap/Scribble.h): every 8-byte word of the dead cells becomes
/// JSValue::encode(bit_cast<JSCell*>(SCRIBBLE_WORD)).
@inline(__always)
var scribbleWord: UInt64 { 0xbadbeef0 }

/// scribble() over the free interval begin ..< end, which the C++ does before writing the interval's FreeCell: so
/// every word but the first cell's scrambledBits (already written by the sweep, kept here) becomes scribbleWord.
@inline(__always)
func scribbleInterval(_ atoms: inout MutableSpan<MarkedBlockAtom>, from begin: Int, to end: Int) {
    let scrambledBits = atoms[begin][1]
    for a in begin ..< end {
        atoms[a] = MarkedBlockAtom(repeating: scribbleWord)
    }
    atoms[begin][1] = scrambledBits
}

/// Scribble mode, after a SweepToFreeList sweep: scribbleInterval over each interval of the free list, found by the
/// same walk of the live bits as the sweep's (or the whole payload when allDead). Kept out of the sweep's loops,
/// since only debug builds and Options::scribbleFreeCells scribble.
@inline(never)
func scribbleFreeIntervals(_ atoms: inout MutableSpan<MarkedBlockAtom>, live: Span<UInt64>, allDead: Bool, startAtom: Int, atomsPerCell: Int) {
    let count = atoms.count
    if allDead {
        scribbleInterval(&atoms, from: 0, to: count)
        return
    }
    var potentiallyFreeAtom = 0
    for w in live.indices {
        var word = live[w]
        let base = w &* 64 &- startAtom
        while word != 0 {
            let index = base &+ word.trailingZeroBitCount
            word &= word &- 1
            if index != potentiallyFreeAtom {
                scribbleInterval(&atoms, from: potentiallyFreeAtom, to: index)
            }
            potentiallyFreeAtom = index &+ atomsPerCell
        }
    }
    if count != potentiallyFreeAtom {
        scribbleInterval(&atoms, from: potentiallyFreeAtom, to: count)
    }
}

/// The deref list: one slot per atom of a block (MarkedBlock::atomsPerBlock), so that no sweep can fill it, as a
/// single fixed-size element: `derefs[0][n & 1023]` needs no bounds check, and the mask never changes n.
typealias MarkedBlockDerefs = InlineArray<1024, UInt64>

/// destroy(cell) as specializedSweep's `destroy` lambda: if the cell at atom `a` is not zapped yet, run its
/// destructor and zap it with HeapCell::Destruction.
@inline(__always)
func destroyCell(_ atoms: inout MutableSpan<MarkedBlockAtom>, _ a: Int, rope: Bool, _ derefs: inout MutableSpan<MarkedBlockDerefs>, _ n: inout Int) {
    let header = atoms[a][0]
    if header & lowHalfMask != 0 {
        let fiber = atoms[a][1]
        let isRope = rope && (fiber & 1) != 0
        // ~String (unless the rope returned early) sets m_fiber to nullptr and derefs the old StringImpl, if any.
        // zap(Destruction) then clears the header word and stores Destruction in the low half of m_fiber.
        atoms[a][0] = header & ~lowHalfMask
        atoms[a][1] = (isRope ? fiber & ~lowHalfMask : 0) | UInt64(HeapCellZapReason.destruction)
        // Branch-free append; n < 1024 since there are at most 1024 cells.
        derefs[0][n & 1023] = fiber
        n &+= (isRope || fiber == 0) ? 0 : 1
    }
}

/// destroy(cell) for every cell whose first atom is in begin ..< end, stepping by atomsPerCell: an interval's loop.
@inline(__always)
func destroyCells(_ atoms: inout MutableSpan<MarkedBlockAtom>, from begin: Int, to end: Int, atomsPerCell: Int, rope: Bool,
                  _ derefs: inout MutableSpan<MarkedBlockDerefs>, _ derefCount: inout Int) {
    var n = derefCount
    var a = begin
    while a < end {
        destroyCell(&atoms, a, rope: rope, &derefs, &n)
        a &+= atomsPerCell
    }
    derefCount = n
}

/// destroy(cell) for every cell of the payload. The loop runs to atoms.count itself, so the per-cell bounds checks
/// fold into the loop condition.
@inline(__always)
func destroyAllCells(_ atoms: inout MutableSpan<MarkedBlockAtom>, atomsPerCell: Int, rope: Bool,
                     _ derefs: inout MutableSpan<MarkedBlockDerefs>, _ derefCount: inout Int) {
    var n = derefCount
    var a = 0
    while a < atoms.count {
        destroyCell(&atoms, a, rope: rope, &derefs, &n)
        a &+= atomsPerCell
    }
    derefCount = n
}

/// One specialization of the sweep, like specializedSweep's template parameters: a generic parameter, so that
/// each (destroyKind, toFreeList) pair gets its own copy of the loops with the constants folded.
protocol MarkedBlockSweepVariant {
    static var destroyKind: MarkedBlockSweepDestroyKind { get }
    static var toFreeList: Bool { get }
}
enum SweepStringToFreeList: MarkedBlockSweepVariant {
    static var destroyKind: MarkedBlockSweepDestroyKind { .string }
    static var toFreeList: Bool { true }
}
enum SweepStringOnly: MarkedBlockSweepVariant {
    static var destroyKind: MarkedBlockSweepDestroyKind { .string }
    static var toFreeList: Bool { false }
}
enum SweepRopeToFreeList: MarkedBlockSweepVariant {
    static var destroyKind: MarkedBlockSweepDestroyKind { .rope }
    static var toFreeList: Bool { true }
}
enum SweepRopeOnly: MarkedBlockSweepVariant {
    static var destroyKind: MarkedBlockSweepDestroyKind { .rope }
    static var toFreeList: Bool { false }
}

/// specializedSweep's sweep of one block, for one Variant.
@inline(never)
func sweepMarkedBlockSpecialized<Variant: MarkedBlockSweepVariant>(_ variant: Variant.Type, _ atoms: consuming MutableSpan<MarkedBlockAtom>,
                                                                   live: Span<UInt64>, allDead: Bool, startAtom: UInt32, atomsPerCell: UInt32,
                                                                   secret: UInt64, scribble: Bool, derefs: consuming MutableSpan<MarkedBlockDerefs>) -> MarkedBlockSweepResult {
    // (The spans are owned here, not inout, so that the compiler keeps their base addresses in registers.)
    let rope = Variant.destroyKind == .rope
    let toFreeList = Variant.toFreeList
    // At most a block of atoms, so at most 1024 derefs, and indices small enough that the loops cannot overflow.
    // (Unsigned, so that the compiler also knows the count is not negative.)
    precondition(atomsPerCell > 0 && UInt(bitPattern: atoms.count) <= 1024)
    let startAtom = Int(startAtom)
    let atomsPerCell = Int(atomsPerCell)
    let count = atoms.count
    var derefCount = 0
    precondition(derefs.count == 1)
    // Every live bit is then at an atom below the end of the payload (so every interval ends within it).
    precondition(allDead || live.count &* 64 <= startAtom &+ count)

    if allDead {
        // The `emptyMode == IsEmpty || ...` path: every cell is dead, and the free list is one interval.
        destroyAllCells(&atoms, atomsPerCell: atomsPerCell, rope: rope, &derefs, &derefCount)
        guard toFreeList else {
            return MarkedBlockSweepResult(headOffset: -1, freedBytes: 0, derefCount: derefCount)
        }
        // FreeCell::makeLast(payloadEnd - payloadBegin, secret) on the first cell.
        atoms[0][1] = scrambleFreeCell(offsetToNext: 1, lengthInBytes: UInt32(truncatingIfNeeded: count &* 16), secret: secret)
        if scribble {
            scribbleFreeIntervals(&atoms, live: live, allDead: true, startAtom: startAtom, atomsPerCell: atomsPerCell)
        }
        return MarkedBlockSweepResult(headOffset: 0, freedBytes: count &* 16, derefCount: derefCount)
    }

    // With one-atom cells (JSString's), every payload atom is a cell, so the cells the interval walk below would
    // destroy are exactly the atoms with a clear bit. Destroy them first, in the same (address) order, walking the
    // dead bits a word at a time; then the interval walk only builds the free list. Each interval's first cell gets
    // its scrambledBits from setNext or makeLast after it was destroyed either way, so the bytes are the same.
    var destroyInIntervals = true
    if atomsPerCell == 1 && live.count &* 64 == startAtom &+ count {
        for w in live.indices {
            let base = w &* 64 &- startAtom // the payload index of the word's bit 0
            var dead = ~live[w]
            if base < 0 {
                if base <= -64 {
                    precondition(live[w] == 0) // a live bit before the first cell, as in the interval walk
                    continue
                }
                let beforePayload = ~(UInt64.max &<< UInt64(truncatingIfNeeded: -base))
                precondition(live[w] & beforePayload == 0)
                dead &= ~beforePayload
            }
            while dead != 0 {
                destroyCell(&atoms, base &+ (dead.trailingZeroBitCount & 63), rope: rope, &derefs, &derefCount)
                dead &= dead &- 1
            }
        }
        guard toFreeList else {
            return MarkedBlockSweepResult(headOffset: -1, freedBytes: 0, derefCount: derefCount)
        }
        destroyInIntervals = false
    }

    // The sweepBlock lambda. handleLiveCell(index) for each live atom index in order, then for endAtom: the dead
    // cells between the end of the previous live cell and `index` form one interval (pushInterval).
    var potentiallyFreeAtom = 0 // relative to startAtom, like every index below
    var head = -1
    var cursor = -1
    var cursorBytes = 0
    var freedBytes = 0

    @inline(__always)
    func pushInterval(_ atoms: inout MutableSpan<MarkedBlockAtom>, _ begin: Int, _ end: Int, _ derefs: inout MutableSpan<MarkedBlockDerefs>) {
        if destroyInIntervals {
            destroyCells(&atoms, from: begin, to: end, atomsPerCell: atomsPerCell, rope: rope, &derefs, &derefCount)
        }
        if toFreeList {
            if head < 0 {
                head = begin
            }
            // `if (cursor)`: cursor is -1 or a payload index, so the unsigned compare is that test and the bounds check.
            if UInt(bitPattern: cursor) < UInt(bitPattern: count) {
                // cursor->setNext(cell, cursorIntervalBytes, secret): the int32 byte distance to the next interval.
                atoms[cursor][1] = scrambleFreeCell(offsetToNext: (begin &- cursor) &* 16,
                                                    lengthInBytes: UInt32(truncatingIfNeeded: cursorBytes), secret: secret)
            }
            cursor = begin
            cursorBytes = (end &- begin) &* 16
            freedBytes &+= cursorBytes
        }
    }

    for w in live.indices {
        var word = live[w]
        let base = w &* 64 &- startAtom
        while word != 0 {
            let index = base &+ word.trailingZeroBitCount
            word &= word &- 1
            if index != potentiallyFreeAtom {
                // A live bit before the first cell or inside the previous live cell: a corrupt bitmap, where the
                // C++ would build an interval of negative length.
                precondition(index > potentiallyFreeAtom)
                pushInterval(&atoms, potentiallyFreeAtom, index, &derefs)
            }
            potentiallyFreeAtom = index &+ atomsPerCell
        }
    }
    // handleLiveCell(endAtom).
    if count != potentiallyFreeAtom {
        precondition(count > potentiallyFreeAtom)
        pushInterval(&atoms, potentiallyFreeAtom, count, &derefs)
    }
    if toFreeList && UInt(bitPattern: cursor) < UInt(bitPattern: count) {
        // cursor->makeLast(cursorIntervalBytes, secret).
        atoms[cursor][1] = scrambleFreeCell(offsetToNext: 1, lengthInBytes: UInt32(truncatingIfNeeded: cursorBytes), secret: secret)
    }
    if toFreeList && scribble {
        scribbleFreeIntervals(&atoms, live: live, allDead: false, startAtom: startAtom, atomsPerCell: atomsPerCell)
    }
    return MarkedBlockSweepResult(headOffset: head < 0 ? -1 : head &* 16, freedBytes: freedBytes, derefCount: derefCount)
}

/// Sweeps the payload `atoms` (atoms m_startAtom ..< endAtom of the block). `live` is the live-cell bitmap
/// (marks, or marks | newlyAllocated) as BitSet<atomsPerBlock> words, ignored when `allDead` (the IsEmpty path).
/// With `toFreeList`, builds the free list in the dead cells (scribbled over first if `scribble`) and returns its
/// head and size, as passed to FreeList::initialize. `derefs` is one MarkedBlockDerefs.
func sweepMarkedBlock(_ atoms: consuming MutableSpan<MarkedBlockAtom>, live: Span<UInt64>, allDead: Bool,
                      startAtom: UInt32, atomsPerCell: UInt32, secret: UInt64, toFreeList: Bool, scribble: Bool,
                      destroyKind: MarkedBlockSweepDestroyKind, derefs: consuming MutableSpan<MarkedBlockDerefs>) -> MarkedBlockSweepResult {
    switch (destroyKind, toFreeList) {
    case (.string, true):
        return sweepMarkedBlockSpecialized(SweepStringToFreeList.self, atoms, live: live, allDead: allDead, startAtom: startAtom, atomsPerCell: atomsPerCell, secret: secret, scribble: scribble, derefs: derefs)
    case (.string, false):
        return sweepMarkedBlockSpecialized(SweepStringOnly.self, atoms, live: live, allDead: allDead, startAtom: startAtom, atomsPerCell: atomsPerCell, secret: secret, scribble: scribble, derefs: derefs)
    case (.rope, true):
        return sweepMarkedBlockSpecialized(SweepRopeToFreeList.self, atoms, live: live, allDead: allDead, startAtom: startAtom, atomsPerCell: atomsPerCell, secret: secret, scribble: scribble, derefs: derefs)
    case (.rope, false):
        return sweepMarkedBlockSpecialized(SweepRopeOnly.self, atoms, live: live, allDead: allDead, startAtom: startAtom, atomsPerCell: atomsPerCell, secret: secret, scribble: scribble, derefs: derefs)
    }
}
