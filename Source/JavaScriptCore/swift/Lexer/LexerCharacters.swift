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

// The source text of the Swift lexer scans (LexerRegExp.swift, LexerEscapedLiterals.swift): the code units from the
// lexer's m_code to m_codeEnd, Latin-1 or UTF-16. The scans are generic over it, so each kind of source gets its own
// copy of the loops:
//   Latin1Characters, a Span<UInt8>;
//   UTF16Characters, a Span<UInt16>: UTF-16 text at an even address (the usual case);
//   UnalignedUTF16Characters: UTF-16 text at an odd address, which a Span<UInt16> cannot view (a String can point
//   into a buffer at any alignment), read two bytes at a time through a RawSpan.
protocol LexerCharacters: ~Escapable {
    // UTF-16: LS and PS are line terminators, and code units above U+00FF exist.
    static var isWide: Bool { get }
    // The number of code units.
    var count: Int { get }
    // The code unit at index i; traps unless 0 <= i < count.
    func unit(_ i: Int) -> UInt32
    // The text as bytes (two per code unit when wide), for the SIMD scans.
    var bytes: RawSpan { @_lifetime(copy self) get }
}

struct Latin1Characters: ~Escapable, LexerCharacters {
    let units: Span<UInt8>

    @inline(__always)
    @_lifetime(copy units)
    init(_ units: Span<UInt8>) {
        self.units = units
    }

    @inline(__always)
    static var isWide: Bool { false }

    @inline(__always)
    var count: Int { units.count }

    @inline(__always)
    func unit(_ i: Int) -> UInt32 { UInt32(units[i]) }

    var bytes: RawSpan {
        @inline(__always)
        @_lifetime(copy self)
        get { units.bytes }
    }
}

struct UTF16Characters: ~Escapable, LexerCharacters {
    let units: Span<UInt16>

    @inline(__always)
    @_lifetime(copy units)
    init(_ units: Span<UInt16>) {
        self.units = units
    }

    @inline(__always)
    static var isWide: Bool { true }

    @inline(__always)
    var count: Int { units.count }

    @inline(__always)
    func unit(_ i: Int) -> UInt32 { UInt32(units[i]) }

    var bytes: RawSpan {
        @inline(__always)
        @_lifetime(copy self)
        get { units.bytes }
    }
}

struct UnalignedUTF16Characters: ~Escapable, LexerCharacters {
    let bytes: RawSpan

    @inline(__always)
    @_lifetime(copy bytes)
    init(_ bytes: RawSpan) {
        self.bytes = bytes
    }

    @inline(__always)
    static var isWide: Bool { true }

    @inline(__always)
    var count: Int { bytes.byteCount &>> 1 }

    // RawSpan.load is an unaligned, bounds-checked load.
    @inline(__always)
    func unit(_ i: Int) -> UInt32 { UInt32(bytes.load(fromByteOffset: i &* 2, as: UInt16.self)) }
}
