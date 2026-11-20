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

// What used to be WTF::FastBitVector::clearRange(begin, end), which only DFG code called (the CFA's
// InPlaceAbstractState::beginBasicBlock and StoreBarrierClusteringPhase). A FastBitVector stores bit i in bit
// (i % 32) of 32-bit word i / 32. The words are passed as one MutableSpan holding all fastBitVectorArrayLength(numBits)
// words, including the unused high bits of the last word.
//
// The C++ cleared a range shorter than 32 bits, and the partial words at either end of a longer range, one bit at a
// time through FastBitVector::at(i), which RELEASE_ASSERTs i < numBits. It cleared the whole words in between through a
// std::span, which libc++'s hardening bounds-checks. This code clears the same bits with at most two masked word
// updates plus the run of whole words in between. It keeps the same results and the same traps:
// - Each bit-wise range [b, e) that the C++ would have walked traps if e > numBits (it walks b ..< e with b < e, and
//   e - 1 >= numBits is the first failing index).
// - The whole-word range traps if it ends beyond the words, like the hardened span.
// - The degenerate begin > end case behaves the same too: as in the C++, end - begin wraps to a huge size, so only the
//   partial words at either end are cleared.
// A trap may come before or after the in-range stores that the C++ would have done first. Either way the process dies.

enum BitRangeClear {
    // FastBitVector::clearRange.
    @inline(__always)
    static func clearRange(_ words: inout MutableSpan<UInt32>, numBits: UInt, _ begin: UInt, _ end: UInt) {
        if begin == 0 {
            clearPrefix(&words, numBits: numBits, end)
            return
        }

        // As in the C++, a reversed range wraps to a huge size and takes the slop path below.
        if end &- begin < 32 {
            if begin < end {
                clearShortRange(&words, numBits: numBits, begin, end)
            }
            return
        }

        let endBeginSlop = (begin &+ 31) & ~31
        let beginEndSlop = end & ~31

        // The C++ cleared [begin, endBeginSlop) bit by bit: when begin is not a multiple of 32, the bits from begin % 32
        // up of word begin / 32. (When begin + 31 wraps, endBeginSlop is 0 and the C++ loop is empty too.)
        if begin < endBeginSlop {
            precondition(endBeginSlop <= numBits)
            words[Int(truncatingIfNeeded: begin &>> 5)] &= ~(UInt32.max &<< UInt32(truncatingIfNeeded: begin & 31))
        }
        // The C++ cleared [beginEndSlop, end) bit by bit: when end is not a multiple of 32, the low end % 32 bits of word
        // end / 32.
        if beginEndSlop < end {
            precondition(end <= numBits)
            words[Int(truncatingIfNeeded: end &>> 5)] &= UInt32.max &<< UInt32(truncatingIfNeeded: end & 31)
        }

        // The whole words [endBeginSlop / 32, beginEndSlop / 32). Both are at most UInt.max / 32, so they fit in an Int.
        let firstWord = Int(truncatingIfNeeded: endBeginSlop &>> 5)
        let lastWord = Int(truncatingIfNeeded: beginEndSlop &>> 5)
        if firstWord < lastWord {
            precondition(lastWord <= words.count)
            zeroWords(&words, firstWord, lastWord)
        }
    }

    // clearRange(0, end), what both DFG callers do. It is the general path with begin = 0 folded in: endBeginSlop is
    // 0, so there is no partial first word, and a range shorter than 32 bits (end < 32) is the partial word at end
    // with no whole words, which the C++ cleared bit by bit with the same RELEASE_ASSERT.
    @inline(__always)
    static func clearPrefix(_ words: inout MutableSpan<UInt32>, numBits: UInt, _ end: UInt) {
        // The first call of each InPlaceAbstractState has an empty vector. Returning first makes this about 1.4 ns
        // instead of 1.7 ns, with no measurable cost to the other lengths.
        if end == 0 {
            return
        }
        let wholeWords = Int(truncatingIfNeeded: end &>> 5)
        if end & 31 != 0 {
            precondition(end <= numBits)
            words[wholeWords] &= UInt32.max &<< UInt32(truncatingIfNeeded: end & 31)
        }
        if wholeWords > 0 {
            precondition(wholeWords <= words.count)
            zeroWords(&words, 0, wholeWords)
        }
    }

    // Clears bits [begin, end) of a range shorter than 32 bits with begin < end, which covers at most two words: what
    // the C++ did bit by bit.
    @inline(__always)
    static func clearShortRange(_ words: inout MutableSpan<UInt32>, numBits: UInt, _ begin: UInt, _ end: UInt) {
        precondition(end <= numBits)
        let last = end &- 1
        let firstWord = Int(truncatingIfNeeded: begin &>> 5)
        let lastWord = Int(truncatingIfNeeded: last &>> 5)
        let lowMask = UInt32.max &<< UInt32(truncatingIfNeeded: begin & 31)
        let highMask = UInt32.max &>> UInt32(truncatingIfNeeded: 31 &- (last & 31))
        if firstWord == lastWord {
            words[firstWord] &= ~(lowMask & highMask)
        } else {
            words[firstWord] &= ~lowMask
            words[lastWord] &= ~highMask
        }
    }

    // words[first ..< last] = 0, with 0 <= first < last <= words.count. The DFG's vectors are short (the profile
    // averages 1.5 whole words per call), and a plain loop becomes a bzero call, which costs more than the stores for a
    // few words. So up to 8 words take two overlapping runs of 4, 2 or 1 stores (a word stored twice is 0 both times,
    // and nothing else can see these words during the call), and only longer runs use the loop.
    @inline(__always)
    static func zeroWords(_ words: inout MutableSpan<UInt32>, _ first: Int, _ last: Int) {
        let count = last &- first
        if count > 8 {
            var i = first
            while i < last {
                words[i] = 0
                i &+= 1
            }
        } else if count >= 4 {
            zero4(&words, first)
            zero4(&words, last &- 4)
        } else if count >= 2 {
            zero2(&words, first)
            zero2(&words, last &- 2)
        } else {
            words[first] = 0
        }
    }

    @inline(__always)
    static func zero2(_ words: inout MutableSpan<UInt32>, _ at: Int) {
        words[at] = 0
        words[at &+ 1] = 0
    }

    @inline(__always)
    static func zero4(_ words: inout MutableSpan<UInt32>, _ at: Int) {
        words[at] = 0
        words[at &+ 1] = 0
        words[at &+ 2] = 0
        words[at &+ 3] = 0
    }
}
