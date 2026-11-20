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

// C entry points of SVGPropertyRegistryTable (declared to C++ in SVGPropertyRegistryTableSwiftBoundary.h). This is the
// only file of the unit with `unsafe`: it wraps the table's words (owned by C++; null only when the count is 0) and the
// grown table in spans and calls the safe core in SVGPropertyRegistryTable.swift.

@_cdecl("WebCoreSwift_SVGPropertyRegistryTable_find")
@inline(__always)
func WebCoreSwift_SVGPropertyRegistryTable_find(_ table: UnsafePointer<UInt64>?, _ tableWords: Int, _ localName: UInt64, _ namespaceURI: UInt64) -> UInt64 {
    return unsafe SVGPropertyRegistryTable.find(UnsafeBufferPointer(start: table, count: tableWords).span, localName, namespaceURI)
}

@_cdecl("WebCoreSwift_SVGPropertyRegistryTable_add")
func WebCoreSwift_SVGPropertyRegistryTable_add(_ table: UnsafeMutablePointer<UInt64>?, _ tableWords: Int, _ localName: UInt64, _ namespaceURI: UInt64, _ key: UInt64, _ accessor: UInt64) -> Int {
    var tableSpan = unsafe UnsafeMutableBufferPointer(start: table, count: tableWords).mutableSpan
    return SVGPropertyRegistryTable.add(&tableSpan, localName, namespaceURI, key, accessor)
}

@_cdecl("WebCoreSwift_SVGPropertyRegistryTable_rehash")
func WebCoreSwift_SVGPropertyRegistryTable_rehash(_ table: UnsafePointer<UInt64>?, _ tableWords: Int, _ grown: UnsafeMutablePointer<UInt64>?, _ grownWords: Int) {
    var grownSpan = unsafe UnsafeMutableBufferPointer(start: grown, count: grownWords).mutableSpan
    unsafe SVGPropertyRegistryTable.rehash(UnsafeBufferPointer(start: table, count: tableWords).span, &grownSpan)
}
