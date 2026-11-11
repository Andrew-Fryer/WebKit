/*
 * Copyright (C) 2025 Apple Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. AND ITS CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL APPLE INC. OR ITS CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "NameValidation.h"

#include "CommonAtomStrings.h"
#include "ExceptionOr.h"
#include "QualifiedName.h"
#include "XMLNSNames.h"
#include "XMLNames.h"
#include <wtf/text/MakeString.h>
#include <wtf/text/StringView.h>

#include "NameValidationSwiftBoundary.h"

namespace WebCore::NameValidation {

// https://dom.spec.whatwg.org/#validate-and-extract

// The character checks run in Swift (Source/WebCore/swift/NameValidation).
NODELETE static bool isValidName(StringView name, WebCoreSwiftNameValidationKind kind)
{
    return WebCoreSwift_NameValidation_isValidName(name.rawCharacters(), name.length(), name.is8Bit(), kind);
}

// https://dom.spec.whatwg.org/#valid-element-name
bool isValidElementName(StringView name)
{
    return isValidName(name, WebCoreSwiftNameValidationElementName);
}

bool isValidElementName(const QualifiedName& name)
{
    return isValidElementName(name.localName());
}

// https://dom.spec.whatwg.org/#valid-attribute-name
bool isValidAttributeName(StringView name)
{
    return isValidName(name, WebCoreSwiftNameValidationAttributeName);
}

// https://dom.spec.whatwg.org/#valid-namespace-prefix
NODELETE static bool isValidNamespacePrefix(StringView prefix)
{
    return isValidName(prefix, WebCoreSwiftNameValidationNamespacePrefix);
}

// https://dom.spec.whatwg.org/#valid-doctype-name
bool isValidDoctypeName(StringView name)
{
    return isValidName(name, WebCoreSwiftNameValidationDoctypeName);
}

// https://www.w3.org/TR/xml/#NT-Name
bool isValidXMLName(StringView name)
{
    return isValidName(name, WebCoreSwiftNameValidationXMLName);
}

enum class QualifiedNameType : bool { Element, Attribute };

static ExceptionOr<std::pair<AtomString, AtomString>> parseQualifiedNameToPair(const AtomString& qualifiedName, QualifiedNameType type)
{
    if (qualifiedName.isEmpty())
        return Exception { ExceptionCode::InvalidCharacterError };

    auto firstColonPosition = qualifiedName.find(':');
    if (firstColonPosition == notFound) {
        bool isValidLocal = type == QualifiedNameType::Element
            ? isValidElementName(qualifiedName)
            : isValidAttributeName(qualifiedName);
        if (!isValidLocal)
            return Exception { ExceptionCode::InvalidCharacterError, makeString("Invalid name: '"_s, qualifiedName, '\'') };
        return std::pair<AtomString, AtomString> { nullAtom(), qualifiedName };
    }

    auto prefix = StringView { qualifiedName }.left(firstColonPosition);
    if (!isValidNamespacePrefix(prefix))
        return Exception { ExceptionCode::InvalidCharacterError, makeString("Invalid namespace prefix in '"_s, qualifiedName, '\'') };

    auto localName = StringView { qualifiedName }.substring(firstColonPosition + 1);

    if (localName.isEmpty())
        return Exception { ExceptionCode::InvalidCharacterError, makeString("Invalid local name in '"_s, qualifiedName, '\'') };

    bool isValidLocal = type == QualifiedNameType::Element
        ? isValidElementName(localName)
        : isValidAttributeName(localName);
    if (!isValidLocal)
        return Exception { ExceptionCode::InvalidCharacterError, makeString("Invalid local name in '"_s, qualifiedName, '\'') };

    return std::pair<AtomString, AtomString> { prefix.toAtomString(), localName.toAtomString() };
}

ExceptionOr<QualifiedName> parseQualifiedElementName(const AtomString& namespaceURI, const AtomString& qualifiedName)
{
    auto parseResult = parseQualifiedNameToPair(qualifiedName, QualifiedNameType::Element);
    if (parseResult.hasException())
        return parseResult.releaseException();
    auto [prefix, localName] = parseResult.releaseReturnValue();
    return QualifiedName { prefix, localName, namespaceURI };
}

ExceptionOr<QualifiedName> parseQualifiedAttributeName(const AtomString& namespaceURI, const AtomString& qualifiedName)
{
    auto parseResult = parseQualifiedNameToPair(qualifiedName, QualifiedNameType::Attribute);
    if (parseResult.hasException())
        return parseResult.releaseException();
    auto [prefix, localName] = parseResult.releaseReturnValue();
    return QualifiedName { prefix, localName, namespaceURI };
}

ExceptionOr<std::pair<AtomString, AtomString>> parseQualifiedAttributeName(const AtomString& qualifiedName)
{
    return parseQualifiedNameToPair(qualifiedName, QualifiedNameType::Attribute);
}

// These checks are from DOM Core Level 2, createElementNS
// http://www.w3.org/TR/DOM-Level-2-Core/core.html#ID-DocCrElNS
bool hasValidNamespaceForElements(const QualifiedName& qName)
{
    if (!qName.prefix().isEmpty() && qName.namespaceURI().isNull()) // createElementNS(null, "html:div")
        return false;
    if (qName.prefix() == xmlAtom() && qName.namespaceURI() != XMLNames::xmlNamespaceURI) // createElementNS("http://www.example.com", "xml:lang")
        return false;

    // Required by DOM Level 3 Core and unspecified by DOM Level 2 Core:
    // http://www.w3.org/TR/2004/REC-DOM-Level-3-Core-20040407/core.html#ID-DocCrElNS
    // createElementNS("http://www.w3.org/2000/xmlns/", "foo:bar"), createElementNS(null, "xmlns:bar"), createElementNS(null, "xmlns")
    if (qName.prefix() == xmlnsAtom() || (qName.prefix().isEmpty() && qName.localName() == xmlnsAtom()))
        return qName.namespaceURI() == XMLNSNames::xmlnsNamespaceURI;
    return qName.namespaceURI() != XMLNSNames::xmlnsNamespaceURI;
}

bool hasValidNamespaceForAttributes(const QualifiedName& qName)
{
    return hasValidNamespaceForElements(qName);
}

} // namespace WebCore::NameValidation
