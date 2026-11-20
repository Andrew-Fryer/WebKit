/*
 * Copyright (C) 2014-2023 Apple Inc. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY APPLE INC. ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL APPLE INC. OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "config.h"
#include "BuiltinExecutables.h"

#include "BuiltinNames.h"
#include "BuiltinSourceScanSwiftBoundary.h"
#include "JSCJSValueInlines.h"
#include "Parser.h"
#include <wtf/NeverDestroyed.h>
#include <wtf/StdLibExtras.h>
#include <wtf/TZoneMallocInlines.h>

WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN

namespace JSC {

WTF_MAKE_TZONE_ALLOCATED_IMPL(BuiltinExecutables);

BuiltinExecutables::BuiltinExecutables(VM& vm)
    : m_vm(vm)
    , m_combinedSourceProvider(StringSourceProvider::create(StringImpl::createWithoutCopying({ s_JSCCombinedCode, s_JSCCombinedCodeLength }), { }, String(), SourceTaintedOrigin::Untainted))
{
}

SourceCode BuiltinExecutables::defaultConstructorSourceCode(ConstructorKind constructorKind)
{
    switch (constructorKind) {
    case ConstructorKind::None:
    case ConstructorKind::Naked:
        break;
    case ConstructorKind::Base: {
        static NeverDestroyed<const String> baseConstructorCode(MAKE_STATIC_STRING_IMPL("(function () { })"));
        return makeSource(baseConstructorCode, { }, SourceTaintedOrigin::Untainted);
    }
    case ConstructorKind::Extends: {
        static NeverDestroyed<const String> derivedConstructorCode(MAKE_STATIC_STRING_IMPL("(function (...args) { super(...args); })"));
        return makeSource(derivedConstructorCode, { }, SourceTaintedOrigin::Untainted);
    }
    }
    RELEASE_ASSERT_NOT_REACHED();
    return SourceCode();
}

UnlinkedFunctionExecutable* BuiltinExecutables::createDefaultConstructor(ConstructorKind constructorKind, const Identifier& name, NeedsClassFieldInitializer needsClassFieldInitializer, PrivateBrandRequirement privateBrandRequirement)
{
    switch (constructorKind) {
    case ConstructorKind::None:
    case ConstructorKind::Naked:
        break;
    case ConstructorKind::Base:
    case ConstructorKind::Extends:
        return createExecutable(m_vm, defaultConstructorSourceCode(constructorKind), name, ImplementationVisibility::Public, constructorKind, ConstructAbility::CanConstruct, InlineAttribute::Always, needsClassFieldInitializer, privateBrandRequirement);
    }
    ASSERT_NOT_REACHED();
    return nullptr;
}

UnlinkedFunctionExecutable* BuiltinExecutables::createBuiltinExecutable(const SourceCode& code, const BuiltinSourceMetadata& metadata, const Identifier& name, ImplementationVisibility implementationVisibility, ConstructorKind constructorKind, ConstructAbility constructAbility, InlineAttribute inlineAttribute)
{
    return createExecutable(m_vm, code, metadata, name, implementationVisibility, constructorKind, constructAbility, inlineAttribute, NeedsClassFieldInitializer::No, PrivateBrandRequirement::None);
}

// FIXME: Can we just make MetaData computation be constexpr and have the compiler do this for us?
// https://bugs.webkit.org/show_bug.cgi?id=193272
// Scanning by hand rather than parsing keeps us from recursing into the parser, and hence from
// overflowing the stack on a builtin. The scan is BuiltinSourceScan.swift; this only unpacks its result.
static BuiltinSourceMetadata computeBuiltinSourceMetadata(std::span<const Latin1Character> characters)
{
    // BuiltinSourceScan.swift writes the fields in this order, one word each (BuiltinSourceScan.wordCount).
    static_assert(JSCSwift_BuiltinSourceScan_WordCount == 6 && JSCSwift_BuiltinSourceScan_IsInStrictContext == 5);
    std::array<uint32_t, JSCSwift_BuiltinSourceScan_WordCount> words;
    JSCSwift_BuiltinSourceScan_computeMetadata(byteCast<uint8_t>(characters.data()), characters.size(), words.data());

    BuiltinSourceMetadata result;
    result.sourceLength = words[JSCSwift_BuiltinSourceScan_SourceLength];
    result.parametersStart = words[JSCSwift_BuiltinSourceScan_ParametersStart];
    result.parameterCount = words[JSCSwift_BuiltinSourceScan_ParameterCount];
    result.closeBraceOffsetFromEnd = static_cast<int>(words[JSCSwift_BuiltinSourceScan_CloseBraceOffsetFromEnd]);
    result.isAsyncFunction = words[JSCSwift_BuiltinSourceScan_IsAsyncFunction];
    result.isInStrictContext = words[JSCSwift_BuiltinSourceScan_IsInStrictContext];
    return result;
}

UnlinkedFunctionExecutable* BuiltinExecutables::createExecutable(VM& vm, const SourceCode& source, const Identifier& name, ImplementationVisibility implementationVisibility, ConstructorKind constructorKind, ConstructAbility constructAbility, InlineAttribute inlineAttribute, NeedsClassFieldInitializer needsClassFieldInitializer, PrivateBrandRequirement privateBrandRequirement)
{
    StringView view = source.view();
    RELEASE_ASSERT(!view.isNull());
    RELEASE_ASSERT(view.is8Bit());
    return createExecutable(vm, source, computeBuiltinSourceMetadata(view.span8()), name, implementationVisibility, constructorKind, constructAbility, inlineAttribute, needsClassFieldInitializer, privateBrandRequirement);
}

UnlinkedFunctionExecutable* BuiltinExecutables::createExecutable(VM& vm, const SourceCode& source, const BuiltinSourceMetadata& scanned, const Identifier& name, ImplementationVisibility implementationVisibility, ConstructorKind constructorKind, ConstructAbility constructAbility, InlineAttribute inlineAttribute, NeedsClassFieldInitializer needsClassFieldInitializer, PrivateBrandRequirement privateBrandRequirement)
{
    StringView view = source.view();
    RELEASE_ASSERT(scanned.sourceLength == view.length());

    unsigned parametersStart = scanned.parametersStart;
    int functionKeywordStart = strlen("(");
    int functionNameStart = parametersStart;
    bool isArrowFunctionBodyExpression = false;

    SourceCode newSource = source.subExpression(source.startOffset() + parametersStart, source.startOffset() + (view.length() - scanned.closeBraceOffsetFromEnd));
    bool isBuiltinDefaultClassConstructor = constructorKind != ConstructorKind::None && constructorKind != ConstructorKind::Naked;
    UnlinkedFunctionKind kind = isBuiltinDefaultClassConstructor ? UnlinkedNormalFunction : UnlinkedBuiltinFunction;

    SourceParseMode parseMode = scanned.isAsyncFunction ? SourceParseMode::AsyncFunctionMode : SourceParseMode::NormalFunctionMode;

    JSTokenLocation start;
    start.startOffset = source.startOffset() + parametersStart;
    start.endOffset = std::numeric_limits<unsigned>::max();

    JSTokenLocation end;
    end.startOffset = source.startOffset() + strlen("(");
    end.endOffset = std::numeric_limits<unsigned>::max();

    FunctionMetadataNode metadata(
        start, end, source.startOffset() + functionKeywordStart, source.startOffset() + functionNameStart, source.startOffset() + parametersStart, implementationVisibility,
        scanned.isInStrictContext ? StrictModeLexicallyScopedFeature : NoLexicallyScopedFeatures, constructorKind, constructorKind == ConstructorKind::Extends ? SuperBinding::Needed : SuperBinding::NotNeeded,
        scanned.parameterCount, parseMode, isArrowFunctionBodyExpression);

    metadata.finishParsing(newSource, Identifier(), FunctionMode::FunctionExpression);
    metadata.overrideName(name);

    if (ASSERT_ENABLED || Options::validateBytecode()) [[unlikely]] {
        ParserError error;
        JSParserBuiltinMode builtinMode = isBuiltinDefaultClassConstructor ? JSParserBuiltinMode::NotBuiltin : JSParserBuiltinMode::Builtin;
        std::unique_ptr<ProgramNode> program = parseRootNode<ProgramNode>(
            vm, source, implementationVisibility, builtinMode,
            NoLexicallyScopedFeatures, JSParserScriptMode::Classic, SourceParseMode::ProgramMode, error,
            constructorKind);

        if (program) {
            StatementNode* exprStatement = program->singleStatement();
            RELEASE_ASSERT(exprStatement);
            RELEASE_ASSERT(exprStatement->isExprStatement());
            ExpressionNode* funcExpr = static_cast<ExprStatementNode*>(exprStatement)->expr();
            RELEASE_ASSERT(funcExpr);
            RELEASE_ASSERT(funcExpr->isFuncExprNode());
            FunctionMetadataNode* metadataFromParser = static_cast<FuncExprNode*>(funcExpr)->metadata();
            RELEASE_ASSERT(!program->hasCapturedVariables());
            
            RELEASE_ASSERT(metadataFromParser);
            RELEASE_ASSERT(metadataFromParser->ident().isNull());
            
            // This function assumes an input string that would result in a single anonymous function expression.
            RELEASE_ASSERT(metadataFromParser);
            metadataFromParser->overrideName(name);
            if (metadata != *metadataFromParser) {
                dataLogLn("Expected Metadata:\n", metadata);
                dataLogLn("Metadata from parser:\n", *metadataFromParser);
                WTFLogAlways("Metadata of parser and hand rolled parser don't match\n");
                CRASH();
            }
        } else {
            RELEASE_ASSERT(error.isValid());
            RELEASE_ASSERT(error.type() == ParserError::StackOverflow);
        }
    }

    UnlinkedFunctionExecutable* functionExecutable = UnlinkedFunctionExecutable::create(vm, source, &metadata, kind, constructAbility, inlineAttribute, JSParserScriptMode::Classic, nullptr, { }, std::nullopt, DerivedContextType::None, EvalContextType::FunctionEvalContext, needsClassFieldInitializer, privateBrandRequirement, isBuiltinDefaultClassConstructor);
    return functionExecutable;
}

template<typename Visitor>
void BuiltinExecutables::visitAggregateImpl(Visitor& visitor)
{
    for (auto*& unlinkedExecutable : m_unlinkedExecutables) {
        if (unlinkedExecutable)
            visitor.appendUnbarriered(unlinkedExecutable);
    }
}

DEFINE_VISIT_AGGREGATE(BuiltinExecutables);

void BuiltinExecutables::clear()
{
    std::ranges::fill(m_unlinkedExecutables, nullptr);
}

#define DEFINE_BUILTIN_EXECUTABLES(name, functionName, overrideName, length) \
SourceCode BuiltinExecutables::name##Source() \
{\
    return SourceCode { m_combinedSourceProvider.copyRef(), static_cast<int>(s_##name - s_JSCCombinedCode), static_cast<int>((s_##name - s_JSCCombinedCode) + length) };\
}\
\
UnlinkedFunctionExecutable* BuiltinExecutables::name##Executable() \
{\
    unsigned index = static_cast<unsigned>(BuiltinCodeIndex::name);\
    if (!m_unlinkedExecutables[index]) {\
        Identifier executableName = m_vm.propertyNames->builtinNames().functionName##PublicName();\
        if (overrideName)\
            executableName = Identifier::fromString(m_vm, overrideName);\
        m_unlinkedExecutables[index] = createBuiltinExecutable(name##Source(), s_JSCBuiltinSourceMetadata[index], executableName, s_##name##ImplementationVisibility, s_##name##ConstructorKind, s_##name##ConstructAbility, s_##name##InlineAttribute);\
    }\
    return m_unlinkedExecutables[index];\
}
JSC_FOREACH_BUILTIN_CODE(DEFINE_BUILTIN_EXECUTABLES)
#undef DEFINE_BUILTIN_EXECUTABLES

}

WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
