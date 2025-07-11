/*
 * Copyright (C) 2020-2022 Apple Inc. All rights reserved.
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
#include "JITOperationList.h"

#include "Disassembler.h"
#include "Gate.h"
#include "JITOperationValidation.h"
#include "LLIntData.h"
#include "Opcode.h"
#include "Options.h"

#if HAVE(DLADDR)
#include <cxxabi.h>
#include <dlfcn.h>
#endif

WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN

namespace JSC {

#if ENABLE(JIT_OPERATION_VALIDATION) || ENABLE(JIT_OPERATION_DISASSEMBLY)

LazyNeverDestroyed<JITOperationList> jitOperationList;

extern const JITOperationAnnotation startOfJITOperationsInJSC __asm("section$start$__DATA_CONST$__jsc_ops");
extern const JITOperationAnnotation endOfJITOperationsInJSC __asm("section$end$__DATA_CONST$__jsc_ops");

void JITOperationList::initialize()
{
    jitOperationList.construct();
}

#if ENABLE(JIT_OPERATION_VALIDATION)

#if ENABLE(JIT_OPERATION_VALIDATION_ASSERT)
void JITOperationList::addInverseMap(void* validationEntry, void* pointer)
{
    m_validatedOperationsInverseMap.add(validationEntry, pointer);
}

#define JSC_REGISTER_INVERSE_JIT_CAGED_POINTER_FOR_DEBUG(validationEntry, pointer) \
    addInverseMap(validationEntry, pointer)
#else
#define JSC_REGISTER_INVERSE_JIT_CAGED_POINTER_FOR_DEBUG(validationEntry, pointer)
#endif // ENABLE(JIT_OPERATION_VALIDATION_ASSERT)

SUPPRESS_ASAN ALWAYS_INLINE void JITOperationList::addPointers(const JITOperationAnnotation* begin, const JITOperationAnnotation* end)
{
    auto& map = m_validatedOperations;
#if ENABLE(JIT_CAGE)
    if (Options::useJITCage()) {
        JSC_JIT_CAGED_POINTER_REGISTRATION();
        return;
    }
#endif
#if ENABLE(JIT_OPERATION_VALIDATION_ASSERT)
        for (const auto* current = begin; current != end; ++current) {
            void* operation = removeCodePtrTag(current->operation);
            if (operation) {
                void* validator = removeCodePtrTag(Options::useJITCage() ? current->operationWithValidation : current->operation);
                validator = WTF::tagNativeCodePtrImpl<OperationPtrTag>(validator);
                map.add(operation, validator);
                JSC_REGISTER_INVERSE_JIT_CAGED_POINTER_FOR_DEBUG(validator, operation);
            }
        }
#endif
}

void JITOperationList::populatePointersInJavaScriptCore()
{
    static std::once_flag onceKey;
    std::call_once(onceKey, [] {
        if (Options::useJIT())
            jitOperationList->addPointers(&startOfJITOperationsInJSC, &endOfJITOperationsInJSC);
#if ENABLE(JIT_OPERATION_DISASSEMBLY)
        if (Options::needDisassemblySupport()) [[unlikely]]
            populateDisassemblyLabelsInJavaScriptCore();
#endif
    });
}

#endif // ENABLE(JIT_OPERATION_VALIDATION)

LLINT_DECLARE_ROUTINE_VALIDATE(llint_function_for_call_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_function_for_construct_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_function_for_call_arity_check);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_function_for_construct_arity_check);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_eval_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_program_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_module_program_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_function_prologue_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_function_prologue);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_function_prologue_simd_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_function_prologue_simd);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_throw_during_call_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(llint_handle_uncaught_exception);
LLINT_DECLARE_ROUTINE_VALIDATE(checkpoint_osr_exit_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(checkpoint_osr_exit_from_inlined_call_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(normal_osr_exit_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(fuzzer_return_early_from_loop_hint);
LLINT_DECLARE_ROUTINE_VALIDATE(js_to_wasm_wrapper_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_to_wasm_wrapper_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_to_wasm_ipint_wrapper_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(wasm_to_js_wrapper_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_function_prologue_simd_trampoline);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_function_prologue_simd);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_catch_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_catch_all_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_table_catch_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_table_catch_ref_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_table_catch_all_entry);
LLINT_DECLARE_ROUTINE_VALIDATE(ipint_table_catch_allref_entry);

#if ENABLE(JIT_OPERATION_VALIDATION)
#define LLINT_OP_EXTRAS(validateLabel) reinterpret_cast<void*>(validateLabel)
#else // ENABLE(JIT_OPERATION_DISASSEMBLY)
#define LLINT_OP_EXTRAS(validateLabel)
#endif

#define LLINT_ROUTINE(functionName) { \
        std::bit_cast<void*>(LLInt::getCodeFunctionPtr<CFunctionPtrTag>(functionName)), \
        LLINT_OP_EXTRAS(LLINT_ROUTINE_VALIDATE(functionName)) \
    },

#define LLINT_OP(name) { \
        std::bit_cast<void*>(LLInt::getCodeFunctionPtr<CFunctionPtrTag>(name)), \
        LLINT_OP_EXTRAS(LLINT_RETURN_VALIDATE(name)) \
    }, { \
        std::bit_cast<void*>(LLInt::getWide16CodeFunctionPtr<CFunctionPtrTag>(name)), \
        LLINT_OP_EXTRAS(LLINT_RETURN_WIDE16_VALIDATE(name)) \
    }, { \
        std::bit_cast<void*>(LLInt::getWide32CodeFunctionPtr<CFunctionPtrTag>(name)), \
        LLINT_OP_EXTRAS(LLINT_RETURN_WIDE32_VALIDATE(name)) \
    },

#define LLINT_RETURN_LOCATION(name, ...) \
    LLINT_OP(name##_return_location)

struct LLIntOperations {
    const JITOperationAnnotation* operations;
    size_t numberOfOperations;
};

static LLIntOperations llintOperations()
{
    static const JITOperationAnnotation* operations = nullptr;
    static size_t numberOfOperations = 0;
    static std::once_flag onceKey;
    std::call_once(onceKey, [&] {
        static const JITOperationAnnotation operationsStorage[] = {
            LLINT_ROUTINE(llint_function_for_call_prologue)
            LLINT_ROUTINE(llint_function_for_construct_prologue)
            LLINT_ROUTINE(llint_function_for_call_arity_check)
            LLINT_ROUTINE(llint_function_for_construct_arity_check)
            LLINT_ROUTINE(llint_eval_prologue)
            LLINT_ROUTINE(llint_program_prologue)
            LLINT_ROUTINE(llint_module_program_prologue)
            LLINT_ROUTINE(wasm_function_prologue_trampoline)
            LLINT_ROUTINE(wasm_function_prologue)
            LLINT_ROUTINE(wasm_function_prologue_simd_trampoline)
            LLINT_ROUTINE(wasm_function_prologue_simd)
            LLINT_ROUTINE(llint_throw_during_call_trampoline)
            LLINT_ROUTINE(llint_handle_uncaught_exception)
            LLINT_ROUTINE(checkpoint_osr_exit_trampoline)
            LLINT_ROUTINE(checkpoint_osr_exit_from_inlined_call_trampoline)
            LLINT_ROUTINE(normal_osr_exit_trampoline)
            LLINT_ROUTINE(fuzzer_return_early_from_loop_hint)
            LLINT_ROUTINE(js_to_wasm_wrapper_entry)
            LLINT_ROUTINE(wasm_to_wasm_wrapper_entry)
            LLINT_ROUTINE(wasm_to_wasm_ipint_wrapper_entry)
            LLINT_ROUTINE(wasm_to_js_wrapper_entry)
            LLINT_ROUTINE(ipint_trampoline)
            LLINT_ROUTINE(ipint_entry)
            LLINT_ROUTINE(ipint_function_prologue_simd_trampoline)
            LLINT_ROUTINE(ipint_function_prologue_simd)
            LLINT_ROUTINE(ipint_catch_entry)
            LLINT_ROUTINE(ipint_catch_all_entry)
            LLINT_ROUTINE(ipint_table_catch_entry)
            LLINT_ROUTINE(ipint_table_catch_ref_entry)
            LLINT_ROUTINE(ipint_table_catch_all_entry)
            LLINT_ROUTINE(ipint_table_catch_allref_entry)

            LLINT_OP(op_catch)
            LLINT_OP(wasm_catch)
            LLINT_OP(wasm_catch_all)
            LLINT_OP(wasm_try_table_catch)
            LLINT_OP(wasm_try_table_catchref)
            LLINT_OP(wasm_try_table_catchall)
            LLINT_OP(wasm_try_table_catchallref)
            LLINT_OP(llint_generic_return_point)

            FOR_EACH_LLINT_OPCODE_WITH_RETURN(LLINT_RETURN_LOCATION)

            JSC_JS_GATE_OPCODES(LLINT_RETURN_LOCATION)
            JSC_WASM_GATE_OPCODES(LLINT_RETURN_LOCATION)
        };
        operations = operationsStorage;
        numberOfOperations = std::size(operationsStorage);
    });
    return { operations, numberOfOperations };
}

#undef LLINT_ROUTINE
#undef LLINT_OP
#undef LLINT_RETURN_LOCATION
#undef LLINT_OP_EXTRAS

#if ENABLE(JIT_OPERATION_VALIDATION)

void JITOperationList::populatePointersInJavaScriptCoreForLLInt()
{
    static std::once_flag onceKey;
    std::call_once(onceKey, [] {
        if (Options::useJIT()) {
            auto list = llintOperations();
            jitOperationList->addPointers(list.operations, list.operations + list.numberOfOperations);
        }
#if ENABLE(JIT_OPERATION_DISASSEMBLY)
        if (Options::needDisassemblySupport()) [[unlikely]]
            JITOperationList::populateDisassemblyLabelsInJavaScriptCoreForLLInt();
#endif
    });
}

void JITOperationList::populatePointersInEmbedder(const JITOperationAnnotation* beginOperations, const JITOperationAnnotation* endOperations)
{
    if (Options::useJIT())
        jitOperationList->addPointers(beginOperations, endOperations);
}

#endif // ENABLE(JIT_OPERATION_VALIDATION)

#if ENABLE(JIT_OPERATION_DISASSEMBLY)

SUPPRESS_ASAN void JITOperationList::addDisassemblyLabels(const JITOperationAnnotation* begin, const JITOperationAnnotation* end)
{
    RELEASE_ASSERT(Options::needDisassemblySupport());
    for (const auto* current = begin; current != end; ++current) {
#if ENABLE(JIT_OPERATION_VALIDATION)
        auto* operation = current->operationWithValidation;
#else
        auto* operation = current->operation;
#endif
        Dl_info info;
        if (dladdr(operation, &info) && info.dli_sname)
            registerLabel(removeCodePtrTag(operation), info.dli_sname);
    }
}

void JITOperationList::populateDisassemblyLabelsInJavaScriptCore()
{
    ASSERT(Options::needDisassemblySupport());
    static std::once_flag onceKey;
    std::call_once(onceKey, [] {
        if (Options::useJIT())
            addDisassemblyLabels(&startOfJITOperationsInJSC, &endOfJITOperationsInJSC);
    });
}

void JITOperationList::populateDisassemblyLabelsInJavaScriptCoreForLLInt()
{
    ASSERT(Options::needDisassemblySupport());
    static std::once_flag onceKey;
    std::call_once(onceKey, [] {
        if (Options::useJIT()) {
            auto list = llintOperations();
            addDisassemblyLabels(list.operations, list.operations + list.numberOfOperations);
        }
    });
}

void JITOperationList::populateDisassemblyLabelsInEmbedder(const JITOperationAnnotation* beginOperations, const JITOperationAnnotation* endOperations)
{
    if (!Options::needDisassemblySupport()) [[likely]]
        return;
    if (Options::useJIT())
        addDisassemblyLabels(beginOperations, endOperations);
}

#endif // ENABLE(JIT_OPERATION_DISASSEMBLY)

#endif // ENABLE(JIT_OPERATION_VALIDATION) || ENABLE(JIT_OPERATION_DISASSEMBLY)

WTF_ALLOW_UNSAFE_BUFFER_USAGE_END

} // namespace JSC
