/*
 * Copyright (C) 2013-2023 Apple Inc. All rights reserved.
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
#include "DFGInPlaceAbstractState.h"

#if ENABLE(DFG_JIT)

#include "DFGBasicBlock.h"
#include "JSCJSValueInlines.h"
#include <wtf/TZoneMallocInlines.h>

#include "BitRangeClearSwiftBoundary.h"
#include "DFGCFAMergeSwiftBoundary.h"

namespace JSC { namespace DFG {

WTF_MAKE_SEQUESTERED_ARENA_ALLOCATED_IMPL(InPlaceAbstractState);

namespace DFGInPlaceAbstractStateInternal {
static constexpr bool verbose = false;
}

InPlaceAbstractState::InPlaceAbstractState(Graph& graph)
    : m_graph(graph)
    , m_abstractValues(*graph.m_abstractValuesCache)
    , m_variables(OperandsLike, graph.block(0)->variablesAtHead)
    , m_tupleAbstractValues(graph.m_tupleData.size())
    , m_block(nullptr)
{
}

InPlaceAbstractState::~InPlaceAbstractState() = default;

void InPlaceAbstractState::beginBasicBlock(BasicBlock* basicBlock)
{
    ASSERT(!m_block);
    
    ASSERT(basicBlock->variablesAtHead.numberOfLocals() == basicBlock->valuesAtHead.numberOfLocals());
    ASSERT(basicBlock->variablesAtTail.numberOfLocals() == basicBlock->valuesAtTail.numberOfLocals());
    ASSERT(basicBlock->variablesAtHead.numberOfLocals() == basicBlock->variablesAtTail.numberOfLocals());
    ASSERT(basicBlock->variablesAtHead.numberOfTmps() == basicBlock->valuesAtHead.numberOfTmps());
    ASSERT(basicBlock->variablesAtTail.numberOfTmps() == basicBlock->valuesAtTail.numberOfTmps());
    ASSERT(basicBlock->variablesAtHead.numberOfTmps() == basicBlock->variablesAtTail.numberOfTmps());

    m_abstractValues.resize();

    AbstractValueClobberEpoch epoch = AbstractValueClobberEpoch::first(basicBlock->cfaStructureClobberStateAtHead);
    m_epochAtHead = epoch;
    m_effectEpoch = epoch;

    m_block = basicBlock;

    // FastBitVector::clearRange, in Swift (BitRangeClear.swift).
    auto activeWords = m_activeVariables.unsafeWords().words();
    JSCSwift_BitRangeClear_clearRange(activeWords.data(), activeWords.size(), m_activeVariables.numBits(), 0, std::min(m_variables.size(), m_activeVariables.size()));
    if (m_variables.size() > m_activeVariables.size())
        m_activeVariables.resize(m_variables.size());
    
    if (m_graph.m_form == SSA) {
        for (NodeAbstractValuePair& entry : basicBlock->ssa->valuesAtHead) {
            if (entry.node.isStillValid()) {
                AbstractValue& value = m_abstractValues.at(entry.node);
                value = entry.value;
                value.m_effectEpoch = epoch;
            }
        }
    }
    basicBlock->cfaShouldRevisit = false;
    basicBlock->cfaHasVisited = true;
    m_isValid = true;
    m_branchDirection = InvalidBranchDirection;
    m_structureClobberState = basicBlock->cfaStructureClobberStateAtHead;
}

static void setLiveValues(Vector<NodeAbstractValuePair>& values, const Vector<NodeFlowProjection>& live)
{
    values = live.map([](auto node) {
        return NodeAbstractValuePair { node, AbstractValue() };
    });
}

Operands<AbstractValue>& InPlaceAbstractState::variablesForDebugging()
{
    activateAllVariables();
    return m_variables;
}

void InPlaceAbstractState::activateAllVariables()
{
    for (size_t i = m_variables.size(); i--;)
        activateVariableIfNecessary(i);
}

static uint64_t* abstractValueWords(Operands<AbstractValue>&);

// The value resets of initialize() for one block, done by Swift (DFGCFAMerge.swift) on the values' words: every value at
// the block's head and tail becomes clear (AbstractValue::clear()), except that the first argumentFormats.size() values
// at head (a ThreadedCPS root's arguments) start out as their FlushFormat says. Swift writes out the words of the fat
// structure sets it cleared, and this frees their lists, since C++ owns them. Returns false, changing nothing, for a bad
// flush format.
ALWAYS_INLINE static bool resetBlockValues(BasicBlock& block, std::span<const FlushFormat> argumentFormats, Vector<uint64_t, 24>& freedLists)
{
    // The constants DFGCFAMerge.swift uses for the arguments: setNonCellType(SpecInt32Only), setNonCellType(SpecBoolean),
    // setType(graph, SpecCellCheck) (a cell type that is neither only strings nor only symbols, so TOP structures) and
    // makeBytecodeTop().
    static_assert(FlushedInt32 == 1 && FlushedCell == 4 && FlushedBoolean == 5 && FlushedJSValue == 6 && sizeof(FlushFormat) == 1);
    static_assert(SpecInt32Only == 0x60'0000'0000 && SpecBoolean == 0x2000'0000'0000 && !((SpecInt32Only | SpecBoolean) & SpecCell));
    static_assert(SpecCellCheck == 0x3'801f'ffff'fff7 && ((SpecCellCheck & SpecCell) & ~SpecString) && (SpecCellCheck & SpecCell) != SpecSymbol);
    static_assert(SpecBytecodeTop == 0x3'ee7f'ffff'fff7 && ALL_ARRAY_MODES == 0xfeff'3ffb);
    ASSERT(block.valuesAtHead.numberOfArguments() == block.valuesAtTail.numberOfArguments());
    ASSERT(block.valuesAtHead.numberOfLocals() == block.valuesAtTail.numberOfLocals());
    size_t count = block.valuesAtHead.size();
    if (count > freedLists.size() / 2) [[unlikely]]
        freedLists.grow(2 * count);
    ptrdiff_t freedCount = JSCSwift_DFGCFAMerge_resetValues(abstractValueWords(block.valuesAtHead), count, abstractValueWords(block.valuesAtTail), block.valuesAtTail.size(),
        std::bit_cast<const uint8_t*>(argumentFormats.data()), argumentFormats.size(), freedLists.mutableSpan().data(), freedLists.size());
    if (freedCount < 0) [[unlikely]]
        return false;
    for (uint64_t word : freedLists.span().first(static_cast<size_t>(freedCount)))
        StructureAbstractValue::destroyListForSwift(word);
    return true;
}

void InPlaceAbstractState::initialize()
{
    for (BasicBlock* entrypoint : m_graph.m_roots) {
        entrypoint->cfaShouldRevisit = true;
        entrypoint->cfaHasVisited = false;
        entrypoint->cfaStructureClobberStateAtHead = StructuresAreWatched;
        entrypoint->cfaStructureClobberStateAtTail = StructuresAreWatched;

        if (m_graph.m_form == SSA) {
            bool ok = resetBlockValues(*entrypoint, { }, m_structureListScratch);
            ASSERT_UNUSED(ok, ok);
            continue;
        }
        const ArgumentsVector& arguments = m_graph.m_rootToArguments.find(entrypoint)->value;
        Vector<FlushFormat, 16> formats(entrypoint->valuesAtHead.numberOfArguments(), [&](size_t i) {
            Node* node = arguments[i];
            if (!node)
                return FlushedJSValue;
            ASSERT(node->op() == SetArgumentDefinitely);
            return node->variableAccessData()->flushFormat();
        });
        if (!resetBlockValues(*entrypoint, formats.span(), m_structureListScratch))
            DFG_CRASH(m_graph, nullptr, "Bad flush format for argument"_s);
    }

    for (BasicBlock* block : m_graph.blocksInNaturalOrder()) {
        if (m_graph.isRoot(block)) {
            // We bootstrapped the CFG roots above.
            continue;
        }

        ASSERT(block->isReachable);
        block->cfaShouldRevisit = false;
        block->cfaHasVisited = false;
        block->cfaStructureClobberStateAtHead = StructuresAreWatched;
        block->cfaStructureClobberStateAtTail = StructuresAreWatched;
        bool ok = resetBlockValues(*block, { }, m_structureListScratch);
        ASSERT_UNUSED(ok, ok);
    }

    if (m_graph.m_form == SSA) {
        for (BlockIndex blockIndex = 0; blockIndex < m_graph.numBlocks(); ++blockIndex) {
            BasicBlock* block = m_graph.block(blockIndex);
            if (!block)
                continue;
            setLiveValues(block->ssa->valuesAtHead, block->ssa->liveAtHead);
            setLiveValues(block->ssa->valuesAtTail, block->ssa->liveAtTail);
        }
    }
}

bool InPlaceAbstractState::endBasicBlock()
{
    ASSERT(m_block);
    
    BasicBlock* block = m_block; // Save the block for successor merging.

    block->cfaDidFinish = m_isValid;
    block->cfaBranchDirection = m_branchDirection;
    
    if (!m_isValid) {
        reset();
        return false;
    }

    AbstractValueClobberEpoch epochAtHead = m_epochAtHead;
    AbstractValueClobberEpoch currentEpoch = m_effectEpoch;

    block->cfaStructureClobberStateAtTail = m_structureClobberState;
    
    switch (m_graph.m_form) {
    case ThreadedCPS: {
        ASSERT(block->variablesAtTail.size() == block->valuesAtTail.size());
        ASSERT(block->variablesAtTail.size() == m_variables.size());
        for (size_t index = m_variables.size(); index--;) {
            Node* node = block->variablesAtTail[index];
            if (!node)
                continue;
            AbstractValue& destination = block->valuesAtTail[index];
            
            if (!m_activeVariables[index]) {
                destination = block->valuesAtHead[index];
                destination.fastForwardFromTo(m_graph, epochAtHead, currentEpoch);
                continue;
            }
            
            switch (node->op()) {
            case Phi:
            case SetArgumentDefinitely:
            case SetArgumentMaybe:
            case PhantomLocal:
            case Flush: {
                // The block transfers the value from head to tail.
                destination = atIndex(index);
                break;
            }
                
            case GetLocal: {
                // The block refines the value with additional speculations.
                destination = forNode(node);

                // We need to make sure that we don't broaden the type beyond what the flush
                // format says it will be. The value may claim to have changed abstract state
                // but it's type cannot change without a store. For example:
                //
                // Block #1:
                // 0: GetLocal(loc42, FlushFormatInt32)
                // 1: PutStructure(Check: Cell: @0, ArrayStructure)
                // ...
                // 2: Branch(T: #1, F: #2)
                //
                // In this case the AbstractState of @0 will say it's an SpecArray but the only
                // reason that would have happened is because we would have exited the cell check.

                FlushFormat flushFormat = node->variableAccessData()->flushFormat();
                destination.filter(typeFilterFor(flushFormat));
                break;
            }
            case SetLocal: {
                // The block sets the variable, and potentially refines it, both
                // before and after setting it. Since the SetLocal already did
                // a type check based on the flush format's type, we're only interested
                // in refinements within that type hierarchy. Otherwise, we may end up
                // saying that any GetLocals reachable from this basic block load something
                // outside of that hierarchy, e.g:
                //
                // a: JSConstant(jsNumber(0))
                // b: SetLocal(Int32:@a, loc1, FlushedInt32)
                // c: ArrayifyToStructure(Cell:@a)
                // d: Jump(...)
                //
                // In this example, we can't trust whatever type ArrayifyToStructure sets
                // @a to. We're only interested in the subset of that type that intersects
                // with Int32.
                AbstractValue value = forNode(node->child1());
                value.filter(typeFilterFor(node->variableAccessData()->flushFormat()));
                destination = value;
                break;
            }
                
            default:
                RELEASE_ASSERT_NOT_REACHED();
                break;
            }
        }
        break;
    }

    case SSA: {
        for (size_t i = 0; i < block->valuesAtTail.size(); ++i) {
            AbstractValue& destination = block->valuesAtTail[i];

            if (!m_activeVariables[i]) {
                destination = block->valuesAtHead[i];
                destination.fastForwardFromTo(m_graph, epochAtHead, currentEpoch);
                continue;
            }
            
            block->valuesAtTail[i] = atIndex(i);
        }

        for (NodeAbstractValuePair& valueAtTail : block->ssa->valuesAtTail) {
            NodeFlowProjection& node = valueAtTail.node;
            if (node->isTuple()) {
                ASSERT(hasClearedAbstractState(node));
                valueAtTail.value = AbstractValue();
                continue;
            }
            valueAtTail.value = forNode(node);
        }
        break;
    }

    default:
        RELEASE_ASSERT_NOT_REACHED();
    }

    reset();
    
    return mergeToSuccessors(block);
}

void InPlaceAbstractState::reset()
{
    m_block = nullptr;
    m_isValid = false;
    m_branchDirection = InvalidBranchDirection;
    m_structureClobberState = StructuresAreWatched;
}

void InPlaceAbstractState::activateVariable(size_t variableIndex)
{
    AbstractValue& value = m_variables[variableIndex];
    value = m_block->valuesAtHead[variableIndex];
    value.m_effectEpoch = m_epochAtHead;
    m_activeVariables[variableIndex] = true;
}

// An Operands<AbstractValue> as DFGCFAMerge.swift sees it: four 64-bit words per element (the layout is
// static_asserted in DFGAbstractValue.h), null when empty. An Operands<Node*> is one word per element.
static uint64_t* abstractValueWords(Operands<AbstractValue>& values)
{
    return values.size() ? std::bit_cast<uint64_t*>(&values[0]) : nullptr;
}

static const uint64_t* nodeWords(const Operands<Node*>& nodes)
{
    return nodes.size() ? std::bit_cast<const uint64_t*>(&nodes[0]) : nullptr;
}

// destination.merge(source) for a value whose merge reads, allocates or frees a structure list (a fat set on either
// side, or two different single structures): Swift does the merge on flat copies of the two sets' lists and says what
// the destination's set becomes; this stores that, since C++ owns the lists.
static bool mergeValueWithStructureLists(AbstractValue& destination, const AbstractValue& source, Vector<uint64_t, 24>& scratch)
{
    auto destinationList = destination.m_structure.listForSwift();
    auto sourceList = source.m_structure.listForSwift();
    // Room for the result's elements (Swift writes them before it reads them).
    if (size_t needed = destinationList.size() + sourceList.size() + 2; needed > scratch.size()) [[unlikely]]
        scratch.grow(needed);
    std::span<uint64_t> list = scratch.mutableSpan();
    std::array<uint64_t, 3> result;
    bool changed = JSCSwift_DFGCFAMerge_mergeSlowValue(std::bit_cast<uint64_t*>(&destination), std::bit_cast<const uint64_t*>(&source),
        std::bit_cast<const uint64_t*>(destinationList.data()), destinationList.size(), destination.m_structure.listCapacityForSwift(),
        std::bit_cast<const uint64_t*>(sourceList.data()), sourceList.size(), list.data(), list.size(), result.data());
    uint32_t kind = static_cast<uint32_t>(result[0]);
    switch (kind) {
    case 0:
        break;
    case 1:
        destination.m_structure.setWordForSwift(static_cast<uintptr_t>(result[1]));
        break;
    case 2:
        RELEASE_ASSERT(result[2] <= list.size());
        destination.m_structure.appendToListForSwift(list.first(result[2]), result[1]);
        break;
    case 3:
        RELEASE_ASSERT(result[2] <= list.size());
        destination.m_structure.setNewListForSwift(list.first(result[2]), static_cast<unsigned>(result[0] >> 32), result[1]);
        break;
    case 4:
        destination.m_structure = source.m_structure;
        break;
    default:
        RELEASE_ASSERT_NOT_REACHED();
    }
    return changed;
}

// The per-variable part of merge(from, to) runs in Swift (DFGCFAMerge.swift): from->valuesAtTail[i] into
// to->valuesAtHead[i] for every variable (in ThreadedCPS form only those with a to->variablesAtHead node).
bool InPlaceAbstractState::mergeValues(BasicBlock* from, BasicBlock* to)
{
    bool isCPS = m_graph.m_form == ThreadedCPS;
    size_t count = isCPS ? from->variablesAtTail.size() : from->valuesAtTail.size();
    if (m_slowMergeIndices.size() < count)
        m_slowMergeIndices.grow(count);
    size_t slowCount = 0;
    bool changed = JSCSwift_DFGCFAMerge_mergeValues(abstractValueWords(to->valuesAtHead), to->valuesAtHead.size(), abstractValueWords(from->valuesAtTail), from->valuesAtTail.size(),
        isCPS ? nodeWords(to->variablesAtHead) : nullptr, isCPS ? to->variablesAtHead.size() : 0, isCPS, count, m_slowMergeIndices.mutableSpan().data(), m_slowMergeIndices.size(), &slowCount);
    // The values whose merge needs a structure list: with bit 31 set, Swift merged the other words and the set
    // becomes a copy of the source's (*this = other); otherwise the value is untouched.
    RELEASE_ASSERT(slowCount <= m_slowMergeIndices.size());
    for (uint32_t entry : m_slowMergeIndices.span().first(slowCount)) {
        uint32_t index = entry & ~(1u << 31);
        RELEASE_ASSERT(index < count);
        if (entry & (1u << 31)) {
            // *this = other (Swift copied the other words): copy the structure set.
            to->valuesAtHead[index].m_structure = from->valuesAtTail[index].m_structure;
            continue;
        }
        changed |= mergeValueWithStructureLists(to->valuesAtHead[index], from->valuesAtTail[index], m_structureListScratch);
    }
    return changed;
}

bool InPlaceAbstractState::merge(BasicBlock* from, BasicBlock* to)
{
    dataLogLnIf(DFGInPlaceAbstractStateInternal::verbose, "   Merging from ", pointerDump(from), " to ", pointerDump(to));
    ASSERT(from->variablesAtTail.numberOfArguments() == to->variablesAtHead.numberOfArguments());
    ASSERT(from->variablesAtTail.numberOfLocals() == to->variablesAtHead.numberOfLocals());
    ASSERT(from->variablesAtTail.numberOfTmps() == to->variablesAtHead.numberOfTmps());
    
    bool changed = false;
    
    changed |= checkAndSet(
        to->cfaStructureClobberStateAtHead,
        DFG::merge(from->cfaStructureClobberStateAtTail, to->cfaStructureClobberStateAtHead));
    
    switch (m_graph.m_form) {
    case ThreadedCPS: {
        changed |= mergeValues(from, to);
        break;
    }
        
    case SSA: {
        changed |= mergeValues(from, to);

        for (NodeAbstractValuePair& entry : to->ssa->valuesAtHead) {
            NodeFlowProjection node = entry.node;
            dataLogLnIf(DFGInPlaceAbstractStateInternal::verbose, "      Merging for ", node, ": from ", forNode(node), " to ", entry.value);
#ifndef NDEBUG
            unsigned valueCountInFromBlock = 0;
            for (NodeAbstractValuePair& fromBlockValueAtTail : from->ssa->valuesAtTail) {
                if (fromBlockValueAtTail.node == node) {
                    if (node->isTuple())
                        ASSERT(hasClearedAbstractState(node) && !fromBlockValueAtTail.value);
                    else
                        ASSERT(fromBlockValueAtTail.value == forNode(node));
                    ++valueCountInFromBlock;
                }
            }
            ASSERT(valueCountInFromBlock == 1);
#endif

            if (!node->isTuple())
                changed |= entry.value.merge(forNode(node));

            dataLogLnIf(DFGInPlaceAbstractStateInternal::verbose, "         Result: ", entry.value);
        }
        break;
    }
        
    default:
        RELEASE_ASSERT_NOT_REACHED();
        break;
    }

    if (!to->cfaHasVisited)
        changed = true;
    
    dataLogLnIf(DFGInPlaceAbstractStateInternal::verbose, "      Will revisit: ", changed);
    to->cfaShouldRevisit |= changed;
    
    return changed;
}

inline bool InPlaceAbstractState::mergeToSuccessors(BasicBlock* basicBlock)
{
    Node* terminal = basicBlock->terminal();
    
    ASSERT(terminal->isTerminal());
    
    switch (terminal->op()) {
    case Jump: {
        ASSERT(basicBlock->cfaBranchDirection == InvalidBranchDirection);
        return merge(basicBlock, terminal->targetBlock());
    }
        
    case Branch: {
        ASSERT(basicBlock->cfaBranchDirection != InvalidBranchDirection);
        bool changed = false;
        if (basicBlock->cfaBranchDirection != TakeFalse)
            changed |= merge(basicBlock, terminal->branchData()->taken.block);
        if (basicBlock->cfaBranchDirection != TakeTrue)
            changed |= merge(basicBlock, terminal->branchData()->notTaken.block);
        return changed;
    }
        
    case Switch: {
        // FIXME: It would be cool to be sparse conditional for Switch's. Currently
        // we're not. However I somehow doubt that this will ever be a big deal.
        ASSERT(basicBlock->cfaBranchDirection == InvalidBranchDirection);
        SwitchData* data = terminal->switchData();
        bool changed = merge(basicBlock, data->fallThrough.block);
        for (unsigned i = data->cases.size(); i--;)
            changed |= merge(basicBlock, data->cases[i].target.block);
        return changed;
    }
    
    case EntrySwitch: {
        EntrySwitchData* data = terminal->entrySwitchData();
        bool changed = false;
        for (unsigned i = data->cases.size(); i--;)
            changed |= merge(basicBlock, data->cases[i]);
        return changed;
    }

    case Return:
    case TailCall:
    case DirectTailCall:
    case TailCallVarargs:
    case TailCallForwardVarargs:
    case Unreachable:
    case Throw:
    case ThrowStaticError:
        ASSERT(basicBlock->cfaBranchDirection == InvalidBranchDirection);
        return false;

    default:
        RELEASE_ASSERT_NOT_REACHED();
        return false;
    }
}

} } // namespace JSC::DFG

#endif // ENABLE(DFG_JIT)

