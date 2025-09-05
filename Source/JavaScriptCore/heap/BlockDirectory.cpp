/*
 * Copyright (C) 2012-2024 Apple Inc. All rights reserved.
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
#include "BlockDirectory.h"

#include "BlockDirectoryInlines.h"
#include "DeferGCInlines.h"
#include "Heap.h"
#include "MarkedSpaceInlines.h"
#include "SubspaceInlines.h"
#include "SuperSampler.h"

#include <wtf/FunctionTraits.h>
#include <wtf/Lock.h>
#include <wtf/SimpleStats.h>

namespace JSC {

namespace BlockDirectoryInternal {
static constexpr bool verbose = false;
}

DEFINE_ALLOCATOR_WITH_HEAP_IDENTIFIER(BlockDirectory);


BlockDirectory::BlockDirectory(size_t cellSize)
    : m_cellSize(static_cast<unsigned>(cellSize))
{
}

BlockDirectory::~BlockDirectory()
{
    Locker locker { m_localAllocatorsLock };
    while (!m_localAllocators.isEmpty())
        m_localAllocators.begin()->remove();
}

void BlockDirectory::setSubspace(Subspace* subspace)
{
    m_attributes = subspace->attributes();
    m_subspace = subspace;
}

void BlockDirectory::updatePercentageOfPagedOutPages(SimpleStats& stats)
{
    // FIXME: We should figure out a solution for Windows and PlayStation.
    // QNX doesn't have mincore(), though the information can be had. But since all mapped
    // pages are resident, does it matter?
#if OS(UNIX) && !PLATFORM(PLAYSTATION) && !OS(QNX) && !OS(HAIKU)
    size_t pageSize = WTF::pageSize();
    ASSERT(!(MarkedBlock::blockSize % pageSize));
    auto numberOfPagesInMarkedBlock = MarkedBlock::blockSize / pageSize;
    // For some reason this can be unsigned char or char on different OSes...
    using MincoreBufferType = std::remove_pointer_t<FunctionTraits<decltype(mincore)>::ArgumentType<2>>;
    static_assert(std::is_same_v<std::make_unsigned_t<MincoreBufferType>, unsigned char>);
    Vector<MincoreBufferType, 16> pagedBits(numberOfPagesInMarkedBlock, MincoreBufferType { });

    for (auto* handle : m_blocks) {
        if (!handle)
            continue;

        auto* pageStart = handle->pageStart();
        auto markedBlockSizeInBytes = handle->backingStorageSize();
        RELEASE_ASSERT(markedBlockSizeInBytes / pageSize <= numberOfPagesInMarkedBlock);
        // We could cache this in bulk (e.g. 25 MB chunks) but we haven't seen any data that it actually matters.
        auto result = mincore(pageStart, markedBlockSizeInBytes, pagedBits.mutableSpan().data());
        RELEASE_ASSERT(!result);
        constexpr unsigned pageIsResidentAndNotCompressed = 1;
        for (unsigned i = 0; i < numberOfPagesInMarkedBlock; ++i)
            stats.add(!(pagedBits[i] & pageIsResidentAndNotCompressed));
    }
#else
    UNUSED_PARAM(stats);
#endif
}

MarkedBlock::Handle* BlockDirectory::findEmptyBlockToSteal()
{
    Locker locker(bitvectorLock());
    m_emptyCursor = (emptyBits() & ~inUseBits()).findBit(m_emptyCursor, true);
    if (m_emptyCursor >= m_blocks.size())
        return nullptr;
    dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", m_emptyCursor, " in use (findEmptyBlockToSteal) for ", *this);
    setIsInUse(m_emptyCursor, true);
    return m_blocks[m_emptyCursor];
}

MarkedBlock::Handle* BlockDirectory::findBlockForAllocation(LocalAllocator& allocator)
{
    Locker locker(bitvectorLock());
    for (;;) {
        allocator.m_allocationCursor = ((canAllocateButNotEmptyBits() | emptyBits()) & ~inUseBits()).findBit(allocator.m_allocationCursor, true);
        if (allocator.m_allocationCursor >= m_blocks.size())
            return nullptr;
        
        unsigned blockIndex = allocator.m_allocationCursor++;
        MarkedBlock::Handle* result = m_blocks[blockIndex];
        setIsCanAllocateButNotEmpty(blockIndex, false); // afryer what if it was empty?!? Oh, sweeping will fix this.
        dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", blockIndex, " in use (findBlockForAllocation) for ", *this);
        setIsInUse(blockIndex, true);
        return result;
    }
}

MarkedBlock::Handle* BlockDirectory::tryAllocateBlock(JSC::Heap& heap)
{
    MarkedBlock::Handle* handle = MarkedBlock::tryCreate(heap, subspace()->alignedMemoryAllocator());
    if (!handle)
        return nullptr;
    
    markedSpace().didAddBlock(handle);
    
    return handle;
}

void BlockDirectory::addBlock(MarkedBlock::Handle* block)
{
    Locker locker { m_bitvectorLock };
    unsigned index;
    if (m_freeBlockIndices.isEmpty()) {
        index = m_blocks.size();

        size_t oldCapacity = m_blocks.capacity();
        m_blocks.append(block);
        if (m_blocks.capacity() != oldCapacity) {
            ASSERT(m_bits.numBits() == oldCapacity);
            ASSERT(m_blocks.capacity() > oldCapacity);
            
            subspace()->didResizeBits(m_blocks.capacity());
            m_bits.resize(m_blocks.capacity());
        }
    } else {
        index = m_freeBlockIndices.takeLast();
        ASSERT(!m_blocks[index]);
        m_blocks[index] = block;
    }
    
    forEachBitVector(
        [&](auto vectorRef) {
            ASSERT_UNUSED(vectorRef, !vectorRef[index]);
        });

    // This is the point at which the block learns of its cellSize() and attributes().
    block->didAddToDirectory(this, index);
    
    setIsLive(index, true);
    setIsEmpty(index, true);
    dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", index, " in use (addBlock) for ", *this);
    setIsInUse(index, true);
}

void BlockDirectory::removeBlock(MarkedBlock::Handle* block, WillDeleteBlock willDelete)
{
    assertIsMutatorOrMutatorIsStopped();
    ASSERT(block->directory() == this);
    ASSERT(m_blocks[block->index()] == block);
    ASSERT(isInUse(block));
    
    subspace()->didRemoveBlock(block->index());
    
    m_blocks[block->index()] = nullptr;
    m_freeBlockIndices.append(block->index());
    
    releaseAssertAcquiredBitVectorLock();
    Locker locker(bitvectorLock());
    forEachBitVector(
        [&](auto vectorRef) {
            vectorRef[block->index()] = false; // Ahhh, this clears all bits
        });

    if (willDelete == WillDeleteBlock::No)
        block->didRemoveFromDirectory();
}

void BlockDirectory::stopAllocating()
{
    dataLogLnIf(BlockDirectoryInternal::verbose, RawPointer(this), ": BlockDirectory::stopAllocating!");
    m_localAllocators.forEach(
        [&] (LocalAllocator* allocator) {
            allocator->stopAllocating();
        });

#if ASSERT_ENABLED
    assertIsMutatorOrMutatorIsStopped();
    if (!inUseBitsView().isEmpty()) [[unlikely]] {
        dataLogLn("Not all inUse bits are clear at stopAllocating");
        dataLogLn(*this);
        dumpBits();
        RELEASE_ASSERT_NOT_REACHED();
    }
#endif
}

void BlockDirectory::prepareForAllocation()
{
    m_localAllocators.forEach(
        [&] (LocalAllocator* allocator) {
            allocator->prepareForAllocation();
        });
    
    m_unsweptCursor = 0;
    m_emptyCursor = 0;
    
    assertSweeperIsSuspended();
    edenBits().clearAll();

    if (Options::useImmortalObjects()) [[unlikely]] {
        // FIXME: Make this work again.
        // https://bugs.webkit.org/show_bug.cgi?id=162296
        RELEASE_ASSERT_NOT_REACHED();
    }
}

void BlockDirectory::stopAllocatingForGood()
{
    dataLogLnIf(BlockDirectoryInternal::verbose, RawPointer(this), ": BlockDirectory::stopAllocatingForGood!");
    
    m_localAllocators.forEach(
        [&] (LocalAllocator* allocator) {
            allocator->stopAllocatingForGood();
        });

    Locker locker { m_localAllocatorsLock };
    while (!m_localAllocators.isEmpty())
        m_localAllocators.begin()->remove();
}

void BlockDirectory::lastChanceToFinalize()
{
    forEachBlock(
        [&] (MarkedBlock::Handle* block) {
            block->lastChanceToFinalize();
        });
}

void BlockDirectory::resumeAllocating()
{
    dataLogLnIf(BlockDirectoryInternal::verbose, RawPointer(this), ": BlockDirectory::resumeAllocating!");
    m_localAllocators.forEach(
        [&] (LocalAllocator* allocator) {
            allocator->resumeAllocating();
        });
}

void BlockDirectory::beginMarkingForFullCollection()
{
    assertSweeperIsSuspended();

    // Mark bits are sticky and so is our summary of mark bits. We only clear these during full
    // collections, so if you survived the last collection you will survive the next one so long
    // as the next one is eden.
    markingNotEmptyBits().clearAll();
    markingRetiredBits().clearAll();
}

void BlockDirectory::endMarking()
{
    assertSweeperIsSuspended();

    allocatedBits().clearAll();
    
#if ASSERT_ENABLED
    if (!inUseBitsView().isEmpty()) [[unlikely]] {
        dataLogLn("Block is inUse at end marking.");
        dataLogLn(*this);
        dumpBits();
        RELEASE_ASSERT_NOT_REACHED();
    }
#endif

    // It's surprising and frustrating to comprehend, but the end-of-marking flip does not need to
    // know what kind of collection it is. That knowledge is already encoded in the m_markingXYZ
    // vectors.
    
    // Sweeper is suspended so we don't need the lock here.
    emptyBits() = liveBits() & ~markingNotEmptyBits() & ~opportunisticallyFreeListedBits();
    canAllocateButNotEmptyBits() = liveBits() & markingNotEmptyBits() & ~markingRetiredBits() & ~opportunisticallyFreeListedBits();

    switch (m_attributes.destruction) {
    case NeedsDestruction: {
        // There are some blocks that we didn't allocate out of in the last cycle, but we swept them. This
        // will forget that we did that and we will end up sweeping them again and attempting to call their
        // destructors again. That's fine because of zapping. The only time when we cannot forget is when
        // we just allocate a block or when we move a block from one size class to another. That doesn't
        // happen here.
        destructibleBits() = liveBits();
        break;
    }

    case MayNeedDestruction: {
        // When this destruction mode is specified, each cell notifies whether this MarkedBlock needs destructor runs conservatively.
        // The bit will be set from the mutator and we use this bit to decide whether we run a destructor.
        // Until we clear the MarkedBlock completely, once this bit is set, this bit is stickily set to the MarkedBlock.
        break;
    }

    case DoesNotNeedDestruction:
        break;
    }

    if (BlockDirectoryInternal::verbose) {
        dataLogLn("Bits for ", m_cellSize, ", ", m_attributes, " after endMarking:");
        dumpBits(WTF::dataFile());
    }
}

void BlockDirectory::snapshotUnsweptForEdenCollection()
{
    assertSweeperIsSuspended();
    unsweptBits() |= edenBits() & ~opportunisticallyFreeListedBits();
}

void BlockDirectory::snapshotUnsweptForFullCollection()
{
    assertSweeperIsSuspended();
    unsweptBits() = liveBits() & ~opportunisticallyFreeListedBits();
}

MarkedBlock::Handle* BlockDirectory::findBlockToSweep(unsigned& unsweptCursor)
{
    Locker locker(bitvectorLock());
    unsweptCursor = (unsweptBits() & ~inUseBits()).findBit(unsweptCursor, true);
    if (unsweptCursor >= m_blocks.size())
        return nullptr;
    dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", unsweptCursor, " in use (findBlockToSweep) for ", *this);
    setIsInUse(unsweptCursor, true);
    return m_blocks[unsweptCursor];
}

bool BlockDirectory::tryOpportunisticSweepOneBlock(VM& vm, bool shouldShrinkOrFree)
{
    if (m_opportunisticallySweptFreeListsVersion != markedSpace().newlyAllocatedVersion() && m_opportunisticallySweptFreeLists.size()) {
        // Unfreelist a block rather than sweeping a block
        DeferGCForAWhile deferGC(vm);
        auto it = m_opportunisticallySweptFreeLists.begin();
        ASSERT(it.get());
        if (!it->value) {
            WTFLogAlways("afryer_tryOpportunisticSweepOneBlock found nullptr %u\n", it->key);
            return true;
        }
        std::unique_ptr<FreeList> freeListToDestroy = m_opportunisticallySweptFreeLists.takeFirst();
        ASSERT(freeListToDestroy);
        freeListToDestroy->unfreelist();
        {
            Locker locker { m_bitvectorLock };
            setIsOpportunisticallyFreeListed(freeListToDestroy->block(), false);
            setIsCanAllocateButNotEmpty(freeListToDestroy->block(), true); // afryer what if it was empty?!? ... I think this just means we won't be able to steal this block.
            setIsUnswept(freeListToDestroy->block(), true);
            m_unsweptCursor = std::min(m_unsweptCursor, freeListToDestroy->block()->index());
        }
        return true;
    }

    auto block = findBlockToSweep();
    if (block) {
        DeferGCForAWhile deferGC(vm);
        bool blockIsFreed = false;
        if (shouldShrinkOrFree) {
            block->sweep(nullptr); // don't opportunistically construct a FreeList because we're going to shrink/free
            if (!block->isEmpty())
                block->shrink();
            else {
                vm.heap.objectSpace().freeBlock(block);
                blockIsFreed = true;
            }
        } else {
            bool didConstructFreeList = opportunisticSweep(block);

            if (didConstructFreeList) {
                // for debugging, let's unfreelist right after free listing and see if we still have crashes
                ASSERT(m_opportunisticallySweptFreeLists.size());
                // Unfreelist a block rather than sweeping a block
                DeferGCForAWhile deferGC(vm);
                auto it = m_opportunisticallySweptFreeLists.begin();
                ASSERT(it.get());
                if (!it->value) {
                    WTFLogAlways("afryer_tryOpportunisticSweepOneBlock found nullptr %u\n", it->key);
                    ASSERT(false);
                }
                std::unique_ptr<FreeList> freeListToDestroy = m_opportunisticallySweptFreeLists.takeFirst();
                ASSERT(freeListToDestroy);
                freeListToDestroy->unfreelist();
                {
                    Locker locker { m_bitvectorLock };
                    setIsOpportunisticallyFreeListed(freeListToDestroy->block(), false);
                    setIsCanAllocateButNotEmpty(freeListToDestroy->block(), true); // afryer what if it was empty?!? ... I think this just means we won't be able to steal this block.
                    setIsUnswept(freeListToDestroy->block(), true);
                    m_unsweptCursor = std::min(m_unsweptCursor, freeListToDestroy->block()->index());
                }
            }
        }

        if (!blockIsFreed)
            didFinishUsingBlock(block);
        return true;
    }
    return false;
}

bool BlockDirectory::areOpportunisticallySweptFreeListsStale()
{
    return m_opportunisticallySweptFreeListsVersion != markedSpace().newlyAllocatedVersion();
}

bool BlockDirectory::opportunisticSweep(MarkedBlock::Handle* block)
{
    ASSERT(!m_opportunisticallySweptFreeLists.contains(block->index() + 1));
    if (!m_opportunisticallySweptFreeLists.size())
        m_opportunisticallySweptFreeListsVersion = markedSpace().newlyAllocatedVersion(); // this version is incremented when GC finishes marking
    ASSERT(m_opportunisticallySweptFreeListsVersion == markedSpace().newlyAllocatedVersion());
    // auto addResult = m_opportunisticallySweptFreeLists.add(block->index(), [&] -> FreeList { return FreeList(cellSize()); });
    // auto addResult = m_opportunisticallySweptFreeLists.add(block->index(), [&] -> FreeList { return FreeList(static_cast<unsigned>(cellSize())); });
    ASSERT(block->index() != std::numeric_limits<unsigned>::max());
    std::unique_ptr<FreeList> freeList = std::make_unique<FreeList>(cellSize());
    block->sweep(&*freeList);
    if (freeList->allocationWillFail()) {
        ASSERT(block->isFreeListed());
        block->unsweepWithNoNewlyAllocated();
        ASSERT(!block->isFreeListed());
        {
            Locker locker { m_bitvectorLock };
            ASSERT(!isEmpty(block));
            setIsCanAllocateButNotEmpty(block, false);
            ASSERT(!isInUse(block)); // this was cleared by `unsweepWithNoNewlyAllocated`
            setIsInUse(block, true); // we need to set this because it'll be cleared in the caller
            setIsOpportunisticallyFreeListed(block, false); // just in case
        }
        WTFLogAlways("afryer_opportunisticSweep_block_was_full\n");
        return false;
    }
    WTFLogAlways("afryer_opportunisticSweep %p %u %p %u\n", block, block->index(), freeList->block(), freeList->block() == freeList->block());
    // auto addResult = m_opportunisticallySweptFreeLists.add(block->index() + 1, std::make_unique<FreeList>(static_cast<unsigned>(cellSize())));
    auto addResult = m_opportunisticallySweptFreeLists.add(block->index() + 1, WTFMove(freeList));
    // auto addResult = m_opportunisticallySweptFreeLists.add(block->index(), FreeList((unsigned)cellSize()));
    // block->sweep(&(*addResult.iterator->value));
    if (!addResult.iterator->value->block()) {
        WTFLogAlways("afryer_opportunisticSweep_failed_sweep_to_freelist_in_hash_map\n");
        RELEASE_ASSERT(false);
    }
    {
        Locker locker { m_bitvectorLock };
        setIsOpportunisticallyFreeListed(block, true);
        setIsCanAllocateButNotEmpty(block, false);
        setIsEmpty(block, false);
    }
    return true;
}

std::optional<FreeList> BlockDirectory::findOpportunisticallyConstructedFreeList()
{
    if (!m_opportunisticallySweptFreeLists.size())
        return std::nullopt;

    size_t cursor;
    {
        Locker locker { m_bitvectorLock };
        cursor = opportunisticallyFreeListedBits().findBit(0, true); // TODO: maybe we should search for a non-empty block first so that we reduce fragmentation?
    }
    ASSERT(cursor < m_blocks.size());
    ASSERT(m_opportunisticallySweptFreeLists.contains(cursor + 1));
    FreeList freeList = *m_opportunisticallySweptFreeLists.take(cursor + 1);
    ASSERT(freeList.block()->index() == cursor);
    if (m_opportunisticallySweptFreeListsVersion != markedSpace().newlyAllocatedVersion()) {
        MarkedBlock::Handle* block = freeList.block();
        {
            Locker locker { m_bitvectorLock };
            setIsInUse(block, true);
        }
        freeList.unfreelist();
        block->sweep(&freeList);
    }
    {
        Locker locker { m_bitvectorLock };
        if (!freeList.block())
            WTFLogAlways("afryer_findOpportunisticallyConstructedFreeList_nullptr_freelist_block\n");
        setIsOpportunisticallyFreeListed(freeList.block(), false);
        setIsInUse(freeList.block(), true);
    }
    freeList.block()->sweepWeak();
    return { freeList };
}

FreeList BlockDirectory::takeOpportunisticallyConstructedFreeList(MarkedBlock::Handle* block)
{
    ASSERT(m_opportunisticallySweptFreeLists.contains(block->index() + 1));
    FreeList freeList = *m_opportunisticallySweptFreeLists.take(block->index() + 1);
    return freeList;
}

void BlockDirectory::sweep()
{
    // We need to be careful of a weird race where while we are sweeping a block
    // the concurrent sweeper comes along and takes the inUse bit for a block
    // in the same bit vector word as we're currently scanning. If we did't
    // refresh our view into the word we could see stale data and try to scan
    // a block already in use.

    Locker locker(bitvectorLock());
    for (size_t index = 0; index < m_blocks.size(); ++index) {
        index = (unsweptBits() & ~inUseBits()).findBit(index, true);
        if (index >= m_blocks.size())
            break;

        MarkedBlock::Handle* block = m_blocks[index];
        ASSERT(!isInUse(index));
        dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", index, " in use (sweep) for ", *this);
        setIsInUse(index, true);
        {
            DropLockForScope scope(locker);
            block->sweep(nullptr);
        }
        ASSERT(!isUnswept(index));
        setIsInUse(index, false);
    }
}

void BlockDirectory::shrink()
{
    // We need to be careful of a weird race where while we are sweeping a block
    // the concurrent sweeper comes along and takes the inUse bit for a block
    // in the same bit vector word as we're currently scanning. If we did't
    // refresh our view into the word we could see stale data and try to scan
    // a block already in use.

    Locker locker(bitvectorLock());
    for (size_t index = 0; index < m_blocks.size(); ++index) {
        index = (emptyBits() & ~destructibleBits() & ~inUseBits()).findBit(index, true);
        if (index >= m_blocks.size())
            break;

        ASSERT(!isInUse(index));
        dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", index, " in use (shrink) for ", *this);
        setIsInUse(index, true);
        {
            DropLockForScope scope(locker);
            markedSpace().freeBlock(m_blocks[index]);
        }
        setIsInUse(index, false);
    }
}

// FIXME: rdar://139998916
MarkedBlock::Handle* BlockDirectory::findMarkedBlockHandleDebug(MarkedBlock* block)
{
    for (size_t index = 0; index < m_blocks.size(); ++index) {
        MarkedBlock::Handle* handle = m_blocks[index];
        if (handle && &handle->block() == block)
            return handle;
    }
    return nullptr;
}

void BlockDirectory::assertNoUnswept()
{
    if (!ASSERT_ENABLED)
        return;

    assertIsMutatorOrMutatorIsStopped();

    if (unsweptBitsView().isEmpty())
        return;
    
    dataLog("Assertion failed: unswept not empty in ", *this, ".\n");
    dumpBits();
    ASSERT_NOT_REACHED();
}

void BlockDirectory::didFinishUsingBlock(MarkedBlock::Handle* handle)
{
    Locker locker(bitvectorLock());
    didFinishUsingBlock(locker, handle);
}

void BlockDirectory::didFinishUsingBlock(AbstractLocker&, MarkedBlock::Handle* handle)
{
    if (!isInUse(handle)) [[unlikely]] {
        dataLogLn("Finish using on a block that's not in use: ", handle->index());
        dumpBits();
        RELEASE_ASSERT_NOT_REACHED();
    }

    dataLogLnIf(BlockDirectoryInternal::verbose, "Setting block ", handle->index(), " not in use (didFinishUsingBlock) for ", *this);
    setIsInUse(handle, false);
}

RefPtr<SharedTask<MarkedBlock::Handle*()>> BlockDirectory::parallelNotEmptyBlockSource()
{
    class Task final : public SharedTask<MarkedBlock::Handle*()> {
    public:
        Task(BlockDirectory& directory)
            : m_directory(directory)
        {
        }
        
        MarkedBlock::Handle* run() final
        {
            if (m_done)
                return nullptr;
            Locker locker { m_lock };
            m_directory.assertIsMutatorOrMutatorIsStopped();
            m_index = m_directory.m_bits.markingNotEmpty().findBit(m_index, true);
            if (m_index >= m_directory.m_blocks.size()) {
                m_done = true;
                return nullptr;
            }
            return m_directory.m_blocks[m_index++];
        }
        
    private:
        BlockDirectory& m_directory WTF_GUARDED_BY_LOCK(m_lock);
        size_t m_index WTF_GUARDED_BY_LOCK(m_lock) { 0 };
        Lock m_lock;
        bool m_done { false };
    };
    
    return adoptRef(new Task(*this));
}

void BlockDirectory::dump(PrintStream& out) const
{
    out.print(RawPointer(this), ":", m_cellSize, "/", m_attributes);
}

void BlockDirectory::dumpBits(PrintStream& out)
{
    unsigned maxNameLength = 0;
    forEachBitVectorWithName(
        [&](auto vectorRef, const char* name) {
            UNUSED_PARAM(vectorRef);
WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN
            unsigned length = strlen(name);
WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
            maxNameLength = std::max(maxNameLength, length);
        });
    
    forEachBitVectorWithName(
        [&](auto vectorRef, const char* name) {
            out.print("    ", name, ": ");
WTF_ALLOW_UNSAFE_BUFFER_USAGE_BEGIN
            for (unsigned i = maxNameLength - strlen(name); i--;)
WTF_ALLOW_UNSAFE_BUFFER_USAGE_END
                out.print(" ");
            out.print(vectorRef, "\n");
        });
}

MarkedSpace& BlockDirectory::markedSpace() const
{
    return m_subspace->space();
}

#if ASSERT_ENABLED
void BlockDirectory::assertIsMutatorOrMutatorIsStopped() const
{
    auto& heap = markedSpace().heap();
    if (!heap.worldIsStopped()) {
        if (auto owner = heap.vm().apiLock().ownerThread())
            ASSERT(owner->get() == &Thread::currentSingleton());
        else {
            // FIXME: It feels like heap access should be tied to holding the API lock.
            ASSERT(heap.hasAccess());
        }
    }
}

void BlockDirectory::assertSweeperIsSuspended() const
{
    assertIsMutatorOrMutatorIsStopped();
}
#endif
} // namespace JSC

