/*
 * Copyright (C) 2020 Apple Inc. All rights reserved.
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
#include "SerializedNFA.h"

#include "NFA.h"
#include <wtf/FileHandle.h>
#include <wtf/FileSystem.h>
#include <wtf/text/ParsingUtilities.h>

#if ENABLE(CONTENT_EXTENSIONS)

namespace WebCore {
namespace ContentExtensions {

template<typename T>
bool writeAllToFile(FileSystem::FileHandle& file, const T& container)
{
    auto bytes = spanReinterpretCast<const uint8_t>(container.span());
    while (!bytes.empty()) {
        auto written = file.write(bytes);
        if (!written)
            return false;
        skip(bytes, *written);
    }
    return true;
}

std::optional<SerializedNFA> SerializedNFA::serialize(NFA&& nfa)
{
    auto [filename, fileHandle] = FileSystem::openTemporaryFile("SerializedNFA"_s);
    if (!fileHandle)
        return std::nullopt;

    bool wroteSuccessfully = writeAllToFile(fileHandle, nfa.nodes)
        && writeAllToFile(fileHandle, nfa.transitions)
        && writeAllToFile(fileHandle, nfa.targets)
        && writeAllToFile(fileHandle, nfa.epsilonTransitionsTargets)
        && writeAllToFile(fileHandle, nfa.actions);
    if (!wroteSuccessfully) {
        fileHandle = { };
        FileSystem::deleteFile(filename);
        return std::nullopt;
    }

    auto mappedFile = fileHandle.map(FileSystem::MappedFileMode::Private);
    fileHandle = { };
    FileSystem::deleteFile(filename);
    if (!mappedFile)
        return std::nullopt;

    Metadata metadata {
        nfa.nodes.size(),
        nfa.transitions.size(),
        nfa.targets.size(),
        nfa.epsilonTransitionsTargets.size(),
        nfa.actions.size(),
        0,
        nfa.nodes.size() * sizeof(nfa.nodes[0]),
        nfa.nodes.size() * sizeof(nfa.nodes[0])
            + nfa.transitions.size() * sizeof(nfa.transitions[0]),
        nfa.nodes.size() * sizeof(nfa.nodes[0])
            + nfa.transitions.size() * sizeof(nfa.transitions[0])
            + nfa.targets.size() * sizeof(nfa.targets[0]),
        nfa.nodes.size() * sizeof(nfa.nodes[0])
            + nfa.transitions.size() * sizeof(nfa.transitions[0])
            + nfa.targets.size() * sizeof(nfa.targets[0])
            + nfa.epsilonTransitionsTargets.size() * sizeof(nfa.epsilonTransitionsTargets[0])
    };

    nfa.clear();

    return { { WTFMove(*mappedFile), WTFMove(metadata) } };
}

SerializedNFA::SerializedNFA(FileSystem::MappedFileData&& file, Metadata&& metadata)
    : m_file(WTFMove(file))
    , m_metadata(WTFMove(metadata))
{
}

template<typename T>
std::span<const T> SerializedNFA::spanAtOffsetInFile(size_t offset, size_t length) const
{
    return spanReinterpretCast<const T>(m_file.span().subspan(offset).first(length * sizeof(T)));
}

auto SerializedNFA::nodes() const -> const Range<ImmutableNFANode>
{
    return spanAtOffsetInFile<ImmutableNFANode>(m_metadata.nodesOffset, m_metadata.nodesSize);
}

auto SerializedNFA::transitions() const -> const Range<ImmutableRange<char>>
{
    return spanAtOffsetInFile<ImmutableRange<char>>(m_metadata.transitionsOffset, m_metadata.transitionsSize);
}

auto SerializedNFA::targets() const -> const Range<uint32_t>
{
    return spanAtOffsetInFile<uint32_t>(m_metadata.targetsOffset, m_metadata.targetsSize);
}

auto SerializedNFA::epsilonTransitionsTargets() const -> const Range<uint32_t>
{
    return spanAtOffsetInFile<uint32_t>(m_metadata.epsilonTransitionsTargetsOffset, m_metadata.epsilonTransitionsTargetsSize);
}

auto SerializedNFA::actions() const -> const Range<uint64_t>
{
    return spanAtOffsetInFile<uint64_t>(m_metadata.actionsOffset, m_metadata.actionsSize);
}

} // namespace ContentExtensions
} // namespace WebCore

#endif // ENABLE(CONTENT_EXTENSIONS)
