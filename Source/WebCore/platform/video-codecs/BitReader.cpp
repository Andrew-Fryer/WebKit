/*
 * Copyright (C) 2022 Apple Inc. All rights reserved.
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
 *
 */

#include "config.h"
#include "BitReader.h"

namespace WebCore {

std::optional<uint64_t> BitReader::read(size_t bits)
{
    ASSERT(bits <= 64);

    // FIXME: We should optimize this routine.
    size_t value = 0;
    do {
        auto bit = readBit();
        if (!bit)
            return { };
        value = (value << 1) | (*bit ? 1 : 0);
        --bits;
    } while (bits);
    return value;
}

std::optional<bool> BitReader::readBit()
{
    if (!m_remainingBits) {
        if (m_index >= m_data.size())
            return { };
        m_currentByte = m_data[m_index++];
        m_remainingBits = 8;
    }

    bool value = m_currentByte & 0x80;
    --m_remainingBits;
    m_currentByte = m_currentByte << 1;
    return value;
}

bool BitReader::skipBytes(size_t bytes)
{
    m_index += bytes;
    if (m_index >= m_data.size()) {
        bool over = m_index > m_data.size() || m_remainingBits;
        m_index = m_data.size();
        m_remainingBits = 0;
        return !over;
    }
    m_currentByte = m_data[m_index];
    return true;
}

size_t BitReader::bitOffset() const
{
    ASSERT(m_index <= m_data.size());
    return (m_remainingBits ? 8 - m_remainingBits : 0) + m_index * 8;
}

} // namespace WebCore
