/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef MELONDS_MEMORYFREEZE_H
#define MELONDS_MEMORYFREEZE_H

#include <algorithm>
#include <cstdint>
#include <vector>

#include "types.h"

namespace melonDS
{

// Bytes of emulated memory that guest writes must leave unchanged (frozen
// by an external tool such as rtcv-ish). The bus write paths call Store()
// or the Filter functions, which cost a single predictable branch while
// nothing is frozen. A write partially covering frozen bytes keeps those
// bytes and changes the others.
//
// Two kinds of blocks exist: host memory (RAM arrays addressed by host
// pointer, which also covers all mirrors of them) and the ARM9 VRAM view at
// 0x06000000-0x067FFFFF (addressed by offset, since VRAM banks are mapped
// dynamically).
//
// Covered: ARM9/ARM7 bus writes (NDS and DSi) to MainRAM, shared WRAM,
// ARM7 WRAM, NWRAM, palette, OAM and the ARM9 VRAM view; ITCM/DTCM writes
// (interpreter and JIT slow path); DMA and NDMA (they use the bus); JIT
// code (slow path, and fastmem through write-protected pages, see
// ARMJIT_Memory::RefreshFreezeProtection).
// Not covered: VRAM written through another alias than the frozen view
// address (LCDC 0x06800000+, mirrors, ARM7 VRAM mapping) or by display
// capture, NWRAM accessed by the DSi DSP, and the GDB stub.
class MemoryFreeze
{
public:
    // [Start, End) relative to the block.
    struct Range
    {
        u32 Start, End;
    };

    static constexpr u32 PageShift = 12;
    static constexpr u32 VRAMViewSize = 0x800000;

    // Replace the frozen ranges of host memory [base, base+size).
    void SetHost(const u8* base, u32 size, std::vector<Range> ranges)
    {
        uintptr_t key = reinterpret_cast<uintptr_t>(base);
        Host.erase(std::remove_if(Host.begin(), Host.end(), [&](const Block& b) { return b.Base == key; }),
                   Host.end());
        Block b;
        if (Build(b, key, size, std::move(ranges)))
            Host.push_back(std::move(b));
        Update();
    }

    // Replace the frozen ranges of the ARM9 VRAM view (offsets from 0x06000000).
    void SetVRAM(std::vector<Range> ranges)
    {
        VRAM = Block();
        Build(VRAM, 0, VRAMViewSize, std::move(ranges));
        Update();
    }

    void Clear()
    {
        Host.clear();
        VRAM = Block();
        Update();
    }

    bool Active() const noexcept { return IsActive; }

    // *(T*)dst = val, keeping the frozen bytes of dst.
    template <typename T>
    void Store(u8* dst, T val) noexcept
    {
        if (IsActive)
            val = FilterHost(dst, val);
        *(T*)dst = val;
    }

    // val with the frozen bytes of dst replaced by their current contents.
    template <typename T>
    T FilterHost(const u8* dst, T val) const noexcept
    {
        uintptr_t p = reinterpret_cast<uintptr_t>(dst);
        for (const Block& b : Host)
        {
            if (p - b.Base < b.Size)
                return Merge(b, u32(p - b.Base), val, [&] { return *(const T*)dst; });
        }
        return val;
    }

    // val with the frozen bytes of the VRAM view at `offset` replaced by
    // readOld(), which returns the current contents.
    template <typename T, typename F>
    T FilterVRAM(u32 offset, T val, F readOld) const noexcept
    {
        if (offset >= VRAM.Size)
            return val;
        return Merge(VRAM, offset, val, readOld);
    }

    // True when a byte of host memory [p, p+size) is frozen.
    bool HostFrozen(const u8* p, u32 size) const noexcept
    {
        uintptr_t start = reinterpret_cast<uintptr_t>(p);
        for (const Block& b : Host)
        {
            if (start + size <= b.Base || start >= b.Base + b.Size)
                continue;
            u32 lo = start > b.Base ? u32(start - b.Base) : 0;
            u32 hi = u32(std::min<uintptr_t>(start + size - b.Base, b.Size));
            auto it = FirstEndingAfter(b, lo);
            if (it != b.Ranges.end() && it->Start < hi)
                return true;
        }
        return false;
    }

private:
    struct Block
    {
        uintptr_t Base = 0;
        u32 Size = 0;
        std::vector<u64> Pages; // one bit per 4 KiB page with frozen bytes
        std::vector<Range> Ranges; // sorted, disjoint
    };

    static bool Build(Block& b, uintptr_t base, u32 size, std::vector<Range> ranges)
    {
        std::sort(ranges.begin(), ranges.end(), [](const Range& a, const Range& c) { return a.Start < c.Start; });
        b.Base = base;
        b.Size = size;
        b.Pages.assign(((size >> PageShift) + 64) / 64, 0);
        for (Range r : ranges)
        {
            r.End = std::min(r.End, size);
            if (r.Start >= r.End)
                continue;
            if (!b.Ranges.empty() && r.Start <= b.Ranges.back().End)
                b.Ranges.back().End = std::max(b.Ranges.back().End, r.End);
            else
                b.Ranges.push_back(r);
            for (u32 page = r.Start >> PageShift; page <= (r.End - 1) >> PageShift; page++)
                b.Pages[page / 64] |= u64(1) << (page % 64);
        }
        if (b.Ranges.empty())
        {
            b = Block();
            return false;
        }
        return true;
    }

    void Update() { IsActive = !Host.empty() || !VRAM.Ranges.empty(); }

    static bool PageFrozen(const Block& b, u32 offset) noexcept
    {
        u32 page = offset >> PageShift;
        return (b.Pages[page / 64] >> (page % 64)) & 1;
    }

    static std::vector<Range>::const_iterator FirstEndingAfter(const Block& b, u32 offset) noexcept
    {
        return std::upper_bound(b.Ranges.begin(), b.Ranges.end(), offset,
                                [](u32 v, const Range& r) { return v < r.End; });
    }

    template <typename T, typename F>
    static T Merge(const Block& b, u32 offset, T val, F readOld) noexcept
    {
        u32 end = std::min<u32>(offset + sizeof(T), b.Size);
        if (!PageFrozen(b, offset) && !PageFrozen(b, end - 1))
            return val;
        u64 mask = 0;
        for (auto it = FirstEndingAfter(b, offset); it != b.Ranges.end() && it->Start < end; ++it)
        {
            u32 s = std::max(it->Start, offset), e = std::min(it->End, end);
            for (u32 i = s; i < e; i++)
                mask |= u64(0xFF) << ((i - offset) * 8);
        }
        if (!mask)
            return val;
        T old = readOld();
        return T((u64(val) & ~mask) | (u64(old) & mask));
    }

    std::vector<Block> Host;
    Block VRAM;
    bool IsActive = false;
};

}

#endif
