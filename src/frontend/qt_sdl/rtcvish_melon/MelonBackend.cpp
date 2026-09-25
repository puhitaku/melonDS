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

#include "MelonBackend.h"

#include <QCoreApplication>
#include <QFileInfo>
#include <QMetaObject>

#include <chrono>
#include <cstring>

#include "DSi.h"
#include "EmuInstance.h"
#include "EmuThread.h"
#include "GPU.h"
#include "MemConstants.h"
#include "NDS.h"
#include "NDSCart.h"
#include "Savestate.h"
#include "version.h"

using namespace melonDS;

namespace
{

enum class Dom
{
    MainRAM,
    VRAM,
    Palette,
    OAM,
    SharedWRAM,
    ARM7WRAM,
    ITCM,
    DTCM,
    CartROM,
    NWRAM_A,
    NWRAM_B,
    NWRAM_C,
    Unknown,
};

constexpr u32 kVRAMSize = 0x800000;
constexpr u32 kPaletteSize = 0x800;
constexpr u32 kOAMSize = 0x800;

Dom domainByName(const std::string& name)
{
    static const struct
    {
        const char* name;
        Dom dom;
    } table[] = {
        {"MainRAM", Dom::MainRAM},
        {"VRAM", Dom::VRAM},
        {"Palette", Dom::Palette},
        {"OAM", Dom::OAM},
        {"SharedWRAM", Dom::SharedWRAM},
        {"ARM7WRAM", Dom::ARM7WRAM},
        {"ITCM", Dom::ITCM},
        {"DTCM", Dom::DTCM},
        {"CartROM", Dom::CartROM},
        {"NWRAM_A", Dom::NWRAM_A},
        {"NWRAM_B", Dom::NWRAM_B},
        {"NWRAM_C", Dom::NWRAM_C},
    };
    for (const auto& e : table)
    {
        if (name == e.name) return e.dom;
    }
    return Dom::Unknown;
}

std::string trimmed(const char* s, size_t n)
{
    std::string out(s, strnlen(s, n));
    while (!out.empty() && (out.back() == ' ' || out.back() == '\0'))
        out.pop_back();
    return out;
}

#ifdef JIT_ENABLED
// Invalidate JIT blocks covering one byte of a region addressed by its raw
// offset, as ARMJIT::CheckAndInvalidate does for bus addresses.
void invalidateLocal(NDS* nds, int region, u32 offset)
{
    AddressRange* ranges = nds->JIT.CodeMemRegions[region];
    if (ranges && (ranges[offset / 512].Code & (1 << ((offset & 0x1FF) / 16))))
        nds->JIT.InvalidateByAddr((u32(region) << 27) | offset);
}
#endif

u8 readVRAM(NDS* nds, u32 offset)
{
    u32 addr = 0x06000000 + offset;
    GPU& gpu = nds->GPU;
    switch (addr & 0x00E00000)
    {
    case 0x00000000: gpu.SyncVRAM_ABG(addr, false); return gpu.ReadVRAM_ABG<u8>(addr);
    case 0x00200000: gpu.SyncVRAM_BBG(addr, false); return gpu.ReadVRAM_BBG<u8>(addr);
    case 0x00400000: gpu.SyncVRAM_AOBJ(addr, false); return gpu.ReadVRAM_AOBJ<u8>(addr);
    default: gpu.SyncVRAM_BOBJ(addr, false); return gpu.ReadVRAM_BOBJ<u8>(addr);
    }
}

void writeVRAM(NDS* nds, u32 offset, u8 val)
{
    u32 addr = 0x06000000 + offset;
    GPU& gpu = nds->GPU;
#ifdef JIT_ENABLED
    nds->JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_VRAM>(addr);
#endif
    switch (addr & 0x00E00000)
    {
    case 0x00000000:
        gpu.SyncVRAM_ABG(addr, true);
        gpu.WriteVRAM_ABG<u8>(addr, val);
        return;
    case 0x00200000:
        gpu.SyncVRAM_BBG(addr, true);
        gpu.WriteVRAM_BBG<u8>(addr, val);
        return;
    case 0x00400000:
        gpu.SyncVRAM_AOBJ(addr, true);
        gpu.WriteVRAM_AOBJ<u8>(addr, val);
        return;
    default:
        gpu.SyncVRAM_BOBJ(addr, true);
        gpu.WriteVRAM_BOBJ<u8>(addr, val);
        return;
    }
}

// Raw backing memory for domains that are plain byte arrays; null otherwise.
u8* rawMemory(NDS* nds, Dom dom)
{
    switch (dom)
    {
    case Dom::MainRAM: return nds->MainRAM;
    case Dom::SharedWRAM: return nds->SharedWRAM;
    case Dom::ARM7WRAM: return nds->ARM7WRAM;
    case Dom::ITCM: return nds->ARM9.ITCM;
    case Dom::DTCM: return nds->ARM9.DTCM;
    case Dom::CartROM:
    {
        auto* cart = nds->NDSCartSlot.GetCart();
        return cart ? const_cast<u8*>(cart->GetROM()) : nullptr;
    }
    case Dom::NWRAM_A:
    case Dom::NWRAM_B:
    case Dom::NWRAM_C:
    {
        if (nds->ConsoleType != 1) return nullptr;
        auto* dsi = static_cast<DSi*>(nds);
        if (dom == Dom::NWRAM_A) return dsi->NWRAM_A;
        if (dom == Dom::NWRAM_B) return dsi->NWRAM_B;
        return dsi->NWRAM_C;
    }
    default: return nullptr;
    }
}

void invalidate(NDS* nds, Dom dom, u32 offset)
{
#ifdef JIT_ENABLED
    switch (dom)
    {
    case Dom::MainRAM:
        nds->JIT.CheckAndInvalidate<0, ARMJIT_Memory::memregion_MainRAM>(0x02000000 + offset);
        break;
    case Dom::SharedWRAM: invalidateLocal(nds, ARMJIT_Memory::memregion_SharedWRAM, offset); break;
    case Dom::ARM7WRAM: nds->JIT.CheckAndInvalidate<1, ARMJIT_Memory::memregion_WRAM7>(offset); break;
    case Dom::ITCM: invalidateLocal(nds, ARMJIT_Memory::memregion_ITCM, offset); break;
    case Dom::NWRAM_A: invalidateLocal(nds, ARMJIT_Memory::memregion_NewSharedWRAM_A, offset); break;
    case Dom::NWRAM_B: invalidateLocal(nds, ARMJIT_Memory::memregion_NewSharedWRAM_B, offset); break;
    case Dom::NWRAM_C: invalidateLocal(nds, ARMJIT_Memory::memregion_NewSharedWRAM_C, offset); break;
    default: break;
    }
#else
    (void)nds;
    (void)dom;
    (void)offset;
#endif
}

} // namespace

MelonBackend::MelonBackend() : server(*this) {}

MelonBackend::~MelonBackend()
{
    server.stop();
}

bool MelonBackend::start(const QString& address, QString& error)
{
    std::string host;
    uint16_t port = 0;
    if (!rtcvish::Server::parseAddress(address.toStdString(), host, port))
    {
        error = "invalid address \"" + address + "\", expected HOST:PORT";
        return false;
    }
    std::string err;
    if (!server.start(host, port, err))
    {
        error = QString::fromStdString(err);
        return false;
    }
    return true;
}

void MelonBackend::attach(EmuInstance* instance)
{
    inst = instance;
    thread = instance->getEmuThread();
    thread->setRtcvish(this);
}

bool MelonBackend::active() const
{
    return inst && thread && thread->emuActive && inst->nds;
}

void MelonBackend::updateRomPathFromInstance()
{
    if (inst->baseROMName.empty())
        romPath.clear();
    else if (inst->baseROMDir.empty())
        romPath = inst->baseROMName;
    else
        romPath = inst->baseROMDir + "/" + inst->baseROMName;
}

void MelonBackend::poll()
{
    if (forceStatus)
    {
        forceStatus = false;
        server.statusChanged();
    }
    server.pollJobs();
}

void MelonBackend::idleWait(int timeoutMs)
{
    server.waitForJobs(std::chrono::milliseconds(timeoutMs));
}

void MelonBackend::applyInput(u32& keyMask, bool& touching, u16& touchX, u16& touchY)
{
    if (!inputOverride) return;
    keyMask &= ~(input.buttons & 0xFFF);
    if (input.touch)
    {
        touching = true;
        touchX = u16(input.touchX > 255 ? 255 : input.touchX);
        touchY = u16(input.touchY > 191 ? 191 : input.touchY);
    }
}

void MelonBackend::beforeFrame()
{
    server.runFrame();
}

void MelonBackend::afterFrame()
{
    frame++;
    if (framesToRun > 0) framesToRun--;
    server.frameCompleted(frame);
}

void MelonBackend::onConsoleReset(bool romChanged)
{
    frame = 0;
    server.scheduler().clear();
    if (romChanged) updateRomPathFromInstance();
    forceStatus = true;
}

void MelonBackend::onConsoleStopped()
{
    framesToRun = 0;
    server.scheduler().clear();
    forceStatus = true;
}

rtcvish::Info MelonBackend::hello()
{
    rtcvish::Info info;
    info.emulator = "melonDS";
    info.version = MELONDS_VERSION;
    info.system = "nds";
    info.capabilities.savestates = true;
    info.capabilities.screenshot = true;
    info.capabilities.input = true;
    info.capabilities.loadRom = true;
    info.capabilities.reset = true;
    return info;
}

rtcvish::Status MelonBackend::status()
{
    rtcvish::Status st;
    st.frame = frame;
    if (!active()) return st;

    NDS* nds = inst->nds;
    st.state =
        thread->emuStatus == EmuThread::emuStatus_Running ? rtcvish::State::Running : rtcvish::State::Paused;
    st.romPath = romPath;
    st.console = nds->ConsoleType == 1 ? "DSi" : "DS";
    if (auto* cart = nds->NDSCartSlot.GetCart())
    {
        const NDSHeader& header = cart->GetHeader();
        st.gameTitle = trimmed(header.GameTitle, sizeof(header.GameTitle));
        st.gameCode = trimmed(header.GameCode, sizeof(header.GameCode));
    }
    return st;
}

std::vector<rtcvish::Domain> MelonBackend::domains()
{
    std::vector<rtcvish::Domain> out;
    if (!active()) return out;

    NDS* nds = inst->nds;
    auto add = [&](const char* name, uint64_t size, uint32_t word, bool hidden)
    {
        rtcvish::Domain d;
        d.name = name;
        d.size = size;
        d.wordSize = word;
        d.bigEndian = false;
        d.writable = true;
        d.hidden = hidden;
        out.push_back(d);
    };

    add("MainRAM", uint64_t(nds->MainRAMMask) + 1, 4, false);
    add("VRAM", kVRAMSize, 4, false);
    add("Palette", kPaletteSize, 2, false);
    add("OAM", kOAMSize, 2, false);
    add("SharedWRAM", SharedWRAMSize, 4, true);
    add("ARM7WRAM", ARM7WRAMSize, 4, true);
    add("ITCM", ITCMPhysicalSize, 4, true);
    add("DTCM", DTCMPhysicalSize, 4, true);
    if (auto* cart = nds->NDSCartSlot.GetCart()) add("CartROM", cart->GetROMLength(), 4, true);
    if (nds->ConsoleType == 1)
    {
        add("NWRAM_A", NWRAMSize, 4, true);
        add("NWRAM_B", NWRAMSize, 4, true);
        add("NWRAM_C", NWRAMSize, 4, true);
    }
    return out;
}

bool MelonBackend::read(const std::string& domain, uint64_t address, uint32_t size, uint8_t* out,
                        rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    NDS* nds = inst->nds;
    Dom dom = domainByName(domain);
    u32 base = u32(address);
    switch (dom)
    {
    case Dom::VRAM:
        for (u32 i = 0; i < size; i++)
            out[i] = readVRAM(nds, base + i);
        return true;
    case Dom::Palette:
        for (u32 i = 0; i < size; i++)
            out[i] = nds->GPU.ReadPalette<u8>(base + i);
        return true;
    case Dom::OAM:
        for (u32 i = 0; i < size; i++)
            out[i] = nds->GPU.ReadOAM<u8>(base + i);
        return true;
    default: break;
    }

    u8* mem = rawMemory(nds, dom);
    if (!mem) return err.set(rtcvish::ErrorCode::NotFound, "unknown domain " + domain);
    memcpy(out, mem + base, size);
    return true;
}

bool MelonBackend::write(const std::string& domain, uint64_t address, const uint8_t* data, uint32_t size,
                         rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    NDS* nds = inst->nds;
    Dom dom = domainByName(domain);
    u32 base = u32(address);
    switch (dom)
    {
    case Dom::VRAM:
        for (u32 i = 0; i < size; i++)
            writeVRAM(nds, base + i, data[i]);
        return true;
    case Dom::Palette:
        for (u32 i = 0; i < size; i++)
            nds->GPU.WritePalette<u8>(base + i, data[i]);
        return true;
    case Dom::OAM:
        for (u32 i = 0; i < size; i++)
            nds->GPU.WriteOAM<u8>(base + i, data[i]);
        return true;
    default: break;
    }

    u8* mem = rawMemory(nds, dom);
    if (!mem) return err.set(rtcvish::ErrorCode::NotFound, "unknown domain " + domain);
    for (u32 i = 0; i < size; i++)
    {
        invalidate(nds, dom, base + i);
        mem[base + i] = data[i];
    }
    return true;
}

bool MelonBackend::saveState(std::vector<uint8_t>& out, rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    Savestate state;
    if (state.Error || !inst->nds->DoSavestate(&state) || state.Error)
        return err.set(rtcvish::ErrorCode::Failed, "failed to create savestate");

    const u8* buf = static_cast<const u8*>(state.Buffer());
    out.assign(buf, buf + state.Length());
    return true;
}

bool MelonBackend::loadState(const uint8_t* data, size_t size, rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    // Savestate() only reads from the buffer when loading.
    Savestate state(const_cast<uint8_t*>(data), u32(size), false);
    if (state.Error) return err.set(rtcvish::ErrorCode::InvalidArgument, "invalid savestate");

    Savestate backup;
    if (backup.Error || !inst->nds->DoSavestate(&backup) || backup.Error)
        return err.set(rtcvish::ErrorCode::Failed, "failed to back up the current state");

    if (!inst->nds->DoSavestate(&state) || state.Error)
    {
        backup.Rewind(false);
        inst->nds->DoSavestate(&backup);
        return err.set(rtcvish::ErrorCode::InvalidArgument, "savestate rejected by the emulator");
    }
    return true;
}

bool MelonBackend::loadRom(const std::string& path, rtcvish::Error& err)
{
    QString qpath = QString::fromStdString(path);
    if (!QFileInfo(qpath).isFile()) return err.set(rtcvish::ErrorCode::NotFound, "no such file: " + path);

    QString setupError = inst->verifySetup();
    if (!setupError.isEmpty()) return err.set(rtcvish::ErrorCode::Failed, setupError.toStdString());

    QString loadError;
    if (!inst->loadROM(QStringList{qpath}, true, loadError))
        return err.set(rtcvish::ErrorCode::Failed, loadError.toStdString());

    inst->nds->Start();
    thread->runNow();

    frame = 0;
    framesToRun = 0;
    romPath = path;
    return true;
}

bool MelonBackend::closeRom(rtcvish::Error&)
{
    framesToRun = 0;
    if (thread->emuActive) thread->stopNow(true);
    romPath.clear();
    return true;
}

bool MelonBackend::reset(rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    bool paused = thread->emuStatus == EmuThread::emuStatus_Paused;
    framesToRun = 0;
    thread->resetNow();
    if (paused) thread->pauseNow();
    frame = 0;
    return true;
}

bool MelonBackend::pause(rtcvish::Error&)
{
    thread->pauseNow();
    return true;
}

bool MelonBackend::resume(rtcvish::Error&)
{
    framesToRun = 0;
    thread->unpauseNow();
    return true;
}

bool MelonBackend::runFrames(uint32_t frames, rtcvish::Error&)
{
    framesToRun = frames;
    return true;
}

void MelonBackend::cancelFrames()
{
    framesToRun = 0;
}

bool MelonBackend::setInput(const rtcvish::Input& in, rtcvish::Error&)
{
    inputOverride = !in.clear;
    input = in;
    return true;
}

bool MelonBackend::screenshot(std::vector<rtcvish::Image>& screens, rtcvish::Error& err)
{
    if (!active()) return err.set(rtcvish::ErrorCode::NoRom, "no ROM loaded");

    void* fb[2] = {nullptr, nullptr};
    if (!inst->nds->GPU.GetFramebuffers(&fb[0], &fb[1]) || !fb[0] || !fb[1])
        return err.set(rtcvish::ErrorCode::Unsupported, "screenshots need the software renderer");

    constexpr u32 w = 256, h = 192;
    for (void* p : fb)
    {
        const u32* src = static_cast<const u32*>(p);
        rtcvish::Image img;
        img.width = w;
        img.height = h;
        img.rgba.resize(w * h * 4);
        for (u32 i = 0; i < w * h; i++)
        {
            u32 px = src[i];
            img.rgba[i * 4 + 0] = u8(px >> 16);
            img.rgba[i * 4 + 1] = u8(px >> 8);
            img.rgba[i * 4 + 2] = u8(px);
            img.rgba[i * 4 + 3] = 0xFF;
        }
        screens.push_back(std::move(img));
    }
    return true;
}

void MelonBackend::quit()
{
    QMetaObject::invokeMethod(qApp, [] { QCoreApplication::quit(); }, Qt::QueuedConnection);
}
