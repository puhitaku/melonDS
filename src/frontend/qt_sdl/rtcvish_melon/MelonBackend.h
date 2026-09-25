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

#ifndef RTCVISH_MELONBACKEND_H
#define RTCVISH_MELONBACKEND_H

#include <QString>

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "rtcvish/Backend.h"
#include "rtcvish/Server.h"

#include "types.h"

class EmuInstance;
class EmuThread;

// rtcv-ish emulator API for melonDS (see design/emulator-api.md in the
// rtcv-ish repository). Owns the API server; the EmuThread calls the hooks
// below from its main loop. Every Backend method runs on the emulation thread.
class MelonBackend : public rtcvish::Backend
{
public:
    MelonBackend();
    ~MelonBackend() override;

    // Main thread, before attach().
    bool start(const QString& address, QString& error);

    // Main thread: bind to the emulator instance once it exists.
    void attach(EmuInstance* inst);

    // --- EmuThread hooks (emulation thread) ---

    // After handleMessages(): run API jobs.
    void poll();
    // Paused loop: sleep until a request arrives or the timeout expires.
    void idleWait(int timeoutMs);
    // True while frames requested by Step are pending.
    bool framesPending() const { return framesToRun > 0; }
    // Pre-frame: OR the input override into the user's input.
    void applyInput(melonDS::u32& keyMask, bool& touching, melonDS::u16& touchX, melonDS::u16& touchY);
    // Right before / after RunFrame() actually emulates a frame.
    void beforeFrame();
    void afterFrame();
    // The UI booted a ROM or firmware (romChanged) or reset the console.
    void onConsoleReset(bool romChanged);
    // The UI stopped emulation.
    void onConsoleStopped();

    // --- rtcvish::Backend ---
    rtcvish::Info hello() override;
    rtcvish::Status status() override;
    std::vector<rtcvish::Domain> domains() override;
    bool read(const std::string& domain, uint64_t address, uint32_t size, uint8_t* out,
              rtcvish::Error& err) override;
    bool write(const std::string& domain, uint64_t address, const uint8_t* data, uint32_t size,
               rtcvish::Error& err) override;
    bool saveState(std::vector<uint8_t>& out, rtcvish::Error& err) override;
    bool loadState(const uint8_t* data, size_t size, rtcvish::Error& err) override;
    bool loadRom(const std::string& path, rtcvish::Error& err) override;
    bool closeRom(rtcvish::Error& err) override;
    bool reset(rtcvish::Error& err) override;
    bool pause(rtcvish::Error& err) override;
    bool resume(rtcvish::Error& err) override;
    bool runFrames(uint32_t frames, rtcvish::Error& err) override;
    void cancelFrames() override;
    bool setInput(const rtcvish::Input& input, rtcvish::Error& err) override;
    bool screenshot(std::vector<rtcvish::Image>& screens, rtcvish::Error& err) override;
    void quit() override;

private:
    bool active() const;
    void updateRomPathFromInstance();

    rtcvish::Server server;
    EmuInstance* inst = nullptr;
    EmuThread* thread = nullptr;

    uint64_t frame = 0;
    uint32_t framesToRun = 0;
    bool forceStatus = false;
    std::string romPath;

    bool inputOverride = false;
    rtcvish::Input input;
};

#endif // RTCVISH_MELONBACKEND_H
