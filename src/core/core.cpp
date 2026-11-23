/*
    Copyright 2023-2026 Hydr8gon

    This file is part of 3Beans.

    3Beans is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    3Beans is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with 3Beans. If not, see <https://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <cstring>
#include "core.h"

Core::Core(std::string &cartPath, std::function<void()> *contextFunc, const std::string &firmPath): aes(*this), arms { ArmInterp(*this, ARM11A),
        ArmInterp(*this, ARM11B), ArmInterp(*this, ARM11C), ArmInterp(*this, ARM11D), ArmInterp(*this, ARM9) },
        cartridge(*this, cartPath), cdmas { Cdma(*this, CDMA0), Cdma(*this, CDMA1), Cdma(*this, XDMA) }, cp15(*this),
        csnd(*this), gpu(*this, contextFunc), i2c(*this), input(*this), interrupts(*this), memory(*this),
        ndma(*this), pdc(*this), pxi(*this), rsa(*this), sdMmcs { SdMmc(*this), SdMmc(*this) }, shas { Sha(*this,
        0), Sha(*this, 1) }, timers(*this), vfp11s { Vfp11Interp(*this, ARM11A), Vfp11Interp(*this, ARM11B),
        Vfp11Interp(*this, ARM11C), Vfp11Interp(*this, ARM11D) }, wifi(*this), y2rs { Y2r(*this, 0), Y2r(*this, 1) } {
    // Determine system type automatically, or override if hardcoded
    n3dsMode = sdMmcs[0].init(sdMmcs[1]); // MMC init tells us
    if (Settings::systemType) n3dsMode = (Settings::systemType == 2);
    LOG_INFO("Running in %s 3DS mode\n", n3dsMode ? "new" : "old");

    // Run any other post-construction initializers
    if (!memory.init()) throw ERROR_BOOTROM;
    if (!firmPath.empty()) loadFirm(firmPath);
    for (int i = 0; i < MAX_CPUS; i++)
        arms[i].init();
    initDsp();

    // Define static tasks that can be scheduled
    tasks[RESET_CYCLES] = std::bind(&Core::resetCycles, this);
    tasks[END_FRAME] = std::bind(&Core::endFrame, this);
    tasks[UPDATE_RUN_FUNC] = std::bind(&Core::updateRunFunc, this);
    tasks[ARM_STOP_CYCLES] = std::bind(&ArmInterp::stopCycles, this);
    tasks[ARM11A_INTERRUPT] = std::bind(&Interrupts::interrupt, &interrupts, ARM11A);
    tasks[ARM11B_INTERRUPT] = std::bind(&Interrupts::interrupt, &interrupts, ARM11B);
    tasks[ARM11C_INTERRUPT] = std::bind(&Interrupts::interrupt, &interrupts, ARM11C);
    tasks[ARM11D_INTERRUPT] = std::bind(&Interrupts::interrupt, &interrupts, ARM11D);
    tasks[ARM9_INTERRUPT] = std::bind(&Interrupts::interrupt, &interrupts, ARM9);
    tasks[TMR11A_UNDERFLOW0] = std::bind(&Timers::underflowMp, &timers, ARM11A, 0);
    tasks[TMR11A_UNDERFLOW1] = std::bind(&Timers::underflowMp, &timers, ARM11A, 1);
    tasks[TMR11B_UNDERFLOW0] = std::bind(&Timers::underflowMp, &timers, ARM11B, 0);
    tasks[TMR11B_UNDERFLOW1] = std::bind(&Timers::underflowMp, &timers, ARM11B, 1);
    tasks[TMR11C_UNDERFLOW0] = std::bind(&Timers::underflowMp, &timers, ARM11C, 0);
    tasks[TMR11C_UNDERFLOW1] = std::bind(&Timers::underflowMp, &timers, ARM11C, 1);
    tasks[TMR11D_UNDERFLOW0] = std::bind(&Timers::underflowMp, &timers, ARM11D, 0);
    tasks[TMR11D_UNDERFLOW1] = std::bind(&Timers::underflowMp, &timers, ARM11D, 1);
    tasks[TMR9_OVERFLOW0] = std::bind(&Timers::overflowTm, &timers, 0);
    tasks[TMR9_OVERFLOW1] = std::bind(&Timers::overflowTm, &timers, 1);
    tasks[TMR9_OVERFLOW2] = std::bind(&Timers::overflowTm, &timers, 2);
    tasks[TMR9_OVERFLOW3] = std::bind(&Timers::overflowTm, &timers, 3);
    tasks[AES_UPDATE] = std::bind(&Aes::update, &aes);
    tasks[CDMA0_UPDATE] = std::bind(&Cdma::update, &cdmas[CDMA0]);
    tasks[CDMA1_UPDATE] = std::bind(&Cdma::update, &cdmas[CDMA1]);
    tasks[XDMA_UPDATE] = std::bind(&Cdma::update, &cdmas[XDMA]);
    tasks[NDMA_UPDATE] = std::bind(&Ndma::update, &ndma);
    tasks[SHA0_UPDATE] = std::bind(&Sha::update, &shas[0]);
    tasks[SHA1_UPDATE] = std::bind(&Sha::update, &shas[1]);
    tasks[Y2R0_UPDATE] = std::bind(&Y2r::update, &y2rs[0]);
    tasks[Y2R1_UPDATE] = std::bind(&Y2r::update, &y2rs[1]);
    tasks[GPU_END_FILL0] = std::bind(&Gpu::endFill, &gpu, 0);
    tasks[GPU_END_FILL1] = std::bind(&Gpu::endFill, &gpu, 1);
    tasks[GPU_END_COPY] = std::bind(&Gpu::endCopy, &gpu);
    tasks[CSND_SAMPLE] = std::bind(&Csnd::runSample, &csnd);
    tasks[SDMMC0_READ_BLOCK] = std::bind(&SdMmc::readBlock, &sdMmcs[0]);
    tasks[SDMMC1_READ_BLOCK] = std::bind(&SdMmc::readBlock, &sdMmcs[1]);
    tasks[SDMMC0_WRITE_BLOCK] = std::bind(&SdMmc::writeBlock, &sdMmcs[0]);
    tasks[SDMMC1_WRITE_BLOCK] = std::bind(&SdMmc::writeBlock, &sdMmcs[1]);
    tasks[WIFI_READ_BLOCK] = std::bind(&Wifi::readBlock, &wifi);
    tasks[WIFI_WRITE_BLOCK] = std::bind(&Wifi::writeBlock, &wifi);
    tasks[NTR_WORD_READY] = std::bind(&Cartridge::ntrWordReady, &cartridge);
    tasks[CTR_WORD_READY] = std::bind(&Cartridge::ctrWordReady, &cartridge);
    tasks[BOOT_FIRM] = std::bind(&Core::bootFirm, this);

    // Schedule the initial tasks
    schedule(RESET_CYCLES, 0x7FFFFFFFFFFFFFFF);
    schedule(END_FRAME, 268111856 / 60);
    schedule(CSND_SAMPLE, 2048);
}

Core::~Core() {
    // Clean up the DSP
    delete dsp;
}

void Core::loadFirm(const std::string &path) {
    FILE *file = fopen(path.c_str(), "rb");
    if (!file) throw ERROR_FIRM;
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    if (size < 0x200 || size > 0x4000000) {
        fclose(file);
        throw ERROR_FIRM;
    }
    firmData.resize(size);
    rewind(file);
    bool loaded = (fread(firmData.data(), 1, size, file) == size);
    fclose(file);
    if (!loaded || memcmp(firmData.data(), "FIRM", 4)) throw ERROR_FIRM;

    // Validate every section before any guest memory is changed
    bool entry9 = false, entry11 = false;
    for (int i = 0; i < 4; i++) {
        uint32_t *section = (uint32_t*)&firmData[0x40 + i * 0x30];
        uint32_t offset = section[0], address = section[1], length = section[2];
        if (!length) continue;
        uint64_t end = uint64_t(address) + length;
        bool ram = (address >= 0x08000000 && end <= (n3dsMode ? 0x08180000U : 0x08100000U)) ||
            (address >= 0x18000000 && end <= 0x18600000) ||
            (address >= 0x1FF80000 && end <= 0x20000000) ||
            (address >= 0x20000000 && end <= (n3dsMode ? 0x30000000U : 0x28000000U));
        if (((offset | address | length) & 3) || offset < 0x200 ||
                uint64_t(offset) + length > firmData.size() || !ram) throw ERROR_FIRM;
        uint32_t pc9 = U8TO32(firmData, 0xC) & ~1U, pc11 = U8TO32(firmData, 0x8) & ~1U;
        entry9 |= (pc9 >= address && pc9 < end);
        entry11 |= (pc11 >= address && pc11 < end);
    }
    if (!entry9 || !entry11) throw ERROR_FIRM;
}

void Core::scheduleFirmBoot() {
    // Replace the NAND FIRM at the BootROM handoff, after hardware/key initialization
    if (firmData.empty() || firmScheduled) return;
    firmScheduled = true;
    schedule(BOOT_FIRM, 0);
}

void Core::bootFirm() {
    for (int i = 0; i < 4; i++) {
        uint32_t *section = (uint32_t*)&firmData[0x40 + i * 0x30];
        for (uint32_t offset = 0; offset < section[2]; offset += 4)
            memory.write<uint32_t>(ARM9, section[1] + offset, U8TO32(firmData, section[0] + offset));
    }
    // Protect both boot ROMs before handing off to the chosen payload
    memory.write<uint8_t>(ARM9, 0x10000000, 3);
    memory.write<uint8_t>(ARM9, 0x10000001, 1);
    for (int i = ARM11B; i <= ARM11D; i++) arms[i].halt(BIT(0));
    arms[ARM11A].bootFirm(U8TO32(firmData, 0x8));
    arms[ARM9].bootFirm(U8TO32(firmData, 0xC));
    firmData.clear();
}

void Core::initDsp() {
    // Make a list of possible DSP tasks
    Task dspTasks[] = {
        TEAK_STOP_CYCLES, TEAK_INTERRUPT0, TEAK_INTERRUPT1, TEAK_INTERRUPT2, TEAK_INTERRUPT3,
        DSP_UNDERFLOW0, DSP_UNDERFLOW1, DSP_UNSIGNAL0, DSP_UNSIGNAL1, DSP_SEND_AUDIO, DSP_HLE_UPDATE
    };

    // Remove any tasks scheduled for the current DSP
    for (int i = 0; i < events.size(); i++)
        for (int j = 0; j < sizeof(dspTasks) / sizeof(Task); j++)
            if (events[i].task == &tasks[dspTasks[j]])
                events.erase(events.begin() + i--);

    // Clean up the current DSP
    delete dsp;
    DspLle *dspLle;
    DspHle *dspHle;

    // Create a new DSP using the selected backend and redefine its tasks
    switch (dspCurrent = Settings::dspBackend) {
    default: // Interpreter
        dsp = dspLle = new DspLle(*this);
        tasks[TEAK_STOP_CYCLES] = std::bind(&TeakInterp::stopCycles, &dspLle->teak);
        tasks[TEAK_INTERRUPT0] = std::bind(&TeakInterp::interrupt, &dspLle->teak, 0);
        tasks[TEAK_INTERRUPT1] = std::bind(&TeakInterp::interrupt, &dspLle->teak, 1);
        tasks[TEAK_INTERRUPT2] = std::bind(&TeakInterp::interrupt, &dspLle->teak, 2);
        tasks[TEAK_INTERRUPT3] = std::bind(&TeakInterp::interrupt, &dspLle->teak, 3);
        tasks[DSP_UNDERFLOW0] = std::bind(&DspLle::underflowTmr, dspLle, 0);
        tasks[DSP_UNDERFLOW1] = std::bind(&DspLle::underflowTmr, dspLle, 1);
        tasks[DSP_UNSIGNAL0] = std::bind(&DspLle::unsignalTmr, dspLle, 0);
        tasks[DSP_UNSIGNAL1] = std::bind(&DspLle::unsignalTmr, dspLle, 1);
        tasks[DSP_SEND_AUDIO] = std::bind(&DspLle::sendAudio, dspLle);
        return;

    case 1: // HLE
        dsp = dspHle = new DspHle(*this);
        tasks[DSP_HLE_UPDATE] = std::bind(&DspHle::update, dspHle);
        return;
    }
}

void Core::resetCycles() {
    // Reset the global cycle count eventually to prevent overflow
    for (uint32_t i = 0; i < events.size(); i++)
        events[i].cycles -= globalCycles;
    for (int i = 0; i < MAX_CPUS; i++)
        arms[i].resetCycles();
    dsp->resetCycles();
    timers.resetCycles();
    globalCycles = 0;
    schedule(RESET_CYCLES, 0x7FFFFFFFFFFFFFFF);
}

void Core::endFrame() {
    // Break execution at the end of a frame and count it
    running.store(false);
    // BootROM publishes its WiFi-flash error only after all boot sources fail.
    // Direct FIRM boot can still proceed with initialized hardware and no bootable NAND.
    if (!firmData.empty() && !firmScheduled) {
        uint8_t wifiError = memory.read<uint32_t>(ARM9, 0x1FFFE00C) >> 16;
        if (wifiError && wifiError != 0xFF) scheduleFirmBoot();
    }
    fpsCount++;

    // Update the FPS counter and save files every second
    std::chrono::duration<double> fpsTime = std::chrono::steady_clock::now() - lastFpsTime;
    if (fpsTime.count() >= 1.0f) {
        fps = fpsCount;
        fpsCount = 0;
        lastFpsTime = std::chrono::steady_clock::now();
        cartridge.updateSave();
        i2c.updateMcuRam();
    }

    // Handle per-frame tasks and schedule the next one
    pdc.drawFrame();
    input.updateHome();
    schedule(END_FRAME, 268111856 / 60);
}

void Core::updateRunFunc() {
    // Swap out the run function based on ARM11 cores 2/3 and DSP backend
    bool dspOff = (dspCurrent == 1 || ((DspLle*)dsp)->teak.cycles == -1);
    if ((interrupts.cfg11MpBootcnt[0] | interrupts.cfg11MpBootcnt[1]) & BIT(4)) // Cores enabled
        runFunc = dspOff ? &ArmInterp::runFrame<true, false> : &ArmInterp::runFrame<true, true>;
    else // Cores disabled
        runFunc = dspOff ? &ArmInterp::runFrame<false, false> : &ArmInterp::runFrame<false, true>;
    running.store(false);
}

void Core::schedule(Task task, uint64_t cycles) {
    // Add a task to the scheduler, sorted by least to most cycles until execution
    Event event(&tasks[task], globalCycles + cycles);
    auto it = std::upper_bound(events.cbegin(), events.cend(), event);
    events.insert(it, event);
}
