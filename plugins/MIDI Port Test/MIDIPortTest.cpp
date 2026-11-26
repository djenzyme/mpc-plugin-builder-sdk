// MIDI Port Test - an MPC instrument plugin by ArpLab
//
// Purpose: find out whether an SDK plugin can publish a virtual MIDI port on the
// MPC. The official host discards VST MIDI output, so this plugin does what Acid
// does: it talks to the Linux ALSA sequencer directly and creates its own port.
//
// libasound is NOT linked. The SDK validator only allows libc/libm/libpthread/libdl,
// so the library is opened at run time with dlopen() and the handful of functions
// we need are looked up with dlsym(). If anything is missing the plugin still loads
// and the "Port Status" parameter tells you what failed.
//
// What it does once the port exists:
//   * Pass MIDI: notes / CC / pitch bend / program change arriving on the track are
//     forwarded to the port on the channel chosen by "Out Chan".
//   * Test Beat: sends a short note every half second, so you can check the port
//     without playing anything.
//
// The plugin makes no sound. Route the port to another track to hear it.
//
// Threading: the ALSA port is opened in prepare() (UI thread). process() only
// calls snd_seq_event_output / snd_seq_drain_output on a non-blocking client,
// the same calls Acid imports. A full output queue drops events instead of blocking.

#include "mpc_plugin.h"

#include <dlfcn.h>
#include <cstdint>
#include <cstring>

namespace {

// ---------------------------------------------------------------------------
// Minimal copy of the ALSA sequencer ABI (alsa/seq_event.h). No ALSA headers are
// needed to build. Layout: 28 bytes, checked below.
// ---------------------------------------------------------------------------
struct AlsaAddr { uint8_t client, port; };

struct AlsaEvent
{
    uint8_t  type, flags, tag, queue;
    uint32_t time[2];
    AlsaAddr source, dest;
    union
    {
        struct { uint8_t channel, note, velocity, offVelocity; uint32_t duration; } note;
        struct { uint8_t channel, unused[3]; uint32_t param; int32_t value; } control;
        uint8_t raw[12];
    } data;
};
static_assert(sizeof(AlsaEvent) == 28, "AlsaEvent must match snd_seq_event_t (28 bytes)");

constexpr uint8_t kEvNoteOn = 6, kEvNoteOff = 7, kEvController = 10, kEvPgmChange = 11,
                  kEvChanPress = 12, kEvPitchBend = 13;
constexpr uint8_t kQueueDirect = 253, kAddrSubscribers = 254, kAddrUnknown = 253;
constexpr int      kOpenOutput = 1, kOpenNonblock = 1;                 // SND_SEQ_OPEN_OUTPUT, SND_SEQ_NONBLOCK
constexpr unsigned kPortCaps = 1u | 32u;                               // READ | SUBS_READ
constexpr unsigned kPortType = 2u | (1u << 20);                        // MIDI_GENERIC | APPLICATION

typedef int (*FnOpen)(void** seq, const char* name, int streams, int mode);
typedef int (*FnSetName)(void* seq, const char* name);
typedef int (*FnCreatePort)(void* seq, const char* name, unsigned caps, unsigned type);
typedef int (*FnOutput)(void* seq, AlsaEvent* ev);
typedef int (*FnDrain)(void* seq);
typedef int (*FnClose)(void* seq);

enum ParamIndex { kOutChan = 0, kPass, kBeat, kBeatNote, kStatus, kNumParams };
enum Status { kIdle = 0, kPortOpen, kNoLib, kNoSymbols, kOpenFailed, kPortFailed };

} // namespace

class MIDIPortTest : public mpc::Plugin
{
public:
    MIDIPortTest() : mpc::Plugin(makeInfo())
    {
        // Param 0 is nudged by the host on every prepare: keep it harmless.
        addParam(mpc::intParam   ("chan",   "Out Chan",    1, 16, 1));
        addParam(mpc::boolParam  ("pass",   "Pass MIDI",   true));
        addParam(mpc::boolParam  ("beat",   "Test Beat",   false));
        addParam(mpc::intParam   ("bnote",  "Beat Note",   0, 127, 60));
        addParam(mpc::choiceParam("status", "Port Status",
                 {"Not opened", "Port open", "No libasound", "Missing funcs", "Seq open fail", "Port fail"}, 0));
        addProgram("Init", {1, 1, 0, 60, 0});
    }

    ~MIDIPortTest() override { closePort(); }

    // UI thread. Opens the port once; later calls just refresh the status value.
    void prepare(double sampleRate, int maxBlockSize) override
    {
        (void)maxBlockSize;
        sampleRate_ = sampleRate;
        if (!portReady_.load() && lib_ == nullptr) openPort();
        set(kStatus, (float)status_);
    }

    void reset() override
    {
        allNotesOff();
        beatOn_ = false;
        beatPos_ = 0;
    }

    void process(mpc::ProcessContext& ctx) override
    {
        if (!portReady_.load(std::memory_order_acquire)) return;

        const int ch = mpc::clamp(getInt(kOutChan), 1, 16) - 1;
        bool sent = false;

        if (getBool(kPass))
        {
            for (int i = 0; i < ctx.numMidiEvents; ++i)
                sent |= forward(ctx.midi[i], ch);
        }

        // Test beat: block-accurate, which is plenty for a connectivity test.
        const int  period = (int)(sampleRate_ * 0.5);
        const int  gate   = (int)(sampleRate_ * 0.1);
        const bool want   = getBool(kBeat);
        if (!want && beatOn_) { sent |= sendNote(false, ch, beatNote_, 0); beatOn_ = false; }
        if (want)
        {
            beatPos_ += ctx.numFrames;
            if (!beatOn_ && beatPos_ >= period)
            {
                beatPos_ -= period;
                beatNote_ = mpc::clamp(getInt(kBeatNote), 0, 127);
                sent |= sendNote(true, ch, beatNote_, 100);
                beatOn_ = true;
                offIn_ = gate;
            }
            if (beatOn_)
            {
                offIn_ -= ctx.numFrames;
                if (offIn_ <= 0) { sent |= sendNote(false, ch, beatNote_, 0); beatOn_ = false; }
            }
        }

        if (sent) drain_(seq_);
    }

private:
    static mpc::PluginInfo makeInfo()
    {
        mpc::PluginInfo info;
        info.name         = "MIDI Port Test";
        info.vendor       = "ArpLab";
        info.uniqueId     = MPC_FOURCC('A', 'p', 'l', 'T');
        info.version      = 1000;
        info.isInstrument = true;
        info.numInputs    = 0;
        info.numOutputs   = 2;
        info.tailSamples  = 0;
        return info;
    }

    // ---- ALSA plumbing ------------------------------------------------------
    void openPort()
    {
        lib_ = dlopen("libasound.so.2", RTLD_NOW | RTLD_LOCAL);
        if (!lib_) lib_ = dlopen("libasound.so", RTLD_NOW | RTLD_LOCAL);
        if (!lib_) { status_ = kNoLib; return; }

        FnOpen       open_    = (FnOpen)      dlsym(lib_, "snd_seq_open");
        FnSetName    setName  = (FnSetName)   dlsym(lib_, "snd_seq_set_client_name");
        FnCreatePort mkPort   = (FnCreatePort)dlsym(lib_, "snd_seq_create_simple_port");
        output_ = (FnOutput)dlsym(lib_, "snd_seq_event_output");
        drain_  = (FnDrain) dlsym(lib_, "snd_seq_drain_output");
        close_  = (FnClose) dlsym(lib_, "snd_seq_close");
        if (!open_ || !setName || !mkPort || !output_ || !drain_ || !close_)
        { status_ = kNoSymbols; dlclose(lib_); lib_ = nullptr; return; }

        if (open_(&seq_, "default", kOpenOutput, kOpenNonblock) < 0 || !seq_)
        { seq_ = nullptr; status_ = kOpenFailed; dlclose(lib_); lib_ = nullptr; return; }

        setName(seq_, "MIDI Port Test");
        int port = mkPort(seq_, "Out", kPortCaps, kPortType);
        if (port < 0)
        { close_(seq_); seq_ = nullptr; status_ = kPortFailed; dlclose(lib_); lib_ = nullptr; return; }

        port_ = (uint8_t)port;
        status_ = kPortOpen;
        portReady_.store(true, std::memory_order_release);
    }

    void closePort()
    {
        if (portReady_.load())
        {
            allNotesOff();
            portReady_.store(false, std::memory_order_release);
        }
        if (seq_ && close_) close_(seq_);
        seq_ = nullptr;
        if (lib_) dlclose(lib_);
        lib_ = nullptr;
    }

    bool push(AlsaEvent& ev)
    {
        ev.queue = kQueueDirect;
        ev.source.port = port_;
        ev.dest.client = kAddrSubscribers;
        ev.dest.port = kAddrUnknown;
        return output_(seq_, &ev) >= 0;
    }

    bool sendNote(bool on, int ch, int note, int vel)
    {
        AlsaEvent ev; std::memset(&ev, 0, sizeof ev);
        ev.type = on ? kEvNoteOn : kEvNoteOff;
        ev.data.note.channel = (uint8_t)ch;
        ev.data.note.note = (uint8_t)note;
        ev.data.note.velocity = (uint8_t)vel;
        return push(ev);
    }

    bool sendControl(uint8_t type, int ch, int param, int value)
    {
        AlsaEvent ev; std::memset(&ev, 0, sizeof ev);
        ev.type = type;
        ev.data.control.channel = (uint8_t)ch;
        ev.data.control.param = (uint32_t)param;
        ev.data.control.value = value;
        return push(ev);
    }

    bool forward(const mpc::MidiEvent& e, int ch)
    {
        if (e.isNoteOn())        return sendNote(true,  ch, e.note(), e.velocity());
        if (e.isNoteOff())       return sendNote(false, ch, e.note(), 0);
        if (e.isController())    return sendControl(kEvController, ch, e.controller(), e.controllerValue());
        if (e.isPitchBend())     return sendControl(kEvPitchBend,  ch, 0, e.pitchBend());
        if (e.isAftertouch())    return sendControl(kEvChanPress,  ch, 0, e.data[1]);
        if (e.isProgramChange()) return sendControl(kEvPgmChange,  ch, 0, e.data[1]);
        return false;
    }

    void allNotesOff()
    {
        if (!portReady_.load()) return;
        const int ch = mpc::clamp(getInt(kOutChan), 1, 16) - 1;
        sendControl(kEvController, ch, 123, 0);
        sendControl(kEvController, ch, 120, 0);
        drain_(seq_);
    }

    double sampleRate_ = 44100.0;

    void*        lib_ = nullptr;
    void*        seq_ = nullptr;
    FnOutput     output_ = nullptr;
    FnDrain      drain_  = nullptr;
    FnClose      close_  = nullptr;
    uint8_t      port_ = 0;
    int          status_ = kIdle;
    std::atomic<bool> portReady_{false};

    bool beatOn_ = false;
    int  beatPos_ = 0, offIn_ = 0, beatNote_ = 60;
};

MPC_PLUGIN_ENTRY(MIDIPortTest)
