// SdxClap.cpp
// TB SDX-330 (the Roland SDX-330 Dimensional Expander) as a CLAP plugin. clap-wrapper builds the
// VST3 and AU versions from this same code, so all three formats behave identically. The DSP,
// the programs and the panel are the Mountain Lion build's, unchanged.
// BYPASS is the plugin's own parameter (not flagged as the host's bypass), so it bypasses this
// plugin only: it fades to the dry input over about 10 ms and programs leave it alone.
//
// Threads:
//  - Audio thread: process() / flush(). Only these touch the DSP.
//  - Main thread: parameter info, state, the panel and the program memory.
//  Parameter values live in atomics both sides can read. Changes from the panel
//  (or a loaded state) travel to the audio thread through a lock-free FIFO and
//  are reported to the host as gesture/value events.

#include <clap/clap.h>

#include "Editor.h"
#include "Params.h"
#include "Programs.h"
#include "SdxClap.h"
#include "../DSP/Sdx330.h"
#include "../DSP/Factory.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <string>
#include <strings.h>
#include <cstdint>

#if defined(__SSE__) || defined(__x86_64__) || defined(_M_X64)
  #include <xmmintrin.h>
  #define TBSDX_HAVE_SSE 1
#endif

using namespace tbsdx;

namespace
{

const char* const kPluginId = "com.tonybuzzcock.tb-sdx-330";

const char* const kFeatures[] = { CLAP_PLUGIN_FEATURE_AUDIO_EFFECT, CLAP_PLUGIN_FEATURE_CHORUS, CLAP_PLUGIN_FEATURE_FLANGER,
                                  CLAP_PLUGIN_FEATURE_STEREO, nullptr };

const clap_plugin_descriptor_t kDescriptor = {
    CLAP_VERSION_INIT,
    kPluginId,
    "TB SDX-330",
    "TB",
    "https://github.com/tonybuzzcock/TB-Stereo-Chorus-sdx330-2026-MAC",
    "",
    "",
    "1.0.0",
    "Model of the Roland SDX-330 Dimensional Expander: 16 chorus algorithms, 100 factory programs",
    kFeatures
};

// The group each parameter shows under in hosts that sort by module
const char* moduleOf (int i)
{
    if (i <= kBypass)       return "Panel";
    if (i <= kDirectMix)    return "System";
    if (i <= kInMode)       return "Program";
    if (i <= kHigQ)         return "EQ";
    if (i <= kPanR)         return "Chorus";
    if (i <= kElevR)        return "3D";
    if (i <= kPanStart)     return "3D Panner";
    if (i == kMode)         return "Mode";
    if (i <= kXov3)         return "Band Chorus";
    if (i < kDtPit1)        return "8 Phase Chorus";
    if (i < kIntns)         return "Detune Chorus";
    if (i == kIntns)        return "Vintage Chorus";
    if (i <= kResnc)        return "Flanger";
    if (i <= kDiffusion)    return "Rotary";
    return "Ambience Chorus";
}

// Values the host sees: choices as their index, Algorithm / Program / Panner Speed as the
// unit's numbers, everything else 0..1 as the panel's knobs
bool isStepped (int i) { return kInfo[i].taper == tChoice || i == kAlgo || i == kProgram || i == kPanSpeed; }
bool realPlain (int i) { return i == kAlgo || i == kProgram || i == kPanSpeed; }
double plainMin (int i) { return realPlain (i) ? kInfo[i].lo : 0.0; }
double plainMax (int i) { return kInfo[i].taper == tChoice ? kInfo[i].hi : (realPlain (i) ? kInfo[i].hi : 1.0); }

double toPlain (int i, float internal)
{
    if (kInfo[i].taper == tChoice || realPlain (i)) return toReal (i, internal);
    return internal;
}

float fromPlain (int i, double plain)
{
    if (! (plain == plain)) return defaultNorm (i);
    const double lo = plainMin (i), hi = plainMax (i);
    if (plain < lo) plain = lo;
    if (plain > hi) plain = hi;
    if (kInfo[i].taper == tChoice || realPlain (i)) return (float) toNorm (i, std::floor (plain + 0.5));
    return (float) plain;
}

const char* unitLabel (int i)
{
    const int u = kInfo[i].unit;
    if (u == uHz) return "Hz";
    if (u == uMs) return "ms";
    if (u == uDeg) return "deg";
    if (u == udB) return "dB";
    if (i >= kDtPit1 && i < kDtPit1 + 6) return "cent";
    return "";
}

void formatValue (int i, float v, char* buf, size_t size, bool withUnit, int algo)
{
    char t[32];
    formatParamFor (i, v, algo, t, sizeof (t));
    if (i == kAlgo && withUnit)
    {
        const int a = algoIndex (v);
        std::snprintf (buf, size, "%d %s", a + 1, kAlgoNames[a]);
        return;
    }
    const char* u = withUnit ? unitLabel (i) : "";
    if (*u) std::snprintf (buf, size, "%s %s", t, u);
    else    std::snprintf (buf, size, "%s", t);
}

// Single-producer (main thread) / single-consumer (audio thread) queue.
struct ParamMsg { int16_t index; uint8_t kind; float value; };
enum { kMsgValue = 0, kMsgBegin, kMsgEnd, kMsgValueQuiet };   // quiet: state load, host already knows

class MsgQueue
{
public:
    MsgQueue() : head (0), tail (0) {}
    bool push (const ParamMsg& m)
    {
        const unsigned h = head.load (std::memory_order_relaxed);
        const unsigned n = (h + 1) & (kSize - 1);
        if (n == tail.load (std::memory_order_acquire)) return false;   // full
        buf[h] = m;
        head.store (n, std::memory_order_release);
        return true;
    }
    bool pop (ParamMsg& m)
    {
        const unsigned t = tail.load (std::memory_order_relaxed);
        if (t == head.load (std::memory_order_acquire)) return false;
        m = buf[t];
        tail.store ((t + 1) & (kSize - 1), std::memory_order_release);
        return true;
    }
private:
    enum { kSize = 4096 };     // a program load sends every parameter
    ParamMsg buf[kSize];
    std::atomic<unsigned> head, tail;
};

//==============================================================================
class Plugin : public EditorHost
{
public:
    explicit Plugin (const clap_host_t* h)
        : host (h), hostParams (nullptr), active (false), sampleRate (44100.0),
          editor (nullptr), guiCreated (false), cacheNumber (-1)
    {
        std::memset (&plugin, 0, sizeof (plugin));
        plugin.desc = &kDescriptor;
        plugin.plugin_data = this;
        plugin.init = &sInit;
        plugin.destroy = &sDestroy;
        plugin.activate = &sActivate;
        plugin.deactivate = &sDeactivate;
        plugin.start_processing = &sStartProcessing;
        plugin.stop_processing = &sStopProcessing;
        plugin.reset = &sReset;
        plugin.process = &sProcess;
        plugin.get_extension = &sGetExtension;
        plugin.on_main_thread = &sOnMainThread;

        // Power on as the unit does: program 201 STEREO CHORUS
        float v[kNumParams];
        for (int i = 0; i < kNumParams; ++i) v[i] = defaultNorm (i);
        programs.get (201, v);
        v[kProgram] = programValue (201);
        for (int i = 0; i < kNumParams; ++i) { values[i].store (v[i]); powerOn[i] = v[i]; }
    }

    ~Plugin()
    {
#if defined(__APPLE__)
        if (editor) editorClose (editor);
#endif
    }

    clap_plugin_t plugin;
    float powerOn[kNumParams];

    //--------------------------------------------------------------------------
    // EditorHost (main thread)
    float getParam (int index) const override
    {
        return (index >= 0 && index < kNumParams) ? values[index].load (std::memory_order_relaxed) : 0.0f;
    }
    void getParamDisplay (int index, char* text) const override
    {
        if (index < 0 || index >= kNumParams) { text[0] = 0; return; }
        formatValue (index, getParam (index), text, 9, false, algoIndex (getParam (kAlgo)));
    }
    float getMeter (int side) const override { return unit.meter (side); }
    void guiBeginEdit (int index) override { queue (index, kMsgBegin, 0.0f); }
    void guiEndEdit (int index) override   { queue (index, kMsgEnd, 0.0f); requestFlush(); }
    void guiSetParam (int index, float v) override
    {
        if (index < 0 || index >= kNumParams) return;
        v = clamp01f (v);
        values[index].store (v);
        queue (index, kMsgValue, v);
        requestFlush();
    }

    // Program memory: user 1-200 (shared file), presets 201-300
    void refreshPrograms() override { programs.load(); cacheNumber = -1; }
    void programName (int number, char* text, int size) const override
    {
        if (! text || size <= 0) return;
        const std::string n = programs.name (number);
        std::strncpy (text, n.c_str(), (size_t) size - 1);
        text[size - 1] = 0;
    }
    bool loadProgram (int number) override
    {
        if (number < 1 || number > kNumPrograms) return false;
        float v[kNumParams];
        for (int i = 0; i < kNumParams; ++i) v[i] = getParam (i);
        programs.get (number, v);
        // The number first: the unit fades the effect out while the new program comes in
        guiBeginEdit (kProgram); guiSetParam (kProgram, programValue (number)); guiEndEdit (kProgram);
        for (int i = kAlgo; i < kNumParams; ++i)
        {
            if (getParam (i) == v[i]) continue;
            guiBeginEdit (i); guiSetParam (i, v[i]); guiEndEdit (i);
        }
        cacheNumber = -1;
        return true;
    }
    bool writeProgram (int number) override
    {
        if (! ProgramStore::isUser (number)) return false;
        float v[kNumParams];
        for (int i = 0; i < kNumParams; ++i) v[i] = getParam (i);
        programs.load();                        // keep programs written by other instances
        if (! programs.put (number, v)) return false;
        const bool ok = programs.save();
        guiBeginEdit (kProgram); guiSetParam (kProgram, programValue (number)); guiEndEdit (kProgram);
        cacheNumber = -1;
        return ok;
    }
    bool renameProgram (int number, const char* name) override
    {
        programs.load();
        if (! programs.setName (number, name ? name : "")) return false;
        return programs.save();
    }
    bool programEdited() const override
    {
        const int number = programNumber (getParam (kProgram));
        if (number != cacheNumber)
        {
            for (int i = 0; i < kNumParams; ++i) cache[i] = getParam (i);
            programs.get (number, cache);
            cacheNumber = number;
        }
        for (int i = kAlgo; i < kNumParams; ++i)
            if (std::fabs (cache[i] - getParam (i)) > 1e-4f) return true;
        return false;
    }

private:
    static Plugin* self (const clap_plugin_t* p) { return static_cast<Plugin*> (p->plugin_data); }
    static float clamp01f (float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

    void queue (int index, uint8_t kind, float v)
    {
        ParamMsg m; m.index = (int16_t) index; m.kind = kind; m.value = v;
        if (! msgs.push (m) && kind != kMsgEnd && kind != kMsgBegin) overflowed.store (true);
    }
    void requestFlush()
    {
        if (hostParams && hostParams->request_flush) hostParams->request_flush (host);
    }

    // Audio thread (or main thread while inactive): push a value into the DSP
    void applyToDsp (int index) { unit.setParam (index, values[index].load (std::memory_order_relaxed)); }

    // Apply panel/state messages to the DSP and tell the host about panel edits.
    void drainMessages (const clap_output_events_t* out)
    {
        // If the queue ever filled up (many program loads with no audio running), take
        // every value from the atomics so the DSP never misses one
        if (overflowed.exchange (false))
            for (int i = 0; i < kNumParams; ++i) applyToDsp (i);
        ParamMsg m;
        while (msgs.pop (m))
        {
            if (m.index < 0 || m.index >= kNumParams) continue;
            if (m.kind == kMsgValue || m.kind == kMsgValueQuiet) applyToDsp (m.index);
            if (! out || m.kind == kMsgValueQuiet) continue;
            if (m.kind == kMsgValue)
            {
                clap_event_param_value_t e;
                std::memset (&e, 0, sizeof (e));
                e.header.size = sizeof (e);
                e.header.time = 0;
                e.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
                e.header.type = CLAP_EVENT_PARAM_VALUE;
                e.param_id = (clap_id) m.index;
                e.note_id = -1; e.port_index = -1; e.channel = -1; e.key = -1;
                e.value = toPlain (m.index, m.value);
                out->try_push (out, &e.header);
            }
            else
            {
                clap_event_param_gesture_t e;
                std::memset (&e, 0, sizeof (e));
                e.header.size = sizeof (e);
                e.header.time = 0;
                e.header.space_id = CLAP_CORE_EVENT_SPACE_ID;
                e.header.type = (m.kind == kMsgBegin) ? CLAP_EVENT_PARAM_GESTURE_BEGIN : CLAP_EVENT_PARAM_GESTURE_END;
                e.param_id = (clap_id) m.index;
                out->try_push (out, &e.header);
            }
        }
    }

    void handleInputEvents (const clap_input_events_t* in)
    {
        if (! in) return;
        const uint32_t n = in->size (in);
        for (uint32_t k = 0; k < n; ++k)
        {
            const clap_event_header_t* h = in->get (in, k);
            if (! h || h->space_id != CLAP_CORE_EVENT_SPACE_ID || h->type != CLAP_EVENT_PARAM_VALUE) continue;
            const clap_event_param_value_t* e = (const clap_event_param_value_t*) h;
            if (e->param_id >= (clap_id) kNumParams) continue;
            const int i = (int) e->param_id;
            values[i].store (fromPlain (i, e->value));
            applyToDsp (i);
        }
    }

    //--------------------------------------------------------------------------
    // clap_plugin
    static bool sInit (const clap_plugin_t* p)
    {
        Plugin* s = self (p);
        s->hostParams = (const clap_host_params_t*) s->host->get_extension (s->host, CLAP_EXT_PARAMS);
        for (int i = 0; i < kNumParams; ++i) s->applyToDsp (i);
        s->unit.prepare (s->sampleRate);
        s->prepareBypass();
        return true;
    }
    static void sDestroy (const clap_plugin_t* p) { delete self (p); }
    static bool sActivate (const clap_plugin_t* p, double sr, uint32_t, uint32_t)
    {
        Plugin* s = self (p);
        ParamMsg m;
        while (s->msgs.pop (m)) {}                       // values are already in the atomics
        for (int i = 0; i < kNumParams; ++i) s->applyToDsp (i);
        s->sampleRate = sr;
        s->unit.prepare (sr);
        s->prepareBypass();
        s->active = true;
        return true;
    }
    static void sDeactivate (const clap_plugin_t* p) { self (p)->active = false; }
    static bool sStartProcessing (const clap_plugin_t*) { return true; }
    static void sStopProcessing (const clap_plugin_t*) {}
    static void sReset (const clap_plugin_t* p) { self (p)->unit.reset(); }
    static void sOnMainThread (const clap_plugin_t*) {}

    static clap_process_status sProcess (const clap_plugin_t* p, const clap_process_t* proc)
    {
        Plugin* s = self (p);
        s->drainMessages (proc->out_events);
        s->handleInputEvents (proc->in_events);

        const uint32_t n = proc->frames_count;
        if (n == 0 || proc->audio_inputs_count < 1 || proc->audio_outputs_count < 1) return CLAP_PROCESS_CONTINUE;
        const clap_audio_buffer_t& ib = proc->audio_inputs[0];
        const clap_audio_buffer_t& ob = proc->audio_outputs[0];
        if (! ib.data32 || ! ob.data32 || ob.channel_count < 1 || ib.channel_count < 1) return CLAP_PROCESS_CONTINUE;

        const float* inL = ib.data32[0];
        const float* inR = ib.channel_count > 1 ? ib.data32[1] : ib.data32[0];
        float* outL = ob.data32[0];
        float* outR = ob.channel_count > 1 ? ob.data32[1] : nullptr;

#if TBSDX_HAVE_SSE
        const unsigned int oldCsr = _mm_getcsr();
        _mm_setcsr (oldCsr | 0x8040);   // flush denormals to zero
#endif
#if defined(__aarch64__)
        uint64_t oldFpcr;
        __asm__ __volatile__ ("mrs %0, fpcr" : "=r" (oldFpcr));
        __asm__ __volatile__ ("msr fpcr, %0" : : "r" (oldFpcr | (1ull << 24)));   // FZ: flush denormals to zero
#endif
        // The SDX-330 is a stereo unit: one engine for both sides. Inputs may alias the
        // outputs, so go through scratch.
        uint32_t done = 0;
        while (done < n)
        {
            const uint32_t chunk = (n - done) < (uint32_t) kScratch ? (n - done) : (uint32_t) kScratch;
            std::memcpy (s->inBuf[0], inL + done, chunk * sizeof (float));
            std::memcpy (s->inBuf[1], inR + done, chunk * sizeof (float));
            float* oR = outR ? outR + done : s->spareR;
            s->unit.process (s->inBuf[0], s->inBuf[1], outL + done, oR, (int) chunk);
            // BYPASS: fade to the dry input (no latency, so the two line up), then pass it bit for bit.
            // This plugin's own switch, not the host's bypass.
            const float target = isOn (s->getParam (kBypass)) ? 1.0f : 0.0f;
            if (target != s->byp || s->byp != 0.0f)
            {
                const float start = s->byp;
                s->byp = s->applyBypass (outL + done, s->inBuf[0], chunk, start, target);
                if (outR) s->applyBypass (outR + done, s->inBuf[1], chunk, start, target);
            }
            done += chunk;
        }
        // No subnormal numbers leave the plugin, whatever the CPU
        for (uint32_t i = 0; i < n; ++i)
        {
            if (std::fabs (outL[i]) < 1.0e-30f) outL[i] = 0.0f;
            if (outR && std::fabs (outR[i]) < 1.0e-30f) outR[i] = 0.0f;
        }
#if defined(__aarch64__)
        __asm__ __volatile__ ("msr fpcr, %0" : : "r" (oldFpcr));
#endif
#if TBSDX_HAVE_SSE
        _mm_setcsr (oldCsr);
#endif
        return CLAP_PROCESS_CONTINUE;
    }

    //--------------------------------------------------------------------------
    // Extensions
    static const void* sGetExtension (const clap_plugin_t*, const char* id)
    {
        if (! std::strcmp (id, CLAP_EXT_AUDIO_PORTS)) return &audioPortsExt;
        if (! std::strcmp (id, CLAP_EXT_PARAMS))      return &paramsExt;
        if (! std::strcmp (id, CLAP_EXT_STATE))       return &stateExt;
        if (! std::strcmp (id, CLAP_EXT_LATENCY))     return &latencyExt;
        if (! std::strcmp (id, CLAP_EXT_TAIL))        return &tailExt;
#if defined(__APPLE__)
        if (! std::strcmp (id, CLAP_EXT_GUI))         return &guiExt;
#endif
        return nullptr;
    }

    // Audio ports: one stereo in, one stereo out
    static uint32_t apCount (const clap_plugin_t*, bool) { return 1; }
    static bool apGet (const clap_plugin_t*, uint32_t index, bool isInput, clap_audio_port_info_t* info)
    {
        if (index != 0) return false;
        info->id = isInput ? 0 : 1;
        std::snprintf (info->name, sizeof (info->name), "%s", isInput ? "In" : "Out");
        info->flags = CLAP_AUDIO_PORT_IS_MAIN;
        info->channel_count = 2;
        info->port_type = CLAP_PORT_STEREO;
        info->in_place_pair = isInput ? 1 : 0;
        return true;
    }
    static const clap_plugin_audio_ports_t audioPortsExt;

    // Parameters
    static uint32_t pCount (const clap_plugin_t*) { return kNumParams; }
    static bool pInfo (const clap_plugin_t* p, uint32_t index, clap_param_info_t* info)
    {
        if (index >= (uint32_t) kNumParams) return false;
        const int i = (int) index;
        std::memset (info, 0, sizeof (*info));
        info->id = (clap_id) i;
        info->flags = CLAP_PARAM_IS_AUTOMATABLE | (isStepped (i) ? CLAP_PARAM_IS_STEPPED : 0);
        std::snprintf (info->name, sizeof (info->name), "%s", kInfo[i].vst);
        std::snprintf (info->module, sizeof (info->module), "%s", moduleOf (i));
        info->min_value = plainMin (i);
        info->max_value = plainMax (i);
        info->default_value = toPlain (i, self (p)->powerOn[i]);
        return true;
    }
    static bool pValue (const clap_plugin_t* p, clap_id id, double* out)
    {
        if (id >= (clap_id) kNumParams) return false;
        *out = toPlain ((int) id, self (p)->getParam ((int) id));
        return true;
    }
    static bool pToText (const clap_plugin_t* p, clap_id id, double value, char* out, uint32_t size)
    {
        if (id >= (clap_id) kNumParams || size == 0) return false;
        formatValue ((int) id, fromPlain ((int) id, value), out, size, true, algoIndex (self (p)->getParam (kAlgo)));
        return true;
    }
    static bool pFromText (const clap_plugin_t* p, clap_id id, const char* text, double* out)
    {
        if (id >= (clap_id) kNumParams || ! text) return false;
        const int i = (int) id;
        while (*text == ' ') ++text;
        // the value without its unit ("0.65 Hz" -> "0.65")
        char t[64];
        int n = 0;
        while (text[n] && text[n] != ' ' && n < 63) { t[n] = text[n]; ++n; }
        t[n] = 0;
        if (! n) return false;
        const int algo = algoIndex (self (p)->getParam (kAlgo));
        double r;
        if (kInfo[i].taper == tChoice)
        {
            for (int k = 0; k <= (int) kInfo[i].hi; ++k)
                if (! strcasecmp (t, kInfo[i].choices[k])) { *out = k; return true; }
            if (i == kMode)
            {
                const int m = std::atoi (t);
                if (m >= 1 && m <= 7) { *out = m - 1; return true; }
            }
            if (i == kBypass)
            {
                if (! strcasecmp (t, "on") || ! strcasecmp (t, "1")) { *out = 1.0; return true; }
                if (! strcasecmp (t, "off") || ! strcasecmp (t, "0")) { *out = 0.0; return true; }
            }
            char* e = nullptr;
            const double d = std::strtod (t, &e);
            if (e == t || d < 0.0 || d > kInfo[i].hi) return false;
            *out = std::floor (d + 0.5);
            return true;
        }
        (void) algo;
        if (! parseValue (i, t, r)) return false;
        if (realPlain (i)) { *out = std::floor (r + 0.5); return true; }
        *out = toNorm (i, r);
        return true;
    }
    static void pFlush (const clap_plugin_t* p, const clap_input_events_t* in, const clap_output_events_t* out)
    {
        Plugin* s = self (p);
        s->drainMessages (out);
        s->handleInputEvents (in);
    }
    static const clap_plugin_params_t paramsExt;

    // State: every parameter as text, so it survives future versions
    static bool sSave (const clap_plugin_t* p, const clap_ostream_t* stream)
    {
        Plugin* s = self (p);
        std::string text = "TBSDX330-STATE 1\n";
        char line[96];
        for (int i = 0; i < kNumParams; ++i)
        {
            std::snprintf (line, sizeof (line), "%d %.7f\n", i, (double) s->getParam (i));
            text += line;
        }
        const char* data = text.c_str();
        int64_t left = (int64_t) text.size();
        while (left > 0)
        {
            const int64_t w = stream->write (stream, data, (uint64_t) left);
            if (w <= 0) return false;
            data += w; left -= w;
        }
        return true;
    }
    static bool sLoad (const clap_plugin_t* p, const clap_istream_t* stream)
    {
        Plugin* s = self (p);
        std::string text;
        char buf[512];
        for (int guard = 0; guard < 256; ++guard)
        {
            const int64_t r = stream->read (stream, buf, sizeof (buf));
            if (r < 0) return false;
            if (r == 0) break;
            text.append (buf, (size_t) r);
        }
        static const char kHeader[] = "TBSDX330-STATE 1";
        const size_t hl = sizeof (kHeader) - 1;
        if (text.compare (0, hl, kHeader) != 0) return false;
        const char* q = text.c_str() + hl;
        while (*q)
        {
            char* end = nullptr;
            const long idx = std::strtol (q, &end, 10);
            if (end == q) break;
            q = end;
            const double v = std::strtod (q, &end);
            if (end == q) break;
            q = end;
            if (idx < 0 || idx >= kNumParams || ! (v == v)) continue;
            const float f = clamp01f ((float) v);   // state holds internal 0..1 values
            s->values[idx].store (f);
            if (s->active) s->queue ((int) idx, kMsgValueQuiet, f);
            else s->applyToDsp ((int) idx);
        }
        if (s->hostParams)
        {
            if (s->hostParams->rescan) s->hostParams->rescan (s->host, CLAP_PARAM_RESCAN_VALUES);
            s->requestFlush();
        }
        return true;
    }
    static const clap_plugin_state_t stateExt;

    static uint32_t latGet (const clap_plugin_t*) { return (uint32_t) Sdx330::latencySamples(); }
    static const clap_plugin_latency_t latencyExt;

    // The ambience rings up to 3 s; the flanger's feedback dies away well inside that
    static uint32_t tailGet (const clap_plugin_t* p) { return (uint32_t) (self (p)->sampleRate * 4.0); }
    static const clap_plugin_tail_t tailExt;

#if defined(__APPLE__)
    // GUI: the Cocoa panel, fixed size
    static bool gSupported (const clap_plugin_t*, const char* api, bool floating)
    {
        return ! floating && api && ! std::strcmp (api, CLAP_WINDOW_API_COCOA);
    }
    static bool gPreferred (const clap_plugin_t*, const char** api, bool* floating)
    {
        *api = CLAP_WINDOW_API_COCOA; *floating = false; return true;
    }
    static bool gCreate (const clap_plugin_t* p, const char* api, bool floating)
    {
        if (! gSupported (p, api, floating)) return false;
        self (p)->guiCreated = true;
        return true;
    }
    static void gDestroy (const clap_plugin_t* p)
    {
        Plugin* s = self (p);
        if (s->editor) { editorClose (s->editor); s->editor = nullptr; }
        s->guiCreated = false;
    }
    static bool gSetScale (const clap_plugin_t*, double) { return false; }   // Cocoa scales by itself
    static bool gGetSize (const clap_plugin_t*, uint32_t* w, uint32_t* h)
    {
        *w = kEditorWidth; *h = kEditorHeight; return true;
    }
    static bool gCanResize (const clap_plugin_t*) { return false; }
    static bool gHints (const clap_plugin_t*, clap_gui_resize_hints_t*) { return false; }
    static bool gAdjust (const clap_plugin_t*, uint32_t* w, uint32_t* h)
    {
        *w = kEditorWidth; *h = kEditorHeight; return true;
    }
    static bool gSetSize (const clap_plugin_t*, uint32_t w, uint32_t h)
    {
        return w == (uint32_t) kEditorWidth && h == (uint32_t) kEditorHeight;
    }
    static bool gSetParent (const clap_plugin_t* p, const clap_window_t* win)
    {
        Plugin* s = self (p);
        if (! s->guiCreated || ! win || ! win->cocoa) return false;
        if (s->editor) { editorClose (s->editor); s->editor = nullptr; }
        s->editor = editorOpen (win->cocoa, s);
        return s->editor != nullptr;
    }
    static bool gTransient (const clap_plugin_t*, const clap_window_t*) { return false; }
    static void gTitle (const clap_plugin_t*, const char*) {}
    static bool gShow (const clap_plugin_t*) { return true; }
    static bool gHide (const clap_plugin_t*) { return true; }
    static const clap_plugin_gui_t guiExt;
#endif

    //--------------------------------------------------------------------------
    const clap_host_t* host;
    const clap_host_params_t* hostParams;
    std::atomic<float> values[kNumParams];
    MsgQueue msgs;
    std::atomic<bool> overflowed { false };
    bool active;
    double sampleRate;
    enum { kScratch = 512 };
    Sdx330 unit;
    float spareR[kScratch];
    float byp = 0.0f, bypK = 0.01f;        // BYPASS crossfade (1 = dry)
    void prepareBypass()
    {
        bypK = (float) (1.0 - std::exp (-1.0 / (0.003 * sampleRate)));   // about 10 ms to settle
        byp = isOn (getParam (kBypass)) ? 1.0f : 0.0f;
    }
    float applyBypass (float* out, const float* dry, uint32_t n, float start, float target) const
    {
        float b = start;
        for (uint32_t i = 0; i < n; ++i)
        {
            b += (target - b) * bypK;
            if (std::fabs (target - b) < 1e-5f) b = target;
            if (b >= 1.0f) out[i] = dry[i];
            else           out[i] += (dry[i] - out[i]) * b;
        }
        return b;
    }
    float inBuf[2][kScratch];
    void* editor;
    bool guiCreated;
    ProgramStore programs;
    mutable float cache[kNumParams];
    mutable int cacheNumber;
};

const clap_plugin_audio_ports_t Plugin::audioPortsExt = { &Plugin::apCount, &Plugin::apGet };
const clap_plugin_params_t Plugin::paramsExt = { &Plugin::pCount, &Plugin::pInfo, &Plugin::pValue,
                                                 &Plugin::pToText, &Plugin::pFromText, &Plugin::pFlush };
const clap_plugin_state_t Plugin::stateExt = { &Plugin::sSave, &Plugin::sLoad };
const clap_plugin_latency_t Plugin::latencyExt = { &Plugin::latGet };
const clap_plugin_tail_t Plugin::tailExt = { &Plugin::tailGet };
#if defined(__APPLE__)
const clap_plugin_gui_t Plugin::guiExt = { &Plugin::gSupported, &Plugin::gPreferred, &Plugin::gCreate,
                                           &Plugin::gDestroy, &Plugin::gSetScale, &Plugin::gGetSize,
                                           &Plugin::gCanResize, &Plugin::gHints, &Plugin::gAdjust,
                                           &Plugin::gSetSize, &Plugin::gSetParent, &Plugin::gTransient,
                                           &Plugin::gTitle, &Plugin::gShow, &Plugin::gHide };
#endif

//==============================================================================
// Factory

uint32_t fCount (const clap_plugin_factory_t*) { return 1; }
const clap_plugin_descriptor_t* fDescriptor (const clap_plugin_factory_t*, uint32_t index)
{
    return index == 0 ? &kDescriptor : nullptr;
}
const clap_plugin_t* fCreate (const clap_plugin_factory_t*, const clap_host_t* host, const char* id)
{
    if (! host || ! id || std::strcmp (id, kPluginId) != 0) return nullptr;
    if (! clap_version_is_compatible (host->clap_version)) return nullptr;
    Plugin* p = new (std::nothrow) Plugin (host);
    return p ? &p->plugin : nullptr;
}
const clap_plugin_factory_t kClapFactory = { &fCount, &fDescriptor, &fCreate };

} // namespace

bool tbsdx330_entry_init (const char*) { return true; }
void tbsdx330_entry_deinit() {}
const void* tbsdx330_entry_get_factory (const char* factoryId)
{
    if (factoryId && ! std::strcmp (factoryId, CLAP_PLUGIN_FACTORY_ID)) return &kClapFactory;
    return nullptr;
}
