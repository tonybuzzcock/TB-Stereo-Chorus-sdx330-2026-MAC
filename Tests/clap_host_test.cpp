// clap_host_test.cpp
// A minimal CLAP host: loads the built plugin binary, then checks the
// descriptor, parameters, text conversion, processing, automation events,
// state save/restore, latency and tail.
//
//   clap_host_test <path to the .clap binary>
// On macOS pass the binary inside the bundle:
//   "TB SDX-330.clap/Contents/MacOS/TB SDX-330"

#include <clap/clap.h>

#include "Params.h"
#include "Sdx330.h"
#include "Factory.h"

#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <string>
#include <vector>

static int failures = 0;
static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

// ---- Host ----
static int flushRequests = 0, rescans = 0;
static void hpRescan (const clap_host_t*, clap_param_rescan_flags) { ++rescans; }
static void hpClear (const clap_host_t*, clap_id, clap_param_clear_flags) {}
static void hpRequestFlush (const clap_host_t*) { ++flushRequests; }
static const clap_host_params_t hostParams = { hpRescan, hpClear, hpRequestFlush };

static const void* hGetExt (const clap_host_t*, const char* id)
{
    return ! std::strcmp (id, CLAP_EXT_PARAMS) ? &hostParams : nullptr;
}
static void hNoop (const clap_host_t*) {}
static const clap_host_t host = { CLAP_VERSION_INIT, nullptr, "clap_host_test", "TB", "", "1.0",
                                  hGetExt, hNoop, hNoop, hNoop };

// ---- Event lists ----
struct InEvents
{
    std::vector<clap_event_param_value_t> ev;
    clap_input_events_t list;
    InEvents() { list.ctx = this; list.size = &sSize; list.get = &sGet; }
    static uint32_t sSize (const clap_input_events_t* l) { return (uint32_t) ((InEvents*) l->ctx)->ev.size(); }
    static const clap_event_header_t* sGet (const clap_input_events_t* l, uint32_t i) { return &((InEvents*) l->ctx)->ev[i].header; }
    void add (clap_id id, double v)
    {
        clap_event_param_value_t e;
        std::memset (&e, 0, sizeof (e));
        e.header.size = sizeof (e); e.header.space_id = CLAP_CORE_EVENT_SPACE_ID; e.header.type = CLAP_EVENT_PARAM_VALUE;
        e.param_id = id; e.note_id = -1; e.port_index = -1; e.channel = -1; e.key = -1; e.value = v;
        ev.push_back (e);
    }
};

struct OutEvents
{
    int values = 0, begins = 0, ends = 0;
    clap_output_events_t list;
    OutEvents() { list.ctx = this; list.try_push = &sPush; }
    static bool sPush (const clap_output_events_t* l, const clap_event_header_t* h)
    {
        OutEvents* o = (OutEvents*) l->ctx;
        if (h->type == CLAP_EVENT_PARAM_VALUE) ++o->values;
        if (h->type == CLAP_EVENT_PARAM_GESTURE_BEGIN) ++o->begins;
        if (h->type == CLAP_EVENT_PARAM_GESTURE_END) ++o->ends;
        return true;
    }
};

// ---- Streams ----
struct OStream { std::string data; clap_ostream_t s; OStream() { s.ctx = this; s.write = &w; }
    static int64_t w (const clap_ostream_t* st, const void* b, uint64_t n) { ((OStream*) st->ctx)->data.append ((const char*) b, (size_t) n); return (int64_t) n; } };
struct IStream { std::string data; size_t pos = 0; clap_istream_t s; IStream() { s.ctx = this; s.read = &r; }
    static int64_t r (const clap_istream_t* st, void* b, uint64_t n)
    {
        IStream* is = (IStream*) st->ctx;
        // Hand the data over a few bytes at a time to prove partial reads work
        size_t k = std::min ((size_t) n, std::min ((size_t) 37, is->data.size() - is->pos));
        std::memcpy (b, is->data.data() + is->pos, k); is->pos += k; return (int64_t) k;
    } };

static bool finite (const std::vector<float>& x, float lim)
{
    for (size_t i = 0; i < x.size(); ++i) if (! (x[i] == x[i]) || std::fabs (x[i]) > lim) return false;
    return true;
}

// Run audio through the plugin in blocks.
static void run (const clap_plugin_t* p, std::vector<float>& l, std::vector<float>& r,
                 std::vector<float>& ol, std::vector<float>& orr, uint32_t block,
                 InEvents* in = nullptr, OutEvents* out = nullptr)
{
    InEvents emptyIn; OutEvents sink;
    for (size_t pos = 0; pos < l.size(); pos += block)
    {
        const uint32_t n = (uint32_t) std::min ((size_t) block, l.size() - pos);
        float* ins[2] = { &l[pos], &r[pos] };
        float* outs[2] = { &ol[pos], &orr[pos] };
        clap_audio_buffer_t ib; std::memset (&ib, 0, sizeof (ib)); ib.data32 = ins; ib.channel_count = 2;
        clap_audio_buffer_t ob; std::memset (&ob, 0, sizeof (ob)); ob.data32 = outs; ob.channel_count = 2;
        clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
        pr.steady_time = -1; pr.frames_count = n;
        pr.audio_inputs = &ib; pr.audio_inputs_count = 1;
        pr.audio_outputs = &ob; pr.audio_outputs_count = 1;
        pr.in_events = (in && pos == 0) ? &in->list : &emptyIn.list;
        pr.out_events = out ? &out->list : &sink.list;
        p->process (p, &pr);
    }
}

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("usage: %s <plugin binary>\n", argv[0]); return 2; }
    setenv ("TBSDX_PROGRAM_FILE", "/tmp/tbsdx_clap_test_programs.txt", 1);
    std::printf ("Loading %s\n", argv[1]);
    void* lib = dlopen (argv[1], RTLD_NOW | RTLD_LOCAL);
    if (! lib) { std::printf ("dlopen failed: %s\n", dlerror()); return 1; }
    const clap_plugin_entry_t* entry = (const clap_plugin_entry_t*) dlsym (lib, "clap_entry");
    check (entry != nullptr, "exports clap_entry");
    if (! entry) return 1;
    check (entry->init (argv[1]), "entry init");
    const clap_plugin_factory_t* f = (const clap_plugin_factory_t*) entry->get_factory (CLAP_PLUGIN_FACTORY_ID);
    check (f && f->get_plugin_count (f) == 1, "one plugin in the factory");
    const clap_plugin_descriptor_t* d = f->get_plugin_descriptor (f, 0);
    std::printf ("  %s by %s, id %s, version %s\n", d->name, d->vendor, d->id, d->version);
    check (! std::strcmp (d->name, "TB SDX-330"), "plugin name");

    const clap_plugin_t* p = f->create_plugin (f, &host, d->id);
    check (p && p->init (p), "create and init");
    if (! p) return 1;

    using namespace tbsdx;
    std::printf ("\nParameters\n");
    const clap_plugin_params_t* pp = (const clap_plugin_params_t*) p->get_extension (p, CLAP_EXT_PARAMS);
    check (pp && pp->count (p) == kNumParams, "every parameter of the 16 algorithms, the EQ and the panel");
    bool namesOk = true, defaultsOk = true, roundOk = true;
    for (uint32_t i = 0; i < pp->count (p); ++i)
    {
        clap_param_info_t info;
        pp->get_info (p, i, &info);
        double v = -1; pp->get_value (p, info.id, &v);
        char txt[64] = { 0 }; pp->value_to_text (p, info.id, v, txt, sizeof (txt));
        if (i < 12 || i == kRate || i == kLfo || i == kAzmtL)
            std::printf ("  %-8s %-8s = %-22s (%.3f, %g..%g%s)\n", info.name, info.module, txt, v, info.min_value, info.max_value,
                         (info.flags & CLAP_PARAM_IS_STEPPED) ? ", stepped" : "");
        if (std::strcmp (info.name, kInfo[i].vst) != 0) namesOk = false;
        if (std::fabs (v - info.default_value) > 1e-6) defaultsOk = false;
        // Every value's text reads back to the same text
        for (int k = 0; k <= 20; ++k)
        {
            const double pv = info.min_value + (info.max_value - info.min_value) * k / 20.0;
            char t1[64] = { 0 }, t2[64] = { 0 };
            double back = -1;
            pp->value_to_text (p, info.id, pv, t1, sizeof (t1));
            if (! pp->text_to_value (p, info.id, t1, &back)) { roundOk = false; std::printf ("  no parse: %s '%s'\n", info.name, t1); break; }
            pp->value_to_text (p, info.id, back, t2, sizeof (t2));
            if (std::strcmp (t1, t2)) { roundOk = false; std::printf ("  round trip %s: '%s' -> '%s'\n", info.name, t1, t2); break; }
        }
    }
    check (namesOk, "parameters in the 10.8.5 build's order, with its names");
    check (defaultsOk, "values start at program 201 STEREO CHORUS (the unit's power-on program)");
    check (roundOk, "every parameter's text converts back to the same value");
    char txt[64];
    double tv = 0;
    pp->value_to_text (p, kRate, toNorm (kRate, 0.65), txt, sizeof (txt));
    check (! std::strcmp (txt, "0.65 Hz"), "Rate reads 0.65 Hz");
    check (pp->text_to_value (p, kRate, "2.5 Hz", &tv) && std::fabs (toReal (kRate, tv) - 2.5) < 0.01, "text '2.5 Hz' -> Rate 2.5 Hz");
    check (pp->text_to_value (p, kLfo, "sin", &tv) && tv == 1.0, "text 'sin' -> LFO SIN");
    check (pp->text_to_value (p, kAzmtL, "L90", &tv) && std::fabs (toReal (kAzmtL, tv) + 90.0) < 0.5, "text 'L90' -> azimuth left 90");
    check (pp->text_to_value (p, kPanL, "30:70", &tv) && std::fabs (toReal (kPanL, tv) - 70.0) < 0.5, "text '30:70' -> pan");
    check (pp->text_to_value (p, kAlgo, "4", &tv) && tv == 4.0, "text '4' -> algorithm 4");
    pp->value_to_text (p, kAlgo, 4.0, txt, sizeof (txt));
    check (! std::strcmp (txt, "4 Space Chorus"), "Algo reads its number and name");
    pp->value_to_text (p, kProgram, 226.0, txt, sizeof (txt));
    check (! std::strcmp (txt, "226"), "Program reads its number");
    {
        clap_param_info_t ai; pp->get_info (p, kAlgo, &ai);
        check (ai.min_value == 1.0 && ai.max_value == 16.0 && (ai.flags & CLAP_PARAM_IS_STEPPED), "Algo is stepped 1..16");
    }

    std::printf ("\nAudio\n");
    const clap_plugin_audio_ports_t* ap = (const clap_plugin_audio_ports_t*) p->get_extension (p, CLAP_EXT_AUDIO_PORTS);
    clap_audio_port_info_t pi;
    check (ap && ap->count (p, true) == 1 && ap->count (p, false) == 1 && ap->get (p, 0, false, &pi) && pi.channel_count == 2,
           "one stereo input and output");
    const clap_plugin_latency_t* lat = (const clap_plugin_latency_t*) p->get_extension (p, CLAP_EXT_LATENCY);
    const clap_plugin_tail_t* tail = (const clap_plugin_tail_t*) p->get_extension (p, CLAP_EXT_TAIL);
    check (p->activate (p, 44100.0, 1, 4096), "activate at 44.1 kHz");
    p->start_processing (p);
    check (lat && lat->get (p) == 0, "no latency");
    check (tail && tail->get (p) > 44100, "reports a tail (ambience)");

    const int sr = 44100;
    std::vector<float> l (sr), r (sr), ol (sr), orr (sr);
    for (int i = 0; i < sr; ++i) { l[i] = (float) (0.2 * std::sin (2.0 * 3.14159265358979 * 196.0 * i / sr)); r[i] = (float) (0.2 * std::sin (2.0 * 3.14159265358979 * 247.0 * i / sr)); }

    // Reference: the DSP core directly with program 201's settings
    {
        Sdx330* ref = new Sdx330();
        float v[kNumParams];
        for (int i = 0; i < kNumParams; ++i) v[i] = defaultNorm (i);
        factoryValues (0, v);
        v[kProgram] = programValue (201);
        for (int i = 0; i < kNumParams; ++i) ref->setParam (i, v[i]);
        ref->prepare (sr);
        std::vector<float> rl (sr), rr (sr);
        for (int pos = 0; pos < sr; pos += 512) ref->process (&l[pos], &r[pos], &rl[pos], &rr[pos], std::min (512, sr - pos));
        delete ref;

        const std::clock_t t0 = std::clock();
        run (p, l, r, ol, orr, 512);
        const double secs = (double) (std::clock() - t0) / CLOCKS_PER_SEC;
        float pk = 0; double diff = 0;
        for (int i = 0; i < sr; ++i)
        {
            pk = std::max (pk, std::fabs (ol[i]));
            diff = std::max (diff, (double) std::max (std::fabs (ol[i] - rl[i]), std::fabs (orr[i] - rr[i])));
        }
        std::printf ("  peak %.3f, max diff vs DSP core %.2g, CPU %.1f%% of one core\n", pk, diff, 100.0 * secs);
        check (pk > 0.02 && finite (ol, 10.0f), "produces output");
        check (diff < 1e-6, "matches the DSP core sample for sample (same as the 10.8.5 build)");
    }

    // Mono in, one-channel buses
    {
        std::vector<float> a (sr), oa (sr), ob (sr);
        for (int i = 0; i < sr; ++i) a[i] = (float) (0.2 * std::sin (2.0 * 3.14159265358979 * 330.0 * i / sr));
        float* ins[1] = { &a[0] };
        float* outs[2] = { &oa[0], &ob[0] };
        clap_audio_buffer_t ib; std::memset (&ib, 0, sizeof (ib)); ib.data32 = ins; ib.channel_count = 1;
        clap_audio_buffer_t ob2; std::memset (&ob2, 0, sizeof (ob2)); ob2.data32 = outs; ob2.channel_count = 2;
        InEvents ei; OutEvents eo;
        clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
        pr.steady_time = -1; pr.frames_count = (uint32_t) sr; pr.audio_inputs = &ib; pr.audio_inputs_count = 1;
        pr.audio_outputs = &ob2; pr.audio_outputs_count = 1; pr.in_events = &ei.list; pr.out_events = &eo.list;
        p->process (p, &pr);
        check (finite (oa, 10.0f) && finite (ob, 10.0f), "a one-channel input feeds both sides");
        ob2.channel_count = 1;
        p->process (p, &pr);
        check (finite (oa, 10.0f), "a one-channel output works");
    }

    run (p, l, r, l, r, 64);
    check (finite (l, 10.0f) && finite (r, 10.0f), "in-place buffers");
    for (int i = 0; i < sr; ++i) { l[i] = (float) (0.2 * std::sin (2.0 * 3.14159265358979 * 196.0 * i / sr)); r[i] = 0.5f * l[i]; }

    std::printf ("\nBypass (this plugin only)\n");
    {
        clap_param_info_t bi; pp->get_info (p, kBypass, &bi);
        check ((bi.flags & CLAP_PARAM_IS_STEPPED) && ! (bi.flags & CLAP_PARAM_IS_BYPASS), "Bypass is the plugin's own switch, not the host's bypass");
        char bt[32]; double bv = -1;
        pp->value_to_text (p, kBypass, 1.0, bt, sizeof (bt));
        check (! std::strcmp (bt, "Bypass") && pp->text_to_value (p, kBypass, "In", &bv) && bv == 0.0, "Bypass reads Bypass / In");
        InEvents on; on.add (kBypass, 1.0);
        std::vector<float> a (l), b (r), oa (sr), ob (sr);
        run (p, a, b, oa, ob, 512, &on);
        bool exact = true;
        for (int i = sr / 4; i < sr; ++i) if (oa[i] != a[i] || ob[i] != b[i]) exact = false;
        check (exact, "Bypass passes the input bit for bit");
        std::vector<float> ia (l), ib2 (r);
        run (p, ia, ib2, ia, ib2, 37);
        bool exact2 = true;
        for (int i = sr / 4; i < sr; ++i) if (ia[i] != l[i] || ib2[i] != r[i]) exact2 = false;
        check (exact2, "and in place with odd block sizes");
        InEvents off; off.add (kBypass, 0.0);
        run (p, a, b, oa, ob, 512, &off);
        double d = 0;
        for (int i = sr / 4; i < sr; ++i) d += std::fabs (oa[i] - a[i]);
        check (d > 1.0, "effect back in after Bypass");
    }

    std::printf ("\nEvery algorithm, and automation\n");
    {
        bool all = true;
        for (int a = 0; a < kNumAlgos; ++a)
        {
            InEvents in; in.add (kAlgo, a + 1.0);
            std::vector<float> c (l), cr (r), oc (sr), ocr (sr);
            run (p, c, cr, oc, ocr, 512, &in);
            double s = 0; for (int i = sr / 2; i < sr; ++i) s += std::fabs (oc[i]) + std::fabs (ocr[i]);
            if (! finite (oc, 10.0f) || ! finite (ocr, 10.0f) || s < 1.0) { all = false; std::printf ("  algorithm %d silent or unstable\n", a + 1); }
        }
        check (all, "all 16 algorithms run");
        double v;
        pp->get_value (p, kAlgo, &v); check (v == 16.0, "host automation sets the algorithm");
        std::vector<float> c (sr), cr (sr);
        for (int i = 0; i < sr; ++i) c[i] = cr[i] = (float) (0.3 * std::sin (2.0 * 3.14159265358979 * 110.0 * i / sr));
        bool ok = true;
        for (size_t pos = 0; pos < c.size(); pos += 64)
        {
            InEvents e;
            const double t = (double) pos / c.size();
            for (int k = 0; k < kNumParams; ++k)
            {
                if (k == kBypass) continue;
                clap_param_info_t info; pp->get_info (p, (uint32_t) k, &info);
                e.add ((clap_id) k, info.min_value + (info.max_value - info.min_value) * (0.5 + 0.5 * std::sin (6.2831853 * (k + 1) * t)));
            }
            std::vector<float> a1 (c.begin() + pos, c.begin() + std::min (c.size(), pos + 64));
            std::vector<float> a2 (a1), o1 (a1.size()), o2 (a1.size());
            run (p, a1, a2, o1, o2, 64, &e);
            if (! finite (o1, 100.0f) || ! finite (o2, 100.0f)) ok = false;
        }
        check (ok, "all parameters automated together");
    }

    std::printf ("\nState\n");
    {
        const clap_plugin_state_t* st = (const clap_plugin_state_t*) p->get_extension (p, CLAP_EXT_STATE);
        InEvents set; set.add (kAlgo, 4.0); set.add (kMode, 4.0); set.add (kRate, 0.3); set.add (kProgram, 27.0);
        std::vector<float> a0 (512, 0.0f), b0 (512, 0.0f), oa0 (512), ob0 (512);
        run (p, a0, b0, oa0, ob0, 512, &set);
        OStream os;
        check (st && st->save (p, &os.s) && os.data.size() > 200, "saves state");
        InEvents in; in.add (kAlgo, 9.0); in.add (kMode, 1.0); in.add (kRate, 0.9); in.add (kProgram, 250.0);
        std::vector<float> a (512, 0.0f), b (512, 0.0f), oa (512), ob (512);
        run (p, a, b, oa, ob, 512, &in);
        IStream is; is.data = os.data;
        rescans = 0;
        check (st->load (p, &is.s), "loads state (in small pieces)");
        double g, t, su, pg;
        pp->get_value (p, kAlgo, &g); pp->get_value (p, kMode, &t); pp->get_value (p, kRate, &su); pp->get_value (p, kProgram, &pg);
        check (g == 4.0 && t == 4.0 && std::fabs (su - 0.3) < 1e-6 && pg == 27.0, "state restores every value (algorithm, mode, rate, program number)");
        check (rescans > 0, "host is told to re-read the values");
        IStream junk; junk.data = "not a state";
        check (! st->load (p, &junk.s), "rejects foreign data");
    }

    std::printf ("\nFlush while stopped\n");
    {
        p->stop_processing (p);
        InEvents in; in.add (kOutput, 0.25);
        OutEvents out;
        pp->flush (p, &in.list, &out.list);
        double v; pp->get_value (p, kOutput, &v);
        check (std::fabs (v - 0.25) < 1e-6, "flush applies host changes without audio running");
    }

    std::printf ("\nSample-rate change to 96 kHz\n");
    p->deactivate (p);
    check (p->activate (p, 96000.0, 1, 4096), "re-activate at 96 kHz");
    p->start_processing (p);
    {
        std::vector<float> a (96000), b (96000), oa (96000), ob (96000);
        for (int i = 0; i < 96000; ++i) { a[i] = (float) (0.2 * std::sin (2.0 * 3.14159265358979 * 196.0 * i / 96000)); b[i] = 0.5f * a[i]; }
        run (p, a, b, oa, ob, 1024);
        check (finite (oa, 10.0f) && finite (ob, 10.0f), "runs at 96 kHz");
    }
    {
        InEvents in; in.add (kAlgo, 16.0); in.add (kAmDcy, 1.0); in.add (kOutput, 0.5);
        std::vector<float> z (96000 * 6, 0.0f), z2 (96000 * 6, 0.0f), oa (96000 * 6), ob (96000 * 6);
        run (p, z, z2, oa, ob, 1024, &in);
        float t = 0; for (size_t i = 96000 * 5; i < oa.size(); ++i) t = std::max (t, std::fabs (oa[i]));
        std::printf ("  residual after 5 s of silence (longest ambience): %.2g\n", t);
        check (t < 1e-4f, "silence settles");
    }

    p->stop_processing (p);
    p->deactivate (p);
    p->destroy (p);
    entry->deinit();
    check (true, "destroy and deinit");
    dlclose (lib);

    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL CLAP HOST CHECKS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
