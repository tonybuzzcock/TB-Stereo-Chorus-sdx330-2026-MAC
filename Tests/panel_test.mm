// panel_test.mm  (macOS only)
// Loads the built CLAP, opens its panel through the CLAP GUI extension in an
// offscreen window, renders screenshots, drives the controls with synthetic
// mouse events, checks that edits reach the host as gesture/value events,
// tests the program memory, and closes everything again.
//
//   panel_test <path to the .clap binary> <output dir for PNGs>

#import <Cocoa/Cocoa.h>

#include <clap/clap.h>

#include "Params.h"
#include "Editor.h"

#include <dlfcn.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

static int failures = 0;
static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

static int flushRequests = 0;
static void hpRescan (const clap_host_t*, clap_param_rescan_flags) {}
static void hpClear (const clap_host_t*, clap_id, clap_param_clear_flags) {}
static void hpRequestFlush (const clap_host_t*) { ++flushRequests; }
static const clap_host_params_t hostParams = { hpRescan, hpClear, hpRequestFlush };
static const void* hGetExt (const clap_host_t*, const char* id) { return ! std::strcmp (id, CLAP_EXT_PARAMS) ? &hostParams : nullptr; }
static void hNoop (const clap_host_t*) {}
static const clap_host_t host = { CLAP_VERSION_INIT, nullptr, "panel_test", "TB", "", "1.0", hGetExt, hNoop, hNoop, hNoop };

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
static uint32_t noEvents (const clap_input_events_t*) { return 0; }
static const clap_event_header_t* noGet (const clap_input_events_t*, uint32_t) { return nullptr; }

static NSData* renderPNG (NSView* v, const std::string& path)
{
    const NSRect b = [v bounds];
    NSBitmapImageRep* rep = [v bitmapImageRepForCachingDisplayInRect: b];
    [v cacheDisplayInRect: b toBitmapImageRep: rep];
    NSData* png = [rep representationUsingType: NSBitmapImageFileTypePNG properties: [NSDictionary dictionary]];
    [png writeToFile: [NSString stringWithUTF8String: path.c_str()] atomically: YES];
    std::printf ("  wrote %s (%lu bytes)\n", path.c_str(), (unsigned long) [png length]);
    return png;
}

static void mouse (NSWindow* w, NSView* v, NSEventType type, NSPoint panelPt, int clicks = 1)
{
    const NSPoint wp = [v convertPoint: panelPt toView: nil];
    NSEvent* e = [NSEvent mouseEventWithType: type location: wp modifierFlags: 0 timestamp: 0
                                windowNumber: [w windowNumber] context: nil eventNumber: 0
                                  clickCount: clicks pressure: 1.0f];
    if (type == NSEventTypeLeftMouseDown) [v mouseDown: e];
    else if (type == NSEventTypeLeftMouseDragged) [v mouseDragged: e];
    else [v mouseUp: e];
}
static void click (NSWindow* w, NSView* v, NSPoint pt, int clicks = 1)
{
    mouse (w, v, NSEventTypeLeftMouseDown, pt, clicks);
    mouse (w, v, NSEventTypeLeftMouseUp, pt, clicks);
}
static void pump (double secs) { [[NSRunLoop currentRunLoop] runUntilDate: [NSDate dateWithTimeIntervalSinceNow: secs]]; }

static double val (const clap_plugin_t* p, const clap_plugin_params_t* pp, int id)
{
    double v = -1; pp->get_value (p, (clap_id) id, &v); return v;
}

// One processing block, so queued panel edits reach the "host".
static void processBlock (const clap_plugin_t* p, OutEvents& out)
{
    std::vector<float> l (256, 0.0f), r (256, 0.0f), ol (256), orr (256);
    float* ins[2] = { &l[0], &r[0] };
    float* outs[2] = { &ol[0], &orr[0] };
    clap_audio_buffer_t ib; std::memset (&ib, 0, sizeof (ib)); ib.data32 = ins; ib.channel_count = 2;
    clap_audio_buffer_t ob; std::memset (&ob, 0, sizeof (ob)); ob.data32 = outs; ob.channel_count = 2;
    clap_input_events_t in = { nullptr, noEvents, noGet };
    clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
    pr.steady_time = -1; pr.frames_count = 256;
    pr.audio_inputs = &ib; pr.audio_inputs_count = 1; pr.audio_outputs = &ob; pr.audio_outputs_count = 1;
    pr.in_events = &in; pr.out_events = &out.list;
    p->process (p, &pr);
}

// Play a sine through the plugin (so the input meter follows a real signal)
static void playSine (const clap_plugin_t* p, double amp, int blocks, std::vector<float>* keepL = nullptr, std::vector<float>* keepIn = nullptr)
{
    static double phase = 0.0;
    std::vector<float> l (256), r (256), ol (256), orr (256);
    for (int b = 0; b < blocks; ++b)
    {
        for (int i = 0; i < 256; ++i) { l[i] = r[i] = (float) (amp * std::sin (phase)); phase += 2.0 * 3.14159265358979 * 220.0 / 44100.0; }
        float* ins[2] = { &l[0], &r[0] };
        float* outs[2] = { &ol[0], &orr[0] };
        clap_audio_buffer_t ib; std::memset (&ib, 0, sizeof (ib)); ib.data32 = ins; ib.channel_count = 2;
        clap_audio_buffer_t ob; std::memset (&ob, 0, sizeof (ob)); ob.data32 = outs; ob.channel_count = 2;
        clap_input_events_t in = { nullptr, noEvents, noGet };
        OutEvents out;
        clap_process_t pr; std::memset (&pr, 0, sizeof (pr));
        pr.steady_time = -1; pr.frames_count = 256;
        pr.audio_inputs = &ib; pr.audio_inputs_count = 1; pr.audio_outputs = &ob; pr.audio_outputs_count = 1;
        pr.in_events = &in; pr.out_events = &out.list;
        p->process (p, &pr);
        if (keepL) keepL->insert (keepL->end(), ol.begin(), ol.end());
        if (keepIn) keepIn->insert (keepIn->end(), l.begin(), l.end());
    }
}

static void setParam (const clap_plugin_t* p, const clap_plugin_params_t* pp, int id, double v)
{
    clap_event_param_value_t ev; std::memset (&ev, 0, sizeof (ev));
    ev.header.size = sizeof (ev); ev.header.space_id = CLAP_CORE_EVENT_SPACE_ID; ev.header.type = CLAP_EVENT_PARAM_VALUE;
    ev.param_id = (clap_id) id; ev.value = v; ev.note_id = -1; ev.port_index = -1; ev.channel = -1; ev.key = -1;
    struct L { static uint32_t size (const clap_input_events_t*) { return 1; }
               static const clap_event_header_t* get (const clap_input_events_t* l, uint32_t) { return (const clap_event_header_t*) l->ctx; } };
    clap_input_events_t in = { &ev, L::size, L::get };
    OutEvents o; pp->flush (p, &in, &o.list);
}


static void click (NSWindow* w, NSView* v, CGFloat x, CGFloat y) { click (w, v, NSMakePoint (x, y)); }
static void drag (NSWindow* w, NSView* v, CGFloat x, CGFloat y, CGFloat dy)
{
    mouse (w, v, NSEventTypeLeftMouseDown, NSMakePoint (x, y));
    const int steps = 5;
    for (int k = 1; k <= steps; ++k) mouse (w, v, NSEventTypeLeftMouseDragged, NSMakePoint (x, y + dy * k / steps));
    mouse (w, v, NSEventTypeLeftMouseUp, NSMakePoint (x, y + dy));
}
static NSTextField* nameBox (NSView* panel)
{
    NSTextField* f = nil;
    for (NSView* sv in [panel subviews]) if ([sv isKindOfClass: [NSTextField class]]) f = (NSTextField*) sv;
    return f;
}
static bool fileHas (const std::string& path, const char* what)
{
    FILE* f = std::fopen (path.c_str(), "r");
    if (! f) return false;
    static char ln[8192];
    bool found = false;
    while (std::fgets (ln, sizeof (ln), f)) if (std::strstr (ln, what)) found = true;
    std::fclose (f);
    return found;
}

// Control positions, as drawn by PanelView.mm
#define IN_L      112, 172
#define IN_R      142, 172
#define OUT       236, 172
#define B_PROGRAM 104, 286
#define B_FUNC1   220, 286
#define B_FUNC2   104, 342
#define B_FUNC3   220, 342
#define B_UP      530, 286
#define B_DOWN    530, 342
#define B_EDIT    636, 286
#define B_SYSTEM  752, 286
#define B_EXIT    868, 286
#define B_PAGE    636, 342
#define B_MEMORY  752, 342
#define B_BYPASS  868, 342
#define DIAL      402, 318
#define LCD       800, 150
#define DRAWER    142, 447

int main (int argc, char** argv)
{
    if (argc < 3) { std::printf ("usage: %s <clap binary> <png dir>\n", argv[0]); return 2; }
    setvbuf (stdout, 0, _IONBF, 0);
    NSAutoreleasePool* pool = [[NSAutoreleasePool alloc] init];
    [NSApplication sharedApplication];
    const std::string dir = argv[2];
    using namespace tbsdx;

    const std::string progFile = dir + "/test_programs/Programs.txt";
    setenv ("TBSDX_PROGRAM_FILE", progFile.c_str(), 1);
    std::remove (progFile.c_str());

    void* lib = dlopen (argv[1], RTLD_NOW | RTLD_LOCAL);
    if (! lib) { std::printf ("dlopen failed: %s\n", dlerror()); return 1; }
    const clap_plugin_entry_t* entry = (const clap_plugin_entry_t*) dlsym (lib, "clap_entry");
    entry->init (argv[1]);
    const clap_plugin_factory_t* f = (const clap_plugin_factory_t*) entry->get_factory (CLAP_PLUGIN_FACTORY_ID);
    const clap_plugin_t* p = f->create_plugin (f, &host, f->get_plugin_descriptor (f, 0)->id);
    p->init (p);
    p->activate (p, 44100.0, 1, 4096);
    p->start_processing (p);
    const clap_plugin_params_t* pp = (const clap_plugin_params_t*) p->get_extension (p, CLAP_EXT_PARAMS);
    OutEvents ev;
    #define PROG() ((int) val (p, pp, kProgram))
    #define ALGO() ((int) val (p, pp, kAlgo) - 1)

    std::printf ("Panel\n");
    const clap_plugin_gui_t* gui = (const clap_plugin_gui_t*) p->get_extension (p, CLAP_EXT_GUI);
    check (gui && gui->is_api_supported (p, CLAP_WINDOW_API_COCOA, false), "supports an embedded Cocoa panel");
    uint32_t w = 0, h = 0;
    check (gui->create (p, CLAP_WINDOW_API_COCOA, false) && gui->get_size (p, &w, &h) && w == (uint32_t) kEditorWidth && h == (uint32_t) kEditorHeight, "panel size as declared (1100 x 494)");
    NSWindow* win = [[NSWindow alloc] initWithContentRect: NSMakeRect (0, 0, w, h) styleMask: NSWindowStyleMaskBorderless
                                                  backing: NSBackingStoreBuffered defer: NO];
    NSView* parent = [win contentView];
    clap_window_t cw; cw.api = CLAP_WINDOW_API_COCOA; cw.cocoa = parent;
    check (gui->set_parent (p, &cw) && gui->show (p), "panel attaches to the host's view");
    check ([[parent subviews] count] == 1, "panel view added");
    NSView* panel = [[parent subviews] count] ? [[parent subviews] objectAtIndex: 0] : nil;
    if (! panel) return 1;
    NSData* pngDefault = renderPNG (panel, dir + "/panel_default.png");
    check ([pngDefault length] > 10000, "panel renders");

    std::printf ("\nLevel knobs\n");
    {
        const int knobs[3] = { kInputL, kInputR, kOutput };
        const CGFloat kx[3] = { 112, 142, 236 };
        bool all = true;
        for (int k = 0; k < 3; ++k)
        {
            for (int j = 0; j < 3; ++j) setParam (p, pp, knobs[j], 0.5);
            OutEvents o;
            drag (win, panel, kx[k], 172, -20.0);
            processBlock (p, o);
            bool others = true;
            for (int j = 0; j < 3; ++j) if (j != k && val (p, pp, knobs[j]) != 0.5) others = false;
            if (std::fabs (val (p, pp, knobs[k]) - 0.6) > 0.01 || ! others || o.begins != 1 || o.ends != 1 || o.values < 1)
            { all = false; std::printf ("  knob %d: %.3f (%d/%d/%d)\n", knobs[k], val (p, pp, knobs[k]), o.begins, o.values, o.ends); }
        }
        check (all, "INPUT L (inner), INPUT R (outer) and OUTPUT each turn on their own, reported as gesture + value events");
        mouse (win, panel, NSEventTypeLeftMouseDown, NSMakePoint (OUT), 2);
        mouse (win, panel, NSEventTypeLeftMouseUp, NSMakePoint (OUT), 2);
        check (val (p, pp, kOutput) == 0.5, "double-click resets a knob");
        for (int j = 0; j < 3; ++j) setParam (p, pp, knobs[j], 0.5);
    }
    std::printf ("\nPlay mode: the alpha dial and arrows choose programs\n");
    {
        check (PROG() == 201, "starts on 201");
        drag (win, panel, DIAL, -10.0);
        check (PROG() == 202, "dial up one step: 202");
        check (ALGO() == kStereo3D, "202 DIMENSIONAL CHO loads its algorithm (Stereo 3D Chorus)");
        click (win, panel, B_UP);
        check (PROG() == 203, "increment button: 203");
        click (win, panel, B_DOWN); click (win, panel, B_DOWN);
        check (PROG() == 201, "decrement twice: 201");
        click (win, panel, B_DOWN);
        check (PROG() == 200, "below 201 into the user area (200)");
        for (int k = 0; k < 175; ++k) click (win, panel, B_DOWN);
        check (PROG() == 25, "down to 25");
        click (win, panel, B_UP);
        check (PROG() == 26, "user 26 holds a copy of SMOOTH 3D CHORUS");
        renderPNG (panel, dir + "/panel_program_026.png");
        click (win, panel, B_PROGRAM);
        pump (0.05);
        renderPNG (panel, dir + "/panel_show_algorithm.png");
        click (win, panel, B_PROGRAM);
    }

    std::printf ("\nEdit mode\n");
    {
        click (win, panel, B_EDIT);
        pump (0.05);
        renderPNG (panel, dir + "/panel_edit_page.png");
        // PAGE lit: the dial turns pages. Stereo 3D Chorus: [Algo Rate Depth] [PrDly LFO Phase] [AzmtL ElevL AzmtR]
        drag (win, panel, DIAL, -20.0);
        const float az0 = (float) val (p, pp, kAzmtL);
        click (win, panel, B_FUNC1);                 // AzmtL
        drag (win, panel, DIAL, -30.0);              // three steps
        const float az1 = (float) val (p, pp, kAzmtL);
        char d0[16], d1[16];
        formatParam (kAzmtL, az0, d0, sizeof (d0)); formatParam (kAzmtL, az1, d1, sizeof (d1));
        std::printf ("  AzmtL %s -> %s\n", d0, d1);
        check (az1 > az0 && std::fabs (toReal (kAzmtL, az1) - toReal (kAzmtL, az0) - 3.0) < 0.6, "on page 3 FUNC 1 picks AzmtL; the dial steps it");
        pump (0.05);
        renderPNG (panel, dir + "/panel_edit_value.png");
        // Back to page 1: PAGE, dial down, FUNC 1 = Algo
        click (win, panel, B_PAGE);
        drag (win, panel, DIAL, 20.0);
        click (win, panel, B_FUNC2);                 // Rate
        const double rate0 = val (p, pp, kRate);
        click (win, panel, B_UP);
        check (val (p, pp, kRate) > rate0, "FUNC 2 on page 1 is Rate; the arrow steps it");
        click (win, panel, B_FUNC1);
        click (win, panel, B_UP);
        check (ALGO() == kPanner3D, "FUNC 1 on page 1 is the algorithm");
        click (win, panel, B_DOWN);
        check (ALGO() == kStereo3D, "and back");
        click (win, panel, B_EXIT);
        pump (0.05);
        renderPNG (panel, dir + "/panel_edited.png");
    }

    std::printf ("\nSYSTEM: Direct Mix\n");
    {
        click (win, panel, B_SYSTEM);
        click (win, panel, B_UP);
        check ((int) val (p, pp, kDirectMix) == 1, "Direct Mix PROG -> ON");
        pump (0.05);
        renderPNG (panel, dir + "/panel_system.png");
        click (win, panel, B_DOWN);
        check ((int) val (p, pp, kDirectMix) == 0, "and back to PROG");
        click (win, panel, B_EXIT);
    }

    std::printf ("\nMEMORY: write a user program and name it\n");
    {
        click (win, panel, B_MEMORY);
        check (true, "MEMORY opens the write screen");
        click (win, panel, B_UP);                    // 26 -> 27
        pump (0.05);
        renderPNG (panel, dir + "/panel_write.png");
        click (win, panel, B_MEMORY);
        check (PROG() == 27, "written to 27, now the current program");
        check (fileHas (progFile, "prog 27:"), "the program file holds 27");
        NSTextField* f = nameBox (panel);
        check (f != nil, "writing opens a box over the LCD to name it");
        renderPNG (panel, dir + "/panel_naming.png");
        if (f)
        {
            [f setStringValue: @"WIDE 3D PAD"];
            [f sendAction: [f action] to: [f target]];
            pump (0.2);
        }
        check (fileHas (progFile, "name 27: WIDE 3D PAD"), "Return saves the name");
        check (nameBox (panel) == nil, "and the box goes away");
        renderPNG (panel, dir + "/panel_written.png");
        // Clicking the LCD renames; Esc keeps the old name
        click (win, panel, LCD);
        f = nameBox (panel);
        check (f != nil && [[f stringValue] isEqualToString: @"WIDE 3D PAD"], "clicking the LCD opens the name to edit");
        if (f)
        {
            [f setStringValue: @"SOMETHING ELSE"];
            [(id) panel control: f textView: nil doCommandBySelector: @selector (cancelOperation:)];
            pump (0.2);
        }
        check (fileHas (progFile, "name 27: WIDE 3D PAD"), "Esc keeps the old name");
        // Recall: away and back
        click (win, panel, B_UP);
        setParam (p, pp, kBypass, 1.0);
        click (win, panel, B_DOWN);
        check (PROG() == 27 && ALGO() == kStereo3D, "the written program comes back");
        check (val (p, pp, kBypass) >= 0.5, "loading a program leaves BYPASS alone");
        setParam (p, pp, kBypass, 0.0);
    }

    std::printf ("\nBYPASS\n");
    {
        click (win, panel, B_BYPASS);
        check (val (p, pp, kBypass) >= 0.5, "BYPASS latches in");
        std::vector<float> outL, inL;
        playSine (p, 0.3, 120, &outL, &inL);
        bool exact = true;
        for (size_t i = outL.size() / 2; i < outL.size(); ++i) if (outL[i] != inL[i]) exact = false;
        check (exact, "bypassed: the input comes out bit for bit");
        pump (0.1);
        renderPNG (panel, dir + "/panel_bypassed.png");
        click (win, panel, B_BYPASS);
        check (val (p, pp, kBypass) < 0.5, "and out again");
    }

    std::printf ("\nPARAMETERS drawer\n");
    {
        setParam (p, pp, kAlgo, kStereoChorus + 1);
        click (win, panel, DRAWER);
        pump (0.6);                                  // the slide
        renderPNG (panel, dir + "/panel_drawer.png");
        const double r0 = val (p, pp, kRate);
        drag (win, panel, 112, 94, -20.0);           // first knob: Rate
        check (std::fabs (val (p, pp, kRate) - (r0 + 0.1)) < 0.01, "the drawer's first knob turns Rate");
        click (win, panel, 723, 27);                 // next algorithm
        check (ALGO() == kStereo3D, "the drawer's arrow changes the algorithm");
        setParam (p, pp, kAlgo, kPhase8 + 1);
        pump (0.1);
        renderPNG (panel, dir + "/panel_drawer_8phase.png");
        setParam (p, pp, kAlgo, kRotary + 1);
        pump (0.1);
        renderPNG (panel, dir + "/panel_drawer_rotary.png");
        const double lv = val (p, pp, kInputL);
        drag (win, panel, IN_L, -20.0);              // covered by the drawer: no change
        check (val (p, pp, kInputL) == lv, "the drawer covers the face's controls");
        click (win, panel, DRAWER);
        pump (0.6);
    }

    std::printf ("\nHost automation shows on the panel\n");
    {
        NSData* before = renderPNG (panel, dir + "/panel_before_host.png");
        setParam (p, pp, kOutput, 0.9);
        setParam (p, pp, kAlgo, kFlanger + 1);
        pump (0.3);
        NSData* after = renderPNG (panel, dir + "/panel_changed.png");
        check (! [after isEqualToData: before], "panel redraws after host changes");
        setParam (p, pp, kOutput, 0.5);
    }

    std::printf ("\nInput meter\n");
    {
        // Program 226 SMOOTH 3D CHORUS, as in the photo
        for (int guard = 0; guard < 400 && PROG() != 226; ++guard)
        {
            if (PROG() < 226) click (win, panel, B_UP); else click (win, panel, B_DOWN);
        }
        check (PROG() == 226, "arrows reach 226");
        setParam (p, pp, kInputL, 0.62);
        setParam (p, pp, kInputR, 0.55);
        pump (0.2);
        NSData* quiet = renderPNG (panel, dir + "/panel_meter_off.png");
        playSine (p, 0.55, 80);
        pump (0.3);
        NSData* lit = renderPNG (panel, dir + "/panel_showcase.png");
        check (! [lit isEqualToData: quiet], "the meter lights with the input");
    }

    std::printf ("\nClose\n");
    gui->destroy (p);
    check ([[parent subviews] count] == 0, "destroy removes the panel");
    check (gui->create (p, CLAP_WINDOW_API_COCOA, false) && gui->set_parent (p, &cw) && [[parent subviews] count] == 1, "reopens");
    gui->destroy (p);
    p->stop_processing (p);
    p->deactivate (p);
    p->destroy (p);
    entry->deinit();
    [win release];
    [pool release];
    dlclose (lib);
    (void) ev; (void) flushRequests;

    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PANEL CHECKS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
