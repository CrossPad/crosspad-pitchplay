/**
 * @file PitchPlayApp.cpp
 * @brief Pitch Play -- one sample or many, played chromatically on the pads and
 *        from a MIDI keyboard.
 *
 *   top     back, the instrument -- a drop-down of the saved instruments, the
 *           Recorder's takes and "the card ..." for any WAV --, EDIT, SAVE
 *   pads    the sixteen pads as they sound, bottom left first: each its note,
 *           underlined in the colour of the zone that plays it, lit while it
 *           sounds; a touch plays it too
 *   bottom  OCT- / the pads' range / OCT+, VOL (turned)
 *
 * A sample picked from the list is an instrument of one zone, its pitch
 * detected. EDIT lists the zones -- each a sample and the note it sounds
 * at -- adds one from the card, removes one, sets a root by hand or back to
 * AUTO; leaving EDIT rebuilds the bank. SAVE names the instrument and keeps it
 * in Pitch Play's library, its samples copied in.
 *
 * The engine is borrowed through crosspad::IInstrumentHost -- the first
 * instrument played takes it from the kits, leaving the app gives it back --
 * and everything that reads or writes the card runs on the host's worker or
 * through crosspad::ISampleTools; this file only asks and draws.
 */

#include "crosspad-pitchplay/PitchPlayApp.hpp"

#if __has_include("pc_stubs/PcApp.hpp")
  #include "pc_stubs/PcApp.hpp"
#else
  #include "crosspad-gui/app/App.hpp"
#endif

#include "crosspad/app/AppRegistrar.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/instrument/PitchedInstrument.hpp"
#include "crosspad/led/RgbColor.hpp"
#include "crosspad/pad/IPadLogicHandler.hpp"
#include "crosspad/pad/PadLedController.hpp"
#include "crosspad/pad/PadManager.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad-gui/widgets/gui_helpers.h"

#include "lvgl.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#ifdef ESP_PLATFORM
extern "C" {
  extern lv_indev_t* indev_encoderTop;
}
#define ROM_USE_ENCODER 1
#endif

namespace {

constexpr uint32_t kTickMs   = 50;
constexpr uint8_t  kPads     = 16;
constexpr int32_t  kRowH     = 26;
constexpr uint32_t kPageRows = 40;
constexpr uint8_t  kVolStep  = 5;

const lv_color_t kBg     = lv_color_hex(0x16181B);
const lv_color_t kPanel  = lv_color_hex(0x20242A);
const lv_color_t kText   = lv_color_hex(0xE8EAED);
const lv_color_t kDim    = lv_color_hex(0x7B828B);
const lv_color_t kInk    = lv_color_hex(0x16181B);
const lv_color_t kAccent = lv_color_hex(0xC6FF47);
const lv_color_t kHead   = lv_color_hex(0xFFFFFF);

/// A colour per zone, round again past eight: the pads say which zone plays them.
const uint32_t kZoneRgb[] = {0xC6FF47, 0x4FE0B0, 0x8B5CF6, 0xFFC857,
                             0xFF5C5C, 0x5CA8FF, 0xFF7AD9, 0xFF9F43};
constexpr size_t kZoneColors = sizeof(kZoneRgb) / sizeof(kZoneRgb[0]);

enum class Wait : uint8_t { None, Load, Save };
enum class Listing : uint8_t { None, Library, Takes };
enum class Overlay : uint8_t { None, Edit, Browse, Name };
enum class EntryKind : uint8_t { Instrument, Sample, Card };

struct Entry {
    EntryKind   kind;
    std::string path;   ///< logical: an instrument file or a WAV
};

lv_obj_t* s_root = nullptr;
lv_obj_t* s_dropdown = nullptr;
lv_obj_t* s_info = nullptr;
lv_obj_t* s_range = nullptr;
lv_obj_t* s_volBtn = nullptr;
lv_obj_t* s_cells[kPads] = {};
lv_obj_t* s_overlay = nullptr;
lv_obj_t* s_zoneList = nullptr;
lv_obj_t* s_rootLabel = nullptr;
lv_obj_t* s_nameArea = nullptr;
lv_timer_t* s_timer = nullptr;
lv_group_t* s_encGroup = nullptr;
lv_group_t* s_prevEncGroup = nullptr;
lv_group_t* s_overlayGroup = nullptr;
crosspad::SubscriptionHandle s_noteOnSub = nullptr;
crosspad::SubscriptionHandle s_noteOffSub = nullptr;

/* Kept while the board runs: the app opens on what it last played. */
crosspad::KitInfo s_spec;
bool    s_haveSpec = false;
uint8_t s_base = 48;           // the note of pad 1, C3
uint8_t s_volume = 80;         // every pad's level, 0..100: zones peak at -1 dBFS, a chord needs room
bool    s_followRoot = false;  // a sample just picked: its root goes to the middle of the pads

bool    s_dirty = false;       // zones edited since the bank was built
Wait    s_wait = Wait::None;
Listing s_listing = Listing::None;
std::vector<std::string> s_library;   // instrument names
std::vector<std::string> s_takes;     // logical paths
std::vector<Entry> s_entries;         // what the drop-down offers, in its order
char    s_ddText[48] = "Pick a sample";
bool    s_fillPending = false;        // a listing came in while the list was open

Overlay s_ov = Overlay::None;
bool    s_browseForZone = false;       // the card's WAV becomes a zone, not an instrument
std::string          s_browseDir;      // "" = the recordings
crosspad::CardFolder s_folder;
bool     s_folderPending = false;
uint32_t s_page = 0;
int      s_zoneSel = 0;

std::atomic<bool>     s_playable{false};
std::atomic<uint32_t> s_lit{0};
std::atomic<uint8_t>  s_padNote[kPads];
uint32_t s_drawnLit = 0xFFFFFFFFu;

crosspad::IInstrumentHost* host() { return crosspad::getPlatformServices().instrumentHost; }
crosspad::PitchedInstrument* inst() { return crosspad::getPlatformServices().pitchedInstrument; }
crosspad::ISampleTools* tools() { return crosspad::getPlatformServices().sampleTools; }

std::string base_of(const std::string& p)
{
    const size_t s = p.find_last_of('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

/// A listing's full path as the instrument's logical one: the card's top off.
std::string logical(const std::string& full)
{
    auto* st = tools();
    const std::string root = st ? st->cardRoot() : std::string();
    if (!root.empty() && full.compare(0, root.size(), root) == 0) return full.substr(root.size());
    return full;
}

void note_name(int note, char* buf, size_t n)
{
    static const char* kNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    if (note < 0 || note > 127) { std::snprintf(buf, n, "--"); return; }
    std::snprintf(buf, n, "%s%d", kNames[note % 12], note / 12 - 1);
}

/// The zone the engine picks for @p note: the nearest root, the lower on a tie.
int zone_for(int note)
{
    int best = -1;
    float bestDist = 1e9f;
    const auto& zones = s_spec.instrument.zones;
    for (size_t i = 0; i < zones.size(); ++i) {
        const float d = std::fabs((float)note - zones[i].root);
        if (d < bestDist || (d == bestDist && best >= 0 && zones[i].root < zones[best].root)) {
            best = (int)i;
            bestDist = d;
        }
    }
    return best;
}

uint8_t pad_note(uint8_t pad) { return (uint8_t)std::min(127, s_base + pad); }

// ── playing ────────────────────────────────────────────────────────────────

/* A pad lights while its note sounds, from the pads, a touch or a keyboard:
 * with a pad logic active, PadManager leaves the LEDs to it. */
void play(uint8_t note, uint8_t vel, uint8_t pad)
{
    auto* p = inst();
    if (!p || !s_playable.load()) return;
    p->noteOn(note, vel ? vel : 1, pad);
    if (note >= s_base && note < s_base + kPads) {
        s_lit.fetch_or(1u << (note - s_base));
        crosspad::getPadManager().notifyNoteOn(note - s_base, vel ? vel : 1, true);
    }
}

void stop(uint8_t note)
{
    auto* p = inst();
    if (!p) return;
    p->noteOff(note);
    if (note >= s_base && note < s_base + kPads) {
        s_lit.fetch_and(~(1u << (note - s_base)));
        crosspad::getPadManager().notifyNoteOff(note - s_base, true);
    }
}

/// The pads play the instrument and nothing else; the kits are not loaded.
class PitchPlayPadLogic : public crosspad::IPadLogicHandler {
public:
    void onActivate(crosspad::PadManager&) override {}
    void onDeactivate(crosspad::PadManager&) override {}
    void onPadPress(crosspad::PadManager&, uint8_t pad, uint8_t vel) override
    {
        if (pad >= kPads) return;
        const uint8_t note = pad_note(pad);
        s_padNote[pad].store(note);      // released as pressed, whatever OCT does meanwhile
        play(note, vel, pad);
    }
    void onPadRelease(crosspad::PadManager& pm, uint8_t pad) override
    {
        if (pad >= kPads) return;
        stop(s_padNote[pad].load());
        pm.notifyNoteOff(pad, true);   // its note may have left the range with OCT
    }
    void onPadPressure(crosspad::PadManager&, uint8_t, uint8_t) override {}
};
std::shared_ptr<PitchPlayPadLogic> s_padLogic;

/// A keyboard on any MIDI input; the pads' own notes come back as
/// EventSource::PadManager and are played above already.
void midi_note_cb(const void* data, crosspad::EventType type, void*)
{
    if (type == crosspad::EventType::NoteOn) {
        auto* d = static_cast<const crosspad::NoteOnData*>(data);
        if (d->source != crosspad::EventSource::MidiInput) return;
        if (d->velocity) play(d->note, d->velocity, 0);
        else             stop(d->note);
    } else {
        auto* d = static_cast<const crosspad::NoteOffData*>(data);
        if (d->source == crosspad::EventSource::MidiInput) stop(d->note);
    }
}

// ── building blocks ────────────────────────────────────────────────────────

/// Focus as a frame inside the control: the theme's ring stands 4 px out of
/// it, and the rows these controls fill cut it to brackets.
void focus_frame(lv_obj_t* b)
{
    lv_obj_set_style_outline_width(b, 0, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 0, LV_STATE_EDITED);
    lv_obj_set_style_border_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_border_width(b, 2, LV_STATE_FOCUS_KEY);
}

void style_button(lv_obj_t* b, int32_t w, int32_t h)
{
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_set_style_bg_color(b, kPanel, 0);
    lv_obj_set_style_bg_color(b, kAccent, LV_STATE_EDITED);
    lv_obj_set_style_text_color(b, kText, 0);
    lv_obj_set_style_text_color(b, kInk, LV_STATE_EDITED);
    lv_obj_set_style_text_font(b, &lv_font_montserrat_14, 0);
    focus_frame(b);
}

lv_obj_t* add_label(lv_obj_t* b, const char* text)
{
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return l;
}

lv_obj_t* make_button(lv_obj_t* parent, int32_t w, const char* text, lv_event_cb_t cb,
                      lv_group_t* g, void* ud = nullptr)
{
    lv_obj_t* b = lv_button_create(parent);
    style_button(b, w, kRowH);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    add_label(b, text);
    if (g) lv_group_add_obj(g, b);
    return b;
}

lv_obj_t* make_row(lv_obj_t* parent, int32_t h)
{
    lv_obj_t* r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, LV_PCT(100), h);
    lv_obj_set_flex_flow(r, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(r, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(r, 4, 0);
    lv_obj_remove_flag(r, LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

/// A click toggles turning; a turn while turning says which way.
int number_step(lv_event_t* e)
{
    lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
    lv_group_t* g = b ? lv_obj_get_group(b) : nullptr;
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (g) {
            const bool editing = lv_group_get_editing(g) && lv_group_get_focused(g) == b;
            lv_group_focus_obj(b);
            lv_group_set_editing(g, !editing);
        }
        return 0;
    }
    const uint32_t key = lv_event_get_key(e);
    return key == LV_KEY_RIGHT || key == LV_KEY_UP ? 1 : (key == LV_KEY_LEFT || key == LV_KEY_DOWN ? -1 : 0);
}

// ── showing ────────────────────────────────────────────────────────────────

void show_pads()
{
    char name[8];
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        lv_obj_t* c = s_cells[pad];
        if (!c) continue;
        const int note = pad_note(pad);
        note_name(note, name, sizeof(name));
        lv_label_set_text(lv_obj_get_child(c, 0), name);
        const int z = s_haveSpec ? zone_for(note) : -1;
        lv_obj_set_style_border_color(c, z >= 0 ? lv_color_hex(kZoneRgb[z % kZoneColors]) : kPanel, 0);
    }
    if (s_range) {
        char a[8], b[8], buf[24];
        note_name(pad_note(0), a, sizeof(a));
        note_name(pad_note(kPads - 1), b, sizeof(b));
        std::snprintf(buf, sizeof(buf), "%s - %s", a, b);
        lv_label_set_text(s_range, buf);
    }
    s_drawnLit = 0xFFFFFFFFu;   // redrawn on the next tick
}

/// The pads' LEDs in their zones' colours: dim at rest, full while hit.
void show_leds()
{
    auto& leds = crosspad::getPadLedController();
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        const int z = s_haveSpec ? zone_for(pad_note(pad)) : -1;
        const uint32_t rgb = z >= 0 ? kZoneRgb[z % kZoneColors] : 0x202020;
        const crosspad::RgbColor c((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
        leds.setPadColor(pad, c.Dim(70));
        leds.setPadPressedColor(pad, c);
    }
}

void show_lit()
{
    const uint32_t lit = s_lit.load();
    if (lit == s_drawnLit) return;
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        if (!s_cells[pad]) continue;
        const bool on = lit & (1u << pad);
        lv_obj_set_style_bg_color(s_cells[pad], on ? kAccent : kPanel, 0);
        lv_obj_set_style_text_color(s_cells[pad], on ? kInk : kText, 0);
    }
    s_drawnLit = lit;
}

void show_volume()
{
    if (!s_volBtn) return;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "VOL %u", (unsigned)s_volume);
    lv_label_set_text(lv_obj_get_child(s_volBtn, 0), buf);
}

void show_name()
{
    if (!s_dropdown) return;
    std::snprintf(s_ddText, sizeof(s_ddText), "%s%s", s_haveSpec ? s_spec.name.c_str() : "Pick a sample",
                  s_dirty ? " *" : "");
    lv_dropdown_set_text(s_dropdown, s_ddText);
    lv_obj_invalidate(s_dropdown);
}

void show_info(const char* text = nullptr)
{
    if (!s_info) return;
    if (text) { lv_label_set_text(s_info, text); return; }
    if (!s_haveSpec) {
        lv_label_set_text(s_info, "A sample from the list plays on every pad, in tune.");
        return;
    }
    const size_t n = s_spec.instrument.zones.size();
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%u zone%s   |   %s", (unsigned)n, n == 1 ? "" : "s",
                  s_playable.load() ? "pads and MIDI play it" : "not loaded");
    lv_label_set_text(s_info, buf);
}

// ── the engine ─────────────────────────────────────────────────────────────

void apply_volume()
{
    for (auto& p : s_spec.pads) p.volume = s_volume;
    if (auto* p = inst(); p && s_playable.load()) {
        for (uint8_t pad = 0; pad < kPads; ++pad) p->setPadMix(pad, s_volume, 0);
    }
}

/// Build s_spec's bank; the timer takes the answer.
void load_spec()
{
    auto* h = host();
    if (!h) { show_info("No sample engine on this platform."); return; }
    apply_volume();
    if (!h->requestLoad(s_spec)) { show_info("Busy -- try again."); return; }
    s_wait = Wait::Load;
    s_playable.store(false);
    show_info("Loading the samples ...");
}

void open_instrument(const std::string& path)
{
    auto* h = host();
    if (!h) return;
    if (!h->requestOpen(path)) { show_info("Busy -- try again."); return; }
    s_wait = Wait::Load;
    s_playable.store(false);
    show_info("Opening ...");
}

/// One WAV is an instrument of one zone, its pitch detected.
void open_sample(const std::string& logicalPath)
{
    s_spec = crosspad::KitInfo();
    std::string name = base_of(logicalPath);
    if (name.size() > 4) name.resize(name.size() - 4);
    s_spec.name = name;
    s_spec.engine = crosspad::KitEngine::Pitched;
    crosspad::InstrumentZoneSpec z;
    z.sample = logicalPath;
    s_spec.instrument.zones.push_back(z);
    s_haveSpec = true;
    s_dirty = false;
    s_followRoot = true;
    show_name();
    load_spec();
}

// ── the drop-down ──────────────────────────────────────────────────────────

/// The library, then the Recorder's takes: two listings, one after the other.
void request_listing()
{
    auto* st = tools();
    auto* h = host();
    if (!st || !h) return;
    if (st->requestFolder(st->cardRoot() + h->libraryRoot())) s_listing = Listing::Library;
}

/* The first row is a heading, and the list goes back to it after every
 * choice: LVGL tells a choice made with the encoder only when it differs
 * from the one before, so picking the same sample twice -- or the first
 * row -- would otherwise do nothing. */
constexpr const char* kHeading = "Instruments and samples";

void fill_dropdown()
{
    if (!s_dropdown) return;
    /* Not under the finger: options replaced while the list is open close it
     * or shift what the encoder points at. */
    if (lv_dropdown_is_open(s_dropdown)) { s_fillPending = true; return; }
    s_fillPending = false;
    s_entries.clear();
    std::string opts = kHeading;
    auto add = [&](EntryKind k, const std::string& path, const std::string& label) {
        if (!opts.empty()) opts += "\n";
        opts += label;
        s_entries.push_back({k, path});
    };
    auto* h = host();
    for (const auto& n : s_library) {
        add(EntryKind::Instrument, h->libraryRoot() + "/" + n + "/" + crosspad::IInstrumentHost::kFileName,
            std::string(LV_SYMBOL_KEYBOARD " ") + n);
    }
    for (const auto& t : s_takes) add(EntryKind::Sample, t, std::string(LV_SYMBOL_AUDIO " ") + base_of(t));
    add(EntryKind::Card, "", LV_SYMBOL_DIRECTORY " The card ...");
    lv_dropdown_set_options(s_dropdown, opts.c_str());
    lv_dropdown_set_selected(s_dropdown, 0);
    show_name();
}

void open_browser(bool forZone);

void dropdown_cb(lv_event_t* e)
{
    lv_obj_t* dd = static_cast<lv_obj_t*>(lv_event_get_target(e));
    const uint32_t row = lv_dropdown_get_selected(dd);
    lv_dropdown_set_selected(dd, 0);
    show_name();
    if (row == 0 || row - 1 >= s_entries.size()) return;
    const Entry entry = s_entries[row - 1];
    switch (entry.kind) {
    case EntryKind::Instrument: open_instrument(entry.path); break;
    case EntryKind::Sample:     open_sample(entry.path); break;
    case EntryKind::Card:       open_browser(false); break;
    }
}

// ── overlays ───────────────────────────────────────────────────────────────

void close_overlay()
{
    if (!s_overlay) return;
#ifdef ROM_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_overlayGroup);     // the click that closed it is still being read
#endif
    s_overlayGroup = nullptr;
    lv_obj_delete_async(s_overlay);
    s_overlay = s_zoneList = s_rootLabel = s_nameArea = nullptr;
    s_ov = Overlay::None;
}

/// A full-screen panel over the app, a column, with its own encoder group.
lv_obj_t* open_overlay(Overlay kind)
{
    close_overlay();
    s_ov = kind;
    s_overlay = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_overlay);
    lv_obj_set_size(s_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(s_overlay, LV_OBJ_FLAG_FLOATING);
    lv_obj_set_style_bg_color(s_overlay, kBg, 0);
    lv_obj_set_style_bg_opa(s_overlay, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_overlay, 6, 0);
    lv_obj_set_style_pad_row(s_overlay, 4, 0);
    lv_obj_set_flex_flow(s_overlay, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_text_color(s_overlay, kText, 0);
#ifdef ROM_USE_ENCODER
    s_overlayGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_overlayGroup);
#endif
    return s_overlay;
}

bool back_handler(lv_obj_t*);

lv_obj_t* overlay_header(const char* title)
{
    lv_obj_t* top = make_row(s_overlay, kRowH);
    make_button(top, 30, LV_SYMBOL_LEFT, [](lv_event_t*) { back_handler(nullptr); }, s_overlayGroup);
    lv_obj_t* t = lv_label_create(top);
    lv_obj_set_flex_grow(t, 1);
    lv_label_set_long_mode(t, LV_LABEL_LONG_DOT);
    lv_label_set_text(t, title);
    return top;
}

/// A full-width row of a list: what the browser and EDIT show.
lv_obj_t* list_item(lv_obj_t* parent, const char* text, lv_event_cb_t cb, void* ud)
{
    lv_obj_t* b = lv_button_create(parent);
    style_button(b, LV_PCT(100), kRowH);
    lv_obj_set_style_pad_hor(b, 8, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
    lv_obj_set_width(l, LV_PCT(100));
    lv_label_set_text(l, text);
    lv_obj_align(l, LV_ALIGN_LEFT_MID, 0, 0);
    if (s_overlayGroup) lv_group_add_obj(s_overlayGroup, b);
    return b;
}

lv_obj_t* scroll_column(lv_obj_t* parent)
{
    lv_obj_t* c = lv_obj_create(parent);
    lv_obj_remove_style_all(c);
    lv_obj_set_width(c, LV_PCT(100));
    lv_obj_set_flex_grow(c, 1);
    lv_obj_set_flex_flow(c, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(c, 3, 0);
    lv_obj_set_scrollbar_mode(c, LV_SCROLLBAR_MODE_OFF);
    return c;
}

// ── the card ───────────────────────────────────────────────────────────────

void browse(const std::string& dir)
{
    auto* st = tools();
    if (!st) return;
    if (st->requestFolder(dir)) {
        s_folderPending = true;
        show_info("Reading the card ...");
    } else {
        show_info("The card is busy -- try again.");
    }
}

void open_edit();

void browse_entry_cb(lv_event_t* e)
{
    const size_t i = (size_t)(intptr_t)lv_event_get_user_data(e);
    if (i == SIZE_MAX - 1) { s_page += kPageRows; open_browser(s_browseForZone); return; }
    if (i >= s_folder.size()) return;
    const std::string full = s_folder.path + "/" + s_folder.name(i);
    if (s_folder.isFolder(i)) { browse(full); return; }
    const std::string path = logical(full);
    if (!s_browseForZone) {
        close_overlay();
        open_sample(path);
        return;
    }
    if (s_spec.instrument.zones.size() >= crosspad::kInstrumentMaxZones) {
        show_info("An instrument has room for 24 zones.");
    } else {
        crosspad::InstrumentZoneSpec z;
        z.sample = path;
        s_spec.instrument.zones.push_back(z);
        s_zoneSel = (int)s_spec.instrument.zones.size() - 1;
        s_dirty = true;
        show_name();
    }
    open_edit();
}

void browse_up_cb(lv_event_t*)
{
    browse(s_folder.path.substr(0, s_folder.path.find_last_of('/')));
}

/// The listing in s_folder, a page of it, or a request for one.
void open_browser(bool forZone)
{
    s_browseForZone = forZone;
    auto* st = tools();
    if (!st) return;
    /* The folder last listed, at once; the first time, the recordings, once
     * the card has answered (take_tools() comes back here). */
    if (s_folder.path.empty()) {
        if (!s_folderPending) browse(s_browseDir);
        return;
    }
    open_overlay(Overlay::Browse);
    const std::string root = st->cardRoot();
    const bool atRoot = s_folder.path.size() <= root.size();
    const std::string where = atRoot ? std::string("/") : s_folder.path.substr(root.size());
    overlay_header((std::string(forZone ? "Add a zone: " : "A sample: ") + where).c_str());
    lv_obj_t* list = scroll_column(s_overlay);
    if (!atRoot) list_item(list, LV_SYMBOL_UP " ..", browse_up_cb, nullptr);
    const size_t end = std::min<size_t>(s_folder.size(), (size_t)s_page + kPageRows);
    for (size_t i = s_page; i < end; ++i) {
        const std::string label = std::string(s_folder.isFolder(i) ? LV_SYMBOL_DIRECTORY " " : LV_SYMBOL_AUDIO " ") +
                                  s_folder.name(i);
        list_item(list, label.c_str(), browse_entry_cb, (void*)(intptr_t)i);
    }
    if (end < s_folder.size()) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), LV_SYMBOL_DOWN " %u more", (unsigned)(s_folder.size() - end));
        list_item(list, buf, browse_entry_cb, (void*)(intptr_t)(SIZE_MAX - 1));
    }
    if (!s_folder.size()) {
        lv_obj_t* l = lv_label_create(list);
        lv_obj_set_style_text_color(l, kDim, 0);
        lv_label_set_text(l, "No folders or WAVs here.");
    }
}

// ── EDIT ───────────────────────────────────────────────────────────────────

void show_zones();

void zone_cb(lv_event_t* e)
{
    s_zoneSel = (int)(intptr_t)lv_event_get_user_data(e);
    show_zones();
}

void show_root()
{
    if (!s_rootLabel) return;
    const auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) { lv_label_set_text(s_rootLabel, "--"); return; }
    const float r = zones[s_zoneSel].root;
    char buf[16];
    if (r < 0) std::snprintf(buf, sizeof(buf), "AUTO");
    else       note_name((int)std::lround(r), buf, sizeof(buf));
    lv_label_set_text(s_rootLabel, buf);
}

void show_zones()
{
    if (!s_zoneList) return;
    lv_obj_clean(s_zoneList);
    const auto& zones = s_spec.instrument.zones;
    for (size_t i = 0; i < zones.size(); ++i) {
        char root[8], buf[96];
        if (zones[i].root < 0) std::snprintf(root, sizeof(root), "auto");
        else note_name((int)std::lround(zones[i].root), root, sizeof(root));
        std::snprintf(buf, sizeof(buf), "%u  %s   %s", (unsigned)(i + 1), root, base_of(zones[i].sample).c_str());
        lv_obj_t* b = list_item(s_zoneList, buf, zone_cb, (void*)(intptr_t)i);
        lv_obj_set_style_border_side(b, LV_BORDER_SIDE_LEFT, 0);
        lv_obj_set_style_border_width(b, (int)i == s_zoneSel ? 3 : 0, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(kZoneRgb[i % kZoneColors]), 0);
        if ((int)i == s_zoneSel && s_overlayGroup) lv_group_focus_obj(b);
    }
    if (zones.empty()) {
        lv_obj_t* l = lv_label_create(s_zoneList);
        lv_obj_set_style_text_color(l, kDim, 0);
        lv_label_set_text(l, "No zones. ADD takes a WAV from the card.");
    }
    show_root();
}

void root_by(int step)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const int r = z.root < 0 ? 60 : (int)std::lround(z.root);
    z.root = (float)std::max(0, std::min(127, r + step));
    s_dirty = true;
    show_zones();
}

void open_edit()
{
    open_overlay(Overlay::Edit);
    lv_obj_t* top = overlay_header("Zones");
    make_button(top, 56, LV_SYMBOL_PLUS " ADD", [](lv_event_t*) { open_browser(true); }, s_overlayGroup);
    s_zoneList = scroll_column(s_overlay);
    lv_obj_t* tools = make_row(s_overlay, kRowH);
    lv_obj_t* l = lv_label_create(tools);
    lv_obj_set_style_text_color(l, kDim, 0);
    lv_label_set_text(l, "ROOT");
    make_button(tools, 34, "-", [](lv_event_t*) { root_by(-1); }, s_overlayGroup);
    s_rootLabel = lv_label_create(tools);
    lv_obj_set_width(s_rootLabel, 48);
    lv_obj_set_style_text_align(s_rootLabel, LV_TEXT_ALIGN_CENTER, 0);
    make_button(tools, 34, "+", [](lv_event_t*) { root_by(1); }, s_overlayGroup);
    make_button(tools, 54, "AUTO", [](lv_event_t*) {
        auto& zones = s_spec.instrument.zones;
        if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
        zones[s_zoneSel].root = -1.0f;
        s_dirty = true;
        show_zones();
    }, s_overlayGroup);
    make_button(tools, 48, "DEL", [](lv_event_t*) {
        auto& zones = s_spec.instrument.zones;
        if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
        zones.erase(zones.begin() + s_zoneSel);
        s_zoneSel = std::min<int>(s_zoneSel, (int)zones.size() - 1);
        s_dirty = true;
        show_zones();
    }, s_overlayGroup);
    show_zones();
}

/// Leaving EDIT: what was changed is built and played.
void edit_done()
{
    close_overlay();
    if (!s_dirty) return;
    show_name();
    if (s_spec.instrument.zones.empty()) {
        s_haveSpec = false;
        s_playable.store(false);
        show_pads();
        show_info("No zones left.");
        return;
    }
    s_haveSpec = true;
    load_spec();
}

void edit_cb(lv_event_t*)
{
    if (!s_haveSpec) {
        s_spec = crosspad::KitInfo();
        s_spec.name = "New instrument";
        s_spec.engine = crosspad::KitEngine::Pitched;
    }
    s_zoneSel = s_spec.instrument.zones.empty() ? -1 : std::max(0, s_zoneSel);
    open_edit();
}

// ── SAVE ───────────────────────────────────────────────────────────────────

bool valid_name(const std::string& n)
{
    if (n.empty() || n.size() > 32) return false;
    for (char c : n) {
        if (c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' || c == '"' ||
            c == '<' || c == '>' || c == '|') return false;
    }
    return n != "." && n != "..";
}

void name_done_cb(lv_event_t* e)
{
    const bool ok = lv_event_get_code(e) == LV_EVENT_READY;
    const std::string name = s_nameArea ? lv_textarea_get_text(s_nameArea) : "";
    close_overlay();
    if (!ok) return;
    if (!valid_name(name)) { show_info("A name of 1-32 letters, without / \\ : * ? \" < > |"); return; }
    auto* h = host();
    if (!h) return;
    crosspad::KitInfo spec = s_spec;
    spec.name = name;
    spec.path = h->libraryRoot() + "/" + name + "/" + crosspad::IInstrumentHost::kFileName;
    if (!h->requestSave(spec)) { show_info("Busy -- try again."); return; }
    s_wait = Wait::Save;
    show_info("Saving ...");
}

void save_cb(lv_event_t*)
{
    if (!s_haveSpec || s_spec.instrument.zones.empty()) { show_info("Nothing to save yet."); return; }
    open_overlay(Overlay::Name);
    lv_obj_set_style_pad_row(s_overlay, 2, 0);
    s_nameArea = lv_textarea_create(s_overlay);
    lv_textarea_set_one_line(s_nameArea, true);
    lv_textarea_set_max_length(s_nameArea, 32);
    lv_textarea_set_text(s_nameArea, s_spec.name.c_str());
    lv_obj_set_width(s_nameArea, LV_PCT(100));
    lv_obj_t* kb = lv_keyboard_create(s_overlay);
    lv_obj_set_size(kb, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_grow(kb, 1);
    lv_keyboard_set_textarea(kb, s_nameArea);
    lv_obj_add_event_cb(kb, name_done_cb, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(kb, name_done_cb, LV_EVENT_CANCEL, nullptr);
    if (s_overlayGroup) {
        lv_group_add_obj(s_overlayGroup, kb);
        lv_group_focus_obj(kb);
        lv_group_set_editing(s_overlayGroup, true);
    }
}

// ── the main row callbacks ─────────────────────────────────────────────────

void back_cb(lv_event_t* e)
{
    auto* app = static_cast<App*>(lv_event_get_user_data(e));
    if (app) app->destroyAppAsync();
}

void oct_cb(lv_event_t* e)
{
    const int step = (int)(intptr_t)lv_event_get_user_data(e);
    const int base = (int)s_base + 12 * step;
    if (base < 0 || base + kPads - 1 > 127) return;
    s_base = (uint8_t)base;
    show_pads();
    show_leds();
}

void volume_cb(lv_event_t* e)
{
    const int step = number_step(e);
    if (!step) return;
    s_volume = (uint8_t)std::max(0, std::min(100, (int)s_volume + step * kVolStep));
    apply_volume();
    show_volume();
}

/// A touch on a pad's cell plays it, as the pad would.
void cell_cb(lv_event_t* e)
{
    const uint8_t pad = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        s_padNote[pad].store(pad_note(pad));
        play(pad_note(pad), 100, pad);
    } else {
        stop(s_padNote[pad].load());
    }
}

bool back_handler(lv_obj_t*)
{
    if (s_ov == Overlay::Edit) { edit_done(); return true; }
    if (s_ov == Overlay::Browse && s_browseForZone) { open_edit(); return true; }
    if (s_overlay) { close_overlay(); return true; }
    return false;
}

// ── the timer ──────────────────────────────────────────────────────────────

void take_host()
{
    auto* h = host();
    if (!h || s_wait == Wait::None) return;
    const auto state = h->state();
    if (state == crosspad::IInstrumentHost::State::Busy) return;
    const Wait was = s_wait;
    s_wait = Wait::None;
    const std::string msg = h->message();
    if (state == crosspad::IInstrumentHost::State::Failed) {
        if (was == Wait::Load) s_playable.store(false);
        s_followRoot = false;
        show_info(msg.empty() ? "That did not work." : msg.c_str());
        return;
    }
    s_spec = h->instrument();
    s_haveSpec = true;
    if (was == Wait::Load) {
        s_dirty = false;
        /* One sample: pad 8 plays it as recorded, the pads around it a few
         * semitones either way -- a drum loop detected at F#6 otherwise
         * played three octaves down, a murmur. */
        if (s_followRoot && s_spec.instrument.zones.size() == 1 && s_spec.instrument.zones[0].root >= 0) {
            const int root = (int)std::lround(s_spec.instrument.zones[0].root);
            s_base = (uint8_t)std::max(0, std::min(127 - (kPads - 1), root - 7));
        }
        s_followRoot = false;
        s_playable.store(state == crosspad::IInstrumentHost::State::Ready);
        apply_volume();
        show_pads();
        show_leds();
        show_name();
        if (!msg.empty()) show_info(msg.c_str());
        else show_info();
    } else {
        s_dirty = false;
        show_name();
        char buf[80];
        std::snprintf(buf, sizeof(buf), "Saved as %s", s_spec.name.c_str());
        show_info(buf);
        request_listing();
    }
}

void take_tools()
{
    auto* st = tools();
    if (!st) return;
    const auto job = st->job();
    if (job == crosspad::ToolJob::Busy) return;
    if (s_folderPending) {
        s_folderPending = false;
        if (job == crosspad::ToolJob::Ready) {
            s_folder = st->folder();
            s_browseDir = s_folder.path;
            s_page = 0;
            show_info();
            open_browser(s_browseForZone);
        } else {
            show_info(st->jobResult().c_str());
        }
        return;
    }
    if (s_listing == Listing::Library) {
        s_library.clear();
        if (job == crosspad::ToolJob::Ready) {
            const auto f = st->folder();
            for (size_t i = 0; i < f.folders; ++i) s_library.push_back(f.name(i));
        }
        s_listing = st->requestFolder("") ? Listing::Takes : Listing::None;
        if (s_listing == Listing::None) fill_dropdown();
        return;
    }
    if (s_listing == Listing::Takes) {
        s_takes.clear();
        if (job == crosspad::ToolJob::Ready) {
            const auto f = st->folder();
            for (size_t i = f.folders; i < f.size(); ++i) s_takes.push_back(logical(f.path + "/" + f.name(i)));
        }
        s_listing = Listing::None;
        fill_dropdown();
    }
}

void tick_cb(lv_timer_t*)
{
    take_host();
    take_tools();
    if (s_fillPending) fill_dropdown();
    show_lit();
}

} // namespace

lv_obj_t* PitchPlay_create(lv_obj_t* parent, App* app)
{
#ifdef ROM_USE_ENCODER
    if (indev_encoderTop) s_prevEncGroup = lv_indev_get_group(indev_encoderTop);
    if (!s_encGroup) s_encGroup = lv_group_create();
    if (indev_encoderTop) lv_indev_set_group(indev_encoderTop, s_encGroup);
#endif
    lv_group_t* g = s_encGroup;

    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_root, kBg, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 5, 0);
    lv_obj_set_style_pad_row(s_root, 4, 0);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_text_color(s_root, kText, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    // -- back, the instrument, EDIT, SAVE --
    lv_obj_t* top = make_row(s_root, kRowH);
    make_button(top, 30, LV_SYMBOL_LEFT, back_cb, g, app);
    s_dropdown = lv_dropdown_create(top);
    lv_obj_set_height(s_dropdown, kRowH);
    lv_obj_set_flex_grow(s_dropdown, 1);
    lv_obj_set_style_pad_ver(s_dropdown, 4, 0);
    lv_obj_set_style_pad_hor(s_dropdown, 8, 0);
    lv_obj_set_style_radius(s_dropdown, 6, 0);
    lv_obj_set_style_border_width(s_dropdown, 0, 0);
    lv_obj_set_style_bg_color(s_dropdown, kPanel, 0);
    lv_obj_set_style_text_color(s_dropdown, kText, 0);
    lv_obj_set_style_text_font(s_dropdown, &lv_font_montserrat_14, 0);
    focus_frame(s_dropdown);
    lv_dropdown_set_options(s_dropdown, kHeading);
    lv_obj_add_event_cb(s_dropdown, dropdown_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    if (lv_obj_t* list = lv_dropdown_get_list(s_dropdown)) {
        lv_obj_set_style_bg_color(list, kPanel, 0);
        lv_obj_set_style_text_color(list, kText, 0);
        lv_obj_set_style_text_font(list, &lv_font_montserrat_14, 0);
        lv_obj_set_style_border_color(list, kAccent, 0);
        lv_obj_set_style_bg_color(list, kAccent, (lv_style_selector_t)LV_PART_SELECTED | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(list, kInk, (lv_style_selector_t)LV_PART_SELECTED | LV_STATE_CHECKED);
        lv_obj_set_style_max_height(list, 180, 0);
    }
    if (g) lv_group_add_obj(g, s_dropdown);
    make_button(top, 48, "EDIT", edit_cb, g);
    make_button(top, 48, "SAVE", save_cb, g);

    // -- the pads, bottom left first --
    lv_obj_t* grid = lv_obj_create(s_root);
    lv_obj_remove_style_all(grid);
    lv_obj_set_width(grid, LV_PCT(100));
    lv_obj_set_flex_grow(grid, 1);
    static const int32_t kCol[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static const int32_t kRow[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(grid, kCol, kRow);
    lv_obj_set_style_pad_column(grid, 4, 0);
    lv_obj_set_style_pad_row(grid, 4, 0);
    lv_obj_remove_flag(grid, LV_OBJ_FLAG_SCROLLABLE);
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        lv_obj_t* c = lv_obj_create(grid);
        lv_obj_remove_style_all(c);
        lv_obj_set_grid_cell(c, LV_GRID_ALIGN_STRETCH, pad % 4, 1, LV_GRID_ALIGN_STRETCH, 3 - pad / 4, 1);
        lv_obj_set_style_bg_color(c, kPanel, 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(c, 6, 0);
        lv_obj_set_style_border_side(c, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_border_width(c, 3, 0);
        lv_obj_set_style_border_color(c, kPanel, 0);
        lv_obj_set_style_text_color(c, kText, 0);
        lv_obj_set_style_text_font(c, &lv_font_montserrat_14, 0);
        lv_obj_add_flag(c, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(c, cell_cb, LV_EVENT_PRESSED, (void*)(intptr_t)pad);
        lv_obj_add_event_cb(c, cell_cb, LV_EVENT_RELEASED, (void*)(intptr_t)pad);
        lv_obj_t* l = lv_label_create(c);
        lv_obj_center(l);
        s_cells[pad] = c;
    }

    // -- octave and level --
    lv_obj_t* bottom = make_row(s_root, kRowH);
    make_button(bottom, 50, "OCT-", oct_cb, g, (void*)(intptr_t)-1);
    s_range = lv_label_create(bottom);
    lv_obj_set_flex_grow(s_range, 1);
    lv_obj_set_style_text_align(s_range, LV_TEXT_ALIGN_CENTER, 0);
    make_button(bottom, 50, "OCT+", oct_cb, g, (void*)(intptr_t)1);
    s_volBtn = lv_CreateEditableButton(bottom);
    style_button(s_volBtn, 74, kRowH);
    lv_obj_add_event_cb(s_volBtn, volume_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(s_volBtn, volume_cb, LV_EVENT_CLICKED, nullptr);
    add_label(s_volBtn, "");
    if (g) lv_group_add_obj(g, s_volBtn);

    s_info = lv_label_create(s_root);
    lv_obj_set_style_text_font(s_info, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_info, kDim, 0);
    lv_label_set_long_mode(s_info, LV_LABEL_LONG_DOT);
    lv_obj_set_size(s_info, LV_PCT(100), 12);

    show_pads();
    show_volume();
    show_name();
    show_info();
    request_listing();

    // The pads are the instrument's: framework-owned activation (REGISTER_APP_PL).
    if (!s_padLogic) {
        s_padLogic = std::make_shared<PitchPlayPadLogic>();
        crosspad::getPadManager().registerPadLogic("PitchPlay", s_padLogic);
    }
    auto& bus = crosspad::getEventBus();
    s_noteOnSub = bus.subscribe(crosspad::EventType::NoteOn, midi_note_cb, nullptr);
    s_noteOffSub = bus.subscribe(crosspad::EventType::NoteOff, midi_note_cb, nullptr);

    // What it played last time plays again.
    if (s_haveSpec && !s_spec.instrument.zones.empty()) load_spec();

    if (g) lv_group_focus_obj(s_dropdown);
    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnBack(back_handler);
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
#ifdef ROM_USE_ENCODER
            if (indev_encoderTop) {
                lv_group_t* want = s_overlayGroup ? s_overlayGroup : s_encGroup;
                if (want) lv_indev_set_group(indev_encoderTop, want);
            }
#endif
        });
        app->setOnHide([](lv_obj_t*) {
            if (s_timer) lv_timer_pause(s_timer);
#ifdef ROM_USE_ENCODER
            if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
#endif
        });
    }
    return s_root;
}

void PitchPlay_destroy(lv_obj_t* root)
{
    if (s_timer) {
        lv_timer_delete(s_timer);
        s_timer = nullptr;
    }
    auto& bus = crosspad::getEventBus();
    if (s_noteOnSub) { bus.unsubscribe(s_noteOnSub); s_noteOnSub = nullptr; }
    if (s_noteOffSub) { bus.unsubscribe(s_noteOffSub); s_noteOffSub = nullptr; }
    s_playable.store(false);
    if (auto* p = inst()) p->allNotesOff();
    s_lit.store(0);
    /* The pads' colours are the next app's again: a colour set here pins a
     * pad until it is cleared (PadLedController::clearPadColor). */
    for (uint8_t pad = 0; pad < kPads; ++pad) crosspad::getPadLedController().clearPadColor(pad);
    // The kits get their engine back.
    if (auto* h = host()) h->release();
    s_wait = Wait::None;
    s_listing = Listing::None;
    s_folderPending = false;
    s_fillPending = false;

    s_overlay = s_zoneList = s_rootLabel = s_nameArea = nullptr;
    s_ov = Overlay::None;
#ifdef ROM_USE_ENCODER
    if (s_overlayGroup) { lv_group_delete(s_overlayGroup); s_overlayGroup = nullptr; }
    if (indev_encoderTop && s_prevEncGroup) lv_indev_restore_group(indev_encoderTop, s_prevEncGroup);
    if (s_encGroup) {
        lv_group_delete(s_encGroup);
        s_encGroup = nullptr;
    }
    s_prevEncGroup = nullptr;
#endif
    s_root = s_dropdown = s_info = s_range = s_volBtn = nullptr;
    for (auto& c : s_cells) c = nullptr;
    lv_obj_delete_async(root);
}

REGISTER_APP_PL(PitchPlay, nullptr, "pitchplay.png",
                PitchPlay_create, PitchPlay_destroy,
                nullptr, nullptr, nullptr, 0, "PitchPlay")
