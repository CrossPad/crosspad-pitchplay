/**
 * @file PitchPlayApp.cpp
 * @brief Pitch Play -- one sample or many, played chromatically on the pads and
 *        from a MIDI keyboard.
 *
 *   context  the instrument (tap: the sample window), then what the pads are
 *            set to as plain fields -- the key and scale, the octave, what a
 *            pad makes. No back button: back is the power button's click.
 *   pads     the sixteen pads as they sound, bottom left first: each its
 *            note, underlined in the colour of the zone that plays it, lit
 *            while it sounds; a touch plays it too
 *   right    the note last played and how hard, the selected zone's waveform,
 *            its envelope, the pads' range, and the level (turned)
 *   keys     SCALE, OCT-, OCT+, CHORD, SAMPLE > -- the five of UX 2000 5.9.
 *            EDIT and SAVE live in the sample window behind SAMPLE >, which
 *            is the one key on the row that leads somewhere.
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
#include "crosspad/instrument/InstrumentPlayState.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/instrument/PitchedInstrument.hpp"
#include "crosspad/music/MusicTheory.hpp"
#ifdef ESP_PLATFORM
  #include "nvs.h"
#endif
#include "crosspad/led/RgbColor.hpp"
#include "crosspad/pad/IPadLogicHandler.hpp"
#include "crosspad/pad/PadLedController.hpp"
#include "crosspad/pad/PadManager.hpp"
#include "crosspad/platform/PlatformServices.hpp"
#include "crosspad/sampletools/ISampleTools.hpp"
#include "crosspad-gui/components/transport_bar.h"
#include "crosspad-gui/components/ui_toast.h"
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
/* A screen y as a y inside the root, which begins under the transport bar.
 * The whole screen is laid out in the design's screen coordinates and this is
 * the only place the shift happens. */
#define PP_Y(v) ((v) - TRANSPORT_BAR_HEIGHT)
constexpr uint8_t  kPads     = 16;
constexpr int      kChordMax = 4;    ///< notes in the widest chord a pad makes

/* This board boots with under 150 bytes of internal RAM to spare: the 64 kB
 * DMA pool is reserved after the static data is placed, and the first build
 * of this feature added 144 bytes of it and aborted in main_task with
 * "Could not reserve internal/DMA pool (0x104)". Everything this file keeps
 * per-pad or per-widget therefore lives in PSRAM. None of it is touched from
 * an interrupt or with the cache off, which is the only thing that would
 * forbid it. */
#ifdef ESP_PLATFORM
  #include "esp_attr.h"
  #define PP_SLOW_BSS EXT_RAM_BSS_ATTR
#else
  #define PP_SLOW_BSS
#endif
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
/* The instrument's own colour -- the first zone's, which is also the one the
 * pads of zone 1 are underlined in -- and the well its picture sits in. */
const lv_color_t kZone0  = lv_color_hex(0xA77BF0);
const lv_color_t kWaveBg = lv_color_hex(0x1B1622);

/// A colour per zone, round again past eight: the pads say which zone plays them.
const uint32_t kZoneRgb[] = {0xC6FF47, 0x4FE0B0, 0x8B5CF6, 0xFFC857,
                             0xFF5C5C, 0x5CA8FF, 0xFF7AD9, 0xFF9F43};
constexpr size_t kZoneColors = sizeof(kZoneRgb) / sizeof(kZoneRgb[0]);

enum class Wait : uint8_t { None, Load, Save };
enum class Listing : uint8_t { None, Library, Takes };
enum class Overlay : uint8_t { None, Edit, Browse, Name, Scale, Prog, Sample };
enum class EntryKind : uint8_t { Instrument, Sample, Card };

struct Entry {
    EntryKind   kind;
    std::string path;   ///< logical: an instrument file or a WAV
};

lv_obj_t* s_root = nullptr;
lv_obj_t* s_range = nullptr;
lv_obj_t* s_volBtn = nullptr;
lv_obj_t* s_scaleBtn = nullptr;     ///< context row: the key and scale, in short
lv_obj_t* s_nameBtn  = nullptr;     ///< context row: the instrument, in its colour
lv_obj_t* s_octField = nullptr;     ///< context row: "OCT 3"
lv_obj_t* s_chordField = nullptr;   ///< context row: "CHORD off"
lv_obj_t* s_noteLbl  = nullptr;     ///< right panel: "G3 - vel 96"
lv_obj_t* s_waveBox  = nullptr;     ///< right panel: the selected zone's picture
lv_obj_t* s_envLbl   = nullptr;     ///< right panel: "ATK 2  REL 40  GLIDE 0"
PP_SLOW_BSS lv_obj_t* s_scaleVal[4];   ///< the window's four value labels
PP_SLOW_BSS lv_obj_t* s_progChip[8];    ///< the progression editor's step chips
PP_SLOW_BSS lv_obj_t* s_progVal[2];     ///< its degree and chord values
lv_obj_t* s_cells[kPads] = {};
lv_obj_t* s_overlay = nullptr;
lv_obj_t* s_zoneList = nullptr;
lv_obj_t* s_rootLabel = nullptr;
lv_obj_t* s_loopLabel = nullptr;   ///< "0.42 - 1.85 s", or OFF
lv_obj_t* s_loopFill  = nullptr;   ///< the lit part of the loop bar
lv_obj_t* s_loopBtn   = nullptr;   ///< ON/OFF for the selected zone's loop
PP_SLOW_BSS lv_obj_t* s_keyCell[12];   ///< the scale strip, one per pitch class
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
uint8_t s_volume = 80;         // every pad's level, 0..100: zones peak at -1 dBFS, a chord needs room
bool    s_followRoot = false;  // a sample just picked: its root goes to the middle of the pads

bool    s_dirty = false;       // zones edited since the bank was built
Wait    s_wait = Wait::None;
Listing s_listing = Listing::None;
std::vector<std::string> s_library;   // instrument names
std::vector<std::string> s_takes;     // logical paths
std::vector<Entry> s_entries;         // what the sample window offers, in its order
std::vector<std::string> s_labels;    // and how each of them reads
/* The window shows a page of them, not all. A board with a few hundred takes
 * made 229 rows of this list, and the 200 kB of PSRAM they took is most of
 * what the pitched bank then had to load into -- the same trap the kit
 * selector is a virtual list for. Measured: 404 kB free before the window,
 * 204 kB with it open. */
uint32_t s_samplePage = 0;
char    s_ddText[48] = "Pick a sample";
bool    s_fillPending = false;        // a listing came in while the sample window is open

/* The note last played, for the right panel. Written from play_one(), so a
 * pad, a touch and a MIDI key all say the same thing. */
std::atomic<uint8_t> s_lastNote{0};
std::atomic<uint8_t> s_lastVel{0};

/* The selected zone's picture: columns asked of the tools and drawn in the
 * waveform box. Requested once per zone, and never while a folder listing is
 * out -- the tools run one job at a time. */
crosspad::WaveOverview s_wave;
bool        s_wavePending = false;
std::string s_waveWant;               ///< the file the pending request is for

Overlay s_ov = Overlay::None;
bool    s_browseForZone = false;       // the card's WAV becomes a zone, not an instrument
std::string          s_browseDir;      // "" = the recordings
crosspad::CardFolder s_folder;
bool     s_folderPending = false;
uint32_t s_page = 0;
int      s_zoneSel = 0;

std::atomic<bool>     s_playable{false};
std::atomic<uint32_t> s_lit{0};
/* What a pad struck, so its release stops those notes and not whatever the
 * mapping says now: four 7-bit notes and a 3-bit count in one aligned word,
 * which a single store publishes and a single load reads. */
PP_SLOW_BSS uint32_t s_padHeld[kPads];
constexpr uint32_t pack_chord(const uint8_t* n, int count)
{
    return (uint32_t)count | ((uint32_t)n[0] << 3) | ((uint32_t)n[1] << 10)
         | ((uint32_t)n[2] << 17) | ((uint32_t)n[3] << 24);
}
inline int unpack_chord(uint32_t w, uint8_t* out)
{
    const int count = (int)(w & 7u);
    out[0] = (uint8_t)((w >> 3) & 0x7F);
    out[1] = (uint8_t)((w >> 10) & 0x7F);
    out[2] = (uint8_t)((w >> 17) & 0x7F);
    out[3] = (uint8_t)((w >> 24) & 0x7F);
    return count > kChordMax ? kChordMax : count;
}
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

PP_SLOW_BSS int s_key;          ///< pitch class of the tonic, 0..11

/* Two spellings of the same twelve. A progression written in Eb that reads
 * "D#" on the pads is not a different sound, it is a different piece of
 * paper -- and this list is full of flat keys (Db, Eb, Ab, Bb). The key
 * decides: the six whose signatures carry flats get flats, the rest sharps.
 * F is among them with its one flat, though F itself is natural. */
constexpr const char* kSharpName[12] = {"C", "C#", "D", "D#", "E", "F", "F#",
                                        "G", "G#", "A", "A#", "B"};
constexpr const char* kFlatName[12]  = {"C", "Db", "D", "Eb", "E", "F", "Gb",
                                        "G", "Ab", "A", "Bb", "B"};

bool key_is_flat(int key)
{
    switch (key) {
        case 1: case 3: case 5: case 6: case 8: case 10: return true;   // Db Eb F Gb Ab Bb
        default: return false;
    }
}

const char* pitch_name(int pitchClass, int key)
{
    const int pc = ((pitchClass % 12) + 12) % 12;
    return key_is_flat(key) ? kFlatName[pc] : kSharpName[pc];
}

/// The key itself, spelled the way its own signature would.
const char* key_name(int key) { return pitch_name(key, key); }

void note_name(int note, char* buf, size_t n)
{
    if (note < 0 || note > 127) { std::snprintf(buf, n, "--"); return; }
    std::snprintf(buf, n, "%s%d", pitch_name(note, s_key), note / 12 - 1);
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

// ── key, scale, chords and progressions ───────────────────────────────────

/* All of this is crosspad::MusicTheory's: nine scales, fourteen chord types
 * (Diatonic among them, which takes its quality from the degree), eight
 * progressions and room for eight more of the player's own. It was in core
 * all along with nothing wired to it -- the linker was dropping the whole
 * translation unit. This app is the first thing to ask for it, so the
 * mapping, the chord voicings and the degree-to-quality tables are not
 * repeated here.
 *
 * PlaySettings is the answer: sixteen pads' notes and chord notes,
 * recomputed by rebuildPadMap() whenever the key, octave, scale or chord
 * changes. Transpose is ours, applied on top of what it returns -- a
 * semitone shift of everything, which is what a singer asks for, and not the
 * same gesture as changing the key.
 *
 * In PSRAM, all of it: PlaySettings is ~150 bytes and the custom
 * progressions another ~400, and this board aborts at boot over 144. */

PP_SLOW_BSS crosspad::MusicTheory  s_mt;
PP_SLOW_BSS crosspad::PlaySettings s_ps;

PP_SLOW_BSS int s_oct;          ///< PlaySettings octave, -2..+4
constexpr int kOctDefault = 1;  ///< MusicTheory's base is (3+oct)*12, so 1 puts pad 1 on C3,
                                ///< where this app has always started
PP_SLOW_BSS int s_scale;        ///< index into MusicTheory's scales
PP_SLOW_BSS int s_chordIdx;     ///< 0..11 a chord type, 12 Diatonic, 13 Types, 14+ a progression
PP_SLOW_BSS int s_transpose;    ///< semitones, -12..+12, on top of all of it


int clamp_note(int n) { return n < 0 ? 0 : (n > 127 ? 127 : n); }

/* Pitch Play is the only writer of the shared state (core's
 * InstrumentPlayState): the Sequencer recording an instrument and the
 * Recorder bouncing one have to sound like these pads do, and they only can
 * if they read this instead of keeping a copy of the key, the scale and the
 * chord. Published from the two places the state can change -- the map here,
 * the instrument when a load lands. */
void publish_play_state()
{
    auto& st = crosspad::instrumentPlayState();
    st.play = s_ps;                  // rebuildPadMap() has already run on it
    const char* cn = s_mt.chordName((uint8_t)s_chordIdx);
    st.chordName = cn ? cn : "";     // named here: only this MusicTheory knows
                                     // the player's own progressions
    crosspad::instrumentPlayStatePublished();
}

void rebuild_pads()
{
    s_ps.rootNote   = (uint8_t)s_key;
    s_ps.octave     = (int8_t)s_oct;
    s_ps.scaleIndex = (uint8_t)s_scale;
    s_ps.chordIndex = (uint8_t)s_chordIdx;
    s_mt.rebuildPadMap(s_ps);
    if (s_transpose == 0) { publish_play_state(); return; }
    for (int pad = 0; pad < kPads; ++pad) {
        s_ps.padMidiMap[pad] = (uint8_t)clamp_note(s_ps.padMidiMap[pad] + s_transpose);
        for (int c = 0; c < 6; ++c) {
            if (s_ps.padChordNotes[pad][c] == 0xFF) continue;
            s_ps.padChordNotes[pad][c] =
                (uint8_t)clamp_note(s_ps.padChordNotes[pad][c] + s_transpose);
        }
    }
    publish_play_state();
}

/// Which pad sounds @p note, or -1. Scans: a scale leaves gaps.
int pad_of_note(int note)
{
    for (uint8_t pad = 0; pad < kPads; ++pad) if (s_ps.padMidiMap[pad] == note) return pad;
    return -1;
}

/// The notes a press on @p pad makes, into @p out. Always at least one.
int chord_of(uint8_t pad, uint8_t* out)
{
    if (pad >= kPads) { out[0] = 0; return 1; }
    int n = 0;
    const int have = s_ps.padChordCount[pad];
    for (int c = 0; c < have && c < 6 && n < kChordMax; ++c) {
        const uint8_t note = s_ps.padChordNotes[pad][c];
        if (note != 0xFF) out[n++] = note;
    }
    if (n == 0) out[n++] = s_ps.padMidiMap[pad];
    return n;
}

/* The player's own progressions.
 *
 * MusicTheory holds up to eight and plays them, but it has no way to read
 * one back, so this keeps the copies: they are what the editor edits, what
 * NVS stores and what is handed back to MusicTheory on every start. A
 * progression is steps of (degree, chord type) -- degrees of whatever scale
 * is current, so the same progression follows a key or a mode change. */

constexpr int kMaxCustom = crosspad::MusicTheory::MAX_CUSTOM_PROGS;
constexpr int kMaxSteps  = 8;      ///< of MusicTheory's sixteen; eight fits the screen

struct CustomProgression {
    char              name[12];
    crosspad::ProgStep steps[kMaxSteps];
    uint8_t           count;
};

PP_SLOW_BSS CustomProgression s_custom[kMaxCustom];
PP_SLOW_BSS int s_customN;
PP_SLOW_BSS int s_editSlot;        ///< which of them the editor is on
PP_SLOW_BSS int s_editSel;         ///< which step is being changed
PP_SLOW_BSS CustomProgression s_edit;   ///< the working copy, so Back discards

constexpr const char* kNvsNs  = "pitchplay";
constexpr const char* kNvsKey = "progs";

/// Hand every stored progression back to MusicTheory, in slot order.
void register_customs()
{
    s_mt.clearCustomProgressions();
    for (int i = 0; i < s_customN; ++i) {
        s_mt.addCustomProgression(s_custom[i].name, s_custom[i].steps, s_custom[i].count);
    }
}

/// The chord index that plays custom progression @p slot.
int custom_chord_index(int slot)
{
    return crosspad::MusicTheory::CHORD_PROG_START_IDX
         + (int)crosspad::MusicTheory::builtinProgCount() + slot;
}

void save_customs()
{
#ifdef ESP_PLATFORM
    nvs_handle_t h;
    if (nvs_open(kNvsNs, NVS_READWRITE, &h) != ESP_OK) return;
    const uint8_t n = (uint8_t)s_customN;
    nvs_set_u8(h, "n", n);
    if (n) nvs_set_blob(h, kNvsKey, s_custom, sizeof(CustomProgression) * n);
    nvs_commit(h);
    nvs_close(h);
#endif
}

void load_customs()
{
#ifdef ESP_PLATFORM
    nvs_handle_t h;
    if (nvs_open(kNvsNs, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t n = 0;
    if (nvs_get_u8(h, "n", &n) == ESP_OK && n > 0 && n <= kMaxCustom) {
        size_t len = sizeof(CustomProgression) * n;
        if (nvs_get_blob(h, kNvsKey, s_custom, &len) == ESP_OK) s_customN = n;
    }
    nvs_close(h);
#endif
    register_customs();
}

/// "C", "Am", "G7" -- the pad's root and the quality its chord was built with.
void pad_chord_name(uint8_t pad, char* buf, size_t len)
{
    if (pad >= kPads) { std::snprintf(buf, len, "--"); return; }
    const int note = s_ps.padMidiMap[pad];
    const uint8_t q = s_ps.padChordTypeIdx[pad];
    const char qc = crosspad::MusicTheory::chordQualChar(q);
    const char* root = pitch_name(note, s_key);
    if (qc && qc != ' ') std::snprintf(buf, len, "%s%c", root, qc);
    else                 std::snprintf(buf, len, "%s", root);
}

/// True while a pad sounds more than its own note.
bool pads_are_chords() { return s_ps.padChordCount[0] > 1; }


uint8_t pad_note(uint8_t pad) { return pad < kPads ? s_ps.padMidiMap[pad] : 0; }

// ── playing ────────────────────────────────────────────────────────────────

/* A pad lights while its note sounds, from the pads, a touch or a keyboard:
 * with a pad logic active, PadManager leaves the LEDs to it. */
/// One note, no lighting: what a chord is built from.
void play_one(uint8_t note, uint8_t vel, uint8_t pad)
{
    auto* p = inst();
    if (!p || !s_playable.load()) return;
    /* What the right panel names. Here rather than in the pad handler: a
     * touch, a pad and a MIDI key all come through this one call, so the
     * panel says what was actually sounded whichever way it arrived. */
    s_lastNote.store(note);
    s_lastVel.store(vel ? vel : 1);
    p->noteOn(note, vel ? vel : 1, pad);
}

void light_on(int pad, uint8_t vel)
{
    if (pad < 0) return;
    s_lit.fetch_or(1u << pad);
    crosspad::getPadManager().notifyNoteOn((uint8_t)pad, vel ? vel : 1, true);
}

void light_off(int pad)
{
    if (pad < 0) return;
    s_lit.fetch_and(~(1u << pad));
    crosspad::getPadManager().notifyNoteOff((uint8_t)pad, true);
}

/* A note from somewhere other than the pads -- a MIDI keyboard, a touch on a
 * cell -- lights whichever pad sounds it, found in the table: with a scale
 * the pads are not contiguous, so the old note-minus-base arithmetic pointed
 * at the wrong cell as soon as the scale was anything but chromatic. */
void play(uint8_t note, uint8_t vel, uint8_t pad)
{
    play_one(note, vel, pad);
    light_on(pad_of_note(note), vel);
}

void stop(uint8_t note)
{
    auto* p = inst();
    if (!p) return;
    p->noteOff(note);
    light_off(pad_of_note(note));
}

/// The pads play the instrument and nothing else; the kits are not loaded.
class PitchPlayPadLogic : public crosspad::IPadLogicHandler {
public:
    void onActivate(crosspad::PadManager&) override {}
    void onDeactivate(crosspad::PadManager&) override {}
    void onPadPress(crosspad::PadManager&, uint8_t pad, uint8_t vel) override
    {
        if (pad >= kPads) return;
        /* The chord is kept as it was struck, so a release still stops the
         * right notes after OCT, the key or the chord itself has moved. */
        uint8_t notes[kChordMax] = {};
        const int n = chord_of(pad, notes);
        s_padHeld[pad] = pack_chord(notes, n);
        for (int i = 0; i < n; ++i) play_one(notes[i], vel, pad);
        light_on(pad, vel);
    }
    void onPadRelease(crosspad::PadManager& pm, uint8_t pad) override
    {
        if (pad >= kPads) return;
        auto* p = inst();
        uint8_t notes[kChordMax];
        const int n = unpack_chord(s_padHeld[pad], notes);
        s_padHeld[pad] = 0;
        for (int i = 0; i < n; ++i) if (p) p->noteOff(notes[i]);
        light_off(pad);
        pm.notifyNoteOff(pad, true);
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

void show_scale_line();
void show_key_strip();     // the chromatic strip in the SCALE window

void show_pads()
{
    char name[8];
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        lv_obj_t* c = s_cells[pad];
        if (!c) continue;
        const int note = pad_note(pad);
        if (pads_are_chords()) pad_chord_name(pad, name, sizeof(name));
        else                   note_name(note, name, sizeof(name));
        lv_label_set_text(lv_obj_get_child(c, 0), name);
        const int z = s_haveSpec ? zone_for(note) : -1;
        lv_obj_set_style_border_color(c, z >= 0 ? lv_color_hex(kZoneRgb[z % kZoneColors]) : kPanel, 0);
    }
    if (s_range) {
        char a[8], b[8], buf[32];
        note_name(pad_note(0), a, sizeof(a));
        note_name(pad_note(kPads - 1), b, sizeof(b));
        std::snprintf(buf, sizeof(buf), "%s - %s", a, b);
        lv_label_set_text(s_range, buf);
    }
    show_scale_line();
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
    if (!s_nameBtn) return;
    std::snprintf(s_ddText, sizeof(s_ddText), "%s%s", s_haveSpec ? s_spec.name.c_str() : "Pick a sample",
                  s_dirty ? " *" : "");
    lv_label_set_text(lv_obj_get_child(s_nameBtn, 0), s_ddText);
}

/// What this screen has to say that the panel does not.
///
/// A toast, because the design gives Pitch Play no status row: the right
/// panel is the state -- the note, the picture, the envelope, the range --
/// and everything else here is something that just happened, which is what a
/// toast is for. Only when it changes, or a line would be raised again every
/// tick.
void show_info(const char* text = nullptr)
{
    if (!text || !*text) return;
    static char last[128] = {};
    if (std::strncmp(last, text, sizeof(last)) == 0) return;
    std::snprintf(last, sizeof(last), "%s", text);
    ui_toast_show(text, 2500, UI_TOAST_INFO);
}

// ── the right panel (UX 2000 5.9) ─────────────────────────────────────────

/// "G3 - vel 96": the note last sounded, however it was played.
void show_note()
{
    if (!s_noteLbl) return;
    const uint8_t n = s_lastNote.load(), v = s_lastVel.load();
    if (!v) { lv_label_set_text(s_noteLbl, "--"); return; }
    char name[8], buf[24];
    note_name(n, name, sizeof(name));
    std::snprintf(buf, sizeof(buf), "%s  -  vel %u", name, (unsigned)v);
    lv_label_set_text(s_noteLbl, buf);
}

/// The envelope, from the spec. The design's third field is GLIDE; the
/// pitched engine has no glide, so it is not drawn rather than drawn as a
/// zero that cannot be changed.
void show_env()
{
    if (!s_envLbl) return;
    char buf[40];
    std::snprintf(buf, sizeof(buf), "ATK %u ms    REL %u ms",
                  (unsigned)s_spec.instrument.attackMs, (unsigned)s_spec.instrument.releaseMs);
    lv_label_set_text(s_envLbl, buf);
}

/// The context row's plain fields: what the pads are set to.
void show_fields()
{
    char buf[24];
    if (s_octField) {
        std::snprintf(buf, sizeof(buf), "OCT %d", s_oct + 3);   // MusicTheory's base is (3+oct)*12
        lv_label_set_text(s_octField, buf);
    }
    if (s_chordField) {
        const char* cn = s_mt.chordName((uint8_t)s_chordIdx);
        std::snprintf(buf, sizeof(buf), "%.10s", (pads_are_chords() && cn) ? cn : "CHORD off");
        lv_label_set_text(s_chordField, buf);
    }
}

/// The selected zone's file, or "" when there is none to draw.
std::string wave_file()
{
    const auto& zs = s_spec.instrument.zones;
    if (!s_haveSpec || zs.empty()) return {};
    const int i = (s_zoneSel >= 0 && s_zoneSel < (int)zs.size()) ? s_zoneSel : 0;
    return zs[(size_t)i].sample;
}

/// Ask the tools for the columns of the zone now selected.
///
/// One job at a time is all the tools run, and the folder listings this app
/// makes are the ones a player is waiting on, so a picture is only ever asked
/// for when nothing else is out -- and asked for once per file.
void request_wave()
{
    auto* st = tools();
    if (!st || s_wavePending || s_folderPending || s_listing != Listing::None) return;
    const std::string f = wave_file();
    if (f.empty() || f == s_wave.file) return;
    if (!st->requestOverview(f, 0, 0, 150)) return;
    s_wavePending = true;
    s_waveWant = f;
}

/// The zone, drawn as the min/max columns the tools read, in the colour of
/// the zone it is -- the same colour the pads it plays are underlined in.
void wave_draw_cb(lv_event_t* e)
{
    if (s_wave.hi.empty() || s_wave.hi.size() != s_wave.lo.size()) return;
    lv_obj_t* box = lv_event_get_target_obj(e);
    lv_layer_t* layer = lv_event_get_layer(e);
    lv_area_t a;
    lv_obj_get_coords(box, &a);
    const int32_t w = a.x2 - a.x1 + 1, h = a.y2 - a.y1 + 1;
    if (w <= 2 || h <= 4) return;
    const int32_t mid = a.y1 + h / 2, half = h / 2 - 1;

    lv_draw_fill_dsc_t d;
    lv_draw_fill_dsc_init(&d);
    const int zi = (s_zoneSel >= 0) ? s_zoneSel : 0;
    d.color = lv_color_hex(kZoneRgb[(size_t)zi % kZoneColors]);
    d.opa = LV_OPA_COVER;
    const size_t cols = s_wave.hi.size();
    for (int32_t x = 0; x < w; ++x) {
        const size_t i = (size_t)((uint64_t)x * cols / (uint64_t)w);
        if (i >= cols) break;
        int32_t hi = (int32_t)s_wave.hi[i] * half / 127;
        int32_t lo = (int32_t)s_wave.lo[i] * half / 127;
        if (hi < lo) { const int32_t t = hi; hi = lo; lo = t; }
        if (hi - lo < 1) { hi = 1; lo = -1; }
        const lv_area_t r = {a.x1 + x, mid - hi, a.x1 + x, mid - lo};
        lv_draw_fill(layer, &d, &r);
    }
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

void open_sample_window();

/// What the sample window offers, in its order: the saved instruments, the
/// Recorder's takes, and the card for any WAV.
void fill_dropdown()
{
    s_entries.clear();
    s_labels.clear();
    auto* h = host();
    if (!h) return;
    for (const auto& n : s_library) {
        s_entries.push_back({EntryKind::Instrument,
                             h->libraryRoot() + "/" + n + "/" + crosspad::IInstrumentHost::kFileName});
        s_labels.push_back(std::string(LV_SYMBOL_KEYBOARD " ") + n);
    }
    for (const auto& t : s_takes) {
        s_entries.push_back({EntryKind::Sample, t});
        s_labels.push_back(std::string(LV_SYMBOL_AUDIO " ") + base_of(t));
    }
    s_entries.push_back({EntryKind::Card, ""});
    s_labels.push_back(LV_SYMBOL_DIRECTORY " The card ...");
    show_name();
    /* A listing that came in while the window is open fills it where it
     * stands, rather than leaving the player looking at an empty list. */
    if (s_ov == Overlay::Sample) { s_fillPending = false; open_sample_window(); }
}

void open_browser(bool forZone);
void close_overlay();

/// A row of the sample window. The entry it stands for is its index, or the
/// "more" row, which turns the page instead of picking anything.
void sample_pick_cb(lv_event_t* e)
{
    const size_t i = (size_t)(intptr_t)lv_event_get_user_data(e);
    if (i == SIZE_MAX - 1) { s_samplePage += kPageRows; open_sample_window(); return; }
    if (i >= s_entries.size()) return;
    const Entry entry = s_entries[i];
    close_overlay();
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
    for (auto& v : s_scaleVal) v = nullptr;   // children of the overlay, gone with it
    for (auto& c : s_progChip) c = nullptr;
    for (auto& v : s_progVal) v = nullptr;
    s_ov = Overlay::None;
    s_loopLabel = s_loopFill = s_loopBtn = nullptr;
    for (auto& c : s_keyCell) c = nullptr;

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
void show_loop();          // the loop row, defined with the rest of EDIT

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
    show_loop();
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

/* ── the selected zone's loop ──────────────────────────────────────────────
 *
 * The spec stores loop points as frames, which say nothing on their own, so
 * the window asks the host how long the zone's PCM is and at what rate
 * (IInstrumentHost::zoneAudio) and shows seconds. A host that cannot say --
 * nothing loaded yet, or the PC simulator -- reports zero frames, and then
 * the row says so instead of inventing a length.
 *
 * loopEnd == 0 is the one-shot the loader already understands, so OFF is not
 * a separate flag to keep in step: it is end = 0. */
crosspad::IInstrumentHost::ZoneAudio zone_audio()
{
    auto* h = host();
    if (!h || s_zoneSel < 0) return {};
    return h->zoneAudio((uint8_t)s_zoneSel);
}

void show_loop()
{
    const auto& zones = s_spec.instrument.zones;
    const bool have = s_zoneSel >= 0 && s_zoneSel < (int)zones.size();
    const auto za = zone_audio();
    if (s_loopLabel) {
        char buf[48];
        if (!have) {
            std::snprintf(buf, sizeof(buf), "--");
        } else if (!zones[s_zoneSel].loopEnd) {
            std::snprintf(buf, sizeof(buf), "one-shot");
        } else if (!za.frames || !za.sampleRate) {
            /* The points are real and will be used; only their length is not
             * known yet, so show the frames rather than a wrong time. */
            std::snprintf(buf, sizeof(buf), "%u-%u fr", (unsigned)zones[s_zoneSel].loopStart,
                          (unsigned)zones[s_zoneSel].loopEnd);
        } else {
            const float sr = (float)za.sampleRate;
            std::snprintf(buf, sizeof(buf), "%.2f - %.2f s",
                          (double)(zones[s_zoneSel].loopStart / sr),
                          (double)(zones[s_zoneSel].loopEnd / sr));
        }
        lv_label_set_text(s_loopLabel, buf);
    }
    if (s_loopBtn) {
        const bool on = have && zones[s_zoneSel].loopEnd != 0;
        lv_label_set_text(lv_obj_get_child(s_loopBtn, 0), on ? "ON" : "OFF");
        lv_obj_set_style_bg_color(s_loopBtn, on ? kAccent : kPanel, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(s_loopBtn, 0), on ? kInk : kText, 0);
    }
    if (s_loopFill) {
        /* The region over the whole sample. Nothing to scale against means
         * nothing to draw, which is honest. */
        const bool drawable = have && zones[s_zoneSel].loopEnd && za.frames;
        lv_obj_set_style_opa(s_loopFill, drawable ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
        if (drawable) {
            const uint32_t f = za.frames;
            int x = (int)((uint64_t)zones[s_zoneSel].loopStart * 100 / f);
            int w = (int)((uint64_t)(zones[s_zoneSel].loopEnd - zones[s_zoneSel].loopStart) * 100 / f);
            if (w < 1) w = 1;
            if (x > 99) x = 99;
            if (x + w > 100) w = 100 - x;
            lv_obj_set_x(s_loopFill, LV_PCT(x));
            lv_obj_set_width(s_loopFill, LV_PCT(w));
        }
    }
}

/// Two hundred steps across the sample, so one press is a visible move on any
/// length; a zone whose length is unknown steps by a flat 1000 frames.
uint32_t loop_step()
{
    const auto za = zone_audio();
    return za.frames ? std::max<uint32_t>(1, za.frames / 200) : 1000u;
}

/// @p which 0 = start, 1 = end. The two never cross, and the end is what
/// switches a one-shot into a loop.
void loop_by(int which, int dir)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const auto za = zone_audio();
    const uint32_t len = za.frames;                 // 0 = unknown, so unclamped
    const int64_t step = (int64_t)loop_step() * dir;
    if (which == 0) {
        int64_t v = (int64_t)z.loopStart + step;
        const int64_t hi = z.loopEnd ? (int64_t)z.loopEnd - 1 : (len ? (int64_t)len - 1 : v);
        z.loopStart = (uint32_t)std::max<int64_t>(0, std::min(hi, v));
    } else {
        /* From a one-shot, the first + opens a loop over the whole sample
         * rather than a zero-length one at the start, which would be silent. */
        int64_t cur = z.loopEnd ? (int64_t)z.loopEnd : (len ? (int64_t)len : 0);
        if (!z.loopEnd && dir > 0) { z.loopEnd = (uint32_t)cur; s_dirty = true; show_loop(); show_zones(); return; }
        int64_t v = cur + step;
        const int64_t lo = (int64_t)z.loopStart + 1;
        const int64_t hi = len ? (int64_t)len : v;
        v = std::max(lo, std::min(hi, v));
        z.loopEnd = (uint32_t)v;
    }
    s_dirty = true;
    show_loop();
    show_zones();
}

void loop_toggle_cb(lv_event_t*)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    if (z.loopEnd) {
        z.loopEnd = 0;                       // one-shot; the start is kept
    } else {
        const auto za = zone_audio();
        z.loopStart = 0;
        z.loopEnd = za.frames ? za.frames : 0;
        if (!z.loopEnd) { show_loop(); return; }   // nothing to loop over yet
    }
    s_dirty = true;
    show_loop();
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

    /* LOOP, under ROOT: the start and the end of the selected zone's loop,
     * ON/OFF being end = 0 (the one-shot the loader already knows). The bar
     * under the row is the region over the whole sample -- no waveform, on
     * purpose: a decoded picture here is the sort of PSRAM a pitched bank
     * needs, and the numbers plus a region answer "where am I" already. */
    lv_obj_t* lrow = make_row(s_overlay, kRowH);
    lv_obj_t* ll = lv_label_create(lrow);
    lv_obj_set_style_text_color(ll, kDim, 0);
    lv_label_set_text(ll, "LOOP");
    s_loopBtn = make_button(lrow, 42, "OFF", loop_toggle_cb, s_overlayGroup);
    make_button(lrow, 28, "S-", [](lv_event_t*) { loop_by(0, -1); }, s_overlayGroup);
    make_button(lrow, 28, "S+", [](lv_event_t*) { loop_by(0, 1); }, s_overlayGroup);
    s_loopLabel = lv_label_create(lrow);
    lv_obj_set_flex_grow(s_loopLabel, 1);
    lv_obj_set_style_text_align(s_loopLabel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(s_loopLabel, kAccent, 0);
    lv_label_set_text(s_loopLabel, "--");
    make_button(lrow, 28, "E-", [](lv_event_t*) { loop_by(1, -1); }, s_overlayGroup);
    make_button(lrow, 28, "E+", [](lv_event_t*) { loop_by(1, 1); }, s_overlayGroup);

    lv_obj_t* bar = lv_obj_create(s_overlay);
    lv_obj_remove_style_all(bar);
    lv_obj_set_size(bar, LV_PCT(100), 6);
    lv_obj_set_style_radius(bar, 3, 0);
    lv_obj_set_style_bg_color(bar, kPanel, 0);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(bar, LV_OBJ_FLAG_SCROLLABLE);
    s_loopFill = lv_obj_create(bar);
    lv_obj_remove_style_all(s_loopFill);
    lv_obj_set_height(s_loopFill, LV_PCT(100));
    lv_obj_set_width(s_loopFill, LV_PCT(0));
    lv_obj_set_style_radius(s_loopFill, 3, 0);
    lv_obj_set_style_bg_color(s_loopFill, kAccent, 0);
    lv_obj_set_style_bg_opa(s_loopFill, LV_OPA_TRANSP, 0);

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

// ── SAMPLE > : the one key on the row that leads somewhere ────────────────

/* The saved instruments, the Recorder's takes and the card -- what the
 * drop-down in the old top row offered -- and with them the two things that
 * make an instrument rather than pick one: EDIT (its zones) and SAVE. The
 * design's key row has five keys and neither EDIT nor SAVE is one of them, so
 * this is where they belong: they are about the sample, and this is the
 * sample window. */
void open_sample_window()
{
    open_overlay(Overlay::Sample);
    overlay_header("Sample");
    lv_obj_t* col = scroll_column(s_overlay);
    if (s_entries.empty()) {
        lv_obj_t* l = lv_label_create(col);
        lv_obj_set_style_text_color(l, kDim, 0);
        lv_label_set_text(l, "Reading the card ...");
    }
    const size_t total = std::min(s_entries.size(), s_labels.size());
    if (s_samplePage >= total) s_samplePage = 0;
    const size_t end = std::min<size_t>(total, (size_t)s_samplePage + kPageRows);
    for (size_t i = s_samplePage; i < end; ++i) {
        list_item(col, s_labels[i].c_str(), sample_pick_cb, (void*)(intptr_t)i);
    }
    if (end < total) {
        char more[48];
        std::snprintf(more, sizeof(more), LV_SYMBOL_DOWN "  %u more ...", (unsigned)(total - end));
        list_item(col, more, sample_pick_cb, (void*)(intptr_t)(SIZE_MAX - 1));
    }
    list_item(col, LV_SYMBOL_SETTINGS "  Edit the zones ...",
              [](lv_event_t* e) { close_overlay(); edit_cb(e); }, nullptr);
    list_item(col, LV_SYMBOL_SAVE "  Save as ...",
              [](lv_event_t* e) { close_overlay(); save_cb(e); }, nullptr);
}

void sample_cb(lv_event_t*)
{
    if (s_ov != Overlay::None) return;
    s_samplePage = 0;          // the window opens at the top, every time
    open_sample_window();
}

/// CHORD: the next of what a pad makes. The field in the context row says
/// which; SCALE is where the rest of the mapping is set.
void step_chord(int d);
void chord_cb(lv_event_t*) { step_chord(+1); }

// ── the mapping ────────────────────────────────────────────────────────────

/* No back_cb: this screen has no back button. Back is the power button's
 * click, which goes up one level and then out, and apps are switched with
 * MODE and a pad (UX 2000 section 2). */

/* Sixteen degrees of a scale span more than sixteen semitones, so the top
 * pad is what has to stay a note, so the setting is tried on the real map
 * and rolled back when the top of it has been clamped away. */
bool set_mapping(int oct, int key, int scale, int transpose, int chordIdx)
{
    const int wasO = s_oct, wasK = s_key, wasS = s_scale, wasT = s_transpose, wasC = s_chordIdx;
    if (oct < -2 || oct > 4 || transpose < -12 || transpose > 12) return false;
    s_oct = oct; s_key = key; s_scale = scale; s_transpose = transpose; s_chordIdx = chordIdx;
    rebuild_pads();
    /* The map is clamped to 0..127, so a setting that runs off the top shows
     * as the last pads landing on the same note. Refuse it and go back. */
    if (s_ps.padMidiMap[kPads - 1] == s_ps.padMidiMap[kPads - 2]) {
        s_oct = wasO; s_key = wasK; s_scale = wasS; s_transpose = wasT; s_chordIdx = wasC;
        rebuild_pads();
        return false;
    }
    show_pads();          // which calls show_scale_line()
    show_leds();
    return true;
}

void oct_cb(lv_event_t* e)
{
    const int step = (int)(intptr_t)lv_event_get_user_data(e);
    set_mapping(s_oct + step, s_key, s_scale, s_transpose, s_chordIdx);
}

void volume_cb(lv_event_t* e)
{
    const int step = number_step(e);
    if (!step) return;
    s_volume = (uint8_t)std::max(0, std::min(100, (int)s_volume + step * kVolStep));
    apply_volume();
    show_volume();
}

/* The key, the scale, the transpose and the chord: four of the same row, a
 * name on the left and -/value/+ on the right. A window rather than four more
 * controls on the main screen -- the bottom row is already OCT, the range and
 * VOL, and these are set once and then played, not reached for mid-phrase.
 * The top-row button carries the answer at a glance ("C maj"). */

void show_fields();

void show_scale_line()
{
    show_fields();
    if (s_scaleBtn) {
        char buf[16];
        /* Short enough for a 56 px button: the key, three letters of the
         * scale, and a dot when a chord is on so the pads' sound is never a
         * surprise. */
        const char* sc = crosspad::MusicTheory::scaleName((uint8_t)s_scale);
        std::snprintf(buf, sizeof(buf), "%s %.3s%s", key_name(s_key), sc ? sc : "?",
                      pads_are_chords() ? "." : "");
        lv_label_set_text(lv_obj_get_child(s_scaleBtn, 0), buf);
    }
    if (!s_scaleVal[0]) return;
    char buf[16];
    lv_label_set_text(s_scaleVal[0], key_name(s_key));
    const char* sc = crosspad::MusicTheory::scaleName((uint8_t)s_scale);
    lv_label_set_text(s_scaleVal[1], sc ? sc : "?");
    std::snprintf(buf, sizeof(buf), "%+d", s_transpose);
    lv_label_set_text(s_scaleVal[2], buf);
    lv_label_set_text(s_scaleVal[3], s_mt.chordName((uint8_t)s_chordIdx));
    show_key_strip();        // the picture and the names are one thing
}

/// The next (or previous) of what a pad can make: one note, the chord types,
/// the two modes, then the progressions. Its own function because the CHORD
/// key on the soft row steps it as well as the SCALE window's row.
void step_chord(int d)
{
    const int chords = (int)s_mt.totalChordCount();
    if (chords <= 0) return;
    /* A progression is degrees, so it is written in a mode: picking one
     * brings its scale with it, or "Backdoor" would have no flat seventh to
     * walk down and "Lydian Dream" no major II. The twelve chord types and
     * the two modes leave the scale alone, and so do the player's own
     * progressions -- nothing knows what key those were meant in. */
    const int next = (s_chordIdx + chords + d) % chords;
    const int first = crosspad::MusicTheory::CHORD_PROG_START_IDX;
    const int builtins = (int)crosspad::MusicTheory::builtinProgCount();
    int scale = s_scale;
    if (next >= first && next < first + builtins) {
        scale = (int)crosspad::MusicTheory::builtinProgScale((uint8_t)(next - first));
    }
    set_mapping(s_oct, s_key, scale, s_transpose, next);
}

/// -1 or +1 on one of the four, from the button's user data: (which << 1) | up.
void scale_step_cb(lv_event_t* e)
{
    const int code = (int)(intptr_t)lv_event_get_user_data(e);
    const int which = code >> 1;
    const int d = (code & 1) ? 1 : -1;
    const int scales = (int)crosspad::MusicTheory::scaleCount();
    switch (which) {
        case 0: set_mapping(s_oct, (s_key + 12 + d) % 12, s_scale, s_transpose, s_chordIdx); break;
        case 1: set_mapping(s_oct, s_key, (s_scale + scales + d) % scales, s_transpose, s_chordIdx); break;
        case 2: set_mapping(s_oct, s_key, s_scale, s_transpose + d, s_chordIdx); break;
        default: step_chord(d); break;
    }
}

lv_obj_t* scale_row(const char* name, int which)
{
    lv_obj_t* r = make_row(s_overlay, kRowH);
    lv_obj_t* l = lv_label_create(r);
    lv_obj_set_flex_grow(l, 1);
    lv_label_set_text(l, name);
    make_button(r, 30, LV_SYMBOL_MINUS, scale_step_cb, s_overlayGroup,
                (void*)(intptr_t)(which << 1));
    lv_obj_t* v = lv_label_create(r);
    lv_obj_set_width(v, 86);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(v, kAccent, 0);
    lv_label_set_text(v, "");
    s_scaleVal[which] = v;
    make_button(r, 30, LV_SYMBOL_PLUS, scale_step_cb, s_overlayGroup,
                (void*)(intptr_t)((which << 1) | 1));
    return r;
}

/* The editor. One row of chips -- the steps -- then the two things a step
 * is: its degree of the scale and the chord built on it. Eight steps fit the
 * row; MusicTheory would take sixteen. */

void prog_show()
{
    for (int i = 0; i < kMaxSteps; ++i) {
        if (!s_progChip[i]) continue;
        const bool used = i < s_edit.count;
        lv_obj_set_style_bg_color(s_progChip[i], i == s_editSel ? kAccent : kPanel, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(s_progChip[i], 0),
                                    i == s_editSel ? kInk : (used ? kText : kDim), 0);
        char b[8];
        if (used) {
            const char qc = crosspad::MusicTheory::chordQualChar(s_edit.steps[i].chordType);
            std::snprintf(b, sizeof(b), "%d%c", s_edit.steps[i].degree + 1, qc ? qc : ' ');
        } else {
            std::snprintf(b, sizeof(b), "-");
        }
        lv_label_set_text(lv_obj_get_child(s_progChip[i], 0), b);
    }
    if (!s_progVal[0]) return;
    char b[16];
    std::snprintf(b, sizeof(b), "%d", s_edit.steps[s_editSel].degree + 1);
    lv_label_set_text(s_progVal[0], b);
    lv_label_set_text(s_progVal[1], s_mt.chordShortName(s_edit.steps[s_editSel].chordType));
}

void prog_chip_cb(lv_event_t* e)
{
    const int i = (int)(intptr_t)lv_event_get_user_data(e);
    if (i < s_edit.count) { s_editSel = i; prog_show(); }
}

/// (which << 1) | up, as in the scale window: 0 the degree, 1 the chord.
void prog_step_cb(lv_event_t* e)
{
    const int code = (int)(intptr_t)lv_event_get_user_data(e);
    const int d = (code & 1) ? 1 : -1;
    auto& st = s_edit.steps[s_editSel];
    if ((code >> 1) == 0) {
        /* Degrees of the scale that is current: a seven-note scale has
         * seven, a pentatonic five. */
        const int n = 7;   // MusicTheory wraps a degree into the scale itself
        st.degree = (uint8_t)((st.degree + n + d) % n);
    } else {
        const int n = crosspad::MusicTheory::playableChordCount();
        st.chordType = (uint8_t)((st.chordType + n + d) % n);
    }
    prog_show();
}

void prog_add_cb(lv_event_t*)
{
    if (s_edit.count >= kMaxSteps) return;
    s_edit.steps[s_edit.count] = s_edit.steps[s_edit.count ? s_edit.count - 1 : 0];
    s_editSel = s_edit.count;
    ++s_edit.count;
    prog_show();
}

void prog_del_cb(lv_event_t*)
{
    if (s_edit.count <= 1) return;
    --s_edit.count;
    if (s_editSel >= s_edit.count) s_editSel = s_edit.count - 1;
    prog_show();
}

void prog_save_cb(lv_event_t*)
{
    const int slot = s_editSlot >= 0 && s_editSlot < kMaxCustom ? s_editSlot : s_customN;
    if (slot >= kMaxCustom) { show_info("No room for another progression."); return; }
    /* (slot & 7) rather than slot + 1: kMaxCustom is 8, and masking is what
     * lets the compiler see the number is one digit. */
    std::snprintf(s_edit.name, sizeof(s_edit.name), "Mine %d", (slot & 7) + 1);
    s_custom[slot] = s_edit;
    if (slot == s_customN) ++s_customN;
    register_customs();
    save_customs();
    close_overlay();
    /* Play it at once -- that is what the player came here for. */
    set_mapping(s_oct, s_key, s_scale, s_transpose, custom_chord_index(slot));
}

lv_obj_t* prog_value_row(const char* name, int which)
{
    lv_obj_t* r = make_row(s_overlay, kRowH);
    lv_obj_t* l = lv_label_create(r);
    lv_obj_set_flex_grow(l, 1);
    lv_label_set_text(l, name);
    make_button(r, 30, LV_SYMBOL_MINUS, prog_step_cb, s_overlayGroup, (void*)(intptr_t)(which << 1));
    lv_obj_t* v = lv_label_create(r);
    lv_obj_set_width(v, 70);
    lv_obj_set_style_text_align(v, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(v, kAccent, 0);
    lv_label_set_text(v, "");
    s_progVal[which] = v;
    make_button(r, 30, LV_SYMBOL_PLUS, prog_step_cb, s_overlayGroup, (void*)(intptr_t)((which << 1) | 1));
    return r;
}

/// @p slot < 0 starts a new one, otherwise edits that stored progression.
void prog_open(int slot)
{
    s_editSlot = slot;
    if (slot >= 0 && slot < s_customN) {
        s_edit = s_custom[slot];
    } else {
        /* I - V - vi - IV to start from: the one everybody already knows. */
        s_edit.count = 4;
        s_edit.steps[0] = {0, 1};
        s_edit.steps[1] = {4, 1};
        s_edit.steps[2] = {5, 2};
        s_edit.steps[3] = {3, 1};
    }
    s_editSel = 0;

    open_overlay(Overlay::Prog);
    overlay_header(slot >= 0 ? "Edit progression" : "New progression");

    lv_obj_t* chips = make_row(s_overlay, kRowH);
    lv_obj_set_style_pad_column(chips, 3, 0);
    for (int i = 0; i < kMaxSteps; ++i) {
        lv_obj_t* b = make_button(chips, 0, "-", prog_chip_cb, s_overlayGroup, (void*)(intptr_t)i);
        lv_obj_set_flex_grow(b, 1);
        s_progChip[i] = b;
    }

    prog_value_row("Degree", 0);
    prog_value_row("Chord", 1);

    lv_obj_t* tools = make_row(s_overlay, kRowH);
    make_button(tools, 56, "+ STEP", prog_add_cb, s_overlayGroup);
    make_button(tools, 56, "- STEP", prog_del_cb, s_overlayGroup);
    lv_obj_t* gap = lv_obj_create(tools);
    lv_obj_remove_style_all(gap);
    lv_obj_set_flex_grow(gap, 1);
    make_button(tools, 56, "SAVE", prog_save_cb, s_overlayGroup);

    lv_obj_t* note = lv_label_create(s_overlay);
    lv_obj_set_width(note, LV_PCT(100));
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(note, kDim, 0);
    lv_obj_set_style_text_font(note, &lv_font_montserrat_10, 0);
    lv_label_set_text(note,
        "Steps are degrees of the scale, so the progression follows the key "
        "and the mode. The pads play it over and over, an octave higher each "
        "time round.");
    prog_show();
}

void prog_new_cb(lv_event_t*)
{
    /* Editing the one that is playing, if it is one of ours; otherwise a new
     * one from I - V - vi - IV. */
    const int first = custom_chord_index(0);
    const int slot = (s_chordIdx >= first && s_chordIdx < first + s_customN)
                   ? s_chordIdx - first : -1;
    prog_open(slot);
}

/* ── the scale, as a shape ─────────────────────────────────────────────────
 *
 * Four rows of names tell you the key and the scale; they do not tell you
 * what you are about to play. One chromatic octave does: twelve cells laid
 * out like a keyboard, the ones in the scale lit, the key ringed, and a dot
 * on every cell some pad actually sounds.
 *
 * Pitch classes, not notes, so it reads the same whatever octave or
 * transpose is set -- and the dots come from pad_note(), the same function
 * the pads are labelled from, so the picture cannot drift from the sound.
 * No scale table is read here: whether a pitch class is in the scale is
 * answered by asking whether any pad plays it, which is what the player
 * cares about and what rebuildPadMap() has already decided. */
constexpr bool kBlackKey[12] = {false, true, false, true, false,
                                false, true, false, true, false, true, false};

void show_key_strip()
{
    if (!s_keyCell[0]) return;
    uint16_t played = 0;                       // pitch classes some pad sounds
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        const int n = pad_note(pad);
        if (n >= 0) played = (uint16_t)(played | (1u << (n % 12)));
    }
    for (int pc = 0; pc < 12; ++pc) {
        lv_obj_t* c = s_keyCell[pc];
        const bool in   = (played & (1u << pc)) != 0;
        const bool root = (pc == s_key);
        lv_obj_set_style_bg_color(c, in ? kAccent : (kBlackKey[pc] ? kInk : kPanel), 0);
        lv_obj_set_style_border_width(c, root ? 2 : 0, 0);
        lv_obj_set_style_border_color(c, kText, 0);
        lv_obj_t* l = lv_obj_get_child(c, 0);
        if (l) {
            lv_obj_set_style_text_color(l, in ? kInk : kDim, 0);
            /* The name only on the key and on the black notes' neighbours
             * would be guesswork; every cell carries it, two characters at
             * most, and the lit ones read first because of the contrast. */
            lv_label_set_text(l, pitch_name(pc, s_key));
        }
    }
}

void build_key_strip()
{
    lv_obj_t* row = make_row(s_overlay, 30);
    lv_obj_set_style_pad_column(row, 2, 0);
    for (int pc = 0; pc < 12; ++pc) {
        lv_obj_t* c = lv_obj_create(row);
        lv_obj_remove_style_all(c);
        lv_obj_set_flex_grow(c, 1);
        lv_obj_set_height(c, LV_PCT(100));
        lv_obj_set_style_radius(c, 3, 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* l = lv_label_create(c);
        lv_obj_center(l);
        lv_label_set_text(l, "");
        s_keyCell[pc] = c;
    }
    show_key_strip();
}

void scale_cb(lv_event_t*)
{
    open_overlay(Overlay::Scale);
    overlay_header("Key, scale and chords");
    build_key_strip();
    scale_row("Key", 0);
    scale_row("Scale", 1);
    scale_row("Transpose", 2);
    scale_row("Chord", 3);
    /* The way to a progression of one's own; it lands at the end of the Chord
     * list and starts playing straight away. */
    lv_obj_t* mine = make_row(s_overlay, kRowH);
    lv_obj_t* ml = lv_label_create(mine);
    lv_obj_set_flex_grow(ml, 1);
    lv_label_set_text(ml, "Own progression");
    make_button(mine, 86, "EDIT ...", prog_new_cb, s_overlayGroup);
    lv_obj_t* note = lv_label_create(s_overlay);
    lv_obj_set_width(note, LV_PCT(100));
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(note, kDim, 0);
    lv_obj_set_style_text_font(note, &lv_font_montserrat_10, 0);
    lv_label_set_text(note,
        "The pads are sixteen degrees of the scale from the key, shifted by "
        "the transpose. A chord is built in the scale, so it stays in key.");
    show_scale_line();
}

/// A touch on a pad's cell plays it, as the pad would.
void cell_cb(lv_event_t* e)
{
    const uint8_t pad = (uint8_t)(intptr_t)lv_event_get_user_data(e);
    if (pad >= kPads) return;
    if (lv_event_get_code(e) == LV_EVENT_PRESSED) {
        uint8_t notes[kChordMax] = {};
        const int n = chord_of(pad, notes);
        s_padHeld[pad] = pack_chord(notes, n);
        for (int i = 0; i < n; ++i) play_one(notes[i], 100, pad);
        light_on(pad, 100);
    } else {
        auto* p = inst();
        uint8_t notes[kChordMax];
        const int n = unpack_chord(s_padHeld[pad], notes);
        s_padHeld[pad] = 0;
        for (int i = 0; i < n; ++i) if (p) p->noteOff(notes[i]);
        light_off(pad);
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
            /* Put the sample's own note under the pads rather than three
             * octaves below them: its pitch class becomes the key and its
             * octave the one the grid starts in. */
            const int root = (int)std::lround(s_spec.instrument.zones[0].root);
            const int oct  = std::max(-2, std::min(4, root / 12 - 3));
            set_mapping(oct, ((root % 12) + 12) % 12, s_scale, s_transpose, s_chordIdx);
        }
        s_followRoot = false;
        s_playable.store(state == crosspad::IInstrumentHost::State::Ready);
        /* The instrument, for whoever wants to borrow the same one. The host's
         * own copy is the authority on the path -- s_spec came from it. */
        if (state == crosspad::IInstrumentHost::State::Ready) {
            auto& shared = crosspad::instrumentPlayState();
            shared.instrumentPath = s_spec.path;
            shared.instrumentName = s_spec.name;
            crosspad::instrumentPlayStatePublished();
        }
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
    if (s_wavePending) {
        s_wavePending = false;
        if (job == crosspad::ToolJob::Ready) {
            s_wave = st->overview();
            /* The tools answer whatever was asked last; only our own file is
             * kept, so a listing that overtook this request cannot leave the
             * box drawing somebody else's sample. */
            if (s_wave.file != s_waveWant) s_wave = crosspad::WaveOverview{};
        } else {
            s_wave = crosspad::WaveOverview{};
        }
        if (s_waveBox) lv_obj_invalidate(s_waveBox);
        return;
    }
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
    request_wave();
    show_note();
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
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_set_style_text_color(s_root, kText, 0);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);

    /* Absolute positions in the design's screen coordinates; PP_Y shifts them
     * into the root, which begins under the transport bar. */
    auto place = [](lv_obj_t* o, int32_t x, int32_t y) { if (o) lv_obj_set_pos(o, x, y); return o; };

    /* ── context row: the instrument, then what the pads are set to ──────
     * The instrument is the one thing here that leads somewhere (the sample
     * window); the other three are fields, because the keys at the foot are
     * what changes them and two controls for one value is one too many. */
    s_nameBtn = make_button(s_root, 118, "", sample_cb, g);
    lv_obj_set_height(s_nameBtn, 22);
    place(s_nameBtn, 6, PP_Y(35));
    lv_obj_set_style_text_color(lv_obj_get_child(s_nameBtn, 0), kZone0, 0);
    lv_label_set_long_mode(lv_obj_get_child(s_nameBtn, 0), LV_LABEL_LONG_DOT);
    lv_obj_set_width(lv_obj_get_child(s_nameBtn, 0), 106);

    auto field = [&](int32_t x, int32_t w) {
        lv_obj_t* f = lv_obj_create(s_root);
        lv_obj_remove_style_all(f);
        lv_obj_set_pos(f, x, PP_Y(35));
        lv_obj_set_size(f, w, 22);
        lv_obj_set_style_radius(f, 4, 0);
        lv_obj_set_style_bg_color(f, kPanel, 0);
        lv_obj_set_style_bg_opa(f, LV_OPA_COVER, 0);
        lv_obj_remove_flag(f, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t* l = lv_label_create(f);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(l, kText, 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_CLIP);
        lv_label_set_text(l, "");
        lv_obj_center(l);
        return l;
    };
    s_scaleBtn = make_button(s_root, 56, "", scale_cb, g);
    lv_obj_set_height(s_scaleBtn, 22);
    place(s_scaleBtn, 128, PP_Y(35));
    s_octField   = field(188, 48);
    s_chordField = field(240, 74);

    load_customs();   // and hand them back to MusicTheory
    if (s_oct == 0 && s_key == 0 && s_scale == 0 && s_chordIdx == 0) s_oct = kOctDefault;
    rebuild_pads();

    /* ── the pads, bottom left first ─────────────────────────────────────
     * 150 px of the 320, so the right half can say what the left one is
     * playing: four columns of 36 and four rows of 33, 2 px apart. */
    constexpr int32_t kCellW = 36, kCellH = 33, kCellGap = 2;
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        lv_obj_t* c = lv_obj_create(s_root);
        lv_obj_remove_style_all(c);
        lv_obj_set_size(c, kCellW, kCellH);
        place(c, 6 + (pad % 4) * (kCellW + kCellGap),
              PP_Y(62) + (3 - pad / 4) * (kCellH + kCellGap));
        lv_obj_set_style_bg_color(c, kPanel, 0);
        lv_obj_set_style_bg_opa(c, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(c, 6, 0);
        lv_obj_set_style_border_side(c, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_border_width(c, 3, 0);
        lv_obj_set_style_border_color(c, kPanel, 0);
        lv_obj_set_style_text_color(c, kText, 0);
        lv_obj_set_style_text_font(c, &lv_font_montserrat_14, 0);
        lv_obj_remove_flag(c, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(c, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(c, cell_cb, LV_EVENT_PRESSED, (void*)(intptr_t)pad);
        lv_obj_add_event_cb(c, cell_cb, LV_EVENT_RELEASED, (void*)(intptr_t)pad);
        lv_obj_t* l = lv_label_create(c);
        lv_obj_center(l);
        s_cells[pad] = c;
    }

    /* ── the right panel: what is sounding, and out of what ──────────────*/
    s_noteLbl = lv_label_create(s_root);
    place(s_noteLbl, 162, PP_Y(63));
    lv_obj_set_width(s_noteLbl, 152);
    lv_obj_set_style_text_font(s_noteLbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_noteLbl, kText, 0);
    lv_label_set_long_mode(s_noteLbl, LV_LABEL_LONG_CLIP);
    lv_label_set_text(s_noteLbl, "--");

    s_waveBox = lv_obj_create(s_root);
    lv_obj_remove_style_all(s_waveBox);
    place(s_waveBox, 162, PP_Y(82));
    lv_obj_set_size(s_waveBox, 152, 58);
    lv_obj_set_style_radius(s_waveBox, 4, 0);
    lv_obj_set_style_bg_color(s_waveBox, kWaveBg, 0);
    lv_obj_set_style_bg_opa(s_waveBox, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_waveBox, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_waveBox, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_waveBox, wave_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

    s_envLbl = lv_label_create(s_root);
    place(s_envLbl, 162, PP_Y(144));
    lv_obj_set_width(s_envLbl, 152);
    lv_obj_set_style_text_font(s_envLbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_envLbl, kZone0, 0);
    lv_label_set_long_mode(s_envLbl, LV_LABEL_LONG_CLIP);
    lv_label_set_text(s_envLbl, "");

    s_range = lv_label_create(s_root);
    place(s_range, 162, PP_Y(158));
    lv_obj_set_width(s_range, 152);
    lv_obj_set_style_text_font(s_range, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_range, kDim, 0);
    lv_label_set_long_mode(s_range, LV_LABEL_LONG_CLIP);
    lv_label_set_text(s_range, "");

    /* The level: turned, because it is set by ear while the pads are played
     * and a key would need a second one to come back down. */
    s_volBtn = lv_CreateEditableButton(s_root);
    style_button(s_volBtn, 152, 22);
    place(s_volBtn, 162, PP_Y(174));
    lv_obj_add_event_cb(s_volBtn, volume_cb, LV_EVENT_KEY, nullptr);
    lv_obj_add_event_cb(s_volBtn, volume_cb, LV_EVENT_CLICKED, nullptr);
    add_label(s_volBtn, "");
    if (g) lv_group_add_obj(g, s_volBtn);

    /* ── soft keys: the five of the design ───────────────────────────────*/
    {
        struct { const char* text; lv_event_cb_t cb; void* ud; } keys[5] = {
            { "SCALE",             scale_cb,  nullptr },
            { "OCT " LV_SYMBOL_MINUS, oct_cb, (void*)(intptr_t)-1 },
            { "OCT " LV_SYMBOL_PLUS,  oct_cb, (void*)(intptr_t)1  },
            { "CHORD",             chord_cb,  nullptr },
            { "SAMPLE " LV_SYMBOL_RIGHT, sample_cb, nullptr },
        };
        for (int i = 0; i < 5; ++i) {
            lv_obj_t* b = make_button(s_root, 58, keys[i].text, keys[i].cb, g, keys[i].ud);
            lv_obj_set_height(b, 28);
            place(b, 6 + i * 62, PP_Y(206));
        }
    }

#undef PP_Y

    show_pads();
    show_volume();
    show_name();
    show_scale_line();
    show_env();
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

    /* The instrument first: the first thing a player reaches for here is
     * which sound, and everything else on the screen is set against it. */
    if (g && s_nameBtn) lv_group_focus_obj(s_nameBtn);
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
    s_root = s_range = s_volBtn = s_scaleBtn = nullptr;
    s_nameBtn = s_octField = s_chordField = nullptr;
    s_noteLbl = s_waveBox = s_envLbl = nullptr;
    s_wave = crosspad::WaveOverview{};
    s_wavePending = false;
    s_waveWant.clear();
    for (auto& v : s_scaleVal) v = nullptr;
    for (auto& c : s_cells) c = nullptr;
    lv_obj_delete_async(root);
}

REGISTER_APP_PL(PitchPlay, nullptr, "pitchplay.png",
                PitchPlay_create, PitchPlay_destroy,
                nullptr, nullptr, nullptr, 0, "PitchPlay")
