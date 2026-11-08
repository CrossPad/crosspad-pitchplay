/**
 * @file PitchPlayApp.cpp
 * @brief Pitch Play -- one sample or many, played chromatically on the pads and
 *        from a MIDI keyboard.
 *
 *   context  the instrument (tap: the sample window -- the card, the takes,
 *            the saved instruments, and SAVE), then what the pads are
 *            set to as plain fields -- the key and scale, the octave, what a
 *            pad makes. No back button: back is the power button's click.
 *   pads     the sixteen pads as they sound, bottom left first: each its
 *            note, underlined in the colour of the zone that plays it, lit
 *            while it sounds; a touch plays it too
 *   right    the note last played and how hard, the selected zone's waveform,
 *            its envelope, the pads' range, and the level (turned)
 *   keys     SCALE, OCT-, OCT+, CHORD, EDIT > -- CHORD opens the Chord
 *            Designer, where up to sixteen chords are spelled out and pad n
 *            plays chord n; EDIT goes straight into the zones, their loops
 *            and crossfades. It was SAMPLE >, a second door to the window the
 *            instrument's name already opens, with EDIT at the foot of a list
 *            of every take on the card.
 *
 * A sample picked from the list is an instrument of one zone, its pitch
 * detected. EDIT lists the zones -- each a sample and the note it sounds
 * at -- adds one from the card, removes one, sets a root by hand or back to
 * AUTO, where a note begins, and a loop whose end crossfades into what leads
 * up to its start, so one sample holds for as long as the pad does; leaving
 * EDIT rebuilds the bank. SAVE names the instrument and keeps it
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
#include "crosspad/input/EncoderAccelerator.hpp"
#include "crosspad/instrument/InstrumentPlayState.hpp"
#include "crosspad/event/EventData.hpp"
#include "crosspad/event/IEventBus.hpp"
#include "crosspad/instrument/IInstrumentHost.hpp"
#include "crosspad/instrument/InstrumentGroups.hpp"
#include "crosspad/instrument/PitchedInstrument.hpp"
#include "crosspad/kit/IKitManager.hpp"
#include "crosspad/kit/KitPathUtils.hpp"
#include "crosspad/music/ChordDesigner.hpp"
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
#include "crosspad/song/ArrangeFocus.hpp"
#include "crosspad/song/ISongEngine.hpp"
#include "crosspad/transport/SongTransport.hpp"
#include "crosspad-gui/components/mode_overlay.h"
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
 * The whole screen is laid out in the design's screen coordinates -- drawn
 * under a 30 px bar -- and this is the only place the shift happens. Under the
 * 40 px bar (crosspad-gui 2.4) the root is 200 px, not 210: subtracting the
 * bar's own height put the context row 5 px under it. From 33 the row is 2 px
 * below the bar and the soft keys move up 2 (PP_Y(204)) to end inside the
 * screen, as Arrange's do (kExportTop). */
constexpr int32_t kDesignTop = 33;
#define PP_Y(v) ((v) - kDesignTop)
constexpr uint8_t  kPads     = 16;
constexpr int      kChordMax = crosspad::chords::kMaxNotes;   ///< notes in the widest chord a pad makes

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
enum class Overlay : uint8_t { None, Edit, Browse, Name, Scale, Chord, Sample };
enum class EntryKind : uint8_t { Instrument, Group, Sample, Card };

struct Entry {
    EntryKind   kind;
    std::string path;   ///< logical: an instrument file or a WAV; a group's name
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
PP_SLOW_BSS lv_obj_t* s_scaleVal[3];   ///< the window's three value labels
lv_obj_t* s_cells[kPads] = {};
lv_obj_t* s_overlay = nullptr;
lv_obj_t* s_zoneList = nullptr;
lv_obj_t* s_rootLabel = nullptr;
lv_obj_t* s_loopLabel = nullptr;   ///< EDIT: "S 0.42 s", the loop's start
PP_SLOW_BSS lv_obj_t* s_loopEndLabel;   ///< EDIT: "E 1.85 s"
lv_obj_t* s_loopWave  = nullptr;   ///< EDIT: the zone's picture, its loop and crossfade marked
lv_obj_t* s_loopBtn   = nullptr;   ///< ON/OFF for the selected zone's loop
PP_SLOW_BSS lv_obj_t* s_startLabel;   ///< EDIT: where a note begins
PP_SLOW_BSS lv_obj_t* s_xfLabel;      ///< EDIT: the loop's crossfade
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
/* A zone's root, loop, start and crossfade reach the bank that is playing
 * once the knob has rested kLiveMs (PitchedInstrument::editZone() silences
 * the zone's notes, so not on every detent); zones added or removed wait for
 * the rebuild when EDIT closes -- a removal renumbers them. */
bool     s_liveWanted = false;
uint32_t s_liveAt = 0;
bool     s_zonesMoved = false;
constexpr uint32_t kLiveMs = 150;
bool    s_saveEdits = false;   // the build under way is of edits: saved to the instrument's file once played
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
/* The group of instruments the window shows, "" for the list of groups. Kept
 * when the window closes: the next sound tried is most often of the same kind. */
PP_SLOW_BSS char s_sampleGroup[24];
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
/// EDIT's picture zoomed in round a point being turned FINE: [s_viewA,
/// s_viewB) of the zone, 0..0 for all of it. s_waveAsk* is the range the
/// pending request is for, s_waveHas* the one s_wave holds.
uint32_t    s_viewA = 0, s_viewB = 0;
uint32_t    s_waveAskA = 0, s_waveAskB = 0, s_waveHasA = 0, s_waveHasB = 0;
/// The frame field being turned (EditField), -1 for none, and whether FINE.
int         s_turnField = -1;
int         s_fineField = -1;
/// Each side of the point while FINE, ~0.25 s.
constexpr uint32_t kZoomFrames = 12000;

Overlay s_ov = Overlay::None;
bool    s_browseForZone = false;       // the card's WAV becomes a zone, not an instrument
std::string          s_browseDir;      // "" = the recordings
crosspad::CardFolder s_folder;
bool     s_folderPending = false;
uint32_t s_page = 0;
int      s_zoneSel = 0;

std::atomic<bool>     s_playable{false};
std::atomic<uint32_t> s_lit{0};
std::atomic<uint8_t>  s_padPicked{0};   ///< pad + 1 last struck, for the designer's tick; 0 = none
/* What a pad struck, so its release stops those notes and not whatever the
 * mapping says now: five 7-bit notes and a 3-bit count in one word, which a
 * single store publishes and a single load reads -- the pads and a touch on a
 * cell come from two tasks. 64 bits since the ninths and the 13 have five
 * notes; zero (the PSRAM BSS clear) is "nothing held". */
PP_SLOW_BSS std::atomic<uint64_t> s_padHeld[kPads];
uint64_t pack_chord(const uint8_t* n, int count)
{
    uint64_t w = (uint64_t)count;
    for (int i = 0; i < count && i < kChordMax; ++i) w |= (uint64_t)(n[i] & 0x7F) << (3 + 7 * i);
    return w;
}
inline int unpack_chord(uint64_t w, uint8_t* out)
{
    const int count = std::min((int)(w & 7u), kChordMax);
    for (int i = 0; i < count; ++i) out[i] = (uint8_t)((w >> (3 + 7 * i)) & 0x7F);
    return count;
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

/* The scales are crosspad::MusicTheory's: PlaySettings is the answer, sixteen
 * pads' notes recomputed by rebuildPadMap() whenever the key, octave or scale
 * changes, one note a pad. Transpose is ours, applied on top of what it
 * returns -- a semitone shift of everything, which is what a singer asks
 * for, and not the same gesture as changing the key.
 *
 * Chords are the Chord Designer's (core crosspad::chords, CHORD on the keys):
 * a list of up to sixteen chords spelled out -- root, quality, inversion --
 * and, while it is on, pad n plays chord n in the octave OCT says, voiced the
 * way the list says, shifted by the transpose; pads past its end are silent.
 * It replaces what this app had -- a chord type on every pad, the Diatonic and
 * Types modes, progressions of scale degrees -- because a genre's chords are
 * a record of particular chords, not degrees of whatever key is set.
 *
 * In PSRAM, all of it: this board aborts at boot over 144 bytes more of
 * internal RAM. */

PP_SLOW_BSS crosspad::MusicTheory  s_mt;
PP_SLOW_BSS crosspad::PlaySettings s_ps;

PP_SLOW_BSS int s_oct;          ///< PlaySettings octave, -2..+4
constexpr int kOctDefault = 1;  ///< MusicTheory's base is (3+oct)*12, so 1 puts pad 1 on C3,
                                ///< where this app has always started
PP_SLOW_BSS int s_scale;        ///< index into MusicTheory's scales
PP_SLOW_BSS int s_transpose;    ///< semitones, -12..+12, on top of all of it

PP_SLOW_BSS crosspad::chords::Progression s_prog;   ///< the Chord Designer's list
PP_SLOW_BSS bool s_chordsOn;    ///< the pads play its chords rather than the scale
PP_SLOW_BSS bool s_progDirty;   ///< changed since it was last stored

/// The C a chord counts from: the octave pad 1 is in, so a chord of the key
/// sits where the key's own note does.
int chord_octave() { return s_oct + 2; }

int clamp_note(int n) { return n < 0 ? 0 : (n > 127 ? 127 : n); }

/// "FUNK", "FUNK*" once edited, "USER" for one of the player's own.
void prog_label(char* buf, size_t n)
{
    std::snprintf(buf, n, "%s%s", crosspad::chords::presetName(s_prog.preset), s_prog.edited ? "*" : "");
}

/* Pitch Play is the only writer of the shared state (core's
 * InstrumentPlayState): the Sequencer recording an instrument and the
 * Recorder bouncing one have to sound like these pads do, and they only can
 * if they read this instead of keeping a copy of the key, the scale and the
 * chords. Published from the two places the state can change -- the map here,
 * the instrument when a load lands. */
PP_SLOW_BSS uint32_t s_pubGen;  ///< the shared state's generation as this app last left it

void publish_play_state()
{
    auto& st = crosspad::instrumentPlayState();
    st.play = s_ps;                  // rebuildPadMap() and the chords are already on it
    st.transpose = (int8_t)s_transpose;
    st.chordsOn = s_chordsOn;
    if (s_chordsOn) {
        char b[16];
        prog_label(b, sizeof(b));
        st.chordName = b;
    } else {
        const char* cn = s_mt.chordName(0);
        st.chordName = cn ? cn : "";
    }
    crosspad::instrumentPlayStatePublished();
    s_pubGen = st.generation;
}

/// The list onto the pads: chord n's notes on pad n, the rest silent.
void put_chords()
{
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        uint8_t notes[crosspad::chords::kMaxNotes] = {};
        const uint8_t n = pad < s_prog.count
            ? crosspad::chords::notes(s_prog.chords[pad], chord_octave(), s_prog.voicing, notes) : 0;
        for (int c = 0; c < 6; ++c) s_ps.padChordNotes[pad][c] = c < n ? notes[c] : 0xFF;
        s_ps.padChordCount[pad] = n;
        if (!n) s_ps.padSilent = (uint16_t)(s_ps.padSilent | (1u << pad));
    }
}

void rebuild_pads()
{
    s_ps.rootNote   = (uint8_t)s_key;
    s_ps.octave     = (int8_t)s_oct;
    s_ps.scaleIndex = (uint8_t)s_scale;
    s_ps.chordIndex = 0;             // one note a pad; chords come from the list
    s_mt.rebuildPadMap(s_ps);
    if (s_chordsOn) put_chords();
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

/// The notes a press on @p pad makes, into @p out: none on a silent pad, its
/// own note when it has no chord.
int chord_of(uint8_t pad, uint8_t* out)
{
    if (pad >= kPads || (s_ps.padSilent & (1u << pad))) return 0;
    int n = 0;
    const int have = s_ps.padChordCount[pad];
    for (int c = 0; c < have && c < 6 && n < kChordMax; ++c) {
        const uint8_t note = s_ps.padChordNotes[pad][c];
        if (note != 0xFF) out[n++] = note;
    }
    if (n == 0) out[n++] = s_ps.padMidiMap[pad];
    return n;
}

/* The list is kept across a reboot: one blob in Pitch Play's NVS namespace,
 * written when the designer closes (not on every tap -- flash wears) and when
 * the app does. A blob that is not exactly this, or does not pass valid(),
 * is the first preset instead. The degree progressions this app used to keep
 * ("progs") are left where they are: nothing here plays them any more. */
constexpr const char* kNvsNs  = "pitchplay";
constexpr const char* kNvsKey = "chords";
constexpr uint8_t kChordsBlobVersion = 1;

struct StoredChords {
    uint8_t version;
    uint8_t on;
    crosspad::chords::Progression prog;
};

void save_chords()
{
    if (!s_progDirty) return;
    s_progDirty = false;
#ifdef ESP_PLATFORM
    StoredChords b = {kChordsBlobVersion, (uint8_t)(s_chordsOn ? 1 : 0), s_prog};
    nvs_handle_t h;
    if (nvs_open(kNvsNs, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, kNvsKey, &b, sizeof(b));
    nvs_commit(h);
    nvs_close(h);
#endif
}

void load_chords()
{
    crosspad::chords::init(s_prog);
    s_chordsOn = false;
    s_progDirty = false;
#ifdef ESP_PLATFORM
    nvs_handle_t h;
    if (nvs_open(kNvsNs, NVS_READONLY, &h) != ESP_OK) return;
    StoredChords b;
    size_t len = sizeof(b);
    if (nvs_get_blob(h, kNvsKey, &b, &len) == ESP_OK && len == sizeof(b) &&
        b.version == kChordsBlobVersion && crosspad::chords::valid(b.prog)) {
        s_prog = b.prog;
        s_chordsOn = b.on != 0;
    }
    nvs_close(h);
#endif
}

/// "Em7" -- the chord pad @p pad plays, or "" past the end of the list.
void pad_chord_name(uint8_t pad, char* buf, size_t len)
{
    if (pad >= s_prog.count) { if (len) buf[0] = '\0'; return; }
    crosspad::chords::shortName(s_prog.chords[pad], buf, len);
}

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
    void onPadPress(crosspad::PadManager& pm, uint8_t pad, uint8_t vel) override
    {
        /* The sequencer's presses are the kit's notes (route 0 of a song, the
         * Drums track): the kit plays them. Played here, a song under this
         * screen came out as the Drums track's notes on the instrument. */
        if (pm.pressSource(pad) == crosspad::EventSource::SequencerCallback) {
            if (auto base = beneath(pm)) base->onPadPress(pm, pad, vel);
            return;
        }
        if (pad >= kPads) return;
        /* The chord is kept as it was struck, so a release still stops the
         * right notes after OCT, the key or the chord itself has moved. */
        uint8_t notes[kChordMax] = {};
        const int n = chord_of(pad, notes);
        s_padHeld[pad].store(pack_chord(notes, n));
        if (!n) return;                        // past the end of the chord list
        for (int i = 0; i < n; ++i) play_one(notes[i], vel, pad);
        light_on(pad, vel);
        s_padPicked.store((uint8_t)(pad + 1));  // the designer selects what was played
    }
    void onPadRelease(crosspad::PadManager& pm, uint8_t pad) override
    {
        if (pm.pressSource(pad) == crosspad::EventSource::SequencerCallback) {
            if (auto base = beneath(pm)) base->onPadRelease(pm, pad);
            return;
        }
        if (pad >= kPads) return;
        auto* p = inst();
        uint8_t notes[kChordMax];
        const int n = unpack_chord(s_padHeld[pad].exchange(0), notes);
        for (int i = 0; i < n; ++i) if (p) p->noteOff(notes[i]);
        light_off(pad);
        pm.notifyNoteOff(pad, true);
    }
    void onPadPressure(crosspad::PadManager&, uint8_t, uint8_t) override {}

private:
    std::shared_ptr<crosspad::IPadLogicHandler> beneath(crosspad::PadManager& pm) const
    {
        auto base = pm.getPadLogic(pm.getBasePadLogic());
        return base.get() == this ? nullptr : base;
    }
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

/// The lowest note @p pad sounds -- its own, or its chord's -- or -1 when silent.
int sound_note(uint8_t pad)
{
    if (pad >= kPads || (s_ps.padSilent & (1u << pad))) return -1;
    return s_chordsOn && s_ps.padChordCount[pad] ? s_ps.padChordNotes[pad][0] : pad_note(pad);
}

void show_pads()
{
    char name[8];
    for (uint8_t pad = 0; pad < kPads; ++pad) {
        lv_obj_t* c = s_cells[pad];
        if (!c) continue;
        const int note = sound_note(pad);
        if (s_chordsOn) pad_chord_name(pad, name, sizeof(name));
        else            note_name(note, name, sizeof(name));
        lv_obj_t* l = lv_obj_get_child(c, 0);
        lv_label_set_text(l, name);
        /* "C#maj7" is wider than a 36 px cell in 14 px type. */
        lv_obj_set_style_text_font(l, std::strlen(name) > 4 ? &lv_font_montserrat_10
                                                             : &lv_font_montserrat_14, 0);
        const int z = s_haveSpec && note >= 0 ? zone_for(note) : -1;
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

/* The chord list's colours on the pads, as the designer draws its strip:
 * a chord in dim purple, the selected one lime, purple while it sounds, and
 * nothing past the end of the list. */
constexpr uint32_t kChordRgb    = 0x3B2A5E;
constexpr uint32_t kChordSelRgb = 0xC6FF47;
constexpr uint32_t kChordHitRgb = 0xA77BF0;

crosspad::RgbColor rgb_of(uint32_t rgb)
{
    return crosspad::RgbColor((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
}

/// The pads' LEDs in their zones' colours: dim at rest, full while hit.
void show_leds()
{
    auto& leds = crosspad::getPadLedController();
    if (s_chordsOn) {
        for (uint8_t pad = 0; pad < kPads; ++pad) {
            const bool used = pad < s_prog.count;
            leds.setPadColor(pad, rgb_of(!used ? 0 : pad == s_prog.selected ? kChordSelRgb : kChordRgb));
            leds.setPadPressedColor(pad, rgb_of(used ? kChordHitRgb : 0));
        }
        return;
    }
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
        if (s_chordsOn) prog_label(buf, sizeof(buf));
        else            std::snprintf(buf, sizeof(buf), "CHORD off");
        lv_label_set_text(s_chordField, buf);
    }
}

/// The selected zone's file on the card, or "" when there is none to draw.
///
/// The tools open what they are given as it stands, and a zone's sample is
/// logical -- absolute without the card's mount ("/crosspad/..."), or
/// relative to the instrument's folder. Handed over as it was, it opened
/// nothing and the box stayed empty. Joined rather than resolved:
/// resolveKitSamplePath() asks the card which of two layouts a relative
/// sample is in, and this runs every tick on the UI thread. Pitch Play writes
/// its instruments flat ("SAMPLES/x.wav" beside the file), the layout that
/// resolver tries first.
std::string wave_file()
{
    auto* st = tools();
    const auto& zs = s_spec.instrument.zones;
    if (!st || !s_haveSpec || zs.empty()) return {};
    const int i = (s_zoneSel >= 0 && s_zoneSel < (int)zs.size()) ? s_zoneSel : 0;
    const std::string& s = zs[(size_t)i].sample;
    if (s.empty()) return {};
    if (s[0] == '/') return st->cardRoot() + s;
    const size_t slash = s_spec.path.rfind('/');
    if (slash == std::string::npos) return {};
    return st->cardRoot() + s_spec.path.substr(0, slash + 1) + s;
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
    if (f.empty() || (f == s_wave.file && s_viewA == s_waveHasA && s_viewB == s_waveHasB)) return;
    if (!st->requestOverview(f, s_viewA, s_viewB, 150)) return;
    s_wavePending = true;
    s_waveWant = f;
    s_waveAskA = s_viewA;
    s_waveAskB = s_viewB;
}

/// The zone, drawn as the min/max columns the tools read, in the colour of
/// the zone it is -- the same colour the pads it plays are underlined in.
/// Frames [litFrom, litTo) of @p total are in that colour and the rest dim;
/// litFrom == litTo lights all of it.
void draw_wave(lv_layer_t* layer, const lv_area_t& a, uint32_t viewA, uint32_t viewLen, uint32_t litFrom,
               uint32_t litTo)
{
    if (s_wave.hi.empty() || s_wave.hi.size() != s_wave.lo.size()) return;
    const int32_t w = a.x2 - a.x1 + 1, h = a.y2 - a.y1 + 1;
    if (w <= 2 || h <= 4) return;
    const int32_t mid = a.y1 + h / 2, half = h / 2 - 1;

    lv_draw_fill_dsc_t d;
    lv_draw_fill_dsc_init(&d);
    const int zi = (s_zoneSel >= 0) ? s_zoneSel : 0;
    const lv_color_t lit = lv_color_hex(kZoneRgb[(size_t)zi % kZoneColors]);
    d.opa = LV_OPA_COVER;
    const size_t cols = s_wave.hi.size();
    for (int32_t x = 0; x < w; ++x) {
        const size_t i = (size_t)((uint64_t)x * cols / (uint64_t)w);
        if (i >= cols) break;
        const uint32_t f = viewA + (viewLen ? (uint32_t)((uint64_t)x * viewLen / (uint64_t)w) : 0);
        d.color = (litFrom == litTo || (f >= litFrom && f < litTo)) ? lit : kDim;
        int32_t hi = (int32_t)s_wave.hi[i] * half / 127;
        int32_t lo = (int32_t)s_wave.lo[i] * half / 127;
        if (hi < lo) { const int32_t t = hi; hi = lo; lo = t; }
        if (hi - lo < 1) { hi = 1; lo = -1; }
        const lv_area_t r = {a.x1 + x, mid - hi, a.x1 + x, mid - lo};
        lv_draw_fill(layer, &d, &r);
    }
}

void wave_draw_cb(lv_event_t* e)
{
    lv_area_t a;
    lv_obj_get_coords(lv_event_get_target_obj(e), &a);
    draw_wave(lv_event_get_layer(e), a, 0, 0, 0, 0);
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

/// One WAV is an instrument of one zone, its pitch detected; named after the
/// file unless @p name says otherwise.
void open_sample(const std::string& logicalPath, const std::string& name = {})
{
    s_spec = crosspad::KitInfo();
    std::string file = base_of(logicalPath);
    if (file.size() > 4) file.resize(file.size() - 4);
    s_spec.name = name.empty() ? file : name;
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

/// What the sample window offers, in its order: the saved instruments by
/// group -- the groups, or the one open, its instruments --, the Recorder's
/// takes, and the card for any WAV.
void fill_dropdown()
{
    s_entries.clear();
    s_labels.clear();
    auto* h = host();
    if (!h) return;
    if (s_sampleGroup[0]) {
        std::vector<std::string> names;
        for (const auto& n : s_library)
            if (crosspad::InstrumentGroups::group(n) == s_sampleGroup) names.push_back(n);
        std::sort(names.begin(), names.end());
        for (const auto& n : names) {
            s_entries.push_back({EntryKind::Instrument,
                                 h->libraryRoot() + "/" + n + "/" + crosspad::IInstrumentHost::kFileName});
            s_labels.push_back(std::string(LV_SYMBOL_KEYBOARD " ") + crosspad::InstrumentGroups::shortName(n));
        }
        show_name();
        if (s_ov == Overlay::Sample) { s_fillPending = false; open_sample_window(); }
        return;
    }
    std::vector<std::pair<std::string, unsigned>> groups;
    for (const auto& n : s_library) {
        const std::string g = crosspad::InstrumentGroups::group(n);
        auto it = std::find_if(groups.begin(), groups.end(), [&](const auto& e) { return e.first == g; });
        if (it == groups.end()) groups.push_back({g, 1});
        else                    ++it->second;
    }
    std::sort(groups.begin(), groups.end());
    for (const auto& g : groups) {
        s_entries.push_back({EntryKind::Group, g.first});
        char label[48];
        std::snprintf(label, sizeof(label), LV_SYMBOL_DIRECTORY " %s  (%u)", g.first.c_str(), g.second);
        s_labels.push_back(label);
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
    if (entry.kind == EntryKind::Group) {
        std::snprintf(s_sampleGroup, sizeof(s_sampleGroup), "%s", entry.path.c_str());
        s_samplePage = 0;
        fill_dropdown();          // the window is open: it shows the group
        return;
    }
    close_overlay();
    switch (entry.kind) {
    case EntryKind::Instrument: open_instrument(entry.path); break;
    case EntryKind::Sample:     open_sample(entry.path); break;
    case EntryKind::Group:      break;
    case EntryKind::Card:       open_browser(false); break;
    }
}

// ── overlays ───────────────────────────────────────────────────────────────

void cd_close();

void close_overlay()
{
    if (!s_overlay) return;
    if (s_ov == Overlay::Chord) cd_close();
#ifdef ROM_USE_ENCODER
    if (indev_encoderTop && s_encGroup) lv_indev_set_group(indev_encoderTop, s_encGroup);
    lv_group_delete_deferred(s_overlayGroup);     // the click that closed it is still being read
#endif
    s_overlayGroup = nullptr;
    lv_obj_delete_async(s_overlay);
    s_overlay = s_zoneList = s_rootLabel = s_nameArea = nullptr;
    for (auto& v : s_scaleVal) v = nullptr;   // children of the overlay, gone with it
    s_ov = Overlay::None;
    s_loopLabel = s_loopEndLabel = s_loopWave = s_loopBtn = s_startLabel = s_xfLabel = nullptr;
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

/// What zone @p i's row in EDIT says: its number, its root and its file.
void zone_row_text(size_t i, char* buf, size_t cap)
{
    const auto& z = s_spec.instrument.zones[i];
    char root[8];
    if (z.root < 0) std::snprintf(root, sizeof(root), "auto");
    else note_name((int)std::lround(z.root), root, sizeof(root));
    std::snprintf(buf, cap, "%u  %s   %s", (unsigned)(i + 1), root, base_of(z.sample).c_str());
}

/// The selected zone's root changed: its row and the ROOT field, in place.
/// show_zones() rebuilds the list and puts the focus on the zone, which took
/// it out of the field being turned after a single detent.
void show_zone_root()
{
    if (s_zoneList && s_zoneSel >= 0 && s_zoneSel < (int)s_spec.instrument.zones.size()) {
        lv_obj_t* row = lv_obj_get_child(s_zoneList, s_zoneSel);
        lv_obj_t* label = row ? lv_obj_get_child(row, 0) : nullptr;
        if (label) {
            char buf[96];
            zone_row_text((size_t)s_zoneSel, buf, sizeof(buf));
            lv_label_set_text(label, buf);
        }
    }
    show_root();
}

void show_zones()
{
    if (!s_zoneList) return;
    lv_obj_clean(s_zoneList);
    const auto& zones = s_spec.instrument.zones;
    for (size_t i = 0; i < zones.size(); ++i) {
        char buf[96];
        zone_row_text(i, buf, sizeof(buf));
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

void zone_edited()
{
    s_liveWanted = true;
    s_liveAt = lv_tick_get();
}

/// The selected zone's root, loop, start and crossfade into the bank that is
/// playing, as the sampler's pitched editor does it. A zone the bank does not
/// have yet (added in EDIT) is left to the rebuild.
void apply_zone_live()
{
    s_liveWanted = false;
    auto* in = inst();
    auto* bank = in ? in->bank() : nullptr;
    const auto& zones = s_spec.instrument.zones;
    if (!bank || s_zonesMoved || s_zoneSel < 0 || s_zoneSel >= (int)zones.size() ||
        s_zoneSel >= bank->zoneCount()) {
        return;
    }
    const auto& z = zones[s_zoneSel];
    const float root = z.root < 0.0f ? bank->zone((uint8_t)s_zoneSel).rootMidi : z.root;
    in->editZone((uint8_t)s_zoneSel, root, z.loopStart, z.loopEnd, 10, nullptr, z.loopXfade, z.start);
}

void root_by(int step)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const int r = z.root < 0 ? 60 : (int)std::lround(z.root);
    z.root = (float)std::max(0, std::min(127, r + step));
    s_dirty = true;
    zone_edited();
    show_zone_root();
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

/// The selected zone's length: what the bank made of it, or -- a zone added
/// in EDIT and not built yet -- its file's, from the picture. 0 = unknown.
uint32_t zone_frames()
{
    const auto za = zone_audio();
    if (za.frames) return za.frames;
    if (!s_wave.hi.empty() && s_wave.file == wave_file()) return s_wave.frames;
    return 0;
}

/// @p frames as the row shows them: seconds when the rate is known.
void frames_text(uint32_t frames, char* buf, size_t cap, bool ms = false)
{
    const auto za = zone_audio();
    if (!za.sampleRate) { std::snprintf(buf, cap, "%u fr", (unsigned)frames); return; }
    const double s = (double)frames / (double)za.sampleRate;
    if (ms) std::snprintf(buf, cap, "%u ms", (unsigned)std::lround(s * 1000.0));
    else    std::snprintf(buf, cap, "%.2f s", s);
}

/// EDIT's picture of the selected zone: the waveform with the loop lit, the
/// stretch that crossfades shaded at both its ends -- the loop's end, which
/// fades out, and what leads into the loop start, which fades in -- and a
/// line where a note begins.
void loop_wave_draw_cb(lv_event_t* e)
{
    const auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    const auto& z = zones[s_zoneSel];
    const uint32_t total = zone_frames();
    if (!total) return;
    lv_layer_t* layer = lv_event_get_layer(e);
    lv_area_t a;
    lv_obj_get_coords(lv_event_get_target_obj(e), &a);
    const int32_t w = a.x2 - a.x1 + 1;
    if (w <= 2) return;
    /* All of the zone, or -- turned FINE -- the stretch round the point. */
    const bool zoomed = s_wave.file == wave_file() && s_waveHasB > s_waveHasA;
    const uint32_t viewA = zoomed ? s_waveHasA : 0;
    const uint32_t viewLen = zoomed ? s_waveHasB - s_waveHasA : total;
    auto xOf = [&](uint32_t f) {
        f = f < viewA ? 0 : std::min(f - viewA, viewLen);
        return a.x1 + (int32_t)((uint64_t)f * (uint64_t)(w - 1) / viewLen);
    };

    const bool looping = z.loopEnd > z.loopStart;
    lv_draw_fill_dsc_t d;
    lv_draw_fill_dsc_init(&d);
    d.color = kAccent;
    if (looping) {
        const uint32_t xf = crosspad::SampleBank::loopXfadeFor(z.loopStart, z.loopEnd, z.loopXfade);
        d.opa = LV_OPA_10;
        const lv_area_t loop = {xOf(z.loopStart), a.y1, xOf(z.loopEnd), a.y2};
        lv_draw_fill(layer, &d, &loop);
        if (xf) {
            d.opa = LV_OPA_30;
            const lv_area_t out = {xOf(z.loopEnd - xf), a.y1, xOf(z.loopEnd), a.y2};
            lv_draw_fill(layer, &d, &out);
            const lv_area_t in = {xOf(z.loopStart - xf), a.y1, xOf(z.loopStart), a.y2};
            lv_draw_fill(layer, &d, &in);
        }
    }
    if (s_wave.file == wave_file()) {
        draw_wave(layer, a, viewA, viewLen, looping ? z.loopStart : 0, looping ? z.loopEnd : 0);
    }
    d.color = kHead;
    d.opa = LV_OPA_COVER;
    const int32_t sx = xOf(crosspad::SampleBank::startFor(total, z.loopStart, z.loopEnd, z.start));
    const lv_area_t mark = {sx, a.y1, sx, a.y2};
    lv_draw_fill(layer, &d, &mark);
}

void show_loop()
{
    const auto& zones = s_spec.instrument.zones;
    const bool have = s_zoneSel >= 0 && s_zoneSel < (int)zones.size();
    if (s_loopLabel && s_loopEndLabel) {
        /* Seconds, or frames while the length is not known yet: the points
         * are real and will be used, only a time for them would be wrong. */
        char s[24] = "S --", e[24] = "E --", t[16];
        if (have && zones[s_zoneSel].loopEnd) {
            frames_text(zones[s_zoneSel].loopStart, t, sizeof(t));
            std::snprintf(s, sizeof(s), "S %s", t);
            frames_text(zones[s_zoneSel].loopEnd, t, sizeof(t));
            std::snprintf(e, sizeof(e), "E %s", t);
        }
        lv_label_set_text(s_loopLabel, s);
        lv_label_set_text(s_loopEndLabel, e);
    }
    if (s_loopBtn) {
        const bool on = have && zones[s_zoneSel].loopEnd != 0;
        lv_label_set_text(lv_obj_get_child(s_loopBtn, 0), on ? "ON" : "OFF");
        lv_obj_set_style_bg_color(s_loopBtn, on ? kAccent : kPanel, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(s_loopBtn, 0), on ? kInk : kText, 0);
    }
    if (s_startLabel) {
        char buf[24] = "START --", t[16];
        if (have) {
            frames_text(zones[s_zoneSel].start, t, sizeof(t));
            std::snprintf(buf, sizeof(buf), "START %s", t);
        }
        lv_label_set_text(s_startLabel, buf);
    }
    if (s_xfLabel) {
        char buf[24] = "XF --", t[16];
        if (have && zones[s_zoneSel].loopEnd > zones[s_zoneSel].loopStart) {
            const auto& z = zones[s_zoneSel];
            const uint32_t xf = crosspad::SampleBank::loopXfadeFor(z.loopStart, z.loopEnd, z.loopXfade);
            if (xf) frames_text(xf, t, sizeof(t), true);
            else    std::snprintf(t, sizeof(t), "off");
            std::snprintf(buf, sizeof(buf), "XF %s", t);
        }
        lv_label_set_text(s_xfLabel, buf);
    }
    if (s_loopWave) lv_obj_invalidate(s_loopWave);
}

/// Two hundred steps across the sample, so one press is a visible move on any
/// length; a zone whose length is unknown steps by a flat 1000 frames.
uint32_t loop_step()
{
    /* FINE: a millisecond. */
    if (s_fineField >= 0) {
        const uint32_t rate = zone_audio().sampleRate ? zone_audio().sampleRate : s_wave.sampleRate;
        return std::max<uint32_t>(1, (rate ? rate : 44100) / 1000);
    }
    const uint32_t len = zone_frames();
    return len ? std::max<uint32_t>(1, len / 200) : 1000u;
}

/// @p which 0 = start, 1 = end. The two never cross, and the end is what
/// switches a one-shot into a loop.
void loop_by(int which, int dir)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const uint32_t len = zone_frames();             // 0 = unknown, so unclamped
    const int64_t step = (int64_t)loop_step() * dir;
    if (which == 0) {
        int64_t v = (int64_t)z.loopStart + step;
        const int64_t hi = z.loopEnd ? (int64_t)z.loopEnd - 1 : (len ? (int64_t)len - 1 : v);
        z.loopStart = (uint32_t)std::max<int64_t>(0, std::min(hi, v));
    } else {
        /* From a one-shot, the first + opens a loop over the whole sample
         * rather than a zero-length one at the start, which would be silent. */
        int64_t cur = z.loopEnd ? (int64_t)z.loopEnd : (len ? (int64_t)len : 0);
        if (!z.loopEnd && dir > 0) { z.loopEnd = (uint32_t)cur; s_dirty = true; zone_edited(); show_loop(); return; }
        int64_t v = cur + step;
        const int64_t lo = (int64_t)z.loopStart + 1;
        const int64_t hi = len ? (int64_t)len : v;
        v = std::max(lo, std::min(hi, v));
        z.loopEnd = (uint32_t)v;
    }
    s_dirty = true;
    zone_edited();
    show_loop();
}

void loop_toggle_cb(lv_event_t*)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    if (z.loopEnd) {
        z.loopEnd = 0;                       // one-shot; the start is kept
    } else {
        const uint32_t len = zone_frames();
        if (!len) { show_loop(); return; }   // nothing to loop over yet
        /* A held note out of one sample, ready to hear: the loop past the
         * attack and short of the tail, a quarter of it crossfaded. The points
         * move from there; the note still begins at START. */
        z.loopStart = len * 3 / 10;
        z.loopEnd   = len * 9 / 10;
        z.loopXfade = (z.loopEnd - z.loopStart) / 4;
    }
    s_dirty = true;
    zone_edited();
    show_loop();
}

/// Where a note begins -- the attack, before the loop -- a step at a time.
/// Never at or past the loop's end, which it would then never reach.
void start_by(int dir)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const uint32_t len = zone_frames();
    const int64_t v = (int64_t)z.start + (int64_t)loop_step() * dir;
    const int64_t hi = z.loopEnd > z.loopStart ? (int64_t)z.loopEnd - 1
                                               : (len ? (int64_t)len - 1 : v);
    z.start = (uint32_t)std::max<int64_t>(0, std::min(hi, v));
    s_dirty = true;
    zone_edited();
    show_loop();
}

/// The loop's crossfade, in twentieths of the most this loop can have: its
/// own length, or what there is before its start.
void xf_by(int dir)
{
    auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
    auto& z = zones[s_zoneSel];
    const uint32_t most = crosspad::SampleBank::loopXfadeFor(z.loopStart, z.loopEnd, UINT32_MAX);
    if (!most) return;
    const int64_t step = std::max<int64_t>(1, most / 20);
    const int64_t cur = std::min<uint32_t>(z.loopXfade, most);
    z.loopXfade = (uint32_t)std::max<int64_t>(0, std::min<int64_t>(most, cur + step * dir));
    s_dirty = true;
    zone_edited();
    show_loop();
}

/* EDIT's numbers are turned, not pressed: a click on one -- the knob or a
 * finger -- starts turning it, the encoder moves it, a click stops. The way
 * the level is set, and it replaced a - and a + beside every value. */
enum EditField : int { kFieldRoot, kFieldLoopStart, kFieldLoopEnd, kFieldStart, kFieldXfade };

/* One step a detent is right for a fine move and two hundred detents for a
 * point to cross the sample, so the frame fields take bigger steps while the
 * knob turns fast. A semitone stays a semitone. */
crosspad::EncoderAccelerator s_editAccel;

/// The frame a frame field stands for: the loop's ends (a one-shot's end is
/// the zone's), or where a note begins.
uint32_t field_point(int which)
{
    const auto& zones = s_spec.instrument.zones;
    if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return 0;
    const auto& z = zones[s_zoneSel];
    if (which == kFieldLoopStart) return z.loopStart;
    if (which == kFieldLoopEnd) return z.loopEnd ? z.loopEnd : zone_frames();
    return z.start;
}

/// The picture round @p at, or -- moved within a quarter of its edge -- moved
/// with it.
void zoom_on(uint32_t at, bool follow = false)
{
    const uint32_t len = zone_frames();
    if (!len) return;
    if (follow && s_viewB > s_viewA && at >= s_viewA + kZoomFrames / 4 && at + kZoomFrames / 4 <= s_viewB) return;
    s_viewA = at > kZoomFrames ? at - kZoomFrames : 0;
    s_viewB = std::min(len, at + kZoomFrames);
}

void zoom_off()
{
    s_fineField = -1;
    s_turnField = -1;
    s_viewA = s_viewB = 0;
    if (s_loopWave) lv_obj_invalidate(s_loopWave);
}

bool is_frame_field(int which)
{
    return which == kFieldLoopStart || which == kFieldLoopEnd || which == kFieldStart;
}

/// The frame fields click through three states, as the pad editor's markers
/// do: turned a step of the sample at a time, then FINE -- a millisecond,
/// the picture zoomed in round the point -- then done.
void edit_field_cb(lv_event_t* e)
{
    const int which = (int)(intptr_t)lv_event_get_user_data(e);
    const lv_event_code_t code = lv_event_get_code(e);
    if (is_frame_field(which) && (code == LV_EVENT_CLICKED || code == LV_EVENT_FOCUSED)) {
        /* Its own state, as the pad editor's markers keep theirs: LVGL turns
         * the group's editing by itself on the way to these events. */
        lv_obj_t* b = static_cast<lv_obj_t*>(lv_event_get_current_target(e));
        lv_group_t* g = b ? lv_obj_get_group(b) : nullptr;
        if (code == LV_EVENT_FOCUSED) {
            /* The encoder's first click: LVGL says so with FOCUSED. A finger's
             * press focuses the field first, which ends editing on the way;
             * its CLICKED comes next and decides. */
            lv_indev_t* in = lv_indev_active();
            if (in && lv_indev_get_type(in) == LV_INDEV_TYPE_POINTER) return;
            const bool editing = g && lv_group_get_editing(g);
            if (editing && s_turnField != which) { zoom_off(); s_turnField = which; }
            else if (!editing && s_turnField == which) zoom_off();
            return;
        }
        if (s_turnField == which && s_fineField == which) {
            zoom_off();
            if (g) lv_group_set_editing(g, false);
        } else if (s_turnField == which) {
            s_fineField = which;
            zoom_on(field_point(which));
            if (g) lv_group_set_editing(g, true);
        } else {
            zoom_off();
            s_turnField = which;
            if (g) { lv_group_focus_obj(b); lv_group_set_editing(g, true); }
        }
        return;
    }
    int step = number_step(e);
    if (!step) return;
    if (which != kFieldRoot) step *= s_editAccel.update(1, lv_tick_get());
    switch (which) {
        case kFieldRoot:      root_by(step);     break;
        case kFieldLoopStart: loop_by(0, step);  break;
        case kFieldLoopEnd:   loop_by(1, step);  break;
        case kFieldStart:     start_by(step);    break;
        case kFieldXfade:     xf_by(step);       break;
        default: break;
    }
    if (s_fineField == which) zoom_on(field_point(which), true);
}

/// FINE ends with the turning however that ends -- the power button, another
/// field picked, the window closed -- and the picture shows all of the zone.
void zoom_check()
{
    if (s_turnField < 0) return;
    lv_obj_t* label = s_turnField == kFieldLoopStart ? s_loopLabel
                    : s_turnField == kFieldLoopEnd   ? s_loopEndLabel : s_startLabel;
    lv_obj_t* b = label ? lv_obj_get_parent(label) : nullptr;
    /* A finger on it ends editing for the length of the press (LVGL focuses
     * it first); the click that follows decides. */
    if (b && lv_obj_has_state(b, LV_STATE_PRESSED)) return;
    lv_group_t* g = b ? lv_obj_get_group(b) : nullptr;
    if (!g || !lv_group_get_editing(g) || lv_group_get_focused(g) != b) zoom_off();
}

/// A turned field in @p row; @p w 0 shares the row's free width. Returns the
/// label inside it, which the show_* functions write.
lv_obj_t* edit_field(lv_obj_t* row, int32_t w, int which)
{
    lv_obj_t* b = lv_CreateEditableButton(row);
    style_button(b, w ? w : 40, kRowH);
    if (!w) lv_obj_set_flex_grow(b, 1);
    lv_obj_add_event_cb(b, edit_field_cb, LV_EVENT_KEY, (void*)(intptr_t)which);
    lv_obj_add_event_cb(b, edit_field_cb, LV_EVENT_CLICKED, (void*)(intptr_t)which);
    lv_obj_add_event_cb(b, edit_field_cb, LV_EVENT_FOCUSED, (void*)(intptr_t)which);
    if (s_overlayGroup) lv_group_add_obj(s_overlayGroup, b);
    return add_label(b, "--");
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
    s_rootLabel = edit_field(tools, 0, kFieldRoot);
    make_button(tools, 54, "AUTO", [](lv_event_t*) {
        auto& zones = s_spec.instrument.zones;
        if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
        zones[s_zoneSel].root = -1.0f;
        s_dirty = true;
        zone_edited();
        show_zone_root();
    }, s_overlayGroup);
    make_button(tools, 48, "DEL", [](lv_event_t*) {
        auto& zones = s_spec.instrument.zones;
        if (s_zoneSel < 0 || s_zoneSel >= (int)zones.size()) return;
        zones.erase(zones.begin() + s_zoneSel);
        s_zonesMoved = true;
        s_zoneSel = std::min<int>(s_zoneSel, (int)zones.size() - 1);
        s_dirty = true;
        show_zones();
    }, s_overlayGroup);

    /* LOOP, under ROOT: the start and the end of the selected zone's loop,
     * ON/OFF being end = 0 (the one-shot the loader already knows). START is
     * where a note begins and XF how much of the loop's end crossfades into
     * what leads up to its start, so a held note has no seam. Under them the
     * zone itself, marked: the same 150 columns the right panel draws --
     * 300 bytes, not a decoded picture a pitched bank would miss. */
    lv_obj_t* lrow = make_row(s_overlay, kRowH);
    lv_obj_t* ll = lv_label_create(lrow);
    lv_obj_set_style_text_color(ll, kDim, 0);
    lv_label_set_text(ll, "LOOP");
    s_loopBtn = make_button(lrow, 42, "OFF", loop_toggle_cb, s_overlayGroup);
    s_loopLabel = edit_field(lrow, 0, kFieldLoopStart);
    s_loopEndLabel = edit_field(lrow, 0, kFieldLoopEnd);

    lv_obj_t* srow = make_row(s_overlay, kRowH);
    s_startLabel = edit_field(srow, 0, kFieldStart);
    s_xfLabel = edit_field(srow, 0, kFieldXfade);

    s_loopWave = lv_obj_create(s_overlay);
    lv_obj_remove_style_all(s_loopWave);
    lv_obj_set_size(s_loopWave, LV_PCT(100), 30);
    lv_obj_set_style_radius(s_loopWave, 4, 0);
    lv_obj_set_style_bg_color(s_loopWave, kWaveBg, 0);
    lv_obj_set_style_bg_opa(s_loopWave, LV_OPA_COVER, 0);
    lv_obj_remove_flag(s_loopWave, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_loopWave, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_loopWave, loop_wave_draw_cb, LV_EVENT_DRAW_MAIN_END, nullptr);

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
    s_saveEdits = true;
    load_spec();
}

void edit_cb(lv_event_t*)
{
    if (s_ov != Overlay::None) return;
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

// ── the sample window, behind the instrument's name ───────────────────────

/* The saved instruments, the Recorder's takes and the card -- what the
 * drop-down in the old top row offered -- and SAVE, which keeps what is
 * playing as one of them. EDIT is a key of its own now: at the foot of this
 * list it sat under every take on the card, forty to a page. */
void open_sample_window()
{
    open_overlay(Overlay::Sample);
    char title[40];
    std::snprintf(title, sizeof(title), s_sampleGroup[0] ? "Sample  " LV_SYMBOL_RIGHT "  %s" : "Sample", s_sampleGroup);
    overlay_header(title);
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
    list_item(col, LV_SYMBOL_SAVE "  Save as ...",
              [](lv_event_t* e) { close_overlay(); save_cb(e); }, nullptr);
}

void sample_cb(lv_event_t*)
{
    if (s_ov != Overlay::None) return;
    s_samplePage = 0;          // the window opens at the top, every time
    open_sample_window();
}

/// CHORD: the Chord Designer. The field in the context row says what the
/// pads play; SCALE is where the rest of the mapping is set.
void chord_open();
void chord_cb(lv_event_t*) { chord_open(); }

// ── the mapping ────────────────────────────────────────────────────────────

/* No back_cb: this screen has no back button. Back is the power button's
 * click, which goes up one level and then out, and apps are switched with
 * MODE and a pad (UX 2000 section 2). */

/* Sixteen degrees of a scale span more than sixteen semitones, so the top
 * pad is what has to stay a note, so the setting is tried on the real map
 * and rolled back when the top of it has been clamped away. */
bool set_mapping(int oct, int key, int scale, int transpose)
{
    const int wasO = s_oct, wasK = s_key, wasS = s_scale, wasT = s_transpose;
    if (oct < -2 || oct > 4 || transpose < -12 || transpose > 12) return false;
    s_oct = oct; s_key = key; s_scale = scale; s_transpose = transpose;
    rebuild_pads();
    /* The map is clamped to 0..127, so a setting that runs off the top shows
     * as the last pads landing on the same note. Refuse it and go back. */
    if (s_ps.padMidiMap[kPads - 1] == s_ps.padMidiMap[kPads - 2]) {
        s_oct = wasO; s_key = wasK; s_scale = wasS; s_transpose = wasT;
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
    set_mapping(s_oct + step, s_key, s_scale, s_transpose);
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
                      s_chordsOn ? "." : "");
        lv_label_set_text(lv_obj_get_child(s_scaleBtn, 0), buf);
    }
    if (!s_scaleVal[0]) return;
    char buf[16];
    lv_label_set_text(s_scaleVal[0], key_name(s_key));
    const char* sc = crosspad::MusicTheory::scaleName((uint8_t)s_scale);
    lv_label_set_text(s_scaleVal[1], sc ? sc : "?");
    std::snprintf(buf, sizeof(buf), "%+d", s_transpose);
    lv_label_set_text(s_scaleVal[2], buf);
    show_key_strip();        // the picture and the names are one thing
}

/// -1 or +1 on one of the three, from the button's user data: (which << 1) | up.
void scale_step_cb(lv_event_t* e)
{
    const int code = (int)(intptr_t)lv_event_get_user_data(e);
    const int which = code >> 1;
    const int d = (code & 1) ? 1 : -1;
    const int scales = (int)crosspad::MusicTheory::scaleCount();
    switch (which) {
        case 0: set_mapping(s_oct, (s_key + 12 + d) % 12, s_scale, s_transpose); break;
        case 1: set_mapping(s_oct, s_key, (s_scale + scales + d) % scales, s_transpose); break;
        default: set_mapping(s_oct, s_key, s_scale, s_transpose + d); break;
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

/* ── CHORD: the Chord Designer (UX 2000) ─────────────────────────────────
 *
 * The screen under the transport bar, laid out the way the design draws it
 * (LVGL PROTO/lvgl_chord), in its own coordinates:
 *   context  the preset (a tap: the twelve), OCT, VOICE, CHORD ON/OFF
 *   strip    the chords eight at a time and a page key for the next eight;
 *            the selected one ringed lime, a "+" after the last
 *   roots    C D E F G A B and #
 *   types    maj min 7 maj7 m7 / 9 m9 13 dim sus4
 *   keys     INV n, ADD, DEL, < MOVE, MOVE >
 * A key acts on press, as a pad does, and an edit plays the selected chord
 * for 900 ms, so it is heard as it is made. A pad selects the chord it plays.
 * OCT is the pads' octave, the one OCT- / OCT+ move; opening the designer
 * puts the chords on the pads, and ON/OFF takes them off again. Back (the
 * power button's click) closes the presets, then the designer, which stores
 * the list. The encoder walks the keys, as everywhere. No BARS and no
 * playback: the pads play the chords, the transport does not. */

enum ChordAct : uint8_t { A_PRESET, A_OCT, A_VOICE, A_ON, A_SLOT, A_PAGE, A_ROOT, A_SHARP, A_TYPE, A_SK, A_LOAD };

constexpr int kStripSlots = 8;
constexpr uint32_t kAuditionMs = 900;
constexpr uint8_t kNatural[7] = {0, 2, 4, 5, 7, 9, 11};
const lv_color_t kPress = lv_color_hex(0x2C3138);
const lv_color_t kWell  = lv_color_hex(0x1B1E22);
const lv_color_t kMint  = lv_color_hex(0x00FF88);

/// Children of the overlay: gone with it, so dropped in close_overlay().
struct ChordView {
    lv_obj_t* preset;
    lv_obj_t* presetL;
    lv_obj_t* octV;
    lv_obj_t* voiceV;
    lv_obj_t* onKey;
    lv_obj_t* onV;
    lv_obj_t* pick;                  ///< strip, roots and types: hidden under the presets
    lv_obj_t* slot[kStripSlots];
    lv_obj_t* slotR[kStripSlots];
    lv_obj_t* slotQ[kStripSlots];
    lv_obj_t* slotBar[kStripSlots];  ///< the chord sounding, along the top
    lv_obj_t* page1;
    lv_obj_t* page2;
    lv_obj_t* root[7];
    lv_obj_t* sharp;
    lv_obj_t* type[crosspad::chords::QualityCount];
    lv_obj_t* invL;
    lv_obj_t* presets;
    lv_obj_t* presetKey[16];
    lv_timer_t* audition;
    uint8_t page;
    uint32_t drawnLit;
};
PP_SLOW_BSS ChordView s_cv;
PP_SLOW_BSS uint64_t s_auditionHeld;   ///< the audition's notes, packed as a pad's are
PP_SLOW_BSS uint8_t  s_auditionPad;    ///< and the pad it lit

lv_obj_t* cd_box(lv_obj_t* p, int32_t x, int32_t y, int32_t w, int32_t h, lv_color_t c)
{
    lv_obj_t* o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_radius(o, 4, 0);
    lv_obj_set_style_bg_color(o, c, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
    return o;
}

/// A label; one in the dim colour is marked so cd_on() gives it back its dim.
lv_obj_t* cd_text(lv_obj_t* p, const lv_font_t* f, lv_color_t c, const char* t)
{
    lv_obj_t* l = lv_label_create(p);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    lv_label_set_text(l, t);
    if (lv_color_eq(c, kDim)) lv_obj_set_user_data(l, (void*)1);
    return l;
}

void cd_ev(lv_event_t* e);

lv_obj_t* cd_key(lv_obj_t* p, int32_t x, int32_t y, int32_t w, int32_t h, int act, int arg)
{
    lv_obj_t* b = cd_box(p, x, y, w, h, kPanel);
    lv_obj_set_style_bg_color(b, kPress, LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(b, kHead, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_width(b, 2, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_pad(b, 1, LV_STATE_FOCUS_KEY);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(b, cd_ev, LV_EVENT_PRESSED, (void*)(intptr_t)((arg << 8) | act));
    if (s_overlayGroup) lv_group_add_obj(s_overlayGroup, b);
    return b;
}

/// A key with a dim name and a value: "OCT 3". Returns the value.
lv_obj_t* cd_field(int32_t x, int32_t w, const char* name, int act, lv_obj_t** key = nullptr)
{
    lv_obj_t* b = cd_key(s_overlay, x, 4, w, 22, act, 0);
    lv_obj_set_flex_flow(b, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(b, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(b, 5, 0);
    cd_text(b, &lv_font_montserrat_10, kDim, name);
    if (key) *key = b;
    return cd_text(b, &lv_font_montserrat_14, kText, "");
}

/// Lime with ink letters when on, the panel when not.
void cd_on(lv_obj_t* b, bool on)
{
    lv_obj_set_style_bg_color(b, on ? kAccent : kPanel, 0);
    const uint32_t n = lv_obj_get_child_count(b);
    for (uint32_t i = 0; i < n; ++i) {
        lv_obj_t* c = lv_obj_get_child(b, (int32_t)i);
        if (!lv_obj_check_type(c, &lv_label_class)) continue;
        lv_obj_set_style_text_color(c, on ? kInk : (lv_obj_get_user_data(c) ? kDim : kText), 0);
    }
}

void cd_audition_off(lv_timer_t* = nullptr)
{
    uint8_t notes[kChordMax];
    const int n = unpack_chord(s_auditionHeld, notes);
    s_auditionHeld = 0;
    if (auto* p = inst()) for (int i = 0; i < n; ++i) p->noteOff(notes[i]);
    if (n) light_off(s_auditionPad);
    if (s_cv.audition) lv_timer_pause(s_cv.audition);
}

/// The selected chord for a moment, as the pads would play it.
void cd_audition()
{
    cd_audition_off();
    uint8_t notes[kChordMax] = {};
    const int n = crosspad::chords::notes(s_prog.chords[s_prog.selected], chord_octave(),
                                          s_prog.voicing, notes);
    for (int i = 0; i < n; ++i) notes[i] = (uint8_t)clamp_note(notes[i] + s_transpose);
    s_auditionHeld = pack_chord(notes, n);
    s_auditionPad = s_prog.selected;
    for (int i = 0; i < n; ++i) play_one(notes[i], 100, s_auditionPad);
    light_on(s_auditionPad, 100);
    if (s_cv.audition) {
        lv_timer_reset(s_cv.audition);
        lv_timer_resume(s_cv.audition);
    }
}

void cd_show_strip()
{
    const auto& p = s_prog;
    const uint32_t lit = s_lit.load();
    for (int j = 0; j < kStripSlots; ++j) {
        lv_obj_t* sl = s_cv.slot[j];
        if (!sl) continue;
        const int i = s_cv.page * kStripSlots + j;
        lv_obj_set_style_border_width(sl, i == p.selected && i < p.count ? 2 : 0, 0);
        lv_obj_add_flag(s_cv.slotBar[j], LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(sl, LV_OBJ_FLAG_CLICKABLE);
        if (i < p.count) {
            const bool sounding = (lit & (1u << i)) != 0;
            lv_obj_set_style_bg_color(sl, sounding ? kPress : kPanel, 0);
            lv_label_set_text(s_cv.slotR[j], crosspad::chords::noteName(p.chords[i].root));
            lv_obj_set_style_text_color(s_cv.slotR[j], kText, 0);
            lv_label_set_text(s_cv.slotQ[j], crosspad::chords::qualityName(p.chords[i].quality));
            lv_obj_align(s_cv.slotR[j], LV_ALIGN_TOP_MID, 0, 4);
            if (sounding) lv_obj_remove_flag(s_cv.slotBar[j], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_set_style_bg_color(sl, kWell, 0);
            lv_label_set_text(s_cv.slotQ[j], "");
            if (i == p.count && p.count < crosspad::chords::kMaxChords) {
                lv_label_set_text(s_cv.slotR[j], "+");
                lv_obj_set_style_text_color(s_cv.slotR[j], kDim, 0);
                lv_obj_align(s_cv.slotR[j], LV_ALIGN_CENTER, 0, 0);
            } else {
                lv_label_set_text(s_cv.slotR[j], "");
                lv_obj_remove_flag(sl, LV_OBJ_FLAG_CLICKABLE);
            }
        }
    }
    s_cv.drawnLit = lit;
}

void cd_show()
{
    if (!s_cv.preset) return;
    namespace ch = crosspad::chords;
    const auto& p = s_prog;
    const ch::Chord& c = p.chords[p.selected];
    char b[16];
    const bool presets = !lv_obj_has_flag(s_cv.presets, LV_OBJ_FLAG_HIDDEN);

    prog_label(b, sizeof(b));
    lv_label_set_text(s_cv.presetL, b);
    cd_on(s_cv.preset, presets);
    std::snprintf(b, sizeof(b), "%d", s_oct + 3);   // what the OCT field on the main screen says
    lv_label_set_text(s_cv.octV, b);
    lv_label_set_text(s_cv.voiceV, ch::voicingName(p.voicing));
    lv_label_set_text(s_cv.onV, s_chordsOn ? "ON" : "OFF");
    cd_on(s_cv.onKey, s_chordsOn);

    const uint8_t lastPage = p.count >= kStripSlots ? 1 : 0;
    if (s_cv.page > lastPage) s_cv.page = lastPage;
    cd_show_strip();
    lv_obj_set_style_text_color(s_cv.page1, s_cv.page ? kDim : kAccent, 0);
    lv_obj_set_style_text_color(s_cv.page2, s_cv.page ? kAccent : kDim, 0);

    const uint8_t nat = ch::natural(c.root);
    for (int i = 0; i < 7; ++i) cd_on(s_cv.root[i], kNatural[i] == nat);
    cd_on(s_cv.sharp, ch::isSharp(c.root));
    lv_obj_set_style_opa(s_cv.sharp, ch::canSharp(c.root) ? LV_OPA_COVER : LV_OPA_40, 0);
    for (int i = 0; i < ch::QualityCount; ++i) cd_on(s_cv.type[i], c.quality == i);
    std::snprintf(b, sizeof(b), "INV %u", (unsigned)c.inversion);
    lv_label_set_text(s_cv.invL, b);

    for (uint8_t i = 0; i < ch::presetCount(); ++i) cd_on(s_cv.presetKey[i], p.preset == i && !p.edited);
}

void cd_presets(bool show)
{
    lv_obj_set_flag(s_cv.presets, LV_OBJ_FLAG_HIDDEN, !show);
    lv_obj_set_flag(s_cv.pick, LV_OBJ_FLAG_HIDDEN, show);
}

/// What the pads play has changed: the map, the cells, the LEDs, the field.
void cd_apply()
{
    s_progDirty = true;
    rebuild_pads();
    show_pads();
    show_leds();
}

void cd_ev(lv_event_t* e)
{
    namespace ch = crosspad::chords;
    const intptr_t d = (intptr_t)lv_event_get_user_data(e);
    const int act = (int)(d & 0xFF), i = (int)(d >> 8);
    auto& p = s_prog;
    bool hear = false;
    cd_presets(act == A_PRESET && lv_obj_has_flag(s_cv.presets, LV_OBJ_FLAG_HIDDEN));
    switch (act) {
        case A_PRESET: cd_show(); return;
        case A_LOAD:   ch::loadPreset(p, (uint8_t)i); s_cv.page = 0; hear = true; break;
        case A_OCT:
            /* C2 .. C5 for the chords, round again: a tap, not a turn. */
            set_mapping(s_oct >= 3 ? 0 : s_oct + 1, s_key, s_scale, s_transpose);
            hear = true;
            break;
        case A_VOICE:  ch::cycleVoicing(p); hear = true; break;
        case A_ON:     s_chordsOn = !s_chordsOn; break;
        case A_SLOT: {
            const int idx = s_cv.page * kStripSlots + i;
            if (idx < p.count) ch::select(p, (uint8_t)idx);
            else if (!ch::append(p)) return;
            hear = true;
            break;
        }
        case A_PAGE:
            s_cv.page ^= 1;
            if (s_cv.page * kStripSlots < p.count) p.selected = (uint8_t)(s_cv.page * kStripSlots);
            break;
        case A_ROOT:   ch::setRoot(p, kNatural[i]); hear = true; break;
        case A_SHARP:  ch::toggleSharp(p); hear = true; break;
        case A_TYPE:   ch::setQuality(p, (uint8_t)i); hear = true; break;
        case A_SK:
            if (i == 0) { ch::cycleInversion(p); hear = true; }
            if (i == 1) hear = ch::add(p);
            if (i == 2) ch::remove(p);
            if (i == 3) ch::move(p, -1);
            if (i == 4) ch::move(p, +1);
            break;
        default: return;
    }
    if (act != A_PAGE) s_cv.page = (uint8_t)(p.selected / kStripSlots);
    cd_apply();
    if (hear) cd_audition();
    cd_show();
}

/// A pad played: its chord is the one being edited.
void cd_take_pad()
{
    const uint8_t picked = s_padPicked.exchange(0);
    if (!picked || !s_chordsOn || picked - 1 >= s_prog.count || picked - 1 == s_prog.selected) return;
    crosspad::chords::select(s_prog, (uint8_t)(picked - 1));
    s_progDirty = true;
    show_leds();
    if (s_ov != Overlay::Chord) return;
    s_cv.page = (uint8_t)(s_prog.selected / kStripSlots);
    cd_show();
}

void cd_close()
{
    cd_audition_off();
    if (s_cv.audition) lv_timer_delete(s_cv.audition);
    s_cv = ChordView{};
    save_chords();
}

void chord_open()
{
    namespace ch = crosspad::chords;
    lv_obj_t* o = open_overlay(Overlay::Chord);
    lv_obj_set_layout(o, LV_LAYOUT_NONE);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    s_cv = ChordView{};
    s_cv.page = (uint8_t)(s_prog.selected / kStripSlots);
    s_cv.audition = lv_timer_create(cd_audition_off, kAuditionMs, nullptr);
    lv_timer_pause(s_cv.audition);

    /* CHORD means a pad plays a chord: opening the designer turns them on. */
    if (!s_chordsOn) {
        s_chordsOn = true;
        cd_apply();
    }

    // context row
    s_cv.preset = cd_key(o, 4, 4, 86, 22, A_PRESET, 0);
    s_cv.presetL = cd_text(s_cv.preset, &lv_font_montserrat_14, kText, "");
    lv_obj_center(s_cv.presetL);
    s_cv.octV   = cd_field(94, 48, "OCT", A_OCT);
    s_cv.voiceV = cd_field(146, 92, "VOICE", A_VOICE);
    s_cv.onV    = cd_field(242, 74, "CHORD", A_ON, &s_cv.onKey);

    // strip, roots, types: one box the presets take the place of
    s_cv.pick = cd_box(o, 0, 28, 320, 146, kBg);
    lv_obj_set_style_radius(s_cv.pick, 0, 0);
    for (int j = 0; j < kStripSlots; ++j) {
        lv_obj_t* sl = cd_key(s_cv.pick, 4 + j * 35, 2, 32, 40, A_SLOT, j);
        lv_obj_set_style_border_color(sl, kAccent, 0);
        lv_obj_set_style_clip_corner(sl, true, 0);
        s_cv.slotR[j] = cd_text(sl, &lv_font_montserrat_16, kText, "");
        lv_obj_align(s_cv.slotR[j], LV_ALIGN_TOP_MID, 0, 4);
        s_cv.slotQ[j] = cd_text(sl, &lv_font_montserrat_10, kZone0, "");
        lv_obj_align(s_cv.slotQ[j], LV_ALIGN_TOP_MID, 0, 24);
        s_cv.slotBar[j] = cd_box(sl, 0, 0, 32, 3, kMint);
        lv_obj_set_style_radius(s_cv.slotBar[j], 0, 0);
        s_cv.slot[j] = sl;
    }
    lv_obj_t* pg = cd_key(s_cv.pick, 285, 2, 31, 40, A_PAGE, 0);
    s_cv.page1 = cd_text(pg, &lv_font_montserrat_14, kAccent, "1");
    lv_obj_align(s_cv.page1, LV_ALIGN_TOP_MID, 0, 3);
    s_cv.page2 = cd_text(pg, &lv_font_montserrat_14, kDim, "2");
    lv_obj_align(s_cv.page2, LV_ALIGN_BOTTOM_MID, 0, -3);
    for (int i = 0; i < 7; ++i) {
        s_cv.root[i] = cd_key(s_cv.pick, 4 + i * 39, 46, 36, 30, A_ROOT, i);
        lv_obj_center(cd_text(s_cv.root[i], &lv_font_montserrat_16, kText, ch::noteName(kNatural[i])));
    }
    s_cv.sharp = cd_key(s_cv.pick, 277, 46, 39, 30, A_SHARP, 0);
    lv_obj_center(cd_text(s_cv.sharp, &lv_font_montserrat_16, kText, "#"));
    for (int i = 0; i < ch::QualityCount; ++i) {
        s_cv.type[i] = cd_key(s_cv.pick, 4 + (i % 5) * 63, 80 + (i / 5) * 34, 59, 30, A_TYPE, i);
        lv_obj_center(cd_text(s_cv.type[i], &lv_font_montserrat_16, kText, ch::qualityName((uint8_t)i)));
    }

    // soft keys
    static const char* const kSoft[5] = {"INV 0", "ADD", "DEL", "< MOVE", "MOVE >"};
    for (int i = 0; i < 5; ++i) {
        lv_obj_t* b = cd_key(o, 4 + i * 63, 176, 59, 30, A_SK, i);
        lv_obj_t* l = cd_text(b, &lv_font_montserrat_14, kText, kSoft[i]);
        lv_obj_center(l);
        if (!i) s_cv.invL = l;
    }

    // the presets, in the strip's and pickers' place; the soft keys stay
    s_cv.presets = cd_box(o, 0, 28, 320, 146, kBg);
    lv_obj_set_style_radius(s_cv.presets, 0, 0);
    for (uint8_t i = 0; i < ch::presetCount() && i < 16; ++i) {
        lv_obj_t* b = cd_key(s_cv.presets, 4 + (i % 4) * 79, 2 + (i / 4) * 48, 75, 44, A_LOAD, i);
        lv_obj_t* n = cd_text(b, &lv_font_montserrat_14, kText, ch::presetName((int8_t)i));
        lv_obj_align(n, LV_ALIGN_CENTER, 0, -7);
        char pv[24];
        ch::presetPreview(i, pv, sizeof(pv));
        lv_obj_t* v = cd_text(b, &lv_font_montserrat_10, kDim, pv);
        lv_obj_align(v, LV_ALIGN_CENTER, 0, 9);
        s_cv.presetKey[i] = b;
    }
    cd_presets(false);
    cd_show();
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

/// Opened by Arrange for a Pitch track: this screen is that track's.
bool pitch_track()
{
    const crosspad::ArrangeFocus& f = crosspad::arrangeFocus();
    return f.held() && f.seq && f.source == crosspad::SeqSource::Pitch;
}

/// SEQ on a Pitch track's screen: the track's notes in the sequencer's seq
/// view, the piano roll this screen has no room for. The focus is kept, so
/// that view is the track's and its EDIT comes back here.
void seq_cb(lv_event_t*)
{
    if (s_ov != Overlay::None || !pitch_track()) return;
    crosspad_gui::mode_overlay_open_app("Sequencer");
}

void scale_cb(lv_event_t*)
{
    open_overlay(Overlay::Scale);
    overlay_header("Key and scale");
    build_key_strip();
    scale_row("Key", 0);
    scale_row("Scale", 1);
    scale_row("Transpose", 2);
    lv_obj_t* note = lv_label_create(s_overlay);
    lv_obj_set_width(note, LV_PCT(100));
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(note, kDim, 0);
    lv_obj_set_style_text_font(note, &lv_font_montserrat_10, 0);
    lv_label_set_text(note,
        "The pads are sixteen degrees of the scale from the key, shifted by "
        "the transpose. CHORD puts chords of your own on them instead.");
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
        s_padHeld[pad].store(pack_chord(notes, n));
        if (!n) return;
        for (int i = 0; i < n; ++i) play_one(notes[i], 100, pad);
        light_on(pad, 100);
    } else {
        auto* p = inst();
        uint8_t notes[kChordMax];
        const int n = unpack_chord(s_padHeld[pad].exchange(0), notes);
        for (int i = 0; i < n; ++i) if (p) p->noteOff(notes[i]);
        light_off(pad);
    }
}

bool back_handler(lv_obj_t*)
{
    if (s_ov == Overlay::Edit) { edit_done(); return true; }
    if (s_ov == Overlay::Chord && s_cv.presets && !lv_obj_has_flag(s_cv.presets, LV_OBJ_FLAG_HIDDEN)) {
        cd_presets(false);
        cd_show();
        return true;
    }
    if (s_ov == Overlay::Browse && s_browseForZone) { open_edit(); return true; }
    if (s_ov == Overlay::Sample && s_sampleGroup[0]) {   // out of the group, to the groups
        s_sampleGroup[0] = 0;
        s_samplePage = 0;
        fill_dropdown();
        return true;
    }
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
        s_saveEdits = false;
        show_info(msg.empty() ? "That did not work." : msg.c_str());
        return;
    }
    s_spec = h->instrument();
    s_haveSpec = true;
    if (was == Wait::Load) {
        s_dirty = false;
        s_zonesMoved = false;
        /* One sample: pad 8 plays it as recorded, the pads around it a few
         * semitones either way -- a drum loop detected at F#6 otherwise
         * played three octaves down, a murmur. */
        if (s_followRoot && s_spec.instrument.zones.size() == 1 && s_spec.instrument.zones[0].root >= 0) {
            /* Put the sample's own note under the pads rather than three
             * octaves below them: its pitch class becomes the key and its
             * octave the one the grid starts in. */
            const int root = (int)std::lround(s_spec.instrument.zones[0].root);
            const int oct  = std::max(-2, std::min(4, root / 12 - 3));
            set_mapping(oct, ((root % 12) + 12) % 12, s_scale, s_transpose);
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
        /* An instrument with a file of its own keeps its edits there, as a
         * kit keeps its own: a song names the file, and opening it again
         * played the instrument as it was before the edit. */
        const bool keep = s_saveEdits && state == crosspad::IInstrumentHost::State::Ready && !s_spec.path.empty();
        s_saveEdits = false;
        if (keep && h->requestSave(s_spec)) {
            s_wait = Wait::Save;
            show_info("Saving ...");
        }
    } else {
        s_dirty = false;
        s_zonesMoved = false;
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
            s_waveHasA = s_waveAskA;
            s_waveHasB = s_waveAskB;
        } else {
            /* Kept as asked: forgotten, a file that cannot be read was asked
             * for again every tick, and the tools did nothing else. */
            s_wave = crosspad::WaveOverview{};
            s_wave.file = s_waveWant;
            s_waveHasA = s_waveAskA;
            s_waveHasB = s_waveAskB;
        }
        if (s_waveBox) lv_obj_invalidate(s_waveBox);
        if (s_loopWave) lv_obj_invalidate(s_loopWave);
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

/// Nothing to play yet -- no instrument picked, none in the host, not a Pitch
/// track's screen: the loaded kit's first pad that has a sample, so the keys
/// sound the moment a new player opens this, and the name button says what
/// it is and where to change it.
/// The instrument already up -- a song's, or its Pitch track's in Arrange --
/// is the one this screen shows and edits. Without it the screen said "Pick
/// a sample" over the song's instrument, EDIT began a new one, and the last
/// one shown here was built again in the song's place. One still being built
/// is taken when the host is through (take_host()).
bool take_playing()
{
    auto* h = host();
    if (!h) return false;
    const auto state = h->state();
    if (state == crosspad::IInstrumentHost::State::Busy) {
        s_wait = Wait::Load;
        s_playable.store(false);
        return true;
    }
    if (state != crosspad::IInstrumentHost::State::Ready) return false;
    const crosspad::KitInfo spec = h->instrument();
    if (spec.instrument.zones.empty()) return false;
    s_spec = spec;
    s_haveSpec = true;
    s_dirty = false;
    s_zonesMoved = false;
    s_playable.store(true);
    apply_volume();
    show_pads();
    show_leds();
    show_name();
    show_env();
    return true;
}

void open_kit_sample()
{
    if (s_haveSpec || pitch_track()) return;
    auto* h = host();
    if (!h || h->state() == crosspad::IInstrumentHost::State::Ready) return;
    if (!crosspad::instrumentPlayState().instrumentPath.empty()) return;
    crosspad::IKitManager* kits = crosspad::getKitManager();
    const crosspad::KitInfo* kit = kits ? kits->getCurrentKit() : nullptr;
    if (!kit) return;
    for (size_t i = 0; i < KIT_PADS; ++i) {
        const auto& pad = kit->pads[i];
        if (pad.samples.empty() || pad.samples[0].sample.empty()) continue;
        open_sample(crosspad::resolveKitSamplePath(kit->path, pad.samples[0].sample),
                    kit->name + " pad " + std::to_string(i + 1));
        return;
    }
}

void tick_cb(lv_timer_t*)
{
    take_host();
    take_tools();
    zoom_check();
    if (s_liveWanted && lv_tick_elaps(s_liveAt) >= kLiveMs) apply_zone_live();
    if (s_fillPending) fill_dropdown();
    request_wave();
    show_note();
    show_lit();
    cd_take_pad();
    if (s_ov == Overlay::Chord && s_lit.load() != s_cv.drawnLit) cd_show_strip();
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
    {
        /* One line, cut with dots: a label as tall as its text wraps a long
         * name ("GUITAR Slap Low") onto a second line out of the button. */
        lv_obj_t* l = lv_obj_get_child(s_nameBtn, 0);
        lv_label_set_long_mode(l, LV_LABEL_LONG_DOT);
        lv_obj_set_size(l, 106, lv_font_get_line_height(lv_obj_get_style_text_font(l, LV_PART_MAIN)));
    }

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

    load_chords();
    /* A map someone else published since -- the open song's, which keeps the
     * key, scale, octave and transpose its Pitch track was played in -- is
     * the one the pads take up. */
    bool adopted = false;
    {
        const auto& st = crosspad::instrumentPlayState();
        if (st.generation != 0 && st.generation != s_pubGen && st.play.padMidiMap[kPads - 1] != 0) {
            adopted = true;
            s_key = st.play.rootNote % 12;
            s_oct = std::clamp<int>(st.play.octave, -2, 4);
            s_scale = st.play.scaleIndex < crosspad::MusicTheory::scaleCount() ? st.play.scaleIndex : 0;
            s_transpose = std::clamp<int>(st.transpose, -12, 12);
            s_chordsOn = st.chordsOn;
        }
    }
    if (!adopted && s_oct == 0 && s_key == 0 && s_scale == 0) s_oct = kOctDefault;
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

    /* ── soft keys: the five of the design ───────────────────────────────
     * On a Pitch track's screen the first is SEQ, the way to the track's
     * notes; SCALE is the field at the top as well. */
    {
        const bool track = pitch_track();
        struct { const char* text; lv_event_cb_t cb; void* ud; } keys[5] = {
            { track ? "SEQ " LV_SYMBOL_RIGHT : "SCALE", track ? seq_cb : scale_cb, nullptr },
            { "OCT " LV_SYMBOL_MINUS, oct_cb, (void*)(intptr_t)-1 },
            { "OCT " LV_SYMBOL_PLUS,  oct_cb, (void*)(intptr_t)1  },
            { "CHORD",             chord_cb,  nullptr },
            { "EDIT " LV_SYMBOL_RIGHT, edit_cb, nullptr },
        };
        for (int i = 0; i < 5; ++i) {
            lv_obj_t* b = make_button(s_root, 58, keys[i].text, keys[i].cb, g, keys[i].ud);
            lv_obj_set_height(b, 28);
            place(b, 6 + i * 62, PP_Y(204));
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

    /* One the kit browser picked, or what it played last time again; nothing
     * yet, the kit's first pad. */
    std::string asked;
    if (crosspad::instrumentTakeOpenRequest(asked)) open_instrument(asked);
    else if (take_playing()) {}
    else if (s_haveSpec && !s_spec.instrument.zones.empty()) load_spec();
    else open_kit_sample();

    /* The instrument first: the first thing a player reaches for here is
     * which sound, and everything else on the screen is set against it. */
    if (g && s_nameBtn) lv_group_focus_obj(s_nameBtn);
    s_timer = lv_timer_create(tick_cb, kTickMs, nullptr);

    if (app) {
        app->setOnBack(back_handler);
        app->setOnShow([](lv_obj_t*) {
            if (s_timer) lv_timer_resume(s_timer);
            /* Opened by Arrange for a Pitch track, this is that track's screen:
             * PLAY, REC and DUB are the arrangement's, as in Arrange -- REC
             * records what these keys play onto the track from the playhead
             * (every pad hit reaches the sequencer; its playback presses the
             * pads, and they sound here). A song already playing keeps
             * playing. Opened from MODE the bar is left to whoever holds it. */
            if (pitch_track()) {
                auto* se = crosspad::getPlatformServices().songEngine;
                crosspad::transportClaim(&crosspad::songTransport(), !(se && se->playing()));
            }
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
    if (s_ov == Overlay::Chord) cd_close();
    save_chords();
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
    /* EDIT's widgets go with the root even when it was open: left set, the
     * next start's first picture invalidated a deleted s_loopWave. */
    s_loopLabel = s_loopEndLabel = s_loopWave = s_loopBtn = s_startLabel = s_xfLabel = nullptr;
    for (auto& c : s_keyCell) c = nullptr;
    s_turnField = s_fineField = -1;
    s_viewA = s_viewB = s_waveHasA = s_waveHasB = 0;
    s_liveWanted = s_zonesMoved = false;
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
    if (root) lv_obj_delete_async(root);
}

REGISTER_APP_PL(PitchPlay, nullptr, "pitchplay.png",
                PitchPlay_create, PitchPlay_destroy,
                nullptr, nullptr, nullptr, 0, "PitchPlay")
