use std::cell::RefCell;
use std::collections::HashMap;
use std::ops::Range;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::Arc;
use std::time::Instant;

use crate::engine::frame::Frame;
use crate::engine::input::InputFile;
use crate::engine::record::{Keys, Recorder};
use crate::replay::GhostTrack;
use gearbox_engine::Savestate;
use crate::engine::session::{InputSel, Source};
use crate::engine::display::World;
use gearbox_engine::Controls;
use glam::{Vec2, Vec3};
use gpui::{App, Bounds, Context, FocusHandle, Pixels, Window, actions};

use crate::aux::{self, Aux, GateInfo, Key};
use crate::camera::{Camera, View};
use crate::doc::{self, Doc, Goal, GoalKind, Id, Origin, Reach, Run, StartState, Undo};
use crate::gfx::renderer::{Composite, Output};
use crate::optimize::{self, Search, Spec};
use crate::picker::Picker;
use crate::project::{self, Layout};
use crate::scene::Scene;
use crate::sim::{Sim, Waker};
use crate::stats::Stats;
use crate::ui::widgets::FieldEdit;

actions!(
    editor,
    [
        TogglePlay, StepBack, StepForward, StepBack10, StepForward10, PrevMarker, NextMarker, WindowStart, WindowEnd, UndoAction, RedoAction, DeleteSelection, Duplicate,
        CopyInputs, PasteInputs, LockSelection, UnlockSelection, ClearInputs, ShiftLeft, ShiftRight, ShiftLeft10, ShiftRight10, SelectAllTicks, Escape, ToolSelect, ToolMove, ToolRotate, ToolScale, MergeGoals,
        ToolCar, ToolGate, ToolPoint, ToolBox, ToolPose, ToolLine, Focus, FrameAll, ToggleFollow, ViewTop, ViewFront, ViewRight, Recall1, Recall2, Recall3, Recall4, Recall5, SaveView1, SaveView2,
        SaveView3, SaveView4, SaveView5, TogglePhysics, ToggleStadium, ToggleContacts, ToggleRecord, RunSearch, StopSearch, OpenMap, OpenProject, SaveProject, SaveProjectAs,
        ImportReplay, AddGhost, Cheatsheet, PaintTool, FitTimeline, ZoomIn, ZoomOut, SetValue, Quit, SimulateFromHere, UseSelectionAsWindow, NextField, CommitField, CommandPalette,
        PmUp, PmDown, PmLeft, PmRight, PmOpen, PmRename, PmDuplicate, PmTrash, PmUndo, PmFind, PmBack, PmBrowse, PmShow
    ]
);

pub const PM_KEYS: &str = "Projects && !TextField";

pub const KEYS: &str = "Editor && !TextField && !Fly && !Drive";

pub fn bind_keys(cx: &mut App) {
    use gpui::KeyBinding as K;
    let k = Some(KEYS);
    cx.bind_keys([
        K::new("space", TogglePlay, k),
        K::new("left", StepBack, k),
        K::new("right", StepForward, k),
        K::new("shift-left", StepBack10, k),
        K::new("shift-right", StepForward10, k),
        K::new("cmd-left", PrevMarker, k),
        K::new("cmd-right", NextMarker, k),
        K::new("[", WindowStart, k),
        K::new("]", WindowEnd, k),
        K::new("cmd-z", UndoAction, k),
        K::new("cmd-shift-z", RedoAction, k),
        K::new("backspace", DeleteSelection, k),
        K::new("delete", DeleteSelection, k),
        K::new("cmd-d", Duplicate, k),
        K::new("cmd-j", MergeGoals, k),
        K::new("cmd-c", CopyInputs, k),
        K::new("cmd-v", PasteInputs, k),
        K::new("cmd-l", LockSelection, k),
        K::new("cmd-shift-l", UnlockSelection, k),
        K::new("x", ClearInputs, k),
        K::new("alt-left", ShiftLeft, k),
        K::new("alt-right", ShiftRight, k),
        K::new("alt-shift-left", ShiftLeft10, k),
        K::new("alt-shift-right", ShiftRight10, k),
        K::new("cmd-a", SelectAllTicks, k),
        K::new("escape", Escape, Some("Editor && !TextField")),
        K::new("q", ToolSelect, k),
        K::new("w", ToolMove, k),
        K::new("e", ToolRotate, k),
        K::new("s", ToolScale, k),
        K::new("c", ToolCar, k),
        K::new("1", ToolGate, k),
        K::new("2", ToolPoint, k),
        K::new("3", ToolBox, k),
        K::new("4", ToolPose, k),
        K::new("5", ToolLine, k),
        K::new("f", Focus, k),
        K::new("home", FrameAll, k),
        K::new("shift-f", ToggleFollow, k),
        K::new("alt-1", ViewTop, k),
        K::new("alt-2", ViewFront, k),
        K::new("alt-3", ViewRight, k),
        K::new("ctrl-1", Recall1, k),
        K::new("ctrl-2", Recall2, k),
        K::new("ctrl-3", Recall3, k),
        K::new("ctrl-4", Recall4, k),
        K::new("ctrl-5", Recall5, k),
        K::new("ctrl-shift-1", SaveView1, k),
        K::new("ctrl-shift-2", SaveView2, k),
        K::new("ctrl-shift-3", SaveView3, k),
        K::new("ctrl-shift-4", SaveView4, k),
        K::new("ctrl-shift-5", SaveView5, k),
        K::new("p", TogglePhysics, k),
        K::new("b", ToggleStadium, k),
        K::new("shift-c", ToggleContacts, k),
        K::new("r", ToggleRecord, k),
        K::new("cmd-r", RunSearch, k),
        K::new("cmd-.", StopSearch, k),
        K::new("cmd-o", OpenProject, k),
        K::new("cmd-shift-o", OpenMap, k),
        K::new("cmd-s", SaveProject, k),
        K::new("cmd-shift-s", SaveProjectAs, k),
        K::new("cmd-i", ImportReplay, k),
        K::new("cmd-shift-i", AddGhost, k),
        K::new("shift-/", Cheatsheet, k),
        K::new("?", Cheatsheet, k),
        K::new("v", PaintTool, k),
        K::new("\\", FitTimeline, k),
        K::new("=", ZoomIn, k),
        K::new("-", ZoomOut, k),
        K::new("enter", SetValue, k),
        K::new("cmd-p", SimulateFromHere, k),
        K::new("cmd-w", UseSelectionAsWindow, k),
        K::new("cmd-k", CommandPalette, Some("Editor")),
        K::new("up", PmUp, Some(PM_KEYS)),
        K::new("down", PmDown, Some(PM_KEYS)),
        K::new("left", PmLeft, Some(PM_KEYS)),
        K::new("right", PmRight, Some(PM_KEYS)),
        K::new("enter", PmOpen, Some(PM_KEYS)),
        K::new("cmd-down", PmOpen, Some(PM_KEYS)),
        K::new("f2", PmRename, Some(PM_KEYS)),
        K::new("cmd-d", PmDuplicate, Some(PM_KEYS)),
        K::new("cmd-backspace", PmTrash, Some(PM_KEYS)),
        K::new("cmd-z", PmUndo, Some(PM_KEYS)),
        K::new("cmd-f", PmFind, Some("Projects")),
        K::new("/", PmFind, Some(PM_KEYS)),
        K::new("escape", PmBack, Some(PM_KEYS)),
        K::new("cmd-shift-o", PmBrowse, Some(PM_KEYS)),
        K::new("cmd-shift-r", PmShow, Some(PM_KEYS)),
        K::new("cmd-q", Quit, None),
    ]);
}

pub const SHORTCUTS: &[(&str, &str, &str)] = &[
    ("\u{2318}K", "Search commands and objects", "Edit"),
    ("Right click", "What you can do with the thing under the pointer", "Edit"),
    ("Space", "Play / pause (record mode: record)", "Playback"),
    ("\u{2190} \u{2192}", "Step one tick", "Playback"),
    ("\u{21e7}\u{2190} \u{21e7}\u{2192}", "Step ten ticks", "Playback"),
    ("\u{2318}\u{2190} \u{2318}\u{2192}", "Previous / next marker", "Playback"),
    ("[ ]", "Search window start / end", "Playback"),
    ("\u{2318}P", "Simulate from the start state", "Playback"),
    ("R", "Record mode (drive with arrows/WASD, \u{21e7} half steer)", "Playback"),
    ("MMB, \u{2325}LMB, drag empty space, \u{2325} swipe", "Orbit about the centre", "Camera"),
    ("\u{21e7}MMB, two-finger swipe", "Pan", "Camera"),
    ("Wheel, pinch, \u{2303}MMB, \u{2318} swipe", "Zoom to the cursor", "Camera"),
    ("Hold RMB + WASD Q/E", "Fly (ramps up; \u{21e7} fast, \u{2325} slow, wheel: speed)", "Camera"),
    ("Double-click", "Move the orbit centre to that surface", "Camera"),
    ("F", "Focus the selection", "Camera"),
    ("Home", "Frame the track", "Camera"),
    ("\u{21e7}F", "Follow the car", "Camera"),
    ("\u{2325}1 \u{2325}2 \u{2325}3", "Top / front / right view", "Camera"),
    ("\u{2303}1\u{2026}5 / \u{2303}\u{21e7}1\u{2026}5", "Recall / save camera bookmark", "Camera"),
    ("Q W E S", "Select / Move / Rotate / Scale", "Tools"),
    ("C", "Place car (start state)", "Tools"),
    ("1 2 3 4", "Gate / point / box / pose goal", "Tools"),
    ("5", "Draw a guide line (Esc or double-click ends it)", "Tools"),
    ("V", "Paint inputs (or hold \u{2325} while dragging)", "Inputs"),
    ("\u{2318}A", "Select every tick", "Inputs"),
    ("Enter", "Set the selection's value", "Inputs"),
    ("X", "Clear the selection to neutral", "Inputs"),
    ("\u{2325}\u{2190} \u{2325}\u{2192} (\u{21e7}: 10)", "Shift the selection's inputs", "Inputs"),
    ("\u{2318}C \u{2318}V", "Copy / paste inputs at the cursor", "Inputs"),
    ("\u{2318}L / \u{2318}\u{21e7}L", "Lock / unlock the selection", "Inputs"),
    ("\u{2318}W", "Selection becomes the search window", "Inputs"),
    ("\\", "Fit the selection, or the run, in the timeline", "Inputs"),
    ("= -", "Zoom the timeline in / out", "Inputs"),
    ("\u{2318}R / \u{2318}.", "Run / stop the search", "Search"),
    ("\u{2318}Z / \u{2318}\u{21e7}Z", "Undo / redo", "Edit"),
    ("\u{232b}", "Delete the selected object", "Edit"),
    ("\u{2318}D", "Duplicate the selected goal", "Edit"),
    ("Esc", "Cancel tool, drag or field edit", "Edit"),
    ("\u{2318}O / \u{2318}\u{21e7}O", "Open project / map", "File"),
    ("\u{2318}S / \u{2318}\u{21e7}S", "Save project / save as", "File"),
    ("\u{2318}I / \u{2318}\u{21e7}I", "Import replay inputs / add replay ghost", "File"),
    ("P B \u{21e7}C", "Physics view, stadium, wheel contacts", "View"),
    ("?", "This sheet", "View"),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tool {
    Select,
    Move,
    Rotate,
    Scale,
    PlaceCar,
    GateGoal,
    PointGoal,
    BoxGoal,
    PoseGoal,
    Line,
}

impl Tool {
    pub fn label(self) -> &'static str {
        match self {
            Self::Select => "Select",
            Self::Move => "Move",
            Self::Rotate => "Rotate",
            Self::Scale => "Scale",
            Self::PlaceCar => "Place car",
            Self::GateGoal => "Gate goal",
            Self::PointGoal => "Point goal",
            Self::BoxGoal => "Box goal",
            Self::PoseGoal => "Pose goal",
            Self::Line => "Guide line",
        }
    }
    pub fn key(self) -> &'static str {
        match self {
            Self::Select => "Q",
            Self::Move => "W",
            Self::Rotate => "E",
            Self::Scale => "S",
            Self::PlaceCar => "C",
            Self::GateGoal => "1",
            Self::PointGoal => "2",
            Self::BoxGoal => "3",
            Self::PoseGoal => "4",
            Self::Line => "5",
        }
    }
    pub fn icon(self) -> &'static str {
        match self {
            Self::Select => "icons/select.svg",
            Self::Move => "icons/move.svg",
            Self::Rotate => "icons/rotate.svg",
            Self::Scale => "icons/scale.svg",
            Self::PlaceCar => "icons/car.svg",
            Self::GateGoal => "icons/gate.svg",
            Self::PointGoal => "icons/point.svg",
            Self::BoxGoal => "icons/box.svg",
            Self::PoseGoal => "icons/pose.svg",
            Self::Line => "icons/line.svg",
        }
    }

    pub fn hint(self) -> &'static str {
        match self {
            Self::Select => "Click objects to select them; drag a point goal's handle to lift it.",
            Self::Move => "Drag an axis to move along it, the centre to slide over the surface.",
            Self::Rotate => "Drag a ring: yaw (green), pitch (red), roll (blue).",
            Self::Scale => "Drag an axis's cube to size along it, the centre to size evenly; \u{21e7} snaps to 0.5 m. Boxes, points (radius) and car poses (how close counts).",
            Self::PlaceCar => "The car follows the surface under the cursor; click to place the start state.",
            Self::GateGoal => "Click a checkpoint or finish trigger to make it a goal.",
            Self::PointGoal => "Click a surface to place a point goal (engine raycast); lift it with its handle.",
            Self::BoxGoal => "Click a surface to stand a box trigger on it, across the run; size it in the Inspector.",
            Self::PoseGoal => "Click a surface to place a car pose goal facing along the run.",
            Self::Line => "Click along the way you want to go to draw a guide line; the search can measure misses along it. Esc or a double-click ends the line.",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Sel {
    None,
    Goal(Id),
    Start,
    Current,
    Run(Id),
    Ghost(usize),
    Gate(u32),
    Candidate(u64),
    Guide(Id),
    Setup(Id),
    Route,
}

#[derive(Clone, Debug)]
pub struct Splice {
    pub goal: String,
    pub old_tick: usize,
    pub new_tick: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RunRef {
    Current,
    Previous,
    Run(Id),
    Ghost(usize),
    Candidate(u64),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

pub struct LogEntry {
    pub at: String,
    pub level: LogLevel,
    pub source: &'static str,
    pub text: String,
}

pub struct GhostEntry {
    pub track: GhostTrack,
    pub visible: bool,
}

#[derive(Clone, Debug)]
pub enum Drag {
    Press { at: Vec2, double: bool },
    Orbit { last: Vec2, center: Option<Vec3> },
    Pan { last: Vec2 },
    Zoom { last: Vec2, at: Option<Vec3> },
    Fly { last: Vec2, at: Vec2, t0: std::time::Instant, moved: bool },
    Gizmo { handle: crate::gizmo::Handle, target: GizmoTarget, start: f32, pos0: Vec3, rot0: glam::Quat, size0: Vec3 },
    Velocity { vy: f32 },
    Field { id: String, x0: f32, v0: f64, step: f64, moved: bool },
    Split { which: Split, start: f32, size0: f32 },
    Timeline(crate::ui::dopesheet::TlDrag),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Split {
    Left,
    Right,
    Bottom,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GizmoTarget {
    Goal(Id),
    Start,
}

pub enum Setter {
    Num(Rc<dyn Fn(&mut Editor, f64, &mut Context<Editor>)>),
    Text(Rc<dyn Fn(&mut Editor, String, &mut Context<Editor>)>),
}

#[derive(Default)]
pub struct ProjectsUi {
    pub entries: Vec<project::Entry>,
    pub filter: String,
    pub picking_map: bool,
    pub renaming: Option<PathBuf>,
    pub selected: Option<PathBuf>,
    pub selected_map: Option<PathBuf>,
    pub sort: ProjectSort,
    pub map_sort: MapSort,
    pub new_name: Option<String>,
    pub trashed: Option<(String, Vec<(PathBuf, PathBuf)>, Instant)>,
    pub tab: MapTab,
    pub cards: HashMap<PathBuf, crate::mapcards::MapCard>,
    pub cards_total: usize,
    pub cols: usize,
    pub card_w: f32,
    pub cards_rx: Option<std::sync::mpsc::Receiver<crate::mapcards::MapCard>>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ProjectSort {
    #[default]
    Edited,
    Name,
    Map,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MapSort {
    #[default]
    Newest,
    Name,
    AuthorTime,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MapTab {
    #[default]
    Recent,
    Mine,
    Downloads,
    Test,
}

pub struct Popup {
    pub at: gpui::Point<Pixels>,
    pub items: Vec<(String, Option<String>, Rc<dyn Fn(&mut Editor, &mut Context<Editor>)>)>,
}

#[derive(Clone, Debug, Default)]
pub struct Options {
    pub map: Option<PathBuf>,
    pub inputs: Option<Option<PathBuf>>,
    pub project: Option<PathBuf>,
    pub ghost: Option<PathBuf>,
    pub composite: Option<Composite>,
    pub size: Option<[f32; 2]>,
    pub shot: Option<PathBuf>,
    pub exit: bool,
    pub physics: bool,
    pub stadium: bool,
    pub pick_map: bool,
    pub script: Option<PathBuf>,
    pub smoke: bool,
    pub game: Option<PathBuf>,
    pub record: Option<PathBuf>,
    pub sync: bool,
    pub eye: Option<[f32; 3]>,
    pub look: Option<[f32; 3]>,
}

pub struct Editor {
    pub focus: FocusHandle,
    pub opts: Options,
    pub home: PathBuf,
    pub game: gearbox_packs::install::Game,
    pub waker: Arc<Waker>,
    pub sim: Sim,
    pub aux: Option<Aux>,
    pub picker: Option<Picker>,
    pub origin: Option<Savestate>,
    pub search: Search,
    pub scene: Rc<RefCell<Scene>>,
    pub stats: Stats,
    pub layout: Layout,
    pub map: Option<PathBuf>,
    pub doc: Doc,
    pub undo: Undo,
    pub saved: Option<Doc>,
    pub project_path: Option<PathBuf>,
    pub spec: Spec,
    pub route: crate::lab::RouteSpec,
    pub experiment: crate::lab::Experiment,
    pub palette: Option<crate::ui::menus::Palette>,
    pub job: String,
    pub trick_spans: Vec<crate::lab::TrickSpan>,
    pub setup_ticks: usize,
    pub trick_checks: HashMap<Id, [u32; 3]>,
    pending_doc: Option<Doc>,
    pub race_ms0: i32,
    pub world: Option<Arc<World>>,
    pub world_ready: bool,
    pub world_tris: usize,
    pub gates: Vec<GateInfo>,
    pub gate_names: HashMap<u32, String>,
    gate_names_gen: Option<u64>,
    gate_names_final: bool,
    pub loading: Option<String>,
    pub open_error: Option<String>,
    pub visual_state: Option<String>,
    pub frames_gen: u64,
    pub reach: Vec<Reach>,
    pub reach_gen: u64,
    pub diff_cache: crate::ui::search::DiffCache,
    pub ref_times: Vec<Option<i32>>,
    pub runs_frames: HashMap<Key, Arc<Vec<Frame>>>,
    pub runs_pending: std::collections::HashSet<Key>,
    pub ghosts: Vec<GhostEntry>,
    pub compare: (RunRef, RunRef),
    pub reference: RunRef,
    pub sel: Sel,
    pub tool: Tool,
    pub preview: Option<u64>,
    pub checks: HashMap<u64, String>,
    pub accept_check: Option<(u64, Arc<Vec<Frame>>, usize)>,
    pub camera: Camera,
    pub viewport: Bounds<Pixels>,
    pub vp: glam::Mat4,
    pub drag: Option<Drag>,
    pub hover_handle: Option<crate::gizmo::Handle>,
    pub hover_velocity: bool,
    pub hover_hit: Option<(Vec2, Vec3, Instant)>,
    pub hover_gate: Option<u32>,
    pub mouse: Vec2,
    pub car_preview: Option<StartState>,
    pub fly_keys: [bool; 6],
    pub drawing: Option<Id>,
    pub shift_down: bool,
    pub alt_down: bool,
    pub search_seen: u64,
    pub tries: Arc<Vec<(Vec<[f32; 3]>, crate::optimize::Mark)>>,
    pub tries_gen: u64,
    pub multi: Vec<Sel>,
    pub sel_anchor: Option<Sel>,
    pub search_running: bool,
    pub go_on_pending: bool,
    pub search_key: String,
    pub search_stale: Option<(String, Instant)>,
    pub follow_timer: bool,
    pub auto_preview: bool,
    pub last_paint: Option<Instant>,
    pub last_output: Option<Output>,
    pub present_pending: bool,
    pub dirty: bool,
    pub framed: bool,
    pub scale: f32,
    pub show: Show,
    pub contacts: Option<(usize, [(Vec3, Option<Vec3>); 4])>,
    pub contacts_requested: Option<usize>,
    pub pick_us: Option<u64>,
    pub cursor: usize,
    pub playing: bool,
    pub play_pos: f64,
    pub play_speed: f32,
    pub last_tick: Option<Instant>,
    pub record_mode: bool,
    pub recorder: Option<Recorder>,
    pub drive: Keys,
    pub tl: crate::ui::dopesheet::TlView,
    pub tick_sel: Option<Range<usize>>,
    pub clipboard: Option<Vec<Controls>>,
    pub paint_tool: bool,
    pub set_value: f32,
    pub lanes: Bounds<Pixels>,
    pub editing: Option<FieldEdit>,
    pub setters: HashMap<String, Setter>,
    pub field_value: HashMap<String, String>,
    pub field_order: Vec<String>,
    pub popup: Option<Popup>,
    pub cheatsheet: bool,
    pub collapsed: std::collections::HashSet<String>,
    pub log: Vec<LogEntry>,
    pub status: Option<(String, LogLevel, Instant)>,
    pub field_errors: HashMap<String, String>,
    pub catalog: Vec<PathBuf>,
    pub window_id: Option<isize>,
    pub ui_frame_start: Option<Instant>,
    pub script: Option<crate::script::Script>,
    pub script_ran: Instant,
    pub script_in_frame: bool,
    pub last_undo_merge: Option<(String, Instant)>,
    pub project_name: String,
    pub projects_open: bool,
    pub pm: ProjectsUi,
    saved_spec: Option<crate::optimize::Spec>,
    saved_route: Option<crate::lab::RouteSpec>,
    autosave_at: Option<Instant>,
    save_after_open: bool,
    saved_finishes: Option<project::Finishes>,
    loaded_finishes: HashMap<Id, i32>,
}

#[derive(Clone, Debug)]
pub struct Show {
    pub physics: bool,
    pub stadium: bool,
    pub contacts: bool,
    pub gates: bool,
    pub trail: bool,
    pub ghosts: bool,
    pub tries: bool,
    pub branches: bool,
    pub tricks: bool,
    pub setups: bool,
    pub hidden_families: Vec<crate::materials::Family>,
}

pub fn v2(p: gpui::Point<Pixels>) -> Vec2 {
    Vec2::new(f32::from(p.x), f32::from(p.y))
}

const SEARCH_SETTLE: std::time::Duration = std::time::Duration::from_millis(300);

impl Editor {
    pub fn new(opts: Options, game: gearbox_packs::install::Game, home: PathBuf, scene: Scene, window: &mut Window, cx: &mut Context<Self>) -> Self {
        let (waker, mut rx) = Waker::new();
        let w = waker.clone();
        let sim = Sim::spawn(game.packs.clone(), Arc::new(move || w.wake()));
        let wk = waker.clone();
        if opts.script.is_some() || opts.record.is_some() || opts.shot.is_some() {
            cx.spawn_in(window, async move |this, cx| {
                loop {
                    cx.background_executor().timer(std::time::Duration::from_millis(20)).await;
                    let ran = this.update_in(cx, |e, window, cx| {
                        if e.script_ran.elapsed() > std::time::Duration::from_millis(100) {
                            e.script_in_frame = false;
                            e.advance_unpainted();
                            e.automate(window, cx);
                            window.refresh();
                        }
                    });
                    if ran.is_err() {
                        break;
                    }
                }
            })
            .detach();
        }
        if opts.script.is_some() {
            cx.on_app_quit(|e, _| {
                e.quit_script();
                async {}
            })
            .detach();
        }
        cx.spawn(async move |this, cx| {
            use futures::StreamExt;
            while rx.next().await.is_some() {
                wk.taken();
                if this.update(cx, |app, cx| app.poll(cx)).is_err() {
                    break;
                }
            }
        })
        .detach();
        let focus = cx.focus_handle();
        window.focus(&focus, cx);
        let layout = Layout::load(&home);
        let catalog = crate::catalog::scan_maps(&game);
        let mut app = Self {
            focus,
            home,
            game,
            waker,
            sim,
            aux: None,
            picker: None,
            origin: None,
            search: Search::default(),
            scene: Rc::new(RefCell::new(scene)),
            stats: Stats::default(),
            layout,
            map: None,
            doc: Doc::new(InputFile::blank(-1500), 6000, Origin::Blank),
            undo: Undo::default(),
            saved: None,
            project_path: None,
            spec: Spec::default(),
            route: crate::lab::RouteSpec::default(),
            experiment: crate::lab::Experiment::default(),
            trick_spans: Vec::new(),
            palette: None,
            job: "Search".into(),
            setup_ticks: 150,
            trick_checks: HashMap::new(),
            pending_doc: None,
            race_ms0: -1500,
            world: None,
            world_ready: false,
            world_tris: 0,
            gates: Vec::new(),
            gate_names: HashMap::new(),
            gate_names_gen: None,
            gate_names_final: false,
            loading: None,
            open_error: None,
            visual_state: Some("Loading textured models \u{2026}".into()),
            frames_gen: 0,
            reach: Vec::new(),
            reach_gen: 0,
            diff_cache: Default::default(),
            ref_times: Vec::new(),
            runs_frames: HashMap::new(),
            runs_pending: Default::default(),
            ghosts: Vec::new(),
            compare: (RunRef::Current, RunRef::Previous),
            reference: RunRef::Previous,
            sel: Sel::None,
            tool: Tool::Select,
            preview: None,
            checks: HashMap::new(),
            accept_check: None,
            camera: Camera::default(),
            viewport: Bounds::default(),
            vp: glam::Mat4::IDENTITY,
            drag: None,
            hover_handle: None,
            hover_velocity: false,
            hover_hit: None,
            hover_gate: None,
            mouse: Vec2::ZERO,
            car_preview: None,
            fly_keys: [false; 6],
            drawing: None,
            shift_down: false,
            alt_down: false,
            search_seen: 0,
            tries: Arc::default(),
            tries_gen: 0,
            multi: Vec::new(),
            sel_anchor: None,
            search_running: false,
            go_on_pending: false,
            search_key: String::new(),
            search_stale: None,
            follow_timer: false,
            auto_preview: false,
            last_paint: None,
            last_output: None,
            present_pending: false,
            dirty: true,
            framed: false,
            scale: window.scale_factor(),
            show: Show { physics: opts.physics, stadium: opts.stadium, contacts: false, gates: true, trail: true, ghosts: true, tries: true, branches: false, tricks: true, setups: true, hidden_families: Vec::new() },
            contacts: None,
            contacts_requested: None,
            pick_us: None,
            cursor: 0,
            playing: false,
            play_pos: 0.0,
            play_speed: 1.0,
            last_tick: None,
            record_mode: false,
            recorder: None,
            drive: Keys::default(),
            tl: Default::default(),
            tick_sel: None,
            clipboard: None,
            paint_tool: false,
            set_value: 0.0,
            lanes: Bounds::default(),
            editing: None,
            setters: HashMap::new(),
            field_value: HashMap::new(),
            field_order: Vec::new(),
            popup: None,
            cheatsheet: false,
            collapsed: ["world".to_string()].into_iter().collect(),
            log: Vec::new(),
            status: None,
            field_errors: HashMap::new(),
            catalog,
            window_id: None,
            ui_frame_start: None,
            script: opts.script.clone().or_else(|| opts.record.as_ref().map(|p| p.parent().map_or(PathBuf::from("."), Path::to_path_buf))).map(crate::script::Script::new),
            script_ran: Instant::now(),
            script_in_frame: false,
            last_undo_merge: None,
            project_name: "project".into(),
            projects_open: false,
            pm: ProjectsUi::default(),
            saved_spec: None,
            saved_route: None,
            autosave_at: None,
            save_after_open: false,
            saved_finishes: None,
            loaded_finishes: HashMap::new(),
            opts,
        };
        if app.opts.script.is_some() && app.opts.project.is_none() {
            app.opts.inputs.get_or_insert(None);
        }
        if let Some(p) = app.opts.project.clone() {
            app.open_project(&p, cx);
        } else if app.opts.map.is_none() {
            app.show_projects();
            app.pm.picking_map |= app.opts.pick_map;
        } else {
            let map = app.opts.map.clone().unwrap();
            app.project_name = crate::catalog::map_stem(&map).unwrap_or_else(|| "Project".into());
            let inputs = match &app.opts.inputs {
                Some(Some(p)) => InputSel::File(p.clone()),
                Some(None) => InputSel::New,
                None => crate::catalog::inputs_for_map(&app.home, &map).map_or(InputSel::New, InputSel::File),
            };
            app.open_map(&map, inputs, None, cx);
        }
        if let Some(g) = app.opts.ghost.clone() {
            app.add_ghost(&g);
        }
        app
    }

    pub fn log(&mut self, level: LogLevel, source: &'static str, text: impl Into<String>) {
        let text = text.into();
        if level != LogLevel::Info || source == "search" || source == "io" {
            eprintln!("[{source}] {text}");
        }
        if level != LogLevel::Info || matches!(source, "goal" | "search" | "edit" | "io" | "view" | "start") {
            self.status = Some((text.clone(), level, Instant::now()));
        }
        self.log.push(LogEntry { at: doc::clock(), level, source, text });
        if self.log.len() > 2000 {
            self.log.remove(0);
        }
    }

    pub fn info(&mut self, source: &'static str, text: impl Into<String>) {
        self.log(LogLevel::Info, source, text);
    }

    pub fn error(&mut self, source: &'static str, text: impl Into<String>) {
        self.log(LogLevel::Error, source, text);
    }

    pub fn open_map(&mut self, map: &Path, inputs: InputSel, doc: Option<Doc>, cx: &mut Context<Self>) {
        let same_map = self.map.as_deref() == Some(map);
        self.map = Some(map.to_path_buf());
        self.loading = Some(format!("Building {} from the map and packs \u{2026}", crate::catalog::map_stem(map).unwrap_or_default()));
        self.open_error = None;
        let len = doc.as_ref().map(|d| d.len);
        self.pending_doc = doc;
        self.sim.open(Source(map.to_path_buf()), inputs, len);
        self.world_ready = same_map && self.world_ready;
        if !same_map {
            self.world_ready = false;
            self.framed = false;
            self.gates.clear();
            self.gate_names.clear();
            self.gate_names_gen = None;
            self.gate_names_final = false;
            self.ghosts.clear();
            self.load_visuals(map, cx);
        }
        self.runs_frames.clear();
        self.runs_pending.clear();
        self.preview = None;
        self.info("io", format!("opening {}", map.display()));
        cx.notify();
    }

    fn load_visuals(&mut self, map: &Path, cx: &mut Context<Self>) {
        self.visual_state = Some("Loading textured models \u{2026}".into());
        self.scene.borrow_mut().visual = None;
        let bc = self.scene.borrow().renderer.gpu.bc;
        let (vpacks, vmap) = (self.game.packs.clone(), map.to_path_buf());
        let task = cx.background_executor().spawn(async move { crate::visual::load(&vpacks, &vmap, bc) });
        let scene = self.scene.clone();
        cx.spawn(async move |this, cx| {
            let r = task.await;
            let _ = this.update(cx, |app, cx| {
                match r {
                    Ok(v) => {
                        let g = crate::scene::VisualGpu::upload(&mut scene.borrow_mut().renderer, &v);
                        scene.borrow_mut().visual = Some(g);
                        app.visual_state = None;
                        app.info("view", v.report.clone());
                    }
                    Err(e) => {
                        app.visual_state = Some(format!("Realistic view unavailable: {e:#}"));
                        app.show.physics = true;
                        app.log(LogLevel::Warn, "view", format!("realistic view unavailable: {e:#}"));
                    }
                }
                app.dirty = true;
                cx.notify();
            });
        })
        .detach();
    }

    pub fn open_project(&mut self, path: &Path, cx: &mut Context<Self>) {
        match project::load(path, &self.home) {
            Ok(l) => {
                self.project_path = Some(path.to_path_buf());
                let named = self.pm.entries.iter().find(|e| e.path == path).is_none_or(|e| e.named);
                self.project_name = match self.pm.cards.get(&l.map) {
                    Some(card) if !named => card.name.clone(),
                    _ => l.name.clone(),
                };
                self.save_after_open = self.project_name != l.name;
                self.projects_open = false;
                self.loaded_finishes = l.doc.runs.iter().zip(&l.finishes.runs).filter_map(|(r, f)| Some((r.id, (*f)?))).collect();
                self.saved_finishes = Some(l.finishes.clone());
                if let Some(s) = l.search {
                    self.spec = s;
                }
                self.route = l.route.unwrap_or_default();
                self.saved_spec = Some(self.spec.clone());
                self.saved_route = Some(self.route.clone());
                let inputs = InputSel::Memory((*l.doc.inputs).clone(), l.doc.run_name.clone());
                let map = l.map.clone();
                self.saved = Some(l.doc.clone());
                self.undo = Undo::default();
                self.open_map(&map, inputs, Some(l.doc), cx);
                self.ghosts.clear();
                for g in l.ghosts {
                    self.add_ghost(&g);
                }
                self.info("io", format!("opened project {}", path.display()));
            }
            Err(e) => self.error("io", format!("open project: {e:#}")),
        }
    }

    pub fn save_project(&mut self, path: Option<PathBuf>) {
        let Some(map) = self.map.clone() else { return };
        let path = path.or_else(|| self.project_path.clone()).unwrap_or_else(|| project::new_path(&self.home, &self.project_name));
        let ghosts: Vec<PathBuf> = self.ghosts.iter().map(|g| g.track.path.clone()).collect();
        let finishes = self.finishes();
        match project::save(&path, &self.home, &self.project_name, &finishes, &map, &self.doc, &self.spec, &self.route, &ghosts) {
            Ok(_) => {
                self.project_path = Some(path.clone());
                self.saved_finishes = Some(finishes);
                self.saved = Some(self.doc.clone());
                self.saved_spec = Some(self.spec.clone());
                self.saved_route = Some(self.route.clone());
                self.autosave_at = None;
            }
            Err(e) => self.error("io", format!("save: {e:#}")),
        }
    }

    pub fn finishes(&self) -> project::Finishes {
        let saved = self.saved_finishes.as_ref();
        let run = if self.sim.done() { self.sim.frames.iter().find_map(|f| f.finish).map(|t| t as i32) } else { saved.and_then(|f| f.run) };
        let runs = self.doc.runs.iter().map(|r| self.runs_frames.get(&Key::Run(r.id)).map_or_else(|| self.loaded_finishes.get(&r.id).copied(), |f| f.iter().find_map(|f| f.finish).map(|t| t as i32))).collect();
        project::Finishes { run, runs }
    }

    pub fn dirty_doc(&self) -> bool {
        self.saved.as_ref() != Some(&self.doc) || self.saved_spec.as_ref().is_some_and(|s| *s != self.spec) || self.saved_route.as_ref().is_some_and(|r| *r != self.route)
    }

    fn autosave(&mut self) {
        if self.map.is_none() || self.sim.info.is_none() || self.pending_doc.is_some() || self.opts.shot.is_some() || self.opts.script.is_some() || self.opts.record.is_some() {
            return;
        }
        let new_times = self.project_path.is_some() && self.saved_finishes.as_ref() != Some(&self.finishes());
        if !self.dirty_doc() && !new_times {
            self.autosave_at = None;
            return;
        }
        let now = Instant::now();
        match self.autosave_at {
            None => self.autosave_at = Some(now + std::time::Duration::from_millis(1000)),
            Some(t) if now >= t => self.save_project(None),
            _ => {}
        }
    }

    pub fn show_projects(&mut self) {
        if self.map.is_some() && self.dirty_doc() {
            self.save_project(None);
        }
        self.pm.entries = project::list(&self.home);
        self.pm.picking_map = self.pm.entries.is_empty();
        self.pm.filter.clear();
        self.pm.renaming = None;
        self.pm.selected = self.project_path.clone().filter(|p| self.pm.entries.iter().any(|e| e.path == *p)).or_else(|| self.pm.entries.first().map(|e| e.path.clone()));
        self.projects_open = true;
        self.load_map_cards();
    }

    pub fn show_maps(&mut self, map: Option<PathBuf>) {
        self.show_projects();
        self.pm.picking_map = true;
        self.pm.new_name = None;
        if let Some(m) = map {
            if let Some(card) = self.pm.cards.get(&m) {
                self.pm.tab = match card.source {
                    crate::mapcards::Source::Test => MapTab::Test,
                    _ => MapTab::Recent,
                };
            }
            self.pm.selected_map = Some(m);
        }
    }

    pub fn load_map_cards(&mut self) {
        if self.pm.cards_rx.is_some() || !self.pm.cards.is_empty() {
            return;
        }
        let mut paths: Vec<(bool, std::time::SystemTime, PathBuf)> = self
            .catalog
            .iter()
            .map(|m| {
                let test = crate::catalog::test_maps().is_some_and(|t| m.starts_with(t));
                let t = std::fs::metadata(m).and_then(|x| x.modified()).unwrap_or(std::time::SystemTime::UNIX_EPOCH);
                (test, t, m.clone())
            })
            .collect();
        paths.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.cmp(&a.1)));
        let mut order: Vec<PathBuf> = Vec::new();
        for p in self.pm.entries.iter().map(|e| e.map.clone()).chain(paths.into_iter().map(|p| p.2)) {
            if !order.contains(&p) && p.exists() {
                order.push(p);
            }
        }
        self.pm.cards_total = order.len();
        let w = self.waker.clone();
        self.pm.cards_rx = Some(crate::mapcards::load(order, Arc::new(move || w.wake())));
    }

    pub fn new_project(&mut self, map: &Path, name: Option<String>, cx: &mut Context<Self>) {
        if self.dirty_doc() {
            self.save_project(None);
        }
        let named = name.map(|n| n.trim().to_string()).filter(|n| !n.is_empty());
        self.project_name = named.or_else(|| self.pm.cards.get(map).map(|c| c.name.clone())).or_else(|| crate::catalog::map_stem(map)).unwrap_or_else(|| "Project".into());
        self.saved_finishes = None;
        self.loaded_finishes.clear();
        self.project_path = Some(project::new_path(&self.home, &self.project_name));
        self.projects_open = false;
        self.save_after_open = true;
        let inputs = crate::catalog::inputs_for_map(&self.home, map).map_or(InputSel::New, InputSel::File);
        self.open_map(map, inputs, None, cx);
    }

    pub fn open_listed(&mut self, path: &Path, cx: &mut Context<Self>) {
        if self.dirty_doc() {
            self.save_project(None);
        }
        self.open_project(path, cx);
    }

    pub fn rename_project(&mut self, path: &Path, name: &str) {
        let name = name.trim();
        if name.is_empty() {
            return;
        }
        if self.project_path.as_deref() == Some(path) {
            self.project_name = name.to_string();
            self.save_project(None);
        } else if let Err(e) = project::rename(path, name) {
            self.error("io", format!("rename: {e:#}"));
        }
        self.pm.entries = project::list(&self.home);
    }

    pub fn duplicate_project(&mut self, path: &Path) {
        if self.project_path.as_deref() == Some(path) && self.dirty_doc() {
            self.save_project(None);
        }
        match project::duplicate(path, &self.home) {
            Ok(_) => self.pm.entries = project::list(&self.home),
            Err(e) => self.error("io", format!("duplicate: {e:#}")),
        }
    }

    pub fn trash_project(&mut self, path: &Path) {
        let name = self.pm.entries.iter().find(|e| e.path == path).map_or_else(|| path.display().to_string(), |e| e.name.clone());
        let at = self.pm.entries.iter().position(|e| e.path == path);
        match project::trash(path) {
            Ok(moved) => {
                if self.project_path.as_deref() == Some(path) {
                    self.project_path = None;
                    self.saved = None;
                    self.saved_spec = None;
                    self.saved_route = None;
                    self.saved_finishes = None;
                    self.map = None;
                }
                self.pm.trashed = Some((name, moved, Instant::now()));
                self.pm.entries = project::list(&self.home);
                if self.pm.selected.as_deref() == Some(path) {
                    self.pm.selected = at.and_then(|i| self.pm.entries.get(i.min(self.pm.entries.len().saturating_sub(1)))).map(|e| e.path.clone());
                }
            }
            Err(e) => self.error("io", format!("move to Trash: {e:#}")),
        }
    }

    pub fn untrash_project(&mut self) {
        let Some((name, moved, _)) = self.pm.trashed.take() else { return };
        match project::restore(&moved) {
            Ok(()) => {
                self.pm.entries = project::list(&self.home);
                self.pm.selected = moved.iter().map(|(_, to)| if to.is_dir() { to.join("project.tmproj.json") } else { to.clone() }).find(|p| self.pm.entries.iter().any(|e| e.path == *p));
                self.info("io", format!("put {name} back"));
            }
            Err(e) => self.error("io", format!("put back: {e:#}")),
        }
    }

    pub fn poll(&mut self, cx: &mut Context<Self>) {
        let ch = self.sim.poll();
        for e in ch.errors {
            self.error("sim", e);
        }
        if let Some(e) = ch.failed {
            self.loading = None;
            self.open_error = Some(e.to_string());
            self.error("sim", format!("open failed: {e}"));
        }
        if let Some(o) = ch.opened {
            self.opened(o, cx);
        }
        if ch.frames {
            self.frames_gen += 1;
            self.trick_spans = crate::lab::trick_spans(&self.sim.frames);
            self.cursor = self.cursor.max(self.race_start_tick()).min(self.doc.len);
            self.dirty = true;
            self.reevaluate();
            self.frame_run_once();
            self.check_accept();
        }
        if ch.probe {
            cx.notify();
        }
        let picked = self.picker.as_ref().map(Picker::take).unwrap_or_default();
        for p in picked {
            self.picked(p);
        }
        let done = self.aux.as_ref().map(Aux::take).unwrap_or_default();
        for d in done {
            match d {
                aux::Done::Gates(Ok(g)) => {
                    self.info("world", format!("{} gates ({} checkpoints, {} finishes)", g.len(), g.iter().filter(|g| g.gate.kind != 1).count(), g.iter().filter(|g| g.gate.kind == 1).count()));
                    self.gates = g;
                    self.merge_finish_goals();
                }
                aux::Done::Gates(Err(e)) => self.error("world", format!("gates: {e}")),
                aux::Done::Frames { key, frames, micros, error } => {
                    self.runs_pending.remove(&key);
                    if let Some(e) = error {
                        self.log(LogLevel::Warn, "sim", format!("{key:?}: {e}"));
                    }
                    if let Key::Candidate(id) = key {
                        self.candidate_simulated(id, &frames, micros);
                    }
                    self.runs_frames.insert(key, frames);
                    self.dirty = true;
                }
            }
        }
        let (tries_gen, newest) = {
            let l = self.search.live.lock().unwrap();
            if l.tries_gen != self.tries_gen {
                self.tries = Arc::new(l.tries.iter().cloned().collect());
                self.dirty = true;
            }
            (l.tries_gen, l.candidates.last().map(|c| c.id))
        };
        self.tries_gen = tries_gen;
        let (running, best) = {
            let l = self.search.live.lock().unwrap();
            let better = l.best.as_ref().zip(l.base.as_ref()).is_some_and(|(b, base)| b.quality_cmp(base).is_gt());
            (l.running, (better && l.message.starts_with("done")).then(|| l.candidates.last().map(|c| c.id)).flatten())
        };
        if self.search_running && !running && self.spec.go_on
            && let Some(id) = best
        {
            self.go_on(id);
        }
        self.search_running = running;
        if self.go_on_pending && self.sim.done() {
            self.go_on_pending = false;
            self.start_search();
        }
        if let Some(id) = newest
            && id > self.search_seen
        {
            self.search_seen = id;
            let _ = self.ref_frames(RunRef::Candidate(id));
            if self.preview.is_none() || self.auto_preview {
                self.preview = Some(id);
                self.auto_preview = true;
                self.dirty = true;
            }
        }
        self.name_gates();
        self.autosave();
        if let Some(rx) = &self.pm.cards_rx {
            while let Ok(c) = rx.try_recv() {
                self.pm.cards.insert(c.path.clone(), c);
            }
        }
        cx.notify();
    }

    fn opened(&mut self, o: crate::sim::Opened, cx: &mut Context<Self>) {
        let old_ms0 = self.race_ms0;
        self.race_ms0 = self.sim.info.as_ref().map_or(-1500, |i| i.race_ms0);
        let shift = (old_ms0 - self.race_ms0) / 10;
        let label = self.sim.info.as_ref().map_or(String::new(), |i| i.inputs_label.clone());
        match self.pending_doc.take() {
            Some(mut d) => {
                if shift != 0 {
                    let mv = |k: usize| (k as i64 + shift as i64).max(0) as usize;
                    if let Some(s) = &mut d.start {
                        s.tick = mv(s.tick);
                    }
                    d.locks = d.locks.iter().map(|r| mv(r.start)..mv(r.end)).collect();
                    self.spec.window = mv(self.spec.window.start)..mv(self.spec.window.end);
                }
                d.inputs = Arc::new(o.inputs.clone());
                d.len = o.len;
                self.doc = d;
                self.merge_finish_goals();
            }
            None => {
                let origin = if label.starts_with("new") { Origin::Blank } else { Origin::Inputs(label) };
                self.doc = Doc::new(o.inputs.clone(), o.len, origin);
                self.undo = Undo::default();
                self.saved = Some(self.doc.clone());
                let start = ((-self.race_ms0).max(0) / 10) as usize;
                self.spec.window = start..(start + 300).min(o.len);
                self.saved_spec = Some(self.spec.clone());
                self.saved_route = Some(self.route.clone());
                self.cursor = 0;
            }
        }
        if let Some(e) = &o.world_error {
            self.error("world", format!("collision world: {e}"));
        }
        let w = self.waker.clone();
        self.picker = Some(Picker::spawn(o.origin.clone(), Arc::new(move || w.wake())));
        let w = self.waker.clone();
        let aux = Aux::spawn(o.origin.clone(), Arc::new(move || w.wake()));
        aux.send(aux::Job::Gates);
        self.aux = Some(aux);
        self.origin = Some(o.origin);
        if let Some(s) = &self.doc.start {
            self.sim.set_start(Some((s.tick, s.placement())));
        }
        self.world_tris = o.world.tris.len();
        self.world = Some(o.world.clone());
        self.rebuild_world_mesh(cx);
        self.info("sim", format!("opened {} \u{00b7} race time at tick 0: {} ms \u{00b7} {} ticks", self.sim.info.as_ref().map_or("", |i| i.stem.as_str()), self.race_ms0, o.len));
        if std::mem::take(&mut self.save_after_open) {
            self.save_project(None);
        }
    }

    pub fn rebuild_world_mesh(&mut self, cx: &mut Context<Self>) {
        let Some(world) = self.world.clone() else { return };
        self.loading.get_or_insert_with(|| "Meshing the collision world \u{2026}".into());
        let hidden = self.show.hidden_families.clone();
        let scene = self.scene.clone();
        let task = cx.background_executor().spawn(async move {
            let mesh = crate::scene::build_world(&world, &hidden);
            (mesh, Vec3::from(world.min), Vec3::from(world.max))
        });
        cx.spawn(async move |this, cx| {
            let (mesh, min, max) = task.await;
            let _ = this.update(cx, |app, cx| {
                scene.borrow_mut().set_world(&mesh, min, max);
                app.world_ready = true;
                app.loading = None;
                if !app.framed {
                    app.camera.frame(min, max);
                    app.camera.snap_to(app.camera.goal);
                }
                app.frame_run_once();
                app.dirty = true;
                cx.notify();
            });
        })
        .detach();
    }

    fn frame_run_once(&mut self) {
        if self.framed || !self.sim.done() || !self.world_ready {
            return;
        }
        self.framed = true;
        self.tl.fit = true;
        if let Some((min, max)) = self.run_bounds() {
            self.camera.frame(min, max);
        }
        if let (Some(eye), Some(look)) = (self.opts.eye, self.opts.look) {
            self.camera.look_at(Vec3::from(eye), Vec3::from(look));
        }
    }

    pub fn run_bounds(&self) -> Option<(Vec3, Vec3)> {
        if self.sim.frames.len() > 10 {
            let (mut lo, mut hi) = (Vec3::splat(f32::MAX), Vec3::splat(f32::MIN));
            for f in &self.sim.frames {
                lo = lo.min(Vec3::from(f.position()));
                hi = hi.max(Vec3::from(f.position()));
            }
            let pad = Vec3::splat(((hi - lo).length() * 0.08).max(20.0));
            return Some((lo - pad, hi + pad));
        }
        self.scene.borrow().bounds
    }

    pub fn edit(&mut self, label: &str, merge: Option<&str>, f: impl FnOnce(&mut Doc)) {
        let before = self.doc.clone();
        f(&mut self.doc);
        if self.doc == before {
            return;
        }
        let merged = match (merge, &self.last_undo_merge) {
            (Some(k), Some((last, at))) => k == last && at.elapsed().as_secs_f32() < 1.0,
            _ => false,
        };
        if !merged {
            self.undo.push(label, before.clone());
        }
        self.last_undo_merge = merge.map(|k| (k.to_string(), Instant::now()));
        self.sync(&before);
    }

    pub fn sync(&mut self, before: &Doc) {
        let d = &self.doc;
        if d.start != before.start {
            self.sim.set_start(d.start.as_ref().map(|s| (s.tick, s.placement())));
        }
        if !Arc::ptr_eq(&d.inputs, &before.inputs) || d.len != before.len {
            let from = doc::first_difference(&before.inputs, &d.inputs, self.race_ms0).unwrap_or(d.len.min(before.len));
            self.sim.simulate(d.inputs.clone(), from, d.len, self.recorder.as_ref().map(|_| self.cursor + 1));
        }
        if d.goals != before.goals || d.guides != before.guides {
            self.reevaluate();
        }
        self.dirty = true;
    }

    pub fn undo(&mut self) {
        if let Some((label, prev)) = self.undo.undo.pop() {
            let cur = std::mem::replace(&mut self.doc, prev);
            self.undo.redo.push((label.clone(), cur.clone()));
            self.last_undo_merge = None;
            self.sync(&cur);
            self.info("edit", format!("undo: {label}"));
        }
    }

    pub fn redo(&mut self) {
        if let Some((label, next)) = self.undo.redo.pop() {
            let cur = std::mem::replace(&mut self.doc, next);
            self.undo.undo.push((label.clone(), cur.clone()));
            self.last_undo_merge = None;
            self.sync(&cur);
            self.info("edit", format!("redo: {label}"));
        }
    }

    pub fn replace_run(&mut self, label: &str, inputs: InputFile, origin: Origin) {
        self.edit(label, None, |d| {
            d.remember(label);
            d.inputs = Arc::new(inputs);
            d.run_origin = origin;
        });
    }

    pub fn set_ticks(&mut self, label: &str, merge: Option<&str>, ticks: impl Iterator<Item = (usize, Controls)>) {
        let ms0 = self.race_ms0;
        let changes: Vec<(usize, Controls)> = ticks.collect();
        self.edit(label, merge, |d| {
            let mut f = (*d.inputs).clone();
            for (k, c) in changes {
                if !d.locked(k) {
                    f.set(ms0 + 10 * k as i32, c);
                }
            }
            d.inputs = Arc::new(f);
            d.run_origin = Origin::Edit;
        });
    }

    pub fn control(&self, k: usize) -> Controls {
        doc::control_at(&self.doc.inputs, self.race_ms0, k)
    }

    pub fn reevaluate(&mut self) {
        self.reach = doc::evaluate_each(&self.doc.goals, &self.sim.frames);
        let order = doc::run_order(&self.doc.goals, &self.reach, &self.doc.guides);
        if order.iter().enumerate().any(|(k, &i)| k != i) && order.len() == self.reach.len() {
            self.doc.goals = order.iter().map(|&i| self.doc.goals[i].clone()).collect();
            self.reach = order.iter().map(|&i| self.reach[i]).collect();
        }
        self.reach_gen += 1;
        self.ref_times = self.ref_reach(self.reference).unwrap_or_default();
    }

    pub fn ref_frames(&mut self, r: RunRef) -> Option<Arc<Vec<Frame>>> {
        let (key, inputs, len, start) = match r {
            RunRef::Current => return Some(Arc::new(self.sim.frames.clone())),
            RunRef::Ghost(_) => return None,
            RunRef::Previous => {
                let run = self.doc.history.last()?;
                (Key::Run(run.id), run.inputs.clone(), run.len, run.start.clone())
            }
            RunRef::Run(id) => {
                let run = self.doc.run(id)?;
                (Key::Run(id), run.inputs.clone(), run.len, run.start.clone())
            }
            RunRef::Candidate(id) => {
                let (c, start) = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).map(|c| (c.inputs.clone(), c.start.clone()))?;
                (Key::Candidate(id), c, self.doc.len, start.or_else(|| self.doc.start.clone()))
            }
        };
        if let Some(f) = self.runs_frames.get(&key) {
            return Some(f.clone());
        }
        if self.runs_pending.insert(key)
            && let Some(a) = &self.aux
        {
            a.send(aux::Job::Simulate { key, inputs, len, start: start.map(|s| (s.tick, s.placement())) });
        }
        None
    }

    pub fn ref_label(&self, r: RunRef) -> String {
        match r {
            RunRef::Current => format!("{} (current)", self.doc.run_name),
            RunRef::Previous => self.doc.history.last().map_or("previous version (none yet)".into(), |h| format!("previous version ({})", h.at)),
            RunRef::Run(id) => self.doc.run(id).map_or("?".into(), |r| r.name.clone()),
            RunRef::Ghost(i) => self.ghosts.get(i).map_or("?".into(), |g| format!("ghost {}", g.track.name)),
            RunRef::Candidate(id) => format!("candidate #{id}"),
        }
    }

    pub fn ref_reach(&mut self, r: RunRef) -> Option<Vec<Option<i32>>> {
        if let RunRef::Ghost(i) = r {
            let g = &self.ghosts.get(i)?.track;
            return Some(crate::ui::compare::ghost_reach(&self.doc.goals, g, self.race_ms0));
        }
        let f = self.ref_frames(r)?;
        Some(doc::evaluate(&self.doc.goals, &f).iter().map(|x| x.ms).collect())
    }

    pub fn select(&mut self, s: Sel) {
        self.sel = s;
        self.multi.clear();
        self.sel_anchor = Some(s);
        self.dirty = true;
        if !matches!(s, Sel::Candidate(_) | Sel::None) {
            self.layout.right_tab = 0;
        }
        if let Sel::Candidate(id) = s {
            self.preview = Some(id);
            self.auto_preview = false;
            let _ = self.ref_frames(RunRef::Candidate(id));
        }
    }

    pub fn add_goal(&mut self, kind: GoalKind) -> Id {
        let prefix = match &kind {
            GoalKind::Point { .. } => "P",
            GoalKind::Pose(_) => "Pose ",
            GoalKind::Box { .. } => "Box ",
            GoalKind::Gate(g) if g.kind == 1 => "Finish",
            GoalKind::Gate(_) => "CP",
        };
        let mut id = 0;
        let name = match &kind {
            GoalKind::Gate(g) => self.gate_name(g.waypoint),
            _ => self.doc.goal_name(prefix),
        };
        self.edit("add goal", None, |d| {
            id = d.id();
            d.goals.push(Goal { id, name, kind, visible: true, when: Default::default() });
        });
        self.select(Sel::Goal(id));
        id
    }

    pub fn finish_groups(&self) -> Vec<Vec<u32>> {
        let fins: Vec<&gearbox_search::gate::Gate> = self.gates.iter().map(|g| &g.gate).filter(|g| g.kind == 1).collect();
        doc::touching(&fins)
    }

    pub fn gate_goal(&self, w: u32) -> Option<doc::GateGoal> {
        let g = self.gates.iter().find(|g| g.gate.waypoint == w)?;
        let also: Vec<u32> = if g.gate.kind == 1 { self.finish_groups().into_iter().find(|f| f.contains(&w)).unwrap_or_default().into_iter().filter(|x| *x != w).collect() } else { Vec::new() };
        let boxes = std::iter::once(&g.gate).chain(self.gates.iter().map(|g| &g.gate).filter(|g| also.contains(&g.waypoint))).flat_map(|g| g.boxes.iter().cloned()).collect();
        Some(doc::GateGoal { waypoint: w, kind: g.gate.kind, boxes, ordinal: None, also, parts: Vec::new() })
    }

    fn merge_finish_goals(&mut self) {
        let mut renamed: Vec<(Id, Id)> = Vec::new();
        let mut doc = self.doc.clone();
        for group in self.finish_groups() {
            let ids: Vec<Id> = doc.goals.iter().filter(|g| matches!(&g.kind, GoalKind::Gate(x) if group.iter().any(|w| x.has(*w)))).map(|g| g.id).collect();
            let Some(&keep) = ids.first() else { continue };
            let GoalKind::Gate(x) = &doc.goal(keep).expect("listed goal").kind else { continue };
            let Some(full) = self.gate_goal(x.waypoint) else { continue };
            let ordinal = x.ordinal;
            doc.goals.retain(|g| g.id == keep || !ids.contains(&g.id));
            if let Some(g) = doc.goals.iter_mut().find(|g| g.id == keep) {
                g.kind = GoalKind::Gate(doc::GateGoal { ordinal, ..full });
            }
            renamed.extend(ids[1..].iter().map(|id| (*id, keep)));
        }
        if doc == self.doc {
            return;
        }
        self.edit("one goal per finish", None, |d| *d = doc);
        let s = &mut self.spec;
        for (from, to) in renamed {
            for list in [&mut s.goals, &mut s.by_speed, &mut s.both] {
                for id in list.iter_mut().filter(|id| **id == from) {
                    *id = to;
                }
                let mut seen = Vec::new();
                list.retain(|id| !seen.contains(id) && {
                    seen.push(*id);
                    true
                });
            }
        }
    }

    pub fn gate_name(&self, waypoint: u32) -> String {
        if let Some(n) = self.gate_names.get(&waypoint) {
            return n.clone();
        }
        let Some(g) = self.gates.iter().find(|g| g.gate.waypoint == waypoint) else { return format!("Gate {waypoint}") };
        if g.gate.kind == 1 {
            let fins = self.finish_groups();
            if fins.len() > 1 {
                let i = fins.iter().position(|f| f.contains(&waypoint)).unwrap_or(0);
                return format!("Finish {}", i + 1);
            }
            return "Finish".into();
        }
        let mut cps: Vec<(u32, u32)> = self.gates.iter().filter(|g| g.gate.kind != 1).map(|g| (g.order.unwrap_or(u32::MAX), g.gate.waypoint)).collect();
        cps.sort();
        let ordered = cps.iter().all(|c| c.0 != u32::MAX);
        match cps.iter().position(|c| c.1 == waypoint) {
            Some(i) if ordered => format!("CP{}", i + 1),
            _ => format!("CP w{waypoint}"),
        }
    }

    fn name_gates(&mut self) {
        if self.gate_names_final || self.gates.is_empty() || !self.sim.done() || self.gate_names_gen == Some(self.frames_gen) {
            return;
        }
        self.gate_names_gen = Some(self.frames_gen);
        let mut first: HashMap<u32, u32> = HashMap::new();
        for f in &self.sim.frames {
            if let Some((w, t)) = f.last_crossing {
                first.entry(w).or_insert(t);
            }
        }
        let mut names = HashMap::new();
        let mut cps: Vec<(bool, u32, u32, u32)> = self.gates.iter().filter(|g| g.gate.kind != 1).map(|g| (!first.contains_key(&g.gate.waypoint), first.get(&g.gate.waypoint).copied().unwrap_or(0), g.order.unwrap_or(u32::MAX), g.gate.waypoint)).collect();
        cps.sort();
        for (i, g) in cps.into_iter().enumerate() {
            names.insert(g.3, format!("CP{}", i + 1));
        }
        let mut fins: Vec<((bool, u32, u32), Vec<u32>)> = self
            .finish_groups()
            .into_iter()
            .map(|f| (f.iter().map(|w| (!first.contains_key(w), first.get(w).copied().unwrap_or(0), *w)).min().unwrap_or((true, 0, 0)), f))
            .collect();
        fins.sort();
        let n = fins.len();
        for (i, (_, f)) in fins.into_iter().enumerate() {
            for w in f {
                names.insert(w, if n == 1 { "Finish".to_string() } else { format!("Finish {}", i + 1) });
            }
        }
        let old = std::mem::replace(&mut self.gate_names, names);
        for g in &mut self.doc.goals {
            if let GoalKind::Gate(x) = &g.kind {
                let was = old.get(&x.waypoint).cloned();
                if was.as_deref() == Some(g.name.as_str()) || (was.is_none() && (g.name.starts_with("CP") || g.name.starts_with("Finish"))) {
                    g.name = self.gate_names.get(&x.waypoint).cloned().unwrap_or_else(|| g.name.clone());
                }
            }
        }
        self.gate_names_final = !first.is_empty();
    }

    pub fn go_on(&mut self, id: u64) {
        self.accept(id);
        let w = self.spec.window.clone();
        let len = w.len().max(1);
        if w.end + 1 >= self.doc.len {
            return self.info("search", "went on to the end of the run");
        }
        self.spec.window = w.end..(w.end + len).min(self.doc.len);
        if let Some(at) = self.spec.at_tick {
            self.spec.at_tick = Some(at + len);
        }
        self.job = format!("Going on from {} s", crate::fmt::fmt_ms(self.race_ms0 + 10 * w.end as i32));
        self.go_on_pending = true;
    }

    pub fn toggle(&mut self, s: Sel) {
        let mut set = std::mem::take(&mut self.multi);
        if set.is_empty() && matches!(self.sel, Sel::Goal(_) | Sel::Gate(_)) {
            set.push(self.sel);
        }
        if let Some(i) = set.iter().position(|x| *x == s) {
            set.remove(i);
        } else {
            set.push(s);
        }
        self.sel = set.last().copied().unwrap_or(Sel::None);
        if set.len() > 1 {
            self.multi = set;
        }
        self.sel_anchor = Some(s);
        self.dirty = true;
    }

    pub fn select_range(&mut self, s: Sel, order: &[Sel]) {
        let from = self.sel_anchor.and_then(|a| order.iter().position(|x| *x == a));
        let Some((a, b)) = from.zip(order.iter().position(|x| *x == s)) else { return self.select(s) };
        let anchor = self.sel_anchor;
        let range = order[a.min(b)..=a.max(b)].to_vec();
        self.select(s);
        self.sel_anchor = anchor;
        if range.len() > 1 {
            self.multi = range;
        }
    }

    pub fn selected(&self) -> Vec<Sel> {
        if !self.multi.is_empty() {
            return self.multi.clone();
        }
        if matches!(self.sel, Sel::Goal(_) | Sel::Gate(_)) { vec![self.sel] } else { Vec::new() }
    }

    pub fn merge_selected(&mut self) {
        let mut picks: Vec<(Option<Id>, Vec<doc::GatePart>)> = Vec::new();
        for s in self.selected() {
            let goal = match s {
                Sel::Goal(id) => Some(id),
                Sel::Gate(w) => self.doc.goals.iter().find(|g| matches!(&g.kind, GoalKind::Gate(x) if x.has(w))).map(|g| g.id),
                _ => None,
            };
            if goal.is_some() && picks.iter().any(|p| p.0 == goal) {
                continue;
            }
            let parts = match (goal.and_then(|id| self.doc.goal(id)), s) {
                (Some(g), _) => match &g.kind {
                    GoalKind::Gate(x) => x.split(&g.name),
                    _ => return self.error("goals", format!("{} is not a gate: only gates merge", g.name)),
                },
                (None, Sel::Gate(w)) => match self.gate_goal(w) {
                    Some(x) => vec![doc::GatePart { name: self.gate_name(w), gate: x }],
                    None => continue,
                },
                _ => continue,
            };
            picks.push((goal, parts));
        }
        if picks.len() < 2 {
            return self.error("goals", "select two or more gates or gate goals to merge (\u{2318}/\u{21e7}-click in the outliner)");
        }
        let parts: Vec<doc::GatePart> = picks.iter().flat_map(|p| p.1.iter().cloned()).collect();
        if parts.iter().any(|p| p.gate.kind != parts[0].gate.kind) {
            return self.error("goals", "only gates of one kind merge: finishes with finishes, checkpoints with checkpoints");
        }
        let name = doc::parts_name(&parts);
        let merged = doc::GateGoal::of_parts(parts);
        let goals: Vec<Id> = picks.iter().filter_map(|p| p.0).collect();
        let keep = match goals.first() {
            Some(&id) => {
                self.edit("merge gates", None, |d| {
                    if let Some(g) = d.goals.iter_mut().find(|g| g.id == id) {
                        (g.kind, g.name) = (GoalKind::Gate(merged), name.clone());
                    }
                    d.goals.retain(|g| g.id == id || !goals.contains(&g.id));
                });
                id
            }
            None => {
                let id = self.add_goal(GoalKind::Gate(merged));
                self.edit("name merged goal", None, |d| {
                    if let Some(g) = d.goal_mut(id) {
                        g.name = name.clone();
                    }
                });
                id
            }
        };
        let spec = &mut self.spec.goals;
        let mut placed = spec.contains(&keep);
        spec.retain_mut(|g| {
            if *g != keep && goals.contains(g) {
                if placed {
                    return false;
                }
                (*g, placed) = (keep, true);
            }
            true
        });
        self.select(Sel::Goal(keep));
        self.info("goals", format!("merged into {name}"));
    }

    pub fn unmerge(&mut self, id: Id) {
        let Some(GoalKind::Gate(g)) = self.doc.goal(id).map(|g| g.kind.clone()) else { return };
        if g.parts.len() < 2 {
            return;
        }
        let mut ids = Vec::new();
        self.edit("unmerge", None, |d| {
            let Some(i) = d.goals.iter().position(|x| x.id == id) else { return };
            let base = d.goals[i].clone();
            for (j, p) in g.parts.iter().enumerate() {
                let goal = doc::Goal { id: if j == 0 { id } else { d.id() }, name: p.name.clone(), kind: GoalKind::Gate(p.gate.clone()), ..base.clone() };
                ids.push(goal.id);
                if j == 0 {
                    d.goals[i] = goal;
                } else {
                    d.goals.insert(i + j, goal);
                }
            }
        });
        if let Some(i) = self.spec.goals.iter().position(|g| *g == id) {
            self.spec.goals.splice(i..=i, ids.iter().copied());
        }
        self.info("goals", format!("unmerged into {} goals", ids.len()));
    }

    pub fn take_out(&mut self, id: Id, j: usize) {
        let Some(GoalKind::Gate(g)) = self.doc.goal(id).map(|g| g.kind.clone()) else { return };
        if g.parts.len() <= 2 {
            return self.unmerge(id);
        }
        let mut rest = g.parts.clone();
        let out = rest.remove(j);
        let name = doc::parts_name(&rest);
        self.edit("take out of merged goal", None, |d| {
            let Some(i) = d.goals.iter().position(|x| x.id == id) else { return };
            let base = d.goals[i].clone();
            let new = doc::Goal { id: d.id(), name: out.name.clone(), kind: GoalKind::Gate(out.gate.clone()), ..base.clone() };
            (d.goals[i].kind, d.goals[i].name) = (GoalKind::Gate(doc::GateGoal::of_parts(rest.clone())), name.clone());
            d.goals.insert(i + 1, new);
        });
        self.info("goals", format!("took {} out", out.name));
    }

    pub fn delete_selected(&mut self) {
        match self.sel {
            Sel::Goal(_) => {
                let gone: Vec<Id> = self.selected().into_iter().filter_map(|s| if let Sel::Goal(id) = s { Some(id) } else { None }).collect();
                self.edit("delete goals", None, |d| d.goals.retain(|g| !gone.contains(&g.id)));
                self.select(Sel::None);
            }
            Sel::Start => {
                self.edit("remove start state", None, |d| d.start = None);
                self.sel = Sel::None;
            }
            Sel::Setup(id) => {
                self.edit("delete setup", None, |d| d.setups.retain(|s| s.id != id));
                self.sel = Sel::None;
            }
            Sel::Guide(id) => {
                self.edit("delete line", None, |d| d.guides.retain(|g| g.id != id));
                self.drawing = self.drawing.filter(|d| *d != id);
                self.sel = Sel::None;
            }
            Sel::Run(id) => {
                self.edit("delete run", None, |d| {
                    d.runs.retain(|r| r.id != id);
                    d.history.retain(|r| r.id != id);
                });
                self.sel = Sel::None;
            }
            Sel::Ghost(i) if i < self.ghosts.len() => {
                self.ghosts.remove(i);
                self.sel = Sel::None;
                self.dirty = true;
            }
            _ => {}
        }
    }

    pub fn duplicate_selected(&mut self) {
        if let Sel::Goal(id) = self.sel
            && let Some(g) = self.doc.goal(id).cloned()
        {
            let mut nid = 0;
            self.edit("duplicate goal", None, |d| {
                nid = d.id();
                let mut g2 = g.clone();
                g2.id = nid;
                g2.name = format!("{} copy", g.name);
                if let GoalKind::Point { pos, .. } | GoalKind::Box { pos, .. } = &mut g2.kind {
                    pos[0] += 2.0;
                }
                let at = d.goals.iter().position(|x| x.id == id).map_or(d.goals.len(), |i| i + 1);
                d.goals.insert(at, g2);
            });
            self.select(Sel::Goal(nid));
        }
    }

    pub fn race_start_tick(&self) -> usize {
        ((-self.race_ms0).max(0) / 10) as usize
    }

    pub fn place_start(&mut self, s: StartState) {
        let label = if self.doc.start.is_some() { "move start state" } else { "place start state" };
        self.edit(label, Some("start"), |d| d.start = Some(s));
        self.sel = Sel::Start;
    }

    pub fn start_from_cursor(&self) -> Option<StartState> {
        let f = self.sim.frames.get(self.cursor)?;
        Some(StartState {
            tick: self.cursor.max(self.race_start_tick()),
            pos: f.position(),
            ypr: doc::rot_ypr(doc::frame_rot(f)),
            velocity: f.velocity(),
            angular: f.angular_velocity().map(f32::to_degrees),
            reset_contacts: false,
        })
    }

    pub fn set_cursor(&mut self, k: usize) {
        let k = k.max(self.race_start_tick()).min(self.doc.len);
        if k != self.cursor {
            self.cursor = k;
            self.dirty = true;
            self.sim.probe(k, self.show.branches);
        }
    }

    pub fn step(&mut self, d: isize) {
        self.playing = false;
        let target = (self.cursor as isize + d).max(0) as usize;
        if target >= self.doc.len {
            let len = target + 100;
            self.edit("extend run", Some("extend"), |doc| doc.len = len);
        }
        self.set_cursor(target);
    }

    pub fn toggle_play(&mut self) {
        if self.playing {
            self.playing = false;
            self.stop_recording();
        } else {
            if !self.record_mode && self.cursor + 1 >= self.sim.frames.len() && self.sim.done() {
                self.cursor = 0;
            }
            self.playing = true;
            self.play_pos = self.cursor as f64;
            self.last_tick = None;
            if self.record_mode {
                self.recorder = Some(Recorder::new(self.cursor));
                self.edit("record", None, |d| d.remember("recording"));
            }
        }
    }

    pub fn stop_recording(&mut self) {
        if self.recorder.take().is_some() {
            self.sim.simulate(self.doc.inputs.clone(), self.cursor, self.doc.len, None);
            self.info("edit", format!("recorded up to tick {}", self.cursor));
        }
    }

    pub fn playback(&mut self) {
        if !self.playing {
            self.last_tick = None;
            return;
        }
        let now = Instant::now();
        let dt = self.last_tick.map_or(0.0, |t| (now - t).as_secs_f64());
        self.last_tick = Some(now);
        let simulated = self.sim.frames.len().saturating_sub(1);
        let mut pos = self.play_pos + dt * 100.0 * self.play_speed as f64;
        if self.recorder.is_some() {
            let until = (pos as usize + 1).min(self.cursor + 20);
            let ms0 = self.race_ms0;
            let keys = self.drive;
            let mut inputs = (*self.doc.inputs).clone();
            let rec = self.recorder.as_mut().unwrap();
            if let Some(first) = rec.write(&mut inputs, ms0, until, keys) {
                self.doc.inputs = Arc::new(inputs);
                self.doc.run_origin = Origin::Record;
                if until + 200 > self.doc.len {
                    self.doc.len = until + 500;
                }
                self.sim.simulate(self.doc.inputs.clone(), first, self.doc.len, Some(until));
            }
            pos = pos.min(simulated as f64 + 0.99);
        } else if pos > simulated as f64 {
            pos = simulated as f64;
            if self.sim.done() && simulated + 1 >= self.doc.len {
                self.playing = false;
            }
        }
        self.play_pos = pos;
        let k = pos as usize;
        if k != self.cursor {
            self.cursor = k;
            self.dirty = true;
        }
    }

    pub fn display_frame(&self) -> Option<Frame> {
        let fr = &self.sim.frames;
        let a = fr.get(self.cursor)?;
        if !self.playing {
            return Some(*a);
        }
        let k = self.play_pos.floor() as usize;
        match (fr.get(k), fr.get(k + 1)) {
            (Some(a), Some(b)) => Some(crate::scene::lerp_frame(a, b, (self.play_pos - k as f64) as f32)),
            _ => Some(*a),
        }
    }

    pub fn markers(&self) -> Vec<usize> {
        let mut v: Vec<usize> = self.reach.iter().filter_map(|r| r.tick).collect();
        for k in 1..self.sim.frames.len() {
            if self.sim.frames[k].waypoints != self.sim.frames[k - 1].waypoints {
                v.push(k);
            }
        }
        if let Some(s) = &self.doc.start {
            v.push(s.tick);
        }
        v.extend([self.spec.window.start, self.spec.window.end]);
        v.sort();
        v.dedup();
        v
    }

    pub fn focus_selection(&mut self) {
        let target: Option<(Vec3, f32)> = match self.sel {
            Sel::Goal(id) => self.doc.goal(id).map(|g| match &g.kind {
                GoalKind::Gate(gate) => {
                    let b = &gate.boxes[0];
                    ((Vec3::from(b.min) + Vec3::from(b.max)) * 0.5, (Vec3::from(b.max) - Vec3::from(b.min)).length() * 0.6)
                }
                GoalKind::Point { radius, .. } => (g.pos(), radius.max(3.0) * 2.0),
                GoalKind::Pose(_) => (g.pos(), 6.0),
                GoalKind::Box { size, .. } => (g.pos(), Vec3::from(*size).length().max(4.0)),
            }),
            Sel::Start => self.doc.start.as_ref().map(|s| (Vec3::from(s.pos), 6.0)),
            Sel::Gate(w) => self.gates.iter().find(|g| g.gate.waypoint == w).map(|g| {
                let b = &g.gate.boxes[0];
                ((Vec3::from(b.min) + Vec3::from(b.max)) * 0.5, 20.0)
            }),
            _ => self.display_frame().map(|f| (Vec3::from(f.position()), 6.0)),
        };
        if let Some((c, r)) = target {
            self.camera.focus(c, r);
            self.dirty = true;
        }
    }

    pub fn frame_all(&mut self) {
        if let Some((min, max)) = self.run_bounds() {
            self.camera.frame(min, max);
        }
        self.dirty = true;
    }

    pub fn frame_route(&mut self) {
        let Some((a, b)) = self.route_ticks() else { return };
        let pts = self.sim.frames.get(a..=b.min(self.sim.frames.len().saturating_sub(1))).unwrap_or(&[]);
        if pts.is_empty() {
            return;
        }
        let (min, max) = pts.iter().map(|f| Vec3::from(f.position())).fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), p| (lo.min(p), hi.max(p)));
        self.camera.frame(min - Vec3::splat(8.0), max + Vec3::splat(8.0));
        self.dirty = true;
    }

    pub fn recall(&mut self, i: usize) {
        if let Some(b) = self.doc.bookmarks.get(i).cloned() {
            self.camera.go(View { pivot: Vec3::from(b.pivot), yaw: b.yaw, pitch: b.pitch, distance: b.distance });
            self.info("view", format!("camera bookmark {}: {}", i + 1, b.name));
        } else {
            self.log(LogLevel::Warn, "view", format!("no camera bookmark {} (save one with Ctrl+Shift+{})", i + 1, i + 1));
        }
    }

    pub fn save_view(&mut self, i: usize) {
        let v = self.camera.goal;
        let b = doc::Bookmark { name: format!("View {}", i + 1), pivot: v.pivot.into(), yaw: v.yaw, pitch: v.pitch, distance: v.distance };
        self.edit("save camera bookmark", None, |d| {
            while d.bookmarks.len() <= i {
                d.bookmarks.push(b.clone());
            }
            d.bookmarks[i] = b;
        });
        self.info("view", format!("saved camera bookmark {}", i + 1));
    }

    pub fn add_ghost(&mut self, path: &Path) {
        match GhostTrack::read(path) {
            Ok(g) => {
                self.info("io", format!("ghost {}: {} samples, {}", g.name, g.samples.len(), g.race_time_ms.map_or("unfinished".into(), crate::fmt::fmt_ms)));
                self.ghosts.push(GhostEntry { track: g, visible: true });
                self.compare.1 = RunRef::Ghost(self.ghosts.len() - 1);
                self.dirty = true;
            }
            Err(e) => self.error("io", format!("ghost {}: {e:#}", path.display())),
        }
    }

    pub fn open_file(&mut self, path: &Path, cx: &mut Context<Self>) {
        let name = path.file_name().map(|n| n.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
        if name.ends_with(".replay.gbx") || name.ends_with(".ghost.gbx") {
            if self.map.is_none() {
                return self.error("io", format!("open a map before importing {}", path.display()));
            }
            self.import_replay(path, cx);
        } else {
            self.new_project(path, None, cx);
        }
        cx.notify();
    }

    pub fn import_replay(&mut self, path: &Path, cx: &mut Context<Self>) {
        let f = match crate::replay::load(path).and_then(|r| crate::replay::inputs(&r)) {
            Ok(f) => f,
            Err(e) => return self.error("io", format!("import {}: {e:#}", path.display())),
        };
        let name = path.file_name().unwrap_or_default().to_string_lossy().to_string();
        let start_ms = gearbox_engine::session::Clock::new(f.t0).ok().map(|c| c.race_ms);
        self.info("io", format!("imported {name}: {} ticks from {} ms, seed {:?}", f.controls.len(), f.t0, f.seed));
        if start_ms != Some(self.race_ms0) {
            let mut d = self.doc.clone();
            self.undo.push("import replay", d.clone());
            d.remember("import");
            d.run_origin = Origin::Replay(name.clone());
            let end = (f.end_ms() - start_ms.unwrap_or(f.t0)) / 10;
            d.len = d.len.max(end.max(0) as usize + 100);
            let map = self.map.clone().unwrap();
            self.info("sim", "rebuilding the session for the replay's countdown");
            self.open_map(&map, InputSel::Memory(f, name), Some(d), cx);
            return;
        }
        let need = ((f.end_ms() - self.race_ms0).max(0) / 10) as usize + 100;
        self.edit("import replay", None, |d| {
            d.remember("import");
            d.inputs = Arc::new(f);
            d.run_origin = Origin::Replay(name);
            d.len = d.len.max(need);
        });
    }

    pub fn search_ctx_parts(&self) -> (Vec<Option<usize>>, Vec<u16>) {
        (self.aimed_reach(&self.sim.frames).iter().map(|r| r.tick).collect(), self.sim.frames.iter().map(|f| f.waypoints).collect())
    }

    pub fn aimed_goals(&self) -> Vec<Goal> {
        use crate::optimize::ObjKind;
        match self.spec.kind {
            ObjKind::ReachGoals => self.spec.ordered_goals(&self.doc.goals).into_iter().cloned().collect(),
            _ => Vec::new(),
        }
    }

    pub fn aimed_reach(&self, frames: &[Frame]) -> Vec<Reach> {
        let aimed = self.aimed_goals();
        let r = doc::evaluate(&aimed, frames);
        self.doc.goals.iter().map(|g| aimed.iter().position(|a| a.id == g.id).map(|i| r[i]).unwrap_or_default()).collect()
    }

    pub fn speed_worth(&self, id: Id, next: Option<Id>) -> f32 {
        let at = |id: Id| self.doc.goals.iter().position(|g| g.id == id).and_then(|i| self.reach.get(i)).and_then(|r| r.tick);
        let Some(k) = at(id) else { return 0.0 };
        let Some(f) = self.sim.frames.get(k) else { return 0.0 };
        let v = f.speed().max(10.0);
        let end = next.and_then(at).or_else(|| self.sim.frames.iter().position(|f| f.finish.is_some())).filter(|e| *e > k);
        let d = end.map_or(100.0, |e| self.sim.frames[k..=e.min(self.sim.frames.len() - 1)].windows(2).map(|w| Vec3::from(w[0].position()).distance(Vec3::from(w[1].position()))).sum::<f32>());
        (1000.0 * d / (3.6 * v * v)).clamp(0.1, 100.0)
    }

    pub fn start_search(&mut self) {
        let k0 = self.race_start_tick();
        let blank = (k0..self.spec.window.start).all(|k| self.control(k) == gearbox_engine::Controls::default());
        if self.spec.window.start < k0 || (self.doc.start.is_none() && blank) {
            self.spec.window = k0..self.spec.window.end.max(k0 + 1);
        }
        let Some(origin) = self.origin.clone() else { return self.error("search", "nothing open") };
        let (reach, wps) = self.search_ctx_parts();
        let goals = self.doc.goals.clone();
        let w = |k: usize| wps.get(k.min(wps.len().saturating_sub(1))).copied().unwrap_or(0) as usize;
        let ctx = optimize::Ctx { goals: &goals, race_ms0: self.race_ms0, reach: &reach, waypoints_at: &w };
        let sentence = self.spec.sentence(&ctx);
        let (mut objective, until_ms) = match self.spec.objective(&ctx) {
            Ok(o) => o,
            Err(e) => {
                self.field_errors.insert("search".into(), format!("{e:#}"));
                return self.error("search", format!("{e:#}"));
            }
        };
        if let gearbox_search::objective::Objective::Goals(g) = &mut objective
            && let Some(last) = self.spec.ordered_goals(&self.doc.goals).last().map(|g| g.id)
            && self.spec.both.contains(&last)
        {
            g.speed_worth_ms = f64::from(self.speed_worth(last, None));
        }
        let until = ((until_ms - self.race_ms0) / 10) as usize;
        self.field_errors.remove("search");
        if until > self.doc.len {
            self.edit("extend run for the search", None, |d| d.len = until + 100);
        }
        let inputs = self.doc.inputs.covering(self.race_ms0, until_ms);
        let start_line = match self.spec.start_from {
            optimize::StartFrom::Run => None,
            optimize::StartFrom::Line(id) => match self.doc.guide(id) {
                Some(g) => Some(g.points.clone()),
                None => return self.error("search", "the line to start from is gone: pick another"),
            },
            optimize::StartFrom::Ghost => match self.ghosts.first() {
                Some(g) => Some(g.track.samples.iter().map(|s| s.position).fold(Vec::new(), |mut v: Vec<[f32; 3]>, p| {
                    if v.last().is_none_or(|q| Vec3::from(*q).distance(Vec3::from(p)) >= 10.0) {
                        v.push(p);
                    }
                    v
                })),
                None => return self.error("search", "no ghost to start from: add one (+ Replay under Ghosts)"),
            },
        };
        let mut origin_tick = self.spec.window.start;
        if matches!(objective, gearbox_search::objective::Objective::Goals(_)) {
            for &k in reach.iter().flatten() {
                if k <= self.spec.window.start {
                    origin_tick = origin_tick.min(k.saturating_sub(1));
                }
            }
        }
        if let Some(s) = &self.doc.start {
            origin_tick = origin_tick.max(s.tick);
        }
        let req = optimize::Request {
            origin,
            inputs,
            race_ms0: self.race_ms0,
            spec: self.spec.clone(),
            objective,
            until_ms,
            locks: self.doc.locks.clone(),
            start: self.doc.start.as_ref().map(|s| (s.tick, s.placement())),
            origin_tick,
            start_line,
            world: self.world.clone().map(|w| {
                let pts = self.sim.frames.iter().map(|f| f.position()).chain(self.doc.goals.iter().map(|g| g.pos().into()));
                let (mut lo, mut hi) = ([f32::MAX; 3], [f32::MIN; 3]);
                for p in pts {
                    for a in 0..3 {
                        lo[a] = lo[a].min(p[a]);
                        hi[a] = hi[a].max(p[a]);
                    }
                }
                (w, lo.map(|v| v - 150.0), hi.map(|v| v + 150.0))
            }),
        };
        self.info("search", format!("{sentence} \u{00b7} from tick {origin_tick}"));
        let w = self.waker.clone();
        if self.auto_preview {
            self.preview = None;
        }
        self.layout.right_tab = 1;
        self.tries = Arc::default();
        self.search.start(req, Arc::new(move || w.wake()));
        self.checks.clear();
        self.search_key = self.aim_key();
        self.search_stale = None;
    }

    pub fn aim_key(&self) -> String {
        let mut s = self.spec.clone();
        (s.auto, s.by_time, s.seconds, s.attempts, s.go_on, s.threads) = (true, true, 0.0, 0, false, 0);
        let goals = self.aimed_goals().into_iter().map(|g| (g.id, g.kind, g.when)).collect::<Vec<_>>();
        let locks = if s.honour_locks { self.doc.locks.clone() } else { Vec::new() };
        serde_json::to_string(&(s, goals, &self.doc.start, locks)).unwrap_or_default()
    }

    pub fn follow_search(&mut self, cx: &mut Context<Self>) {
        if !self.search.running() || self.search.cancel.load(std::sync::atomic::Ordering::Relaxed) {
            self.search_stale = None;
            return;
        }
        let key = self.aim_key();
        if key == self.search_key {
            self.search_stale = None;
            return;
        }
        match &self.search_stale {
            Some((k, at)) if *k == key && self.drag.is_none() && at.elapsed() >= SEARCH_SETTLE => {
                self.info("search", "aim changed: searching again");
                self.search.stop();
                self.start_search();
                return;
            }
            Some((k, _)) if *k == key => {}
            _ => self.search_stale = Some((key, Instant::now())),
        }
        if !self.follow_timer {
            self.follow_timer = true;
            cx.spawn(async move |this, cx| {
                cx.background_executor().timer(SEARCH_SETTLE).await;
                let _ = this.update(cx, |e, cx| {
                    e.follow_timer = false;
                    e.follow_search(cx);
                    cx.notify();
                });
            })
            .detach();
        }
    }

    fn candidate_simulated(&mut self, id: u64, frames: &[Frame], micros: u64) {
        let cand = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).cloned();
        let Some(c) = cand else { return };
        let goals = self.aimed_reach(frames);
        let mut msg = format!("editor re-simulation: {} ticks in {:.0} ms", frames.len(), micros as f32 / 1000.0);
        if let Some(times) = optimize::goal_times(&c.score)
            && self.spec.kind.goals()
        {
            let ids = self.spec.ordered_goals(&self.doc.goals).iter().map(|g| g.id).collect::<Vec<_>>();
            let ours: Vec<Option<i32>> = ids.iter().filter_map(|id| self.doc.goals.iter().position(|g| g.id == *id)).map(|i| goals.get(i).and_then(|r| r.ms)).collect();
            let agree = times.iter().enumerate().all(|(i, t)| ours.get(i).copied().flatten().is_some_and(|o| (o - t).abs() <= 10));
            msg += if agree { " \u{00b7} goal times agree" } else { " \u{00b7} goal times DIFFER (see console)" };
            if !agree {
                self.log(LogLevel::Warn, "search", format!("candidate #{id}: search times {times:?}, editor {ours:?}"));
            }
        }
        self.checks.insert(id, msg);
    }

    pub fn accept(&mut self, id: u64) {
        let c = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).cloned();
        let Some(c) = c else { return };
        let label = format!("accept candidate #{id}");
        let (inputs, splice) = self.spliced(id).map_or(((*c.inputs).clone(), None), |(i, s)| (i, Some(s)));
        self.replace_run(&label, inputs, Origin::Search(format!("#{id} {}", self.search.live.lock().unwrap().metric(&c.score).text)));
        if let Some(st) = c.start.clone() {
            self.edit("the car it found", Some("start"), |d| d.start = Some(st));
        }
        if let Some(s) = &splice {
            let shift = 10 * (s.new_tick as i32 - s.old_tick as i32);
            self.info("search", format!("kept your route after {}: your inputs from there run {} s {}", s.goal, crate::fmt::fmt_ms(shift.abs()), if shift <= 0 { "earlier" } else { "later" }));
        }
        if let Some(fr) = self.runs_frames.get(&Key::Candidate(id)) {
            self.accept_check = Some((id, fr.clone(), splice.as_ref().map_or(usize::MAX, |s| s.new_tick + 1)));
        }
        self.preview = None;
        self.sel = Sel::Current;
        self.info("search", format!("accepted candidate #{id}"));
    }

    pub fn spliced(&self, id: u64) -> Option<(InputFile, Splice)> {
        let c = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).cloned()?;
        let frames = self.runs_frames.get(&Key::Candidate(id))?;
        let (new, old) = (self.aimed_reach(frames), self.aimed_reach(&self.sim.frames));
        let aimed = self.aimed_goals();
        let at = |g: &Goal| self.doc.goals.iter().position(|x| x.id == g.id);
        let judged = aimed.iter().rev().find(|g| at(g).and_then(|i| new.get(i)?.tick).is_some())?;
        let i = at(judged)?;
        let (goal, new_tick, old_tick) = (judged.name.clone(), new.get(i)?.tick?, old.get(i)?.tick?);
        let out = doc::splice(&c.inputs, &self.doc.inputs, self.race_ms0, new_tick, old_tick)?;
        Some((out, Splice { goal, old_tick, new_tick }))
    }

    pub fn keep_candidate(&mut self, id: u64) {
        let c = self.search.live.lock().unwrap().candidates.iter().find(|c| c.id == id).cloned();
        let Some(c) = c else { return };
        let len = self.doc.len;
        let start = c.start.clone().or_else(|| self.doc.start.clone());
        let inputs = self.spliced(id).map_or(c.inputs.clone(), |(i, _)| Arc::new(i));
        let text = self.search.live.lock().unwrap().metric(&c.score).text;
        let mut rid = 0;
        self.edit("keep candidate", None, |d| {
            rid = d.id();
            d.runs.push(Run { id: rid, name: format!("Candidate #{id}"), inputs, len, start, origin: Origin::Search(text), at: doc::clock() });
        });
        if let Some(f) = self.runs_frames.get(&Key::Candidate(id)).cloned() {
            self.runs_frames.insert(Key::Run(rid), f);
        }
        self.info("search", format!("kept candidate #{id} as an alternative run"));
    }

    pub fn new_line_from_here(&mut self) {
        let k = self.cursor.max(self.start_tick().unwrap_or(0));
        let (len, start, inputs, name) = (self.doc.len, self.doc.start.clone(), self.doc.inputs.clone(), self.doc.run_name.clone());
        let at = crate::fmt::fmt_ms(self.race_ms0 + 10 * k as i32);
        let label = format!("{name} before {at} s");
        self.edit("keep the old line", None, |d| {
            let rid = d.id();
            d.runs.push(Run { id: rid, name: label, inputs, len, start, origin: Origin::Edit, at: doc::clock() });
        });
        self.set_ticks("new line from here", None, (k..self.doc.len).map(|t| (t, Controls::default())));
        self.spec.window = k..(k + 300).min(self.doc.len.max(k + 1));
        self.spec.until = None;
        self.info("edit", format!("new line from {at} s: inputs after it released; the old line is under Runs. Place a goal ahead to search."));
    }

    pub fn add_line_point(&mut self, p: Vec3) {
        let p: [f32; 3] = (p + Vec3::Y * 0.5).into();
        let mut id = self.drawing.unwrap_or(0);
        let n = self.doc.guides.len() + 1;
        self.edit("draw line", Some(&format!("line-{}", self.drawing.map_or(0, |d| d))), |d| match d.guide_mut(id) {
            Some(g) => g.points.push(p),
            None => {
                id = d.id();
                d.guides.push(crate::doc::Guide { id, name: format!("Line {n}"), points: vec![p] });
            }
        });
        self.drawing = Some(id);
        self.sel = Sel::Guide(id);
    }

    pub fn discard_candidate(&mut self, id: u64) {
        self.search.live.lock().unwrap().candidates.retain(|c| c.id != id);
        if self.preview == Some(id) {
            self.preview = None;
        }
        if self.sel == Sel::Candidate(id) {
            self.sel = Sel::None;
        }
        self.dirty = true;
    }

    fn check_accept(&mut self) {
        let Some((id, cand, upto)) = &self.accept_check else { return };
        if !self.sim.done() {
            return;
        }
        let n = cand.len().min(self.sim.frames.len()).min(*upto);
        let differs = (0..n).find(|&k| cand[k].body.map(f32::to_bits) != self.sim.frames[k].body.map(f32::to_bits));
        let same = differs.is_none();
        let msg = match differs {
            None => format!("session re-simulation identical to the preview: {n} ticks, every body bit"),
            Some(k) => format!("session re-simulation DIFFERS from the preview from tick {k} (of {n} compared)"),
        };
        let id = *id;
        if same {
            self.info("search", format!("candidate #{id}: {msg}"));
        } else {
            self.error("search", format!("candidate #{id}: {msg}"));
        }
        let prev = self.checks.get(&id).cloned().unwrap_or_default();
        self.checks.insert(id, format!("{prev} \u{00b7} {msg}"));
        self.accept_check = None;
    }

    pub fn save_layout(&self) {
        let _ = self.layout.save(&self.home);
    }

    pub fn start_tick(&self) -> Option<usize> {
        self.doc.start.as_ref().map(|s| s.tick)
    }
}

pub fn drive_key(k: &str) -> Option<usize> {
    match k {
        "up" | "w" => Some(0),
        "down" | "s" => Some(1),
        "left" | "a" => Some(2),
        "right" | "d" => Some(3),
        _ => None,
    }
}
