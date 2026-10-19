use std::path::{Path, PathBuf};
use std::sync::Arc;

use crate::engine::input::InputFile;
use anyhow::{Context as _, Result, ensure};
use serde::{Deserialize, Serialize};

use crate::doc::{Bookmark, Doc, Goal, Origin, Run, StartState};
use crate::optimize::Spec;

#[derive(Serialize, Deserialize)]
struct RunJson {
    name: String,
    inputs: String,
    len: usize,
    #[serde(default)]
    start: Option<StartState>,
    origin: Origin,
    #[serde(default)]
    at: String,
    #[serde(default)]
    finish_ms: Option<i32>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Finishes {
    pub run: Option<i32>,
    pub runs: Vec<Option<i32>>,
}

#[derive(Serialize, Deserialize)]
pub struct ProjectJson {
    pub version: u32,
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub finish_ms: Option<i32>,
    pub map: String,
    run: RunJson,
    #[serde(default)]
    runs: Vec<RunJson>,
    #[serde(default)]
    goals: Vec<Goal>,
    #[serde(default)]
    start: Option<StartState>,
    #[serde(default)]
    locks: Vec<[usize; 2]>,
    #[serde(default)]
    bookmarks: Vec<Bookmark>,
    #[serde(default)]
    guides: Vec<crate::doc::Guide>,
    #[serde(default)]
    setups: Vec<crate::doc::Setup>,
    #[serde(default)]
    pub search: Option<serde_json::Value>,
    #[serde(default)]
    pub route: Option<crate::lab::RouteSpec>,
    #[serde(default)]
    pub ghosts: Vec<String>,
}

fn slug(s: &str) -> String {
    let s: String = s
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() || c == '-' { c.to_ascii_lowercase() } else { '_' })
        .collect();
    let s = s.trim_matches('_').to_string();
    if s.is_empty() { "run".into() } else { s }
}

pub fn rel(home: &Path, p: &Path) -> String {
    p.strip_prefix(home).map_or_else(|_| p.display().to_string(), |r| r.display().to_string())
}

pub fn abs(home: &Path, s: &str) -> PathBuf {
    let p = PathBuf::from(s);
    if p.is_absolute() { p } else { home.join(p) }
}

#[allow(clippy::too_many_arguments)]
pub fn save(
    path: &Path,
    home: &Path,
    name: &str,
    finishes: &Finishes,
    map: &Path,
    doc: &Doc,
    search: &Spec,
    route: &crate::lab::RouteSpec,
    ghosts: &[PathBuf],
) -> Result<Vec<PathBuf>> {
    let dir = path.parent().context("project path has no directory")?;
    std::fs::create_dir_all(dir)?;
    let stem = path
        .file_name()
        .and_then(|n| n.to_str())
        .and_then(|n| n.strip_suffix(".tmproj.json"))
        .context("project files end in .tmproj.json")?
        .to_string();
    let mut written = Vec::new();
    let mut used = std::collections::HashSet::new();
    let mut write_run = |name: &str,
                         inputs: &InputFile,
                         len: usize,
                         start: &Option<StartState>,
                         origin: &Origin,
                         at: &str,
                         finish_ms: Option<i32>|
     -> Result<RunJson> {
        let mut s = slug(name);
        while !used.insert(s.clone()) {
            s.push('_');
        }
        let file = format!("{stem}.{s}.txt.in");
        let p = dir.join(&file);
        inputs.trimmed().write(&p)?;
        written.push(p);
        Ok(RunJson {
            name: name.into(),
            inputs: file,
            len,
            start: start.clone(),
            origin: origin.clone(),
            at: at.into(),
            finish_ms,
        })
    };
    let run = write_run(&doc.run_name, &doc.inputs, doc.len, &doc.start, &doc.run_origin, "", finishes.run)?;
    let runs = doc
        .runs
        .iter()
        .enumerate()
        .map(|(i, r)| {
            write_run(
                &r.name,
                &r.inputs,
                r.len,
                &r.start,
                &r.origin,
                &r.at,
                finishes.runs.get(i).copied().flatten(),
            )
        })
        .collect::<Result<Vec<_>>>()?;
    let j = ProjectJson {
        version: 1,
        name: Some(name.to_string()),
        finish_ms: finishes.run,
        map: rel(home, map),
        run,
        runs,
        goals: doc.goals.clone(),
        start: doc.start.clone(),
        locks: doc.locks.iter().map(|r| [r.start, r.end]).collect(),
        bookmarks: doc.bookmarks.clone(),
        guides: doc.guides.clone(),
        setups: doc.setups.clone(),
        search: Some(serde_json::to_value(search)?),
        route: Some(route.clone()),
        ghosts: ghosts.iter().map(|g| rel(home, g)).collect(),
    };
    std::fs::write(path, serde_json::to_string_pretty(&j)?)
        .with_context(|| format!("writing {}", path.display()))?;
    written.insert(0, path.to_path_buf());
    Ok(written)
}

pub struct Loaded {
    pub map: PathBuf,
    pub doc: Doc,
    pub search: Option<Spec>,
    pub route: Option<crate::lab::RouteSpec>,
    pub ghosts: Vec<PathBuf>,
    pub name: String,
    pub finishes: Finishes,
}

pub fn load(path: &Path, home: &Path) -> Result<Loaded> {
    let text = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let j: ProjectJson =
        serde_json::from_str(&text).with_context(|| format!("parsing {}", path.display()))?;
    ensure!(j.version == 1, "project version {} (this editor reads 1)", j.version);
    let dir = path.parent().unwrap_or(Path::new("."));
    let read = |r: &RunJson| InputFile::read(&dir.join(&r.inputs));
    let inputs = read(&j.run)?;
    let mut doc = Doc::new(inputs, j.run.len, j.run.origin.clone());
    doc.run_name = j.run.name.clone();
    doc.goals = j.goals;
    doc.next_id = doc.goals.iter().map(|g| g.id + 1).max().unwrap_or(1);
    doc.start = j.start;
    doc.locks = j.locks.iter().map(|l| l[0]..l[1]).collect();
    doc.bookmarks = j.bookmarks;
    doc.guides = j.guides;
    doc.next_id = doc.next_id.max(doc.guides.iter().map(|g| g.id + 1).max().unwrap_or(1));
    doc.setups = j.setups;
    doc.next_id = doc.next_id.max(doc.setups.iter().map(|g| g.id + 1).max().unwrap_or(1));
    for r in &j.runs {
        let id = doc.id();
        doc.runs.push(Run {
            id,
            name: r.name.clone(),
            inputs: Arc::new(read(r)?),
            len: r.len,
            start: r.start.clone(),
            origin: r.origin.clone(),
            at: r.at.clone(),
        });
    }
    let name = j.name.clone().unwrap_or_else(|| default_name(path));
    let search =
        j.search.map(|v| Spec::from_saved(v, &doc.goals)).transpose().context("reading the search setup")?;
    let finishes =
        Finishes { run: j.finish_ms.or(j.run.finish_ms), runs: j.runs.iter().map(|r| r.finish_ms).collect() };
    Ok(Loaded {
        map: abs(home, &j.map),
        doc,
        search,
        route: j.route,
        ghosts: j.ghosts.iter().map(|g| abs(home, g)).collect(),
        name,
        finishes,
    })
}

pub fn library(home: &Path) -> PathBuf {
    home.join("projects")
}

fn in_library(path: &Path) -> bool {
    path.file_name().and_then(|n| n.to_str()) == Some("project.tmproj.json")
        && path.parent().and_then(Path::parent).is_some_and(|l| l.ends_with("editor/projects"))
}

fn default_name(path: &Path) -> String {
    let file = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
    if in_library(path) {
        path.parent()
            .and_then(|d| d.file_name())
            .map_or("Project".into(), |d| d.to_string_lossy().into_owned())
    } else {
        file.strip_suffix(".tmproj.json").unwrap_or(file).to_string()
    }
}

#[derive(Clone, Debug)]
pub struct Entry {
    pub path: PathBuf,
    pub name: String,
    pub named: bool,
    pub map: PathBuf,
    pub modified: std::time::SystemTime,
    pub runs: Vec<(String, Option<i32>)>,
    pub goals: usize,
    pub setups: usize,
    pub legacy: bool,
}

impl Entry {
    pub fn best_ms(&self) -> Option<i32> {
        self.runs.iter().filter_map(|r| r.1).min()
    }
}

pub fn list(home: &Path) -> Vec<Entry> {
    let mut files: Vec<(PathBuf, bool)> = Vec::new();
    if let Ok(rd) = std::fs::read_dir(library(home)) {
        files.extend(
            rd.flatten().map(|e| (e.path().join("project.tmproj.json"), false)).filter(|(p, _)| p.exists()),
        );
    }
    if let Ok(rd) = std::fs::read_dir(home.join("inputs")) {
        files.extend(
            rd.flatten()
                .map(|e| e.path())
                .filter(|p| p.to_string_lossy().ends_with(".tmproj.json"))
                .map(|p| (p, true)),
        );
    }
    let mut out: Vec<Entry> = files
        .into_iter()
        .filter_map(|(path, legacy)| {
            let j: ProjectJson = serde_json::from_str(&std::fs::read_to_string(&path).ok()?).ok()?;
            let modified = std::fs::metadata(&path).and_then(|m| m.modified()).ok()?;
            let map = abs(home, &j.map);
            let name = j.name.clone().unwrap_or_else(|| default_name(&path));
            let named = name != default_name(&path) && Some(&name) != crate::catalog::map_stem(&map).as_ref();
            let mut runs = vec![(j.run.name.clone(), j.finish_ms.or(j.run.finish_ms))];
            runs.extend(j.runs.iter().map(|r| (r.name.clone(), r.finish_ms)));
            Some(Entry {
                name,
                named,
                map,
                modified,
                runs,
                goals: j.goals.len(),
                setups: j.setups.len(),
                legacy,
                path,
            })
        })
        .collect();
    out.sort_by(|a, b| b.modified.cmp(&a.modified));
    out
}

pub fn new_path(home: &Path, name: &str) -> PathBuf {
    let base = slug(name);
    let lib = library(home);
    let mut dir = lib.join(&base);
    let mut n = 2;
    while dir.exists() {
        dir = lib.join(format!("{base}-{n}"));
        n += 1;
    }
    dir.join("project.tmproj.json")
}

pub fn rename(path: &Path, name: &str) -> Result<()> {
    let mut j: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(path)?)?;
    j["name"] = serde_json::Value::String(name.to_string());
    std::fs::write(path, serde_json::to_string_pretty(&j)?)?;
    Ok(())
}

pub fn duplicate(path: &Path, home: &Path) -> Result<PathBuf> {
    let l = load(path, home)?;
    let name = format!("{} copy", l.name);
    let to = new_path(home, &name);
    save(
        &to,
        home,
        &name,
        &l.finishes,
        &l.map,
        &l.doc,
        &l.search.unwrap_or_default(),
        &l.route.unwrap_or_default(),
        &l.ghosts,
    )?;
    Ok(to)
}

pub fn files(path: &Path) -> Result<Vec<PathBuf>> {
    let dir = path.parent().context("project folder")?;
    if in_library(path) {
        return Ok(vec![dir.to_path_buf()]);
    }
    let stem = path
        .file_name()
        .and_then(|n| n.to_str())
        .and_then(|n| n.strip_suffix(".tmproj.json"))
        .context("project file")?
        .to_string();
    let mut out = Vec::new();
    for e in std::fs::read_dir(dir)?.flatten() {
        let n = e.file_name().to_string_lossy().into_owned();
        if n == format!("{stem}.tmproj.json")
            || (n.starts_with(&format!("{stem}.")) && n.ends_with(".txt.in"))
        {
            out.push(e.path());
        }
    }
    Ok(out)
}

pub fn trash(path: &Path) -> Result<Vec<(PathBuf, PathBuf)>> {
    files(path)?.into_iter().map(|f| Ok((trash_item(&f)?, f))).collect()
}

pub fn restore(moved: &[(PathBuf, PathBuf)]) -> Result<()> {
    for (from, to) in moved {
        ensure!(!to.exists(), "{} is taken", to.display());
        std::fs::rename(from, to).with_context(|| format!("restoring {}", to.display()))?;
    }
    Ok(())
}

#[cfg(target_os = "macos")]
fn trash_item(p: &Path) -> Result<PathBuf> {
    use objc2_foundation::{NSFileManager, NSString, NSURL};
    let url = NSURL::fileURLWithPath(&NSString::from_str(&p.to_string_lossy()));
    let mut out = None;
    NSFileManager::defaultManager()
        .trashItemAtURL_resultingItemURL_error(&url, Some(&mut out))
        .map_err(|e| anyhow::anyhow!("moving {} to the Trash: {}", p.display(), e.localizedDescription()))?;
    out.and_then(|u| u.path()).map(|s| PathBuf::from(s.to_string())).context("the Trash gave no location")
}

#[cfg(not(target_os = "macos"))]
fn trash_item(p: &Path) -> Result<PathBuf> {
    anyhow::bail!("no Trash here for {}", p.display())
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Layout {
    pub left: f32,
    pub right: f32,
    pub bottom: f32,
    pub bottom_tab: u8,
    #[serde(default)]
    pub right_tab: u8,
    #[serde(default)]
    pub guide_dismissed: bool,
    #[serde(default)]
    pub trackpad_orbit: bool,
}

impl Default for Layout {
    fn default() -> Self {
        Self {
            left: 248.0,
            right: 350.0,
            bottom: 300.0,
            bottom_tab: 0,
            right_tab: 0,
            guide_dismissed: false,
            trackpad_orbit: false,
        }
    }
}

impl Layout {
    pub fn path(home: &Path) -> PathBuf {
        home.join("layout.json")
    }
    pub fn load(home: &Path) -> Self {
        std::fs::read_to_string(Self::path(home))
            .ok()
            .and_then(|t| serde_json::from_str(&t).ok())
            .unwrap_or_default()
    }
    pub fn save(&self, home: &Path) -> Result<()> {
        let p = Self::path(home);
        std::fs::create_dir_all(p.parent().unwrap())?;
        std::fs::write(&p, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::doc::{GoalKind, PoseSpec, When};

    #[test]
    fn only_library_folders_go_whole() {
        let root = std::env::temp_dir().join(format!("tmproj-del-{}", std::process::id()));
        let inputs = root.join("inputs");
        std::fs::create_dir_all(&inputs).unwrap();
        std::fs::write(inputs.join("project.tmproj.json"), "{}").unwrap();
        std::fs::write(inputs.join("project.run.txt.in"), "T0 0").unwrap();
        std::fs::write(inputs.join("other.txt.in"), "T0 0").unwrap();
        let mut f = files(&inputs.join("project.tmproj.json")).unwrap();
        f.sort();
        assert_eq!(f, vec![inputs.join("project.run.txt.in"), inputs.join("project.tmproj.json")]);
        let lib = root.join("editor/projects/mine");
        std::fs::create_dir_all(&lib).unwrap();
        std::fs::write(lib.join("project.tmproj.json"), "{}").unwrap();
        assert_eq!(files(&lib.join("project.tmproj.json")).unwrap(), vec![lib.clone()]);
        std::fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn a_project_round_trips_with_its_runs() {
        let dir = std::env::temp_dir().join(format!("tmproj-{}", std::process::id()));
        let data = dir.join("editor");
        std::fs::create_dir_all(data.join("inputs")).unwrap();
        let map = data.join("tmx/x/1_Map.Map.Gbx");
        let mut doc = Doc::new(InputFile::parse("T0 -1500\n1 0 0.5\n1 1 -1\n").unwrap(), 600, Origin::Blank);
        doc.run_name = "Main line".into();
        let id = doc.id();
        doc.goals.push(Goal {
            id,
            name: "P1".into(),
            kind: GoalKind::Point { pos: [1.0, 2.0, 3.0], radius: 4.0 },
            visible: true,
            when: Default::default(),
        });
        let id = doc.id();
        doc.goals.push(Goal {
            id,
            name: "Pose 1".into(),
            kind: GoalKind::Pose(PoseSpec::at(glam::Vec3::new(5.0, 6.0, 7.0), 45.0)),
            visible: false,
            when: When { min_kmh: Some(250.0), pitch: Some([-90.0, -50.0]), slide: None },
        });
        doc.start = Some(StartState {
            tick: 150,
            pos: [1.0, 2.0, 3.0],
            ypr: [10.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 20.0],
            angular: [0.0; 3],
            reset_contacts: true,
        });
        doc.lock(10..20);
        doc.bookmarks.push(Bookmark {
            name: "Jump".into(),
            pivot: [1.0, 2.0, 3.0],
            yaw: 0.5,
            pitch: 0.3,
            distance: 40.0,
        });
        doc.guides.push(crate::doc::Guide {
            id: 77,
            name: "Cut".into(),
            points: vec![[0.0, 1.0, 2.0], [10.0, 1.0, 2.0]],
        });
        let placed = doc.start.clone().unwrap();
        doc.setups.push(crate::doc::Setup {
            id: 90,
            name: "Nosebug".into(),
            start: placed,
            controls: vec![[1.0, 0.0, -0.5], [0.0, 1.0, 1.0]],
            end_pos: [4.0, 5.0, 6.0],
            end_ypr: [1.0, -40.0, 0.0],
            end_velocity: [0.0, -3.0, 50.0],
            tricks: Some([40, 0, 2]),
        });
        let alt = doc.current(Origin::Edit);
        doc.runs.push(Run { name: "Alt".into(), ..alt });
        let spec = Spec { seed: 7, ..Default::default() };
        let p = new_path(&data, "My project");
        let route = crate::lab::RouteSpec {
            from: crate::lab::RouteFrom::Stop(2),
            to: crate::lab::RouteAt::Goal(id),
            seconds: 30.0,
        };
        let fin = Finishes { run: Some(12_340), runs: vec![None] };
        let files =
            save(&p, &data, "test", &fin, &map, &doc, &spec, &route, &[data.join("tmx/x/1_a.Replay.Gbx")])
                .unwrap();
        assert_eq!(files.len(), 3);
        let back = load(&p, &data).unwrap();
        assert_eq!(back.map, map);
        assert_eq!(back.doc.goals, doc.goals);
        assert_eq!(back.doc.start, doc.start);
        assert_eq!(back.doc.locks, doc.locks);
        assert_eq!(back.doc.bookmarks, doc.bookmarks);
        assert_eq!(back.doc.guides, doc.guides);
        assert_eq!(back.doc.setups, doc.setups);
        assert_eq!(back.route, Some(route));
        assert!(back.doc.next_id > 90);
        assert_eq!(back.doc.inputs.to_text(), doc.inputs.to_text());
        assert_eq!(back.doc.run_name, "Main line");
        assert_eq!(back.doc.runs.len(), 1);
        assert_eq!(back.doc.runs[0].name, "Alt");
        assert_eq!(back.search.unwrap().seed, 7);
        assert_eq!(back.ghosts, vec![data.join("tmx/x/1_a.Replay.Gbx")]);
        assert_eq!(back.finishes, fin);
        let e = list(&data).into_iter().find(|e| e.path == p).unwrap();
        assert_eq!((e.runs.len(), e.best_ms(), e.goals, e.setups), (2, Some(12_340), 2, 1));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
