use anyhow::{Context, Result, bail, ensure};
use gearbox_engine::Session;

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Bounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(try_from = "GateJson")]
pub struct Gate {
    pub waypoint: u32,
    pub kind: u32,
    pub boxes: Vec<Bounds>,
}

#[derive(serde::Deserialize)]
struct GateJson {
    waypoint: u32,
    kind: u32,
    #[serde(default)]
    boxes: Vec<Bounds>,
    min: Option<[f32; 3]>,
    max: Option<[f32; 3]>,
}

impl TryFrom<GateJson> for Gate {
    type Error = String;
    fn try_from(j: GateJson) -> Result<Self, String> {
        let mut boxes = j.boxes;
        match (j.min, j.max) {
            (Some(min), Some(max)) => boxes.insert(0, Bounds { min, max }),
            (None, None) => {}
            _ => return Err("gate needs both min and max".into()),
        }
        Ok(Self { waypoint: j.waypoint, kind: j.kind, boxes })
    }
}

impl Gate {
    pub fn distance(&self, point: [f32; 3]) -> f64 {
        self.boxes
            .iter()
            .map(|b| {
                (0..3)
                    .map(|k| {
                        let p = f64::from(point[k]);
                        (p - p.clamp(f64::from(b.min[k]), f64::from(b.max[k]))).powi(2)
                    })
                    .sum::<f64>()
                    .sqrt()
            })
            .fold(f64::INFINITY, f64::min)
    }

    pub fn validate(&self) -> Result<()> {
        ensure!(!self.boxes.is_empty(), "gate has no bounds");
        ensure!(
            self.boxes.iter().all(|b| (0..3).all(|k| b.min[k].is_finite()
                && b.max[k].is_finite()
                && b.min[k] <= b.max[k])),
            "invalid gate bounds"
        );
        Ok(())
    }
}

pub fn gates(native: &mut Session) -> Result<Vec<Gate>> {
    let items = native
        .waypoint_triggers()
        .into_iter()
        .map(|trigger| (trigger.waypoint, trigger.kind, Bounds { min: trigger.min, max: trigger.max }));
    let mut gates = std::collections::BTreeMap::<u32, Gate>::new();
    for (waypoint, kind, bounds) in items {
        gates
            .entry(waypoint)
            .or_insert_with(|| Gate { waypoint, kind, boxes: Vec::new() })
            .boxes
            .push(bounds);
    }
    for gate in gates.values() {
        gate.validate()?;
    }
    Ok(gates.into_values().collect())
}

pub fn target_gate(native: &mut Session, ordinal: usize) -> Result<Gate> {
    ensure!(ordinal > 0, "waypoint indices start at 1");
    let gates = gates(native)?;
    let order = |g: &Gate| native.waypoint_order(g.waypoint).unwrap_or(u32::MAX);
    let (mut checkpoints, finishes): (Vec<_>, Vec<_>) = gates.into_iter().partition(|g| g.kind != 1);
    checkpoints.sort_by_cached_key(|g| (order(g), g.waypoint));
    if checkpoints.len() > 1 {
        let mut orders = std::collections::BTreeSet::new();
        ensure!(
            checkpoints.iter().all(|g| {
                let o = order(g);
                o != u32::MAX && orders.insert(o)
            }),
            "unordered checkpoints: select an explicit waypoint ID or supply a reference run"
        );
    }
    if ordinal <= checkpoints.len() {
        return Ok(checkpoints.swap_remove(ordinal - 1));
    }
    ensure!(
        ordinal == checkpoints.len() + 1,
        "no gate for ordinal {ordinal} ({} checkpoints, then the finish); \
         laps need an explicit waypoint gate",
        checkpoints.len()
    );
    match <[Gate; 1]>::try_from(finishes) {
        Ok([finish]) => Ok(finish),
        Err(finishes) if finishes.is_empty() => bail!("the map has no finish gate"),
        Err(finishes) => bail!(
            "ordinal {ordinal} is the finish and the map has {} alternate finishes \
             (waypoint IDs {}): select one with --waypoint or supply a reference run",
            finishes.len(),
            finishes.iter().map(|g| g.waypoint.to_string()).collect::<Vec<_>>().join(", ")
        ),
    }
}

pub fn crossing_waypoint(native: &mut Session, ordinal: usize) -> Option<u32> {
    native.crossing_waypoint(ordinal)
}

pub fn reference_gate(
    native: &mut Session,
    ordinal: usize,
    inputs: &crate::input::Timeline,
    until_ms: i32,
) -> Result<Option<Gate>> {
    let saved = native.save();
    let result = (|| {
        let start = inputs.index(native.race_ms())?;
        let end = inputs.index(until_ms)?;
        for c in &inputs.controls[start..end] {
            native.step(*c);
            if native.race().is_some_and(|r| r.checkpoints.len() >= ordinal) {
                let wp = crossing_waypoint(native, ordinal).context("crossing has no waypoint ID")?;
                return Ok(gates(native)?.into_iter().find(|g| g.waypoint == wp));
            }
        }
        Ok(None)
    })();
    native.restore(&saved);
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_is_to_the_nearest_box_and_old_json_still_loads() {
        let gate: Gate = serde_json::from_str(
            r#"{"waypoint":3,"kind":1,"min":[0,0,0],"max":[1,1,1],
                "boxes":[{"min":[10,0,0],"max":[11,1,1]}]}"#,
        )
        .unwrap();
        assert_eq!(gate.boxes.len(), 2);
        gate.validate().unwrap();
        assert_eq!(gate.distance([10.5, 0.5, 0.5]), 0.0);
        assert_eq!(gate.distance([0.5, 0.5, 0.5]), 0.0);
        assert!((gate.distance([5.0, 0.5, 0.5]) - 4.0).abs() < 1e-12);
        let round_trip: Gate = serde_json::from_str(&serde_json::to_string(&gate).unwrap()).unwrap();
        assert_eq!(round_trip, gate);
        assert!(serde_json::from_str::<Gate>(r#"{"waypoint":3,"kind":1,"min":[0,0,0]}"#).is_err());
        let empty = Gate { boxes: Vec::new(), ..gate };
        assert!(empty.validate().is_err());
    }
}
