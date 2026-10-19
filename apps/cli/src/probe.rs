use anyhow::{Result, ensure};
use std::{io::Write, path::Path};
use gearbox_search::{
    input::Timeline,
    tricks::{Detector, Observation},
};
pub fn main(args: Vec<String>) -> Result<()> {
    let args: Vec<String> = std::iter::once("gearbox probe".to_string()).chain(args).collect();
    ensure!(args.len() == 4, "usage: gearbox probe MAP INPUTS CSV");
    let inputs = Timeline::read(Path::new(&args[2]))?;
    let mut session = gearbox_search::session::open(&crate::packs()?, Path::new(&args[1]), Some(&inputs))?;
    ensure!(
        session.race_ms() == inputs.t0,
        "input coverage must begin at session start"
    );
    let saved = session.save();
    let mut csv = std::io::BufWriter::new(std::fs::File::create(&args[3])?);
    writeln!(
        csv,
        "ms,speed,slip_deg,pitch_deg,fl,fr,rr,rl,hard_contact,bugslide,nosebug,uberbug"
    )?;
    let mut detector = Detector::default();
    let mut bodies = Vec::new();
    for c in &inputs.controls {
        session.step(*c);
        let body = gearbox_search::Body::from(session.body());
        bodies.push(body.0.map(f32::to_bits));
        let o = Observation::read(&mut session, body);
        let prev = detector.events;
        detector.observe(o);
        let e = detector.events;
        if e != prev && (e.bugslide_ticks == 1 || e.nosebug_ticks == 1 || e.uberbug_ticks == 1) {
            eprintln!("signature at {} ms: {:?}", session.race_ms(), e);
        }
        writeln!(
            csv,
            "{},{:.6},{:.3},{:.3},{},{},{},{},{},{},{},{}",
            session.race_ms(),
            o.speed,
            o.slip.to_degrees(),
            o.pitch.to_degrees(),
            o.wheels[0],
            o.wheels[1],
            o.wheels[2],
            o.wheels[3],
            o.body_contact,
            e.bugslide_ticks,
            e.nosebug_ticks,
            e.uberbug_ticks
        )?;
    }
    csv.flush()?;
    println!(
        "signatures {:?}; race {:?}",
        detector.events,
        session.race()
    );
    session.restore(&saved);
    for (i, (c, body)) in inputs.controls.iter().zip(bodies).enumerate() {
        session.step(*c);
        ensure!(
            gearbox_search::Body::from(session.body()).0.map(f32::to_bits) == body,
            "replay differs at tick {i}"
        );
    }
    println!(
        "full trajectory replay: exact ({} ticks)",
        inputs.controls.len()
    );
    Ok(())
}
