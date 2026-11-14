//! `gear.planetary`: the parts, joints and links of a planetary stage, the rates they turn at,
//! where they sit, and that their teeth miss each other at rest and turning. Expected values are
//! analytic: with the ring held the carrier turns sun / (sun + ring) of the sun, and each planet
//! turns -ring / planet of the carrier on the carrier.

use std::f64::consts::TAU;

use serde_json::{Value, json};

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

/// A joint's first value as `joint.list` gives it (degrees for a revolute joint).
fn value(s: &mut Session, joint: &str) -> f64 {
    let l = run(s, "joint.list", json!({"quiet": true}));
    let j = l["joints"].as_array().into_iter().flatten().find(|j| j["name"] == joint).cloned().unwrap_or_default();
    j["values"][0]["value"].as_f64().unwrap_or(f64::NAN)
}

fn close(got: f64, want: f64, tol: f64) -> bool {
    (got - want).abs() <= tol
}

/// An occurrence's centre of mass in the world.
fn centre(s: &mut Session, occurrence: &Value) -> [f64; 3] {
    let p = run(s, "occurrence.properties", json!({"occurrence": occurrence}));
    let c = &p["total"]["center_of_mass"];
    [0, 1, 2].map(|i| c[i].as_f64().unwrap_or(f64::NAN))
}

/// The pairs of bodies that overlap now.
fn overlaps(s: &mut Session) -> Vec<Value> {
    let r = run(s, "inspect.interference", json!({}));
    assert!(r["unchecked"].as_array().is_none_or(Vec::is_empty), "{r}");
    r["interferences"].as_array().cloned().unwrap_or_default()
}

/// A 12-tooth sun, three 18-tooth planets and a 48-tooth ring of module 1, 5 mm thick, with
/// printing play: few flank points keep it quick.
fn stage(s: &mut Session, extra: Value) -> Value {
    let mut p = json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "face_width": 5, "backlash": 0.3, "flank_points": 4});
    if let (Some(p), Some(extra)) = (p.as_object_mut(), extra.as_object()) {
        p.extend(extra.clone());
    }
    run(s, "gear.planetary", p)
}

#[test]
fn a_stage_turns_at_the_rates_its_teeth_give() {
    let mut s = Session::default();
    let r = stage(&mut s, json!({}));
    assert_eq!(r["ratio"].as_f64(), Some(5.0), "{r}");
    assert_eq!(r["centre_distance"].as_f64(), Some(15.0));
    assert_eq!(r["planets"]["occurrences"].as_array().map(Vec::len), Some(3));
    assert_eq!(r["links"], json!([0, 1, 2, 3]));
    assert_eq!(r["ring"]["gear"]["teeth"], 48);
    // The 18-tooth planets' tips would dig into the 12-tooth sun's roots, and the ring's into
    // the planets': both are cut back, and the reply says so.
    assert_eq!(r["planets"]["gear"]["tip_cut_from"].as_f64(), Some(20.0));
    assert_eq!(r["ring"]["gear"]["tip_cut_from"].as_f64(), Some(46.0));
    assert!(r["sun"]["gear"].get("tip_cut_from").is_none());
    assert_eq!(r["notes"].as_array().map(Vec::len), Some(2), "{r}");
    assert!(r["checks"]["contact_ratio_sun_planet"].as_f64().is_some_and(|c| c > 1.2), "{r}");
    // The ring holds still; one drive of the sun turns everything.
    let l = run(&mut s, "joint.list", json!({}));
    assert!(l["fixed"].as_array().is_some_and(|f| f.contains(&r["ring"]["occurrence"])), "{l}");
    assert!(l["conflicts"].as_array().is_none_or(Vec::is_empty), "{l}");
    let d = run(&mut s, "joint.drive", json!({"joint": "Planetary sun", "value": 90}));
    assert_eq!(d["moved"], true, "{d}");
    assert!(close(value(&mut s, "Planetary carrier"), 18.0, 1e-6));
    for k in 1..=3 {
        // 18 degrees of carrier turn each planet 48/18 of that backwards on its pin.
        assert!(close(value(&mut s, &format!("Planetary planet {k}")), -48.0, 1e-6));
    }
}

#[test]
fn the_parts_sit_on_their_axes() {
    let mut s = Session::default();
    let r = stage(&mut s, json!({"origin": [10, -5, 20]}));
    // Each part is symmetric about its own axis, so its centre of mass is on it, halfway up.
    for part in ["sun", "ring"] {
        let c = centre(&mut s, &r[part]["occurrence"]);
        assert!(close(c[0], 10.0, 1e-3) && close(c[1], -5.0, 1e-3) && close(c[2], 22.5, 1e-3), "{part}: {c:?}");
    }
    let occs = r["planets"]["occurrences"].as_array().cloned().unwrap_or_default();
    for (k, occ) in occs.iter().enumerate() {
        let a = TAU * k as f64 / 3.0;
        let c = centre(&mut s, occ);
        assert!(close(c[0], 10.0 + 15.0 * a.cos(), 1e-3) && close(c[1], -5.0 + 15.0 * a.sin(), 1e-3) && close(c[2], 22.5, 1e-3), "planet {k}: {c:?}");
    }
    assert_eq!(r["height"].as_f64(), Some(5.0 + 0.5 + 2.5));
}

#[test]
fn the_teeth_miss_each_other_at_rest_and_turning() {
    // Exact intersection of the bodies, not their meshes: every planet's teeth fall into the
    // spaces of the sun and the ring, and stay there as the stage turns.
    let mut s = Session::default();
    stage(&mut s, json!({}));
    let o = overlaps(&mut s);
    assert!(o.is_empty(), "{o:?}");
    // Not a whole tooth pitch, so the teeth are somewhere else in the mesh.
    run(&mut s, "joint.drive", json!({"joint": "Planetary sun", "value": 13}));
    let o = overlaps(&mut s);
    assert!(o.is_empty(), "{o:?}");
}

#[test]
fn a_planet_turned_off_its_phase_runs_into_the_sun() {
    // The check above can fail: turn one planet on its pin with the links gone and its teeth
    // hit the sun's or the ring's.
    let mut s = Session::default();
    let r = stage(&mut s, json!({"links": false}));
    assert_eq!(r["links"], json!([]));
    run(&mut s, "joint.drive", json!({"joint": "Planetary planet 2", "value": 5}));
    let o = overlaps(&mut s);
    assert!(!o.is_empty());
}

#[test]
fn without_links_each_part_turns_on_its_own() {
    let mut s = Session::default();
    stage(&mut s, json!({"links": false}));
    run(&mut s, "joint.drive", json!({"joint": "Planetary sun", "value": 90}));
    assert!(close(value(&mut s, "Planetary sun"), 90.0, 1e-6));
    assert!(close(value(&mut s, "Planetary carrier"), 0.0, 1e-9));
}

#[test]
fn a_second_stage_rides_on_the_first_carrier() {
    // Two stages, 5:1 then 4:1 (15 and 15 teeth in a 45-tooth ring): 20:1 in all.
    let mut s = Session::default();
    let first = stage(&mut s, json!({"name": "Stage 1"}));
    let top = first["height"].as_f64().unwrap_or(f64::NAN) + 0.5;
    let second = run(
        &mut s,
        "gear.planetary",
        json!({"module": 1, "sun_teeth": 15, "planet_teeth": 15, "face_width": 5, "backlash": 0.3, "flank_points": 4,
               "name": "Stage 2", "origin": [0, 0, top], "input": "Stage 1 carrier"}),
    );
    assert_eq!(second["ratio"].as_f64(), Some(4.0));
    // The second sun is fixed to the first carrier, the second ring to the first ring.
    let l = run(&mut s, "joint.list", json!({"quiet": true}));
    let kind = |name: &str| l["joints"].as_array().into_iter().flatten().find(|j| j["name"] == name).map(|j| j["type"].clone());
    assert_eq!(kind("Stage 2 sun mount"), Some(json!("rigid")), "{l}");
    assert_eq!(kind("Stage 2 ring mount"), Some(json!("rigid")), "{l}");
    run(&mut s, "joint.drive", json!({"joint": "Stage 1 sun", "value": 200}));
    assert!(close(value(&mut s, "Stage 1 carrier"), 40.0, 1e-6));
    assert!(close(value(&mut s, "Stage 2 carrier"), 10.0, 1e-6));
    assert!(close(value(&mut s, "Stage 2 planet 1"), -30.0, 1e-6));
}

#[test]
fn an_input_on_another_axis_is_refused() {
    let mut s = Session::default();
    stage(&mut s, json!({"name": "Stage 1"}));
    let e = s
        .execute("gear.planetary", &json!({"module": 1, "sun_teeth": 15, "planet_teeth": 15, "origin": [40, 0, 0], "input": "Stage 1 carrier"}))
        .map(|_| ())
        .unwrap_err()
        .to_string();
    assert!(e.contains("another axis"), "{e}");
}

#[test]
fn hostile_stages_are_refused_and_leave_the_design_alone() {
    let mut s = Session::default();
    for p in [
        json!({}),
        json!({"module": 1, "sun_teeth": 12}),
        json!({"module": 0, "sun_teeth": 12, "planet_teeth": 18}),
        json!({"module": -1, "sun_teeth": 12, "planet_teeth": 18}),
        json!({"module": 1e300, "sun_teeth": 12, "planet_teeth": 18}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "ring_teeth": 47}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "planets": 7}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "planets": 6}),
        json!({"module": 1, "sun_teeth": 4294967295u64, "planet_teeth": 4294967295u64}),
        json!({"module": 1, "sun_teeth": -12, "planet_teeth": 18}),
        json!({"module": 1, "sun_teeth": 12.5, "planet_teeth": 18}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "pressure_angle": 90}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "backlash": 5}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "face_width": 0}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "planet_bore": 100}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "pin_diameter": 8, "planet_bore": 6}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "sun_bore": 50}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "flank_points": 100000}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "origin": [1, 2]}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "name": ""}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "links": "yes"}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "input": "no such joint"}),
        json!({"module": 1, "sun_teeth": 12, "planet_teeth": 18, "housing": "no such part"}),
    ] {
        assert!(s.execute("gear.planetary", &p).is_err(), "{p}");
    }
    let l = run(&mut s, "joint.list", json!({}));
    assert!(l["joints"].as_array().is_none_or(Vec::is_empty), "{l}");
    let c = run(&mut s, "component.list", json!({}));
    assert!(!c.to_string().contains("Planetary"), "{c}");
}

#[test]
fn a_stage_is_one_undo_step() {
    let mut s = Session::default();
    stage(&mut s, json!({}));
    run(&mut s, "edit.undo", json!({}));
    let l = run(&mut s, "joint.list", json!({}));
    assert!(l["joints"].as_array().is_none_or(Vec::is_empty), "{l}");
    let c = run(&mut s, "component.list", json!({}));
    assert!(!c.to_string().contains("Planetary"), "{c}");
}
