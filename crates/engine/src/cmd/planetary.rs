//! `gear.planetary`: a planetary gear stage in one step. A sun, evenly spaced planets on a
//! carrier and a ring, each a component built from sketches and extrudes like any other part;
//! the joints a bearing would give each of them; the motion links their teeth give them; and the
//! checks a designer makes by hand. The geometry is worked out in `planetary_geometry`.

use serde_json::{Value, json};
use solidcore_record::{Vec2, Vec3};
use solidcraft_doc::joints::JointKind;
use solidcraft_sketch::CurveKind;

use super::CommandSpec;
use super::planetary_geometry::{Gear, Plan, Stage, outline, plan};
use crate::params::{bad, num, str_};
use crate::schema::planetary::{GearCircles, PlanetaryParams, PlanetaryStage, StageChecks, StagePart, StagePlanets};
use crate::schema::{TypedContract, output_schema_for};
use crate::{Result, Session};

const CMD: &str = "gear.planetary (Planetary Gears)";

const CONTRACT: TypedContract<PlanetaryParams> = TypedContract::new();

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("gear.planetary", "Planetary Gears", crate::checked!("gear.planetary", planetary_atomic))
    .icon("gear")
    .contract(&CONTRACT)
    .output(output_schema_for::<PlanetaryStage>)
    .params(
        "module (mm), sun_teeth, planet_teeth (6..1000 each), ring_teeth? (sun + 2 x planet, the default), planets? (2..32, default 3; sun + ring \
         a multiple of it), pressure_angle? (deg, default 20), face_width? (mm, default 10 x module), backlash? (mm per mesh, default 0; printed \
         gears want 0.1 to 0.2), rim? (mm of ring outside its roots, default 3 x module), pin_diameter?, planet_bore?, sun_bore?, \
         carrier_thickness?, axial_gap? (mm), origin?: [x,y,z] (the axis runs along +Z from it), name? (default Planetary), input?: a revolute \
         joint about the same axis that turns the sun (the sun is fixed to the part it turns, as on the carrier of the stage before), housing?: \
         the occurrence the ring is fixed to (default: input's other part, else the ring is grounded), links?: bool (default true: the carrier \
         and planets follow the sun), flank_points? (2..64, default 8) → ratio, sun, planets, ring, carrier (component, occurrence(s), \
         joint(s), gear circles), centre_distance, height, links, checks {neighbour_gap, contact ratios}, notes (tips cut back), warnings",
    )];

/// A number parameter that may be left out; one that is present but not a usable number is refused.
fn optional(p: &Value, k: &str) -> Result<Option<f64>> {
    match p.get(k) {
        None | Some(Value::Null) => Ok(None),
        Some(_) => num(p, k).map(Some).ok_or_else(|| bad(CMD, format!("`{k}` must be a finite number below 1e9"))),
    }
}

/// A whole number parameter that may be left out.
fn count(p: &Value, k: &str) -> Result<Option<u32>> {
    match p.get(k) {
        None | Some(Value::Null) => Ok(None),
        Some(v) => v.as_u64().and_then(|n| u32::try_from(n).ok()).map(Some).ok_or_else(|| bad(CMD, format!("`{k}` must be a whole number"))),
    }
}

/// A length above 0, `default` when left out.
fn length(p: &Value, k: &str, default: f64) -> Result<f64> {
    let v = optional(p, k)?.unwrap_or(default);
    if v > 0.0 { Ok(v) } else { Err(bad(CMD, format!("`{k}` must be above 0"))) }
}

/// How the parts around the gears are built.
struct Build {
    width: f64,
    rim: f64,
    pin_radius: f64,
    bore_radius: f64,
    sun_bore_radius: Option<f64>,
    plate: f64,
    gap: f64,
    origin: Vec3,
    name: String,
    links: bool,
    flank_points: u32,
}

fn read(p: &Value) -> Result<(Stage, Build)> {
    let module = num(p, "module").ok_or_else(|| bad(CMD, "`module` is required (a positive number of mm)"))?;
    let sun = count(p, "sun_teeth")?.ok_or_else(|| bad(CMD, "`sun_teeth` is required (a whole number, 6 to 1000)"))?;
    let planet = count(p, "planet_teeth")?.ok_or_else(|| bad(CMD, "`planet_teeth` is required (a whole number, 6 to 1000)"))?;
    // Out-of-range counts are refused by the plan; saturate so the default ring can't overflow.
    let ring = count(p, "ring_teeth")?.unwrap_or_else(|| sun.saturating_add(planet.saturating_mul(2)));
    let stage = Stage {
        module,
        sun,
        planet,
        ring,
        planets: count(p, "planets")?.unwrap_or(3),
        pressure_angle: optional(p, "pressure_angle")?.unwrap_or(20.0).to_radians(),
        backlash: optional(p, "backlash")?.unwrap_or(0.0),
    };
    let width = length(p, "face_width", module * 10.0)?;
    let origin = match p.get("origin") {
        None | Some(Value::Null) => Vec3::new(0.0, 0.0, 0.0),
        Some(v) => crate::params::vec3(v).ok_or_else(|| bad(CMD, "`origin` must be [x, y, z] in mm"))?,
    };
    let name = match p.get("name") {
        None | Some(Value::Null) => "Planetary".to_string(),
        Some(_) => match str_(p, "name").map(str::trim) {
            Some(n) if !n.is_empty() && n.chars().count() <= 200 => n.to_string(),
            _ => return Err(bad(CMD, "`name` must be text, 1 to 200 characters")),
        },
    };
    let links = match p.get("links") {
        None | Some(Value::Null) => true,
        Some(v) => v.as_bool().ok_or_else(|| bad(CMD, "`links` must be true or false"))?,
    };
    let gap = optional(p, "axial_gap")?.unwrap_or(0.5);
    if gap < 0.0 {
        return Err(bad(CMD, "`axial_gap` must be 0 or more"));
    }
    let build = Build {
        width,
        rim: length(p, "rim", module * 3.0)?,
        // Filled in by `sizes` once the plan gives the planets' roots.
        pin_radius: optional(p, "pin_diameter")?.map_or(0.0, |d| d / 2.0),
        bore_radius: optional(p, "planet_bore")?.map_or(0.0, |d| d / 2.0),
        sun_bore_radius: optional(p, "sun_bore")?.map(|d| d / 2.0),
        plate: length(p, "carrier_thickness", width / 2.0)?,
        gap,
        origin,
        name,
        links,
        flank_points: count(p, "flank_points")?.unwrap_or(8),
    };
    if p.get("pin_diameter").is_some_and(|v| !v.is_null()) && build.pin_radius <= 0.0 {
        return Err(bad(CMD, "`pin_diameter` must be above 0"));
    }
    if p.get("planet_bore").is_some_and(|v| !v.is_null()) && build.bore_radius <= 0.0 {
        return Err(bad(CMD, "`planet_bore` must be above 0"));
    }
    if build.sun_bore_radius.is_some_and(|r| r <= 0.0) {
        return Err(bad(CMD, "`sun_bore` must be above 0"));
    }
    if !(2..=64).contains(&build.flank_points) {
        return Err(bad(CMD, "`flank_points` must be 2 to 64"));
    }
    Ok((stage, build))
}

/// Fill in the pin and bore sizes the plan leaves to the planets' roots, and refuse holes that
/// would cut into the teeth.
fn sizes(plan: &Plan, b: &mut Build) -> Result<()> {
    let root = plan.planet.root_radius;
    if b.pin_radius <= 0.0 {
        b.pin_radius = 0.4 * root;
    }
    if b.bore_radius <= 0.0 {
        b.bore_radius = b.pin_radius + 0.1;
    }
    if b.bore_radius < b.pin_radius {
        return Err(bad(CMD, format!("the planet bore ({:.4} mm) is narrower than the pin ({:.4} mm)", 2.0 * b.bore_radius, 2.0 * b.pin_radius)));
    }
    if b.bore_radius >= root {
        return Err(bad(CMD, format!("a planet bore of {:.4} mm cuts into the planets' {:.4} mm root circle", 2.0 * b.bore_radius, 2.0 * root)));
    }
    if let Some(r) = b.sun_bore_radius
        && r >= plan.sun.root_radius
    {
        return Err(bad(CMD, format!("a sun bore of {:.4} mm cuts into the sun's {:.4} mm root circle", 2.0 * r, 2.0 * plan.sun.root_radius)));
    }
    Ok(())
}

/// The joint that turns the sun, from `input`.
#[derive(Clone, Copy)]
struct Input {
    joint: u64,
    /// The occurrence it turns, which the sun is fixed to.
    turns: u64,
    /// The occurrence it turns against.
    against: u64,
    /// -1 when the joint's axis points along -Z, so its angles turn the sun the other way.
    sign: f64,
}

fn input_joint(s: &Session, v: &Value, origin: Vec3, scale: f64) -> Result<Input> {
    let id = super::joints::joint_id(s, Some(v), CMD)?;
    let j = s.doc.assembly.joints.iter().find(|j| j.id == id).ok_or_else(|| bad(CMD, "no such `input` joint"))?;
    if j.kind != JointKind::Revolute {
        return Err(bad(CMD, format!("`input` must be a revolute joint, and `{}` is not", j.name)));
    }
    let frame = s.doc.origin_world(&s.model.state(), &j.a).map_err(|e| bad(CMD, e.to_string()))?;
    let (z, at) = (frame[2], frame[3]);
    let len = (z[0] * z[0] + z[1] * z[1] + z[2] * z[2]).sqrt();
    let tilt = z[0].hypot(z[1]) / len;
    let off = (at[0] - origin.x).hypot(at[1] - origin.y);
    if !(len > 0.0) || !(tilt <= 1e-6) || !(off <= 1e-6 * scale.max(1.0)) {
        return Err(bad(
            CMD,
            format!(
                "`input` `{}` turns about another axis: the stage turns about Z through ({}, {}), its joint through ({:.6}, {:.6}) along ({:.6}, {:.6}, {:.6})",
                j.name,
                origin.x,
                origin.y,
                at[0],
                at[1],
                z[0] / len,
                z[1] / len,
                z[2] / len
            ),
        ));
    }
    Ok(Input { joint: id, turns: j.b.occurrence, against: j.a.occurrence, sign: z[2].signum() })
}

fn planetary_atomic(s: &mut Session, p: &Value) -> Result<Value> {
    let mut computed = s.scratch();
    let result = planetary(&mut computed, p)?;
    computed.undo.clear();
    computed.active_component = s.active_component;
    computed.active_occurrence_path = s.active_occurrence_path.clone();
    computed.active_sketch = s.active_sketch.filter(|id| computed.doc.feature(*id).is_some());
    s.accept_computed(computed);
    Ok(result)
}

fn sketch_err(e: impl std::fmt::Display) -> crate::EngineError {
    bad(CMD, e.to_string())
}

fn id(v: &Value, k: &str) -> Result<u64> {
    v[k].as_u64().ok_or_else(|| bad(CMD, format!("no `{k}` in a step's reply")))
}

fn component_name(s: &Session, component: u64) -> String {
    s.doc.components.iter().find(|c| c.id == component).map(|c| c.name.clone()).unwrap_or_default()
}

/// One gear as a component of its own at the origin: its outline sketched on XY with `circles`
/// about the axis (a bore inside the outline, or the ring's outside around it), extruded along
/// +Z. With `material`, the extrude takes the region at that point of the sketch, with the
/// loops inside it as its holes; without, everything drawn.
fn gear_part(s: &mut Session, b: &Build, name: &str, g: &Gear, circles: &[f64], material: Option<[f64; 2]>) -> Result<(u64, u64)> {
    let points = outline(g, b.flank_points).map_err(|e| bad(CMD, format!("{name}: {e}")))?;
    let flank = points.len() / (2 * g.teeth as usize).max(1);
    let active = s.active_component;
    let c = s.execute("component.create", &json!({"name": name, "grounded": false}))?;
    let (comp, occ) = (id(&c, "component")?, id(&c, "occurrence")?);
    s.execute("sketch.create", &json!({"plane": "XY"}))?;
    // Each flank is one spline through its points; between flanks the outline runs along a
    // circle about the axis, across a tip or a root. All in one sketch edit.
    super::sketch::edit(s, &json!({}), CMD, false, |sk, _| {
        let centre = sk.add_point(Vec2::new(0.0, 0.0), None).map_err(sketch_err)?;
        let mut ids = Vec::with_capacity(points.len());
        for q in &points {
            ids.push(sk.add_point(Vec2::new(q[0], q[1]), None).map_err(sketch_err)?);
        }
        let flanks = ids.len() / flank;
        for i in 0..flanks {
            let chunk = &ids[i * flank..(i + 1) * flank];
            let kind = CurveKind::Spline { pts: chunk.to_vec(), control: false, degree: 3, periodic: false, nurbs: None, handles: Vec::new() };
            sk.add_curve(kind, None).map_err(sketch_err)?;
            let (from, to) = ((i + 1) * flank - 1, ((i + 1) % flanks) * flank);
            let (pf, pt) = (points[from], points[to]);
            sk.add_arc(Vec2::new(0.0, 0.0), Vec2::new(pf[0], pf[1]), Vec2::new(pt[0], pt[1]), [Some(centre), Some(ids[from]), Some(ids[to])], None)
                .map_err(sketch_err)?;
        }
        for r in circles {
            sk.add_circle(Vec2::new(0.0, 0.0), *r, None, None).map_err(sketch_err)?;
        }
        Ok(())
    })?;
    // The edit changed the design without evaluating it: the extrude below finds its profiles,
    // and the holes in them, in the evaluated sketch, so evaluate it first.
    s.refresh();
    s.execute("sketch.finish", &json!({}))?;
    let mut extrude = json!({"distance": format!("{} mm", b.width), "direction": "positive", "operation": "new"});
    if let (Some(at), Some(e)) = (material, extrude.as_object_mut()) {
        e.insert("profiles".into(), json!([{"point": at}]));
        e.insert("exact_regions".into(), json!(true));
    }
    s.execute("solid.extrude", &extrude)?;
    s.execute("component.activate", &json!({"component": active}))?;
    Ok((comp, occ))
}

/// The carrier as a component at the origin: a plate above the gears and a pin up through each
/// planet into it.
fn carrier_part(s: &mut Session, b: &Build, name: &str, plan: &Plan) -> Result<(u64, u64)> {
    let active = s.active_component;
    let c = s.execute("component.create", &json!({"name": name, "grounded": false}))?;
    let (comp, occ) = (id(&c, "component")?, id(&c, "occurrence")?);
    let a = plan.centre_distance;
    let floor = b.width + b.gap;
    s.execute(
        "solid.cylinder",
        &json!({"radius": a + 2.0 * b.pin_radius, "height": b.plate, "base": [0.0, 0.0, floor], "axis": "Z", "operation": "new"}),
    )?;
    for angle in &plan.planet_angles {
        // Into the plate, not just up to it, so the pin and the plate share volume.
        let base = [a * angle.cos(), a * angle.sin(), 0.0];
        s.execute(
            "solid.cylinder",
            &json!({"radius": b.pin_radius, "height": floor + b.plate / 2.0, "base": base, "axis": "Z", "operation": "join"}),
        )?;
    }
    s.execute("component.activate", &json!({"component": active}))?;
    Ok((comp, occ))
}

/// Put an occurrence at `at`, turned `turn` radians about +Z.
fn place(s: &mut Session, occ: u64, at: Vec3, turn: f64) -> Result<()> {
    s.execute(
        "occurrence.move",
        &json!({"occurrence": occ, "space": "world", "to": {"position": [at.x, at.y, at.z], "axis": [0.0, 0.0, 1.0], "angle": turn}}),
    )?;
    Ok(())
}

fn as_built(s: &mut Session, p: Value) -> Result<u64> {
    let r = s.execute("joint.as_built", &p)?;
    id(&r, "joint")
}

fn link(s: &mut Session, a: u64, b: u64, ratio: f64) -> Result<usize> {
    s.execute("joint.motion_link", &json!({"a": a, "b": b, "ratio": ratio}))?;
    Ok(s.doc.assembly.links.len().saturating_sub(1))
}

fn circles(g: &Gear) -> GearCircles {
    GearCircles {
        teeth: g.teeth,
        pitch_diameter: 2.0 * g.pitch_radius,
        base_diameter: 2.0 * g.base_radius,
        tip_diameter: 2.0 * g.tip_radius,
        tip_cut_from: g.tip_cut().then_some(2.0 * g.standard_tip_radius),
        root_diameter: 2.0 * g.root_radius,
    }
}

fn planetary(s: &mut Session, p: &Value) -> Result<Value> {
    let (stage, mut b) = read(p)?;
    let plan = plan(&stage).map_err(|e| bad(CMD, e))?;
    sizes(&plan, &mut b)?;
    // What the stage hangs on, found before anything is made.
    let scale = plan.ring.root_radius + b.rim;
    let input = match p.get("input") {
        None | Some(Value::Null) => None,
        Some(v) => Some(input_joint(s, v, b.origin, scale)?),
    };
    let housing = match p.get("housing") {
        None | Some(Value::Null) => input.map(|i| i.against).filter(|o| *o != 0),
        Some(v) => Some(super::joints::occurrence_id(s, Some(v), CMD)?).filter(|o| *o != 0),
    };
    let n = &b.name;
    // The material of each holed part is picked by a point in it: the ring's rim, and the
    // middle of the first tooth's root between the bore and the bottom of the spaces.
    let rim = plan.ring.root_radius + b.rim;
    let (ring, ring_occ) = gear_part(s, &b, &format!("{n} ring"), &plan.ring, &[rim], Some([plan.ring.root_radius + b.rim / 2.0, 0.0]))?;
    let sun_holes: Vec<f64> = b.sun_bore_radius.into_iter().collect();
    let sun_material = b.sun_bore_radius.map(|r| [(r + plan.sun.root_radius) / 2.0, 0.0]);
    let (sun, sun_occ) = gear_part(s, &b, &format!("{n} sun"), &plan.sun, &sun_holes, sun_material)?;
    let planet_material = [(b.bore_radius + plan.planet.root_radius) / 2.0, 0.0];
    let (planet, first_planet) = gear_part(s, &b, &format!("{n} planet"), &plan.planet, &[b.bore_radius], Some(planet_material))?;
    let (carrier, carrier_occ) = carrier_part(s, &b, &format!("{n} carrier"), &plan)?;
    // Place everything before any joint holds it.
    let o = b.origin;
    place(s, ring_occ, o, plan.ring_turn)?;
    place(s, sun_occ, o, 0.0)?;
    place(s, carrier_occ, o, 0.0)?;
    let a = plan.centre_distance;
    let centres: Vec<Vec3> = plan.planet_angles.iter().map(|t| Vec3::new(o.x + a * t.cos(), o.y + a * t.sin(), o.z)).collect();
    let mut planet_occs = Vec::with_capacity(centres.len());
    for (k, (centre, turn)) in centres.iter().zip(&plan.planet_turns).enumerate() {
        let occ = if k == 0 { first_planet } else { id(&s.execute("occurrence.copy", &json!({"component": planet}))?, "occurrence")? };
        place(s, occ, *centre, *turn)?;
        planet_occs.push(occ);
    }
    // The bearings: the ring held, the sun and the carrier turning in it, each planet on its pin.
    let at = |occ: u64, x: f64, y: f64| json!({"occurrence": occ, "point": [x, y, o.z], "z": [0.0, 0.0, 1.0]});
    let ring_joint = match housing {
        Some(h) => Some(as_built(s, json!({"type": "rigid", "a": h, "b": ring_occ, "name": format!("{n} ring mount"), "quiet": true}))?),
        None => {
            s.execute("occurrence.ground", &json!({"occurrence": ring_occ, "grounded": true}))?;
            None
        }
    };
    let sun_joint = match input {
        Some(i) => as_built(s, json!({"type": "rigid", "a": i.turns, "b": sun_occ, "name": format!("{n} sun mount"), "quiet": true}))?,
        None => as_built(
            s,
            json!({"type": "revolute", "a": ring_occ, "b": sun_occ, "at": at(ring_occ, o.x, o.y), "name": format!("{n} sun"), "quiet": true}),
        )?,
    };
    let carrier_joint = as_built(
        s,
        json!({"type": "revolute", "a": ring_occ, "b": carrier_occ, "at": at(ring_occ, o.x, o.y), "name": format!("{n} carrier"), "quiet": true}),
    )?;
    let mut planet_joints = Vec::with_capacity(planet_occs.len());
    for (k, (occ, c)) in planet_occs.iter().zip(&centres).enumerate() {
        let name = format!("{n} planet {}", k + 1);
        planet_joints.push(as_built(
            s,
            json!({"type": "revolute", "a": carrier_occ, "b": occ, "at": at(carrier_occ, c.x, c.y), "name": name, "quiet": true}),
        )?);
    }
    // The teeth: carrier = sun x sun / (sun + ring); each planet on the carrier = carrier x
    // -ring / planet, with the ring held.
    let mut links = Vec::new();
    if b.links {
        let (driver, sign) = input.map_or((sun_joint, 1.0), |i| (i.joint, i.sign));
        links.push(link(s, driver, carrier_joint, sign * plan.carrier_per_sun)?);
        for j in &planet_joints {
            links.push(link(s, carrier_joint, *j, plan.planet_per_carrier)?);
        }
    }
    let reply = PlanetaryStage {
        ratio: plan.ratio,
        sun: StagePart { component: sun, occurrence: sun_occ, name: component_name(s, sun), joint: Some(sun_joint), gear: Some(circles(&plan.sun)) },
        planets: StagePlanets {
            component: planet,
            name: component_name(s, planet),
            occurrences: planet_occs,
            joints: planet_joints,
            gear: circles(&plan.planet),
        },
        ring: StagePart { component: ring, occurrence: ring_occ, name: component_name(s, ring), joint: ring_joint, gear: Some(circles(&plan.ring)) },
        carrier: StagePart { component: carrier, occurrence: carrier_occ, name: component_name(s, carrier), joint: Some(carrier_joint), gear: None },
        centre_distance: a,
        height: b.width + b.gap + b.plate,
        links,
        checks: StageChecks {
            neighbour_gap: plan.neighbour_gap,
            contact_ratio_sun_planet: plan.contact_ratio_sun_planet,
            contact_ratio_planet_ring: plan.contact_ratio_planet_ring,
        },
        notes: plan.notes.clone(),
        warnings: plan.warnings.clone(),
        param_warnings: Vec::new(),
    };
    serde_json::to_value(reply).map_err(|e| bad(CMD, e.to_string()))
}
