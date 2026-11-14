//! A planetary gear stage worked out from its tooth counts: the circles of the sun, planet and
//! ring gears, where each planet sits and how far each part turns so every tooth lands in a space
//! of its partner, the tips cut back where a tooth would dig into its partner's root, the rates
//! the parts turn at, and the checks a designer makes by hand.
//!
//! Pure geometry from the public involute equations, as in `gear_profile`: ISO 53 proportions
//! (addendum `m`, dedendum `1.25 m`) without profile shift. Lengths are mm, angles radians.

use std::f64::consts::{PI, TAU};

/// What a stage is made of.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stage {
    /// Module (mm), the same for the three gears.
    pub module: f64,
    /// Teeth of the sun, of each planet and of the ring.
    pub sun: u32,
    pub planet: u32,
    pub ring: u32,
    /// How many planets, spaced evenly around the sun.
    pub planets: u32,
    /// Pressure angle (radians).
    pub pressure_angle: f64,
    /// Play between the teeth of each mesh on the pitch circle (mm), shared evenly by the two
    /// gears: a sun or planet tooth is half of it thinner, a space of the ring half of it wider.
    pub backlash: f64,
}

/// One gear of the stage, in its own frame: the middle of a tooth (sun, planet) or of a space
/// (ring) on +x, the axis along +z.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gear {
    pub teeth: u32,
    /// The ring: its teeth point inwards, and its outline is the hole they leave.
    pub internal: bool,
    pub pitch_radius: f64,
    pub base_radius: f64,
    /// Where the teeth end: outside the sun and planets, inside the ring.
    pub tip_radius: f64,
    /// The tip radius of ISO 53, before any cut.
    pub standard_tip_radius: f64,
    /// The bottom of the spaces: inside the sun and planets, outside the ring.
    pub root_radius: f64,
    /// On the pitch circle, the width of a tooth (sun, planet) or of a space (ring).
    pub thickness: f64,
    pub pressure_angle: f64,
}

impl Gear {
    /// Whether the tip was cut back from its standard radius.
    pub fn tip_cut(&self) -> bool {
        (self.tip_radius - self.standard_tip_radius).abs() > 1e-9 * self.pitch_radius.max(1.0)
    }

    /// Half the angle a tooth (sun, planet) or a space (ring) spans at radius `r`. Both are
    /// bounded by involutes of the base circle that close in going outwards; below the base
    /// circle the flank runs along the radius.
    pub fn half_lobe(&self, r: f64) -> f64 {
        let rb = self.base_radius;
        let at_base = self.thickness / (2.0 * self.pitch_radius) + inv(self.pressure_angle);
        if r <= rb { at_base } else { at_base - inv((rb / r).clamp(-1.0, 1.0).acos()) }
    }

    /// The lobes (teeth, or the ring's spaces) run from `inner` to `outer`.
    fn lobe_span(&self) -> (f64, f64) {
        if self.internal { (self.tip_radius, self.root_radius) } else { (self.root_radius, self.tip_radius) }
    }

    /// How far along the line of action from where it touches the base circle the tip circle
    /// cuts it (0 for a tip inside the base circle).
    fn reach(&self) -> f64 {
        (self.tip_radius.powi(2) - self.base_radius.powi(2)).max(0.0).sqrt()
    }

    /// Whether the point `p` of this gear's own plane is in its material.
    #[cfg(test)]
    pub fn contains(&self, p: [f64; 2]) -> bool {
        let r = p[0].hypot(p[1]);
        let (inner, outer) = self.lobe_span();
        let pitch = TAU / f64::from(self.teeth);
        let a = p[1].atan2(p[0]);
        let rel = a - pitch * (a / pitch).round();
        let in_lobe = r < inner || (r <= outer && rel.abs() <= self.half_lobe(r));
        // A tooth is material; a space of the ring is the hole, and inside the ring's tips too.
        if self.internal { !in_lobe && r >= inner } else { in_lobe }
    }
}

fn inv(a: f64) -> f64 {
    a.tan() - a
}

/// An angle in (-pi, pi].
fn wrap(a: f64) -> f64 {
    let w = a - TAU * (a / TAU).round();
    if w <= -PI { w + TAU } else { w }
}

/// Everything about a stage that the model is built from.
#[derive(Clone, Debug, PartialEq)]
pub struct Plan {
    pub stage: Stage,
    pub sun: Gear,
    pub planet: Gear,
    pub ring: Gear,
    /// From the stage's axis to each planet's axis (mm).
    pub centre_distance: f64,
    /// Where each planet's axis is, as an angle about the stage's axis from +x.
    pub planet_angles: Vec<f64>,
    /// How far each planet is turned about its own axis from its frame (the middle of a tooth
    /// on +x), so its teeth fall into the spaces of the sun and the ring.
    pub planet_turns: Vec<f64>,
    /// How far the ring is turned about the axis from its frame.
    pub ring_turn: f64,
    /// Sun turns per carrier turn with the ring held: 1 + ring / sun.
    pub ratio: f64,
    /// Carrier turn per sun turn with the ring held: sun / (sun + ring).
    pub carrier_per_sun: f64,
    /// A planet's turn on the carrier per carrier turn with the ring held: -ring / planet.
    pub planet_per_carrier: f64,
    /// Between the tip circles of two neighbouring planets (mm).
    pub neighbour_gap: f64,
    /// Teeth in contact on average in each mesh (the contact ratio).
    pub contact_ratio_sun_planet: f64,
    pub contact_ratio_planet_ring: f64,
    /// A tip cut back, in words.
    pub notes: Vec<String>,
    /// What the designer should look at.
    pub warnings: Vec<String>,
}

/// The fewest teeth in contact on average that make a smooth mesh; below it the reply warns.
pub const SMOOTH_CONTACT_RATIO: f64 = 1.2;

/// Work out a stage, or say why it can't be made.
pub fn plan(st: &Stage) -> Result<Plan, String> {
    let m = st.module;
    if !(m.is_finite() && m > 0.0 && m <= 1e4) {
        return Err("the module must be a positive number of mm, at most 10000".into());
    }
    let alpha = st.pressure_angle;
    if !(alpha.is_finite() && alpha > 0.0 && alpha <= 35f64.to_radians()) {
        return Err("the pressure angle must be above 0 and at most 35 degrees".into());
    }
    if st.sun < 6 || st.planet < 6 || st.sun > 1000 || st.planet > 1000 {
        return Err("the sun and the planets take 6 to 1000 teeth each".into());
    }
    let need = st.sun + 2 * st.planet;
    if need > 1000 {
        return Err(format!("a sun of {} teeth and planets of {} take a ring of {need} teeth: at most 1000", st.sun, st.planet));
    }
    if st.ring != need {
        return Err(format!(
            "a sun of {} teeth and planets of {} take a ring of {need} teeth (sun + 2 x planet), not {}: other rings need profile-shifted gears, which this does not make",
            st.sun, st.planet, st.ring
        ));
    }
    if !(2..=32).contains(&st.planets) {
        return Err("a stage takes 2 to 32 planets".into());
    }
    if !(st.backlash.is_finite() && st.backlash >= 0.0 && st.backlash < PI * m / 2.0) {
        return Err(format!("the backlash must be 0 or more mm and less than half a tooth pitch, {:.4} mm", PI * m / 2.0));
    }
    let (ca, sa) = (alpha.cos(), alpha.sin());
    let radius = |z: u32| m * f64::from(z) / 2.0;
    let half_play = st.backlash / 2.0;
    let external = |z: u32| {
        let r = radius(z);
        Gear {
            teeth: z,
            internal: false,
            pitch_radius: r,
            base_radius: r * ca,
            tip_radius: r + m,
            standard_tip_radius: r + m,
            root_radius: r - 1.25 * m,
            thickness: PI * m / 2.0 - half_play,
            pressure_angle: alpha,
        }
    };
    let (mut sun, mut planet) = (external(st.sun), external(st.planet));
    let rr = radius(st.ring);
    let mut ring = Gear {
        teeth: st.ring,
        internal: true,
        pitch_radius: rr,
        base_radius: rr * ca,
        tip_radius: rr - m,
        standard_tip_radius: rr - m,
        root_radius: rr + 1.25 * m,
        thickness: PI * m / 2.0 + half_play,
        pressure_angle: alpha,
    };
    let a = sun.pitch_radius + planet.pitch_radius;
    // Both meshes have the planets' centre distance, so both lines of action run this far
    // between the points where they touch the two base circles.
    let e = a * sa;
    // A tip that reaches past where the line of action touches its partner's base circle would
    // dig into the partner's root, where the flank is no longer an involute (involute
    // interference). Cut it back to that point.
    let mut notes = Vec::new();
    let sun_limit = sun.base_radius.hypot(e);
    if sun.tip_radius > sun_limit {
        sun.tip_radius = sun_limit;
    }
    let planet_limit = planet.base_radius.hypot(e);
    if planet.tip_radius > planet_limit {
        planet.tip_radius = planet_limit;
    }
    let ring_limit = ring.base_radius.hypot(e);
    if ring.tip_radius < ring_limit {
        ring.tip_radius = ring_limit;
    }
    for (g, whose, into) in
        [(&sun, "the sun's", "the planets' roots"), (&planet, "the planets'", "the sun's roots"), (&ring, "the ring's", "the planets' roots")]
    {
        if g.tip_cut() {
            let (cut, past) = if g.internal { ("teeth are cut back on the inside", "further in") } else { ("tips are cut back", "further out") };
            notes.push(format!(
                "{whose} {cut} from {:.3} to {:.3} mm diameter: {past} they would dig into {into} (involute interference)",
                2.0 * g.standard_tip_radius,
                2.0 * g.tip_radius
            ));
        }
    }
    for (g, whose) in [(&sun, "the sun's"), (&planet, "the planets'"), (&ring, "the ring's")] {
        check_lobes(g, whose)?;
    }
    let n = f64::from(st.planets);
    let neighbour_gap = 2.0 * a * (PI / n).sin() - 2.0 * planet.tip_radius;
    if neighbour_gap <= 0.0 {
        let fit: Vec<u32> = (2..st.planets).filter(|k| 2.0 * a * (PI / f64::from(*k)).sin() > 2.0 * planet.tip_radius).collect();
        return Err(format!(
            "{} planets of {} teeth don't fit around a sun of {}: their tips overlap by {:.4} mm. At most {} fit; or use a bigger sun",
            st.planets,
            st.planet,
            st.sun,
            -neighbour_gap,
            fit.last().copied().unwrap_or(1)
        ));
    }
    // Evenly spaced planets mesh with the sun and the ring together only when sun + ring is a
    // multiple of the planet count.
    let sum = st.sun + st.ring;
    if !sum.is_multiple_of(st.planets) {
        let fit: Vec<String> = (2..=12u32)
            .filter(|k| sum.is_multiple_of(*k) && 2.0 * a * (PI / f64::from(*k)).sin() > 2.0 * planet.tip_radius)
            .map(|k| k.to_string())
            .collect();
        let others = if fit.is_empty() { "no other count fits either".to_string() } else { format!("{} planets fit", fit.join(", ")) };
        return Err(format!("{} planets can't be spaced evenly: sun + ring is {sum} teeth, not a multiple of {}; {others}", st.planets, st.planets));
    }
    let pb = PI * m * ca;
    let contact_ratio_sun_planet = (sun.reach() + planet.reach() - e) / pb;
    let contact_ratio_planet_ring = (planet.reach() - ring.reach() + e) / pb;
    let mut warnings = Vec::new();
    for (ratio, pair) in [(contact_ratio_sun_planet, "the sun and planets"), (contact_ratio_planet_ring, "the planets and ring")] {
        if ratio < 1.0 {
            warnings.push(format!(
                "{pair} have a contact ratio of {ratio:.3}: for part of each tooth's turn no pair of teeth touches, so the gears jump instead of turning each other smoothly. Use more teeth or a smaller pressure angle"
            ));
        } else if ratio < SMOOTH_CONTACT_RATIO {
            warnings.push(format!(
                "{pair} have a contact ratio of {ratio:.3}, below {SMOOTH_CONTACT_RATIO}: often one pair of teeth carries the whole load, so the mesh runs rough. Use more teeth or a smaller pressure angle"
            ));
        }
    }
    let planet_angles: Vec<f64> = (0..st.planets).map(|k| TAU * f64::from(k) / n).collect();
    // A sun tooth on +x faces planet 0. Planet k's teeth must leave a space where the sun's
    // pitch circle meets its own: the fraction of a sun tooth pitch at its angle, mirrored.
    let pitch_p = TAU / f64::from(st.planet);
    let planet_turns = planet_angles
        .iter()
        .zip(0..st.planets)
        .map(|(theta, k)| {
            let frac = f64::from((k * st.sun) % st.planets) / n;
            wrap(theta + PI - pitch_p * (0.5 - frac))
        })
        .collect();
    // The ring then needs a tooth where planet 0 leaves a space on +x.
    let ring_turn = wrap(PI * f64::from(st.planet - 1) / f64::from(st.ring));
    Ok(Plan {
        stage: *st,
        sun,
        planet,
        ring,
        centre_distance: a,
        planet_angles,
        planet_turns,
        ring_turn,
        ratio: 1.0 + f64::from(st.ring) / f64::from(st.sun),
        carrier_per_sun: f64::from(st.sun) / f64::from(sum),
        planet_per_carrier: -f64::from(st.ring) / f64::from(st.planet),
        neighbour_gap,
        contact_ratio_sun_planet,
        contact_ratio_planet_ring,
        notes,
        warnings,
    })
}

/// Refuse a gear whose teeth come to a point, or whose spaces close, before they end. `whose` is
/// the gear's name as a possessive ("the sun's").
fn check_lobes(g: &Gear, whose: &str) -> Result<(), String> {
    let (inner, outer) = g.lobe_span();
    let pitch = TAU / f64::from(g.teeth);
    if g.half_lobe(outer) <= 0.0 {
        return Err(if g.internal {
            format!("{whose} spaces close before their bottom: use more teeth, a smaller pressure angle or less backlash")
        } else {
            format!("{whose} teeth come to a point before their tips: use more teeth, a smaller pressure angle or less backlash")
        });
    }
    if pitch - 2.0 * g.half_lobe(inner) <= 0.0 {
        return Err(if g.internal {
            format!("{whose} teeth come to a point before their tips: use more teeth or less backlash")
        } else {
            format!("{whose} teeth touch each other at the root: use more teeth")
        });
    }
    Ok(())
}

/// The closed outline of a gear in its own frame, counter-clockwise: for each lobe (a tooth of
/// the sun or a planet, a space of the ring) the points up one flank from the inner end to the
/// outer one, then the same points down the other. Lobe `t` is centred at angle
/// `t * 2 pi / teeth`. A flank has `points` points along its involute; where the inner end is
/// below the base circle the flank first runs along the radius, from one more point where it
/// leaves the inner circle. Every flank has as many points, `len / (2 * teeth)`. Between two
/// flanks the outline runs along a circle about the origin (a land): across the outer end of a
/// lobe, then across the inner end of the gap to the next lobe.
pub fn outline(g: &Gear, points: u32) -> Result<Vec<[f64; 2]>, String> {
    if !(2..=64).contains(&points) {
        return Err("a flank needs between 2 and 64 points".into());
    }
    let (inner, outer) = g.lobe_span();
    let rb = g.base_radius;
    // Along the involute, evenly in its roll angle, so the points crowd where it bends most.
    let roll = |r: f64| ((r / rb).powi(2) - 1.0).max(0.0).sqrt();
    let start = inner.max(rb);
    let (t0, t1) = (roll(start), roll(outer));
    let mut flank = Vec::with_capacity(points as usize + 1);
    if inner < rb {
        flank.push((inner, g.half_lobe(rb)));
    }
    for i in 0..points {
        let r = if i == 0 {
            start
        } else if i + 1 == points {
            outer
        } else {
            rb * (t0 + (t1 - t0) * f64::from(i) / f64::from(points - 1)).hypot(1.0)
        };
        flank.push((r, g.half_lobe(r)));
    }
    let pitch = TAU / f64::from(g.teeth);
    let mut out = Vec::with_capacity(g.teeth as usize * flank.len() * 2);
    for t in 0..g.teeth {
        let c = pitch * f64::from(t);
        let at = |r: f64, a: f64| [r * (c + a).cos(), r * (c + a).sin()];
        out.extend(flank.iter().map(|&(r, h)| at(r, -h)));
        out.extend(flank.iter().rev().map(|&(r, h)| at(r, h)));
    }
    if out.iter().any(|p| !p[0].is_finite() || !p[1].is_finite()) {
        return Err("the gear outline is not finite".into());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stage(sun: u32, planet: u32, planets: u32) -> Stage {
        Stage { module: 1.0, sun, planet, ring: sun + 2 * planet, planets, pressure_angle: 20f64.to_radians(), backlash: 0.0 }
    }

    fn turn(p: [f64; 2], a: f64) -> [f64; 2] {
        let (s, c) = a.sin_cos();
        [p[0] * c - p[1] * s, p[0] * s + p[1] * c]
    }

    /// A gear placed in the stage: its centre, and how far it is turned.
    #[derive(Clone, Copy)]
    struct Placed<'a> {
        gear: &'a Gear,
        centre: [f64; 2],
        turn: f64,
    }

    impl Placed<'_> {
        fn world(&self, p: [f64; 2]) -> [f64; 2] {
            let q = turn(p, self.turn);
            [q[0] + self.centre[0], q[1] + self.centre[1]]
        }
        fn local(&self, p: [f64; 2]) -> [f64; 2] {
            turn([p[0] - self.centre[0], p[1] - self.centre[1]], -self.turn)
        }
    }

    /// The outline as drawn, densely: 24 points along each involute, 8 along each land.
    fn dense(g: &Gear) -> Vec<[f64; 2]> {
        let pts = outline(g, 24).unwrap();
        let per = pts.len() / (2 * g.teeth as usize);
        let mut out = Vec::new();
        for (i, p) in pts.iter().enumerate() {
            out.push(*p);
            if i % per == per - 1 {
                let q = pts[(i + 1) % pts.len()];
                let (r, a0) = (p[0].hypot(p[1]), p[1].atan2(p[0]));
                let mut a1 = q[1].atan2(q[0]);
                if a1 < a0 {
                    a1 += TAU;
                }
                out.extend((1..8).map(|j| {
                    let a = a0 + (a1 - a0) * f64::from(j) / 8.0;
                    [r * a.cos(), r * a.sin()]
                }));
            }
        }
        out
    }

    fn seg_dist(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
        (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
    }

    fn dist(p: [f64; 2], q: [f64; 2]) -> f64 {
        (p[0] - q[0]).hypot(p[1] - q[1])
    }

    /// The least gap between two placed gears' drawn outlines (`da`, `db`: their dense
    /// outlines) near `near`, where they mesh: negative when a point of one is inside the other,
    /// by how far it is from the other's outline. The least distance between two polylines is
    /// at a vertex of one of them, so both ways round are measured.
    fn gap(a: Placed, da: &[[f64; 2]], b: Placed, db: &[[f64; 2]], near: [f64; 2], window: f64) -> f64 {
        let world = |pl: &Placed, d: &[[f64; 2]]| -> Vec<[f64; 2]> { d.iter().map(|p| pl.world(*p)).collect() };
        let (wa, wb) = (world(&a, da), world(&b, db));
        let pts = |w: &[[f64; 2]]| -> Vec<[f64; 2]> { w.iter().copied().filter(|p| dist(*p, near) < window).collect() };
        let segs = |w: &[[f64; 2]]| -> Vec<([f64; 2], [f64; 2])> {
            (0..w.len())
                .map(|i| (w[i], w[(i + 1) % w.len()]))
                .filter(|(p, q)| dist(*p, near) < window + 1.0 || dist(*q, near) < window + 1.0)
                .collect()
        };
        let side = |pts: &[[f64; 2]], segs: &[([f64; 2], [f64; 2])], other: &Placed| {
            pts.iter()
                .map(|p| {
                    let d = segs.iter().map(|(s0, s1)| seg_dist(*p, *s0, *s1)).fold(f64::INFINITY, f64::min);
                    if other.gear.contains(other.local(*p)) { -d } else { d }
                })
                .fold(f64::INFINITY, f64::min)
        };
        side(&pts(&wa), &segs(&wb), &b).min(side(&pts(&wb), &segs(&wa), &a))
    }

    /// Turn the sun through one tooth pitch with the ring held, as the joints and links will, and
    /// give the least gap in the sun-planet and planet-ring meshes of planet `k`.
    fn sweep(p: &Plan, k: usize, steps: u32) -> (f64, f64) {
        let (ds, dp, dr) = (dense(&p.sun), dense(&p.planet), dense(&p.ring));
        let (mut sp, mut pr) = (f64::INFINITY, f64::INFINITY);
        let a = p.centre_distance;
        for i in 0..=steps {
            let sun_turn = TAU / f64::from(p.sun.teeth) * f64::from(i) / f64::from(steps);
            let carrier = p.carrier_per_sun * sun_turn;
            let angle = p.planet_angles[k] + carrier;
            let centre = [a * angle.cos(), a * angle.sin()];
            let planet_turn = p.planet_turns[k] + carrier + p.planet_per_carrier * carrier;
            let sun = Placed { gear: &p.sun, centre: [0.0, 0.0], turn: sun_turn };
            let planet = Placed { gear: &p.planet, centre, turn: planet_turn };
            let ring = Placed { gear: &p.ring, centre: [0.0, 0.0], turn: p.ring_turn };
            let window = 4.0 * p.stage.module;
            let toward = |r: f64| [r * angle.cos(), r * angle.sin()];
            sp = sp.min(gap(sun, &ds, planet, &dp, toward(p.sun.pitch_radius), window));
            pr = pr.min(gap(planet, &dp, ring, &dr, toward(p.ring.pitch_radius), window));
        }
        (sp, pr)
    }

    #[test]
    fn a_stage_has_the_textbook_sizes_and_rates() {
        let p = plan(&stage(12, 18, 3)).unwrap();
        assert_eq!(p.ring.teeth, 48);
        assert!((p.centre_distance - 15.0).abs() < 1e-12);
        assert!((p.ratio - 5.0).abs() < 1e-12);
        assert!((p.carrier_per_sun - 0.2).abs() < 1e-12);
        assert!((p.planet_per_carrier + 48.0 / 18.0).abs() < 1e-12);
        assert!((p.sun.root_radius - 4.75).abs() < 1e-12 && (p.ring.root_radius - 25.25).abs() < 1e-12);
        // Planets 120 degrees apart; tips 14.0 apart... their tip circles leave 2 x 15 sin 60 - 20.
        assert!((p.neighbour_gap - (30.0 * (PI / 3.0).sin() - 2.0 * p.planet.tip_radius)).abs() < 1e-12);
    }

    #[test]
    fn tips_are_cut_back_only_where_they_would_dig_into_a_root() {
        // A 12-tooth sun with 18-tooth planets: the planets' tips reach past the sun's base
        // circle, and so does the inside of a 48-tooth ring's teeth past the planets'.
        let p = plan(&stage(12, 18, 3)).unwrap();
        let e = 15.0 * 20f64.to_radians().sin();
        assert!((p.planet.tip_radius - p.planet.base_radius.hypot(e)).abs() < 1e-12 && p.planet.tip_cut());
        assert!((p.ring.tip_radius - p.ring.base_radius.hypot(e)).abs() < 1e-12 && p.ring.tip_cut());
        assert!(!p.sun.tip_cut() && (p.sun.tip_radius - 7.0).abs() < 1e-12);
        assert_eq!(p.notes.len(), 2, "{:?}", p.notes);
        // Big gears clear each other: 30 and 30 teeth in a 90-tooth ring need no cut at all.
        let q = plan(&stage(30, 30, 3)).unwrap();
        assert!(!q.sun.tip_cut() && !q.planet.tip_cut(), "{:?}", q.notes);
    }

    #[test]
    fn contact_ratios_match_the_textbook_formula() {
        // 30 and 30 teeth, m 1, 20 degrees: (2 sqrt(16^2 - (15 cos a)^2) - 30 sin a) / (pi cos a).
        let p = plan(&stage(30, 30, 3)).unwrap();
        let a = 20f64.to_radians();
        let ext = (2.0 * (16f64.powi(2) - (15.0 * a.cos()).powi(2)).sqrt() - 30.0 * a.sin()) / (PI * a.cos());
        assert!((p.contact_ratio_sun_planet - ext).abs() < 1e-12);
        assert!(p.contact_ratio_sun_planet > 1.6 && p.contact_ratio_planet_ring > p.contact_ratio_sun_planet);
        assert!(p.warnings.is_empty(), "{:?}", p.warnings);
    }

    #[test]
    fn a_stage_that_cannot_be_built_is_refused_with_what_would_fit() {
        let wrong_ring = Stage { ring: 47, ..stage(12, 18, 3) };
        assert!(plan(&wrong_ring).unwrap_err().contains("48 teeth"));
        // 12 + 52 = 64 is not a multiple of 3; 2 and 4 planets of 20 teeth fit, 8 would overlap.
        let e = plan(&stage(12, 20, 3)).unwrap_err();
        assert!(e.contains("not a multiple of 3") && e.contains("2, 4 planets fit"), "{e}");
        // Six 18-tooth planets around a 12-tooth sun overlap; four fit.
        let e = plan(&stage(12, 18, 6)).unwrap_err();
        assert!(e.contains("overlap") && e.contains("At most 4"), "{e}");
    }

    #[test]
    fn hostile_stages_are_refused_not_panics() {
        let ok = stage(12, 18, 3);
        for bad in [
            Stage { module: 0.0, ..ok },
            Stage { module: -1.0, ..ok },
            Stage { module: f64::NAN, ..ok },
            Stage { module: f64::INFINITY, ..ok },
            Stage { module: 1e300, ..ok },
            Stage { pressure_angle: 0.0, ..ok },
            Stage { pressure_angle: f64::NAN, ..ok },
            Stage { pressure_angle: 1.0, ..ok },
            Stage { sun: 0, ring: 36, ..ok },
            Stage { sun: 5, ring: 41, ..ok },
            Stage { sun: u32::MAX, ..ok },
            Stage { planet: 2000, ..ok },
            Stage { sun: 400, planet: 400, ring: 1200, ..ok },
            Stage { planets: 0, ..ok },
            Stage { planets: 1, ..ok },
            Stage { planets: u32::MAX, ..ok },
            Stage { backlash: -0.1, ..ok },
            Stage { backlash: f64::NAN, ..ok },
            Stage { backlash: 2.0, ..ok },
        ] {
            assert!(plan(&bad).is_err(), "{bad:?}");
        }
        // Teeth too small for their pressure angle come to a point.
        assert!(plan(&Stage { pressure_angle: 35f64.to_radians(), ..stage(6, 6, 3) }).is_err());
    }

    #[test]
    fn the_outline_has_its_lobes_where_the_gear_says() {
        let p = plan(&stage(12, 18, 3)).unwrap();
        for g in [&p.sun, &p.planet, &p.ring] {
            let pts = outline(g, 8).unwrap();
            // A flank below the base circle starts with one more point, along the radius.
            let per = 8 + usize::from(g.lobe_span().0 < g.base_radius);
            assert_eq!(pts.len(), g.teeth as usize * 2 * per);
            let (lo, hi) = (g.tip_radius.min(g.root_radius) - 1e-9, g.tip_radius.max(g.root_radius) + 1e-9);
            assert!(pts.iter().all(|q| (lo..=hi).contains(&q[0].hypot(q[1]))));
            // Each flank runs from the inner circle to the outer one.
            let (inner, outer) = g.lobe_span();
            assert!((pts[0][0].hypot(pts[0][1]) - inner).abs() < 1e-9 && (pts[per - 1][0].hypot(pts[per - 1][1]) - outer).abs() < 1e-9);
            // The first lobe straddles +x: its two flanks are mirror images.
            for i in 0..per {
                let (a, b) = (pts[i], pts[2 * per - 1 - i]);
                assert!((a[0] - b[0]).abs() < 1e-9 && (a[1] + b[1]).abs() < 1e-9);
            }
            // A point just off the middle of the first lobe's outer end is outside the outline's
            // lobe for an external gear (beyond the tip) and in the ring's material.
            let out = g.lobe_span().1 + 0.01;
            assert_eq!(g.contains([out, 0.0]), g.internal);
        }
    }

    #[test]
    fn two_flank_points_still_reach_from_the_root_to_the_tip() {
        // The sun's root circle is inside its base circle: the radial point comes on top.
        let p = plan(&stage(12, 18, 3)).unwrap();
        let pts = outline(&p.sun, 2).unwrap();
        assert_eq!(pts.len(), 12 * 2 * 3);
        let r: Vec<f64> = pts[..3].iter().map(|q| q[0].hypot(q[1])).collect();
        assert!(
            (r[0] - p.sun.root_radius).abs() < 1e-9 && (r[1] - p.sun.base_radius).abs() < 1e-9 && (r[2] - p.sun.tip_radius).abs() < 1e-9,
            "{r:?}"
        );
        assert!(outline(&p.sun, 1).is_err() && outline(&p.sun, 65).is_err());
    }

    #[test]
    fn every_planet_meshes_with_the_sun_and_the_ring_through_a_whole_tooth() {
        // With backlash, the teeth never touch: each flank keeps half the play, along the line
        // of action (cos a of it), on both sides. 3 and 4 planets, odd and even planet counts.
        for (sun, planet, n) in [(12u32, 18u32, 3u32), (15, 15, 3), (20, 16, 4), (13, 13, 4)] {
            let st = Stage { backlash: 0.2, ..stage(sun, planet, n) };
            let p = plan(&st).unwrap();
            let want = 0.1 * st.pressure_angle.cos();
            for k in 0..n as usize {
                let (sp, pr) = sweep(&p, k, 12);
                assert!((sp - want).abs() < 0.01 && (pr - want).abs() < 0.01, "{sun}/{planet}/{n} planet {k}: {sp} {pr}, want {want}");
            }
        }
    }

    #[test]
    fn without_backlash_the_teeth_touch_and_never_overlap() {
        let p = plan(&stage(12, 18, 3)).unwrap();
        let (sp, pr) = sweep(&p, 1, 24);
        assert!(sp > -2e-3 && sp < 2e-3 && pr > -2e-3 && pr < 2e-3, "{sp} {pr}");
    }

    #[test]
    fn the_sweep_finds_a_planet_turned_a_quarter_tooth_wrong() {
        // The check above has teeth: a planet turned off its phase runs into the sun.
        let mut p = plan(&Stage { backlash: 0.2, ..stage(12, 18, 3) }).unwrap();
        p.planet_turns[1] += TAU / 18.0 / 4.0;
        let (sp, pr) = sweep(&p, 1, 24);
        assert!(sp < -0.1 || pr < -0.1, "{sp} {pr}");
    }

    #[test]
    fn uncut_tips_would_dig_in() {
        // Put the standard tips back. 27-tooth planets dig about 0.018 mm into a 9-tooth sun's
        // roots; a 36-tooth ring's teeth about 0.040 mm into 12-tooth planets'.
        let uncut = |sun: u32, planet: u32| {
            let mut p = plan(&stage(sun, planet, 3)).unwrap();
            for g in [&mut p.sun, &mut p.planet, &mut p.ring] {
                g.tip_radius = g.standard_tip_radius;
            }
            sweep(&p, 0, 96)
        };
        let (sp, _) = uncut(9, 27);
        assert!(sp < -0.01, "{sp}");
        let (_, pr) = uncut(12, 12);
        assert!(pr < -0.02, "{pr}");
    }
}
