//! Contract and reply type for `gear.planetary` in `cmd/planetary.rs`: a planetary gear stage.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::joints::{JointKey, OccurrenceKey};

/// Build a planetary gear stage in one step: a sun, evenly spaced planets on a carrier, and a
/// ring around them, each a component, about the world Z axis through `origin`. The ring is held
/// (grounded, or fixed to `housing`). The sun turns on a revolute joint, or is fixed to the part
/// that `input` turns; the carrier turns on a revolute joint, and each planet on one on the
/// carrier, as bearings would hold them. Motion links turn them at the rates their teeth give.
/// Tips that would dig into a partner's root are cut back. One undo step.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct PlanetaryParams {
    /// Module in millimetres, the same for the three gears. Above 0.
    pub module: f64,
    /// Teeth of the sun, 6 to 1000.
    pub sun_teeth: u32,
    /// Teeth of each planet, 6 to 1000.
    pub planet_teeth: u32,
    /// Teeth of the ring: sun + 2 x planet (the default), as gears without profile shift need.
    #[serde(default)]
    pub ring_teeth: Option<u32>,
    /// How many planets, 2 to 32 (default 3). Sun + ring must be a multiple of it.
    #[serde(default)]
    pub planets: Option<u32>,
    /// Pressure angle in degrees, above 0 and at most 35 (default 20).
    #[serde(default)]
    pub pressure_angle: Option<f64>,
    /// Thickness of the gears along +Z in millimetres (default: the module times 10).
    #[serde(default)]
    pub face_width: Option<f64>,
    /// Play between the teeth of each mesh on the pitch circle in millimetres (default 0),
    /// shared by its two gears. Printed gears want some, such as 0.1 to 0.2.
    #[serde(default)]
    pub backlash: Option<f64>,
    /// The ring's wall outside the bottom of its teeth in millimetres (default: the module
    /// times 3).
    #[serde(default)]
    pub rim: Option<f64>,
    /// Diameter of the carrier's pins in millimetres (default: 0.4 times a planet's root
    /// diameter).
    #[serde(default)]
    pub pin_diameter: Option<f64>,
    /// Diameter of the hole through each planet in millimetres (default: the pin's plus 0.2).
    #[serde(default)]
    pub planet_bore: Option<f64>,
    /// Diameter of a hole through the sun in millimetres (default: none).
    #[serde(default)]
    pub sun_bore: Option<f64>,
    /// Thickness of the carrier's plate in millimetres (default: half the face width).
    #[serde(default)]
    pub carrier_thickness: Option<f64>,
    /// Gap between the gears and the carrier's plate along +Z in millimetres (default 0.5).
    #[serde(default)]
    pub axial_gap: Option<f64>,
    /// Where the stage's axis meets the bottom of its gears, in world millimetres (default: the
    /// origin). The axis runs along +Z.
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// The stage's name, which starts its parts' and joints' names (default `Planetary`).
    #[serde(default)]
    pub name: Option<String>,
    /// A revolute joint about the stage's axis that turns its sun, such as the carrier joint of
    /// the stage before: the sun is fixed to the part it turns, and the motion links follow it.
    #[serde(default)]
    #[schemars(with = "Option<JointKey>")]
    pub input: Option<Value>,
    /// The occurrence the ring is fixed to. Default: the part `input` turns against, such as the
    /// ring of the stage before; without `input`, the ring is grounded.
    #[serde(default)]
    #[schemars(with = "Option<OccurrenceKey>")]
    pub housing: Option<Value>,
    /// Make the motion links that turn the carrier and the planets with the sun (default true).
    /// Without them each part turns on its own joint.
    #[serde(default)]
    pub links: Option<bool>,
    /// Points along each tooth flank, 2 to 64 (default 8).
    #[serde(default)]
    pub flank_points: Option<u32>,
}

/// A gear's circles, in millimetres.
#[derive(Debug, Serialize, JsonSchema)]
pub struct GearCircles {
    /// Number of teeth.
    pub teeth: u32,
    /// Pitch diameter: the module times the teeth.
    pub pitch_diameter: f64,
    /// Base circle diameter, where the involute flanks start.
    pub base_diameter: f64,
    /// Where the teeth end: outside the sun and planets, inside the ring.
    pub tip_diameter: f64,
    /// The standard tip diameter, when the tip was cut back from it so the teeth clear their
    /// partner's root (see `notes`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_cut_from: Option<f64>,
    /// The bottom of the spaces: inside the sun and planets, outside the ring.
    pub root_diameter: f64,
}

/// A part of the stage.
#[derive(Debug, Serialize, JsonSchema)]
pub struct StagePart {
    /// The component's id.
    pub component: u64,
    /// Its occurrence's id.
    pub occurrence: u64,
    /// The component's name.
    pub name: String,
    /// The joint it turns on, or that holds it to `input`'s part or the housing. Left out for a
    /// grounded ring.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub joint: Option<u64>,
    /// Its gear. Left out for the carrier.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gear: Option<GearCircles>,
}

/// The planets: one component, placed once for each planet.
#[derive(Debug, Serialize, JsonSchema)]
pub struct StagePlanets {
    /// The component's id.
    pub component: u64,
    /// The component's name.
    pub name: String,
    /// Each planet's occurrence, in the order they go round from +x.
    pub occurrences: Vec<u64>,
    /// Each planet's revolute joint on the carrier, in the same order.
    pub joints: Vec<u64>,
    /// Their gear.
    pub gear: GearCircles,
}

/// Checks worked out from the teeth.
#[derive(Debug, Serialize, JsonSchema)]
pub struct StageChecks {
    /// Between the tip circles of two neighbouring planets, mm. Always above 0: planets that
    /// would overlap are refused.
    pub neighbour_gap: f64,
    /// Teeth in contact on average in the sun-planet mesh (its contact ratio). Below 1.2 a mesh
    /// runs rough; below 1 it jumps, and `warnings` say so.
    pub contact_ratio_sun_planet: f64,
    /// Teeth in contact on average in the planet-ring mesh.
    pub contact_ratio_planet_ring: f64,
}

/// What `gear.planetary` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct PlanetaryStage {
    /// Sun turns per carrier turn with the ring held: 1 + ring teeth / sun teeth.
    pub ratio: f64,
    /// The sun.
    pub sun: StagePart,
    /// The planets.
    pub planets: StagePlanets,
    /// The ring.
    pub ring: StagePart,
    /// The carrier: a plate above the gears with a pin through each planet.
    pub carrier: StagePart,
    /// From the stage's axis to each planet's axis, mm.
    pub centre_distance: f64,
    /// From `origin` along +Z to the top of the carrier's plate, mm: where a next stage can start.
    pub height: f64,
    /// The motion links made, by their index in `joint.list` `links`: the carrier's, then each
    /// planet's. Empty with `links: false`.
    pub links: Vec<usize>,
    /// Checks of the stage worked out from its teeth.
    pub checks: StageChecks,
    /// Tips cut back so they clear their partner's root, and why.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
    /// What to look at: a mesh that runs rough or jumps.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}
