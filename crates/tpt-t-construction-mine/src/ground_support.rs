// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Bolt carrier / shotcrete rig coordination (spec.txt §6): a mining
//! machine sharing a tunnel with active ground-support equipment must
//! keep clear of its working radius while it's actually bolting or
//! spraying — an idle rig imposes no such restriction.

/// Whether a ground-support rig is actively working (and so needs
/// clearance) or idle (safe to approach or pass).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RigStatus {
    Idle,
    Working,
}

/// A ground-support rig's position, status, and the radius around it
/// that must stay clear while it's working.
#[derive(Debug, Clone, Copy)]
pub struct GroundSupportRig {
    pub position: [f32; 2],
    pub status: RigStatus,
    pub work_radius_m: f32,
}

/// Whether it's safe for a machine at `machine_position` to proceed,
/// given `rig`'s current state and an additional `safety_margin_m`
/// beyond its bare working radius.
pub fn is_safe_to_proceed(
    machine_position: [f32; 2],
    rig: &GroundSupportRig,
    safety_margin_m: f32,
) -> bool {
    if rig.status == RigStatus::Idle {
        return true;
    }
    let dx = machine_position[0] - rig.position[0];
    let dy = machine_position[1] - rig.position[1];
    let distance_m = (dx * dx + dy * dy).sqrt();
    distance_m >= rig.work_radius_m + safety_margin_m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn working_rig() -> GroundSupportRig {
        GroundSupportRig {
            position: [0.0, 0.0],
            status: RigStatus::Working,
            work_radius_m: 5.0,
        }
    }

    #[test]
    fn idle_rig_never_blocks_passage() {
        let rig = GroundSupportRig {
            status: RigStatus::Idle,
            ..working_rig()
        };
        assert!(is_safe_to_proceed([0.0, 0.0], &rig, 2.0));
    }

    #[test]
    fn working_rig_blocks_passage_within_its_radius_plus_margin() {
        let rig = working_rig();
        assert!(!is_safe_to_proceed([3.0, 0.0], &rig, 2.0));
        assert!(!is_safe_to_proceed([6.0, 0.0], &rig, 2.0)); // within radius+margin
    }

    #[test]
    fn working_rig_allows_passage_beyond_radius_plus_margin() {
        let rig = working_rig();
        assert!(is_safe_to_proceed([8.0, 0.0], &rig, 2.0));
    }
}
