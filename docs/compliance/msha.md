# MSHA Compliance Mapping

See `docs/compliance/README.md` for what this document is and isn't.

The U.S. Mine Safety and Health Administration (MSHA) regulates mining
equipment under 30 CFR (surface metal/nonmetal mining under Part 56,
underground metal/nonmetal under Part 57, underground coal under Part
75, among others). MSHA approval of a specific piece of equipment for
use at a specific mine is a regulatory process involving MSHA itself —
no software repository, however complete, can grant that approval. This
document maps the requirement *categories* MSHA's proximity-detection
and equipment-safety rules are built around against what this workspace
implements, so it's clear what groundwork exists and what is inherently
a regulatory-submission activity.

| Requirement category | Status | Where |
|---|---|---|
| Proximity detection systems (PDS) for continuous mining machines and similar equipment — detect miners in a hazardous zone and warn/stop | Implemented (software) | `tpt-t-construction-safety` end-to-end: `perception.rs` (detection fusion) -> `classification.rs` (distinguishing a person from equipment) -> `tracking.rs` (persistent identity across frames) -> `response.rs` (zone classification and the warn/slowdown/stop decision) -> `tests/zero_false_stop.rs` (Phase 11 validation that benign conditions don't trigger false stops, and that genuine sustained presence still does). |
| PDS reliability/self-monitoring (the system must detect its own faults, not just the miner) | Implemented (software, partial) | `tpt-t-construction-safety::tmr` (Phase 11 TMR voter) gives a decision function for cross-checking redundant channels and flagging disagreement; `tpt-t-construction-core::messages::Fault`/`FaultSeverity` and the new `tpt-t-construction-telemetry::audit` hash-chained log (Phase 11) give a way to record and later prove a fault was raised and handled. Actual redundant sensor *hardware* to feed the voter is a Phase 12 concern. |
| Ground control / roof and rib support coordination in underground work | Implemented (software) | `tpt-t-construction-mine::ground_support` — safe working distances for bolt carrier / shotcrete rig coordination. |
| Ventilation monitoring/routing (underground) | Implemented (software) | `tpt-t-construction-mine::ventilation` — Dijkstra-based ventilation-aware routing. This addresses route planning; it presumes real sensor input for actual air quality/methane monitoring, which MSHA additionally regulates as its own equipment category this workspace doesn't implement. |
| Record-keeping for incident investigation (an auditable trail of machine state, faults, and operator actions) | Implemented (software) | `tpt-t-construction-telemetry` — 100Hz parameter logging (`record.rs`/`logger.rs`), post-shift export for warranty/incident evidence (`export.rs`), and the Phase 11 tamper-evident audit trail (`audit.rs`) specifically added for this kind of certification/incident-review record-keeping. |
| Equipment guarding, machine-specific hazard controls (blast area clearance, haul road design, etc.) | Implemented (software, where machine-control logic applies) | `tpt-t-construction-drill::pattern` (blast hole pattern execution respects a planned pattern rather than ad hoc drilling), `tpt-t-construction-haul::road`/`speed` (edge drop-off avoidance, downgrade/curve/visibility speed limits). Physical guarding (fencing, physical barriers, blast-area personnel clearance procedures) is a site/operations control, not software. |
| Field certification testing at an actual mine site | Blocked on hardware | Requires a real site, real equipment, and observed miners — this is precisely what Phase 12's "Full 12-hour shift field trial" is for. |
| MSHA approval / regulatory submission | Blocked on external process | MSHA approval is granted by MSHA following a formal submission and its own review process, following whatever petition or approval pathway applies to the specific equipment class. No repository state changes this. |

**Overall status:** the software-addressable requirement categories
(detection, response, redundancy logic, record-keeping) have concrete
implementations. The categories that are inherently physical-site or
regulatory-agency activities are recorded as blocked rather than implied
to be handled.
