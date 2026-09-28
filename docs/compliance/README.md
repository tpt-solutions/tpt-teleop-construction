# Compliance Mapping Documents

Phase 11 item: "ISO 19014 (earth-moving machinery safety) compliance
review", "MSHA regulation compliance review", "IEC 61508 SIL 2/3
functional safety certification documentation."

These three documents map this workspace's current implementation
against each standard's requirement areas. They are **gap-analysis
documents, not certification claims** — none of these standards can be
satisfied by source code review alone, and none of them are satisfied
here. Every standard in this set requires, at minimum, some combination
of: a named accredited certification body, documented hazard/risk
assessments signed off by qualified assessors, physical hardware to test
against, and (for MSHA specifically) formal regulatory approval — none
of which a sandboxed software workspace can produce honestly. What these
documents *can* honestly say is: which requirement areas already have a
concrete software implementation to point to, which are structurally
blocked on Phase 12 hardware bring-up, and which are blocked on
engagement with an external body regardless of what this repository
ever contains.

Each document uses the same three status labels:

- **Implemented (software)** — a concrete module/function exists that
  addresses this requirement area; cited by file path.
- **Blocked on hardware** — the requirement can only be verified against
  physical equipment (a real machine, real sensors, an environmental
  chamber), i.e. Phase 12 in `todo.md`.
- **Blocked on external process** — the requirement is inherently a
  third-party or regulatory activity (an accredited assessor, a
  government agency) that no amount of code changes this repository can
  satisfy on its own.

- [ISO 19014](./iso_19014.md)
- [MSHA](./msha.md)
- [IEC 61508](./iec_61508.md)
