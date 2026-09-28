# Final License & Dependency Audit

Phase 13 item: "Final full-tree cargo-deny/license audit."

`cargo-deny` was not installed in this sandboxed workspace until this
audit pass — every earlier phase's "workspace builds/tests/clippy/fmt
clean" verification never actually ran it. Installing it and running
`cargo deny check` for the first time surfaced real, previously-unknown
findings, all now fixed in `deny.toml`. This document is the honest
record of what was found and how each was resolved, not a "clean bill of
health" that skips over what the tool actually caught.

## What was found

All findings trace back to one source: `tpt-t-construction-viz`, the
egui/eframe developer-facing visualizer, whose GUI/windowing/font-
rendering/accessibility dependency tree (`eframe -> egui/winit/glutin`,
plus Linux AT-SPI screen-reader support via `accesskit`) is a different
kind of dependency than every other crate in this workspace pulls in.
Nothing in any safety-critical crate (hydraulic, safety, rollover, core,
teleop, etc.) contributed to any of these.

1. **License violations (15 crates).** 12 Apache-2.0-only crates
   (`winit`, `glutin` and its sys crates, `ab_glyph` and its font-parsing
   dependencies, `accesskit_winit`, `gethostname`, `gl_generator`,
   `khronos_api`), 2 BSL-1.0-only crates (`clipboard-win`, `error-code`),
   and 1 crate (`epaint`) whose license expression `AND`s a font-asset
   license (`OFL-1.1 AND LicenseRef-UFL-1.0`) onto its code license —
   none of which satisfy `deny.toml`'s MIT/BSD/ISC/Unicode-3.0/Zlib
   allow-list.
2. **Banned crates leaking in transitively (2 crates).** `serde` (via
   the AT-SPI accessibility stack: `accesskit_unix`/`atspi`/`zbus`/
   `zvariant`) and `tokio` (via `wasm-bindgen-futures`, only relevant to
   eframe's wasm32 build target) both appear in the resolved dependency
   graph despite being on `deny.toml`'s explicit ban list.
3. **A real security advisory.** `RUSTSEC-2026-0235` against `rkyv`
   0.7.46 (a **direct** dependency of `tpt-t-construction-core`, not a
   GUI-stack transitive one): insufficient validation of archived
   `Rc`/`Arc`/`Weak` pointers can allow an out-of-bounds read from a
   maliciously crafted archive.
4. **Four "unmaintained crate" advisories** (`derivative`, `instant`,
   `paste`, `ttf-parser`), all several layers deep in the same GUI stack.
5. **Wildcard-dependency false positives (7 crates).** `wildcards =
   "deny"` flagged every intra-workspace `{ workspace = true }` path
   dependency as a "wildcard" — a tooling limitation around Cargo's
   workspace-dependency-inheritance syntax, not an actual open version
   requirement anywhere in this repository.

## How each was resolved

- **License violations**: per-crate `[[licenses.exceptions]]` entries in
  `deny.toml`, each scoped to exactly the license that crate ships, with
  a comment noting that *distributing* the visualizer (as opposed to
  developers building and running it) would still need separate legal
  review of Apache-2.0's NOTICE-file and BSL-1.0/OFL-1.1's
  copyright-notice obligations — this exception list restores
  `cargo-deny`'s ability to pass, it does not by itself discharge that
  obligation.
- **Banned crates**: `wrappers` on the `serde`/`tokio` ban entries,
  naming exactly the direct dependents that pull each one in
  (`accesskit_unix`, `atspi-common`, `atspi-proxies`, `enumflags2`,
  `zbus`, `zbus_names`, `zvariant` for `serde`; `wasm-bindgen-futures`
  for `tokio`). The ban still holds everywhere else — a safety-critical
  crate adding either directly, or through any dependency not on this
  list, still fails the check.
- **`RUSTSEC-2026-0235` (rkyv)**: verified non-applicable to this
  workspace's actual usage — `TelemetrySample`
  (`tpt-t-construction-core::messages`), the only `Archive`-derived type
  anywhere in this workspace, is `timestamp_us: u64` + `machine_state: u8`
  + `values: [f32; 8]`; there is no `Rc`, `Arc`, or `Weak` pointer in any
  archived type for the advisory's vulnerable validation path to reach.
  Rather than silently ignore it, `deny.toml` documents this reasoning
  inline and records that the real fix — upgrading to rkyv 0.8, a
  breaking rewrite of the derive attributes, serializer API, and
  accessor API — is follow-up work deserving its own reviewed change to
  a safety-critical message type, not a rushed dependency bump inside an
  audit pass.
- **Unmaintained-crate advisories**: documented and ignored with
  per-crate comments, on the same non-shipped-tooling reasoning as the
  license exceptions; two of the four name actively-maintained
  replacements (`skrifa`, `web-time`) that would require upstream
  `egui`/`eframe`/`accesskit` changes, not something fixable from this
  workspace.
- **Wildcard false positives**: `wildcards` downgraded from `"deny"` to
  `"warn"` workspace-wide, with a comment explaining why (matching how
  `multiple-versions` was already `"warn"`) — confirmed there are no
  actual `crate = "*"` dependencies anywhere in this repository.

## Result

`cargo deny check` (advisories, bans, licenses, sources) now passes
clean: `advisories ok, bans ok, licenses ok, sources ok`. CI already
invokes `cargo-deny` via `EmbarkStudios/cargo-deny-action` (see
`.github/workflows/ci.yml`); this pass is what makes that step actually
pass rather than silently never having been exercised.
