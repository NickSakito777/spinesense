# Where the dissertation's code references live in this repository

The dissertation appendix cites code by path and, in places, by SHA-256. Those paths refer to the working repository, which is not what was published: the release drops the study's private material and reorganises what remains. This table maps each cited location to where it is here.

If a reference is missing from this table, it points at something the release does not contain — see [Not published](#not-published) below.

## Direct matches

| Cited in the appendix | In this repository |
|---|---|
| `tools/twist_bench/coverage_gate.py` | same path |
| `tools/twist_bench/locked_track_a/core.py` | same path |
| `tools/twist_bench/ablation_build.py` | same path |
| `tools/twist_bench/signed_diagnostic.py` | same path |
| `tools/twist_bench/corrected_validation_rebuild.py` | same path |
| `firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino` | same path |
| `firmware/imu_i3c_xyz_sflp/imu_i3c_xyz_sflp.ino` | same path |
| `firmware/*/build_opt.h` | same path |
| `tools/twist_bench/test_sflp_plumbing.py` | same path |
| `audit_acquisition.py` | `tools/twist_bench/paper_supplements/audit_acquisition.py` |
| `make_fig8_2_agreement_summary.py` | `tools/twist_bench/paper_supplements/make_fig8_2_agreement_summary.py` |

Three of these were edited during release preparation — imports rewired, hard-coded cohort lists moved into configuration — so their SHA-256 differs from the appendix. Current values are in [Hashes](#hashes).

## Moved

The per-session processing scripts were merged into one module, because the reusable half of each was identical and only the session constants differed. Everything the appendix cites by function name is present, under a different file:

| Cited as | Now in | Functions |
|---|---|---|
| `process_t06_recipe.py` | `tools/twist_bench/session_recipe.py` | `tolerant_load_five_streams`, `local_twist`, `local_swing`, `pre_mask`, `qrel`, `movement_bouts`, `score_block`, `win` |
| `process_t09_recipe.py` | `tools/twist_bench/session_recipe.py` | `peak_bouts`, `choose_bend`, `mark_bend_gain_corrected`, `signals_for`, `jsonable` |
| `ml_classify/build_dataset.py` (dataset layer) | `tools/twist_bench/dataset_adapter.py` | `subject_blocks`, `imu_path`, `mocap_path`, `manifest_path`, `quality_for`, `LABELS`, `SIG_BY_LABEL`, `SIGN_BY_LABEL` |
| `ml_classify/build_dataset.py` (geometry) | `tools/twist_bench/session_recipe.py` | `swing_rotvec_deg`, `tared_bend_quat` |

Two behavioural notes, since a reader comparing against the appendix will notice them:

- `local_twist` and `local_swing` now take the chain role names as arguments instead of hard-coding `sacrum`/`sternum`. Defaults reproduce the cited behaviour.
- `dataset_adapter` reads the session list, per-session block corrections, and exclusion records from a config file rather than module-level constants. `docs/dataset_config.example.json` documents the schema. The study's own config is not published.

## Not published

| Cited | Why not |
|---|---|
| `ml_classify/build_dataset.py` (feature construction: `bout_features`) | Tightly coupled to the participant-level dataset. Its per-feature mirror `features_from_scalars` in `ablation_build.py` **is** published and is what the appendix compares against. |
| `locked_track_a/runner.py`, `report.py` | The study's nested-CV experiment pipeline. See the README section "Classification". |
| `runs/ch9_supplement_2026-08-01/ch9_supplement.py`, `runs/ch10_supplement_2026-08-02/ch10_supplement.py` | Chapter-specific supplementary analyses that read participant-level result tables. |
| `config/placement_maps_v1.json` | The trial-to-participant mapping. Publishing it would undo the de-identification. |
| Any `.csv` / `.json` under `runs/` or `data_clean/`, any `.log` under `data/` | Participant recordings and everything derived from them. See the README section "Participant data". |
| `probe_feature_invariance.py`, `probe_feature_curves.py` | Appendix C.3's sensitivity probes. They read the cohort list and per-session block overrides as module-level constants, which this release moved into configuration; porting them requires rewriting that data-loading path and re-checking the reported values. |
| `make_feature_motivation.py`, other `make_fig*.py` | Figure scripts bound to the dissertation's own typesetting style module and cached plot data. `make_fig8_2_agreement_summary.py` has no such dependency and is published. |

## Hashes

SHA-256 of the files this repository publishes, at the release tag. Cited files whose hash is unchanged from the appendix are marked accordingly.

| File | SHA-256 | vs appendix |
|---|---|---|
| `firmware/imu_i3c_xyz_sflp/imu_i3c_xyz_sflp.ino` | `ac32e418aa5fcdf4f2a4677a0f0f636a410c6833b5ade5694618c0435227604e` | **unchanged** |
| `firmware/imu_i3c_xyz_sflp/build_opt.h` | `b8ac71182d9d4cf65da10b76bc307a966085bfa372ce0afe6a2d1c07d2bc93cf` | **unchanged** |
| `firmware/imu_i3c_connectivity_5/imu_i3c_connectivity_5.ino` | `5bcbab27f5a3a5c782433082f553623783eaedee79db61e90add7022c584db14` | **unchanged** |
| `tools/twist_bench/coverage_gate.py` | `b2a535ee3b2c43edb69f610361337bfabda6f97471c07e740afc8b8373b5b1d3` | **unchanged** |
| `tools/twist_bench/locked_track_a/core.py` | `5ac472961c4b4aad5a85e7bbb1484284087c055b42aac06176b1b8c47285757e` | **unchanged** |
| `tools/twist_bench/paper_supplements/audit_acquisition.py` | `a30ee682fff7064f91982adb97e8565234905cf3b67e28ba1cc279f78bff6d52` | newly published |
| `tools/twist_bench/paper_supplements/make_fig8_2_agreement_summary.py` | `02b4b5d7a38e0f18e77c6cf880cf7ff5e61b57739336f4a6443c364eed0062d6` | **unchanged** |
| `tools/twist_bench/ablation_build.py` | `990f373e9637e837308eaa9b798a80397aa2d09790d5eddc0bca034b46790b8c` | changed — appendix value is stale |
| `tools/twist_bench/signed_diagnostic.py` | `a61034166a26b8a21e740274c25eb2405307bdf15521c878f00cba3924b4f1ff` | changed — appendix value is stale |
| `tools/twist_bench/corrected_validation_rebuild.py` | `cf12d880eea1ae085ac61ac02421523e82234277cc2bb43da3ac51bbd48a3840` | changed — appendix value is stale |
| `tools/twist_bench/session_recipe.py` | `30a7e5edd4fb8bef4cfa63b396cf677be23ca5e44ea38a70cb253e671a0a01c4` | not cited (new in this release) |
| `tools/twist_bench/dataset_adapter.py` | `3cc767f55870efb14ba0b6893149eb7fcb6fc0af58e3c94acf2675d70903a456` | not cited (new in this release) |
| `tools/twist_bench/paper_supplements/ch9_supplement.py` | `a31e8a15cd5e49a3944bf2dd84ab0287a20ac383994f31f8657dc610d5730a06` | not cited (new in this release) |
| `tools/twist_bench/paper_supplements/ch10_supplement.py` | `ba63f41ac7aa584620e9eb680b9d28e508f50c88f4b67407e606e275149a0315` | not cited (new in this release) |

The three changed files were edited for release: imports rewired to `session_recipe` / `dataset_adapter`, and hard-coded cohort lists moved into configuration. The logic the appendix describes is unchanged.

The firmware `.bin` and `.hex` whose hashes the appendix also lists are build outputs and are not committed. Rebuild from the source above with the toolchain recorded in appendix A.12 — the archive that produced those binaries verified byte-identical rebuilds.
