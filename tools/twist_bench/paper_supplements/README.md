# Paper supplements

Scripts the dissertation cites as the implementation of record for specific appendix sections. They are here because a claim that an analysis has an executable implementation is worth little if the implementation is not there to read.

| Script | Cited by | What it does | Runs from a clean clone? |
|---|---|---|---|
| `audit_acquisition.py` | A.16.1 | Three-level acquisition audit of a raw log: continuous segments, software frames, record slots. Reports observed frame rate, slot completeness, `READ_FAIL` and non-finite quaternion counts. | **Yes** — takes the log path as an argument, so it runs on any log in the documented format, including the samples in `docs/` |
| `make_fig8_2_agreement_summary.py` | Figure 8.2 | Renders the agreement summary figure | No — reads a result table under `runs/` |
| `ch9_supplement.py` | E.7.1 | Cross-subject conversion, per-block correlation structure, swing/rotation cross-talk | No — reads participant-level result tables |
| `ch10_supplement.py` | E.7.6 | Misclassification structure, feature contribution, cross-branch contrast. Includes assertions checking each recomputed number against the value quoted in the dissertation | No — same |

The three that cannot run need result tables from `runs/`, which is not published: it holds participant-level output. You can still read exactly what each analysis computes, which is what the appendix cites them for.

`audit_acquisition.py` is the exception worth trying. It needs the 14-field format (raw + SFLP quaternion), so point it at the SFLP sample rather than the raw-only one:

```bash
python audit_acquisition.py ../../../docs/sample_sflp_frames.log
```

```
frames (analysis set)              1,200
duration (s)                       10.0
observed frame rate (Hz)           119.996
mean inter-frame interval (ms)     8.3336
record-slot completeness           6,000/6,000 = 100.0%
five-interface complete frames     1,200 = 100.0%
READ_FAIL records                  0
```

A 10-field raw-only log parses to zero frames and the script reports "too few frames to audit".

In the working repository these lived under `runs/<run-id>/` or in the dissertation's own LaTeX repository. They were moved here for publication so the `runs/` tree could be excluded wholesale — a run directory holds participant-level output, and a rule that admits scripts from inside it has proved too easy to get wrong.

Path constants at the top of each file were rewritten for this location. Nothing else changed.
