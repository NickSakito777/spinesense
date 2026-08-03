from __future__ import annotations

"""Explicit opt-in gate for historical MoCap-selected staging recipes."""

import os


def require_legacy_staging_opt_in(script_name: str) -> None:
    if os.environ.get("SPINESENSE_ALLOW_LEGACY_STAGING_RECIPE") != "1":
        raise RuntimeError(
            f"{script_name} is a historical staging recipe whose global/local readout "
            "was selected on the same LORO results. It cannot produce canonical evidence. "
            "Use corrected_validation_rebuild.py, or set "
            "SPINESENSE_ALLOW_LEGACY_STAGING_RECIPE=1 for an explicitly historical rerun."
        )
