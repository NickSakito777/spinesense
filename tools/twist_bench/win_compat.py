"""Make ``locked_track_a.core`` importable on Windows without altering any computation.

``core.py`` imports the POSIX-only ``resource`` module and uses it in exactly one place,
``peak_rss_bytes()``, which is diagnostic logging only.  On Windows that import fails and the
whole locked module becomes unusable, so this shim registers a minimal stand-in *before*
``core`` is imported.

Nothing numerical changes.  The shim affects only the reported peak-RSS figure, which is
recorded in the runtime ledger and never enters a score, a fold split, or a selection.  On
POSIX the real module is used and this file is a no-op.
"""

from __future__ import annotations

import sys
import types

__all__ = ["IS_WINDOWS", "RESOURCE_IS_SHIMMED", "describe"]

IS_WINDOWS = sys.platform.startswith("win")
RESOURCE_IS_SHIMMED = False


def _install_resource_shim() -> None:
    global RESOURCE_IS_SHIMMED

    if "resource" in sys.modules:
        return
    try:
        import resource  # noqa: F401
        return
    except ImportError:
        pass

    module = types.ModuleType("resource")
    module.RUSAGE_SELF = 0
    module.RUSAGE_CHILDREN = -1

    class _Usage:
        """Only ``ru_maxrss`` is consumed by core.peak_rss_bytes()."""

        __slots__ = ("ru_maxrss",)

        def __init__(self, ru_maxrss: int):
            self.ru_maxrss = ru_maxrss

    def getrusage(_who: int = 0) -> "_Usage":
        try:
            import ctypes
            import ctypes.wintypes as wintypes

            class PROCESS_MEMORY_COUNTERS(ctypes.Structure):
                _fields_ = [
                    ("cb", wintypes.DWORD),
                    ("PageFaultCount", wintypes.DWORD),
                    ("PeakWorkingSetSize", ctypes.c_size_t),
                    ("WorkingSetSize", ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t),
                    ("PagefileUsage", ctypes.c_size_t),
                    ("PeakPagefileUsage", ctypes.c_size_t),
                ]

            counters = PROCESS_MEMORY_COUNTERS()
            counters.cb = ctypes.sizeof(PROCESS_MEMORY_COUNTERS)
            handle = ctypes.windll.kernel32.GetCurrentProcess()
            ok = ctypes.windll.psapi.GetProcessMemoryInfo(
                handle, ctypes.byref(counters), counters.cb
            )
            if ok:
                # core.peak_rss_bytes() multiplies by 1024 off darwin, so hand back kibibytes.
                return _Usage(int(counters.PeakWorkingSetSize // 1024))
        except Exception:  # pragma: no cover - diagnostics must never break a run
            pass
        return _Usage(0)

    module.getrusage = getrusage
    sys.modules["resource"] = module
    RESOURCE_IS_SHIMMED = True


def describe() -> dict[str, object]:
    return {
        "platform": sys.platform,
        "is_windows": IS_WINDOWS,
        "resource_module_shimmed": RESOURCE_IS_SHIMMED,
        "shim_scope": "peak RSS reporting only; no effect on any score, split, or selection",
    }


_install_resource_shim()
