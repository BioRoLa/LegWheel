from __future__ import annotations

import ctypes
from pathlib import Path

import numpy as np

_LIB = None
_LOAD_ERROR = None


def _load_library():
    global _LIB, _LOAD_ERROR
    if _LIB is not None:
        return _LIB
    if _LOAD_ERROR is not None:
        return None

    module_dir = Path(__file__).resolve().parent
    candidates = [module_dir / "libdls_solver.so", module_dir / "dls_solver.so"]

    for path in candidates:
        if not path.exists():
            continue
        try:
            lib = ctypes.CDLL(str(path))
            lib.dls_solve_3x3.argtypes = [
                ctypes.POINTER(ctypes.c_double),
                ctypes.POINTER(ctypes.c_double),
                ctypes.c_double,
                ctypes.POINTER(ctypes.c_double),
            ]
            lib.dls_solve_3x3.restype = ctypes.c_int
            _LIB = lib
            return _LIB
        except OSError as exc:
            _LOAD_ERROR = exc
            return None

    _LOAD_ERROR = FileNotFoundError("C backend library not found")
    return None


def is_c_backend_available() -> bool:
    return _load_library() is not None


def dls_solve_3x3_c(J: np.ndarray, v: np.ndarray, damping: float) -> np.ndarray:
    lib = _load_library()
    if lib is None:
        raise RuntimeError(f"C backend unavailable: {_LOAD_ERROR}")

    J = np.ascontiguousarray(np.asarray(J, dtype=np.float64))
    v = np.ascontiguousarray(np.asarray(v, dtype=np.float64).reshape(3))

    if J.shape != (3, 3):
        raise ValueError(f"Expected J shape (3, 3), got {J.shape}")

    out = np.zeros(3, dtype=np.float64)
    rc = lib.dls_solve_3x3(
        J.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        v.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        float(damping),
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    if rc != 0:
        raise RuntimeError("C DLS solver failed (likely singular matrix)")
    return out
