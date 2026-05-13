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
    candidates = [module_dir / "libcbase_kernels.so", module_dir / "cbase_kernels.so"]

    for path in candidates:
        if not path.exists():
            continue
        try:
            lib = ctypes.CDLL(str(path))

            lib.bezier_point_3d_12.argtypes = [
                ctypes.POINTER(ctypes.c_double),
                ctypes.POINTER(ctypes.c_double),
                ctypes.c_double,
                ctypes.c_double,
                ctypes.c_double,
                ctypes.c_double,
                ctypes.POINTER(ctypes.c_double),
            ]
            lib.bezier_point_3d_12.restype = None

            lib.screw_exp6.argtypes = [
                ctypes.POINTER(ctypes.c_double),
                ctypes.POINTER(ctypes.c_double),
                ctypes.c_double,
                ctypes.POINTER(ctypes.c_double),
            ]
            lib.screw_exp6.restype = None

            _LIB = lib
            return _LIB
        except OSError as exc:
            _LOAD_ERROR = exc
            return None

    _LOAD_ERROR = FileNotFoundError("C kernels library not found")
    return None


def is_kernel_backend_available() -> bool:
    return _load_library() is not None


def bezier_point_3d_12_c(
    control_points: np.ndarray,
    bz_coeff: np.ndarray,
    t: float,
    offset_x: float,
    offset_y: float,
    offset_z: float,
) -> np.ndarray:
    lib = _load_library()
    if lib is None:
        raise RuntimeError(f"C backend unavailable: {_LOAD_ERROR}")

    cp = np.ascontiguousarray(np.asarray(control_points, dtype=np.float64).reshape(12, 3))
    coeff = np.ascontiguousarray(np.asarray(bz_coeff, dtype=np.float64).reshape(12))
    out = np.zeros(3, dtype=np.float64)

    lib.bezier_point_3d_12(
        cp.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        coeff.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        t,
        offset_x,
        offset_y,
        offset_z,
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    return out


def screw_exp6_c(omega: np.ndarray, v: np.ndarray, theta: float) -> np.ndarray:
    lib = _load_library()
    if lib is None:
        raise RuntimeError(f"C backend unavailable: {_LOAD_ERROR}")

    w = np.ascontiguousarray(np.asarray(omega, dtype=np.float64).reshape(3))
    vv = np.ascontiguousarray(np.asarray(v, dtype=np.float64).reshape(3))
    out = np.zeros(16, dtype=np.float64)

    lib.screw_exp6(
        w.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        vv.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        theta,
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )

    return out.reshape(4, 4)
