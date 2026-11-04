"""Table comparison for the golden tests: same columns and rows; floats within a tolerance, everything else exact."""
import numpy as np
import pandas as pd

TOL = 1e-9


def assert_frames_equal(got: pd.DataFrame, exp: pd.DataFrame, tol: float = TOL, what: str = ""):
    assert list(got.columns) == list(exp.columns), f"{what}: columns {list(got.columns)} != {list(exp.columns)}"
    assert len(got) == len(exp), f"{what}: {len(got)} rows != {len(exp)}"
    for c in exp.columns:
        a, b = got[c], exp[c]
        if pd.api.types.is_float_dtype(b) or pd.api.types.is_float_dtype(a):
            assert pd.api.types.is_numeric_dtype(a) and pd.api.types.is_numeric_dtype(b), f"{what}.{c}: dtypes"
            x, y = a.to_numpy(float), b.to_numpy(float)
            nan = np.isnan(y)
            assert np.array_equal(np.isnan(x), nan), f"{what}.{c}: NaN positions differ"
            inf = np.isinf(y)
            assert np.array_equal(x[inf], y[inf]) and not np.isinf(x[~inf]).any(), f"{what}.{c}: inf differs"
            fin = ~nan & ~inf
            err = np.abs(x[fin] - y[fin])
            assert err.size == 0 or err.max() <= tol, f"{what}.{c}: max |diff| {err.max():.3g} > {tol}"
        else:
            assert a.dtype == b.dtype, f"{what}.{c}: dtype {a.dtype} != {b.dtype}"
            x, y = a.to_numpy(object), b.to_numpy(object)
            bad = [i for i in range(len(y)) if not (x[i] == y[i] or (pd.isna(x[i]) and pd.isna(y[i])))]
            assert not bad, f"{what}.{c}: {len(bad)} rows differ, first {bad[0]}: {x[bad[0]]!r} != {y[bad[0]]!r}"
