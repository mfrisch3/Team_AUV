import numpy as np

# Precompute
_s2 = np.sqrt(2) / 2
_M_pinv = 0.5 * np.array([
    [ _s2, -_s2],
    [ _s2,  _s2],
    [-_s2, -_s2],
    [-_s2,  _s2]
])  # shape (4,2)

def mix(Fx: float, Fy: float) -> np.ndarray:
    return _M_pinv.dot(np.array([Fx, Fy]))

def forward(thrust: float) -> np.ndarray:
    return mix( thrust, 0.0)

def reverse(thrust: float) -> np.ndarray:
    return mix(-thrust, 0.0)

def turn_right(thrust: float) -> np.ndarray:
    return np.array([ thrust,  thrust,
                     -thrust, -thrust])

def turn_left(thrust: float) -> np.ndarray:
    return np.array([-thrust, -thrust,
                      thrust,  thrust])
