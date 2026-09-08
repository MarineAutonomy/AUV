# Thruster mix matching MAV-GUI thrusterMixer.ts (keyboard teleop).

NEUTRAL_PWM = 1500
DEFAULT_SCALE_US = 300
DEFAULT_PWM_MIN = 1100
DEFAULT_PWM_MAX = 1900


def clamp01(x: float) -> float:
    return max(-1.0, min(1.0, x))


def mix_to_pwm(
    surge: float,
    sway: float,
    heave: float,
    yaw: float,
    pitch: float,
    roll: float,
    *,
    scale_us: float = DEFAULT_SCALE_US,
    pwm_min: int = DEFAULT_PWM_MIN,
    pwm_max: int = DEFAULT_PWM_MAX,
) -> list[int]:
    """Return [PS-FR, SB-FR, PS-AF, SB-AF, PS-MS1, SB-MS1, PS-MS2, SB-MS2]."""
    t0 = clamp01(surge + sway + yaw)
    t1 = clamp01(surge - sway - yaw)
    t2 = clamp01(surge - sway + yaw)
    t3 = clamp01(surge + sway - yaw)

    v_ps1 = clamp01(heave + roll - pitch)
    v_sb1 = clamp01(heave - roll - pitch)
    v_ps2 = clamp01(heave + roll + pitch)
    v_sb2 = clamp01(heave - roll + pitch)

    def to_pwm(u: float) -> int:
        return int(round(NEUTRAL_PWM + scale_us * u))

    pwm = [
        to_pwm(t0),
        to_pwm(t1),
        to_pwm(t2),
        to_pwm(t3),
        to_pwm(v_ps1),
        to_pwm(v_sb1),
        to_pwm(v_ps2),
        to_pwm(v_sb2),
    ]
    return [max(pwm_min, min(pwm_max, p)) for p in pwm]


def neutral_pwm() -> list[int]:
    return [NEUTRAL_PWM] * 8
