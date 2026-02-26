"""Generic PID controller with output clamping and integral anti-windup."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

from followme.config import PIDConfig


@dataclass
class PIDController:
    """Proportional-Integral-Derivative controller.

    Computes control output as:
        u(t) = Kp * e(t) + Ki * integral(e) + Kd * de/dt

    Supports output clamping and integral windup prevention.

    Args:
        config: PID gain parameters (kp, ki, kd).
        output_limits: Min/max bounds for the controller output.
        integral_limit: Max absolute value of the integral term to prevent windup.
    """

    config: PIDConfig
    output_limits: Tuple[float, float] = (-100.0, 100.0)
    integral_limit: Optional[float] = None

    # Internal state (mutable)
    _integral: float = field(default=0.0, init=False, repr=False)
    _prev_error: Optional[float] = field(default=None, init=False, repr=False)
    _last_time: Optional[float] = field(default=None, init=False, repr=False)

    def update(self, error: float, dt: Optional[float] = None) -> float:
        """Compute the PID output for the given error.

        Args:
            error: Current error (setpoint - measurement).
            dt: Time delta in seconds. If None, computed from wall clock.

        Returns:
            Clamped control output.
        """
        now = time.monotonic()

        if dt is None:
            dt = (now - self._last_time) if self._last_time is not None else 0.0
        self._last_time = now

        # Proportional
        p_term = self.config.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = max(
                -self.integral_limit,
                min(self.integral_limit, self._integral),
            )
        i_term = self.config.ki * self._integral

        # Derivative
        if self._prev_error is not None and dt > 0:
            d_term = self.config.kd * (error - self._prev_error) / dt
        else:
            d_term = 0.0
        self._prev_error = error

        # Sum and clamp
        output = p_term + i_term + d_term
        lo, hi = self.output_limits
        return max(lo, min(hi, output))

    def reset(self) -> None:
        """Clear all internal state for a fresh start."""
        self._integral = 0.0
        self._prev_error = None
        self._last_time = None
