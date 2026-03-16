#!/usr/bin/env python3
"""
YAVUZ AUV - PID Kontrolcü
Anti-windup, derivative filtresi, gain scheduling destekli profesyonel PID.
"""

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class PIDConfig:
    """PID konfigürasyon parametreleri."""
    kp: float
    ki: float
    kd: float
    output_min: float = -1.0
    output_max: float = 1.0
    integral_min: float = -0.5
    integral_max: float = 0.5
    derivative_filter_tau: float = 0.05  # Türev filtresi zaman sabiti (saniye)
    deadband: float = 0.0                # Deadband (|error| < deadband → output=0)
    setpoint_ramp_rate: float = 0.0      # Setpoint rampa hızı (unit/s), 0=devre dışı


class PIDController:
    """
    Profesyonel PID kontrolcü.
    
    Özellikler:
    - Anti-windup (integral clamping + back-calculation)
    - Türev low-pass filtresi (gürültü azaltma)
    - Setpoint rampa
    - Deadband
    - Reset / pause desteği
    
    Kullanım:
        cfg = PIDConfig(kp=2.0, ki=0.5, kd=0.1, output_min=-1.0, output_max=1.0)
        pid = PIDController(cfg, name="depth")
        
        # Her kontrol döngüsünde:
        output = pid.update(setpoint=target_depth, measurement=current_depth, dt=0.01)
    """

    def __init__(self, config: PIDConfig, name: str = "pid"):
        self.cfg = config
        self.name = name
        self._reset()

    def _reset(self):
        """Dahili durumu sıfırla."""
        self._integral = 0.0
        self._prev_error = 0.0
        self._filtered_derivative = 0.0
        self._prev_measurement = None
        self._current_setpoint = None
        self._last_output = 0.0
        self._is_active = True
        self._total_steps = 0

    def reset(self):
        """Dışarıdan sıfırlama."""
        self._reset()

    def set_gains(self, kp: float, ki: float, kd: float):
        """Çalışma sırasında kazanç güncelleme."""
        self.cfg.kp = kp
        self.cfg.ki = ki
        self.cfg.kd = kd

    def update(self,
               setpoint: float,
               measurement: float,
               dt: float,
               feedforward: float = 0.0) -> float:
        """
        PID çıkışını hesapla.

        Args:
            setpoint:    İstenen değer
            measurement: Ölçülen değer
            dt:          Zaman adımı (saniye)
            feedforward: İleri besleme terimi

        Returns:
            float: Kontrol çıkışı [output_min, output_max]
        """
        if not self._is_active or dt <= 0.0:
            return 0.0

        # Setpoint rampa
        if self.cfg.setpoint_ramp_rate > 0.0 and self._current_setpoint is not None:
            max_change = self.cfg.setpoint_ramp_rate * dt
            delta = setpoint - self._current_setpoint
            delta = max(-max_change, min(max_change, delta))
            self._current_setpoint += delta
        else:
            self._current_setpoint = setpoint

        # Hata hesapla
        error = self._current_setpoint - measurement

        # Deadband
        if abs(error) < self.cfg.deadband:
            error = 0.0

        # ---- Proportional ----
        p_term = self.cfg.kp * error

        # ---- Integral (anti-windup: clamping) ----
        self._integral += error * dt
        self._integral = max(self.cfg.integral_min,
                             min(self.cfg.integral_max, self._integral))
        i_term = self.cfg.ki * self._integral

        # ---- Derivative (low-pass filtreli, measurement üzerinden - setpoint kick yok) ----
        if self._prev_measurement is None:
            raw_derivative = 0.0
        else:
            raw_derivative = -(measurement - self._prev_measurement) / dt

        # Birinci dereceden düşük geçiren filtre
        alpha = dt / (self.cfg.derivative_filter_tau + dt)
        self._filtered_derivative = (alpha * raw_derivative
                                     + (1.0 - alpha) * self._filtered_derivative)
        d_term = self.cfg.kd * self._filtered_derivative

        # ---- Toplam çıkış ----
        output = p_term + i_term + d_term + feedforward

        # ---- Anti-windup: back-calculation ----
        saturated_output = max(self.cfg.output_min, min(self.cfg.output_max, output))
        if output != saturated_output and self.cfg.ki > 0:
            # Integral'ı geri hesapla
            back_calc = (saturated_output - output) / self.cfg.ki
            self._integral += back_calc * dt
            self._integral = max(self.cfg.integral_min,
                                 min(self.cfg.integral_max, self._integral))

        output = saturated_output
        self._prev_measurement = measurement
        self._prev_error = error
        self._last_output = output
        self._total_steps += 1

        return output

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def last_output(self) -> float:
        return self._last_output

    def pause(self):
        self._is_active = False

    def resume(self):
        self._is_active = True


class AngularPIDController(PIDController):
    """
    Açısal PID kontrolcü.
    Açı sarması (±π) ve heading wraparound'ı doğru işler.
    """

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Açıyı [-π, π] aralığına sar."""
        import math
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def update(self, setpoint: float, measurement: float,
               dt: float, feedforward: float = 0.0) -> float:
        """Açı hatası için wrap işlemi uygulayarak PID hesapla."""
        import math
        # Setpoint'i sar
        sp = self._wrap_angle(setpoint)
        # Hata hesapla ve sar
        raw_error = sp - self._wrap_angle(measurement)
        wrapped_error = self._wrap_angle(raw_error)

        # Sanki measurement = 0, setpoint = wrapped_error gibi davran
        return super().update(
            setpoint=wrapped_error,
            measurement=0.0,
            dt=dt,
            feedforward=feedforward
        )
