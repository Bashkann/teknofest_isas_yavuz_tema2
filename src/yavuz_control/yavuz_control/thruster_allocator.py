#!/usr/bin/env python3
"""
YAVUZ AUV - Thruster Allocator
6-DOF kuvvet/tork talebini 6 thruster komutuna çevirir.

Thruster Konfigürasyonu:
  T0: Yatay Sol-Ön   @ (0.18,  0.11, 0.0) yaw=135°
  T1: Yatay Sağ-Ön  @ (0.18, -0.11, 0.0) yaw=-135°
  T2: Yatay Sol-Ark @ (-0.18, 0.11, 0.0) yaw=45°
  T3: Yatay Sağ-Ark @ (-0.18,-0.11, 0.0) yaw=-45°
  T4: Dikey Sol     @ (0.12,  0.22, 0.0) pitch=90°
  T5: Dikey Sağ     @ (0.12, -0.22, 0.0) pitch=90°

Çözüm yöntemi: Weighted Pseudo-Inverse (WPI)
  F_cmd = B+ * wrench
  B+ = W^-1 * B^T * (B * W^-1 * B^T)^-1
"""

import numpy as np
from typing import Tuple


class ThrusterAllocator:
    """
    6-thruster konfigürasyonu için kuvvet dağıtım matrisi tabanlı allocator.
    
    Kullanım:
        alloc = ThrusterAllocator()
        forces = alloc.allocate(
            surge=1.0, sway=0.0, heave=-0.5,
            roll=0.0, pitch=0.0, yaw=0.3
        )
        # forces: [f0, f1, f2, f3, f4, f5] (N cinsinden)
    """

    # Fiziksel limitler (DEGZ Mitras Datasheet)
    THRUST_MAX =  38.0  # N (forward)
    THRUST_MIN = -22.0  # N (backward, genellikle %60 forward thrust)
    
    # Thruster ağırlıkları (güç tüketimi eşitleme)
    WEIGHTS = np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

    def __init__(self):
        self._build_allocation_matrix()

    def _build_allocation_matrix(self):
        """
        B matrisi: her kolonu bir thruster'ın ürettiği wrench'i gösterir.
        B ∈ R^(6×n_thrusters)
        Satırlar: [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        import math

        # Thruster konumları ve yönleri
        # (x, y, z, roll, pitch, yaw) - yaw en önemli parametre
        config = [
            # x,    y,     z,   roll, pitch,   yaw (radyan)
            ( 0.18,  0.11, 0.0, 0.0, 0.0, math.radians(135)),  # T0
            ( 0.18, -0.11, 0.0, 0.0, 0.0, math.radians(-135)), # T1
            (-0.18,  0.11, 0.0, 0.0, 0.0, math.radians(45)),   # T2
            (-0.18, -0.11, 0.0, 0.0, 0.0, math.radians(-45)),  # T3
            ( 0.12,  0.22, 0.0, 0.0, math.pi/2, 0.0),          # T4 (dikey)
            ( 0.12, -0.22, 0.0, 0.0, math.pi/2, 0.0),          # T5 (dikey)
        ]

        n = len(config)
        B = np.zeros((6, n))

        for i, (x, y, z, roll, pitch, yaw) in enumerate(config):
            # Thrust yön vektörü (body frame)
            cr, sr = math.cos(roll),  math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw),   math.sin(yaw)

            # Rotation matrix: ZYX Euler
            fx = cy * cp
            fy = sy * cp
            fz = -sp

            # Kuvvet katkısı
            B[0, i] = fx  # Fx
            B[1, i] = fy  # Fy
            B[2, i] = fz  # Fz

            # Tork katkısı: τ = r × F
            B[3, i] = y * fz - z * fy  # Tx
            B[4, i] = z * fx - x * fz  # Ty
            B[5, i] = x * fy - y * fx  # Tz

        self.B = B
        self._compute_pseudo_inverse()

    def _compute_pseudo_inverse(self):
        """Ağırlıklı pseudo-inverse hesapla."""
        W_inv = np.linalg.inv(self.WEIGHTS)
        BW = self.B @ W_inv @ self.B.T
        # Moore-Penrose via NumPy (daha stabil)
        try:
            BW_inv = np.linalg.inv(BW + 1e-8 * np.eye(BW.shape[0]))
            self.B_pinv = W_inv @ self.B.T @ BW_inv
        except np.linalg.LinAlgError:
            self.B_pinv = np.linalg.pinv(self.B)

    def allocate(self,
                 surge: float = 0.0,
                 sway: float  = 0.0,
                 heave: float = 0.0,
                 roll: float  = 0.0,
                 pitch: float = 0.0,
                 yaw: float   = 0.0) -> np.ndarray:
        """
        İstenen wrench'i thruster komutlarına çevir.

        Args:
            surge, sway, heave: Kuvvetler (N)
            roll, pitch, yaw:   Torklar (N·m)

        Returns:
            np.ndarray: 6 elemanlı thruster kuvveti vektörü (N)
        """
        wrench = np.array([surge, sway, heave, roll, pitch, yaw])
        raw_thrusts = self.B_pinv @ wrench

        # Scaling: herhangi bir thruster satüre oluyorsa tüm seti orantılı olarak scale et
        scaled = self._scale_thrusts(raw_thrusts)
        return scaled

    def _scale_thrusts(self, thrusts: np.ndarray) -> np.ndarray:
        """Thruster satürasyonunu önlemek için orantılı scaling."""
        result = np.copy(thrusts)
        
        # Pozitif yönde maksimum aşım
        pos_ratio = np.max(result / self.THRUST_MAX) if self.THRUST_MAX > 0 else 1.0
        # Negatif yönde
        neg_ratio = np.min(result / self.THRUST_MIN) if self.THRUST_MIN < 0 else 1.0
        
        scale = max(pos_ratio, neg_ratio, 1.0)
        if scale > 1.0:
            result /= scale
        
        # Hard clamp (güvenlik)
        result = np.clip(result, self.THRUST_MIN, self.THRUST_MAX)
        return result

    def thrust_to_pwm(self, thrust_n: float) -> int:
        """
        Newton → PWM dönüşümü (ESC için).
        Doğrusal model (gerçekte quadratic, daha sonra kalibre edilecek).
        
        Returns:
            int: PWM mikrosaniye [1100, 1900]
        """
        if thrust_n >= 0:
            normalized = thrust_n / self.THRUST_MAX
        else:
            normalized = thrust_n / abs(self.THRUST_MIN)
        
        pwm = 1500 + int(normalized * 400)
        return max(1100, min(1900, pwm))

    def get_allocation_matrix(self) -> np.ndarray:
        """B matrisini döndür (debug için)."""
        return self.B.copy()

    def get_pseudoinverse(self) -> np.ndarray:
        """B+ pseudo-inverse matrisini döndür."""
        return self.B_pinv.copy()
