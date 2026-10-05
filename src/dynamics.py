"""Ters sarkaç (cart-pole) dinamiği.

Durum vektörü: s = [x, x_dot, theta, theta_dot]
  x      : araba konumu (m)
  theta  : çubuğun DİK konumdan sapması (rad), theta = 0 -> dik duruyor
  u      : arabaya yatay uygulanan kuvvet (N)

Çubuk, ucunda m kütlesi olan ağırlıksız çubuk (nokta kütle) olarak modellenir.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Params:
    M: float = 1.0    # araba kütlesi (kg)
    m: float = 0.1    # çubuk ucundaki kütle (kg)
    l: float = 0.5    # çubuk uzunluğu (m)
    g: float = 9.81   # yerçekimi (m/s^2)
    b: float = 0.1    # araba sürtünme katsayısı (N s/m)


def nonlinear_rhs(s, u, p: Params):
    """Doğrusal OLMAYAN hareket denklemleri -> ds/dt."""
    _, xd, th, thd = s
    sn, cs = np.sin(th), np.cos(th)
    xdd = (u - p.b * xd + p.m * p.l * thd**2 * sn - p.m * p.g * sn * cs) / (p.M + p.m * sn**2)
    thdd = (p.g * sn - xdd * cs) / p.l
    return np.array([xd, xdd, thd, thdd])


def linearized_matrices(p: Params):
    """theta = 0 civarında doğrusallaştırılmış sistem: ds/dt = A s + B u"""
    A = np.array([
        [0, 1, 0, 0],
        [0, -p.b / p.M, -p.m * p.g / p.M, 0],
        [0, 0, 0, 1],
        [0, p.b / (p.M * p.l), p.g * (p.M + p.m) / (p.M * p.l), 0],
    ])
    B = np.array([[0], [1 / p.M], [0], [-1 / (p.M * p.l)]])
    return A, B


def rk4_step(s, u, p: Params, dt: float):
    k1 = nonlinear_rhs(s, u, p)
    k2 = nonlinear_rhs(s + 0.5 * dt * k1, u, p)
    k3 = nonlinear_rhs(s + 0.5 * dt * k2, u, p)
    k4 = nonlinear_rhs(s + dt * k3, u, p)
    return s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
