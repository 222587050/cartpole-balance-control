"""LQR denetleyicisinin dengelediği cart-pole için GIF üretir.

    python src/animate.py
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from controllers import LQR
from dynamics import Params
from simulate import simulate

RESULTS = Path(__file__).resolve().parent.parent / "results"
RESULTS.mkdir(exist_ok=True)

p = Params()
t, S, U, _ = simulate(LQR(p), [0, 0, np.deg2rad(20), 0], p, T=6, dt=0.005, push=(3.0, 0.2, 15.0))

step = 6  # her 6. adımı çiz (~33 kare/sn)
frames = range(0, len(t), step)

fig, ax = plt.subplots(figsize=(7, 3.2))
ax.set(xlim=(-2.6, 2.6), ylim=(-0.3, 0.9), aspect="equal", yticks=[])
ax.axhline(0, color="k", lw=1)
cart = plt.Rectangle((-0.2, 0), 0.4, 0.15, color="tab:blue")
ax.add_patch(cart)
rod, = ax.plot([], [], "-", color="tab:red", lw=3)
bob, = ax.plot([], [], "o", color="tab:red", ms=10)
title = ax.set_title("")


def draw(i):
    x, th = S[i, 0], S[i, 2]
    cart.set_x(x - 0.2)
    px, py = x + p.l * np.sin(th), 0.15 + p.l * np.cos(th)
    rod.set_data([x, px], [0.15, py])
    bob.set_data([px], [py])
    title.set_text(f"LQR  t={t[i]:.1f}s  açı={np.rad2deg(th):+.1f}°" + ("  (itme!)" if 3.0 <= t[i] < 3.2 else ""))
    return cart, rod, bob, title


FuncAnimation(fig, draw, frames=frames, blit=False).save(RESULTS / "lqr_balance.gif", writer=PillowWriter(fps=30))
print("Kaydedildi: results/lqr_balance.gif")
