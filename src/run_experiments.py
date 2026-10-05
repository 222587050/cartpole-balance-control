"""Tüm deneyleri çalıştırır ve results/ altına grafikleri kaydeder.

    python src/run_experiments.py
"""
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controllers import LQR, PID
from dynamics import Params
from simulate import is_success, simulate

RESULTS = Path(__file__).resolve().parent.parent / "results"
RESULTS.mkdir(exist_ok=True)

p = Params()
DT = 0.005
S0 = [0.0, 0.0, np.deg2rad(10), 0.0]   # başlangıç: 10 derece eğik


def make_pid():
    return PID(kp=60, ki=0, kd=10, dt=DT)


def make_pid_pos():
    return PID(kp=60, ki=0, kd=10, kx=2.0, kv=4.0, dt=DT)


def make_lqr():
    return LQR(p)


CONTROLLERS = {"PID (sadece açı)": make_pid, "PID + konum PD": make_pid_pos, "LQR": make_lqr}


def exp1_compare():
    """1) Aynı başlangıç koşulunda üç denetleyici."""
    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for name, mk in CONTROLLERS.items():
        t, S, U, fail = simulate(mk(), S0, p, T=10, dt=DT)
        tag = "" if fail is None else f" (başarısız, t={fail:.1f}s)"
        ax[0].plot(t, np.rad2deg(S[:, 2]), label=name + tag)
        ax[1].plot(t, S[:, 0])
        ax[2].plot(t, U)
    ax[0].set(ylabel="Açı (derece)", title="10° başlangıç eğiminden toparlanma")
    ax[1].set(ylabel="Araba konumu (m)")
    ax[2].set(ylabel="Kuvvet (N)", xlabel="Zaman (s)")
    ax[0].legend()
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "compare_pid_lqr.png", dpi=150)
    plt.close(fig)


def exp2_disturbance():
    """2) t=3 s'de arabaya 0.2 s'lik 15 N'luk itme."""
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for name, mk in CONTROLLERS.items():
        t, S, U, fail = simulate(mk(), S0, p, T=10, dt=DT, push=(3.0, 0.2, 15.0))
        tag = "" if fail is None else f" (başarısız, t={fail:.1f}s)"
        ax[0].plot(t, np.rad2deg(S[:, 2]), label=name + tag)
        ax[1].plot(t, S[:, 0])
    ax[0].axvspan(3.0, 3.2, color="gray", alpha=0.3)
    ax[0].set(ylabel="Açı (derece)", title="Dış bozucu (gri bölge: 15 N itme)")
    ax[1].set(ylabel="Araba konumu (m)", xlabel="Zaman (s)")
    ax[0].legend()
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "disturbance.png", dpi=150)
    plt.close(fig)


def exp3_basin():
    """3) Başlangıç açısı taraması: hangi açıdan başlayınca denge sağlanıyor?"""
    angles = np.arange(2, 62, 2)
    rates = {}
    print("\nBaşlangıç açısı taraması (başarılı olan en büyük açı):")
    for name, mk in CONTROLLERS.items():
        ok = []
        for a in angles:
            _, S, _, fail = simulate(mk(), [0, 0, np.deg2rad(a), 0], p, T=10, dt=DT)
            ok.append(is_success(S, fail))
        rates[name] = ok
        best = max((a for a, o in zip(angles, ok) if o), default=0)
        print(f"  {name:18s}: {best}° ({sum(ok)}/{len(ok)} başarılı)")
    fig, ax = plt.subplots(figsize=(8, 3 + 0.4 * len(CONTROLLERS)))
    for i, (name, ok) in enumerate(rates.items()):
        ax.scatter(angles, [i] * len(angles), c=["tab:green" if o else "tab:red" for o in ok], s=60)
    ax.set_yticks(range(len(rates)), list(rates.keys()))
    ax.set(xlabel="Başlangıç açısı (derece)", title="Denge sağlanan başlangıç açıları (yeşil: başarılı)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "basin.png", dpi=150)
    plt.close(fig)


def exp4_lqr_tuning():
    """4) LQR ağırlık seçimi: açıya verilen önem (Q_theta) arttıkça ne olur?"""
    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    print("\nLQR Q_theta taraması:")
    for qth in [1, 10, 100, 1000]:
        c = LQR(p, Q=np.diag([1, 1, qth, 1]))
        t, S, U, fail = simulate(c, S0, p, T=6, dt=DT)
        ax[0].plot(t, np.rad2deg(S[:, 2]), label=f"Q_theta={qth}")
        ax[1].plot(t, U)
        print(f"  Q_theta={qth:5d}: max |u|={np.max(np.abs(U)):.1f} N, K={np.round(c.K, 1)}")
    ax[0].set(ylabel="Açı (derece)", title="LQR: açı ağırlığının etkisi")
    ax[1].set(ylabel="Kuvvet (N)", xlabel="Zaman (s)")
    ax[0].legend()
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(RESULTS / "lqr_tuning.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    exp1_compare()
    exp2_disturbance()
    exp3_basin()
    exp4_lqr_tuning()
    print("\nKaydedildi: results/compare_pid_lqr.png, disturbance.png, basin.png, lqr_tuning.png")
