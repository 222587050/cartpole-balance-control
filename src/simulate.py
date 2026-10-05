"""Kapalı çevrim simülasyonu: kuvvet doygunluğu, dış bozucu ve ray sınırı dahil."""
import numpy as np

from dynamics import Params, rk4_step

U_MAX = 30.0      # motorun uygulayabileceği maksimum kuvvet (N)
TRACK = 2.4       # ray yarı uzunluğu (m): |x| bunu aşarsa başarısız


def simulate(controller, s0, p: Params, T=10.0, dt=0.005, push=None, u_max=U_MAX):
    """push = (t_baslangic, sure, kuvvet) -> arabaya geçici dış kuvvet (bozucu)."""
    n = int(T / dt)
    t = np.arange(n) * dt
    S = np.zeros((n, 4))
    U = np.zeros(n)
    s = np.array(s0, dtype=float)
    failed_at = None
    for i in range(n):
        S[i] = s
        u = float(np.clip(controller(s), -u_max, u_max))
        U[i] = u
        d = 0.0
        if push is not None and push[0] <= t[i] < push[0] + push[1]:
            d = push[2]
        s = rk4_step(s, u + d, p, dt)
        if failed_at is None and (abs(s[2]) > np.pi / 2 or abs(s[0]) > TRACK):
            failed_at = t[i]
            S[i + 1:] = s
            U[i + 1:] = 0.0
            break
    return t, S, U, failed_at


def is_success(S, failed_at):
    """Başarı: hiç devrilmedi, raydan çıkmadı ve sonunda |theta| < 0.02 rad."""
    return failed_at is None and abs(S[-1, 2]) < 0.02
