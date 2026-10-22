import os
import glob
import math
import csv
import json
import matplotlib.pyplot as plt
import numpy as np

# Set publication-quality style
plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 11
plt.rcParams['axes.titlesize'] = 13
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['figure.autolayout'] = True

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, 'data', 'raw')
FIGURES_DIR = os.path.join(BASE_DIR, 'docs', 'figures')
os.makedirs(FIGURES_DIR, exist_ok=True)

def read_odom_csv(path):
    t_list, x_list, y_list, th_list, v_list, w_list = [], [], [], [], [], []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            t_list.append(float(r['t']))
            x_list.append(float(r['x']))
            y_list.append(float(r['y']))
            th_list.append(float(r['theta']))
            v_list.append(float(r['v']))
            w_list.append(float(r['w']))
    return np.array(t_list), np.array(x_list), np.array(y_list), np.array(th_list), np.array(v_list), np.array(w_list)

def plot_square_trajectory():
    csv_file = os.path.join(DATA_DIR, 'C_closed_loop_01_odom.csv')
    if not os.path.exists(csv_file):
        print(f"Skipping square plot: {csv_file} not found")
        return

    t, x, y, th, v, w = read_odom_csv(csv_file)
    # Local coordinate transform relative to start
    x0, y0, th0 = x[0], y[0], th[0]
    xl = np.cos(th0) * (x - x0) + np.sin(th0) * (y - y0)
    yl = -np.sin(th0) * (x - x0) + np.cos(th0) * (y - y0)

    # Reference points
    ref_x = [0.0, 1.0, 1.0, 0.0, 0.0]
    ref_y = [0.0, 0.0, 1.0, 1.0, 0.0]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Referensi (1x1 m)')
    ax.plot(xl, yl, 'b-', linewidth=2.2, label='Odometri Aktual (Closed-Loop)')
    ax.scatter([xl[0]], [yl[0]], color='green', s=100, zorder=5, label='Start (0,0)')
    ax.scatter([xl[-1]], [yl[-1]], color='red', s=100, marker='X', zorder=5, label='Finish')

    # Waypoints markers
    for px, py in [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]:
        ax.scatter([px], [py], color='gray', s=50, marker='o')

    ax.set_title('Lintasan C (Persegi 1x1 m): Referensi vs Odometri', fontweight='bold')
    ax.set_xlabel("X' Lokal (m)")
    ax.set_ylabel("Y' Lokal (m)")
    ax.set_aspect('equal')
    ax.set_xlim(-0.2, 1.3)
    ax.set_ylim(-0.2, 1.3)
    ax.legend(loc='upper left', frameon=True)
    ax.grid(True, linestyle=':', alpha=0.6)

    out_path = os.path.join(FIGURES_DIR, 'trajectory_square.png')
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_figure8_trajectory():
    csv_file = os.path.join(DATA_DIR, 'D_closed_loop_01_odom.csv')
    if not os.path.exists(csv_file):
        print(f"Skipping figure8 plot: {csv_file} not found")
        return

    t, x, y, th, v, w = read_odom_csv(csv_file)
    x0, y0, th0 = x[0], y[0], th[0]
    xl = np.cos(th0) * (x - x0) + np.sin(th0) * (y - y0)
    yl = -np.sin(th0) * (x - x0) + np.cos(th0) * (y - y0)

    # Reference figure 8 formula from Page 18:
    # Lingkaran 1: x = 0.5*sin(u), y = 0.5*(1 - cos(u))
    # Lingkaran 2: x = 0.5*sin(u), y = -0.5*(1 - cos(u))
    u = np.linspace(0, 2*np.pi, 200)
    ref1_x = 0.5 * np.sin(u)
    ref1_y = 0.5 * (1.0 - np.cos(u))
    ref2_x = 0.5 * np.sin(u)
    ref2_y = -0.5 * (1.0 - np.cos(u))

    ref_x = np.concatenate([ref1_x, ref2_x])
    ref_y = np.concatenate([ref1_y, ref2_y])

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Referensi Figure-Eight (R=0.5m)')
    ax.plot(xl, yl, color='darkorange', linewidth=2.2, label='Odometri Aktual (Closed-Loop)')
    ax.scatter([xl[0]], [yl[0]], color='green', s=100, zorder=5, label='Start (0,0)')
    ax.scatter([xl[-1]], [yl[-1]], color='red', s=100, marker='X', zorder=5, label='Finish')

    ax.set_title('Lintasan D (Figure-Eight): Referensi vs Odometri', fontweight='bold')
    ax.set_xlabel("X' Lokal (m)")
    ax.set_ylabel("Y' Lokal (m)")
    ax.set_aspect('equal')
    ax.set_xlim(-0.8, 0.8)
    ax.set_ylim(-1.2, 1.2)
    ax.legend(loc='upper left', frameon=True)
    ax.grid(True, linestyle=':', alpha=0.6)

    out_path = os.path.join(FIGURES_DIR, 'trajectory_figure8.png')
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_error_time_series():
    csv_file = os.path.join(DATA_DIR, 'A_closed_loop_01_odom.csv')
    if not os.path.exists(csv_file):
        print(f"Skipping error time series: {csv_file} not found")
        return

    t, x, y, th, v, w = read_odom_csv(csv_file)
    t_rel = t - t[0]

    # Target: local (1.0, 0.0) -> world (x0 + cos(th0)*1.0, y0 + sin(th0)*1.0)
    x0, y0, th0 = x[0], y[0], th[0]
    xg = x0 + math.cos(th0) * 1.0
    yg = y0 + math.sin(th0) * 1.0
    thg = th0

    e_p = np.hypot(xg - x, yg - y)
    e_th_deg = np.abs(np.arctan2(np.sin(thg - th), np.cos(thg - th))) * (180.0 / np.pi)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    # Subplot 1: Position Error e_p(t)
    ax1.plot(t_rel, e_p, color='#1f77b4', linewidth=2.2, label=r'$e_p(t)$ (Error Posisi)')
    ax1.axhline(0.03, color='crimson', linestyle='--', linewidth=1.5, label='Toleransi $tol_{pos} = 0.03$ m')
    ax1.set_ylabel('Error Posisi (m)')
    ax1.set_title('Respon Transien Trial A Closed-Loop: $e_p(t)$ dan $e_\\theta(t)$', fontweight='bold')
    ax1.legend(loc='upper right', frameon=True)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Heading Error e_th(t)
    ax2.plot(t_rel, e_th_deg, color='#2ca02c', linewidth=2.2, label=r'$e_\theta(t)$ (Error Heading)')
    ax2.axhline(5.0, color='darkorange', linestyle='--', linewidth=1.5, label='Toleransi $tol_{heading} = 5.0^\circ$')
    ax2.set_xlabel('Waktu sejak Start (s)')
    ax2.set_ylabel(r'Error Heading ($^\circ$)')
    ax2.legend(loc='upper right', frameon=True)
    ax2.grid(True, linestyle=':', alpha=0.6)

    out_path = os.path.join(FIGURES_DIR, 'error_time_series.png')
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_challenge_comparison():
    slew_file = os.path.join(DATA_DIR, 'challenge_slew_odom.csv')
    noslew_file = os.path.join(DATA_DIR, 'challenge_noslew_odom.csv')
    if not os.path.exists(slew_file) or not os.path.exists(noslew_file):
        print("Skipping challenge comparison: CSV files missing")
        return

    ts, xs, ys, ths, vs, ws = read_odom_csv(slew_file)
    tn, xn, yn, thn, vn, wn = read_odom_csv(noslew_file)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6))

    # Subplot 1: Trajectory in XY
    ref_x = [0.0, 1.0, 1.0]
    ref_y = [0.0, 0.0, 1.0]

    ax1.plot(ref_x, ref_y, 'k--', linewidth=2.0, label='Target Path (1m Fwd -> 90L -> 1m Fwd)')
    ax1.plot(xs, ys, color='#1f77b4', linewidth=2.2, label='Dengan Pembatas Akselerasi (Slew Limiter)')
    ax1.plot(xn, yn, color='#d62728', linestyle='-.', linewidth=2.0, label='Tanpa Pembatas Akselerasi')
    ax1.scatter([0.0], [0.0], color='green', s=100, zorder=5, label='Start')
    ax1.scatter([1.0], [1.0], color='gold', edgecolor='black', s=120, marker='*', zorder=5, label='Goal (1,1)')

    ax1.set_title('Lintasan Engineering Challenge (Section 6.9)', fontweight='bold')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_aspect('equal')
    ax1.legend(loc='lower right', frameon=True)
    ax1.grid(True, linestyle=':', alpha=0.6)

    # Subplot 2: Linear and Angular Velocity Profiles
    t_rel_s = ts - ts[0]
    t_rel_n = tn - tn[0]

    ax2.plot(t_rel_s, vs, color='#1f77b4', linewidth=2.0, label='Linear Vel (Slew Limiter)')
    ax2.plot(t_rel_n, vn, color='#d62728', linestyle=':', linewidth=1.8, label='Linear Vel (No Limiter)')
    ax2.plot(t_rel_s, ws, color='#2ca02c', linewidth=2.0, label='Angular Vel (Slew Limiter)')
    ax2.plot(t_rel_n, wn, color='#ff7f0e', linestyle=':', linewidth=1.8, label='Angular Vel (No Limiter)')

    ax2.set_title('Profil Kecepatan: Mulus vs Sentakan Akselerasi', fontweight='bold')
    ax2.set_xlabel('Waktu (s)')
    ax2.set_ylabel('Kecepatan (m/s atau rad/s)')
    ax2.legend(loc='upper right', frameon=True)
    ax2.grid(True, linestyle=':', alpha=0.6)

    out_path = os.path.join(FIGURES_DIR, 'challenge_slew_comparison.png')
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def plot_gain_sensitivity_comparison():
    g03 = os.path.join(DATA_DIR, 'G03_10_01_odom.csv')
    g06 = os.path.join(DATA_DIR, 'G06_15_01_odom.csv')
    g12 = os.path.join(DATA_DIR, 'G12_30_01_odom.csv')

    if not (os.path.exists(g03) and os.path.exists(g06) and os.path.exists(g12)):
        print("Skipping gain sensitivity plot: one or more CSV missing")
        return

    t1, x1, y1, th1, v1, w1 = read_odom_csv(g03)
    t2, x2, y2, th2, v2, w2 = read_odom_csv(g06)
    t3, x3, y3, th3, v3, w3 = read_odom_csv(g12)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t1 - t1[0], np.hypot(1.0 - x1, 0.0 - y1), label=r'$k_\rho=0.3, k_\alpha=1.0$ (Overdamped)', linewidth=2.0)
    ax.plot(t2 - t2[0], np.hypot(1.0 - x2, 0.0 - y2), label=r'$k_\rho=0.6, k_\alpha=1.5$ (Nominal)', linewidth=2.0)
    ax.plot(t3 - t3[0], np.hypot(1.0 - x3, 0.0 - y3), label=r'$k_\rho=1.2, k_\alpha=3.0$ (Underdamped)', linewidth=2.0)
    ax.axhline(0.03, color='red', linestyle='--', label=r'Toleransi $tol_{pos} = 0.03$ m')

    ax.set_title('Sensitivitas Gain: Perbandingan Respon $e_p(t)$ (Tabel 6.3)', fontweight='bold')
    ax.set_xlabel('Waktu sejak Start (s)')
    ax.set_ylabel('Error Posisi $e_p(t)$ (m)')
    ax.legend(loc='upper right', frameon=True)
    ax.grid(True, linestyle=':', alpha=0.6)

    out_path = os.path.join(FIGURES_DIR, 'gain_sensitivity_comparison.png')
    plt.savefig(out_path, dpi=300)
    plt.close()
    print(f"Saved: {out_path}")

def main():
    print("Generating publication plots for Modul 6...")
    plot_square_trajectory()
    plot_figure8_trajectory()
    plot_error_time_series()
    plot_challenge_comparison()
    plot_gain_sensitivity_comparison()
    print("All plots generated successfully.")

if __name__ == '__main__':
    main()
