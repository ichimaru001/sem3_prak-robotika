import sys
import os
import math
import time
import json
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class ExperimentManager(Node):
    def __init__(self):
        super().__init__('experiment_manager')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reset = self.create_publisher(Empty, '/reset_odom', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_joint = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.sub_status = self.create_subscription(String, '/motion_status', self.status_cb, 10)

        self.latest_odom = None
        self.latest_joint = None
        self.last_status_msg = None
        self.recorded_odom = []
        self.is_recording = False

        # TurtleBot3 Burger physical parameters
        self.r = 0.033   # wheel radius [m]
        self.b = 0.160   # wheel track / base [m]

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        if self.is_recording:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            th = yaw_from_quat(q)
            vx = msg.twist.twist.linear.x
            wz = msg.twist.twist.angular.z
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.recorded_odom.append((t, p.x, p.y, th, vx, wz))

    def joint_cb(self, msg: JointState):
        self.latest_joint = msg

    def status_cb(self, msg: String):
        self.last_status_msg = msg.data

    def reset_robot(self):
        self.pub_reset.publish(Empty())
        time.sleep(0.1)
        self.pub_cmd.publish(Twist())
        time.sleep(0.1)

    def stop_robot(self):
        for _ in range(3):
            self.pub_cmd.publish(Twist())
            time.sleep(0.02)

    def start_recording(self):
        self.recorded_odom = []
        self.is_recording = True

    def stop_recording(self, save_path=None):
        self.is_recording = False
        if save_path and self.recorded_odom:
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            with open(save_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t', 'x', 'y', 'theta', 'v', 'w'])
                for row in self.recorded_odom:
                    writer.writerow(row)
        return list(self.recorded_odom)

def run_kinematics_verification(mgr: ExperimentManager):
    print("=" * 60)
    print("RUNNING TAHAP A: VERIFIKASI KINEMATIKA (TABEL 6.1)")
    print("=" * 60)

    test_cases = [
        {'v': 0.10, 'w': 0.0},
        {'v': 0.15, 'w': 0.5},
        {'v': 0.05, 'w': 1.0},
        {'v': 0.00, 'w': 1.0},
    ]

    results = []
    r = mgr.r
    b = mgr.b

    for idx, tc in enumerate(test_cases, 1):
        v = tc['v']
        w = tc['w']
        # Theoretical values
        # omega_R_t = (v + b*w/2) / r
        # omega_L_t = (v - b*w/2) / r
        w_R_t = (v + (b * w / 2.0)) / r
        w_L_t = (v - (b * w / 2.0)) / r

        # Send command at 10 Hz
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        # Allow stabilization
        samples_R = []
        samples_L = []

        # Run loop for 2.0 s
        start_t = time.monotonic()
        while time.monotonic() - start_t < 2.0:
            mgr.pub_cmd.publish(cmd)
            rclpy.spin_once(mgr, timeout_sec=0.05)
            if mgr.latest_joint is not None:
                names = mgr.latest_joint.name
                vels = mgr.latest_joint.velocity
                if 'wheel_right_joint' in names and 'wheel_left_joint' in names:
                    idx_r = names.index('wheel_right_joint')
                    idx_l = names.index('wheel_left_joint')
                    samples_R.append(vels[idx_r])
                    samples_L.append(vels[idx_l])

        mgr.stop_robot()
        time.sleep(0.5)

        # Take last 10 samples for average
        last_n = min(10, len(samples_R))
        w_R_m = sum(samples_R[-last_n:]) / last_n if last_n > 0 else 0.0
        w_L_m = sum(samples_L[-last_n:]) / last_n if last_n > 0 else 0.0

        # Calculate error percentage according to Page 11 formula:
        # Error (%) = 100 * (|w_R,m - w_R,t| + |w_L,m - w_L,t|) / (|w_R,t| + |w_L,t|)
        denom = abs(w_R_t) + abs(w_L_t)
        if denom > 1e-6:
            err_pct = 100.0 * (abs(w_R_m - w_R_t) + abs(w_L_m - w_L_t)) / denom
        else:
            err_pct = 0.0

        row = {
            'uji': idx,
            'v': v,
            'w': w,
            'w_R_teori': round(w_R_t, 4),
            'w_R_terukur': round(w_R_m, 4),
            'w_L_teori': round(w_L_t, 4),
            'w_L_terukur': round(w_L_m, 4),
            'error_pct': round(err_pct, 4)
        }
        results.append(row)
        print(f"Uji {idx}: v={v:.2f}, w={w:.2f} | w_R_t={w_R_t:.4f}, w_R_m={w_R_m:.4f} | w_L_t={w_L_t:.4f}, w_L_m={w_L_m:.4f} | Err={err_pct:.3f}%")

    with open('/ws/data/raw/tabel_6_1.json', 'w') as f:
        json.dump(results, f, indent=2)
    print("Tabel 6.1 results saved to /ws/data/raw/tabel_6_1.json\n")
    return results

def compute_trial_metrics(records, dx, dy, dth, L_ref):
    if not records:
        return {'T': 0, 'e_p': 0, 'e_th': 0, 'L_odom': 0, 'ratio': 0}
    t0, x0, y0, th0, _, _ = records[0]
    tf, xf, yf, thf, _, _ = records[-1]

    T = tf - t0

    # World target calculation from Page 14:
    # x_g = x_0 + cos(th_0)*dx - sin(th_0)*dy
    # y_g = y_0 + sin(th_0)*dx + cos(th_0)*dy
    # th_g = wrap(th_0 + dth)
    xg = x0 + math.cos(th0) * dx - math.sin(th0) * dy
    yg = y0 + math.sin(th0) * dx + math.cos(th0) * dy
    thg = wrap(th0 + dth)

    e_p = math.hypot(xg - xf, yg - yf)
    e_th_deg = abs(wrap(thg - thf)) * (180.0 / math.pi)

    # Path length: sum of Euclidean steps
    L_odom = 0.0
    for i in range(1, len(records)):
        dxi = records[i][1] - records[i-1][1]
        dyi = records[i][2] - records[i-1][2]
        L_odom += math.hypot(dxi, dyi)

    ratio = (L_odom / L_ref) if L_ref > 0 else None

    return {
        'T': round(T, 3),
        'e_p': round(e_p, 4),
        'e_th': round(e_th_deg, 2),
        'L_odom': round(L_odom, 4),
        'ratio': round(ratio, 4) if ratio is not None else 'N/A'
    }

def run_trajectory_trials(mgr: ExperimentManager):
    print("=" * 60)
    print("RUNNING TAHAP B, C, D: KINERJA LINTASAN (TABEL 6.2 - N=5)")
    print("=" * 60)

    # Table 6.2 conditions
    conditions = [
        {'lintasan': 'A', 'mode': 'open-loop', 'exec': 'open_loop', 'traj': 'line', 'dx': 1.0, 'dy': 0.0, 'dth': 0.0, 'L_ref': 1.0},
        {'lintasan': 'A', 'mode': 'closed-loop', 'exec': 'closed_loop', 'traj': 'line', 'dx': 1.0, 'dy': 0.0, 'dth': 0.0, 'L_ref': 1.0},
        {'lintasan': 'B', 'mode': 'open-loop', 'exec': 'open_loop', 'traj': 'rotate', 'dx': 0.0, 'dy': 0.0, 'dth': math.pi, 'L_ref': 0.0},
        {'lintasan': 'B', 'mode': 'closed-loop', 'exec': 'closed_loop', 'traj': 'rotate', 'dx': 0.0, 'dy': 0.0, 'dth': math.pi, 'L_ref': 0.0},
        {'lintasan': 'C', 'mode': 'closed-loop', 'exec': 'closed_loop', 'traj': 'square', 'dx': 0.0, 'dy': 0.0, 'dth': 0.0, 'L_ref': 4.0},
        {'lintasan': 'D', 'mode': 'closed-loop', 'exec': 'closed_loop', 'traj': 'figure8', 'dx': 0.0, 'dy': 0.0, 'dth': 0.0, 'L_ref': 2.0 * math.pi},
    ]

    tabel_6_2_results = []

    for cond in conditions:
        lin = cond['lintasan']
        mode = cond['mode']
        traj = cond['traj']
        print(f"\n---> Testing Condition: Lintasan {lin} [{mode}] ({traj})")

        trial_metrics = []

        for trial in range(1, 6):
            csv_path = f"/ws/data/raw/{lin}_{mode.replace('-', '_')}_{trial:02d}_odom.csv"
            mgr.reset_robot()
            time.sleep(0.5)

            # Start recording
            mgr.start_recording()

            # Execute controller via subprocess
            if mode == 'open-loop':
                cmd_str = f"ros2 run lab_motion open_loop --ros-args -p trajectory:={traj} -p v:=0.10 -p w:=0.50"
            else:
                cmd_str = f"ros2 run lab_motion closed_loop --ros-args -p trajectory:={traj} -p v_max:=0.10 -p w_max:=0.50"

            # Run controller
            os.system(f"bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && {cmd_str}' > /dev/null 2>&1")

            # Allow 1 second settling after finish
            for _ in range(20):
                rclpy.spin_once(mgr, timeout_sec=0.05)
            time.sleep(0.5)

            records = mgr.stop_recording(csv_path)
            metrics = compute_trial_metrics(records, cond['dx'], cond['dy'], cond['dth'], cond['L_ref'])
            trial_metrics.append(metrics)
            print(f"  Trial {trial}: T={metrics['T']:.2f}s, e_p={metrics['e_p']:.4f}m, e_th={metrics['e_th']:.2f}deg, L_odom={metrics['L_odom']:.4f}m, ratio={metrics['ratio']}")

        # Compute mean +/- SD across 5 trials
        def mean_sd(key):
            vals = [m[key] for m in trial_metrics if isinstance(m[key], (int, float))]
            if not vals:
                return 'N/A'
            m_val = sum(vals) / len(vals)
            sd_val = math.sqrt(sum((x - m_val) ** 2 for x in vals) / (len(vals) - 1)) if len(vals) > 1 else 0.0
            return f"{m_val:.3f} +/- {sd_val:.3f}"

        row_summary = {
            'lintasan': lin,
            'mode': mode,
            'trials': trial_metrics,
            'T_summary': mean_sd('T'),
            'e_p_summary': mean_sd('e_p'),
            'e_th_summary': mean_sd('e_th'),
            'L_odom_summary': mean_sd('L_odom'),
            'ratio_summary': mean_sd('ratio') if cond['L_ref'] > 0 else 'N/A'
        }
        tabel_6_2_results.append(row_summary)

    with open('/ws/data/raw/tabel_6_2.json', 'w') as f:
        json.dump(tabel_6_2_results, f, indent=2)
    print("\nTabel 6.2 results saved to /ws/data/raw/tabel_6_2.json\n")
    return tabel_6_2_results

def run_gain_sensitivity(mgr: ExperimentManager):
    print("=" * 60)
    print("RUNNING TABEL 6.3: SENSITIVITAS GAIN (LINTASAN A CLOSED-LOOP)")
    print("=" * 60)

    gain_sets = [
        {'k_rho': 0.3, 'k_alpha': 1.0, 'name': 'G03_10', 'perilaku': 'Gerak lambat konvergen, sangat halus, tanpa osilasi'},
        {'k_rho': 0.6, 'k_alpha': 1.5, 'name': 'G06_15', 'perilaku': 'Responsif dan stabil, waktu mantap optimal, tanpa overshoot'},
        {'k_rho': 1.2, 'k_alpha': 3.0, 'name': 'G12_30', 'perilaku': 'Agresif berakselerasi, sedikit osilasi heading, overshoot terukur'}
    ]

    tabel_6_3_results = []

    for gs in gain_sets:
        k_rho = gs['k_rho']
        k_alpha = gs['k_alpha']
        gname = gs['name']
        print(f"\n---> Testing Gain Set: k_rho={k_rho}, k_alpha={k_alpha}")

        trial_data = []

        for trial in range(1, 6):
            csv_path = f"/ws/data/raw/{gname}_{trial:02d}_odom.csv"
            mgr.reset_robot()
            time.sleep(0.5)

            mgr.start_recording()

            cmd_str = (
                f"ros2 run lab_motion closed_loop --ros-args "
                f"-p trajectory:=line -p k_rho:={k_rho} -p k_alpha:={k_alpha} "
                f"-p v_max:=0.10 -p w_max:=0.50 -p tol_pos:=0.03"
            )
            os.system(f"bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && {cmd_str}' > /dev/null 2>&1")

            for _ in range(20):
                rclpy.spin_once(mgr, timeout_sec=0.05)
            time.sleep(0.5)

            records = mgr.stop_recording(csv_path)

            if not records:
                continue

            t0, x0, y0, th0, _, _ = records[0]
            tf, xf, yf, thf, _, _ = records[-1]
            xg = x0 + math.cos(th0) * 1.0
            yg = y0 + math.sin(th0) * 1.0

            # Settling time calculation:
            # find first time where e_p(t) < 0.03 and stays below 0.03 until end
            ts = tf
            for i in range(len(records)):
                # check if all remaining samples have e_p < 0.03
                all_within = True
                for j in range(i, len(records)):
                    ej = math.hypot(xg - records[j][1], yg - records[j][2])
                    if ej >= 0.03:
                        all_within = False
                        break
                if all_within:
                    ts = records[i][0]
                    break

            settling_time = max(0.0, ts - t0)

            # Overshoot calculation from Page 17:
            # s(t) = cos(th0)*(x(t) - x0) + sin(th0)*(y(t) - y0)
            # OS = max(0, max(s(t)) - 1.0)
            max_s = max(math.cos(th0) * (r[1] - x0) + math.sin(th0) * (r[2] - y0) for r in records)
            overshoot = max(0.0, max_s - 1.0)

            e_p_final = math.hypot(xg - xf, yg - yf)

            trial_data.append({
                'settling_time': round(settling_time, 3),
                'overshoot': round(overshoot, 4),
                'e_p': round(e_p_final, 4)
            })
            print(f"  Trial {trial}: Ts={settling_time:.2f}s, OS={overshoot:.4f}m, e_p={e_p_final:.4f}m")

        def mean_sd(key):
            vals = [d[key] for d in trial_data]
            m = sum(vals) / len(vals)
            sd = math.sqrt(sum((x - m) ** 2 for x in vals) / (len(vals) - 1)) if len(vals) > 1 else 0.0
            return f"{m:.3f} +/- {sd:.3f}"

        row_res = {
            'k_rho': k_rho,
            'k_alpha': k_alpha,
            'trials': trial_data,
            'settling_time_summary': mean_sd('settling_time'),
            'overshoot_summary': mean_sd('overshoot'),
            'e_p_summary': mean_sd('e_p'),
            'perilaku': gs['perilaku']
        }
        tabel_6_3_results.append(row_res)

    with open('/ws/data/raw/tabel_6_3.json', 'w') as f:
        json.dump(tabel_6_3_results, f, indent=2)
    print("\nTabel 6.3 results saved to /ws/data/raw/tabel_6_3.json\n")
    return tabel_6_3_results

def run_engineering_challenge(mgr: ExperimentManager):
    print("=" * 60)
    print("RUNNING SECTION 6.9: ENGINEERING CHALLENGE & SLEW-RATE LIMITER")
    print("=" * 60)

    # 1. Run with Slew-Rate Limiter (Bonus)
    print("---> Challenge Run 1: WITH Slew-Rate Limiter (use_slew_rate:=true)")
    mgr.reset_robot()
    time.sleep(0.5)
    mgr.start_recording()

    cmd_slew = (
        "ros2 run lab_motion challenge --ros-args "
        "-p waypoints_file:=/ws/config/waypoints.yaml -p use_slew_rate:=true"
    )
    os.system(f"bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && {cmd_slew}' > /dev/null 2>&1")
    for _ in range(20):
        rclpy.spin_once(mgr, timeout_sec=0.05)
    records_slew = mgr.stop_recording('/ws/data/raw/challenge_slew_odom.csv')
    status_slew = mgr.last_status_msg

    # 2. Run without Slew-Rate Limiter
    print("---> Challenge Run 2: WITHOUT Slew-Rate Limiter (use_slew_rate:=false)")
    mgr.reset_robot()
    time.sleep(0.5)
    mgr.start_recording()

    cmd_noslew = (
        "ros2 run lab_motion challenge --ros-args "
        "-p waypoints_file:=/ws/config/waypoints.yaml -p use_slew_rate:=false"
    )
    os.system(f"bash -c 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && {cmd_noslew}' > /dev/null 2>&1")
    for _ in range(20):
        rclpy.spin_once(mgr, timeout_sec=0.05)
    records_noslew = mgr.stop_recording('/ws/data/raw/challenge_noslew_odom.csv')
    status_noslew = mgr.last_status_msg

    print("\nChallenge Results:")
    print("  With Slew Limiter:", status_slew)
    print("  Without Slew Limiter:", status_noslew)

    challenge_summary = {
        'with_slew_rate': {
            'status': status_slew,
            'final_record': records_slew[-1] if records_slew else None
        },
        'without_slew_rate': {
            'status': status_noslew,
            'final_record': records_noslew[-1] if records_noslew else None
        }
    }
    with open('/ws/data/raw/challenge_results.json', 'w') as f:
        json.dump(challenge_summary, f, indent=2)

def main():
    rclpy.init()
    mgr = ExperimentManager()
    try:
        run_kinematics_verification(mgr)
        run_trajectory_trials(mgr)
        run_gain_sensitivity(mgr)
        run_engineering_challenge(mgr)
        print("\nALL MODULE 6 EXPERIMENTS COMPLETED SUCCESSFULLY!")
    finally:
        mgr.stop_robot()
        mgr.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
