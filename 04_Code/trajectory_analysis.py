"""
Trajectory Analysis -- Encoder vs Vision Comparison
====================================================
BLEGS Quadruped Robot: 2-DOF Separated Five-Bar Linkage
เปรียบเทียบตำแหน่งปลายขาจาก Encoder (Forward Kinematics)
กับตำแหน่งจริงที่วัดจาก ArUco Marker (Vision-based)

ที่มา: https://github.com/M-TRCH/BLEGS_Actuator-Unit/tree/migrate-python-files
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DATA_DIR = Path(__file__).resolve().parent.parent / "03_Data"
REAL_DIR = DATA_DIR / "real"
VISION_DIR = DATA_DIR / "vision"

# ไฟล์ข้อมูล (เปลี่ยนชื่อตามไฟล์จริง)
REAL_FILE = REAL_DIR / "real_walk600.csv"
VISION_FILE = VISION_DIR / "linkage_data.csv"

# พารามิเตอร์ลิงก์ (mm)
L_AC = 105.0
L_BD = 105.0
L_CE = 145.0
L_DE = 145.0
MOTOR_SPACING = 85.0  # mm


# ---------------------------------------------------------------------------
# Data Loading
# ---------------------------------------------------------------------------
def load_real_data(filepath: Path) -> pd.DataFrame:
    """โหลดข้อมูล Log จากมอเตอร์จริง (Encoder)"""
    df = pd.read_csv(filepath)
    return df


def load_vision_data(filepath: Path) -> pd.DataFrame:
    """โหลดข้อมูลตำแหน่งจาก ArUco Vision Tracking"""
    df = pd.read_csv(filepath)
    return df


# ---------------------------------------------------------------------------
# RMSE Calculation
# ---------------------------------------------------------------------------
def compute_rmse(actual: np.ndarray, reference: np.ndarray) -> float:
    """คำนวณ RMSE ระหว่างข้อมูล actual กับ reference"""
    diff = actual - reference
    return np.sqrt(np.mean(diff**2))


def compute_trajectory_rmse(
    real_x: np.ndarray,
    real_y: np.ndarray,
    ref_x: np.ndarray,
    ref_y: np.ndarray,
    n_grid: int = 500,
) -> dict:
    """
    คำนวณ RMSE ของ trajectory โดยใช้ interpolation บน common X grid
    คืนค่า RMSE แยกตามแกนและรวม
    """
    # หา overlap ของ X range
    x_lo = max(real_x.min(), ref_x.min())
    x_hi = min(real_x.max(), ref_x.max())
    x_grid = np.linspace(x_lo, x_hi, n_grid)

    # Sort ตาม X ก่อน interpolate
    real_sort = np.argsort(real_x)
    ref_sort = np.argsort(ref_x)

    real_interp = np.interp(x_grid, real_x[real_sort], real_y[real_sort])
    ref_interp = np.interp(x_grid, ref_x[ref_sort], ref_y[ref_sort])

    rmse_y = compute_rmse(real_interp, ref_interp)

    return {
        "rmse_y": rmse_y,
        "x_range": (x_lo, x_hi),
        "n_points": n_grid,
    }


# ---------------------------------------------------------------------------
# Motor Angle RMSE
# ---------------------------------------------------------------------------
def compute_motor_rmse(
    df_real: pd.DataFrame,
    leg: str,
    joint: str,
    time_window: float = 2.0,
) -> dict:
    """
    คำนวณ RMSE ของมุมมอเตอร์ (Real Actual vs Target Setpoint)
    สำหรับขาและข้อต่อที่ระบุ
    """
    col_setpoint = f"{joint}_setpoint_deg"
    col_actual = f"{joint}_actual_deg"

    data = df_real[df_real["leg"] == leg].copy()
    data["time_norm"] = data["time"] - data["time"].min()
    data = data[data["time_norm"] <= time_window]

    if data.empty or col_setpoint not in data.columns:
        return {"rmse": np.nan, "n_points": 0}

    rmse = compute_rmse(data[col_actual].values, data[col_setpoint].values)
    return {
        "rmse": rmse,
        "n_points": len(data),
        "mean_error": (data[col_actual] - data[col_setpoint]).mean(),
        "max_error": (data[col_actual] - data[col_setpoint]).abs().max(),
    }


# ---------------------------------------------------------------------------
# Dominant Frequency Estimation
# ---------------------------------------------------------------------------
def estimate_dominant_frequency(time_arr: np.ndarray, angle_arr: np.ndarray) -> float:
    """ประมาณความถี่หลักของสัญญาณด้วย FFT"""
    signal = angle_arr - angle_arr.mean()
    dt = np.mean(np.diff(time_arr))
    if dt <= 0 or len(signal) < 4:
        return np.nan
    fs = 1.0 / dt
    n = len(signal)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    fft_mag = np.abs(np.fft.rfft(signal))
    dominant_idx = np.argmax(fft_mag[1:]) + 1
    return freqs[dominant_idx]


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_foot_trajectory(
    vision_x: np.ndarray,
    vision_y: np.ndarray,
    encoder_x: np.ndarray = None,
    encoder_y: np.ndarray = None,
    save_path: Path = None,
) -> None:
    """พล็อตเปรียบเทียบ trajectory ปลายขา (Vision vs Encoder)"""
    fig, ax = plt.subplots(figsize=(10, 6))

    ax.plot(vision_x, vision_y, "r-", linewidth=2.5, alpha=0.8,
            label="Real (ArUco Vision)")

    if encoder_x is not None and encoder_y is not None:
        ax.plot(encoder_x, encoder_y, "b--", linewidth=2,
                label="Encoder (FK)")

    ax.set_xlabel("X (mm)", fontsize=14)
    ax.set_ylabel("Y (mm)", fontsize=14)
    ax.set_title("Foot Trajectory Comparison", fontsize=16, fontweight="bold")
    ax.legend(loc="upper right", fontsize=12)
    ax.grid(True, linestyle=":", alpha=0.7)
    ax.set_aspect("equal", adjustable="box")

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


def plot_motor_tracking(
    df_real: pd.DataFrame,
    time_window: float = 2.0,
    save_path: Path = None,
) -> None:
    """พล็อตเปรียบเทียบมุมมอเตอร์ของทุกขา (Target vs Actual)"""
    legs = ["FL", "FR", "RL", "RR"]
    joints = ["theta1", "theta2"]

    fig, axs = plt.subplots(4, 2, figsize=(16, 16), sharex=True)
    fig.suptitle("Motor Tracking: Target Setpoint vs Real Actual",
                 fontsize=20, fontweight="bold", y=0.98)

    for i, leg in enumerate(legs):
        for j, joint in enumerate(joints):
            ax = axs[i, j]
            col_sp = f"{joint}_setpoint_deg"
            col_act = f"{joint}_actual_deg"

            data = df_real[df_real["leg"] == leg].copy()
            if data.empty:
                ax.set_title(f"Leg: {leg} | Motor: {joint} -- (no data)")
                continue

            data["time_norm"] = data["time"] - data["time"].min()
            zoom = data[data["time_norm"] <= time_window]

            ax.plot(zoom["time_norm"], zoom[col_sp],
                    color="black", linestyle="--", linewidth=2,
                    label="Target Setpoint")
            ax.plot(zoom["time_norm"], zoom[col_act],
                    color="red", linestyle="-", linewidth=2.5, alpha=0.8,
                    label="Real Actual")

            rmse = compute_rmse(zoom[col_act].values, zoom[col_sp].values)
            ax.set_title(f"Leg: {leg} | Motor: {joint} (RMSE: {rmse:.2f} deg)",
                         fontsize=14)
            ax.set_ylabel("Angle (deg)", fontsize=12)
            ax.grid(True, linestyle=":", alpha=0.7)

            if i == 0 and j == 0:
                ax.legend(loc="upper right", fontsize=12)
            if i == 3:
                ax.set_xlabel("Time (s)", fontsize=14)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    if save_path:
        plt.savefig(save_path, dpi=300)
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 60)
    print("  BLEGS Trajectory Analysis")
    print("  Encoder vs Vision Comparison")
    print("=" * 60)

    # -- โหลดข้อมูล Vision --
    if VISION_FILE.exists():
        print(f"\nVision data: {VISION_FILE}")
        df_vision = load_vision_data(VISION_FILE)
        vision_x = df_vision["E_x_mm"].values
        vision_y = df_vision["E_y_mm"].values
        print(f"  Frames: {len(df_vision)}")
        print(f"  X range: {vision_x.min():.1f} -- {vision_x.max():.1f} mm")
        print(f"  Y range: {vision_y.min():.1f} -- {vision_y.max():.1f} mm")
    else:
        print(f"\nไม่พบไฟล์ Vision: {VISION_FILE}")
        print("กรุณาคัดลอกข้อมูลจาก ArUco Tracking มาไว้ที่ 03_Data/vision/")
        return

    # -- โหลดข้อมูล Real Motor Log --
    if REAL_FILE.exists():
        print(f"\nReal motor data: {REAL_FILE}")
        df_real = load_real_data(REAL_FILE)
        print(f"  Records: {len(df_real)}")

        # คำนวณ RMSE ของมอเตอร์ทุกตัว
        print("\n--- Motor Angle RMSE ---")
        legs = ["FL", "FR", "RL", "RR"]
        joints = ["theta1", "theta2"]
        for leg in legs:
            for joint in joints:
                result = compute_motor_rmse(df_real, leg, joint)
                if not np.isnan(result["rmse"]):
                    print(f"  {leg}_{joint}: RMSE = {result['rmse']:.3f} deg"
                          f"  (max error = {result['max_error']:.3f} deg)")

        # พล็อต
        save_dir = Path(__file__).resolve().parent.parent / "02_Figures"
        plot_motor_tracking(df_real, save_path=save_dir / "motor_tracking.png")
    else:
        print(f"\nไม่พบไฟล์ Real: {REAL_FILE}")
        print("กรุณาคัดลอกข้อมูล Motor Log มาไว้ที่ 03_Data/real/")

    # -- พล็อต Vision trajectory --
    save_dir = Path(__file__).resolve().parent.parent / "02_Figures"
    plot_foot_trajectory(
        vision_x, vision_y,
        save_path=save_dir / "foot_trajectory_vision.png",
    )

    print("\nDone.")


if __name__ == "__main__":
    main()
