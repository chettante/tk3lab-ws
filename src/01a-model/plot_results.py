import os
import sys
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_genom3_log(file_path):
    """Carica un file .log GenoM3 estraendo l'header dai commenti e pulendo i tipi di dato."""
    if not os.path.exists(file_path):
        print(f"Warning: File non trovato -> {file_path}")
        return None

    header_cols = None
    with open(file_path, "r") as f:
        for line in f:
            if line.startswith("#"):
                parts = line.strip("#").strip().split()
                if len(parts) > 0 and parts[0] in ["ts", "date", "time"]:
                    header_cols = parts
            else:
                if header_cols is None:
                    header_cols = line.strip().split()
                break

    if not header_cols:
        print(
            f"Error: Impossibile identificare l'header nel file {file_path}"
        )
        return None

    try:
        df = pd.read_csv(
            file_path, comment="#", sep=r"\s+", header=None, low_memory=False
        )
    except Exception as e:
        print(f"Error durante la lettura di {file_path}: {e}")
        return None

    if df.empty:
        return None

    min_cols = min(df.shape[1], len(header_cols))
    df = df.iloc[:, :min_cols]
    df.columns = header_cols[:min_cols]

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["ts"]).reset_index(drop=True)
    return df


def plot_simulation_data(robot_name="quad"):
    ws_dir = os.environ.get("TK3LAB_WS", os.path.expanduser("~"))
    log_dir = os.path.join(ws_dir, "logs", "01a-model", robot_name)

    print(f"Caricamento log da: {log_dir}")

    df_opti = load_genom3_log(os.path.join(log_dir, "optitrack.log"))
    df_pom = load_genom3_log(os.path.join(log_dir, "pom-state.log"))
    df_nhfc = load_genom3_log(os.path.join(log_dir, "nhfc.log"))
    df_rotor = load_genom3_log(os.path.join(log_dir, "rotorcraft.log"))

    ts_list = []
    for df in [df_opti, df_pom, df_nhfc, df_rotor]:
        if df is not None and "ts" in df.columns:
            ts_list.append(df["ts"].min())

    if not ts_list:
        print("Errore: Nessun dato temporale valido trovato nei log!")
        return

    t0 = min(ts_list)

    if df_opti is not None:
        df_opti["t"] = df_opti["ts"] - t0
    if df_pom is not None:
        df_pom["t"] = df_pom["ts"] - t0
    if df_nhfc is not None:
        df_nhfc["t"] = df_nhfc["ts"] - t0
    if df_rotor is not None:
        df_rotor["t"] = df_rotor["ts"] - t0

    colors = {"x": "red", "y": "green", "z": "blue"}

    # FIGURA 1: Tracking Plots
    fig1, axs1 = plt.subplots(3, 2, figsize=(14, 10))
    fig1.suptitle(
        f"Tracking Plots (Desired vs Measured) - [{robot_name}]", fontsize=14
    )

    # Subplot (1,1): Position [m]
    ax = axs1[0, 0]
    if df_pom is not None and df_nhfc is not None:
        ax.plot(df_pom["t"], df_pom["x"], color=colors["x"], label="x (meas)")
        ax.plot(df_pom["t"], df_pom["y"], color=colors["y"], label="y (meas)")
        ax.plot(df_pom["t"], df_pom["z"], color=colors["z"], label="z (meas)")
        ax.plot(
            df_nhfc["t"],
            df_nhfc["xd"],
            color=colors["x"],
            linestyle="--",
            label="xd (des)",
        )
        ax.plot(
            df_nhfc["t"],
            df_nhfc["yd"],
            color=colors["y"],
            linestyle="--",
            label="yd (des)",
        )
        ax.plot(
            df_nhfc["t"],
            df_nhfc["zd"],
            color=colors["z"],
            linestyle="--",
            label="zd (des)",
        )
    ax.set_title("Position [m]")
    ax.set_xlim(left=0)
    ax.grid(True)
    ax.legend(loc="upper right", fontsize="small")

    # Subplot (1,2): Attitude Euler [deg] -- (EXCHANGED TO POSITION 1,2)
    ax = axs1[0, 1]
    if df_pom is not None and df_nhfc is not None:
        ax.plot(
            df_pom["t"],
            np.degrees(df_pom["roll"]),
            color=colors["x"],
            label="roll",
        )
        ax.plot(
            df_pom["t"],
            np.degrees(df_pom["pitch"]),
            color=colors["y"],
            label="pitch",
        )
        ax.plot(
            df_pom["t"],
            np.degrees(df_pom["yaw"]),
            color=colors["z"],
            label="yaw",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["rolld"]),
            color=colors["x"],
            linestyle="--",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["pitchd"]),
            color=colors["y"],
            linestyle="--",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["yawd"]),
            color=colors["z"],
            linestyle="--",
        )
    ax.set_title("Attitude Euler [deg]")
    ax.set_xlim(left=0)
    ax.grid(True)

    # Subplot (2,1): Linear Velocity [m/s] -- (EXCHANGED TO POSITION 2,1)
    ax = axs1[1, 0]
    if df_pom is not None and df_nhfc is not None:
        ax.plot(df_pom["t"], df_pom["vx"], color=colors["x"], label="vx (meas)")
        ax.plot(df_pom["t"], df_pom["vy"], color=colors["y"], label="vy (meas)")
        ax.plot(df_pom["t"], df_pom["vz"], color=colors["z"], label="vz (meas)")
        ax.plot(
            df_nhfc["t"],
            df_nhfc["vxd"],
            color=colors["x"],
            linestyle="--",
            label="vxd (des)",
        )
        ax.plot(
            df_nhfc["t"],
            df_nhfc["vyd"],
            color=colors["y"],
            linestyle="--",
            label="vyd (des)",
        )
        ax.plot(
            df_nhfc["t"],
            df_nhfc["vzd"],
            color=colors["z"],
            linestyle="--",
            label="vzd (des)",
        )
    ax.set_title("Linear Velocity [m/s]")
    ax.set_xlim(left=0)
    ax.grid(True)

    # Subplot (2,2): Angular Velocity [deg/s]
    ax = axs1[1, 1]
    if df_pom is not None and df_nhfc is not None:
        ax.plot(
            df_pom["t"], np.degrees(df_pom["wx"]), color=colors["x"], label="wx"
        )
        ax.plot(
            df_pom["t"], np.degrees(df_pom["wy"]), color=colors["y"], label="wy"
        )
        ax.plot(
            df_pom["t"], np.degrees(df_pom["wz"]), color=colors["z"], label="wz"
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["wxd"]),
            color=colors["x"],
            linestyle="--",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["wyd"]),
            color=colors["y"],
            linestyle="--",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["wzd"]),
            color=colors["z"],
            linestyle="--",
        )
    ax.set_title("Angular Velocity [deg/s]")
    ax.set_xlim(left=0)
    ax.grid(True)

    # Subplot (3,1): Linear Acceleration [m/s^2]
    ax = axs1[2, 0]
    if df_pom is not None and df_nhfc is not None:
        ax.plot(df_pom["t"], df_pom["ax"], color=colors["x"], label="ax")
        ax.plot(df_pom["t"], df_pom["ay"], color=colors["y"], label="ay")
        ax.plot(df_pom["t"], df_pom["az"], color=colors["z"], label="az")
        ax.plot(
            df_nhfc["t"], df_nhfc["axd"], color=colors["x"], linestyle="--"
        )
        ax.plot(
            df_nhfc["t"], df_nhfc["ayd"], color=colors["y"], linestyle="--"
        )
        ax.plot(
            df_nhfc["t"], df_nhfc["azd"], color=colors["z"], linestyle="--"
        )
    ax.set_title("Linear Acceleration [m/s^2]")
    ax.set_xlim(left=0)
    ax.grid(True)

    fig1.delaxes(axs1[2, 1])
    fig1.tight_layout()

    # FIGURA 2: Controller Errors
    fig2, axs2 = plt.subplots(2, 2, figsize=(12, 8))
    fig2.suptitle(f"Controller Errors - [{robot_name}]", fontsize=14)

    ax = axs2[0, 0]
    if df_nhfc is not None:
        ax.plot(df_nhfc["t"], df_nhfc["e_x"], color=colors["x"], label="e_x")
        ax.plot(df_nhfc["t"], df_nhfc["e_y"], color=colors["y"], label="e_y")
        ax.plot(df_nhfc["t"], df_nhfc["e_z"], color=colors["z"], label="e_z")
    ax.set_title("Position Errors [m]")
    ax.set_xlim(left=0)
    ax.grid(True)
    ax.legend()

    ax = axs2[0, 1]
    if df_nhfc is not None:
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_rx"]),
            color=colors["x"],
            label="e_roll",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_ry"]),
            color=colors["y"],
            label="e_pitch",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_rz"]),
            color=colors["z"],
            label="e_yaw",
        )
    ax.set_title("Attitude Errors [deg]")
    ax.set_xlim(left=0)
    ax.grid(True)

    ax = axs2[1, 0]
    if df_nhfc is not None:
        ax.plot(df_nhfc["t"], df_nhfc["e_vx"], color=colors["x"], label="e_vx")
        ax.plot(df_nhfc["t"], df_nhfc["e_vy"], color=colors["y"], label="e_vy")
        ax.plot(df_nhfc["t"], df_nhfc["e_vz"], color=colors["z"], label="e_vz")
    ax.set_title("Linear Velocity Errors [m/s]")
    ax.set_xlim(left=0)
    ax.grid(True)

    ax = axs2[1, 1]
    if df_nhfc is not None:
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_wx"]),
            color=colors["x"],
            label="e_wx",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_wy"]),
            color=colors["y"],
            label="e_wy",
        )
        ax.plot(
            df_nhfc["t"],
            np.degrees(df_nhfc["e_wz"]),
            color=colors["z"],
            label="e_wz",
        )
    ax.set_title("Angular Velocity Errors [deg/s]")
    ax.set_xlim(left=0)
    ax.grid(True)

    fig2.tight_layout()

    # FIGURA 3: Controller Inputs (Single Plot)
    fig3, ax3 = plt.subplots(1, 1, figsize=(10, 5))
    fig3.suptitle(f"Controller Inputs - [{robot_name}]", fontsize=14)

    if df_nhfc is not None:
        ax3.plot(df_nhfc["t"], df_nhfc["fx"], label="fx [N]", color="red")
        ax3.plot(df_nhfc["t"], df_nhfc["fy"], label="fy [N]", color="green")
        ax3.plot(df_nhfc["t"], df_nhfc["fz"], label="fz [N]", color="blue")
        ax3.plot(
            df_nhfc["t"],
            df_nhfc["tx"],
            label="tx [Nm]",
            color="orange",
            linestyle="--",
        )
        ax3.plot(
            df_nhfc["t"],
            df_nhfc["ty"],
            label="ty [Nm]",
            color="purple",
            linestyle="--",
        )
        ax3.plot(
            df_nhfc["t"],
            df_nhfc["tz"],
            label="tz [Nm]",
            color="brown",
            linestyle="--",
        )
    ax3.set_title("Controller Forces [N] and Torques [Nm]")
    ax3.set_xlim(left=0)
    ax3.grid(True)
    ax3.legend(ncol=2)

    fig3.tight_layout()
    plt.show()


if __name__ == "__main__":
    robot = sys.argv[1] if len(sys.argv) > 1 else "quad"
    plot_simulation_data(robot)