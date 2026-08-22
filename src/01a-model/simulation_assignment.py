import math
import os
import sys
import shutil
import time
import genomix

g = genomix.connect()
g.rpath(os.environ["HOME"] + "/openrobots/lib/genom/pocolibs/plugins")

optitrack = g.load("optitrack")
rotorcraft = g.load("rotorcraft")
pom = g.load("pom")
nhfc = g.load("nhfc")
TMP_LOG_DIR = "/tmp/tk3_logs"


def setup(robot_type):
    """Inizializza la configurazione e usa percorsi brevi per il logging GenoM3."""
    os.makedirs(TMP_LOG_DIR, exist_ok=True)

    # Definizione dati robot
    if robot_type == "quad":
        num_rotors = 4
        mocap_body = "optitrack/bodies/QR_4"
        pty_serial = "/tmp/pty-qr4"
        att_mode = "::nhfc::tilt_prioritized"
    elif robot_type in ["hexa-ua", "hexa-fa"]:
        num_rotors = 6
        mocap_body = "optitrack/bodies/HR_6"
        pty_serial = "/tmp/pty-hr6"
        att_mode = (
            "::nhfc::tilt_prioritized"
            if robot_type == "hexa-ua"
            else "::nhfc::full_attitude"
        )
    else:
        raise ValueError(f"Unknown robot_type: {robot_type}")

    # 1. OptiTrack
    optitrack.connect(
        {"host": "localhost", "host_port": "1509", "mcast": "", "mcast_port": "0"}
    )

    # 2. Rotorcraft
    rotorcraft.connect({"serial": pty_serial, "baud": 0})
    rotorcraft.set_sensor_rate(
        {"rate": {"imu": 1000, "mag": 0, "motor": 20, "battery": 1}}
    )
    rotorcraft.set_imu_filter(
        {"gfc": [20, 20, 20], "afc": [5, 5, 5], "mfc": [20, 20, 20]}
    )
    rotorcraft.connect_port(
        {"local": "rotor_input", "remote": "nhfc/rotor_input"}
    )

    # 3. NHFC
    nhfc.set_gtmrp_geom(
        {
            "rotors": num_rotors,
            "cx": 0,
            "cy": 0,
            "cz": 0,
            "armlen": 0.23,
            "mass": 1.28,
            "rx": 0,
            "ry": 0,
            "rz": -1,
            "cf": 6.5e-4,
            "ct": 1e-5,
        }
    )
    nhfc.set_emerg(
        {"emerg": {"descent": 0.1, "dx": 0.5, "dq": 1, "dv": 3, "dw": 3}}
    )
    nhfc.set_saturation({"sat": {"x": 1, "v": 1, "ix": 0}})
    nhfc.set_servo_gain(
        {
            "gain": {
                "Kpxy": 5,
                "Kpz": 5,
                "Kqxy": 4,
                "Kqz": 0.1,
                "Kvxy": 6,
                "Kvz": 6,
                "Kwxy": 1,
                "Kwz": 0.1,
                "Kixy": 0,
                "Kiz": 0,
            }
        }
    )
    nhfc.set_control_mode({"att_mode": att_mode})

    nhfc.connect_port(
        {"local": "rotor_measure", "remote": "rotorcraft/rotor_measure"}
    )
    nhfc.connect_port({"local": "state", "remote": "pom/frame/robot"})

    # 4. POM
    pom.set_prediction_model("::pom::constant_acceleration")
    pom.set_process_noise({"max_jerk": 100, "max_dw": 50})
    pom.set_history_length({"history_length": 0.25})
    pom.set_mag_field(
        {"magdir": {"x": 23.8e-06, "y": -0.4e-06, "z": -39.8e-06}}
    )

    pom.connect_port({"local": "measure/imu", "remote": "rotorcraft/imu"})
    pom.add_measurement("imu")
    pom.connect_port({"local": "measure/mag", "remote": "rotorcraft/mag"})
    pom.add_measurement("mag")
    pom.connect_port({"local": "measure/mocap", "remote": mocap_body})
    pom.add_measurement("mocap")

    # 5. Logging usando percorsi brevi (< 64 caratteri) in /tmp/tk3_logs
    optitrack.set_logfile(os.path.join(TMP_LOG_DIR, "optitrack.log"))
    pom.log_state(os.path.join(TMP_LOG_DIR, "pom-state.log"))
    pom.log_measurements(os.path.join(TMP_LOG_DIR, "pom-meas.log"))
    rotorcraft.log(os.path.join(TMP_LOG_DIR, "rotorcraft.log"))
    nhfc.log(os.path.join(TMP_LOG_DIR, "nhfc.log"))


def start():
    rotorcraft.start()
    rotorcraft.servo(ack=True)
    nhfc.set_current_position()


def run_waypoints():
    nhfc.set_position({"x": 0.0, "y": 0.0, "z": 10.0, "yaw": 0.0})
    time.sleep(2)
    nhfc.set_position({"x": 1.0, "y": 1.0, "z": 6.0, "yaw": 0.0})
    time.sleep(5)
    nhfc.set_position({"x": 1.0, "y": -1.0, "z": 3.0, "yaw": math.pi / 2.0})
    time.sleep(5)
    nhfc.set_position({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
    time.sleep(5)


def stop():
    rotorcraft.stop()
    rotorcraft.log_stop()
    nhfc.stop()
    nhfc.log_stop()
    pom.log_stop()
    optitrack.unset_logfile()


def simulation(robot_name):
    ws_dir = os.environ.get("TK3LAB_WS", os.path.expanduser("~"))
    log_directory = os.path.join(ws_dir, "logs", "01a-model", robot_name)
    os.makedirs(log_directory, exist_ok=True)

    print(f"--- Starting simulation for model: {robot_name} ---")
    setup(robot_name)
    start()
    run_waypoints()
    stop()

    # Spostamento dei file di log da /tmp/tk3_logs alla cartella finale del workspace
    for filename in os.listdir(TMP_LOG_DIR):
        src = os.path.join(TMP_LOG_DIR, filename)
        # Rinomina pom-meas.log in pom-measurements.log se necessario per gli script di plot
        dst_name = (
            "pom-measurements.log" if filename == "pom-meas.log" else filename
        )
        dst = os.path.join(log_directory, dst_name)
        shutil.move(src, dst)

    print(f"--- Complete. Logs moved to: {log_directory} ---")


if __name__ == "__main__":
    target_model = sys.argv[1] if len(sys.argv) > 1 else "quad"
    simulation(target_model)
