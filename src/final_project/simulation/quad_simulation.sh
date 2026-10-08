#!/bin/sh


# select the used middleware and the gazebo world file to use
middleware=pocolibs
gz_world=~/tk3lab-ws/gazebo/worlds/example.world
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
WORKSPACE_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../../.." && pwd)
GZ_SIM_RESOURCE_PATH="${WORKSPACE_ROOT}/gazebo/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_RESOURCE_PATH

# Genom3 components that exist once, shared by all drones
shared_components="
  optitrack
"

# Genom3 components that must run once per drone. Each is launched twice below,
# with instance names <component>_1 and <component>_2, matching the '-i' names
# used in functions.py (rotorcraft_1/2, pom_1/2, nhfc_1/2, maneuver_1/2).
per_drone_components="
  nhfc
  pom
  rotorcraft
  maneuver
"

# list of process ids to clean, populated after each spawn
pids=

# cleanup, called after ctrl-C
atexit() {
    trap - 0 INT TERM CHLD
    set +e

    kill $pids
    # gz sim (wrapper ruby) e vglrun non inoltrano il segnale ai processi figli:
    # senza questo il server Gazebo resta vivo e il run dopo non riesce ad aprire
    # la porta di optitrack (1509)
    pkill -f "gz sim" 2>/dev/null
    wait
    case $middleware in
        pocolibs) h2 end;;
    esac
    exit 0
}
trap atexit 0 INT TERM
set -e

# middleware init, pocolibs or ros
case $middleware in
    pocolibs) h2 init;;
    ros) roscore & pids="$pids $!";;
    *) echo "invalid middleware: $middleware";
esac

# optionally run a genomix server for remote control
genomixd & pids="$pids $!"

# spawn shared components (one instance, default name)
for c in $shared_components; do
    $c-$middleware & pids="$pids $!"
done

# spawn per-drone components: two instances each, named <component>_1/_2.
# -f bypasses pocolibs' "multiple instances of the same component" detection,
# which would otherwise refuse to start the second instance.
for c in $per_drone_components; do
    $c-$middleware -f -i ${c}_1 & pids="$pids $!"
    $c-$middleware -f -i ${c}_2 & pids="$pids $!"
done

# If there is an error in the world file print it
if [ ! -f $gz_world ]; then
    echo "Cannot find world file: $gz_world"
    usage
fi

# allinea FOV e range della camera nel model.sdf a final_project/config.py
python3 "${SCRIPT_DIR}/sync_camera_fov.py"

# start gazebo
gazebo_partition=${GZ_PARTITION:-tk3lab}
gazebo_ip=${GZ_IP:-$(hostname -I | awk '{print $1}')}
gazebo_transport_args="GZ_PARTITION=$gazebo_partition GZ_IP=$gazebo_ip USER=$(id -un) LOGNAME=$(id -un)"

intel_gpu=0
for gpu_device in /sys/class/drm/card[0-9]*/device; do
    if [ -r "$gpu_device/vendor" ] && [ "$(cat "$gpu_device/vendor")" = "0x8086" ]; then
        intel_gpu=1
        break
    fi
done

if [ "$intel_gpu" -eq 1 ]; then
    intel_render_node=
    for render_node in /dev/dri/renderD*; do
        if [ -c "$render_node" ]; then
            intel_render_node=$render_node
            break
        fi
    done

    if [ -z "$intel_render_node" ]; then
        echo "Intel GPU detected, but no /dev/dri/renderD* node is mounted in the container." >&2
        exit 1
    fi

    if ! command -v setpriv >/dev/null 2>&1 || ! sudo -n true; then
        echo "Intel GPU render node found, but sudo/setpriv is unavailable for assigning its device groups to Gazebo." >&2
        exit 1
    fi

    gpu_group_ids=$(id -G | tr ' ' ',')
    for gpu_device in /dev/dri/card* /dev/dri/renderD*; do
        [ -c "$gpu_device" ] || continue
        gpu_gid=$(stat -c '%g' "$gpu_device")
        case ",${gpu_group_ids}," in
            *,"${gpu_gid}",*) ;;
            *) gpu_group_ids="${gpu_group_ids},${gpu_gid}" ;;
        esac
    done

    # Render the simulation and sensors through EGL on Intel; Xvnc's GUI is separate.
    echo "GPU Intel disponibile ($intel_render_node): server Gazebo headless su Mesa Iris"
    sudo -n setpriv --reuid="$(id -u)" --regid="$(id -g)" \
        --groups="$gpu_group_ids" env HOME="$(getent passwd "$(id -u)" | cut -d: -f6)" \
        $gazebo_transport_args \
        GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH" \
        GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}" \
        LIBGL_ALWAYS_SOFTWARE=0 \
        MESA_LOADER_DRIVER_OVERRIDE=iris gz sim -s -r --headless-rendering \
        "$gz_world" & pids="$pids $!"
    env $gazebo_transport_args gz sim -g & pids="$pids $!"
elif command -v vglrun >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    # container avviato con la GPU (tk3lab-run-gpu): tutto il rendering sulla NVIDIA
    #  - server: fisica + camera del follower renderizzata via EGL, senza display
    #  - GUI: renderizzata da VirtualGL e copiata nel desktop VNC
    echo "GPU NVIDIA disponibile: server headless + GUI con VirtualGL"
    gz sim -s --headless-rendering $gz_world & pids="$pids $!"
    vglrun -d egl gz sim -g & pids="$pids $!"
else
    # Other supported GPUs can still be selected automatically by Mesa.
    echo "GPU Intel/NVIDIA dedicata non rilevata: avvio Gazebo con renderer predefinito"
    gz sim $gz_world & pids="$pids $!"
fi

# Wait for ctrl-C or an explicit termination signal. A CHLD trap would run the
# global cleanup when any one component exits, even if the simulation is healthy.
wait
