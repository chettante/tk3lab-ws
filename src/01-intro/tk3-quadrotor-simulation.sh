#!/bin/sh

usage() {
  echo ""
  echo "Usage: $(basename $0) [options]"
  echo ""
  echo "Run the quadrotor simulation."
  echo "Precisely, run all the necessary GenoM3 components - for the target"
  echo "middleware - and the Gazebo simulator, using the telekyb3 example world"
  echo "file."
  echo ""
  echo "Options:"
  echo "  -h, --help"
  echo "      Show this helper message."
  echo "  -m, --middleware <pocolibs|ros>"
  echo "      Run the Genom3 components for target middleware."
  echo "      Default: pocolibs"
  echo "  -w, --world <filename>"
  echo "      Run the Gazebo simulator with the world file whose path is"
  echo "      'filename'."
  echo "      Default: /opt/openrobots/share/gazebo/worlds/example.world"
  echo ""
  exit 1
}

# parse input arguments
while [ $# -gt 0 ]; do
    case $1 in
        -m|--middleware)
            opt_middleware=$2; shift; shift;;
        -w|--world)
            opt_world="$2"; shift; shift;;
        -h|--help)
            usage; shift;;
        *)
            echo "Invalid option: $1"; usage;;
    esac
done
dft_middleware=pocolibs
dft_world=/opt/openrobots/share/gazebo/worlds/example.world

# Genom3 components to run
components="
  rotorcraft
  nhfc
  pom
  optitrack
  maneuver
"

# list of process ids to clean, populated after each spawn
pids=

# cleanup, called after ctrl-C
atexit() {
    trap - 0 INT CHLD
    set +e

    kill $pids
    wait
    case $middleware in
        pocolibs) h2 end;;
    esac
    exit 0
}
trap atexit 0 INT
set -e

# init middleware: if unset or null, then default is used
middleware=${opt_middleware:-$dft_middleware}
case $middleware in
    pocolibs) h2 init;;
    ros) roscore & pids="$pids $!";;
    *) echo "invalid middleware: $middleware"; usage;;
esac

# optionally run a genomix server for remote control
genomixd & pids="$pids $!"

# spawn required components
for c in $components; do
    $c-$middleware & pids="$pids $!"
done

# check if gazebo world exists: if unset or null, default is used
world=${opt_world:-$dft_world}
if [ ! -f $world ]; then
    echo "Cannot find world file: $world"
    usage
fi

# start gazebo
gz sim $world & pids="$pids $!"

# wait for ctrl-C or any background process failure
trap atexit CHLD
wait
