#!/bin/sh


# select the used middleware and the gazebo world file to use
middleware=pocolibs
gz_world=~/tk3lab-ws/gazebo/worlds/example.world

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

# start gazebo
gz sim $gz_world & pids="$pids $!"

# wait for ctrl-C or any background process failure
trap atexit CHLD
wait
