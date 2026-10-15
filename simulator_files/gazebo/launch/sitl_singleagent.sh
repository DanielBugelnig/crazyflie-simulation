#!/bin/bash
function cleanup() {
	pkill -x cf2
	pkill -f crazysim_cpx.py
	pkill -9 ruby
}

function spawn_model() {
	MODEL=$1
	N=$2 # Cf ID
	X=$3 # spawn x position
	Y=$4 # spawn y position
	YAW=${5:-0} # spawn yaw [deg], counter-clockwise from +x
	X=${X:=$X}
	Y=${Y:=$Y}
	# yaw as a quaternion about z (the firmware gets Gazebo's pose, so its estimate starts with this yaw)
	QZ=$(python3 -c "import math; print(math.sin(math.radians(${YAW}) / 2))")
	QW=$(python3 -c "import math; print(math.cos(math.radians(${YAW}) / 2))")
	SUPPORTED_MODELS=("crazyflie", "crazyflie_thrust_upgrade")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null

	set --
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/models/${MODEL}/model.sdf.jinja
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo
	set -- ${@} --cffirm_udp_port $((19950+${N}))
	set -- ${@} --cflib_udp_port $((19850+${N}))
	set -- ${@} --cf_id $((${N}))
	set -- ${@} --cf_name cf
	set -- ${@} --camera ${camera}
	set -- ${@} --cam_frame_port $((5200+${N}))
	set -- ${@} --flowdeck ${flowdeck}
	set -- ${@} --multiranger ${multiranger}
	set -- ${@} --euroc ${euroc}
	[ -n "${profile}" ] && set -- ${@} --noise_profile ${profile}
	set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

	python3 ${@}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}, yaw ${YAW} deg"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}, orientation: {x: 0, y: 0, z: '${QZ}', w: '${QW}'}}, name: "'${MODEL}_${N}'", allow_renaming: 1'
	
	echo "starting instance $N in $(pwd)"
	$build_path/cf2 $((19950+${N})) > out.log 2> error.log &

	if [ "$camera" == "1" ]; then
		echo "starting AI-deck bridge $N: camera on tcp://127.0.0.1:$((5050+${N}))"
		python3 -u ${src_path}/tools/crazyflie-simulation/simulator_files/mujoco/crazysim_cpx.py --cpx-port $((5050+${N})) --cflib-port $((19850+${N})) --frame-port $((5200+${N})) --cf-id ${N} ${profile:+--profile ${profile}} > cpx.log 2>&1 &
	fi

	popd &>/dev/null
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
    echo "Description: This script is used to spawn a single vehicle in gazebo."
	echo "Usage: $0 [-m <vehicle_model>] [-x <x_coordinate>] [-y <y_coordinate>] [-Y <yaw_deg>] [-w <world>] [-c] [-d <decks>] [-p <noise_profile>] [-H]"
	echo "  -Y  spawn yaw in degrees, counter-clockwise from +x (default 0)"
	echo "  -c  enable the AI-deck camera, streamed on tcp://127.0.0.1:<5050+N>"
	echo "  -d  comma separated deck stack: aideck (same as -c), flowdeck, multiranger,"
	echo "      euroc (EuRoC VI-sensor stereo cameras, needs a profile with a euroc section: -p euroc)"
	echo "  -p  noise profile: path or name in simulator_files/gazebo/noise_profiles (e.g. cf21_datasheet)"
	echo "  -w  world: name in simulator_files/gazebo/worlds, or a path to an .sdf file (models in ../models)"
	echo "  -H  headless: no Gazebo GUI (saves GPU time; stop with Ctrl+C)"
	exit 1
fi

while getopts m:x:y:Y:w:cd:p:H option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
		c) CAMERA=1;;
		d) DECKS=${OPTARG};;
		p) PROFILE=${OPTARG};;
		H) HEADLESS=1;;
        x) X_CORD=${OPTARG};;
        y) Y_CORD=${OPTARG};;
        Y) YAW_DEG=${OPTARG};;
		w) WORLD=${OPTARG};;
	esac
done

world=${WORLD:=crazysim_default}
camera=${CAMERA:=0}
flowdeck=0
multiranger=0
euroc=0
for deck in ${DECKS//,/ }; do
	case "${deck}" in
		aideck) camera=1;;
		flowdeck) flowdeck=1;;
		multiranger) multiranger=1;;
		euroc) euroc=1;;
		*) echo "ERROR: unknown deck '${deck}' (supported: aideck, flowdeck, multiranger, euroc)"; exit 1;;
	esac
done
profile=${PROFILE:=}
# spawn_model runs in the build directory, so make a relative profile path absolute
[ -n "${profile}" ] && [ -f "${profile}" ] && profile=$(realpath "${profile}")
x_cord=${X_CORD:=0}
y_cord=${Y_CORD:=0}
yaw_deg=${YAW_DEG:=0}
[[ "${yaw_deg}" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "ERROR: -Y needs a number in degrees, got '${yaw_deg}'"; exit 1; }
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
export CF2_SIM_MODEL=gz_${vehicle_model}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."

build_path=${src_path}/sitl_make/build

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true
pkill -f crazysim_cpx.py || true

sleep 1

source ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash ${src_path} ${build_path}

# -w: a world name from simulator_files/gazebo/worlds, or a path to an .sdf file.
# For a file, the world name comes from <world name=...>, and the models next to
# it (<world dir>/../models) are found through model:// URIs.
if [ -f "${world}" ]; then
	world_file=$(realpath "${world}")
	world=$(grep -oP '<world\s+name="\K[^"]+' "${world_file}" | head -1)
	export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:$(dirname "${world_file}"):$(dirname "$(dirname "${world_file}")")/models
else
	world_file=${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/worlds/${world}.sdf
fi

echo "Starting gazebo"
gz sim -s -r --headless-rendering ${world_file} -v 3 &
sleep 3

n=0
spawn_model ${vehicle_model} $(($n)) ${x_cord} ${y_cord} ${yaw_deg}

trap "cleanup" SIGINT SIGTERM EXIT

if [ -n "${HEADLESS}" ]; then
	echo "Running headless (no GUI), Ctrl+C to stop"
	wait
else
	echo "Starting gazebo gui"
	#gdb ruby
	gz sim -g
fi
