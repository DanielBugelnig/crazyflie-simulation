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
	X=${X:=$X}
	Y=${Y:=$Y}
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

	echo "Spawning ${MODEL}_${N} at ${X} ${Y}"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: 0.5}}, name: "'${MODEL}_${N}'", allow_renaming: 1'
	
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
	echo "Description: This script is used to spawn multiple vehicles in gazebo from a coordinates list text file."
	echo "Usage: $0 [-m <vehicle_model>] [-w <world>] [-f <file_name>] [-c] [-d <decks>] [-p <noise_profile>] [-H]"
	echo "  -c  enable the AI-deck camera, streamed on tcp://127.0.0.1:<5050+N>"
	echo "  -d  comma separated deck stack: aideck (same as -c), flowdeck, multiranger,"
	echo "      euroc (EuRoC VI-sensor stereo cameras, needs a profile with a euroc section: -p euroc),"
	echo "      corax (Corax layout of the same cameras: one down, one forward 45 deg up; -p corax unless -p is given)"
	echo "  -p  noise profile: path or name in simulator_files/gazebo/noise_profiles:"
	echo "        euroc           EuRoC stereo rig (with -d euroc): 2x MT9V034 752x480 20 Hz, 20 deg down, BMI088 IMU"
	echo "        corax           the same cameras in the Corax layout (with -d corax, its default):"
	echo "                        cam0 down, cam1 forward 45 deg up, no overlap"
	echo "        cf21_datasheet  Crazyflie 2.1 datasheet sensor noise and nominal deck masses"
	echo "        <path>          e.g. /workspace/configs/vio_sensor_profile.yaml for the AI-deck mono VIO (with -c)"
	echo "  -w  world: name in simulator_files/gazebo/worlds, or a path to an .sdf file (models in ../models)"
	echo "  -H  headless: no Gazebo GUI (saves GPU time; stop with Ctrl+C)"
	exit 1
fi

while getopts m:w:f:cd:p:H option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
		c) CAMERA=1;;
		d) DECKS=${OPTARG};;
		p) PROFILE=${OPTARG};;
		H) HEADLESS=1;;
		w) WORLD=${OPTARG};;
		f) COORDINATES_FILE=${OPTARG};;
	esac
done

world=${WORLD:=crazysim_default}
camera=${CAMERA:=0}
flowdeck=0
multiranger=0
euroc=0
default_profile=
for deck in ${DECKS//,/ }; do
	case "${deck}" in
		aideck) camera=1;;
		flowdeck) flowdeck=1;;
		multiranger) multiranger=1;;
		euroc) euroc=1;;
		corax) euroc=1; default_profile=corax;;
		*) echo "ERROR: unknown deck '${deck}' (supported: aideck, flowdeck, multiranger, euroc, corax)"; exit 1;;
	esac
done
profile=${PROFILE:=${default_profile}}
# spawn_model runs in the build directory, so make a relative profile path absolute
[ -n "${profile}" ] && [ -f "${profile}" ] && profile=$(realpath "${profile}")
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
coordinates_file=${COORDINATES_FILE:="single_origin.txt"}
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
while IFS= read -r line || [ -n "$line" ];do
	fields=($(printf "%s" "$line"|cut -d',' --output-delimiter=' ' -f1-))
	spawn_model ${vehicle_model} $(($n)) ${fields[0]} ${fields[1]}
	n=$(($n + 1))
done < "${src_path}/tools/crazyflie-simulation/drone_spawn_list/${coordinates_file}"

trap "cleanup" SIGINT SIGTERM EXIT

if [ -n "${HEADLESS}" ]; then
	echo "Running headless (no GUI), Ctrl+C to stop"
	wait
else
	echo "Starting gazebo gui"
	#gdb ruby
	gz sim -g
fi
