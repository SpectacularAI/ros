set -e

: "${ARCHITECTURE:=$(uname -m)}"
: "${ROS_DISTRO:=humble}"
: "${DEPTHAI_WS:=/underlay_ws}"

# Select correct release artifact for this OS architecture

if [ "$ARCHITECTURE" = "aarch64" ]; then

    SPECTACULARAI_SDK_RELEASE_ID="155423538"

elif [ "$ARCHITECTURE" = "x86_64" ]; then

    SPECTACULARAI_SDK_RELEASE_ID="155423514"

else
    echo "Unsupported architecture: $ARCHITECTURE"
    exit 1
fi

WORKDIR=$(pwd)
SDK=$(realpath sai_sdk)

# Remove temp files on exit
cleanup_sdk () {
    rm -rf $SDK
    find $WORKDIR -name 'libspectacularAI.a' -delete
    find $WORKDIR -name 'spectacular*.gz' -delete
    find $WORKDIR -name 'spectacular*.zip' -delete
    echo "Cleanup done (removed $SDK)"
}
trap cleanup_sdk EXIT

# Download SDK

mkdir -p $SDK
cd $SDK
curl -L -H "Accept: application/octet-stream" -H "Authorization: Bearer ${GITHUB_RAE_PAT_TOKEN}" -H "X-GitHub-Api-Version: 2022-11-28" -o spectacularai_sdk.zip https://api.github.com/repos/spectacularAI/sdk-for-rae/releases/assets/${SPECTACULARAI_SDK_RELEASE_ID}
unzip spectacularai_sdk.zip
cd $WORKDIR

# Build ROS node

. /opt/ros/${ROS_DISTRO}/setup.sh
. $DEPTHAI_WS/install/setup.sh
./scripts/build_all_static.sh sai_sdk/spectacularAI_*_static_rae.tar.gz
