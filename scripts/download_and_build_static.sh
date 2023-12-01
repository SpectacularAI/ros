set -e

: "${ARCHITECTURE:=$(uname -m)}"
: "${ROS_DISTRO:=humble}"
: "${DEPTHAI_WS:=/underlay_ws}"

# Select correct release artifact for this OS architecture

if [ "$ARCHITECTURE" = "aarch64" ]; then

    SPECTACULARAI_SDK_RELEASE_ID="138415391"

elif [ "$ARCHITECTURE" = "x86_64" ]; then

    SPECTACULARAI_SDK_RELEASE_ID="138415384"

else
    echo "Unsupported architecture: $ARCHITECTURE"
    exit 1
fi

# Download SDK

mkdir -p sai_sdk
cd sai_sdk
curl -L -H "Accept: application/octet-stream" -H "Authorization: Bearer ${GITHUB_RAE_PAT_TOKEN}" -H "X-GitHub-Api-Version: 2022-11-28" -o spectacularai_sdk.zip https://api.github.com/repos/spectacularAI/sdk-for-rae/releases/assets/${SPECTACULARAI_SDK_RELEASE_ID}
unzip spectacularai_sdk.zip
cd ..

# Build ROS node

. /opt/ros/${ROS_DISTRO}/setup.sh
. $DEPTHAI_WS/install/setup.sh
./scripts/build_all_static.sh sai_sdk/spectacularAI_*_static_rae.tar.gz

# Remove temp files

rm -rf sai_sdk
