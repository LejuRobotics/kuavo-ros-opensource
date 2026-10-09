#!/bin/bash
xhost +

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

CCACHE_DIR="${HOME}/.ccache"
mkdir -p "$CCACHE_DIR"

DIR_HASH=$(echo "$PARENT_DIR" | md5sum | cut -c1-8)
echo "Directory $PARENT_DIR hash: $DIR_HASH"
CONTAINER_NAME="kuavo_container_${DIR_HASH}"
# 自动使用最新的镜像版本
IMAGE_NAME=$(docker images kuavo_opensource_mpc_wbc_img --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1)

if [[ -z "$IMAGE_NAME" ]]; then
    echo -e "\033[33mWarning: No 'kuavo_opensource_mpc_wbc_img' Docker image found.\033[0m"
    read -r -p "The script can attempt to automatically download and import the image. Would you like to proceed? (yes/no): " response
    if [[ "$response" =~ ^([yY][eE][sS])$ ]]; then
        echo "Attempting to download/import 'kuavo_opensource_mpc_wbc_img'..."
        IMAGE_TARBALL_URL="https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_latest.tar.gz"
        IMAGE_TARBALL_NAME="kuavo_opensource_mpc_wbc_img_latest.tar.gz"
        DOWNLOAD_PATH="${SCRIPT_DIR}/${IMAGE_TARBALL_NAME}"

        echo "Downloading Docker image from ${IMAGE_TARBALL_URL}..."
        if wget -O "${DOWNLOAD_PATH}" "${IMAGE_TARBALL_URL}"; then
            echo "Download successful. Loading image into Docker..."
            if sudo docker load -i "${DOWNLOAD_PATH}"; then
                echo "Docker image loaded successfully."
                # Clean up the downloaded tarball
                rm -f "${DOWNLOAD_PATH}"
                # Re-evaluate IMAGE_NAME
                IMAGE_NAME=$(docker images kuavo_opensource_mpc_wbc_img --format "{{.Repository}}:{{.Tag}}" | sort -V | tail -n1)
                if [[ -z "$IMAGE_NAME" ]]; then
                    echo -e "\033[31mError: Failed to find the image name even after loading. Please check the image details.\033[0m"
                    exit 1
                else
                    echo -e "\033[32mSuccessfully loaded image: ${IMAGE_NAME}\033[0m"
                fi
            else
                echo -e "\033[31mError: Failed to load Docker image from ${DOWNLOAD_PATH}.\033[0m"
                rm -f "${DOWNLOAD_PATH}" # Clean up even on failure
                exit 1
            fi
        else
            echo -e "\033[31mError: Failed to download Docker image from ${IMAGE_TARBALL_URL}.\033[0m"
            exit 1
        fi
    else
        echo "Okay. Please build or pull the 'kuavo_opensource_mpc_wbc_img' image manually."
        echo "You can typically do this by navigating to the docker directory and running a build script (e.g., './build.sh'), or by pulling it from a registry."
        exit 1
    fi
fi

show_container_info() {
    local div_line="━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "\n$div_line"
    echo -e "📌 \033[34mContainer Info\033[0m: $CONTAINER_NAME"
    echo -e "📂 \033[32mWorking Directory\033[0m:"
    echo -e "   $PARENT_DIR"
    echo -e "🔗 \033[33mMounted Volumes\033[0m:"
    # docker inspect -f '{{range .Mounts}}{{.Source}} -> {{.Destination}}{{println}}{{end}}' $CONTAINER_NAME

    docker inspect -f '{{range .Mounts}}   {{.Source}} → {{.Destination}}{{println}}{{end}}' $CONTAINER_NAME
    echo -e "$div_line\n"
}


if [[ $(docker ps -aq -f ancestor=${IMAGE_NAME} -f name=${CONTAINER_NAME}) ]]; then
    echo "Container '${CONTAINER_NAME}' based on image '${IMAGE_NAME}' is already exists."
    if [[ $(docker ps -aq -f status=exited -f name=${CONTAINER_NAME}) ]]; then
        echo "Restarting exited container '$CONTAINER_NAME' ..."
        docker start $CONTAINER_NAME
    fi
    show_container_info
    echo "Exec into container '$CONTAINER_NAME' ..."
    docker exec -it $CONTAINER_NAME zsh
else
    echo "Creating a new container '${CONTAINER_NAME}' based on image '${IMAGE_NAME}' ..."
	docker run -it --net host \
		--name $CONTAINER_NAME \
		--privileged \
		-v /dev:/dev \
		-v "${HOME}/.ros:/root/.ros" \
		-v "$CCACHE_DIR:/root/.ccache" \
		-v "$PARENT_DIR:/root/kuavo_ws" \
		-v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig" \
		--group-add=dialout \
		--ulimit rtprio=99 \
		--cap-add=sys_nice \
		-e DISPLAY=$DISPLAY \
		-e ROBOT_VERSION=42 \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		${IMAGE_NAME} \
		zsh
fi
