#!/bin/bash

IMAGE_NAME=ghcr.io/aica-technology/collections_kinova_gen3_collection
IMAGE_TAG=latest

HELP_MESSAGE="Usage: build.sh [options]
Options:
  --tag <TAG>              Specify the tag of the generated image.
                           (default: $IMAGE_TAG)

  -v|--verbose             Set the build output to verbose.

  --cache-id <id>          Invalidate the mount cache (e.g. CMake build folder)
                           by providing a new value.

  -r|--no-cache            Invalidate all cache (layer + mount).

  -h|--help                Show this help message.

  --                       Pass all following arguments to Docker
"

BUILD_FLAGS=()
CACHEID=0
while [ "$#" -gt 0 ]; do
  case "$1" in
    --tag) IMAGE_TAG=$2; shift 2;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain); shift 1;;
    --cache-id) CACHEID=$2; shift 2;;
    -r|--no-cache) BUILD_FLAGS+=(--no-cache); shift 1;;
    -h|--help) echo "$HELP_MESSAGE"; exit 0;;
    --) shift; break;;
    -*) echo "Unknown option: $1" >&2; echo "$HELP_MESSAGE"; exit 1;;
  esac
done

BUILD_FLAGS+=(--build-arg config.developer.caching.id="${CACHEID}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}:${IMAGE_TAG}")

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" -f aica-package.toml ${*} . || exit 1
