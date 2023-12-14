#!/bin/bash

# Set the name and tag for the Docker image
REPO_NAME=$(basename "$(git rev-parse --show-toplevel)")
IMAGE_NAME="$REPO_NAME"
IMAGE_TAG="latest"

# Build the Docker image
docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" -f "$(dirname "$0")/Dockerfile" .
