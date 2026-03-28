#!/usr/bin/env bash
# Deploy RPi logger and run setup on the target Pi
# Usage: ./scripts/deploy_rpi.sh [hostname]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
RPI_HOST="${1:-axon-command.local}"
RPI_USER="${2:-axon}"
REMOTE_DIR="~/wake-on-radio"

echo "=== Deploying to ${RPI_USER}@${RPI_HOST} ==="

# Copy rpi/ directory
echo "Copying files..."
ssh "${RPI_USER}@${RPI_HOST}" "mkdir -p ${REMOTE_DIR}"
scp -r "${PROJECT_DIR}/rpi/"* "${RPI_USER}@${RPI_HOST}:${REMOTE_DIR}/"

# Make scripts executable
ssh "${RPI_USER}@${RPI_HOST}" "chmod +x ${REMOTE_DIR}/setup_rpi.sh ${REMOTE_DIR}/verify.sh"

# Run setup
echo ""
echo "Running setup..."
ssh -t "${RPI_USER}@${RPI_HOST}" "bash ${REMOTE_DIR}/setup_rpi.sh"
