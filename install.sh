#!/usr/bin/env bash

# Create a log file of the installation process as well as displaying it
exec > >(tee view-adaptation-install.log)
exec 2>&1

echo "Setup View Adaptation"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="${SCRIPT_DIR}"/..

# TELECARLA
git clone https://github.com/hofbi/telecarla.git "$WS_SRC"/telecarla
"$WS_SRC"/telecalra/install.sh

echo "Finished setup of View Adpatation"
