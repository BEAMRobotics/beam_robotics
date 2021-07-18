#!/bin/bash
set -e

# This script installs and compiles the entire beam robotics software stack
# Running this script with some parts already installed should be fine

# Specify location of installation scripts
INSTALL_SCRIPTS=$"$HOME/software/beam_install_scripts"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Ensure that Beam install scripts are installed
if [ -d $INSTALL_SCRIPTS ]; then
    echo "Beam install scripts found"
else
    echo "Cloning Beam install scripts into:"
    echo $INSTALL_SCRIPTS
    echo "Make sure they are not installed somewhere else."
    if [ ! -d "$HOME/software" ]; then
      mkdir -p "$HOME/software"
    fi
    git clone git@github.com:BEAMRobotics/beam_install_scripts.git $INSTALL_SCRIPTS
    echo "Success"
fi

main()
{
  bash $INSTALL_SCRIPTS/install.bash $@
}

main $@
