#!/bin/bash
set -e

# This script 
# 1) installs and compiles the entire beam robotics software stack
# 2) installs and compiles optional installations 

# Ensure that Beam install scripts are installed
INSTALL_SCRIPTS=$"$HOME/software/beam_install_scripts"
if [ -d $INSTALL_SCRIPTS ]; then
    echo "Beam install scripts found"
else
    echo "Cloning Beam install scripts into: $INSTALL_SCRIPTS..."
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
