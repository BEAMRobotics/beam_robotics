#!/bin/bash
set -e
# This script installs and compiles the entire beam robotics software stack
# Running this script with some parts already installed should be fine
#
# Arguments:
# -y = skips the prompt
# -l = link only, creates all symlinks for the various parts of the project
# -u = unlink only, removes all symlinks


# Specify location of installation scripts
INSTALL_SCRIPTS=$"$HOME/beam_install_scripts"

# Ensure that Beam install scripts are installed
if [ -d $INSTALL_SCRIPTS ]; then
    echo "Beam install scripts found"
else
    echo "Cloning Beam install scripts into home directory"
    git clone https://github.com/BEAMRobotics/beam_install_scripts.git $INSTALL_SCRIPTS
    echo "Success"
fi



SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set the repo directory as an environment variable
export REPO_DIR=$(dirname "$SCRIPT_DIR")

# get UBUNTU_CODENAME, ROS_DISTRO, CATKIN_DIR
source $INSTALL_SCRIPTS/identify_environment.bash

: ${SYMLINKS_REPO_DIR:=$REPO_DIR}


read_args()
{
    ARG_NO_MENU=
    ARG_LINK=
    ARG_UNLINK=
    for arg in "$@"; do
        case $arg in
            -y)
                ARG_NO_MENU="true";;
            -l)
                ARG_LINK="true";;
            -u)
                ARG_UNLINK="true";;
        esac
    done
}

main()
{
    if [ -n "$ARG_LINK" ]; then
        link_routine
        exit
    elif [ -n "$ARG_UNLINK" ]; then
        unlink_routine
        exit
    fi

    install_routine
}

install_routine()
{
    sudo -v
    if [ -z "$ARG_NO_MENU" ] && [ -z "$CONTINUOUS_INTEGRATION" ]; then
        menu
    fi
    
    cd "$SCRIPTS_DIR"
    unlink_routine
    catkin_clean

    # submodule_init

    bash $INSTALL_SCRIPTS/ros_install.bash
    bash $INSTALL_SCRIPTS/create_catkin_workspace.bash

    link_routine
    bash $INSTALL_SCRIPTS/rosdeps_install.bash
    
    # Import functions to install required dependencies
    source $INSTALL_SCRIPTS/beam_dependencies_install.bash
    

    # Ensure wget is available
    sudo apt-get install -qq wget  > /dev/null
    # Install dependencies
    install_ceres
    install_pcl
    install_geographiclib
    install_gtsam
    install_libwave

    compile

    echo "Beam robotics installation completed. Please open a new terminal to re-source environment variables."
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        notify-send "Beam Robotics installation completed"
    fi
}

link_routine()
{
    ln -sfn "$SYMLINKS_REPO_DIR" "$CATKIN_DIR/src"
    echo "Symlink to $SYMLINKS_REPO_DIR created successfully"
}

unlink_routine()
{
    # Need to remove just the symlink for the linked repo
    REPO_BASE_NAME=$(basename "$SYMLINKS_REPO_DIR")
    rm -f "$CATKIN_DIR/src/$REPO_BASE_NAME"
    echo "Symlink $CATKIN_DIR/src/$REPO_BASE_NAME removed successfully"
}

menu()
{
    echo "Running this script will delete your /build /devel and /logs folders in your $CATKIN_DIR directory and re-build them."
    echo "Also, this script assumes the following things:"
    echo "  - Your ROS version is $ROS_DISTRO"
    echo "  - Your catkin workspace is located at: $CATKIN_DIR"
    echo "  - Catkin tools is installed"
    echo "  - Your bashrc sources $CATKIN_DIR/devel/setup.bash"
    echo "If any of the above assumptions are not true, the following script will make them so."
    echo "Do you wish to continue? (y/n):"

    while read ans; do
        case "$ans" in
            y) break;;
            n) exit; break;;
            *) echo "(y/n):";;
        esac
    done
}

catkin_clean()
{
    rm -rf "$CATKIN_DIR/devel"
    rm -rf "$CATKIN_DIR/build"
    rm -rf "$CATKIN_DIR/install"
    rm -rf "$CATKIN_DIR/logs"
    rm -f "$CATKIN_DIR/.catkin_workspace"
    echo "Catkin workspace cleaned"
}

submodule_init()
{
    git submodule -q update --init --recursive
}

compile()
{
    cd "$CATKIN_DIR"
    source /opt/ros/$ROS_DISTRO/setup.bash
    if [ -z "$CONTINUOUS_INTEGRATION" ]; then
        catkin build
    else
        if [ -n "$CIRCLECI" ]; then
            # Build libwave by itself first, since the job is so large
            catkin build --no-status -j2 libwave
            catkin build --no-status --mem-limit 6G
        else
            catkin build --no-status
        fi
    fi
}

main
