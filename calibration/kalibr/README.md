# Kalibr Docker

## Description

Though several methods exist for installing [Kalibr](https://github.com/ethz-asl/kalibr/wiki), Beam recommends using the docker image of Kalibr that is automatically installed via our [Installation Guide](https://github.com/BEAMRobotics/beam_robotics/wiki/Beam-Robotics-Installation-Guide)

For a description of how to use Kalibr for calibrating beam robots, see our [Calibration Procedure for Beam Robots](https://github.com/BEAMRobotics/beam_robotics/wiki/Calibration-Procedure-for-Beam-Robots)

## Installation

In a terminal, run

```docker
docker pull stereolabs/kalibr
```

To pull the most recent docker image. This image may be updated after the automatic installation

## Run Kalibr as a docker image

In a terminal, run

```shell
chmod +x run_kalibr.sh
sudo ./run_kalibr.sh <path-to-data-dir>
```

where `<path-to-data-dir>` is the path to the directory on your computer which contains your calibration bag file. This will open an interactive shell session inside the Kalibr docker container, which is automatically installed using the installation procedure for beam_robotics. See [Section C](https://github.com/ethz-asl/kalibr/wiki/installation) if a manual installation of Docker and the docker image of Kalibr is necessary. The data directory will be mounted inside the container at the path /root/data. The shell has the Kalibr workspace loaded.
