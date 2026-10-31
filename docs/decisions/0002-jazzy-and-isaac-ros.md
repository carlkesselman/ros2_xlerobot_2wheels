# ADR-0002: ROS 2 Jazzy, JetPack 7.2 on the Orin NX

- Status: Accepted
- Date: 2026-09-20

## Context

The robot's computer is a Jetson Orin NX, and the GPU packages worth having later are Isaac ROS's (cuVSLAM, nvblox,
cuMotion). Isaac ROS 4 supports only ROS 2 Jazzy and needs JetPack 7.2, the first JetPack 7 release for the Orin NX.
Isaac ROS 3.2 on JetPack 6 runs Humble.

## Decision

Jazzy for the whole stack, on JetPack 7.2 (L4T 39, Ubuntu 24.04, CUDA 13, TensorRT 10.16) on the Orin NX. The
development machine runs the same Ubuntu and Jazzy.

## Why

- Isaac ROS chose Jazzy; anything else gives up the reason for the Jetson.
- Jazzy is the current LTS, supported to 2029, and `sts_hardware_interface` is tested on it.
- One distribution, CUDA and TensorRT generation on the robot and the development machine: engines, containers and
  packages are built and tested the same way on both.

## Alternatives considered

Staying on JetPack 6.2 and running Isaac ROS 3.2 on Humble in a container, bridged to a Jazzy graph over DDS. It works
for plain message types, but it means two ROS graphs to maintain, and Isaac ROS's own development has moved on.

## Consequences

- Isaac ROS 4 packages can run on the robot; adopting them (cuVSLAM, nvblox, cuMotion) is a separate decision for each.
- Containers or tools that target JetPack 6 (L4T r36), including upstream XLeRobot's `jetson-containers` package, need
  checking on JetPack 7.2 before use.
