#!/bin/bash
./gradlew frcUserProgramLinuxathenaReleaseExecutable
scp -v build/exe/frcUserProgram/linuxathena/release/frcUserProgram lvuser@roborio-4739-FRC.local:/home/lvuser/frcUserProgram
