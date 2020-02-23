#!/usr/bin/bash
echo 'updating python pyfrc...'
pip install -U pyfrc --pre
echo 'updating python modules for robot'
pip install -U robotpy-commands-v1 robotpy-ctre robotpy-navx robotpy-rev robotpy-rev-color wpilib robotpy-wpiutil
echo 'Done'
