#!/bin/sh
#Bertrand Vandeportaele 2018
#deprecated, generation of stl file is now done in the python program
mkdir -p stl
for i in `seq 1 9`; do
  echo "processing part $i"
  COMMAND="openscad -D printerpart=$i -o 'stl/p$i.stl' RoboArm_parts.scad"
  echo $COMMAND
  #execute the command in the variable
  eval "$COMMAND"
done


 
