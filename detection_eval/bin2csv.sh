#!/bin/bash

# This script converts bin files into csv usign the 'mavlogdump_modified.py' script

#declare -a all_types=( "IMU" "ATT" "BARO" "RCOU" "RGY2" "RAC2" "RGY" "RAC" "GT" "AC")

#declare -a all_types=( "RGY2" "RAC2" "RGY" "RAC")

declare -a all_types=("EVL6")

# declare -a all_types=( "MSG" "BARO" "POS")

# Remove .BIN suffix
newName=$(echo "$1" | cut -f 1 -d '.')
echo $newName
for i in "${all_types[@]}"
do 
  echo "extracting $i from $1"
  ./mavlogdump_modified.py --planner --format csv --types "$i"  $1 > logs/$newName"_"$i".csv"
  echo "$i"
done

