#!/bin/bash

# FIXME: Use script args
STL_DIRECTORY="../../NightlyTestModels/"
DEFINITION_FILE="../../Cura/resources/definitions/ultimaker_s5.def.json"
export CURA_ENGINE_SEARCH_PATH="../../Cura/resources/definitions:../../Cura/resources/extruders"
CURA_ENGINE_EXECUTABLE="../build/Release/CuraEngine"

RESULTS_DIRECTORY="results"
GCODE_DIRECTORY="$RESULTS_DIRECTORY/gcodes"
mkdir -p $GCODE_DIRECTORY

# install audria and activate environment
conan install audria/cci.20220429@_/_ --build=missing --update -if generators
. generators/conanrun.sh

# Loop through each STL file
declare -a LAYER_HEIGHTS=("0.06" "0.1" "0.3")

for STL_FILE in $STL_DIRECTORY/*.stl
do
    for LAYER_HEIGHT in "${LAYER_HEIGHTS[@]}"
    do
      declare -a TREE_SUPPORT_ENABLE=("true" "false")
      for TREE_SUPPORT in "${TREE_SUPPORT_ENABLE[@]}"
      do
          LAYER_HEIGHT_FILENAME=$(echo $LAYER_HEIGHT | tr '.' '_')
          echo "Processing stl: $STL_FILE with layer height: $LAYER_HEIGHT and tree_support_enabled: $TREE_SUPPORT"
          STL_SIZE=$(wc -c <"$STL_FILE")
          GCODE_FILE="$GCODE_DIRECTORY/$(basename $STL_FILE .stl)_${LAYER_HEIGHT_FILENAME}_${TREE_SUPPORT}.gcode"
          CSV_FILE="$RESULTS_DIRECTORY/$(basename $STL_FILE .stl)_${LAYER_HEIGHT_FILENAME}_${TREE_SUPPORT}.csv"

          CURAENGINE_LOG_LEVEL=error $CURA_ENGINE_EXECUTABLE slice --force-read-parent --force-read-nondefault -p -j $DEFINITION_FILE -s layer_height=$LAYER_HEIGHT -s support_structure=tree -s support_enable=$TREE_SUPPORT -l $STL_FILE -o $GCODE_FILE &

          audria $(pgrep CuraEngine) -d -1 -k -o $CSV_FILE
      done
    done
done