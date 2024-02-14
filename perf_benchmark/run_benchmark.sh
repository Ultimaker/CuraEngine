#!/bin/bash

# FIXME: Use script args
STL_DIRECTORY="../../NightlyTestModels/"
DEFINITION_FILE="../../Cura/resources/definitions/ultimaker_s3.def.json"
export CURA_ENGINE_SEARCH_PATH="../../Cura/resources/definitions:../../Cura/resources/extruders"
CURA_ENGINE_EXECUTABLE="../build/Release/CuraEngine"

RESULTS_DIRECTORY="results"
GCODE_DIRECTORY="$RESULTS_DIRECTORY/gcodes"
mkdir -p $GCODE_DIRECTORY

# install audria and activate environment
conan install audria/cci.20220429@_/_ --build=missing --update -if generators
. generators/conanrun.sh

# Loop through each STL file
declare -a LAYER_HEIGHTS=("0.1" "0.15" "0.3")

for STL_FILE in $STL_DIRECTORY/*.stl
do
    for LAYER_HEIGHT in "${LAYER_HEIGHTS[@]}"
    do
        LAYER_HEIGHT_FILENAME=$(echo $LAYER_HEIGHT | tr '.' '_')
        echo "Processing stl: $STL_FILE with layer height: $LAYER_HEIGHT"
        STL_SIZE=$(wc -c <"$STL_FILE")
        OUTPUT_FILE="$(basename $STL_FILE .stl)_${LAYER_HEIGHT_FILENAME}.csv"
        GCODE_FILE="$GCODE_DIRECTORY/$(basename $STL_FILE .stl)_${LAYER_HEIGHT_FILENAME}.gcode"

        CURAENGINE_LOG_LEVEL=info $CURA_ENGINE_EXECUTABLE slice --force-read-parent --force-read-nondefault -v -p -j $DEFINITION_FILE -s layer_height=$LAYER_HEIGHT -l $STL_FILE -o $GCODE_FILE

        audria $(pgrep CuraEngine) -d -1 -k -o "$RESULTS_DIRECTORY/$(basename $STL_FILE .stl)_${LAYER_HEIGHT_FILENAME}.csv"
    done
done