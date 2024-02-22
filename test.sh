for file in `ls ./NightlyTestModels/*.stl`;
          do
              ( time ./build/Release/CuraEngine slice --force-read-parent --force-read-nondefault -v -p -j ../ultimaker/Cura/resources/definitions/ultimaker_s3.def.json -l $file -o ../`basename $file .stl`.gcode ) 2> ../`basename $file .stl`.time
              break
          done