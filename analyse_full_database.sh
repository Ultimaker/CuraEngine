for file in /home/t.kuipers/Development/CuraEngine/output/outlines/*.svg; do
 filename=$(basename "$file")
 fname="${filename%.*}"
 echo $file;
 ./release_build/arachne $file $fname
done
