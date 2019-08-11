for file in /home/t.kuipers/Documents/PhD/Variable_Width_project/implementation/libArachne/input/*.svg; do
 filename=$(basename "$file")
 fname="${filename%.*}"
 echo $file;
 ./release_build/arachne $file $fname
done
