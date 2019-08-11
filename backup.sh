while true; do
CHANGED_FILES=$(find ./src/. -mmin -5 -type f)
now=$(date +"%Y_%m_%d__%H.%M")
if [ -z "$CHANGED_FILES" ]; then
  echo "$now: Nothing to save!"
else
  echo "$now: Backing up..."
  DIFF=$(git diff)
  if [ -n "$DIFF" ]; then
    git diff >> "backup/backup_$now.patch"
  fi
fi
sleep 5m
done
