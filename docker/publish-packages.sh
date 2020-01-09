#!/bin/sh
#
# This scripts lists all .deb and .zip files in the "build/" directory and
# publishes them one by one to Cloudsmith.io
#

for f in build/*curaengine*.deb
do
    echo "Uploading DEB package '${f}' to Cloudsmith..."
    cloudsmith push deb \
      --api-key "${CLOUDSMITH_API_KEY}" \
      --republish "${CLOUDSMITH_DEB_REPO}" "${f}"
done

for f in build/*curaengine*.zip
do
    echo "Uploading RAW package '${f}' to Cloudsmith..."
    cloudsmith push raw \
      --api-key "${CLOUDSMITH_API_KEY}" \
      --republish "${CLOUDSMITH_RAW_REPO}" "${f}"
done
