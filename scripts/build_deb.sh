#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="1.0.0"
PKG="obs-emeet-control"
STAGE="$REPO_DIR/dist/deb_stage"
OUT_DEB="$REPO_DIR/dist/${PKG}_${VERSION}_amd64.deb"
OUT_TAR="$REPO_DIR/dist/${PKG}-${VERSION}-linux-x86_64.tar.gz"

cmake -B "$REPO_DIR/build" -S "$REPO_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO_DIR/build" -j"$(nproc)"

rm -rf "$STAGE"
mkdir -p "$STAGE/DEBIAN" "$STAGE/usr/lib/x86_64-linux-gnu/obs-plugins"
cp "$REPO_DIR/build/obs-emeet-control.so" "$STAGE/usr/lib/x86_64-linux-gnu/obs-plugins/obs-emeet-control.so"

cat > "$STAGE/DEBIAN/control" <<EOF
Package: $PKG
Version: $VERSION
Section: video
Priority: optional
Architecture: amd64
Maintainer: Philippe Demartin <phdemartin@gmail.com>
Depends: obs-studio (>= 30.0), libopencv-core410 | libopencv-core4.6t64 | libopencv-core4.6, libopencv-imgproc410 | libopencv-imgproc4.6t64 | libopencv-imgproc4.6, libopencv-imgcodecs410 | libopencv-imgcodecs4.6t64 | libopencv-imgcodecs4.6, libopencv-objdetect410 | libopencv-objdetect4.6t64 | libopencv-objdetect4.6, opencv-data
Description: Native OBS Studio filter plugin for EMEET SmartCam S800 & UVC webcams
 Provides hardware V4L2/UVC controls (Zoom/FOV 73-40 deg, PDAF+ToF autofocus,
 exposure, white balance, anti-flicker), OpenCV face auto-framing, software
 HDR shadow lift, 3D LUTs (.png/.cube), and flip directly inside OBS Studio
 without needing a separate virtual camera.
EOF

chmod 755 "$STAGE/DEBIAN"
dpkg-deb --build --root-owner-group "$STAGE" "$OUT_DEB"

# Also package portable OBS user-plugin tarball
TAR_STAGE="$REPO_DIR/dist/tar_stage"
rm -rf "$TAR_STAGE"
mkdir -p "$TAR_STAGE/obs-emeet-control/bin/64bit"
cp "$REPO_DIR/build/obs-emeet-control.so" "$TAR_STAGE/obs-emeet-control/bin/64bit/obs-emeet-control.so"
tar -C "$TAR_STAGE" -czf "$OUT_TAR" obs-emeet-control

echo "Paquet .deb : $OUT_DEB"
echo "Archive OBS : $OUT_TAR"
