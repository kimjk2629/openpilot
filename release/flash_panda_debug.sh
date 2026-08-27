#!/usr/bin/env bash
# Force-build panda firmware WITHOUT RELEASE=1, so ALLOW_DEBUG-gated features
# (e.g. Tesla "Alpha Longitudinal" -> tesla_longitudinal in safety_tesla.h,
# which controls whether openpilot may send DAS_control / cruise commands)
# are actually compiled in.
#
# Use this ONLY if `Settings > Software` on the device shows the panda
# firmware version string ending in "-RELEASE" (see panda/SConscript's
# get_version(): f"{builder}-{git}-{build_type}"). If it already ends in
# "-DEBUG", this script won't change anything -- ALLOW_DEBUG is already
# compiled in and the cruise-button issue has a different root cause.
#
# NOTE: This produces panda firmware signed with the DEBUG cert, not the
# RELEASE cert. After reflashing you must enable "Unsafe mode" / allow
# uncertified hardware in the device's developer settings, or the device
# will refuse to run with it.
set -e

cd "$(git rev-parse --show-toplevel)"

echo "[-] Building panda firmware WITHOUT RELEASE=1 (includes -DALLOW_DEBUG)..."
scons -j"$(nproc)" panda/

echo "[-] Flashing connected panda..."
python3 panda/board/flash.py

echo "[-] Done."
echo "[-] Reboot the device, then check Settings > Software again:"
echo "    the panda firmware string should now end in -DEBUG instead of -RELEASE."
echo "[-] Also make sure 'Unsafe mode' (uncertified hardware) is enabled,"
echo "    since this is a debug-signed firmware."
