#!/usr/bin/env bash
#
# Reproducible build of the RTL8720DN (AmebaD) eRPC device firmware -- issue #20, N1.
#
# The STM32 side of this board talks to the WiFi module over eRPC; the module runs
# Seeed's `seeed-ambd-firmware`.  This script rebuilds that firmware from pinned
# sources so we can then patch it (N2 bounded handlers, N3 worker dispatch) and flash
# the result with the on-device flasher from issue #19 (`wifi imgload` +
# `wifi flashwrite`).  Nothing here touches the STM32 firmware.
#
# Everything is pinned and verified before a single byte is compiled, because the
# artifact ends up in the flash of the only surviving board:
#
#   - firmware source   Seeed-Studio/seeed-ambd-firmware @ $FW_COMMIT  (branch Wio-Lite-AI)
#   - Arduino core      Seeed-Studio/ArduinoCore-ambd    @ $CORE_COMMIT
#                       (NOT the board-manager `realtek:AmebaD@3.0.5` tarball -- that one
#                        lacks the lwIP patches the firmware needs; see README.md)
#   - toolchain/tools   realtek:ameba_d_asdk_toolchain@1.0.1, realtek:ameba_d_tools@1.0.4
#                       (from the board index; these DO come from the tarball)
#
# The arduino-cli data directory is kept inside _ref/ambd/ so the user's own
# ~/.arduino15 (esp32, rp2040, ...) is never touched.
#
# Usage:
#   ./build.sh setup     one-time: fetch sources, install core+tools, swap in the fork
#   ./build.sh build     verify pins -> apply patches/ -> compile -> collect -> gate  (default)
#   ./build.sh gate      re-run the static gates on the artifact already in out/
#   ./build.sh clean     remove out/
#
set -euo pipefail

# ---------------------------------------------------------------- pinned versions
FW_REPO=https://github.com/Seeed-Studio/seeed-ambd-firmware.git
FW_BRANCH=Wio-Lite-AI
FW_COMMIT=fc9526d5bf68ad6756e47e43520dc8b49d924492      # 2021-07-23, "a branch fo Wio Lite AI"

CORE_REPO=https://github.com/Seeed-Studio/ArduinoCore-ambd.git
CORE_COMMIT=f81bca75e433e35b2d27cadd178596888940fb5a    # last functional change 2582acf (2021-03-19)
CORE_VERSION=3.0.5                                      # directory name arduino-cli expects

BOARD_INDEX=https://files.seeedstudio.com/arduino/package_realtek.com_amebad_index.json
FQBN=realtek:AmebaD:ameba_rtl8721d
TOOLCHAIN_REL=ameba_d_asdk_toolchain/1.0.1
TOOLS_REL=ameba_d_tools/1.0.4

# ---------------------------------------------------------------- paths
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../.." && pwd)
REF=$ROOT/_ref/ambd
FW_SRC=$ROOT/_ref/seeed-ambd-firmware
CORE_SRC=$REF/ArduinoCore-ambd
DATA=$REF/arduino15
USERDIR=$REF/arduino_user
PKG=$DATA/packages/realtek
CORE=$PKG/hardware/AmebaD/$CORE_VERSION
TOOLS=$PKG/tools/$TOOLS_REL
STOCK=$REF/board2-stock/rtl8720_000000_200000.bin
FLASHLOADER=$REF/imgtool_flashloader_amebad.bin

OUT=$HERE/out
SKETCH=$OUT/sketch/seeed-ambd-firmware                  # dir name must match the .ino name
BUILDDIR=$OUT/build
CORE_STAMP=$CORE/.wio-core-commit

export ARDUINO_DIRECTORIES_DATA=$DATA
export ARDUINO_DIRECTORIES_USER=$USERDIR

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31mERROR:\033[0m %s\n' "$*" >&2; exit 1; }
warn() { printf '\033[1;33mWARN:\033[0m %s\n' "$*" >&2; }

# The fork's distinguishing feature is the patched lwIP the firmware compiles against;
# the board-manager tarball does not have it.  Used to tell the two platforms apart.
is_fork() {
	grep -q 'struct rpc_tcp_pcb' \
		"$1/system/libameba/sdk/component/common/network/lwip/lwip_v2.0.2/src/include/lwip/tcp.h" \
		2>/dev/null
}

# setup and build both mutate shared state -- the arduino-cli data directory, and the
# vendor postbuild step, which ignores --build-path and drops its output straight into
# the shared tools directory.  Two concurrent runs would interleave their artifacts, so
# serialise them instead of trying to make the vendor tool behave.
take_lock() {
	mkdir -p "$REF"
	exec 9>"$REF/.build.lock"
	flock -n 9 || {
		say "another fw/rtl8720 build is running -- waiting for the lock"
		flock 9
	}
}

# ---------------------------------------------------------------- setup
do_setup() {
	take_lock
	command -v arduino-cli >/dev/null || die "arduino-cli not on PATH (see README.md)"
	say "arduino-cli $(arduino-cli version | head -1)"

	if [ ! -d "$FW_SRC/.git" ]; then
		say "cloning firmware source -> $FW_SRC"
		git clone --branch "$FW_BRANCH" "$FW_REPO" "$FW_SRC"
	fi
	git -C "$FW_SRC" cat-file -e "$FW_COMMIT^{commit}" 2>/dev/null || {
		say "fetching $FW_COMMIT"
		git -C "$FW_SRC" fetch origin "$FW_BRANCH"
	}

	if [ ! -d "$CORE_SRC/.git" ]; then
		say "cloning Arduino core fork -> $CORE_SRC"
		git clone "$CORE_REPO" "$CORE_SRC"
	fi
	git -C "$CORE_SRC" cat-file -e "$CORE_COMMIT^{commit}" 2>/dev/null || {
		say "fetching $CORE_COMMIT"
		git -C "$CORE_SRC" fetch origin
	}

	# The board-manager package supplies the toolchain and ameba_d_tools (postbuild
	# image builder + the prebuilt km0/km4 boot images).  We keep those and throw away
	# only the `hardware/AmebaD/3.0.5` platform, which the fork replaces.
	if [ ! -d "$PKG/tools/$TOOLCHAIN_REL" ] || [ ! -d "$TOOLS" ]; then
		say "installing realtek:AmebaD@$CORE_VERSION (core + toolchain + tools)"
		arduino-cli --additional-urls "$BOARD_INDEX" core update-index
		arduino-cli --additional-urls "$BOARD_INDEX" core install "realtek:AmebaD@$CORE_VERSION"
	fi
	[ -d "$TOOLS" ] || die "ameba_d_tools not installed at $TOOLS"

	# Keep the untouched board-manager platform around for reference/diffing, then put
	# the fork in its place (this is what upstream's README tells you to do).  Only ever
	# stash something that is actually the board-manager platform: on a re-run $CORE
	# already holds the fork, and stashing that would destroy the pristine copy.
	if [ -d "$CORE" ] && ! is_fork "$CORE"; then
		[ -d "$REF/core-bm-$CORE_VERSION" ] &&
			die "$REF/core-bm-$CORE_VERSION already exists -- move it aside first"
		say "stashing the board-manager platform -> $REF/core-bm-$CORE_VERSION"
		mv "$CORE" "$REF/core-bm-$CORE_VERSION"
	fi
	# Stage the new platform beside the old one and swap at the end, so an interrupted
	# run cannot leave a half-extracted tree where arduino-cli expects a platform.  The
	# stamp is written last: without it `verify_pins` refuses to build, so every partial
	# state fails closed.
	say "installing core fork $CORE_COMMIT -> $CORE"
	rm -rf "$CORE.new"
	mkdir -p "$CORE.new"
	git -C "$CORE_SRC" archive "$CORE_COMMIT" | tar -x -C "$CORE.new"
	# arduino-cli wants the platform metadata that came with the tarball.
	[ -f "$REF/core-bm-$CORE_VERSION/installed.json" ] &&
		cp "$REF/core-bm-$CORE_VERSION/installed.json" "$CORE.new/installed.json"
	rm -rf "$CORE"
	mv "$CORE.new" "$CORE"
	echo "$CORE_COMMIT" > "$CORE_STAMP"

	say "setup done"
}

# ---------------------------------------------------------------- verification
verify_pins() {
	command -v arduino-cli >/dev/null || die "arduino-cli not on PATH -- run './build.sh setup'"
	[ -d "$FW_SRC/.git" ]  || die "firmware source missing -- run './build.sh setup'"
	[ -d "$CORE" ]         || die "Arduino core missing -- run './build.sh setup'"
	[ -d "$TOOLS" ]        || die "ameba_d_tools missing -- run './build.sh setup'"

	git -C "$FW_SRC" cat-file -e "$FW_COMMIT^{commit}" 2>/dev/null ||
		die "firmware commit $FW_COMMIT not in $FW_SRC -- run './build.sh setup'"

	# Drift guard: the core is a plain snapshot (no .git), so a stamp file records what
	# it was cut from.  If this does not match, the build is not the pinned one.
	local have
	have=$(cat "$CORE_STAMP" 2>/dev/null || echo "<none>")
	[ "$have" = "$CORE_COMMIT" ] ||
		die "core at $CORE is $have, expected $CORE_COMMIT -- run './build.sh setup'"

	# The fork carries the patched lwIP the firmware compiles against.  Cheap, direct
	# check that we did not silently fall back to the board-manager platform.
	is_fork "$CORE" ||
		die "core lwIP has no 'struct rpc_tcp_pcb' -- this is the unpatched board-manager platform"

	say "firmware  $FW_COMMIT"
	say "core      $CORE_COMMIT"
	say "tools     $(basename "$TOOLS_REL")  toolchain $(basename "$TOOLCHAIN_REL")"
}

# ---------------------------------------------------------------- build
do_build() {
	take_lock
	verify_pins

	# Build from a pristine export of the pinned commit plus patches/, never from the
	# reference checkout itself -- _ref/seeed-ambd-firmware stays an untouched mirror.
	say "exporting $FW_COMMIT -> $SKETCH"
	rm -rf "$OUT/sketch"
	mkdir -p "$SKETCH"
	git -C "$FW_SRC" archive "$FW_COMMIT" | tar -x -C "$SKETCH"

	shopt -s nullglob
	local patches=("$HERE"/patches/*.patch)
	shopt -u nullglob
	if [ ${#patches[@]} -eq 0 ]; then
		say "no patches -- building pristine upstream (N1 baseline)"
	else
		for p in "${patches[@]}"; do
			say "applying $(basename "$p")"
			# The export sits under out/, which is git-ignored *inside this repo*, so a
			# plain `git apply` here discovers the wio-lite-ai repo, sees the target as
			# ignored, and silently no-ops (rc 0, nothing changed) -- an unpatched image
			# that still looks built.  Stop git's repo discovery at out/ with
			# GIT_CEILING_DIRECTORIES so it treats the export as the plain tree it is and
			# actually patches it.
			( cd "$SKETCH" && GIT_CEILING_DIRECTORIES="$OUT" \
				git apply -p1 --whitespace=nowarn "$p" ) ||
				die "patch failed: $p"
			# Never trust a silent success: a reverse-check can only pass if the patch is
			# now present, so it turns the no-op above into a hard failure.
			( cd "$SKETCH" && GIT_CEILING_DIRECTORIES="$OUT" \
				git apply -p1 -R --check "$p" ) ||
				die "patch reported success but did not take (path/repo issue?): $p"
		done
	fi

	# Upstream's arduino-build.sh passes these as build.extra_flags, which REPLACES the
	# board's own extra_flags (-DBOARD_RTL8721D and the USB_* defines).  We reproduce
	# that exactly -- it is how the shipped firmware was built, and the only consumer of
	# BOARD_RTL8721D is the Wire library, which this sketch does not use.
	local inc=""
	for d in easylogger easylogger/inc ble wifi esp_lib erpc erpc_shim mDNS; do
		inc+=" -I$SKETCH/src/$d"
	done

	# Drop the previous artifacts BEFORE compiling.  `./build.sh gate` deliberately
	# validates whatever is in out/, so a failed build must never be able to leave a
	# stale-but-passing image sitting there.
	rm -f "$OUT"/*.bin "$OUT"/application.axf "$OUT"/compile.log
	rm -rf "$BUILDDIR"
	mkdir -p "$BUILDDIR"
	say "compiling ($FQBN)"
	# No pipeline here on purpose: `cmd | tee | grep || true` makes ${PIPESTATUS[0]}
	# report the status of `true`, which silently masks a failed compile.
	local rc=0
	arduino-cli compile --fqbn "$FQBN" \
		--build-path "$BUILDDIR" \
		--build-property build.extra_flags="$inc" \
		"$SKETCH" > "$OUT/compile.log" 2>&1 || rc=$?
	grep -E 'error:|バイト|bytes' "$OUT/compile.log" || true
	[ "$rc" -eq 0 ] || die "compile failed (rc $rc) -- see $OUT/compile.log"

	# The postbuild step (`postbuild_img2_arduino_linux`) runs with its cwd inside the
	# shared tools directory and drops the images there, so collect them out.  It also
	# does `rm -f bsp/image/*.bin`, which deletes the AmebaD flashloader stub that the
	# STM32 build embeds at configure time -- put it back.
	[ -f "$TOOLS/km0_km4_image2.bin" ] || die "postbuild produced no km0_km4_image2.bin"
	cp "$TOOLS/km0_km4_image2.bin" "$TOOLS/km0_image2_all.bin" \
	   "$TOOLS/km4_image2_all.bin" "$TOOLS/application.axf" "$OUT/"
	cp "$TOOLS/bsp/image/km0_boot_all.bin" "$TOOLS/bsp/image/km4_boot_all.bin" "$OUT/"
	if [ ! -f "$TOOLS/bsp/image/imgtool_flashloader_amebad.bin" ] && [ -f "$FLASHLOADER" ]; then
		cp "$FLASHLOADER" "$TOOLS/bsp/image/"
	fi

	say "artifacts in $OUT/"
	do_gate
}

# ---------------------------------------------------------------- gates
do_gate() {
	[ -f "$OUT/km0_km4_image2.bin" ] || die "no $OUT/km0_km4_image2.bin -- run './build.sh build'"
	python3 "$HERE/gate.py" \
		--image "$OUT/km0_km4_image2.bin" \
		--km0-boot "$OUT/km0_boot_all.bin" \
		--km4-boot "$OUT/km4_boot_all.bin" \
		--stock "$STOCK"
}

case "${1:-build}" in
	setup) do_setup ;;
	build) do_build ;;
	gate)  do_gate ;;
	clean) rm -rf "$OUT"; say "removed $OUT" ;;
	*)     die "usage: $0 {setup|build|gate|clean}" ;;
esac
