#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) HOST_OS="windows" ;;
    Linux*)               HOST_OS="linux" ;;
    *) echo "Unsupported operating system: $(uname -s)" >&2; exit 2 ;;
esac

if [[ "$HOST_OS" == "windows" ]]; then
    PICO_TOOLS_DIR="${PICO_TOOLS_DIR:-${USERPROFILE}/.pico-sdk}"
    OPENOCD="${OPENOCD:-${PICO_TOOLS_DIR}/openocd/0.12.0+dev/openocd.exe}"
    OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-${PICO_TOOLS_DIR}/openocd/0.12.0+dev/scripts}"
else
    OPENOCD="${OPENOCD:-$(command -v openocd || true)}"
    OPENOCD_SCRIPTS="${OPENOCD_SCRIPTS:-/usr/share/openocd/scripts}"
fi
MODEL_SERIAL="${MODEL_PROBE_SERIAL:-}"
CTRL_SERIAL="${CTRL_PROBE_SERIAL:-}"

usage() {
    cat <<'EOF'
Usage:
  utils/flash_boards.sh --list-probes
    utils/flash_boards.sh [--build-dir DIR] [--model-serial SERIAL] [--ctrl-serial SERIAL]

Serials may also be supplied through MODEL_PROBE_SERIAL and CTRL_PROBE_SERIAL.
The build directory defaults to the root build directory. The script rebuilds
both firmware images, flashes and verifies each board, resumes execution, and
checks that both RP2350 cores report running.
EOF
}

list_probes() {
    if [[ "$HOST_OS" == "windows" ]]; then
        powershell.exe -NoProfile -Command '
            Get-CimInstance Win32_PnPEntity |
                Where-Object { $_.PNPDeviceID -match "^USB\\VID_2E8A&PID_000C\\" } |
                ForEach-Object { ($_.PNPDeviceID -split "\\")[-1] } |
                Sort-Object -Unique
        ' | tr -d '\r' | sed '/^$/d'
    else
        for device in /sys/bus/usb/devices/*; do
            [[ -r "$device/idVendor" && -r "$device/idProduct" && -r "$device/serial" ]] || continue
            [[ "$(<"$device/idVendor")" == "2e8a" && "$(<"$device/idProduct")" == "000c" ]] || continue
            cat "$device/serial"
        done | sort -u
    fi
}

windows_path() {
    if [[ "$HOST_OS" == "windows" ]] && command -v cygpath >/dev/null 2>&1; then
        cygpath -m "$1"
    else
        printf '%s\n' "$1"
    fi
}

flash_board() {
    local label="$1"
    local serial="$2"
    local elf
    local output

    elf="$(windows_path "$3")"
    printf '\nFlashing %s with %s via CMSIS-DAP %s\n' "$label" "$elf" "$serial"

    if ! output=$("$OPENOCD" \
        -s "$OPENOCD_SCRIPTS" \
        -f interface/cmsis-dap.cfg \
        -c "adapter serial $serial" \
        -f target/rp2350.cfg \
        -c "adapter speed 5000; program \"$elf\" verify; reset halt; resume; echo ${label}_cm0=[rp2350.cm0 curstate]; echo ${label}_cm1=[rp2350.cm1 curstate]; shutdown" \
        2>&1); then
        printf '%s\n' "$output" >&2
        return 1
    fi

    printf '%s\n' "$output"
    grep -Fq '** Verified OK **' <<<"$output"
    grep -Fq "${label}_cm0=running" <<<"$output"
    grep -Fq "${label}_cm1=running" <<<"$output"
}

while (($# > 0)); do
    case "$1" in
        --list-probes)
            list_probes
            exit 0
            ;;
        --build-dir)
            [[ $# -ge 2 ]] || { echo "Missing value for --build-dir" >&2; exit 2; }
            BUILD_DIR="$2"
            shift 2
            ;;
        --model-serial)
            [[ $# -ge 2 ]] || { echo "Missing value for --model-serial" >&2; exit 2; }
            MODEL_SERIAL="$2"
            shift 2
            ;;
        --ctrl-serial)
            [[ $# -ge 2 ]] || { echo "Missing value for --ctrl-serial" >&2; exit 2; }
            CTRL_SERIAL="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

[[ -n "$MODEL_SERIAL" ]] || { echo "Set MODEL_PROBE_SERIAL or use --model-serial." >&2; exit 2; }
[[ -n "$CTRL_SERIAL" ]] || { echo "Set CTRL_PROBE_SERIAL or use --ctrl-serial." >&2; exit 2; }
[[ "$MODEL_SERIAL" != "$CTRL_SERIAL" ]] || { echo "Model and controller probe serials must differ." >&2; exit 2; }
[[ -x "$OPENOCD" ]] || { echo "OpenOCD not found: $OPENOCD" >&2; exit 2; }
[[ -f "$BUILD_DIR/CMakeCache.txt" ]] || {
    echo "Existing root build directory is not configured: $BUILD_DIR" >&2
    exit 2
}

cmake --build "$BUILD_DIR" --target HIL_Model HIL_Ctrl -j

flash_board MODEL "$MODEL_SERIAL" "$BUILD_DIR/HIL_Model.elf"
flash_board CTRL "$CTRL_SERIAL" "$BUILD_DIR/HIL_Ctrl.elf"

printf '\nBoth firmware images verified; all RP2350 cores are running.\n'