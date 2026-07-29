#!/bin/bash
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Load a Croc ELF onto a running Genesys2 FPGA over JTAG and attach a serial
# terminal to view its UART output.
#
# Usage:
#   ./run_sw.sh <path/to/prog.elf> [serial-port] [baud]
#
# Example:
#   ./run_sw.sh ../sw/bin/helloworld.elf
#   ./run_sw.sh ../sw/bin/helloworld.elf /dev/ttyUSB0 115200

set -e
set -u

ELF=${1:?"Usage: $0 <path/to/prog.elf> [serial-port] [baud]"}
PORT=${2:-/dev/ttyUSB0}
BAUD=${3:-115200}

XILINX_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GDB=${GDB:-"oseda -2026.04 riscv64-unknown-elf-gdb"}
OPENOCD=${OPENOCD:-openocd}
OPENOCD_CFG="${XILINX_ROOT}/scripts/openocd.genesys2.tcl"
GDB_SCRIPT="${XILINX_ROOT}/scripts/load_run.gdb"

OPENOCD_LOG=$(mktemp)
OPENOCD_PID=""
CAT_PID=""

cleanup() {
    if [ -n "$CAT_PID" ] && kill -0 "$CAT_PID" 2>/dev/null; then
        kill "$CAT_PID" 2>/dev/null || true
        wait "$CAT_PID" 2>/dev/null || true
    fi
    if [ -n "$OPENOCD_PID" ] && kill -0 "$OPENOCD_PID" 2>/dev/null; then
        echo "[INFO] Stopping OpenOCD (pid $OPENOCD_PID)"
        kill "$OPENOCD_PID" 2>/dev/null || true
        wait "$OPENOCD_PID" 2>/dev/null || true
    fi
    rm -f "$OPENOCD_LOG"
}
trap cleanup EXIT INT TERM

# Open and start reading the serial port *before* we load/resume the core, so
# nothing printed right after wake-up is lost while no one was listening yet.
# 'cat' (unlike picocom) only writes to stdout, so it can safely run in the
# background without needing the controlling terminal for input.
echo "[INFO] Opening ${PORT} at ${BAUD} baud and starting capture..."
stty -F "$PORT" "$BAUD" cs8 -cstopb -parenb raw -echo
cat "$PORT" &
CAT_PID=$!

echo "[INFO] Starting OpenOCD..."
"$OPENOCD" -f "$OPENOCD_CFG" >"$OPENOCD_LOG" 2>&1 &
OPENOCD_PID=$!

echo "[INFO] Waiting for OpenOCD to be ready..."
for _ in $(seq 1 50); do
    grep -q "Ready for Remote Connections" "$OPENOCD_LOG" && break
    kill -0 "$OPENOCD_PID" 2>/dev/null || {
        echo "[ERROR] OpenOCD exited early:"
        cat "$OPENOCD_LOG"
        exit 1
    }
    sleep 0.2
done
grep -q "Ready for Remote Connections" "$OPENOCD_LOG" || {
    echo "[ERROR] OpenOCD did not become ready in time:"
    cat "$OPENOCD_LOG"
    exit 1
}

echo "[INFO] Loading ${ELF} and starting execution..."
# Print this before invoking gdb, not after: the core starts printing over
# UART almost immediately once gdb issues 'monitor resume', well before the
# gdb command itself (which still has to detach/quit) returns to the script.
echo "[INFO] Streaming UART output from ${PORT} (Ctrl-C to quit)..."
$GDB -batch -x "$GDB_SCRIPT" "$ELF"

wait "$CAT_PID"
