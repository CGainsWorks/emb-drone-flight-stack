#!/usr/bin/env bash
# Static sanity checks on the linked firmware image.
#
# The board is unassembled, so this cannot prove the firmware runs. What it can
# prove is that the image is correctly formed: vector table at the reset
# address, a plausible initial stack pointer, thumb-encoded vectors, and the
# USART1 handler actually wired into its slot rather than silently falling
# through to the default trap.
#
# Vector words are read straight out of the raw .bin, which begins at
# 0x08000000, so the checks do not depend on objdump's line formatting.
set -euo pipefail

cd "$(dirname "$0")"
ELF=build-arm/fc_firmware.elf
BIN=build-arm/fc_firmware.bin

if [ ! -f "$ELF" ] || [ ! -f "$BIN" ]; then
  echo "no build output - run ./build_firmware.sh first" >&2
  exit 1
fi

fail=0

check() {
  if [ "$2" = "$3" ]; then
    printf '  PASS  %-46s %s\n' "$1" "$2"
  else
    printf '  FAIL  %-46s got %s, expected %s\n' "$1" "$2" "$3"
    fail=1
  fi
}

# 32-bit little-endian word at byte offset $1 of the image.
read_word() {
  printf '0x%08x' "$(( 0x$(od -An -tx4 -j "$1" -N4 "$BIN" | tr -d ' \n') ))"
}

# Address of symbol $1, thumb-encoded (bit 0 set), as functions are called.
sym_thumb() {
  local addr
  addr=$(arm-none-eabi-nm "$ELF" | awk -v s=" $1\$" '$0 ~ s {print $1; exit}')
  if [ -z "$addr" ]; then
    echo "SYMBOL_NOT_FOUND"
    return
  fi
  printf '0x%08x' "$(( 0x$addr | 1 ))"
}

reset_h=$(sym_thumb Reset_Handler)
default_h=$(sym_thumb Default_Handler)
usart1_h=$(sym_thumb USART1_IRQHandler)

echo "=== image layout ==="

isr_addr=$(arm-none-eabi-readelf -S "$ELF" \
           | awk '/\.isr_vector/ {print $5; exit}')
check "vector table at the reset address" "0x$isr_addr" "0x08000000"

entry=$(arm-none-eabi-readelf -h "$ELF" | awk '/Entry point/ {print $NF}')
check "entry point is Reset_Handler" "$(printf '0x%08x' "$((entry))")" "$reset_h"

# Word 0: initial stack pointer, loaded by the core straight out of reset.
check "initial SP = top of 128 KB SRAM" "$(read_word 0)" "0x20020000"

# Word 1: reset vector.
check "reset vector = Reset_Handler" "$(read_word 4)" "$reset_h"

echo
echo "=== interrupt wiring ==="

# 16 system exception slots come first, so external IRQ n sits at word 16+n.
# USART1 is external IRQ 37 -> word 53 -> byte offset 212.
usart1_slot=$(read_word 212)
check "vector slot 53 = USART1_IRQHandler" "$usart1_slot" "$usart1_h"

if [ "$usart1_slot" = "$default_h" ]; then
  echo "  FAIL  USART1 slot fell through to Default_Handler"
  fail=1
fi

# An unused neighbour should still be the trap, confirming the index is right
# and the table is not simply full of the real handler.
check "vector slot 54 (USART2, unused) = trap" "$(read_word 216)" "$default_h"

echo
echo "=== toolchain / ABI ==="
abi=$(arm-none-eabi-readelf -A "$ELF" | grep -c 'Tag_ABI_VFP_args: VFP registers' || true)
check "hard-float ABI (FPU in use)" "$abi" "1"

echo
echo "=== size ==="
arm-none-eabi-size "$ELF"
echo
echo "raw image: $(stat -c %s "$BIN") bytes"

echo
if [ "$fail" -eq 0 ]; then
  echo "All image checks passed."
else
  echo "Image checks FAILED." >&2
  exit 1
fi
