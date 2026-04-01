#!/bin/bash

MGR_CMD="nrfutil mcu-manager serial --serial-port /dev/ttyACM0"

$MGR_CMD image-upload --firmware build/open-earable-2/zephyr/zephyr.signed.bin
$MGR_CMD image-upload --firmware build/signed_by_mcuboot_and_b0_ipc_radio.bin

# Get hashes from image list
IMAGE_LIST=$($MGR_CMD image-list)
HASH_APP=$(echo "$IMAGE_LIST" | grep "hash: " | sed -n 2p | cut -d" " -f6)
HASH_NET=$(echo "$IMAGE_LIST" | grep "hash: " | sed -n 3p | cut -d" " -f6)

# Test both images with their hashes
$MGR_CMD image-test --hash $HASH_APP
$MGR_CMD image-test --hash $HASH_NET

echo "Resetting device..."

$MGR_CMD reset