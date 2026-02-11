#!/bin/bash
# bind_uio.sh — Unbind RTL8168H from kernel driver and bind to UIO
#
# Usage: sudo ./bind_uio.sh [bind|unbind]
# Default action: bind
#
# Uses driver_override (per-device) instead of new_id (per-vendor:device)
# to avoid grabbing both NICs.

set -e

PCI_BDF="0000:01:00.0"

action="${1:-bind}"

case "$action" in
bind)
    echo "=== Binding $PCI_BDF to uio_pci_generic ==="

    modprobe uio_pci_generic

    current=$(basename "$(readlink -f /sys/bus/pci/devices/$PCI_BDF/driver 2>/dev/null)" 2>/dev/null || echo "none")
    echo "Current driver: $current"

    if [ "$current" != "uio_pci_generic" ]; then
        if [ "$current" != "none" ]; then
            echo "Unbinding from $current..."
            echo "$PCI_BDF" > "/sys/bus/pci/drivers/$current/unbind" 2>/dev/null || true
            sleep 0.5
        fi

        # driver_override: only this BDF, not all 10ec:8168 devices
        echo "uio_pci_generic" > /sys/bus/pci/devices/$PCI_BDF/driver_override
        echo "$PCI_BDF" > /sys/bus/pci/drivers/uio_pci_generic/bind
        sleep 0.5
    fi

    # Verify
    if ls /sys/bus/pci/devices/$PCI_BDF/uio/uio* 1>/dev/null 2>&1; then
        uio_dev=$(ls -d /sys/bus/pci/devices/$PCI_BDF/uio/uio* | head -1)
        echo "OK: $(basename $uio_dev) -> /dev/$(basename $uio_dev)"
    else
        echo "ERROR: UIO device not found"
        exit 1
    fi

    # Hugepages
    hp=$(cat /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages)
    if [ "$hp" -lt 4 ]; then
        echo "Allocating hugepages (4 x 2MB)..."
        echo 4 > /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages
    fi
    echo "Hugepages: $(cat /sys/kernel/mm/hugepages/hugepages-2048kB/free_hugepages)/$(cat /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages)"
    ;;

unbind)
    echo "=== Unbinding $PCI_BDF from UIO, rebinding to kernel driver ==="

    current=$(basename "$(readlink -f /sys/bus/pci/devices/$PCI_BDF/driver 2>/dev/null)" 2>/dev/null || echo "none")
    if [ "$current" = "uio_pci_generic" ]; then
        echo "$PCI_BDF" > /sys/bus/pci/drivers/uio_pci_generic/unbind
        sleep 0.5
    fi

    # Clear override so kernel driver can claim it
    echo "" > /sys/bus/pci/devices/$PCI_BDF/driver_override
    # Also clean up any leftover new_id from old script
    echo "10ec 8168" > /sys/bus/pci/drivers/uio_pci_generic/remove_id 2>/dev/null || true
    # Re-probe
    echo "$PCI_BDF" > /sys/bus/pci/drivers_probe

    # Free hugepages if all are unused
    free_hp=$(cat /sys/kernel/mm/hugepages/hugepages-2048kB/free_hugepages)
    nr_hp=$(cat /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages)
    if [ "$free_hp" = "$nr_hp" ] && [ "$nr_hp" != "0" ]; then
        echo "Freeing hugepages ($nr_hp x 2MB)..."
        echo 0 > /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages
    fi

    sleep 0.5
    new_drv=$(basename "$(readlink -f /sys/bus/pci/devices/$PCI_BDF/driver 2>/dev/null)" 2>/dev/null || echo "none")
    echo "Now bound to: $new_drv"
    ;;

*)
    echo "Usage: $0 [bind|unbind]"
    exit 1
    ;;
esac
