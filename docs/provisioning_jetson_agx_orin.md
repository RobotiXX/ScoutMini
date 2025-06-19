# Provisioning NVIDIA Jetson AGX Orin with JetPack 6.X
Date:  2025-06-19

---

## 1. Issues

Flashing a Jetson AGX Orin with a clean **JetPack 6.X** via **NVIDIA SDK Manager** leaves out several kernel modules that were available by default on JetPack 5.X.

| Sub-system | Symptom |
|------------|---------|
| **CAN interface** | `sudo modprobe gs_usb` →<br>`FATAL: Module gs_usb not found in /lib/modules/5.15.136-tegra` |
| **USB-to-Serial (CH340)** | CH340 adapters are not enumerated as `/dev/ttyUSB*`, breaking udev rules and serial communication |
| **Joystick & Userinput** | Input and joy commands from PS4 DualShock controller cannot be accessed with `ds4drv`|


---

## 2. Solution — Kernel Customization

The fix is to add the missing kernel options to the defconfig, rebuild, and re-flash [[1]](#1).

### 2.1 Sync kernel sources 
See official guide [[5]](#5) if needed.


```bash
cd <install-path>/source
./source_sync.sh -k -t jetson_36.4.3      # release tag
# JetPack -> release tag pairs: 6.1 -> 36.4, 6.2 -> 36.4.3 
```

`<install-path>` example:  

```text
~/nvidia/nvidia_sdk/Jetpack_6.2_Linux_JETSON_AGX_ORIN_TARGETS
# Default path after flashing first time with corresponding JetPack version
```

### 2.2 Enable missing modules

Open:

```text
<install-path>/source/kernel/kernel-jammy-src/arch/arm64/configs/defconfig
```

Add (or verify) these lines **at the end**:

```text
CONFIG_CAN_GS_USB=y
CONFIG_USB_SERIAL_CH341=y
CONFIG_INPUT_UINPUT=y
CONFIG_INPUT_JOYDEV=y
```
Note: use `m` option instead of `y` if loadable modules are preferred

### 2.3 Install build dependencies

```bash
sudo apt install flex bison libssl-dev
```

### 2.4 Download and install Toolchain
Refer to [[6]](#6)

### 2.5 Re-build Jetson image

```bash
cd <install-path>/Linux_for_Tegra/source

# Disable RT kernel, then build & install
./generic_rt_build.sh "disable"
export CROSS_COMPILE=<toolchain-path>/bin/aarch64-buildroot-linux-gnu-
make -C kernel
export INSTALL_MOD_PATH=<install-path>/Linux_for_Tegra/rootfs/
sudo -E make install -C kernel

# Copy the Image into the flash directory
cp kernel/kernel-jammy-src/arch/arm64/boot/Image    <install-path>/Linux_for_Tegra/kernel/Image
```

Tool-chain path example:  

```text
~/l4t-gcc/aarch64-glibc-stable-2022.08-1
```

### 2.5 Re-flash with SDK Manager

Follow the usual SDK Manager steps to flash the rebuilt kernel onto the Jetson.

### 2.6 Post-flash fix-ups

```bash
sudo apt update && sudo apt upgrade
sudo apt remove brltty        # resolves /dev/ttyUSB* issues
```

---

## 3 · Notes

* Wi-Fi and Ethernet may appear missing immediately after flashing—`apt upgrade` restores them.  
* If you re-enable the **RT kernel**, the CH340 and gs_usb issues may persist alongside wifi connection issues.

---

## 4 · References

<a id="1">[1]</a> <https://forums.developer.nvidia.com/t/missing-gs-usb-kernel-module-for-jetpack-6/275287/5> 

<a id="2">[2]</a> <https://forums.developer.nvidia.com/t/kernel-update-problem/284819/7>

<a id="3">[3]</a> <https://nvidia-jetson.piveral.com/jetson-orin-nano/orin-nano-wont-detect-arduino-dev-ttyusb-or-dev-ttyacm/>

<a id="4">[4]</a> <https://askubuntu.com/questions/1403705/dev-ttyusb0-not-present-in-ubuntu-22-04>

<a id="5">[5]</a> <https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/SD/Kernel/KernelCustomization.html#to-sync-the-kernel-sources-with-git>

<a id="6">[6]</a> <https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/AT/JetsonLinuxToolchain.html#at-jetsonlinuxtoolchain>
