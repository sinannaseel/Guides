# Installing Ubuntu Server (ARM) on VMware Fusion

## Step 1: Download Ubuntu Server for ARM
- Go to [Ubuntu Server ARM Download](https://ubuntu.com/download/server/arm).
- Download **Ubuntu 24.04 ARM64 ISO**.

## Step 2: Create a New VM in VMware Fusion
- Open **VMware Fusion** → Click **Create a new virtual machine**.
- Select **Install from disc or image**.
- Choose the downloaded **Ubuntu Server ARM64 ISO**.
- Set **OS Type**:
  - If prompted, select **Other > Other 64-bit ARM**.

## Step 3: Configure VM Settings

you might not be allowed to configure settings and go on default, although try changing after installation

-------- Stuck at this stage ------------

- **CPU & RAM** → Allocate at least **2 CPUs, 4GB RAM**.
- **Storage** → Set at least **20GB disk space**.
-   might need to extend to 80 GB {fo obvious reasons }
-   go to Virtual Machine ---> settings.. ---> harddisk
-   change to as needed and also tick the "pre-allocate disk space"
- **Network** → Use **NAT** for internet access.

## Step 4: Install Ubuntu Server
- Start the VM and follow the **Ubuntu Server installation** process.
- Set up a **user account** and install **SSH** for remote access.

## Step 5: (Optional) Install Desktop GUI
To install a **GUI after setup**, run:

### Ubuntu Desktop (Full GUI):
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ubuntu-desktop -y
reboot
```

### Lightweight Xfce Desktop:
```bash
sudo apt install xubuntu-desktop -y
```

This avoids using VMware Fusion.

## Step 7: Important Notes
- **VMware Fusion (Free for Personal Use)**:
  - Fusion **Player** supports Ubuntu ARM VMs.
  - **Fusion Pro** needs to be updated for **Apple Silicon support**.
- **No Official Ubuntu Desktop ARM ISO**:
  - Ubuntu Desktop ARM is **not available** officially for VMware.
  - **Install Ubuntu Server ARM first**, then add a desktop manually.

## Step 8: Conclusion
✅ **Ubuntu Server ARM works on VMware Fusion (M1/M2/M3 Macs)**.
✅ **For GUI**, install **ubuntu-desktop** after setup.
✅ **For CLI-only use**, **Multipass** is a simpler alternative.

---
*Follow this guide to set up Ubuntu ARM quickly on macOS!*

