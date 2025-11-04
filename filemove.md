Procedure: Copying Files to MITS Altair 88-DCDD Disk Format
This procedure details how to copy files from a large (≈1MB) CP/M Double-Density source disk to a smaller (330KB) MITS Altair 88-DCDD target disk using a device emulator for formatting and the PC AltairZ80 simulator for copying.

Phase 1: Formatting the Target Disk (On the SD Card Device)
The goal of this phase is to use the device's native format utility (AFORMAT) to correctly initialize the 330KB Altair 88-DCDD disk image and save it to the SD card.

Stop CPU and Enter Disk Menu:

Ensure the CPU is STOPped.

Toggle UNPROTECT to enter the main configuration menu.

Navigate to the Disk Management Menu (2).

Load Source (A:):

Use [L] Load Disk to load your 1MB source disk image to Drive 0 (A:).

Create Blank Target (B:):

Use [M] Mount Blank to create a new, blank disk buffer for Drive 1 (B:). (This buffer will use the 337,568-byte Altair geometry).

Exit Menu and Boot CP/M:

Exit the menus and restart the CPU to boot into the A> prompt.

Format the Target:

Run the specific formatting utility on the target disk (B:).

Code snippet
A>AFORMAT B:
Verify and Return to Menu:

Run STAT B: to ensure the disk is recognized and formatted.

To return to the emulator menu:

Toggle STOP (to stop the CPU).

Toggle UNPROTECT (to bring up the menu).

Save the Formatted Disk:

In the Disk Management Menu (2), use [S] Save Virtual on Drive 1 (B:). Provide a filename (e.g., /TARGET_ALTAIR.DSK) to write the formatted image to the SD card.

Transfer File:

Shut down the device and move the /TARGET_ALTAIR.DSK file from the SD card to your PC.

Phase 2: Copying Files (On the PC AltairZ80 Simulator)
This phase uses the PC's SIMH environment and the CP/M PIP utility to copy the files, managing the space constraint.

Start Simulator and Attach Disks:

Launch the PC's altairz80 simulator. At the sim> prompt, attach both disk images:

Code snippet
sim> ATTACH DSKA source_dd.dsk
sim> ATTACH DSKB target_altair.dsk
sim> BOOT CP/M
Size Check and File Selection:

Determine Target Capacity: Use A>STAT B: to confirm the maximum usable 128-byte records available on the target disk (B:).

Select Files: Use A>STAT FILENAME.COM on the source disk (A:) to manually select files whose total size does not exceed the record capacity of B:.

Copy Files with PIP:

Use the PIP utility to transfer the selected files from the A: drive (source) to the B: drive (target).

Syntax: PIP B:FILENAME.COM=A:FILENAME.COM

Code snippet
A>PIP B:PROGRAM1.COM=A:PROGRAM1.COM
A>PIP B:DATA.TXT=A:DATA.TXT
⚠️ Warning: The process will fail with Disk Full if the total file size exceeds the target's capacity.

Final Exit and Save:

Verify Files: Check the target disk's directory: A>DIR B:

Exit and Detach: Return to the SIMH prompt and detach the disks to save the files copied to the target_altair.dsk image.

Code snippet
A>^E (or the SIMH escape sequence)
sim> DETACH DSKA
sim> DETACH DSKB
sim> QUIT
The /TARGET_ALTAIR.DSK file now contains the copied files in the smaller MITS Altair 88-DCDD format.
