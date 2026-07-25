# Mainline Linux for the OnePlus 15 (kaanapali)

Linux for the OnePlus 15 (CPH2749, Qualcomm SM8850 "Kaanapali", board
name "infiniti"), aimed at running as close to mainline as possible.

Branch layout:

- `development` — the base of this work: FantomTchi7's
  kaanapali-mainline-linux tree (mainline + Qualcomm's SM8850 groundwork
  + his OnePlus 15 bring-up), carried with full history and authorship.
  Treated as read-only here.
- `oneplus-15` — this project's device work on top: panel driver with
  cmd-mode DSC 1.2, DPU/DSI fixes, S3910 touch, WCN7860 Wi-Fi/Bluetooth,
  SoCCP fuel gauge, and the board devicetree. Default branch.

What works today on device: display at correct geometry and color,
touch, Wi-Fi, Bluetooth (keyboard tested), real battery telemetry, ADSP
boot, ramoops. In progress: suspend/resume, audio, camera, modem, GPS.

Credits: this tree exists because of FantomTchi7's kaanapali mainline
work, which it builds on directly. The touch driver is by Frieder
Hannenheim and Caleb Connolly (postmarketOS); WCN7860 patches by Teguh
Sobirin and Gianni Spadoni are carried with their authorship intact.

Licensing is the kernel's own (GPL-2.0-only with per-file SPDX). The
companion bootloader lives at `infiniti-mainline/u-boot`.
See CONTRIBUTING.md before sending changes.
