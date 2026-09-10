# Use a Scout on a home network

The Moorebot Scout has two Wi-Fi modes:

- **Wi-Fi Direct:** the Scout creates `robot_scout_xxxxxx`; the driver uses the
  robot at `10.42.0.1` automatically.
- **Wi-Fi Router:** the Scout joins a trusted home router; the router chooses the
  robot's address and the driver needs that address in `--master`.

Wi-Fi Direct is easiest for a first test. Router mode lets the computer keep its
normal network connection while it controls the robot.

## Configure Wi-Fi Router mode

1. Install the **Moorebot Scout** app on an Android or iOS phone and complete
   the account setup requested by the app.
2. Put the Scout in its charging station, power it on, and use the physical
   Wi-Fi mode button to select Wi-Fi Direct mode.
3. Join `robot_scout_xxxxxx` from the phone. The factory password is
   `r0123456`; replace it when the app requests a new device password.
4. In the app, select the home Wi-Fi network and enter its password. The Scout
   switches to **Wi-Fi Router mode** after it connects successfully.
5. Reconnect the phone and the computer to the same home network.

The router-mode indicator stays on while connected and blinks when the Scout
cannot reach the router. The procedure comes from Moorebot's [official Scout
setup instructions](https://www.moorebot.com/products/scout-power-pack-scout-4-accessories-track-set-wand-laser-bumper).

Do not edit `/etc/hostapd/hostapd.conf` for this. That file configures the
hotspot created by Wi-Fi Direct mode, not the home-router connection.

## Find the Scout's address

Open the home router's connected-device or DHCP-client page. Look for
`linaro-alip`, `robot_scout_xxxxxx`, or an unnamed device with the Scout's MAC
address. If the phone app has recently contacted the robot, `arp -a` on the
computer may also show the address next to its MAC.

For the supplied classroom fleet, use this map:

| Robot | MAC address | Robot | MAC address |
|---|---|---|---|
| Bombur | `d4:9c:dd:e9:ea:a0` | Bofur | `d4:9c:dd:ea:dc:3a` |
| Bifur | `d4:9c:dd:ea:fd:a4` | Glóin | `d4:9c:dd:ea:3b:16` |
| Thráin | `d4:9c:dd:e9:ea:60` | Thrór | `d4:9c:dd:ea:dc:a2` |
| Thorin | `b8:2d:28:56:92:0e` | Balin | `b8:2d:28:56:87:da` |
| Dwalin | `b8:2d:28:56:91:60` | Fíli | `d4:9c:dd:ea:fd:9c` |
| Kíli | `d4:9c:dd:eb:0c:f6` | Dori | `d4:9c:dd:e9:fa:2e` |
| Nori | `d4:9c:dd:ea:dc:8c` | Ori | `d4:9c:dd:ea:dc:56` |
| Óin | `d4:9c:dd:e9:b9:da` | Dain | `d4:9c:dd:ea:5a:d8` |

For example, suppose the router assigned `192.168.1.73`. First run discovery:

```text
# macOS or Linux
./moorebot-scout --master http://192.168.1.73:11311 discover

# Windows PowerShell
./moorebot-scout.exe --master http://192.168.1.73:11311 discover
```

Then start keyboard control by leaving off the final command:

```text
# macOS or Linux
./moorebot-scout --master http://192.168.1.73:11311

# Windows PowerShell
./moorebot-scout.exe --master http://192.168.1.73:11311
```

Use W/S to drive, A/D to strafe, Q/E to turn, Space to save a camera picture,
and Escape to stop. Keep `--master` on every command in Router mode. The driver
handles the Scout's internal `linaro-alip` hostname and the computer's local
address automatically.

If the address changes after a reboot, look it up again or create a DHCP
reservation for the Scout's MAC in the router settings.

## Network safety and troubleshooting

- Use a trusted private network. ROS 1 does not authenticate peers.
- Never port-forward ROS port 11311 or SSH port 22 to the internet.
- Avoid guest Wi-Fi and client isolation; both can prevent devices on the same
  router from connecting to one another.
- On Windows, allow `moorebot-scout.exe` on **Private** networks only when the
  firewall asks. Do not allow it on Public networks.
- If the app cannot transfer the Wi-Fi settings, switch back to Wi-Fi Direct
  and retry near the router. Moorebot documents support for WPA2 and both 2.4
  GHz and 5 GHz, with 5 GHz preferred when range permits.
- If configuration becomes unusable, the official reset procedure is an
  eight-second press of the recessed rear reset button, followed by up to two
  minutes for factory reset. Resetting erases the saved network setup, so use
  it only when reconfiguration is necessary.

Return to the [student getting-started guide](getting-started.md) for downloads,
SSH diagnostics, sensor monitoring, camera capture, and motion safety.
