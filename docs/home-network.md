# Use a Scout on a home network

The Moorebot Scout has two Wi-Fi modes:

- **Wi-Fi Direct:** the Scout creates `robot_scout_xxxxxx`; the driver uses the
  robot at `10.42.0.1` automatically.
- **Wi-Fi Router:** the Scout joins an isolated network provided by a home
  router; the router chooses the robot's address and the driver needs that
  address in `--master`.

Wi-Fi Direct is easiest for a first test. Router mode lets the computer retain
network access while it controls the robot, but ROS traffic must remain on an
isolated robot network.

## Configure Wi-Fi Router mode with the phone app

1. Install the **Moorebot Scout** app on an Android or iOS phone and complete
   the account setup requested by the app.
2. Put the Scout in its charging station, power it on, and use the physical
   Wi-Fi mode button to select Wi-Fi Direct mode.
3. Join `robot_scout_xxxxxx` from the phone. The factory password is
   `r0123456`; replace it when the app requests a new device password.
4. In the app, select the home Wi-Fi network and enter its password. The Scout
   switches to **Wi-Fi Router mode** after it connects successfully.
5. Reconnect the phone and the computer to the same dedicated robot SSID or
   VLAN. It must permit communication between the computer and Scout while
   isolating them from other household devices. If the home router cannot do
   that, use a separate router only for the Scout and controller.

The router-mode indicator stays on while connected and blinks when the Scout
cannot reach the router. The procedure comes from Moorebot's [official Scout
setup instructions](https://www.moorebot.com/products/scout-power-pack-scout-4-accessories-track-set-wand-laser-bumper).

Do not edit `/etc/hostapd/hostapd.conf` for this. That file configures the
hotspot created by Wi-Fi Direct mode, not the home-router connection.

## Configure router mode with the Rust driver

The Rust driver can submit the destination SSID and password to the Scout's
native ROS service. This is an alternative to the app's Wi-Fi handoff; it does
not reproduce the app's account, cloud, or proprietary transport layer. The
implementation follows the published firmware source and **has not yet been
verified on a physical Scout or across firmware versions**.

1. Power on the Scout in Wi-Fi Direct mode and connect the computer to
   `robot_scout_xxxxxx`.
2. Stop any active driving session, then run:

   ```text
   ./moorebot-scout wifi setup --ssid "Robot Lab"
   ```

   On Windows, use `./moorebot-scout.exe`. Enter the destination network's
   password at the hidden prompt. The target should be the dedicated robot
   SSID/VLAN described above.
3. The service's accepted response means it **queued** the change. It does not
   confirm a successful connection. The Scout may drop its Direct network
   before the command receives a reply; a timeout or disconnect means the
   result is unknown, not success or proof that nothing changed.
4. Manually reconnect the computer to the destination router network, find the
   Scout's DHCP-assigned address, and run `discover` with `--master` as shown
   below. The driver does not change the computer's Wi-Fi settings or determine
   the new address automatically.

The request timeout defaults to 10 seconds; use `--timeout-seconds 30` to
change it within 1–60 seconds. The timeout covers the service request, not the
time the Scout needs to join the router. If the robot does not appear in the
router's device list, use its physical Wi-Fi mode control to return to Direct
mode and check the network and credentials before retrying.

For automation, `--password-stdin` reads a single bounded password line from
standard input. It removes only a final LF or CRLF, preserving spaces. Supply
that input from an appropriate secret source; putting a literal password in a
shell command can expose it in shell history. There is deliberately no
plaintext password argument.

Initially, the SSID must contain 1–32 printable ASCII bytes and the password
8–63 printable ASCII bytes. Neither may contain a single quote, double quote,
or backslash because the stock firmware interpolates credentials into shell
commands and configuration text. Open networks, non-ASCII credentials, and
64-character raw PSKs are not supported by this command.

The driver does not intentionally log or save credentials on the computer.
ROS 1 sends them without encryption or peer authentication, and the published
Scout firmware stores them on the robot and includes them in some logging
paths. A hidden prompt does not change those firmware or transport properties.
Use only a trusted Scout on an isolated robot network; see the
[security policy](../SECURITY.md#router-wi-fi-credentials).

The published firmware also does not replace a saved password when the SSID is
already in its stored network list. Connecting successfully with a changed
password does not confirm that the new password will persist after a reboot.
See the [protocol notes](protocol.md#router-wi-fi-provisioning-service) for the
source evidence and remaining firmware limits.

## Find the Scout's address

Open the home router's connected-device or DHCP-client page. Look for
`linaro-alip`, a manufacturer name, or an unnamed device with the Scout's MAC
address. The computer's ARP cache cannot discover an address it has never
contacted, so use the router's DHCP data rather than relying on `arp -a`.

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

- Use a dedicated SSID, VLAN, or separate router containing only the Scout and
  its controller. ROS 1 does not authenticate peers.
- Never port-forward ROS port 11311 or SSH port 22 to the internet.
- The robot network must permit the Scout and controller to contact each other.
  Typical guest Wi-Fi blocks this with client isolation and will not work.
- On Windows, open **Settings → Network & internet → Wi-Fi → the robot network**
  and set **Network profile type** to **Private** only if this is the dedicated,
  trusted robot network described above. Then allow `moorebot-scout.exe` on
  **Private** networks when the firewall asks. Do not allow it on Public
  networks or mark an untrusted network Private.
- If the app cannot transfer the Wi-Fi settings, switch back to Wi-Fi Direct
  and retry near the router. Moorebot documents support for WPA2 and both 2.4
  GHz and 5 GHz, with 5 GHz preferred when range permits.
- If configuration becomes unusable, the official reset procedure is an
  eight-second press of the recessed rear reset button, followed by up to two
  minutes for factory reset. Resetting erases the saved network setup, so use
  it only when reconfiguration is necessary.

Return to the [student getting-started guide](getting-started.md) for downloads,
SSH diagnostics, sensor monitoring, camera capture, and motion safety.
