# Security policy and threat model

## Supported versions

This project has not made its first crates.io release. Security fixes currently
land on `main` and are not backported.

## Deployment boundary

ROS 1 does not authenticate masters, publishers, subscribers, or service
callers. Run this driver only with a Scout and ROS master you trust, on an
isolated robot network. Do not expose the Scout ROS master or this driver to the
internet. Treat `--master` as a trusted configuration value: the selected master
can direct subscriptions to other addresses reachable from the computer.

Scout firmware advertises some ROS endpoints with its private hostname
`linaro-alip`. The driver rewrites only that exact hostname to the IP address of
the ROS master it has just reached. The mapping exists only inside the driver
process; it does not edit DNS or a system hosts file. This improves usability,
but it does not authenticate the peer. Robot-name detection from local Wi-Fi or
ARP command output is display-only and is never used to authorize motion.

The factory Wi-Fi and SSH credentials mentioned in the student guide are public
defaults, not secrets. Change them through the supported Moorebot setup process
before using the robot on a shared network. Do not place classroom passwords,
private keys, or diagnostic captures containing credentials in GitHub issues.

## Router Wi-Fi credentials

`wifi setup` sends the destination SSID and password through the native
`/WiFiNode/switch_wifi_ssidkey` ROS service. ROS 1 provides neither encryption
nor peer authentication; the selected master can direct the credentials to
the service endpoint it returns. Use only a trusted Scout and master on the
isolated robot network described above.

The CLI hides interactive password input and offers a bounded
`--password-stdin` option for automation. It does not accept a plaintext
password argument, print credentials in diagnostic messages, or intentionally
save them in host files. These measures do not secure the robot's storage or
logs: the published firmware persists network credentials and includes them
in some logging paths.

The initial client accepts only 1–32 printable ASCII bytes for the SSID and
8–63 for the password, excluding single quotes, double quotes, and backslashes.
This conservative subset accommodates the stock firmware's shell and
configuration interpolation; it does not claim to repair that firmware.
Open networks and raw 64-character PSKs are not supported. The feature is
source-backed and has not yet been verified on hardware or across firmware
versions. A queued response is not proof that the robot joined the router.

## Input limits

The project checks every variable-length field in its own ROS codecs before
copying or retaining it. Current maximums are:

- 16 MiB for a Scout media payload;
- 4 KiB for a sensor header frame ID;
- 64 integers for a battery status vector; and
- one pending decoded message in each camera or sensor application queue.

Regression tests cover oversized declared lengths. The full table and audit
scope are in [`docs/protocol.md`](docs/protocol.md#security-and-resource-limit-audit).

## Transport hardening and remaining limitations

The optional default `ros1` feature pins a Moorebot-specific `rosrust` fork at
an exact Git revision. The fork checks the outer TCPROS length before allocating
the body and caps connection headers at 64 KiB. Before starting ROS, commands
using that transport select a process-wide body ceiling: 8 KiB for discovery, motion,
and sensor monitoring, or 16 MiB + 1 KiB for camera commands. The transport's
fixed staging queue is therefore bounded by both message count and bytes.
The allocation-limit change has been submitted upstream as [`rosrust` PR
#221](https://github.com/adnanademovic/rosrust/pull/221).

General `rosrust` service calls are outside that patch's allocation limits.
Wi-Fi setup instead uses a dedicated connection with a single 1–60-second
request deadline, a 16 KiB lookup HTTP header limit, a 64 KiB lookup body limit,
and a 64 KiB/128-field service handshake limit. A successful reply must contain
exactly one status byte; failure text is capped at 1 KiB and discarded. The
client checks the service MD5 before sending credentials and never retries
their transmission automatically. These bounds apply only to this dedicated
Wi-Fi path, not to arbitrary ROS service clients.

Cargo substitutes crates.io versions for Git dependencies when it verifies a
publishable archive. Because crates.io's `rosrust` does not yet contain these
limits, `Cargo.toml` sets `publish = false`. A crates.io release remains blocked
by issue #9; publishing must be re-enabled and full package verification
restored only after the bounded transport comes from a publishable dependency.

A RustSec scan of the current lockfile also reports four vulnerabilities in the
old HTTP/XML-RPC dependency chain used by `rosrust`: RUSTSEC-2020-0071,
RUSTSEC-2021-0078, RUSTSEC-2021-0079, and RUSTSEC-2024-0421. Several related
transitive crates are unmaintained, and `hyper` 0.10.16 and `traitobject` 0.1.1
also carry unsoundness warnings (RUSTSEC-2022-0022 and RUSTSEC-2020-0027).
These dependencies are absent when the crate is built with
`--no-default-features`. They are not silently allowlisted;
[transport modernization is tracked as release-blocking security
work](https://github.com/cmontella/moorebot-scout/issues/9).

## Reporting a vulnerability

Please use GitHub private vulnerability reporting for this repository when it
is available. If it is not available, contact the maintainer address listed in
`Cargo.toml`. Do not disclose a vulnerability in a public issue before a fix is
available.
