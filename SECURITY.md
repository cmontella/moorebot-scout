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

The factory Wi-Fi and SSH credentials mentioned in the student guide are public
defaults, not secrets. Change them through the supported Moorebot setup process
before using the robot on a shared network. Do not place classroom passwords,
private keys, or diagnostic captures containing credentials in GitHub issues.

## Input limits

The project checks every variable-length field in its own ROS codecs before
copying or retaining it. Current maximums are:

- 16 MiB for a Scout media payload;
- 4 KiB for a sensor header frame ID;
- 64 integers for a battery status vector; and
- one pending decoded message in each camera or sensor application queue.

Regression tests cover oversized declared lengths. The full table and audit
scope are in [`docs/protocol.md`](docs/protocol.md#security-and-resource-limit-audit).

## Known transport limitations

The optional default `ros1` feature uses `rosrust` 0.9.12. That dependency
allocates an outer TCPROS packet, and may stage it internally, before this
project's bounded decoder runs. Consequently, the local limits prevent a large
message from being copied or retained by our codecs but do not prevent the
transport's first allocation. Fully addressing this requires a patched or
replacement ROS 1 transport.

A RustSec scan of the current lockfile also reports four vulnerabilities in the
old HTTP/XML-RPC dependency chain used by `rosrust`: RUSTSEC-2020-0071,
RUSTSEC-2021-0078, RUSTSEC-2021-0079, and RUSTSEC-2024-0421. Several related
transitive crates are unmaintained. These dependencies are absent when the
crate is built with `--no-default-features`. They are not silently allowlisted;
transport modernization is tracked as release-blocking security work.

## Reporting a vulnerability

Please use GitHub private vulnerability reporting for this repository when it
is available. If it is not available, contact the maintainer address listed in
`Cargo.toml`. Do not disclose a vulnerability in a public issue before a fix is
available.
