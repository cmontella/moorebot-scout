"""Refresh one velocity command for two seconds, then stop.

For the first run, put the Scout on a stable stand with all wheels clear.
"""

import time

from moorebot_scout import ScoutClient


def main() -> None:
    with ScoutClient() as scout:
        deadline = time.monotonic() + 2.0
        try:
            while time.monotonic() < deadline:
                scout.set_velocity(0.05, 0.0, 0.0, timeout=0.25)
                time.sleep(0.05)
        finally:
            scout.stop()


if __name__ == "__main__":
    main()
