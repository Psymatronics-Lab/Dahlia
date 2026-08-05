"""Client for the SPI state exchange brick.

The service runs in its own container, so it is reached by compose service name
rather than localhost. Every call degrades gracefully: the arm keeps running on
the last valid frame if the brick is slow to come up or drops out.
"""

import json
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

BASE_URL = "http://spi_service:9000"
REQUEST_TIMEOUT = 0.1   # Short enough not to stall the control loop
READY_TIMEOUT = 2   # The readiness poll can afford to wait


class SPIService:

    def __init__(self, base_url=BASE_URL):
        self.base_url = base_url
        self.last_feedback = {"ok": False, "error": "No frame yet"}

    def begin(self, timeout=20):
        """Wait for the brick container to finish starting up.
        @param timeout Seconds to keep retrying before giving up
        @return True once the service answers, False if it never does
        """
        start = time.time()

        while time.time() - start < timeout:

            try:
                with urlopen(self.base_url + "/feedback", timeout=READY_TIMEOUT) as response:
                    response.read()

                return True

            except (URLError, OSError):
                time.sleep(0.5)

        return False

    def read_feedback(self):
        """Newest arm state, keeping the last valid frame if the MCU goes quiet."""
        try:
            with urlopen(self.base_url + "/feedback", timeout=REQUEST_TIMEOUT) as response:
                data = json.loads(response.read().decode("utf-8"))

        except (URLError, OSError, ValueError):
            return self.last_feedback

        if data.get("ok", False):
            self.last_feedback = data

        return self.last_feedback

    def write_targets(self, positions, velocities, gripper):
        """Send the newest command.
        @return True if the service accepted it
        """
        body = json.dumps({
            "positions": list(positions),
            "velocities": list(velocities),
            "gripper": int(gripper)
        }).encode("utf-8")

        request = Request(
            self.base_url + "/targets",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST"
        )

        try:
            with urlopen(request, timeout=REQUEST_TIMEOUT) as response:
                response.read()

            return True

        except (URLError, OSError):
            return False
