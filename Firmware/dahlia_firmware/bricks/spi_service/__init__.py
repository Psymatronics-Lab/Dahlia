"""
Interface for the SPI state exchange brick, creating the SPIService object.

The service runs in its own containers, so it is reached by the compose
service name rather than localhost. 
"""

import json
import time
from urllib.error import URLError
from urllib.request import Request, urlopen

BASE_URL = "http://spi_service:9000"
REQUEST_TIMEOUT = 0.1   # Timeout in seconds for reading data from the MCU
READY_TIMEOUT = 2   # Timeout in seconds for waiting for the brick to boot
STALE_AFTER = 0.5   # Timeout in seconds for the last received command data


class SPIService:

    def __init__(self, base_url=BASE_URL):
        self.base_url = base_url
        self.last_feedback = {"ok": False, "error": "No frame yet"}
        self.last_good_at = None

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
        """Newest arm state, keeping the last valid frame if the MCU goes briefly quiet."""
        try:
            with urlopen(self.base_url + "/feedback", timeout=REQUEST_TIMEOUT) as response:
                data = json.loads(response.read().decode("utf-8"))
        except (URLError, OSError, ValueError):
            data = {}

        if data.get("ok", False):
            self.last_feedback = data
            self.last_good_at = time.monotonic()

        elif self.last_good_at is None or time.monotonic() - self.last_good_at > STALE_AFTER:
            return {"ok": False, "error": "No fresh frame"}

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
