import time

import requests

from arduino.app_utils import App
from control.control import ArmController, NUM_JOINTS, PERIOD
from spi_service import SPIService

CONTROLLER_URL = "http://172.17.0.1:8000/controller"

JOINT_VEL = 5.0   # Constant velocity limit sent for every joint, near the servo maximum

spi = SPIService()
arm = ArmController()


def loop():

    if not arm.ready:
        arm.seed(spi.read_feedback())
        time.sleep(PERIOD)
        return

    try:
        data = requests.get(
            CONTROLLER_URL,
            timeout=0.1
        ).json()

    except Exception as e:
        print("Controller API error:", e)
        time.sleep(PERIOD)
        return

    if not data["connected"]:
        print("Controller disconnected")
        time.sleep(PERIOD)
        return

    targets = arm.update(data["state"], spi.read_feedback())

    if not spi.write_targets(targets, [JOINT_VEL] * NUM_JOINTS, arm.gripper()):
        print("SPI service unavailable")
        arm.ready = False   # Re-seed from the measured pose once it comes back

    time.sleep(PERIOD)


if not spi.begin():
    print("SPI service did not come up, retrying in the loop")

App.run(user_loop=loop)
