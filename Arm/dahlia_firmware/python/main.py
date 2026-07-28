import time
import requests

from arduino.app_utils import App

CONTROLLER_URL = "http://172.17.0.1:8000/controller"

def loop():

    try:
        response = requests.get(
            CONTROLLER_URL,
            timeout=0.1
        )

        data = response.json()

        if data["connected"]:
            state = data["state"]

            print(
                "JOY:",
                state["joy_x"],
                state["joy_y"]
            )

            print(
                "ENC:",
                state["enc_pos"]
            )

        else:
            print(
                "Controller disconnected"
            )

    except Exception as e:
        print(
            "Controller API error:",
            e
        )

    time.sleep(0.05)


App.run(user_loop=loop)