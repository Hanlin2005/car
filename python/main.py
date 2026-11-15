"""Autonomous car on the Arduino Uno Q.

The Linux side runs this program and the page at http://<uno>:8080.
The microcontroller runs sketch/sketch.ino: motors, encoders, and the lidar.
Open this folder in Arduino App Lab and run it there.
"""

import threading
import time

import config
import hardware
import modes


def _app_loop():
    hardware.pump()
    time.sleep(0.05)


def main():
    from arduino.app_utils import App

    modes.install_log()
    hardware.start()
    import web
    if config.MODE == "web":
        modes.run()
    threading.Thread(target=web.serve, name="web", daemon=True).start()
    try:
        App.run(user_loop=_app_loop)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        modes.stop()
        hardware.shutdown()


if __name__ == "__main__":
    main()
