import time
from robstride_dynamics import Motor, ParameterType, RobstrideBus

bus = RobstrideBus(
    channel="can0",
    motors={"motor": Motor(id=1, model="rs-03")},
)

try:
    print("Connecting to CAN bus...")
    bus.connect()
    print("Reading telemetry... Press Ctrl+C to stop.\n")

    while True:
        pos = bus.read("motor", ParameterType.MECHANICAL_POSITION)
        vel = bus.read("motor", ParameterType.MECHANICAL_VELOCITY)

        # Overwrites the current line for a clean terminal output
        print(f"Current Position: {pos:+.4f} rad | Velocity: {vel:+.4f} rad/s", end="\r")
        time.sleep(0.1)  # Poll at 10 Hz

except KeyboardInterrupt:
    print("\n\nStopped reading telemetry.")

except Exception as e:
    print(f"\nError: {e}")

finally:
    bus.disconnect()
    print("Disconnected.")