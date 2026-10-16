import time
import math
from robstride_dynamics import Motor, ParameterType, RobstrideBus

bus = RobstrideBus(
    channel="can0",
    motors={"motor": Motor(id=1, model="rs-03")},
)

try:
    print("Connecting to CAN bus...")
    bus.connect()

    # Raise the firmware velocity limit to 35.0 rad/s (~334 RPM)
    new_limit_rad_s = 100.0
    print(f"Setting VELOCITY_LIMIT to {new_limit_rad_s} rad/s...")
    bus.write("motor", ParameterType.VELOCITY_LIMIT, new_limit_rad_s)
    time.sleep(0.1)

    # Read back to confirm
    updated_limit = bus.read("motor", ParameterType.VELOCITY_LIMIT)
    updated_rpm = updated_limit * (60 / (2 * math.pi))

    print(f"\n--- Updated Motor Velocity Limit ---")
    print(f"New Limit (rad/s): {updated_limit:.2f} rad/s")
    print(f"New Limit (RPM)  : {updated_rpm:.1f} RPM")

except Exception as e:
    print(f"\nError: {e}")

finally:
    bus.disconnect()
    print("Disconnected.")