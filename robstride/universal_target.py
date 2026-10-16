import time
from robstride_dynamics import Motor, RobstrideBus

# Initialize bus with Motor 1
bus = RobstrideBus(
    channel="can0",
    motors={
        "motor": Motor(
            id=1,
            model="rs-03",
        )
    },
)

print("Connecting to can0...")
bus.connect()

try:
    print("Enabling Motor 1...")
    bus.enable("motor")
    time.sleep(0.2)

    print("Commanding movement to +0.5 rad (~28°) under MIT Control...")
    
    # 5-second loop streaming MIT frames at 100 Hz
    for i in range(500):
        bus.write_operation_frame(
            motor="motor",
            position=20.0,   # Target angle in radians
            velocity=0.0,   # Target velocity
            kp=20.0,        # Stiffness (spring constant)
            kd=1.0,         # Damping
            torque=0.0,     # Feedforward torque
        )
        if i % 100 == 0:
            print(f"[{i//100 + 1}/5s] Holding +10.0 rad...")
        time.sleep(0.01)

    print("Target duration complete!")

except Exception as e:
    print(f"Error during execution: {e}")

finally:
    print("Disabling motor torque safely...")
    try:
        bus.disable("motor")
    except Exception:
        pass
    bus.disconnect(disable_torque=True)
    print("Clean exit.")