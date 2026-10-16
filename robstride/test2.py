import math
import time
from robstride_dynamics import Motor, RobstrideBus

# --- 1. PRE-FLIGHT INITIALIZATION ---
bus = RobstrideBus(
    channel="can0",
    motors={"motor": Motor(id=1, model="rs-03")},
)

print("Connecting to CAN bus...")
bus.connect()

try:
    print("Disabling torque for safe initialization...")
    bus.disable("motor")
    time.sleep(0.1)

    print("Enabling Motor 1...")
    bus.enable("motor")
    time.sleep(0.1)

    # Read initial status
    status = bus.read_operation_frame("motor")
    print(f"Motor ready! Current Position: {math.degrees(status.position):.1f}°")

    # --- 2. INTERACTIVE MENU ---
    while True:
        print("\n--- RobStride Motor Control Menu ---")
        print("1. Move to Degree (Position Control)")
        print("2. Run at Target RPM (Velocity Control)")
        print("3. Exit")
        choice = input("Select an option (1-3): ").strip()

        if choice == "1":
            deg = float(input("Enter target angle in degrees (e.g. 60 or 100): "))
            
            # Convert degrees to radians
            target_rad = math.radians(deg)
            
            # Read current position to ramp smoothly
            current_rad = bus.read_operation_frame("motor").position
            
            steps = 200  # 2 seconds
            for i in range(steps):
                progress = (1 - math.cos(math.pi * (i / steps))) / 2
                cmd_pos = current_rad + ((target_rad - current_rad) * progress)
                
                bus.write_operation_frame(
                    motor="motor",
                    position=cmd_pos,
                    velocity=0.0,
                    kp=20.0,
                    kd=1.0,
                    torque=0.0,
                )
                time.sleep(0.01)
            print(f"Done! Motor is holding at {deg}°.")

        elif choice == "2":
            rpm = float(input("Enter target RPM (e.g. 30 or 60): "))
            dur = float(input("Enter run duration in seconds: "))
            
            rad_s = rpm * (2 * math.pi / 60)
            print(f"Spinning at {rpm} RPM...")
            
            steps = int(dur * 100)
            for _ in range(steps):
                bus.write_operation_frame(
                    motor="motor",
                    position=0.0,
                    velocity=rad_s,
                    kp=0.0,    # Kp=0 for pure velocity tracking
                    kd=2.0,    # Damping gain tracks speed
                    torque=0.0,
                )
                time.sleep(0.01)
                
            # Stop motor
            for _ in range(50):
                bus.write_operation_frame(motor="motor", position=0.0, velocity=0.0, kp=0.0, kd=2.0, torque=0.0)
                time.sleep(0.01)
            print("Stopped.")

        elif choice == "3":
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    print("Disabling motor safely...")
    try:
        bus.disable("motor")
    except Exception:
        pass
    bus.disconnect(disable_torque=True)
    print("Clean exit.")