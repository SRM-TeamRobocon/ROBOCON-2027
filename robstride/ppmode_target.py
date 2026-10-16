import time
from robstride_dynamics import Motor, ParameterType, RobstrideBus

# Initialize bus with Motor 1
bus = RobstrideBus(
    channel="can0",
    motors={"motor": Motor(id=1, model="rs-03")},
)

try:
    print("Connecting to CAN bus...")
    bus.connect()

    print("Disabling motor to safely change mode...")
    bus.disable("motor")
    time.sleep(0.1)

    print("Switching to Profile Position (PP) Mode (Mode 1)...")
    bus.write("motor", ParameterType.MODE, 1)
    time.sleep(0.1)

    print("Setting motion parameters (Vel Max & Acceleration)...")
    bus.write("motor", ParameterType.PP_VELOCITY_MAX, 10.0)       # rad/s
    bus.write("motor", ParameterType.PP_ACCELERATION_TARGET, 10.0) # rad/s^2
    time.sleep(0.1)

    print("Enabling Motor 1...")
    bus.enable("motor")
    time.sleep(0.1)

    print("\n--- PP Mode Control & Live Telemetry Ready ---")

    while True:
        user_input = input("\nEnter target angle in radians (or 'q' to quit): ").strip()
        if user_input.lower() == 'q':
            break

        try:
            target_rad = float(user_input)
            print(f"Commanding position target: {target_rad} rad...\n")
            
            # Write target position directly to register
            bus.write("motor", ParameterType.POSITION_TARGET, target_rad)

            # --- THE COMBINED TELEMETRY LOOP ---
            while True:
                # Read live telemetry
                pos = bus.read("motor", ParameterType.MECHANICAL_POSITION)
                vel = bus.read("motor", ParameterType.MECHANICAL_VELOCITY)

                # Overwrite the terminal line with live updates
                print(f"Target: {target_rad:+.4f} rad | Actual Pos: {pos:+.4f} rad | Vel: {vel:+.4f} rad/s", end="\r")
                
                # Auto-break out of the loop when the motor reaches the target and stops moving
                # (Error margin of 0.05 radians and velocity near zero)
                if abs(pos - target_rad) < 0.05 and abs(vel) < 0.1:
                    print(f"\nTarget {target_rad} reached and holding!")
                    break
                
                time.sleep(0.05)  # Poll at 20 Hz

        except ValueError:
            print("Invalid input. Please enter a valid number.")
            
        except KeyboardInterrupt:
            # If you press Ctrl+C while it's moving, it just interrupts the telemetry 
            # and lets you enter a new target, without crashing the program.
            print("\nTelemetry tracking interrupted. Motor will finish its move.")

except Exception as e:
    print(f"\nAn error occurred: {e}")

finally:
    print("\nSafely disabling motor and disconnecting...")
    try:
        bus.disable("motor")
    except Exception:
        pass
    bus.disconnect(disable_torque=True)
    print("Clean exit.")