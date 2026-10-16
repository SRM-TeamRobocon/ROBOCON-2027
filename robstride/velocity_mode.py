import time
import math
from robstride_dynamics import Motor, ParameterType, RobstrideBus

# Initialize the bus with Motor 1
bus = RobstrideBus(
    channel="can0",
    motors={"motor": Motor(id=1, model="rs-03")},
)

try:
    print("Connecting to CAN bus...")
    bus.connect()

    print("Disabling motor to safely change modes...")
    bus.disable("motor")
    time.sleep(0.1)

    print("Switching to Profile Velocity (PV) Mode (Mode 2)...")
    bus.write("motor", ParameterType.MODE, 2)
    time.sleep(0.1)

    # --- THE FIX: UNLOCK MAXIMUM SPEED ---
    print("Raising firmware speed limit to max (20.0 rad/s or ~191 RPM)...")
    bus.write("motor", ParameterType.VELOCITY_LIMIT, 50.0)
    time.sleep(0.1)
    # -------------------------------------

    print("Enabling Motor 1...")
    bus.enable("motor")
    time.sleep(0.1)

    print("\n--- Profile Velocity (PV) Mode Ready ---")
    print("Enter a target in RPM. The motor will maintain this speed internally.")

    while True:
        user_input = input("\nEnter target speed in RPM (or 'q' to quit): ").strip()
        if user_input.lower() == 'q':
            break

        try:
            target_rpm = float(user_input)
            target_rad_s = target_rpm * (2 * math.pi / 60)
            
            print(f"Commanding {target_rpm} RPM... (Press Ctrl+C to stop the motor)")
            
            # Send the velocity target exactly ONCE
            bus.write("motor", ParameterType.VELOCITY_TARGET, target_rad_s)

            # Drop into an infinite telemetry loop
            try:
                while True:
                    # Read live telemetry
                    pos = bus.read("motor", ParameterType.MECHANICAL_POSITION)
                    vel = bus.read("motor", ParameterType.MECHANICAL_VELOCITY)
                    
                    # Convert rad/s back to RPM for easy reading
                    actual_rpm = vel * (60 / (2 * math.pi))

                    # Overwrite the terminal line with live updates
                    print(f"Target: {target_rpm:6.1f} RPM | Actual: {actual_rpm:6.1f} RPM | Pos: {pos:8.2f} rad", end="\r")
                    
                    time.sleep(0.05)  # Poll at 20 Hz

            except KeyboardInterrupt:
                # Catching Ctrl+C inside this inner loop lets us stop the motor 
                # without crashing the entire Python script.
                print("\n\nStopping motor...")
                
                # Command 0 RPM to stop
                bus.write("motor", ParameterType.VELOCITY_TARGET, 0.0)
                
                # Wait a moment for the motor to physically spin down before asking for next input
                time.sleep(0.5) 

        except ValueError:
            print("Invalid input. Please enter a valid number.")

except Exception as e:
    print(f"\nAn error occurred: {e}")

finally:
    print("\nSafely disabling motor and disconnecting...")
    try:
        bus.write("motor", ParameterType.VELOCITY_TARGET, 0.0)
        time.sleep(0.1)
        bus.disable("motor")
    except Exception:
        pass
    bus.disconnect(disable_torque=True)
    print("Clean exit.")