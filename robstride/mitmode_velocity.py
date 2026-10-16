import time
import math
from robstride_dynamics import Motor, RobstrideBus
from robstride_dynamics.protocol import ParameterType

# Initialize the bus and motor
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

    print("Forcing motor back to MIT Mode (Mode 0)...")
    bus.write("motor", ParameterType.MODE, 0)  # THIS IS THE MAGIC LINE
    time.sleep(0.1)

    print("Enabling Motor 1...")
    bus.enable("motor")
    time.sleep(0.1)

    print("Enabling Motor 1 in MIT Mode (Mode 0)...")
    bus.disable("motor")
    time.sleep(0.1)
    bus.enable("motor")
    time.sleep(0.1)

    print("\n--- MIT Velocity Control with Live Feedback ---")
    print("Enter velocity in RPM (e.g., 30, 60, -30). Type 'q' to quit.")

    while True:
        user_input = input("\nEnter target RPM: ").strip()
        if user_input.lower() == 'q':
            break

        try:
            target_rpm = float(user_input)
            
            # Convert RPM to radians per second for the motor controller
            target_rad_s = target_rpm * (2 * math.pi / 60)
            
            print(f"Spinning at {target_rpm} RPM... (Press Ctrl+C to stop this run)")
            
            # Stream MIT frames continuously for 10 seconds (1000 steps at 100Hz)
            duration_sec = 15.0
            steps = int(duration_sec * 100)
            
            for i in range(steps):
                # 1. Send the command frame
                bus.write_operation_frame(
                    motor="motor",
                    position=0.0,      # Ignored because Kp = 0
                    velocity=target_rad_s,
                    kp=0.0,            # Kp=0 disables the position hold
                    kd=15.0,            # Kd acts as our Velocity Proportional Gain
                    torque=0.0,
                )
                
                # 2. Immediately read the status reply
                status = bus.read_operation_frame("motor")
                
                # 3. Extract data from the tuple (Position, Velocity, Torque, Temperature)
                actual_rad_s = status[1]
                temperature = status[3]
                
                # Convert actual velocity back to RPM for the terminal
                actual_rpm = actual_rad_s * (60 / (2 * math.pi))
                
                # Print every 10th loop (10 Hz) for a clean, flicker-free terminal
                if i % 10 == 0:
                    print(f"Target: {target_rpm:5.1f} RPM  |  Actual: {actual_rpm:5.1f} RPM  |  Temp: {temperature:2.0f}°C", end="\r")
                
                # Maintain the 100Hz control loop heartbeat
                time.sleep(0.01)
                
            print() # Clear the carriage return line after the run finishes

            # Smooth stop
            print("Decelerating to stop...")
            for _ in range(30):
                bus.write_operation_frame(
                    motor="motor", position=0.0, velocity=0.0, kp=0.0, kd=1.5, torque=0.0
                )
                time.sleep(0.01)

        except ValueError:
            print("Invalid input. Please enter a valid number.")
        except KeyboardInterrupt:
            # Catch Ctrl+C during a run to stop the motor without exiting the whole program
            print("\nRun interrupted! Stopping motor...")
            for _ in range(30):
                bus.write_operation_frame(
                    motor="motor", position=0.0, velocity=0.0, kp=0.0, kd=1.5, torque=0.0
                )
                time.sleep(0.01)

except Exception as e:
    print(f"\nAn error occurred: {e}")

finally:
    print("\nSafely disabling motor...")
    try:
        bus.disable("motor")
    except Exception:
        pass
    bus.disconnect(disable_torque=True)
    print("Clean exit.")