import time
import math

from robstride_dynamics import Motor, ParameterType, RobstrideBus


# ============================================================
# CONFIGURATION
# ============================================================

CAN_CHANNEL = "can0"
MOTOR_ID = 1
MOTOR_NAME = "motor"

# Telemetry refresh interval
TELEMETRY_INTERVAL = 0.5


# ============================================================
# INITIALIZE ROBSTRIDE CAN BUS
# ============================================================

bus = RobstrideBus(
    channel=CAN_CHANNEL,
    motors={
        MOTOR_NAME: Motor(
            id=MOTOR_ID,
            model="rs-03"
        )
    }
)


try:

    # ========================================================
    # CONNECT
    # ========================================================

    print("Connecting to CAN bus...")
    bus.connect()

    time.sleep(0.1)


    # ========================================================
    # DISABLE MOTOR
    # ========================================================

    print("Disabling motor to safely change modes...")

    bus.disable(MOTOR_NAME)

    time.sleep(0.1)


    # ========================================================
    # POSITION PROFILE MODE
    # ========================================================
    #
    # Mode 1 = PP / Position Profile
    #
    # The exact mode number should match your RS03 firmware.
    # ========================================================

    print("Switching to Position Profile (PP) Mode...")

    bus.write(
        MOTOR_NAME,
        ParameterType.MODE,
        1
    )

    time.sleep(0.1)


    # ========================================================
    # ENABLE MOTOR
    # ========================================================

    print("Enabling Motor 1...")

    bus.enable(MOTOR_NAME)

    time.sleep(0.1)


    # ========================================================
    # READY
    # ========================================================

    print()
    print("=" * 100)
    print("                 RS03 POSITION PROFILE MODE")
    print("=" * 100)
    print()
    print("  Input target      : degrees")
    print("  Internal target   : radians")
    print("  Position          : degrees")
    print("  Velocity          : RPM")
    print("  VBUS              : volts")
    print("  IQ filtered       : A")
    print("  Torque            : N.m")
    print()
    print(f"  Telemetry refresh : {TELEMETRY_INTERVAL:.1f} seconds")
    print()
    print("  Ctrl+C            : Stop current movement")
    print("  q                 : Quit program")
    print()
    print("=" * 100)


    # ========================================================
    # MAIN COMMAND LOOP
    # ========================================================

    while True:

        # ----------------------------------------------------
        # Ask for target angle
        # ----------------------------------------------------

        user_input = input(
            "\nEnter target angle in degrees (or 'q' to quit): "
        ).strip()


        # ----------------------------------------------------
        # Quit
        # ----------------------------------------------------

        if user_input.lower() == "q":
            break


        # ----------------------------------------------------
        # Convert input
        # ----------------------------------------------------

        try:

            target_deg = float(user_input)

        except ValueError:

            print(
                "Invalid input. "
                "Please enter a valid angle in degrees."
            )

            continue


        # ====================================================
        # DEGREES -> RADIANS
        # ====================================================

        target_rad = math.radians(target_deg)


        print()
        print(
            f"Commanding {target_deg:.2f}° "
            f"({target_rad:.4f} rad)"
        )

        print("Press Ctrl+C to stop the current movement.\n")


        # ====================================================
        # SEND POSITION TARGET
        # ====================================================

        bus.write(
            MOTOR_NAME,
            ParameterType.POSITION_TARGET,
            target_rad
        )


        # ====================================================
        # TELEMETRY LOOP
        # ====================================================

        try:

            while True:

                # --------------------------------------------
                # Read position
                # --------------------------------------------

                position_rad = bus.read(
                    MOTOR_NAME,
                    ParameterType.MEASURED_POSITION
                )


                # --------------------------------------------
                # Read velocity
                # --------------------------------------------

                velocity_rad_s = bus.read(
                    MOTOR_NAME,
                    ParameterType.MEASURED_VELOCITY
                )


                # --------------------------------------------
                # Read torque
                # --------------------------------------------

                torque = bus.read(
                    MOTOR_NAME,
                    ParameterType.MEASURED_TORQUE
                )


                # --------------------------------------------
                # Read VBUS
                # --------------------------------------------

                vbus = bus.read(
                    MOTOR_NAME,
                    ParameterType.VBUS
                )


                # --------------------------------------------
                # Read filtered q-axis current
                # --------------------------------------------

                iq_filtered = bus.read(
                    MOTOR_NAME,
                    ParameterType.IQ_FILTERED
                )


                # --------------------------------------------
                # Convert position
                # --------------------------------------------

                position_deg = math.degrees(position_rad)


                # --------------------------------------------
                # Convert velocity
                # --------------------------------------------

                actual_rpm = velocity_rad_s * (
                    60.0 / (2.0 * math.pi)
                )


                # --------------------------------------------
                # Position error
                # --------------------------------------------

                position_error = target_deg - position_deg


                # --------------------------------------------
                # Display
                # --------------------------------------------

                print(
                    f"Target: {target_deg:9.2f}° | "
                    f"Actual: {position_deg:9.2f}° | "
                    f"Error: {position_error:8.2f}° | "
                    f"Vel: {actual_rpm:8.1f} RPM | "
                    f"VBUS: {vbus:6.2f} V | "
                    f"IQ: {iq_filtered:6.2f} A | "
                    f"Torque: {torque:6.2f} N.m",
                    end="\r",
                    flush=True
                )


                # --------------------------------------------
                # Check whether target has been reached
                # --------------------------------------------

                if abs(position_error) < 0.5 and abs(actual_rpm) < 2.0:

                    print()
                    print(
                        f"Target reached: "
                        f"{position_deg:.2f}°"
                    )

                    break


                # --------------------------------------------
                # Telemetry refresh
                # --------------------------------------------

                time.sleep(TELEMETRY_INTERVAL)


        # ====================================================
        # CTRL+C
        # ====================================================

        except KeyboardInterrupt:

            print("\n\nStopping movement...")

            # Hold current position
            try:

                current_position = bus.read(
                    MOTOR_NAME,
                    ParameterType.MEASURED_POSITION
                )

                bus.write(
                    MOTOR_NAME,
                    ParameterType.POSITION_TARGET,
                    current_position
                )

            except Exception:
                pass

            time.sleep(0.2)

            print("Movement stopped.")


# ============================================================
# ERROR HANDLING
# ============================================================

except Exception as e:

    print()
    print("=" * 60)
    print("ERROR")
    print("=" * 60)
    print(e)
    print("=" * 60)


# ============================================================
# SAFE SHUTDOWN
# ============================================================

finally:

    print()
    print("Safely disabling motor and disconnecting...")

    try:

        # Read current position and command it as target
        # so the motor does not receive an unexpected target
        # during shutdown.

        current_position = bus.read(
            MOTOR_NAME,
            ParameterType.MEASURED_POSITION
        )

        bus.write(
            MOTOR_NAME,
            ParameterType.POSITION_TARGET,
            current_position
        )

        time.sleep(0.1)

        bus.disable(MOTOR_NAME)

    except Exception:
        pass


    try:

        bus.disconnect(disable_torque=True)

    except Exception:
        pass


    print("Clean exit.")
