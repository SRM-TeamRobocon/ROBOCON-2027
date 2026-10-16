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
# INITIALIZE CAN BUS
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
    # DISABLE MOTOR BEFORE CHANGING MODE
    # ========================================================

    print("Disabling motor to safely change modes...")

    bus.disable(MOTOR_NAME)

    time.sleep(0.1)


    # ========================================================
    # POSITION PROFILE MODE
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
    print("=" * 120)
    print("                         RS03 POSITION PROFILE MODE")
    print("=" * 120)
    print()
    print("  Target input       : degrees")
    print("  Internal command   : radians")
    print("  Position feedback  : degrees")
    print("  Velocity feedback  : RPM")
    print("  Bus voltage        : V")
    print("  IQ feedback        : A")
    print("  Torque feedback    : N.m")
    print("  Temperature        : °C")
    print()
    print(f"  Telemetry refresh  : {TELEMETRY_INTERVAL:.1f} seconds")
    print()
    print("  Ctrl+C             : Stop / hold current position")
    print("  q                  : Quit")
    print()
    print("=" * 120)


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
        # Convert input to float
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

        print("Press Ctrl+C to stop and hold the motor.\n")


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

                # =================================================
                # POSITION
                # =================================================
                #
                # This is the parameter you tested separately
                # and confirmed gives the actual motor position.
                #

                position_rad = bus.read(
                    MOTOR_NAME,
                    ParameterType.MECHANICAL_POSITION
                )


                # =================================================
                # VELOCITY
                # =================================================

                velocity_rad_s = bus.read(
                    MOTOR_NAME,
                    ParameterType.MECHANICAL_VELOCITY
                )


                # =================================================
                # TORQUE
                # =================================================

                torque = bus.read(
                    MOTOR_NAME,
                    ParameterType.MEASURED_TORQUE
                )


                # =================================================
                # BUS VOLTAGE
                # =================================================

                vbus = bus.read(
                    MOTOR_NAME,
                    ParameterType.VBUS
                )


                # =================================================
                # IQ FILTERED
                # =================================================

                iq_filtered = bus.read(
                    MOTOR_NAME,
                    ParameterType.IQ_FILTERED
                )


                # =================================================
                # TEMPERATURE
                # =================================================
                #
                # Temperature is contained in the operation
                # status frame.
                #
                # read_operation_frame() returns:
                #
                # position, velocity, torque, temperature
                #

                try:

                    (
                        status_position,
                        status_velocity,
                        status_torque,
                        temperature
                    ) = bus.read_operation_frame(MOTOR_NAME)

                except Exception:

                    temperature = float("nan")


                # =================================================
                # UNIT CONVERSIONS
                # =================================================

                # Position:
                # radians -> degrees

                position_deg = math.degrees(position_rad)


                # Velocity:
                # rad/s -> RPM

                actual_rpm = velocity_rad_s * (
                    60.0 / (2.0 * math.pi)
                )


                # Position error

                position_error = target_deg - position_deg


                # =================================================
                # DISPLAY
                # =================================================

                print(
                    f"Target: {target_deg:8.2f}° | "
                    f"Actual: {position_deg:8.2f}° | "
                    f"Error: {position_error:+8.2f}° | "
                    f"Vel: {actual_rpm:+7.2f} RPM | "
                    f"VBUS: {vbus:6.2f} V | "
                    f"IQ: {iq_filtered:+6.2f} A | "
                    f"Torque: {torque:+6.2f} N.m | "
                    f"Temp: {temperature:5.1f} °C",
                    end="\r",
                    flush=True
                )


                # =================================================
                # REFRESH
                # =================================================

                time.sleep(TELEMETRY_INTERVAL)


        # ====================================================
        # CTRL+C
        # ====================================================

        except KeyboardInterrupt:

            print("\n\nStopping movement...")


            # ------------------------------------------------
            # Read current position
            # ------------------------------------------------

            try:

                current_position = bus.read(
                    MOTOR_NAME,
                    ParameterType.MECHANICAL_POSITION
                )


                # ------------------------------------------------
                # Hold current position
                # ------------------------------------------------

                bus.write(
                    MOTOR_NAME,
                    ParameterType.POSITION_TARGET,
                    current_position
                )


            except Exception as e:

                print(
                    f"Could not hold position: {e}"
                )


            time.sleep(0.2)

            print("Movement stopped and position held.")


# ============================================================
# ERROR HANDLING
# ============================================================

except Exception as e:

    print()
    print("=" * 70)
    print("ERROR")
    print("=" * 70)
    print(e)
    print("=" * 70)


# ============================================================
# SAFE SHUTDOWN
# ============================================================

finally:

    print()
    print("Safely disabling motor and disconnecting...")

    try:

        # ----------------------------------------------------
        # Read current position
        # ----------------------------------------------------

        current_position = bus.read(
            MOTOR_NAME,
            ParameterType.MECHANICAL_POSITION
        )


        # ----------------------------------------------------
        # Command current position
        # ----------------------------------------------------

        bus.write(
            MOTOR_NAME,
            ParameterType.POSITION_TARGET,
            current_position
        )

        time.sleep(0.1)


        # ----------------------------------------------------
        # Disable motor
        # ----------------------------------------------------

        bus.disable(MOTOR_NAME)

    except Exception:

        pass


    # --------------------------------------------------------
    # Disconnect CAN
    # --------------------------------------------------------

    try:

        bus.disconnect(
            disable_torque=True
        )

    except Exception:

        pass


    print("Clean exit.")
