import asyncio
import csv
import time
from datetime import datetime

from asyncua import Client, ua


# ============================================================
# CONFIGURATION
# ============================================================

SERVER_URL = "opc.tcp://127.0.0.1:4840/freeopcua/server/"
NAMESPACE = 2

NUM_ITERATIONS = 10
SEQUENCE_COMMANDS = [101, 102, 103]

# OIP polling interval: 100 ms
# Client samples at 50 ms to reduce missed updates
SAMPLE_INTERVAL_S = 0.02
COMMAND_DELAY_S = 0.02

# Motion detection
MOVEMENT_THRESHOLD_DEG = 0.05
STABILITY_THRESHOLD_DEG = 0.05
STABILITY_DURATION_S = 0.8

START_TIMEOUT_S = 8.0
MOVEMENT_TIMEOUT_S = 30.0

TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
CSV_FILENAME = f"robot_log_{TIMESTAMP}.csv"


# ============================================================
# ROBOT LOGGER
# ============================================================

class RobotLogger:

    def __init__(self, client):

        self.rows = []

        self.simulation_time = client.get_node(
            f"ns={NAMESPACE};s=SimulationTime"
        )

        self.motion_speed = client.get_node(
            f"ns={NAMESPACE};s=MotionSpeed"
        )

        self.joint_nodes = [
            client.get_node(f"ns={NAMESPACE};s=J{i}Angle")
            for i in range(1, 7)
        ]

        # Fault information
        self.fault_active = client.get_node(
            f"ns={NAMESPACE};s=FaultActive"
        )

        self.fault_applied = client.get_node(
            f"ns={NAMESPACE};s=FaultApplied"
        )

        self.fault_magnitude = client.get_node(
            f"ns={NAMESPACE};s=FaultMagnitude"
        )

        self.nodes = [
            self.simulation_time,
            self.motion_speed,
            *self.joint_nodes,
            self.fault_active,
            self.fault_applied,
            self.fault_magnitude,
        ]

    async def sample(self, iteration, sequence):

        # Read all variables concurrently
        values = await asyncio.gather(
            *[node.read_value() for node in self.nodes]
        )

        angles = [float(value) for value in values[2:8]]

        row = {
            "timestamp": datetime.now().isoformat(),
            "simulation_time": float(values[0]),
            "iteration": iteration,
            "sequence": sequence,
            "motion_speed": float(values[1]),
        }

        for i, angle in enumerate(angles, start=1):
            row[f"j{i}_angle"] = angle

        row["fault_active"] = bool(values[8])
        row["fault_applied"] = int(values[9])
        row["fault_magnitude"] = float(values[10])

        self.rows.append(row)

        return angles, row

    def save_csv(self):

        if not self.rows:
            print("No data recorded.")
            return

        fieldnames = [
            "timestamp",
            "simulation_time",
            "iteration",
            "sequence",
            "motion_speed",
            "j1_angle",
            "j2_angle",
            "j3_angle",
            "j4_angle",
            "j5_angle",
            "j6_angle",
            "fault_active",
            "fault_applied",
            "fault_magnitude",
        ]

        with open(
            CSV_FILENAME,
            "w",
            newline="",
            encoding="utf-8",
        ) as file:

            writer = csv.DictWriter(
                file,
                fieldnames=fieldnames,
            )

            writer.writeheader()
            writer.writerows(self.rows)

        fault_samples = sum(
            1 for row in self.rows
            if row["fault_active"]
        )

        print(f"\nSaved {len(self.rows)} samples to {CSV_FILENAME}")
        print(f"Samples with active faults: {fault_samples}")


# ============================================================
# ROBOT CONTROLLER
# ============================================================

class RobotController:

    def __init__(self, client, logger):

        self.logger = logger

        self.command = client.get_node(
            f"ns={NAMESPACE};s=Command"
        )

        self.execute = client.get_node(
            f"ns={NAMESPACE};s=Execute"
        )

    async def set_execute(self, value):

        await self.execute.write_value(
            ua.Variant(value, ua.VariantType.Boolean)
        )

    async def set_command(self, value):

        await self.command.write_value(
            ua.Variant(value, ua.VariantType.Int16)
        )

    async def run_sequence(self, command_value, iteration):

        print(
            f"Cycle {iteration}/{NUM_ITERATIONS} | "
            f"Command {command_value}"
        )

        # Reset trigger
        await self.set_execute(False)

        # Record initial joint positions
        initial_angles, _ = await self.logger.sample(
            iteration,
            command_value,
        )

        # Send command and trigger execution
        await self.set_command(command_value)
        await self.set_execute(True)

        start_time = time.monotonic()
        next_sample_time = start_time

        movement_started = False
        stable_since = None
        last_angles = initial_angles.copy()

        try:

            while True:

                # Maintain approximately constant sampling intervals
                next_sample_time += SAMPLE_INTERVAL_S

                await asyncio.sleep(
                    max(0, next_sample_time - time.monotonic())
                )

                angles, row = await self.logger.sample(
                    iteration,
                    command_value,
                )

                now = time.monotonic()
                elapsed = now - start_time

                # --------------------------------------------
                # FAULT MONITORING
                # --------------------------------------------

                if row["fault_active"]:

                    print(
                        f"  Fault detected | "
                        f"Type={row['fault_applied']} | "
                        f"Magnitude={row['fault_magnitude']:.2f} | "
                        f"Time={row['simulation_time']:.2f}s"
                    )

                # --------------------------------------------
                # DETECT MOVEMENT START
                # --------------------------------------------

                if not movement_started:

                    total_change = max(
                        abs(a - b)
                        for a, b in zip(angles, initial_angles)
                    )

                    if total_change > MOVEMENT_THRESHOLD_DEG:

                        movement_started = True
                        stable_since = None

                        await self.set_execute(False)

                        print("  Movement started")

                    elif elapsed > START_TIMEOUT_S:

                        raise TimeoutError(
                            f"Command {command_value}: "
                            "Movement start timeout."
                        )

                # --------------------------------------------
                # DETECT MOVEMENT COMPLETION
                # --------------------------------------------

                if movement_started:

                    step_change = max(
                        abs(a - b)
                        for a, b in zip(angles, last_angles)
                    )

                    if step_change < STABILITY_THRESHOLD_DEG:

                        if stable_since is None:
                            stable_since = now

                        elif now - stable_since >= STABILITY_DURATION_S:

                            print(
                                f"  Movement completed ({elapsed:.2f}s)"
                            )

                            break

                    else:
                        stable_since = None

                    if elapsed > MOVEMENT_TIMEOUT_S:

                        raise TimeoutError(
                            f"Command {command_value}: "
                            "Movement completion timeout."
                        )

                last_angles = angles.copy()

        finally:

            await self.set_execute(False)


# ============================================================
# MAIN
# ============================================================

async def main():

    logger = None
    completed_cycles = 0

    try:

        async with Client(url=SERVER_URL) as client:

            print("Connected to OPC UA server")

            logger = RobotLogger(client)
            controller = RobotController(client, logger)

            try:

                await controller.set_execute(False)

                for iteration in range(1, NUM_ITERATIONS + 1):

                    print(
                        f"\n--- Cycle {iteration}/{NUM_ITERATIONS} ---"
                    )

                    for command_value in SEQUENCE_COMMANDS:

                        await controller.run_sequence(
                            command_value,
                            iteration,
                        )

                        await asyncio.sleep(COMMAND_DELAY_S)

                    completed_cycles += 1

                print("\nAll cycles completed.")

            finally:

                await controller.set_execute(False)

    finally:

        if logger is not None:
            logger.save_csv()

        print(
            f"Completed cycles: "
            f"{completed_cycles}/{NUM_ITERATIONS}"
        )


if __name__ == "__main__":
    asyncio.run(main())