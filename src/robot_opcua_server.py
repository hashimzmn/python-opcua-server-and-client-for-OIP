
import asyncio
from asyncua import Server, ua


async def main():
    server = Server()
    await server.init()

    server.set_endpoint("opc.tcp://0.0.0.0:4840/freeopcua/server/")
    server.set_server_name("OIP Six-Axis Robot OPC UA Server")

    uri = "urn:oip:sixaxis-robot"
    idx = await server.register_namespace(uri)

    objects = server.nodes.objects

    # -------------------------
    # FAULT INFORMATION
    # -------------------------
    fault_active = await objects.add_variable(
        ua.NodeId("FaultActive", idx),
        "FaultActive",
        False,
        ua.VariantType.Boolean,
    )

    fault_applied = await objects.add_variable(
        ua.NodeId("FaultApplied", idx),
        "FaultApplied",
        0,
        ua.VariantType.Int16,
    )

    fault_magnitude = await objects.add_variable(
        ua.NodeId("FaultMagnitude", idx),
        "FaultMagnitude",
        0.0,
        ua.VariantType.Float,
    )

    # -------------------------
    # JOINT ANGLES
    # -------------------------
    joint_angles = {}

    for i in range(1, 7):
        name = f"J{i}Angle"

        joint_angles[name] = await objects.add_variable(
            ua.NodeId(name, idx),
            name,
            0.0,
            ua.VariantType.Float,
        )

    # -------------------------
    # SIMULATION TIME
    # -------------------------
    simulation_time = await objects.add_variable(
        ua.NodeId("SimulationTime", idx),
        "SimulationTime",
        0.0,
        ua.VariantType.Float,
    )

    # -------------------------
    # MOVEMENT SPEED
    # -------------------------
    motion_speed = await objects.add_variable(
        ua.NodeId("MotionSpeed", idx),
        "MotionSpeed",
        120.0,
        ua.VariantType.Float,
    )

    # -------------------------
    # COMMAND CONTROL
    # -------------------------
    command = await objects.add_variable(
        ua.NodeId("Command", idx),
        "Command",
        0,
        ua.VariantType.Int16,
    )

    execute = await objects.add_variable(
        ua.NodeId("Execute", idx),
        "Execute",
        False,
        ua.VariantType.Boolean,
    )

    # -------------------------
    # ALLOW OPC UA WRITES
    # -------------------------
    variables = [
        fault_active,
        fault_applied,
        fault_magnitude,
        simulation_time,
        motion_speed,
        *joint_angles.values(),
        command,
        execute,
    ]

    for variable in variables:
        await variable.set_writable()

    # -------------------------
    # SERVER INFORMATION
    # -------------------------
    print("Server started")
    print("Endpoint: opc.tcp://127.0.0.1:4840/freeopcua/server/")
    print(f"Namespace URI: {uri}")
    print(f"Namespace index: {idx}")

    print("\nNodeIds:")
    for variable in variables:
        print(f"  {variable.nodeid.to_string()}")

    print("\nCommand controls:")
    print("  101: Home -> Marker 1")
    print("  102: Marker 1 -> Marker 2")
    print("  103: Marker 2 -> Home")
    print("  Write a command integer to Command.")
    print("  Set Execute to True to trigger the command.")
    print("  Reset Execute to False before the next command.")

    # -------------------------
    # MONITORING LOOP
    # -------------------------
    async with server:
        while True:
            active = await fault_active.read_value()
            applied = await fault_applied.read_value()
            magnitude = await fault_magnitude.read_value()
            sim_time = await simulation_time.read_value()
            speed = await motion_speed.read_value()

            command_value = await command.read_value()
            execute_value = await execute.read_value()

            angles = {}

            for name, variable in joint_angles.items():
                angles[name] = await variable.read_value()

            # print(
            #     f"Time={sim_time:.2f}s | "
            #     f"FaultActive={active} | "
            #     f"FaultApplied={applied} | "
            #     f"FaultMagnitude={magnitude:.2f} | "
            #     + " | ".join(
            #         f"{name}={value:.2f}°"
            #         for name, value in angles.items()
            #     )
            #     + f" | MotionSpeed={speed:.2f}°/s"
            #     + f" | Command={command_value}"
            #     + f" | Execute={execute_value}"
            # )

            await asyncio.sleep(0.02)


if __name__ == "__main__":
    asyncio.run(main())
