import asyncio  # To run async loops
import websockets  # Required for device communication
import orjson  # JSON reader for fast processing

# Main asynchronous loop
async def main():
    uri = 'ws://localhost:10001'  # WebSocket port for Inverse Service 3.1 json format
    first_message = True
    inverse3_device_id = None
    force = {"x": 0, "y": 0, "z": 0}  # Forces to send to the Inverse3 device.

    # Haptic loop
    async with websockets.connect(uri) as ws:
        while True:
            # Receive data from the device
            response = await ws.recv()
            data = orjson.loads(response)

            # Get devices list from the data
            inverse3_devices = data.get("inverse3", [])
            verse_grip_devices = data.get("wireless_verse_grip", [])

            # Get the first device from the list
            inverse3_data = inverse3_devices[0] if inverse3_devices else {}
            verse_grip_data = verse_grip_devices[0] if verse_grip_devices else {}

            # Handle the first message to get device IDs and extra information
            if first_message:
                first_message = False

                if not inverse3_data:
                    print("No Inverse3 device found.")
                    break
                if not verse_grip_data:
                    print("No Wireless Verse Grip device found.")

                # Store device ID for sending forces
                inverse3_device_id = inverse3_data.get("device_id")

                # Get handedness from Inverse3 device config data (only available in the first message)
                handedness = inverse3_devices[0].get("config", {}).get("handedness")

                print(f"Inverse3 device ID: {inverse3_device_id}, Handedness: {handedness}")

                if verse_grip_data:
                    # print(f"Wireless Verse Grip device ID: {verse_grip_data.get("device_id")}")
                    pass

            # Extract position, velocity from Inverse3 device state
            position = inverse3_data["state"].get("cursor_position", {})
            velocity = inverse3_data["state"].get("cursor_velocity", {})

            # Extract buttons and orientation from Wireless Verse Grip device state (or default if not found)
            buttons = verse_grip_data.get("state", {}).get("buttons", {})
            orientation = verse_grip_data.get("state", {}).get("orientation", {})

            print(f"Position: {position} Velocity: {velocity} Orientation: {orientation} Buttons: {buttons}")

            # Prepare the force command message to send
            # Must send forces to receive state updates (even if forces are 0)
            request_msg = {
                "inverse3": [
                    {
                        "device_id": inverse3_device_id,
                        "commands": {
                            "set_cursor_force": {
                                "values": force
                            }
                        }
                    }
                ]
            }

            # Send the force command message to the server
            await ws.send(orjson.dumps(request_msg))


# Run the asynchronous main function
if __name__ == "__main__":
    asyncio.run(main())