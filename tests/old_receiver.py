import asyncio
import websockets
import json
import binascii
import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import signal

# WebSocket Server Host and Port
host = "127.0.0.1"  # Listen on all available interfaces
port = 8764  # Ensure this matches the sender

# Initialize map and storage
previous_position = None
value = 1.0 # change
map_fig, map_ax = plt.subplots(figsize=(8, 8))
map_ax.set_title("Map with Audio Data Visualization")
map_ax.set_xlabel("X Coordinate")
map_ax.set_ylabel("Y Coordinate")
map_ax.grid(True)

def generate_random_float(audio_hex):
    """Simulates converting audio data from hex to a positive float."""
    try:
        # Decode the hex audio data into binary
        audio_data = binascii.unhexlify(audio_hex)
        
        # Placeholder: Simulate processing the audio data
        # In the future, this is where you'd analyze 'audio_data' to extract meaningful metrics
        print("Audio data decoded successfully. (Simulated processing here)")

        # Generate a random float based on audio data
        random_float = random.uniform(1, 10)
        return random_float
    except binascii.Error:
        print("Error decoding audio hex data.")
        return 0.0  # Return a default value in case of an error

def draw_line_with_color(coord1, coord2, value):
    """Draw a line between two coordinates with a color based on the float value."""
    if coord1 and coord2:
        # Normalize the float value to a color on the grayscale spectrum (lighter = higher value)
        norm = mcolors.Normalize(vmin=1, vmax=10)
        color = mcolors.to_rgba(plt.cm.gray(norm(value)))  # Use grayscale color

        # Plot the line (half a meter thickness roughly equates to width=0.5 in this context)
        map_ax.plot([coord1[0], coord2[0]], [coord1[1], coord2[1]], color=color, linewidth=0.5)
        map_ax.scatter(coord1[0], coord1[1], color=color, s=10)  # Mark the start point
        plt.draw()
        print(f"Line drawn between {coord1} and {coord2} with value {value:.2f}.")

async def handle_client(websocket):
    global previous_position
    print("Client connected...")
    try:
        async for message in websocket:
            print("Message received from sender.")
            # Parse the received message
            payload = json.loads(message)

            # Extract x, y coordinates and audio data
            current_position = payload.get("position_before", (None, None))
            audio_hex = payload.get("audio", "")

            # Decode hex audio data (not used for now, but saved for reference)
            # try:
            #     audio_data = binascii.unhexlify(audio_hex)
            #     with open("received_audio.wav", "wb") as audio_file:
            #         audio_file.write(audio_data)
            #     print("Audio data successfully saved as 'received_audio.wav'.")
            # except binascii.Error:
            #     print("Error decoding audio hex data.")
            #     audio_data = None

            # Draw on the map using the current and previous position
            if previous_position is not None:
                draw_line_with_color(previous_position, current_position, value)
                # Generate a float from the received audio hex data
                value = generate_random_float(audio_hex)
                print(f"Float generated from audio data: {value:.2f}")
            previous_position = current_position  # Update for the next packet

    except websockets.exceptions.ConnectionClosed:
        print(f"Connection closed")
    except Exception as e:
        print(f"Error: {e}")

async def main():
    server = await websockets.serve(handle_client, host, port)
    print(f"Starting WebSocket server on {host}:{port}...")

    plt.show()
    await server.wait_closed()
    #async with websockets.serve(handle_client, host, port):
    #    plt.show()  # Keep the map visualization open
    #    await asyncio.Future()  # Run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram terminated by user (Ctrl+C). Goodbye!")
