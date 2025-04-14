# Program written by Carina Vale
# Audio processing by Kayla Myklebust

import asyncio
import websockets
from datetime import datetime
import json
import binascii
import random
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import soundfile as sf
from numpy.fft import fft, fftfreq
from scipy.signal import stft
from scipy.signal import butter, lfilter
import os
 
# Initialize map and storage
previous_position = None
value = 10000
lines = []
global_max = 10000
global_min = 9000
comp_max = global_max * 1.1
comp_min = global_min * 0.9
map_fig, map_ax = plt.subplots(figsize=(8, 8))
map_ax.set_title("Map with Audio Data Visualization")
map_ax.set_xlabel("X Coordinate (m)")
map_ax.set_ylabel("Y Coordinate (m)")
map_ax.grid(True)
recolor_task = None
 
colormap = plt.cm.coolwarm
norm = mcolors.Normalize(vmin=global_min, vmax=global_max) # fill these with the actual float min/max?
cbar = map_fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=colormap), ax=map_ax, orientation='vertical')
cbar.set_label("Delamination Severity", rotation=270, labelpad=15)
 
def audio_manager(filepath):
    # Check if the file exists at the specified path
    if not os.path.exists(filepath):
        print(f"Error: The file '{filepath}' does not exist.")
        return  # Exit the function if the file is not found
   
    try:
        samples, sample_rate = sf.read(filepath)
    except FileNotFoundError:
        print(f"Error: The file '{filepath}' was not found.")
    except Exception as e:
        print(f"Error occurred: {e}")
       
    fs = 48000
    lowcut = 500        #delam start freq range
    highcut = 5000      #delam end freq range
   
    order = 5
       
    nyq = 0.5 * fs      # nyquist frequency
    band_low = lowcut / nyq
    band_high = highcut / nyq
    b, a = butter(order, [band_low, band_high], btype='band')      #butter imported from scipy
    filtered_samples = lfilter(b, a, samples)     #apply filter to signal
    n = len(filtered_samples)        #number of samples
    fft_result = np.fft.fft(filtered_samples, axis = 0)       #calculate fft
    freq = fftfreq(n, 1/fs)          #generate frequencies for FFT
    normalized_fft = abs(fft_result/n)       #normalize magnitude
    single_normalized_fft = 2* (normalized_fft[0:int((n/2)+1)])     #make data single-sided and double amplitude

    single_normalized_fft = np.array(single_normalized_fft)
    freq = np.array(freq)
 
    sum_mag = 0
    low_index = lowcut
    high_index = 0
    # Assuming f is a numpy array of frequencies
    for i in range(1, len(freq)):  # Start from index 1 to avoid f[i-1] out of bounds
        if ((freq[i] >= lowcut) and (freq[i-1] <= lowcut)):
            low_index = i
        if (freq[i] > lowcut and freq[i] <= highcut and freq[i+1] >=highcut):
            high_index = i                         

    for i in range(low_index, high_index + 1):  # Include high_index
        if (type(single_normalized_fft[i]) == list):
            sum_mag += (single_normalized_fft[i][0])**2 + (single_normalized_fft[i][1])**2  # Sum the FFT magnitudes average
        else:
            sum_mag += single_normalized_fft[i]**2
           
    return float(sum_mag)
 
# def generate_random_float(audio_hex):
#     # global count # testing line recolor
#     # count += 1
#     # if count > 10:
#     #     return 30.0
#     """Simulates converting audio data from hex to a positive float."""
#     try:
#         # Decode the hex audio data into binary
#         audio_data = binascii.unhexlify(audio_hex)
       
#         # Placeholder: Simulate processing the audio data
#         # In the future, this is where you'd analyze 'audio_data' to extract meaningful metrics
#         print("Audio data decoded successfully. (Simulated processing here)")
 
#         # Generate a random float based on audio data
#         random_float = random.uniform(1, 10)
#         return random_float
#     except binascii.Error:
#         print("Error decoding audio hex data.")
#         return 0.0  # Return a default value in case of an error
 
async def recolor_lines(batch_size=100):
    for i in range(0, len(lines), batch_size):
        batch = lines[i:i+batch_size]
        for line, value in batch:
            new_color = colormap(norm(value))
            line.set_color(new_color)
        await asyncio.sleep(0)
    plt.draw()
    print("Lines recolored")
 
def draw_line_with_color(coord1, coord2, value):
    global global_max, global_min, norm, comp_max, comp_min, recolor_task
    """Draw a line between two coordinates with a color based on the float value."""
    if coord1 and coord2:
        if global_max == 10000:
            global_max = value + 0.001
            global_min = value
            comp_max = global_max * 1.1
            comp_min = global_min * 0.9
            print(f"New global min: {global_min}, New global max: {global_max}")
            norm = mcolors.Normalize(vmin=global_min, vmax=global_max)
            cbar.mappable.set_norm(norm)
        elif value > comp_max:
            global_max = value
            comp_max = global_max * 1.1
            print(f"Max: New global min: {global_min}, New global max: {global_max}")
            norm = mcolors.Normalize(vmin=global_min, vmax=global_max)
            cbar.mappable.set_norm(norm)
            if recolor_task and not recolor_task.done():
                recolor_task.cancel()
            recolor_task = asyncio.create_task(recolor_lines())
        elif value < comp_min:
            global_min = value
            comp_min = global_min * 0.9
            print(f"Min: New global min: {global_min}, New global max: {global_max}")
            norm = mcolors.Normalize(vmin=global_min, vmax=global_max)
            cbar.mappable.set_norm(norm)
            if recolor_task and not recolor_task.done():
                recolor_task.cancel()
            recolor_task = asyncio.create_task(recolor_lines())
        color = colormap(norm(value))
        # Plot the line (half a meter thickness roughly equates to width=0.5 in this context)
        line, = map_ax.plot([coord1[0], coord2[0]], [coord1[1], coord2[1]], color=color, linewidth=1.0)
        map_ax.scatter(coord1[0], coord1[1], color=color, s=10)  # Mark the start point
        lines.append((line, value))
        if len(lines) % 10 == 0:
            #plt.draw()
            plt.pause(0.01)
        print(f"Line drawn between {coord1} and {coord2} with value {value:.2f}.")
 
async def save_audio(websocket):
    global previous_position, value
    try:
        async for message in websocket:
            print("Message received from sender.")
            payload = json.loads(message)
            current_position = payload.get("position_before", (None, None))
            current_position = (-1*current_position[1], current_position[0])
            audio_hex = str(payload.get("audio", ""))
            audio_binary = bytes.fromhex(audio_hex)
            with open("received.wav", "wb") as f:
                f.write(audio_binary)
            
            if previous_position is not None:
                draw_line_with_color(previous_position, current_position, value)
                await asyncio.sleep(0)
                print(f"Float generated from audio data: {value:.2f}")
            previous_position = current_position  # Update for the next packet
            value = audio_manager("C:/Users/cathe/Desktop/received.wav")#"C:/Users/alsha/Downloads/received.wav") # UPDATE TO PROPER PATH
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")
 
async def main():
    server = await websockets.serve(save_audio, "0.0.0.0", 8764)  # Replace with your server address and port
    print("Server started. Waiting for audio data...")
    plt.ion() #<-- this as show causes a problem
    plt.pause(0.1)
    while True:
        await asyncio.sleep(0.1)
        plt.pause(0.01)
    await server.wait_closed()
 
if __name__ == "__main__":
    asyncio.run(main())
