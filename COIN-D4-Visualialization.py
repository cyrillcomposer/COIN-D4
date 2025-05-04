import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import collections
import time

# --- Configuration ---
SERIAL_PORT = 'COM8'  # Change this to your ESP32's COM port
SERIAL_BAUD = 115200  # Must match SERIAL_BAUD in Arduino code
PLOT_RANGE_METERS = 2  # <---- ADJUST THIS: Set the plot range in meters (1-10)
MAX_POINTS_HISTORY = 1440 # Store roughly 2 full rotations (adjust as needed)
PLOT_INTERVAL_MS = 30   # How often to update the plot (in milliseconds)

# Validate plot range
if PLOT_RANGE_METERS < 1 or PLOT_RANGE_METERS > 10:
    raise ValueError("PLOT_RANGE_METERS must be between 1 and 10")

# Calculate max distance in mm based on plot range
MAX_DISTANCE_MM = PLOT_RANGE_METERS * 1000

# --- Distance and Intensity Thresholds ---
MIN_DISTANCE_MM = 50  # Filter out distances below 50mm (likely artifacts)
INTENSITY_THRESHOLD = 100 # <---- Adjust this value

# --- Global Variables ---
ser = None # Serial object
lidar_data = collections.deque(maxlen=MAX_POINTS_HISTORY) # Store (angle_rad, dist_m, intensity) tuples

# --- Plot Setup ---
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Explicitly set the radial limits (view range) from 0 to max distance in meters
max_distance_m = MAX_DISTANCE_MM / 1000.0
ax.set_rlim(0, max_distance_m)

ax.set_theta_zero_location("N")  # Set 0 degrees to the top (North)
ax.set_theta_direction(-1)  # Set angles to increase clockwise
ax.set_title(f"LIDAR Data (Range: {PLOT_RANGE_METERS}m, Min: {MIN_DISTANCE_MM}mm, Intensity >= {INTENSITY_THRESHOLD})")

# Disable default grid
ax.grid(False)

# Create custom grid lines
# First, create thin lines every 10cm (0.1m)
for radius in np.arange(0.1, max_distance_m + 0.1, 0.1):
    if radius <= max_distance_m:  # Only draw within the plot range
        circle = plt.Circle((0, 0), radius, transform=ax.transData._b, 
                           fill=False, color='gray', linewidth=0.5, alpha=0.3)
        ax.add_artist(circle)

# Then, create thick lines every 1m
for radius in np.arange(1.0, max_distance_m + 1.0, 1.0):
    if radius <= max_distance_m:  # Only draw within the plot range
        circle = plt.Circle((0, 0), radius, transform=ax.transData._b, 
                           fill=False, color='gray', linewidth=2.0, alpha=0.7)
        ax.add_artist(circle)

# Set radial ticks with labels only at 1m intervals
tick_range = np.arange(0, max_distance_m + 1, 1)
ax.set_yticks(tick_range) # Ticks at 0, 1, 2, ..., up to max_distance_m
ax.set_yticklabels([f'{int(r)}m' for r in tick_range])

# Keep the angular grid
ax.grid(True, axis='x', alpha=0.3)  # Only angular grid lines

# Initialize scatter plot (empty at first)
scatter = ax.scatter([], [], s=5, c=[], cmap='viridis', vmin=0, vmax=255, alpha=0.75)

# --- Serial Communication ---
def connect_serial():
    """Tries to connect to the serial port."""
    global ser
    while True:
        try:
            print(f"Attempting to connect to {SERIAL_PORT} at {SERIAL_BAUD} baud...")
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            print(f"Connected to {SERIAL_PORT}")
            ser.reset_input_buffer() # Flush input buffer
            return True
        except serial.SerialException as e:
            print(f"Error connecting: {e}")
            print("Retrying in 5 seconds...")
            time.sleep(5)
        except KeyboardInterrupt:
            print("Connection attempt cancelled by user.")
            return False

def read_serial_data():
    """Reads and parses data from the serial port, adding it to lidar_data."""
    if ser and ser.is_open:
        try:
            lines_read = 0
            max_lines_per_update = 50 # Limit reads per cycle to keep UI responsive
            while ser.in_waiting > 0 and lines_read < max_lines_per_update:
                line = ser.readline()
                lines_read += 1
                if not line:
                    continue

                try:
                    data_str = line.decode('utf-8', errors='ignore').strip()
                    parts = data_str.split(',')

                    if len(parts) == 3:
                        angle_deg = float(parts[0])
                        distance_mm = int(parts[1])
                        intensity = int(parts[2])

                        angle_rad = np.radians(angle_deg)
                        distance_m = distance_mm / 1000.0

                        # --- Filtering Logic ---
                        # Add valid data ONLY if:
                        # 1. Distance is above minimum threshold (50mm)
                        # 2. Distance is within the desired radius
                        # 3. Intensity meets the threshold
                        if distance_mm >= MIN_DISTANCE_MM and distance_m <= (MAX_DISTANCE_MM / 1000.0) and intensity >= INTENSITY_THRESHOLD:
                            lidar_data.append((angle_rad, distance_m, intensity))
                        # Optional: else print discarded points for debugging
                        # else:
                        #     if distance_mm < MIN_DISTANCE_MM:
                        #         print(f"Discarded point: distance {distance_mm}mm < {MIN_DISTANCE_MM}mm")
                        #     elif intensity < INTENSITY_THRESHOLD:
                        #         print(f"Discarded point: intensity {intensity} < {INTENSITY_THRESHOLD}")

                except ValueError:
                    # print(f"Could not parse: {line}") # Optional debug
                    pass
                except Exception as e:
                    print(f"Error processing line: {line} - {e}")

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            ser.close()
            connect_serial() # Attempt to reconnect
        except IOError:
             print("IOError reading from serial port.")
             ser.close()
             connect_serial() # Attempt to reconnect
        except Exception as e:
            print(f"An unexpected error occurred during serial reading: {e}")

# --- Animation Update Function ---
def update_plot(frame):
    """Called periodically by the animation driver."""
    read_serial_data()

    if lidar_data:
        # Unpack data efficiently
        angles, distances, intensities = zip(*lidar_data)

        # Prepare data for scatter plot (Nx2 array for offsets)
        plot_data = np.vstack([angles, distances]).T
        scatter.set_offsets(plot_data)
        # Update colors based on intensity
        scatter.set_array(np.array(intensities))
    else:
        # Clear plot if no data meets criteria
        scatter.set_offsets(np.empty((0, 2)))
        scatter.set_array(np.empty((0,)))

    return scatter,

# --- Main Execution ---
if __name__ == "__main__":
    if connect_serial():
        # Update the plot title one time initially
        ax.set_title(f"LIDAR Data (Range: {PLOT_RANGE_METERS}m, Min: {MIN_DISTANCE_MM}mm, Intensity >= {INTENSITY_THRESHOLD})")

        ani = animation.FuncAnimation(fig, update_plot, interval=PLOT_INTERVAL_MS, blit=True, cache_frame_data=False)
        try:
            plt.show()
        except Exception as e:
            print(f"Error displaying plot: {e}")
        finally:
            if ser and ser.is_open:
                ser.close()
                print("Serial port closed.")
    else:
        print("Could not connect to serial port. Exiting.")
