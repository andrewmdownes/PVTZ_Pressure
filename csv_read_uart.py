import serial
import csv
import time

# Configure the serial connection
ser = serial.Serial('COM4', 115200) 

# Open the CSV file
csv_filename = 'C:\\Users\Mason\OneDrive - University of Florida\Documents\Research_fall2024\PT_data.csv'

print(f"Receiving data and writing to {csv_filename}...")
print("Press Ctrl+C to stop.")

try:
    with open(csv_filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['Time(s)', 'Temperature(C)', 'Pressure(PSI)'])  # Write header
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                data = line.split(',')
                if len(data) == 3:
                    csvwriter.writerow(data)
                    csvfile.flush()  # Ensure data is written immediately to the file
                    print(f"New entry: Time: {data[0]}s, Temp: {data[1]}°C, Pressure: {data[2]} PSI")
            
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage

except KeyboardInterrupt:
    print("\nData collection stopped.")
finally:
    ser.close()
    print("Serial connection closed.")
