
import serial
import time
import plotter
import struct
import os

ser = None
output_file = 'output.csv'

def init_BackEnd_Connection(mode='continue'):
    # Initialize serial port once and keep it open
    port = '/dev/cu.usbmodem1101'
    serial_connection = serial.Serial(port, 2000000)
    global ser
    ser = serial_connection

    # Handshake process
    handshake_successful = perform_handshake(mode)
    
    if handshake_successful:
        print("Handshake successful, running pre-setup function.")
    else:
        print("No handshake received.")

def perform_handshake(mode):
    print("Attempting handshake with Arduino...")
    # Wait a moment to ensure the connection is stable
    time.sleep(2)
    
    # Send the handshake character 'C' if continuing old run
    if mode == 'continue':
        ser.write(b'C')
    else: # N if new run
        ser.write(b'N')
    time.sleep(2)
    
    # Wait for an acknowledgment for up to 3 seconds
    start_time = time.time()
    while time.time() - start_time < 3:
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"Received response: {response!r}")  # Always print raw response
            if response == "Continuing old run" or response == 'Starting new run':
                return True
    return False

def send_start_time():
    # Compute and send the startTime for continuation
    file = output_file  # Adjust the filename as needed
    try:
        # Use generate_dfs to split the CSV into experiments
        experiments = plotter.generate_dfs(file)
        
        # Get the last experiment's DataFrame
        if experiments:
            last_experiment_df = experiments[-1]
            last_row = last_experiment_df.iloc[-1]
            unix_time = last_row['unixTime']
            up_time = last_row['upTime']
            start_time = int(unix_time - up_time)
            print(unix_time)
            print(up_time)
            print(start_time)
            # Send startTime to the Arduino as a 4-byte unsigned long
            ser.write(struct.pack('<L', start_time))
            print(f"Sent start time: {start_time}")
        else:
            print("No experiments found in the file.")
        
    except Exception as e:
        print(f"Error in send_start_time: {e}")

# Function to send 'get csv' command to Arduino
# 'file' is output filename
def csv_transfer(file):
    ser.write(b'send\n') # tell arduino to send info here
    with open(file, 'wb') as f:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline()
                if "EOF" in line.decode():  # Check for the end-of-file indicator
                    print("File transfer complete.")
                    break
                f.write(line)

def populate_dropdown():
    # Testing mode
    if ser is None:
        print('No serial connection, plotting test file')
        file = 'output_test.csv'
    else: # there's a connection
        csv_transfer(output_file)
        file = output_file 
    exps = plotter.generate_dfs(file)
    result = []
    for i in range(len(exps) - 1):
        result.append('Experiment ' + str(i + 1))
    result.append('Current Experiment')
    return result
    
def plot_OD(ax, experiment_number, start_time_hours=None, end_time_hours=None):
    # Testing mode
    if ser is None:
        print('No serial connection, plotting test file')
        file = 'output_test.csv'
    else: # there's a connection
        file = output_file
        csv_transfer(file)
    
    # Pass time scale to the plotter function
    plotter.read_and_plot_OD(file, ax, experiment_number, start_time_hours, end_time_hours)

def read_stats():
    if ser is None: # Testing mode
        print('No serial connection')
        file = 'output_test.csv'
    else: # there's a connection
        file = output_file
        csv_transfer(file)
    experiments = plotter.generate_dfs(file)
    stats = experiments[-1].tail(1).squeeze()
    return stats
