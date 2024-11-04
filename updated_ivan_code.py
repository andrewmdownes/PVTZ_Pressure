from machine import Pin, ADC, PWM, UART
import time
import math

#---------------------------------PINS & VARIABLES----------------------------------#

# Set pump frequency to 150 Hz
pump_f = 150

# Initialize pump sThutdown pin
pump_sd = Pin(22, Pin.OUT)
pump_sd.off()

# Initialize pump frequency and amplitude
pump_ctrl = PWM(Pin(23), pump_f, duty=380)
pump_sd.on()

# Pressure ADC Pins
press_adc = ADC(Pin(13))
press_adc.atten(ADC.ATTN_11DB)

# Temperature ADC Pins
temp1_adc = ADC(Pin(26))
temp1_adc.atten(ADC.ATTN_11DB)
temp2_adc = ADC(Pin(27))
temp2_adc.atten(ADC.ATTN_11DB)

# Motor Pins
mtr_en = Pin(2, Pin.OUT)
mtr_en.off()
mtr_stp = PWM(Pin(4), freq=800, duty=0)
mtr_dir = Pin(5, Pin.OUT)

# E-Stop Pin
limit_switch = Pin(18, Pin.IN, Pin.PULL_UP)

# Target Pressure
target_pressure = 29.4

# Storage Arrays
pres_arr = [None] * 200
temp_arr = [None] * 200

# Data File
file_name = 'PT_data.csv'

uart_com = UART(2, baudrate=115200, tx=17, rx=16)

#-------------------------------------FUNCTIONS------------------------------------#

# Calculate the pressure from the transducer
def calc_Pressure(x):
    pressure = 14.696 * abs(3.321 * pow(10, -3) * x - 0.1057)
    return pressure

# Calculate the temperature using a polynomial curve
def calc_Temp(x):
    x_norm = (x - 136.2) / 72.35
    temperature = -2.149 * pow(x_norm, 4) + 0.4015 * pow(x_norm, 3) + 7.324 * pow(x_norm, 2) + 25.43 * x_norm + 33.61
    return temperature

# Move the syringe up or down to pressurize or depressurize volume
def move_Syringe(target_pressure, current_pressure):    
    if (abs(target_pressure - current_pressure) / target_pressure < 0.01) or limit_switch.value():
        mtr_stp.duty(0)
        return
    
    if current_pressure < target_pressure:
        mtr_dir.value(0)
    else:
        mtr_dir.value(1)
        
    mtr_stp.duty(512)
        
    freq = int(1100 * abs(target_pressure - current_pressure) / target_pressure + 100)
    mtr_stp.freq(freq)
    


# Reset Syringe
#def reset_Syringe

# Debug motor
def debug_Motor(direction):
    mtr_dir.value(direction)
    mtr_stp.duty(512)
    
# Create the CSV file and write the header
'''
with open(file_name, 'w') as file:
    file.write("Time(s),Temperature(C),Pressure(PSI)\n")
'''
# Get the start time
start_time = time.time()

#-----------------------------------MAIN LOOP-------------------------------------#

while True:
    
    for i in range(0, 200):
        # Read the individual ADC inputs
        pres_mv = press_adc.read_uv() / 1000
        node1_v = temp1_adc.read_uv()
        node2_v = temp2_adc.read_uv()
        temp_mv = (node1_v - node2_v) / 1000
        
        # Store in array
        pres_arr[i] = pres_mv
        temp_arr[i] = temp_mv
        time.sleep_ms(5)
    
    # Take the average over 50 readings
    pres_avg = sum(pres_arr) / len(pres_arr)
    temp_avg = sum(temp_arr) / len(temp_arr)
    
    # Calculate the elapsed time
    current_time = time.time()
    elapsed_time = current_time - start_time
    
    # Convert from voltage readings to real-world
    pressure = calc_Pressure(pres_avg)
    temperature = calc_Temp(temp_avg)
    '''
    with open(file_name, 'a') as file:
        file.write("{},{:.2f},{:.2f}\n".format(elapsed_time, temperature, pressure))
    '''
        
    data_string = f"{elapsed_time},{temperature:.2f},{pressure:.2f}\n"
    uart_com.write(data_string)
    
    # Move Syringe to Adjust Pressure
    move_Syringe(target_pressure, pressure)
    #debug_Motor(1)
    
    print(data_string)

    # Print Readings
    '''print("Pressure:\t", pressure, "PSI")
    print("Temperature:\t", temperature, "C\n")
    '''
    time.sleep_ms(500)
    
