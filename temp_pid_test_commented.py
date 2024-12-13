# Temperature Control System for Peltier Module
# Controls temperature using PID and manages communication over UART

from machine import Pin, ADC, PWM, UART, Timer
import time
import math
from pid_controller import pid

# Setup hardware pins and interfaces
#--------------------------------- PIN INITIALIZATION ---------------------------------#
# Configure UART for data communication at 115200 baud
uart_com = UART(2, baudrate=115200, tx=17, rx=16)

# Setup ADC pins for temperature sensors with 11dB attenuation
temp1_adc = ADC(Pin(26))
temp1_adc.atten(ADC.ATTN_11DB)
temp2_adc = ADC(Pin(27))
temp2_adc.atten(ADC.ATTN_11DB)

# Configure pins for Peltier module control
X_PIN = Pin(15, Pin.OUT, Pin.PULL_DOWN)  # Cooling side control
Y_PIN = Pin(2, Pin.OUT)                  # Heating side control
FAN = Pin(5, Pin.OUT)                    # Cooling fan control

# Initialize all control pins to off state
X_PIN.off()
Y_PIN.off()
FAN.off()

# Setup pressure sensor ADCs
PRS_IN = ADC(Pin(13))
ADC_0_REF = ADC(Pin(14))

PRS_IN.atten(ADC.ATTN_11DB)
ADC_0_REF.atten(ADC.ATTN_2_5DB)

# Initial readings
PRS_IN.read_uv()
ADC_0_REF.read_uv()

# Setup Bartels pump control
SHUTDOWN = Pin(22, Pin.OUT)
SHUTDOWN.off()
f_pump = 150          # Pump frequency
f_peltier = 500       # Peltier PWM frequency
PUMP_CONTROL = PWM(Pin(23), f_pump, duty=380)

# Array for temperature averaging
temp_arr = [None] * 200

#--------------------------------- TEMPERATURE FUNCTIONS ---------------------------------#
def calc_Temp(x):
    """Convert sensor reading to temperature using calibration curve"""
    x_norm = (x - 136.2) / 72.35
    temperature = -2.149 * pow(x_norm, 4) + 0.4015 * pow(x_norm, 3) + 7.324 * pow(x_norm, 2) + 25.43 * x_norm + 33.61
    return temperature

def get_temperature():
    """Sample temperature 200 times and return averaged reading"""
    for i in range(0, 200):
        # Get differential temperature reading
        node1_v = temp1_adc.read_uv()
        node2_v = temp2_adc.read_uv()
        temp_mv = (node1_v - node2_v) / 1000
        
        temp_arr[i] = temp_mv
        time.sleep_ms(5)
    
    temp_avg = sum(temp_arr) / len(temp_arr)
    return calc_Temp(temp_avg)

def control_peltier(pid_error, f_peltier):
    """Adjust Peltier module based on PID error magnitude and direction"""
    if -5 <= pid_error < -2:             # Small heating needed
        X_PIN.off()
        peltier_pwm_y = PWM(Pin(2), f_peltier, duty=700)
        FAN.on()
        timer = Timer(0)
        timer.init(period=5000, mode=Timer.ONE_SHOT)
    elif pid_error < -5:                 # Large heating needed
        X_PIN.off()
        peltier_pwm_y = PWM(Pin(2), f_peltier, duty=900)
        FAN.on()
    elif 5 >= pid_error > 2:            # Small cooling needed
        Y_PIN.off()
        peltier_pwm_x = PWM(Pin(15), f_peltier, duty=700)
        FAN.on()
    elif pid_error > 5:                 # Large cooling needed
        Y_PIN.off()
        peltier_pwm_x = PWM(Pin(15), f_peltier, duty=900)
        FAN.on()
        
def read_pressure(wait_time_us, trials):
    """Read and average pressure sensor over multiple trials"""
    total_p_value = 0
    
    for _ in range(trials):
        total_p_value += (PRS_IN.read_uv()-ADC_0_REF.read_uv())
        time.sleep(wait_time_us/1E6)
    
    global pressure_val
    pressure_val = ((total_p_value/trials)/1E6)+0.01
        
# Setup control buttons
button_1 = Pin(34,Pin.IN)
button_2 = Pin(35, Pin.IN)

# Button function definitions
def func1():
    print("fast cool")
    return -10
    
def func2():
    print("slow cool")
    return -1
    
def func3():
    print("room temp")
    return 0
    
button1_functions = [func1, func2, func3]

#--------------------------------- MAIN PROGRAM ---------------------------------#
def main():
    # Setup PID controller with tuning parameters
    Kp = 1.0
    Ki = 0.1
    Kd = 0.05
    pid_sys = pid.PID(Kp, Ki, Kd)
    
    # Get user input for target temperature
    print("Enter target temperature (4-40°C):")
    try:
        target_temp = float(input())
        if not (4 <= target_temp <= 40):
            print("Temperature must be between 4°C and 40°C")
            return
    except ValueError:
        print("Invalid input. Please enter a number.")
        return
    
    read_pressure(10, 100)
    
    # Initialize system
    pid_sys.target = target_temp
    start_time = time.time()
    SHUTDOWN.on()  # Start pump
    
    # Create data logging file
    file_name = 'temp_control_data.csv'
    with open(file_name, 'w') as file:
        file.write("Time(s),Temperature(C),Error(C),PID_Output\n")
    
    global pid_error
    pid_error = -10
    
    button_1_index = 0
    
    # Main control loop
    while True:
        # Handle button input for manual control
        print(button_1.value())
        if (button_1.value() == 1):
            time.sleep_ms(20)  # Debounce protection
            
            if (button_1.value() == 1):
                func = button1_functions[button_1_index]
                pid_error = func()
                time.sleep_ms(3000)
                
                if button_1_index >= 2:
                    button_1_index = 0
                else:
                    button_1_index += 1
                    
        # Get current temperature and calculate control response
        current_temp = get_temperature()
        pid_output = pid_sys(current_temp)
        
        # Calculate elapsed time
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        # Send data over UART
        data_string = f"{elapsed_time:.2f},{current_temp:.2f},{pid_error:.2f},{f_pump:.2f},{pressure_val:2f}\n"
        uart_com.write(data_string)
        
        # Display current system status
        print(f"Time: {elapsed_time:.2f}s")
        print(f"Temperature: {current_temp:.2f}°C")
        print(f"Error: {pid_error:.2f}°C")
        print(f"Pump Freq: {f_pump:.2f}")
        print(f"Pressure: {pressure_val:2f}\n")
        
        # Check if temperature is within target range
        if abs(pid_error) <= target_temp * 0.05:
            print("Temperature is within tolerance range")
            continue
        
        # Adjust Peltier module based on current error
        control_peltier(pid_error, f_peltier)
        
        time.sleep_ms(500)

# Main program execution with cleanup handling
try:
    main()
except KeyboardInterrupt:
    print("\nProgram interrupted by user")
finally:
    # Ensure all outputs are disabled on exit
    SHUTDOWN.off()
    FAN.off()
    X_PIN.off()
    Y_PIN.off()