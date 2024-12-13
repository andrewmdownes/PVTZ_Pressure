from machine import Pin, ADC, PWM, UART, Timer
import time
import math
from pid_controller import pid

#--------------------------------- PIN INITIALIZATION ---------------------------------#
# UART Configuration
uart_com = UART(2, baudrate=115200, tx=17, rx=16)

# Temperature ADC Pins
temp1_adc = ADC(Pin(26))
temp1_adc.atten(ADC.ATTN_11DB)
temp2_adc = ADC(Pin(27))
temp2_adc.atten(ADC.ATTN_11DB)

# Peltier Control Pins
X_PIN = Pin(15, Pin.OUT, Pin.PULL_DOWN)  # Cooling
Y_PIN = Pin(2, Pin.OUT)                  # Heating
FAN = Pin(5, Pin.OUT)

# Initialize pins to off state
X_PIN.off()
Y_PIN.off()
FAN.off()

PRS_IN = ADC(Pin(13))
ADC_0_REF = ADC(Pin(14))

PRS_IN.atten(ADC.ATTN_11DB)
ADC_0_REF.atten(ADC.ATTN_2_5DB)

PRS_IN.read_uv()
ADC_0_REF.read_uv()

# Bartels Pump Configuration
SHUTDOWN = Pin(22, Pin.OUT)
SHUTDOWN.off()
f_pump = 150
f_peltier = 500
PUMP_CONTROL = PWM(Pin(23), f_pump, duty=380)

# Storage Arrays for averaging
temp_arr = [None] * 200

#--------------------------------- TEMPERATURE FUNCTIONS ---------------------------------#
def calc_Temp(x):
    """Calculate temperature using provided polynomial curve."""
    x_norm = (x - 136.2) / 72.35
    temperature = -2.149 * pow(x_norm, 4) + 0.4015 * pow(x_norm, 3) + 7.324 * pow(x_norm, 2) + 25.43 * x_norm + 33.61
    return temperature

def get_temperature():
    """Read and average temperature measurements."""
    for i in range(0, 200):
        # Read differential voltage between nodes
        node1_v = temp1_adc.read_uv()
        node2_v = temp2_adc.read_uv()
        temp_mv = (node1_v - node2_v) / 1000
        
        # Store in array
        temp_arr[i] = temp_mv
        time.sleep_ms(5)
    
    # Take average and convert to temperature
    temp_avg = sum(temp_arr) / len(temp_arr)
    return calc_Temp(temp_avg)

def control_peltier(pid_error, f_peltier):
    """Control Peltier module based on PID error."""
    if -5 <= pid_error < -2:
        X_PIN.off()
        peltier_pwm_y = PWM(Pin(2), f_peltier, duty=700)
        FAN.on()
        timer = Timer(0)
        timer.init(period=5000, mode=Timer.ONE_SHOT)
    elif pid_error < -5:
        X_PIN.off()
        peltier_pwm_y = PWM(Pin(2), f_peltier, duty=900)
        FAN.on()
    elif 5 >= pid_error > 2:
        Y_PIN.off()
        peltier_pwm_x = PWM(Pin(15), f_peltier, duty=700)
        FAN.on()
    elif pid_error > 5:
        Y_PIN.off()
        peltier_pwm_x = PWM(Pin(15), f_peltier, duty=900)
        FAN.on()
        
def read_pressure(wait_time_us, trials):
    
    total_p_value = 0
    
    for _ in range(trials):
        total_p_value += (PRS_IN.read_uv()-ADC_0_REF.read_uv())
        #print(ADC_0_REF.read_uv()/1E6)
        time.sleep(wait_time_us/1E6)
    
    global pressure_val
    pressure_val = ((total_p_value/trials)/1E6)+0.01
        
button_1 = Pin(34,Pin.IN)
button_2 = Pin(35, Pin.IN)

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
    # Initialize PID controller
    Kp = 1.0
    Ki = 0.1
    Kd = 0.05
    pid_sys = pid.PID(Kp, Ki, Kd)
    
    # Get target temperature from user
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
    
    pid_sys.target = target_temp
    start_time = time.time()
    SHUTDOWN.on()  # Enable pump
    
    # Create data file
    file_name = 'temp_control_data.csv'
    with open(file_name, 'w') as file:
        file.write("Time(s),Temperature(C),Error(C),PID_Output\n")
    
    global pid_error
    pid_error = -10
    
    button_1_index = 0
    
    while True:
        
        print(button_1.value())
        if (button_1.value() == 1):
            time.sleep_ms(20)  #debounce
            
            if (button_1.value() == 1):
                func = button1_functions[button_1_index]
                pid_error = func()
                time.sleep_ms(3000)
                
                if button_1_index >= 2:
                    button_1_index = 0
                    
                else:
                    button_1_index += 1
                    
        # Read current temperature
        current_temp = get_temperature()
        
        # Calculate PID error and output
        #pid_error = pid_sys.target - current_temp
        pid_output = pid_sys(current_temp)
        
        # Get elapsed time
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        # Send data over UART
        data_string = f"{elapsed_time:.2f},{current_temp:.2f},{pid_error:.2f},{f_pump:.2f},{pressure_val:2f}\n"
        uart_com.write(data_string)
        
        # Print current status
        print(f"Time: {elapsed_time:.2f}s")
        print(f"Temperature: {current_temp:.2f}°C")
        print(f"Error: {pid_error:.2f}°C")
        print(f"Pump Freq: {f_pump:.2f}")
        print(f"Pressure: {pressure_val:2f}\n")
        
        # Check if temperature is within tolerance
        if abs(pid_error) <= target_temp * 0.05:
            print("Temperature is within tolerance range")
            continue
        
        # Control Peltier based on PID error
        control_peltier(pid_error, f_peltier)
        
        time.sleep_ms(500)

try:
    main()
except KeyboardInterrupt:
    print("\nProgram interrupted by user")
finally:
    # Cleanup
    SHUTDOWN.off()
    FAN.off()
    X_PIN.off()
    Y_PIN.off()



