
from machine import ADC, Pin, PWM, freq, Timer, UART
from time import sleep, time
import math
import esp
#import esp32
import inspect
from pid_controller import pid
import threading
import datetime

uart_com = UART(2, baudrate=115200, tx=17, rx=16)

MTR_EN = Pin(2, Pin.OUT)
MTR_EN.off()
MTR_STP = PWM(Pin(4))
MTR_STP.freq(300)
MTR_STP.duty(0)

MTR_DIR = Pin(5, Pin.OUT)

PRS_IN = ADC(Pin(13))
ADC_0_REF = ADC(Pin(14))
TMP_IN = ADC(Pin(33))

PRS_IN.atten(ADC.ATTN_11DB)
ADC_0_REF.atten(ADC.ATTN_2_5DB)

SOL_EN = Pin(12, Pin.OUT)
SOL_EN_probe = Pin(19, Pin.OUT)
SOL_EN.off()
SOL_EN_probe.off()

PRS_IN.read_uv()
ADC_0_REF.read_uv()

LIMIT_SW = Pin(18, Pin.IN, Pin.PULL_UP)

#AMP_PIN = Pin(25, Pin.OUT)

pressure_val = 0
temp_val = 0
#target_p_val = 0.529
target_p_val = 0.35

MTR_RUN = 0

F_MIN = 150
F_MAX = 1000

Shutdown = Pin(22,Pin.OUT)

f = F_MIN
delta_f = 1
#x = PWM(Pin(18), f,duty = 380) #max 403 for bartels DONT MIND THIS CODE ITS FOR THERMOELECTRIC

p = PWM(Pin(23), f ,duty = 200) #max 403

start_time = time()

def start_up(time_s, frequency):
    
    MTR_DIR.value(0)
    MTR_STP.freq(frequency)
    MTR_STP.duty(512)
    
    while True:
        
        LIMIT_SW_VAL = LIMIT_SW.value()
        
        if LIMIT_SW_VAL == 1:
            
            enable_motor(1, time_s*1000)
            
            sleep(time_s)
            
            return

def read_pressure(wait_time_us, trials):
    
    total_p_value = 0
    
    for _ in range(trials):
        total_p_value += (PRS_IN.read_uv()-ADC_0_REF.read_uv())
        #print(ADC_0_REF.read_uv()/1E6)
        sleep(wait_time_us/1E6)
    
    global pressure_val
    pressure_val = ((total_p_value/trials)/1E6)+0.01
    
def read_temperature(wait_time_us, trials):
    
    total_t_value = 0
    
    for _ in range(trials):
        total_t_value += (TMP_IN.read_uv()-ADC_0_REF.read_uv())
        #print(ADC_0_REF.read_uv()/1E6)
        sleep(wait_time_us/1E6)
    
    global temp_val
    temp_val = ((total_t_value/trials)/1E6)+0.01
        

def enable_motor(direction, period_ms):
    MTR_EN.off()
    MTR_DIR.value(direction)
    MTR_STP.duty(512)  # Set the duty cycle to 50%
    timer = Timer(0)
    timer.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_motor)
    #print("trying")
    global MTR_RUN
    MTR_RUN = True

def disable_motor(timer):
    global MTR_RUN
    MTR_RUN = False
    print("Motor Disabled")
    MTR_STP.duty(0)
    MTR_EN.off()


def enable_sol(period_ms):
 
    timer2 = Timer(1)
    timer2.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_sol)
    SOL_EN.on()
    SOL_EN_probe.on()


def disable_sol(timer2):
    SOL_EN.off()
    SOL_EN_probe.off()
    
'''    
def record_data(sleep_interval_s, p_val):
    
    csv_file_path = f"C:/Users/Mason/OneDrive - University of Florida/Documents/Senior_Design/test_1.csv"

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(csv_file_path, "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([timestamp, p_val])
        
    sleep(sleep_interval_s)  # Record data every 500 ms

    threading.Thread(target=record_data).start()
'''
# Initialize PID controller
Kp = 1.5 # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.1  # Derivative gain
pid_sys = pid.PID(Kp, Ki, Kd)
pid_sys.target = target_p_val


def main():
    
   
    start_up(3, 600)
    
    MTR_STP.freq(300)
    
    valid = True
    # Convert to use timers to measure ADC every second or so
    while valid:
        
        p.freq(f)
     #  x.freq(f)
        sleep(10 / F_MIN)
        Shutdown.on()
        
        read_pressure(10, 100)
        read_temperature(10, 100)
        print(pressure_val)
        atm_pressure = (pressure_val - 0.275) / (0.529 - 0.275) + 1
        # print("P: ", atm_pressure, " atm")
        # print("T (V): ", temp_val)

        # Calculate elapsed time
        current_time = time()
        elapsed_time = current_time - start_time
        
        # Format and send data over UART
        data_string = f"{elapsed_time},{temp_val:.2f},{atm_pressure:.2f}\n"
        uart_com.write(data_string)
        print(data_string)  # Also print to console for debugging
        
        #record_data(0.5, pressure_val)

        # Calculate the error
        error = pid_sys.target - pressure_val
        
        # Check if pressure is within 2% tolerance range
        if abs(error) <= target_p_val * 0.02:
            print("Pressure is within tolerance range.")
            continue

        # Get the control output from the PID controller
        control_output = pid_sys(pressure_val)

        # Determine action based on control output
        if control_output < 0:
            #continue
            if LIMIT_SW.value() != 1:
            # Enable motor to increase pressure
                enable_motor(0, int(abs(control_output) * 1000))  # Adjust motor duration based on control output
            else:
                pass
                # print("Limit Reached")
                # Shutdown.off()
                # valid = False
            
        elif control_output > 0:
            # Enable solenoid to decrease pressure
            enable_sol(int(abs(control_output) * 1000))  # Adjust solenoid duration based on control output


main()

'''                
try:
    main()
    
except KeyboardInterrupt:
    print("Program interrupted by user.")

finally:
    #AMP_PIN.off()
    p.off()
    Shutdown.off()
'''
'''
def main():
    
    while True:
        
        read_pressure(10, 100)
        print(pressure_val)
        
        if pressure_val < target_p_val:
            enable_motor(1, 1000)
             
        elif pressure_val > target_p_val + 0.05:
            enable_sol(1000)
        
        
main()
'''      





