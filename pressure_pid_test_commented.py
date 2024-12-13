# Pressure and Temperature Control System with PID and UART Communication
from machine import ADC, Pin, PWM, freq, Timer, UART
from time import sleep, time
import math
import esp
#import esp32
import inspect
from pid_controller import pid
import threading
import datetime

# Setup UART communication
uart_com = UART(2, baudrate=115200, tx=17, rx=16)

# Motor control pins initialization
MTR_EN = Pin(2, Pin.OUT)      # Motor enable
MTR_EN.off()                  # Start with motor disabled
MTR_STP = PWM(Pin(4))        # Motor step control
MTR_STP.freq(300)            # Set initial frequency
MTR_STP.duty(0)              # Start with duty cycle 0

MTR_DIR = Pin(5, Pin.OUT)     # Motor direction control

# ADC setup for sensors
PRS_IN = ADC(Pin(13))         # Pressure sensor input
ADC_0_REF = ADC(Pin(14))      # Reference voltage
TMP_IN = ADC(Pin(33))         # Temperature sensor input

# Configure ADC attenuation
PRS_IN.atten(ADC.ATTN_11DB)
ADC_0_REF.atten(ADC.ATTN_2_5DB)

# Solenoid control setup
SOL_EN = Pin(12, Pin.OUT)          # Main solenoid
SOL_EN_probe = Pin(19, Pin.OUT)    # Probe solenoid
SOL_EN.off()                       # Initialize solenoids off
SOL_EN_probe.off()

# Initial ADC readings
PRS_IN.read_uv()
ADC_0_REF.read_uv()

# Limit switch for safety
LIMIT_SW = Pin(18, Pin.IN, Pin.PULL_UP)

# System variables
pressure_val = 0              # Current pressure
temp_val = 0                 # Current temperature
target_p_val = 0.35         # Target pressure

MTR_RUN = 0                 # Motor status flag

# Frequency limits
F_MIN = 150
F_MAX = 1000

# Shutdown control
Shutdown = Pin(22,Pin.OUT)

# PWM setup
f = F_MIN                   # Current frequency
delta_f = 1                 # Frequency step
p = PWM(Pin(23), f ,duty = 200)    # PWM control pin

start_time = time()         # Record start time

def start_up(time_s, frequency):
    """Initialize system and move to home position"""
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
    """Read and average pressure sensor values"""
    total_p_value = 0
    
    for _ in range(trials):
        total_p_value += (PRS_IN.read_uv()-ADC_0_REF.read_uv())
        sleep(wait_time_us/1E6)
    
    global pressure_val
    pressure_val = ((total_p_value/trials)/1E6)+0.01
    
def read_temperature(wait_time_us, trials):
    """Read and average temperature sensor values"""
    total_t_value = 0
    
    for _ in range(trials):
        total_t_value += (TMP_IN.read_uv()-ADC_0_REF.read_uv())
        sleep(wait_time_us/1E6)
    
    global temp_val
    temp_val = ((total_t_value/trials)/1E6)+0.01

def enable_motor(direction, period_ms):
    """Enable motor with specified direction and duration"""
    MTR_EN.off()
    MTR_DIR.value(direction)
    MTR_STP.duty(512)  # 50% duty cycle
    timer = Timer(0)
    timer.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_motor)
    global MTR_RUN
    MTR_RUN = True

def disable_motor(timer):
    """Disable motor and reset control signals"""
    global MTR_RUN
    MTR_RUN = False
    print("Motor Disabled")
    MTR_STP.duty(0)
    MTR_EN.off()

def enable_sol(period_ms):
    """Enable solenoids for specified duration"""
    timer2 = Timer(1)
    timer2.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_sol)
    SOL_EN.on()
    SOL_EN_probe.on()

def disable_sol(timer2):
    """Disable both solenoids"""
    SOL_EN.off()
    SOL_EN_probe.off()

# Configure PID controller
Kp = 1.5      # Proportional gain
Ki = 0.05     # Integral gain
Kd = 0.1      # Derivative gain
pid_sys = pid.PID(Kp, Ki, Kd)
pid_sys.target = target_p_val

def main():
    """Main control loop"""
    start_up(3, 600)        # Initialize system
    
    MTR_STP.freq(300)       # Set operating frequency
    
    valid = True
    while valid:
        # Update PWM frequency
        p.freq(f)
        sleep(10 / F_MIN)
        Shutdown.on()
        
        # Read sensor values
        read_pressure(10, 100)
        read_temperature(10, 100)
        print(pressure_val)
        
        # Calculate atmospheric pressure
        atm_pressure = (pressure_val - 0.275) / (0.529 - 0.275) + 1

        # Calculate and send timing data over UART
        current_time = time()
        elapsed_time = current_time - start_time
        data_string = f"{elapsed_time},{temp_val:.2f},{atm_pressure:.2f}\n"
        uart_com.write(data_string)
        print(data_string)

        # Calculate control error
        error = pid_sys.target - pressure_val
        
        # Check if within tolerance
        if abs(error) <= target_p_val * 0.02:
            print("Pressure is within tolerance range.")
            continue

        # Get PID control output
        control_output = pid_sys(pressure_val)

        # Apply control action
        if control_output < 0:
            if LIMIT_SW.value() != 1:
                # Increase pressure using motor
                enable_motor(0, int(abs(control_output) * 1000))
            else:
                pass
        elif control_output > 0:
            # Decrease pressure using solenoid
            enable_sol(int(abs(control_output) * 1000))

main()