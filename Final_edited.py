from machine import ADC, Pin, PWM, freq, Timer
from time import sleep
import math
import esp
import inspect
from pid_controller import pid
import threading
import datetime

# Set up motor control pins
MTR_EN = Pin(2, Pin.OUT)  # Motor enable pin
MTR_EN.off()  # Disable motor by default
MTR_STP = PWM(Pin(4))  # Motor step pin (PWM)
MTR_STP.freq(300)  # Set motor step frequency to 300 Hz
MTR_STP.duty(0)  # Set motor step duty cycle to 0 (stopped)

MTR_DIR = Pin(5, Pin.OUT)  # Motor direction pin

# Set up pressure and temperature sensor pins
PRS_IN = ADC(Pin(13))  # Pressure input pin (ADC)
ADC_0_REF = ADC(Pin(14))  # ADC reference pin (ADC)
TMP_IN = ADC(Pin(33))  # Temperature input pin (ADC)

PRS_IN.atten(ADC.ATTN_11DB)  # Set pressure input attenuation to 11 dB
ADC_0_REF.atten(ADC.ATTN_2_5DB)  # Set ADC reference attenuation to 2.5 dB

# Set up solenoid control pins
SOL_EN = Pin(12, Pin.OUT)  # Solenoid enable pin
SOL_EN_probe = Pin(19, Pin.OUT)  # Solenoid probe enable pin
SOL_EN.off()  # Disable solenoid by default
SOL_EN_probe.off()  # Disable solenoid probe by default

PRS_IN.read_uv()  # Read pressure input voltage
ADC_0_REF.read_uv()  # Read ADC reference voltage

LIMIT_SW = Pin(18, Pin.IN, Pin.PULL_UP)  # Limit switch pin

# Initialize variables
pressure_val = 0  # Current pressure value
temp_val = 0  # Current temperature value
target_p_val = 0.35  # Target pressure value

MTR_RUN = 0  # Motor running status

F_MIN = 150  # Minimum frequency
F_MAX = 1000  # Maximum frequency

Shutdown = Pin(22, Pin.OUT)  # Shutdown pin

f = F_MIN  # Current frequency
delta_f = 1  # Frequency step size

p = PWM(Pin(23), f, duty=200)  # PWM pin for pressure control (max duty cycle: 403)

def start_up(time_s, frequency):
    """
    Start-up function to initialize the system.
    """
    MTR_DIR.value(0)  # Set motor direction to forward
    MTR_STP.freq(frequency)  # Set motor step frequency
    MTR_STP.duty(512)  # Set motor step duty cycle to 50%
    
    while True:
        LIMIT_SW_VAL = LIMIT_SW.value()  # Read limit switch value
        
        if LIMIT_SW_VAL == 1:
            enable_motor(1, time_s*1000)  # Enable motor for specified time
            sleep(time_s)  # Wait for the specified time
            return

def read_pressure(wait_time_us, trials):
    """
    Read pressure value from the sensor.
    """
    total_p_value = 0
    
    for _ in range(trials):
        total_p_value += (PRS_IN.read_uv()-ADC_0_REF.read_uv())
        sleep(wait_time_us/1E6)
    
    global pressure_val
    pressure_val = ((total_p_value/trials)/1E6)+0.01

def read_temperature(wait_time_us, trials):
    """
    Read temperature value from the sensor.
    """
    total_t_value = 0
    
    for _ in range(trials):
        total_t_value += (TMP_IN.read_uv()-ADC_0_REF.read_uv())
        sleep(wait_time_us/1E6)
    
    global temp_val
    temp_val = ((total_t_value/trials)/1E6)+0.01

def enable_motor(direction, period_ms):
    """
    Enable the motor with the specified direction and period.
    """
    MTR_EN.off()
    MTR_DIR.value(direction)
    MTR_STP.duty(512)  # Set the duty cycle to 50%
    timer = Timer(0)
    timer.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_motor)
    global MTR_RUN
    MTR_RUN = True

def disable_motor(timer):
    """
    Disable the motor.
    """
    global MTR_RUN
    MTR_RUN = False
    print("Motor Disabled")
    MTR_STP.duty(0)
    MTR_EN.off()

def enable_sol(period_ms):
    """
    Enable the solenoid for the specified period.
    """
    timer2 = Timer(1)
    timer2.init(period=period_ms, mode=Timer.ONE_SHOT, callback=disable_sol)
    SOL_EN.on()
    SOL_EN_probe.on()

def disable_sol(timer2):
    """
    Disable the solenoid.
    """
    SOL_EN.off()
    SOL_EN_probe.off()

# Initialize PID controller
Kp = 1.5  # Proportional gain
Ki = 0.05  # Integral gain
Kd = 0.1  # Derivative gain
pid_sys = pid.PID(Kp, Ki, Kd)
pid_sys.target = target_p_val

def main():
    """
    Main function to run the pressure control system.
    """
    start_up(37, 600)  # Start-up the system
    
    MTR_STP.freq(300)  # Set motor step frequency to 300 Hz
    
    valid = True
    while valid:
        p.freq(f)  # Set PWM frequency
        sleep(10 / F_MIN)
        Shutdown.on()
        
        read_pressure(10, 100)  # Read pressure value
        read_temperature(10, 100)  # Read temperature value
        print(pressure_val)
        atm_pressure = (pressure_val - 0.275) / (0.529 - 0.275) + 1
        print("P: ", atm_pressure, " atm")
        print("T (V): ", temp_val)
        
        # Calculate the error
        error = pid_sys.target - pressure_val
        
        # Check if pressure is within 5% tolerance range
        if abs(error) <= target_p_val * 0.02:
            print("Pressure is within tolerance range.")
            continue
        
        # Get the control output from the PID controller
        control_output = pid_sys(pressure_val)
        
        # Determine action based on control output
        if control_output < 0:
            if LIMIT_SW.value() != 1:
                # Enable motor to increase pressure
                enable_motor(0, int(abs(control_output) * 1000))
            else:
                print("Limit Reached")
                Shutdown.off()
                valid = False
        elif control_output > 0:
            # Enable solenoid to decrease pressure
            enable_sol(int(abs(control_output) * 1000))

main()
