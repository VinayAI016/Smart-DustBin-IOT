from machine import Pin, time_pulse_us,I2C
import machine
import time
import machine
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep

# Pin configuration for servo and RFID
SERVO_PIN = 15  # GPIO 14

# Create servo object
servo = machine.PWM(machine.Pin(SERVO_PIN), freq=50)

I2C_ADDR = 0x27
totalRows = 2
totalColumns = 16

#i2c= I2C(scl=Pin(22), sda=Pin(21), freq=16000)
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=10000)
lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)
def stop_servo():
    # Stop the servo motor
    servo.duty(130)
    
def unlock_door():
    # Rotate servo to unlock the door
    servo.duty(30)  # Adjust duty cycle based on your servo

def lock_door():
    # Rotate servo to lock the door
    servo.duty(130)  # Adjust duty cycle based on your servo

SOUND_SPEED = 343
TRIG_PULSE_DURATION_US = 10

trig_pin = Pin(22, Pin.OUT)
echo_pin = Pin(18, Pin.IN)
buz = Pin(2, Pin.OUT)

while True:
    lcd.clear()
    lcd.putstr('    USE ME   ')
    stop_servo()
    trig_pin.value(0)
    time.sleep_us(5)
    trig_pin.value(1)
    time.sleep_us(TRIG_PULSE_DURATION_US)
    trig_pin.value(0)
    
    ultrason_duration = time_pulse_us(echo_pin, 1, 30000)
    distance_cm = SOUND_SPEED*ultrason_duration/20000
    
    if distance_cm<=20:
        lcd.clear()
        lcd.putstr('Opening......')
        unlock_door()
        time.sleep(5)
        lcd.clear()
        lcd.putstr('Closing...')
        lock_door()
    
    print(f"Distance: {distance_cm} cm")
    time.sleep_ms(500)
