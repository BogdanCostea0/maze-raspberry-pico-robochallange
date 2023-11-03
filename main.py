from machine import Pin, I2C, PWM
from utime import sleep_ms, sleep_us, ticks_ms, ticks_diff
from vl53l0x import setup_tofl_device, TBOOT
from mpu6050 import MPU6050

def wrap(angle):
    while (angle > +180):
        angle -= 360
    while (angle < -180):
        angle += 360
    return angle

# shutdown pins for each device
device_0_xshut = Pin(14, Pin.OUT)
device_1_xshut = Pin(15, Pin.OUT)
device_2_xshut = Pin(18, Pin.OUT)

# setup led
glb = Pin(22, Pin.OUT)
vrd = Pin(28, Pin.OUT)

# setup dc motor
dr = Pin(21, Pin.IN)
st = Pin(20, Pin.IN)

#KILL SWITCH START PIN
START = Pin(9, Pin.IN)

#LEDS
glb.value(1)
vrd.value(0)

# setup i2c bus 0
i2c_0 = I2C(id=0, sda=Pin(16), scl=Pin(17))

# setup bus 1
i2c_1 = I2C(id=1, sda=Pin(26), scl=Pin(27), freq=40000)

#PWM DC Mot
m_left1 = PWM(Pin(10))
m_left2 = PWM(Pin(11))

m_right1 = PWM(Pin(13))
m_right2 = PWM(Pin(12))

# set the PWM freq
PWM_FREQ = 1000

m_left1.freq(PWM_FREQ)
m_left2.freq(PWM_FREQ)
m_right1.freq(PWM_FREQ)
m_right2.freq(PWM_FREQ)

MOTOR_STATE = 'STOP'
MOTOR_MAX = 65025
MOTOR_SPEED = 0.4
REAL_SPEED = int(MOTOR_SPEED * MOTOR_MAX)

BRAKE_TIME = 350
BACK_TIME = 250
TURN_ANGLE = 80

# reset procedure for each TOF device
device_0_xshut.value(0)
device_1_xshut.value(0)
device_2_xshut.value(0)
sleep_ms(100)
device_0_xshut.value(1)
device_1_xshut.value(1)
device_2_xshut.value(1)

# setting up device TOF 0
print("Setting up device 0")
# keep active just the 1st sensor
device_0_xshut.value(1)
device_1_xshut.value(0)
device_2_xshut.value(0)

sleep_us(TBOOT)

tofl0 = setup_tofl_device(i2c_0, 40000, 12, 8) 
tofl0.set_address(0x31)

# setting up device TOF 1
print("Setting up device 1")
# turn on the 2nd sensor

device_1_xshut.value(1)
sleep_us(TBOOT)

tofl1 = setup_tofl_device(i2c_0, 40000, 12, 8)
tofl1.set_address(0x32)

# setting up device TOF 2
print("Setting up device 2")
# turn on the 3rd sensor

device_2_xshut.value(1)
sleep_us(TBOOT)

tofl2 = setup_tofl_device(i2c_0, 40000, 12, 8)
tofl2.set_address(0x33)

mpu = MPU6050(i2c_1)
print("init mpu")
mpu.Initialize()
print("calibrating")
mpu.Calibrate()
print('done calibrating')

def m_drive(leftPower, rightPower, direction):
    glb.value(1)
    vrd.value(1)

    left_1 = leftPower
    left_2 = leftPower

    right_1 = rightPower
    right_2 = rightPower

    if direction == 1:
        left_1 = 0
        right_1 = 0
    elif direction == -1:
        left_2 = 0
        right_2 = 0

    if direction != 0:
        m_left1.init(freq=PWM_FREQ, duty_u16=left_1) # type: ignore
        m_left1.duty_u16(left_1)
        m_left2.init(freq=PWM_FREQ, duty_u16=left_2) # type: ignore
        m_left2.duty_u16(left_2)
        m_right1.init(freq=PWM_FREQ, duty_u16=right_1) # type: ignore
        m_right1.duty_u16(right_1)
        m_right2.init(freq=PWM_FREQ, duty_u16=right_2) # type: ignore
        m_right2.duty_u16(right_2)

def m_turn(turnPower, direction):
    mpu.read()
    target = mpu._angZ
    if direction == 'LEFT':
        glb.value(1)
        vrd.value(0)
        target += TURN_ANGLE
    elif direction == 'RIGHT':
        glb.value(0)
        vrd.value(1)
        target -= TURN_ANGLE
    target = wrap(target)
    
    left_1 = turnPower
    left_2 = turnPower

    right_1 = turnPower
    right_2 = turnPower

    if direction == 'LEFT':
        left_2 = 0
        right_1 = 0
    elif direction == 'RIGHT':
        left_1 = 0
        right_2 = 0
    
    if direction == 'LEFT' or direction == 'RIGHT':
        m_left1.init(freq=PWM_FREQ, duty_u16=left_1) # type: ignore
        m_left1.duty_u16(left_1)
        m_left2.init(freq=PWM_FREQ, duty_u16=left_2) # type: ignore
        m_left2.duty_u16(left_2)
        m_right1.init(freq=PWM_FREQ, duty_u16=right_1) # type: ignore
        m_right1.duty_u16(right_1)
        m_right2.init(freq=PWM_FREQ, duty_u16=right_2) # type: ignore
        m_right2.duty_u16(right_2)

    if direction == 'LEFT':
        while mpu._angZ < target:
            mpu.read()
            
    elif direction == 'RIGHT':
        while mpu._angZ > target:
            mpu.read()
        
    mpu._angZ = 0
    m_stop()


def m_stop():
    m_left1.duty_u16(0)
    m_left2.duty_u16(0)
    m_right1.duty_u16(0)
    m_right2.duty_u16(0)
    #let the duty(0) take effect
    sleep_ms(1)
    m_left1.deinit()
    m_left2.deinit()
    m_right1.deinit()
    m_right2.deinit()

def m_brake():
    mpu.read()
    m_drive(MOTOR_MAX, MOTOR_MAX, 1)
    startTime = ticks_ms()
    while ticks_diff(ticks_ms(), startTime) <= BRAKE_TIME:
        mpu.read()
    m_stop()

def m_boost():
    mpu.read()
    m_drive(MOTOR_MAX, MOTOR_MAX, 1)
    startTime = ticks_ms()
    while ticks_diff(ticks_ms(), startTime) <= BRAKE_TIME:
        mpu.read()
    m_stop()

def back_little():
    mpu.read()
    m_drive(int(MOTOR_MAX * 0.9), int(MOTOR_MAX * 0.9), -1)
    startTime = ticks_ms()
    while ticks_diff(ticks_ms(), startTime) <= BACK_TIME:
        mpu.read()
    m_drive(int(MOTOR_MAX * 0.6), int(MOTOR_MAX * 0.6), 1)
    sleep_ms(50)
    m_stop()

#mark finished init
glb.value(0)
vrd.value(0)

#wait for reset
while START.value() == 1:
    a = 0

#wait for start
while START.value() == 0:
    a = 0

m_boost()
print('START')

first = 'RIGHT'

while True:
    mpu.read()
    centerDist = tofl0.ping()
    rightDist = tofl1.ping()
    leftDist = tofl2.ping()

    print(leftDist, ' ', centerDist, ' ', rightDist )


    if first == 'RIGHT':
        rightDist = 0
        centerDist = 2000
        first = 0

    if rightDist > 250:
        print('DREAPTA')
        # print(f'left{leftDist},right {rightDist},center{centerDist}')
        m_brake()
        m_turn(int(MOTOR_MAX * 1), 'RIGHT')
        m_boost()

    elif centerDist > 250:
        first = 0
        print('Center')
        # print(f'left{leftDist},right {rightDist},center{centerDist}')
        # m_turn(MOTOR_MAX, 'CENTER')
        m_drive(REAL_SPEED, REAL_SPEED, 1)

    elif leftDist > 250:
        print('DREAPTA')
        # print(f'left{leftDist},right {rightDist},center{centerDist}')
        m_brake()
        m_turn(int(MOTOR_MAX * 1), 'LEFT')
        m_boost()


    while (START.value() == 0):
        m_stop()
    