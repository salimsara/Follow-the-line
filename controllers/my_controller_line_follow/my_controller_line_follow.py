from controller import Robot

robot = Robot()

timestep = 64
max_speed = 6.28

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Enable e-puck IR sensors
sensors = []
names = ['ir0', 
'ir1']
readings = [0,
 0]

for i in range(2):
    sensors.append(robot.getDevice(names[i]))
    sensors[i].enable(timestep)

p_error = 0.0
kp = 2.5
kd = 0.5
ki = 0.0
Integral = 0.0

def getreading():
    for i in range(2):
        if int(sensors[i].getValue()) > 512:
            readings[i] = 0
        else:
            readings[i] = 1
            
def PID():
    error = 0
    coe = [-1000,
     1000]

    for i in range(2):
        error += coe[i] * readings[i]

    P = kp * error
    I = Integral + (ki * error)
    D = kd * (error - p_error)

    correction = (P + I + D) / 1000
    l_speed = 5 + correction
    r_speed = 5 - correction

    if l_speed < 0.0:
        l_speed = 0.0
    if l_speed > max_speed:
        l_speed = max_speed
    if r_speed < 0.0:
        r_speed = 0.0
    if r_speed > max_speed:
        r_speed = max_speed

    left_motor.setVelocity(l_speed)
    right_motor.setVelocity(r_speed)

    print(l_speed, r_speed, readings)

    return I, error

while robot.step(timestep) != -1:
    
    for i in range(2):
        readings[i]=sensors[i].getValue()
        
    
    #print(readings)
    Integral, p_error = PID()
    getreading()
