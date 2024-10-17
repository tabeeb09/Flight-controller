import time

import os

import csv

import keyboard

from vpython import * 

import numpy as np

import random 

import math

from scipy.spatial.transform import Rotation as R

from simple_pid import PID


def PIDyaw(kp, ki, kd, error):
    global previous_yaw_error, delta_t, integraly
    proportional = kp * error * delta_t
    
    integraly += ki * error * 3 * delta_t

    differential = kd * (error - previous_yaw_error)/ (3 * delta_t)
    print(differential)

    previous_yaw_error = error

    return proportional + integraly + differential

def PIDroll(kp, ki, kd, error):
    global previous_roll_error, delta_t, integralr
    proportional = kp * error * delta_t

    
    integralr += ki * error * 3 * delta_t


    differential = kd * (error - previous_roll_error)/( 3 * delta_t)


    previous_roll_error = error

    return proportional + integralr + differential

def PIDpitch(kp, ki, kd, error):
    global previous_pitch_error, delta_t, integralp, current_angular_velocity

    #if abs(error) < current_angular_velocity['pitch']*2:
    #    proportional = -kp**3 * error * delta_t
    #else:
    #    proportional = kp * error * delta_t
   
    proportional = kp * error * delta_t
    
    integralp += -ki * error * 3 * delta_t


    differential = kd * (error - previous_pitch_error)/(3 * delta_t)

 

    previous_yaw_error = error


    return proportional + integralp + differential

def update_throttle(desired_throttle, motor_number):

    global motors

    try:
        if desired_throttle <= 1:
            try:
                motors["motor"+str(motor_number)] = desired_throttle
            except:
                print('invalid motor number provided:' + str(motor_number))
    
            #print('desired throttle out of range float from 0 to 1:' + str(desired_throttle))
    except:
        print('incorrect data type entered for desired throttle, should be a float or integer between 0 and 1')

def maxfilter(desired_rate, motor_number):

    global motors
    
    try:
        current_throttle = motors["motor"+str(motor_number)] 
        if 1 - current_throttle <= desired_rate:
            return 1
        else:
            return current_throttle + desired_rate
        
    except:
        print('invalid motor number provided:' + str(motor_number))
        return False

   

def minfilter(desired_rate, motor_number):

    global motors
    
    try:
        current_throttle = motors["motor"+str(motor_number)] 
        if current_throttle < desired_rate:
            return 0
        else:
            return current_throttle - desired_rate
        
        
    except:
        print('invalid motor number provided:' + str(motor_number))
        return False

def linearisation(axis, omega, omega_naught, thrust, delta_t):

    rate = moment_of_inertia[axis] * 9.81 * (omega - omega_naught) / (2 * delta_t * 70 * (10 ** -3) * (battery_voltage/4.1*2) * 40 * (10 ** -3))   

    return rate

def yaw(desired_rate):

    if desired_rate > 0:
        update_throttle(maxfilter(desired_rate, 2), 2)
        update_throttle(maxfilter(desired_rate, 3), 3)
        update_throttle(minfilter(desired_rate, 1), 1)
        update_throttle(minfilter(desired_rate, 4), 4)
        
    else:
        update_throttle(minfilter(-desired_rate, 2), 2)
        update_throttle(minfilter(-desired_rate, 3), 3)
        update_throttle(maxfilter(-desired_rate, 1), 1)
        update_throttle(maxfilter(-desired_rate, 4), 4)
 
def roll(desired_rate):

    if desired_rate > 0:
        update_throttle(maxfilter(desired_rate, 1), 1)
        update_throttle(maxfilter(desired_rate, 3), 3)
        update_throttle(minfilter(desired_rate, 2), 2)
        update_throttle(minfilter(desired_rate, 4), 4)
    else:
        update_throttle(minfilter(-desired_rate, 1), 1)
        update_throttle(minfilter(-desired_rate, 3), 3)
        update_throttle(maxfilter(-desired_rate, 2), 2)
        update_throttle(maxfilter(-desired_rate, 4), 4)

def pitch(desired_rate):

    if desired_rate > 0:
        update_throttle(maxfilter(desired_rate, 4), 4)
        update_throttle(maxfilter(desired_rate, 3), 3)
        update_throttle(minfilter(desired_rate, 2), 2)
        update_throttle(minfilter(desired_rate, 1), 1)
    else:
        update_throttle(minfilter(-desired_rate, 4), 4)
        update_throttle(minfilter(-desired_rate, 3), 3)
        update_throttle(maxfilter(-desired_rate, 2), 2)
        update_throttle(maxfilter(-desired_rate, 1), 1)

def get_pwm_map():

    global motors, battery_voltage
    
    pwm_map = {}
    #print(motors)
    for thrust in motors:
        #(1/2)*battery voltage(battery voltage + 1) thrust = 1/2 ((battery_volatage/4.1)*pwm duty cycle)**2 +(1/2)* ((battery_volatage/4.1)*pwm duty cycle)
        a = ((battery_voltage/4.1)**2)*(1/2)
        b = (battery_voltage/4.1)*(1/2)
        c = -(1/2) * battery_voltage * (battery_voltage + 1) * motors[thrust]
        #print(a,b,c)
        pwm_duty_cycle = ((1/(2*a))*(-b+math.sqrt((b**2)-4*a*c)))
        pwm_map[thrust] = pwm_duty_cycle

    return pwm_map


def get_battery_voltage():
    """interface witht he adc to find the voltmeter reading of the cell, return the value as a float"""
    return battery_voltage

def get_desired_attitude(attitude_hold_mode):
    global max_rotation_angle, channel_input, current_attitude, desired_attitude

    if attitude_hold_mode == 1:
        desired_attitude_vector = {}

        desired_attitude_vector['roll'] = np.radians(channel_input['roll'] * max_rotation_angle['roll'])
        desired_attitude_vector['yaw'] = np.radians(channel_input['yaw'] * max_rotation_angle['yaw'])
        desired_attitude_vector['pitch'] = np.radians(channel_input['pitch'] * max_rotation_angle['pitch'])

        #print(desired_attitude_vector, np.radians(channel_input['yaw'] * max_rotation_angle['yaw']))

        
        return desired_attitude_vector

    desired_attitude_vector = {}
    
    desired_attitude_vector['roll'] = desired_attitude['roll'] + np.radians(channel_input['roll'] * max_rotation_angle['roll']),
    desired_attitude_vector['yaw'] = desired_attitude['yaw'] + np.radians(channel_input['yaw'] * max_rotation_angle['yaw']),
    desired_attitude_vector['pitch'] = desired_attitude['pitch'] + np.radians(channel_input['pitch'] * max_rotation_angle['pitch'])

    return desired_attitude_vector

#def get_current_attitude():
    #interface with the gyro to get orientation in 3d space. return a vectr, in degrees / radians 

def error():
    global current_attitude, desired_attitude

    calculated_error = {}
    
    calculated_error['yaw'] = current_attitude['yaw'] - desired_attitude['yaw']
    calculated_error['pitch'] = current_attitude['pitch'] - desired_attitude['pitch']
    calculated_error['roll'] = current_attitude['roll'] - desired_attitude['roll']
    
    return calculated_error

def ibus_decoder():
    """read ibus serial pins and recieve data packets, decode into channel values.
return normalised channel values, in the form of a dictionary"""

    global channel_input
    
    if keyboard.is_pressed("r") and channel_input['roll'] < 1:
        channel_input['roll'] += 0.1

    elif keyboard.is_pressed("f") and channel_input['roll'] > 0:
        channel_input['roll'] -= 0.1

    if keyboard.is_pressed("p") and channel_input['pitch'] < 1:
        channel_input['pitch'] += 0.1

    elif keyboard.is_pressed("l") and channel_input['pitch'] > 0:
        channel_input['pitch'] -= 0.1

    if keyboard.is_pressed("y") and channel_input['yaw'] < 1:
        channel_input['yaw'] += 0.1

    elif keyboard.is_pressed("h") and channel_input['yaw'] > 0:
        channel_input['yaw'] -= 0.1

    if keyboard.is_pressed("t") and channel_input['throttle'] < 1:
        channel_input['throttle'] += 0.1

    elif keyboard.is_pressed("g") and channel_input['throttle'] > 0:
        channel_input['throttle'] -= 0.1

def get_roll_torque():
    
    global motors, battery_voltage
    
    total_torque = 40 * (10**-3) * (motors['motor1'] + motors['motor3']) * 70 * (10**-3) * (1/9.81) * (battery_voltage/4.1*2) * ((battery_voltage/4.1) + 1) - 40 * (10 ** -3) * (motors['motor2'] + motors['motor4']) * 70 * (10 ** -3) * (1/9.81) * (battery_voltage/4.1*2) * ((battery_voltage/4.1) + 1) 

    return total_torque

def get_pitch_torque():
    
    global motors, battery_voltage
    
    total_torque = -(40 * (10**-3) * (motors['motor1'] + motors['motor2']) * 70 * (10 ** -3) * (1/9.81) * (battery_voltage/4.1*2) * ((battery_voltage/4.1) + 1) - 40 * (10**-3) * (motors['motor3'] + motors['motor4']) * 70 *(10 ** -3) * (1/9.81) * (battery_voltage/4.1*2) * ((battery_voltage/4.1) + 1))

    return total_torque

def get_yaw_angular_velocity():
    
    global motors, moment_of_inertia
    
    thrust_to_rpm = get_pwm_map()

   
    angular_velocity = 2 * -4.096 * (10**-8) * ((53000 * 0.9))*(thrust_to_rpm['motor1'] + thrust_to_rpm['motor4'] - (thrust_to_rpm['motor2'] + thrust_to_rpm['motor3'])) / (moment_of_inertia['yaw'] * 60 * 2 * np.pi)
    
    return angular_velocity

def get_angular_acceleration(torque, moment_of_inertia):

    angular_acceleration = torque / moment_of_inertia

    return angular_acceleration

def get_mean(data_points):
    sum = 0
    for i in data_points:
        sum += i
    
    return sum/len(data_points)

# def get_samples(no_samples, quantity):
#     global samples 
#     if len(samples)
    
def cubic_prediction(p1, p2, p3, derivative):
    np.multiply( np.inv([[p1[0] ** 3, p1[0] ** 2, p1[0], 1], [ p1[0] ** 3, p1[0] ** 2, p1[0], 1 ], [ p3[0] ], [3 *derivative[0] ]]), [p1[1], p2[1], p3[1], derivative])
     
    
     



##
##def apply_pwm_map(pwm_map):
##    """find the physical pin connection to each pwm motor controller transistor, and set
##the clock and pwm duty cycle with a set pwm frequency"""



max_rotation_angle = {'roll':45,
                      'pitch':-45,
                      'yaw':180}

channel_input = {'roll':1,
                 'pitch':1,
                 'yaw':27/180,
                 'throttle':0.4,
                 'attitude_hold_mode': 1 }

battery_voltage = 4.1


desired_attitude = {'roll':0,
                 'pitch':0,
                 'yaw':0}

motors = {'motor1':0,
          'motor2':0,
          'motor3':0,
          'motor4':0}

moment_of_inertia = {"roll":0.000048,
                 "pitch":0.00007384958464,
                 "yaw":0.000135168}


current_angular_acceleration = {"roll": get_angular_acceleration(get_roll_torque(),moment_of_inertia['roll']),
                                "pitch":get_angular_acceleration(get_pitch_torque(),moment_of_inertia['pitch'])}

current_attitude = {"roll": 0,
                 "pitch":0,
                 "yaw":0}
                                                      
current_angular_velocity ={"roll": 0,
                           "pitch":0,
                           "yaw":0}

previous_yaw_error = 0
previous_roll_error = 0
previous_pitch_error = 0

integraly = 0
integralr = 0
integralp = 0


delta_t = 0.1

example = []


def main():   

    global current_attitude, current_angular_velocity, current_angular_acceleration, desired_attitude, previous_pitch_error, channel_input, previous_pitch_error, previous_roll_error, moment_of_inertia

    a = arrow(pos=vector(0,0,0),axis=vector(1,0,0),color=color.red)
    b = arrow(pos=vector(0,0,0),axis=vector(0,1,0),color=color.blue)

    timer = 0

    delta_t = 0.001

##    pid_yaw = PIDyaw(
##    pid_yaw.sample_time = delta_t
##
##    pid_roll = PIDroll(0, 00, 0, setpoint = desired_attitude['roll'])#PID(0.001, 0.0000001, 0.03, setpoint = desired_attitude['roll'])
##    pid_roll.sample_time = delta_t
##
##    pid_pitch = PIDpitch(1, 10000, 0.08, setpoint = desired_attitude['pitch'])
##    pid_pitch.sample_time = delta_t

    
    x = True
    y = True
    
    while timer < 1000 * delta_t:
        ibus_decoder() # update the inputs

        for i in range(len(motors)):
            update_throttle(channel_input['throttle'], i+1)

        timer += delta_t

        
        
        desired_attitude = get_desired_attitude(channel_input['attitude_hold_mode'])

        while x == True:
            previous_yaw_error = error()['yaw']
            previous_roll_error = error()['roll']
            previous_pitch_error = error()['pitch']
            x = False
              
##        pid_yaw.setpoint = desired_attitude['yaw']
##        pid_roll.setpoint = desired_attitude['roll']
##        pid_pitch.setpoint = desired_attitude['pitch'] 
##        
        #error_runtime = error()

        #yaw(float(pid_yaw(current_attitude['yaw'])))
        #pitch(float(pid_pitch(current_attitude['pitch'])))
        #yaw(float(PIDyaw(10, 10, 10, error()['yaw'])))
        #roll(float(PIDroll(10, 10, 10, error()['roll'])))

        if math.floor(timer/delta_t) % 3 == 0:  
            pitch(linearisation('pitch', -PIDpitch(32, 0.000, 0.000418, error()['pitch']), current_angular_velocity['pitch'], channel_input['throttle'], 3*delta_t))
        if math.floor(timer/delta_t) % 3 == 1:
            yaw(linearisation('yaw', -PIDyaw(100, 0.000, 0.0418, error()['yaw']), current_angular_velocity['yaw'], channel_input['throttle'], 3*delta_t))
        if math.floor(timer/delta_t) % 3 == 2:
            roll(linearisation('roll', -PIDroll(80, 0.000, 0.0318, error()['roll']), current_angular_velocity['roll'], channel_input['throttle'], 3*delta_t))
        
       # example.append([timer,current_angular_velocity['pitch'], current_attitude['pitch'], -PIDpitch(0.8, 0.000, 0.00418, error()['pitch']), current_attitude['pitch'], linearisation('pitch', -PIDpitch(0.08, 0.000, 0.000, error()['pitch']), current_angular_velocity['pitch'], channel_input['throttle'], delta_t)])
        
        #print(pid_yaw(current_attitude['roll']),pid_yaw(current_attitude['yaw']))

       # print('desired attitude',desired_attitude['yaw'])

        #print('angular_velocity', current_angular_velocity['yaw'])

        
        
        current_angular_acceleration = {"roll": get_angular_acceleration(get_roll_torque(),moment_of_inertia['roll']),
                         "pitch":get_angular_acceleration(get_pitch_torque(),moment_of_inertia['pitch'])}
    
                                                          
        current_attitude['roll'] += current_angular_velocity['roll'] * delta_t + (1/2) * current_angular_acceleration['roll'] * (delta_t)**2 #+ random.choice([1, -1]) * random.uniform(0, 0.1)
        current_attitude['pitch'] += current_angular_velocity['pitch'] * delta_t + (1/2) * current_angular_acceleration['pitch'] * (delta_t)**2 #+ random.choice([1, -1]) * random.uniform(0, 0.1)
        current_angular_velocity['roll'] += current_angular_acceleration['roll'] * delta_t 
        current_angular_velocity['pitch'] += current_angular_acceleration['pitch'] * delta_t  


        current_angular_velocity['yaw'] = get_yaw_angular_velocity()
        current_attitude['yaw'] += current_angular_velocity['yaw'] * delta_t

        

        #print(current_angular_velocity['yaw'])
        
        
        y = R.from_rotvec(current_attitude['yaw'] * np.array([0, 1, 0]))
        roll_matrix = R.from_rotvec(current_attitude['roll'] * np.array([0, 0, 1]))
        p = R.from_rotvec(current_attitude['pitch'] * np.array([1, 0, 0]))
        #print('current attitude:', current_attitude['yaw'])
        #print('error', error_runtime)

        example.append([timer, current_attitude['pitch'], current_attitude['roll'], current_attitude['yaw']])
        
        Radius = 2
        
        a.axis = vector(p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([1,0,0])))[0],
                       p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([1,0,0])))[1],
                       p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([1,0,0])))[2])

        b.axis = vector(p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([0,1,0])))[0],
                       p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([0,1,0])))[1],
                       p.as_matrix().dot(roll_matrix.as_matrix().dot(y.as_matrix().dot([0,1,0])))[2])
        time.sleep(delta_t)

    file_path = os.path.join(os.path.expanduser('~'), 'Documents', 'pid.csv')

    row_list = example
    
    try:
        with open(file_path, 'w', newline='') as file:
           writer = csv.writer(file)
           writer.writerows(row_list)
        print(f"File written successfully at {file_path}")
    except Exception as e:
       print(f"An error occurred while writing the file: {e}")

        
        
        
        


main()
#print(linearisation('pitch', -0.00000001, 1, 0.5, 0.1))





    


        
    
        



