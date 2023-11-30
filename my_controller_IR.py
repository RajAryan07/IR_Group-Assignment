from controller import Robot
import numpy as np

class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32  # ms
        self.max_speed = 1   # m/s

        
        self.was_above_threshold = False
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Enable Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.time_step)
            self.light_sensors.append(sensor)

        # Enable Ground Sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)

        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.time_step)
            self.proximity_sensors.append(sensor)

        # State Flags
        self.isAvoidingObstacle = False
        self.light_detected = False
        
        left_ir_value = self.left_ir.getValue()
        center_ir_value = self.center_ir.getValue()
        right_ir_value = self.right_ir.getValue()

        # Data
        self.inputs = []
        self.inputsPrevious = []
        self.flag_turn = 0

    def clip_value(self, value, min_max):
        if value > min_max:
            return min_max
        elif value < -min_max:
            return -min_max
        return value

    def sense_compute_and_actuate(self):
        if len(self.inputs) > 0 and len(self.inputsPrevious) > 0:
            # Turn logic
            if self.flag_turn:
                self.left_motor.setVelocity(-0.3)
                self.right_motor.setVelocity(0.3)
                if np.min(self.inputs[0:3]) < 0.35:
                    self.flag_turn = 0
            else:
                # Line following logic
                if self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]:
                    self.left_motor.setVelocity(0.5)
                    self.right_motor.setVelocity(1)
                elif self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]:
                    self.left_motor.setVelocity(1)
                    self.right_motor.setVelocity(1)
                elif self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]:
                    self.left_motor.setVelocity(1)
                    self.right_motor.setVelocity(0.5)

    def handle_light_sensors(self):
        # Read Light Sensors and calculate sum
        light_sensor_sum = sum(sensor.getValue() for sensor in self.light_sensors)
        #print("Light Sensor Sum:", light_sensor_sum)

        # Check light sensor sum
        if light_sensor_sum <= 20500:
            self.light_detected = True
            
    def crossing_object(self):
        # Read Light Sensors and calculate sum
        left = self.left_ir.getValue()
        center = self.center_ir.getValue()
        right = self.right_ir.getValue()
        #print("Light Sensor Sum:", light_sensor_sum)
        if left >= 640:
            self.isCross_object = True
            
        else:
            self.isCross_object = False    

    def obstacle_avoidance(self):
        # Read Proximity Sensors
        self.ps_values = [sensor.getValue() for sensor in self.proximity_sensors]

        # Check if any sensor value exceeds the threshold
        if self.ps_values[7] > 90 or self.ps_values[6] > 90 or self.ps_values[5] > 90 or self.ps_values[0] > 80 or self.ps_values[1] > 80 or self.ps_values[2] > 80:            
            self.isAvoidingObstacle = True
            

                       
                               

     
        
        
    def run_robot(self):        
        count = 0
        avoid = 0
        crossing_count = 0
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:

            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()
            # Check if all sensors are currently below 300
            if left < 300 and center < 300 and right < 300:
                if self.was_above_threshold:
                    crossing_count += 1
                    self.was_above_threshold = False
            else:
                # Check if all sensors are above 720
                if left > 720 and center > 720 and right > 720:
                    self.was_above_threshold = True
                # Read Proximity Sensors
            self.ps_values = [sensor.getValue() for sensor in self.proximity_sensors]
        
            # Check if any sensor value exceeds the threshold

                # Print all proximity sensor values
            #print("Proximity sensor values:", self.ps_values)

            #print("Ground sensor values: Left =", left, ", Center =", center, ", Right =", right)


            # Handle light sensors
            self.handle_light_sensors()
            # Retrieve sensor readings
            self.crossing_object()
            
            # Print the values
            if self.ps_values[7] > 100 and self.ps_values[6] > 100 and self.ps_values[5] > 100 and self.ps_values[2] > 100 and self.ps_values[1] > 100 and self.ps_values[0] > 100:
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                print("TASK ACHIEVED HURRAY") 
                break
                                               
                        # Obstacle detection
            self.obstacle_avoidance()
            if crossing_count < 5:
                if self.isAvoidingObstacle:
                    if self.light_detected:
                    # Wall-following logic for left path
                        left_wall = self.ps_values[5] > 80
                        front_wall = self.ps_values[7] > 80
                        left_m_wall = self.ps_values[6] > 80
                        
                        
                        if front_wall:
                            #print("Turn Right in place")
                            self.left_motor.setVelocity(self.max_speed)
                            self.right_motor.setVelocity(-self.max_speed) 
                            
                        if left_m_wall:
                            self.left_motor.setVelocity(self.max_speed)
                            self.right_motor.setVelocity(self.max_speed/8)   
                            
                        elif left_wall and not front_wall:  
                                #print("Drive Forward")
                                self.left_motor.setVelocity(self.max_speed)
                                self.right_motor.setVelocity(self.max_speed)
                                
                        elif not left_wall and not front_wall:
                                #print("Turn Left")
                                self.left_motor.setVelocity(self.max_speed/8)
                                self.right_motor.setVelocity(self.max_speed)
                         
                        
                    else:                        
                    # Wall-following logic for right path
                        #print("not")
                        left_r_wall = self.ps_values[2] > 80
                        front_r_wall = self.ps_values[7] > 80
                        left_m_r_wall = self.ps_values[0] > 80
                        
                        
                        if front_r_wall:
                            #print("Turn Left in place")
                            self.left_motor.setVelocity(-self.max_speed)
                            self.right_motor.setVelocity(self.max_speed) 

                        if left_m_r_wall:
                            #print("Turn Left in place")
                            self.left_motor.setVelocity(-self.max_speed)
                            self.right_motor.setVelocity(self.max_speed) 
                                                        
   
                            
                        elif left_r_wall and not front_r_wall:  
                                #print("Drive Forward")
                                self.left_motor.setVelocity(self.max_speed)
                                self.right_motor.setVelocity(self.max_speed)
                                
                        elif not left_r_wall and not front_r_wall:
                                #print("Turn Right")
                                self.left_motor.setVelocity(self.max_speed)
                                self.right_motor.setVelocity(self.max_speed/8)
                        
       
                        
                        
                    
            if crossing_count ==2 or crossing_count == 4:
                if self.light_detected:
                    self.left_motor.setVelocity(self.max_speed)
                    self.right_motor.setVelocity(self.max_speed)                                  
                    if left >= 728 and center >= 728 and right >= 728:
                        if self.light_detected:
                            self.left_motor.setVelocity(0.3)
                            self.right_motor.setVelocity(-0.3)
                        else:
                            self.left_motor.setVelocity(-0.3)
                            self.right_motor.setVelocity(0.3) 
                
                if not self.light_detected:            
                    if left >= 728 and center >= 728 and right >= 728:
                        if self.light_detected:
                            self.left_motor.setVelocity(0.3)
                            self.right_motor.setVelocity(-0.3)
                        else:
                            self.left_motor.setVelocity(-0.3)
                            self.right_motor.setVelocity(0.3)                
            #if self.isAvoidingObstacle:
                # Obstacle avoidance logic

            else:
                # Read Ground Sensors
                        self.inputs = []
                        left = self.left_ir.getValue()
                        center = self.center_ir.getValue()
                        right = self.right_ir.getValue()
        
                        # Normalize sensor values
                        left = self.clip_value(left, 1000)
                        center = self.clip_value(center, 1000)
                        right = self.clip_value(right, 1000)
        
                        # Save normalized data
                        self.inputs.append((left - 0) / (1000 - 0))
                        self.inputs.append((center - 0) / (1000 - 0))
                        self.inputs.append((right - 0) / (1000 - 0))

        
                        # Smooth filter (Average)
                        smooth = 30
                        if count == smooth:
                            inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                            self.inputs = [x / smooth for x in inputs_avg]
                            self.sense_compute_and_actuate()
                            count = 0
                            inputs_avg = []
                            self.inputsPrevious = self.inputs
                        else:
                            inputs_avg.append(self.inputs)
                            count += 1

                        # Check if all three ground sensors have values of 758 or higher
                        if crossing_count < 1:
                            if left >= 758 and center >= 758 and right >= 758:
                                self.left_motor.setVelocity(0)
                                self.right_motor.setVelocity(0)
            
                                # Turn based on light_detected flag
                                if not self.light_detected:
                                    self.left_motor.setVelocity(0.3)
                                    self.right_motor.setVelocity(-0.3)
                                else:
                                    self.left_motor.setVelocity(-0.3)
                                    self.right_motor.setVelocity(0.3)
                                continue
                                
                        if crossing_count == 5:
                            if self.ps_values[1] > 610 :
                                    self.left_motor.setVelocity(-0.3)
                                    self.right_motor.setVelocity(0.7) 
                            if self.ps_values[6] > 610 :
                                    self.left_motor.setVelocity(0.7)
                                    self.right_motor.setVelocity(-0.3)
                            continue
                                                                        

                                
                            

                                                 
            #print("Crossing Count:", crossing_count)

if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()