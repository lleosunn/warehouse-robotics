import pygame
from pygame.math import Vector2
import numpy as np
import matplotlib
import math
matplotlib.use('TkAgg')

reference_position = pygame.Vector2()
ppu = 30

# IDM related parameters
# TO_DO Define parameter specifications of your IDM model and the IDM variant

T = 1 # desired time headway
a = 2 # comfortable acceleration
b = 1.5 # comfortable deceleration
s_0 = 1 # minimum spacing
v_0 = 25 # max velocity
delta = 4 # exponent

class Car:
    def __init__(self, x, y, id_value, screen_width):
        # initializing the vehicle parameters
        self.position = Vector2(x, y)
        self.velocity = Vector2(25, 0.0)
        self.acceleration = Vector2(2, 0.0)
        self.id = id_value
        self.screen_width = screen_width

    def car_following_model(self, dt, vehicle_lead, vehicle_follow, reference_position_x, run_type):
        # 'vehicle_lead' is the lead vehicle of the current vehicle (self). It is a 'Car' object.
        # 'vehicle_follow' is the vehicle behind the current vehicle (self). It is a 'Car' object.
        if run_type == "IDM":
            self.IDM_model(dt, vehicle_lead, reference_position_x)
        elif run_type == "Custom":
            self.Custom_model(dt, vehicle_lead, vehicle_follow, reference_position_x)
        else:
            self.Test_model(dt, vehicle_lead, reference_position_x)
    
    def compute_current_lead_gap(self, vehicle_lead, reference_position_x):
        # Compute the gap between ego vehicle and lead vehicle
        current_gap = pygame.Vector2()
        current_gap.xy = (0, 0)

        if self.position.x < vehicle_lead.position.x:
            current_gap = vehicle_lead.position - self.position
        else: # handles the case of screen looping around
            reference_position.xy = (reference_position_x, 0)
            temp = pygame.Vector2()
            temp.xy = (self.screen_width - 48) / ppu, 0
            current_gap.xy = vehicle_lead.position - reference_position + (temp - self.position)
        
        return current_gap
        
    def compute_current_follow_gap(self, vehicle_follow, reference_position_x):
        # Compute the gap between ego vehicle and vehicle behind it
        current_gap = pygame.Vector2()
        current_gap.xy = (0, 0)
    
        if self.position.x > vehicle_follow.position.x:
            current_gap = self.position - vehicle_follow.position
        else: # handles the case of the screen looping around
            reference_position.xy = (reference_position_x, 0)
            temp = pygame.Vector2()
            temp.xy = (self.screen_width - 48) / ppu, 0
            current_gap.xy = self.position - reference_position + (temp - vehicle_follow.position)
    
        return current_gap


    def IDM_model(self, dt, vehicle_lead, reference_position_x):
        """ 
        Implementation of the Intelligent Driver Model (IDM) for car following.

        Parameters
        ----------
            dt: float
                time step
            vehicle_lead: float
                leading vehicle
            reference_position_x: float
                reference position on the roadmap
        """

        # This is the net distance between the two vehicles (self and vehicle_lead)
        # (denoted as 's' in the lecture note)
        # use this value (in meters) instead of computing it.
        current_gap = self.compute_current_lead_gap(vehicle_lead, reference_position_x)

        # TO_DO : Write the IDM model below
        v = self.velocity.x
        delta_v = v - vehicle_lead.velocity.x
        s_opt = s_0 + max(0, v*T + (v*delta_v)/(2*np.sqrt(a*b)))
        accel = a*(1-np.power((v/v_0),delta) - np.power((s_opt/current_gap.x),2))

        # TO_DO: You need to set the vehicle's new acceleration here by setting 'self.acceleration' parameter
        self.acceleration.x = accel

        # TO_DO Modify the 'next_step' method to use euler numerical integration scheme
        self.next_step(dt, reference_position_x)

    def next_step(self, dt, reference_position_x):

        # TO_DO : In this method, make the velocity update of the vehicle according
        # to the euler numerical integration scheme.
        # In updating vehicle the position, we call the method 'update_car_position' with
        # the amount of change of position (delta change) and reference_position_x as inputs.
        # Also note that velocity and position are pygame 2-D vectors that can be initiated with v = pygame.Vector2()
        # More details on pygame vectors can be found here : https://www.pygame.org/docs/ref/math.html
        velocity_change = pygame.Vector2()

        # forward euler method
        velocity_change.xy = self.acceleration.x * dt, 0
        self.velocity += velocity_change

        # We make sure that vehicles do not have negative velocities (they do not move backwards)
        if self.velocity.x < 0:
            v = pygame.Vector2()
            v.xy = 0, 0
            self.velocity = v

        # we can update the position here. We use backward euler method.
        # Try to understand the difference between forward euler and backward euler methods.
        position_change = pygame.Vector2()
        position_change.xy = self.velocity.x * dt, 0

        # Calling the method 'update_car_position' with the amount of change of position (position_change) as input
        # This method call is mandatory to convert position_change to pixel level position_change for rendering purposes
        self.update_car_position(position_change, reference_position_x)

    def Custom_model(self, dt, vehicle_lead, vehicle_follow, reference_position_x):
        # TO_DO Define your IDM variant.
        # Remember that in this framework, we create a circular road but visualize it as a straight road.
        # This means, each vehicle that leaves the visualization from the right most corner of the screen,
        # will join the road from the left of the screen. To avoid any issues with this setting, we encourage
        # you to follow the same code structure as in IDM model when defining your own model.
        
        # 'current_lead_gap' the net distance between the ego vehicle and vehicle in front
        # 'current_follow_gap' the net distance between the ego vehicle and vehicle behind it
        # use these value (in meters) instead of computing it.
        current_lead_gap = self.compute_current_lead_gap(vehicle_lead, reference_position_x)
        current_follow_gap = self.compute_current_follow_gap(vehicle_follow, reference_position_x)

        v = self.velocity.x
        v_behind = vehicle_follow.velocity.x
        delta_v = v - vehicle_lead.velocity.x
        s_opt = s_0 + max(0, v*T + (v*delta_v)/(2*np.sqrt(a*b)))
        accel_one = a*(1-np.power((v/v_0),delta) - np.power((s_opt/current_lead_gap.x),2))
        accel_two = (np.power(v-v_behind+b*T,2))/(2*(current_follow_gap.x-s_0-v_behind*T)) - b

        if v_behind > v + b*T: # then consider car behind
            accel = max(accel_one, accel_two)
        else:
            accel = accel_one

        self.acceleration.x = accel

        # TO_DO Modify the 'next_step' method to use Euler numerical integration scheme
        self.next_step(dt, reference_position_x)
    
    def Test_model(self, dt, vehicle_lead, reference_position_x):
        # This is only a test model. You can use this to understand the framework.
        # This method will act as the default method if you do not specify the model type
        self.acceleration = 0, 0
        velocity_change = pygame.Vector2()
        velocity_change.xy = 0.1, 0
        self.velocity += velocity_change
        position_change = pygame.Vector2()
        position_change.xy = 0.1, 0
        self.update_car_position(position_change, reference_position_x)

    def update_car_position(self, position_change, reference_position_x):

        # Position_change is a 2-D vector which indicates the amount of position change in x and y directions.
        # This method is necessary to place the vehicles in the right place at each time step.
        # Remember that in this framework, we create a circular road but visualize it as a straight road.
        # This means, each vehicle that leaves the visualization from the right most corner of the screen,
        # will join the road from the left of the screen.
        if self.position.x > ((self.screen_width - 48) / ppu):
            reference_position.xy = (reference_position_x, 2)
            self.position = reference_position + position_change
        else:
            self.position += position_change
