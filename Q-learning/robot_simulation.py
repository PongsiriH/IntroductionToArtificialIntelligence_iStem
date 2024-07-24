"""
Implementation of crawling robot in pymunk.
"""

import sys
import numpy as np
import pygame
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT, K_x
from pygame.color import THECOLORS

import pymunk
from pymunk import Vec2d
import pymunk.pygame_util

class ServoMotor(pymunk.SimpleMotor):
    # Mostly from: https://stackoverflow.com/questions/55302406/how-to-rotate-pymunk-joints-at-will
    def __init__(self, body_a, body_b, max_force, max_rate, p_gain, min_angle=None, max_angle=None):
        super().__init__(body_a, body_b, 0)  # Start with rate = 0
        self.body_a = body_a
        self.body_b = body_b
        self.max_force = max_force
        self.max_rate = max_rate
        self.p_gain = p_gain
        self.target_angle = 0
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.last_target_was_out_of_bounded = False
        
    def angle(self):
        """Get the current angle of the motor."""
        return self.body_a.angle - self.body_b.angle

    def set_target_angle(self, target_angle):
        """Set the target angle for the motor to rotate towards."""
        if self.min_angle is not None and target_angle < self.min_angle:
            self.target_angle = self.min_angle
            self.last_target_was_out_of_bounded = True
        elif self.max_angle is not None and target_angle > self.max_angle:
            self.target_angle = self.max_angle
            self.last_target_was_out_of_bounded = True
        else:
            self.target_angle = target_angle
            self.last_target_was_out_of_bounded = False
            
    def update(self):
        rate = (self.target_angle - self.angle()) * self.p_gain
        self.rate = max(min(rate, self.max_rate), -self.max_rate)

class CrawlingRobotSimulator(object):
    def __init__(self):
        self.fps = 50
        self.control_flag = True
        self.display_flags = 0
        self.display_size = (400, 200)
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        self.ground_y = 100
        self.ground = pymunk.Segment(self.space.static_body, (-100, self.ground_y), (1000, self.ground_y), 5.0)
        self.ground.friction = 1
        self.space.add(self.ground)
        self.screen = None
        self.draw_options = None
        self.handContactingGround = False
        
    def run(self):
        while sim.running:
            # take action:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key in (pygame.K_q, pygame.K_ESCAPE)):
                    pygame.quit()
                    sys.exit(0)
                if self.control_flag:
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                        sim.body.position = sim.bodyXY
                        sim.reset_bodies(sim.bodyXY)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                        new_angle = sim.motor_hand.angle() + np.deg2rad(40)
                        sim.motor_hand.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                        new_angle = sim.motor_hand.angle() - np.deg2rad(40)
                        sim.motor_hand.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                        new_angle = sim.motor_arm.angle() - np.deg2rad(20)
                        sim.motor_arm.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_w:
                        new_angle = sim.motor_arm.angle() + np.deg2rad(20)
                        sim.motor_arm.set_target_angle(new_angle)
                    elif event.type == pygame.KEYUP:
                        sim.motor_arm.rate = 0
                        sim.motor_hand.rate = 0
            for motor in sim.motor_list:
                motor.update()
            sim.update_simulation()
            sim.draw()
    
    def get_states(self):
        position_x = self.body.position[0]
        handAngle = self.motor_hand.angle()
        armAngle = self.motor_arm.angle()
        return position_x, handAngle, armAngle
    
    def reset_bodies(self,bodyXY):
        armWd_a = 75
        armWd_b = 75
        chWd = 70

        self.body.position = bodyXY
        self.arm_body.position = bodyXY + ((chWd / 2) + (armWd_a / 2), 0)
        self.arm_body.angle = 0
        self.hand_body.position = self.arm_body.position + ((armWd_a / 2) + (armWd_b / 2), 0)
        self.hand_body.angle = 0

    def draw(self):
        self.screen.fill(THECOLORS["white"])  ### Clear the screen
        self.space.debug_draw(self.draw_options)  ### Draw space
        pygame.display.flip()  ### All done, lets flip the display
        self.clock.tick(self.fps)
        
    def init(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
        self.width, self.height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        pymunk.pygame_util.positive_y_is_up = True

        self.clock = pygame.time.Clock()
        self.running = True
        self.font = pygame.font.Font(None, 16)

        # Create the spider
        self.bodyXY = Vec2d(20, self.ground_y + 20)
        chWd = 70
        chHt = 50
        bodyMass = 5

        armWd_a = 60
        armHt_a = 8
        armWd_b = 28
        armHt_b = 12
        armMass = 1

        self.min_arm_angle = np.deg2rad(-10)
        self.max_arm_angle = np.deg2rad(10)
        self.min_hand_angle = np.deg2rad(-140)
        self.max_hand_angle = np.deg2rad(0)
        min_arm_angle = self.min_arm_angle
        max_arm_angle = self.max_arm_angle
        min_hand_angle = self.min_hand_angle
        max_hand_angle = self.max_hand_angle
        
        motor_max_force = np.inf
        motor_max_rate = 5
        motor_p_gain = 10
        # ---body
        self.body = pymunk.Body(bodyMass, pymunk.moment_for_box(bodyMass, (chWd, chHt)))
        self.body.position = self.bodyXY
        self.body_shape = pymunk.Poly.create_box(self.body, (chWd, chHt))
        self.body_shape.color = 200, 200, 200, 100
        self.body_shape.friction = 0.1
        print("body position")
        print(self.body.position)

        # ---first right arm a
        armMass = 0.2
        self.arm_body = pymunk.Body(armMass, pymunk.moment_for_box(armMass, (armWd_a, armHt_a)))
        self.arm_body.position = self.bodyXY + ((chWd / 2) + (armWd_a / 2), 0)
        arm_shape = pymunk.Poly.create_box(self.arm_body, (armWd_a, armHt_a))
        arm_shape.color = 255, 0, 0, 100
        arm_shape.friction = 1
        arm_shape.elasticity = 0
        # ---first right arm b
        self.hand_body = pymunk.Body(armMass, pymunk.moment_for_segment(armMass, (0, 0), (armWd_b, 0), 2))
        self.hand_body.position = self.arm_body.position + ((armWd_a / 2) + (armWd_b / 2), 0)
        # Define the start and end points of the segment relative to the body's center
        start_point = (-armWd_b / 2, 0)
        end_point = (armWd_b / 2, 0)
        hand_shape = pymunk.Segment(self.hand_body, start_point, end_point, armHt_b / 2)
        hand_shape.color = 0, 255, 0, 100
        hand_shape.friction = 1
        hand_shape.elasticity = 0
        # ---link hand with the arm
        pj_hand = pymunk.PinJoint(self.hand_body, self.arm_body, (-armWd_b / 2, 0),
                                      (armWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space
        self.motor_hand = ServoMotor(self.hand_body, self.arm_body, motor_max_force,motor_max_rate,motor_p_gain)
        self.motor_hand.min_angle = min_hand_angle
        self.motor_hand.max_angle = max_hand_angle
        
        # ---link arm a with body
        pj_arm = pymunk.PinJoint(self.arm_body, self.body, (-armWd_a / 2, 0), (chWd / 2, 0))
        self.motor_arm = ServoMotor(self.arm_body, self.body, motor_max_force,motor_max_rate,motor_p_gain)
        self.motor_arm.min_angle = min_arm_angle
        self.motor_arm.max_angle = max_arm_angle
        self.space.add(self.body, self.body_shape)
        self.space.add(self.arm_body, arm_shape)
        self.space.add(self.hand_body, hand_shape)
        self.space.add(pj_hand, self.motor_hand, pj_arm, self.motor_arm)

        self.motor_list = [self.motor_hand,self.motor_arm]
        # ---prevent collisions with ShapeFilter
        shape_filter = pymunk.ShapeFilter(group=1)
        self.body_shape.filter = shape_filter
        arm_shape.filter = shape_filter
        hand_shape.filter = shape_filter
        # c.filter = shape_filter

        # ---collision handler
        def on_collision_begin(arbiter, space, data):
            self.handContactingGround = True
            return True
        def on_collision_separate(arbiter, space, data):
            self.handContactingGround = False
            return True
        self.ground.collision_type = 1
        hand_shape.collision_type = 2
        handler = self.space.add_collision_handler(1, 2)
        handler.begin = on_collision_begin
        handler.separate = on_collision_separate

    def update_simulation(self):
        iterations = 25
        dt = 1.0 / float(self.fps) / float(iterations)
        for x in range(iterations):  # 10 iterations to get a more stable simulation
            self.space.step(dt)

    def isHandContactingGround(self):
        return self.handContactingGround
    
if __name__ == '__main__':
    sim = CrawlingRobotSimulator()
    sim.init()
    sim.run()