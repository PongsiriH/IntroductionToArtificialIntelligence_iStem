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
        # self.space.damping = 0.999 # to prevent it from blowing up.
        # Pymunk physics coordinates start from the lower right-hand corner of the screen.
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
                        sim.chassis_b.position = sim.chassisXY
                        sim.reset_bodies(sim.chassisXY)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                        new_angle = sim.motor_ba1Right.angle() + np.deg2rad(40)
                        sim.motor_ba1Right.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_a:
                        new_angle = sim.motor_ba1Right.angle() - np.deg2rad(40)
                        sim.motor_ba1Right.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                        new_angle = sim.motor_ac1Right.angle() - np.deg2rad(20)
                        sim.motor_ac1Right.set_target_angle(new_angle)
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_w:
                        new_angle = sim.motor_ac1Right.angle() + np.deg2rad(20)
                        sim.motor_ac1Right.set_target_angle(new_angle)
                    elif event.type == pygame.KEYUP:
                        sim.motor_ac1Right.rate = 0
                        sim.motor_ba1Right.rate = 0
            for motor in sim.motor_list:
                motor.update()
            sim.update_simulation()
            sim.draw()
    
    def get_states(self):
        position_x = self.chassis_b.position[0]
        ba1Angle = self.motor_ba1Right.angle()
        ac1Angle = self.motor_ac1Right.angle()
        return position_x, ba1Angle, ac1Angle
    
    def reset_bodies(self,chassisXY):
        legWd_a = 75
        legWd_b = 75
        chWd = 70

        self.chassis_b.position = chassisXY
        self.rightLeg_1a_body.position = chassisXY + ((chWd / 2) + (legWd_a / 2), 0)
        self.rightLeg_1a_body.angle = 0
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a / 2) + (legWd_b / 2), 0)
        self.rightLeg_1b_body.angle = 0

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
        self.chassisXY = Vec2d(20, self.ground_y + 20)
        chWd = 70
        chHt = 50
        chassisMass = 5

        legWd_a = 60
        legHt_a = 8
        legWd_b = 28
        legHt_b = 12
        legMass = 1

        self.min_rightLeg_1a_angle = np.deg2rad(-10)
        self.max_rightLeg_1a_angle = np.deg2rad(10)
        self.min_rightLeg_1b_angle = np.deg2rad(-140)
        self.max_rightLeg_1b_angle = np.deg2rad(0)
        min_rightLeg_1a_angle = self.min_rightLeg_1a_angle
        max_rightLeg_1a_angle = self.max_rightLeg_1a_angle
        min_rightLeg_1b_angle = self.min_rightLeg_1b_angle
        max_rightLeg_1b_angle = self.max_rightLeg_1b_angle
        
        motor_max_force = np.inf
        motor_max_rate = 5
        motor_p_gain = 10
        # ---chassis
        self.chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        self.chassis_b.position = self.chassisXY
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_b, (chWd, chHt))
        self.chassis_shape.color = 200, 200, 200, 100
        self.chassis_shape.friction = 0.1
        print("chassis position")
        print(self.chassis_b.position)
        # ---chassis wheel
        # moment = pymunk.moment_for_circle(10, 1, 5)
        # cb = pymunk.Body(1, moment)
        # c = pymunk.Circle(cb, 15)
        # c.friction = 50
        # c.elasticity = 0
        # cb.position = self.chassisXY + Vec2d(-chWd/2+10, -chHt/2+10)
        # p = pymunk.PinJoint(self.chassis_b, cb, (-chWd/2+10, -chHt/2+10))
        # self.space.add(cb, c, p)

        # ---first right leg a
        legMass = 0.2
        self.rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.rightLeg_1a_body.position = self.chassisXY + ((chWd / 2) + (legWd_a / 2), 0)
        rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))
        rightLeg_1a_shape.color = 255, 0, 0, 100
        rightLeg_1a_shape.friction = 1
        rightLeg_1a_shape.elasticity = 0
        # ---first right leg b
        self.rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_segment(legMass, (0, 0), (legWd_b, 0), 2))
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a / 2) + (legWd_b / 2), 0)
        # Define the start and end points of the segment relative to the body's center
        start_point = (-legWd_b / 2, 0)
        end_point = (legWd_b / 2, 0)
        rightLeg_1b_shape = pymunk.Segment(self.rightLeg_1b_body, start_point, end_point, legHt_b / 2)
        rightLeg_1b_shape.color = 0, 255, 0, 100
        rightLeg_1b_shape.friction = 1
        rightLeg_1b_shape.elasticity = 0
        # ---link right leg b with right leg a
        pj_ba1Right = pymunk.PinJoint(self.rightLeg_1b_body, self.rightLeg_1a_body, (-legWd_b / 2, 0),
                                      (legWd_a / 2, 0))  # anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = ServoMotor(self.rightLeg_1b_body, self.rightLeg_1a_body, motor_max_force,motor_max_rate,motor_p_gain)
        self.motor_ba1Right.min_angle = min_rightLeg_1b_angle
        self.motor_ba1Right.max_angle = max_rightLeg_1b_angle
        # ---link right leg a with chassis
        pj_ac1Right = pymunk.PinJoint(self.rightLeg_1a_body, self.chassis_b, (-legWd_a / 2, 0), (chWd / 2, 0))
        self.motor_ac1Right = ServoMotor(self.rightLeg_1a_body, self.chassis_b, motor_max_force,motor_max_rate,motor_p_gain)
        self.motor_ac1Right.min_angle = min_rightLeg_1a_angle
        self.motor_ac1Right.max_angle = max_rightLeg_1a_angle
        self.space.add(self.chassis_b, self.chassis_shape)
        self.space.add(self.rightLeg_1a_body, rightLeg_1a_shape)
        self.space.add(self.rightLeg_1b_body, rightLeg_1b_shape)
        # self.space.add(pj_ba1left, motor_ba1Left, pj_ac1left, motor_ac1Left)
        self.space.add(pj_ba1Right, self.motor_ba1Right, pj_ac1Right, self.motor_ac1Right)

        self.motor_list = [self.motor_ba1Right,self.motor_ac1Right]
        # ---prevent collisions with ShapeFilter
        shape_filter = pymunk.ShapeFilter(group=1)
        self.chassis_shape.filter = shape_filter
        rightLeg_1a_shape.filter = shape_filter
        rightLeg_1b_shape.filter = shape_filter
        # c.filter = shape_filter

        # ---collision handler
        def on_collision_begin(arbiter, space, data):
            self.handContactingGround = True
            return True
        def on_collision_separate(arbiter, space, data):
            self.handContactingGround = False
            return True
        self.ground.collision_type = 1
        rightLeg_1b_shape.collision_type = 2
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