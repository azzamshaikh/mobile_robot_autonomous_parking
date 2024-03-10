import pygame
from pygame.locals import *
from queue import PriorityQueue
import numpy as np


# ---------------------------------------Vehicle Class------------------------------------------------------------------


class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y, color, alpha=None):
        pygame.sprite.Sprite.__init__(self)
        self.pixels_per_meter = 35
        self.x = x
        self.y = y
        self.angle = None
        self.L = 2.8 * self.pixels_per_meter
        self.height = 50
        self.original_image = pygame.Surface([self.L, self.height], pygame.SRCALPHA)
        self.original_image.fill(color)
        self.original_image.set_alpha(alpha)
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def draw(self,screen):
        self.image = pygame.transform.rotate(self.original_image, -np.rad2deg(self.angle))
        self.rect = self.image.get_rect(center = (self.x + (self.L/2)*np.cos(self.angle), self.y + (self.L/2)*np.sin(self.angle)))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)

# ---------------------------------------Obstacle Class-----------------------------------------------------------------


class Obstacle(pygame.sprite.Sprite):
    def __init__(self, x, y, width, height):
        pygame.sprite.Sprite.__init__(self)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.original_image = pygame.Surface([self.width,self.height],pygame.SRCALPHA)
        self.original_image.fill((255,255,255))
        self.image = self.original_image
        self.rect = self.image.get_rect(center = (self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)

    def updatepose(self, mouse_pose):
        self.x = mouse_pose[0]
        self.y = mouse_pose[1]

    def draw(self,screen):
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)

# ---------------------------------------Node Class---------------------------------------------------------------------


class Node:
    def __init__(self, state, parent):
        self.state = state
        self.parent = parent
        self.h = 0
        self.g = 0
        self.f = 0

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.h < other.h

# ---------------------------------------Static Functions for Heuristic Calculation-------------------------------------


def distance(current_pos, end_pos):
    return np.sqrt((end_pos[0] - current_pos[0])**2 + (end_pos[1]-current_pos[1])**2)


def cost(current_pos, start_pos):
    return np.sqrt((start_pos[0] - current_pos[0])**2 + (start_pos[1]-current_pos[1])**2)


def angular_cost(current_theta,goal_theta):
    return abs(goal_theta-current_theta)

# ---------------------------------------Planner Class------------------------------------------------------------------


class Planner:

    def __init__(self,start, end, obstacles):
        self.start_node = Node(start,None)
        self.goal_node = Node(end, None)
        self.open_list = PriorityQueue()
        self.open_list_visuals = []
        self.closed_list = []
        self.open_list.put((self.start_node.f, self.start_node.h, self.start_node))
        self.solution_node = None
        self.solution_found = False
        self.node_sequence = []
        self.pose_sequence = []
        self.iterations = 0
        self.obstacles = obstacles
        self.original_image = pygame.Surface([280, 100], pygame.SRCALPHA)

        self.motion_primitives = [(-1, -35), (-1, 0), (-1, 35), (1, -35), (1, 0), (1, 35)]
        self.meters2pixels = 35
        self.L = 2.8 * self.meters2pixels
        self.dt = 3

        self.test_vehicle = Vehicle(0, 0, (0, 0, 255), 0)

    def __call__(self, screen):
        f, h, current_node = self.open_list.get()
        self.open_list_visuals.append(current_node)

        print(str(current_node.state) + "    " + str(self.goal_node.state) + "    " +
              str(distance(current_node.state,self.goal_node.state)))

        if self.iterations != 0:
            if (self.angular_diff(current_node.state[2],self.goal_node.state[2]) < 0.15 and
                    distance(current_node.state, self.goal_node.state) < self.meters2pixels*2):
                print('found')
                current = current_node
                self.solution_found = True
                if self.solution_found is True:
                    print('getting path')
                    node_path = []
                    pose_path = []
                    while current is not None:
                        node_path.append(current)
                        pose_path.append(current.state)
                        current = current.parent
                    self.node_sequence = node_path[::-1]
                    self.pose_sequence = pose_path[::-1]

        self.closed_list.append(current_node)

        children = []

        for motion in self.motion_primitives:
            v = motion[0]*self.meters2pixels
            w = motion[1]
            beta = (v/self.L)*np.tan(np.deg2rad(w))*self.dt
            theta_new = round((current_node.state[2] + beta),2)

            x_new = round(current_node.state[0] + v * np.cos(theta_new) * self.dt)
            y_new = round(current_node.state[1] + v * np.sin(theta_new) * self.dt)
            new_state = (x_new,y_new,theta_new)

            if self.is_not_valid(screen,new_state):
                continue
            else:
                new_node = Node(new_state,current_node)
                new_node.h = round(distance(current_node.state,self.goal_node.state),2)
                new_node.g = round(current_node.g + angular_cost(new_node.state[2], self.goal_node.state[2])
                                   + cost(current_node.state, new_node.state), 2)
                new_node.f = round(new_node.g + new_node.h,2)
                children.append(new_node)

        for child in children:
            if child not in self.closed_list and not any([node == child for f, h, node in self.open_list.queue]):
                self.open_list.put((child.f, child.h,child))

        self.iterations += 1

    def is_rect_out_of_bounds(self, topleft, bottomleft, topright, bottomright):
        if (0 <= topleft[0] <= 1000 and 0 <= topleft[1] <= 1000 and 0 <= bottomleft[0] <= 1000 and 0 <= bottomleft[1] <=
                1000 and 0 <= topright[0] <= 1000 and 0 <= topright[1] <= 1000 and 0 <= bottomright[0] <= 1000 and 0 <=
                bottomright[1] <= 1000):
            return False
        else:
            return True

    def is_not_valid(self, screen, new_state):
        self.test_vehicle.set_pose((new_state[0], new_state[1], new_state[2]))
        self.test_vehicle.draw(screen)

        left_car_collide = pygame.sprite.collide_mask(self.test_vehicle, self.obstacles[0])
        right_car_collide = pygame.sprite.collide_mask(self.test_vehicle, self.obstacles[2])
        center_collide = pygame.sprite.collide_mask(self.test_vehicle, self.obstacles[1])
        if left_car_collide or right_car_collide or center_collide:
            print('predicting collision')
            return True
        elif self.is_rect_out_of_bounds(self.test_vehicle.rect.topleft,self.test_vehicle.rect.bottomleft,
                                        self.test_vehicle.rect.topright, self.test_vehicle.rect.bottomright):
            print('out of bounds')
            return True
        return False

    def angular_diff(self, current, goal):
        return abs(goal-current)

    def set_pixels(self,screen):
        screen.set_at((self.start_node.state[0],self.start_node.state[1]),(255,255,255))
        for opened in self.open_list_visuals:
            screen.set_at((opened.state[0], opened.state[1]), (0, 0, 255))
        for closed in self.closed_list:
            screen.set_at((closed.state[0],closed.state[1]),(255,255,0))
        for path in self.node_sequence:
            screen.set_at((path.state[0], path.state[1]), (255, 0, 255))
        screen.set_at((self.goal_node.state[0], self.goal_node.state[1]), (255, 255, 255))

# ---------------------------------------Simulation Class---------------------------------------------------------------


class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Car Simulation")

        self.vehicle = Vehicle(50, 50,(0,0,255),255)
        self.obstacles = [Obstacle(150, 925, 300, 150),
                          Obstacle(width/2+175, height/2-150, 300, 300),
                          Obstacle(850, 925, 300, 150)]


    def run(self):
        planner = Planner((50, 50, -0), (500, 950, 0.0), self.obstacles)
        obj = Obstacle(0,0,100,100)
        clock = pygame.time.Clock()
        counter = 0
        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    return

            mouse_pose = pygame.mouse.get_pos()
            obj.updatepose(mouse_pose)

            dt = clock.tick(60) / 1000.0  # Convert milliseconds to seconds
            self.screen.fill((0, 0, 0))  # Clear the screen
            if not planner.solution_found:
                planner(self.screen)
            if planner.solution_found and counter < len(planner.pose_sequence):
                self.vehicle.set_pose(planner.pose_sequence[counter])
                self.vehicle.draw(self.screen)
                counter += 1
                pygame.time.delay(100)
            elif planner.solution_found:
                self.vehicle.set_pose(planner.pose_sequence[-1])
                self.vehicle.draw(self.screen)
                for idx in range(0,len(planner.pose_sequence)-1):
                    pygame.draw.line(self.screen, (255, 0, 255), planner.pose_sequence[idx][:2],
                                     planner.pose_sequence[idx+1][:2], width=1)

            for obstacle in self.obstacles:
                obstacle.draw(self.screen)

            planner.set_pixels(self.screen)
            pygame.display.flip()


if __name__ == "__main__":
    simulation = Simulation(1000, 1000)
    simulation.run()
