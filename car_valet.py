import pygame
from pygame.locals import *
from queue import PriorityQueue
from itertools import count
import numpy as np
from pygame.locals import Rect
import math

grid_size = 100

cell_width = 1000 // grid_size
cell_height = 1000 // grid_size


## rev 2 - modifying model size to scale it down

class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y,color,alpha=None):
        pygame.sprite.Sprite.__init__(self)
        self.pixels_per_meter = 35
        self.x = x
        self.y = y
        self.speed = 5  # Adjust as needed
        self.angle = 0  # initial heading angle
        self.L = 2.8 * self.pixels_per_meter
        self.height = 50
        self.max_speed = 5
        self.max_steering_angle = 30


        # Create a surface for the vehicle
        self.original_image = pygame.Surface([self.L, self.height], pygame.SRCALPHA)
        self.original_image.fill(color)
        self.original_image.set_alpha(alpha)
        self.image = self.original_image
        self.rect = self.image.get_rect(center = (self.x,self.y))
        self.mask = pygame.mask.from_surface(self.image)

        #self.rect = pygame.draw.rect(self.surface, (0, 0, 255), Rect(0, 0, self.L, self.height))  # Blue rectangle

    def set_pose(self,pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def draw(self,screen):
        # Draw the rotated vehicle surface
        self.image = pygame.transform.rotate(self.original_image, -np.rad2deg(self.angle))
        self.rect = self.image.get_rect(center = (self.x, self.y))
        #rect = rotated_surface.get_rect(midtop = (pose[0]+self.L/2,pose[1]-self.height/2) ,midleft = (pose[0],pose[1]),midbottom =(pose[0]+self.L/2,pose[1]+self.height/2) , midright = (pose[0]+self.L,pose[1]))#center=(pose[0], pose[1])) # x = pose[0], y=pose[1]-self.height/2
        #rect = rotated_surface.get_rect(center = (pose[0],pose[1]))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)

    # def update(self):
    #     self.draw()

        #screen.blit(rotated_surface, self.rect)





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
        self.x = mouse_pose[0] #- self.width/2
        self.y = mouse_pose[1] #- self.height/2

    # def update(self):
    #     self.draw()
    def draw(self,screen):
        # Implement obstacle drawing logic
        self.image = self.original_image
        self.rect = self.image.get_rect(center=(self.x, self.y))
        self.mask = pygame.mask.from_surface(self.image)
        screen.blit(self.image,self.rect)
        #self.rect = pygame.draw.rect(screen, (255, 255, 255), (self.x, self.y, self.width, self.height))

class Node:
    def __init__(self, state, parent):
        self.state = state
        #self.theta = theta
        self.parent = parent
        self.h = 0
        self.g = 0
        self.f = 0

    def __eq__(self, other):
        return self.state == other.state

    def __lt__(self, other):
        return self.h < other.h

    # def __hash__(self):
    #     # We use the hash value of the state
    #     # stored in the node instead of the node
    #     # object itself to quickly search a node
    #     # with the same state in a Hash Table
    #     return hash(self.state)

def pixel_to_grid(x, y):
    grid_x = x // cell_width
    grid_y = y // cell_height
    return grid_x, grid_y

def grid_to_pixel(gx,gy):
    x = gx * cell_width
    y = gy * cell_height
    return x,y


def distance(current_pos, end_pos):
    return np.sqrt((end_pos[0] - current_pos[0])**2 + (end_pos[1]-current_pos[1])**2) #+ 2 * abs(current_pos[2] - end_pos[2])

def cost(current_pos, start_pos):
    return np.sqrt((start_pos[0] - current_pos[0])**2 + (start_pos[1]-current_pos[1])**2)

def angular_cost(current_theta,goal_theta):
    return abs(goal_theta-current_theta)


class Planner:

    def __init__(self,start, end, obstacles):
        self.meters2pixels = 35
        self.start_node = Node(start,None)
        self.goal_node = Node(end, None)
        self.open_list = PriorityQueue()
        self.open_list_visuals = []
        #self.open_list = []
        self.closed_list = []
        self.unique = count()
        self.open_list.put((self.start_node.h, self.start_node.f, self.start_node))
        #self.open_list.append(self.start_node)
        self.solution_node = None
        self.solution_found = False
        self.node_sequence = []
        self.pose_sequence = []
        self.iterations = 0
        self.obstacles = obstacles

        self.original_image = pygame.Surface([280, 100], pygame.SRCALPHA)

        #self.original_image.fill((0, 0, 255))

    def __call__(self, screen):
        h, f, current_node = self.open_list.get()
        self.open_list_visuals.append(current_node)

        # current_idx = 0
        # current_node = self.open_list[current_idx]
        # for idx, item in enumerate(self.open_list):
        #     if item.f < current_node.f:
        #         current_node = item
        #         current_idx = idx

        #self.open_list.pop(current_idx)
        self.closed_list.append(current_node)
        print(str(current_node.state) + "    " + str(self.goal_node.state) + "    " + str(distance(current_node.state,self.goal_node.state)))
        #print(str(current_node.state) + " " + str(round(current_node.theta,2)) + "    " + str(self.goal_node.state) +" " + str(self.goal_node.theta))

        if self.iterations != 0:
            #if current_node.state == self.goal_node.state or distance(current_node.state,self.goal_node.state) < 10.0:
            if self.angular_diff(current_node.state[2],self.goal_node.state[2]) < 0.15 and distance(current_node.state, self.goal_node.state) < 35:
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


        # if distance(current_node.state,self.goal_node.state) < 10:
        #     print('found')
        #     self.solution_found = True
        #     print('getting path')
        #
        #     path = []
        #     while current_node is not None:
        #         path.append(current_node)
        #         self.sequence.append(current_node)
        #         print(current_node)
        #         print(type(current_node))
        #         print(current_node.parent)
        #         print(type(current_node.parent))
        #         current_node = current_node.parent

        children = []

        #inputs = [(-1, -45), (-1, -15), (-1, 0), (-1, 15), (-1, 45),
        #           (1, -45), (1, -15), (1, 0), (1, 15), (1, 45)]

        inputs = [(-1, -35), (-1,0),(-1,35), (1,-35), (1,0), (1,35)]

        L = 2.8*self.meters2pixels
        dt = 2
        for num, input in enumerate(inputs):
            v = input[0]*self.meters2pixels
            w = input[1]
            beta = (v/L)*np.tan(np.deg2rad(w))*dt
            theta_new = (current_node.state[2] + beta)

            x_new = round(current_node.state[0] + v * np.cos(theta_new) * dt)
            y_new = round(current_node.state[1] + v * np.sin(theta_new) * dt)
            new_state = (x_new,y_new,round(theta_new,2))

            # if self.is_out_of_bounds(x_new, y_new) or self.is_colliding(screen,x_new,y_new,theta_new):
            #     continue
            if self.is_not_valid(screen,x_new,y_new,theta_new):
                continue
            else:
            #screen.set_at((x_new, y_new), (255, 255, 255))

                new_node = Node(new_state,current_node)
                new_node.h = distance(current_node.state,self.goal_node.state)
                #new_node.g = current_node.g + cost(current_node.state,self.start_node.state)
                new_node.g = current_node.g + cost(current_node.state,new_node.state)# + angular_cost(new_node.state[2],self.goal_node.state[2]) # WORKS
                new_node.f = new_node.g + new_node.h
                children.append(new_node)

        for child in children:
            s = child.state
            flag = False
            for closed in self.closed_list:
                if closed == child:
                    flag = True
                    break
            # for open in self.open_list:
            #     if open == child and child.g >= open.g:
            #         break
            if flag is True:
                continue
            elif flag is False:
                self.open_list.put((child.h, child.f,child))
                #self.open_list.append(child)
        self.iterations += 1

    def is_out_of_bounds(self,x,y):
        if 0 <= x <= 1000 or 0 <= y <= 1000:
            return False
        return True

    def is_rect_out_of_bounds(self,topleft,bottomleft,topright,bottomright):
        if 0 <= topleft[0] <= 1000 or 0 <= topleft[1] <= 1000 or 0 <= bottomleft[0] <= 1000 or 0 <= bottomleft[1] <= 1000 or 0 <= topright[0] <= 1000 or 0 <= topright[1] <= 1000 or 0 <= bottomright[0] <= 1000 or 0 <= bottomright[1] <= 1000:
            return False
        else:
            return True

    def is_colliding(self,screen,x,y,angle):
        test_vehicle = Vehicle(0,0,((0,0,255)),255)
        test_vehicle.set_pose((x,y,angle))
        test_vehicle.draw(screen)

        left_car_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[0])
        right_car_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[2])
        center_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[1])
        if left_car_collide or right_car_collide or center_collide:
            print('predicting collision')
            return True
        return False

    def is_not_valid(self,screen,x,y,angle):
        test_vehicle = Vehicle(0,0,((0,0,255)),255)
        test_vehicle.set_pose((x,y,angle))
        test_vehicle.draw(screen)

        left_car_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[0])
        right_car_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[2])
        center_collide = pygame.sprite.collide_mask(test_vehicle,self.obstacles[1])
        if left_car_collide or right_car_collide or center_collide:
            print('predicting collision')
            return True
        elif self.is_rect_out_of_bounds(test_vehicle.rect.topleft,test_vehicle.rect.bottomleft,test_vehicle.rect.topright, test_vehicle.rect.bottomright):
            print('out of bounds')
            return True
        return False


    def angular_diff(self,current,goal):
        return abs(goal-current)

    def set_pixels(self,screen):
        screen.set_at((self.start_node.state[0],self.start_node.state[1]),(255,255,255))
        for opened in self.open_list_visuals:
            screen.set_at((opened.state[0], opened.state[1]), (0, 0, 255))
        for closed in self.closed_list:
            screen.set_at((closed.state[0],closed.state[1]),(255,0,0))
        for path in self.node_sequence:
            screen.set_at((path.state[0], path.state[1]), (0, 255, 0))
        screen.set_at((self.goal_node.state[0], self.goal_node.state[1]), (255, 255, 255))




class Simulation:
    def __init__(self, width, height):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Car Simulation")

        self.vehicle = Vehicle(150, 100,(0,0,255),255)
        self.obstacles = [Obstacle(150, 950, 300, 100),
                          Obstacle(width/2+100, height/2-150, 150, 150),
                          Obstacle(850, 950, 300, 100)]


    def run(self):
        # start near 10,50 and go to 500, 520 ish planner = Planner((10,50,-0),(500,520,0.0))
        # park manuver
        # 500, 520 to 750 700 deg2rad(35)  planner = Planner((500,575,70),(825,700,0.0))
        # 750, 700 to 600 800  planner = Planner((850,700,0),(525,850,0.0))

        #

        # planner = Planner((10,50,-0),(800,700,0.0),self.obstacles) THIS WAS USED WITH ANGULAR COST AND GOT TO THE LOCATION
        planner = Planner((10, 50, -0), (500, 950, 0.0), self.obstacles)
        obj = Obstacle(0,0,100,100)
        clock = pygame.time.Clock()
        counter = 0

        all_sprites_list = pygame.sprite.Group([self.vehicle] + self.obstacles)
        moving_sprite_box = pygame.sprite.Group([obj])

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
                pass
                planner(self.screen)
            if planner.solution_found and counter < len(planner.pose_sequence):
                self.vehicle.set_pose(planner.pose_sequence[counter])
                self.vehicle.draw(self.screen)
                counter += 1
                pygame.time.delay(500)
            elif planner.solution_found:
                self.vehicle.set_pose(planner.pose_sequence[-1])
                self.vehicle.draw(self.screen)
                #print(planner.pose_sequence)

            #self.vehicle.draw(self.screen, (300,300,45))
            #self.vehicle.draw(self.screen)


            for obstacle in self.obstacles:
                obstacle.draw(self.screen)



            #self.vehicle.set_pose((300, 300, 45))
            #self.static_objects.update()
            obj.update()
            #self.vehicle.update()
            self.vehicle.draw(self.screen)
            #self.static_objects.draw(self.screen)
            obj.draw(self.screen)


            #collide_main = pygame.sprite.collide_mask(self.vehicle,obj)
            #left_car_collide = pygame.sprite.collide_mask(self.obstacles[0],obj)
            #right_car_collide = pygame.sprite.collide_mask(self.obstacles[2],obj)
            #center_collide = pygame.sprite.collide_mask(self.obstacles[1],obj)



            #if collide_main or left_car_collide or right_car_collide or center_collide:
            #    print('collision')




            planner.set_pixels(self.screen)
            pygame.display.flip()

            # print(dt)




if __name__ == "__main__":
    # pixel_x = 456
    # pixel_y = 789
    #
    # grid_x, grid_y = pixel_to_grid(pixel_x, pixel_y)
    #
    # print(f"Node at pixel ({pixel_x}, {pixel_y}) is in grid cell ({grid_x}, {grid_y})")
    #
    #
    # gx = 45
    # gy = 78
    # x, y = grid_to_pixel(gx,gy)
    #
    # print(f"Grid cell ({gx}, {gy}) is pixel  ({x}, {y})")

    simulation = Simulation(1000, 1000)
    simulation.run()