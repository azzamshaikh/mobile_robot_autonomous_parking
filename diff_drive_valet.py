import math
from collections import deque
import pygame
from itertools import islice

from utils import *


class Controller:
    def __init__(self, start,goal=None,num_waypoints=None):
        self.state = start
        self.num_waypoints = num_waypoints
        self.current_waypoint = 1
        self.goal = goal

        self.Kp_pose = 10
        self.Ki_pose = 0.5
        self.Kd_pose = 0.5

        self.Kp_theta = 5
        self.Ki_theta = 0.1
        self.Kd_theta = 0.1

        self.prev_v = None
        self.prev_w = None

        self.pose_total_error = 0
        self.pose_previous_error = 0

        self.theta_total_error = 0
        self.theta_previous_error = 0

        self.pose_goal = False
        self.theta_goal = False
        self.at_goal = False

        self.v_max = 10

        self.position_tolerance = 0.1
        self.orientation_tolerance = 0.01

    def __call__(self, state):
        if not self.pose_goal:
            v, w = self.positionPID(state)
        if self.pose_goal and self.current_waypoint != self.num_waypoints:
            self.at_goal = True
            # v = self.prev_v
            # w = self.prev_w
            v = 0
            w = 0
        if self.pose_goal and not self.theta_goal and self.current_waypoint == self.num_waypoints:
            v, w = self.orientionPID(state)
        if self.pose_goal is True and self.theta_goal is True:
            self.at_goal = True
            v = 0
            w = 0
            print('Parked!')
        return v,w

    def update_waypoints(self,num_waypts):
        self.num_waypoints = num_waypts

    def update_goal(self,new_goal):
        self.current_waypoint += 1
        self.goal = new_goal
        self.at_goal = False
        self.pose_goal = False
        self.theta_goal = False

    def set_goal(self,goal):
        self.goal = goal

    def PID(self,state,waypoint):
        pass

    def positionPID(self,state):
        self.position_goal(state)
        dx = self.goal[0] - state[0]
        dy = self.goal[1] - state[1]
        dtheta = np.arctan2(dy,dx)
        pose_error = dtheta - state[2]
        pose_error = np.arctan2(np.sin(pose_error),np.cos(pose_error))

        pose_e_P = pose_error
        pose_e_I = self.pose_total_error + pose_error
        pose_e_D = pose_error - self.pose_previous_error

        w = self.Kp_pose*pose_e_P + +self.Ki_pose*pose_e_I + self.Ki_pose*pose_e_D
        v = 15
        self.pose_previous_error = pose_error
        # print(w)
        # if v > self.v_max:
        #     v = self.v_max
        self.prev_v = v
        self.prev_w = w
        return v,w

    def orientionPID(self, state):
        self.orientation_goal(state)

        angle_error = self.goal[2] - state[2]

        omega_e_P = angle_error
        omega_e_I = self.theta_total_error + angle_error
        omega_e_D = angle_error - self.theta_previous_error

        w = self.Kp_theta*omega_e_P + +self.Ki_theta*omega_e_I + self.Kd_theta*omega_e_D
        v = 0.0

        self.theta_previous_error = angle_error

        if v > self.v_max:
            v = self.v_max

        return v,w

    def position_goal(self, state):
        position_error = np.sqrt((state[0]-self.goal[0])**2 + (state[1]-self.goal[1])**2)
        # print(position_error)
        if position_error < self.position_tolerance:
            self.pose_goal = True
        # else:
        #     self.at_goal = False

    def orientation_goal(self, state):
        orientation_error =  self.goal[2] - state[2]
        # print(orientation_error)
        if abs(orientation_error) < self.orientation_tolerance:
            self.theta_goal = True
        # else:
        #     self.at_goal = False


class Objects:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.size = None

    def draw(self,screen):
        raise NotImplementedError


class Pothole(Objects):

    def __init__(self,x,y):
        super(Pothole,self).__init__(x,y)
        self.collision = pygame.Rect(self.x,self.y,200,200)

    def draw(self,screen):
        pothole = pygame.image.load("pothole.png")
        self.size = (200,200)
        pothole = pygame.transform.smoothscale(pothole, self.size)
        screen.blit(pothole,(self.x, self.y))


class Vehicle(Objects):

    def __init__(self,x,y):
        super(Vehicle,self).__init__(x,y)
        self.collision = pygame.Rect(self.x,self.y,200,100)

    def draw(self,screen):
        car = pygame.image.load("car.png")
        car = pygame.transform.grayscale(car)
        screen.blit(car,(self.x, self.y))
# ---------------------------------------------------------------------------------------------------


def out_of_bound(idx):
    # Static function to check if the indices for the neighbors are valid
    if 0 <= idx[0] < 800 and 0 <= idx[1] < 600:
        return False
    else:
        # print("The following neighbor at " + str(idx) + " is out of bounds.")
        return True


def is_obstacle(idx,obs):
    # for point index
    # for o in obs:
    #     if o.collidepoint(idx[0], idx[1]):
    #         return True
    # return False

    bot = pygame.Rect(0, 0, 60, 60)
    bot.center = (idx[0], idx[1])
    return bot.collidelistall(obs)


class Search:
    def __init__(self, initial, obs, goal=None):
        # Initialization of GridSearch object
        self.initial = initial
        self.obs = obs
        self.goal = goal

    def action(self, state):
        # Determine any possible actions based on the current state
        possible_actions = ['Up', 'UpLeft', 'Left', 'DownLeft', 'Down', 'DownRight', 'Right', 'UpRight']

        x, y = state[0], state[1]

        # add logic to prevent obstacle and bound collisions
        if out_of_bound((x, y+1)) or is_obstacle((x, y+1),self.obs):
            if 'Up' in possible_actions:
                possible_actions.remove('Up')
        if out_of_bound((x-1, y+1)) or is_obstacle((x-1, y+1),self.obs):
            if 'UpLeft' in possible_actions:
                possible_actions.remove('UpLeft')
        if out_of_bound((x-1, y)) or is_obstacle((x-1, y),self.obs):
            if 'Left' in possible_actions:
                possible_actions.remove('Left')
        if out_of_bound((x-1, y-1)) or is_obstacle((x-1, y-1),self.obs):
            if 'DownLeft' in possible_actions:
                possible_actions.remove('DownLeft')
        if out_of_bound((x, y-1)) or is_obstacle((x, y-1),self.obs):
            if 'Down' in possible_actions:
                possible_actions.remove('Down')
        if out_of_bound((x+1, y-1)) or is_obstacle((x+1, y-1),self.obs):
            if 'DownRight' in possible_actions:
                possible_actions.remove('DownRight')
        if out_of_bound((x+1, y)) or is_obstacle((x+1, y),self.obs):
            if 'Right' in possible_actions:
                possible_actions.remove('Right')
        if out_of_bound((x+1, y+1)) or is_obstacle((x+1, y+1),self.obs):
            if 'UpRight' in possible_actions:
                possible_actions.remove('UpRight')

        return possible_actions

    def result(self, state, action):
        # Return the resulting new state from the action applied to the current state

        x = state[0]
        y = state[1]
        possible_location = list()

        # Move Up
        if action == 'Up':
            possible_location = (x, y+1)

        elif action == 'UpLeft':
            possible_location = (x-1, y+1)

        elif action == 'Left':
            possible_location = (x-1, y)

        elif action == 'DownLeft':
            possible_location = (x-1, y-1)

        elif action == 'Down':
            possible_location = (x, y-1)

        elif action == 'DownRight':
            possible_location = (x+1, y-1)

        elif action == 'Right':
            possible_location = (x+1, y)

        elif action == 'UpRight':
            possible_location = (x+1, y+1)

        state = possible_location

        return state

    def path_cost(self, cost_so_far, action):
        # Determine the path cost for a certain action
        cost = None
        if action == 'Up':
            cost = 1
        elif action == 'UpLeft':
            cost = 1
        elif action == 'Left':
            cost = 1
        elif action == 'DownLeft':
            cost = 1
        elif action == 'Down':
            cost = 1
        elif action == 'DownRight':
            cost = 1
        elif action == 'Right':
            cost = 1
        elif action == 'UpRight':
            cost = 1
        return cost_so_far + cost

    def h(self, node):
        """h function is straight-line distance from a node's state to goal."""
        # print("Distance: is" + str(distance(node.state, self.goal)))
        return distance(node.state[:2],self.goal)

    def goal_test(self, state):
        # Test to see if the current state is the goal state
        if state == self.goal:
            return True
        else:
            return False


class Node:
    # The Node class contains the information about each node/cell that the GridSearch algorithm traverses

    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, search):
        # Search neighboring cells
        return [self.child_node(search, action) for action in search.action(self.state)]

    def child_node(self, search: Search, action):
        # Obtain the child node of the current cell
        next_state = search.result(self.state, action)
        next_node = Node(next_state, self, action, search.path_cost(self.path_cost,action))
        return next_node

    def solution(self):
        # Return the solution of the path
        return [node.state for node in self.path()[1:]]

    def path(self):
        # Obtain the path back
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)


class Agent:

    def __init__(self, problem):
        self.seq_node = None
        self.seq = []
        self.problem = problem
        self.h = memoize(self.problem.h, 'h')
        node = Node(self.problem.initial)
        self.frontier = PriorityQueue('min', self.h)
        self.frontier.append(node)
        self.explored = set()
        self.goal_found = False
        self.waypoints = None

    def __call__(self):
        node = self.frontier.pop()
        if self.problem.goal_test(node.state):
            self.seq_node = node
            self.goal_found = True
        self.explored.add(node.state)
        for child in node.expand(self.problem):
            if child.state not in self.explored and child not in self.frontier:
                self.frontier.append(child)
            elif child in self.frontier:
                if self.h(child) < self.frontier[child]:
                    del self.frontier[child]
                    self.frontier.append(child)
        self.seq = node.solution()

    def downsample(self):
        downsampled = []
        points = int(len(self.seq)*0.05)
        inc = len(self.seq) / points
        inc_total = 0
        for _ in range(0, points):
            downsampled.append(self.seq[math.floor(inc_total)])
            inc_total += inc
        downsampled.pop(0)
        goal = list(self.seq[-1])
        goal.append(0)
        downsampled.append(goal)
        return downsampled


    def add_pixel(self,screen,initial, goal, start_color,goal_color,path_color,expl_color):
        screen.set_at((initial[0], initial[1]),start_color)
        screen.set_at((goal[0], goal[1]), goal_color)
        for expl in self.explored:
            screen.set_at((expl[0], expl[1]), expl_color)
        for pixels in self.seq:
            screen.set_at((pixels[0], pixels[1]), path_color)

    def search(self,problem):
        h = memoize(problem.h, 'h')
        self.seq_node, self.explored = self.best_first_graph_search(problem, lambda n: n.path_cost + h(n), display=False)
        try:
            self.seq = self.seq_node.solution()
        except AttributeError:
            print('No solution present.')
        print(self.seq)

    @staticmethod
    def dijkstra(problem):
        node = Node(problem.initial)
        if problem.goal_test(node.state):
            print("Success! Goal is found at " + str(node.state))
            return node
        frontier = deque([node])
        explored = {problem.initial}
        while frontier:
            node = frontier.pop()
            if problem.goal_test(node.state):
                print('Success! Reached the goal state of ' + str(problem.goal))
                return node, explored
            for child in node.expand(problem):
                s = child.state
                if s not in explored:
                    explored.add(s)
                    frontier.appendleft(child)
        print('Failed to find goal.')
        return None, explored

    @staticmethod
    def best_first_graph_search(problem, f, display=False):
        """Search the nodes with the lowest f scores first.
        You specify the function f(node) that you want to minimize; for example,
        if f is a heuristic estimate to the goal, then we have greedy best
        first search; if f is node. depth then we have breadth-first search.
        There is a subtlety: the line "f = memoize(f, 'f')" means that the f
        values will be cached on the nodes as they are computed. So after doing
        the best first search you can examine the f values of the path returned."""
        f = memoize(f, 'f')
        node = Node(problem.initial)
        frontier = PriorityQueue('min', f)
        frontier.append(node)
        explored = set()
        while frontier:
            node = frontier.pop()
            if problem.goal_test(node.state):
                if display:
                    print(len(explored), "paths have been expanded and", len(frontier), "paths remain in the frontier")
                return node, explored
            explored.add(node.state)
            for child in node.expand(problem):
                if child.state not in explored and child not in frontier:
                    frontier.append(child)
                elif child in frontier:
                    if f(child) < frontier[child]:
                        del frontier[child]
                        frontier.append(child)
        return None, explored


class Robot:
    def __init__(self, start, img, width):
        self.meters2pixels = 3779.52

        self.w = width
        self.x = start[0]
        self.y = start[1]
        self.theta = start[2]
        self.vl = 0.01*self.meters2pixels
        self.vr = 0.01*self.meters2pixels
        self.maxspeed = 0.02 * self.meters2pixels
        self.minspeed = 0.01 * self.meters2pixels

        self.img = pygame.image.load(img)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def draw(self,screen):
        screen.blit(self.rotated,self.rect)

    def get_state(self):
        if self.theta > 2* math.pi or self.theta < -2*math.pi:
            self.theta = 0
        print(str(self.x) + ' ' +  str (self.y)  +' ' +  str(self.theta))
        return (self.x, self.y, self.theta)

    def drive(self,v,w,dt):
        print(str(v) + ' ' + str(w))
        x_dt = v*np.cos(self.theta)
        y_dt = v*np.sin(self.theta)
        theta_dt = w
        # print(dt)
        self.x = self.x + x_dt*dt
        self.y = self.y + y_dt*dt
        self.theta = self.theta + np.arctan2(np.sin(theta_dt*dt),np.cos(theta_dt*dt))
        #result = np.arctan2(np.sin(theta_dt*dt),np.cos(theta_dt*dt))
        #self.theta = np.arctan2(np.sin(self.theta + result),np.cos(self.theta + result))
        # print(self.x)
        # print(self.y)
        # print(self.theta)
        # phi_L, phi_R = self.ddr_ik(v,w)
        # self.vl = phi_L
        # self.vr = phi_R
        #
        # self.x += ((self.vl + self.vr) / 2) * math.cos(self.theta) * dt
        # self.y -= ((self.vl + self.vr) / 2) * math.sin(self.theta) * dt
        # self.theta += (self.vr - self.vl) / self.L * dt
        self.rotated = pygame.transform.rotozoom(self.img,
                                                 math.degrees(-self.theta),1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))
        # print(str(self.x) + " " + str(self.y) + " " + str(self.theta))

class Simulation:

    def __init__(self, dims):
        self.red = (200, 0, 0)
        self.blue = (0, 0, 255)
        self.green = (0, 155, 0)
        self.yellow = (155, 155, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
        self.grey = (127, 127, 127)

        self.width = dims[0]
        self.height = dims[1]
        pygame.init()
        pygame.display.set_caption("Valet - Differential Drive")
        self.screen=pygame.display.set_mode((self.width, self.height))
        self.ticks = 60
        self.running = True
        self.dt = 0
        self.last_time = None
        self.obstacles = []
        self.trail_set=[]

    def trail(self,pos):
        for i in range(0,len(self.trail_set)-1):
            pygame.draw.line(self.screen,self.blue,(self.trail_set[i][0],self.trail_set[i][1]),
                                                      (self.trail_set[i+1][0],self.trail_set[i+1][1]))
        # if self.trail_set.__sizeof__()>20000:
        #     self.trail_set.pop(0)
        self.trail_set.append(pos)

    def main(self,robot_start,goal):
        robot = Robot(robot_start,
                      'diffbot3.png',
                      0.02*3779.52)
        idx = 0
        controller = Controller(robot_start)
        self.last_time = pygame.time.get_ticks()
        self.dt = (pygame.time.get_ticks() - self.last_time) / 1000
        pothole = Pothole(self.width / 2, (self.height / 2) - 150)
        pothole2 = Pothole(100, 100)
        car1 = Vehicle(10, self.height - 110)
        car2 = Vehicle(500, self.height - 110)
        self.obstacles.append(pothole.collision)
        #self.obstacles.append(pygame.Rect(100, 100, 100, 100))
        self.obstacles.append(pothole2.collision)
        self.obstacles.append(car1.collision)
        self.obstacles.append(car2.collision)
        astar = Search(robot_start,self.obstacles, goal)
        agent = Agent(astar)
        pygame.time.wait(1000)



        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            self.last_time = pygame.time.get_ticks()
            self.screen.fill(self.grey)
            pothole.draw(self.screen)
            pothole2.draw(self.screen)
            car1.draw(self.screen)
            car2.draw(self.screen)
            #pygame.draw.rect(self.screen, self.blue, pygame.Rect(100, 100, 100, 100))
            if not agent.goal_found:

                agent()
                if agent.goal_found:
                    agent.waypoints = agent.downsample()
                    controller.update_waypoints(len(agent.waypoints))
                    controller.set_goal(agent.waypoints[0])
            if agent.goal_found and not controller.at_goal:
                print(agent.waypoints)
                print('Going to ' + str(controller.goal))
                v, w = controller(robot.get_state())
                robot.drive(v, w, self.dt)
            if controller.at_goal and idx < len(agent.waypoints)-1:
                print('Updating waypoint!')
                idx += 1
                controller.update_goal(agent.waypoints[idx])
            robot.draw(self.screen)
            self.trail((robot.x, robot.y))
            agent.add_pixel(self.screen, astar.initial, astar.goal, self.white, self.green, self.white, self.yellow)
            self.dt = (pygame.time.get_ticks()-self.last_time)/1000
            self.last_time = pygame.time.get_ticks()
            pygame.display.update()



if __name__ == '__main__':
    start_pose = (60,60,0)
    goal_pose = (360,540)
    screen_dims = (800, 600)

    sim = Simulation(screen_dims)
    sim.main(start_pose, goal_pose)
