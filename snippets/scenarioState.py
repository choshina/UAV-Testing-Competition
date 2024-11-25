import os
import random
import logging
import math
import numpy as np
import pandas
import subprocess
from copy import deepcopy
from typing import List
from aerialist.px4.drone_test import DroneTest
from aerialist.px4.obstacle import Obstacle
from testcase import TestCase
from utils import random_nonintersecting_rectangle, get_boundary_distance, random_rectangle, plot_rectangle, circle_coverage, random_nonintersecting_circle


class ScenarioState:
    min_size = Obstacle.Size(2, 2, 10)
    max_size = Obstacle.Size(20, 20, 25)

    # fixed area: -40 < x < 30, 10 < y < 40;
    # for the rotation, if length is larger than width, the rotation is based on x-axis; otherwise y-axis
    min_position = Obstacle.Position(-40, 10, 0, 0)
    max_position = Obstacle.Position(30, 40, 0, 90)

    def __init__(self, mission_yaml=None, scenario: List[Obstacle] = []):
        self.scenario = scenario
        self.mission_yaml = os.path.join(os.path.dirname(os.path.abspath(__file__)), mission_yaml)

        # drone's trajectory: [(x0,y0), (x1,y1), ...]
        self.trajectory_2d = []
        self.trajectory = None

        self.min_reward = 0.0
        self.max_distance = 5.0
        self.max_obstacles = 3.0

    def next_state(self):
        """Generate a new obstacle on the path of the drone"""
        new_obstacle = self.generate()
        new_state = deepcopy(self)
        if new_obstacle is not None:
            new_state.scenario.append(new_obstacle)
        return new_state

    def random_rotation_modification(self, modified_state):
        new_r = random.uniform(0, 90)
        modified_position = Obstacle.Position(modified_state.scenario[-1].position.x,
                                              modified_state.scenario[-1].position.y, 0, new_r)
        size = Obstacle.Size(modified_state.scenario[-1].size.l, modified_state.scenario[-1].size.w, self.max_size.h)
        modified_obstacle = Obstacle(size, modified_position)
        modified_state.scenario.pop()
        modified_state.scenario.append(modified_obstacle)
        return modified_state

    def projection_modification(self, modified_state):
        original_center_x = modified_state.scenario[-1].position.x
        original_center_y = modified_state.scenario[-1].position.y
        closest_point, rotation, min_distance = self.find_closest_point_with_rotation(modified_state.trajectory_2d,
                                                                                      [original_center_x,
                                                                                       original_center_y])
        new_center_x = (closest_point[0] + original_center_x) / 2
        new_center_y = (closest_point[1] + original_center_y) / 2
        last_obstacle = modified_state.scenario.pop()
        all_other_circles = []
        other_rectangles = [(ob.position.x, ob.position.y, ob.size.l, ob.size.w, ob.position.r) for ob in
                            modified_state.scenario]
        for other_rectangle in other_rectangles:
            all_other_circles += circle_coverage(*other_rectangle, 4)
        circle = random_nonintersecting_circle(new_center_x,
                                               new_center_y,
                                               self.max_position.y,
                                               self.min_position.y,
                                               self.min_position.x,
                                               self.max_position.x,
                                               all_other_circles
                                               )
        if circle is not None:
            x, y, l, w, r = random_rectangle(circle[0], circle[1], min(circle[2], min_distance / 2))
            new_l, new_w, new_r = 0, 0, 0
            if rotation > 90.0:
                new_r = rotation - 90
                new_l = min(l, w)
                new_w = max(l, w)
            else:
                new_r = rotation
                new_l = max(l, w)
                new_w = min(l, w)

            size = Obstacle.Size(l=new_l, w=new_w, h=25)
            position = Obstacle.Position(x=x, y=y, z=0, r=new_r)
            new_obstacle = Obstacle(size, position)
            modified_state.scenario.append(new_obstacle)
            return modified_state
        else:
            modified_state.scenario.append(last_obstacle)
            return self.random_rotation_modification(modified_state)

    def random_generate_modification(self, modified_state):
        candidate_positions = []
        for position in modified_state.trajectory_2d:
            if self.min_position.x < position[0] < self.max_position.x and \
                    self.min_position.y < position[1] < self.max_position.y:
                candidate_positions.append(position)

        candidate_position = random.choice(candidate_positions)

        x, y, l, w, r = random_nonintersecting_rectangle(
            candidate_position[0],
            candidate_position[1],
            self.max_position.y,
            self.min_position.y,
            self.min_position.x,
            self.max_position.x,
            [(ob.position.x, ob.position.y, ob.size.l, ob.size.w, ob.position.r) for ob in self.scenario]
        )
        position = Obstacle.Position(x, y, 0, r)
        size = Obstacle.Size(l, w, self.max_size.h)
        new_obstacle = Obstacle(size, position)
        modified_state.scenario.pop()
        modified_state.scenario.append(new_obstacle)

        return modified_state

    def modify_state(self):
        modified_state = deepcopy(self)
        return self.projection_modification(modified_state)

    def get_reward(self):
        """Simulate the scenario and calculate the reward"""
        test = TestCase(DroneTest.from_yaml(self.mission_yaml), self.scenario)
        try:
            self.trajectory = test.execute()
            self.trajectory_2d = [(position.x, position.y) for position in self.trajectory.positions]
        except Exception as e:
            return self.min_reward, self.max_distance, test

        if len(self.scenario) == 0:
            return self.min_reward, self.max_distance, test

        min_distance = min(test.get_distances())
        reward = -1.0 * min_distance
        test.plot()
        return reward, min_distance, test

    def is_terminal(self):
        if len(self.scenario) == 3:
            return True
        return False

    def check_min_distance_to_last_obstacle(self) -> bool:
        min_distance = min([
            self.trajectory.min_distance_to_obstacles([obst])
            for obst in self.scenario
        ])
        if min_distance == self.trajectory.min_distance_to_obstacles([self.scenario[-1]]):
            return True
        return False

    def generate(self):
        """Randomly choose a point on the drone's trajectory, as the center point of the new rectangle"""
        candidate_positions = []
        for position in self.trajectory_2d:
            if len(self.scenario) == 0 and \
                    position[1] > self.min_position.y + 1/6 * (self.max_position.y - self.min_position.y):
                break
            if self.min_position.x < position[0] < self.max_position.x and \
                    self.min_position.y < position[1] < self.max_position.y:
                candidate_positions.append(position)

        candidate_position = random.choice(candidate_positions)
        if len(self.scenario) == 0:
            radius = get_boundary_distance(
                            candidate_position[0],
                            candidate_position[1],
                            self.max_position.y,
                            self.min_position.y,
                            self.min_position.x,
                            self.max_position.x
                        )
            x, y, l, w, r = random_rectangle(candidate_position[0], candidate_position[1], radius)
        else:
            x, y, l, w, r = random_nonintersecting_rectangle(
                                candidate_position[0],
                                candidate_position[1],
                                self.max_position.y,
                                self.min_position.y,
                                self.min_position.x,
                                self.max_position.x,
                                [(ob.position.x, ob.position.y, ob.size.l, ob.size.w, ob.position.r) for ob in self.scenario]
                            )
        position = Obstacle.Position(x, y, 0, r)
        size = Obstacle.Size(l, w, self.max_size.h)
        return Obstacle(size, position)

    @staticmethod
    def find_closest_point_with_rotation(trajectory_2d, original_center_point):
        trajectory_2d = np.array(trajectory_2d)
        original_center_point = np.array(original_center_point)
        distances = np.linalg.norm(trajectory_2d[:, :2] - original_center_point[:2], axis=1)
        min_index = np.argmin(distances)
        min_distance = distances[min_index]
        closest_point = trajectory_2d[min_index]
        vector_to_closest_point = closest_point - original_center_point
        x_axis = np.array([1, 0])  # x-axis unit vector in 2D
        dot_product = np.dot(vector_to_closest_point, x_axis)

        # Get the magnitudes of the vectors
        magnitude_projected_vector = np.linalg.norm(vector_to_closest_point)
        magnitude_x_axis = np.linalg.norm(x_axis)  # This is 1, but we'll include it for completeness
        angle_radians = np.arccos(dot_product / (magnitude_projected_vector * magnitude_x_axis))
        angle_degrees = math.degrees(angle_radians)

        return closest_point, angle_degrees, min_distance

    def __eq__(self, other):
        list1 = [str(obstacle.to_dict()) for obstacle in self.scenario]
        list2 = [str(obstacle.to_dict()) for obstacle in other.scenario]
        return set(list1) == set(list2)

    def __str__(self):
        s = ""
        for ob in self.scenario:
            s += str((ob.position.x, ob.position.y, ob.size.l, ob.size.w, ob.position.r)) + '\n'
        return s


def replay(obstacles):
    state = ScenarioState()
    for obst in obstacles:
        position = Obstacle.Position(obst[0], obst[1], 0, obst[4])
        size = Obstacle.Size(obst[2], obst[3], 25)
        state.scenario.append(Obstacle(size, position))
    state.get_reward()


if __name__ == '__main__':
    pass


