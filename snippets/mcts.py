import math
import random
from datetime import datetime
from typing import List
from scenarioState import ScenarioState
from testcase import TestCase
import sys
import os


class Node:
    def __init__(self, state: ScenarioState, parent):
        self.state = state
        self.parent = parent
        self.visits = 0
        self.reward = 0.0
        self.children = []
        self.score = 0
        self.id = 0

    def __str__(self):
        return f"state: \n {str(self.state)}, visits: {self.visits}, reward: {self.reward}"


class MCTS:
    def __init__(self, case_study_file: str)-> None:
        self.initial_state = ScenarioState(case_study_file)
        self.root = Node(self.initial_state, None)
        self.count = 0

        # hyperparameters for UCB1 and progressive widening
        self.exploration_rate = 1 / math.sqrt(2)
        self.C = 0.5
        self.alpha = 0.5
        self.C_list = [0.4, 0.5, 0.6, 0.7]

        # results
        self.test_cases = []

    def select(self, node: Node):
        while not node.state.is_terminal():
            layer = len(node.state.scenario)
            # progressive widening
            if len(node.children) <= self.C_list[layer] * (node.visits ** self.alpha):
                return self.expand(node)
            else:
                node = self.best_child(node)

        return node

    def expand(self, node: Node):
        tried_children_state = []
        candidate_siblings = []
        for child in node.children:
            tried_children_state.append(child.state)
            if (child.score == 1 or child.score == 2) and child.state.check_min_distance_to_last_obstacle() is True:
                candidate_siblings.append(child)

        if len(candidate_siblings) != 0:  # modify an existing sibling
            sibling = random.choice(candidate_siblings)
            new_state = sibling.state.modify_state()
        else:  # add a new obstacle to this node
            new_state = node.state.next_state()
            while new_state in tried_children_state and not new_state.is_terminal():
                new_state = node.state.next_state()

        if new_state is None or len(node.state.scenario) == len(new_state.scenario):
            return None
        else:
            new_node = Node(new_state, node)
            self.count += 1
            new_node.id = self.count
            node.children.append(new_node)
            return new_node

    def simulate(self, state):
        return state.get_reward()

    @staticmethod
    def back_propogate(node, reward):
        while node is not None:
            node.visits += 1
            node.reward += reward
            node = node.parent

    def search(self):
        node = self.select(self.root)
        if node is not None:
            reward, min_distance, test_case = self.simulate(node.state)
            self.count += 1
            if abs(min_distance) < 0.25:
                node.score = 5
            elif 0.25 <= abs(min_distance) <= 1:
                node.score = 2
            elif 1 <= abs(min_distance) <= 1.5:
                node.score = 1

            # delete the node if it is invalid, or it is a hard failure (min_dis < 0.25m)
            if (reward == 0.0 and len(node.state.scenario) != 0) or abs(min_distance) < 0.25:
                node.parent.children.remove(node)
                node.parent = None

            if 0 <= abs(min_distance) <= 1.5:
                self.test_cases.append(test_case)

            self.back_propogate(node, reward)

    def generate(self, budget: int) -> List[TestCase]:
        reward, distance, test_case = self.simulate(self.root.state)
        self.back_propogate(self.root, reward)
        for i in range(budget):
            self.search()
        return self.test_cases

    def best_child(self, node):
        # UCB1
        return max(node.children, key=lambda child: child.reward / child.visits +
                                             self.exploration_rate * math.sqrt(2 * math.log(node.visits) / child.visits))


if __name__ == '__main__':
    generator = MCTS("case_studies/mission2.yaml")
    generator.generate(200)
