
from dataclasses import dataclass, field
from queue import PriorityQueue
from typing import List, Any

from robominer_locomotion_control.pathfinder.graph.node_position import NodePosition
from robominer_locomotion_control.pathfinder.graph.graph import Graph


@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)


def a_star(graph: Graph, start: NodePosition, goal: NodePosition) -> List[NodePosition]:
    frontier: PriorityQueue[PrioritizedItem] = PriorityQueue()
    frontier.put(PrioritizedItem(0, start))
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get().item

        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(PrioritizedItem(priority, next))
                came_from[next] = current

    if goal not in came_from:
        return []

    path = [goal]
    while path[-1] != start and path[-1] != None:
        path.append(came_from[path[-1]])

    return list(reversed(path))  # Exclude start from path

def heuristic(goal: NodePosition, next: NodePosition):
    return ((goal.x - next.x)**2 + (goal.y - next.y)**2)**0.5
