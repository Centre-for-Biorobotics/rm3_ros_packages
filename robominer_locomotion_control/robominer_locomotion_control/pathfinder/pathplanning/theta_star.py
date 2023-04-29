
from dataclasses import dataclass, field
from queue import PriorityQueue
from typing import List, Any

from robominer_locomotion_control.pathfinder.graph.node_position import NodePosition
from robominer_locomotion_control.pathfinder.graph.graph import Graph


@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)

def theta_star(graph: Graph, start: NodePosition, goal: NodePosition) -> List[NodePosition]:
    if graph is None or start is None or goal is None or not graph.node_passable(goal):
        return []
    
    frontier: PriorityQueue[PrioritizedItem] = PriorityQueue()
    frontier.put(PrioritizedItem(0, start))
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current : NodePosition = frontier.get().item

        if current == goal:
            break
        
        for next in graph.neighbors(current):
            if not graph.node_passable(next):
                continue
            # check if line of sight is free between parent and next
            parent = came_from[current]
            if parent is None:
                parent = current
            else:
                los_nodes = graph.line_of_sight_nodes(parent, next)
                for pos in los_nodes:
                    # no line of sight, revert to current parent
                    if not graph.node_passable(pos):
                        parent = current
                        break
                
            new_cost = cost_so_far[parent] + heuristic(parent, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                came_from[next] = parent
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(PrioritizedItem(priority, next))

    if goal not in came_from:
        return []

    path = [goal]
    while path[-1] != start and path[-1] != None:
        path.append(came_from[path[-1]])

        if len(came_from) >= 2:
            los_nodes = graph.line_of_sight_nodes(path[-1], path[-2])
            for pos in los_nodes:
                # no line of sight, revert to current parent
                if not graph.node_passable(pos):
                    parent = current
                    break

    return list(reversed(path))[1:]  # Exclude start from path

def heuristic(goal: NodePosition, next: NodePosition):
    return ((goal.x - next.x)**2 + (goal.y - next.y)**2)**0.5
