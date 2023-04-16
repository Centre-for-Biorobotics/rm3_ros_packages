
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
    info = ""
    if graph is None or start is None or goal is None:
        return [], info
    
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
            # check if line of sight is free between parent and next
            parent = came_from[current]
            if parent is None:
                parent = current
            else:
                #info += "LOS between {}-{}".format(str(parent), str(next)) + "\n"
                los_nodes, nf = graph.line_of_sight_nodes(parent, next)
                info += nf + "\n"
                for pos in los_nodes:
                    # no line of sight, revert to current parent
                    if not graph.node_passable(pos):
                        #info += "REVERTING, NOT PASSABLE: {}".format(str(pos)) + "\n"
                        parent = current
                        break
                    #else:
                    #    info += "PASSABLE: {}".format(str(pos)) + "\n"
                
            new_cost = cost_so_far[parent] + heuristic(parent, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                came_from[next] = parent
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(PrioritizedItem(priority, next))

    if goal not in came_from:
        return [], info

    path = [goal]
    while path[-1] != start and path[-1] != None:
        path.append(came_from[path[-1]])

        if len(came_from) >= 2:
            info += "LOS between {}-{}".format(str(path[-1]), str(path[-2])) + "\n"
            los_nodes, nf = graph.line_of_sight_nodes(path[-1], path[-2])
            info += nf + "\n"
            for pos in los_nodes:
                # no line of sight, revert to current parent
                if not graph.node_passable(pos):
                    info += "REVERTING, NOT PASSABLE: {}".format(str(pos)) + "\n"
                    parent = current
                    break
                else:
                    info += "PASSABLE: {}".format(str(pos)) + "\n"

    return list(reversed(path)), info  # Exclude start from path

def heuristic(goal: NodePosition, next: NodePosition):
    return ((goal.x - next.x)**2 + (goal.y - next.y)**2)**0.5
