from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import json
import easygui
import networkx as nx
import matplotlib.pyplot as plt

from utils import *



def main():
    # Instantiate the data problem.
    
    path = easygui.fileopenbox()

    if path == None:
        data = create_example()
    else:
        with open(path) as f:
            data = json.load(f)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data["time_matrix"]), data["num_vehicles"], data["depot"]
    )

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["time_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = "Time"
    routing.AddDimension(
        transit_callback_index,
        60,  # allow waiting time
        60,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time,
    )
    time_dimension = routing.GetDimensionOrDie(time)

    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data["time_windows"]):
        if location_idx == 0:
            continue
        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node.
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        time_dimension.CumulVar(index).SetRange(
            data["time_windows"][0][0], data["time_windows"][0][1]
        )

    # Add resource constraints at the depot.
    solver = routing.solver()
    intervals = []
    for i in range(data["num_vehicles"]):
        # Add time windows at start of routes
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.Start(i)),
                data["vehicle_load_time"],
                "depot_interval",
            )
        )
        # Add time windows at end of routes.
        intervals.append(
            solver.FixedDurationIntervalVar(
                time_dimension.CumulVar(routing.End(i)),
                data["vehicle_unload_time"],
                "depot_interval",
            )
        )

    depot_usage = [1 for _ in range(len(intervals))]
    solver.Add(
        solver.Cumulative(intervals, depot_usage, data["depot_capacity"], "depot")
    )

    # Instantiate route start and end times to produce feasible times.
    for i in range(data["num_vehicles"]):
        routing.AddVariableMinimizedByFinalizer(
            time_dimension.CumulVar(routing.Start(i))
        )
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        print_solution(data, manager, routing, solution)
    else:
        print("No solution found !")
    # Graph the solution
    graph_solution(data, manager, routing, solution)


# Graph the solution of this problem
def graph_solution(data, manager, routing, solution):
    G = nx.DiGraph()
    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        while not routing.IsEnd(index):
            from_node = manager.IndexToNode(index)
            index = solution.Value(routing.NextVar(index))
            to_node = manager.IndexToNode(index)
            G.add_edge(from_node, to_node)
    nx.draw(G, with_labels=True, node_size=1000, node_color="skyblue")
    plt.show()


if __name__ == "__main__":
    main()