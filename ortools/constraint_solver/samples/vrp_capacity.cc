#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "util/json.hpp"
#include "absl/flags/parse.h"

using json = nlohmann::json;

ABSL_FLAG(std::string, input_filepath, "example.json",
          "Input file that contains the data to process");

namespace operations_research {

const auto* distance_dimension_name = "Distance";
const auto* capacity_dimension_name = "Capacity";

struct DataModel {
  std::vector<std::vector<int64_t>> distance_matrix{};
  std::vector<int64_t> demands{};
  std::vector<int64_t> vehicle_capacities{};
  int num_vehicles;
  RoutingIndexManager::NodeIndex depot{};
  int64_t vehicle_distance_limit;
};

DataModel loadDataFromJson() {
  std::string filePath = absl::GetFlag(FLAGS_input_filepath);
  std::string fileContent;
  std::ifstream inFile(filePath);

  DataModel data;

  if (inFile.is_open()) {
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    fileContent = buffer.str();
    inFile.close();

    json j = json::parse(fileContent);

    data.distance_matrix =
        j["distance_matrix"].get<std::vector<std::vector<int64_t>>>();
    data.demands = j["demands"].get<std::vector<int64_t>>();
    data.vehicle_capacities =
        j["vehicle_capacities"].get<std::vector<int64_t>>();
    data.num_vehicles = j["num_vehicles"].get<int>();
    data.depot = j["depot"].get<int>();
    data.vehicle_distance_limit = j["vehicle_distances"].get<int64_t>();
    
    //std::cout << fileContent;

  } else {
    std::cerr << "Error opening file for reading.\n";
  }

  return data;
}

void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  int64_t total_distance = 0;
  int64_t total_load = 0;
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64_t index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64_t route_distance = 0;
    int64_t route_load = 0;
    std::stringstream route;
    while (!routing.IsEnd(index)) {
      const int node_index = manager.IndexToNode(index).value();
      route_load += data.demands[node_index];
      route << node_index << " Load(" << route_load << ") -> ";
      const int64_t previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64_t{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Distance of the route: " << route_distance << "m";
    LOG(INFO) << "Load of the route: " << route_load;
    total_distance += route_distance;
    total_load += route_load;
  }
  LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  LOG(INFO) << "Total load of all routes: " << total_load;
  LOG(INFO) << "";
  LOG(INFO) << "Advanced usage:";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void VrpCapacity() {
  DataModel data = loadDataFromJson();

  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  RoutingModel routing(manager);

  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  routing.AddDimension(transit_callback_index, 0, data.vehicle_distance_limit,
                       true, distance_dimension_name);
  auto routing_dimension = routing.GetMutableDimension(distance_dimension_name);
  routing_dimension->SetGlobalSpanCostCoefficient(100);

  const int demand_callback_index = routing.RegisterUnaryTransitCallback(
      [&data, &manager](const int64_t from_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        return data.demands[from_node];
      });

  routing.AddDimensionWithVehicleCapacity(
      demand_callback_index,    // transit callback index
      int64_t{0},               // null capacity slack
      data.vehicle_capacities,  // vehicle maximum capacities
      true,                     // start cumul to zero
      capacity_dimension_name);

  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  search_parameters.set_local_search_metaheuristic(
      LocalSearchMetaheuristic::AUTOMATIC);
  search_parameters.mutable_time_limit()->set_seconds(1);
  search_parameters.set_log_search(true);

  const Assignment* solution = routing.SolveWithParameters(search_parameters);

  if (solution != nullptr) {
    PrintSolution(data, manager, routing, *solution);
  } else {
    LOG(INFO) << "There was an error!";
  }
}

}  // namespace operations_research

int main(int argc, char* argv[]) {
  absl::ParseCommandLine(argc, argv);
  operations_research::VrpCapacity();
  return EXIT_SUCCESS;
}
// [END program]
