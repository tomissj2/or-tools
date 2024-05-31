#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "absl/flags/parse.h"
#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "util/json.hpp"

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
  int calculation_id;
  RoutingIndexManager::NodeIndex depot{};
  int64_t vehicle_distance_limit;
};

DataModel loadDataFromJson() {
  std::string filePath = absl::GetFlag(FLAGS_input_filepath);
  std::cout << "Data loaded from: " << filePath << std::endl;

  std::string fileContent;
  std::ifstream inFile(filePath);

  DataModel data;

  if (inFile.is_open()) {
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    fileContent = buffer.str();
    inFile.close();

    json j = json::parse(fileContent);
    auto indata = j["datas"];
    data.distance_matrix =
        indata["distance_matrix"].get<std::vector<std::vector<int64_t>>>();
    data.demands = indata["demands"].get<std::vector<int64_t>>();
    data.vehicle_capacities =
        indata["vehicle_capacities"].get<std::vector<int64_t>>();
    data.num_vehicles = indata["num_vehicles"].get<int>();
    data.calculation_id = indata["calculation_id"].get<int>();
    data.depot = indata["depot"].get<int>();
    data.vehicle_distance_limit = indata["vehicle_distances"].get<int64_t>();

    // std::cout << fileContent;

  } else {
    std::cerr << "Error opening file for readin: " + filePath + "\n";
  }

  return data;
}

void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  json results = {{"max_route_distance", 0}, {"routes", json::array()}};

  int64_t max_route_distance = 0;
  int64_t total_load = 0;
  std::string filePath = absl::GetFlag(FLAGS_input_filepath);
  size_t start_pos = filePath.find(".json");
  filePath = filePath.replace(start_pos, 5, "_out.json");

  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int index = routing.Start(vehicle_id);
    std::vector<int64_t> plan_output;
    int64_t route_distance = 0;
    int route_load = 0;
    int current_index = manager.IndexToNode(index).value();

    while (!routing.IsEnd(current_index)) {
      plan_output.push_back(current_index);
      int next_index = solution.Value(routing.NextVar(current_index));
      route_distance +=
          routing.GetArcCostForVehicle(current_index, next_index, vehicle_id);
      route_load += data.demands[current_index];
      current_index = next_index;
    }

    plan_output.push_back(manager.IndexToNode(index).value());

    json route_info = {{"routes", plan_output},
                       {"vehicle_id", vehicle_id},
                       {"distance", route_distance},
                       {"load", route_load}};

    results["routes"].insert(results["routes"].begin() + vehicle_id,
                             route_info);

    max_route_distance = std::max(route_distance, max_route_distance);
  }

  results["max_route_distance"] = max_route_distance;
  results["calculation_time"] = routing.solver()->wall_time();

  json return_json = {{"result", results},
                      {"calculation_id", data.calculation_id}};

  std::ofstream file(filePath);
  if (file.is_open()) {
    file << return_json.dump(4);  // 4-space indentation for pretty printing
    file.close();

    std::cout << "Data saved to:    " << filePath << std::endl;
  } else {
    std::cerr << "Error opening file " << filePath << " for writing."
              << std::endl;
  }
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
  // search_parameters.set_log_search(true);

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
