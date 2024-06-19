#include <cstdint>
#include <fstream>
#include <vector>

#include "google/protobuf/text_format.h"
#include "ortools/base/commandlineflags.h"
#include "ortools/base/init_google.h"
#include "ortools/base/logging.h"
#include "ortools/base/types.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
#include "ortools/constraint_solver/routing_parameters.pb.h"
#include "util/json.hpp"

using json = nlohmann::json;

ABSL_FLAG(std::string, input_filepath, "example.json",
          "Input file that contains the data to process");

using operations_research::Assignment;
using operations_research::DefaultRoutingSearchParameters;
using operations_research::RoutingDimension;
using operations_research::RoutingIndexManager;
using operations_research::RoutingModel;
using operations_research::RoutingNodeIndex;
using operations_research::RoutingSearchParameters;

// ABSL_FLAG(std::string, input_filepath, "example.json",
//           "Input file that contains the data to process");
const char* kTime = "Time";
const char* kCapacity = "Capacity";
const auto* kDistance = "Distance";

const int64_t kMaxNodesPerGroup = 10;
const int64_t kSameVehicleCost = 1000;

struct DataModel {
  std::vector<std::vector<int64_t>> distance_matrix{};
  std::vector<std::vector<int64_t>> time_matrix;
  std::vector<int64_t> demands{};
  std::vector<int64_t> vehicle_capacities{};
  std::vector<std::vector<int64_t>> time_windows;
  int num_vehicles;
  int calculation_id;
  RoutingIndexManager::NodeIndex depot{};
  int64_t vehicle_distance_limit;
  int64_t vehicleWaitTime;
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
    std::cout << "Data loaded from: " << filePath << std::endl;
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

    data.time_matrix =
        indata["time_matrix"].get<std::vector<std::vector<int64_t>>>();
    data.time_windows =
        indata["time_windows"].get<std::vector<std::vector<int64_t>>>();
    // std::cout << fileContent;

  } else {
    std::cerr << "Error opening file for readin: " + filePath + "\n";
  }

  return data;
}

void SaveSolution(const DataModel& data, const RoutingIndexManager& manager,
                  const RoutingModel& routing, const Assignment& solution) {
  json results = {{"max_route_distance", 0}, {"routes", json::array()}};

  int64_t max_route_distance = 0;
  int64_t total_load = 0;
  std::string filePath = absl::GetFlag(FLAGS_input_filepath);
  size_t start_pos = filePath.find(".json");
  filePath = filePath.replace(start_pos, 5, "_out.json");

  const RoutingDimension& time_dimension = routing.GetDimensionOrDie(kTime);
  int64_t total_time{0};

  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int index = routing.Start(vehicle_id);

    std::vector<std::vector<int64_t>> time_planOutput;
    std::vector<int64_t> plan_output;
    int64_t route_distance = 0;
    int route_load = 0;

    while (!routing.IsEnd(index)) {
      auto time_var = time_dimension.CumulVar(index);
      std::vector<int64_t> time_data;
      time_data.push_back(manager.IndexToNode(index).value());
      time_data.push_back(solution.Min(time_var));
      time_data.push_back(solution.Max(time_var));
      time_planOutput.push_back(time_data);

      plan_output.push_back(manager.IndexToNode(index).value());
      int next_index = solution.Value(routing.NextVar(index));
      route_distance +=
          routing.GetArcCostForVehicle(index, next_index, vehicle_id);
      route_load += data.demands[manager.IndexToNode(index).value()];

      index = solution.Value(routing.NextVar(index));
    }
    plan_output.push_back(manager.IndexToNode(index).value());

    auto time_var = time_dimension.CumulVar(index);
    std::vector<int64_t> time_data;
    time_data.push_back(manager.IndexToNode(index).value());
    time_data.push_back(solution.Min(time_var));
    time_data.push_back(solution.Max(time_var));
    time_planOutput.push_back(time_data);

    auto route_time = solution.Min(time_var);

    json route_info = {{"vehicle_id", vehicle_id},   {"routes", plan_output},
                       {"times", time_planOutput},   {"route_time", route_time},
                       {"distance", route_distance}, {"load", route_load}};

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

void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) {
  const RoutingDimension& time_dimension = routing.GetDimensionOrDie("Time");
  int64_t total_time{0};
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
    int64_t index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for vehicle " << vehicle_id << ":";
    std::ostringstream route;
    while (!routing.IsEnd(index)) {
      auto time_var = time_dimension.CumulVar(index);
      route << manager.IndexToNode(index).value() << " Time("
            << solution.Min(time_var) << ", " << solution.Max(time_var)
            << ") -> ";
      index = solution.Value(routing.NextVar(index));
    }
    auto time_var = time_dimension.CumulVar(index);
    LOG(INFO) << route.str() << manager.IndexToNode(index).value() << " Time("
              << solution.Min(time_var) << ", " << solution.Max(time_var)
              << ")";
    LOG(INFO) << "Time of the route: " << solution.Min(time_var) << "min";
    total_time += solution.Min(time_var);
  }
  LOG(INFO) << "Total time of all routes: " << total_time << "min";
  LOG(INFO) << "";
  LOG(INFO) << "Advanced usage:";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  DataModel data = loadDataFromJson();

  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);

  RoutingModel routing(manager);

  std::cout << "1 ";
  // Adding distance matrix
  const int vehicle_cost = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });

  routing.SetArcCostEvaluatorOfAllVehicles(vehicle_cost);

  std::cout << "2 ";
  // adding distance limit
  routing.AddDimension(vehicle_cost, 0, data.vehicle_distance_limit, true,
                       kDistance);
  auto routing_dimension = routing.GetMutableDimension(kDistance);
  routing_dimension->SetGlobalSpanCostCoefficient(100);

  // adding vehicle capaties
  const int demand_callback_index = routing.RegisterUnaryTransitCallback(
      [&data, &manager](const int64_t from_index) -> int64_t {
        const int from_node = manager.IndexToNode(from_index).value();
        return data.demands[from_node];
      });

  std::cout << "3 ";
  routing.AddDimensionWithVehicleCapacity(
      demand_callback_index,    // transit callback index
      int64_t{0},               // null capacity slack
      data.vehicle_capacities,  // vehicle maximum capacities
      true,                     // start cumul to zero
      kCapacity);

  // Adding time dimension constraints.
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        // Convert from routing variable Index to time matrix NodeIndex.
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.time_matrix[from_node][to_node];
      });

  std::cout << "4 ";
  // Define cost of each arc.
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

  routing.AddDimension(transit_callback_index,  // transit callback index
                       int64_t{30},             // allow waiting time
                       int64_t{30},             // maximum time per vehicle
                       false,  // Don't force start cumul to zero
                       kTime);
  const RoutingDimension& time_dimension = routing.GetDimensionOrDie(kTime);

  std::cout << "5 ";
  // Add time window constraints for each location except depot.
  for (int i = 1; i < data.time_windows.size(); ++i) {
    const int64_t index =
        manager.NodeToIndex(RoutingIndexManager::NodeIndex(i));
    time_dimension.CumulVar(index)->SetRange(data.time_windows[i][0],
                                             data.time_windows[i][1]);
  }
  // Add time window constraints for each vehicle start node.
  for (int i = 0; i < data.num_vehicles; ++i) {
    const int64_t index = routing.Start(i);
    time_dimension.CumulVar(index)->SetRange(data.time_windows[0][0],
                                             data.time_windows[0][1]);
  }

  std::cout << "6 ";
  // Instantiate route start and end times to produce feasible times.
  for (int i = 0; i < data.num_vehicles; ++i) {
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.Start(i)));
    routing.AddVariableMinimizedByFinalizer(
        time_dimension.CumulVar(routing.End(i)));
  }

  std::cout << "7 ";
  for (int i = 1; i < data.time_windows.size(); ++i) {
    const int64_t index =
        manager.NodeToIndex(RoutingIndexManager::NodeIndex(i));
    time_dimension.CumulVar(index)->SetRange(data.time_windows[i][0],
                                             data.time_windows[i][1]);
  }

  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  // searchParameters.set_first_solution_strategy(
  // FirstSolutionStrategy::PATH_CHEAPEST_ARC);

  const Assignment* solution = routing.SolveWithParameters(searchParameters);

  std::cout << "8 " << std::endl;
  ;
  if (solution != nullptr) {
    // PrintSolution(data, manager, routing, *solution);
    SaveSolution(data, manager, routing, *solution);
  } else {
    LOG(INFO) << "There was an error!";
  }
  return EXIT_SUCCESS;
}