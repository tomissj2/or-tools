// Copyright 2010-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// [START program]
// [START import]
#include <cstdint>
#include <sstream>
#include <vector>

#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
// [END import]

ABSL_FLAG(int, vrp_vehicles, 4, "Number of vehicles in the problem");

namespace operations_research {
// [START data_model]
struct DataModel {
  const std::vector<std::vector<int64_t>> distance_matrix{
      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
      {0,     0,    1111, 2521, 3229, 10006, 5223, 4930, 2317, 3635, 4412,
       2774,  5591, 3263, 5542, 2557, 3091,  1281, 4532, 3005, 3206, 4261,
       4954,  1628, 5511, 3126, 4612, 3191,  3157, 3182, 3871, 4389, 5253,
       11813, 3161, 6097, 2155, 2753, 15399, 2242, 1746, 2445},
      {0,     1111, 0,    1757, 3473, 9190,  5467, 4166, 2398, 2820, 4493,
       3107,  4776, 2499, 4778, 1742, 3171,  1525, 3717, 2189, 3315, 3497,
       4190,  2080, 4747, 2311, 3849, 3435,  3274, 3262, 3108, 3573, 5334,
       10997, 2345, 5334, 2236, 2834, 14584, 2323, 982,  1682},
      {0,     2521, 1757, 0,    4162, 8196,  5185, 2409, 949,  1937, 3395,
       2199,  4087, 742,  3021, 1087, 1716,  2214, 3015, 1488, 3116, 1740,
       2433,  2769, 2990, 826,  2091, 3541,  5063, 2163, 1350, 2636, 3488,
       10003, 1643, 3577, 2037, 1714, 13646, 2124, 1753, 953},
      {0,     3229, 3473, 4162, 0,    12138, 2194, 5782, 3297, 5879, 5392,
       3678,  7877, 4097, 6393, 4911, 4071,  2046, 6886, 5359, 918,  5113,
       5806,  2963, 6450, 4767, 5464, 897,   4449, 4162, 5292, 6578, 6233,
       13945, 5514, 6949, 3135, 3733, 17588, 3222, 2848, 3866},
      {0,     10006, 9190,  8196, 12138, 0,     13492, 9392, 9255,
       6417,  10378, 9625,  7906, 8626,  10004, 7479,  8941, 10521,
       5626,  8237,  11423, 8724, 9417,  11076, 9973,  7788, 9075,
       11848, 11394, 9664,  7834, 7916,  10471, 1835,  7562, 10560,
       10343, 8939,  15955, 9851, 10060, 9259},
      {0,     5223, 5467, 5185, 2194, 13492, 0,    6728, 4541, 7123, 5905,
       4351,  9122, 5225, 7340, 6155, 5017,  4054, 8130, 6603, 2330, 6059,
       6752,  4314, 7360, 6012, 6410, 2392,  5799, 5072, 6536, 7822, 7143,
       15189, 6759, 7895, 4379, 4679, 18832, 4467, 4855, 5110},
      {0,     4930, 4166, 2409, 5782, 9392,  6728, 0,    2652, 3529, 3057,
       2732,  6414, 1733, 1357, 3136, 2048,  4151, 4681, 3907, 4293, 1402,
       769,   4706, 1326, 2875, 857,  4719,  7482, 2771, 1324, 2383, 1824,
       11122, 4062, 1689, 3602, 2046, 12628, 3130, 3690, 3372},
      {0,     2317, 2398, 949,  3297, 9255,  4541, 2652, 0,    3609, 2793,
       1408,  5608, 1499, 3795, 2641, 1472,  1645, 4616, 3089, 2323, 2515,
       3207,  2200, 3852, 2498, 2866, 2748,  5156, 1563, 3022, 4308, 3635,
       11675, 3245, 4351, 1114, 1135, 15318, 624,  1184, 1596},
      {0,    3635, 2820, 1937, 5879, 6417,  7123, 3529, 3609, 0,    4351,
       3599, 3336, 2296, 3977, 1078, 2914,  4030, 1555, 1878, 5040, 2697,
       3390, 4693, 3947, 1387, 3048, 5465,  6145, 3638, 1807, 1890, 4444,
       8136, 1433, 4533, 3960, 2913, 12900, 3468, 3487, 2876},
      {0,     4412, 4493, 3395, 5392, 10378, 5905, 3057, 2793, 4351, 0,
       2306,  7405, 2723, 2106, 4127, 1826,  4290, 5672, 4897, 3867, 1789,
       2337,  4845, 1860, 3865, 2294, 4292,  7801, 1230, 3436, 4745, 1643,
       12113, 5053, 4161, 3176, 2185, 19270, 2838, 3829, 4362},
      {0,     2774, 3107, 2199, 3678, 9625,  4351, 2732, 1408, 3599, 2306,
       0,     6307, 1621, 3379, 2938, 1056,  2982, 4895, 3708, 1957, 2098,
       2791,  3042, 3400, 2676, 2450, 2382,  6284, 1111, 2660, 3969, 3183,
       11336, 3864, 3935, 1266, 718,  14979, 1118, 2438, 3173},
      {0,    5591, 4776, 4087, 7877, 7906,  9122, 6414, 5608, 3336, 7405,
       6307, 0,    4800, 7079, 3462, 5774,  5970, 2345, 3544, 7001, 5798,
       6491, 6655, 7048, 4031, 6149, 7427,  5518, 6221, 4858, 4941, 7546,
       9626, 3083, 7635, 5922, 5772, 15951, 6009, 5427, 4838},
      {0,     3263, 2499, 742,  4097, 8626,  5225, 1733, 1499, 2296, 2723,
       1621,  4800, 0,    2362, 1467, 973,   2444, 3503, 2238, 2830, 1081,
       1774,  2999, 2331, 1206, 1433, 3255,  5813, 1697, 1643, 2952, 2829,
       10319, 2394, 2918, 1913, 972,  13962, 1423, 1983, 1703},
      {0,     5542, 4778, 3021, 6393, 10004, 7340, 1357, 3795, 3977, 2106,
       3379,  7079, 2362, 0,    3749, 2661,  4764, 5294, 4520, 4906, 2015,
       579,   5319, 331,  3488, 1343, 5332,  8095, 2363, 2171, 3440, 891,
       11735, 4675, 2248, 4215, 2659, 14324, 3743, 4303, 3985},
      {0,    2557, 1742, 1087, 4911, 7479,  6155, 3136, 2641, 1078, 4127,
       2938, 3462, 1467, 3749, 0,    2451,  2951, 2264, 913,  4001, 2476,
       3169, 3506, 3726, 569,  2828, 4426,  5066, 2899, 1720, 1831, 4224,
       9199, 1069, 4313, 2921, 2450, 12842, 2650, 2409, 1837},
      {0,     3091, 3171, 1716, 4071, 8941,  5017, 2048, 1472, 2914, 1826,
       1056,  5774, 973,  2661, 2451, 0,     2464, 4226, 3006, 2606, 1087,
       2122,  3019, 2473, 1974, 1780, 3031,  5974, 1083, 1991, 3299, 2256,
       10667, 3162, 3265, 1915, 359,  14310, 1442, 2002, 2471},
      {0,     1281, 1525, 2214, 2046, 10521, 4054, 4151, 1645, 4030, 4290,
       2982,  5970, 2444, 4764, 2951, 2464,  0,    4809, 3282, 2477, 3375,
       4068,  1419, 4713, 3030, 3727, 2841,  4661, 2424, 3555, 4666, 4496,
       12090, 3437, 5212, 1397, 1996, 15676, 1485, 1095, 2128},
      {0,    4532, 3717, 3015, 6886, 5626,  8130, 4681, 4616, 1555, 5672,
       4895, 2345, 3503, 5294, 2264, 4226,  4809, 0,    2676, 5937, 3966,
       4659, 5443, 5216, 2585, 4318, 6362,  5833, 4907, 3077, 3160, 5714,
       7345, 2002, 5803, 4858, 4182, 15848, 4945, 4346, 3774},
      {0,    3005, 2189, 1488, 5359, 8237,  6603, 3907, 3089, 1878, 4897,
       3708, 3544, 2238, 4520, 913,  3006,  3282, 2676, 0,    4311, 3090,
       3783, 3817, 4340, 1323, 3442, 4736,  5377, 3513, 2700, 2586, 4838,
       9028, 375,  4927, 3231, 3064, 13596, 3319, 2719, 2148},
      {0,     3206, 3315, 3116, 918,  11423, 2330, 4293, 2323, 5040, 3867,
       1957,  7001, 2830, 4906, 4001, 2606,  2477, 5937, 4311, 0,    3990,
       4683,  3183, 5291, 3942, 4341, 585,   5367, 3003, 4467, 5753, 5074,
       13120, 4689, 5826, 2310, 2610, 16763, 2397, 2629, 3041},
      {0,     4261, 3497, 1740, 5113, 8724,  6059, 1402, 2515, 2697, 1789,
       2098,  5798, 1081, 2015, 2476, 1087,  3375, 3966, 3090, 3990, 0,
       1262,  4048, 1762, 2216, 537,  4060,  6823, 1350, 1788, 3097, 2197,
       10464, 3404, 2405, 2944, 1388, 14107, 2471, 3031, 2713},
      {0,     4954, 4190, 2433, 5806, 9417,  6752, 769,  3207, 3390, 2337,
       2791,  6491, 1774, 579,  3169, 2122,  4068, 4659, 3783, 4683, 1262,
       0,     4741, 557,  2909, 764,  4753,  7516, 2806, 1593, 2861, 1055,
       11157, 4097, 1670, 3637, 2081, 13746, 3164, 3724, 3406},
      {0,     1628, 2080, 2769, 2963, 11076, 4314, 4706, 2200, 4693, 4845,
       3042,  6655, 2999, 5319, 3506, 3019,  1419, 5443, 3817, 3183, 4048,
       4741,  0,    5710, 4027, 4724, 3431,  3211, 3421, 4552, 5322, 5493,
       12746, 4094, 6209, 2395, 2993, 16332, 2482, 2092, 3126},
      {0,     5511, 4747, 2990, 6450, 9973,  7360, 1326, 3852, 3947, 1860,
       3400,  7048, 2331, 331,  3726, 2473,  4713, 5216, 4340, 5291, 1762,
       557,   5710, 0,    3466, 1321, 5178,  8073, 2117, 2149, 3418, 702,
       11714, 4654, 2226, 4062, 2637, 14302, 3721, 4281, 3963},
      {0,    3126, 2311, 826,  4767, 7788,  6012, 2875, 2498, 1387, 3865,
       2676, 4031, 1206, 3488, 569,  1974,  3030, 2585, 1323, 3942, 2216,
       2909, 4027, 3466, 0,    2538, 4358,  5633, 2610, 1797, 2170, 3934,
       9537, 1416, 4023, 2853, 2161, 13180, 2360, 2569, 1769},
      {0,     4612, 3849, 2091, 5464, 9075,  6410, 857,  2866, 3048, 2294,
       2450,  6149, 1433, 1343, 2828, 1780,  3727, 4318, 3442, 4341, 537,
       764,   4724, 1321, 2538, 0,    4403,  7166, 2455, 1242, 2511, 1833,
       10806, 3746, 1921, 3286, 1730, 13997, 2814, 3374, 3056},
      {0,     3191, 3435, 3541, 897,  11848, 2392, 4719, 2748, 5465, 4292,
       2382,  7427, 3255, 5332, 4426, 3031,  2841, 6362, 4736, 585,  4060,
       4753,  3431, 5178, 4358, 4403, 0,     5313, 4443, 5908, 7194, 6515,
       14561, 6130, 7267, 3751, 4051, 18204, 3838, 4069, 4482},
      {0,     3157, 3274, 5063, 4449, 11394, 5799, 7482, 5156, 6145, 7801,
       6284,  5518, 5813, 8095, 5066, 5974,  4661, 5833, 5377, 5367, 6823,
       7516,  3211, 8073, 5633, 7166, 5313,  0,    6017, 5862, 6328, 8088,
       12780, 5100, 8088, 4990, 5588, 17338, 5078, 3737, 4436},
      {0,     3182, 3262, 2163, 4162, 9664,  5072, 2771, 1563, 3638, 1230,
       1111,  6221, 1697, 2363, 2899, 1083,  2424, 4907, 3513, 3003, 1350,
       2806,  3421, 2117, 2610, 2455, 4443,  6017, 0,    2739, 4048, 2071,
       11416, 3887, 4014, 1945, 798,  15059, 1608, 2728, 3196},
      {0,    3871, 3108, 1350, 5292, 7834,  6536, 1324, 3022, 1807, 3436,
       2660, 4858, 1643, 2171, 1720, 1991,  3555, 3077, 2700, 4467, 1788,
       1593, 4552, 2149, 1797, 1242, 5908,  5862, 2739, 0,    1354, 2667,
       9554, 2535, 2756, 3382, 1949, 12364, 2890, 3098, 2298},
      {0,     4389, 3573, 2636, 6578, 7916,  7822, 2383, 4308, 1890, 4745,
       3969,  4941, 2952, 3440, 1831, 3299,  4666, 3160, 2586, 5753, 3097,
       2861,  5322, 3418, 2170, 2511, 7194,  6328, 4048, 1354, 0,    3598,
       10691, 3706, 3109, 5721, 4317, 11184, 5228, 5437, 4637},
      {0,     5253, 5334, 3488, 6233, 10471, 7143, 1824, 3635, 4444, 1643,
       3183,  7546, 2829, 891,  4224, 2256,  4496, 5714, 4838, 5074, 2197,
       1055,  5493, 702,  3934, 1833, 6515,  8088, 2071, 2667, 3598, 0,
       12211, 5151, 2724, 3845, 2614, 14800, 3698, 4258, 4461},
      {0,     11813, 10997, 10003, 13945, 1835,  15189, 11122, 11675,
       8136,  12113, 11336, 9626,  10319, 11735, 9199,  10667, 12090,
       7345,  9028,  13120, 10464, 11157, 12746, 11714, 9537,  10806,
       14561, 12780, 11416, 9554,  10691, 12211, 0,     9326,  12324,
       12107, 10703, 17719, 11615, 11824, 11024},
      {0,    3161, 2345, 1643, 5514, 7562,  6759, 4062, 3245, 1433, 5053,
       3864, 3083, 2394, 4675, 1069, 3162,  3437, 2002, 375,  4689, 3404,
       4097, 4094, 4654, 1416, 3746, 6130,  5100, 3887, 2535, 3706, 5151,
       9326, 0,    5672, 3977, 3809, 13743, 4064, 3465, 2893},
      {0,     6097, 5334, 3577, 6949, 10560, 7895, 1689, 4351, 4533, 4161,
       3935,  7635, 2918, 2248, 4313, 3265,  5212, 5803, 4927, 5826, 2405,
       1670,  6209, 2226, 4023, 1921, 7267,  8088, 4014, 2756, 3109, 2724,
       12324, 5672, 0,    4224, 2668, 12729, 3752, 4312, 3994},
      {0,     2155, 2236, 2037, 3135, 10343, 4379, 3602, 1114, 3960, 3176,
       1266,  5922, 1913, 4215, 2921, 1915,  1397, 4858, 3231, 2310, 2944,
       3637,  2395, 4062, 2853, 3286, 3751,  4990, 1945, 3382, 5721, 3845,
       12107, 3977, 4224, 0,    1151, 15306, 640,  1172, 1584},
      {0,     2753, 2834, 1714, 3733, 8939,  4679, 2046, 1135, 2913, 2185,
       718,   5772, 972,  2659, 2450, 359,   1996, 4182, 3064, 2610, 1388,
       2081,  2993, 2637, 2161, 1730, 4051,  5588, 798,  1949, 4317, 2614,
       10703, 3809, 2668, 1151, 0,    15049, 1219, 2728, 3274},
      {0,     15399, 14584, 13646, 17588, 15955, 18832, 12628, 15318,
       12900, 19270, 14979, 15951, 13962, 14324, 12842, 14310, 15676,
       15848, 13596, 16763, 14107, 13746, 16332, 14302, 13180, 13997,
       18204, 17338, 15059, 12364, 11184, 14800, 17719, 13743, 12729,
       15306, 15049, 0,     15099, 15308, 14508},
      {0,     2242, 2323, 2124, 3222, 9851,  4467, 3130, 624,  3468, 2838,
       1118,  6009, 1423, 3743, 2650, 1442,  1485, 4945, 3319, 2397, 2471,
       3164,  2482, 3721, 2360, 2814, 3838,  5078, 1608, 2890, 5228, 3698,
       11615, 4064, 3752, 640,  1219, 15099, 0,    1837, 2306},
      {0,     1746, 982,  1753, 2848, 10060, 4855, 3690, 1184, 3487, 3829,
       2438,  5427, 1983, 4303, 2409, 2002,  1095, 4346, 2719, 2629, 3031,
       3724,  2092, 4281, 2569, 3374, 4069,  3737, 2728, 3098, 5437, 4258,
       11824, 3465, 4312, 1172, 2728, 15308, 1837, 0,    2094},
      {0,     2445, 1682, 953,  3866, 9259,  5110, 3372, 1596, 2876, 4362,
       3173,  4838, 1703, 3985, 1837, 2471,  2128, 3774, 2148, 3041, 2713,
       3406,  3126, 3963, 1769, 3056, 4482,  4436, 3196, 2298, 4637, 4461,
       11024, 2893, 3994, 1584, 3274, 14508, 2306, 2094, 0}};

  // [START demands_capacities]
  const std::vector<int64_t> demands{
      0,    500,  1340,  1231, 24480, 6810, 610, 880,  7310, 1268, 5000,
      1000, 1900, 22650, 527,  2605,  720,  328, 2730, 1900, 5000, 1200,
      1675, 480,  1000,  2579, 4000,  879,  520, 4200, 1073, 540,  65,
      17,   84,   1000,  1380, 350,   1548, 776, 5450, 6500};
  const std::vector<int64_t> vehicle_capacities{48413, 151873, 442117, 391820};
  // [END demands_capacities]
  int num_vehicles = 4;
  const RoutingIndexManager::NodeIndex depot{0};
  int64_t vehicle_distance_limit = 9999999999999999;
};
// [END data_model]
const auto* distance_dimension_name = "Distance";
const auto* capacity_dimension_name = "Capacity";

// [START solution_printer]
//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
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
// [END solution_printer]

void VrpCapacity() {
  // Instantiate the data problem.
  // [START data]
  DataModel data;
  // [END data]
  data.num_vehicles = absl::GetFlag(FLAGS_vrp_vehicles);

  // Create Routing Index Manager
  // [START index_manager]
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);
  // [END index_manager]

  // Create Routing Model.
  // [START routing_model]
  RoutingModel routing(manager);
  // [END routing_model]

  // Create and register a transit callback.
  // [START transit_callback]
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](const int64_t from_index,
                        const int64_t to_index) -> int64_t {
        // Convert from routing variable Index to distance matrix NodeIndex.
        const int from_node = manager.IndexToNode(from_index).value();
        const int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });
  // [END transit_callback]

  // Define cost of each arc.
  // [START arc_cost]
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  // [END arc_cost]

  routing.AddDimension(transit_callback_index, 0, data.vehicle_distance_limit,
                       true, distance_dimension_name);
  auto routing_dimension = routing.GetMutableDimension(distance_dimension_name);
  routing_dimension->SetGlobalSpanCostCoefficient(100);

  // Add Capacity constraint.
  // [START capacity_constraint]
  const int demand_callback_index = routing.RegisterUnaryTransitCallback(
      [&data, &manager](const int64_t from_index) -> int64_t {
        // Convert from routing variable Index to demand NodeIndex.
        const int from_node = manager.IndexToNode(from_index).value();
        return data.demands[from_node];
      });

  routing.AddDimensionWithVehicleCapacity(
      demand_callback_index,    // transit callback index
      int64_t{0},               // null capacity slack
      data.vehicle_capacities,  // vehicle maximum capacities
      true,                     // start cumul to zero
      capacity_dimension_name);
  // [END capacity_constraint]

  // Setting first solution heuristic.
  // [START parameters]
  RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
  search_parameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  search_parameters.set_local_search_metaheuristic(
      LocalSearchMetaheuristic::AUTOMATIC);
  search_parameters.mutable_time_limit()->set_seconds(1);
  search_parameters.set_log_search(true);
  //  [END parameters]

  // Solve the problem.
  // [START solve]
  const Assignment* solution = routing.SolveWithParameters(search_parameters);
  // [END solve]

  // Print solution on console.
  // [START print_solution]
  if (solution != nullptr) {
    PrintSolution(data, manager, routing, *solution);
  } else {
    LOG(INFO) << "nope:";
  }
  // [END print_solution]
}
}  // namespace operations_research

int main(int /*argc*/, char* /*argv*/[]) {
  operations_research::VrpCapacity();
  return EXIT_SUCCESS;
}
// [END program]
