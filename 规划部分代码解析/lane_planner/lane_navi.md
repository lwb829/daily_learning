# 节点lane_navi源码解析

路径：`autoware/core_planning/lane_planner/nodes/lane_navi/lane_navi.cpp`



## 功能

- 根据路由请求在矢量地图中寻找通往目的地的各条可行路径，并发布至话题 `"/lane_waypoints_array"` ，也就是说在给定起点和终点后，在矢量地图中找到对应的车道路径，然后再将车道路径转换为 `autoware_msgs::Lane` 格式的车道

  注意：这里可能存在多条路径，所以最后发布的是车道集合 `autoware_msgs::LaneArray lane_waypoint`

- 所属的功能包 lane_planner。
- 其订阅的话题包括 `"/route_cmd"` ，`"/vector_map_info/point"` ，`"/vector_map_info/lane"` 和 `"/vector_map_info/node"`。



## 分析

```
lane_planner::vmap::VectorMap all_vmap;
lane_planner::vmap::VectorMap lane_vmap;
```

在`autoware\core_planning\lane_planner\include\lane_planner\lane_planner_vmap.hpp`下，有：

```
struct VectorMap {
  std::vector<vector_map::Point> points;
  std::vector<vector_map::Lane> lanes;
  std::vector<vector_map::Node> nodes;
  std::vector<vector_map::StopLine> stoplines;
  std::vector<vector_map::DTLane> dtlanes;
};
```

它包含了构建矢量地图的所有元素，`all_vmap` 和 `lane_map` 均是 `lane_planner::vmap::VectorMap` 格式的数据

这些关于地图格式的消息，例如 `Point，Lane, Node` 等 ，其路径位于`/autoware/messages/vector_map_msgs/msg`



### `void create_waypoint(const tablet_socket_msgs::route_cmd& msg)`函数

是话题 `"/route_cmd"` 的回调函数，将在矢量地图中找寻到的车道级规划转化为轨迹点，并输出到对应的话题，注意，这里可能有多条车道。

1.1 `lane_planner::vmap::VectorMap coarse_vmap = lane_planner::vmap::create_coarse_vmap_from_route(msg)` 函数将传入的 `route_cmd` 数据中的路点 waypoint 转换为 `VectorMap` 类型的数据 `coarse_vmap`。通过route_cmd msg得到一个粗糙的vector map

1.2 `VectorMap create_fine_vmap(const VectorMap& lane_vmap, int lno, const VectorMap& coarse_vmap, double search_radius, int waypoint_max)` 主要根据粗略的导航地图 `coarse_vmap` ，在完备的矢量地图中查找精确的车道，将对应的信息完备的导航地图进行返回。

 `lno` 有三个选择：

- `int LNO_ALL = -1` : 在矢量地图的所有车道的所有点中查找起始点与终点；
- `int LNO_CROSSING = 0`：在矢量地图的交叉路口寻找合适的点；
- `int LNO_MOSTLEFT = 1`：在矢量地图的最左侧道路寻找合适的点；



### 三个分别订阅点云、车道和节点的程序

```
void cache_point(const vector_map::PointArray& msg);
void cache_lane(const vector_map::LaneArray& msg);
void cache_node(const vector_map::NodeArray& msg);
```

三个函数分别为话题 `"/vector_map_info/point"` ，`"/vector_map_info/lane"` 和 `"/vector_map_info/node"` 的回调函数，用于更新 `all_vmap` 中的点云、车道和节点，从而创建完备的矢量地图。

2.1 `void update_values()` 在三个回调函数中都调用了 `update_values()` 函数，其一方面根据更新后的 `all_vmap` 创建同类型的 `lane_vmap`，然后根据 `cached_route` 中的路点创建 waypoint，也就是调用了 `create_waypoint()` 函数。

2.2 `lane_vmap = lane_planner::vmap::create_lane_vmap(all_vmap, lane_planner::vmap::LNO_ALL)` 主要将 `all_vmap` 中的 `Lane，Node,Point,StopLine,DTLane` 复制到 `lane_vmap` 中

其节点逻辑为：先由回调函数 `create_waypoint` 从 `"/route_cmd"` 中取出起点终点信息，然后放入 `cache_route` 当中。然后三个回调函数依次订阅全局矢量图信息，直到矢量图信息完备，执行 `create_waypoint` ，最后将车道数组发布在话题 `"/lane_waypoints_array"` 中