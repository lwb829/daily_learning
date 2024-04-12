## 一、op_global_planner节点

读入矢量地图，在设定的起点/当前位置与终点之间进行全局路径规划。

起点是一个点，终点是包含多个waypoint的vector，在实际运行中，第一次设置起点和终点后，会先从当前位置or起始位置规划到第一个终点的位置，然后再规划走到vector里面的下一个终点。

1、launch文件：

```xml
<launch>
  <arg name="pathDensity"           default="0.5" /> <!-- 两个路径点之间的距离为0.5-->
  <arg name="enableSmoothing"         default="true" /> <!-- 1 or 0 -->
  <arg name="enableLaneChange"         default="false" /> <!-- 1 or 0 --> 不换道
  <arg name="enableRvizInput"         default="true" /> <!-- 1 or 0 -->
  <arg name="goalDataPath"            default = "" />
  <arg name="enableReplan"            default="false" /> <!-- 1 or 0 -->  不进行重规划
  <arg name="velocitySource"          default="1" /> <!-- 速度来源 (0-Odometry, 1-autoware current_velocities, 2-car_info) "" -->
  <arg name="mapSource"             default="0" /> 地图来自autoware自己定义的地图格式<!-- Autoware=0, Vector Map Folder=1, kml file=2 -->
  <arg name="mapFileName"           default="" /> <!-- incase of kml map source -->
  <arg name="enableDynamicMapUpdate"       default="false" />

  <arg name="RoadEdge"      default="false"/>
  <arg name="DistanceToRoadEdge" default="1.0"/>  到道路边沿的距离
  <arg name="ReedsShepp" default="false"/>
  
<node pkg="op_global_planner" type="op_global_planner" name="op_global_planner" output="screen">
    
    <param name="pathDensity"         value="$(arg pathDensity)" />
    <param name="enableSmoothing"       value="$(arg enableSmoothing)" />
    <param name="enableLaneChange"       value="$(arg enableLaneChange)" />
    <param name="enableRvizInput"       value="$(arg enableRvizInput)" />
    <param name="enableReplan"         value="$(arg enableReplan)" />        
    <param name="velocitySource"       value="$(arg velocitySource)" />
    <param name="mapSource"         value="$(arg mapSource)" />
    <param name="mapFileName"         value="$(arg mapFileName)" />
    <param name="goalDataPath"        value="$(arg goalDataPath)" />
    <param name="ReedsShepp"          value="$(arg ReedsShepp)"/>
    <param name="RoadEdge"            value="$(arg RoadEdge)"/>
    <param name="DistanceToRoadEdge"  value="$(arg DistanceToRoadEdge)"/>
    <param name="enableDynamicMapUpdate"   value="$(arg enableDynamicMapUpdate)" />
    <remap from="move_base_simple/goal" to="scenario_manager/modified_goal" />  
  </node> 
  
</launch>
```

2、主函数。都在GlobalPlanningNS namespace下。

```c++
  ros::init(argc, argv, "op_global_planner");
  GlobalPlanningNS::GlobalPlanner global_planner;
  global_planner.MainLoop();
  return 0;
```

实例化GlobalPlanner类，在构造函数中进行初始化，读入launch文件的参数，并定义输入输出话题。

1）获取map-->world之间的坐标关系：

函数调用：

```
  tf::StampedTransform transform;
  PlannerHNS::ROSHelpers::GetTransformFromTF("map", "world", transform);
```

函数定义：

```
void ROSHelpers::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
  static tf::TransformListener listener;
  int nFailedCounter = 0;
  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException& ex)
    {
      if(nFailedCounter > 2) //3s内没查到
      {
        ROS_ERROR("%s", ex.what());
      }
      ros::Duration(1.0).sleep();  //1s查询1次
      nFailedCounter ++;
    }
  }
}
```

2）输入：

```
if(m_params.bEnableRvizInput)  //如果RVIZ赋值有效，默认有效
  {
    sub_start_pose = nh.subscribe("/initialpose", 1, &GlobalPlanner::callbackGetStartPose, this);
    // sub_goal_pose = nh.subscribe("move_base_simple/goal", 1, &GlobalPlanner::callbackGetGoalPose, this);
    sub_scenario = nh.subscribe("scenario_manager/scenario_cmd",1,&GlobalPlanner::callbackGetScenario,this);
  }
  else
  {
    LoadSimulationData(goalDataPath);
  }
  //订阅当前位姿
  sub_current_pose = nh.subscribe("/current_pose", 10, &GlobalPlanner::callbackGetCurrentPose, this);
  //订阅当前速度，3选1
  int bVelSource = 1;
  nh.getParam("/op_global_planner/velocitySource", bVelSource);
  if(bVelSource == 0)
    sub_robot_odom = nh.subscribe("/odom", 10, &GlobalPlanner::callbackGetRobotOdom, this);
  else if(bVelSource == 1) 默认
    sub_current_velocity = nh.subscribe("/current_velocity", 10, &GlobalPlanner::callbackGetVehicleStatus, this);
  else if(bVelSource == 2)
    sub_can_info = nh.subscribe("/can_info", 10, &GlobalPlanner::callbackGetCANInfo, this);

  if(m_params.bEnableDynamicMapUpdate)//如果地图动态更新，默认不更新
    sub_road_status_occupancy = nh.subscribe<>("/occupancy_road_status", 1, &GlobalPlanner::callbackGetRoadStatusOccupancyGrid, this);

  //矢量地图部分
  sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &GlobalPlanner::callbackGetVMLanes,  this);
  sub_points = nh.subscribe("/vector_map_info/point", 1, &GlobalPlanner::callbackGetVMPoints,  this);
  sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &GlobalPlanner::callbackGetVMdtLanes,  this);
  sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &GlobalPlanner::callbackGetVMIntersections,  this);
  sup_area = nh.subscribe("/vector_map_info/area", 1, &GlobalPlanner::callbackGetVMAreas,  this);
  sub_lines = nh.subscribe("/vector_map_info/line", 1, &GlobalPlanner::callbackGetVMLines,  this);
  sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &GlobalPlanner::callbackGetVMStopLines,  this);
  sub_signals = nh.subscribe("/vector_map_info/signal", 1, &GlobalPlanner::callbackGetVMSignal,  this);
  sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &GlobalPlanner::callbackGetVMVectors,  this);
  sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &GlobalPlanner::callbackGetVMCurbs,  this);
  sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &GlobalPlanner::callbackGetVMRoadEdges,  this);
  sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &GlobalPlanner::callbackGetVMWayAreas,  this);
  sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &GlobalPlanner::callbackGetVMCrossWalks,  this);
  sub_nodes = nh.subscribe("/vector_map_info/node", 1, &GlobalPlanner::callbackGetVMNodes,  this);
```

3）输出：

```
  //输出：
  pub_Paths = nh.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);//全局路径
  pub_PathsRviz = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_rviz", 1, true);
  pub_MapRviz  = nh.advertise<visualization_msgs::MarkerArray>("vector_map_center_lines_rviz", 1, true);
  pub_GoalsListRviz = nh.advertise<visualization_msgs::MarkerArray>("op_destinations_rviz", 1, true);
  vehicle_status_pub_ = nh.advertise<autoware_msgs::DrivingStatus>("driving_status", 1); //车辆状态
```

3、具体规划操作在MainLoop()函数中实现，**全局路径规划频率为25Hz。最后将规划得到的全局路径发布出来，话题为lane_waypoints_array**

1）根据订阅的矢量地图创建道路网络：

```
else if (m_params.mapSource == PlannerHNS::MAP_AUTOWARE && !m_bKmlMap)//默认
    {
      std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

      if(m_MapRaw.GetVersion()==2)//第2版地图
      {
        std::cout << "Map Version 2" << endl;
        m_bKmlMap = true;
        //构建道路网络
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
            m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
      }
      else if(m_MapRaw.GetVersion()==1) //第1版地图
      {
        std::cout << "Map Version 1" << endl;
        m_bKmlMap = true;
        PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
            m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
            m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,  m_MapRaw.pSignals->m_data_list,
            m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
            m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_params.bEnableLaneChange, false);
      }

      if(m_bKmlMap)
      {
        visualization_msgs::MarkerArray map_marker_array;
        PlannerHNS::ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, map_marker_array);
        pub_MapRviz.publish(map_marker_array);//可视化
      }
    }
```

2）判断是否需要规划：

```
    //判断是否需要规划
    if(m_GoalsPos.size() > 0 && activate_scenario)//如果目标点不为空，且
    {
      //产生的路径不为空，且第一条路径点数大于3
      if(m_GeneratedTotalPaths.size() > 0 && m_GeneratedTotalPaths.at(0).size() > 3)
      {
        if(m_params.bEnableReplanning)//重规划
        {
          PlannerHNS::RelativeInfo info;
          bool ret = PlannerHNS::PlanningHelpers::GetRelativeInfoRange(m_GeneratedTotalPaths, m_CurrentPose, 0.75, info);
          if(ret == true && info.iGlobalPath >= 0 &&  info.iGlobalPath < m_GeneratedTotalPaths.size() && info.iFront > 0 && info.iFront < m_GeneratedTotalPaths.at(info.iGlobalPath).size())
          {
            double remaining_distance =m_GeneratedTotalPaths.at(info.iGlobalPath).at(m_GeneratedTotalPaths.at(info.iGlobalPath).size()-1).cost - (m_GeneratedTotalPaths.at(info.iGlobalPath).at(info.iFront).cost + info.to_front_distance);
            if(remaining_distance <= REPLANNING_DISTANCE)
            {
              bMakeNewPlan = true;
              if(m_GoalsPos.size() > 0)
                m_iCurrentGoalIndex = (m_iCurrentGoalIndex + 1) % m_GoalsPos.size();
              std::cout << "Current Goal Index = " << m_iCurrentGoalIndex << std::endl << std::endl;
            }
          }
        }
        if(get_new_goal)
        {
          bMakeNewPlan = true;
        }
      }
      else
        bMakeNewPlan = true;
```

3）进行全局路径规划;

```
      //进行全局路径规划
      if(bMakeNewPlan || (m_params.bEnableDynamicMapUpdate && UtilityHNS::UtilityH::GetTimeDiffNow(m_ReplnningTimer) > REPLANNING_TIME))
      {
        UtilityHNS::UtilityH::GetTickCount(m_ReplnningTimer);
        PlannerHNS::WayPoint goalPoint = m_GoalsPos.at(m_iCurrentGoalIndex);
        bool bNewPlan = GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths);

        if(bNewPlan)
        {
          bMakeNewPlan = false;
          get_new_goal = false;
          VisualizeAndSend(m_GeneratedTotalPaths);
        }
      }
```

全局路径规划函数：GenerateGlobalPlan(m_CurrentPose, goalPoint, m_GeneratedTotalPaths)

```
//全局路径规划
//参数：起点，终点，路径集
bool GlobalPlanner::GenerateGlobalPlan(PlannerHNS::WayPoint& startPoint, PlannerHNS::WayPoint& goalPoint, std::vector<std::vector<PlannerHNS::WayPoint> >& generatedTotalPaths)
{
  std::vector<int> predefinedLanesIds;
  double ret = 0;

  ret = m_PlannerH.PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);

  if(ret == 0)
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString()
                        << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
    return false;
  }


  if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size()>0)
  {
    if(m_params.bEnableSmoothing)
    {
      for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
      {
        PlannerHNS::PlanningHelpers::FixPathDensity(generatedTotalPaths.at(i), m_params.pathDensity);
        PlannerHNS::PlanningHelpers::SmoothPath(generatedTotalPaths.at(i), 0.49, 0.35 , 0.01);
      }
    }

    for(unsigned int i=0; i < generatedTotalPaths.size(); i++)
    {
      PlannerHNS::PlanningHelpers::CalcAngleAndCost(generatedTotalPaths.at(i));
      if(m_GlobalPathID > 10000)
        m_GlobalPathID = 1;

      for(unsigned int j=0; j < generatedTotalPaths.at(i).size(); j++)
        generatedTotalPaths.at(i).at(j).gid = m_GlobalPathID;

      m_GlobalPathID++;

      std::cout << "New DP Path -> " << generatedTotalPaths.at(i).size() << std::endl;

      // 还要额外发布一个end_point_status = 0，因为在a*场景的最后，这个标志位发布了2，在mpc_follower中对2进行了停车处理。所以在op planner开始的时候要把这个标志位重新置为0
      end_point_status_.vehicle_status.data = 0;
      vehicle_status_pub_.publish(end_point_status_);

    }
    return true;
  }
  else
  {
    std::cout << "Can't Generate Global Path for Start (" << startPoint.pos.ToString() << ") and Goal (" << goalPoint.pos.ToString() << ")" << std::endl;
  }
  return false;
}
```

真正的规划还在PlanUsingDP(startPoint, goalPoint, MAX_GLOBAL_PLAN_DISTANCE, m_params.bEnableLaneChange, predefinedLanesIds, m_Map, generatedTotalPaths);函数中：

此函数首先获取起点和终点的相关信息：

在由用户输入的start和goalPos（一般在Rviz上画出来）调用GetClosestWaypointFromMap，获得vector map中的最近的位置pStart和pGoal后，再调用GetRelativeInfo获取pStart和pGoal的relativeInfo start_info和goal_info。

```
//路径规划
double PlannerH::PlanUsingDP(const WayPoint& start,
    const WayPoint& goalPos,
    const double& maxPlanningDistance,
    const bool bEnableLaneChange,
    const std::vector<int>& globalPath,
    RoadNetwork& map,
    std::vector<std::vector<WayPoint> >& paths, vector<WayPoint*>* all_cell_to_delete)
{
  //调用GetCloseWaypointFromMap()方法获得vector map中的最近的pStart和pGoal
  PlannerHNS::WayPoint* pStart = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(start, map);
  PlannerHNS::WayPoint* pGoal = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(goalPos, map);
  bool bEnableGoalBranching = false;

  if(!pStart ||  !pGoal)
  {
    GPSPoint sp = start.pos;
    GPSPoint gp = goalPos.pos;
    cout << endl << "Error: PlannerH -> Can't Find Global Waypoint Nodes in the Map for Start (" <<  sp.ToString() << ") and Goal (" << gp.ToString() << ")" << endl;
    return 0;
  }

  if(!pStart->pLane || !pGoal->pLane)
  {
    cout << endl << "Error: PlannerH -> Null Lane, Start (" << pStart->pLane << ") and Goal (" << pGoal->pLane << ")" << endl;
    return 0;
  }

  RelativeInfo start_info, goal_info;
  //调用GetRelativeInfo()方法获得pStart和pGoal的relativeInfo， 
  //start_info和goal_info即相关信息，数据类型为RelativeInfo
  PlanningHelpers::GetRelativeInfo(pStart->pLane->points, start, start_info);
  PlanningHelpers::GetRelativeInfo(pGoal->pLane->points, goalPos, goal_info);

  vector<WayPoint> start_path, goal_path;

  if(fabs(start_info.perp_distance) > START_POINT_MAX_DISTANCE)
  {
    GPSPoint sp = start.pos;
    cout << endl << "Error: PlannerH -> Start Distance to Lane is: " << start_info.perp_distance
        << ", Pose: " << sp.ToString() << ", LanePose:" << start_info.perp_point.pos.ToString()
        << ", LaneID: " << pStart->pLane->id << " -> Check origin and vector map. " << endl;
    return 0;
  }

  if(fabs(goal_info.perp_distance) > GOAL_POINT_MAX_DISTANCE)
  {
    if(fabs(start_info.perp_distance) > 20)
    {
      GPSPoint gp = goalPos.pos;
      cout << endl << "Error: PlannerH -> Goal Distance to Lane is: " << goal_info.perp_distance
          << ", Pose: " << gp.ToString() << ", LanePose:" << goal_info.perp_point.pos.ToString()
          << ", LaneID: " << pGoal->pLane->id << " -> Check origin and vector map. " << endl;
      return 0;
    }
    else
    {
      WayPoint wp = *pGoal;
      wp.pos.x = (goalPos.pos.x+pGoal->pos.x)/2.0;
      wp.pos.y = (goalPos.pos.y+pGoal->pos.y)/2.0;
      goal_path.push_back(wp);
      goal_path.push_back(goalPos);
    }
  }

  vector<WayPoint*> local_cell_to_delete;
  WayPoint* pLaneCell = 0;
  char bPlan = 'A';
  //调用BuildPlanningSearchTreeV2函数，寻找是否存在由pStart到达pGoal的最短路径
  if(all_cell_to_delete)
    pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeV2(pStart, *pGoal, globalPath, maxPlanningDistance,bEnableLaneChange, *all_cell_to_delete);
  else
    pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeV2(pStart, *pGoal, globalPath, maxPlanningDistance,bEnableLaneChange, local_cell_to_delete);

  if(!pLaneCell)
  {
    //如果BuildPlanningSearchTreeV2失败，则会调用BuildPlanningSearchTreeStraight再次寻找路径，
    //它和BuildPlanningSearchTreeV2的区别在于它不会考虑laneChange造成的pLeft和pRight的wayPoint，
    //只考虑pFronts里的waypoints

    bPlan = 'B';
    cout << endl << "PlannerH -> Plan (A) Failed, Trying Plan (B)." << endl;

    if(all_cell_to_delete)
      pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, *all_cell_to_delete);
    else
      pLaneCell =  PlanningHelpers::BuildPlanningSearchTreeStraight(pStart, BACKUP_STRAIGHT_PLAN_DISTANCE, local_cell_to_delete);

    if(!pLaneCell)
    {
      bPlan = 'Z';
      cout << endl << "PlannerH -> Plan (B) Failed, Sorry we Don't have plan (C) This is the END." << endl;
      return 0;
    }
  }

  vector<WayPoint> path;
  vector<vector<WayPoint> > tempCurrentForwardPathss;
  /*
  由BuildPlanningSearchTreeV2返回的能到达的最接近goal的waypoint（pHead)，遍历回到pStart。
  首先pHead会按直路遍历回pStart，如果存在laneChange导致的pLeft或者pRight，就会先调到pLeft或者pRight，
  然后遍历回到pStart。找到的全部路径会存放进PlanUsingDP函数里的vector<WayPoint> path里
  */
  PlanningHelpers::TraversePathTreeBackwards(pLaneCell, pStart, globalPath, path, tempCurrentForwardPathss);
  if(path.size()==0) return 0;

  paths.clear();

  if(bPlan == 'A')
  {
    PlanningHelpers::ExtractPlanAlernatives(path, paths);
  }
  else if (bPlan == 'B')
  {
    paths.push_back(path);
  }

  //attach start path to beginning of all paths, but goal path to only the path connected to the goal path.
  for(unsigned int i=0; i< paths.size(); i++ )
  {
    paths.at(i).insert(paths.at(i).begin(), start_path.begin(), start_path.end());
    if(paths.at(i).size() > 0)
    {
      //if(hypot(paths.at(i).at(paths.at(i).size()-1).pos.y-goal_info.perp_point.pos.y, paths.at(i).at(paths.at(i).size()-1).pos.x-goal_info.perp_point.pos.x) < 1.5)
      {

        if(paths.at(i).size() > 0 && goal_path.size() > 0)
        {
          goal_path.insert(goal_path.begin(), paths.at(i).end()-5, paths.at(i).end());
          PlanningHelpers::SmoothPath(goal_path, 0.25, 0.25);
          PlanningHelpers::FixPathDensity(goal_path, 0.75);
          PlanningHelpers::SmoothPath(goal_path, 0.25, 0.35);
          paths.at(i).erase(paths.at(i).end()-5, paths.at(i).end());
          paths.at(i).insert(paths.at(i).end(), goal_path.begin(), goal_path.end());
        }
      }
    }
  }

  cout << endl <<"Info: PlannerH -> Plan (" << bPlan << ") Path With Size (" << (int)path.size() << "), MultiPaths No(" << paths.size() << ") Extraction Time : " << endl;


  if(path.size()<2)
  {
    cout << endl << "Err: PlannerH -> Invalid Path, Car Should Stop." << endl;
    if(pLaneCell && !all_cell_to_delete)
      DeleteWaypoints(local_cell_to_delete);
    return 0 ;
  }

  if(pLaneCell && !all_cell_to_delete)
    DeleteWaypoints(local_cell_to_delete);

  double totalPlanningDistance = path.at(path.size()-1).cost;
  return totalPlanningDistance;
}
```



BuildPlanningSearchTreeV2函数实现：

这个函数的大致逻辑为：使用动态规划DP算法寻找是否存在由pStart到达pGoal的最短路径。会从pStart开始，不断遍历它能够到达的周边的下一个waypoint，（判断如果enbaleLaneChange，会找它的pLeft和pRight的wayPoint，这在前面构建vector map时会创建；如果没有enbaleLaneChange，则找它pFronts里的所有waypoints），加入到nextLeafToTrace，然后找到nextLeafToTrace里面WayPoint->cost最小的那个waypoint，把它设为选择的当前waypoint（代码里为pH），然后继续遍历nextLeafToTrace。本质上为贪心算法，只要保证在所有可到达的waypoint里，选择cost最小的那个waypoint，就保证里local optimal solution。这时保证有一条path with min_cost能够到达goal，并返回最终找到的那个waypoint（代码里为pH）。

```
//寻找是否存在由pStart到达pGoal的最短路径
WayPoint* PlanningHelpers::BuildPlanningSearchTreeV2(WayPoint* pStart,
    const WayPoint& goalPos,
    const vector<int>& globalPath,
    const double& DistanceLimit,
    const bool& bEnableLaneChange,
    vector<WayPoint*>& all_cells_to_delete)
{
  if(!pStart) return NULL;
  vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;
  WayPoint* pZero = 0;
  WayPoint* wp    = new WayPoint();
  *wp = *pStart;
  nextLeafToTrace.push_back(make_pair(pZero, wp));
  all_cells_to_delete.push_back(wp);

  double     distance     = 0;
  double     before_change_distance  = 0;
  WayPoint*   pGoalCell     = 0;
  double     nCounter     = 0;

  while(nextLeafToTrace.size()>0)
  {
    nCounter++;
    unsigned int min_cost_index = 0;
    double min_cost = DBL_MAX;
    for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
    {
      if(nextLeafToTrace.at(i).second->cost < min_cost)
      {
        min_cost = nextLeafToTrace.at(i).second->cost;
        min_cost_index = i;
      }
    }

    WayPoint* pH   = nextLeafToTrace.at(min_cost_index).second;
    assert(pH != 0);
    nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);
    double distance_to_goal = distance2points(pH->pos, goalPos.pos);
    double angle_to_goal = UtilityH::AngleBetweenTwoAnglesPositive(UtilityH::FixNegativeAngle(pH->pos.a), UtilityH::FixNegativeAngle(goalPos.pos.a));
    if( distance_to_goal <= 0.1 && angle_to_goal < M_PI_4)
    {
      cout << "Goal Found, LaneID: " << pH->laneId <<", Distance : " << distance_to_goal << ", Angle: " << angle_to_goal*RAD2DEG << endl;
      pGoalCell = pH;
      break;
    }
    else
    {
      if(pH->pLeft && !CheckLaneExits(all_cells_to_delete, pH->pLeft->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pLeft) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
      {
        wp = new WayPoint();
        *wp = *pH->pLeft;
        double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
        distance += d;
        before_change_distance = -LANE_CHANGE_MIN_DISTANCE*3;

        for(unsigned int a = 0; a < wp->actionCost.size(); a++)
        {
          //if(wp->actionCost.at(a).first == LEFT_TURN_ACTION)
            d += wp->actionCost.at(a).second;
        }

        wp->cost = pH->cost + d;
        wp->pRight = pH;
        wp->pLeft = 0;

        nextLeafToTrace.push_back(make_pair(pH, wp));
        all_cells_to_delete.push_back(wp);
      }

      if(pH->pRight && !CheckLaneExits(all_cells_to_delete, pH->pRight->pLane) && !CheckNodeExits(all_cells_to_delete, pH->pRight) && bEnableLaneChange && before_change_distance > LANE_CHANGE_MIN_DISTANCE)
      {
        wp = new WayPoint();
        *wp = *pH->pRight;
        double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
        distance += d;
        before_change_distance = -LANE_CHANGE_MIN_DISTANCE*3;

        for(unsigned int a = 0; a < wp->actionCost.size(); a++)
        {
          //if(wp->actionCost.at(a).first == RIGHT_TURN_ACTION)
            d += wp->actionCost.at(a).second;
        }
        wp->cost = pH->cost + d ;
        wp->pLeft = pH;
        wp->pRight = 0;
        nextLeafToTrace.push_back(make_pair(pH, wp));
        all_cells_to_delete.push_back(wp);
      }

      for(unsigned int i =0; i< pH->pFronts.size(); i++)
      {
        if(CheckLaneIdExits(globalPath, pH->pLane) && pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
        {
          wp = new WayPoint();
          *wp = *pH->pFronts.at(i);
          double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
          distance += d;
          before_change_distance += d;

          for(unsigned int a = 0; a < wp->actionCost.size(); a++)
          {
            //if(wp->actionCost.at(a).first == FORWARD_ACTION)
              d += wp->actionCost.at(a).second;
          }
          wp->cost = pH->cost + d;
          wp->pBacks.push_back(pH);
          nextLeafToTrace.push_back(make_pair(pH, wp));
          all_cells_to_delete.push_back(wp);
        }
      }
    }
    if(distance > DistanceLimit && globalPath.size()==0)
    {
      //if(!pGoalCell)
      cout << "Goal Not Found, LaneID: " << pH->laneId <<", Distance : " << distance << endl;
      pGoalCell = pH;
      break;
    }
    //pGoalCell = pH;
  }
  while(nextLeafToTrace.size()!=0)
    nextLeafToTrace.pop_back();
  //closed_nodes.clear();
  return pGoalCell;
}
```

​    如果BuildPlanningSearchTreeV2失败，则会调用BuildPlanningSearchTreeStraight再次寻找路径，

​    它和BuildPlanningSearchTreeV2的区别在于它不会考虑laneChange造成的pLeft和pRight的wayPoint，

​    只考虑pFronts里的waypoints：

```
WayPoint* PlanningHelpers::BuildPlanningSearchTreeStraight(WayPoint* pStart,
const double& DistanceLimit,vector<WayPoint*>& all_cells_to_delete)
{
  if(!pStart) return NULL;

  vector<pair<WayPoint*, WayPoint*> >nextLeafToTrace;

  WayPoint* pZero = 0;
  WayPoint* wp    = new WayPoint();
  *wp = *pStart;
  wp->cost = 0;
  nextLeafToTrace.push_back(make_pair(pZero, wp));
  all_cells_to_delete.push_back(wp);

  double     distance     = 0;
  WayPoint*   pGoalCell     = 0;
  double     nCounter     = 0;

  while(nextLeafToTrace.size()>0)
  {
    nCounter++;

    unsigned int min_cost_index = 0;
    double min_cost = DBL_MAX;

    for(unsigned int i=0; i < nextLeafToTrace.size(); i++)
    {
      if(nextLeafToTrace.at(i).second->cost < min_cost)
      {
        min_cost = nextLeafToTrace.at(i).second->cost;
        min_cost_index = i;
      }
    }

    WayPoint* pH   = nextLeafToTrace.at(min_cost_index).second;
    assert(pH != 0);

    nextLeafToTrace.erase(nextLeafToTrace.begin()+min_cost_index);

    for(unsigned int i =0; i< pH->pFronts.size(); i++)
    {
      if(pH->pFronts.at(i) && !CheckNodeExits(all_cells_to_delete, pH->pFronts.at(i)))
      {
        wp = new WayPoint();
        *wp = *pH->pFronts.at(i);

        double d = hypot(wp->pos.y - pH->pos.y, wp->pos.x - pH->pos.x);
        distance += d;

        wp->cost = pH->cost + d;
        wp->pBacks.push_back(pH);
        if(wp->cost < DistanceLimit)
        {
          nextLeafToTrace.push_back(make_pair(pH, wp));
          all_cells_to_delete.push_back(wp);
        }
        else
          delete wp;
      }
    }
    pGoalCell = pH;
  }

  while(nextLeafToTrace.size()!=0)
    nextLeafToTrace.pop_back();

  return pGoalCell;
}
```

接下来调用TraversePathTreeBackwards函数，它是由BuildPlanningSearchTreeV2返回的能到达的最接近goal的waypoint（pHead)，遍历回到pStart。首先pHead会按直路遍历回pStart，如果存在laneChange导致的pLeft或者pRight，就会先调到pLeft或者pRight，然后遍历回到pStart。找到的全部路径会存放进PlanUsingDP函数里的vector<WayPoint> path里。
调用：TraversePathTreeBackwards(pLaneCell, pStart, globalPath, path, tempCurrentForwardPathss);

定义：

```
void PlanningHelpers::TraversePathTreeBackwards(WayPoint* pHead, WayPoint* pStartWP,const vector<int>& globalPathIds,
    vector<WayPoint>& localPath, std::vector<std::vector<WayPoint> >& localPaths)
{
  if(pHead != NULL && pHead->id != pStartWP->id)
  {
    if(pHead->pBacks.size()>0)
    {
      localPaths.push_back(localPath);
      TraversePathTreeBackwards(GetMinCostCell(pHead->pBacks, globalPathIds),pStartWP, globalPathIds, localPath, localPaths);
      pHead->bDir = FORWARD_DIR;
      localPath.push_back(*pHead);
    }
    else if(pHead->pLeft && pHead->cost > 0)
    {
      //vector<Vector2D> forward_path;
      //TravesePathTreeForwards(pHead->pLeft, forward_path, FORWARD_RIGHT);
      //localPaths.push_back(forward_path);
      cout << "Global Lane Change  Right " << endl;
      TraversePathTreeBackwards(pHead->pLeft,pStartWP, globalPathIds, localPath, localPaths);
      pHead->bDir = FORWARD_RIGHT_DIR;
      localPath.push_back(*pHead);
    }
    else if(pHead->pRight && pHead->cost > 0)
    {
      //vector<Vector2D> forward_path;
      //TravesePathTreeForwards(pHead->pRight, forward_path, FORWARD_LEFT);
      //localPaths.push_back(forward_path);

      cout << "Global Lane Change  Left " << endl;
      TraversePathTreeBackwards(pHead->pRight,pStartWP, globalPathIds, localPath, localPaths);
      pHead->bDir = FORWARD_LEFT_DIR;
      localPath.push_back(*pHead);
    }
  }
  else
    assert(pHead);
}
```



## 二. op_local_planner

op_global_planner发布全局路径lane_waypoints_array，包含了这一段最小cost的global_path；
==>op_trajectory_generator订阅并处理后发布局部路径集local_trajectores；

==>op_trajectory_evaluator对局部路径集中的局部路径进行代价评价
=>op_behavior_selector订阅并处理后发布current_behavior；

上述三个topic被op_trajectory_evaluator订阅，并同时订阅op_motion_predictor发布的predicted_objects并处理后发布local_weighted_trajectories和local_trajectory_cost.

op_local_planner包含五个node，分别是op_behavior_selector，op_common_params，op_motion_predictor，op_trajectory_evaluator，op_trajectory_generator，每个node对应一个launch文件。

### 2.1、op_trajectory_generator节点：

1、launch文件

共包括5个launch文件，每个node对应一个launch

op_trajectory_generator.launch：

```
<launch>
  <!-- Trajectory Generation Specific Parameters -->
  <arg name="samplingTipMargin"     default="1.5"  /> <!--original 3-->
  <arg name="samplingOutMargin"     default="4.5" />  <!--original 6-->
  <arg name="samplingSpeedFactor"   default="0.25" />    
  <arg name="enableHeadingSmoothing"   default="false" />
      
  <node pkg="op_local_planner" type="op_trajectory_generator" name="op_trajectory_generator" output="screen">
  
  <param name="samplingTipMargin"     value="$(arg samplingTipMargin)"  /> 
  <param name="samplingOutMargin"     value="$(arg samplingOutMargin)" /> 
  <param name="samplingSpeedFactor"     value="$(arg samplingSpeedFactor)" />    
  <param name="enableHeadingSmoothing"   value="$(arg enableHeadingSmoothing)" />
      
  </node>        
      
</launch>
```

在op_trajectory_generator.launch中，提供了4个参数类型，与代码中的对应关系如下
launch.samplingTipMargin <=> cpp.carTipMargin
launch.samplingOutMargin <=> cpp.rollInMargin
samplingSpeedFactor <=> rollInSpeedFactor
enableHeadingSmoothing <=> enableHeadingSmoothing
![img](https://img-blog.csdnimg.cn/b3cb5a2a400e47e993b92150574fcb8d.png)

#### 2、 订阅的topic

```
sub_initialpose=nh.subscribe("/initialpose",1, &TrajectoryGen::callbackGetInitPose, this);
//当前位姿
sub_current_pose=nh.subscribe("/current_pose",10,&TrajectoryGen::callbackGetCurrentPose, this);
sub_current_velocity=nh.subscribe("/current_velocity",10, &TrajectoryGen::callbackGetVehicleStatus,this);
//订阅全局路径
sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryGen::callbackGetGlobalPlannerPath, this);
sub_scenario = nh.subscribe("scenario_manager/scenario_cmd",1,&TrajectoryGen::callbackGetScenario,this);
```

#### 3、 发布的topic

```
//发布局部路径集
pub_LocalTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_trajectories", 1);
pub_LocalTrajectoriesRviz = nh.advertise<visualization_msgs::MarkerArray>("local_trajectories_gen_rviz", 1);
```

4、在op_trajectory_generator节点中实例化一个TrajectoryGen类，主要功能在MainLoop()函数中实现，该函数执行频率100Hz：

**局部路径规划频率为100Hz,最后将生成的局部路径集发布出来，话题为local_trajectories**

```
void TrajectoryGen::MainLoop()
{
  ros::Rate loop_rate(100);

  PlannerHNS::WayPoint prevState, state_change;

  while (ros::ok())
  {
    ros::spinOnce();

    if(bInitPos && m_GlobalPaths.size()>0 && activate_scenario)
    {
      m_GlobalPathSections.clear();//全局路径的一段

      for(unsigned int i = 0; i < m_GlobalPaths.size(); i++)
      {
        t_centerTrajectorySmoothed.clear();
        //传入参数为原始的globalPath，一组WayPoint的vector，currentPos，最小距离，路径密度，返回extractedPath
        PlannerHNS::PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,
            m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);
        //得到全局路径的一段
        m_GlobalPathSections.push_back(t_centerTrajectorySmoothed);
      }

      std::vector<PlannerHNS::WayPoint> sampledPoints_debug;
      //计算Runoff轨迹，返回值为local_rollOutPaths
      m_Planner.GenerateRunoffTrajectory(m_GlobalPathSections, m_CurrentPos,
                m_PlanningParams.enableLaneChange,
                m_VehicleStatus.speed,
                m_PlanningParams.microPlanDistance,
                m_PlanningParams.maxSpeed,
                m_PlanningParams.minSpeed,
                m_PlanningParams.carTipMargin,
                m_PlanningParams.rollInMargin,
                m_PlanningParams.rollInSpeedFactor,
                m_PlanningParams.pathDensity,
                m_PlanningParams.rollOutDensity,
                m_PlanningParams.rollOutNumber,
                m_PlanningParams.smoothingDataWeight,
                m_PlanningParams.smoothingSmoothWeight,
                m_PlanningParams.smoothingToleranceError,
                m_PlanningParams.speedProfileFactor,
                m_PlanningParams.enableHeadingSmoothing,
                -1 , -1,
                m_RollOuts, sampledPoints_debug);

      autoware_msgs::LaneArray local_lanes;
      for(unsigned int i=0; i < m_RollOuts.size(); i++)
      {
        for(unsigned int j=0; j < m_RollOuts.at(i).size(); j++) //第i条
        {
          autoware_msgs::Lane lane;
          PlannerHNS::PlanningHelpers::PredictConstantTimeCostForTrajectory(m_RollOuts.at(i).at(j), m_CurrentPos, m_PlanningParams.minSpeed, m_PlanningParams.microPlanDistance);
          PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(m_RollOuts.at(i).at(j), lane);
          lane.closest_object_distance = 0;
          lane.closest_object_velocity = 0;
          lane.cost = 0;
          lane.is_blocked = false;
          lane.lane_index = i;
          local_lanes.lanes.push_back(lane);
        }
      }
      pub_LocalTrajectories.publish(local_lanes);//发布局部路径集合
    }
    else
      sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array",   1,    &TrajectoryGen::callbackGetGlobalPlannerPath,   this);

    visualization_msgs::MarkerArray all_rollOuts;
    PlannerHNS::ROSHelpers::TrajectoriesToMarkers(m_RollOuts, all_rollOuts);
    pub_LocalTrajectoriesRviz.publish(all_rollOuts);

    loop_rate.sleep();
  }
}
```



4.1、函数ExtractPartFromPointToDistanceDirectionFast(m_GlobalPaths.at(i), m_CurrentPos, m_PlanningParams.horizonDistance ,m_PlanningParams.pathDensity ,t_centerTrajectorySmoothed);传入参数为原始的globalPath，一组WayPoint的vector，currentPos，最小距离，路径密度，返回extractedPath

```
//传入参数为原始的globalPath，一组WayPoint的vector，currentPos，最小距离，路径密度，返回extractedPath
void PlanningHelpers::ExtractPartFromPointToDistanceDirectionFast(const vector<WayPoint>& originalPath, const WayPoint& pos, const double& minDistance,
    const double& pathDensity, vector<WayPoint>& extractedPath)
{
  if(originalPath.size() < 2 ) return;

  extractedPath.clear();
  //获取当前位置点p在路径trajectory上的索引
  int close_index = GetClosestNextPointIndexDirectionFast(originalPath, pos);
  double d = 0;

  if(close_index + 1 >= originalPath.size())
    close_index = originalPath.size() - 2;
  //从最近点向后取10米范围内的点依次填入extractedPath
  for(int i=close_index; i >=  0; i--)
  {
    extractedPath.insert(extractedPath.begin(),  originalPath.at(i));
    if(i < originalPath.size())
      d += hypot(originalPath.at(i).pos.y - originalPath.at(i+1).pos.y, originalPath.at(i).pos.x - originalPath.at(i+1).pos.x);
    if(d > 10)
      break;
  }

  //extractedPath.push_back(info.perp_point);
  //再把最近点后面minDistance范围的点依次填入extractedPath
  d = 0;
  for(int i=close_index+1; i < (int)originalPath.size(); i++)
  {
    extractedPath.push_back(originalPath.at(i));
    if(i > 0)
      d += hypot(originalPath.at(i).pos.y - originalPath.at(i-1).pos.y, originalPath.at(i).pos.x - originalPath.at(i-1).pos.x);
    if(d > minDistance)
      break;
  }

  if(extractedPath.size() < 2)
  {
    cout << endl << "### Planner Z . Extracted Rollout Path is too Small, Size = " << extractedPath.size() << endl;
    return;
  }
  //路径点按照密度进行插值
  FixPathDensity(extractedPath, pathDensity);
  //计算角度和代价(首末点的距离)
  CalcAngleAndCost(extractedPath);
}
```

4.2、GenerateRunoffTrajectory()： 针对所有的全局轨迹，生成对应的Runoff局部路径

```
// 针对所有的全局轨迹，生成对应的Runoff局部路径
//返回的m_RollOuts是rollOutsPaths，里面包含了多条local_rollOutPaths
void PlannerH::GenerateRunoffTrajectory(const std::vector<std::vector<WayPoint> >& referencePaths,const WayPoint& carPos, const bool& bEnableLaneChange, const double& speed, const double& microPlanDistance,
    const double& maxSpeed,const double& minSpeed, const double&  carTipMargin, const double& rollInMargin,
    const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
    const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
    const double& SmoothTolerance, const double& speedProfileFactor, const bool& bHeadingSmooth,
    const int& iCurrGlobalPath, const int& iCurrLocalTraj,
    std::vector<std::vector<std::vector<WayPoint> > >& rollOutsPaths,
    std::vector<WayPoint>& sampledPoints_debug)
{
  //如果referencePaths参考路径的数量不为零，且microPlanDistance最小规划路径长度不小于零，则继续生成rollOuts
  if(referencePaths.size()==0) return;
  if(microPlanDistance <=0 ) return;
  rollOutsPaths.clear();

  sampledPoints_debug.clear(); //for visualization only
  //遍历全局路径
  for(unsigned int i = 0; i < referencePaths.size(); i++)
  {
    std::vector<std::vector<WayPoint> > local_rollOutPaths;
    int s_index = 0, e_index = 0;
    vector<double> e_distances;
    //如果这组globalPath里面的第i段path中的waypoints数量大于零
    if(referencePaths.at(i).size()>0)//遍历第i条全局路径
    {
      PlanningHelpers::CalculateRollInTrajectories(carPos, speed, referencePaths.at(i), s_index, e_index, e_distances,
          local_rollOutPaths, microPlanDistance, maxSpeed, carTipMargin, rollInMargin,
          rollInSpeedFactor, pathDensity, rollOutDensity,rollOutNumber,
          SmoothDataWeight, SmoothWeight, SmoothTolerance, bHeadingSmooth, sampledPoints_debug);
    }
    else
    {
      for(int j=0; j< rollOutNumber+1; j++)
      {
        local_rollOutPaths.push_back(vector<WayPoint>());
      }
    }

    rollOutsPaths.push_back(local_rollOutPaths);
  }
}
```

```
//计算RollIn路径
void PlanningHelpers::CalculateRollInTrajectories(const WayPoint& carPos, const double& speed, const vector<WayPoint>& originalCenter, int& start_index,
    int& end_index, vector<double>& end_laterals ,
    vector<vector<WayPoint> >& rollInPaths, const double& max_roll_distance,
    const double& maxSpeed, const double&  carTipMargin, const double& rollInMargin,
    const double& rollInSpeedFactor, const double& pathDensity, const double& rollOutDensity,
    const int& rollOutNumber, const double& SmoothDataWeight, const double& SmoothWeight,
    const double& SmoothTolerance, const bool& bHeadingSmooth,
    std::vector<WayPoint>& sampledPoints)
{
  WayPoint p;
  double dummyd = 0;
  ////iLimitIndex是限制总waypoints个数的
  int iLimitIndex = (carTipMargin/0.3)/pathDensity;
  if(iLimitIndex >= originalCenter.size())
    iLimitIndex = originalCenter.size() - 1;

  //Get Closest Index
  RelativeInfo info;
  //carPos就是trajectory_generator里面传入的m_CurrentPos
  GetRelativeInfo(originalCenter, carPos, info);
  double remaining_distance = 0;
  int close_index = info.iBack;//将back点的index赋给close_index
  //遍历originalCenter里面的所有waypoints
  for(unsigned int i=close_index; i< originalCenter.size()-1; i++)
    {
    if(i>0)
      remaining_distance += distance2points(originalCenter[i].pos, originalCenter[i+1].pos);
    }
  //初始的rollIn距离就是GetRelativeInfo里面计算得到的垂直距离
  double initial_roll_in_distance = info.perp_distance ; //GetPerpDistanceToTrajectorySimple(originalCenter, carPos, close_index);


  vector<WayPoint> RollOutStratPath;


  //calculate the starting index
  double d_limit = 0;
  unsigned int far_index = close_index;//close_index，前面iBack点的index

  //calculate end index
  //初始位置由速度因子×速度+rollInMargin决定
  double start_distance = rollInSpeedFactor*speed+rollInMargin;
  ////如果初始位置已经超出了总距离，将总距离赋值给初始位置
  if(start_distance > remaining_distance)
    start_distance = remaining_distance;

  d_limit = 0;
  //从close_index开始，就是iBack点的index开始，为originalCenter中的每个点，计算和它和前一个点的distance
  for(unsigned int i=close_index; i< originalCenter.size(); i++)
    {
      if(i>0)
        d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
      //当所有路径点之间的累计距离大于初始距离时，将far_index的值设为当前这个waypoint的index，跳出循环
      if(d_limit >= start_distance)
      {
        far_index = i;
        break;
      }
    }
  //中间trajectory的index就是所有rollOut数目的一半
  int centralTrajectoryIndex = rollOutNumber/2;
  vector<double> end_distance_list;
  //遍历rollOutNumber，用rollOutDensity乘...算出来纵向上，每个rollOut的距离，距离central的距离
  for(int i=0; i< rollOutNumber+1; i++)
    {
      double end_roll_in_distance = rollOutDensity*(i - centralTrajectoryIndex);
      end_distance_list.push_back(end_roll_in_distance);
    }

  start_index = close_index;
  end_index = far_index;
  end_laterals = end_distance_list;

  //calculate the actual calculation starting index
  d_limit = 0;
  unsigned int smoothing_start_index = start_index;
  unsigned int smoothing_end_index = end_index;
  /***平滑轨迹，采用共轭梯度的方法，是一项非线性插值优化技术，同时提升曲率***/
  /**这部分应该是分段nSteps**/
  //从起点开始算一遍，从终点开始算一遍，找所有路径点累计距离大于carTipMargin的路径点的index，找到之前，每次都给flag+1
  for(unsigned int i=smoothing_start_index; i< originalCenter.size(); i++)
  {
    if(i > 0)
      d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
    if(d_limit > carTipMargin)
      break;

    smoothing_start_index++;
  }

  d_limit = 0;
  for(unsigned int i=end_index; i< originalCenter.size(); i++)
  {
    if(i > 0)
      d_limit += distance2points(originalCenter[i].pos, originalCenter[i-1].pos);
    if(d_limit > carTipMargin)
      break;

    smoothing_end_index++;
  }

  int nSteps = end_index - smoothing_start_index;


  vector<double> inc_list;
  rollInPaths.clear();
  vector<double> inc_list_inc;
  for(int i=0; i< rollOutNumber+1; i++)
  {
    double diff = end_laterals.at(i)-initial_roll_in_distance;
    inc_list.push_back(diff/(double)nSteps);
    rollInPaths.push_back(vector<WayPoint>());
    inc_list_inc.push_back(0);
  }


  ////构造空vector，里面存储waypoint的vector
  vector<vector<WayPoint> > execluded_from_smoothing;
  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
    execluded_from_smoothing.push_back(vector<WayPoint>());


  //采样
  //Insert First strait points within the tip of the car range
  for(unsigned int j = start_index; j < smoothing_start_index; j++)
  {
    p = originalCenter.at(j);//遍历初始一段路径的所有路径点
    double original_speed = p.v;
    for(unsigned int i=0; i< rollOutNumber+1 ; i++)
    {
      p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2);
      p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2);
      if(i!=centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed ;

      if(j < iLimitIndex)//如果waypoint在初始路径总路径点个数内，就不用进行平滑
        execluded_from_smoothing.at(i).push_back(p);
      else//否则加入到rollInPaths中去
        rollInPaths.at(i).push_back(p);

      sampledPoints.push_back(p);//无论是否需要平滑，都是采样的点集里面的
    }
  }

  for(unsigned int j = smoothing_start_index; j < end_index; j++)
    {
      p = originalCenter.at(j);
      double original_speed = p.v;
      for(unsigned int i=0; i< rollOutNumber+1 ; i++)
      {
        inc_list_inc[i] += inc_list[i];
        double d = inc_list_inc[i];
        //算采样点的计算方法
        p.pos.x = originalCenter.at(j).pos.x -  initial_roll_in_distance*cos(p.pos.a + M_PI_2) - d*cos(p.pos.a+ M_PI_2);
        p.pos.y = originalCenter.at(j).pos.y -  initial_roll_in_distance*sin(p.pos.a + M_PI_2) - d*sin(p.pos.a+ M_PI_2);
        if(i!=centralTrajectoryIndex)
          p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
        else
          p.v = original_speed ;

        rollInPaths.at(i).push_back(p);

        sampledPoints.push_back(p);
      }
    }

  //Insert last strait points to make better smoothing
  for(unsigned int j = end_index; j < smoothing_end_index; j++)
  {
    p = originalCenter.at(j);
    double original_speed = p.v;
    for(unsigned int i=0; i< rollOutNumber+1 ; i++)
    {
      double d = end_laterals.at(i);
      p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
      p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);
      if(i!=centralTrajectoryIndex)
        p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
      else
        p.v = original_speed ;
      rollInPaths.at(i).push_back(p);

      sampledPoints.push_back(p);
    }
  }

  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
    rollInPaths.at(i).insert(rollInPaths.at(i).begin(), execluded_from_smoothing.at(i).begin(), execluded_from_smoothing.at(i).end());


  d_limit = 0;
  for(unsigned int j = smoothing_end_index; j < originalCenter.size(); j++)
    {
    if(j > 0)
      d_limit += distance2points(originalCenter.at(j).pos, originalCenter.at(j-1).pos);

    if(d_limit > max_roll_distance)
      break;

      p = originalCenter.at(j);
      double original_speed = p.v;
      for(unsigned int i=0; i< rollInPaths.size() ; i++)
      {
        double d = end_laterals.at(i);
        p.pos.x  = originalCenter.at(j).pos.x - d*cos(p.pos.a + M_PI_2);
        p.pos.y  = originalCenter.at(j).pos.y - d*sin(p.pos.a + M_PI_2);

        if(i!=centralTrajectoryIndex)
          p.v = original_speed * LANE_CHANGE_SPEED_FACTOR;
        else
          p.v = original_speed ;

        rollInPaths.at(i).push_back(p);

        sampledPoints.push_back(p);
      }
    }

  for(unsigned int i=0; i< rollOutNumber+1 ; i++)
  {
    SmoothPath(rollInPaths.at(i), SmoothDataWeight, SmoothWeight, SmoothTolerance);
  }

//  for(unsigned int i=0; i< rollInPaths.size(); i++)
//    CalcAngleAndCost(rollInPaths.at(i));
}
```

4.3、PredictConstantTimeCostForTrajectory()







4.4、ConvertFromLocalLaneToAutowareLane()



4.5、SmoothPath function不仅在local_trajectory_generator中有使用，在global_planning_core中和其他许多个地方，都有被使用来做路径平滑，因此是一个很重要的函数

weight_data ,weight_smooth ，tolerance参数均在launch文件中设置

```
void PlanningHelpers::SmoothPath(vector<WayPoint>& path, double weight_data,
    double weight_smooth, double tolerance)
    /**头文件中定义了参数默认值static void SmoothPath(std::vector<WayPoint>& path, double weight_data =0.25,double weight_smooth = 0.25,double tolerance = 0.01)
    每次被调用时，可能都会被赋予不同的weight参数**/
{

  if (path.size() <= 2 )
  {
    //cout << "Can't Smooth Path, Path_in Size=" << path.size() << endl;
    return;
  }

  const vector<WayPoint>& path_in = path;
  vector<WayPoint> smoothPath_out =  path_in;//目前path_in和smoothPath_out里面存放的都是传入的path参数

  double change = tolerance;
  double xtemp, ytemp;
  int nIterations = 0;//只是一个计数器，不参与逻辑判断

  int size = path_in.size();//也就是path中点的总个数

  while (change >= tolerance)//change最一开始等于tolerance
  {
    change = 0.0;//change每次都被重置为零，则下一次while判断的时候，只累计了x和y坐标分别在平滑后的值和原值的差异，这个值大于等于tolerance，就会继续平滑
    for (int i = 1; i < size - 1; i++)
    {
//      if (smoothPath_out[i].pos.a != smoothPath_out[i - 1].pos.a)
//        continue;

      xtemp = smoothPath_out[i].pos.x;
      ytemp = smoothPath_out[i].pos.y;//把原来的xy的坐标都存放在临时变量中

      smoothPath_out[i].pos.x += weight_data//一阶导:a2-a1
          * (path_in[i].pos.x - smoothPath_out[i].pos.x);
      smoothPath_out[i].pos.y += weight_data
          * (path_in[i].pos.y - smoothPath_out[i].pos.y);

      smoothPath_out[i].pos.x += weight_smooth//非连续点的二阶导，Hessian matrix,(a3-a2)-(a2-a1)=a3+a1-2*a2
          * (smoothPath_out[i - 1].pos.x + smoothPath_out[i + 1].pos.x
              - (2.0 * smoothPath_out[i].pos.x));//权重0.25*(前后两个点的x坐标之和减去当前坐标的2倍)，累加到原坐标上
      smoothPath_out[i].pos.y += weight_smooth
          * (smoothPath_out[i - 1].pos.y + smoothPath_out[i + 1].pos.y
              - (2.0 * smoothPath_out[i].pos.y));

      change += fabs(xtemp - smoothPath_out[i].pos.x);
      change += fabs(ytemp - smoothPath_out[i].pos.y);

    }
    nIterations++;
  }

  path = smoothPath_out;
}

```

### 2.2、op_trajectory_evaluator节点：

实例化TrajectoryEval类，在构造函数中进行初始化，读入launch文件的参数，并定义输入输出话题

在MainLoop()函数中进行实现：

#### 1、launch文件

```
<launch>
  <!-- Trajectory Evaluation Specific Parameters -->
  
  <arg name="enablePrediction"       default="false" />       
  <arg name="velocitySource"      default="1" />         
      
  <node pkg="op_local_planner" type="op_trajectory_evaluator" name="op_trajectory_evaluator" output="screen">
  
    <param name="enablePrediction"       value="$(arg enablePrediction)" />   
    <param name="velocitySource"         value="$(arg velocitySource)" />                
      
  </node>        
      
</launch>
```

#### 2. 订阅的topic

```
sub_current_pose = nh.subscribe("/current_pose", 10, &TrajectoryEval::callbackGetCurrentPose, this);
sub_current_velocity = nh.subscribe("/current_velocity", 10, &TrajectoryEval::callbackGetVehicleStatus, this);
sub_GlobalPlannerPaths = nh.subscribe("/lane_waypoints_array", 1, &TrajectoryEval::callbackGetGlobalPlannerPath, this);   //全局路径
sub_LocalPlannerPaths = nh.subscribe("/local_trajectories", 1, &TrajectoryEval::callbackGetLocalPlannerPath, this);   //op_trajectory_generator生成的局部路径

sub_predicted_objects = nh.subscribe("/predicted_objects", 1, &TrajectoryEval::callbackGetPredictedObjects, this);
sub_current_behavior = nh.subscribe("/current_behavior", 1, &TrajectoryEval::callbackGetBehaviorState, this);
sub_mode_order = nh.subscribe("/mode_order",1,&TrajectoryEval::callbackGetModeOrder, this);
sub_inspection_point =nh.subscribe("/inspection_target",1,&TrajectoryEval::callbackGetInspecionPoint, this);
sub_scenario = nh.subscribe("scenario_manager/scenario_cmd",1,&TrajectoryEval::callbackGetScenario,this);
```

#### 3. 发布的topic

```
  pub_LocalWeightedTrajectories = nh.advertise<autoware_msgs::LaneArray>("local_weighted_trajectories", 1);
  pub_TrajectoryCost = nh.advertise<autoware_msgs::Lane>("local_trajectory_cost", 1);
```

4、MainLoop()，以100Hz的频率执行

1）、和generator中一样，先调用ExtractPartFromPointToDistanceDirectionFast()方法把路径点做一次提取。
2）、如果m_bUseMoveingObjectsPrediction参数为真，即开启检测移动障碍物，则调用DoOneStepDynamic()，否则调用DoOneStepDoOneStepStatic()，只计算静态障碍物。两个方法都是用来计算cost，返回值为BestTrajectory。该部分内容包含了清扫逻辑。最后发布到local_trajectory_cost话题













3）、调用`ConvertFromLocalLaneToAutowareLane()`方法，获取local_lanes，发布到话题`local_weighted_trajectories`





























