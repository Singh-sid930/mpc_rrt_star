#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/LaserScan.h>
#include <chrono>
#include "rrt.h"
#include <cstdlib>
#include <ctime>

//Global variable containing the most recent lidarData 
vector<float> lidarData(1081,0);
geometry_msgs::Point goal_point;
geometry_msgs::Point next_point;
float yaw = 0.0;
float maxDist = 5.0; //This is the total range of lidar data we care for. 
float scalingFactor = 10.0/maxDist; //This is the factor for how much the map should be scaled by 

// Calculates the distance between two nodes
float RRT::dist(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
		// returning int might result in bug!!!
		return sqrt(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2));
	}

// Steer random node towards the closet node based on the max allowable distance
geometry_msgs::Point RRT::steer(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
	if (dist(p1,p2) <= maxDist) return p2;
	else {
		float the = atan2((p2.y - p1.y),(p2.x - p1.x));
		geometry_msgs::Point temp_p;
		temp_p.x = p1.x + maxDist * cos(the);
		temp_p.y = p1.y + maxDist * sin(the);
		return temp_p;
	}
}

// Find if the path between two points is collision free
bool RRT::collision_free(geometry_msgs::Point& p1, geometry_msgs::Point& p2){
	// check if the path between p1 and p2 is collision free or not

	if (abs(p2.y-p1.y)<0.0001 && abs(p2.x-p1.x)<0.0001) {
		// ROS_INFO("1 false");
		return false;
	}

	else if (abs(p2.y-p1.y)<0.0001){
		float x_small = min(p1.x,p2.x);
		float x_large = max(p1.x,p2.x);
		float step = (x_large-x_small)/20.0;
		int count = 0;
		for (float i=x_small; i<x_large; i+=step){
			if (count>30) return false;
			// ROS_INFO("2 %f %f", p1.y , i);
			int y_cal = p1.y;
			int x_cal = i;
			if (occu_grid[y_cal][x_cal]==1) return false;
			count++;
		}
		return true;
	}

	else if (abs(p2.x-p1.x)<0.0001){
		float y_small = min(p2.y,p1.y);
		float y_large = max(p2.y,p1.y);
		float step = (y_large-y_small)/20.0;
		int count = 0;
		for (float i=y_small; i<y_large; i+=step){
			if (count>30) return false;
			// ROS_INFO("3 %f %f", i , p1.x);
			int y_cal = i;
			int x_cal = p1.x;
			if (occu_grid[y_cal][x_cal]==1) return false;
			count++;
		}
		return true;
	}

	else {
		float slope = (p2.y - p1.y)/(p2.x - p1.x);
		float c = p1.y - p1.x * slope;
		float x_small = min(p1.x,p2.x);
		float x_large = max(p1.x,p2.x);
		double step = (x_large-x_small)/20.0;
		int count = 0;
		for (double i=x_small; i<x_large;i+=step){
			if (count>30) return false;
			int y_cal = slope*i+c;
			int x_cal = i;
			if (occu_grid[y_cal][x_cal]==1) return false;
			count++;
		}
		return true;			
	}
}

// Check if the random node is in free space
bool RRT::_InFreeSpace(geometry_msgs::Point& p){
	if (occu_grid[p.y][p.x]==0) return true;
	else return false;
}

// Generate a random point
geometry_msgs::Point RRT::random_point(){
	geometry_msgs::Point temp_p;
	temp_p.x = (int)rand() % 200 ; //x max
	temp_p.y = (int)rand() % 100 ; // y max
	temp_p.z = 0;
	return temp_p;
}

// Find the struct containing the nearest point from the random point
int RRT::findNearestStruct(geometry_msgs::Point &p){
	
	//start with the root node of the tree	
	geometry_msgs::Point n = structVect[0].node;
	int nearest_idx = 0;
	
	for (int i=0; i<structVect.size();i++){
		if (dist(p,structVect[i].node) < dist(p,n)) {
			n = structVect[i].node;	
			nearest_idx = i;
		}
	}

	return nearest_idx;
}

RRT::RRT() : MAX_DIST(5) {

	pub_marker = n_ros.advertise<visualization_msgs::Marker>("viz_marker",10);

	points.header.frame_id = line_list_final.header.frame_id = line_list.header.frame_id = "/my_frame";
	points.header.stamp = line_list_final.header.stamp = line_list.header.stamp = ros::Time::now();
	points.ns = line_list_final.ns = line_list.ns = "points_and_lines";
	points.action = line_list_final.action = line_list.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_list_final.pose.orientation.w = line_list.pose.orientation.w = 1.0;

	points.type = visualization_msgs::Marker::POINTS;

	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.r = 1.0f;
	points.color.a = 1.0;

	line_list.scale.x = 0.1;
	line_list.id = 2;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.r = 1.0;
	line_list.color.a = 0.5;

	line_list_final.scale.x = 0.1;
	line_list_final.id = 2;
	line_list_final.type = visualization_msgs::Marker::LINE_LIST;
	line_list_final.color.b = 1.0;
	line_list_final.color.a = 1.0;	

}

void RRT::plan_it(geometry_msgs::Point &p1, geometry_msgs::Point &p2, vector<vector<int>> &traj){

	//Create starting nodeStruct and add
	struct treeNode starting;
	starting.node = p1;
	starting.parent_idx = -1;

	structVect.push_back(starting);

	bool pathFound = false;

	//As a first pass, see if we can connect directly to the end node with no obstacles in the way. 
	// If so, create a path directly to it and return that, skipping the tree altogether. 
	
	if(collision_free(p1,p2)) {
		struct treeNode endStruct;
		endStruct.node = p2;
		endStruct.parent_idx = 0; //the parent is the starting node
		
		structVect.push_back(endStruct);

		pathFound = true;
	}

	geometry_msgs::Point p_start, p_end;

	//If there wasn't a direct path, check whether the final point is within free space first, if not. Move it laterally
	if(!pathFound && occu_grid[p2.y][p2.x] == 1) { 
		bool alternateFound = false;
		
		//search left and right, and return the point in freespace which is closest
		for(int i = 1; i < 199 ; i++) {
			if((p2.x - i) > 0 && occu_grid[p2.y][p2.x - i] == 0) {
				p_end.x = p2.x - i; 
				p_end.y = p2.y;
				alternateFound = true;
				break;
			}

			if((p2.x + i) < 199 && occu_grid[p2.y][p2.x + i] == 0) {
				p_end.x = p2.x + i; 
				p_end.y = p2.y;
				alternateFound = true;
				break;
			}
		}
		
		if(!alternateFound) {
			for(int i = 1; i < 99 ; i++) {
				if((p2.y + i) < 99 && occu_grid[p2.y + i][p2.x] == 0) {
					p_end.x = p2.x;
					p_end.y = p2.y + i; 
					break;
				}
			}
		}
	}
	//If there's no problem with the endpoint or if the path was found
	else{
		p_start.x = p1.x;
		p_start.y = p1.y;
		p_end.x = p2.x;
		p_end.y = p2.y;
	}
	
	const int N = 10000; //max number of iterations in the search 

	for (int i=0; i<N; i++){
		
		//once the path has been found, the shortest path must be generated  		
		if(pathFound) { 
			break; 
		}

		geometry_msgs::Point rd = random_point();

		if(_InFreeSpace(rd)) {
	
			int nearestStruct_idx = findNearestStruct(rd);

			//cout<<"nearest struct: "<< structVect[nearestStruct_idx].node.x <<" "<< structVect[nearestStruct_idx].node.y <<endl;
			
			//limit the size of the node connection
			rd = steer(structVect[nearestStruct_idx].node,rd);
			
			if(collision_free(structVect[nearestStruct_idx].node,rd)) {
				
				
				//This point should now be added to the structVector
				treeNode newStruct; 
				newStruct.node = rd;
				newStruct.parent_idx = nearestStruct_idx;

				structVect.push_back(newStruct);
		
				//Check if the new point is within the max dist of the endpoint
				if (dist(rd,p_end) < maxDist) {

					//now check if there is a collision free path to the endpoint from this random point
					if (collision_free(rd,p_end)) {
					
						//Create a new struct representing the endNode
						struct treeNode endStruct;
						endStruct.node = p_end;
						endStruct.parent_idx = nearestStruct_idx; //the parent is the struct containing the random node
						
						structVect.push_back(endStruct);

						pathFound = true;
						//cout << "Path found." << endl;
					}					
				}
			}
		
		} // end if(freespace)
	} // end for loop
	
	if(pathFound) {
		treeNode currNode = structVect[structVect.size() - 1]; //endNode
		line_list_final.points.push_back(currNode.node);
		
		while(currNode.parent_idx != -1) {
			vector<int> newCoords;
			newCoords.push_back(currNode.node.x);
			newCoords.push_back(currNode.node.y);
			traj.push_back(newCoords);
			currNode = structVect[currNode.parent_idx];
			line_list_final.points.push_back(currNode.node);
			line_list_final.points.push_back(currNode.node);
		}

		currNode = structVect[0]; // startNode
		line_list_final.points.push_back(currNode.node);	

		vector<int> newCoords;
		newCoords.push_back(currNode.node.x);
		newCoords.push_back(currNode.node.y);
		traj.push_back(newCoords);
	}

}// end plan_it

void RRT::markObstacle(){
	for (int i=0;i<occu_grid.size();i++){
		for (int j=0;j<occu_grid[0].size();j++){
			if (occu_grid[i][j]==1){
				geometry_msgs::Point p;
				p.x = j;
				p.y = i;
				p.z = 0;
				points.points.push_back(p);
			}
		}
	}
}

void RRT::publish_it() {
	pub_marker.publish(points);
	pub_marker.publish(line_list_final);
}

void RRT::vec_delete(){
	line_list_final.points.clear();
	line_list.points.clear();
	structVect.clear();
	points.points.clear();
}

void RRT::buildOccuGrid(vector<float> &lidarScan) {

	//default range is 10 meters, so this gives each grid a resolution of 0.1 meters x 0.1 meters with no scaling

	//An occupancy grid object that shows the space around the car. This is not the member variable
	vector<vector<int>> occugrid {100, vector<int>(200,1)};

	//The original FOV of the lidar is 270 degrees
	//We only care about the 180 degrees in front of the car, so toss out the first 1/6 of data and the last 1/6 of data
	//instead of creating a new data structure to handle this, we can just look at the points from index 180 to 900

	//The car can be assumed to be at the position [100],[100]. Ray trace from this position to set occupancy grid data
	//For the lidar, step 0 is on the left and step 1079 is on the right

	float degree = 0.0;
	for(int i = 180; i < 901; i++) {
		float furthestPoint = lidarScan[i];
		if(furthestPoint > maxDist) { furthestPoint = maxDist; }

		//Now march from the car to the scanned point by dividing the furthest point by 100
		if (furthestPoint== 0.0) continue;
		for(float step = 0.0; step <= furthestPoint; step += furthestPoint/100.0) {
		    float xValue = 99.0 + step * 10.0 * scalingFactor * cosf(M_PI/180.0 * degree);
		    float yValue = 99.0 - step *10.0 * scalingFactor *  sinf(M_PI/180.0 * degree);

		    //cast the value to int to get the relevant coordinate on the occupancy grid
		    int xCoord = (int) xValue;
		    int yCoord = (int) yValue;

		    if(occugrid[yCoord][xCoord] == 1) {
			occugrid[yCoord][xCoord] = 0;
		    }
		}

		degree += 0.25;
	}

	vector<vector<int>> inflated_grid {100, vector<int>(200,0)};

	//Inflate the grid by making all of the 1s' neighbors 1s as well
	for(int i = 1; i < 99; i++) {
		for(int j = 1; j < 199; j++) {
		    if (occugrid[i][j] == 1) {
			for(int k = -1; k <= 1; k++){
			    for(int l = -1; l <= 1; l++) {
				inflated_grid[i+k][j+l] = 1;
			    }
			}
		    }
		}
	}
	//set the RRT's member variable to be the inflated grid
	occu_grid = inflated_grid;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
   for(int i=0;i<1081;i++) {   
		if(isinf(scan->ranges[i])) { lidarData[i] = 10.0; }
		else { 
			lidarData[i] = scan->ranges[i];
	 	}
    }
}

void goalCallback(const geometry_msgs::Point::ConstPtr& data)
{
   //Convert the goal point from the car frame to the occupancy grid frame
   //In the car frame, the y axis is positive to the left, and the x axis is positive up
   // (2,3) in car frame is (69,79) in occupancy grid frame at a scaling factor of 1, where 10 m is represented by 100 tiles
   // (2,3) becomes 20,30 and then axes flip. If we set our max distance to be 5, these points would be (40,60) before axes flipping
   // (2,3) becomes (39,59) on the grid if the max distance is 5 

   float potentialX = 99 - (data->y * 10 * scalingFactor);
   float potentialY = 99 - (data->x * 10 * scalingFactor);

   //check to make sure the new point would not cause a seg fault
   if(potentialX >= 0 && potentialX <= 199 && potentialY >= 0 && potentialY <= 99) {
	goal_point.x = potentialX;
   	goal_point.y = potentialY;
	cout << "data in: " << data->x << "," << data->y << endl;
	cout << "occu goal" << goal_point.x << "," << goal_point.y << endl;
   	yaw = data->z;
    }  
}

int main(int argc, char * argv[]) {

	//seed the random number generator
	srand(time(0));

	ros::init(argc, argv, "rrt_points_lines");	
	
	//subsriber for the laser ranges
	ros::NodeHandle n;
	ros::Subscriber subs = n.subscribe("/scan", 10, scanCallback);
	ros::Subscriber subs_goal = n.subscribe("/waypoint/goal", 10, goalCallback);
	ros::Publisher pub_next = n.advertise<geometry_msgs::Point>("/waypoint/next", 100);

  	ros::Rate r(10);

	RRT rrt;

	geometry_msgs::Point P1,P2;

	//P1 is always the car's origin position in the occupancy grid frame
	P1.x = 99;
	P1.y = 99;

	while (ros::ok()){	
	
		//used for profiling the RRT code
		//auto start = std::chrono::high_resolution_clock::now();

		//build the occupancy grid based on the latest lidar data
		rrt.buildOccuGrid(lidarData);

		rrt.markObstacle();

		//Create a vector of int arrays that represent the x,y coordinates of the local path 
		vector<vector<int>> trajectory; 
		
		P2.x = (int) goal_point.x;
		P2.y = (int) goal_point.y;

		rrt.plan_it(P1,P2,trajectory);

		//auto finish = std::chrono::high_resolution_clock::now();
		//std::chrono::duration<double> total_elapsed = finish - start;
		//ROS_INFO("Elapsed time: %f", total_elapsed.count());

		if (!trajectory.empty()){
	 	   //Convert the goal point from the occupancy grid frame to the car frame
		   //In the car frame, the y axis is positive to the left, and the x axis is positive up
		   // (2,3) in car frame is (69,79) in occupancy grid frame at a scaling factor of 1, where 10 m is represented by 100 tiles
		   // (2,3) becomes 20,30 and then axes flip. If we set our max distance to be 5, these points would be (40,60) before flipping
		   // (2,3) becomes (39,59) on the grid if the max distance is 5 
			next_point.x = (99 - trajectory[(int) (trajectory.size()-1)/2][1])/(10.0 * scalingFactor); 
			next_point.y = (99 - trajectory[(int) (trajectory.size()-1)/2][0])/(10.0 * scalingFactor);
			next_point.z = yaw;
		}

		pub_next.publish(next_point);

		rrt.publish_it();
		ros::spinOnce(); //Ensures all the callbacks are called, this will update the lidar data with the most recent scan
		// r.sleep();
		rrt.vec_delete();
	}

	return 0;
}
