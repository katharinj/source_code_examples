/*
Katharin Jensen
This code takes in the coordinates of the vertices of a polygon over the ros topic /coordinate_list, creates a grid on the polygon, and creates a spiral path out of the grid, returning the path over the ros topic /pathpoints.
*/

#define _USE_MATH_DEFINES //gives me M_PI for pi

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <ros/console.h>
#include <test_quad/Coordinate.h>
#include <test_quad/Coordinate_List.h>

struct Shape {
	//A polygon with n_vert vertices
	int n_vert; //number of vertices
	std::vector<double> V_x; //x locations of the vertices
	std::vector<double> V_y; //y locations of the vertices
};

struct GridNode {
	//A node in the grid
	int n_node; //the number of the node
	double x; //the x-coordinate of the node
	double y; // the y-coordinate of the node

	//the numbers of the adjacent nodes. Note: -1 means there is no node in that direction
	int n_left = -1; //left
	int n_right = -1; //right
	int n_up = -1; //up
	int n_down = -1; //down

	//the value/weight assigned to the node for path planning purposes. Note: -1 means no weight assigned
	int node_val = -1;
};


//ros subscriber function to read coordinates

std::vector<double> XFromGUI(2, 1.0);
std::vector<double> YFromGUI(2, 1.0);
double resFromGUI = 0.1;

void readCoordinates(const test_quad::Coordinate_List coordlist) {
	std::vector<double> tmpX;
	std::vector<double> tmpY;
	for (int i = 0; i < coordlist.list.size(); i++) {
		tmpX.push_back(coordlist.list[i].x);
		tmpY.push_back(coordlist.list[i].y);
	}
	XFromGUI = tmpX;
	YFromGUI = tmpY;
	resFromGUI = coordlist.resolution;
}

std::vector<double> GetGridPoints(Shape S, double x_corner, double y_corner, double res);
std::vector<GridNode> ConnectGrid(std::vector<double> gridx, std::vector<double> gridy, double res);
std::vector<int> SpiralPath(std::vector<GridNode> digraph);
int max(std::vector<double> v);
int min(std::vector<double> v);
int min_int(std::vector<int> v);
int max_int(std::vector<int> v);
int LoopAdd(int i_vert, int a, int n_vert);
bool IsIn(int a, std::vector<int> v);

//main function
int main(int argc, char **argv) {
	//initialize ROS stuff
	ros::init(argc, argv, "PathPlanning");
	ros::NodeHandle n;
	ROS_DEBUG("Init Successful");

	//get the area points from the GUI

	//set up a ROS subscriber
	ros::Subscriber sub = n.subscribe("/coordinate_list", 1, readCoordinates);
	//set up a ROS publisher
	ros::Publisher pathpub = n.advertise<test_quad::Coordinate_List>("/pathpoints", 1);

	ros::Rate loop_rate(1);
	ROS_DEBUG("Sub & Pub Setup Successful");
	//save the old shape from the GUI
	std::vector<double> OldXFromGUI(2, 1.0);
	std::vector<double> OldYFromGUI(2, 1.0);
	ROS_DEBUG("Shape Intialization Successful");

	while (ros::ok()) {
		//only run the path planning if there's a new shape from the GUI
		if (OldXFromGUI != XFromGUI || OldYFromGUI != YFromGUI) {
			//initialize
			int n_corners = XFromGUI.size(); //number of corners
			std::vector<double> A_x = XFromGUI; //x coordinates
			std::vector<double> A_y = YFromGUI; //y coordinates

			double res = resFromGUI; //grid square size/resolution

			// math to find the grid space locations

			//Set up the box
			Shape Box;

			Box.n_vert = n_corners;
			Box.V_x = A_x;
			Box.V_y = A_y;

			//find the top corner
			//find the topmost and leftmost vertices
			int I_l = min(Box.V_x);
			int I_t = max(Box.V_y);

			double x_corner = Box.V_x[I_l] - std::fmod(Box.V_x[I_l], res) - res;
			double y_corner = Box.V_y[I_t] + std::fmod(Box.V_y[I_t], res);

			//get the grid points
			std::vector<double> gridpts_x;
			std::vector<double> gridpts_y;

			//get the grid for the box
			std::vector<double> gridpts_S = GetGridPoints(Box, x_corner, y_corner, res);
			//put the gridpoints into the appropriate vectors
			for (int j = 0; j < gridpts_S.size(); j++) {
				if (j % 2 == 0) {
					gridpts_x.push_back(gridpts_S[j]);
				}
				else {
					gridpts_y.push_back(gridpts_S[j]);
				}
			}


			//set up the node connections
			std::vector<GridNode> digraph = ConnectGrid(gridpts_x, gridpts_y, res);


			//create the path
			std::vector<double> waypoints_x;
			std::vector<double> waypoints_y;

			std::vector<int> path_nodes = SpiralPath(digraph);
			
			
			//DO NOT UNCOMMENT THE BLOCK OF CODE BELOW UNTIL A CONVERSION BETWEEN GPS COORDINATES AND ACTUAL DISTANCE IS PUT IN SOMEWHERE
			/*//get modified path based on path energy
			double energy_tot = 2 * 5700; //total battery power (mAh)
			double path_energy = 0; //energy the path has used

			std::vector<int> path_mod;
			path_mod.push_back(path_nodes[0]);
			for (int i = 1; i < path_nodes.size(); i++) {
				//get energy traveling from the previous node to this node
				double travel_energy = GetEnergyTravel(digraph[path_nodes[i - 1]], digraph[path_nodes[i]]);
				//get energy spent at this node
				double node_energy = 0;
				if (i < path_nodes.size() - 1) {
					node_energy = GetEnergyNode(digraph[path_nodes[i - 1]], digraph[path_nodes[i]], digraph[path_nodes[i + 1]]);
				}
				path_energy += node_energy + travel_energy;
				if (energy_tot - path_energy < 0.1*energy_tot) {
					break;
				}
				path_mod.push_back(path_nodes[i]);
			}

			for (int i = 0; i < path_mod.size(); i++) {
				waypoints_x.push_back(digraph[path_mod[i]].x);
				waypoints_y.push_back(digraph[path_mod[i]].y);
			}

			//get total distance from all unvisited nodes to the closest visited node
			double disttot = DistEval(digraph, path_mod);*/


			//--------------------output----------------------------
			//put together the coordinate list
			test_quad::Coordinate_List PathList;
			PathList.resolution = static_cast<float>(n_corners); //arbitrary output resolution value
			for (int i = 0; i < path_nodes.size(); i++) {
				test_quad::Coordinate outcoord;
				outcoord.x = digraph[path_nodes[i]].x;
				outcoord.y = digraph[path_nodes[i]].y;
				PathList.list.push_back(outcoord);
			}
			pathpub.publish(PathList);
		}

		//save the previous shape to make the path planning only run if a new shape is given
		OldXFromGUI = XFromGUI;
		OldYFromGUI = YFromGUI;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

int min(std::vector<double> v) {
	//find the index of the minimum value of a vector
	//input: the vector
	//output: the index of the minimum

	//get the length of the vector
	int l = static_cast<int>(v.size());

	//get the first element
	double min = v[0];
	int i_min = 0;

	//loop through the vector elements
	for (int i = 1; i < l; i++) {
		//check if there's a new min
		if (v[i] < min) {
			min = v[i];
			i_min = i;
		}
	}

	return i_min;
}

int min_int(std::vector<int> v) {
	//find the index of the minimum value of a vector
	//input: the vector
	//output: the index of the minimum

	//get the length of the vector
	int l = static_cast<int>(v.size());

	//get the first element
	int min = v[0];
	int i_min = 0;

	//loop through the vector elements
	for (int i = 1; i < l; i++) {
		//check if there's a new min
		if (v[i] < min) {
			min = v[i];
			i_min = i;
		}
	}

	return i_min;
}

int max(std::vector<double> v) {
	//find the index of the maximum value of a vector
	//input: the vector
	//output: the index of the maximum

	//get the length of the vector
	int l = static_cast<int>(v.size());

	//get the first element
	double max = v[0];
	int i_max = 0;

	//loop through the vector elements
	for (int i = 1; i < l; i++) {
		//check if there's a new min
		if (v[i] > max) {
			max = v[i];
			i_max = i;
		}
	}

	return i_max;
}

int max_int(std::vector<int> v) {
	//find the index of the maximum value of a vector
	//input: the vector
	//output: the index of the maximum

	//get the length of the vector
	int l = static_cast<int>(v.size());

	//get the first element
	int max = v[0];
	int i_max = 0;

	//loop through the vector elements
	for (int i = 1; i < l; i++) {
		//check if there's a new min
		if (v[i] > max) {
			max = v[i];
			i_max = i;
		}
	}

	return i_max;
}

int LoopAdd(int i_vert, int a, int n_vert) {
	//move forward (or backward) a certain number of vertices, looping back to 0 (or the max vertex number) if necessary
	//inputs: i_vert, the current vertex; n_vert, the number of vertices in the shape; a, the number to add
	//output: the vertex reached through the addition

	int j_vert = i_vert + a; //the vertex reached through addition

							 //check if the number is over the largest vertex number (n_vert-1)
	if (j_vert >= n_vert) {
		//get new j_vert value
		j_vert -= n_vert;
	}
	//check if the number is negative
	if (j_vert < 0) {
		//get new j_vert value
		j_vert += n_vert;
	}

	return j_vert;
}

bool IsIn(int a, std::vector<int> v) {
	//check if a vector contains a specific value
	//inputs: a, number to check; v, vector to see if a is in
	//output: bool for if a is in v

	bool isin = false; //whether a is in v

	for (int i = 0; i < v.size(); i++) {
		if (a == v[i]) {
			isin = true;
		}
	}
	return isin;
}

std::vector<double> GetGridPoints(Shape S, double x_corner, double y_corner, double res) {
	//creates a list of all of the centers of the grid squares that are in a convex shape
	//inputs: S, the shape to put a grid on;  x_corner and y_corner, the coordinates of the top-leftmost point to start checking; res, the grid square size
	//output: a list of all of the grid points, alternating x and y values

	//find the extreme points
	int i_l = min(S.V_x); //left
	int i_r = max(S.V_x); //right
	int i_b = min(S.V_y); //bottom

						  //find the slopes from the leftmost point to its adjacent points
	bool inf_slope1 = false; //if slope 1 is infinite
	bool inf_slope2 = false; //if slope 2 is infinite
	double slope1 = (S.V_y[i_l] - S.V_y[LoopAdd(i_l, -1, S.n_vert)]) / (S.V_x[i_l] - S.V_x[LoopAdd(i_l, -1, S.n_vert)]);
	if (std::isinf(slope1)) {
		slope1 = (S.V_y[LoopAdd(i_l, -2, S.n_vert)] - S.V_y[LoopAdd(i_l, -1, S.n_vert)]) / (S.V_x[LoopAdd(i_l, -2, S.n_vert)] - S.V_x[LoopAdd(i_l, -1, S.n_vert)]);
		inf_slope1 = true;
	}
	double slope2 = (S.V_y[i_l] - S.V_y[LoopAdd(i_l, 1, S.n_vert)]) / (S.V_x[i_l] - S.V_x[LoopAdd(i_l, 1, S.n_vert)]);
	if (std::isinf(slope2)) {
		slope2 = (S.V_y[LoopAdd(i_l, 1, S.n_vert)] - S.V_y[LoopAdd(i_l, 2, S.n_vert)]) / (S.V_x[LoopAdd(i_l, 1, S.n_vert)] - S.V_x[LoopAdd(i_l, 2, S.n_vert)]);
		inf_slope2 = true;
	}

	//keep track of the vertices the x-value is between
	int top_vert = i_l;
	int bot_vert = i_l;

	//figure out which is the top and which is the bottom, get the direction to go in each case
	int dir = 0;
	double top_slope = 0;
	double bot_slope = 0;

	if (!inf_slope1 && !inf_slope2) {
		if (slope1 > slope2) {
			//going CCW
			dir = -1;
			top_slope = slope1;
			bot_slope = slope2;
		}
		else {
			//going CW
			dir = 1;
			top_slope = slope2;
			bot_slope = slope1;
		}
	}
	else {
		if (inf_slope1) {
			if (S.V_y[i_l] < S.V_y[LoopAdd(i_l, -1, S.n_vert)]) {
				//going CCW
				dir = -1;
				top_slope = slope1;
				bot_slope = slope2;
				top_vert = LoopAdd(top_vert, dir, S.n_vert);
			}
			else {
				//going CW
				dir = 1;
				top_slope = slope2;
				bot_slope = slope1;
				bot_vert = LoopAdd(bot_vert, -dir, S.n_vert);
			}
		}
		if (inf_slope2) {
			if (S.V_y[i_l] > S.V_y[LoopAdd(i_l, 1, S.n_vert)]) {
				//going CCW
				dir = -1;
				top_slope = slope1;
				bot_slope = slope2;
				bot_vert = LoopAdd(bot_vert, -dir, S.n_vert);
			}
			else {
				//going CW
				dir = 1;
				top_slope = slope2;
				bot_slope = slope1;
				top_vert = LoopAdd(top_vert, dir, S.n_vert);
			}
		}
	}

	//find the y-intercepts of the lines for the two sides
	double top_b = S.V_y[top_vert] - top_slope*S.V_x[top_vert];
	double bot_b = S.V_y[bot_vert] - bot_slope*S.V_x[bot_vert];

	//keep track of the current x-location, starting at the left
	double x_current = x_corner;
	//get inside the shape
	while (x_current < S.V_x[i_l]) {
		x_current += res;
	}

	//vector containing the grid points
	std::vector<double> gridpoints;

	//loop through the x-values
	while (x_current <= S.V_x[i_r]) {

		//keep track of the current y-location, starting at the top
		double y_current = y_corner;

		//if the x-value has passed a vertex, calculate slope for the next side
		if (x_current > S.V_x[LoopAdd(top_vert, dir, S.n_vert)]) {
			//new top slope
			top_vert = LoopAdd(top_vert, dir, S.n_vert);
			top_slope = (S.V_y[top_vert] - S.V_y[LoopAdd(top_vert, dir, S.n_vert)]) / (S.V_x[top_vert] - S.V_x[LoopAdd(top_vert, dir, S.n_vert)]);
			top_b = S.V_y[top_vert] - top_slope*S.V_x[top_vert];
		}
		if (x_current > S.V_x[LoopAdd(bot_vert, -dir, S.n_vert)]) {
			//new bottom slope
			bot_vert = LoopAdd(bot_vert, -dir, S.n_vert);
			bot_slope = (S.V_y[bot_vert] - S.V_y[LoopAdd(bot_vert, -dir, S.n_vert)]) / (S.V_x[bot_vert] - S.V_x[LoopAdd(bot_vert, -dir, S.n_vert)]);
			bot_b = S.V_y[bot_vert] - bot_slope*S.V_x[bot_vert];
		}

		//loop through the y-values and see if they're in the shape
		while (y_current >= S.V_y[i_b]) {
			//check if the point is below the top side and above the bottom side
			double check_topb = y_current - top_slope*x_current;
			double check_botb = y_current - bot_slope*x_current;

			if (check_topb <= top_b && check_botb >= bot_b) {
				gridpoints.push_back(x_current);
				gridpoints.push_back(y_current);
			}

			//get next y-value
			y_current -= res;
		}

		//get next x-value
		x_current += res;

	}
	return gridpoints;
}

std::vector<GridNode> ConnectGrid(std::vector<double> gridx, std::vector<double> gridy, double res) {
	//connects the grid points to the adjacent grid points
	//inputs: gridx and gridy, vectors of the x-y coordinates of the grid points; res, the grid space size
	//outputs: a vector of the grid nodes with the associated connections

	std::vector<GridNode> digraph;

	for (int i = 0; i < gridx.size(); i++) {
		//create a node
		GridNode node;
		node.x = gridx[i];
		node.y = gridy[i];
		node.n_node = i;
		//find connections to other nodes
		for (int j = 0; j < digraph.size(); j++) {
			//if the other node is to the left of this node
			if (digraph[j].x + res == node.x && digraph[j].y == node.y) {
				node.n_left = j;
				digraph[j].n_right = i;
			}
			//if the other node is to the right of this node
			if (digraph[j].x - res == node.x && digraph[j].y == node.y) {
				node.n_right = j;
				digraph[j].n_left = i;
			}
			//if the other node is above this node
			if (digraph[j].y - res == node.y && digraph[j].x == node.x) {
				node.n_up = j;
				digraph[j].n_down = i;
			}
			//if the other node is below this node
			if (digraph[j].y + res == node.y && digraph[j].x == node.x) {
				node.n_down = j;
				digraph[j].n_up = i;
			}
		}
		digraph.push_back(node);
	}

	return digraph;
}

std::vector<int> SpiralPath(std::vector<GridNode> digraph) {
	//creates a spiral path 
	//inputs: digraph, directed graph of grid nodes
	//output: a list of the grid nodes to visit, in order

	//create the list
	std::vector<int> path;

	//give each node a number based on its distance from the edge
	bool all_valued = false; //whether each node has been given a distance value
	int dist = 0; //distance value to apply to the nodes

	while (all_valued == false) {
		//go through all of the nodes
		for (int i = 0; i < digraph.size(); i++) {
			//for finding the edge nodes
			if (dist == 0) {
				//check if any of the node connections are -1 (signifies an edge node)
				if (digraph[i].n_down == -1 || digraph[i].n_up == -1 || digraph[i].n_left == -1 || digraph[i].n_right == -1) {
					digraph[i].node_val = dist;
				}
			}
			else {
				//if the node has not already been assigned a value, check if any of the adjoining nodes have 1 lower dist value
				if (digraph[i].node_val == -1 && (digraph[digraph[i].n_up].node_val == dist - 1 || digraph[digraph[i].n_down].node_val == dist - 1 || digraph[digraph[i].n_left].node_val == dist - 1 || digraph[digraph[i].n_right].node_val == dist - 1)) {
					digraph[i].node_val = dist;
				}
			}
		}
		//check whether all of the nodes have been given values
		all_valued = true;
		for (int i = 0; i < digraph.size(); i++) {
			if (digraph[i].node_val == -1) {
				all_valued = false;
			}
		}
		dist++;
	}

	//find a starting node
	GridNode currentnode;
	for (int i = 0; i < digraph.size(); i++) {
		if (digraph[i].node_val == 0) {
			currentnode = digraph[i];
			break;
		}
	}
	//add the node to the path
	path.push_back(currentnode.n_node);

	//save previous node
	GridNode prevnode;

	//set previous direction values (note: 0 = left, 1 = right, 2 = up, 3 = down, -1 = initialization value)
	int prev_dir1 = -1;
	int prev_dir2 = -1;

	//as long as the path does not contain all of the nodes
	//int current_val = 0; //the value of the nodes being visited
	while (path.size() < digraph.size()) {
		prevnode = currentnode;
		//find the lowest value of the adjacent nodes to the current node that is not already in the path
		std::vector<int> adjacentvals;
		std::vector<int> adjacentnodes;
		//std::vector<int> nodedirs;
		if (!IsIn(currentnode.n_left, path) && currentnode.n_left >= 0) {
			adjacentvals.push_back(digraph[currentnode.n_left].node_val); //left
			adjacentnodes.push_back(currentnode.n_left);
			//nodedirs.push_back(1);
		}
		if (!IsIn(currentnode.n_right, path) && currentnode.n_right >= 0) {
			adjacentvals.push_back(digraph[currentnode.n_right].node_val); //right
			adjacentnodes.push_back(currentnode.n_right);
			//nodedirs.push_back(2);
		}
		if (!IsIn(currentnode.n_up, path) && currentnode.n_up >= 0) {
			adjacentvals.push_back(digraph[currentnode.n_up].node_val); //up
			adjacentnodes.push_back(currentnode.n_up);
			//nodedirs.push_back(3);
		}
		if (!IsIn(currentnode.n_down, path) && currentnode.n_down >= 0) {
			adjacentvals.push_back(digraph[currentnode.n_down].node_val); //down
			adjacentnodes.push_back(currentnode.n_down);
			//nodedirs.push_back(4);
		}
		//if none of the nodes are in the surrounding path, jump to the closest node not currently on the path
		if (adjacentvals.empty()) {
			//initialize the node with the minimum distance
			int minnode = -1;
			double minnode_dist = 0;
			for (int i = 0; i < digraph.size(); i++) {
				if (!IsIn(digraph[i].n_node, path)) {
					minnode = i;
					double x = digraph[i].x - currentnode.x;
					double y = digraph[i].y - currentnode.y;
					minnode_dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
					break;
				}
			}
			//loop through all nodes not on the path
			for (int i = 0; i < digraph.size(); i++) {
				if (!IsIn(digraph[i].n_node, path)) {
					double x = digraph[i].x - currentnode.x;
					double y = digraph[i].y - currentnode.y;
					double node_dist = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
					if (node_dist < minnode_dist) {
						minnode = i;
						minnode_dist = node_dist;
					}
				}
			}
			currentnode = digraph[minnode];
			path.push_back(currentnode.n_node);
			//break;
		}
		//otherwise find the minimum valued node in the best direction and go to it, and add it to the path
		else {
			//find the node with the highest directional priority
			std::vector<int> adj_node_vals; //index 0 = left, index 1 = right, index 2 = up, index 3 = down
			std::vector<GridNode> adj_nodes; //vector of adjacent nodes
											 //fill the node vector
											 //if an adjacent node is not in the path, or is a nonexistent node, add a dummy node instead
			GridNode dummy;
			if (!IsIn(currentnode.n_left, path) && currentnode.n_left != -1) {
				adj_nodes.push_back(digraph[currentnode.n_left]);
			}
			else {
				adj_nodes.push_back(dummy);
			}
			if (!IsIn(currentnode.n_right, path) && currentnode.n_right != -1) {
				adj_nodes.push_back(digraph[currentnode.n_right]);
			}
			else {
				adj_nodes.push_back(dummy);
			}
			if (!IsIn(currentnode.n_up, path) && currentnode.n_up != -1) {
				adj_nodes.push_back(digraph[currentnode.n_up]);
			}
			else {
				adj_nodes.push_back(dummy);
			}
			if (!IsIn(currentnode.n_down, path) && currentnode.n_down != -1) {
				adj_nodes.push_back(digraph[currentnode.n_down]);
			}
			else {
				adj_nodes.push_back(dummy);
			}
			//make a vector of the node values
			for (int i = 0; i < 4; i++) {
				//if an adjacent node has a value of -1, save the value instead to be more than the max distance so it doesn't count
				//(to keep indexing correct)
				if (adj_nodes[i].node_val == -1) {
					adj_node_vals.push_back(dist + 1);
				}
				else {
					adj_node_vals.push_back(adj_nodes[i].node_val);
				}
			}

			//find all of the adjacent nodes with minimum values
			std::vector<int> minnodes; //vector of adj_nodes indeces containing nodes with minimum values
									   //find the first node with a minimum value
			int minval = min_int(adj_node_vals);
			minnodes.push_back(minval);
			//find any other nodes with minimum values
			for (int i = 0; i < 4; i++) {
				if (adj_node_vals[minval] == adj_node_vals[i] && minval != i) {
					minnodes.push_back(i);
				}
			}
			//if there's only one minimum valued node
			if (minnodes.size() == 1) {
				currentnode = adj_nodes[minnodes[0]];
			}
			//otherwise, find the minimum valued node with the smallest sum of adjacent node values
			else {
				//find the sums of all of the adjacent node values
				std::vector<int> nodesums;
				for (int i = 0; i < minnodes.size(); i++) {
					int left0;
					int right0;
					int up0;
					int down0;
					if (adj_nodes[minnodes[i]].n_left == -1) {
						left0 = -1;
					}
					else {
						left0 = digraph[adj_nodes[minnodes[i]].n_left].node_val;
					}
					if (adj_nodes[minnodes[i]].n_right == -1) {
						right0 = -1;
					}
					else {
						right0 = digraph[adj_nodes[minnodes[i]].n_right].node_val;
					}
					if (adj_nodes[minnodes[i]].n_up == -1) {
						up0 = -1;
					}
					else {
						up0 = digraph[adj_nodes[minnodes[i]].n_up].node_val;
					}
					if (adj_nodes[minnodes[i]].n_down == -1) {
						down0 = -1;
					}
					else {
						down0 = digraph[adj_nodes[minnodes[i]].n_down].node_val;
					}
					nodesums.push_back(left0 + right0 + up0 + down0);
				}
				//find the minimum node sum
				std::vector<int> minnodes2;
				int minval2 = min_int(nodesums);
				minnodes2.push_back(minnodes[minval2]);
				//see if the other sum(s) are also minimum values
				for (int i = 1; i < nodesums.size(); i++) {
					if (nodesums[i] == nodesums[minval2] && i != minval2) {
						minnodes2.push_back(minnodes[i]);
					}
				}

				//if there's only one node with a minimum sum, that's the next node
				if (minnodes2.size() == 1) {
					currentnode = adj_nodes[minnodes2[0]];
				}
				//otherwise use directional priority
				else {
					int minnode = minnodes2[0];
					for (int i = 0; i < minnodes2.size(); i++) {
						if (minnodes2[i] == prev_dir1) {
							minnode = minnodes2[i];
							break;
						}
						else if (minnodes2[i] == prev_dir2) {
							minnode = minnodes2[i];
						}
					}
					currentnode = adj_nodes[minnode];
				}

			}
			path.push_back(currentnode.n_node);
			//set directions
			if (currentnode.n_node == prevnode.n_left && prev_dir1 != 0) {
				prev_dir2 = prev_dir1;
				prev_dir1 = 0;
			}
			else if (currentnode.n_node == prevnode.n_right && prev_dir1 != 1) {
				prev_dir2 = prev_dir1;
				prev_dir1 = 1;
			}
			if (currentnode.n_node == prevnode.n_up && prev_dir1 != 2) {
				prev_dir2 = prev_dir1;
				prev_dir1 = 2;
			}
			if (currentnode.n_node == prevnode.n_down && prev_dir1 != 3) {
				prev_dir2 = prev_dir1;
				prev_dir1 = 3;
			}

		}

		//

	}

	return path;
}
