/*
 * Circle.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

/**
 * \file   Circle.cpp
 * \brief  Example with 250 agents navigating through a circular environment.
 */

// #ifndef HRVO_OUTPUT_TIME_AND_POSITIONS
// #define HRVO_OUTPUT_TIME_AND_POSITIONS 0
// #endif

#include <cmath>
#include <cstdlib>

// #if HRVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
// #endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/imgproc.hpp> // drawing shapes

#include <HRVO.h>
#include <vector>

using namespace std;
using namespace hrvo;
using namespace cv;

const float HRVO_TWO_PI = 6.283185307179586f;

void add_min_distance(vector<double>& distances, const Simulator& simulator){
	double min_dist = 9999999999;
	for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
		for (std::size_t j = i+1; j < simulator.getNumAgents(); ++j){
			double dist_x = simulator.getAgentPosition(i).getX() - simulator.getAgentPosition(j).getX();
			double dist_y = simulator.getAgentPosition(i).getY() - simulator.getAgentPosition(j).getY();
			double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
			if (dist < min_dist){
				min_dist = dist;
			}
		}
	}
	distances.push_back(min_dist);
}

void write_max_min(const vector<double>& distances){
	double min_dist = 9999999999;
	double max_dist = -9999999999;
	for (std::size_t i = 0; i < distances.size(); ++i) {
		if (distances[i] < min_dist){
			min_dist = distances[i];
		}
		if (distances[i] > max_dist){
			max_dist = distances[i];
		}
	}
	cout << "Max distance of the minimum distances: " << max_dist << endl;
	cout << "Min distance of the minimum distances: " << min_dist << endl;
}

void plot_vector(const vector<double>& distances){
	Mat data(distances.size()+1, 1, CV_64F );
	for(size_t i = 0; i<distances.size(); i++){
		data.at<double>(i, 0) = 15-distances[i];
	}
	data.at<double>(distances.size(), 0) = 15;
	Mat plot_result;

    Ptr<plot::Plot2d> plot = cv::plot::createPlot2d(data);
    plot->setPlotBackgroundColor( Scalar( 50, 50, 50 ) );
    plot->setPlotLineColor( Scalar( 50, 50, 255 ) );
    plot->render( plot_result );

    imshow( "plot", plot_result );
}

int main()
{
	Simulator simulator;
	vector<double> distances;

	simulator.setTimeStep(0.25f);
	float robot_size = 5.0f;
	float collision_size = robot_size* 10 * 2;
	size_t num_agents = 75;
	float angle = 1 / float(num_agents);
	float radius = 175.0f;
	size_t windows_size = 500;
	assert(windows_size > radius * 2); 

	simulator.setAgentDefaults(collision_size, 50, robot_size, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < num_agents; ++i)
	{
		const Vector2 position = radius * Vector2(std::cos(angle * i * HRVO_TWO_PI), std::sin(angle * i * HRVO_TWO_PI));
		simulator.addAgent(position, simulator.addGoal(-position));
	}

	do
	{
// #if HRVO_OUTPUT_TIME_AND_POSITIONS
		// std::cout << simulator.getGlobalTime();

		// for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
		// 	std::cout << " " << simulator.getAgentPosition(i);
		// }

		// std::cout << std::endl;

		// First create a black image.
		cv::Mat image(windows_size, windows_size, CV_8UC3, cv::Scalar(0, 0, 0));

		// Check if the image is created successfully.
		if (!image.data)
		{
			std::cout << "Could not open or find the image" << std::endl;
			exit(EXIT_FAILURE);
		}
		srand(0);
		for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
			//####################(  Draw Circle  )#########################
			// filled circle
			int radiusCircle = robot_size;
			cv::Point centerCircle(simulator.getAgentPosition(i).getX() + windows_size/2, simulator.getAgentPosition(i).getY() + windows_size/2);
			
			cv::Scalar colorCircle(rand()%256,rand()%256,rand()%256);
			
			cv::circle(image, centerCircle, radiusCircle, colorCircle, -1);
			//#################( Draw Shapes on Image )######################
			cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
			cv::imshow( "Display window", image );
			
		}
		cv::waitKey(1);
		

// #endif /* HRVO_OUTPUT_TIME_AND_POSITIONS */
		add_min_distance(distances, simulator);
		simulator.doStep();
	} while (!simulator.haveReachedGoals());
	plot_vector(distances);
	write_max_min(distances);

	cv::waitKey(0);
	return 0;
}
