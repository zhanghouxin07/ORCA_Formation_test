/*
 * Sphere.cpp
 * RVO2-3D Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
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
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

/* Example file showing a demo with 812 agents initially positioned evenly distributed on a sphere attempting to move to the antipodal position on the sphere. */

#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>
#include <cstddef>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <algorithm>

#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <RVO.h>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif
#include <fstream>
using namespace std;

#define n 343

fstream in,in_Dk,in_errk,final_Dk,final_errk;
/* Store the goals of the agents. */
std::vector<RVO::Vector3> goals;

//destination
float destination_x[n], destination_y[n], destination_z[n];
//uav position
float px[n],py[n],pz[n];

float m=7, l=7, radius=12;
float offset_dest = (m-1)/2;
//两个全局变量
# define all_step 300
float all_disk[all_step+1] = {0};
float all_errk[all_step+1] = {0};
float one_disk[all_step+1] = {0};
float one_errk[all_step+1] = {0};
void setupScenario(RVO::RVOSimulator *sim)
{
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(1.0f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(100.0f, n, 10.0f, 0.51f, 1.0f);
	//neighborDist 10.0
	//maxNeighbors 125
	// /* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
	// for (float a = 0; a < M_PI; a += 1.0f) {
	// 	const float z = 100.0f * std::cos(a);
	// 	const float r = 100.0f * std::sin(a);

	// 	for (size_t i = 0; i < r / 2.5f; ++i) {
	// 		const float x = r * std::cos(i * 2.0f * M_PI / (r / 2.5f));
	// 		const float y = r * std::sin(i * 2.0f * M_PI / (r / 2.5f));

	// 		sim->addAgent(RVO::Vector3(x, y, z));
			
	// 		goals.push_back(-sim->getAgentPosition(sim->getNumAgents() - 1));
	// 	}
	//  }
	struct agent_pos{
		float x;
		float y;
		float z;
	};
	
	// initial set
	// 固定初始位置的初始化
	cout<<"乱序初始位置初始化"<<endl;
	float init_interval = 1.5;
	srand((unsigned int)time(0));
	float befor_assign_x[n],befor_assign_y[n],befor_assign_z[n];
	vector<int> assign_index;
	for (int i=1; i<=n; ++i){
		float zz=floor((i-1)/m/l);
    	float yy=floor((i-zz*m*l-1)/m);
    	float xx=(i-zz*m*l-yy*m-1);
		//记录索引编号
		assign_index.push_back(i);
		befor_assign_x[i] = (xx-2)*init_interval;
		befor_assign_y[i] = (yy-2)*init_interval;
		befor_assign_z[i] = (zz-2)*init_interval;
		// sim->addAgent(RVO::Vector3((xx-2)*init_interval, (yy-2)*init_interval, (zz-2)*init_interval));		
		// 设置目的地
		
		RVO::Vector3 subgoals((xx-offset_dest)*radius, (yy-offset_dest)*radius, (zz-offset_dest)*radius);
		goals.push_back(subgoals);
		destination_x[i-1] = (xx-offset_dest)*radius;
		destination_y[i-1] = (yy-offset_dest)*radius;
		destination_z[i-1] = (zz-offset_dest)*radius;
	}
	//乱序索引
	random_shuffle(assign_index.begin(), assign_index.end());
	//按乱序加入目的地
	for(int i=0; i<assign_index.size(); ++i){
		int tmp = assign_index[i];
		sim->addAgent(RVO::Vector3(befor_assign_x[tmp], befor_assign_y[tmp], 
			befor_assign_z[tmp]));		
	}
	//打印终点
	// for(int i=0; i<n; i++)
		// cout<<destination_x[i]<<' '<<destination_y[i]<<' '<<destination_z[i]<<endl;
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
void updateVisualization(RVO::RVOSimulator *sim)
{
	/* Output the current global time. */
	std::cout << sim->getGlobalTime();

	/* Output the position for all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		// std::cout << " " << sim->getAgentPosition(i);
		RVO::Vector3  tmp = sim->getAgentPosition(i);

		//store uav position
		px[i] = tmp[0];
		py[i] = tmp[1];
		pz[i] = tmp[2];
		
		in << tmp[0]<< ' '<<tmp[1]<<' '<<tmp[2]<<' ';
	}
	std::cout << std::endl;
	in<<endl;
	//计算Dk
	float Dk = 9999;
	for(int i=0;i<sim->getNumAgents(); ++i){
		for(int j=0;j<sim->getNumAgents(); ++j){
			float dis_ = (px[i]-px[j])*(px[i]-px[j]) + 
				(py[i]-py[j])*(py[i]-py[j]) +(pz[i]-pz[j])*(pz[i]-pz[j]);
			dis_ = sqrt(dis_);
			if(i!=j && dis_<Dk) Dk = dis_;
		}
	}
	one_disk[(int)sim->getGlobalTime()] = Dk;
	// in_Dk<<Dk<<endl;
	//计算errork
	float errk = -1;
	float sum_errk = 0;
	for (int i=0; i<sim->getNumAgents(); ++i){
		float dis_ = (px[i]-destination_x[i])*(px[i]-destination_x[i]) + 
			(py[i]-destination_y[i])*(py[i]-destination_y[i]) +
			(pz[i]-destination_z[i])*(pz[i]-destination_z[i]) ;
		dis_ = sqrt(dis_);
		sum_errk += dis_;
		if(dis_>errk) errk = dis_;
	}
	one_errk[(int)sim->getGlobalTime()] = sum_errk;
	// in_errk<<errk<<endl;
}
#endif

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		RVO::Vector3 goalVector = goals[i] - sim->getAgentPosition(i);

		// if (RVO::absSq(goalVector) > 1.0f) {
		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);
		// sim->setAgentVelocity(i, goalVector);
	}
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
	/* Check if all agents have reached their goals. *///根据位置判断是否到达
	// for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		
	// 	if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 1.0f * sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
		
	// 		return false;
	// 	}
	// }
	// 根据时间判断是否到达
	if (sim->getGlobalTime()>all_step)
	return true;
}
int fail_cnt = 0;
vector<int> convergence_period;
int main()
{
	for(int i=0 ;i<all_step+1; i++) all_disk[i] = 99999.0;
	int simu = 10;
	final_errk.open("/home/lift/Downloads/RVO2-3D/examples/final_errk.txt", ios::out);
	if(!in_errk){
        cout << "open file failed" << endl;
        return 0;
    }
	final_Dk.open("/home/lift/Downloads/RVO2-3D/examples/final_Dk.txt",ios::out);
	if(!in_errk){
        cout << "open file failed" << endl;
        return 0;
    }

for(int i=0; i<simu; i++)
	{
		cout<<"第 "<<i+1<<" 次仿真实验"<<endl;
		/* Create a new simulator instance. */
	RVO::RVOSimulator *sim = new RVO::RVOSimulator();

	/* Set up the scenario. */
	setupScenario(sim);
	//store files
	in.open("/home/lift/Downloads/RVO2-3D/examples/data_orca.txt",ios::out);
	if(!in){
        cout << "open file failed" << endl;
        return 0;
    }
	// in_Dk.open("/home/lift/Downloads/RVO2-3D/examples/data_Dk.txt");
	// if(!in_Dk){
    //     cout << "open file failed" << endl;
    //     return 0;
    // }
	// in_errk.open("/home/lift/Downloads/RVO2-3D/examples/data_errk.txt");
	// if(!in_errk){
    //     cout << "open file failed" << endl;
    //     return 0;
    // }
	/* Perform (and manipulate) the simulation. */
	do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
		updateVisualization(sim);
#endif
		setPreferredVelocities(sim);
		sim->doStep();
	}
	while (!reachedGoal(sim));

	delete sim;
	in.close();
	//更新参数
	//disk
	for(int i=0;i<all_step+1;i++){
		if(one_disk[i] < all_disk[i]) all_disk[i] = one_disk[i];
		all_errk[i] += one_errk[i]; 
		
	}
	//计算收敛周期
	for(int i=0; i<all_step+1; ++i){
		if(one_errk[i]<0.25) {
			convergence_period.push_back(i);
			break;
		}
	}
	//判断是否失败
	if(all_errk[all_step]>0.25) fail_cnt++;
	
}
	for(int i=0;i<all_step+1;i++){
		final_Dk<<all_disk[i]<<endl;
		final_errk<<all_errk[i]/simu<<endl;
	}
	//输出失败次数 和 收敛周期
	cout<<"fail_cnt = "<<fail_cnt<<endl;
	int average_convergence = 0;
	for(int i=0 ;i< convergence_period.size(); ++i){
		average_convergence+=convergence_period[i];
		cout<<"each_convergence = "<<convergence_period[i]<<' ';
	}
	cout<<endl;
	cout<<"convergence_period = "<<average_convergence/convergence_period.size()<<endl;
	return 0;
}
