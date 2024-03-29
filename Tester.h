#pragma once

#include "CPLPSolver.h"
#include <vector>



struct Test
{
	Test(float u, float v, std::vector<float> linearConstraints, std::vector<float> circleConstraints, bool feasible, float optX, float optY);
	//problem
	float u, v;
	//constraints: linear constraints
	std::vector<float> constraints;
	std::vector<float> circleConstraints;
	
	
	//expected results
	bool feasible;
	float optX, optY;
};


class Tester
{
public:
	Tester();
	~Tester();
	
	void InitTests();
	void AddTest(Test t);
	void RunTests();
	
private:
	CPLPSolver solver;
	
	std::vector<Test> tests;
};


