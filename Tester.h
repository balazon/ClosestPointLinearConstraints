#pragma once

#include "CPLPSolver.h"
#include <vector>

#define EPS 0.001

struct Test
{
	Test(float u, float v, std::vector<float> constraints, bool feasible, float optX, float optY);
	//problem
	float u, v;
	std::vector<float> constraints;
	
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


