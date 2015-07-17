#include "Tester.h"

#include <stdio.h>
#include <iostream>

#include <cmath>

Test::Test(float u, float v, std::vector<float> constraints, bool feasible, float optX, float optY)
	: u{u}, v{v}, constraints{constraints}, feasible{feasible}, optX{optX}, optY{optY}
{
	
}


Tester::Tester() : solver{}
{
	
}

Tester::~Tester()
{
}

void Tester::InitTests()
{
	std::vector<float> constraints;
	constraints.reserve(20);
	
	float c1[] = {-1.f, -1.f, -5.f,
				  1.f, -1.f, 0,
				  0, 1.f, 0};
	constraints.assign(c1, c1 + 3 * 3);
	Test t{1.f, 1.f, constraints, false, 0.f, 0.f};
	AddTest(t);
	
}

void Tester::AddTest(Test t)
{
	tests.push_back(t);
}

void Tester::RunTests()
{
	
	
	int testCount = tests.size();
	int passedTest = 0;
	for(Test t : tests)
	{
		float x = 0.f;
		float y = 0.f;
	
		solver.Reset();
		solver.SetDestination(t.u, t.v);
		int constraintCount = t.constraints.size() / 3;
		for(int i = 0; i < constraintCount; i++)
		{
			solver.AddConstraint(t.constraints[i * 3], t.constraints[i * 3 + 1], t.constraints[i * 3 + 2]);
		}
		solver.Solve(x, y);
		bool feasible = solver.HasSolution();
		if(!t.feasible && !feasible)
		{
			passedTest++;
			continue;
		}
		if(t.feasible != feasible)
		{
			continue;
		}
		if(fabs(x - t.optX) < EPS && fabs(y - t.optY) < EPS)
		{
			passedTest++;
		}
		
	}
	
	std::cout << passedTest << " out of " << testCount << " tests passed : ";
	if(passedTest == testCount)
	{
		std::cout << "PASS";
	}
	else
	{
		std::cout << "FAIL";
	}
	std::cout << std::endl;
}

