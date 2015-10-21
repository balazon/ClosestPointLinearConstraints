#include "Tester.h"

#include <stdio.h>
#include <iostream>

#include <cmath>

#define EPS 0.001

Test::Test(float u, float v, std::vector<float> linearConstraints, std::vector<float> circleConstraints, bool feasible, float optX, float optY)
	: u{u}, v{v}, constraints{linearConstraints}, circleConstraints{circleConstraints}, feasible{feasible}, optX{optX}, optY{optY}
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
	constraints.reserve(30);
	std::vector<float> circleConstraints;
	circleConstraints.reserve(9);
	float cc[] = {};
	circleConstraints.assign(cc, cc);
	
	float c1[] = {-1.f, -1.f, -5.f,
				  1.f, -1.f, 0,
				  0, 1.f, 0};
	constraints.assign(c1, c1 + 3 * 3);
	Test t1{1.f, 1.f, constraints, circleConstraints, true, 2.5f, 2.5f * (sqrtf(2.f) - 1)};
	AddTest(t1);
	
	float c2[] = {-1.f, 1.f, 2.5f,
				  0.f, 1.f, 5.f,
				  1.f, 0.f, 5.f};
	constraints.assign(c2, c2 + 3 * 3);
	Test t2{2.5f, 2.5f, constraints, circleConstraints, true, 2.5f, 2.5f};
	AddTest(t2);
	
	Test t3{7.f, -2.f, constraints, circleConstraints, true, 5.f, -2.f};
	AddTest(t3);
	
	Test t4{7.f, 7.f, constraints, circleConstraints, true, 5.f, 5.f};
	AddTest(t4);
	
	float c5[] = {1.f, 1.f, 7.f,
				  -2.f, 1.f, 4.f,
				  -2.f, -1.f, 4.f,
				  2.f/3.f, - 1.f, 4.f/3.f};
	constraints.assign(c5, c5 + 3 * 4);
	Test t5{-3.f, 3.f, constraints, circleConstraints, true, -1.f, 2.f};
	AddTest(t5);
	
	float c6[] = {1.f, 1.f, 7.f,
				  -2.f, 1.f, 4.f,
				  -2.f, -1.f, 4.f,
				  2.f/3.f, - 1.f, 4.f/3.f,
				  0.8f, 1.f, 10.f,
				  1.f/3.f, -1.f, 3.f};
	constraints.assign(c6, c6 + 3 * 6);
	Test t6{-3.f, 3.f, constraints, circleConstraints, true, -1.f, 2.f};
	AddTest(t6);
	
	float c7[] = {1.f, 1.f, 7.f,
				  -2.f, 1.f, 4.f,
				  -2.f, -1.f, 4.f,
				  2.f/3.f, - 1.f, 4.f/3.f,
				  0.8f, 1.f, 10.f,
				  1.f/3.f, -1.f, 3.f};
	constraints.assign(c7, c7 + 3 * 6);
	Test t7{-7.f, -1.f, constraints, circleConstraints, true, -2.f, 0.f};
	AddTest(t7);
	
	float c8[] = {1.f, 1.f, 7.f,
				  -2.f, 1.f, 4.f,
				  -2.f, -1.f, 4.f,
				  2.f/3.f, - 1.f, 4.f/3.f,
				  0.8f, 1.f, 10.f,
				  1.f/3.f, -1.f, 3.f};
	constraints.assign(c8, c8 + 3 * 6);
	Test t8{6.f, 5.f, constraints, circleConstraints, true, 4.f, 3.f};
	AddTest(t8);
	
	constraints.assign(c2, c2 + 3 * 3);
	float cc1[] = {3.f, 2.f, sqrtf(2.f)};
	circleConstraints.assign(cc1, cc1 + 3 * 1);
	Test t9{7.f, -2.f, constraints, circleConstraints, true, 4.f, 1.f};
	AddTest(t9);
	
	
	constraints.assign(c2, c2 + 3 * 3);
	float cc2[] = {3.f, 2.f, sqrtf(2.f),
				   5.5f, 1.f, 1.5f};
	circleConstraints.assign(cc2, cc2 + 3 * 2);
	Test t10{3.f, -3.f, constraints, circleConstraints, true, 4.f, 1.f};
	AddTest(t10);
	
	
	float c11[] = {1.f, 0.f, 0.f,
				   0.f, 1.f, 0.f,
				   -3.f, -4.f, -12.f};
	constraints.assign(c11, c11 + 3 * 3);
	circleConstraints.assign(cc, cc);
	Test t11{5.f, 0.f, constraints, circleConstraints, true, 1.f, 1.f};
	AddTest(t11);
	
	float c12[] = {1.f, 0.f, 0.f,
				   -1.f, 0.f, -4.f,
				   0.f, 1.f, 0.f};
	constraints.assign(c12, c12 + 3 * 3);
	Test t12{1.f, 0.5f, constraints, circleConstraints, true, 2.f, 0.5f};
	AddTest(t12);
	
	Test t13{3.f, 4.f, constraints, circleConstraints, true, 2.f, 2.f};
	AddTest(t13);
	
	float c14[] = {-3.f, 1.f, -24.f,
				   -3.f, -1.f, -30.f};
	float cc14[] = {3.f, 3.f, 3.f};
	constraints.assign(c14, c14 + 3 * 2);
	circleConstraints.assign(cc14, cc14 + 3 * 1);
	Test t14{0.f, 0.f, constraints, circleConstraints, true, 6.f, 3.f};
	AddTest(t14);
	
	float c15[] = {-1.f, 0.f, -5.f,
				   1.f, -1.f, 5.f};
	float cc15[] = {0.f, 3.f, 3.f};
	constraints.assign(c15, c15 + 3 * 2);
	circleConstraints.assign(cc15, cc15 + 3 * 1);
	Test t15{0.f, 0.f, constraints, circleConstraints, true, 3.f, 3.f};
	AddTest(t15);
	
	float c16[] = {-.5f, 1.f, -3.f};
	float cc16[] = {1.f, 2.f, sqrtf(10.f),
					4.f, 2.f, 1.f};
	constraints.assign(c16, c16 + 3 * 1);
	circleConstraints.assign(cc16, cc16 + 3 * 2);
	Test t16{0.f, 0.f, constraints, circleConstraints, true, 4.f, 1.f};
	AddTest(t16);

	
}

void Tester::AddTest(Test t)
{
	tests.push_back(t);
}

void Tester::RunTests()
{
	
	
	int testCount = tests.size();
	int passedTest = 0;
	int testIndex = 0;
	for(Test t : tests)
	{
		testIndex++;
		float x = 0.f;
		float y = 0.f;
	
		solver.Reset();
		solver.SetDestination(t.u, t.v);
		int constraintCount = t.constraints.size() / 3;
		for(int i = 0; i < constraintCount; i++)
		{
			float A = t.constraints[i * 3];
			float B = t.constraints[i * 3 + 1];
			float C = t.constraints[i * 3 + 2];
			
			solver.AddConstraintLinear(A, B, C);
		}
		int circleConstraintCount = t.circleConstraints.size() / 3;
		for(int i = 0; i < circleConstraintCount; i++)
		{
			solver.AddConstraintCircle(t.circleConstraints[i * 3], t.circleConstraints[i * 3 + 1], t.circleConstraints[i * 3 + 2]);
		}
		solver.Solve(x, y);
		bool feasible = solver.HasSolution();
		const char* str_feasible = "feasible";
		const char* str_not_feasible = "not feasible";
		
		std::cout << "Test " << testIndex << ": " << (t.feasible ? str_feasible : str_not_feasible);
		if(t.feasible)
			std::cout << ", opt at ("<< t.optX << "," << t.optY << ")\n";
		else
			std::cout << "\n";
		
		std::cout << "Solver result: " << (feasible ? str_feasible : str_not_feasible);
		if(feasible)
			std::cout << ", opt at: ("<< x << "," << y << ")\n\n";
		else
			std::cout << "\n\n";
			
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

