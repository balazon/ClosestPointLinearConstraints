#pragma once

#include <vector>

//CPLP for Closest Point Linear Programming
class CPLPSolver
{
public:
	CPLPSolver();
	~CPLPSolver();
	
	void Reset();
	
	//Ax + By < C
	void AddConstraint(float A, float B, float C);
	
	//Destination coordinates
	void SetDestination(float u, float v);
	
	bool HasSolution();
	void Solve(float& resX, float& resY);
	
private:
	float u, v;
	
	//3 elements define a constraint
	std::vector<float> constraints;
	
	
	bool feasible;
};


