#pragma once

#include <vector>
#include <unordered_set>

//CPLP for Closest Point Linear Programming
class CPLPSolver
{
public:
	CPLPSolver();
	~CPLPSolver();
	
	void Reset();
	
	//Ax + By < C
	void AddConstraintLinear(float A, float B, float C);
	
	void AddConstraintCircle(float U, float V, float R);
	
	//Destination coordinates
	void SetDestination(float u, float v);
	
	bool HasSolution();
	void Solve(float& resX, float& resY);
	
private:
	float u, v;
	
	enum ConstraintType {CT_LINEAR, CT_CIRCLE};
	
	//3 elements define a constraint
	std::vector<float> constraints;
	
	std::vector<ConstraintType> constraintTypes;
	
	bool feasible;
	
	bool pointSatisfiesConstraints(float tx, float ty, const std::unordered_set<int>& filterIndexes = std::unordered_set<int>{});
	
	void solveFeasibility();
	
};


