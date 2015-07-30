#include "CPLPSolver.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include "MathUtils.h"


#define EPS 0.001

CPLPSolver::CPLPSolver()
{
	constraints.reserve(30);
	constraintTypes.reserve(30);
	order.reserve(30);
}

CPLPSolver::~CPLPSolver()
{
}

void CPLPSolver::Reset()
{
	constraints.clear();
	feasible = true;
	
}

void CPLPSolver::AddConstraintLinear(float A, float B, float C)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(A);
	constraints.push_back(B);
	constraints.push_back(C);
	
	constraintTypes.push_back(CT_LINEAR);
}

void CPLPSolver::AddConstraintCircle(float U, float V, float R)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(U);
	constraints.push_back(V);
	constraints.push_back(R);
	
	constraintTypes.push_back(CT_CIRCLE);
}

void CPLPSolver::SetDestination(float u, float v)
{
	this->u = u;
	this->v = v;
}

bool CPLPSolver::HasSolution()
{
	//? how
	return feasible;
}

void printArray(std::vector<float>& arr)
{
	std::cout << "tomb: ";
	for(int i = 0; i < arr.size(); i++)
	{
		
		std::cout << arr[i];
		if(i % 3 == 2)
			std::cout << ", ";
		else
			std::cout << " ";
	}
	std::cout << "\n";
}

void printOrder(std::vector<int>& order)
{
	std::cout << "order: ";
	for(int i = 0; i < order.size(); i++)
	{
		std::cout << order[i] << " ";
	}
	std::cout << "\n";
}

void processOneDim(float& minOfUpper, float& maxOfLower, float k, float q)
{
	
}

bool CPLPSolver::pointSatisfiesConstraints(float tx, float ty, const std::unordered_set<int>& filterIndexes)
{
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(filterIndexes.count(i) == 0)
		{
			if(constraintTypes[i] == CT_LINEAR && constraints[i * 3] * tx + constraints[i * 3 + 1] * ty > constraints[i * 3 + 2]) // + EPS  to the end?)
			{
				return false;
			}
			else if (constraintTypes[i] == CT_CIRCLE &&
				(tx - constraints[i * 3]) * (tx - constraints[i * 3]) + (ty - constraints[i * 3 + 1]) * (ty - constraints[i * 3 + 1]) > constraints[i * 3 + 2] * constraints[i * 3 + 2])
			{
				return false;
			}
		}
	}
	return true;
}

bool CPLPSolver::pointSatisfiesConstraint(float tx, float ty, int n)
{
	if(filter.count(n) == 0)
	{
		if(constraintTypes[n] == CT_LINEAR && constraints[n * 3] * tx + constraints[n * 3 + 1] * ty > constraints[n * 3 + 2]) // + EPS  to the end?)
		{
			return false;
		}
		else if (constraintTypes[n] == CT_CIRCLE &&
			(tx - constraints[n * 3]) * (tx - constraints[n * 3]) + (ty - constraints[n * 3 + 1]) * (ty - constraints[n * 3 + 1]) > constraints[n * 3 + 2] * constraints[n * 3 + 2])
		{
			return false;
		}
	}
	
	return true;
}

bool CPLPSolver::pointSatisfiesConstraints(float tx, float ty, int n)
{
	for(int j = 0; j < n; j++)
	{
		int i = order[j];
		if(filter.count(i) == 0)
		{
			if(constraintTypes[i] == CT_LINEAR && constraints[i * 3] * tx + constraints[i * 3 + 1] * ty > constraints[i * 3 + 2]) // + EPS  to the end?)
			{
				return false;
			}
			else if (constraintTypes[i] == CT_CIRCLE &&
				(tx - constraints[i * 3]) * (tx - constraints[i * 3]) + (ty - constraints[i * 3 + 1]) * (ty - constraints[i * 3 + 1]) > constraints[i * 3 + 2] * constraints[i * 3 + 2])
			{
				return false;
			}
		}
	}
	return true;
}

void CPLPSolver::createRandomOrder()
{
	order.clear();
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		order.push_back(i);
	}
	std::random_shuffle(order.begin(), order.end());
}


void CPLPSolver::solveFeasibility()
{
	//making x coeffs 1, -1 or 0
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		float coeff = fabs(constraints[i * 3]);
		if (coeff < EPS)
		{
			constraints[i * 3] = 0.f;
			continue;
		}
		constraints[i * 3] = constraints[i * 3] > 0 ? 1.f : -1.f;
		constraints[i * 3 + 1] = constraints[i * 3 + 1] / coeff;
		constraints[i * 3 + 2] = constraints[i * 3 + 2] / coeff;
	}
	
	float minOfUpper = 1e9;
	float maxOfLower = 1e9;
	//maxOfLower can calculated as min, and then multiplied by (-1) if needed,
	// but multiplication is ignored now and value is treated as the negative maxOfLower value
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(constraints[i * 3] == 0.f)
		{
			//processOneDim(minOfUpper, maxOfLower, constraints[i * 3 + 1], constraints[i * 3 + 2]);
			float coeff = fabs(constraints[i * 3 + 1]);
			if(coeff < EPS)
			{
				constraints[i * 3 + 1] = 0.f;
				if(constraints[i * 3 + 2] < 0)
				{
					feasible = false;
					return;
				}
				continue;
			}
			if(constraints[i * 3 + 1] > 0 && minOfUpper > constraints[i * 3 + 2] / coeff)
			{
				minOfUpper = constraints[i * 3 + 2] / coeff;
				if(minOfUpper + maxOfLower < 0)
				{
					feasible = false;
					return;
				}
			}
			if(constraints[i * 3 + 1] < 0 && maxOfLower > constraints[i * 3 + 2] / coeff)
			{
				maxOfLower = constraints[i * 3 + 2] / coeff;
				if(minOfUpper + maxOfLower < 0)
				{
					feasible = false;
					return;
				}
			}
			continue;
		}
		for(int j = i + 1; j < constraints.size() / 3; j++)
		{
			if(constraints[j * 3] == 0.f)
			{
				continue;
			}
			if(constraints[i * 3] != constraints[j * 3])
			{
				//k * y < q
				float k = constraints[i * 3 + 1] + constraints[j * 3 + 1];
				float q = constraints[i * 3 + 2] + constraints[j * 3 + 2];
				float coeff = fabs(k);
				
				
				if(coeff < EPS)
				{
					
					if(q < 0)
					{
						feasible = false;
						return;
					}
					continue;
				}
				if(k > 0 && minOfUpper > q / coeff)
				{
					minOfUpper = q / coeff;
					if(minOfUpper + maxOfLower < 0)
					{
						feasible = false;
						return;
					}
				}
				if(k < 0 && maxOfLower > q / coeff)
				{
					maxOfLower = q / coeff;
					if(minOfUpper + maxOfLower < 0)
					{
						feasible = false;
						return;
					}
				}
				
			}
			
		}
	}
	
}

void CPLPSolver::Solve(float& resX, float& resY)
{
	//printArray(constraints);
	
	filter.clear();
	createRandomOrder();
	
	float tx = 0.f;
	float ty = 0.f;
	float tx2 = 0.f;
	float ty2 = 0.f;
	
	resX = u;
	resY = v;
	
	printOrder(order);
	
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(pointSatisfiesConstraint(resX, resY, order[i]))
		{
			continue;
		}
		int id = order[i];
		int pos = id * 3;
		
		if(constraintTypes[id] == CT_LINEAR)
		{
			OrthogonalProjectionOfPointOnLine(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		else
		{
			OrthogonalProjectionOfPointOnCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], u, v, tx, ty);
		}
		filter.clear();
		filter.insert(id);
		
		float minDistSqr = 1e9;
		float tdist;
		if(pointSatisfiesConstraints(tx, ty, i))
		{
			tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
			minDistSqr = tdist;
			resX = tx;
			resY = ty;
		}
		
		
		for(int j = 0; j < i; j++)
		{
			int jd = order[j];
			filter.clear();
			filter.insert(id);
			filter.insert(jd);
			
			int pos2 = jd * 3;
			bool onePoint = false;
			bool hasIntersection = false;
			if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLines(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty);
				onePoint = true;
			}
			else if(constraintTypes[id] == CT_LINEAR && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectLineCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_LINEAR)
			{
				hasIntersection = IntersectLineCircle(constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], constraints[pos], constraints[pos + 1], constraints[pos + 2], tx, ty, tx2, ty2);
			}
			else if(constraintTypes[id] == CT_CIRCLE && constraintTypes[jd] == CT_CIRCLE)
			{
				hasIntersection = IntersectCircleCircle(constraints[pos], constraints[pos + 1], constraints[pos + 2], constraints[pos2], constraints[pos2 + 1], constraints[pos2 + 2], tx, ty, tx2, ty2);
			}
			if(hasIntersection)
			{
				if(pointSatisfiesConstraints(tx, ty, i))
				{
					tdist = (tx - u) * (tx - u) + (ty - v) * (ty - v);
					if(tdist < minDistSqr)
					{
						resX = tx;
						resY = ty;
						minDistSqr = tdist;
					}
				}
				
				if(!onePoint && pointSatisfiesConstraints(tx2, ty2, i))
				{
					tdist = (tx2 - u) * (tx2 - u) + (ty2 - v) * (ty2 - v);
					if(tdist < minDistSqr)
					{
						resX = tx2;
						resY = ty2;
						minDistSqr = tdist;
					}
				}
			}
		}
		if(minDistSqr == 1e9)
		{
			feasible = false;
			return;
		}
	}
	
	//TODO optimizing current solution : currently runtime should be O(n^2) where n is number of constraints,
	// but the average runtime should be O(n) if implementation is the same as in [De Berg et al. 2008]
}

