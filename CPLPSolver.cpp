#include "CPLPSolver.h"

#include <cmath>

CPLPSolver::CPLPSolver()
{
	constraints.reserve(30);
}

CPLPSolver::~CPLPSolver()
{
}

void CPLPSolver::Reset()
{
	constraints.clear();
	feasible = false;
}

void CPLPSolver::AddConstraint(float A, float B, float C)
{
	constraints.reserve(constraints.size() + 3);
	constraints.push_back(A);
	constraints.push_back(B);
	constraints.push_back(C);
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

void CPLPSolver::Solve(float& resX, float& resY)
{
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
	
	std::vector<float> onedimConstraints;
	onedimConstraints.reserve(20);
	for(int i = 0; i < constraints.size() / 3; i++)
	{
		if(constraints[i] == 0.f)
		{
			onedimConstraints.reserve(onedimConstraints.size() + 2);
			onedimConstraints.push_back(constraints[i + 1]);
			onedimConstraints.push_back(constraints[i + 2]);
			continue;
		}
		for(int j = i + 3; j < constraints.size() / 3; j++)
		{
			if(constraints[j] == 0.f)
			{
				continue;
			}
			if(constraints[i] != constraints[j])
			{
				onedimConstraints.push_back(constraints[i + 1] + constraints[j + 1]);
				onedimConstraints.push_back(constraints[i + 2] + constraints[j + 2]);
			}
			
		}
	}
	
	//TODO onedim checking for feasibility
	
	//TODO solving.. by intersections and creating convex polygon ?
}

