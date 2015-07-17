#include "CPLPSolver.h"

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
	return false;
}

void CPLPSolver::Solve(float& resX, float& resY)
{
	
}

