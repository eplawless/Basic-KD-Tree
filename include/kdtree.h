#pragma once
#ifndef EPL_KDTREE_H_
#define EPL_KDTREE_H_

#include "stdafx.h"

// Forward Declarations
class PointKDTreeImpl;

// Result of KDTree::getClosestPointTo()
struct KDTreeClosestPoint
{
	V3x point;
	fpreal distance2;

	KDTreeClosestPoint() 
		: point(numeric_limits<fpreal>::max())
		, distance2(numeric_limits<fpreal>::max())
	{}
};

// Splitting Plane Axis
enum KDTreeAxis 
{
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2
};

class PointKDTree : public Uncopyable 
{
public: // methods
	PointKDTree(const vector<V3x>& arrPoints);
	PointKDTree(vector<V3x>&& arrPoints);
	~PointKDTree();

	bool getClosestPointTo(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	bool isBalanced() const;
	void dump(ostream& out) const;

private: // members
	const unique_ptr<PointKDTreeImpl> m_pImpl;
};

/// @{
/// KD Tree Unit Tests
static inline void fillPoints(
	vector<V3x>& arrPoints,
	const size_t numPoints)
{
	Timer fillTimer("fill points");
	fillTimer.start();

	srand(1987);
	arrPoints.clear();
	arrPoints.reserve(numPoints);
	for (size_t idx = 0; idx < numPoints; ++idx) {
		arrPoints.emplace_back(V3x(rand(), rand(), rand()));
	}

	fillTimer.stop();
	fillTimer.print();
}

static inline unique_ptr<PointKDTree> buildTree(
	vector<V3x>& arrPoints)
{
	Timer treeTimer("build kd tree");
	treeTimer.start();
	unique_ptr<PointKDTree> kdtree(new PointKDTree(move(arrPoints)));
	treeTimer.stop();
	treeTimer.print();

	Timer balancedTimer("check tree balance");
	balancedTimer.start();
	REQUIRE(kdtree->isBalanced());
	balancedTimer.stop();
	balancedTimer.print();

	return kdtree;
}

static inline unique_ptr<PointKDTree> 
createKDTreeTest(size_t numPoints)
{
	cout << "\n";
	vector<V3x> arrPoints;
	fillPoints(arrPoints, numPoints);
	return buildTree(arrPoints);
}

static inline void
fillPointsAlongAxis(
	vector<V3x>& arrPoints, 
	size_t numPoints,
	KDTreeAxis axis)
{
	Timer fillTimer("fill points");
	fillTimer.start();

	arrPoints.clear();
	arrPoints.reserve(numPoints);
	int idxBegin = -static_cast<int>(numPoints/2);
	int idxEnd = idxBegin + numPoints;
	for (int idx = idxBegin; idx < idxEnd; ++idx) {
		V3x point(0);
		point[axis] = idx;
		arrPoints.emplace_back(point);
	}

	fillTimer.stop();
	fillTimer.print();
}

static inline void
queryTreeAlongAxis(
	unique_ptr<PointKDTree>& tree,
	size_t numPoints,
	KDTreeAxis axis)
{
	Timer queryTimer("nearest neighbour query");
	queryTimer.start();

	int idxBegin = -static_cast<int>(numPoints/2);
	int idxEnd = idxBegin + numPoints;
	for (int idx = idxBegin; idx < idxEnd; ++idx) {
		KDTreeClosestPoint result;
		V3x queryPoint(0);
		queryPoint[axis] = idx+0.025;
		bool gotPoint = tree->getClosestPointTo(queryPoint, result);
		assert(gotPoint);
		cout << "closest point: " << result.point << " (dist2: " 
			<< result.distance2 << ")" << endl;
	}
	queryTimer.stop();
	queryTimer.print();
}

static unique_ptr<PointKDTree> 
createAxisSplitTest(KDTreeAxis axis)
{
	static const size_t numPoints = 11;
	cout << "\n";
	vector<V3x> arrPoints;
	fillPointsAlongAxis(arrPoints, numPoints, axis);
	auto kdtree = buildTree(arrPoints);
	cout << "\n";
	kdtree->dump(cout);
	queryTreeAlongAxis(kdtree, numPoints+2, axis);
	return kdtree;
}

namedtest("x axis splits") 
{
	createAxisSplitTest(X_AXIS);
}

namedtest("y axis splits") 
{
	createAxisSplitTest(Y_AXIS);
}

namedtest("z axis splits") 
{
	createAxisSplitTest(Z_AXIS);
}

namedtest("empty kdtree") 
{
	createKDTreeTest(0);
}

namedtest("thousand point kdtree") 
{
	createKDTreeTest(1000);
}

namedtest("256 thousand point kdtree") 
{
	createKDTreeTest(256*1000);
}

namedtest("million point kdtree") 
{
	createKDTreeTest(1000*1000);
}

namedtest("16 million point kdtree") {
	createKDTreeTest(16*1000*1000);
}
/// @}

#endif // EPL_KDTREE_H_
