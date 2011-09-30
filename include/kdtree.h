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
	fpreal distance;
};

class PointKDTree : public Uncopyable 
{
public: // methods
	PointKDTree(const vector<V3x>& arrPoints);
	~PointKDTree();

	bool getClosestPointTo(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	bool isBalanced() const;
	void dump(ostream& out) const;

private: // members
	unique_ptr<PointKDTreeImpl> m_pImpl;
};

/// @{
/// Basic KD Tree Test
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
	const vector<V3x>& arrPoints)
{
	Timer treeTimer("build kd tree");
	treeTimer.start();
	unique_ptr<PointKDTree> kdtree(new PointKDTree(arrPoints));
	treeTimer.stop();
	treeTimer.print();

	Timer balancedTimer("check tree balance");
	balancedTimer.start();
	REQUIRE(kdtree->isBalanced());
	balancedTimer.stop();
	balancedTimer.print();

	return kdtree;
}

static inline unique_ptr<PointKDTree> kdTreeTest(size_t numPoints)
{
	cout << "\n";
	vector<V3x> arrPoints;
	fillPoints(arrPoints, numPoints);
	return buildTree(arrPoints);
}

namedtest("empty kd tree") 
{
	unique_ptr<PointKDTree> kdtree = kdTreeTest(0);
}

namedtest("thousand point kdtree") 
{
	unique_ptr<PointKDTree> kdtree = kdTreeTest(1000);
}

namedtest("million point kdtree") 
{
	unique_ptr<PointKDTree> kdtree = kdTreeTest(1000*1000);
}

namedtest("16 million point kdtree") 
{
	unique_ptr<PointKDTree> kdtree = kdTreeTest(16*1000*1000);
}

namedtest("32 million point kdtree") 
{
	unique_ptr<PointKDTree> kdtree = kdTreeTest(32*1000*1000);
}
/// @}

#endif // EPL_KDTREE_H_
