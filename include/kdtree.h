#pragma once
#ifndef EPL_KDTREE_H_
#define EPL_KDTREE_H_

#include "stdafx.h"

// Forward Declarations
class PointKDTreeImpl;

// Result of KDTree::getClosestPoint()
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

	bool isBalanced() const;
	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;
	void dump(ostream& out) const;

private: // members
	PointKDTreeImpl* m_pImpl;
};

#endif // EPL_KDTREE_H_
