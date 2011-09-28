#pragma once
#ifndef EPL_KDTREE_H_
#define EPL_KDTREE_H_

#include "stdafx.h"

// Forward Declarations
class KDTreeImpl;

class KDTree : public Uncopyable 
{
public: // methods
	KDTree(const vector<V3x>& arrPoints);
	~KDTree();

	bool isBalanced() const;
	void dump(ostream& out) const;

private: // members
	KDTreeImpl* m_pImpl;
};

#endif // EPL_KDTREE_H_
