#include "stdafx.h"
#include "kdtree.h"

#pragma warning(push, 4)

// Splitting Plane Axis
enum KDTreeNodeAxis {
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2
};

/// KD Tree Node
class KDTreeNode {
public: // methods
	virtual ~KDTreeNode() {}

protected: // members
	size_t m_idxPoint;
};

/// KD Tree Inner Node
class KDTreeInnerNode : public KDTreeNode {
public: // methods

private: // members
	KDTreeNodeAxis m_axis;
	size_t m_arrIdxChildNodes[2];
};

/// KD Tree Leaf Node
class KDTreeLeafNode : public KDTreeNode {
public: // methods

};

/// KD Tree Implementation 
class KDTreeImpl {
public: // methods
	KDTreeImpl(const vector<V3x>& arrPoints);

private: // members
	vector<unique_ptr<KDTreeNode> > m_arrNodes;
	vector<V3x> m_arrPoints;
};

////////////////////////////////////////////////////////////////////////////////
// KDTreeImpl Methods
////////////////////////////////////////////////////////////////////////////////

KDTreeImpl::KDTreeImpl(const vector<V3x>& arrPoints)
	: m_arrPoints(arrPoints)
{
}

////////////////////////////////////////////////////////////////////////////////
// KDTree Methods
////////////////////////////////////////////////////////////////////////////////

KDTree::KDTree(const vector<V3x>& arrPoints)
	: m_pImpl(new KDTreeImpl(arrPoints))
{
}

KDTree::~KDTree()
{
	delete m_pImpl;
}

#pragma warning(pop)
