#include "stdafx.h"
#include "kdtree.h"

#pragma warning(push, 4)

// Splitting Plane Axis
enum KDTreeNodeAxis {
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2
};

static const size_t IDX_NONE = numeric_limits<size_t>::max();

/// KD Tree Node
class KDTreeNode {
public: // methods
	KDTreeNode(
		size_t idxPoint,
		KDTreeNodeAxis axis);
	virtual ~KDTreeNode() {}

	size_t getIdxPoint() { return m_idxPoint; }
	KDTreeNodeAxis getAxis() { return m_axis; }

	virtual void dump(const vector<V3x>& arrPoints, ostream& out) const;

protected: // members
	KDTreeNodeAxis m_axis;
	size_t m_idxPoint;
};

/// KD Tree Inner Node
class KDTreeInnerNode : public KDTreeNode {
public: // methods
	KDTreeInnerNode(
		size_t idxPoint,
		size_t idxLeft,
		size_t idxRight,
		KDTreeNodeAxis axis);

	size_t getIdxLeft() { return m_arrIdxChildNodes[0]; }
	size_t getIdxRight() { return m_arrIdxChildNodes[1]; }

	virtual void dump(const vector<V3x>& arrPoints, ostream& out) const;

private: // members
	size_t m_arrIdxChildNodes[2];
};


/// KD Tree Implementation 
class KDTreeImpl {
public: // methods
	KDTreeImpl(const vector<V3x>& arrPoints);

	size_t buildTree(
		size_t idxBegin,
		size_t idxEnd,
		KDTreeNodeAxis axis);

	void dump(ostream& out = cerr) const;

private: // members
	vector<unique_ptr<KDTreeNode> > m_arrpNodes;
	vector<V3x> m_arrPoints;
};

////////////////////////////////////////////////////////////////////////////////
// KDTreeNode Methods
////////////////////////////////////////////////////////////////////////////////

KDTreeNode::KDTreeNode(size_t idxPoint, KDTreeNodeAxis axis)
	: m_idxPoint(idxPoint)
	, m_axis(axis)
{
	assert(m_idxPoint != IDX_NONE);
}

void
KDTreeNode::dump(const vector<V3x>& arrPoints, ostream& out) const
{
	assert(arrPoints.size() > m_idxPoint);

	switch (m_axis) {
	case X_AXIS: out << "X AXIS"; break;
	case Y_AXIS: out << "Y AXIS"; break;
	case Z_AXIS: out << "Z AXIS"; break;
	}
	out << ", POINT " << m_idxPoint << ": " << arrPoints[m_idxPoint] << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// KDTreeInnerNode Methods
////////////////////////////////////////////////////////////////////////////////

KDTreeInnerNode::KDTreeInnerNode(
	size_t idxPoint,
	size_t idxLeft,
	size_t idxRight,
	KDTreeNodeAxis axis)
	: KDTreeNode(idxPoint, axis)
{
	m_arrIdxChildNodes[0] = idxLeft;
	m_arrIdxChildNodes[1] = idxRight;
}

static string
getIdxString(size_t idx)
{
	if (idx == IDX_NONE)
		return "NONE";

	stringstream ss;
	ss << idx;
	return ss.str();
}

void
KDTreeInnerNode::dump(const vector<V3x>& arrPoints, ostream& out) const
{
	KDTreeNode::dump(arrPoints, out);
	size_t idxLeft = m_arrIdxChildNodes[0];
	size_t idxRight = m_arrIdxChildNodes[1];
	out << "  CHILDREN: " << getIdxString(idxLeft) << " "
		<< getIdxString(idxRight) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// KDTreeImpl Methods
////////////////////////////////////////////////////////////////////////////////

KDTreeImpl::KDTreeImpl(const vector<V3x>& arrPoints)
	: m_arrPoints(arrPoints)
{
	size_t numPoints = m_arrPoints.size();
	m_arrpNodes.reserve(numPoints);
	buildTree(0, numPoints, X_AXIS);
}

size_t 
KDTreeImpl::buildTree(
	size_t idxBegin,
	size_t idxEnd,
	KDTreeNodeAxis axis)
{
	if (idxBegin >= idxEnd)
		return IDX_NONE;

	size_t size = idxEnd - idxBegin;
	if (size == 1) {
		unique_ptr<KDTreeNode> newNode(new KDTreeNode(idxBegin, axis));
		m_arrpNodes.push_back(move(newNode));
		return m_arrpNodes.size()-1;
	}

	auto itBegin = begin(m_arrPoints) + idxBegin;
	auto itEnd = begin(m_arrPoints) + idxEnd;
	sort(itBegin, itEnd, [=](const V3x& lhs, const V3x& rhs)->bool {
		return lhs[axis] < rhs[axis];
	});

	KDTreeNodeAxis nextAxis = static_cast<KDTreeNodeAxis>((axis + 1) % 3);
	size_t idxMedian = idxBegin + size / 2;
	size_t idxLeft = buildTree(idxBegin, idxMedian, nextAxis);
	size_t idxRight = buildTree(idxMedian+1, idxEnd, nextAxis);

	unique_ptr<KDTreeNode> newNode(
		new KDTreeInnerNode(idxMedian, idxLeft, idxRight, axis));
	m_arrpNodes.push_back(move(newNode));
	return m_arrpNodes.size()-1;
}

void
KDTreeImpl::dump(ostream& out) const
{
	out << "== KD TREE IMPLEMENTATION ====\n";
	out << "POINT COUNT: " << m_arrPoints.size() << "\n";
	out << "NODE COUNT: " << m_arrpNodes.size() << "\n\n";

	out << "-- NODES ----\n";
	auto itBegin = begin(m_arrpNodes);
	auto itEnd = end(m_arrpNodes);
	for_each(itBegin, itEnd, [&](const unique_ptr<KDTreeNode>& pNode) {
		pNode->dump(m_arrPoints, out);
	});

	out.flush();
}

////////////////////////////////////////////////////////////////////////////////
// KDTree Methods
////////////////////////////////////////////////////////////////////////////////

KDTree::KDTree(const vector<V3x>& arrPoints)
	: m_pImpl(new KDTreeImpl(arrPoints))
{
}

unittest 
{
	// Fill Points
	vector<V3x> arrPoints(10);
	int count = 0;
	generate(begin(arrPoints), end(arrPoints), [&]() -> V3x {
		return V3x(++count);
	});

	KDTree kdtree(arrPoints);
	kdtree.dump();
}

KDTree::~KDTree()
{
	delete m_pImpl;
}

void
KDTree::dump(ostream& out) const
{
	m_pImpl->dump(out);
}

#pragma warning(pop)
