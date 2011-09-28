#include "stdafx.h"
#include "kdtree.h"

#pragma warning(push, 4)

// Splitting Plane Axis
enum KDTreeNodeAxis 
{
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2
};

static const size_t IDX_NONE = numeric_limits<size_t>::max();

class KDTreeNode;
typedef vector<unique_ptr<KDTreeNode> > KDTreeNodePtrList;

/// KD Tree Node
class KDTreeNode 
{
public: // methods
	KDTreeNode(
		size_t idxPoint,
		KDTreeNodeAxis axis);
	virtual ~KDTreeNode() {}

	size_t getIdxPoint() const { return m_idxPoint; }
	KDTreeNodeAxis getAxis() const { return m_axis; }
	virtual size_t getSize(const KDTreeNodePtrList&) const { return 1; }

	virtual bool isBalanced(const KDTreeNodePtrList& arrpNodes) const;
	virtual void dump(const vector<V3x>& arrPoints, ostream& out) const;

protected: // members
	KDTreeNodeAxis m_axis;
	size_t m_idxPoint;
};

/// KD Tree Inner Node
class KDTreeInnerNode : public KDTreeNode 
{
public: // methods
	KDTreeInnerNode(
		size_t idxPoint,
		size_t idxLeft,
		size_t idxRight,
		KDTreeNodeAxis axis);

	size_t getIdxLeft() { return m_idxLeft; }
	size_t getIdxRight() { return m_idxRight; }

	virtual size_t getSize(const KDTreeNodePtrList& arrpNodes) const;

	virtual bool isBalanced(const KDTreeNodePtrList& arrpNodes) const;
	virtual void dump(const vector<V3x>& arrPoints, ostream& out) const;

private: // members
	size_t m_idxLeft;
	size_t m_idxRight;
};

/// KD Tree Implementation 
class KDTreeImpl 
{
public: // methods
	KDTreeImpl(const vector<V3x>& arrPoints);

	size_t buildTree(
		size_t idxBegin,
		size_t idxEnd,
		KDTreeNodeAxis axis);

	bool isBalanced() const;

	void dump(ostream& out = cerr) const;

private: // methods

	KDTreeNode* getRootNode() const;

	inline size_t partitionAroundMedian(
		size_t idxBegin,
		size_t idxEnd,
		KDTreeNodeAxis axis);

private: // members
	KDTreeNodePtrList m_arrpNodes;
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

bool
KDTreeNode::isBalanced(const KDTreeNodePtrList&) const
{
	return true;
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
	, m_idxLeft(idxLeft)
	, m_idxRight(idxRight)
{
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
	out << "  CHILDREN: " << getIdxString(m_idxLeft) << " "
		<< getIdxString(m_idxRight) << "\n";
}

static inline size_t
getChildSize(const KDTreeNodePtrList& arrpNodes, size_t idx)
{
	return (idx == IDX_NONE) ? 0 : arrpNodes[idx]->getSize(arrpNodes);
}

static inline bool
sizesAreBalanced(size_t leftSize, size_t rightSize)
{
	return leftSize <= rightSize+1 && rightSize <= leftSize+1;
}

static inline bool
subtreeIsBalanced(size_t size, size_t idx, const KDTreeNodePtrList& arrpNodes)
{
	return (size == 0) ? true : arrpNodes[idx]->isBalanced(arrpNodes);
}

bool
KDTreeInnerNode::isBalanced(const KDTreeNodePtrList& arrpNodes) const
{
	size_t leftSize = getChildSize(arrpNodes, m_idxLeft);
	size_t rightSize = getChildSize(arrpNodes, m_idxRight);
	if (!sizesAreBalanced(leftSize, rightSize))
		return false;

	return subtreeIsBalanced(leftSize, m_idxLeft, arrpNodes)
		&& subtreeIsBalanced(rightSize, m_idxRight, arrpNodes);
}

size_t
KDTreeInnerNode::getSize(const KDTreeNodePtrList& arrpNodes) const
{
	return 1 + getChildSize(arrpNodes, m_idxLeft) 
		+ getChildSize(arrpNodes, m_idxRight);
}

////////////////////////////////////////////////////////////////////////////////
// KDTreeImpl Methods
////////////////////////////////////////////////////////////////////////////////

static Timer g_allocTimer("allocation in buildTree()");
static Timer g_partitionTimer("partitionAroundMedian()");

KDTreeImpl::KDTreeImpl(const vector<V3x>& arrPoints)
	: m_arrPoints(arrPoints)
{
	size_t numPoints = m_arrPoints.size();
	m_arrpNodes.reserve(numPoints);
	buildTree(0, numPoints, X_AXIS);
}

size_t
KDTreeImpl::partitionAroundMedian(
	size_t idxBegin,
	size_t idxEnd,
	KDTreeNodeAxis axis)
{
	g_partitionTimer.start();

	size_t halfSize = (idxEnd - idxBegin) / 2;
	size_t idxMedian = idxBegin + halfSize;

	auto itGlobalBegin = begin(m_arrPoints);
	auto itBegin = itGlobalBegin + idxBegin;
	auto itMedian = itGlobalBegin + idxMedian;
	auto itEnd = itGlobalBegin + idxEnd;
	nth_element(itBegin, itMedian, itEnd, 
		[=](const V3x& lhs, const V3x& rhs) -> bool {
		return lhs[axis] < rhs[axis];
	});

	g_partitionTimer.stop();
	return idxMedian;
}

size_t 
KDTreeImpl::buildTree(
	size_t idxBegin,
	size_t idxEnd,
	KDTreeNodeAxis axis)
{
	if (idxBegin >= idxEnd)
		return IDX_NONE;

	// Build Leaf Nodes
	size_t size = idxEnd - idxBegin;
	if (size == 1) {
		g_allocTimer.start();
		m_arrpNodes.emplace_back(unique_ptr<KDTreeNode>(
			new KDTreeNode(idxBegin, axis)));
		g_allocTimer.stop();
		return m_arrpNodes.size()-1;
	}

	KDTreeNodeAxis nextAxis = static_cast<KDTreeNodeAxis>((axis + 1) % 3);
	size_t idxMedian = partitionAroundMedian(idxBegin, idxEnd, axis);
	size_t idxLeft = buildTree(idxBegin, idxMedian, nextAxis);
	size_t idxRight = buildTree(idxMedian+1, idxEnd, nextAxis);

	g_allocTimer.start();
	m_arrpNodes.emplace_back(unique_ptr<KDTreeNode>(
		new KDTreeInnerNode(idxMedian, idxLeft, idxRight, axis)));
	g_allocTimer.stop();
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
	int idxNode = 0;
	for_each(itBegin, itEnd, [&](const unique_ptr<KDTreeNode>& pNode) {
		out << idxNode << ": ";
		pNode->dump(m_arrPoints, out);
		++idxNode;
	});

	out.flush();
}

bool
KDTreeImpl::isBalanced() const
{
	if (m_arrpNodes.size() <= 2)
		return true;

	KDTreeNode* pRoot = getRootNode();
	assert(pRoot != NULL);
	return pRoot->isBalanced(m_arrpNodes);
}

KDTreeNode*
KDTreeImpl::getRootNode() const
{
	if (m_arrpNodes.empty())
		return NULL;
	return m_arrpNodes.back().get();
}

////////////////////////////////////////////////////////////////////////////////
// KDTree Methods
////////////////////////////////////////////////////////////////////////////////

KDTree::KDTree(const vector<V3x>& arrPoints)
	: m_pImpl(new KDTreeImpl(arrPoints))
{
}

static inline void fillPoints(
	vector<V3x>& arrPoints,
	const size_t numPoints)
{
	Timer fillTimer("fill points");
	fillTimer.start();

	srand(1987);
	arrPoints.reserve(numPoints);
	for (size_t idx = 0; idx < numPoints; ++idx) {
		arrPoints.emplace_back(V3x(rand(), rand(), rand()));
	}

	fillTimer.stop();
	fillTimer.print();
}

static inline void buildTree(
	const vector<V3x>&arrPoints)
{
	Timer treeTimer("build kd tree");
	g_allocTimer.reset();
	g_partitionTimer.reset();

	treeTimer.start();
	KDTree kdtree(arrPoints);
	treeTimer.stop();

	REQUIRE(kdtree.isBalanced());

	treeTimer.print();
	g_allocTimer.print();
	g_partitionTimer.print();
}

unittest 
{
	cout << "\n";
	static const size_t NUM_POINTS = 1000000;
	vector<V3x> arrPoints;
	fillPoints(arrPoints, NUM_POINTS);
	buildTree(arrPoints);
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

bool
KDTree::isBalanced() const
{
	return m_pImpl->isBalanced();
}

#pragma warning(pop)
