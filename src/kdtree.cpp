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
typedef vector<KDTreeNode> KDTreeNodeList;

/// KD Tree Inner Node
class KDTreeNode
{
public: // methods
	KDTreeNode(
		size_t idxPoint,
		size_t idxLeft,
		size_t idxRight,
		KDTreeNodeAxis axis);

	size_t			getIdxPoint() const { return m_idxPoint; }
	KDTreeNodeAxis	getAxis() const		{ return m_axis; }
	size_t			getIdxLeft()		{ return m_idxLeft; }
	size_t			getIdxRight()		{ return m_idxRight; }

	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	size_t			getSize(const KDTreeNodeList& arrNodes) const;
	bool			isBalanced(const KDTreeNodeList& arrNodes) const;

	void dump(const vector<V3x>& arrPoints, ostream& out) const;

private: // members
	KDTreeNodeAxis m_axis;
	size_t m_idxPoint;
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
	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	void dump(ostream& out = cerr) const;

private: // methods

	const KDTreeNode* getRootNode() const;

	inline size_t partitionAroundMedian(
		size_t idxBegin,
		size_t idxEnd,
		KDTreeNodeAxis axis);

private: // members
	KDTreeNodeList m_arrNodes;
	vector<V3x> m_arrPoints;
};

////////////////////////////////////////////////////////////////////////////////
// KDTreeNode Methods
////////////////////////////////////////////////////////////////////////////////

KDTreeNode::KDTreeNode(
	size_t idxPoint,
	size_t idxLeft,
	size_t idxRight,
	KDTreeNodeAxis axis)
	: m_idxPoint(idxPoint)
	, m_idxLeft(idxLeft)
	, m_idxRight(idxRight)
	, m_axis(axis)
{
	assert(m_idxPoint != IDX_NONE);
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
KDTreeNode::dump(const vector<V3x>& arrPoints, ostream& out) const
{
	assert(arrPoints.size() > m_idxPoint);

	switch (m_axis) {
	case X_AXIS: out << "X AXIS"; break;
	case Y_AXIS: out << "Y AXIS"; break;
	case Z_AXIS: out << "Z AXIS"; break;
	}
	out << ", POINT " << m_idxPoint << ": " << arrPoints[m_idxPoint] << "\n";
	out << "  CHILDREN: " << getIdxString(m_idxLeft) << " "
		<< getIdxString(m_idxRight) << "\n";
}

static inline size_t
getChildSize(const KDTreeNodeList& arrNodes, size_t idx)
{
	return (idx == IDX_NONE) ? 0 : arrNodes[idx].getSize(arrNodes);
}

static inline bool
sizesAreBalanced(size_t leftSize, size_t rightSize)
{
	return leftSize <= rightSize+1 && rightSize <= leftSize+1;
}

static inline bool
subtreeIsBalanced(size_t size, size_t idx, const KDTreeNodeList& arrNodes)
{
	return (size == 0) ? true : arrNodes[idx].isBalanced(arrNodes);
}

bool
KDTreeNode::isBalanced(const KDTreeNodeList& arrNodes) const
{
	size_t leftSize = getChildSize(arrNodes, m_idxLeft);
	size_t rightSize = getChildSize(arrNodes, m_idxRight);
	if (!sizesAreBalanced(leftSize, rightSize))
		return false;

	return subtreeIsBalanced(leftSize, m_idxLeft, arrNodes)
		&& subtreeIsBalanced(rightSize, m_idxRight, arrNodes);
}

size_t
KDTreeNode::getSize(const KDTreeNodeList& arrNodes) const
{
	return 1 + getChildSize(arrNodes, m_idxLeft) 
		+ getChildSize(arrNodes, m_idxRight);
}

bool
KDTreeNode::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	// TODO: implement
	UNUSED(point);
	UNUSED(out_result);
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
	m_arrNodes.reserve(numPoints);
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
		m_arrNodes.emplace_back(
			KDTreeNode(idxBegin, IDX_NONE, IDX_NONE, axis));
		g_allocTimer.stop();
		return m_arrNodes.size()-1;
	}

	KDTreeNodeAxis nextAxis = static_cast<KDTreeNodeAxis>((axis + 1) % 3);
	size_t idxMedian = partitionAroundMedian(idxBegin, idxEnd, axis);
	size_t idxLeft = buildTree(idxBegin, idxMedian, nextAxis);
	size_t idxRight = buildTree(idxMedian+1, idxEnd, nextAxis);

	g_allocTimer.start();
	m_arrNodes.emplace_back(
		KDTreeNode(idxMedian, idxLeft, idxRight, axis));
	g_allocTimer.stop();
	return m_arrNodes.size()-1;
}

void
KDTreeImpl::dump(ostream& out) const
{
	out << "== KD TREE IMPLEMENTATION ====\n";
	out << "POINT COUNT: " << m_arrPoints.size() << "\n";
	out << "NODE COUNT: " << m_arrNodes.size() << "\n\n";

	out << "-- NODES ----\n";
	auto itBegin = begin(m_arrNodes);
	auto itEnd = end(m_arrNodes);
	int idxNode = 0;
	for_each(itBegin, itEnd, [&](const KDTreeNode& node) {
		out << idxNode << ": ";
		node.dump(m_arrPoints, out);
		++idxNode;
	});

	out.flush();
}

bool
KDTreeImpl::isBalanced() const
{
	if (m_arrNodes.size() <= 2)
		return true;

	const KDTreeNode* pRoot = getRootNode();
	assert(pRoot != NULL);
	return pRoot->isBalanced(m_arrNodes);
}

const KDTreeNode*
KDTreeImpl::getRootNode() const
{
	if (m_arrNodes.empty())
		return NULL;
	return &m_arrNodes.back();
}

bool
KDTreeImpl::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	const KDTreeNode* pRootNode = getRootNode();
	if (pRootNode == NULL)
		return false;
	return pRootNode->getClosestPoint(point, out_result);
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

bool
KDTree::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	return m_pImpl->getClosestPoint(point, out_result);
}

#pragma warning(pop)
