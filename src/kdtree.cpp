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

/// @{
/// Invalid Index
template <typename uint_t>
struct InvalidIndex {
	static const uint_t value = 0;
};

#define KD_TREE_INVALID_INDEX(bits) \
	template <> \
	struct InvalidIndex<uint##bits##_t> { \
		static const uint##bits##_t value = UINT##bits##_MAX; \
	};

KD_TREE_INVALID_INDEX(8)
KD_TREE_INVALID_INDEX(16)
KD_TREE_INVALID_INDEX(32)
KD_TREE_INVALID_INDEX(64)
/// @}

/// KD Tree Inner Node
template <typename uint_t>
class KDTreeNode
{
public: // types
	typedef vector<KDTreeNode<uint_t> > KDTreeNodeList;

public: // methods
	KDTreeNode(
		uint_t idxPoint,
		uint_t idxLeft,
		uint_t idxRight,
		KDTreeNodeAxis axis);

	uint_t			getIdxPoint() const { return m_idxPoint; }
	KDTreeNodeAxis	getAxis() const		{ return m_axis; }
	uint_t			getIdxLeft()		{ return m_idxLeft; }
	uint_t			getIdxRight()		{ return m_idxRight; }

	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	uint_t			getSize(const KDTreeNodeList& arrNodes) const;
	bool			isBalanced(const KDTreeNodeList& arrNodes) const;

	void dump(const vector<V3x>& arrPoints, ostream& out) const;

private: // members
	KDTreeNodeAxis m_axis;
	uint_t m_idxPoint;
	uint_t m_idxLeft;
	uint_t m_idxRight;
};

enum KDTreeIndexType
{
	IDX_TYPE_8,
	IDX_TYPE_16,
	IDX_TYPE_32,
	IDX_TYPE_64,
	IDX_TYPE_INVALID
};

/// KD Tree Templated Implementation 
template <typename uint_t>
class PointKDTreeImplImpl 
{
public: // types
	typedef vector<KDTreeNode<uint_t> > KDTreeNodeList;

public: // methods
	PointKDTreeImplImpl(const vector<V3x>& arrPoints);

	uint_t buildTree(
		uint_t idxBegin,
		uint_t idxEnd,
		KDTreeNodeAxis axis);

	bool isBalanced() const;
	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	void dump(ostream& out = cerr) const;

private: // methods

	const KDTreeNode<uint_t>* getRootNode() const;

	inline uint_t partitionAroundMedian(
		uint_t idxBegin,
		uint_t idxEnd,
		KDTreeNodeAxis axis);

private: // members
	KDTreeNodeList m_arrNodes;
	vector<V3x> m_arrPoints;
};

/// KD Tree Implementation
class PointKDTreeImpl : public Uncopyable
{
public: // methods
	PointKDTreeImpl(const vector<V3x>& arrPoints);

	bool isBalanced() const;
	bool getClosestPoint(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;
	void dump(ostream& out) const;

private: // members
	KDTreeIndexType m_idxType;
	unique_ptr<PointKDTreeImplImpl<uint64_t> > m_pImpl64;
	unique_ptr<PointKDTreeImplImpl<uint32_t> > m_pImpl32;
	unique_ptr<PointKDTreeImplImpl<uint16_t> > m_pImpl16;
	unique_ptr<PointKDTreeImplImpl<uint8_t> > m_pImpl8;
};

////////////////////////////////////////////////////////////////////////////////
// KDTreeNode Methods
////////////////////////////////////////////////////////////////////////////////

template <typename uint_t>
KDTreeNode<uint_t>::KDTreeNode(
	uint_t idxPoint,
	uint_t idxLeft,
	uint_t idxRight,
	KDTreeNodeAxis axis)
	: m_idxPoint(idxPoint)
	, m_idxLeft(idxLeft)
	, m_idxRight(idxRight)
	, m_axis(axis)
{
	assert(m_idxPoint != InvalidIndex<uint_t>::value);
}

template <typename uint_t>
static string
getIdxString(uint_t idx)
{
	if (idx == InvalidIndex<uint_t>::value)
		return "NONE";

	stringstream ss;
	ss << idx;
	return ss.str();
}

template <typename uint_t>
void
KDTreeNode<uint_t>::dump(const vector<V3x>& arrPoints, ostream& out) const
{
	assert(arrPoints.size() > m_idxPoint);

	switch (m_axis) {
	case X_AXIS: out << "X AXIS"; break;
	case Y_AXIS: out << "Y AXIS"; break;
	case Z_AXIS: out << "Z AXIS"; break;
	}
	size_t idxPoint = static_cast<size_t>(m_idxPoint);
	out << ", POINT " << m_idxPoint << ": " << arrPoints[idxPoint] << "\n";
	out << "  CHILDREN: " << getIdxString(m_idxLeft) << " "
		<< getIdxString(m_idxRight) << "\n";
}

template <typename uint_t>
static inline uint_t
getChildSize(
	const vector<KDTreeNode<uint_t> >& arrNodes,
	uint_t idx)
{
	size_t idxNode = static_cast<size_t>(idx);
	return (idx == InvalidIndex<uint_t>::value) ?
		0 : arrNodes[idxNode].getSize(arrNodes);
}

template <typename uint_t>
static inline bool
sizesAreBalanced(uint_t leftSize, uint_t rightSize)
{
	return leftSize <= rightSize+1 && rightSize <= leftSize+1;
}

template <typename uint_t>
static inline bool
subtreeIsBalanced(
	uint_t size,
	uint_t idx,
	const vector<KDTreeNode<uint_t> >& arrNodes)
{
	size_t idxNode = static_cast<size_t>(idx);
	return (size == 0) ? true : arrNodes[idxNode].isBalanced(arrNodes);
}

template <typename uint_t>
bool
KDTreeNode<uint_t>::isBalanced(const KDTreeNodeList& arrNodes) const
{
	uint_t leftSize = getChildSize(arrNodes, m_idxLeft);
	uint_t rightSize = getChildSize(arrNodes, m_idxRight);
	if (!sizesAreBalanced(leftSize, rightSize))
		return false;

	return subtreeIsBalanced(leftSize, m_idxLeft, arrNodes)
		&& subtreeIsBalanced(rightSize, m_idxRight, arrNodes);
}

template <typename uint_t>
uint_t
KDTreeNode<uint_t>::getSize(const KDTreeNodeList& arrNodes) const
{
	return 1 + getChildSize(arrNodes, m_idxLeft) 
		+ getChildSize(arrNodes, m_idxRight);
}

template <typename uint_t>
bool
KDTreeNode<uint_t>::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	// TODO: implement
	UNUSED(point);
	UNUSED(out_result);
}

////////////////////////////////////////////////////////////////////////////////
// PointKDTreeImpl Methods
////////////////////////////////////////////////////////////////////////////////

#define KD_TREE_FITS_IN_IDX_SIZE(bits) \
	numPoints < numeric_limits<uint##bits##_t>::max()
#define KD_TREE_INIT_IDX_SIZE(bits) \
	m_pImpl##bits.reset(new PointKDTreeImplImpl<uint##bits##_t>(arrPoints)); \
	m_idxType = IDX_TYPE_##bits;

PointKDTreeImpl::PointKDTreeImpl(const vector<V3x>& arrPoints)
	: m_idxType(IDX_TYPE_INVALID)
{
	size_t numPoints = arrPoints.size();

	// Use the smallest possible index size
	if (KD_TREE_FITS_IN_IDX_SIZE(8)) {
		KD_TREE_INIT_IDX_SIZE(8)
	} else if (KD_TREE_FITS_IN_IDX_SIZE(16)) {
		KD_TREE_INIT_IDX_SIZE(16)
	} else if (KD_TREE_FITS_IN_IDX_SIZE(32)) {
		KD_TREE_INIT_IDX_SIZE(32)
	} else if (KD_TREE_FITS_IN_IDX_SIZE(64)) {
		KD_TREE_INIT_IDX_SIZE(64)
	} 
}

#define KD_TREE_IMPL_CALL_IMPL(prefix, call) \
	switch (m_idxType) { \
	case IDX_TYPE_8: prefix m_pImpl8->call; break; \
	case IDX_TYPE_16: prefix m_pImpl16->call; break; \
	case IDX_TYPE_32: prefix m_pImpl32->call; break; \
	case IDX_TYPE_64: prefix m_pImpl64->call; break; \
	default: assert(false && "internal KD tree error"); exit(1); \
	}

#define KD_TREE_IMPL_RETURN_CALL(call) \
	KD_TREE_IMPL_CALL_IMPL(return, call)

#define KD_TREE_NOTHING
#define KD_TREE_IMPL_CALL(call) \
	KD_TREE_IMPL_CALL_IMPL(KD_TREE_NOTHING, call)

bool
PointKDTreeImpl::isBalanced() const
{
	KD_TREE_IMPL_RETURN_CALL(isBalanced());
}

bool
PointKDTreeImpl::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	#define CALL_WITH_ARGS getClosestPoint(point, out_result)
	KD_TREE_IMPL_RETURN_CALL(CALL_WITH_ARGS)
	#undef CALL_WITH_ARGS
}

void
PointKDTreeImpl::dump(ostream& out) const
{
	#define CALL_WITH_ARGS dump(out)
	KD_TREE_IMPL_CALL(CALL_WITH_ARGS)
	#undef CALL_WITH_ARGS
}

////////////////////////////////////////////////////////////////////////////////
// PointKDTreeImplImpl Methods
////////////////////////////////////////////////////////////////////////////////

template <typename uint_t>
PointKDTreeImplImpl<uint_t>::PointKDTreeImplImpl(const vector<V3x>& arrPoints)
	: m_arrPoints(arrPoints)
{
	uint_t numPoints = static_cast<uint_t>(m_arrPoints.size());
	m_arrNodes.reserve(static_cast<size_t>(numPoints));
	buildTree(0, numPoints, X_AXIS);
}

template <typename uint_t>
uint_t
PointKDTreeImplImpl<uint_t>::partitionAroundMedian(
	uint_t idxBegin,
	uint_t idxEnd,
	KDTreeNodeAxis axis)
{
	uint_t halfSize = (idxEnd - idxBegin) / 2;
	uint_t idxMedian = idxBegin + halfSize;

	auto itGlobalBegin = begin(m_arrPoints);
	auto itBegin = itGlobalBegin + static_cast<size_t>(idxBegin);
	auto itMedian = itGlobalBegin + static_cast<size_t>(idxMedian);
	auto itEnd = itGlobalBegin + static_cast<size_t>(idxEnd);
	nth_element(itBegin, itMedian, itEnd, 
		[=](const V3x& lhs, const V3x& rhs) -> bool {
		return lhs[axis] < rhs[axis];
	});

	return idxMedian;
}

template <typename uint_t>
uint_t 
PointKDTreeImplImpl<uint_t>::buildTree(
	uint_t idxBegin,
	uint_t idxEnd,
	KDTreeNodeAxis axis)
{
	if (idxBegin >= idxEnd)
		return InvalidIndex<uint_t>::value;

	// Build Leaf Nodes
	uint_t size = idxEnd - idxBegin;
	if (size == 1) {
		m_arrNodes.emplace_back(KDTreeNode<uint_t>(
			idxBegin,
			InvalidIndex<uint_t>::value,
			InvalidIndex<uint_t>::value,
			axis));
		return static_cast<uint_t>(m_arrNodes.size()-1);
	}

	KDTreeNodeAxis nextAxis = static_cast<KDTreeNodeAxis>((axis + 1) % 3);
	uint_t idxMedian = partitionAroundMedian(idxBegin, idxEnd, axis);
	uint_t idxLeft = buildTree(idxBegin, idxMedian, nextAxis);
	uint_t idxRight = buildTree(idxMedian+1, idxEnd, nextAxis);

	m_arrNodes.emplace_back(
		KDTreeNode<uint_t>(idxMedian, idxLeft, idxRight, axis));
	return static_cast<uint_t>(m_arrNodes.size()-1);
}

template <typename uint_t>
void
PointKDTreeImplImpl<uint_t>::dump(ostream& out) const
{
	out << "== KD TREE IMPLEMENTATION ====\n";
	out << "POINT COUNT: " << m_arrPoints.size() << "\n";
	out << "NODE COUNT: " << m_arrNodes.size() << "\n\n";

	out << "-- NODES ----\n";
	auto itBegin = begin(m_arrNodes);
	auto itEnd = end(m_arrNodes);
	int idxNode = 0;
	for_each(itBegin, itEnd, [&](const KDTreeNode<uint_t>& node) {
		out << idxNode << ": ";
		node.dump(m_arrPoints, out);
		++idxNode;
	});

	out.flush();
}

template <typename uint_t>
bool
PointKDTreeImplImpl<uint_t>::isBalanced() const
{
	if (m_arrNodes.size() <= 2)
		return true;

	const KDTreeNode<uint_t>* pRoot = getRootNode();
	assert(pRoot != NULL);
	return pRoot->isBalanced(m_arrNodes);
}

template <typename uint_t>
const KDTreeNode<uint_t>*
PointKDTreeImplImpl<uint_t>::getRootNode() const
{
	if (m_arrNodes.empty())
		return NULL;
	return &m_arrNodes.back();
}

template <typename uint_t>
bool
PointKDTreeImplImpl<uint_t>::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	const KDTreeNode<uint_t>* pRootNode = getRootNode();
	if (pRootNode == NULL)
		return false;
	return pRootNode->getClosestPoint(point, out_result);
}


////////////////////////////////////////////////////////////////////////////////
// PointKDTree Methods
////////////////////////////////////////////////////////////////////////////////

PointKDTree::PointKDTree(const vector<V3x>& arrPoints)
	: m_pImpl(new PointKDTreeImpl(arrPoints))
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

	treeTimer.start();
	PointKDTree kdtree(arrPoints);
	treeTimer.stop();

	REQUIRE(kdtree.isBalanced());

	treeTimer.print();
}

unittest 
{
	cout << "\n";
	static const size_t NUM_POINTS = 32770;
	vector<V3x> arrPoints;
	fillPoints(arrPoints, NUM_POINTS);
	buildTree(arrPoints);
}

PointKDTree::~PointKDTree()
{
	delete m_pImpl;
}

void
PointKDTree::dump(ostream& out) const
{
	m_pImpl->dump(out);
}

bool
PointKDTree::isBalanced() const
{
	return m_pImpl->isBalanced();
}

bool
PointKDTree::getClosestPoint(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	return m_pImpl->getClosestPoint(point, out_result);
}

#pragma warning(pop)
