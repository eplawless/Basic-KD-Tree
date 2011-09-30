#include "stdafx.h"
#include "kdtree.h"

#pragma warning(push, 4)

/// @{
/// Helper Macros for Dynamic Index Precision
/// NOTE: these helpers must all be changed to add/remove an index precision
#define KD_TREE_EMPTY_ARGUMENT
#define KD_TREE_FOREACH_IDX_SIZE(macro) \
	macro(8) \
	macro(16) \
	macro(32) \
	macro(64)
#define KD_TREE_FOREACH_IDX_SIZE_ARG1(macro, arg) \
	macro(8, arg) \
	macro(16, arg) \
	macro(32, arg) \
	macro(64, arg)
#define KD_TREE_FOREACH_IDX_SIZE_ARG2(macro, arg1, arg2) \
	macro(8, arg1, arg2) \
	macro(16, arg1, arg2) \
	macro(32, arg1, arg2) \
	macro(64, arg1, arg2)
/// @}

/// @{
/// Invalid Index
template <typename uint_t>
struct InvalidIndex {
	static const uint_t value = 0;
};

#define KD_TREE_INVALID_IDX(bits) \
	template <> \
	struct InvalidIndex<uint##bits##_t> { \
		static const uint##bits##_t value = UINT##bits##_MAX; \
	};

KD_TREE_FOREACH_IDX_SIZE(KD_TREE_INVALID_IDX)
/// @}

/// KD Tree Node
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
		KDTreeAxis axis);

	uint_t			getIdxPoint() const { return m_idxPoint; }
	KDTreeAxis	getAxis() const		{ return m_axis; }
	uint_t			getIdxLeft()		{ return m_idxLeft; }
	uint_t			getIdxRight()		{ return m_idxRight; }

	bool getClosestPointTo(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	uint_t			getSize(const KDTreeNodeList& arrNodes) const;
	bool			isBalanced(const KDTreeNodeList& arrNodes) const;

	void dump(const vector<V3x>& arrPoints, ostream& out) const;

private: // members
	KDTreeAxis m_axis;
	uint_t m_idxPoint;
	uint_t m_idxLeft;
	uint_t m_idxRight;
};

#define DEFINE_IDX_TYPE(bits) \
	IDX_TYPE_##bits,

enum KDTreeIndexType
{
	KD_TREE_FOREACH_IDX_SIZE(DEFINE_IDX_TYPE)
	IDX_TYPE_INVALID
};

/// KD Tree Actual Implementation 
template <typename uint_t>
class PointKDTreeImplImpl 
{
public: // types
	typedef vector<KDTreeNode<uint_t> > KDTreeNodeList;

public: // methods
	PointKDTreeImplImpl(const vector<V3x>& arrPoints);

	uint_t buildTree(
		uint_t idxBegin,
		uint_t idxEnd);

	KDTreeAxis getSplitAxis(
		uint_t idxBegin,
		uint_t idxEnd) const;

	bool isBalanced() const;
	bool getClosestPointTo(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;

	void dump(ostream& out = cerr) const;

private: // methods

	const KDTreeNode<uint_t>* getRootNode() const;

	inline uint_t partitionAroundMedian(
		uint_t idxBegin,
		uint_t idxEnd,
		KDTreeAxis axis);

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
	bool getClosestPointTo(
		const V3x& point,
		KDTreeClosestPoint& out_result) const;
	void dump(ostream& out) const;

private: // members
	KDTreeIndexType m_idxType;

#define KD_TREE_IMPL_MEMBER_PTR(bits) \
	const unique_ptr<PointKDTreeImplImpl<uint##bits##_t> > m_pImpl##bits;

	KD_TREE_FOREACH_IDX_SIZE(KD_TREE_IMPL_MEMBER_PTR)
};

////////////////////////////////////////////////////////////////////////////////
// KDTreeNode Methods
////////////////////////////////////////////////////////////////////////////////

template <typename uint_t>
KDTreeNode<uint_t>::KDTreeNode(
	uint_t idxPoint,
	uint_t idxLeft,
	uint_t idxRight,
	KDTreeAxis axis)
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
	ss << static_cast<size_t>(idx);
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
	out << ", POINT " << idxPoint << ": " << arrPoints[idxPoint] << "\n";
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
KDTreeNode<uint_t>::getClosestPointTo(
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

#define KD_TREE_IDX_SIZE_IS_ENOUGH(bits) \
	numPoints < numeric_limits<uint##bits##_t>::max()
#define KD_TREE_INIT_IMPL(bits) \
	if (KD_TREE_IDX_SIZE_IS_ENOUGH(bits)) { \
		const_cast<unique_ptr<PointKDTreeImplImpl<uint##bits##_t> >&> \
			(m_pImpl##bits).reset( \
				new PointKDTreeImplImpl<uint##bits##_t>(arrPoints)); \
		m_idxType = IDX_TYPE_##bits; \
		return; \
	}

PointKDTreeImpl::PointKDTreeImpl(const vector<V3x>& arrPoints)
	: m_idxType(IDX_TYPE_INVALID)
{
	size_t numPoints = arrPoints.size();
	KD_TREE_FOREACH_IDX_SIZE(KD_TREE_INIT_IMPL)
}

#define KD_TREE_IMPL_CALL_HELPER(bits, prefix, call) \
	case IDX_TYPE_##bits: prefix m_pImpl##bits->call; break; \

#define KD_TREE_IMPL_CALL_IMPL(prefix, call) \
	switch (m_idxType) { \
	KD_TREE_FOREACH_IDX_SIZE_ARG2(KD_TREE_IMPL_CALL_HELPER, prefix, call) \
	default: assert(false && "internal KD tree error"); exit(1); \
	}

#define KD_TREE_IMPL_CALL_RETURN(call) \
	KD_TREE_IMPL_CALL_IMPL(return, call)

#define KD_TREE_IMPL_CALL(call) \
	KD_TREE_IMPL_CALL_IMPL(KD_TREE_EMPTY_ARGUMENT, call)

bool
PointKDTreeImpl::isBalanced() const
{
	KD_TREE_IMPL_CALL_RETURN(isBalanced());
}

bool
PointKDTreeImpl::getClosestPointTo(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	#define CLOSEST_POINT_WITH_ARGS getClosestPointTo(point, out_result)
	KD_TREE_IMPL_CALL_RETURN(CLOSEST_POINT_WITH_ARGS)
	#undef CLOSEST_POINT_WITH_ARGS
}

void
PointKDTreeImpl::dump(ostream& out) const
{
	KD_TREE_IMPL_CALL(dump(out))
}

////////////////////////////////////////////////////////////////////////////////
// PointKDTreeImplImpl Methods
////////////////////////////////////////////////////////////////////////////////

template <typename uint_t>
PointKDTreeImplImpl<uint_t>::PointKDTreeImplImpl(
	const vector<V3x>& arrPoints)
	: m_arrPoints(arrPoints)
{
	uint_t numPoints = static_cast<uint_t>(m_arrPoints.size());
	m_arrNodes.reserve(m_arrPoints.size());
	buildTree(0, numPoints);
}

template <typename uint_t>
uint_t
PointKDTreeImplImpl<uint_t>::partitionAroundMedian(
	uint_t idxBegin,
	uint_t idxEnd,
	KDTreeAxis axis)
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
KDTreeAxis
PointKDTreeImplImpl<uint_t>::getSplitAxis(
	uint_t idxBegin,
	uint_t idxEnd) const
{
	assert(idxBegin < idxEnd);

	if (idxBegin+1 == idxEnd)
		return X_AXIS;

	V3x firstPoint = m_arrPoints[static_cast<size_t>(idxBegin)];
	fpreal xMin = firstPoint.x;
	fpreal yMin = firstPoint.y;
	fpreal zMin = firstPoint.z;
	fpreal xMax = firstPoint.x;
	fpreal yMax = firstPoint.y;
	fpreal zMax = firstPoint.z;

	auto itGlobalBegin = begin(m_arrPoints);
	auto itBegin = itGlobalBegin + static_cast<size_t>(idxBegin) + 1;
	auto itEnd = itGlobalBegin + static_cast<size_t>(idxEnd);
	for_each(itBegin, itEnd, [&](const V3x& point) {
		xMin = min<fpreal>(point.x, xMin);
		yMin = min<fpreal>(point.y, yMin);
		zMin = min<fpreal>(point.z, zMin);
		xMax = max<fpreal>(point.x, xMax);
		yMax = max<fpreal>(point.y, yMax);
		zMax = max<fpreal>(point.z, zMax);
	});

	fpreal xSize = xMax - xMin;
	fpreal ySize = yMax - yMin;
	fpreal zSize = zMax - zMin;

	return (ySize > xSize && ySize > zSize) ? Y_AXIS :
		(zSize > xSize && zSize > ySize) ? Z_AXIS :
			X_AXIS;
}

template <typename uint_t>
uint_t 
PointKDTreeImplImpl<uint_t>::buildTree(
	uint_t idxBegin,
	uint_t idxEnd)
{
	if (idxBegin >= idxEnd)
		return InvalidIndex<uint_t>::value;

	// TODO: use better split heuristic
	KDTreeAxis axis = getSplitAxis(idxBegin, idxEnd);

	// Build Leaf Node
	uint_t size = idxEnd - idxBegin;
	if (size == 1) {
		m_arrNodes.emplace_back(KDTreeNode<uint_t>(
			idxBegin,
			InvalidIndex<uint_t>::value,
			InvalidIndex<uint_t>::value,
			axis));
		return static_cast<uint_t>(m_arrNodes.size()-1);
	}

	// Recurse
	uint_t idxMedian = partitionAroundMedian(idxBegin, idxEnd, axis);
	uint_t idxLeft = buildTree(idxBegin, idxMedian);
	uint_t idxRight = buildTree(idxMedian+1, idxEnd);

	// Build Internal Node
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
PointKDTreeImplImpl<uint_t>::getClosestPointTo(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	const KDTreeNode<uint_t>* pRootNode = getRootNode();
	if (pRootNode == NULL)
		return false;
	return pRootNode->getClosestPointTo(point, out_result);
}

////////////////////////////////////////////////////////////////////////////////
// PointKDTree Methods
////////////////////////////////////////////////////////////////////////////////

PointKDTree::PointKDTree(const vector<V3x>& arrPoints)
	: m_pImpl(new PointKDTreeImpl(arrPoints))
{
}

PointKDTree::~PointKDTree()
{
	// NOTE: To allow ~unique_ptr() to see the complete PointKDTreeImpl type 
	//       we have to create an explicit destructor for PointKDTree here,
	//       after PointKDTreeImpl been defined.
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
PointKDTree::getClosestPointTo(
	const V3x& point,
	KDTreeClosestPoint& out_result) const
{
	return m_pImpl->getClosestPointTo(point, out_result);
}

#pragma warning(pop)
