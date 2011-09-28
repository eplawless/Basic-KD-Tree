#pragma once
#ifndef EPL_STDAFX_H_
#define EPL_STDAFX_H_

// ILM Math Library
#include <ImathVec.h>
#include <ImathVecAlgo.h>
#include <ImathMatrix.h>
#include <ImathMatrixAlgo.h>
using namespace Imath;

// Math Typedefs
typedef double fpreal;
typedef Vec2<fpreal> V2x;
typedef Vec3<fpreal> V3x;
typedef Vec4<fpreal> V4x;

// STL Includes
#include <iostream>
#include <algorithm>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <iterator>
#include <limits>
using namespace std;

// C Includes
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cassert>

// OS Includes
#include <Windows.h>

// Utility Includes
#include "uncopyable.h"
#include "timer.h"
#include "UnitTest.h"
#define UNUSED(a)

#undef min
#undef max

#endif // EPL_STDAFX_H_
