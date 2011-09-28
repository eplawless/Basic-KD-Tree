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
using namespace std;

// C Includes
#include <ctime>
#include <cassert>

// OS Includes
#include <Windows.h>

// Utility Includes
#include "uncopyable.h"
#define UNUSED(a)

#endif // EPL_STDAFX_H_
