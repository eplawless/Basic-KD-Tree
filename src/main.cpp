#include "stdafx.h"
#include "kdtree.h"

#pragma warning(push, 4)

int WINAPI 
wWinMain(HINSTANCE, HINSTANCE, PWSTR, int)
{
	// Fill Points
	vector<V3x> arrPoints(10);
	int count = 0;
	generate(begin(arrPoints), end(arrPoints), [&]() -> V3x {
		return V3x(++count);
	});

	KDTree kdtree(arrPoints);
}

int main() {
	return 0;
}

#pragma warning(pop)
