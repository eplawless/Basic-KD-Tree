#pragma once
#ifndef EPL_UNCOPYABLE_H_
#define EPL_UNCOPYABLE_H_

class Uncopyable {
public: // methods
	Uncopyable() {}

private: // methods
	Uncopyable(const Uncopyable&);
	Uncopyable& operator=(const Uncopyable&);
};

#endif // EPL_UNCOPYABLE_H_
