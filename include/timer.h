#pragma once
#ifndef EPL_TIMER_H_
#define EPL_TIMER_H_

#include "stdafx.h"

class Timer : public Uncopyable
{
public: // methods
	Timer(const string& name, ostream& out = cout);

	void reset();
	void start();
	void stop();
	void print();

	double elapsedSeconds() const;

private: // members
	ostream& m_out;
	string m_name;
	size_t m_elapsedClocks;
	clock_t m_begin;
	clock_t m_end;
};

#endif // EPL_TIMER_H_
