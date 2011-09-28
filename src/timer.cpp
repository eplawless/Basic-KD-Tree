#include "stdafx.h"
#include "timer.h"

////////////////////////////////////////////////////////////////////////////////
// Timer Methods
////////////////////////////////////////////////////////////////////////////////

Timer::Timer(const string& name, ostream& out)
	: m_name(name)
	, m_begin(0)
	, m_end(0)
	, m_elapsedClocks(0)
	, m_out(out)
{
}

void
Timer::start()
{
	m_begin = clock();
	m_end = 0;
}

void
Timer::stop()
{
	if (m_begin == 0 || m_end != 0)
		return;
	m_end = clock();
	m_elapsedClocks += m_end - m_begin;
}

void
Timer::print()
{
	m_out << "TIMER: \"" << m_name << "\"" << " took " << elapsedSeconds() 
		<< "s" << endl;
}

double
Timer::elapsedSeconds() const
{
	return static_cast<double>(m_elapsedClocks) / CLOCKS_PER_SEC;
}

void
Timer::reset()
{
	m_elapsedClocks = 0;
	m_begin = 0;
	m_end = 0;
}
