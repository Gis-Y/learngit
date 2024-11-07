#pragma once
#include <string>

class Alpha
{
public:
	// constructors/destructor
	Alpha() : letter(0) {}
	Alpha(const char _letter ) : letter(_letter) {}
	~Alpha() {}

	// interface
	friend bool operator < (const Alpha& lhs, const Alpha& rhs) 
	{ return lhs.getLetter() < rhs.getLetter(); }
	friend bool operator == (const Alpha& lhs, const Alpha& rhs)
	{ return lhs.getLetter() == rhs.getLetter(); }
	char getLetter() const { return letter; }

private:
	// data
	char letter;
};

