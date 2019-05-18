/*
  	RingCounter.h - A header file that defines a class of a modular arithmetic
	 counter.

    Copyright (C) 2010  Arturo Espinosa-Romero (arturoemx@gmail.com)
    Facultad de Matemáticas, Universidad Autónoma de Yucatán, México.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __RINGCOUNTER__
#define __RINGCOUNTER__

#include <cstdlib>

class RingCounter
{

	size_t C;										 //size_t is the type returned by the sizeof operator and is
	size_t RingSize;						//widely used in the standard library to represent sizes and counts.
	public:

	RingCounter ()
	{
		RingSize = 0;
		C = 0;
	}

	RingCounter (size_t rz)
	{

		RingSize = rz;
		C = 0;
	}

	RingCounter (const RingCounter &R)
	{
		C = R.C;
		RingSize = R.RingSize;
	}

	void SetRingSize(size_t val)
	{
		RingSize = val;
	}

	operator size_t()
	{
		return C;
	}

	RingCounter & operator = (const size_t &v)
	{
		C = v % RingSize;
		return *this;
	}

	RingCounter & operator = (const RingCounter &R)
	{
		C = R.C;
		RingSize = R.RingSize;
		return *this;
	}

	template < typename X >
	RingCounter  operator + (const X & n)
	{
		RingCounter R (RingSize);

		R.C = (C + (size_t)n) % RingSize;
		return R;
	}

	template < typename X >
	RingCounter  operator - (const X & n)
	{
		RingCounter R (RingSize);
		int diff = (int)C - (int)n;

		if ( diff > 0 )
			R.C = (C - n) % RingSize;
		else
			R.C = RingSize - (-diff % RingSize);
		return R;
	}

	template < typename X>
	RingCounter & operator += (X & val)
	{
		C = (C + (size_t)val) % RingSize;
		return *this;
	}

	template < typename X >
	RingCounter & operator -= (const X & n)
	{
		int diff = (int)C - (int)n;

		if ( diff > 0 )
			C = (C - n) % RingSize;
		else
			C = RingSize - (-diff % RingSize);
		return *this;
	}

	RingCounter &operator ++()
	{
		C = (C + 1) % RingSize;
		return *this;
	}

	RingCounter &operator --()
	{
		C = !C ? RingSize - 1 : (C - 1) % RingSize;
		return *this;
	}

	RingCounter &operator ++(int)
	{
		C = (C + 1) % RingSize;
		return *this;
	}

	RingCounter &operator --(int)
	{
		C = !C ? RingSize - 1 : (C - 1) % RingSize;
		return *this;
	}

	bool operator == (const RingCounter &R)
	{
		if ( C == R.C )
			return true;
		return false;
	}

	bool operator != (const RingCounter &R)
	{
		if ( C != R.C )
			return true;
		return false;
	}

/*	bool operator != (int comp)
	{
		if ( R.C != temp )
			return true;
		return false;
	}*/

	bool operator > (const RingCounter &R)
	{
		if ( C > R.C )
			return true;
		return false;
	}

	bool operator >= (const RingCounter &R)
	{
		if (C > R.C && RingSize == R.RingSize)
			return true;
		return false;
	}

	size_t getC()
	{
		return C;
	}
};


#endif
