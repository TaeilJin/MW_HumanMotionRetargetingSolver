//#####################################################################
// Copyright 2010-2015, Sukwon Lee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################
#include "MocapProcessor/mgData.h"

#include <cmath>
#include <ios>
#include <fstream>

void mgData::deallocMemory()
{
	if( m_owner && motions ) delete [] motions;
	if( m_owner && pMotions ) delete [] pMotions;
}

void mgData::allocMemory( const int frame, const int channel )
{
	deallocMemory();

	nMotion = frame;
	nChannel = channel;

	int len = channel * frame;
	pMotions = new CoordinateType[len];
	
	motions = new CoordinateType*[nMotion];
	for( unsigned int row=0; row < nMotion; row++ )
	{
		motions[row] = &( pMotions[ row * channel ] );
	}
}

void mgData::allocMemory()
{
	allocMemory( nMotion, nChannel );
}

//static double round(double number)
//{
//    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
//}

void mgData::sampler(CoordinateType* data, const double frame, SAMPLING_METHOD method)
{
	switch(method)
	{
	case NEAREST_NEIGHBOR:
		{
			int f = round(frame);
			if( f >= nMotion )
			{
				f = nMotion - 1;
			}
			memcpy(data, motions[f], sizeof( CoordinateType ) * nChannel);
		}
		break;
	case LINEAR:
		{
			int f = round(frame);
			double t = frame - f;
			int n = (t < 0.0)? f + 1: f - 1;


		}
		break;
	}
}

void mgData::save(std::ostream& os, const FILE_TYPE type)
{
	switch (type)
	{
	case BINARY:
		{
			os.write(reinterpret_cast<const char*>(pMotions), sizeof(CoordinateType) * nMotion * nChannel);
		}
		break;

	case CSV:
		{
			const std::ios::fmtflags orig_flags = os.flags();
  
			// TODO: need sane values for complex numbers
			os.setf(std::ios::scientific, std::ios::fixed);
			os.unsetf(std::ios::floatfield);
			os.precision(8);
  
			unsigned int x_n_rows = nMotion;
			unsigned int x_n_cols = nChannel;
  
			for(unsigned int row=0; row < x_n_rows; ++row)
			{
				for(unsigned int col=0; col < x_n_cols; ++col)
				{
					//arma_ostream::print_elem(f, x.at(row,col), false);
      
					os << motions[row][col];

					if( col < (x_n_cols-1) )
					{
						os.put(',');
					}
				}
				os.put('\n');
			}
			os.flags(orig_flags);
		}
		break;

	case TAB:
		{
			const std::ios::fmtflags orig_flags = os.flags();
  
			// TODO: need sane values for complex numbers
			os.setf(std::ios::scientific);
			os.precision(12);
  
			unsigned int x_n_rows = nMotion;
			unsigned int x_n_cols = nChannel;
  
			for(unsigned int row=0; row < x_n_rows; ++row)
			{
				for(unsigned int col=0; col < x_n_cols; ++col)
				{
					//arma_ostream::print_elem(f, x.at(row,col), false);
      
					os << motions[row][col];

					if( col < (x_n_cols-1) )
					{
						os.put('\t');
					}
				}
				os.put('\n');
			}
			os.flags(orig_flags);
		}
		break;

	default:
		break;
	}
}

void mgData::save(const char* filename, const FILE_TYPE type)
{
	std::fstream f;
	f.open(filename, std::fstream::out);
	
	save(f, type);

	f.close();
}