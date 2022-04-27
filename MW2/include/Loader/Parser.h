//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#ifndef _MG_PARSER_H
#define _MG_PARSER_H

#include "Base/gBase.h"
#include "Base/gMath.h"
/**
    @class		Parser
    @date		2014/02/18
    @author		Sukwon Lee(sukwonlee@kaist.ac.kr)
    @brief		a basic string parser. The base code is copied from gParser of MBS.
    @warning	
*/

class Parser
{
public:	
					 Parser()					{ _file = NULL; }
	virtual			~Parser()					{ close_file();}
	
	int open_file	(const char* filename);
	void close_file	(void);

	//0: success , -1: EOF
	int	read_word	(char* buf);

	//0: success , -1: EOF
	int go_after(char* str);

	// read a line from file. append '\n' at end
	// 0: success, -1: EOF
	int read_line(char* buf);

	// same as read_line, but append '\0' at end
	int read_line_into_string(char* buf);

	inline FILE*   file(void) { return _file; }

	// Helper
	int		 readInteger(char* buf);
	gReal	 readReal(char* buf);
	gVec3	 readVec3();

	bool read_word_s(char* buf, int buf_size);

protected:

	FILE			*_file; ///< file handler.
};
#endif