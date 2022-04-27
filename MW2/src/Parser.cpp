//#####################################################################
// Copyright 2010-2015 Sung-Hee Lee.
// This file is part of MotionWorks whose distribution is governed by the license contained in the accompanying file MotionWorks_COPYRIGHT.txt.
//#####################################################################

#include "Loader/Parser.h"


int Parser::readInteger(char* buf)
{
	read_word(buf);
	return atoi(buf);
}

gReal Parser::readReal(char* buf)
{
	read_word(buf);
	return gReal(atof(buf));
}

bool Parser::read_word_s(char* buf, int buf_size)
{
	int ch, i = 0;
	do {
		ch = fgetc(_file);
		if ( ch == EOF ) return false;
	} while ( ch == '\0' || ch == '\t' || ch == '\n' || ch == ' ' );
	do {
		buf[i++] = (char)ch;
		ch = fgetc(_file);
	} while ( ch != '\0' && ch != '\t' && ch != '\n' && ch!=' ' && ch != EOF && i < buf_size );
	buf[i] = '\0'; 
	return true;
}

gVec3 Parser::readVec3()
{
	gVec3 v;
	char buf[64];
	v.setX(readReal(buf));
	v.setY(readReal(buf));
	v.setZ(readReal(buf));
	return v;
}

int Parser::open_file	(const char* filename)
{ 
	_file = fopen(filename,"r"); 
	if(!_file) return 0; else return 1; 
}

void Parser::close_file	(void)			
{ 
	if(_file) fclose(_file); 
	_file = 0; 
}

//0: success , -1: EOF
int	Parser::read_word	(char* buf)
{
	int ch, i = 0;
	do {
		ch = fgetc(_file);
		if ( ch == EOF ) return -1;
	} while ( ch == '\0' || ch == '\t' || ch == '\n' || ch == ' ' );
	do {
		buf[i++] = (char)ch;
		ch = fgetc(_file);
	} while ( ch != '\0' && ch != '\t' && ch != '\n' && ch!=' ' && ch != EOF );
	buf[i] = '\0'; 

	//printf("%s", buf);

	return 0;
}

//0: success , -1: EOF
int Parser::go_after(char* str)
{
	char buf[128];
	do {
		if( read_word(buf) != 0 ) return -1;
    } while ( strcmp(buf,str)!= 0 );
	return 0;
}

// read a line from file
// 0: success, -1: EOF
int Parser::read_line(char* buf)
{
	int ch, i = 0;
	do{
        ch = fgetc(_file);
		if(ch==EOF) return -1;
	} while ( ch =='\0' || ch == '\t' || ch == '\n' || ch == ' ' );
	do{
		buf[i++] = (char)ch;
		ch = fgetc(_file);			
	}while( ch != '\n' && ch!= EOF );
	buf[i]= '\n';
	return 0;
}

// read a line from file
// 0: success, -1: EOF
int Parser::read_line_into_string(char* buf)
{
	int ch, i = 0;
	do{
        ch = fgetc(_file);
		if(ch==EOF) return -1;
	} while ( ch =='\0' || ch == '\t' || ch == '\n' || ch == ' ' );
	do{
		buf[i++] = (char)ch;
		ch = fgetc(_file);			
	}while( ch != '\n' && ch!= EOF );
	buf[i]= '\0';
	return 0;
}
