#include "stdafx.h"
#include "cmdPathIO.h"

CmdPathIO::CmdPathIO()
{

};


CmdPathIO::~CmdPathIO( void )
{
	
};	
ofstream & CmdPathIO::GetOutFile()
{
	return m_outfile;
};
ifstream & CmdPathIO::GetInFile()
{
	return m_infile;

};
	
int CmdPathIO::OpenOutFile(string pathfile)
{
	m_outfile.open ( pathfile.c_str(), ios::out | ios::trunc  );
	if (!m_outfile.is_open())
		return -1;
	
	return 0;
};
int CmdPathIO::OpenInFile(string pathfile)
{
	m_infile.open ( pathfile.c_str(), ios::in );
	if (!m_infile.is_open())
		return -1;
	
	return 0;
};
int CmdPathIO::CloseOutFile()
{
	m_outfile.close();
	return 0;
};
int CmdPathIO::CloseInFile()
{
	m_infile.clear();
	m_infile.close();
	return 0;
};


int CmdPathIO::WriteCmd(enCommand enCmd)
{
	if ( (enCmd == enStartPath)|| (enCmd == enEndPath) )
		return 0;

	if (m_outfile.is_open())
		m_outfile << enCmd  << endl;

	return 0;
};
int CmdPathIO::WriteXYZ	(double x, double y, double z)
{
	if (m_outfile.is_open())
		m_outfile << x << '\t' << y << '\t' << z <<  endl;
	
	return 0;	
};
int CmdPathIO::ReadXYZ	(double & x, double & y, double & z)
{
	
	if (m_infile.is_open())
		m_infile >> x >> y >> z ;
	else
		return -1;
	return 0;	
};
int CmdPathIO::ReadCmd	 (enCommand & enCmd)
{
	int cmd;

	if (!m_infile.is_open())
	{
		return enFileNotOpen;
	}
	m_infile >> cmd;
	
	if (!m_infile.eof())
	{
		enCmd=(enCommand)cmd;
	}
	else
	{
		return EOF;
	}

	
	return 0;	
};
