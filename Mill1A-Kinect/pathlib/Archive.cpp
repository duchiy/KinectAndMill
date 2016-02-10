#include "stdafx.h"
#include "Archive.h"
bool Archive::IsReading()
{
	if (m_IsReading)
		return true;
	else
		return false;
}
bool Archive::IsStoring()
{

	if (!m_IsReading)
		return true;
	else
		return false;

}
void Archive::SetIsReading()
{
	m_IsReading=true;
};
void Archive::SetIsStoring()
{
	m_IsReading=false;
};
int Archive::OpenWriteFile(string pathfile)
{
	m_IsReading=false;
	m_file.open ( pathfile.c_str(), ios::out | ios::trunc);
	if (!m_file.is_open())
		return -1;

	return 0;
};
int Archive::OpenReadFile(string pathfile)
{
	m_IsReading=false;
	m_file.open ( pathfile.c_str(), ios::in );
	if (!m_file.is_open())
		return -1;

	
	return 0;
};
int Archive::CloseFile()
{
	m_file.close();
	return 0;
};
