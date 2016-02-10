#ifndef _ARCHIVE_H
#define _ARCHIVE_H
#include <iostream>
#include <fstream>
using namespace std;

class Archive
{
private:
	bool m_IsReading;

public:
	Archive(): m_IsReading(false){};
	bool IsReading();
	bool IsStoring();
	void SetIsReading();
	void SetIsStoring();
	
	fstream m_file;

	int OpenWriteFile(string pathfile);
	int OpenReadFile(string pathfile);
	int CloseFile();

};
#endif
