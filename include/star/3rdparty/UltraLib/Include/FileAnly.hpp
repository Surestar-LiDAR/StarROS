/*-----------------------------------------------------------
	    Author:		           ZQW	
	    File Name:	           FileAnly
	    File Type:	           hpp
        Created Date:	       2008/08/19
	    Created Time:	       19:8:2008   21:35
*----------------------------------------------------------*/
#ifndef FILEANLY_H_ZQW_2008_07_23_15_39_54921
#define FILEANLY_H_ZQW_2008_07_23_15_39_54921

#include "BasicDef.h"
#include <string>
#include <vector>
#include <algorithm> 
#include <functional> 

using namespace::std;

/************************************************************************/
/*
//所有的后缀操作都不包含'.';
GetExt ( lpstrExt );    获取后缀名；
GetName( lpstrName );   获取文件名称,不含后缀；
GetFileName( lpstr );   获取文件名称,含后缀;
GetDir ( lpstrDir );    获取文件所在目录；
CFileAnly::TravalCurtDir( LPCSTR lpstrDir,LPCSTR lpExt,vector<string>& StrPathArray ) 获取当前目录下指定文件后缀所有文件
CFileAnly::TravalAllDir( LPCSTR lpstrDir,LPCSTR lpExt,vector<string>& StrPathArray )  获取当前目录下指定文件后缀所有文件（包括子目录）
*/
/************************************************************************/
namespace nsFileAnly
{
	enum eFileAttribute{eNormal=FILE_ATTRIBUTE_NORMAL,eHide=FILE_ATTRIBUTE_HIDDEN,eReadOldy=FILE_ATTRIBUTE_READONLY,};

	static BOOL GetFileDir(const char* lpstrPath,char* pStrDir)
	{
		*pStrDir='\0';
		if(lpstrPath)
		{
			strcpy(pStrDir,lpstrPath);
			*(strrchr(pStrDir,'\\'))='\0';
			return TRUE;
		}		   
		return FALSE;
	}
	static BOOL GetFileName(const char* lpstrPath,char* pStrName)
	{
		*pStrName='\0';
		if(lpstrPath)
		{
			sprintf(pStrName,"%s",strrchr(lpstrPath,'\\')+1);
			return TRUE;
		}
		return FALSE;
	}
	static BOOL GetName(const char* lpstrPath,char* pStrName)
	{
		*pStrName='\0';
		if(GetFileName(lpstrPath,pStrName))
		{
			const char *p=strrchr(pStrName,'.');
			if(p)
			{
				*(strrchr(pStrName,'.'))='\0';
				return TRUE;
			}
		}
		return FALSE;
	}
	static BOOL GetFileExt(const char* lpstrPath,char* pStrExt)
	{
		*pStrExt='\0';
		if(lpstrPath)
		{
			const char *p=strrchr(lpstrPath,'.');
			if(p)
			{
				sprintf(pStrExt,"%s",p+1);				
				return TRUE;
			}			
		}
		return FALSE;		
	}	
	static BOOL ReplaceExt(const char* lpstrOldPath,const char* lpstrNewExt,char* pStrNewPath)
	{
		*pStrNewPath='\0';
		if(lpstrOldPath&&lpstrNewExt)
		{
			strcpy(pStrNewPath,lpstrOldPath);
			strcpy(strrchr(pStrNewPath,'.')+1,lpstrNewExt);
			return TRUE;
		}
		return FALSE;
	}
	static BOOL CompareExt(const char* lpstrPath,const char* pStrExt)
	{
		if(lpstrPath&&pStrExt)
		{
			char strExt[PATHLEN];
			if(GetFileExt(lpstrPath,strExt))
			{
				if(strcmpi(pStrExt,strExt)==0){return TRUE;}
			}			
		}
		return FALSE;
	}
	static BOOL GetFileProperty(const char* lpstrPath,SYSTEMTIME *pLastWrite=NULL,SYSTEMTIME *pAccess=NULL,SYSTEMTIME *pCreation=NULL,INT64 *pLength=NULL)
	{
		BOOL bExist=TRUE; FILETIME Creation,Access,LastWrite;
		HANDLE hFile=::CreateFile(lpstrPath,GENERIC_READ|GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);
		if(hFile!=INVALID_HANDLE_VALUE)
		{
			::GetFileTime(hFile,&Creation,&Access,&LastWrite);			
			if(pCreation ){::FileTimeToSystemTime(&Creation,pCreation);}
			if(pAccess   ){::FileTimeToSystemTime(&Access,pAccess);}
			if(pLastWrite){::FileTimeToSystemTime(&LastWrite,pLastWrite);}
			if(pLength   ){::GetFileSizeEx(hFile,PLARGE_INTEGER(pLength));}

			::CloseHandle(hFile);
		}
		else{bExist=FALSE;}

		return bExist;
	}
	static void SetFileProperty(const char* lpstrPath,eFileAttribute eMode=eHide)
	{							   
		::SetFileAttributes(lpstrPath,eHide);
	}
	static int TravalDirectory(const char* lpstrDir,const char* lpExt,vector<string>& StrPathArray,BOOL bOnlyCurt=TRUE)
	{	
		char srcPath[PATHLEN]={'\0'};
		sprintf(srcPath,"%s\\%s",lpstrDir,"*.*");

		HANDLE hFind = NULL;
		WIN32_FIND_DATA dat;    
		hFind=::FindFirstFile(srcPath,&dat);
		if(hFind==INVALID_HANDLE_VALUE) return 0;		
		do
		{
			sprintf(srcPath,"%s\\%s",lpstrDir,dat.cFileName);	
			if(strcmpi( dat.cFileName,".")==0)  continue;
			if(strcmpi( dat.cFileName,"..")==0) continue;
			if(strlen(srcPath)<3)  continue;
		

			if(bOnlyCurt){if((dat.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)==FILE_ATTRIBUTE_DIRECTORY) continue;}
			else
			{
				if((dat.dwFileAttributes&FILE_ATTRIBUTE_DIRECTORY)==FILE_ATTRIBUTE_DIRECTORY)
				{
					TravalDirectory(srcPath,lpExt,StrPathArray,FALSE);
				}
			}			
			char strType[PATHLEN]={'\0'}; 
			GetFileExt(srcPath,strType);		
			if(strcmpi(strType,lpExt)==0){StrPathArray.push_back(srcPath);}
							
		}while(::FindNextFile(hFind,&dat));	
		return 1;
	}
	static void ClearDirectoryFile(const char* lpstrDir,const char* lpstrExt)
	{
		vector<string> strList;
		nsFileAnly::TravalDirectory(lpstrDir,lpstrDir,strList);
		if(!strList.empty())
		{
			int i,nNum=strList.size();
			for(i=0;i<nNum;i++)
			{
				::DeleteFile(strList[i].c_str());
			}
		}
	}
};

#endif