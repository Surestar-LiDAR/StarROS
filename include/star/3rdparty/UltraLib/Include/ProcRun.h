/********************************************************************
       Author:		           ZQW	
       File Name:	           ProcRun
	   File Type:	           h
       Created Date:	       2009/4/1
	   Created Time:	       18:07      
*********************************************************************/
#ifndef ProcRun_h_ZQW_2009_4_1_18_07_DEF
#define ProcRun_h_ZQW_2009_4_1_18_07_DEF

#include "FileAnly.hpp"
/************************************************************************/
/*
BOOL RunOpenFile( LPSTR lpstrFilePath )
BOOL RunProcess( LPCTSTR lpstrExePath,LPSTR lpstrCmdLine )
BOOL WaitRunProcess( LPCTSTR lpstrExePath,LPSTR lpstrCmdLine )
*/
/************************************************************************/
namespace nsRunProcess
{
	inline static BOOL RunOpenFile( LPCSTR lpstrFilePath )
	{
		if( strlen( lpstrFilePath )<4 ) return FALSE;
		char strCmd[_MAX_PATH];memset( strCmd,0,_MAX_PATH );	
		char szWinSys[_MAX_PATH];GetWindowsDirectory( szWinSys, _MAX_PATH );
		sprintf( strCmd,"%s\\NotePad.exe  %s", szWinSys,lpstrFilePath );
		::WinExec( strCmd,SW_SHOW );
		
		return TRUE;
	}
	
	inline static BOOL RunProcess( LPCSTR lpstrExePath,LPCSTR lpstrCmdLine )
	{
		if( strlen( lpstrExePath )<4 ) return FALSE;
		char strCmd[512];memset( strCmd,0,512 );
		if( lpstrCmdLine ){ sprintf( strCmd,"%s  %s",lpstrExePath,lpstrCmdLine );}
		else{ strcpy( strCmd,lpstrExePath ); };
		
		PROCESS_INFORMATION stProcessInfo;    
		STARTUPINFO stStartUpInfo; 
		memset(&stStartUpInfo, 0, sizeof(STARTUPINFO));
		stStartUpInfo.cb            = sizeof(STARTUPINFO);
		stStartUpInfo.dwFlags		= STARTF_USESHOWWINDOW;
		stStartUpInfo.wShowWindow	= SW_SHOW ;	
		
		BOOL callSuccess = ::CreateProcess( NULL,strCmd,NULL,NULL,FALSE,NORMAL_PRIORITY_CLASS,NULL,NULL,&stStartUpInfo,&stProcessInfo ); 
		if(!callSuccess)
		{
			::CloseHandle( stProcessInfo.hThread );
			::CloseHandle( stProcessInfo.hProcess );		
			return FALSE;
		}
		::CloseHandle( stProcessInfo.hThread );
		::CloseHandle( stProcessInfo.hProcess );	
		return TRUE;	
	}
	
	inline static BOOL WaitRunProcess(LPCSTR lpstrExePath,DWORD *nExitCode,LPCSTR lpstrCmdLine=NULL,BOOL bAddCurrentDir=FALSE)
	{
		if( strlen( lpstrExePath )<4 ) return FALSE;
		char strCmd[512];memset( strCmd,0,512 );
		if( lpstrCmdLine ){ sprintf( strCmd,"%s  %s",lpstrExePath,lpstrCmdLine );}
		else{ strcpy( strCmd,lpstrExePath ); };
		PROCESS_INFORMATION stProcessInfo;    
		STARTUPINFO stStartUpInfo; 
		memset(&stStartUpInfo, 0, sizeof(STARTUPINFO));
		stStartUpInfo.cb            = sizeof(STARTUPINFO);
		stStartUpInfo.dwFlags		= STARTF_USESHOWWINDOW;
		stStartUpInfo.wShowWindow	= SW_SHOW ;	

		*nExitCode=0;
		BOOL callSuccess=FALSE;
		char strDir[PATHLEN]; 
		if(bAddCurrentDir)
		{
			if(nsFileAnly::GetFileDir(lpstrExePath,strDir))
			{				
				callSuccess=::CreateProcess( NULL,strCmd,NULL,NULL,FALSE,NORMAL_PRIORITY_CLASS, NULL,strDir,&stStartUpInfo,&stProcessInfo );
			}			
		}
		else
		{
			callSuccess=::CreateProcess( NULL,strCmd,NULL,NULL,FALSE,NORMAL_PRIORITY_CLASS, NULL,NULL,&stStartUpInfo,&stProcessInfo );
		}		 
		
		if(!callSuccess)
		{
			::CloseHandle( stProcessInfo.hThread );
			::CloseHandle( stProcessInfo.hProcess );		
			return FALSE;
		}
		if( ::WaitForSingleObject( stProcessInfo.hProcess,INFINITE )==WAIT_OBJECT_0 )
		{
			::GetExitCodeProcess(stProcessInfo.hProcess,nExitCode);
			::CloseHandle( stProcessInfo.hThread );
			::CloseHandle( stProcessInfo.hProcess );
		}
		else { return FALSE; }
		
		return TRUE;	
	}

}

#endif // ProcRun_h__