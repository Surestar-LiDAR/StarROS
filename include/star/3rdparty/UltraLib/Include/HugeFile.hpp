/********************************************************************
       Author:		           zzquan	
       File Name:	           HugeFile
	   File Type:	           hpp
       Created Date:	       2013/2/11
	   Created Time:	       11:27      
*********************************************************************/
#ifndef HugeFile_hpp_ZZQUAN_2013_2_11_11_27_DEF
#define HugeFile_hpp_ZZQUAN_2013_2_11_11_27_DEF

typedef LARGE_INTEGER       BIGINT;
typedef LARGE_INTEGER*      PBIGINT; 

class CIOFile
{
public:
	enum SeekPosition{ BEGIN=FILE_BEGIN,CURRENT=FILE_CURRENT,END=FILE_END, };
	enum OpenFlags 
	{
		modeREAD      = GENERIC_READ,
		modeWRITE     = GENERIC_WRITE,
		modeREADWRITE = GENERIC_READ | GENERIC_WRITE,
	};
	enum ShareFlags 
	{
		shareREAD      = FILE_SHARE_READ,
		shareWRITE     = FILE_SHARE_WRITE,
		shareREADWRITE = FILE_SHARE_READ | FILE_SHARE_WRITE,
	};

	BOOL Open(LPCSTR lpstrPath,DWORD dwOpenFlag,DWORD dwShareMode,
		            LPSECURITY_ATTRIBUTES lpSecurityAttributes=NULL,
		            DWORD dwCreationDispostion=OPEN_ALWAYS, 
		            DWORD dwFlagsAndAttributes=FILE_ATTRIBUTE_NORMAL, 
		            HANDLE hTemplateFile=NULL)
	{
		if(!lpstrPath) return FALSE;
		if(IsOpen()){Close();};	
		if(dwOpenFlag==modeWRITE){::DeleteFile(lpstrPath);}
		m_hFile=::CreateFile(lpstrPath,dwOpenFlag,dwShareMode,lpSecurityAttributes,dwCreationDispostion,dwFlagsAndAttributes,hTemplateFile);
		if(m_hFile!=INVALID_HANDLE_VALUE){m_bOpen=TRUE;};
		strcpy(m_strFilePath,lpstrPath);		
		return m_bOpen;
	};
	void Close()
	{
		if(m_hFile!=INVALID_HANDLE_VALUE)
		{
			::CloseHandle(m_hFile);
			m_hFile=INVALID_HANDLE_VALUE;
		} 	
		m_bOpen=FALSE; 
	};
	BOOL IsOpen(){return (m_hFile!=INVALID_HANDLE_VALUE && m_bOpen);};

	CIOFile()
	{
		m_bOpen=FALSE;
		m_hFile=INVALID_HANDLE_VALUE;
		memset(m_strFilePath,0,PATHLEN);
	};
	virtual ~CIOFile()
	{
		Close();
	};

public:
	BIGINT Seek(UINT64 nDistMove,DWORD mMoveMethod=BEGIN)
	{
		BIGINT nNewPos;
		nNewPos.QuadPart=(LONGLONG)nDistMove;
		DWORD dwNewLow=::SetFilePointer(m_hFile,nNewPos.LowPart,&nNewPos.HighPart,mMoveMethod);
		nNewPos.LowPart=dwNewLow;
		return (nNewPos);
	};
	BIGINT Seek(const BIGINT& nDistMove,DWORD mMoveMethod=BEGIN)
	{
		BIGINT nNewPos;
		nNewPos.QuadPart=(LONGLONG)nDistMove.QuadPart;
		DWORD dwNewLow=::SetFilePointer(m_hFile,nNewPos.LowPart,&nNewPos.HighPart,mMoveMethod);
		nNewPos.LowPart=dwNewLow;
		return (nNewPos);
	};
	DWORD  Write(LPCVOID lpBuffer,DWORD dwBufSize)
	{
		DWORD nWritten=0;
		::WriteFile(m_hFile,lpBuffer,dwBufSize,&nWritten,NULL);
		return nWritten;
	};	  
	DWORD  Read(LPVOID lpBuffer,DWORD dwBufSize)
	{
		DWORD dwReaded=0;
		::ReadFile(m_hFile,lpBuffer,dwBufSize,&dwReaded,NULL);        
		return dwReaded;
	};	

	void   Flush(){::FlushFileBuffers(m_hFile);}
	void   GetFileSize(BIGINT& nFileSize);	
	LPCSTR GetFilePath(){ return m_strFilePath;};

	static UINT64 MakeUnsignedInt64(DWORD nHigh, DWORD nLow){return ((((UINT64) nHigh) << 32) | nLow);};
	static void   SplitUnsignedInt64(const UINT64& nBigInt,DWORD& nHigh,DWORD& nLow){nHigh=(DWORD)((nBigInt & 0xFFFFFFFF00000000) >> 32);nLow=(DWORD)(nBigInt & 0x00000000FFFFFFFF);};

protected:
	BOOL        m_bOpen;
	HANDLE      m_hFile;
	char        m_strFilePath[PATHLEN];	
};

#endif // HugeFile_hpp__