/********************************************************************
       Author:		           ZQW	
       File Name:	           FileDialog
	   File Type:	           hpp
       Created Date:	       2011/8/4
	   Created Time:	       16:12      
*********************************************************************/
#ifndef FileDialog_hpp_ZQW_2011_8_4_16_12_DEF
#define FileDialog_hpp_ZQW_2011_8_4_16_12_DEF

#include "io.h"
#include "BasicDef.h"
#include <shlobj.h>
#include <commdlg.h>

#ifndef BIF_NEWDIALOGSTYLE
#define BIF_NEWDIALOGSTYLE     0x0040   // Use the new dialog layout with the ability to resize
#endif


class CDirDialog 
{
public:
    CDirDialog(HWND hWndPar=NULL,LPCSTR strInitDir=NULL,LPCSTR strTitle="Select A Directory"){ m_hWndPar=hWndPar;strcpy(m_strTitle,strTitle); m_strPath[0]=0; if (strInitDir) strcpy(m_strInitDir,strInitDir); else m_strInitDir[0]=0; };
    virtual ~CDirDialog(){};
	
    BOOL   DoModal()
	{
		BROWSEINFO bi; memset( &bi,0,sizeof(bi) ); m_strPath[0] = 0;
		bi.hwndOwner     = m_hWndPar?m_hWndPar:GetFocus(); 
		bi.lpszTitle	 = m_strTitle; 
		bi.pszDisplayName=m_strPath; bi.ulFlags      =BIF_RETURNONLYFSDIRS|BIF_NEWDIALOGSTYLE; 
		bi.lParam = LPARAM(m_strInitDir); 
		bi.lpfn   = strlen(m_strInitDir)>2?CallbackProc:NULL;
		
		ITEMIDLIST*pidl = SHBrowseForFolder( &bi ); m_strPath[0] = 0;
		if ( !pidl ) return NULL;
		return SHGetPathFromIDList( pidl,m_strPath );
	};
    LPCSTR GetPath(){ return m_strPath; };
	static int CALLBACK CallbackProc(HWND hwnd, UINT uMsg,	LPARAM lParam, LPARAM lpData ){ if ( uMsg==BFFM_INITIALIZED ) ::SendMessage( hwnd, BFFM_SETSELECTION, TRUE, lpData ); return 0; }
	
    static BOOL CreateDir(LPCTSTR szPath)
    {
        if ( _access(szPath,0)==0 ) return TRUE;    
        char strPath[512]; strcpy( strPath,szPath );
        char *pSplit = strrchr( strPath,'\\' );
        if ( !pSplit ) return ::CreateDirectory(strPath,NULL); else *pSplit = 0; 
        if ( !CreateDir(strPath) ) return FALSE;
        return ::CreateDirectory(szPath,NULL);
    }
private:
    char    m_strTitle[128];
    HWND    m_hWndPar;
    char    m_strPath[256];
	char	m_strInitDir[256];
};
//打开文件：
//调用1：Dlg.DoModal( TRUE,"Cloud Files(*.Cld)\0*.Cld\0Laser Files(*.LAS)\0*.LAS\0All Files(*.*)\0*.*\0" );
//调用2：Dlg.DoModal( TRUE,"Point Files(*.Cld;*.LAS)\0*.Cld;*.LAS\0All Files(*.*)\0*.*\0" );
//保存文件：
//Dlg.DoModal( TRUE,"Cloud Files(*.Cld)\0*.Cld\0Laser Files(*.LAS)\0*.LAS\0All Files(*.*)\0*.*\0" );

class CFileExplorer
{
public:
	CFileExplorer( )
	{ 
		m_hWndPar=NULL;
		m_bInit=FALSE;m_pPathLst=NULL;m_nFileMax=m_nPathNum=0; 
		memset(m_strTitle,0,PATHLEN);
	}
	virtual ~CFileExplorer(){ Clear(); };

	BOOL               Init( HWND hWndOwer=NULL,int nMaxFileNum=50 )
	{
		Clear(); m_bInit=FALSE; m_nFileMax=nMaxFileNum;m_hWndPar=hWndOwer;
		m_pPathLst = new char[m_nFileMax*PATHLEN];
		if( m_pPathLst )
		{
			memset( m_pPathLst,0,m_nFileMax*PATHLEN );
			m_bInit=TRUE;
		}
		return m_bInit;
	};
	BOOL               IsInit(){ return m_bInit; };
	void               Clear(){ if( m_pPathLst ){ delete[] m_pPathLst;m_pPathLst=NULL; }; };

	void               SetDlgTitle(const char* strTitle  ){ sprintf(m_strTitle,"%s",strTitle); };
	char*              GetFirstPath(){ return m_pPathLst; };
	char*              GetPathList( int *nPathNum ){ *nPathNum=m_nPathNum;return m_pPathLst; };

public:	
	BOOL               DoModal( BOOL bOpenDialog,BOOL bSelectMulti=TRUE,const char* strFilter="All Files(*.*)\0*.*\0" )
	{
		BOOL bSucess=TRUE;
		if( IsInit()&&m_pPathLst )
		{			
			char *pStrLst = new char[m_nFileMax*PATHLEN];
			if( pStrLst )
			{
				memset( pStrLst,0,m_nFileMax*PATHLEN );
				OPENFILENAME ofn; memset(&ofn,0,sizeof(ofn)); 
				ofn.hwndOwner=m_hWndPar?m_hWndPar:GetFocus();
				char strDir[PATHLEN],*p=NULL,*p1=NULL; int nLen=0;m_nPathNum=0;
				ofn.Flags=OFN_EXPLORER; if( bSelectMulti&&bOpenDialog ){ ofn.Flags|=OFN_ALLOWMULTISELECT; }
				ofn.lStructSize  = sizeof(ofn);
				ofn.lpstrFile    = pStrLst;
				ofn.nMaxFile     = m_nFileMax*PATHLEN;
				ofn.lpstrFile[0] = '\0';				
				ofn.lpstrFilter  = strFilter;
				if( strlen(m_strTitle)==0 )
				{ 
					if(bOpenDialog){ ofn.lpstrTitle="Open File..."; }
					else{ ofn.lpstrTitle="Save File..."; }
				}
				else{ ofn.lpstrTitle=m_strTitle; }

				if( bOpenDialog )
				{
					if( GetOpenFileName(&ofn) )
					{
						strncpy( strDir,pStrLst,ofn.nFileOffset );
						strDir[ofn.nFileOffset]='\0';
						nLen=lstrlen(strDir);
						if(strDir[nLen-1]!='\\'){ strcat(strDir,"\\" );	}
						
						p=pStrLst+ofn.nFileOffset;  
						memset( m_pPathLst,0,m_nFileMax*PATHLEN );					
						p1 = m_pPathLst;
						while (*p)
						{
							sprintf( p1,"%s%s",strDir,p );						
							p+=strlen(p)+1;	p1+=PATHLEN;
							m_nPathNum++;
						}
					}
					else{ bSucess=FALSE; }
				}
				else
				{
					if( GetSaveFileName(&ofn) )
					{
						sprintf( m_pPathLst,"%s",pStrLst );										
					}
				}				
			}
			else{ bSucess=FALSE; }
			
			if( pStrLst ){ delete[] pStrLst;pStrLst=FALSE; }
		}
		else{ bSucess=FALSE; }

		return bSucess;
	}
	

protected:
	BOOL               m_bInit;
	HWND               m_hWndPar;
	char              *m_pPathLst;
	int                m_nFileMax,m_nPathNum;
	char               m_strTitle[PATHLEN];
		
private:
};



#endif // FileDialog_hpp__


