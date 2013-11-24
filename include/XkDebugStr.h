#ifndef XK_DEBUG_STR_H
#define XK_DEBUG_STR_H

#include "XkConfig.h"

#ifdef WIN32
#ifndef _USRDLL
	#include <windows.h>
#else
	#include "stdafx.h"
#endif
#include <stdio.h>
#include <stdarg.h>
#endif

/////////////////////////////////////////////////////////////////////////
//���������ַ����ĺ�
/////////////////////////////////////////////////////////////////////////
#ifdef WIN32
#define WIDEN2(x,y,z) x "(" #y "):" z "\r\n"
#define WIDEN(x,y,z) WIDEN2(x , y,z)
#define __WFILE__ WIDEN( __FILE__ , __LINE__, __FUNCTION__)
#else
#define WIDEN2(x,y,z) x "(" #y ")\n"
#define WIDEN(x,y,z) WIDEN2(x , y,z)
#define __WFILE__ WIDEN( __FILE__ , __LINE__, "")
#endif

#define JOIN2(x,y) x " | " y
#define JOIN(x,y) JOIN2(x,y)
#define MYJOIN(x) JOIN(x,__WFILE__)

static void XkDebugString(char* szFormat,...)
{
	int nListCount = 0;
	va_list pArgList;

	int nMaxLength = 1024;
	char szBuf[1024];
	memset(szBuf,0,1024);

	va_start(pArgList,szFormat);
#ifdef WIN32
    nListCount += _vsnprintf_s(szBuf+nListCount, 1024, nMaxLength-nListCount, szFormat, pArgList);
#else
	nListCount += _vsnprintf(szBuf+nListCount,nMaxLength-nListCount,szFormat,pArgList);
#endif
	va_end(pArgList);

	if(nListCount > (nMaxLength-1)) nListCount = nMaxLength - 1;
	*(szBuf+nListCount) = '\0';

#ifdef WIN32
	OutputDebugStringA((LPCSTR)szBuf);
#else
	printf("%s",szBuf);
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DBGSTRING�꣬��ӡ������Ϣ������ͨ��SNH_DEBUG���������û�������
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
	#if XK_ENABLE_DEBUG_STR == 1
		#define DBGSTRING(format,...) XkDebugString(format,__VA_ARGS__)
	#else
#define DBGSTRING(format,...) /\
/SnhDebugString(format,__VA_ARGS__) 
	#endif
#else	/* Other OS */
#define DBGSTRING(format,...) /\
/XkDebugString(format,__VA_ARGS__)
#endif /* WIN32 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//XDBGSTRING�꣬��ӡ������Ϣ��������Դ�����ļ��ͺ�����Ϣ������ͨ��SNH_DEBUG���������û�������
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef WIN32
	#if XK_ENABLE_DEBUG_STR == 1
		#define XDBGSTRING(format,...) XkDebugString(MYJOIN(format),__VA_ARGS__)
	#else
#define XDBGSTRING(format,...) /\
/XkDebugString(format,__VA_ARGS__) 
	#endif
#else	/* Other OS */
#define XDBGSTRING(format,...) /\
/XkDebugString(format,__VA_ARGS__)
#endif /* WIN32 */

#endif