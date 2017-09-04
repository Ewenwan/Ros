/**
 * @file    msp_cmn.h
 * @brief   Mobile Speech Platform Common Interface Header File
 * 
 *  This file contains the quick common programming interface (API) declarations 
 *  of MSP. Developer can include this file in your project to build applications.
 *  For more information, please read the developer guide.
 
 *  Use of this software is subject to certain restrictions and limitations set
 *  forth in a license agreement entered into between iFLYTEK, Co,LTD.
 *  and the licensee of this software.  Please refer to the license
 *  agreement for license use rights and restrictions.
 *
 *  Copyright (C)    1999 - 2012 by ANHUI USTC iFLYTEK, Co,LTD.
 *                   All rights reserved.
 * 
 * @author  Speech Dept. iFLYTEK.
 * @version 1.0
 * @date    2012/09/01
 * 
 * @see        
 * 
 * History:
 * index    version        date            author        notes
 * 0        1.0            2012/09/01      MSC40        Create this file
 */

#ifndef __MSP_CMN_H__
#define __MSP_CMN_H__

#include "msp_types.h"

#ifdef __cplusplus
extern "C" {
#endif /* C++ */
//#ifdef MSP_WCHAR_SUPPORT
/** 
 * @fn		Wchar2Mbytes
 * @brief	wchar to mbytes
 * 
 *  User login.
 * 
 * @return	int MSPAPI				- Return 0 in success, otherwise return error code.
 * @param	const wchar_t* wcstr	- [in] Null-terminated source string(wchar_t *).
 * @param	char* mbstr				- [in] Destination string(char *).
 * @param   int len					- [in] The maximum number of bytes that can be stored in the multibyte output string.
 * @see		
 */

char *Wchar2Mbytes(const wchar_t* wcstr);

/** 
 * @fn		Mbytes2Wchar
 * @brief	mbytes to wchar
 * 
 *  User login.
 * 
 * @return	int MSPAPI				- Return 0 in success, otherwise return error code.
 * @param	const char* mbstr		- [in] Null-terminated source string(char *).
 * @param	wchar_t* wcstr			- [in] Destination string(wchar_t *).
 * @param   int wlen				- [in] The maximum number of multibyte characters to convert.
 * @see		
 */
wchar_t *Mbytes2Wchar(const char *mbstr);

//#endif /*MSP_WCHAR_SUPPORT*/

/** 
 * @fn		MSPLogin
 * @brief	user login interface
 * 
 *  User login.
 * 
 * @return	int MSPAPI			- Return 0 in success, otherwise return error code.
 * @param	const char* usr		- [in] user name.
 * @param	const char* pwd		- [in] password.
 * @param	const char* params	- [in] parameters when user login.
 * @see		
 */
int MSPAPI MSPLogin(const char* usr, const char* pwd, const char* params);
typedef int (MSPAPI *Proc_MSPLogin)(const char* usr, const char* pwd, const char* params);
//#ifdef MSP_WCHAR_SUPPORT
int MSPAPI MSPLoginW(const wchar_t* usr, const wchar_t* pwd, const wchar_t* params);
typedef int (MSPAPI *Proc_MSPLoginW)(const wchar_t* usr, const wchar_t* pwd, const wchar_t* params);
//#endif/*MSP_WCHAR_SUPPORT*/
/** 
 * @fn		MSPLogout
 * @brief	user logout interface
 * 
 *  User logout
 * 
 * @return	int MSPAPI			- Return 0 in success, otherwise return error code.
 * @see		
 */
int MSPAPI MSPLogout();
typedef int (MSPAPI *Proc_MSPLogout)();
//#ifdef MSP_WCHAR_SUPPORT
int MSPAPI MSPLogoutW();
typedef int (MSPAPI *Proc_MSPLogoutW)();
//#endif/*MSP_WCHAR_SUPPORT*/
/** 
 * @fn		MSPUpload
 * @brief	Upload User Specific Data
 * 
 *  Upload data such as user config, custom grammar, etc.
 * 
 * @return	int MSPAPI				- Return 0 in success, otherwise return error code.
 * @param	const char* dataName	- [in] data name, should be unique to diff other data.
 * @param	const char* params		- [in] parameters about uploading data.
 * @param	const char* dataID		- [in] id of the data to be operated.
 * @see		
 */
int MSPAPI MSPUpload( const char* dataName, const char* params, const char* dataID);
typedef int (MSPAPI* Proc_MSPUpload)( const char* dataName, const char* params, const char* dataID);

/** 
 * @fn		MSPDownload
 * @brief	Download User Specific Data
 * 
 *  Download data such as user config, etc.
 * 
 * @return	int MSPAPI				- Return 0 in success, otherwise return error code.
 * @param	const char* params		- [in] parameters about data to be downloaded.
 * @see		
 */
typedef int (*DownloadStatusCB)(int errorCode, long param1, const void *param2, void *userData);
typedef int (*DownloadResultCB)(const void *data, long dataLen, void *userData);
int MSPAPI MSPDownload(const char* dataName, const char* params, DownloadStatusCB statusCb, DownloadResultCB resultCb, void*userData);
typedef int (MSPAPI* Proc_MSPDownload)(const char* dataName, const char* params, DownloadStatusCB statusCb, DownloadResultCB resultCb, void*userData);
int MSPAPI MSPDownloadW(const wchar_t* wdataName, const wchar_t* wparams, DownloadStatusCB statusCb, DownloadResultCB resultCb, void*userData);
typedef int (MSPAPI* Proc_MSPDownloadW) (const wchar_t* wdataName, const wchar_t* wparams, DownloadStatusCB statusCb, DownloadResultCB resultCb, void*userData);

/** 
 * @fn		MSPAppendData
 * @brief	Append Data.
 * 
 *  Write data to msc, such as data to be uploaded, searching text, etc.
 * 
 * @return	int MSPAPI					- Return 0 in success, otherwise return error code.
 * @param	void* data					- [in] the data buffer pointer, data could be binary.
 * @param	unsigned int dataLen		- [in] length of data.
 * @param	unsigned int dataStatus		- [in] data status, 2: first or continuous, 4: last.
 * @see		
 */
int MSPAPI MSPAppendData(void* data, unsigned int dataLen, unsigned int dataStatus);
typedef int (MSPAPI* Proc_MSPAppendData)(void* data, unsigned int dataLen, unsigned int dataStatus);

/** 
 * @fn		MSPGetResult
 * @brief	Get Result
 * 
 *  Get result of uploading, downloading or searching, etc.
 * 
 * @return	const char* MSPAPI		- Return result of uploading, downloading or searching, etc.
 * @param	int* rsltLen			- [out] Length of result returned.
 * @param	int* rsltStatus			- [out] Status of result returned.
 * @param	int* errorCode			- [out] Return 0 in success, otherwise return error code.
 * @see		
 */
const char* MSPAPI MSPGetResult(unsigned int* rsltLen, int* rsltStatus, int *errorCode);
typedef const char * (MSPAPI *Proc_MSPGetResult)(unsigned int* rsltLen, int* rsltStatus, int *errorCode);

/** 
 * @fn		MSPSetParam
 * @brief	set params of msc
 * 
 *  set param of msc
 * 
 * @return	int	- Return 0 if success, otherwise return errcode.
 * @param	const char* paramName	- [in] param name.
 * @param	const char* paramValue	- [in] param value
 * @see		
 */
int MSPAPI MSPSetParam( const char* paramName, const char* paramValue );
typedef int (MSPAPI *Proc_MSPSetParam)(const char* paramName, const char* paramValue);

/** 
 * @fn		MSPGetParam
 * @brief	get params of msc
 * 
 *  get param of msc
 * 
 * @return	int	- Return 0 if success, otherwise return errcode.
 * @param	const char* paramName	- [in] param name.
 * @param	const char* paramValue	- [out] param value
 * @param	const char* valueLen	- [in/out] param value (buffer) length
 * @see		
 */
int MSPAPI MSPGetParam( const char *paramName, char *paramValue, unsigned int *valueLen );
typedef int (MSPAPI *Proc_MSPGetParam)( const char *paramName, char *paramValue, unsigned int *valueLen );

/** 
 * @fn		MSPUploadData
 * @brief	Upload User Specific Data
 * 
 *  Upload data such as user config, custom grammar, etc.
 * 
 * @return	const char* MSPAPI		- data id returned by Server, used for special command.
 * @param	const char* dataName	- [in] data name, should be unique to diff other data.
 * @param	void* data				- [in] the data buffer pointer, data could be binary.
 * @param	unsigned int dataLen	- [in] length of data.
 * @param	const char* params		- [in] parameters about uploading data.
 * @param	int* errorCode			- [out] Return 0 in success, otherwise return error code.
 * @see		
 */
const char* MSPAPI MSPUploadData(const char* dataName, void* data, unsigned int dataLen, const char* params, int* errorCode);
typedef const char* (MSPAPI* Proc_MSPUploadData)(const char* dataName, void* data, unsigned int dataLen, const char* params, int* errorCode);

/** 
 * @fn		MSPDownloadData
 * @brief	Download User Specific Data
 * 
 *  Download data such as user config, etc.
 * 
 * @return	const void*	MSPAPI		- received data buffer pointer, data could be binary, NULL if failed or data does not exsit.
 * @param	const char* params		- [in] parameters about data to be downloaded.
 * @param	unsigned int* dataLen	- [out] length of received data.
 * @param	int* errorCode			- [out] Return 0 in success, otherwise return error code.
 * @see		
 */
const void* MSPAPI MSPDownloadData(const char* params, unsigned int* dataLen, int* errorCode);
typedef const void* (MSPAPI* Proc_MSPDownloadData)(const char* params, unsigned int* dataLen, int* errorCode);
//#ifdef MSP_WCHAR_SUPPORT
const void* MSPAPI MSPDownloadDataW(const wchar_t* params, unsigned int* dataLen, int* errorCode);
typedef const void* (MSPAPI* Proc_MSPDownloadDataW)(const wchar_t* params, unsigned int* dataLen, int* errorCode);
//#endif/*MSP_WCHAR_SUPPORT*/
/** 
 * @fn		MSPSearch
 * @brief	Search text for result
 * 
 *  Search text content, and got text result
 * 
 * @return	const void*	MSPAPI		- received data buffer pointer, data could be binary, NULL if failed or data does not exsit.
 * @param	const char* params		- [in] parameters about data to be downloaded.
 * @param	unsigned int* dataLen	- [out] length of received data.
 * @param	int* errorCode			- [out] Return 0 in success, otherwise return error code.
 * @see		
 */
const char* MSPAPI MSPSearch(const char* params, const char* text, unsigned int* dataLen, int* errorCode);
typedef const char* (MSPAPI* Proc_MSPSearch)(const char* params, const char* text, unsigned int* dataLen, int* errorCode);



typedef int (*NLPSearchCB)(const char *sessionID, int errorCode, int status, const void* result, long rsltLen, void *userData);
const char* MSPAPI MSPNlpSearch(const char* params, const char* text, unsigned int textLen, int *errorCode, NLPSearchCB callback, void *userData);
typedef const char* (MSPAPI* Proc_MSPNlpSearch)(const char* params, const char* text, unsigned int textLen, int *errorCode, NLPSearchCB callback, void *userData);
int MSPAPI MSPNlpSchCancel(const char *sessionID, const char *hints);

/** 
 * @fn		MSPRegisterNotify
 * @brief	Register a Callback
 * 
 *  Register a Callback
 * 
 * @return	int                     -
 * @param	msp_status_ntf_handler statusCb		- [in] notify handler
 * @param	void *userData                   	- [in] userData
 * @see		
 */
typedef void ( *msp_status_ntf_handler)( int type, int status, int param1, const void *param2, void *userData );
int MSPAPI MSPRegisterNotify( msp_status_ntf_handler statusCb, void *userData );
typedef const char* (MSPAPI* Proc_MSPRegisterNotify)( msp_status_ntf_handler statusCb, void *userData );

/**
 * @fn		MSPGetVersion
 * @brief	Get version of MSC or Local Engine
 *
 * Get version of MSC or Local Engine
 * 
 * @return	const char * MSPAPI		- Return version value if success, NULL if fail.
 * @param	const char *verName		- [in] version name, could be "msc", "aitalk", "aisound", "ivw".
 * @param	int *errorCode			- [out] Return 0 in success, otherwise return error code.
 * @see
 */
const char* MSPAPI MSPGetVersion(const char *verName, int *errorCode);
typedef const char* (MSPAPI * Proc_MSPGetVersion)(const char *verName, int *errorCode);

#ifdef __cplusplus
} /* extern "C" */	
#endif /* C++ */

#endif /* __MSP_CMN_H__ */
