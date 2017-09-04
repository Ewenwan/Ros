/**
 * @file    qisr.h
 * @brief   iFLY Speech Recognizer Header File
 * 
 *  This file contains the quick application programming interface (API) declarations 
 *  of ISR. Developer can include this file in your project to build applications.
 *  For more information, please read the developer guide.
 
 *  Use of this software is subject to certain restrictions and limitations set
 *  forth in a license agreement entered into between iFLYTEK, Co,LTD.
 *  and the licensee of this software.  Please refer to the license
 *  agreement for license use rights and restrictions.
 *
 *  Copyright (C)    1999 - 2007 by ANHUI USTC iFLYTEK, Co,LTD.
 *                   All rights reserved.
 * 
 * @author  Speech Dept. iFLYTEK.
 * @version 1.0
 * @date    2008/12/12
 * 
 * @see        
 * 
 * History:
 * index    version        date            author        notes
 * 0        1.0            2008/12/12      Speech        Create this file
 */

#ifndef __QISR_H__
#define __QISR_H__

#ifdef __cplusplus
extern "C" {
#endif /* C++ */

#include "msp_types.h"

/** 
 * @fn		QISRSessionBegin
 * @brief	Begin a Recognizer Session
 * 
 *  Create a recognizer session to recognize audio data
 * 
 * @return	return sessionID of current session, NULL is failed.
 * @param	const char* grammarList		- [in] grammars list, inline grammar support only one.
 * @param	const char* params			- [in] parameters when the session created.
 * @param	int *errorCode				- [out] return 0 on success, otherwise return error code.
 * @see		
 */
const char* MSPAPI QISRSessionBegin(const char* grammarList, const char* params, int* errorCode);
typedef const char* (MSPAPI *Proc_QISRSessionBegin)(const char* grammarList, const char* params, int *result);
#ifdef MSP_WCHAR_SUPPORT
const wchar_t* MSPAPI QISRSessionBeginW(const wchar_t* grammarList, const wchar_t* params, int *result);
typedef const wchar_t* (MSPAPI *Proc_QISRSessionBeginW)(const wchar_t* grammarList, const wchar_t* params, int *result);
#endif


/** 
 * @fn		QISRAudioWrite
 * @brief	Write Audio Data to Recognizer Session
 * 
 *  Writing binary audio data to recognizer.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* sessionID	- [in] The session id returned by recog_begin
 * @param	const void* waveData	- [in] Binary data of waveform
 * @param	unsigned int waveLen	- [in] Waveform data size in bytes
 * @param	int audioStatus			- [in] Audio status, can be 
 * @param	int *epStatus			- [out] ISRepState
 * @param	int *recogStatus		- [out] ISRrecRecognizerStatus, see isr_rec.h
 * @see		
 */
int MSPAPI QISRAudioWrite(const char* sessionID, const void* waveData, unsigned int waveLen, int audioStatus, int *epStatus, int *recogStatus);
typedef int (MSPAPI *Proc_QISRAudioWrite)(const char* sessionID, const void* waveData, unsigned int waveLen, int audioStatus, int *epStatus, int *recogStatus);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRAudioWriteW(const wchar_t* sessionID, const void* waveData, unsigned int waveLen, int audioStatus, int *epStatus, int *recogStatus);
typedef int (MSPAPI *Proc_QISRAudioWriteW)(const wchar_t* sessionID, const void* waveData, unsigned int waveLen, int audioStatus, int *epStatus, int *recogStatus);
#endif

/** 
 * @fn		QISRGetResult
 * @brief	Get Recognize Result in Specified Format
 * 
 *  Get recognize result in Specified format.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* sessionID	- [in] session id returned by session begin
 * @param	int* rsltStatus			- [out] status of recognition result, 0: success, 1: no match, 2: incomplete, 5:speech complete
 * @param	int *errorCode			- [out] return 0 on success, otherwise return error code.
 * @see		
 */
const char * MSPAPI QISRGetResult(const char* sessionID, int* rsltStatus, int waitTime, int *errorCode);
typedef const char * (MSPAPI *Proc_QISRGetResult)(const char* sessionID, int* rsltStatus, int waitTime, int *errorCode);
#ifdef MSP_WCHAR_SUPPORT
const wchar_t* MSPAPI QISRGetResultW(const wchar_t* sessionID, int* rsltStatus, int waitTime, int *errorCode);
typedef const wchar_t* (MSPAPI *Proc_QISRGetResultW)(const wchar_t* sessionID, int* rsltStatus, int waitTime, int *errorCode);
#endif

/** 
 * @fn		QISRGetBinaryResult
 * @brief	Get Recognize Result in Specified Format
 * 
 *  Get recognize result in Specified format.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* sessionID	- [in] session id returned by session begin
 * @param	int* rsltStatus			- [out] status of recognition result, 0: success, 1: no match, 2: incomplete, 5:speech complete
 * @param	int *errorCode			- [out] return 0 on success, otherwise return error code.
 * @see		
 */
const char * MSPAPI QISRGetBinaryResult(const char* sessionID, unsigned int* rsltLen,int* rsltStatus, int waitTime, int *errorCode);
typedef const char * (MSPAPI *Proc_QISRGetBinaryResult)(const char* sessionID, unsigned int* rsltLen, int* rsltStatus, int waitTime, int *errorCode);
#ifdef MSP_WCHAR_SUPPORT
const wchar_t* MSPAPI QISRGetBinaryResultW(const wchar_t* sessionID, unsigned int* rsltLen, int* rsltStatus, int waitTime, int *errorCode);
typedef const wchar_t* (MSPAPI *Proc_QISRGetBinaryResultW)(const wchar_t* sessionID, unsigned int* rsltLen, int* rsltStatus, int waitTime, int *errorCode);
#endif


/** 
 * @fn		QISRSessionEnd
 * @brief	End a Recognizer Session
 * 
 *  End the recognizer session, release all resource.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* sessionID	- [in] session id string to end
 * @param	const char* hints	- [in] user hints to end session, hints will be logged to CallLog
 * @see		
 */
int MSPAPI QISRSessionEnd(const char* sessionID, const char* hints);
typedef int (MSPAPI *Proc_QISRSessionEnd)(const char* sessionID, const char* hints);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRSessionEndW(const wchar_t* sessionID, const wchar_t* hints);
typedef int (MSPAPI *Proc_QISRSessionEndW)(const wchar_t* sessionID, const wchar_t* hints);
#endif

/** 
 * @fn		QISRGetParam
 * @brief	get params related with msc
 * 
 *  the params could be local or server param, we only support netflow params "upflow" & "downflow" now
 * 
 * @return	int	- Return 0 if success, otherwise return errcode.
 * @param	const char* sessionID	- [in] session id of related param, set NULL to got global param
 * @param	const char* paramName	- [in] param name,could pass more than one param split by ','';'or'\n'
 * @param	const char* paramValue	- [in] param value buffer, malloced by user
 * @param	int *valueLen			- [in, out] pass in length of value buffer, and return length of value string
 * @see		
 */
int MSPAPI QISRGetParam(const char* sessionID, const char* paramName, char* paramValue, unsigned int* valueLen);
typedef int (MSPAPI *Proc_QISRGetParam)(const char* sessionID, const char* paramName, char* paramValue, unsigned int* valueLen);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRGetParamW(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue, unsigned int* valueLen);
typedef int (MSPAPI *Proc_QISRGetParamW)(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue, unsigned int* valueLen);
#endif

/** 
 * @fn		QISRSetParam
 * @brief	get params related with msc
 * 
 *  the params could be local or server param, we only support netflow params "upflow" & "downflow" now
 * 
 * @return	int	- Return 0 if success, otherwise return errcode.
 * @param	const char* sessionID	- [in] session id of related param, set NULL to got global param
 * @param	const char* paramName	- [in] param name,could pass more than one param split by ','';'or'\n'
 * @param	const char* paramValue	- [in] param value buffer, malloced by user
 * @param	int *valueLen			- [in, out] pass in length of value buffer, and return length of value string
 * @see		
 */
int MSPAPI QISRSetParam(const char* sessionID, const char* paramName, const char* paramValue);
typedef int (MSPAPI *Proc_QISRSetParam)(const char* sessionID, const char* paramName, const char* paramValue);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRSetParamW(const wchar_t* sessionID, const wchar_t* paramName, const wchar_t* paramValue);
typedef int (MSPAPI *Proc_QISRSetParamW)(const wchar_t* sessionID, const wchar_t* paramName, const wchar_t* paramValue);
#endif


typedef void ( *recog_result_ntf_handler)( const char *sessionID, const char *result, int resultLen, int resultStatus, void *userData ); 
typedef void ( *recog_status_ntf_handler)( const char *sessionID, int type, int status, int param1, const void *param2, void *userData);
typedef void ( *recog_error_ntf_handler)(const char *sessionID, int errorCode,	const char *detail, void *userData);
int MSPAPI QISRRegisterNotify(const char *sessionID, recog_result_ntf_handler rsltCb, recog_status_ntf_handler statusCb, recog_error_ntf_handler errCb, void *userData);

typedef int ( *UserCallBack)( int, const char*, void*);
typedef int ( *GrammarCallBack)( int, const char*, void*);
typedef int ( *LexiconCallBack)( int, const char*, void*);

int MSPAPI QISRBuildGrammar(const char *grammarType, const char *grammarContent, unsigned int grammarLength, const char *params, GrammarCallBack callback, void *userData);
typedef int (MSPAPI *Proc_QISRBuildGrammar)(const char *grammarType, const char *grammarContent, unsigned int grammarLength, const char *params, GrammarCallBack callback, void *userData);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRBuildGrammarW(const wchar_t *grmmarType, const wchar_t *grammarContent, unsigned int grammarLength, const wchar_t *params, GrammarCallBack callback, void *userData);
typedef int (MSPAPI *Proc_QISRBuildGrammarW)(const wchar_t *grmmarType, const wchar_t *grammarContent, unsigned int grammarLength, const wchar_t *params, GrammarCallBack callback, void *userData);
#endif

int MSPAPI QISRUpdateLexicon(const char *lexiconName, const char *lexiconContent, unsigned int lexiconLength, const char *params, LexiconCallBack callback, void *userData);
typedef int (MSPAPI *Proc_QISRUpdataLexicon)(const char *lexiconName, const char *lexiconContent, unsigned int lexiconLength, const char *params, LexiconCallBack callback, void *userData);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QISRUpdateLexiconW(const wchar_t *lexiconName, const wchar_t *lexiconContent, unsigned int lexiconLength, const wchar_t *params, LexiconCallBack callback, void *userData);
typedef int (MSPAPI Proc_QISRUpdateLexiconW)(const wchar_t *lexiconName, const wchar_t *lexiconContent, unsigned int lexiconLength, const wchar_t *params, LexiconCallBack callback, void *userData);
#endif
#ifdef __cplusplus
} /* extern "C" */	
#endif /* C++ */

#endif /* __QISR_H__ */
