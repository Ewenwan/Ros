/** 
 * @file	qtts.h
 * @brief   iFLY Speech Synthesizer Header File
 * 
 *  This file contains the quick application programming interface (API) declarations 
 *  of TTS. Developer can include this file in your project to build applications.
 *  For more information, please read the developer guide.
 
 *  Use of this software is subject to certain restrictions and limitations set
 *  forth in a license agreement entered into between iFLYTEK, Co,LTD.
 *  and the licensee of this software.  Please refer to the license
 *  agreement for license use rights and restrictions.
 *
 *  Copyright (C)    1999 - 2009 by ANHUI USTC iFLYTEK, Co,LTD.
 *                   All rights reserved.
 * 
 * @author	Speech Dept.
 * @version	1.0
 * @date	2009/11/26
 * 
 * @see		
 * 
 * <b>History:</b><br>
 * <table>
 *  <tr> <th>Version	<th>Date		<th>Author	<th>Notes</tr>
 *  <tr> <td>1.0		<td>2009/11/26	<td>Speech	<td>Create this file</tr>
 * </table>
 * 
 */
#ifndef __QTTS_H__
#define __QTTS_H__

#if !defined(MSPAPI)
#if defined(WIN32)
	#define MSPAPI __stdcall
#else
	#define MSPAPI
#endif /* WIN32 */
#endif /* MSPAPI */

#ifdef __cplusplus
extern "C" {
#endif /* C++ */

#include "msp_types.h"

/** 
 * @fn		QTTSSessionBegin
 * @brief	Begin a TTS Session
 * 
 *  Create a tts session to synthesize data.
 * 
 * @return	const char* - Return the new session id in success, otherwise return NULL, error code.
 * @param	const char* params			- [in] parameters when the session created.
 * @param	const char** sessionID		- [out] return a string to this session.
 * @see		
 */
const char* MSPAPI QTTSSessionBegin(const char* params, int* errorCode);
typedef const char* (MSPAPI *Proc_QTTSSessionBegin)(const char* params, int* errorCode);
#ifdef MSP_WCHAR_SUPPORT
const wchar_t* MSPAPI QTTSSessionBeginW(const wchar_t* params, int* errorCode);
typedef const wchar_t* (MSPAPI *Proc_QTTSSessionBeginW)(const wchar_t* params, int* errorCode);
#endif

/** 
 * @fn		QTTSTextPut
 * @brief	Put Text Buffer to TTS Session
 * 
 *  Writing text string to synthesizer.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* sessionID	- [in] The session id returned by sesson begin
 * @param	const char* textString	- [in] text buffer
 * @param	unsigned int textLen	- [in] text size in bytes
 * @see		
 */
int MSPAPI QTTSTextPut(const char* sessionID, const char* textString, unsigned int textLen, const char* params);
typedef int (MSPAPI *Proc_QTTSTextPut)(const char* sessionID, const char* textString, unsigned int textLen, const char* params);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QTTSTextPutW(const wchar_t* sessionID, const wchar_t* textString, unsigned int textLen, const wchar_t* params);
typedef int (MSPAPI *Proc_QTTSTextPutW)(const wchar_t* sessionID, const wchar_t* textString, unsigned int textLen, const wchar_t* params);
#endif

/** 
 * @fn		QTTSAudioGet
 * @brief	Synthesize text to audio
 * 
 *  Synthesize text to audio, and return audio information.
 * 
 * @return	const void*	- Return current synthesized audio data buffer, size returned by QTTSTextSynth.
 * @param	const char* sessionID	- [in] session id returned by session begin
 * @param	unsigned int* audioLen 	- [out] synthesized audio size in bytes
 * @param	int* synthStatus	- [out] synthesizing status
 * @param	int* errorCode	- [out] error code if failed, 0 to success.
 * @see		
 */
const void* MSPAPI QTTSAudioGet(const char* sessionID, unsigned int* audioLen, int* synthStatus, int* errorCode);
typedef const void* (MSPAPI *Proc_QTTSAudioGet)(const char* sessionID, unsigned int* audioLen, int* synthStatus, int* errorCode);
#ifdef MSP_WCHAR_SUPPORT
const void* MSPAPI QTTSAudioGetW(const wchar_t* sessionID, unsigned int* audioLen, int* synthStatus, int* errorCode);
typedef const void* (MSPAPI *Proc_QTTSAudioGetW)(const wchar_t* sessionID, unsigned int* audioLen, int* synthStatus, int* errorCode);
#endif

/** 
 * @fn		QTTSAudioInfo
 * @brief	Get Synthesized Audio information
 * 
 *  Get synthesized audio data information.
 * 
 * @return	const char * - Return audio info string.
 * @param	const char* sessionID	- [in] session id returned by session begin
 * @see		
 */
const char* MSPAPI QTTSAudioInfo(const char* sessionID);
typedef const char* (MSPAPI *Proc_QTTSAudioInfo)(const char* sessionID);
#ifdef MSP_WCHAR_SUPPORT
const wchar_t* MSPAPI QTTSAudioInfoW(const wchar_t* sessionID);
typedef const wchar_t* (MSPAPI *Proc_QTTSAudioInfoW)(const wchar_t* sessionID);
#endif

/** 
 * @fn		QTTSSessionEnd
 * @brief	End a Recognizer Session
 * 
 *  End the recognizer session, release all resource.
 * 
 * @return	int MSPAPI	- Return 0 in success, otherwise return error code.
 * @param	const char* session_id	- [in] session id string to end
 * @param	const char* hints	- [in] user hints to end session, hints will be logged to CallLog
 * @see		
 */
int MSPAPI QTTSSessionEnd(const char* sessionID, const char* hints);
typedef int (MSPAPI *Proc_QTTSSessionEnd)(const char* sessionID, const char* hints);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QTTSSessionEndW(const wchar_t* sessionID, const wchar_t* hints);
typedef int (MSPAPI *Proc_QTTSSessionEndW)(const wchar_t* sessionID, const wchar_t* hints);
#endif

/** 
 * @fn		QTTSGetParam
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
int MSPAPI QTTSGetParam(const char* sessionID, const char* paramName, char* paramValue, unsigned int* valueLen);
typedef int (MSPAPI *Proc_QTTSGetParam)(const char* sessionID, const char* paramName, char* paramValue, unsigned int* valueLen);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QTTSGetParamW(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue, unsigned int* valueLen);
typedef int (MSPAPI *Proc_QTTSGetParamW)(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue, unsigned int* valueLen);
#endif

/** 
 * @fn		QTTSSetParam
 * @brief	set params related with msc
 * 
 *  the params could be local or server param, we only support netflow params "upflow" & "downflow" now
 * 
 * @return	int	- Return 0 if success, otherwise return errcode.
 * @param	const char* sessionID	- [in] session id of related param, set NULL to got global param
 * @param	const char* paramName	- [in] param name,could pass more than one param split by ','';'or'\n'
 * @param	const char* paramValue	- [in] param value buffer, malloced by user
 * @see		
 */
int MSPAPI QTTSSetParam(const char *sessionID, const char *paramName, const char *paramValue);
typedef int (MSPAPI *Proc_QTTSSetParam)(const char* sessionID, const char* paramName, char* paramValue);
#ifdef MSP_WCHAR_SUPPORT
int MSPAPI QTTSSetParamW(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue);
typedef int (MSPAPI *Proc_QTTSSetParamW)(const wchar_t* sessionID, const wchar_t* paramName, wchar_t* paramValue);
#endif

typedef void ( *tts_result_ntf_handler)( const char *sessionID, const char *audio, int audioLen, int synthStatus, int ced, const char *audioInfo, int audioInfoLen, void *userData ); 
typedef void ( *tts_status_ntf_handler)( const char *sessionID, int type, int status, int param1, const void *param2, void *userData);
typedef void ( *tts_error_ntf_handler)(const char *sessionID, int errorCode,	const char *detail, void *userData);
int MSPAPI QTTSRegisterNotify(const char *sessionID, tts_result_ntf_handler rsltCb, tts_status_ntf_handler statusCb, tts_error_ntf_handler errCb, void *userData);

#ifdef __cplusplus
} /* extern "C" */
#endif /* C++ */

#endif /* __QTTS_H__ */
