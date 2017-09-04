/*
@file
@brief a simple demo to recognize speech from microphone

@author		taozhang9
@date		2016/05/27
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "speech_recognizer.h"
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "linuxrec.h"


#define SR_DBGON 1
#if SR_DBGON == 1
#	define sr_dbg printf
#else
#	define sr_dbg
#endif

#define DEFAULT_SESSION_PARA \
	 "sub = iat, domain = iat, language = zh_cn, accent = mandarin, sample_rate = 16000, result_type = plain, result_encoding = utf8"

#define DEFAULT_FORMAT		\
{\
	WAVE_FORMAT_PCM,	\
	1,			\
	16000,			\
	32000,			\
	2,			\
	16,			\
	sizeof(WAVEFORMATEX)	\
}

/* internal state */
enum {
	SR_STATE_INIT,
	SR_STATE_STARTED
};


#define SR_MALLOC malloc
#define SR_MFREE  free
#define SR_MEMSET	memset


static void Sleep(size_t ms)
{
	usleep(ms*1000);
}


static void end_sr_on_error(struct speech_rec *sr, int errcode)
{
	if(sr->aud_src == SR_MIC)
		stop_record(sr->recorder);
	
	if (sr->session_id) {
		if (sr->notif.on_speech_end)
			sr->notif.on_speech_end(errcode);

		QISRSessionEnd(sr->session_id, "err");
		sr->session_id = NULL;
	}
	sr->state = SR_STATE_INIT;
}

static void end_sr_on_vad(struct speech_rec *sr)
{
	int errcode;
	const char *rslt;

	if (sr->aud_src == SR_MIC)
		stop_record(sr->recorder);	

	while(sr->rec_stat != MSP_REC_STATUS_COMPLETE ){
		rslt = QISRGetResult(sr->session_id, &sr->rec_stat, 0, &errcode);
		if (rslt && sr->notif.on_result)
			sr->notif.on_result(rslt, sr->rec_stat == MSP_REC_STATUS_COMPLETE ? 1 : 0);

		Sleep(100); /* for cpu occupy, should sleep here */
	}

	if (sr->session_id) {
		if (sr->notif.on_speech_end)
			sr->notif.on_speech_end(END_REASON_VAD_DETECT);
		QISRSessionEnd(sr->session_id, "VAD Normal");
		sr->session_id = NULL;
	}
	sr->state = SR_STATE_INIT;
}

/* the record call back */
static void iat_cb(char *data, unsigned long len, void *user_para)
{
	int errcode;
	struct speech_rec *sr;

	if(len == 0 || data == NULL)
		return;

	sr = (struct speech_rec *)user_para;

	if(sr == NULL || sr->ep_stat >= MSP_EP_AFTER_SPEECH)
		return;
	if (sr->state < SR_STATE_STARTED)
		return; /* ignore the data if error/vad happened */
	
	errcode = sr_write_audio_data(sr, data, len);
	if (errcode) {
		end_sr_on_error(sr, errcode);
		return;
	}
}

static char * skip_space(char *s)
{
	while (s && *s != ' ' && *s != '\0')
		s++;
	return s;
}
static int update_format_from_sessionparam(const char * session_para, WAVEFORMATEX *wavefmt)
{
	char *s;
	if ((s = strstr(session_para, "sample_rate"))) {
		s = strstr(s, "=");
		if (s && *s) {
			s = skip_space(s);
			if (s && *s) {
				wavefmt->nSamplesPerSec = atoi(s);
				wavefmt->nAvgBytesPerSec = wavefmt->nBlockAlign * wavefmt->nSamplesPerSec;
			}
		}
		else
			return -1;
	}
	else {
		return -1;
	}

	return 0;
}

/* devid will be ignored if aud_src is not SR_MIC ; use get_default_dev_id
 * to use the default input device. Currently the device list function is
 * not provided yet. 
 */

int sr_init_ex(struct speech_rec * sr, const char * session_begin_params, 
			enum sr_audsrc aud_src, record_dev_id devid, 
				struct speech_rec_notifier * notify)
{
	int errcode;
	size_t param_size;
	WAVEFORMATEX wavfmt = DEFAULT_FORMAT;

	if (aud_src == SR_MIC && get_input_dev_num() == 0) {
		return -E_SR_NOACTIVEDEVICE;
	}

	if (!sr)
		return -E_SR_INVAL;

	if (session_begin_params == NULL) {
		session_begin_params = DEFAULT_SESSION_PARA;
	}

	SR_MEMSET(sr, 0, sizeof(struct speech_rec));
	sr->state = SR_STATE_INIT;
	sr->aud_src = aud_src;
	sr->ep_stat = MSP_EP_LOOKING_FOR_SPEECH;
	sr->rec_stat = MSP_REC_STATUS_SUCCESS;
	sr->audio_status = MSP_AUDIO_SAMPLE_FIRST;

	param_size = strlen(session_begin_params) + 1;
	sr->session_begin_params = (char*)SR_MALLOC(param_size);
	if (sr->session_begin_params == NULL) {
		sr_dbg("mem alloc failed\n");
		return -E_SR_NOMEM;
	}
	strncpy(sr->session_begin_params, session_begin_params, param_size);

	sr->notif = *notify;
	
	if (aud_src == SR_MIC) {
		errcode = create_recorder(&sr->recorder, iat_cb, (void*)sr);
		if (sr->recorder == NULL || errcode != 0) {
			sr_dbg("create recorder failed: %d\n", errcode);
			errcode = -E_SR_RECORDFAIL;
			goto fail;
		}
		update_format_from_sessionparam(session_begin_params, &wavfmt);
	
		errcode = open_recorder(sr->recorder, devid, &wavfmt);
		if (errcode != 0) {
			sr_dbg("recorder open failed: %d\n", errcode);
			errcode = -E_SR_RECORDFAIL;
			goto fail;
		}
	}

	return 0;

fail:
	if (sr->recorder) {
		destroy_recorder(sr->recorder);
		sr->recorder = NULL;
	}

	if (sr->session_begin_params) {
		SR_MFREE(sr->session_begin_params);
		sr->session_begin_params = NULL;
	}
	SR_MEMSET(&sr->notif, 0, sizeof(sr->notif));

	return errcode;
}

/* use the default input device to capture the audio. see sr_init_ex */
int sr_init(struct speech_rec * sr, const char * session_begin_params, 
		enum sr_audsrc aud_src, struct speech_rec_notifier * notify)
{
	return sr_init_ex(sr, session_begin_params, aud_src, 
			get_default_input_dev(), notify);
}

int sr_start_listening(struct speech_rec *sr)
{
	int ret;
	const char*		session_id = NULL;
	int				errcode = MSP_SUCCESS;

	if (sr->state >= SR_STATE_STARTED) {
		sr_dbg("already STARTED.\n");
		return -E_SR_ALREADY;
	}

	session_id = QISRSessionBegin(NULL, sr->session_begin_params, &errcode); //听写不需要语法，第一个参数为NULL
	if (MSP_SUCCESS != errcode)
	{
		sr_dbg("\nQISRSessionBegin failed! error code:%d\n", errcode);
		return errcode;
	}
	sr->session_id = session_id;
	sr->ep_stat = MSP_EP_LOOKING_FOR_SPEECH;
	sr->rec_stat = MSP_REC_STATUS_SUCCESS;
	sr->audio_status = MSP_AUDIO_SAMPLE_FIRST;

	if (sr->aud_src == SR_MIC) {
		ret = start_record(sr->recorder);
		if (ret != 0) {
			sr_dbg("start record failed: %d\n", ret);
			QISRSessionEnd(session_id, "start record fail");
			sr->session_id = NULL;
			return -E_SR_RECORDFAIL;
		}
	}

	sr->state = SR_STATE_STARTED;

	if (sr->notif.on_speech_begin)
		sr->notif.on_speech_begin();

	return 0;
}

/* after stop_record, there are still some data callbacks */
static void wait_for_rec_stop(struct recorder *rec, unsigned int timeout_ms)
{
	while (!is_record_stopped(rec)) {
		Sleep(1);
		if (timeout_ms != (unsigned int)-1)
			if (0 == timeout_ms--)
				break;
	}
}

int sr_stop_listening(struct speech_rec *sr)
{
	int ret = 0;
	const char * rslt = NULL;

	if (sr->state < SR_STATE_STARTED) {
		sr_dbg("Not started or already stopped.\n");
		return 0;
	}

	if (sr->aud_src == SR_MIC) {
		ret = stop_record(sr->recorder);
		if (ret != 0) {
			sr_dbg("Stop failed! \n");
			return -E_SR_RECORDFAIL;
		}
		wait_for_rec_stop(sr->recorder, (unsigned int)-1);
	}
	sr->state = SR_STATE_INIT;
	ret = QISRAudioWrite(sr->session_id, NULL, 0, MSP_AUDIO_SAMPLE_LAST, &sr->ep_stat, &sr->rec_stat);
	if (ret != 0) {
		sr_dbg("write LAST_SAMPLE failed: %d\n", ret);
		QISRSessionEnd(sr->session_id, "write err");
		return ret;
	}

	while (sr->rec_stat != MSP_REC_STATUS_COMPLETE) {
		rslt = QISRGetResult(sr->session_id, &sr->rec_stat, 0, &ret);
		if (MSP_SUCCESS != ret)	{
			sr_dbg("\nQISRGetResult failed! error code: %d\n", ret);
			end_sr_on_error(sr, ret);
			return ret;
		}
		if (NULL != rslt && sr->notif.on_result)
			sr->notif.on_result(rslt, sr->rec_stat == MSP_REC_STATUS_COMPLETE ? 1 : 0);
		Sleep(100);
	}

	QISRSessionEnd(sr->session_id, "normal");
	sr->session_id = NULL;
	return 0;
}

int sr_write_audio_data(struct speech_rec *sr, char *data, unsigned int len)
{
	const char *rslt = NULL;
	int ret = 0;
	if (!sr )
		return -E_SR_INVAL;
	if (!data || !len)
		return 0;

	ret = QISRAudioWrite(sr->session_id, data, len, sr->audio_status, &sr->ep_stat, &sr->rec_stat);
	if (ret) {
		end_sr_on_error(sr, ret);
		return ret;
	}
	sr->audio_status = MSP_AUDIO_SAMPLE_CONTINUE;

	if (MSP_REC_STATUS_SUCCESS == sr->rec_stat) { //已经有部分听写结果
		rslt = QISRGetResult(sr->session_id, &sr->rec_stat, 0, &ret);
		if (MSP_SUCCESS != ret)	{
			sr_dbg("\nQISRGetResult failed! error code: %d\n", ret);
			end_sr_on_error(sr, ret);
			return ret;
		}
		if (NULL != rslt && sr->notif.on_result)
			sr->notif.on_result(rslt, sr->rec_stat == MSP_REC_STATUS_COMPLETE ? 1 : 0);
	}

	if (MSP_EP_AFTER_SPEECH == sr->ep_stat)
		end_sr_on_vad(sr);

	return 0;
}

void sr_uninit(struct speech_rec * sr)
{
	if (sr->recorder) {
		if(!is_record_stopped(sr->recorder))
			stop_record(sr->recorder);
		close_recorder(sr->recorder);
		destroy_recorder(sr->recorder);
		sr->recorder = NULL;
	}

	if (sr->session_begin_params) {
		SR_MFREE(sr->session_begin_params);
		sr->session_begin_params = NULL;
	}
}
