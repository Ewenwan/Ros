/*
@file
@brief 基于录音接口和讯飞MSC接口封装一个MIC录音识别的模块

@author		taozhang9
@date		2016/05/27
*/


enum sr_audsrc
{
	SR_MIC,	/* write data from mic */
	SR_USER	/* write data from user by calling API */
};

//#define DEFAULT_INPUT_DEVID     (-1)


#define E_SR_NOACTIVEDEVICE		1
#define E_SR_NOMEM				2
#define E_SR_INVAL				3
#define E_SR_RECORDFAIL			4
#define E_SR_ALREADY			5


struct speech_rec_notifier {
	void (*on_result)(const char *result, char is_last);
	void (*on_speech_begin)();
	void (*on_speech_end)(int reason);	/* 0 if VAD.  others, error : see E_SR_xxx and msp_errors.h  */
};

#define END_REASON_VAD_DETECT	0	/* detected speech done  */

struct speech_rec {
	enum sr_audsrc aud_src;  /* from mic or manual  stream write */
	struct speech_rec_notifier notif;
	const char * session_id;
	int ep_stat;
	int rec_stat;
	int audio_status;
	struct recorder *recorder;
	volatile int state;
	char * session_begin_params;
};


#ifdef __cplusplus
extern "C" {
#endif

/* must init before start . is aud_src is SR_MIC, the default capture device
 * will be used. see sr_init_ex */
int sr_init(struct speech_rec * sr, const char * session_begin_params, enum sr_audsrc aud_src, struct speech_rec_notifier * notifier);
int sr_start_listening(struct speech_rec *sr);
int sr_stop_listening(struct speech_rec *sr);
/* only used for the manual write way. */
int sr_write_audio_data(struct speech_rec *sr, char *data, unsigned int len);
/* must call uninit after you don't use it */
void sr_uninit(struct speech_rec * sr);

#ifdef __cplusplus
} /* extern "C" */	
#endif /* C++ */
