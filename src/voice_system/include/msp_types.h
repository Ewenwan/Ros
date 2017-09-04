#ifndef __MSP_TYPES_H__
#define __MSP_TYPES_H__

#if !defined(MSPAPI)
#if defined(WIN32) || defined(WINPHONE8) || defined(WIN8)
	#define MSPAPI __stdcall
#else
	#define MSPAPI
#endif /* WIN32 */
#endif /* MSPAPI */


/**
 *  MSPSampleStatus indicates how the sample buffer should be handled
 *  MSP_AUDIO_SAMPLE_FIRST		- The sample buffer is the start of audio
 *								  If recognizer was already recognizing, it will discard
 *								  audio received to date and re-start the recognition
 *  MSP_AUDIO_SAMPLE_CONTINUE	- The sample buffer is continuing audio
 *  MSP_AUDIO_SAMPLE_LAST		- The sample buffer is the end of audio
 *								  The recognizer will cease processing audio and
 *								  return results
 *  Note that sample statii can be combined; for example, for file-based input
 *  the entire file can be written with SAMPLE_FIRST | SAMPLE_LAST as the
 *  status.
 *  Other flags may be added in future to indicate other special audio
 *  conditions such as the presence of AGC
 */
enum
{
	MSP_AUDIO_SAMPLE_INIT           = 0x00,
    MSP_AUDIO_SAMPLE_FIRST          = 0x01,
    MSP_AUDIO_SAMPLE_CONTINUE       = 0x02,
    MSP_AUDIO_SAMPLE_LAST           = 0x04,
};

/*
 *  The enumeration MSPRecognizerStatus contains the recognition status
 *  MSP_REC_STATUS_SUCCESS				- successful recognition with partial results
 *  MSP_REC_STATUS_NO_MATCH				- recognition rejected
 *  MSP_REC_STATUS_INCOMPLETE			- recognizer needs more time to compute results
 *  MSP_REC_STATUS_NON_SPEECH_DETECTED	- discard status, no more in use
 *  MSP_REC_STATUS_SPEECH_DETECTED		- recognizer has detected audio, this is delayed status
 *  MSP_REC_STATUS_COMPLETE				- recognizer has return all result
 *  MSP_REC_STATUS_MAX_CPU_TIME			- CPU time limit exceeded
 *  MSP_REC_STATUS_MAX_SPEECH			- maximum speech length exceeded, partial results may be returned
 *  MSP_REC_STATUS_STOPPED				- recognition was stopped
 *  MSP_REC_STATUS_REJECTED				- recognizer rejected due to low confidence
 *  MSP_REC_STATUS_NO_SPEECH_FOUND		- recognizer still found no audio, this is delayed status
 */
enum
{
	MSP_REC_STATUS_SUCCESS              = 0,
	MSP_REC_STATUS_NO_MATCH             = 1,
	MSP_REC_STATUS_INCOMPLETE			= 2,
	MSP_REC_STATUS_NON_SPEECH_DETECTED  = 3,
	MSP_REC_STATUS_SPEECH_DETECTED      = 4,
	MSP_REC_STATUS_COMPLETE				= 5,
	MSP_REC_STATUS_MAX_CPU_TIME         = 6,
	MSP_REC_STATUS_MAX_SPEECH           = 7,
	MSP_REC_STATUS_STOPPED              = 8,
	MSP_REC_STATUS_REJECTED             = 9,
	MSP_REC_STATUS_NO_SPEECH_FOUND      = 10,
	MSP_REC_STATUS_FAILURE = MSP_REC_STATUS_NO_MATCH,
};

/**
 * The enumeration MSPepState contains the current endpointer state
 *  MSP_EP_LOOKING_FOR_SPEECH	- Have not yet found the beginning of speech
 *  MSP_EP_IN_SPEECH			- Have found the beginning, but not the end of speech
 *  MSP_EP_AFTER_SPEECH			- Have found the beginning and end of speech
 *  MSP_EP_TIMEOUT				- Have not found any audio till timeout
 *  MSP_EP_ERROR				- The endpointer has encountered a serious error
 *  MSP_EP_MAX_SPEECH			- Have arrive the max size of speech
 */
enum
{
	MSP_EP_LOOKING_FOR_SPEECH   = 0,
	MSP_EP_IN_SPEECH            = 1,
	MSP_EP_AFTER_SPEECH         = 3,
	MSP_EP_TIMEOUT              = 4,
	MSP_EP_ERROR                = 5,
	MSP_EP_MAX_SPEECH           = 6,
	MSP_EP_IDLE                 = 7  // internal state after stop and before start
};

/* Synthesizing process flags */
enum
{
    MSP_TTS_FLAG_STILL_HAVE_DATA        = 1,
    MSP_TTS_FLAG_DATA_END               = 2,
    MSP_TTS_FLAG_CMD_CANCELED           = 4,
};

/* Handwriting process flags */
enum
{
	MSP_HCR_DATA_FIRST           = 1,
	MSP_HCR_DATA_CONTINUE        = 2,
	MSP_HCR_DATA_END             = 4,
};

/*ivw message type */
enum
{
	MSP_IVW_MSG_WAKEUP       = 1,
	MSP_IVW_MSG_ERROR        = 2,
	MSP_IVW_MSG_ISR_RESULT   = 3,
	MSP_IVW_MSG_ISR_EPS      = 4,
	MSP_IVW_MSG_VOLUME       = 5,
	MSP_IVW_MSG_ENROLL       = 6,
	MSP_IVW_MSG_RESET        = 7
};

/* Upload data process flags */
enum
{
	MSP_DATA_SAMPLE_INIT           = 0x00,
	MSP_DATA_SAMPLE_FIRST          = 0x01,
	MSP_DATA_SAMPLE_CONTINUE       = 0x02,
	MSP_DATA_SAMPLE_LAST           = 0x04,
};

#endif /* __MSP_TYPES_H__ */
