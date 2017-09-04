/*
 * @file
 * @brief a record demo in linux
 *
 * a simple record code. using alsa-lib APIs.
 * keep the function same as winrec.h
 *
 * Common steps:
 *	create_recorder,
 *	open_recorder, 
 *	start_record, 
 *	stop_record, 
 *	close_recorder,
 *	destroy_recorder
 *
 * @author		taozhang9
 * @date		2016/06/01
 */

#ifndef __IFLY_WINREC_H__
#define __IFLY_WINREC_H__

#include "formats.h"
/* error code */
enum {
	RECORD_ERR_BASE = 0,
	RECORD_ERR_GENERAL,
	RECORD_ERR_MEMFAIL,
	RECORD_ERR_INVAL,
	RECORD_ERR_NOT_READY
};

typedef struct {
	union {
		char *	name;
		int	index;
		void *	resv;
	}u;
}record_dev_id;

/* recorder object. */
struct recorder {
	void (*on_data_ind)(char *data, unsigned long len, void *user_para);
	void * user_cb_para;
	volatile int state;		/* internal record state */

	void * wavein_hdl;
	/* thread id may be a struct. by implementation 
	 * void * will not be ported!! */
	pthread_t rec_thread; 
	/*void * rec_thread_hdl;*/

	void * bufheader;
	unsigned int bufcount; 
	
	char *audiobuf;
	int bits_per_frame;
	unsigned int buffer_time;
	unsigned int period_time;
	size_t period_frames;
	size_t buffer_frames;
};

#ifdef __cplusplus
extern "C" {
#endif /* C++ */

/** 
 * @fn
 * @brief	Get the default input device ID
 *
 * @return	returns "default" in linux.
 *
 */
record_dev_id get_default_input_dev();

/**
 * @fn 
 * @brief	Get the total number of active input devices.
 * @return	
 */
int get_input_dev_num();

/**
 * @fn 
 * @brief	Create a recorder object.
 *
 * Never call the close_recorder in the callback function. as close
 * action will wait for the callback thread to quit. 
 *
 * @return	int			- Return 0 in success, otherwise return error code.
 * @param	out_rec		- [out] recorder object holder
 * @param	on_data_ind	- [in]	callback. called when data coming.
 * @param	user_cb_para	- [in] user params for the callback.
 * @see
 */
int create_recorder(struct recorder ** out_rec, 
				void (*on_data_ind)(char *data, unsigned long len, void *user_para), 
				void* user_cb_para);

/**
 * @fn 
 * @brief	Destroy recorder object. free memory. 
 * @param	rec	- [in]recorder object
 */
void destroy_recorder(struct recorder *rec);

/**
 * @fn 
 * @brief	open the device.
 * @return	int			- Return 0 in success, otherwise return error code.
 * @param	rec			- [in] recorder object
 * @param	dev			- [in] device id, from 0.
 * @param	fmt			- [in] record format.
 * @see
 * 	get_default_input_dev()
 */
int open_recorder(struct recorder * rec, record_dev_id dev, WAVEFORMATEX * fmt);

/**
 * @fn
 * @brief	close the device.
 * @param	rec			- [in] recorder object
 */

void close_recorder(struct recorder *rec);

/**
 * @fn
 * @brief	start record.
 * @return	int			- Return 0 in success, otherwise return error code.
 * @param	rec			- [in] recorder object
 */
int start_record(struct recorder * rec);

/**
 * @fn
 * @brief	stop record.
 * @return	int			- Return 0 in success, otherwise return error code.
 * @param	rec			- [in] recorder object
 */
int stop_record(struct recorder * rec);

/**
 * @fn
 * @brief	test if the recording has been stopped.
 * @return	int			- 1: stopped. 0 : recording.
 * @param	rec			- [in] recorder object
 */
int is_record_stopped(struct recorder *rec);

#ifdef __cplusplus
} /* extern "C" */	
#endif /* C++ */

#endif
