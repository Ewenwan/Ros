/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096


/* 上传用户字典*/
static int upload_userwords()
{
	char*			userwords	=	NULL; //
	size_t			len			=	0;    //
	size_t			read_len	=	0;    //
	FILE*			fp			=	NULL; //
	int				ret			=	-1;   //

	fp = fopen("userwords.txt", "rb");    //打开用户字典文本
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n"); //打开失败
		goto upload_exit;
	}

	fseek(fp, 0, SEEK_END);                          //到文件结尾
	len = ftell(fp);                                 //偏移长度 字节  相当于文件大小
	fseek(fp, 0, SEEK_SET);  	                     //到文件头				
	
	userwords = (char*)malloc(len + 1);              //申请文件字节+1 大小的缓存区
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");               //缓存区申请失败
		goto upload_exit;
	}

	read_len = fread((void*)userwords, 1, len, fp);  //读取文件写入到 缓存区
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");  //文件应该相等
		goto upload_exit;
	}
	userwords[len] = '\0';                           //添加结束符号
	
	//用户数据上传   数据名称字符串 	待上传数据缓冲区的起始地址。  数据长度(如果是字符串，则不包含'\0')。
	//params[in]  "sub = uup,dtt = userword" 	上传用户词表 	iat业务 	UTF-8 编码
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //上传
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);//上传失败
		goto upload_exit;
	}
	
	//退出
upload_exit:
	if (NULL != fp)
	{
		fclose(fp); //关闭文件
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);//释放缓存区
		userwords = NULL;
	}	
	return ret;
}


/*打印结果*/
static void show_result(char *string, char is_over)
{
	printf("\rResult: [ %s ]", string);
	if(is_over)
		putchar('\n');
}

//全局变量
static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;
//对结果采取的措施
void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size);
		show_result(g_result, is_last);
	}
}
/*开始说话*/
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}


/* 读取wav文件 并上传识别 */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	//三个函数指针
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};
	
   //打开文件
	if (NULL == audio_file)   //没有文件
		goto iat_exit;
	f_pcm = fopen(audio_file, "rb");//打开文件 读取 二进制
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file); //打开文件失败
		goto iat_exit;
	}

	//获取文件大小
	fseek(f_pcm, 0, SEEK_END); //最后
	pcm_size = ftell(f_pcm);   //偏移
	fseek(f_pcm, 0, SEEK_SET); //最前面
    //申请缓存区
	p_pcm = (char *)malloc(pcm_size);//申请缓存区
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");//申请内存失败
		goto iat_exit;
	}
    //读文件数据
	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);//读取文件到内存中
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);//失败
		goto iat_exit;
	}
    // speech_recognizer.c 中 语音识别初始化
	// 
	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;
	}
    //speech_recognizer.c 中  开启一次语音识别
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;
	}

	while (1)
	{
		unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;
		
        // 分段上传音频  检测到一句话说完 输出识别结果 
		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);
		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);//上传失败
			goto iat_exit;
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}
    //speech_recognizer.c 中
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;
	}

// 退出 goto 语句
iat_exit:
	if (NULL != f_pcm)
	{
		fclose(f_pcm);
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);
		p_pcm = NULL;
	}

	sr_stop_listening(&iat); //停止监听
	sr_uninit(&iat);         //停止识别
}



/* 从麦克风识别语音 */
static void demo_mic(const char* session_begin_params)
{
	int errcode;//任务返回参数 标志
	int i = 0;  //监听时间 记录

	struct speech_rec iat;                     //语音识别结构体

	struct speech_rec_notifier recnotifier = { //检测 语句 的 开始和结束
		on_result,
		on_speech_begin,
		on_speech_end
	};

	//语音识别初始化  speech_recognizer.c         SR_MIC 代指使用麦克风
	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	//开始识别  speech_recognizer.c
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* 记录时间 */
	while(i++ < 15)
		sleep(1);
		
	//停止监听
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}
	
    //语音识别结束
	sr_uninit(&iat);
}



/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char* argv[])
{
	int ret = MSP_SUCCESS;
	int upload_on =	1; /* whether upload the user word */
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 58dbcf6e, work_dir = .";
	int aud_src = 0; /* from mic or file */

	/*
	* See "iFlytek MSC Reference Manual"
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes 
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}
	
    /*是否上传用户词典*/
	printf("Want to upload the user words ? \n0: No.\n1: Yes\n");
	scanf("%d", &upload_on);
	if (upload_on)
	{
		printf("Uploading the user words ...\n");
		ret = upload_userwords();
		if (MSP_SUCCESS != ret)
			goto exit;	
		printf("Uploaded successfully\n");
	}
	
   /*声音来源 麦克风 ？ wav？*/
	printf("Where the audio comes from?\n0: From a audio file.\n1: From microphone.\n");
	scanf("%d", &aud_src);
	if(aud_src != 0) {
		printf("Demo recognizing the speech from microphone\n");
		printf("Speak in 15 seconds\n");

		demo_mic(session_begin_params);

		printf("15 sec passed\n");
	} else {
		printf("Demo recgonizing the speech from a recorded audio file\n");
		demo_file("wav/iflytek02.wav", session_begin_params); 
	}
	
exit:
	MSPLogout(); // Logout...

	return 0;
}
