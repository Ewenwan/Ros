/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
* voice Activity Detection 语音活动检测  一句话是否说完了
*/

#include<ros/ros.h>
#include<std_msgs/String.h>
#include<std_msgs/Int32.h>

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
#define ASRFLAG     1      //控制命令

using namespace std;       //命名空间
//全局变量
bool flag = false;         //发布语音识别结果话题  标志
bool recorder_Flag = true; //可录音标志，开始麦克风录音并上传识别
string result = "";        //识别结果字符串

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
static void show_result(char *str, char is_over)
{
	printf("\rResult: [ %s ]", str);
	if(is_over)
		putchar('\n');
		
	string s(str);
	result = s;   //得到语音识别结果
	flag = true;  //设置发布话题为真
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
/*一段新的语句开始*/
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
/* 说话结束*/
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT) //已经检测到 语句vad断点了  就是说了一句话了
	{
		printf("\nSpeaking done \n");
		recorder_Flag = false;         //录音标志  录音结束
	}
	else
		printf("\nRecognizer error %d\n", reason);
}

/* 从麦克风识别语音*/
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	
	while(recorder_Flag){ //当可录音标志为真时 开始录音并识别
		sleep(1);
	}
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}

 int asrToText()
  {
	
	int ret = MSP_SUCCESS;
	int upload_on =	1; /* whether upload the user word */
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 58dbcf6e, work_dir = ."; //登陆参数
	int aud_src = 0; /* from mic or file */

	/*
	*  See "iFlytek MSC Reference Manual"
	*  识别参数
	*/
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* Login first. the 1st arg is username, the 2nd arg is password
	 * just set them as NULL. the 3rd arg is login paramertes
	 * 登陆
	 * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}

    /*字典上传？*/
	/*	
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
	*/
	
   /*声音来源  麦克分或 wav文件*/
   /*
	printf("Where the audio comes from?\n0: From a audio file.\n1: From microphone.\n");
	scanf("%d", &aud_src);//键盘输入
	if(aud_src != 0) {
		printf("Demo recognizing the speech from microphone\n");
		printf("Speak in 15 seconds\n");

		demo_mic(session_begin_params);

		printf("15 sec passed\n");
	} else {
		printf("Demo recgonizing the speech from a recorded audio file\n");
		demo_file("wav/iflytek02.wav", session_begin_params); //选者wav文件
	}
	*/
   demo_mic(session_begin_params);
	
exit:
	MSPLogout(); // Logout...
 }


/*
*   根据发布的话题来修改录音标志
*/


void asrCallBack(const std_msgs::Int32::ConstPtr &msg)
{ 
        ROS_INFO_STREAM("Topic is Subscriber, now starting voice recognition");
        if(msg->data == ASRFLAG)  //话题收到相应的 语音识别激活标志
        {
           asrToText();
        }
}

/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char* argv[])
{
     ros::init(argc, argv, "xf_asr_node"); //初始化ros系统 ，在roscore节点管理器注册节点
     ros::NodeHandle nhd;                   //节点句柄
	 //创建一个订阅者sub     节点句柄         话题            缓存区    函数指针   &callbackfunc 得到
     ros::Subscriber sub = nhd.subscribe("voice/xf_asr_topic", 20, &asrCallBack);
	 //节点创建一个发布者
	 ros::Publisher pub = nhd.advertise<std_msgs::String>("voice/tl_nlu_topic", 20);
	 ROS_INFO("please publish the kinds of std_msgs/Int32  1 to the topic voice/xf_asr_topic");
	 ROS_INFO("waitting for the running sign of voice recognition");
	 //ros::spin();
     ros::Rate  rate(10);              //频率
     while(ros::ok()){
		if(flag)                       //成功获取到返回数据
		{
			std_msgs::String msg;
			msg.data = result;
			pub.publish(msg);
			recorder_Flag = true;     //可开始录音识别
			flag = false ;            //发布识别结果 话题 标志 为否
		}
       ros::spinOnce();               //给ROS控制权  可以调用一次回调函数
       rate.sleep();
     }	 
	return 0;;
}
