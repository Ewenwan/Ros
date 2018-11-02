/*
* 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的
* 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的
* 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。
*/
#include "ros/ros.h"         //ros系统头文件
#include "std_msgs/String.h" 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

const char* filename =         "/home/ewenwan/ewenwan/ME/music/music.wav";
const char* playwavpath = "play /home/ewenwan/ewenwan/ME/music/music.wav";
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF" RIFF(Resource Interchange File Format) 资源交换 文件规范
	int		size_8;                 // = FileSize - 8 从下个地址开始到文件尾的总字节数
	char            wave[4];                // = "WAVE"  WAV文件标志（WAVE）
	char            fmt[4];                 // = "fmt "  波形格式标志（fmt ），最后一位空格。
	int		fmt_size;		// = 下一个结构体的大小 : 16  过滤字节（一般为00000010H，16d），若为00000012H则说明数据头携带附加信息（见“附加信息”）。

	short int       format_tag;             // = PCM : 1       格式种类（值为1时，表示数据为线性PCM编码）
	short int       channels;               // = 通道数 : 1     通道数，单声道为1，双声道为2
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' }, //文件规范
	0,                      //后面数据的大小（前面有4个字节）
	{'W', 'A', 'V', 'E'},   //文件格式
	{'f', 'm', 't', ' '},   //波形格式标志 最后一位 空
	16,                     //过滤字节数
	1,                      //格式种类（值为1时，表示数据为线性PCM编码）
	1,                      //通道数，单声道为1，双声道为2
	16000,                  //采样率 : 8000 | 6000 | 11025 | 16000
	32000,                  //每秒字节数 : samples_per_sec * bits_per_sample / 8
	2,                      //每采样点字节数 : wBitsPerSample / 8
	16,                     //量化比特数: 8 | 16
	{'d', 'a', 't', 'a'},   //"data";
	0                       //纯音频数据长度 数据长度 : FileSize - 44 （前面有40个字节）
};
/* 文本合成 函数  text, filename, session_begin_params */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;    //返回参数
	FILE*        fp           = NULL;  //文件句柄 文件头
	const char*  sessionID    = NULL;  //
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr; //wav文件头
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)  //原文本空或者 生成的文件名路径为空 返回
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");                //二进制格式写入
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);  //打开文件失败
		return ret;
	}
	/* 开始合成 合成语音参数 标志*/
	sessionID = QTTSSessionBegin(params, &ret);//开始一次语音合成，分配语音合成资源。
	if (MSP_SUCCESS != ret)                    //参数不合规范
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);                           //关闭文件
		return ret;
	}
	  // int MSPAPI QTTSTextPut 	( const char * sessionID, const char * textString, unsigned int  textLen, const char *  params ) 	
	  // 由QTTSSessionBegin返回的句柄  字符串指针,指向待合成的文本字符串    合成文本长度,最大支持8192个字节（2730个汉字）   本次合成所用的参数，只对本次合成的文本有效。      
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);//写入待合成的文本。
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError"); //发生错误的话 就结束本次语音合成。 
		fclose(fp);                                //关闭文件
		return ret;
	}
	printf("正在合成 ...\n");
	//写入文件  头信息     大小    写入次数   文件名
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */ //    由QTTSSessionBegin返回的句柄   合成音频长度,单位字节   合成音频状态  成功与否 
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret); //获取合成音频。 
		if (MSP_SUCCESS != ret) //为成功
			break;
		if (NULL != data)       //合成的音频有内容
		{
			fwrite(data, audio_len, 1, fp);    //写入内容 长度 次数 文件名
		    wav_hdr.data_size += audio_len;        //总音频长度计算data_size大小  用于记录头文件 size_8 大小  
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status) //合成的音频已经取完
			break;
		printf(">");
		usleep(150*1000);                          //防止频繁占用CPU
	}
	printf("\n");
	if (MSP_SUCCESS != ret)                        //获取音频未成功
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");//发生错误的话 就结束本次语音合成。
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8); //头文件长度+音频长度 -8 
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);  //偏移4个字节 从第4个字节开始写入 size_8
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp);       //写入size_8的值
	fseek(fp, 40, 0); //偏移4个字节                                //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);                                                  //关闭文件
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");                   //结束本次语音合成。
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}
/*
make topic Text To Wave file
*/
int makeTextToWave(const char* text, const char* filename)
{
	int         ret                  = MSP_SUCCESS;                     //默认返回参数
	/* appid 	应用ID。*/
	const char* login_params         = "appid = 58dbcf6e, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
	/*
	* rdn:           合成音频数字发音方式 0：数值优先（车牌号报读）,1：完全数值(1000 一千),2：完全字符串，3：字符串优先。 默认为0 
	* volume:        合成音频的音量     [0,100]，数值越大音量越大。默认为50 
	* pitch:         合成音频的音调     [0,100]，数值越大音调越高。默认为50 
	* speed:         合成音频对应的语速 [0,100]，数值越大语速越快。默认为50 
	* voice_name:    合成发音人        xiaoyan yanping jinger yufeng donaldduck xiaowanzi babyxu nannan xiaoqian（东北话）
	* sample_rate:   合成音频采样率     
	* text_encoding: 合成文本编码格式   GB2312；GBK；BIG5；UNICODE；GB18030；UTF8 
	* background_sound 	合成音频中的背景音 0：无背景音乐 1：有背景音乐。默认为0 
	* 详细参数说明请参阅《讯飞语音云MSC--API文档》
	*/
	const char* session_begin_params = "voice_name = donaldduck, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 0";
	//const char* filename             = "tts_sample.wav"; //合成的语音文件名称
	//const char* text                 = "大家好，我叫小明，车牌号沪A1005，两套房子，五百万存款"; //合成文本

	/* 用户登录 int MSPAPI MSPLogin ( const char *  usr,const char *  pwd,const char *  params )*/
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		goto exit ;//登录失败，退出登录
	}
	
		/* 文本合成 */
	  printf("开始合成 ...\n");
	  /*要合成语音的文本  合成后的语音文件wav名字  合成语音的参数*/
	  ret = text_to_speech(text, filename, session_begin_params);
	  if (MSP_SUCCESS != ret)
	   {
		printf("text_to_speech failed, error code: %d.\n", ret);
	   }
	   else{
	     printf("合成完毕\n");
	    }
exit:
	MSPLogout(); //退出登录
	
	//return 0;
}
/*播放 wav文件*/
void playWav()
{
	system(playwavpath);
}
  /*subcribe topic callbace function*/
void chatterCallback(const std_msgs::String::ConstPtr&  msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
	//const char* text=msg->data.c_str();
    std::cout << " I hear topic text:" << msg->data.c_str() << std::endl;
	makeTextToWave( msg->data.c_str() , filename );
	playWav();
	ROS_INFO("READY TO DO TTS");
}
   
int main(int argc, char** argv)
{
	 const char* start_voice ="科大讯飞在线语音合成启动成功";
	 makeTextToWave( start_voice , filename );
	 playWav();
	 ROS_INFO("READY TO TTS");
	 
	 ros::init(argc, argv, "xf_tts_node"); //初始化ros系统 ，在roscore节点管理器注册节点
     ros::NodeHandle nhd;                   //节点句柄
	 //节点创建一个发布者
     //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("pub_hello_ros", 1000);
	 //创建一个订阅者sub     节点句柄         话题            缓存区    函数指针   &callbackfunc 得到
     ros::Subscriber sub = nhd.subscribe("voice/xf_tts_topic", 100, &chatterCallback);
	 
	 //ros::spin();
     ros::Rate  rate(5);              //发布频率
     while(ros::ok()){
       ros::spinOnce();               //给ROS控制权  可以调用一次回调函数
       rate.sleep();
     }	 
	return 0;
}

