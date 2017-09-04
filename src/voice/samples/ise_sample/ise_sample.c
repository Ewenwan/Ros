#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#define FRAME_LEN	640

int main(int argc, char* argv[])
{

	const char *  lgi_param     = "appid = 58dbcf6e";
	const char *  ise_ssb_param = NULL;
	const char *  sessionID     = NULL;
	long          pcmSize       = 0;
	long          pcmCount      = 0;
	long          txtSize       = 0;
	FILE *        f_pcm         = NULL;
	FILE *        f_txt         = NULL;
	FILE *        f_output      = NULL;
	char *        pPCM          = NULL;
	char *        text          = NULL;
	unsigned int  rlstLen       = 0;
	int           audStat       = MSP_AUDIO_SAMPLE_CONTINUE;
	int           epStatus      = MSP_EP_LOOKING_FOR_SPEECH;
	int           recStatus     = MSP_REC_STATUS_SUCCESS;
	int           ret           = 0;
	enum _category
	{
		read_syllable_cn,
		read_word_cn,
		read_sentence_cn,
		read_word_en,
		read_sentence_en,
	}category;

	ret = MSPLogin(NULL, NULL, lgi_param);
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed error = %d\n",ret);
	}

	category = read_sentence_cn ;
	switch(category)
	{
	case read_syllable_cn:
		ise_ssb_param = "sub=ise,category=read_syllable,language=zh_cn,aue=speex-wb;7,auf=audio/L16;rate=16000";
		//f_txt = fopen("./ise_cn/cn_syll.txt", "rb+");
		f_txt = fopen("./ise_cn/cn_syll2.txt", "rb+");//无拼音标注
		f_pcm = fopen("./ise_cn/cn_syll.wav", "rb+");
		break;
	case read_word_cn:
		ise_ssb_param = "sub=ise,category=read_word,language=zh_cn,aue=speex-wb;7,auf=audio/L16;rate=16000";
		f_txt = fopen("./ise_cn/cn_word.txt", "rb+");
		f_pcm = fopen("./ise_cn/cn_word.wav", "rb+");
		break;
	case read_sentence_cn:
		ise_ssb_param = "sub=ise,category=read_sentence,language=zh_cn,aue=speex-wb;7,auf=audio/L16;rate=16000";
		f_txt = fopen("./ise_cn/cn_sentence.txt", "rb+");
		f_pcm = fopen("./ise_cn/cn_sentence.wav", "rb+");
		break;
	case read_word_en:
		ise_ssb_param = "sub=ise,category=read_word,language=en_us,aue=speex-wb;7,auf=audio/L16;rate=16000";
		f_txt = fopen("./ise_en/en_word.txt", "rb+");
		f_pcm = fopen("./ise_en/en_word.wav", "rb+");
		break;
	case read_sentence_en:
		ise_ssb_param = "sub=ise,category=read_sentence,language=en_us,aue=speex-wb;7,auf=audio/L16;rate=16000";
		f_txt = fopen("./ise_en/en_sentence.txt", "rb+");
		f_pcm = fopen("./ise_en/en_sentence.wav", "rb+");
		break;
	}
	sessionID = QISESessionBegin(ise_ssb_param, NULL, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QISESessionBegin failed error = %d\n",ret);
	}
	if (NULL != f_txt)
	{
		fseek(f_txt, 0, SEEK_END);
		txtSize = ftell(f_txt);
		fseek(f_txt, 0, SEEK_SET);
		text = (char *)malloc(txtSize);
		fread((void *)text, txtSize, 1, f_txt);
		text[txtSize] = '\0';
		fclose(f_txt);
		f_txt = NULL;
	}
	ret = QISETextPut(sessionID, text, (unsigned int)strlen(text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QISETextPut failed error = %d\n",ret);
	}
	if (NULL != f_pcm) {
		fseek(f_pcm, 0, SEEK_END);
		pcmSize = ftell(f_pcm);
		fseek(f_pcm, 0, SEEK_SET);
		pPCM = (char *)malloc(pcmSize);
		fread((void *)pPCM, pcmSize, 1, f_pcm);
		fclose(f_pcm);
		f_pcm = NULL;
	}
	f_output = fopen("output.txt","wb+");
	if (f_output == NULL)
	{
		printf("open output.txt failed\n");
		goto ise_exit;
	}

	while (1) 
	{
		unsigned int len = 10 * FRAME_LEN;// 每次写入200ms音频(16k，16bit)：1帧音频20ms，10帧=200ms。16k采样率的16位音频，一帧的大小为640Byte
		int ret;

		if (pcmSize <= 2*len) 
		{
			len = pcmSize;
		}
		if (len <= 0)
		{
			break;
		}
		audStat = MSP_AUDIO_SAMPLE_CONTINUE;
		if (0 == pcmCount)
		{
			audStat = MSP_AUDIO_SAMPLE_FIRST;
		}
		printf(">");
		ret = QISEAudioWrite(sessionID, (const void *)&pPCM[pcmCount], len, audStat, &epStatus, &recStatus);
		if (MSP_SUCCESS != ret)
		{
			printf("QISEAudioWrite failed error = %d\n",ret);
			goto ise_exit;
		}	

		pcmCount += (long)len;
		pcmSize -= (long)len;
		if (MSP_REC_STATUS_SUCCESS == recStatus) 
		{
			const char *rslt = QISEGetResult(sessionID, &rlstLen,&recStatus, &ret);
			if (MSP_SUCCESS != ret)
			{
				printf("QISEGetResult failed ,error = %d\n",ret);
				goto ise_exit;
			}
			
			if (NULL != rslt)
			{
				printf("\nrslt = %s\n,rlstLen =%u \n", rslt,rlstLen);
				fwrite(rslt,rlstLen,1,f_output);
			}
			
		}
		if (MSP_EP_AFTER_SPEECH == epStatus)
		{
			break;
		}
		usleep(200*1000);
	}
	ret = QISEAudioWrite(sessionID, (const void *)NULL, 0, 4, &epStatus, &recStatus);
	if (MSP_SUCCESS != ret)
	{
		printf("QISEAudioWrite failed error = %d\n",ret);
		goto ise_exit;
	}
		
	while (MSP_REC_STATUS_COMPLETE != recStatus) 
	{
		const char *rslt = QISEGetResult(sessionID, &rlstLen,&recStatus, &ret);
		if (MSP_SUCCESS != ret)
		{
			printf("QISEGetResult failed ,error = %d\n",ret);
			goto ise_exit;
		}
		if (NULL != rslt)
		{
			printf("\nrslt = %s\n,rlstLen =%u \n", rslt,rlstLen);
			fwrite(rslt,rlstLen,1,f_output);
		}
		usleep(150*1000);
	}
	
	printf("=============================================================\n");
	printf("=============================================================\n");

ise_exit:
	if (NULL != pPCM)
	{
		free(pPCM);
		pPCM = NULL;
	}
	if (f_output!=NULL)
	{
		fclose(f_output);
		f_output = NULL;
	}
	ret = QISESessionEnd(sessionID, NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QISESessionEnd failed error = %d\n",ret);
	}
	ret = MSPLogout();
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogout failed error = %d\n",ret);
	}
	printf("press any key to exit ...\n");
	getchar();
	return 0;
}