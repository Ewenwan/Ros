/*
* 文本语义技术能将文本内容进行语义解析。
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "msp_cmn.h"
#include "msp_errors.h"

#define SOURCETEXT  "source.txt" //语义文本资源
#define RESULTTEXT  "result.txt" //语义结果文本

int main(int argc, char* argv[])
{
	const char*  login_params	 =	"appid = 58dbcf6e"; // 登录参数，appid与msc库绑定,请勿随意改动
	const char*  rec_text        =   NULL;
	unsigned int str_len         =   0;
	int	         ret		     =	 MSP_SUCCESS;
	FILE*        fw              =   NULL;
	FILE*        fr              =   NULL;
	long         txtSize         =   0;
	long		 read_size	     =	 0;
	char*        text            =   NULL;
	/* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，均传NULL即可，第三个参数是登录参数	
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; //登录失败，退出登录
	}
	fr=fopen(SOURCETEXT,"rb");
	if(NULL == fr)
	{
		printf("\nopen [%s] failed! \n",SOURCETEXT);
		goto exit;
	}

	fseek(fr, 0, SEEK_END);
	txtSize = ftell(fr);
	fseek(fr, 0, SEEK_SET);

	text = (char *)malloc(txtSize+1);
	if (NULL == text)
	{
		printf("\nout of memory! \n");
		goto exit;
	}

	read_size = fread((void *)text,1, txtSize, fr);
	if (read_size != txtSize)
	{
		printf("\nread [%s] error!\n", SOURCETEXT);
		goto exit;
	}
	text[txtSize]='\0';
	str_len = strlen(text);

	printf("\n开始语义解析...\n");
	rec_text = MSPSearch("nlp_version=2.0",text,&str_len,&ret);
	if(MSP_SUCCESS !=ret)
	{
		printf("MSPSearch failed ,error code is:%d\n",ret);
		goto exit;
	}
	printf("\n语义解析完成!\n");
	fw=fopen(RESULTTEXT,"wb");
	if(NULL == fw)
	{
		printf("\nopen [%s] failed! \n",RESULTTEXT);
		goto exit;
	}

	read_size = fwrite(rec_text,1,str_len,fw);
	if(read_size != str_len)
	{
		printf("\nwrite [%s] error!\n", RESULTTEXT);
		goto exit;
	}
	printf("\n语义解析结果已写入%s文件\n",RESULTTEXT);
exit:
	if (NULL != fr)
	{
		fclose(fr);
		fr = NULL;
	}
	if (NULL != fw)
	{
		fclose(fw);
		fw = NULL;
	}
	if (NULL != text)
	{
		free(text);
		text = NULL;
	}
	printf("\n按任意键退出 ...\n");
	getchar();
	MSPLogout(); //退出登录

	return 0;
}
