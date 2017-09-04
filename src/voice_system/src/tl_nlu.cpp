/*
* 图灵 NLU 在线语意理解
*/
#include "ros/ros.h"           //ros系统头文件
#include "std_msgs/String.h" 
#include <sstream>
#include <jsoncpp/json/json.h> //json字符串
#include <curl/curl.h>         //curl http访问
#include <string>
#include <exception>

using namespace std;
//全局变量
int flag = 0;
string result;

/*解析图灵服务器返回的json字符串*/
int parseJsonResonse(string input_str)
{
	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(input_str, root);
	if( !parsingSuccessful )
	{
		std::cout << "Fail to parse the response data"  << std::endl;
		return -1;
	}
    const Json::Value code = root["code"];//文本类型标识吗
    const Json::Value text = root["text"];//返回的结果
	result = text.asString();             //返回文本内容
	flag = 1;
	std::cout << "response code: " << code << std::endl;
	std::cout << "response text: " << result << std::endl;
	return 0;	
}

/*将接收到的返回数据写如内存*/
int writer(char *data, size_t size, size_t number, string *writerData)
{
   if ( writerData == NULL)
   {
	 return -1;
   }
	unsigned long len = size * number;
	writerData->append(data, len);
	return len;	
}

/*HTTP 请求*/
int HttpPostRequest(string input_str)
{
	string buffer;
	
	std::string strJson = "{" ;
	strJson += "\"key\" : \"fcabefe6a6ca48ff8c7d4f5dfccf0627\",";
	strJson += "\"info\" : ";
	strJson += "\"";
	strJson += input_str;
	strJson += "\"";
	strJson += "}";
	
	std::cout << "post json string: " << strJson << std::endl;
	try
	{
		CURL *pCurl = NULL;
		CURLcode res;                      //返回状态
		curl_global_init(CURL_GLOBAL_ALL); //初始化 pCurl
		pCurl = curl_easy_init();          //头
		
		if( NULL != pCurl)
		{
			curl_easy_setopt(pCurl, CURLOPT_TIMEOUT,5); //延迟时间
			curl_easy_setopt(pCurl, CURLOPT_URL,"http://www.tuling123.com/openapi/api");
			// http头 // 设置http发送的内容类型为JSON
			curl_slist *plist = curl_slist_append(NULL, "Content-Type:application/json;charset=UTF-8");			
			curl_easy_setopt(pCurl, CURLOPT_HTTPHEADER, plist);
			
			// 设置要POST的JSON数据
			curl_easy_setopt(pCurl, CURLOPT_POSTFIELDS,strJson.c_str());
			curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, &writer);
			curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, &buffer);
			
			//执行http请求
			res = curl_easy_perform(pCurl);
			//检查错误
			if (res != CURLE_OK)
			{
				printf("curl_easy_perform failed:%s\n", curl_easy_strerror(res));
			}
			curl_easy_cleanup(pCurl);	//清除当前http请求		
		}
		curl_global_cleanup();          //全部清除   
	}
	catch (std::exception &ex)
	{
		printf("!!! curl exception: %s.\n", ex.what());
	}
	if(buffer.empty())
	{
		std::cout << "!!! ERROR The Tuling server response NULL" << std::endl;
	}
	else
	{
		parseJsonResonse(buffer);
	}
	return 0;
}
/*
*   当voice/tl_nlu 话题有消息时，调用HttpPostRequest向图灵服务器发送内容，返回结果。
*/
void nluCallback(const std_msgs::String::ConstPtr&  msg)
{

    std::cout << " Your question is :" << msg->data << std::endl;
    HttpPostRequest(msg->data);
	ROS_INFO("READY TO DO NLU");
}
   
int main(int argc, char** argv)
{
	 ros::init(argc, argv, "tl_nlu_node"); //初始化ros系统 ，在roscore节点管理器注册节点
     ros::NodeHandle nhd;                   //节点句柄
	 //创建一个订阅者sub     节点句柄         话题            缓存区    函数指针   &callbackfunc 得到
     ros::Subscriber sub = nhd.subscribe("voice/tl_nlu_topic", 20, &nluCallback);
	 //节点创建一个发布者
	 ros::Publisher pub = nhd.advertise<std_msgs::String>("voice/xf_tts_topic", 20);
	 ROS_INFO("READY TO DO NLU");
	 //ros::spin();
     ros::Rate  rate(10);              //频率
     while(ros::ok()){
		if(flag)                       //成功获取到返回数据
		{
			std_msgs::String msg;
			msg.data = result;
			pub.publish(msg); 
			flag = 0 ;
		}
       ros::spinOnce();               //给ROS控制权  可以调用一次回调函数
       rate.sleep();
     }	 
	return 0;
}

