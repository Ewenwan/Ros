#!/usr/bin/python
# -*- coding:utf-8 -*-
### 修改后的 文件
import roslib
roslib.load_manifest('pocketsphinx')
import rospy
import pygtk # Python轻松创建具有图形用户界面的程序  播放音乐等
pygtk.require('2.0')
import gtk   #  GNU Image Manipulation Program (GIMP)   Toolkit

import gobject # 亦称Glib对象系统，是一个程序库，它可以帮助我们使用C语言编写面向对象程序
import pygst   # 与 pygtk 相关
pygst.require('0.10')
gobject.threads_init()# 初始化
import gst

from std_msgs.msg import String
from std_srvs.srv import *
import os
import commands

class recognizer(object):
    """GStreamer是一个多媒体框架，它可以允许你轻易地创建、编辑与播放多媒体文件"""
    # 初始化系统配置
    def __init__(self):
        # 创建节点
        rospy.init_node("recognizer")
        # 全局参数
        self._device_name_param = "~mic_name"  # 麦克风
        self._lm_param = "~lm"                 # 语言模型 language model  
        self._dic_param = "~dict"              # 语言字典
        self._hmm_param = "~hmm"               # 识别网络  hiden markov model 隐马尔可夫模型 分中英文模型
        
        
        # 用 gstreamer launch config 配置 麦克风  一些启动信息
        if rospy.has_param(self._device_name_param):# 按照指定的麦克风
            self.device_name = rospy.get_param(self._device_name_param)# 麦克风名字
            self.device_index = self.pulse_index_from_name(self.device_name)# 麦克风编号 ID
            self.launch_config = "pulsesrc device=" + str(self.device_index)# 启动信息
            rospy.loginfo("Using: pulsesrc device=%s name=%s", self.device_index, self.device_name)
        elif rospy.has_param('~source'):
            # common sources: 'alsasrc'
            self.launch_config = rospy.get_param('~source')
        else:
            self.launch_config = 'gconfaudiosrc'

        rospy.loginfo("麦克风配置: %s", self.launch_config) # "Launch config: %s",self.launch_config

        self.launch_config += " ! audioconvert ! audioresample " \
                            + '! vader name=vad auto-threshold=true ' \
                            + '! pocketsphinx name=asr ! fakesink'

        # 配置ros系统设置
        self.started = False
        rospy.on_shutdown(self.shutdown)# 自主关闭
        self.pub = rospy.Publisher('~output', String, queue_size=5)# 发布 ~output 参数指定的 话题 类型 String  
        
        rospy.Service("~start", Empty, self.start)   # 开始服务
        rospy.Service("~stop", Empty, self.stop)     # 结束服务
        # 检查模型和字典配置
        if rospy.has_param(self._lm_param) and rospy.has_param(self._dic_param):
            self.start_recognizer()
        else:
            rospy.logwarn("启动语音识别器必须指定语言模型lm,以及语言字典dic.")
            # rospy.logwarn("lm and dic parameters need to be set to start recognizer.")
                    
    def start_recognizer(self):
        rospy.loginfo("开始语音识别... ")
        # rospy.loginfo("Starting recognizer... ")
        
        self.pipeline = gst.parse_launch(self.launch_config)# 解析 麦克风配置 
        self.asr = self.pipeline.get_by_name('asr')         # 自动语音识别 模型
        self.asr.connect('partial_result', self.asr_partial_result)# 后面的函数
        self.asr.connect('result', self.asr_result)
        #self.asr.set_property('configured', True) # 需要开启配置  hmm模型
        self.asr.set_property('dsratio', 1)

        # 配置语言模型
        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
        else:
            rospy.logerr('请配置一个语言模型 lm.')
            return

        if rospy.has_param(self._dic_param):
            dic = rospy.get_param(self._dic_param)
        else:
            rospy.logerr('请配置一个语言字典 dic.')
            return
        
        if rospy.has_param(self._hmm_param):
            hmm = rospy.get_param(self._hmm_param)
        else:
            rospy.logerr('请配置一个语言识别模型 hmm.')
            return


        self.asr.set_property('lm', lm)   # 设置asr的语言模型
        self.asr.set_property('dict', dic)# 设置asr的字典
        self.asr.set_property('hmm', hmm) # 设置asr的识别模型
        

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus_id = self.bus.connect('message::application', self.application_message)
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.started = True

    # 解析 麦克风名称 得到 麦克风ID
    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput("pacmd list-sources | grep -B 1 'name: <" + name + ">' | grep -o -P '(?<=index: )[0-9]*'")

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: " + name)
        
    # 停止识别器
    def stop_recognizer(self):
        if self.started:
            self.pipeline.set_state(gst.STATE_NULL)
            self.pipeline.remove(self.asr)
            self.bus.disconnect(self.bus_id)
            self.started = False
    # 程序关闭
    def shutdown(self):
        """ 删除所有的参数，以防影响下次启动"""
        for param in [self._device_name_param, self._lm_param, self._dic_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        """ 关闭 GTK 进程. """
        gtk.main_quit()
    # 开始
    def start(self, req):
        self.start_recognizer()
        rospy.loginfo("识别器启动")
        return EmptyResponse()
    # 停止
    def stop(self, req):
        self.stop_recognizer()
        rospy.loginfo("识别器停止")
        return EmptyResponse()
    
    def asr_partial_result(self, asr, text, uttid):
        """前线部分结果到主线程. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """ 前线结果到主线程 """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """ 从总线上接收应用数据. """
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        if msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])
    # 部分结果
    def partial_result(self, hyp, uttid):
        """ Delete any previous selection, insert text and select it. """
        rospy.logdebug("Partial: " + hyp)
        
    # 最终结果
    def final_result(self, hyp, uttid):
        """ Insert the final result. """
        msg = String()# 话题消息类型
        msg.data = str(hyp)# 识别语音对于成的文字
        rospy.loginfo(msg.data)
        self.pub.publish(msg)

if __name__ == "__main__":
    start = recognizer()
    gtk.main()

