#!/usr/bin/env python3
# coding: utf-8
import openai
from openai import OpenAI
import os
from kuavo_humanoid_sdk.kuavo.core.llm_rtasr_lib.rtasr_llm_client import XFYunRTASRClient
from kuavo_humanoid_sdk.kuavo.core.llm_rtasr_lib.tts_llm_client import XFYunTTSClient
from kuavo_humanoid_sdk.msg.kuavo_msgs.msg import AudioReceiverData, AudioPlaybackStatus
import asyncio
import re
import json
import rospkg
import queue
from queue import Queue
import rospy
import threading
import time
import signal

from kuavo_humanoid_sdk.common.logger import SDKLogger

from std_msgs.msg import Int16MultiArray, Bool


class KuavoRobotLLM:
    """Kuavo 机器人大模型接口，用于控制大模型功能。

    提供大模型相关的功能，如文本生成、问题回答等。

    也需要调用对asr和tts的接口,将语音输入处理成文本,将文本回复处理成语音
    """

    def __init__(self):
        """初始化大模型系统。"""
        self.llm_end = None
        self.asr_end = None
        self.tts_end = None

        self.asr_audio_queue = Queue(maxsize=100)
        self.tts_text_queue = Queue(maxsize=100)  # 文本输入队列
        self.tts_audio_queue = Queue(maxsize=100)  # 音频输出队列
        self._send_asr_audio_running = False  # 控制发送音频线程的运行状态
        self._asr_completed = False  # 标志ASR是否完成
        self._tts_running = False  # 控制TTS合成线程的运行状态
        self._should_stop = False  # 全局停止标志

        self.playing_status = False
        self.playing_status_sub = rospy.Subscriber(
            "audio_status", AudioPlaybackStatus, self._play_status_callback, queue_size=10
        ) 

        package_name = "planarmwebsocketservice"
        self.package_path = rospkg.RosPack().get_path(package_name)

        self.prompt = ""

        self.default_action_list = []
        """已注册的动作列表"""
        self.custom_action_list = []
        """已注册的自定义动作列表"""
        self.custom_functions = {}
        """已注册的自定义函数"""
        self.example_responses = []
        """示例回复列表"""
        self.knowledge_texts = []
        """知识库文本列表"""

        self.chat_history = []
        self.max_chat_history_length = 20
        self.apis = {}
        init_result = self._get_api_keys_from_file()
        if not init_result["success"]:
            SDKLogger.error(init_result["message"])
            raise ValueError(init_result["message"])

        self._init_models()

        # 注册信号处理函数
        self._setup_signal_handlers()

    def _play_status_callback(self, msg: AudioPlaybackStatus):
        """播放状态回调函数"""
        self.playing_status = msg.playing

    def _setup_signal_handlers(self):
        """设置信号处理函数"""

        def handle_sigterm(signum, frame):
            """处理 SIGTERM 信号"""
            print("【SIGTERM】收到停止信号，正在停止大模型系统...")
            self._should_stop = True
            self._stop_all_services()
            exit(0)

        signal.signal(signal.SIGTERM, handle_sigterm)
        signal.signal(signal.SIGINT, handle_sigterm)  # 同时处理 Ctrl+C

    def _stop_all_services(self):
        """停止所有服务"""
        print("【停止服务】开始停止所有服务...")

        # 停止音频发送线程
        self._send_asr_audio_running = False

        # 停止TTS合成线程
        self._tts_running = False

        # 停止ASR
        if self.asr_end and self.asr_end.is_connected:
            self.asr_end.close()

        # 停止TTS
        if self.tts_end and self.tts_end.is_connected:
            self.tts_end.close()

        # 停止播音
        stop_publisher = rospy.Publisher("/stop_music", Bool, queue_size=10)
        stop_msg = Bool()
        stop_msg.data = True
        stop_publisher.publish(stop_msg)

        print("【停止服务】所有服务已停止")

    def stop(self):
        """外部调用的停止方法"""
        self._should_stop = True
        self._stop_all_services()

    def _get_action_folder_path(self, project_name):
        """获取项目路径"""
        return self.package_path + "/upload_files/" + project_name + "/action_files"

    def _get_knowledge_folder_path(self, project_name):
        """获取知识库路径"""
        return self.package_path + "/upload_files/" + project_name + "/knowledge_base"

    def _get_api_keys_from_file(self) -> dict:
        """从文件中获取API密钥
        return:
            dict: 是否成功获取API密钥
        """
        apis_needed = [
            "ark_analysis_key",
            "xfyun_APPID",
            "xfyun_APISecret",
            "xfyun_APIKey",
        ]
        llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis.json")
        try:
            with open(llm_api_storage_path, "r") as f:
                self.apis = json.load(f)
            missed_keys = []
            for item in apis_needed:
                if item not in self.apis:
                    missed_keys.append(item)
            if missed_keys:
                return {
                    "success": False,
                    "message": f'密钥缺失:{",".join(missed_keys)}',
                }
            return {"success": True, "message": "全部密钥获取成功"}

        except FileNotFoundError:
            return {"success": False, "message": "密钥文件不存在"}
        except Exception as e:
            return {"success": False, "message": f"获取密钥时发生错误: {str(e)}"}

    def _dump_prommpts(self):
        self._construct_final_prompt()
        content = self._concat_chat_history()

        for item in content:
            print(f'{item["role"]}:{item["content"]}')

    def _generate_prompt_for_actions(self):
        if not self.default_action_list and not self.custom_action_list:
            print("无动作")
            return ""
        action_prompt = "你可以执行以下动作:\n"
        if self.default_action_list:
            action_prompt += (
                "默认动作: \n -" + "\n - ".join(self.default_action_list) + "\n"
            )
        if self.custom_action_list:
            action_prompt += (
                "自定义动作: \n -" + "\n - ".join(self.custom_action_list) + "\n"
            )
        return action_prompt

    def _generate_prompt_for_functions(self) -> str:
        if not self.custom_functions:
            return ""
        function_prompt = "你可以调用以下函数:\n"
        for name, desc in self.custom_functions.items():
            function_prompt += f"{name}: {desc}\n"
        return function_prompt

    def _generate_prompt_for_examples(self) -> str:
        if not self.example_responses:
            return ""
        example_prompt = "以下是一些示例回复,如果匹配上这些示例,你回复的json应该严格按照示例中的回复来填写,如果用户的提问没有匹配任何示例,认为这是一个chat intent,slot留空,text为你对用户说的话:\n"
        for example in self.example_responses:
            example_prompt += f"- 示例{self.example_responses.index(example)+1}:\n\
  用户输入:'{example['user_input']}'\n\
  text:'{example['robot_response']}'\n\
  intent:'{example['intent']}'\n\
  slot:'{example['slot']}'\n"
        return example_prompt

    def _generate_prompt_for_knowledge(self) -> str:
        if not self.knowledge_texts:
            return ""
        knowledge_prompt = "以下是知识库内容:\n"
        for text in self.knowledge_texts:
            knowledge_prompt += f"内容{self.knowledge_texts.index(text)+1}:{'='*50}\n\
{text}\n\
内容{self.knowledge_texts.index(text)+1}结束\n"
        knowledge_prompt += "以上是知识库内容,请根据以上内容回答问题.\n"
        return knowledge_prompt

    def _construct_final_prompt(self) -> None:
        """构造最终prompt"""
        self.prompt = (
            system_prompt
            + self._generate_prompt_for_actions()
            + self._generate_prompt_for_functions()
            + self._generate_prompt_for_examples()
            + self._generate_prompt_for_knowledge()
            + reply_prompt
        )

    def _concat_chat_history(self) -> list:
        """拼接上下文"""
        result = [{"role": "system", "content": self.prompt}]
        result.extend(self.chat_history)
        return result

    def _add_message_to_history(self, message: dict):
        """将消息添加到上下文历史中

        args:
            message: 消息字典,包含role和content字段
        """
        self.chat_history.append(message)
        if len(self.chat_history) > self.max_chat_history_length:
            self.chat_history.pop(0)

    def trigger_asr(self) -> str:
        """从讯飞语音识别服务将语音输入转为文本"""
        if self._should_stop:
            print("【ASR】系统已停止，取消语音识别")
            return ""

        # 连接到讯飞ASR服务
        if not self.asr_end.is_connected:
            if not self.asr_end.connect():
                print("【ASR错误】无法连接到讯飞语音识别服务")
                SDKLogger.error("【ASR错误】无法连接到讯飞语音识别服务")
                return ""

        # 清空之前的识别结果
        self.asr_end.clear_recognized_text()
        # 重置完成标志
        self._asr_completed = False
        # 启动音频接收和发送线程
        self.audio_sub = rospy.Subscriber(
            "/micphone_data",
            AudioReceiverData,
            self._audio_receiver_callback,
            queue_size=10,
        )
        print("订阅成功")
        self._send_asr_audio_running = True
        send_thread = threading.Thread(target=self._send_asr_audio, daemon=True)
        send_thread.start()
        # 启动响应处理线程
        recv_thread = threading.Thread(target=self._process_asr_response, daemon=True)
        recv_thread.start()

        # 等待ASR完成（完全依赖于服务端的last标志位）
        self.start_wait_time = time.time()
        timeout = 5  # 最大等待时间
        while (
            not self._asr_completed
            and (time.time() - self.start_wait_time) < timeout
            and not self._should_stop
        ):
            time.sleep(0.1)

        # ASR完成后停止音频接收和发送
        self.audio_sub.unregister()
        self._send_asr_audio_running = False
        # self.asr_end.close()

        # 获取识别结果
        recognized_text = self.asr_end.get_recognized_text()
        print(f"【ASR识别完成】识别结果：{recognized_text}")
        return recognized_text

    def _audio_receiver_callback(self, msg: AudioReceiverData):
        """ROS麦克风数据回调函数"""
        try:
            # 播放状态：直接使用真实音频数据
            audio_data = bytes(msg.data)
            self.asr_audio_queue.put(audio_data)
        except Exception as e:
            print(f"处理麦克风数据时出错: {e}")

    def _send_asr_audio(self):
        """发送ASR音频数据"""
        frame_index = 0
        start_time = None
        try:
            while self._send_asr_audio_running and not self._should_stop:
                # 检查队列是否有数据
                if not self.asr_audio_queue.empty():
                    # print(f"{self.asr_audio_queue.qsize()} left in queue")
                    audio_frame = self.asr_audio_queue.get()

                    # 发送音频帧
                    self.asr_end.send_audio_frame(audio_frame)
                    frame_index += 1
                    time.sleep(0.001)
                else:
                    # 队列空时休眠一小段时间
                    time.sleep(0.001)
            print("【发送音频线程】已停止")
        except Exception as e:
            print(f"【发送音频异常】{str(e)}")

    def _process_asr_response(self):
        """处理ASR服务端返回的消息"""
        while self.asr_end.is_connected and not self._should_stop:
            res = self.asr_end._recv_msg()
            if res:
                self.start_wait_time = time.time()  # 重置等待时间
                # 检查是否为最终结果
                if (
                    res.get("data", {}).get("cn", {}).get("st", {}).get("type", None)
                    == "0"
                ):
                    print("【ASR结束】已收到最终识别结果，停止处理")
                    self._asr_completed = True
                    break
            time.sleep(0.01)  # 添加小延迟，避免CPU占用过高

    def _init_models(self):
        """初始化大模型"""
        # 初始化ASR模型（完全同步实现）
        self.asr_end = XFYunRTASRClient(
            app_id=self.apis["xfyun_APPID"],
            access_key_secret=self.apis["xfyun_APISecret"],
            access_key_id=self.apis["xfyun_APIKey"],
            base_url="wss://office-api-ast-dx.iflyaisol.com/ast/communicate/v1",
        )

        self.tts_end = XFYunTTSClient(
            app_id=self.apis["xfyun_APPID"],
            api_key=self.apis["xfyun_APIKey"],
            api_secret=self.apis["xfyun_APISecret"],
        )
        self.llm_end = OpenAI(
            base_url="https://ark.cn-beijing.volces.com/api/v3",
            api_key=self.apis["ark_analysis_key"],
        )

    def import_action_from_files(self, project_name: str):
        """将action_files中所有动作注册到llm类中,使用文件名作为动作名称

        args:
            project_name: 项目名称,用于指定自定义动作文件夹路径

        return:
            None
        """
        default_action_path = "/home/lab/.config/lejuconfig/action_files/"
        custom_action_path = self._get_action_folder_path(project_name)
        if os.path.exists(default_action_path):
            for file in os.listdir(default_action_path):
                if file.endswith(".tact"):
                    self.default_action_list.append(file[:-5])
        else:
            print("未找到预制动作文件夹")

        if os.path.exists(custom_action_path):
            for file in os.listdir(custom_action_path):
                if file.endswith(".tact"):
                    self.custom_action_list.append(file[:-5])
        else:
            print("未找到自定义动作文件夹")

    def register_function(self, function_comment: str, function: str):
        """将函数注册到llm类中,使用函数名作为函数名称

        args:
            function_comment: 函数注释,需要拼接到prompt中供大模型理解函数作用
            function: 自定义函数的名称,将函数积木块拖入后使用引号进行包裹

        return:
            None
        """
        self.custom_functions[function] = function_comment

    def register_case(self, user_input: str, robot_response: str, action: str):
        """将用户输入、机器人回复和动作函数注册到llm类中

        args:
            user_input: 用户输入的文本,作为触发条件
            robot_response: 机器人返回的文本,作为给llm的示例输出
            action: 自定义函数的名称,将函数积木块拖入后使用引号进行包裹,或传入动作函数

        return:
            None
        """
        action_pattern = r"robot_control\.execute_action_file\(\'([^\']+)\'(?:\s*,\s*\'([^\']+)\')?\)"
        # 匹配两种格式: robot_control.execute_action_file("xxx") 或 robot_control.execute_action_file("xxx","yyy")
        match = re.search(action_pattern, action)
        if match:
            intent = "action"
            if match.group(2):
                intent = "action_custom"
            slot = match.group(1)
        else:
            intent = "function_call"
            slot = action
        print(
            f"user_input:{user_input},robot_response:{robot_response},intent:{intent},slot:{slot}"
        )
        self.example_responses.append(
            {
                "user_input": user_input,
                "robot_response": robot_response,
                "intent": intent,
                "slot": slot,
            }
        )

    def add_file_to_prompt(self, file_name: str, project_name: str):
        """从知识库中选择指定文件并添加到prompt中

        args:
            file_name: 知识库中的文件名,需要包含文件扩展名(目前只接受.txt文件)
            project_name: 项目名称,用于指定知识库文件夹路径

        return:
            None
        """
        # 从知识库中选择指定文件并添加到prompt中
        knowledge_path = self._get_knowledge_folder_path(project_name)
        # 如果没有,通知客户
        if not os.path.exists(os.path.join(knowledge_path, file_name)):
            print(f"文件不存在:{knowledge_path}")
            SDKLogger.warning(f"知识库中不存在文件{file_name}")
            return
        with open(os.path.join(knowledge_path, file_name), "r") as file:
            content = file.read()
            self.knowledge_texts.append(content)

    def chat_with_llm(self, user_input: str = "") -> dict:
        """与大模型进行对话

        args:
            user_input: str|None: 用户输入的文本,如果mode为0,则为必填,如果mode为1,则不需要填写

        return:
            dict{
                "success": 0, # 0:成功,1:失败
                "text": "大模型返回的文本", # 大模型返回的文本
                "function_call": "function_name(args)", # 如果有函数调用,则返回函数调用的字符串,否则为空字符串
                "arguments": {
                    "arg1": "value1",
                    "arg2": "value2",
                } # 如果有函数调用,则返回函数调用的参数,否则为空字典
            }
        """
        if not user_input or user_input == "":
            print("【LLM】请输入用户输入")
            return {"success": 1, "text": "", "intent": "", "slot": ""}
        if self._should_stop:
            print("【LLM】系统已停止，取消对话")
            return {"success": 1, "text": "", "intent": "", "slot": ""}

        start_time = time.time()

        # 构建提示词
        self._construct_final_prompt()
        self._add_message_to_history({"role": "user", "content": user_input})
        # 拼接上下文
        context = self._concat_chat_history()
        # 与大模型的对话
        try:
            print(
                f"触发大模型回话,内容为:{user_input},提示词构建用时:{time.time()-start_time:.2f}秒"
            )
            response = self.llm_end.chat.completions.create(
                # 指定您创建的方舟推理接入点 ID，此处已帮您修改为您的推理接入点 ID
                model="doubao-seed-1-6-251015",
                messages=context,
                extra_body={
                    "thinking": {
                        "type": "disabled",  # 不使用深度思考能力
                        # "type": "enabled", # 使用深度思考能力
                        # "type": "auto", # 模型自行判断是否使用深度思考能力
                    }
                },
            )
            output = response.choices[0].message.content
            self._add_message_to_history({"role": "assistant", "content": output})
            output = json.loads(output)
            output["success"] = 0
            end_time = time.time()
            print(f"模型用时:{end_time-start_time:.2f}秒")
            return output
        except Exception as e:
            print(f"【LLM错误】{str(e)}")
            return {"success": 1, "text": "", "intent": "", "slot": ""}

    def response_with_voice(self, llm_output: dict):
        """将文本转换为语音,并播放出来

        args:
            llm_output: dict: 大模型返回的文本与函数调用信息

        return:
            None
        """
        print(f"response with voice triggered.")
        if self._should_stop:
            print("【TTS】系统已停止，取消语音合成")
            return

        if not llm_output.get("success", 1) == 0:
            return
        text = llm_output["text"].strip()
        if not text or not isinstance(text, str):
            return

        print(f"【TTS】开始合成文本：{text}")

        # 启动TTS合成（收发分离方式）
        self._tts_running = True
        # 发送文本
        send_success = self._send_tts_text(text)
        if send_success:
            print(f"【TTS启动】文本发送成功，开始接收音频")
            # 启动音频接收线程并将其保存为实例属性
            self.tts_recv_thread = threading.Thread(
                target=self._receive_tts_audio, daemon=True
            )
            self.tts_recv_thread.start()
            # 启动音频发布线程并将其保存为实例属性
            self.audio_pub_thread = threading.Thread(
                target=self._put_audio_into_ros, daemon=True
            )
            self.audio_pub_thread.start()
        else:
            print("【TTS启动失败】文本发送失败")
            self._tts_running = False
            # 放入None来通知音频发布线程结束
            try:
                self.tts_audio_queue.put_nowait(None)
            except:
                pass

    def _send_tts_text(self, text: str):
        """发送TTS文本数据的方法

        args:
            text: 要合成的文本

        return:
            bool: 发送是否成功
        """
        try:
            # 建立TTS连接
            if not self.tts_end.connect():
                print("【TTS错误】无法连接到讯飞语音合成服务")
                return False

            # 发送文本数据
            self.tts_end.set_synthesis_flag(False)
            if not self.tts_end.send_text(text):
                print("【TTS错误】无法发送文本数据")
                self.tts_end.close()
                return False

            print("【TTS发送】文本发送成功")
            return True
        except Exception as e:
            print(f"【TTS发送异常】{str(e)}")
            self.tts_end.close()
            return False

    def _receive_tts_audio(self):
        """在线程中运行的TTS音频接收方法

        return:
            None
        """
        try:
            # 接收音频流
            while (
                self._tts_running
                and not self.tts_end.is_synthesis_finished()
                and not self._should_stop
            ):
                audio_frame = self.tts_end.recv_audio_frame()
                if audio_frame:
                    try:
                        self.tts_audio_queue.put_nowait(audio_frame)
                        print(
                            f"【TTS音频帧】放入队列，长度: {len(audio_frame)}字节，队列大小: {self.tts_audio_queue.qsize()}"
                        )
                    except:
                        print("【TTS队列】音频队列已满，丢弃当前帧")
                time.sleep(0.01)

            print(
                "[tts状态]",
                self._tts_running,
                self.tts_end.is_synthesis_finished(),
                self._should_stop,
            )

            print(f"【TTS完成】合成结束")
            self.tts_audio_queue.put_nowait(None)
        except Exception as e:
            print(f"【TTS接收异常】{str(e)}")
        finally:
            self.tts_end.close()
            self._tts_running = False

    def _run_tts_synthesis(self, text: str):
        """执行TTS合成的线程方法（保留原有接口，用于兼容现有代码）

        args:
            text: 要合成的文本

        return:
            None
        """
        # 发送文本
        if self._send_tts_text(text):
            # 接收音频
            self._receive_tts_audio()
        else:
            print("【TTS合成】文本发送失败，取消合成")
            self._tts_running = False
            # 放入None来通知音频发布线程结束
            try:
                self.tts_audio_queue.put_nowait(None)
            except:
                pass

    def _put_audio_into_ros(self):
        """将TTS音频队列中的音频数据放入ROS话题中

        args:
            None

        return:
            None
        """
        self.tts_audio_pub = rospy.Publisher(
            "/audio_data", Int16MultiArray, queue_size=10
        )
        while True:
            try:
                # 检查是否需要停止
                if self._should_stop:
                    print("【音频发布】收到停止信号，停止音频发布")
                    break

                audio_frame = self.tts_audio_queue.get_nowait()
                if audio_frame is None:
                    break

                # 将字节数据转换为整数数组（16位音频）
                import struct

                audio_data = struct.unpack(
                    "<" + "h" * (len(audio_frame) // 2), audio_frame
                )

                msg = Int16MultiArray()
                msg.data = list(audio_data)
                self.tts_audio_pub.publish(msg)
            except queue.Empty:
                time.sleep(0.01)
        print("【音频发布】线程已停止")


system_prompt = """
你是机器人助手鲁班,你需要根据用户的问题,回答用户的问题,并根据用户的问题,调用函数执行用户的指令.
"""

reply_prompt = """
你的回复必须符合以下格式:
{
    "text": "你要和用户说的话", 
    "intent": "chat"|"action"|"action_custom"|"function_call", # chat: 普通对话, action: 执行动作, action_custom: 执行自定义动作, function_call: 调用函数
    "slot": ""|"动作函数名称"|"自定义动作名称"|"函数调用字符串", # 如果intent为action或action_custom,则为动作函数名称(如'1.左手打招呼'),如果intent为function_call,则为函数调用的字符串
}
"""
