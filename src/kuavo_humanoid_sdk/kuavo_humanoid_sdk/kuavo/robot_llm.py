#!/usr/bin/env python3
# coding: utf-8
import openai
from openai import OpenAI
import os
import asyncio
import re
import json
import rospkg

from kuavo_humanoid_sdk.kuavo.logger_client import get_logger

logger = get_logger()


class KuavoRobotLLM:
    """Kuavo 机器人大模型接口，用于控制大模型功能。

    提供大模型相关的功能，如文本生成、问题回答等。

    也需要调用对asr和tts的接口,将语音输入处理成文本,将文本回复处理成语音
    """

    # TODO: 接收到stop时候要stop

    def __init__(self):
        """初始化大模型系统。"""
        self.llm_end = None
        self.asr_end = None
        self.tts_end = None

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
            self._send_log(init_result["message"], level="ERROR")
            raise ValueError(init_result["message"])

        self._init_models()

    def _get_action_folder_path(self, project_name):
        """获取项目路径"""
        return self.package_path + "/upload_files/" + project_name + "/action_files"

    def _get_knowledge_folder_path(self, project_name):
        """获取知识库路径"""
        return self.package_path + "/upload_files/" + project_name + "/knowledge_base"

    def _send_log(self, message: str, level: str = "INFO"):
        """发送日志到8889端口的辅助方法"""
        logger.send_log(message, level)

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

        

    def _init_models(self):
        """初始化大模型"""
        # TODO: 初始化大模型
        self.asr_end = None # TODO

        self.tts_end = None  # TODO
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
                    self.default_action_list.append(file)
        else:
            print("未找到预制动作文件夹")

        if os.path.exists(custom_action_path):
            for file in os.listdir(custom_action_path):
                if file.endswith(".tact"):
                    self.custom_action_list.append(file)
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
        action_pattern = (
            r'robot_control\.execute_action_file\("([^"]+)"(?:\s*,\s*"([^"]+)")?\)'
        )
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
            self._send_log(f"知识库中不存在文件{file_name}", "WARNING")
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
        # 构建提示词
        self._construct_final_prompt()
        self._add_message_to_history({"role": "user", "content": user_input})
        # 拼接上下文
        context = self._concat_chat_history()
        # 与大模型的对话
        response = self.llm_end.chat.completions.create(
            # 指定您创建的方舟推理接入点 ID，此处已帮您修改为您的推理接入点 ID
            model="doubao-seed-1-6-251015",
            messages=context,
        )
        output = response.choices[0].message.content
        self._add_message_to_history({"role": "assistant", "content": output})
        try:
            output = json.loads(output)
            output["success"] = 0
            return output
        except:
            return {"success": 1, "text": "", "intent": "", "slot": ""}


system_prompt = """
你是机器人助手鲁班,你需要根据用户的问题,回答用户的问题,并根据用户的问题,调用函数执行用户的指令.
"""

reply_prompt = '''
你的回复必须符合以下格式:
{
    "text": "你要和用户说的话", 
    "intent": "chat"|"action"|"action_custom"|"function_call", # chat: 普通对话, action: 执行动作, action_custom: 执行自定义动作, function_call: 调用函数
    "slot": ""|"动作函数名称"|"自定义动作名称"|"函数调用字符串", # 如果intent为action或action_custom,则为动作函数名称,如果intent为function_call,则为函数调用的字符串
}
'''
