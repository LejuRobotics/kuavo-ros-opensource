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

    def _get_api_keys_from_file(self) -> bool:
        """从文件中获取API密钥
        return:
            bool: 是否成功获取API密钥
        """
        apis_needed = [
            "ark_analysis_key",
            "xfyun_APPID",
            "xfyun_APISecret",
            "xfyun_APIKey",
        ]
        llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis")
        try:
            with open(llm_api_storage_path, "r") as f:
                lines = f.readlines()
                for line in lines:
                    line = line.strip()
                    if not line or line == "\n":
                        continue
                    split_index = line.find(":")
                    if split_index == -1:
                        continue
                    key, value = line[:split_index], line[split_index + 1 :]
                    self.apis[key.strip()] = value.strip()
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
            return {"success": False, "message": "密钥存储文件不存在"}
        except Exception as e:
            return {"success": False, "message": f"获取密钥时发生错误: {str(e)}"}


system_prompt = """
你是机器人助手夸鲁班,你需要根据用户的问题,回答用户的问题,并根据用户的问题,调用函数执行用户的指令.
你的回复需要符合以下格式:
{
    "text": "你要和用户说的话", 
    "intent": "chat"|"action"|"action_custom"|"function_call", # chat: 普通对话, action: 执行动作, action_custom: 执行自定义动作, function: 调用函数
    "slot": ""|"动作函数名称"|"自定义动作名称"|"函数调用字符串", # 如果intent为action或action_custom,则为动作函数名称,如果intent为function,则为函数调用的字符串
}

"""
