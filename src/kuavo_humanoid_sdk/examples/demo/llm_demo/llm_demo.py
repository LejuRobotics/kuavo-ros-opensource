#!/usr/bin/env python
# coding=utf-8

import traceback  # DEBUG

clear_function = {}

from kuavo_humanoid_sdk import RobotControlBlockly
robot_control = RobotControlBlockly()

import time
from kuavo_humanoid_sdk import KuavoSDK
if not KuavoSDK.Init(log_level='DEBUG'):
    print('Init failed')
    exit(1)

from kuavo_humanoid_sdk import KuavoRobotLLM

kuavo_llm = KuavoRobotLLM()


def response_from_llm(llm_output: dict, action_flag: bool = False):
    if not llm_output.get("success", 1) == 0:
        return
    kuavo_llm.response_with_voice(llm_output)
    if action_flag:
        if llm_output.get("intent", "") == "function_call":
            try:
                eval(llm_output.get("slot"))
            except:
                print(f"【函数调用】执行失败:{llm_output.get('slot','')}")
        if llm_output.get("intent", "") == "action":
            robot_control.execute_action_file(llm_output["slot"])

        if llm_output.get("intent", "") == "action_custom":
            robot_control.execute_action_file(
                llm_output["slot"], "demo_project"
            )  # 拼接项目信息
    while not kuavo_llm.tts_end.is_synthesis_finished() or kuavo_llm.playing_status:
        time.sleep(0.1)


def test_function(arg: str):
    print(f"test_function:{arg}")


def main():
    try:
        kuavo_llm.add_file_to_prompt("test.txt", "demo_project")
        kuavo_llm.add_file_to_prompt("non-exist.txt", "demo_project")

        kuavo_llm.register_function("测试函数", "test_function('1')")

        kuavo_llm.register_case(
            "调用一下测试函数试试",
            "好的,我来调用一下",
            'test_function(\'1\')',  # 实际拼接通过转义完成
        )

        kuavo_llm.register_case(
            "跳个舞",
            "好的,这就跳",
            'robot_control.execute_action_file(\'dance\',"test_project")',  # 实际拼接通过转义完成
        )

        kuavo_llm.register_case(
            "比个心",
            "好的,这就做",
            'robot_control.execute_action_file(\'比心\')',  # 实际拼接通过转义完成
        )

        kuavo_llm.import_action_from_files("demo_project")
        # response_with_voice_and_action((kuavo_llm.chat_with_llm("你好")))
        # response_with_voice_and_action((kuavo_llm.chat_with_llm("你是谁")))
        # print((kuavo_llm.chat_with_llm("我刚刚说了什么")))
        # print((kuavo_llm.chat_with_llm("你的回答是什么")))

        response_from_llm((kuavo_llm.chat_with_llm("调用一下测试函数试试")), True)
        response_from_llm((kuavo_llm.chat_with_llm("和我打个招呼")), True)
        # count = 0
        # while count < 20:
        #     response_from_llm(kuavo_llm.chat_with_llm(kuavo_llm.trigger_asr()))
        #     count += 1

        # response_with_voice_and_action((kuavo_llm.chat_with_llm("跳个舞")))
        # response_with_voice_and_action((kuavo_llm.chat_with_llm("比个心")))
        # response_with_voice_and_action((kuavo_llm.chat_with_llm("介绍一下乐聚机器人公司")))

        # response_with_voice_and_action(kuavo_llm.chat_with_llm(kuavo_llm.trigger_asr()))

        # kuavo_llm._send_log("测试打印到log的方法", "INFO")

        # kuavo_llm._dump_prommpts()  # DEBUG

    except Exception as err:
        print(traceback.format_exc())
        print(err)
    finally:
        for name in clear_function:
            clear_function.get(name)()


if __name__ == "__main__":
    main()
