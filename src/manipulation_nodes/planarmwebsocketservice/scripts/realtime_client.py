from kuavo_humanoid_sdk import KuavoSDK

kuavo_sdk = KuavoSDK()
if not kuavo_sdk.Init(log_level='DEBUG'):
    print('Init failed')
    exit(1)


from kuavo_humanoid_sdk import RobotSpeech

robot_speech = RobotSpeech()

import os
import json
import rospy


# load api key
def load_api_keys():
    llm_api_storage_path = os.path.expanduser("~/.config/lejuconfig/llm_apis.json")
    try:
        with open(llm_api_storage_path, "r") as f:
            llm_apis = json.load(f)
            app_id = llm_apis["ark_X-Api-App-ID"]
            access_key = llm_apis["ark_X-Api-Access-Key"]
        return {
            "success": True,
            "message": "api key loaded successfully",
            "app_id": app_id,
            "access_key": access_key,
        }
    except:
        return {"success": False, "message": "api key loading failed"}


if __name__ == "__main__":
    # 语音对话功能
    api_keys = load_api_keys()
    if api_keys["success"]:
        app_id = api_keys["app_id"]
        access_key = api_keys["access_key"]
    else:
        raise Exception(api_keys["message"])
    
    if robot_speech.establish_doubao_speech_connection(app_id=app_id, access_key=access_key):
        # 使用 block=True 参数让函数卡住，保持程序运行
        robot_speech.start_speech(block=True)
