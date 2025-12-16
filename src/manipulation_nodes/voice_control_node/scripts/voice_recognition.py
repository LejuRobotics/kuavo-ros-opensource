import os
import numpy as np
import rospy
import json
from std_srvs.srv import Trigger, TriggerResponse
from funasr import AutoModel
# torch.set_num_threads(1)
import multiprocessing
from queue import Empty
from queue import Full

def _load_model(model_name, local_base_path, tag):
    """通用的模型加载辅助函数"""
    try:
        rospy.loginfo(f"正在加载 {tag} 模型: {model_name}...")
        return AutoModel(
            model=model_name,
            model_revision="v2.0.4",
            disable_update=True,  # 禁用自动更新
            disable_pbar=True     # 禁用进度条日志
        )
    except Exception as e:
        rospy.logwarn(f"通过名称加载 {tag} 模型失败: {e}. 正在尝试使用本地路径...")
        # 如果在线加载失败，则尝试使用本地路径
        local_full_path = os.path.join(local_base_path, model_name)
        if os.path.exists(local_full_path):
            return AutoModel(
                model=local_full_path,
                model_revision="v2.0.4",
                disable_update=True,
                disable_pbar=True
            )
        else:
            # 如果两种方式都失败，则抛出致命错误
            raise RuntimeError(f"加载 {tag} 模型失败，名称和本地路径 {local_full_path} 均无效。")

def vad_process_loop(audio_queue, speech_queue, config, shutdown_event):
    """
    VAD (语音活动检测) 进程循环 (生产者).
    - 从 audio_queue 获取原始音频数据。
    - 在本进程内加载和运行 VAD 模型。
    - 将检测到的完整语音片段放入 speech_queue，供 ASR 进程消费。
    """
    rospy.loginfo("[VAD进程] 启动中...")
    rospy.loginfo(f"[VAD进程] 进程号: {os.getpid()}")
    
    try:
        # 在子进程内部加载模型
        vad_model = _load_model(config["model_name"], config["local_model_path"], "VAD")
        
        # 从配置中解包参数
        vad_chunk_size_ms = config["vad_chunk_size_ms"]
        max_end_silence_time = config["max_end_silence_time"]
        speech_noise_thres = config["speech_noise_thres"]
        min_speech_frames = config["min_speech_frames"]
        MIC_SAMPLE_RATE = config["sample_rate"]
        vad_chunk_size_samples = int(MIC_SAMPLE_RATE * vad_chunk_size_ms / 1000)

        # 状态变量
        audio_buffer = np.array([], dtype=np.float32)
        vad_cache = {}
        is_speaking = False
        speech_frames = []
        # 用于vad音频帧的缓存回溯
        last_chunk = None

        while not shutdown_event.is_set():
            try:
                # 从音频队列获取数据，设置超时以避免永久阻塞
                if(not audio_queue.empty()):
                    data_batch = audio_queue.get_nowait()
                    audio_chunk_int16 = np.frombuffer(data_batch, dtype=np.int16)
                    audio_chunk_float32 = audio_chunk_int16.astype(np.float32)
                    audio_buffer = np.append(audio_buffer, audio_chunk_float32)
                else:
                    # 队列为空是正常情况，休息一段时间继续循环以检查关闭信号
                    rospy.sleep(0.1)
            except Exception as e:
                rospy.logerr(f"[VAD进程] 从音频队列获取数据时出错: {e}")
                continue

            # 当缓冲区中的数据足够一个VAD处理块时，进行处理
            while len(audio_buffer) >= vad_chunk_size_samples:
                feed_data = audio_buffer[:vad_chunk_size_samples]
                audio_buffer = audio_buffer[vad_chunk_size_samples:]

                # VAD模型推理
                vad_res = vad_model.generate(
                    input=feed_data,
                    chunk_size=vad_chunk_size_ms,
                    cache=vad_cache,
                    is_final=False,
                    max_end_silence_time=max_end_silence_time,
                    speech_noise_thres=speech_noise_thres
                )
                
                # print(vad_res)
                if vad_res and len(vad_res[0]["value"]) > 0:
                    segment = vad_res[0]["value"][-1]
                    if segment[1] == -1:  # 检测到语音开始
                        if not is_speaking:
                            rospy.logdebug("[VAD] 检测到语音开始...")
                            is_speaking = True
                            # 加上上一帧，防止vad检测丢失。TODO 通过参数优化
                            if last_chunk is not None:
                                speech_frames.append(last_chunk)
                        speech_frames.append(feed_data)
                    else:  # 检测到语音结束
                        if is_speaking:
                            rospy.logdebug("[VAD] 检测到语音结束。")
                            speech_frames.append(feed_data)
                            if len(speech_frames) >= min_speech_frames:
                                # 合并所有语音帧并放入语音队列
                                full_speech = np.concatenate(speech_frames)
                                
                                try:
                                    if(not speech_queue.full()):
                                        speech_queue.put_nowait(full_speech)
                                        rospy.logdebug("[VAD] 已将完整语音片段发送至ASR队列。")
                                    else:
                                        speech_queue.get_nowait()
                                        speech_queue.put_nowait(full_speech)
                                        rospy.logwarn("[VAD] ASR队列已满，丢弃旧数据以减少延迟。")
                                except Exception as e:
                                    # 罕见情况，刚判断队列为满，但是瞬间变空了，导致get_nowait失败
                                    rospy.logerr(f"[VAD] 出现未知异常： {e}")
                            else:
                                rospy.loginfo(f"[VAD] 语音片段过短({len(speech_frames) * vad_chunk_size_ms}ms)，已丢弃。")
                            # 重置状态
                            speech_frames.clear()
                            is_speaking = False
                elif is_speaking:
                    # 如果正在说话，但VAD未返回开始/结束信号，说明是话中的部分
                    speech_frames.append(feed_data)
                # 把当前块存下来，留给下一次循环用
                last_chunk = feed_data
    
    except Exception as e:
        rospy.logerr(f"[VAD进程] 发生严重错误: {e}")
    finally:
        rospy.loginfo("[VAD进程] 已关闭。")

def asr_process_loop(speech_queue, result_queue, config, shutdown_event):
    """
    ASR (自动语音识别) 进程循环 (消费者).
    - 从 speech_queue 获取完整语音片段。
    - 在本进程内加载和运行 ASR 模型。
    - 将识别和匹配后的动作结果放入 result_queue。
    """
    rospy.loginfo("[ASR进程] 启动中...")
    # 打印进程号
    rospy.loginfo(f"[ASR进程] 进程号: {os.getpid()}")

    try:
        try:
            import psutil
            p = psutil.Process(os.getpid())
            p.nice(10)  # 值越大，优先级越低
            rospy.loginfo(f"[ASR进程] CPU优先级(nice)已设置为: {p.nice()}")
        except (ImportError, psutil.Error) as e:
            rospy.logwarn(f"[ASR进程] 设置CPU优先级失败: {e}")

        # 在子进程内部加载模型
        asr_model = _load_model(config["model_name"], config["local_model_path"], "ASR")
        
        keyword_to_action = config["keyword_to_action"]
        hotword = config["hotword"]

        def _match_keyword_to_action(asr_text):
            """将识别到的文本与关键词进行匹配"""
            cleaned_text = asr_text.strip()
            # 优先完全匹配
            if cleaned_text in keyword_to_action:
                return keyword_to_action[cleaned_text]
            # 然后进行模糊匹配
            for keyword, action in keyword_to_action.items():
                if keyword in cleaned_text:
                    return action
            # rospy.logwarn(f"[ASR] 未匹配到关键词: {keyword_to_action.items()}")
            return None

        while not shutdown_event.is_set():
            try:
                if not speech_queue.empty():
                    audio_input = speech_queue.get_nowait()
                    
                    rospy.logdebug("[ASR] 开始进行ASR识别...")
                    asr_start_time = rospy.get_time()
                    asr_res = asr_model.generate(input=audio_input, hotword=hotword, ncpu=1)
                    asr_time = (rospy.get_time() - asr_start_time) * 1000
                    
                    text = asr_res[0]['text'].strip()
                    
                    if not text:
                        continue  # 空文本无需处理

                    rospy.loginfo(f"[ASR] 识别结果: '{text}' (耗时: {asr_time:.2f} ms)")
                    action = _match_keyword_to_action(text)
                    if action:
                        rospy.loginfo(f"匹配到关键词: {text}, 返回动作：{action}")
                        result_queue.put(action)
                    else:
                        # 换行以提高可读性
                        rospy.loginfo("")
                        rospy.loginfo(f"未匹配到任何关键词: {text}")
                        rospy.loginfo("")
                else:
                    # 队列为空是正常情况，休息一段时间继续循环以检查关闭信号
                    rospy.sleep(0.1)
            except Exception as e:
                rospy.logerr(f"[ASR进程] 处理语音时出错: {e}")

    except Exception as e:
        rospy.logerr(f"[ASR进程] 发生严重错误: {e}")
    finally:
        rospy.loginfo("[ASR进程] 已关闭。")

class VoiceCoordinator:
    """
    语音协调器 (进程管理器).
    负责创建和管理 VAD/ASR 子进程以及它们之间的跨进程队列。
    这是主进程中运行的核心类。
    """
    
    def __init__(self, subscribe_topic="/micphone_data", on_reload_callback=None):
        """
        初始化语音协调器
        
        Args:
            subscribe_topic: 订阅的麦克风话题
            on_reload_callback: 可选的回调函数，在重载配置后调用，用于更新外部配置（如 main.py 的 ACTION_CONFIG）
        """
        # 0. 保存回调函数
        self.on_reload_callback = on_reload_callback
        
        # 1. 创建进程关闭事件（每个进程单独使用独立的事件）
        self.vad_shutdown_event = multiprocessing.Event()  # VAD进程专用关闭事件（程序退出时用）
        self.asr_shutdown_event = multiprocessing.Event()     # ASR进程专用关闭事件

        # 1. 创建跨进程队列，用于在VAD和ASR进程间安全地传递数据
        self.audio_queue = multiprocessing.Queue(maxsize=100)   # 麦克风 -> VAD
        self.speech_queue = multiprocessing.Queue(maxsize=3)  # VAD -> ASR
        self.result_queue = multiprocessing.Queue()  # ASR -> 主进程

        # 2. 初始化麦克风订阅者 (将数据放入 audio_queue)
        from microphone import Microphone
        self.microphone = Microphone(self.audio_queue, subscribe_topic)
    
        # 3. 准备 VAD 和 ASR 进程所需的全部配置信息
        vad_model_name = "iic/speech_fsmn_vad_zh-cn-16k-common-pytorch"
        asr_model_name = "iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch"
        
        # 模型下载缓存路径
        home_dir = os.path.expanduser("~")
        local_model_path = os.path.join(home_dir, ".cache/modelscope/hub/")

        # 加载关键词配置
        keyword_map = self._load_keyword_from_file()
        keyword_to_action = self._build_keyword_to_action_map(keyword_map)
        hotword = self._init_hot_word_from_keyword(keyword_to_action)

        # 音频通用参数
        MIC_SAMPLE_RATE = 16000 # 麦克风数据的采样率
        vad_chunk_size_ms = 150 # VAD每次处理的音频时长(ms)
        MIN_SPEECH_MS = 600     # 如果VAD收集的语音短于此值，则认为是噪音并丢弃

        # VAD进程配置
        self.vad_config = {
            "model_name": vad_model_name,
            "local_model_path": local_model_path,
            "sample_rate": MIC_SAMPLE_RATE,
            "vad_chunk_size_ms": vad_chunk_size_ms,
            "max_end_silence_time": 800, # 最大静音时长(ms)，超过则认为一句话结束
            "speech_noise_thres": -1,   # 噪音检测阈值，[-1,1]，值越高越不容易将噪音误判为语音
            "min_speech_frames": int(MIN_SPEECH_MS / vad_chunk_size_ms), # 最小有效语音帧数
        }

        # ASR进程配置
        self.asr_config = {
            "model_name": asr_model_name,
            "local_model_path": local_model_path,
            "keyword_to_action": keyword_to_action,
            "hotword": hotword,
        }

        # 4. 创建并启动子进程
        self.vad_process = multiprocessing.Process(
            target=vad_process_loop,
            args=(self.audio_queue, self.speech_queue, self.vad_config, self.vad_shutdown_event),
            name="VAD-Process",
            daemon=True  # 设置为守护进程，主进程退出时会自动终止
        )
        
        self.asr_process = multiprocessing.Process(
            target=asr_process_loop,
            args=(self.speech_queue, self.result_queue, self.asr_config, self.asr_shutdown_event),
            name="ASR-Process",
            daemon=True
        )
        
        self.vad_process.start()
        self.asr_process.start()
        
        rospy.loginfo("VAD 和 ASR 子进程已启动。")

        # 5. 创建用于重载关键词的ROS服务
        self.reload_service = rospy.Service('/voice_control/reload_keywords', Trigger, self._handle_reload_keywords)
        rospy.loginfo("'/voice_control/reload_keywords' 服务已启动。")

        # 注册ROS关闭时的回调函数
        rospy.on_shutdown(self.shutdown)

    def _load_keyword_from_file(self):
        """加载关键词映射配置文件"""
        config_path = os.path.join(os.path.dirname(__file__), "key_words.json")
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            rospy.logerr(f"加载关键词文件 '{config_path}' 失败: {e}")
            return {}

    def _build_keyword_to_action_map(self, keyword_map):
        """构建关键词到动作的反向映射，方便快速查找"""
        keyword_to_action = {}
        for action, config in keyword_map.items():
            if "keywords" in config and isinstance(config["keywords"], list):
                for keyword in config["keywords"]:
                    keyword_to_action[keyword] = action
        rospy.loginfo(f"关键词映射构建完成: {keyword_to_action}")
        return keyword_to_action
    
    def _init_hot_word_from_keyword(self, keyword_to_action):
        """从关键词映射中提取热词列表，提升ASR识别准确率"""
        hot_words = " ".join(list(keyword_to_action.keys()))
        rospy.loginfo(f"热词列表初始化完成: {hot_words}")
        return hot_words

    def _handle_reload_keywords(self, req):
        """ROS服务的回调函数，用于重新加载关键词并重启ASR进程。"""
        rospy.loginfo("收到重载关键词请求...")
        try:
            # 1. 终止现有的ASR进程（使用专用的ASR关闭事件）
            if self.asr_process.is_alive():
                rospy.loginfo("正在终止当前的ASR进程...")
                self.asr_shutdown_event.set()  # 触发ASR进程退出
                self.asr_process.join(timeout=2)  # 等待进程终止
                if self.asr_process.is_alive():
                    rospy.logwarn("ASR进程未能优雅退出，将强制终止。")
                    self.asr_process.kill()
                    self.asr_process.join(timeout=2)  # 等待进程终止
                rospy.loginfo("ASR进程已终止")
            
            # 2. 重置ASR关闭事件（关键：为新进程准备干净的事件状态）
            self.asr_shutdown_event.clear()
            
            # 3. 清空speech_queue中的旧数据，避免新进程处理无效数据
            rospy.loginfo("清空ASR队列中的旧数据...")
            while not self.speech_queue.empty():
                try:
                    self.speech_queue.get_nowait()
                except Empty:
                    break
            
            # TODO 封装用于启动ASR进程和VAD进程的启动函数。并在main模块中通过服务的回调函数调用
            # 4. 重新加载关键词配置
            rospy.loginfo("正在重新加载关键词配置文件...")
            keyword_map = self._load_keyword_from_file()
            keyword_to_action = self._build_keyword_to_action_map(keyword_map)
            hotword = self._init_hot_word_from_keyword(keyword_to_action)

            # 5. 更新ASR配置
            self.asr_config["keyword_to_action"] = keyword_to_action
            self.asr_config["hotword"] = hotword
            
            # 6. 创建并启动新的ASR子进程
            rospy.loginfo("正在启动新的ASR进程...")
            self.asr_process = multiprocessing.Process(
                target=asr_process_loop,
                args=(self.speech_queue, self.result_queue, self.asr_config, self.asr_shutdown_event),
                name="ASR-Process",
                daemon=True
            )
            self.asr_process.start()
            
            rospy.loginfo("关键词重载并重启ASR进程成功。")
            
            # 7. 调用外部回调函数
            if self.on_reload_callback:
                try:
                    self.on_reload_callback()
                    rospy.loginfo("外部配置回调函数已执行")
                except Exception as e:
                    rospy.logwarn(f"执行外部配置回调函数时出错: {e}")
            
            return TriggerResponse(success=True, message="Keywords reloaded and ASR process restarted successfully.")

        except Exception as e:
            rospy.logerr(f"重载关键词失败: {e}")
            return TriggerResponse(success=False, message=f"Failed to reload keywords: {e}")

    def reset_micphone_data(self):
        """清空所有队列中的数据，以丢弃陈旧的音频和识别任务。"""
        rospy.loginfo("正在清空所有音频和识别任务队列...")
        for q in [self.audio_queue, self.speech_queue, self.result_queue]:
            while not q.empty():
                try:
                    q.get_nowait()
                except Empty:
                    break
        rospy.loginfo("所有队列已清空。")
    def shutdown(self):
        """在ROS关闭时被调用，优雅地终止子进程。"""
        rospy.loginfo("正在关闭语音识别协调器...")
        
        # 1. 发送关闭信号给所有子进程（只发送一次）
        if not self.vad_shutdown_event.is_set():
            self.vad_shutdown_event.set()
        if not self.asr_shutdown_event.is_set():
            self.asr_shutdown_event.set()
        
        # 2. 等待 VAD 进程结束（增加进程有效性检查）
        try:
            # 先检查进程是否为有效子进程（避免 "can only test a child process" 错误）
            if hasattr(self.vad_process, 'pid') and self.vad_process.pid is not None:
                if self.vad_process.is_alive():
                    rospy.loginfo("等待 VAD 进程结束...")
                    self.vad_process.join(timeout=2)
                    if self.vad_process.is_alive():
                        rospy.logwarn("VAD 进程未能优雅退出，将强制终止。")
                        self.vad_process.terminate()
                        self.vad_process.join(timeout=1)  # 等待强制终止完成
                else:
                    rospy.logdebug("VAD 进程已提前退出")
            else:
                rospy.logdebug("VAD 进程对象已失效，无需等待")
        except (AssertionError, Exception) as e:
            # 捕获所有进程相关异常，仅记录不中断流程
            rospy.logdebug(f"检查/等待VAD进程时的非致命错误: {e}")
        
        # 3. 等待 ASR 进程结束（增加进程有效性检查）
        try:
            # 先检查进程是否为有效子进程
            if hasattr(self.asr_process, 'pid') and self.asr_process.pid is not None:
                if self.asr_process.is_alive():
                    rospy.loginfo("等待 ASR 进程结束...")
                    self.asr_process.join(timeout=2)
                    if self.asr_process.is_alive():
                        rospy.logwarn("ASR 进程未能优雅退出，将强制终止。")
                        self.asr_process.terminate()
                        self.asr_process.join(timeout=1)
                else:
                    rospy.logdebug("ASR 进程已提前退出")
            else:
                rospy.logdebug("ASR 进程对象已失效，无需等待")
        except (AssertionError, Exception) as e:
            rospy.logdebug(f"检查/等待ASR进程时的非致命错误: {e}")
        
        rospy.loginfo("所有语音识别相关的子进程已关闭。")