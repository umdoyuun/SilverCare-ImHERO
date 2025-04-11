import os
import signal
import warnings
warnings.filterwarnings("ignore")

# ALSA 및 JACK 에러 메시지 숨기기
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
# PyAudio가 ALSA/JACK을 초기화하기 전에 환경 변수 설정
os.environ['JACK_NO_START_SERVER'] = '1'

# ALSA 에러 핸들러
import ctypes
ERROR_HANDLER_FUNC = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p, ctypes.c_int,
                                     ctypes.c_char_p)

def py_error_handler(filename, line, function, err, fmt):
    pass

c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)

try:
    asound = ctypes.CDLL('libasound.so.2')
    asound.snd_lib_error_set_handler(c_error_handler)
except:
    pass



import asyncio
import logging
import pyaudio
import queue
import numpy as np
from google.cloud import speech
from google.oauth2 import service_account
from dotenv import load_dotenv
import time
import threading
from enum import Enum
import aiohttp
from gtts import gTTS
import pygame
import subprocess

# robot_eyes 프로세스에 시그널 전송
def send_signal(sig_type):
    try:
        with open('/tmp/robot_eyes.pid', 'r') as f:
            pid = int(f.read().strip())
        if sig_type == 1:
            os.kill(pid, signal.SIGUSR1)
        elif sig_type == 2:
            os.kill(pid, signal.SIGUSR2)
        return True
    except FileNotFoundError:
        return False
    except ProcessLookupError:
        return False
    except Exception as e:
        return False

# 환경 변수 로드
load_dotenv()

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('stt_test.log')
    ]
)
logging.getLogger('pyaudio').setLevel(logging.ERROR)
logger = logging.getLogger(__name__)

# 상수 정의
RATE = 44100
CHUNK = int(RATE / 10)  # 100ms 청크
MAX_CHUNK_SIZE = 15360  # 960ms 이내의 오디오 데이터 크기
DEFAULT_KEYWORDS = ["영웅", "영웅아", "영웅이", "영웅.", "영웅아.", "영웅이.", 
                   "영웅!", "영웅아!", "영웅이!", "영웅?", "영웅아?", "영웅이?", "영웅~", "영웅아~", "영웅이~",
                   "영웅왕.", "영웅왕!", "영웅왕?", "영화", "영화.", "영화!", "영웅왕"]
EMERGENCY_KEYWORDS = ['응급', '응급!', '응급.', '응급?', '위급', '위급!', '위급.', '위급?',
                    '살려줘', '살려줘!', '살려줘.', '살려줘?', '신고', '신고.', '신고?', '신고!',
                    '도와줘', '도와줘!', '도와줘.', '도와줘?']
MESSAGE_KEYWORDS = ["메시지", "메세지", "메시지.", "메세지.", "메시지!", "메세지!", "메시지?", "메세지?", "메시지~", "메세지~",
                    "메시징.", "메세징.", "메시징!", "메세징!", "메시징?", "메세징?", "메시징~", "메세징~"]
TERMINATION_KEYWORDS = ["종료", "그만", "멈춰", "끝", "종료해줘"]

class STTMode(Enum):
    KEYWORD_DETECTION = "keyword_detection"
    SPEECH_RECOGNITION = "speech_recognition"
    EMERGENCY_MESSAGE = "emergency_message"
    MESSAGE_FLOW = "message_flow"
    MESSAGE = "message"

class MessageState(Enum):
    SELECTING_RECIPIENT = "selecting_recipient"
    ENTERING_MESSAGE = "entering_message"

# AudioStream 
class AudioStream:
    def __init__(self, rate=RATE, chunk=CHUNK):
        self._rate = rate
        self._chunk = chunk 
        self._buff = queue.Queue()  
        self.closed = True  
        self._audio_interface = None    
        self._audio_stream = None   
        self._resource_lock = threading.Lock()
        self.message_state = None
        self.temp_recipient = None
        logger.info("AudioStream 초기화 완료")  

    def __enter__(self):
        with self._resource_lock:   
            try:
                self._audio_interface = pyaudio.PyAudio()   
                
                # 사용 가능한 입력 장치 찾기
                device_index = None
                for i in range(self._audio_interface.get_device_count()):
                    device_info = self._audio_interface.get_device_info_by_index(i)
                    print(f"Device {i}: {device_info['name']}")
                    if device_info['maxInputChannels'] > 0:  # 입력 장치인 경우
                        device_index = i
                        break

                self._audio_stream = self._audio_interface.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=self._rate,    
                    input=True,
                    input_device_index=device_index,  # 찾은 장치 인덱스 사용
                    frames_per_buffer=self._chunk,
                    stream_callback=self._fill_buffer,
                )
                self.closed = False 
                logger.info(f"오디오 스트림 시작 (device_index: {device_index})")   
                return self
              
            except Exception as e:
                logger.error(f"오디오 스트림 초기화 실패: {e}")
                self.cleanup()
                raise

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        logger.info("오디오 스트림 종료")

    def cleanup(self):
        with self._resource_lock:
            self.closed = True
            if hasattr(self, '_audio_stream') and self._audio_stream:   
                try:
                    if self._audio_stream.is_active():
                        self._audio_stream.stop_stream()
                    self._audio_stream.close()
                    self._audio_stream = None
                except Exception as e:
                    logger.error(f"오디오 스트림 종료 오류: {e}")
                
            if hasattr(self, '_audio_interface') and self._audio_interface:
                try:
                    self._audio_interface.terminate()
                    self._audio_interface = None
                except Exception as e:
                    logger.error(f"PyAudio 종료 오류: {e}")
            
            # ALSA 디바이스 직접 리셋
            try:
                subprocess.run(['arecord', '-l'], capture_output=True)  # ALSA 디바이스 상태 확인
                subprocess.run(['pulseaudio', '-k'], capture_output=True)  # PulseAudio 재시작
                subprocess.run(['pulseaudio', '--start'], capture_output=True)
            except Exception as e:
                logger.error(f"ALSA 리셋 중 오류: {e}")
                
            self._buff.put(None)

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        try:
            if self.closed:
                return None, pyaudio.paComplete
            
            self._buff.put(in_data)
            return None, pyaudio.paContinue

        except Exception as e:
            logger.error(f"버퍼 채우기 오류: {e}")
            return None, pyaudio.paAbort

    def generator(self):
        accumulated_chunk = b""
        
        while not self.closed:
            chunk = self._buff.get()
            if chunk is None:
                return
            
            accumulated_chunk += chunk
            
            while len(accumulated_chunk) >= MAX_CHUNK_SIZE:
                yield accumulated_chunk[:MAX_CHUNK_SIZE]
                accumulated_chunk = accumulated_chunk[MAX_CHUNK_SIZE:]
            
        if accumulated_chunk:
            yield accumulated_chunk

class STTManager:
    def __init__(self):
        try:
            credentials_path = os.getenv("GOOGLE_CREDENTIALS_PATH")
            if not credentials_path or not os.path.exists(credentials_path):
                raise ValueError(f"Google Cloud 인증 파일을 찾을 수 없습니다: {credentials_path}")

            credentials = service_account.Credentials.from_service_account_file(credentials_path)
            self.client = speech.SpeechClient(credentials=credentials)
            self.session_id_for_openai = None
            self.user_id = None
            self.base_url = " https://dev-api.itdice.net"
            self.current_mode = STTMode.KEYWORD_DETECTION
            self.message_state = None
            self.temp_recipient = None
            
            try:
                with open('user_id.txt', 'r') as f:
                    self.user_id = f.read().strip()
                    logger.info(f"User ID loaded: {self.user_id}")
            except FileExistsError:
                logger.error('user_id.txt 파일을 찾을 수 없습니다.')
                raise

            try:
                with open('session_id.txt', 'r') as f:
                    self.session_id = f.read().strip()
                    logger.info(f"User ID loaded: {self.session_id}")
            except FileExistsError:
                logger.error('user_id.txt 파일을 찾을 수 없습니다.')
                raise


            # TTS 출력 디렉토리 설정
            self.tts_output_dir = "tts_output"
            os.makedirs(self.tts_output_dir, exist_ok=True)
            
            # pygame 초기화 
            pygame.mixer.init()
            
            # 시작음 알림
            self.start_sound = pygame.mixer.Sound('start_audio.mp3')
            
            logger.info("STT 매니저 초기화 완료")
        except Exception as e:
            logger.error(f"STT 매니저 초기화 실패: {e}")
            raise

    def get_config(self, mode: STTMode): 
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code="ko-KR",
            enable_automatic_punctuation=True
        )
        
        return speech.StreamingRecognitionConfig(
            config=config,
            interim_results=True,
            single_utterance=True
        )

    async def detect_keyword(self):
        try:
            logger.info("키워드 감지 모드 시작")
            config = self.get_config(STTMode.KEYWORD_DETECTION)
            
            with AudioStream() as stream:
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in stream.generator()
                )
                responses = self.client.streaming_recognize(config, requests)
                
                for response in responses:
                    if not response.results:
                        continue

                    result = response.results[0]
                    if not result.alternatives:
                        continue

                    transcript = result.alternatives[0].transcript.lower().strip()
                    
                    if result.is_final:
                        logger.info(f"키워드 감지 모드 - 텍스트: {transcript}")
                        
                        # 영웅 키워드 감지
                        if any(keyword in transcript for keyword in DEFAULT_KEYWORDS):
                            logger.info("영웅 키워드 감지됨")
                            self.current_mode = STTMode.SPEECH_RECOGNITION
                            self.start_sound.play()
                            return True
                            
                        # 응급 상황 키워드 감지
                        elif any(keyword in transcript for keyword in EMERGENCY_KEYWORDS):
                            logger.info("위급 상황 키워드 감지됨")
                            self.current_mode = STTMode.EMERGENCY_MESSAGE
                            tts_path = self.generate_speech("위급 상황 메시지를 말씀해주세요.")
                            self.play_audio(tts_path)
                            return True
                        
                        # 일반 메시지 키워드
                        elif any(keyword in transcript for keyword in MESSAGE_KEYWORDS):
                            logger.info("메시지 전송 키워드 감지됨")
                            self.current_mode = STTMode.MESSAGE_FLOW
                            return True

                        logger.info("키워드 없음")
                        return False

            return False

        except Exception as e:
            logger.error(f"키워드 감지 중 오류: {e}")
            return False

    async def process_speech(self):
        try:
            logger.info(f"{self.current_mode} 모드로 음성 인식 시작")
            config = self.get_config(self.current_mode)
            
            if self.current_mode == STTMode.MESSAGE_FLOW:
                tts_path = self.generate_speech("누구에게 메세지를 보내시겠습니까?")
                self.play_audio(tts_path)
                
                with AudioStream() as stream:
                    requests = (
                        speech.StreamingRecognizeRequest(audio_content=content)
                        for content in stream.generator()
                    )
                    responses = self.client.streaming_recognize(config, requests)
                    
                    for response in responses:
                        if not response.results:
                            continue

                        result = response.results[0]
                        transcript = result.alternatives[0].transcript.strip()
                        
                        if result.is_final:
                            logger.info(f"최종 텍스트 감지: {transcript}")
                            recipient_id = await self._find_most_similar_member(transcript)
                            logger.info(f"수신자 매칭 결과: {recipient_id}")
                            
                            if recipient_id:
                                self.current_mode = STTMode.SPEECH_RECOGNITION  
                                # await asyncio.sleep(2)   
                                await self.start_single_voice_message(recipient_id)  
                            else:
                                tts_path = self.generate_speech("일치하는 가족 구성원을 찾을 수 없습니다.")
                                self.play_audio(tts_path)
                            break

            elif self.current_mode == STTMode.SPEECH_RECOGNITION:
                # 기존 채팅 처리
                with AudioStream() as stream:
                    requests = (
                        speech.StreamingRecognizeRequest(audio_content=content)
                        for content in stream.generator()
                    )
                    responses = self.client.streaming_recognize(config, requests)
                    
                    for response in responses:
                        if not response.results:
                            continue

                        result = response.results[0]
                        transcript = result.alternatives[0].transcript.strip()
                        
                        if result.is_final:
                            logger.info(f"최종 텍스트 감지: {transcript}")
                            await self._process_chat(transcript)
                            self.current_mode = STTMode.KEYWORD_DETECTION
                            break
                    
            elif self.current_mode == STTMode.EMERGENCY_MESSAGE:
                # 위급 상황 메시지 처리
                with AudioStream() as stream:
                    requests = (
                        speech.StreamingRecognizeRequest(audio_content=content)
                        for content in stream.generator()
                    )
                    responses = self.client.streaming_recognize(config, requests)
                    
                    for response in responses:
                        if not response.results:
                            continue

                        result = response.results[0]
                        transcript = result.alternatives[0].transcript.strip()
                        
                        if result.is_final:
                            logger.info(f"최종 텍스트 감지: {transcript}")
                            await self._send_emergency_message(transcript)
                            self.current_mode = STTMode.KEYWORD_DETECTION
                            break

            return transcript

        except Exception as e:
            logger.error(f"음성 인식 중 오류: {str(e)}")
            self.current_mode = STTMode.KEYWORD_DETECTION
            return None

    async def _process_chat(self, transcript: str):
        try:
            timeout = aiohttp.ClientTimeout(total=30)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                url = f"{self.base_url}/chats"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                payload = {
                    "user_id": self.user_id,
                    "message": transcript,
                    "session_id": self.session_id_for_openai
                }
                async with session.post(url, json=payload, headers=headers) as response:
                    if response.status == 200:
                        result = await response.json()
                        self.session_id_for_openai = result.get('result', {}).get('session_id')
                        logger.info(f"API 응답: {result}")
                        
                        bot_message = result.get('result', {}).get('bot_message')
                        if bot_message:
                            logger.info(f"TTS 생성 시작: {bot_message}")
                            tts_path = self.generate_speech(bot_message)
                            logger.info(f"TTS 파일 생성됨: {tts_path}")
                            self.play_audio(tts_path)
                            logger.info("오디오 재생 완료")
                    else:
                        logger.error(f"API 오류: {response.status}")
        except Exception as e:
            logger.error(f"API 요청 중 오류: {e}")

    async def _capture_message_input(self):
        """메시지 입력을 캡처하는 별도의 메서드"""
        try:
            # await asyncio.sleep(1)
            
            config = self.get_config(STTMode.MESSAGE)
            
            with AudioStream() as stream:
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in stream.generator()
                )
                responses = self.client.streaming_recognize(config, requests)
                
                for response in responses:
                    if not response.results:
                        continue

                    result = response.results[0]
                    if not result.alternatives:
                        continue

                    transcript = result.alternatives[0].transcript.strip()
                    
                    if result.is_final:
                        logger.info(f"메시지 입력 텍스트: {transcript}")
                        return transcript

            return None
        except Exception as e:
            logger.error(f"메시지 입력 캡처 중 오류: {e}")
            # await asyncio.sleep(1)
            return None

    async def _send_emergency_message(self, content: str):
        """긴급 메시지를 가족 구성원 전체에게 전송"""
        try:
            async with aiohttp.ClientSession() as session:
                # messages/receivable API를 사용하여 수신 가능한 멤버 목록 조회
                url = f"{self.base_url}/messages/receivable/{self.user_id}"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                
                async with session.get(url, headers=headers) as response:
                    if response.status != 200:
                        raise Exception("가족 구성원 조회 실패")
                        
                    data = await response.json()
                    members = data.get('result', [])
                    
                    if not members:
                        raise Exception("메시지를 전송할 수 있는 가족 구성원이 없습니다.")

                    # 각 가족 구성원에게 메시지 전송
                    sent_count = 0
                    for member in members:
                        try:
                            await self._send_single_message(member['user_id'], f"[긴급] {content}")
                            sent_count += 1
                        except Exception as e:
                            logger.error(f"{member['name']}님에게 긴급 메시지 전송 실패: {e}")

                    # 전송 결과 음성 안내
                    if sent_count > 0:
                        tts_text = f"긴급 메시지가 {sent_count}명의 가족 구성원에게 전송되었습니다."
                    else:
                        tts_text = "긴급 메시지 전송에 실패했습니다."
                        
                    tts_path = self.generate_speech(tts_text)
                    self.play_audio(tts_path)

        except Exception as e:
            logger.error(f"긴급 메시지 전송 중 오류: {e}")
            error_msg = "긴급 메시지 전송 중 오류가 발생했습니다."
            tts_path = self.generate_speech(error_msg)
            self.play_audio(tts_path)

    async def start_single_voice_message(self, to_id: str):
        """특정 사용자에게 보낼 음성 메시지 녹음 시작"""
        try:
            # await asyncio.sleep(2) 
            tts_path = self.generate_speech("메시지를 말씀해주세요.")
            self.play_audio(tts_path)
            
            # await asyncio.sleep(1)

            config = self.get_config(STTMode.MESSAGE_FLOW) 
            
            with AudioStream() as stream:
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in stream.generator()
                )
                responses = self.client.streaming_recognize(config, requests)
                
                for response in responses:
                    if not response.results:
                        continue

                    result = response.results[0]
                    transcript = result.alternatives[0].transcript.strip()
                    
                    if result.is_final:
                        logger.info(f"최종 텍스트 감지: {transcript}")
                        await self._send_single_message(to_id, transcript)
                        self.current_mode = STTMode.KEYWORD_DETECTION  # 키워드 감지 모드로 복귀
                        return transcript

            return None
        except Exception as e:
            logger.error(f"음성 메시지 녹음 중 오류: {e}")
            self.current_mode = STTMode.KEYWORD_DETECTION
            return None

    async def _send_single_message(self, to_id: str, content: str):
        """특정 사용자에게 메시지 전송"""
        try:
            async with aiohttp.ClientSession() as session:
                url = f"{self.base_url}/messages/send"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                payload = {
                    "from_id": self.user_id,
                    "to_id": to_id,
                    "content": content
                }
                async with session.post(url, json=payload, headers=headers) as response:
                    if response.status == 201:
                        result = await response.json()
                        tts_path = self.generate_speech(result["message"])
                        self.play_audio(tts_path)
                    else:
                        error_msg = "메시지 전송에 실패했습니다."
                        tts_path = self.generate_speech(error_msg)
                        self.play_audio(tts_path)
        except Exception as e:
            logger.error(f"메시지 전송 중 오류: {e}")
            error_msg = "메시지 전송 중 오류가 발생했습니다."
            tts_path = self.generate_speech(error_msg)
            self.play_audio(tts_path)

    def generate_speech(self, text):
        try:
            print(f"\n입력 텍스트: {text}")
            start_time = time.time()

            output_path = os.path.join(self.tts_output_dir, "tts_output.mp3")
            
            if os.path.exists(output_path):
                os.remove(output_path)
            
            tts = gTTS(text=text, lang='ko')
            tts.save(output_path)

            end_time = time.time()
            print(f"처리 시간: {end_time - start_time:.2f}초")
            print(f"음성 파일 경로: {output_path}")
            print(f"파일 존재 여부: {os.path.exists(output_path)}")

            return output_path

        except Exception as e:
            print(f"음성 생성 중 에러 발생: {str(e)}")
            raise

    def play_audio(self, audio_path):
        try:
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.1)
            os.remove(audio_path)
        except Exception as e:
            logger.error(f"음성 재생 중 오류: {e}")

    async def check_new_messages(self):
        if hasattr(self, '_audio_stream') and self._audio_stream:
            logger.info("오디오 스트림 사용 중, 메시지 체크 스킵")
            return

        timeout = aiohttp.ClientTimeout(total=30)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            try:
                # 메시지 조회
                url = f"{self.base_url}/messages/new"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                async with session.get(url, headers=headers) as response:
                    if response.status == 200:
                        result = await response.json()
                        messages = result.get("result", [])
                        
                        for message in messages:
                            if not message["is_read"]:
                                # TTS 실행
                                tts_text = f"{message['from_id']}님이 보낸 메시지입니다. {message['content']}"
                                tts_path = self.generate_speech(tts_text)
                                self.play_audio(tts_path)
                                
                                # 읽음 처리
                                await self._mark_message_as_read(message['index'])
                    else:
                        logger.error(f"메시지 조회 실패: {response.status}")
                        
            except Exception as e:
                logger.error(f"메시지 체크 중 오류: {e}")

    async def _mark_message_as_read(self, message_index: int):
        """메시지 읽음 처리"""
        timeout = aiohttp.ClientTimeout(total=10)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            try:
                url = f"{self.base_url}/messages/read/{message_index}"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                async with session.post(url, headers=headers) as response:
                    if response.status == 200:
                        logger.info(f"메시지 {message_index} 읽음 처리 성공")
                        return True
                    else:
                        logger.error(f"메시지 읽음 처리 실패: {response.status}")
                        return False
            except Exception as e:
                logger.error(f"메시지 읽음 처리 중 오류: {e}")
                return False

    async def _find_most_similar_member(self, transcript: str) -> str:
        """입력된 텍스트와 가장 유사한 닉네임을 가진 가족 구성원 찾기"""
        try:
            async with aiohttp.ClientSession() as session:
                url = f"{self.base_url}/messages/receivable/{self.user_id}"
                headers = {
                    'Cookie': f"session_id={self.session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
                }
                async with session.get(url, headers=headers) as response:
                    if response.status != 200:
                        return None
                        
                    data = await response.json()
                    members = data.get('result', [])
                    
                    # 정확히 일치하는 경우
                    for member in members:
                        if transcript.lower() == member['name'].lower():
                            return member['user_id']
                    
                    # 부분 문자열 포함 관계 확인
                    for member in members:
                        if transcript.lower() in member['name'].lower() or \
                        member['name'].lower() in transcript.lower():
                            return member['user_id']
                    
                    # 초성 매칭
                    consonants = {
                        'ㄱ': ['가','깋'], 'ㄲ': ['까','낗'], 'ㄴ': ['나','닣'],
                        'ㄷ': ['다','딯'], 'ㄸ': ['따','띻'], 'ㄹ': ['라','맇'],
                        'ㅁ': ['마','밓'], 'ㅂ': ['바','빟'], 'ㅃ': ['빠','삫'],
                        'ㅅ': ['사','싷'], 'ㅆ': ['싸','앃'], 'ㅇ': ['아','잏'],
                        'ㅈ': ['자','짛'], 'ㅉ': ['짜','찧'], 'ㅊ': ['차','칳'],
                        'ㅋ': ['카','킿'], 'ㅌ': ['타','팋'], 'ㅍ': ['파','핗'],
                        'ㅎ': ['하','힣']
                    }
                    
                    def get_consonant(char):
                        for cons, (start, end) in consonants.items():
                            if start <= char <= end:
                                return cons
                        return char

                    transcript_cons = ''.join(get_consonant(c) for c in transcript if c.isalpha())
                    
                    for member in members:
                        member_cons = ''.join(get_consonant(c) for c in member['name'] if c.isalpha())
                        if transcript_cons == member_cons:
                            return member['user_id']
                    
                    return None
                        
        except Exception as e:
            logger.error(f"가족 구성원 조회 중 오류: {e}")
            return None

async def check_messages_periodically(stt_manager):
    while True:
        await stt_manager.check_new_messages()
        await asyncio.sleep(30)

async def main():
    try:
        stt_manager = STTManager()
        
        # 새 메시지 확인 태스크 시작
        message_check_task = asyncio.create_task(check_messages_periodically(stt_manager))
        
        while True:
            keyword_detected = await stt_manager.detect_keyword()
            
            if keyword_detected:
                send_signal(1)
                logger.info(f"{stt_manager.current_mode} 모드 시작")
                text = await stt_manager.process_speech()
                if text:
                    logger.info(f"인식된 텍스트: {text}")
            
    except KeyboardInterrupt:
        logger.info("프로그램 종료")
    except Exception as e:
        logger.error(f"실행 중 오류 발생: {e}")

if __name__ == "__main__":
    asyncio.run(main())
