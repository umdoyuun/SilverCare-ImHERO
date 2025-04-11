import time
import signal
import sys
import os
import threading
from PIL import Image, ImageDraw
import RPi.GPIO as GPIO
from RPLCD.i2c import CharLCD

def save_pid():
    pid = os.getpid()
    with open('/tmp/robot_eyes.pid', 'w') as f:
        f.write(str(pid))

class RobotEyes:
    def __init__(self):
        self.flag = 0   # 0 : sleep  /  1 : awake
        self.timer = 0
        self.lcd = CharLCD('PCF8574', 0x27, cols=16, rows=2)
        self.current_state = 'normal'
        self.states = ['normal', 'blink', 'happy', 'wave', 'sleep']
        self.current_pattern = 'normal'
        self.running_thread = None
        self.thread_stop_event = threading.Event()
        self.lcd_lock = threading.Lock()  # LCD 접근을 위한 mutex

        # 문자 인덱스 정의
        self.char_indices = {
            'normal_tl': 0, 'normal_tr': 1,
            'normal_bl': 2, 'normal_br': 3,
            'blink': 4, 'sleep_icon': 5,
            'special1': 6, 'special2': 7
        }

    def create_eye_characters(self, pattern_type='normal'):
        if pattern_type == 'normal':
            # 기본 눈 패턴
            patterns = {
                'normal_tl': [
                    0b00111,  # ..###
                    0b01111,  # .####
                    0b11111,  # #####
                    0b11100,  # ###..
                    0b11000,  # ##...
                    0b11000,  # ##...
                    0b11000,  # ##...
                    0b11000,  # ##...
                ],
                'normal_tr': [
                    0b11100,  # ###..
                    0b11110,  # ####.
                    0b11111,  # #####
                    0b00111,  # ..###
                    0b00011,  # ...##
                    0b10011,  # #..##
                    0b11011,  # ##.##
                    0b11011,  # ##.##
                ],
                'normal_bl': [
                    0b11000,  # ##...
                    0b11000,  # ##...
                    0b11100,  # ###..
                    0b11111,  # #####
                    0b01111,  # .####
                    0b00011,  # ...##
                    0b00000,  # .....
                    0b00000,  # .....
                ],
                'normal_br': [
                    0b00011,  # ...##
                    0b00011,  # ...##
                    0b00111,  # ..###
                    0b11111,  # #####
                    0b11110,  # ####.
                    0b11000,  # ##...
                    0b00000,  # .....
                    0b00000,  # .....
                ],
                'blink': [
                    0b11111,  # #####
                    0b11111,  # #####
                    0b00000,  # .....
                    0b11111,  # #####
                    0b11111,  # #####
                    0b00000,  # .....
                    0b00000,  # .....
                    0b00000,  # .....
                ],
            }
        else:
            # 웃는 눈과 물결 패턴
            if self.current_state == 'happy':
                # 웃는 눈 패턴
                patterns = {
                    'special1': [  # 웃는 눈 왼쪽
                        0b00000,  # .....
                        0b00000,  # .....
                        0b00000,  # .....
                        0b00000,  # .....
                        0b00001,  # ....#
                        0b00011,  # ...##
                        0b01110,  # .###.
                        0b11000,  # ##...
                    ],
                    'special2': [  # 웃는 눈 오른쪽
                        0b00000,  # .....
                        0b00000,  # .....
                        0b00000,  # .....
                        0b00000,  # .....
                        0b10000,  # #....
                        0b11000,  # ##...
                        0b01110,  # .###.
                        0b00011,  # ...##
                    ],
                }
            elif self.current_state == 'sleep':
                frame = getattr(self, 'sleep_frame', 0)
                if frame == 0:
                    patterns = {
                        'sleep_icon': [  #졸림 1
                            0b00111,  # ..###
                            0b00010,  # ...#.
                            0b00111,  # ..###
                            0b00000,  # .....
                            0b01110,  # .###.
                            0b00100,  # ..#..
                            0b01000,  # .#...
                            0b11100,  # ###..
                        ],
                    }
                elif frame == 1:
                    patterns = {
                        'sleep_icon': [  #졸림 1
                            0b00010,  # ...#.
                            0b00111,  # ..###
                            0b00000,  # .....
                            0b01110,  # .###.
                            0b00100,  # ..#..
                            0b01000,  # .#...
                            0b11100,  # ###..
                            0b00000,  # .....
                        ],
                    }
                elif frame == 2:
                    patterns = {
                        'sleep_icon': [  #졸림 1
                            0b00111,  # ..###
                            0b00000,  # .....
                            0b01110,  # .###.
                            0b00100,  # ..#..
                            0b01000,  # .#...
                            0b11100,  # ###..
                            0b00000,  # .....
                            0b00000,  # .....
                        ],
                    }
                elif frame == 3:
                    patterns = {
                        'sleep_icon': [  #졸림 1
                            0b00000,  # .....
                            0b01110,  # .###.
                            0b00100,  # ..#..
                            0b01000,  # .#...
                            0b11100,  # ###..
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00000,  # .....
                        ],
                    }
            else:
                # 물결 패턴 (3단계로 부드럽게)
                frame = getattr(self, 'wave_frame', 0)
                if frame == 0:
                    patterns = {
                        'special1': [  # 물결 1-1
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00001,  # ....#
                            0b00011,  # ...##
                            0b00110,  # ..##.
                            0b01100,  # .##..
                            0b11000,  # ##...
                        ],
                        'special2': [  # 물결 1-2
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00000,  # .....
                            0b10000,  # #....
                            0b11000,  # ##...
                            0b01100,  # .##..
                            0b00110,  # ..##.
                            0b00011,  # ...##
                        ],
                    }
                elif frame == 1:
                    patterns = {
                        'special1': [  # 물결 2-1
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00011,  # ...##
                            0b00110,  # ..##.
                            0b01100,  # .##..
                            0b11000,  # ##...
                            0b10000,  # #....
                            0b00000,  # .....
                        ],
                        'special2': [  # 물결 2-2
                            0b00000,  # .....
                            0b00000,  # .....
                            0b11000,  # ##...
                            0b01100,  # .##..
                            0b00110,  # ..##.
                            0b00011,  # ...##
                            0b00001,  # ....#
                            0b00000,  # .....
                        ],
                    }
                else:
                    patterns = {
                        'special1': [  # 물결 3-1
                            0b00011,  # ...##
                            0b00110,  # ..##.
                            0b01100,  # .##..
                            0b11000,  # ##...
                            0b10000,  # #....
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00000,  # .....
                        ],
                        'special2': [  # 물결 3-2
                            0b11000,  # ##...
                            0b01100,  # .##..
                            0b00110,  # ..##.
                            0b00011,  # ...##
                            0b00001,  # ....#
                            0b00000,  # .....
                            0b00000,  # .....
                            0b00000,  # .....
                        ],
                    }

        # LCD에 패턴 저장
        with self.lcd_lock:
            for name, pattern in patterns.items():
                self.lcd.create_char(self.char_indices[name], pattern)

    def draw_2x2_eye(self, col, chars):
        with self.lcd_lock:
            if len(chars) == 4:  # 일반 눈/웃는 눈
                self.lcd.cursor_pos = (0, col)
                self.lcd.write_string(chr(chars[0]))
                self.lcd.cursor_pos = (0, col + 1)
                self.lcd.write_string(chr(chars[1]))
                self.lcd.cursor_pos = (1, col)
                self.lcd.write_string(chr(chars[2]))
                self.lcd.cursor_pos = (1, col + 1)
                self.lcd.write_string(chr(chars[3]))
            elif len(chars) == 2:  # 눈 감은 상태
                self.lcd.cursor_pos = (1, col)
                self.lcd.write_string(chr(chars[0]))
                self.lcd.cursor_pos = (1, col + 1)
                self.lcd.write_string(chr(chars[1]))

    def draw_normal_eyes(self):
        # 왼쪽 눈
        self.draw_2x2_eye(2, [
            self.char_indices['normal_tl'],
            self.char_indices['normal_tr'],
            self.char_indices['normal_bl'],
            self.char_indices['normal_br']
        ])
        # 오른쪽 눈
        self.draw_2x2_eye(12, [
            self.char_indices['normal_tl'],
            self.char_indices['normal_tr'],
            self.char_indices['normal_bl'],
            self.char_indices['normal_br']
        ])

    def draw_blink_eyes(self):
        # 왼쪽 눈
        self.draw_2x2_eye(2, [
            self.char_indices['blink'],
            self.char_indices['blink']
        ])
        # 오른쪽 눈
        self.draw_2x2_eye(12, [
            self.char_indices['blink'],
            self.char_indices['blink']
        ])

    def draw_special_eyes(self):
        # 왼쪽 눈
        self.draw_2x2_eye(2, [
            self.char_indices['special1'],
            self.char_indices['special2']
        ])
        # 오른쪽 눈
        self.draw_2x2_eye(12, [
            self.char_indices['special1'],
            self.char_indices['special2']
        ])

    def draw_sleep_icon(self, col, char):
        with self.lcd_lock:
            self.lcd.cursor_pos = (0, col)
            self.lcd.write_string(chr(char))

    def sleep_animation(self, cycles=5):
        for _ in range(cycles):
            for frame in range(4):
                if self.thread_stop_event.is_set():
                    return
                self.sleep_frame = frame
                self.create_eye_characters('sleep')
                with self.lcd_lock:
                    self.lcd.clear()
                self.draw_blink_eyes()
                self.draw_sleep_icon(14, self.char_indices['sleep_icon'])
                time.sleep(1)

    def wave_animation(self, cycles=2):
        for _ in range(cycles):
            for frame in range(3):
                if self.thread_stop_event.is_set():
                    return
                self.wave_frame = frame
                self.create_eye_characters('special')
                with self.lcd_lock:
                    self.lcd.clear()
                self.draw_special_eyes()
                time.sleep(0.2)

    def blink_animation(self, frames=3):
        for _ in range(frames):
            if self.thread_stop_event.is_set():
                return
            with self.lcd_lock:
                self.lcd.clear()
            self.draw_blink_eyes()
            time.sleep(0.1)
        time.sleep(0.1)
        self.draw_normal_eyes()

    def transition_to(self, new_state):
        if new_state not in self.states:
            return

        self.current_state = new_state
        with self.lcd_lock:
            self.lcd.clear()

        if new_state == 'blink':
            self.blink_animation()
            new_state = 'normal'
        elif new_state == 'happy':
            self.create_eye_characters('special')
            self.draw_special_eyes()
        elif new_state == 'wave':
            self.wave_animation()
            new_state = 'normal'
        elif new_state == 'sleep':
            self.sleep_animation()
            new_state = 'normal'
        else:
            self.create_eye_characters('normal')
            self.draw_normal_eyes()

    def awake_demo(self):
        """깨어있는 상태의 데모 실행"""
        try:
            eyes.flag = 1
            while not self.thread_stop_event.is_set():
                eyes.timer += 1
                if eyes.timer > 10:
                    eyes.flag = 2
                    break
                
                if eyes.timer % 3 == 0:
                    # 웃는 눈 패턴으로 전환
                    self.transition_to('happy')
                    time.sleep(2)
                    if self.thread_stop_event.is_set(): break

                    # 물결 눈 패턴
                    self.transition_to('wave')
                    time.sleep(1)
                else:
                    # 기본 눈 모양
                    self.transition_to('normal')
                    time.sleep(2)
                    if self.thread_stop_event.is_set(): break

                    # 눈 깜빡임
                    self.transition_to('blink')
                    time.sleep(2)
                    if self.thread_stop_event.is_set(): break

                    # 연속 눈깜빡임
                    for _ in range(2):
                        if self.thread_stop_event.is_set(): break
                        self.transition_to('blink')
                        time.sleep(0.2)

                if self.thread_stop_event.is_set(): break
                time.sleep(1)

        except Exception as e:
            print(f"Error in awake_demo: {e}")

    def sleep_demo(self):
        """자는 상태의 데모 실행"""
        try:
            eyes.flag = 0
            while not self.thread_stop_event.is_set():
                self.transition_to('sleep')
                if self.thread_stop_event.is_set(): break
                time.sleep(1)
        except Exception as e:
            print(f"Error in sleep_demo: {e}")

    def start_sleep_mode(self):
        """수면 모드 시작"""
        self.stop_current_thread()
        self.thread_stop_event.clear()
        self.running_thread = threading.Thread(target=self.sleep_demo)
        self.running_thread.start()

    def start_awake_mode(self):
        """깨어있는 모드 시작"""
        self.stop_current_thread()
        self.thread_stop_event.clear()
        self.running_thread = threading.Thread(target=self.awake_demo)
        self.running_thread.start()

    def stop_current_thread(self):
        """현재 실행 중인 스레드 정지"""
        if self.running_thread and self.running_thread.is_alive():
            self.thread_stop_event.set()
            self.running_thread.join()

    def cleanup(self):
        """리소스 정리"""
        with self.lcd_lock:
            self.lcd.clear()
            self.lcd.close()

def signal_handler(signum, frame):
    """시그널 핸들러"""
    global eyes
    if signum == signal.SIGUSR1:  # 깨어나기
        print("Wake up signal received")
        eyes.timer = 0
        if(eyes.flag == 0):
            eyes.start_awake_mode()
    elif signum == signal.SIGUSR2:  # 잠자기
        print("Sleep signal received")
        if(eyes.flag == 1):
            eyes.start_sleep_mode()
    elif signum in [signal.SIGTERM, signal.SIGINT]:  # 종료
        print("Termination signal received")
        if eyes:
            eyes.stop_current_thread()
            eyes.cleanup()
        #GPIO.cleanup()
        sys.exit(0)

if __name__ == "__main__":
    save_pid()  # 시작할 때 PID 저장

    # 시그널 핸들러 설정
    signal.signal(signal.SIGUSR1, signal_handler)
    signal.signal(signal.SIGUSR2, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    eyes = None
    try:
        eyes = RobotEyes()
        eyes.start_sleep_mode()  # 처음에는 자는 모드로 시작

        # 메인 스레드는 계속 실행 상태 유지
        while True:
            time.sleep(1)
            if eyes.flag == 2:
                eyes.start_sleep_mode()
    except Exception as e:
        print(f"Main loop error: {e}")
    finally:
        if eyes:
            eyes.stop_current_thread()
            eyes.cleanup()
        #GPIO.cleanup()

