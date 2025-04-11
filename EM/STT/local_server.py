from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import subprocess
import os
import logging
import signal
import sys
import time
import httpx
import json
from socketServer import get_raspberry_instance

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://70.12.246.28:3000", "https://dev-main.itdice.net"],  
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization", "Accept", "Cookie"],
)

class LoginCredentials(BaseModel):
    email: str
    password: str

class SettingValue(BaseModel):
    is_camera_enabled: bool
    is_driving_enabled: bool

processes = {
    'stt': None,
    'rsvp': None,
    'socketServer': None
}


raspberry = get_raspberry_instance()

@app.post("/bluetooth/speaker/connect")
async def connect_bluetooth_speaker():
   try:
       result = subprocess.run(['bluetoothctl', 'connect', '5C:FB:7C:34:59:29'], capture_output=True, text=True)
       if result.returncode == 0:
           return {"status": "success", "message": "bluetooth speaker connected"}
       else:
           return {"status": "error", "message": result.stderr}
   except Exception as e:
       return {"status": "error", "message": str(e)}
   
@app.post("/bluetooth/speaker/toggle")
async def toggle_speaker():
    try:
        with open('user_data.json', 'r') as f:
            user_data = json.load(f)
        family_id = user_data['family_id']
        session_id = user_data['session_id']

        headers = {
            'Cookie': f"session_id={session_id}; Path=/; Domain=itdice.net; Secure; HttpOnly;"
        }
        async with httpx.AsyncClient() as client:
            settings_response = await client.get(
                f"https://dev-api.itdice.net/tools/settings/{family_id}",
                headers=headers
            )
            
            if settings_response.status_code != 200:
                return {"status": "error", "message": "Failed to retrieve settings"}
            
            settings_data = settings_response.json()
            is_microphone_enabled = settings_data['result']['is_microphone_enabled']

            new_microphone_state = not is_microphone_enabled

            toggle_response = await client.patch(
                f"https://dev-api.itdice.net/tools/settings/{family_id}",
                headers=headers,
                json={"is_microphone_enabled": new_microphone_state}
            )
            
            if toggle_response.status_code != 200:
                return {"status": "error", "message": "Failed to update microphone settings"}

            volume = 70 if new_microphone_state else 0
            volume_result = await set_bluetooth_speaker_volume(volume)
            
            return {
                "status": "success", 
                "message": "Speaker and microphone settings updated",
                "volume": volume,
                "is_microphone_enabled": new_microphone_state
            }
    
    except Exception as e:
        logger.error(f"Speaker toggle error: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/bluetooth/speaker/volume")
async def set_bluetooth_speaker_volume(volume: int):
    try:
        if not 0 <= volume <= 100:
            return {"status": "error", "message": "Volume must be between 0 and 100"}
        
        sinks_result = subprocess.run(
            ['pactl', 'list', 'short', 'sinks'], 
            capture_output=True, 
            text=True, 
            encoding='utf-8', 
            errors='ignore'
        )
        
        bluetooth_sink = None
        for line in sinks_result.stdout.split('\n'):
            if 'bluez_output.5C_FB_7C_34_59_29.1' in line:
                bluetooth_sink = line.split()[1]
                break
        
        if not bluetooth_sink:
            return {"status": "error", "message": "Bluetooth speaker sink not found"}
        
        volume_result = subprocess.run(
            ['pactl', 'set-sink-volume', bluetooth_sink, f'{volume}%'], 
            capture_output=True, 
            text=True,
            encoding='utf-8', 
            errors='ignore'
        )
        
        if volume_result.returncode == 0:
            return {
                "status": "success", 
                "message": f"Bluetooth speaker volume set to {volume}%",
                "sink": bluetooth_sink
            }
        else:
            return {
                "status": "error", 
                "message": volume_result.stderr or "Failed to set volume"
            }
            
    except Exception as e:
        logger.error(f"Volume setting error: {e}")
        return {"status": "error", "message": str(e)}

@app.post("/SettingValue")
async def SendToJetson(settings : SettingValue):
    print(f"Received settings: {settings}") 
    try:
        data = {
            "is_camera_enabled": settings.is_camera_enabled,
            "is_driving_enabled": settings.is_driving_enabled
        }
        try:
            with open('user_data.json', 'r', encoding='utf-8') as file:
                user_data = json.load(file)
            data = {
                **user_data,
                "is_camera_enabled": settings.is_camera_enabled,
                "is_driving_enabled": settings.is_driving_enabled
            }
        except:
            pass
        raspberry.send_message(data)
        return {"status": "success", "message": "succsess"}
    except Exception as e:
        return {"status": "error", "message": e}

def start_processes(user_id: str, session_id: str):
    try:
        stop_processes()
        
        logger.info("Starting STT process...")
        processes['stt'] = subprocess.Popen(['python', 'main.py'])
        
        logger.info("Starting RSVP process...")
        processes['rsvp'] = subprocess.Popen(['./rsvp_project'], 
                                           cwd=os.path.expanduser('/home/ssafy/project/rsvp/build'))
        
#        logger.info("Starting SocketServer process...")
#        processes['socketServer'] = subprocess.Popen(['python', 'socketServer.py'])
        
        logger.info("All processes started successfully")
        return True
    except Exception as e:
        logger.error(f"Error starting processes: {e}")
        stop_processes() 
        return False

def stop_processes():
    for name, process in processes.items():
        if process:
            try:
                process.terminate()
                process.wait(timeout=10)
                logger.info(f"Terminated {name} process")
            except subprocess.TimeoutExpired:
                process.terminate()
                time.sleep(2)
                if process.poll() is None:
                    process.kill()
                logger.info(f"Killed {name} process")
            except Exception as e:
                logger.error(f"Error stopping {name} process: {e}")
            processes[name] = None

@app.post("/api/login")
async def login(credentials: LoginCredentials):
    try:
        async with httpx.AsyncClient() as client:
            response = await client.post(
                "https://dev-api.itdice.net/auth/login", 
                json={
                    "email": credentials.email,
                    "password": credentials.password
                }
            )
            
            if response.status_code != 200:
                raise HTTPException(status_code=response.status_code, detail="Login failed")
            
            login_data = response.json()
            
            user_id = login_data['result']['user_data']['id']
            session_id = login_data['result']['session_id']
            
            family_check_response = await client.post(
                "https://dev-api.itdice.net/families/check-exist",
                json={"id": user_id}
            )
            
            if family_check_response.status_code != 200:
                raise HTTPException(status_code=family_check_response.status_code, detail="Family check failed")
            
            family_data = family_check_response.json()
            family_id = family_data['result']['family_id']
            
            user_data = {
                "user_id": user_id,
                "session_id": session_id,
                "family_id": family_id
            }
            
            with open('user_data.json', 'w') as f:
                json.dump(user_data, f, indent=4)
            
            success = start_processes(user_id, session_id)
            
            return {
                "success": success,
                "user_id": user_id,
                "session_id": session_id,
                "family_id": family_id
            }
    
    except Exception as e:
        logger.error(f"Login error: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@app.on_event("shutdown")
async def shutdown_event():
    stop_processes()
    if raspberry:
        raspberry.stop() 

def signal_handler(signum, frame):
    logger.info("Shutdown signal received")
    stop_processes()
    if raspberry:
        raspberry.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8001)
