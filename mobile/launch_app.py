#!/usr/bin/env python
import os
import sys
import subprocess
import time

def launch_roscore():
    print("Starting roscore...")
    try:
        roscore_process = subprocess.Popen(["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Roscore started with PID: {roscore_process.pid}")
        time.sleep(2)  # Wait for roscore to initialize
        print("Roscore initialization wait complete")
        return roscore_process
    except Exception as e:
        print(f"Error starting roscore: {e}")
        return None

def launch_kivy_app():
    print("Starting Kivy navigation app...")
    try:
        kivy_process = subprocess.Popen([sys.executable, "main.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print(f"Kivy app started with PID: {kivy_process.pid}")
        return kivy_process
    except Exception as e:
        print(f"Error starting Kivy app: {e}")
        return None

if __name__ == "__main__":
    print("Starting application launcher")
    print(f"Current directory: {os.getcwd()}")
    print(f"Python executable: {sys.executable}")
    
    print("Launching roscore")
    roscore = launch_roscore()
    if not roscore:
        print("Failed to start roscore, exiting")
        sys.exit(1)
    
    print("Launching Kivy app")
    kivy_app = launch_kivy_app()
    if not kivy_app:
        print("Failed to start Kivy app, terminating roscore")
        roscore.terminate()
        sys.exit(1)
    
    try:
        print("All processes started successfully")
        print("Press Ctrl+C to exit")
        while True:
            time.sleep(1)
            # Check if processes are still running
            if roscore.poll() is not None:
                print(f"Roscore exited with code {roscore.returncode}")
                break
            if kivy_app.poll() is not None:
                print(f"Kivy app exited with code {kivy_app.returncode}")
                break
    except KeyboardInterrupt:
        print("Keyboard interrupt received, shutting down...")
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        print("Terminating processes")
        if kivy_app and kivy_app.poll() is None:
            print(f"Terminating Kivy app (PID: {kivy_app.pid})")
            kivy_app.terminate()
            print("Kivy app terminated")
        
        if roscore and roscore.poll() is None:
            print(f"Terminating roscore (PID: {roscore.pid})")
            roscore.terminate()
            print("Roscore terminated")
        
        print("Shutdown complete")

