import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk, Frame
import socket
import json
import os
import threading
import queue
import time
import pyautogui
import pygetwindow as gw
import win32gui
import win32con

# Global event that will be set when a preset command completes.
preset_done_event = threading.Event()

# ------------------------ GLOBAL SETTINGS ------------------------
teensy_ip = "10.0.7.80"  # Teensy IP address
command_port = 65432           # Port for commands/responses
status_port = 65433            # Port for continuous status updates
response_port = 65434          # Port for responses (errors, logs, completions)

persistent_command_socket = None
default_left_right_steps = 1000
default_up_down_steps = 2000

# Preset positions: preset 1 = 50, preset 2 = 100, ..., preset 9 = 450
preset_positions = {
    1: {"offset": 500,  "home_position": "No"},
    2: {"offset": 1000, "home_position": "No"},
    3: {"offset": 1500, "home_position": "No"},
    4: {"offset": 2000, "home_position": "No"},
    5: {"offset": 2500, "home_position": "No"},
    6: {"offset": 3000, "home_position": "No"},
    7: {"offset": 3500, "home_position": "No"},
    8: {"offset": 4000, "home_position": "No"},
    9: {"offset": 4500, "home_position": "No"}
}
# Tracks the latest offset reported by the robot via status messages.
current_offset = 0

# Queues for asynchronous status and response messages
status_queue = queue.Queue()
response_queue = queue.Queue()

# Profiles file and data (loaded at startup)
profiles_file = "profiles_data.json"
if os.path.exists(profiles_file):
    with open(profiles_file, 'r') as file:
        profiles_data = json.load(file)
else:
    profiles_data = {}

# ------------------------ PERSISTENT COMMAND CONNECTION ------------------------
def open_command_connection():
    global persistent_command_socket
    try:
        persistent_command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        persistent_command_socket.connect((teensy_ip, command_port))
        log_message("command connection established.")
    except socket.error as e:
        log_message(f"Error connecting to command port: {e}")
        persistent_command_socket = None

def send_command(command):
    global persistent_command_socket
    if persistent_command_socket is None:
        open_command_connection()
    try:
        persistent_command_socket.send((command + "\n").encode('utf-8'))
        return ""
    except socket.error as e:
        persistent_command_socket = None
        return f"Error: {e}"

def is_teensy_connected():
    return persistent_command_socket is not None

# ------------------------ RESPONSE THREAD ------------------------
persistent_response_socket = None

def open_response_connection():
    global persistent_response_socket
    try:
        persistent_response_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        persistent_response_socket.connect((teensy_ip, response_port))
        # Remove timeout so the socket waits indefinitely
        persistent_response_socket.settimeout(None)
        # Immediately send a handshake to trigger the Teensy to register the client
        handshake = "CLIENT_READY\n"
        persistent_response_socket.sendall(handshake.encode("utf-8"))
        log_message("response connection established")
    except socket.error as e:
        log_message(f"Error connecting to response port: {e}")
        persistent_response_socket = None

def listen_for_responses():
    global persistent_response_socket
    if persistent_response_socket is None:
        open_response_connection()
    while True:
        try:
            if persistent_response_socket is None:
                open_response_connection()
            data = persistent_response_socket.recv(1024)
            if not data:
                log_message("Persistent response connection closed.")
                try:
                    persistent_response_socket.close()
                except Exception:
                    pass
                persistent_response_socket = None
                time.sleep(1)
                continue
            response_message = data.decode('utf-8')
            
            # Check for either the homing or preset completion replies.
            if "Done HOMING" in response_message or "PRESET command done" in response_message:
                preset_done_event.set()
                #log_message("Received movement confirmation from robot.")
            
            response_queue.put(response_message)
        except Exception as e:
            log_message(f"Response error: {e}")
            try:
                if persistent_response_socket is not None:
                    persistent_response_socket.close()
            except Exception:
                pass
            persistent_response_socket = None
            time.sleep(1)

# ------------------------ STATUS THREAD ------------------------
def listen_for_status_updates():
    """Continuously polls the Teensy status server by sending GET_STATUS and processing the response."""
    while True:
        try:
            # Create a new socket and connect to the status port
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5)
            s.connect((teensy_ip, status_port))
            s.settimeout(None)  # Disable timeout for persistent read
            
            # Signal that we are connected (update your UI)
            root.after(0, update_connection_label, True)
            log_message("Connected to status server.")

            # Now continuously poll for status updates
            while True:
                # Send the GET_STATUS request
                s.sendall("GET_STATUS\n".encode("utf-8"))
                '''log_message("Sent GET_STATUS request.")'''
                
                # Wait for the response from the Teensy
                data = s.recv(1024)
                if not data:
                    raise Exception("Connection closed by status server.")
                status_message = data.decode("utf-8").strip()
                '''log_message(f"Received status: {status_message}")'''
                # Place the received status message into the status_queue for processing
                status_queue.put(status_message)
                
                # Adjust the polling interval as needed
                time.sleep(0.5)
        except Exception as e:
            # Update UI to reflect disconnected state
            root.after(0, update_connection_label, False)
            log_message(f"Status connection error: {e}")
            try:
                s.close()
            except Exception:
                pass
            time.sleep(2)  # Wait before retrying the connection

def process_status_messages():
    try:
        while True:
            message = status_queue.get_nowait()
            '''log_message(message)'''
            update_status_indicators(message)
    except queue.Empty:
        pass
    root.after(100, process_status_messages)

def process_response_messages():
    try:
        while True:
            message = response_queue.get_nowait()
            log_message(message)
    except queue.Empty:
        pass
    root.after(100, process_response_messages)

# ------------------------ LOGGING FUNCTIONS (Combined Log) ------------------------
def log_message(msg):
    combined_log.insert(tk.END, f"{msg}\n")
    combined_log.yview(tk.END)
# ------------------------ STATUS UPDATE FUNCTION ------------------------
        # status_data might look like:
        # {
        #   "X_axis":"False",
        #   "x_pos":"123",
        #   "offset":"123",
        #   "home_position":"Yes",
        #   "Z1_axis":"Down",
        #   "Z2_axis":"Down"
        # }
def update_status_indicators(message):
    try:
        status_data = json.loads(message)
        if "X_axis" in status_data:
            X_axis_status_shared.config(text=status_data["X_axis"])
        if "Z1_axis" in status_data:
            Z1_axis_status_shared.config(text=status_data["Z1_axis"])
        if "Z2_axis" in status_data:
            Z2_axis_status_shared.config(text=status_data["Z2_axis"])
        if "offset" in status_data:
            offset_status_shared.config(text=status_data["offset"])
        if "home_position" in status_data:
            home_position_status_shared.config(text=status_data["home_position"])
    except json.JSONDecodeError:
        pass

# ------------------------ CONNECTION LABEL UPDATE ------------------------
def update_connection_label(is_connected):
    conn_text = "Connected" if is_connected else "Disconnected"
    conn_bg = "green" if is_connected else "red"
    connection_status_shared.config(text=conn_text, bg=conn_bg)

# ------------------------ MACHINE CONTROL FUNCTIONS ------------------------
def emergency_stop():
    response = send_command("STOP")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Emergency Stop Activated")

def home_machine():
    response = send_command("HOME")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Home Command Sent")

def move_motor(direction):
    cmd = ""
    if direction == "UP":
        cmd = "MOVE_UP"
    elif direction == "DOWN":
        cmd = "MOVE_DOWN"
    elif direction == "LEFT":
        cmd = "MOVE_LEFT " + str(default_left_right_steps)
    elif direction == "RIGHT":
        cmd = "MOVE_RIGHT " + str(default_left_right_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_left_single():
    cmd = "MOVE_LEFT " + str(default_left_right_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_left_double():
    cmd = "MOVE_LEFT"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_right_single():
    cmd = "MOVE_RIGHT " + str(default_left_right_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_right_double():
    cmd = "MOVE_RIGHT"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

'''def MOVE_Z1_UP():
    response = send_command("MOVE_Z1_UP")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Sent Command: MOVE_Z1_UP")

def MOVE_Z1_DOWN():
    response = send_command("MOVE_Z1_DOWN")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Sent Command: MOVE_Z1_DOWN")

def MOVE_Z2_UP():
    response = send_command("MOVE_Z2_UP")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Sent Command: MOVE_Z2_UP")

def MOVE_Z2_DOWN():
    response = send_command("MOVE_Z2_DOWN")
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message("Sent Command: MOVE_Z2_DOWN")'''

def move_z1_up_single():
    # Single click sends the command with the additional parameter (e.g., _300)
    cmd = "MOVE_Z1_UP "+ str(default_up_down_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z1_up_double():
    # Double click sends the standard command without the extra parameter
    cmd = "MOVE_Z1_UP"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z1_down_single():
    cmd = "MOVE_Z1_DOWN "+ str(default_up_down_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z1_down_double():
    cmd = "MOVE_Z1_DOWN"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z2_up_single():
    cmd = "MOVE_Z2_UP "+ str(default_up_down_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z2_up_double():
    cmd = "MOVE_Z2_UP"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z2_down_single():
    cmd = "MOVE_Z2_DOWN "+ str(default_up_down_steps)
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_z2_down_double():
    cmd = "MOVE_Z2_DOWN"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Sent Command: {cmd}")

def move_to_preset(preset_number):
    if preset_number not in preset_positions:
        messagebox.showerror("Error", f"Preset position {preset_number} not defined")
        return
    target_offset = preset_positions[preset_number]["offset"]
    cmd = f"PRESET({target_offset})"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
        return
    log_message(f"Sent Command: {cmd}")


def execute_movement_sequence(commands):
    for cmd in commands:
        response = send_command(cmd)
        if response.startswith("Error"):
            messagebox.showerror("Error", response)
            return
        else:
            log_message(f"Sent Command: {cmd}")
        time.sleep(0.5)

def process_number_command(command):
    number = int(command)
    log_message(f"Number {number} sent from laptop.")

def is_numeric_command(command):
    return len(command) == 1 and command.isdigit() and command in "123456789"

def process_received_command(command):
    if is_numeric_command(command):
        process_number_command(command)
    else:
        processCommand(command)

def processCommand(command):
    log_message(f"Received command: {command}")
    if not canExecuteCommand(command):
        return
    if command in ("STOP", "EMERGENCY_STOP"):
        log_message("Emergency Stop command sent")
        emergency_stop()
    elif command == "HOME":
        log_message("Home Command Sent")
        home_machine()
    elif command == "MOVE_UP":
        log_message("MOVE_UP command sent")
        move_motor("UP")
    elif command == "MOVE_DOWN":
        log_message("MOVE_DOWN command sent")
        move_motor("DOWN")
    elif command.startswith("MOVE_LEFT"):
        parts = command.split()
        steps = default_left_right_steps
        if len(parts) > 1:
            try:
                steps = int(parts[1])
            except:
                steps = default_left_right_steps
        log_message(f"MOVE_LEFT {steps} command sent")
        send_command(f"MOVE_LEFT {steps}")
    elif command.startswith("MOVE_RIGHT"):
        parts = command.split()
        steps = default_left_right_steps
        if len(parts) > 1:
            try:
                steps = int(parts[1])
            except:
                steps = default_left_right_steps
        log_message(f"MOVE_RIGHT {steps} command sent")
        send_command(f"MOVE_RIGHT {steps}")
    elif command == "CLEAR_ICE":
        log_message("CLEAR_ICE command sent")
        send_command("CLEAR_ICE")

def sendStatusUpdate():
    send_command("GET_STATUS")

def canExecuteCommand(command):
    # Placeholder - all commands allowed.
    return True

# ------------------------ Event Handlers to Distinguish Single and Double Clicks ------------------------

# For Z1_axis Up button
def on_z1_up_click(event):
    widget = event.widget
    # Schedule the single-click function to run after 250 milliseconds.
    widget.single_click_timer = widget.after(250, move_z1_up_single)

def on_z1_up_double_click(event):
    widget = event.widget
    # Cancel the scheduled single-click action if it exists.
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_z1_up_double()

# For Z1_axis Down button
def on_z1_down_click(event):
    widget = event.widget
    widget.single_click_timer = widget.after(250, move_z1_down_single)

def on_z1_down_double_click(event):
    widget = event.widget
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_z1_down_double()

# For Z2_axis Up button
def on_z2_up_click(event):
    widget = event.widget
    widget.single_click_timer = widget.after(250, move_z2_up_single)

def on_z2_up_double_click(event):
    widget = event.widget
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_z2_up_double()

# For Z2_axis Down button
def on_z2_down_click(event):
    widget = event.widget
    widget.single_click_timer = widget.after(250, move_z2_down_single)

def on_z2_down_double_click(event):
    widget = event.widget
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_z2_down_double()

# For Left movement button
def on_left_click(event):
    widget = event.widget
    widget.single_click_timer = widget.after(250, move_left_single)

def on_left_double_click(event):
    widget = event.widget
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_left_double()

# For Right movement button
def on_right_click(event):
    widget = event.widget
    widget.single_click_timer = widget.after(250, move_right_single)

def on_right_double_click(event):
    widget = event.widget
    if hasattr(widget, 'single_click_timer'):
        widget.after_cancel(widget.single_click_timer)
        widget.single_click_timer = None
    move_right_double()

# ------------------------ PROFILE FUNCTIONS ------------------------
def save_profiles_to_file():
    with open(profiles_file, 'w') as file:
        json.dump(profiles_data, file)

def save_profile():
    profile_name = profile_name_entry.get().strip()
    if not profile_name:
        messagebox.showerror("Error", "Profile name cannot be empty")
        return
    profiles_data[profile_name] = {
        "offset": offset_entry_auto.get().strip(),
        "speed": speed_entry_auto.get().strip(),
        "measure_time": measure_time_entry_auto.get().strip(),
        "num_measures": num_measures_entry_auto.get().strip()
    }
    save_profiles_to_file()
    messagebox.showinfo("Profile Saved", f"Profile '{profile_name}' saved successfully.")
    profile_combo['values'] = list(profiles_data.keys())

def load_profile(event):
    selected_profile = profile_combo.get()
    if selected_profile in profiles_data:
        offset_entry_auto.delete(0, tk.END)
        offset_entry_auto.insert(0, profiles_data[selected_profile].get("offset", ""))
        speed_entry_auto.delete(0, tk.END)
        speed_entry_auto.insert(0, profiles_data[selected_profile].get("speed", ""))
        measure_time_entry_auto.delete(0, tk.END)
        measure_time_entry_auto.insert(0, profiles_data[selected_profile].get("measure_time", ""))
        num_measures_entry_auto.delete(0, tk.END)
        num_measures_entry_auto.insert(0, profiles_data[selected_profile].get("num_measures", ""))


def wait_for_preset_done(timeout=15):
    preset_done_event.clear()  # Reset the event before waiting.
    log_message("Waiting for robot's movement confirmation...")
    if not preset_done_event.wait(timeout):
        log_message("Timeout waiting for robot movement confirmation!")
    else:
        log_message("Robot movement confirmed.")

TARGET_WINDOW_SUBSTRING = "Sensor Toolkit"

def bring_measurement_window_to_front():
    """
    Searches for a window whose title contains TARGET_WINDOW_SUBSTRING,
    restores it if minimized, and brings it to the foreground.
    Returns a pygetwindow Window object if found; otherwise, returns None.
    """
    def enum_callback(hwnd, results):
        if win32gui.IsWindowVisible(hwnd):
            window_text = win32gui.GetWindowText(hwnd)
            if TARGET_WINDOW_SUBSTRING.lower() in window_text.lower():
                results.append(hwnd)
                
    hwnds = []
    win32gui.EnumWindows(enum_callback, hwnds)
    if hwnds:
        hwnd = hwnds[0]
        win32gui.ShowWindow(hwnd, win32con.SW_RESTORE)
        try:
            win32gui.SetForegroundWindow(hwnd)
            print(f"Measurement window (handle {hwnd}) brought to front.")
        except Exception as e:
            print("Error bringing window to front:", e)
        return gw.Window(hwnd)
    else:
        print(f"No window containing '{TARGET_WINDOW_SUBSTRING}' was found.")
        return None

def click_relative(window, rel_x, rel_y, pause=1):
    """
    Clicks at a position relative to the top-left corner of the given window.
    """
    abs_x = window.left + rel_x
    abs_y = window.top + rel_y
    pyautogui.click(abs_x, abs_y)
    time.sleep(pause)

def measure_capacitance():

    window = bring_measurement_window_to_front()
    if not window:
        print("Measurement window not found; aborting measure_capacitance().")
        return

    # Allow some time for the window to become active.
    time.sleep(1)

    # You may choose to set the measurement duration from your GUI entry.
    try:
        measure_duration = float(measure_time_entry_auto.get())
    except Exception:
        measure_duration = 2.0  # default measuring time
    print(f"Using measuring time: {measure_duration} seconds.")

    # ---- Acquisition 1 (1 MHz) ----
    click_relative(window, 1160, 80)   # Step 1: Drop down
    #print("Acq1: Drop down clicked.")
    click_relative(window, 1000, 230)  # Step 2: Select 1 MHz (corrected)
    print("Acq1: 1 MHz selected.")
    '''click_relative(window, 530, 200)   # (Optional) Step 3: Click "log"
    print("Acq1: 'Log' clicked.")
    click_relative(window, 555, 295)   # Step 4: Click name entry
    print("Acq1: Name entry clicked.")
    pyautogui.press('backspace')
    pyautogui.typewrite("1")
    print("Acq1: Typed '1'.")
    click_relative(window, 900, 400)   # Step 6: Click OK
    print("Acq1: OK clicked.")'''
    click_relative(window, 450, 70)    # Step 7: Enable feed
    #print("Acq1: Enable feed clicked.")
    print(f"Acq1: Waiting {measure_duration} seconds for measurement...")
    time.sleep(measure_duration)       # Step 8: Wait measuring time
    click_relative(window, 450, 70)    # Step 9: Disable feed
    #print("Acq1: Disable feed clicked.")
    click_relative(window, 360, 70)    # Step 10: Stop acquisition
    #print("Acq1: Stop acquisition clicked.")

    # ---- Acquisition 2 (2.5 MHz) ----
    click_relative(window, 1160, 80)   # Step 11: Drop down
    #print("Acq2: Drop down clicked.")
    click_relative(window, 1000, 245)  # Step 12: Select 2.5 MHz (x=1000, y=245)
    print("Acq2: 2.5 MHz selected.")
    '''click_relative(window, 530, 200)   # Step 13: Click "log"
    print("Acq2: 'Log' clicked.")
    click_relative(window, 555, 295)   # Step 14: Click name entry
    print("Acq2: Name entry clicked.")
    pyautogui.press('backspace')
    pyautogui.typewrite("2.5")
    print("Acq2: Typed '2.5'.")
    click_relative(window, 900, 400)   # Step 16: Click OK
    print("Acq2: OK clicked.")'''
    click_relative(window, 450, 70)    # Step 17: Enable feed
    #print("Acq2: Enable feed clicked.")
    print(f"Acq2: Waiting {measure_duration} seconds for measurement...")
    time.sleep(measure_duration)       # Step 18: Wait measuring time
    click_relative(window, 450, 70)    # Step 19: Disable feed
    #print("Acq2: Disable feed clicked.")
    click_relative(window, 360, 70)    # Step 20: Stop acquisition
    #print("Acq2: Stop acquisition clicked.")

    print("Automated capacitance measurement sequence complete.")

# --- INTEGRATION INTO THE EXISTING PYTHON APP ---

# Replace or modify your current start_measurement() and CMU control functions.
# For example, if you originally had a function like this:

def start_cmu_control():
    """
    CMU control function that iterates through measurement steps.
    For each measurement, it sends a movement command (HOME for the first,
    then PRESET commands for subsequent positions), waits for robot confirmation,
    and then triggers the capacitance measurement automation.
    """
    try:
        num_measures = int(num_measures_entry_auto.get())
        step_increment = int(offset_entry_auto.get())  # This is the movement step increment.
    except ValueError:
        log_message("Invalid numeric input for profile parameters.")
        return

    current_offset = 0  # Starting position (home)
    for measure_index in range(num_measures):
        if measure_index == 0:
            response = send_command("HOME")
            log_message("Sent HOME command for first measurement.")
        else:
            current_offset += step_increment
            cmd = f"PRESET({current_offset})"
            response = send_command(cmd)
            log_message(f"Sent command: {cmd} for measurement #{measure_index + 1}.")

        # Wait until the Teensy confirms movement is complete.
        wait_for_preset_done()
        # Now trigger the capacitance measurement automation.
        measure_capacitance()

    log_message("CMU control measurement sequence complete.")

def start_measurement():
    selected_profile = profile_combo.get()
    if not selected_profile or selected_profile not in profiles_data:
        messagebox.showerror("Error", "Please select a valid profile to start")
        return

    # Start the CMU control process in a separate thread.
    threading.Thread(target=start_cmu_control, daemon=True).start()

'''def start_measurement():
    selected_profile = profile_combo.get()
    if not selected_profile or selected_profile not in profiles_data:
        messagebox.showerror("Error", "Please select a valid profile to start")
        return
    offset = offset_entry_auto.get()
    speed = speed_entry_auto.get()
    measure_time = measure_time_entry_auto.get()
    num_measures = num_measures_entry_auto.get()
    cmd = f"START_MEASURE OFFSET={offset} SPEED={speed} MEASURE_TIME={measure_time} NUM_MEASURES={num_measures}"
    response = send_command(cmd)
    if response.startswith("Error"):
        messagebox.showerror("Error", response)
    else:
        log_message(f"Started measurement with profile '{selected_profile}'")'''

# ------------------------ GUI SETUP ------------------------
root = tk.Tk()
root.title("Machine Control Interface")
root.geometry("700x950")

notebook = ttk.Notebook(root)
notebook.pack(expand=True, fill='both')

# Manual Control Tab
manual_control_frame = Frame(notebook)
notebook.add(manual_control_frame, text="Manual Control")

send_command_frame = tk.Frame(manual_control_frame)
send_command_frame.grid(row=0, column=0, columnspan=4, padx=10, pady=10, sticky='nsew')
tk.Label(send_command_frame, text="Send Command:").grid(row=0, column=0, padx=10, pady=5, sticky='w')
send_command_combo = ttk.Combobox(send_command_frame, values=[str(i) for i in range(1, 10)], state="readonly")
send_command_combo.grid(row=0, column=1, padx=10, pady=5, sticky='ew')
tk.Button(send_command_frame, text="Send", command=lambda: process_received_command(send_command_combo.get())).grid(row=0, column=2, padx=10, pady=5, sticky='ew')
tk.Button(send_command_frame, text="Home", command=home_machine).grid(row=0, column=3, padx=10, pady=5, sticky='ew')

move_frame = tk.Frame(manual_control_frame)
move_frame.grid(row=1, column=0, columnspan=2, padx=10, pady=10, sticky='nsew')
tk.Label(move_frame, text="Move:").grid(row=0, column=1, pady=5)
tk.Button(move_frame, text="↑", command=lambda: move_motor("UP")).grid(row=1, column=1, padx=5, pady=5, sticky='nsew')
btn_left = tk.Button(move_frame, text="←")
btn_left.grid(row=2, column=0, padx=5, pady=5, sticky='nsew')
btn_left.bind("<Button-1>", on_left_click)
btn_left.bind("<Double-Button-1>", on_left_double_click)
tk.Button(move_frame, text="↓", command=lambda: move_motor("DOWN")).grid(row=2, column=1, padx=5, pady=5, sticky='nsew')
btn_right = tk.Button(move_frame, text="→")
btn_right.grid(row=2, column=2, padx=5, pady=5, sticky='nsew')
btn_right.bind("<Button-1>", on_right_click)
btn_right.bind("<Double-Button-1>", on_right_double_click)

# Z-Axis Controls
z_axis_frame = tk.Frame(manual_control_frame)
z_axis_frame.grid(row=1, column=2, columnspan=2, padx=10, pady=10, sticky='nsew')
tk.Label(z_axis_frame, text="Z-Axis Control:").grid(row=0, column=0, columnspan=2, pady=5)
#tk.Button(z_axis_frame, text="Z1_axis ↑", command=MOVE_Z1_UP).grid(row=1, column=0, padx=5, pady=5, sticky='nsew')
btn_z1_up = tk.Button(z_axis_frame, text="Z1_axis ↑")
btn_z1_up.grid(row=1, column=0, padx=5, pady=5, sticky='nsew')
btn_z1_up.bind("<Button-1>", on_z1_up_click)
btn_z1_up.bind("<Double-Button-1>", on_z1_up_double_click)

#tk.Button(z_axis_frame, text="Z1_axis ↓", command=MOVE_Z1_DOWN).grid(row=2, column=0, padx=5, pady=5, sticky='nsew')
btn_z1_down = tk.Button(z_axis_frame, text="Z1_axis ↓")
btn_z1_down.grid(row=2, column=0, padx=5, pady=5, sticky='nsew')
btn_z1_down.bind("<Button-1>", on_z1_down_click)
btn_z1_down.bind("<Double-Button-1>", on_z1_down_double_click)

#tk.Button(z_axis_frame, text="Z2_axis ↑", command=MOVE_Z2_UP).grid(row=1, column=1, padx=5, pady=5, sticky='nsew')
btn_z2_up = tk.Button(z_axis_frame, text="Z2_axis ↑")
btn_z2_up.grid(row=1, column=1, padx=5, pady=5, sticky='nsew')
btn_z2_up.bind("<Button-1>", on_z2_up_click)
btn_z2_up.bind("<Double-Button-1>", on_z2_up_double_click)

#tk.Button(z_axis_frame, text="Z2_axis ↓", command=MOVE_Z2_DOWN).grid(row=2, column=1, padx=5, pady=5, sticky='nsew')
btn_z2_down = tk.Button(z_axis_frame, text="Z2_axis ↓")
btn_z2_down.grid(row=2, column=1, padx=5, pady=5, sticky='nsew')
btn_z2_down.bind("<Button-1>", on_z2_down_click)
btn_z2_down.bind("<Double-Button-1>", on_z2_down_double_click)

# Preset Positions
preset_frame = tk.Frame(manual_control_frame)
preset_frame.grid(row=2, column=0, columnspan=4, padx=10, pady=10, sticky='nsew')
for col in range(3):
    preset_frame.columnconfigure(col, weight=1, uniform="preset_col")
for row in range(1, 4):
    preset_frame.rowconfigure(row, weight=1, uniform="preset_row")
tk.Label(preset_frame, text="Preset Positions:").grid(row=0, column=0, columnspan=3, pady=5)
for i in range(1, 10):
    r = (i - 1) // 3 + 1
    c = (i - 1) % 3
    tk.Button(preset_frame, text=str(i), command=lambda i=i: move_to_preset(i)).grid(row=r, column=c, padx=5, pady=5, sticky='nsew')

# Automatic Control Tab
automatic_control_frame = Frame(notebook)
notebook.add(automatic_control_frame, text="Automatic Control")

tk.Label(automatic_control_frame, text="Select Profile:").grid(row=0, column=0, padx=10, pady=5, sticky='w')
profile_combo = ttk.Combobox(automatic_control_frame, values=list(profiles_data.keys()), state="readonly")
profile_combo.grid(row=0, column=1, padx=10, pady=5, sticky='ew')
profile_combo.bind("<<ComboboxSelected>>", load_profile)
tk.Button(automatic_control_frame, text="Start", command=start_measurement).grid(row=0, column=2, padx=10, pady=5, sticky='ew')

parameters_frame = tk.LabelFrame(automatic_control_frame, text="Profile Parameters")
parameters_frame.grid(row=1, column=0, columnspan=3, padx=10, pady=10, sticky='nsew')
tk.Label(parameters_frame, text="Profile Name:").grid(row=0, column=0, padx=10, pady=5, sticky='w')
profile_name_entry = tk.Entry(parameters_frame)
profile_name_entry.grid(row=0, column=1, padx=10, pady=5, sticky='ew')
tk.Label(parameters_frame, text="Default Offset:").grid(row=1, column=0, padx=10, pady=5, sticky='w')
offset_entry_auto = tk.Entry(parameters_frame)
offset_entry_auto.grid(row=1, column=1, padx=10, pady=5, sticky='ew')
tk.Label(parameters_frame, text="Speed:").grid(row=2, column=0, padx=10, pady=5, sticky='w')
speed_entry_auto = tk.Entry(parameters_frame)
speed_entry_auto.grid(row=2, column=1, padx=10, pady=5, sticky='ew')
tk.Label(parameters_frame, text="Measuring Time:").grid(row=3, column=0, padx=10, pady=5, sticky='w')
measure_time_entry_auto = tk.Entry(parameters_frame)
measure_time_entry_auto.grid(row=3, column=1, padx=10, pady=5, sticky='ew')
tk.Label(parameters_frame, text="Number of Measures:").grid(row=4, column=0, padx=10, pady=5, sticky='w')
num_measures_entry_auto = tk.Entry(parameters_frame)
num_measures_entry_auto.grid(row=4, column=1, padx=10, pady=5, sticky='ew')
tk.Button(parameters_frame, text="Save Profile", command=save_profile)\
    .grid(row=5, column=0, columnspan=2, padx=10, pady=10, sticky='ew')

# ------------------------ COMBINED LOG WINDOW ------------------------
def clear_logger():
    combined_log.delete("1.0", tk.END)

log_frame = tk.Frame(root)
log_frame.pack(fill='both', padx=10, pady=5)

# Shared Status Frame (placed above the clear button)
status_frame_shared = tk.Frame(log_frame)
status_frame_shared.pack(side=tk.TOP, fill='x', pady=5)

# First row: Connection and X_axis
tk.Label(status_frame_shared, text="Connection:").grid(row=0, column=0, padx=5, pady=5, sticky='w')
connection_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Connecting", bg="yellow")
connection_status_shared.grid(row=0, column=1, padx=5, pady=5, sticky='w')

tk.Label(status_frame_shared, text="X_axis:").grid(row=0, column=2, padx=5, pady=5, sticky='w')
X_axis_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Idle", bg="lightgray")
X_axis_status_shared.grid(row=0, column=3, padx=5, pady=5, sticky='w')

# Second row: Home Position and Z1_axis
tk.Label(status_frame_shared, text="Home Pos:").grid(row=1, column=0, padx=5, pady=5, sticky='w')
home_position_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Unknown")
home_position_status_shared.grid(row=1, column=1, padx=5, pady=5, sticky='w')

tk.Label(status_frame_shared, text="Z1_axis:").grid(row=1, column=2, padx=5, pady=5, sticky='w')
Z1_axis_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Unknown", bg="lightgray")
Z1_axis_status_shared.grid(row=1, column=3, padx=5, pady=5, sticky='w')

# Third row: Offset and Z2_axis
tk.Label(status_frame_shared, text="Offset:").grid(row=2, column=0, padx=5, pady=5, sticky='w')
offset_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Unknown")
offset_status_shared.grid(row=2, column=1, padx=5, pady=5, sticky='w')

tk.Label(status_frame_shared, text="Z2_axis:").grid(row=2, column=2, padx=5, pady=5, sticky='w')
Z2_axis_status_shared = tk.Label(status_frame_shared, width=10, relief="sunken", text="Unknown", bg="lightgray")
Z2_axis_status_shared.grid(row=2, column=3, padx=5, pady=5, sticky='w')

# Emergency Stop button spanning all three rows on the right side
emergency_stop_button_shared = tk.Button(status_frame_shared, text="Emergency Stop", command=emergency_stop, bg="red", fg="white")
emergency_stop_button_shared.grid(row=0, column=4, rowspan=3, padx=10, pady=5, sticky='nsew')

# Clear Log button at the top, centered
clear_log_button = tk.Button(log_frame, text="Clear The Logger", command=clear_logger, width=90)
clear_log_button.pack(side=tk.TOP, pady=5, anchor='center')

# Logger widget below the button
combined_log = scrolledtext.ScrolledText(log_frame, height=15, wrap=tk.WORD)
combined_log.pack(fill='both', expand=True)

# ------------------------ START UP ------------------------
root.after(100, process_status_messages)
root.after(100, process_response_messages)
open_command_connection()
root.after(200, lambda: threading.Thread(target=listen_for_status_updates, daemon=True).start())
root.after(200, lambda: threading.Thread(target=listen_for_responses, daemon=True).start())

root.mainloop()
