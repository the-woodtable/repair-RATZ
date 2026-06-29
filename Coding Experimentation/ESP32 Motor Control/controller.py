import serial
import tkinter as tk
from tkinter import ttk

# ⚠️ Replace COM4 with your actual ESP32 port
try:
    ser = serial.Serial('COM4', 115200)
    print("Connected to ESP32 on COM4")
except Exception as e:
    print("Could not open serial port:", e)
    ser = None

def send_command(cmd, label_text, color="black"):
    if ser:
        ser.write(cmd.encode())
        print("Sent:", cmd)
    else:
        print("No serial connection, command:", cmd)
    status_label.config(text=f"Status: {label_text}", foreground=color)

# Main window
root = tk.Tk()
root.title("ESP32 Servo Sweep Controller")
root.geometry("400x400")
root.configure(bg="#f0f0f0")

# Header
header = tk.Label(root, text="ESP32 Servo Controller", font=("Arial", 16, "bold"), bg="#f0f0f0")
header.pack(pady=10)

# Button style
style = ttk.Style()
style.configure("TButton", font=("Arial", 12), padding=10)

frame = tk.Frame(root, bg="#f0f0f0")
frame.pack(pady=20)

btn_same = ttk.Button(frame, text="⬆ Same Sweep (F)",
                      command=lambda: send_command('F', "Sweeping same direction", "green"))
btn_opposite = ttk.Button(frame, text="⇄ Opposite Sweep (B)",
                          command=lambda: send_command('B', "Sweeping opposite direction", "purple"))
btn_left = ttk.Button(frame, text="⬅ Left Only (L)",
                      command=lambda: send_command('L', "Sweeping left motor only", "blue"))
btn_right = ttk.Button(frame, text="➡ Right Only (R)",
                       command=lambda: send_command('R', "Sweeping right motor only", "orange"))
btn_stop = ttk.Button(frame, text="⏹ Stop (S)",
                      command=lambda: send_command('S', "Stopped", "red"))

btn_same.grid(row=0, column=0, pady=5, sticky="ew")
btn_opposite.grid(row=1, column=0, pady=5, sticky="ew")
btn_left.grid(row=2, column=0, pady=5, sticky="ew")
btn_right.grid(row=3, column=0, pady=5, sticky="ew")
btn_stop.grid(row=4, column=0, pady=5, sticky="ew")

# Status bar
status_label = tk.Label(root, text="Status: Idle", font=("Arial", 12), bg="#f0f0f0")
status_label.pack(side="bottom", fill="x", pady=10)

# Keyboard bindings
root.bind('<Up>', lambda e: send_command('F', "Sweeping same direction", "green"))
root.bind('<Down>', lambda e: send_command('B', "Sweeping opposite direction", "purple"))
root.bind('<Left>', lambda e: send_command('L', "Sweeping left motor only", "blue"))
root.bind('<Right>', lambda e: send_command('R', "Sweeping right motor only", "orange"))
root.bind('<space>', lambda e: send_command('S', "Stopped", "red"))

root.mainloop()
