import serial
import threading
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

PORTS = {
    "Master": "COM3",
    "Slave 1": "COM4",
    "Slave 2": "COM5"
}
BAUDRATE = 115200

class SerialMonitorApp:
    def __init__(self, master):
        self.master = master
        self.master.title("ESP8266 Firework System Monitor")
        self.text_widgets = {}

        for idx, (label, port) in enumerate(PORTS.items()):
            frame = tk.Frame(master)
            frame.grid(row=0, column=idx, padx=10, pady=10)

            title = tk.Label(frame, text=label, font=("Arial", 14, "bold"))
            title.pack()

            text_area = ScrolledText(frame, width=40, height=20, font=("Courier", 9))
            text_area.pack()
            text_area.config(state=tk.DISABLED)

            self.text_widgets[port] = text_area
            threading.Thread(target=self.read_serial, args=(port,), daemon=True).start()

    def read_serial(self, port):
        try:
            ser = serial.Serial(port, BAUDRATE, timeout=1)
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    self.append_text(port, line + '\n')
        except serial.SerialException as e:
            self.append_text(port, f"[ERROR] {e}\n")

    def append_text(self, port, text):
        widget = self.text_widgets[port]
        widget.config(state=tk.NORMAL)
        widget.insert(tk.END, text)
        widget.see(tk.END)
        widget.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialMonitorApp(root)
    root.mainloop()
