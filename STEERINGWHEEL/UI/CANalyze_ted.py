import struct
import logging
import time
import threading

import serial

logger = logging.getLogger("Serializer")
logger.setLevel(logging.INFO)

log_handler = logging.StreamHandler()
log_handler.setLevel(logging.INFO)
log_handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(levelname)s: %(message)s'))
logger.addHandler(log_handler)


class Serializer:
    END = b"\x0A"
    ESC = b"\x0B"
    ESC_END = b"\x1A"
    ESC_ESC = b"\x1B"
    
    def __init__(self, port, baudrate=115200, timeout=0, persistent=True):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.persistent = persistent

        self.connect()

    def connect(self):
        self._ser = None
        while not self._ser:
            try:
                self._ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=self.timeout)
            except serial.serialutil.SerialException as e:
                if not self.persistent:
                    raise e
                logger.info("Connecting to \"{0}\"...".format(self.port))
                time.sleep(1)
        
        logger.info("\"{0}\" Connected.".format(self.port))

    def _read(self):  
        try:
            c = self._ser.read(1)
        except serial.serialutil.SerialException:
            if not self.persistent:
                raise e
            logger.warning("Connection lost. Reconencting...")
            self.connect()
        return c
    
    def _write(self, c):
        try:
            self._ser.write(c)
        except serial.serialutil.SerialException:
            if not self.persistent:
                raise e
            logger.warning("Connection lost. Reconencting...")
            self.connect()
        return True

    def receive(self):
        c = b""
        buffer = b""

        while c != self.END:
            if c == self.ESC:
                c = self._read()
                if c == self.ESC_END:
                    buffer += self.END
                elif c == self.ESC_ESC:
                    buffer += self.ESC
                else:
                    buffer += c
            else:
                buffer += c
            
            c = self._read()
            if c == b"":
                return b""  # timeout
        
        return buffer
    
    def transmit(self, buffer):
        index = 0

        while index < len(buffer):
            c = struct.pack("B", buffer[index])
            if c == self.END:
                self._write(self.ESC)
                self._write(self.ESC_END)
            elif c == self.ESC:
                self._write(self.ESC)
                self._write(self.ESC_ESC)
            else:
                self._write(c)
            index += 1
            
        self._write(self.END)
        
        return index

    def flushRX(self):
        is_empty = True
        
        while self._ser.read():
            is_empty = False
            
        return is_empty

# ============================================================== #


import time
import random
import logging

from flask import Flask, render_template, jsonify

stopped = threading.Event()

database = {}

def serialHandler():
    ser = Serializer("/dev/cu.usbmodem103", 115200, timeout=0.1)

    while not stopped.is_set():
        buffer = ser.receive()

        if len(buffer) != 12:
            continue

        data = struct.unpack("<HBB", buffer[0:4])

        can_id = data[0]
        can_type = data[1]
        size = data[2]
        
        print(hex(can_id), size, buffer[4:])

        if can_id == 0x0C0:
            torque, _, _, _, enabled, _, _ = struct.unpack("<HBBBBBB", buffer[4:])
            database["APPS_torque_command"] = torque
            database["APPS_RMS_enabled_command"] = enabled
        
        elif can_id == 0x200:
            database["STEERINGWHEEL_ready_to_drive"] = buffer[4]
        
        elif can_id == 0x201:
            brake_pedal, acc_pedal = struct.unpack("<ff", buffer[4:])
            database["APPS_brake_pedal"] = brake_pedal
            database["APPS_acc_pedal"] = acc_pedal

        elif can_id == 0x300:
            database["LVPDB"] = buffer[4]
        



t = threading.Thread(target=serialHandler)
t.start()

app = Flask(__name__)

@app.route("/getdata", methods=["GET", "POST"])
def get_api():
    return jsonify(database)

@app.route("/")
def homepage():
    return render_template("index.html")

try:
    app.run(host="0.0.0.0", port=80)
except KeyboardInterrupt:
    stopped.set()

