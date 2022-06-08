import serial

ser = serial.Serial("COM8", baudrate=115200)

while True:
    buffer = ser.read_until(b"\n")
    buffer = buffer.replace(b"\n", b"")
    buffer = buffer.split(b" ")

    can_id = buffer[0]
    data_len = buffer[1]
    data = buffer[2:]
    
    print(can_id, data_len, data)
