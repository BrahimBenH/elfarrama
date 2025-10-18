import serial
import time

ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # allow Arduino to reset

while True:
    if ser.in_waiting > 0:
        command = ser.readline().decode().strip()
        print("Arduino said:", command)

        if command == "GET_NUMBER":
            ser.write(b"42\n")
        elif command == "GET_COLOR":
            ser.write(b"Blue\n")
        elif command == "GET_SHAPE":
            ser.write(b"Circle\n")
        else:
            ser.write(b"Unknown command\n")
