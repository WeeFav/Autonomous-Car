import serial
import time

ser = serial.Serial(
    port='/dev/ttyTHS1',  # UART port
    baudrate=115200,        # Baud rate
    timeout=1             # Read timeout
)

def send_data(data):
    if ser.isOpen():
        print("Sending data...")
        ser.write(data)  # Convert string to bytes and send
    else:
        print("Serial port not open.")

x = "abc"
while True:
    send_data(x.encode())
    time.sleep(1)
    