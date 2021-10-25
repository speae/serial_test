import time
import serial

print("UART Demonstration Program")
print("NVIDIA Jetson Nano Developer Kit")

serial_port = serial.Serial(
    port="/dev/ttyTHS1",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

time.sleep(1)

try:
    serial_port.write("UART Demonstration Program\r\n".encode())
    serial_port.write("NVIDIA Jetson Nano Developer Kit\r\n".encode())

    while True:
        if serial_port.inWaiting() > 0:
            data = serial_port.read()
            print(data)

            if data == "\r".encode():
                serial_port.write("\n".encode())

except KeyboardInterrupt:
    print("Exist Program")

except Exception as exception_error:
    print("Error occured. Exiting Program")
    print("Error : " + str(exception_error))

finally:
    serial_port.close()
    pass
