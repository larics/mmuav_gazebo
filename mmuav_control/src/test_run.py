from struct import *
import serial
import time

if __name__ == '__main__':
    count = 0
    #s = pack('iiiicxxx', 250, 00 , -250, 00, 'C')
    s = pack('iiiicxxx', 0, 0, 0, 0, 'C')
    #time.sleep(1)
    #s = pack('iiiicxxx', 0, 0, 0, 0, 'C')
            #              P  omega rLim dLim
    #s = pack('iiiicxxx', 5, 1200, 1000/100, 18000/100,'S')

    # param1 do param 4 -> referenca broja pulseva
    # 'C' - karakter koji oznacava da poruka predstavlja postavne vrijednosti

    # serial port setup
    print(len(s))
    ser = serial.Serial()
    ser.baudrate = 115200
    # promjeni serisjki port za svoje racunalo
    ser.port = 'COM3'
    ser.parity = serial.PARITY_NONE
    ser.bytesize = serial.EIGHTBITS
    ser.stopbits = serial.STOPBITS_ONE
    ser.open()
    #time.sleep(0.5)
    ser.write(s)

    ser.close()