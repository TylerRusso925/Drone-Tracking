#import statement for necessary libraries (installed with pyserial)
import subprocess, os, keyboard, time, serial
import numpy as np
#initialize port
try:
    ser = serial.Serial(port="COM4",baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS)
    print('Port Details -> ',ser)
    
except serial.SerialException:
    print('Port not available')
    
#def array used to read buffer
def seqw(x):
    seq = []
    ser.reset_input_buffer()
    for c in ser.read(x):
        seq.append(hex(c))
    print(seq)
    return seq
#sets current positions to 0
def set0():
    ser.write(b'\x50\x04\x10\x04\x00\x00\x00\x00')
    time.sleep(1)
    ser.write(b'\x50\x04\x11\x04\x00\x00\x00\x00')
#gets current position as according to the buffer output
def getpos():
    print("AZM: ")
    ser.write(b'\x50\x01\x10\x01\x00\x00\x00\x03')
    gg=seqw(4)
    azm=(int(gg[0],16)*(65536)+int(gg[1],16)*(256)+int(gg[2],16))*360/16777216
    if (azm>180):
        azm=azm-360
    print(azm)
    time.sleep(1)
    print("ALT: ")
    ser.write(b'\x50\x01\x11\x01\x00\x00\x00\x03')
    hh=seqw(4)
    alt=(int(hh[0],16)*(65536)+int(hh[1],16)*(256)+int(hh[2],16))*360/16777216
    if (alt>180):
        alt=alt-360
    print(alt)
    time.sleep(1)
    return azm,alt
#goes to alt home
def gohomealt():
    ser.write(b'\x50\x04\x11\x02\x00\x00\x00\x00')
#goes to azm home
def gohomeazm():
    ser.write(b'\x50\x04\x10\x02\x00\x00\x00\x00')
#goes to azm and alt home
def gohome():
    gohomealt()
    time.sleep(1)
    gohomeazm()
#go-to AZM
def goazm(azm):
    if (azm<0):
        azm=azm+360
    deggx = (azm/360)*16777215
    degx = round(deggx)
    x=hex(degx)
    x = x.lstrip('0x')
    while len(x)<6:
        x='0'+x
    y = '50041002'+ x + '00'
    a=bytes.fromhex(y)
    ser.write(a)
    return a
#go-to AlT
def goalt(alt):
    if (alt<0):
        alt=alt+360
    deggx = (alt/360)*16777215
    degx = round(deggx)
    x = hex(degx)
    x = x.lstrip('0x')
    while len(x)<6:
        x='0'+x
    y = '50041102'+ x + '00'
    a=bytes.fromhex(y)
    ser.write(a)
    return a
#go-to overall
def go(azm,alt):
    goazm(azm)
    goalt(alt)
#go-to coarse send ALT to vertical = 0 -- won't let you go back to typing after cmd
def gov0():
    ser.write(b'\x50\x04\x11\x02\xff\xe4\xba\x00')
#initial adjustable speed (change once hotkey is activated with - and =)
a = 7
keyboard.add_hotkey('`', lambda: hotkeymode())
#turns on hotkeys until the semicolon button is pressed
def hotkeymode():
    keyboard.add_hotkey('s', lambda: stop())
    keyboard.add_hotkey('x', lambda: slewaltdown())
    keyboard.add_hotkey('w', lambda: slewaltup())
    keyboard.add_hotkey('a', lambda: slewazmleft())
    keyboard.add_hotkey('d', lambda: slewazmright())
    keyboard.add_hotkey('e', lambda: slewdiagupright())
    keyboard.add_hotkey('q', lambda: slewdiagupleft())
    keyboard.add_hotkey('c', lambda: slewdiagdownright())
    keyboard.add_hotkey('z', lambda: slewdiagdownleft())
    keyboard.add_hotkey('=', lambda: aw())
    keyboard.add_hotkey('-', lambda: ae())
    keyboard.add_hotkey(';', lambda: removehotkeys())
    keyboard.add_hotkey('h', lambda: gohome())
    keyboard.add_hotkey('n', lambda: startcamera())
    keyboard.add_hotkey('m', lambda: endcamera())
#adjustable speed -- -
def ae():
    global a
    if(a > 0):
        a-=1
    print("Current speed: " + str(a))
#adjustable speed -- =
def aw():
    global a
    if(a < 9):
        a+=1
    print("Current speed: " + str(a))
#turns hotkey mode on when ` is pressed
def removehotkeys():
    keyboard.remove_hotkey('s')
    keyboard.remove_hotkey('x')
    keyboard.remove_hotkey('w')
    keyboard.remove_hotkey('a')
    keyboard.remove_hotkey('d')
    keyboard.remove_hotkey('e')
    keyboard.remove_hotkey('q')
    keyboard.remove_hotkey('c')
    keyboard.remove_hotkey('z')
    keyboard.remove_hotkey('=')
    keyboard.remove_hotkey('-')
    keyboard.remove_hotkey(';')
    keyboard.remove_hotkey('n')
    keyboard.remove_hotkey('m')
#slews at speed 7 in the direction specified
def slewaltdown():
    #ser.write(b'\x50\x02\x11\x25\x06\x00\x00\x00')
    #time.sleep(0.25)
    if(a == 1):
        ser.write(b'\x50\x02\x11\x25\x01\x00\x00\x00')
    elif(a == 2):
        ser.write(b'\x50\x02\x11\x25\x02\x00\x00\x00')
    elif(a == 3):
        ser.write(b'\x50\x02\x11\x25\x03\x00\x00\x00')
    elif(a == 4):
        ser.write(b'\x50\x02\x11\x25\x04\x00\x00\x00')
    elif(a == 5):
        ser.write(b'\x50\x02\x11\x25\x05\x00\x00\x00')
    elif(a == 6):
        ser.write(b'\x50\x02\x11\x25\x06\x00\x00\x00')
    elif(a == 7):
        ser.write(b'\x50\x02\x11\x25\x07\x00\x00\x00')
    elif(a == 8):
        ser.write(b'\x50\x02\x11\x25\x08\x00\x00\x00')
    elif(a == 9):
        ser.write(b'\x50\x02\x11\x25\x09\x00\x00\x00')
   
def slewaltup():
    #ser.write(b'\x50\x02\x11\x24\x06\x00\x00\x00')
    #time.sleep(0.25)
    if(a == 1):
        ser.write(b'\x50\x02\x11\x24\x01\x00\x00\x00')
    elif(a == 2):
        ser.write(b'\x50\x02\x11\x24\x02\x00\x00\x00')
    elif(a == 3):
        ser.write(b'\x50\x02\x11\x24\x03\x00\x00\x00')
    elif(a == 4):
        ser.write(b'\x50\x02\x11\x24\x04\x00\x00\x00')
    elif(a == 5):
        ser.write(b'\x50\x02\x11\x24\x05\x00\x00\x00')
    elif(a == 6):
        ser.write(b'\x50\x02\x11\x24\x06\x00\x00\x00')
    elif(a == 7):
        ser.write(b'\x50\x02\x11\x24\x07\x00\x00\x00')
    elif(a == 8):
        ser.write(b'\x50\x02\x11\x24\x08\x00\x00\x00')
    elif(a == 9):
        ser.write(b'\x50\x02\x11\x24\x09\x00\x00\x00')
def slewazmleft():
    #ser.write(b'\x50\x02\x10\x25\x06\x00\x00\x00')
    #time.sleep(0.36)
    if(a == 1):
        ser.write(b'\x50\x02\x10\x25\x01\x00\x00\x00')
    elif(a == 2):
        ser.write(b'\x50\x02\x10\x25\x02\x00\x00\x00')
    elif(a == 3):
        ser.write(b'\x50\x02\x10\x25\x03\x00\x00\x00')
    elif(a == 4):
        ser.write(b'\x50\x02\x10\x25\x04\x00\x00\x00')
    elif(a == 5):
        ser.write(b'\x50\x02\x10\x25\x05\x00\x00\x00')
    elif(a == 6):
        ser.write(b'\x50\x02\x10\x25\x06\x00\x00\x00')
    elif(a == 7):
        ser.write(b'\x50\x02\x10\x25\x07\x00\x00\x00')
    elif(a == 8):
        ser.write(b'\x50\x02\x10\x25\x08\x00\x00\x00')
    elif(a == 9):
        ser.write(b'\x50\x02\x10\x25\x09\x00\x00\x00')
def slewazmright():
    #ser.write(b'\x50\x02\x10\x24\x06\x00\x00\x00')
    #time.sleep(0.36)
    if(a == 1):
        ser.write(b'\x50\x02\x10\x24\x01\x00\x00\x00')
    elif(a == 2):
        ser.write(b'\x50\x02\x10\x24\x02\x00\x00\x00')
    elif(a == 3):
        ser.write(b'\x50\x02\x10\x24\x03\x00\x00\x00')
    elif(a == 4):
        ser.write(b'\x50\x02\x10\x24\x04\x00\x00\x00')
    elif(a == 5):
        ser.write(b'\x50\x02\x10\x24\x05\x00\x00\x00')
    elif(a == 6):
        ser.write(b'\x50\x02\x10\x24\x06\x00\x00\x00')
    elif(a == 7):
        ser.write(b'\x50\x02\x10\x24\x07\x00\x00\x00')
    elif(a == 8):
        ser.write(b'\x50\x02\x10\x24\x08\x00\x00\x00')
    elif(a == 9):
        ser.write(b'\x50\x02\x10\x24\x09\x00\x00\x00')
def slewdiagupright():
    slewaltup()
    time.sleep(.0375)
    slewazmright()
def slewdiagupleft():
    slewaltup()
    time.sleep(.0375)
    slewazmleft()
def slewdiagdownright():
    slewaltdown()
    time.sleep(.0375)
    slewazmright()
def slewdiagdownleft():
    slewaltdown()
    time.sleep(.0375)
    slewazmleft()
#stops all movement
def stop():
    ser.write(b'\x50\x02\x10\x24\x00\x00\x00\x00')
    time.sleep(.03725)
    ser.write(b'\x50\x02\x11\x24\x00\x00\x00\x00')
    time.sleep(.03725)
    ser.write(b'\x50\x02\x10\x25\x00\x00\x00\x00')
    time.sleep(.03725)
    ser.write(b'\x50\x02\x10\x25\x00\x00\x00\x00')
#startcam bound to n
def startcamera():
    subprocess.run('start microsoft.windows.camera:', shell=True)
    time.sleep(.0375)
#endcam bound to m
def endcamera():
    subprocess.run('Taskkill /IM WindowsCamera.exe /F', shell=True)
    time.sleep(.0375)
