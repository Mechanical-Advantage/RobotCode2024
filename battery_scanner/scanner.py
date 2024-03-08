import serial
import array

timeout = 5
prefix = array.array('B', [0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31])
scan_command = array.array('B',[0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xab, 0xcd])
name_length = 8
response_length = len(prefix) + name_length

with serial.Serial('COM3', 9600, timeout=timeout) as ser:
    
    while True:
        ser.write(scan_command)
        response = ser.read(response_length)
        
        if len(response) == response_length:
            if response.startswith(prefix):
                print(f'prefix matches {response.hex()}')
                print(f"name: {response[-8:]}")
                
            else:
                print(f'prefix doesnt match {response.hex()}')
            print('too short')
            
       
    

