#Dependencies
import serial #pyserial

ser = serial.Serial('/dev/ttyUSB0', 9600) #Open serial port at 9600 baud

count = 0 #Counter for first n-lines of serial information

while True:
    if(ser.in_waiting > 0): #Check if serial stream is available
        count += 1
        line = ser.readline() #Read the serial stream

        if count > 10: #Avoid first 10-lines of serial information
            line_filtered = line.rstrip().lstrip() #Strip serial stream of spaces
            
            line_filtered = str(line_filtered) #Convert serial stream to string
            line_filtered = line_filtered.strip("'") #Strip serial stream of '
            line = line_filtered.strip("b'") #Strip serial stream of b'

            line = float(line) #Convert serial stream to float
            print(line,"\n") #Print the serial stream
