import serial
ser = serial.Serial('/dev/ttyUSB0',230400)
print('Opening port: ')
print(ser.name)

has_quit = False
# menu loop
while not has_quit:
    print('PIC32 MOTOR DRIVER INTERFACE')
    # Display the menu options with consistent spacing
    print(f"{'a: Read current sensor (ADC counts)'.ljust(40)} b: Read current sensor (mA)") 
    print(f"{'c: Read encoder (counts)'.ljust(40)} d: Read encoder (deg)") 
    print(f"{'e: Reset counter'.ljust(40)} f: Set PWM (-100 to 100)")
    print(f"{'g: Set current gains'.ljust(40)} h: Get current gains")
    print(f"{'i: Set position gains'.ljust(40)} j: Get position gains")
    print(f"{'k: Test current control'.ljust(40)} l: Go to angle (deg)")
    print(f"{'m: Load step trajectory'.ljust(40)} n: Load cubic trajectory")
    print(f"{'o: Execute trajectory'.ljust(40)} p: Unpower the motor")
    print(f"{'q: Quit client'.ljust(40)} r: Get mode")
    # read the user's choice
    selection = input('\nENTER COMMAND: ')
    selection_endline = selection+'\n'
     
    # send the command to the PIC32
    ser.write(selection_endline.encode()); # .encode() turns the string into a char array
    
    # take the appropriate action
    # there is no switch() in python, using if elif instead
    if(selection == 'a'):
        n_str = ser.read_until(b'\n')
        n_flt = float(n_str)
        print(f"ADC counts: {n_flt}\n")
    elif(selection == 'b'):
        n_str = ser.read_until(b'\n')
        n_flt = float(n_str)
        print(f"Current Reading: {n_flt}\n")
    elif(selection == 'c'):
        n_str = ser.read_until(b'\n')
        n_flt = float(n_str)
        print(f"Encoder Counts: {n_flt}\n")
    elif (selection == 'd'):
        n_str = ser.read_until(b'\n')
        n_flt = float(n_str)
        print(f"Encoder Angle: {n_flt}\n")
    elif (selection == 'e'):
        print(f"Encoder count set to zero!\n")
    elif (selection == 'f'):
        pwm_input = input("Enter PWM value (-100 to 100): ")
        pwm_int = int(pwm_input)
        print(f"PWM value set to {pwm_int}\n")
        ser.write((str(pwm_int)+'\n').encode())
    elif (selection == 'g'):
        Kp_input = input("Enter proportional gain value: ")
        Ki_input = input("Enter integral gain value: ")
        Kp_flt = float(Kp_input)
        Ki_flt = float(Ki_input)
        print(f"Current gains set to Kp: {Kp_flt} and Ki: {Ki_flt}\n")
        ser.write((str(Kp_flt) + str(Ki_flt) + '\n').encode())
    elif (selection == 'h'):
        gains = ser.readline().decode().strip()
        kp_value = float(gains.split("Kp:")[1].split(",")[0].strip())
        ki_value = float(gains.split("Ki:")[1].strip())
        print(f"Kp: {kp_value} and Ki: {ki_value}\n")
    elif (selection == 'i'):
        pass
    elif (selection == 'j'):
        pass
    elif (selection == 'k'):
        print(f"Testing current gains!\n")
    elif (selection == 'l'):
        pass
    elif (selection == 'm'):
        pass
    elif (selection == 'n'):
        pass
    elif (selection == 'o'):
        pass
    elif (selection == 'p'):
        print(f"Unpowering the motor...")
    elif (selection == 'q'):
        print('Exiting client')
        has_quit = True; # exit client
        # be sure to close the port
        ser.close()
    elif (selection == 'r'):
        mode_str = ser.read_until(b'\n')
        mode_str = mode_str.decode().rstrip('\r\n')
        print("PIC Mode is " + str(mode_str) + "\n")

    else:
        print('Invalid Selection ' + selection_endline)



