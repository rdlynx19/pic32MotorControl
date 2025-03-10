// config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
// include other header files here
#define BUF_SIZE 200
#define CPR 334
#define INA219_REG_CURRENT 0x04
#define PLOTPTS 100
#define MAXPLOTPTS 400
#define DECIMATION 10
#define MAX_ARRAY_SIZE 1000

volatile int dutyCycle = 0;
// Kp = 0.03 and Ki = 0.05 for current control
static volatile float Kp_mA = 0.03, Ki_mA = 0.05;
// Kp = 8, Ki = 0.05 and Kd = 1500 for position control
static volatile float Kp_pos = 8.0, Ki_pos = 0.01, Kd_pos = 1500.0;
static volatile float desiredAngle = 0.0;
static volatile float commandedCurrent = 0.0;
static volatile int trajectorySize = 0;
static float refCurrent[PLOTPTS];
static float actCurrent[PLOTPTS];
static float refAngle[MAXPLOTPTS];
static float curAngle[MAXPLOTPTS];
static float motorAngle[MAX_ARRAY_SIZE];
static float refTraj[MAX_ARRAY_SIZE];


const char *modeToString(enum mode_t mode)
{
	switch (mode)
	{
	case IDLE:
		return "IDLE";
	case PWM:
		return "PWM";
	case ITEST:
		return "ITEST";
	case HOLD:
		return "HOLD";
	case TRACK:
		return "TRACK";
	default:
		return "UNKNOWN";
	}
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void)
{
	enum mode_t currentMode = get_mode();
	static int counter = 0;
	static int plotind = 0;
	static int decctr = 0;
	static float amplitude = -200.0;
	static float eint = 0;
	switch (currentMode)
	{
	case IDLE:
	{
		dutyCycle = 0;
		OC1RS = PR3 * dutyCycle / 100;
		LATAbits.LATA0 = 0;
		break;
	}
	case PWM:
	{
		if (dutyCycle > 0)
		{
			OC1RS = PR3 * dutyCycle / 100;
			LATAbits.LATA0 = 0;
		}
		else if (dutyCycle < 0)
		{
			OC1RS = -PR3 * dutyCycle / 100;
			LATAbits.LATA0 = 1;
		}
		break;
	}
	case ITEST:
	{
		if (counter % 25 == 0)
		{
			amplitude = -amplitude;
		}
		float actualCurrent = INA219_read_current();

		float e = amplitude - actualCurrent;
		
		eint = eint + e;

		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

		if (dutyCycle > 100.0)
		{
			dutyCycle = 100.0;
		}
		if (dutyCycle <= -100.0)
		{
			dutyCycle = -100.0;
		}
		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;
		
		refCurrent[counter] = amplitude;
		actCurrent[counter] = actualCurrent;
		counter++;
		if (counter >= 100)
		{
			set_mode(IDLE);
			counter = 0;
			eint = 0;
		}
		break;
	}
	case HOLD:
	{
		
		float actualCurrent = INA219_read_current();

		float e = commandedCurrent - actualCurrent;
		eint = eint + e;

		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

		if (dutyCycle > 100.0)
		{
			dutyCycle = 100.0;
		}
		if (dutyCycle <= -100.0)
		{
			dutyCycle = -100.0;
		}
		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;

		counter++;
		if(counter >= 10000){
			set_mode(IDLE);
			counter = 0;
			eint = 0;
		}
		break;
	}
	case TRACK:
	{
		float actualCurrent = INA219_read_current();

		float e = commandedCurrent - actualCurrent;
		eint = eint + e;

		dutyCycle = (Kp_mA * e) + (Ki_mA * eint);

		if (dutyCycle > 100.0)
		{
			dutyCycle = 100.0;
		}
		if (dutyCycle <= -100.0)
		{
			dutyCycle = -100.0;
		}
		OC1RS = (unsigned int)(abs(dutyCycle)/100.0 * PR3);
		LATAbits.LATA0 = (dutyCycle >= 0) ? 0 : 1;

		// counter++;
		// if(counter >= 10000){
		// 	set_mode(IDLE);
		// 	counter = 0;
		// 	eint = 0;
		// }
		break;
	}

	default:
	{
		break;
	}
	}

	// insert line to clear interrupt flag
	IFS0bits.T2IF = 0;
}

void __ISR(_TIMER_4_VECTOR, IPL6SOFT) PositionController(void)
{
	static float eprev = 0.0;
	static float eint = 0.0;
	static int index = 0;
	enum mode_t picMode = get_mode();
	if(picMode == HOLD){
		WriteUART2("a");
		while (!get_encoder_flag())
		{
		}
		set_encoder_flag(0);
		int cnt = get_encoder_count();
		float degreesPerCount = 360.0 / (CPR * 4);
		float encoderAngle = cnt * degreesPerCount;
		
		float e = desiredAngle - encoderAngle;
		float edot = e - eprev;
		eint = eint + e;
		commandedCurrent = (Kp_pos*e) + (Ki_pos*eint) + (Kd_pos*edot);
		eprev = e;
		curAngle[index] = encoderAngle;
		refAngle[index] = desiredAngle;
		index++;
	}
	if(picMode == IDLE){
		index = 0;
		eint = 0.0;
		eprev = 0.0;
	}

	if(picMode == TRACK){
		WriteUART2("a");
		while (!get_encoder_flag())
		{
		}
		set_encoder_flag(0);
		int cnt = get_encoder_count();
		float degreesPerCount = 360.0 / (CPR * 4);
		float encoderAngle = cnt * degreesPerCount;
		float desAngle = refTraj[index];
		float e = desAngle - encoderAngle;
		float edot = e - eprev;
		eint = eint + e;
		commandedCurrent = (Kp_pos*e) + (Ki_pos*eint) + (Kd_pos*edot);
		eprev = e;
		motorAngle[index] = encoderAngle;
		index++;
		if(index >= trajectorySize){
			desiredAngle = refTraj[trajectorySize - 1];
			set_mode(HOLD);
			eint = 0;
		}
	}

	IFS0bits.T4IF = 0;
}

void Setup_Timer4(){
	PR4 = 7500 - 1; // 200Hz ISR
	__builtin_disable_interrupts();
	T4CONbits.TCKPS = 0b101; // prescaler 32
	IPC4bits.T4IP = 6;
	IPC4bits.T4IS = 0;
	IFS0bits.T4IF = 0;
	IEC0bits.T4IE = 1;
	T4CONbits.ON = 1;
	TMR4 = 0;
	__builtin_enable_interrupts();
}

void Setup_PWM()
{
	PR3 = 2400 - 1; // 20kHz PWM Waveform
	__builtin_disable_interrupts();
	T2CONbits.TCKPS = 0;
	PR2 = 9600 - 1; // 5kHz ISR
	TMR2 = 0;
	// setting priority for the interrup timer
	IPC2bits.T2IP = 5;
	IPC2bits.T2IS = 0;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	RPB15Rbits.RPB15R = 0b0101;
	OC1CONbits.OCTSEL = 1;
	T3CONbits.TCKPS = 0;	// Timer3 prescaler N=1 (1:1)
	TMR3 = 0;				// initial TMR3 count is 0
	OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OC1CON bits are defaults
	OC1RS = 0;				// duty cycle = OC1RS/(PR3+1)
	OC1R = 0;				// initialize before turning OC1 on; afterward it is read-only
	T3CONbits.ON = 1;		// turn on Timer3
	OC1CONbits.ON = 1;		// turn on OC1
	T2CONbits.ON = 1;
	__builtin_enable_interrupts();
}

int main()
{
	char buffer[BUF_SIZE];
	UART2_Startup();   // Pico and PIC32 communcation
	NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
	INA219_Startup();  // current sensor startup
	Setup_PWM();
	Setup_Timer4();
	TRISBbits.TRISB10 = 0; // set pin B10 as output
	LATBbits.LATB10 = 0; // set the initial bit state
	TRISAbits.TRISA0 = 0; // set pin A0 as output
	LATAbits.LATA0 = 0;	  // set the initial direction of the motor rotation (depends on pin state)
	NU32DIP_YELLOW = 1;	  // turn off the LEDs
	NU32DIP_GREEN = 1;
	set_mode(IDLE);
	__builtin_disable_interrupts();
	// in future, initialize modules or peripherals here
	__builtin_enable_interrupts();

	while (1)
	{
		NU32DIP_ReadUART1(buffer, BUF_SIZE); // we expect the next character to be a menu command
		NU32DIP_GREEN = 1;					 // clear the error LED
		switch (buffer[0])
		{
		case 'a':
		{
			// read the current sensor (ADC counts)
			float adcReading = readINA219(INA219_REG_CURRENT);
			sprintf(buffer, "%f\r\n", adcReading);
			NU32DIP_WriteUART1(buffer);
			break;
		}

		case 'b':
		{
			// read the current sensors (mA current)
			float currentReading = INA219_read_current();
			sprintf(buffer, "%f\r\n", currentReading);
			NU32DIP_WriteUART1(buffer);
			break;
		}
		case 'c':
		{
			// read the encoder count
			WriteUART2("a");
			while (!get_encoder_flag())
			{
			}
			set_encoder_flag(0);
			sprintf(buffer, "%d\r\n", get_encoder_count());
			NU32DIP_WriteUART1(buffer);
			break;
		}

		case 'd':
		{
			// read the encoder count (degrees)
			WriteUART2("a");
			while (!get_encoder_flag())
			{
			}
			set_encoder_flag(0);
			int cnt = get_encoder_count();
			float degreesPerCount = 360.0 / (CPR * 4);
			float encoderAngle = cnt * degreesPerCount;
			sprintf(buffer, "%f\r\n", encoderAngle);
			NU32DIP_WriteUART1(buffer);
			break;
		}

		case 'e':
		{
			// reset the encoder count
			WriteUART2("b");
			break;
		}

		case 'f':
		{
			// set PWM duty cycle
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%d", &dutyCycle);
			set_mode(PWM);
			break;
		}

		case 'g':
		{
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%f %f", &Kp_mA, &Ki_mA);
			break;
		}

		case 'h':
		{
			sprintf(buffer, "Kp: %f, Ki: %f\r\n", Kp_mA, Ki_mA);
			NU32DIP_WriteUART1(buffer);
			break;
		}

		case 'i':
		{	
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%f %f %f", &Kp_pos, &Ki_pos, &Kd_pos);
			break;
		}
		
		case 'j':
		{
			sprintf(buffer, "Kp: %f, Ki: %f, Kd: %f\r\n", Kp_pos, Ki_pos, Kd_pos);
			NU32DIP_WriteUART1(buffer);
			break;
		}

		case 'k':
		{
			set_mode(ITEST);
			enum mode_t currentMode = get_mode();
			while (currentMode != IDLE)
			{
				currentMode = get_mode();
			}
			sprintf(buffer, "%d\r\n", PLOTPTS);
			NU32DIP_WriteUART1(buffer);
			if (currentMode == IDLE)
			{
				for (int i = 0; i < PLOTPTS; i++)
				{
					sprintf(buffer, "%f %f\r\n", refCurrent[i], actCurrent[i]);
					NU32DIP_WriteUART1(buffer);
				}
			}
			break;
		}

		case 'l':
		{
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%f", &desiredAngle);
			set_mode(HOLD);
			enum mode_t currentMode = get_mode();
			while (currentMode != IDLE)
			{
				currentMode = get_mode();
			}
			sprintf(buffer, "%d\r\n", MAXPLOTPTS);
			NU32DIP_WriteUART1(buffer);
			if (currentMode == IDLE)
			{
				for (int i = 0; i < MAXPLOTPTS; i++)
				{
					sprintf(buffer, "%f %f\r\n", refAngle[i], curAngle[i]);
					NU32DIP_WriteUART1(buffer);
				}
			}
			break;
		}
		
		case 'm':
		{
			int index = 0;
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%d", &trajectorySize);
			for(index = 0; index < trajectorySize; index++){
				NU32DIP_ReadUART1(buffer,BUF_SIZE);
				sscanf(buffer, "%f", &refTraj[index]);
			}
			
			break;
		}

		case 'n':
		{
			int index = 0;
			NU32DIP_ReadUART1(buffer, BUF_SIZE);
			sscanf(buffer, "%d", &trajectorySize);
			for(index = 0; index < trajectorySize; index++){
				NU32DIP_ReadUART1(buffer,BUF_SIZE);
				sscanf(buffer, "%f", &refTraj[index]);
			}
			break;
		}

		case 'o':
		{
			set_mode(TRACK);
			enum mode_t currentMode = get_mode();
			while(currentMode != HOLD){
				currentMode = get_mode();
			}
			sprintf(buffer, "%d\r\n", trajectorySize);
			NU32DIP_WriteUART1(buffer);
			if (currentMode == HOLD)
			{
				for (int i = 0; i < trajectorySize; i++)
				{
					sprintf(buffer, "%f %f\r\n", refTraj[i], motorAngle[i]);
					NU32DIP_WriteUART1(buffer);
				}
			}
			break;
		}

		case 'p':
		{
			// set PIC mode
			set_mode(IDLE);
			break;
		}

		case 'q':
		{
			// handle q for quit. Later you may want to return to IDLE mode here.
			enum mode_t currentMode = IDLE;
			set_mode(currentMode);
			break;
		}
		case 'r':
		{
			// read the current mode of the PIC32
			enum mode_t currentMode = get_mode();
			sprintf(buffer, "%s\r\n", modeToString(currentMode));
			NU32DIP_WriteUART1(buffer);
			break;
		}

		default:
		{
			NU32DIP_GREEN = 0; // turn on LED2 to indicate an error
			break;
		}
		}
	}
	return 0;
}
