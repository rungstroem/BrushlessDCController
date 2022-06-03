#define F_CPU 16000000UL
#define BAUDRATE 9600
#define BAUD_PRESC ((F_CPU/(16UL*BAUDRATE))-1)

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
#define minPWM 0x60
#define startPWM 0xA0
#define neutralState 6
#define minSpeed 500

#define servoCenter 22
#define servoMin 9
#define servoMax 35

#include <avr/io.h>
#include <avr/interrupt.h>

unsigned char state = 0x00;
unsigned char PWM = 0x00;
bool isSpinning = false;

volatile unsigned char dataArray[2];
volatile int dataPointer = 0;
volatile bool newData = false;
ISR(USART_RX_vect){
	dataArray[dataPointer] = UDR0; 
	dataPointer++;
	if(dataPointer >= 2){
	  newData = true;	//Wait for second transfer
    dataPointer %= 2;  //Roll-over
	}
}

bool closedLoop = false;
ISR(ANALOG_COMP_vect){
	closedLoop = true;
	set_comp_trigger_mode(state);
	state++;
	state %= 6; //Roll over if state=6

	return;
}

ISR(WDT_vect){

}

void delayMicroS(int i){
	//Remember Kenneth, the compiler knows if you do something just to waste cycles. It will just remove your code from the compiled code.
	//Use the assembly _NOP to faste cycles
	//At 16MHz 1µS is 16 cycles
	for(int j=0; j<(i*16); j++){
		_NOP();
	}
}

void initSerial(){
	//Enable USART on power reduction register PRR
	PRR &= ~(1<<PRUSART0);
	
	//Disable digital input logic on comp pins
	DIDR1 &= ~(1<<AIN1D);
	DIDR1 &= ~(1<<AIN0D);
	//Setup
	UCSR0A = 0b00000000;
	UCSR0B = 0b10011000;  //RX-interrupt, RX-enable, TX-enable
	UCSR0C = 0b00000110;  //8 databits, 1 stopbit
	UBRR0H = (unsigned char)(BAUD_PRESC >> 8);
	UBRR0L = (unsigned char)(BAUD_PRESC);

}
void initAnalogComp(){
	//Enable the ADC MUX so the comparator can use it
	PRR &= ~(1<<PRADC);
	
	ADCSRA = (0 << ADEN);	//Turn off ADC
	ADCSRB |= (1 << ACME);	//Enable MUX on analog comp
	DIDR1 |= (1<<AIN1D) | (1<<AIN0D);
}

void initWDT(){
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0b01000101;
}

void initTimer0(){
	//Enable timer0 on power reduction register PRR
	PRR &= ~(1<<PRTIM0);  //Enable timer0 - enable=0

	//PWM setup
	TCCR0A |= (1<<COM0B1) | (1<<COM0B0);  //Set OCR0B on compare match counting up, reset at bottom (interting PWM)
	//TCCR0A |= (1<<WGM00) | (1<<WGM01);  //Set fast-PWM (Top = 0xFF -> 256)
	TCCR0A |= (1<<WGM00);  //Set phase-correct-PWM (Top = 0xFF -> 256)
	
	//TCCR0B |= (1<<CS01);  //Prescaler = 8 (7812.5Hz)
	TCCR0B |= (1<<CS00);  //Prescaler = 0 (62500Hz)

	//OCR0A & OCR0B - compare registers ( reset = 0xFF )
	//OCR0A = 0xFF; 	//Use this for interrupt if needed - cannot be an output since it is the positive comparator pins
	OCR0B = 0xFF;	//Use this for PWM 

}
//Timer1 will be configured for 8Bit resolution to match timer0
void initTimer1(){
	//Enable timer1 on power reduction register PRR
	PRR &= ~(1<<PRTIM1);  //Enable timer1 - enable=0

	//PWM setup
	TCCR1A |= (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0);  //Set OC1A/B on compare match, reset at bottom (inverting PWM)
	TCCR1A |= (1<<WGM10); //Set 8Bit phase-correct-PWM   (Top = 0x00FF -> 256)
	
	//TCCR1B |= (1<<CS11);  //Prescaler = 8 (7812.5Hz)
	//TCCR1B |= (1<<WGM12) | (1<<CS10);  // fast-PWM & Prescaler = 0 (62500Hz)
	TCCR1B |= (1<<CS10);  //Prescaler = 0 (62500Hz)
		
		/* WGM12 is set, this means that Fast-PWM is used. This is a test for now	*/
  
	//OCR1AH - High byte of compare register
	//OCR1AL - Low byte of compare register
	OCR1AH = 0x00;  //Reset compare registers
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
}
//This timer is for controlling servos
//The 2 output pins should be enough for 2 servos
void initTimer2(){
	//Enable timer2 on power reduction register PRR
	PRR &= ~(1<<PRTIM2);  //Enable timer 2 - enable=0

	//For Servo control
	TCCR2A |= (1<<COM2A1) | (1<<COM2B1);  //Clear OCR2A/B on compare match (non-inverting PWM)
	TCCR2A |= (1<<WGM21) | (1<<WGM20);  //Set Fast-PWM (Top = 0xFF -> 256)
	//TCCR2B |= (1<<CS22);  //Prescaler = 64 (976.6Hz)
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);  //Prescaler = 64 (976.6Hz)

	//OCR2A & OCR2B - compare registers
	OCR2A = servoCenter;     //right elevon
	OCR2B = servoCenter;    //Left elevon
}

void portSetup(){
	DDRB |= (1<<3);
	DDRD |= (1<<3);
	//DDRD = DDRD | 0x38;	//0b00111000
	//DDRB = DDRB | 0x1F;	//0b00011111
}

char readADC(unsigned char ADCChannel){
	ADMUX = (ADMUX & 0xF0) | (ADCChannel & 0x0F); //Select channel

	ADCSRA |= (1<<ADSC);  //Start ADC
	while(ADCSRA & (1<<ADSC));
		//Wait for ADC to complete a read 
	return ADCH;
}

unsigned char BReg[7] = {0x12,0x10,0x01,0x05,0x04,0x02,0x00};
unsigned char DReg[7] = {0x00,0x20,0x20,0x00,0x10,0x10,0x00};
void comutate(unsigned char state){   //This sets the comutation 
	PORTD = DReg[state] | 0x80;
	DDRD = DReg[state] | 0x80;
	PORTB = BReg[state] | 0x80;
	DDRB = BReg[state] | 0x80;
}

void BEMF_A_RISING(){
	ADMUX = 0;
	ACSR |= 0x03;
	//UDR0 = 'R';
}
void BEMF_A_FALLING(){
	ADMUX = 0;
	ACSR |= 0x02;
	//UDR0 = 'F';
}
void BEMF_B_RISING(){
	ADMUX = 1;
	ACSR |= 0x03;
	//UDR0 = 'R';
}
void BEMF_B_FALLING(){
	ADMUX = 1;
	ACSR |= 0x02;
	//UDR0 = 'F';
}
void BEMF_C_RISING(){
	ADMUX = 2;
	ACSR |= 0x03;
	//UDR0 = 'R';
}
void BEMF_C_FALLING(){
	ADMUX = 2;
	ACSR |= 0x02;
	//UDR0 = 'F';
}
// A:	B4 - High B2 - Low	(PWM - Timer1B)
// B:	B0 - High B1 - Low	(PWM - Timer1A)
// C: 	D4 - High D5 - Low  (PWM - Timer0B)
#define A_PORT PORTB
#define A_DDR DDRB
#define AH 0b00010000 //(1<<4);
#define AL 0b00000100 //(1<<2);
#define B_PORT PORTB
#define B_DDR DDRB
#define BH 0b00000001 //(1<<0);
#define BL 0b00000010 //(1<<1);
#define C_PORT PORTD
#define C_DDR DDRD
#define CH 0b00010000 //(1<<4)
#define CL 0b00100000 //(1<<5)

void zero_out(){
	
	PORTD = 0x80;
	DDRD = 0x80;
	PORTB = 0x80;
	DDRB = 0x80;

}

void AH_BL(){
	PORTD = 0x80;
	DDRD = 0x80;
	PORTB = 0x80 | AH | BL;
	DDRB = 0x80 | AH | BL;
}
void AH_CL(){
	PORTD = 0x80 | CL;
	DDRD = 0x80 | CL;
	PORTB = 0x80 | AH;
	DDRB = 0x80 | AH;
}
void BH_CL(){
	PORTD = 0x80 | CL;
	DDRD = 0x80 | CL;
	PORTB = 0x80 | BH;
	DDRB = 0x80 | BH;
}
void BH_AL(){
	PORTD = 0x80;
	DDRD = 0x80;
	PORTB = 0x80 | BH | AL;
	DDRB = 0x80 | BH | AL;
}
void CH_AL(){
	PORTD = 0x80 | CH;
	DDRD = 0x80 | CH;
	PORTB = 0x80 | AL;
	DDRB = 0x80 | AL;
}
void CH_BL(){
	PORTD = 0x80 | CH;
	DDRD = 0x80 | CH;
	PORTB = 0x80 | BL;
	DDRB = 0x80 | BL;
}

unsigned char ADCChannel[6] = {0x02, 0x00, 0x01, 0x02, 0x00, 0x01};
void set_comp_trigger_mode(unsigned char state){
	switch(state){
		case 0:
			AH_BL();
			BEMF_C_RISING();
			break;
		case 1:
			AH_CL();
			BEMF_B_FALLING();
			break;
		case 2:
			BH_CL();
			BEMF_A_RISING();
			break;
		case 3:
			BH_AL();
			BEMF_C_FALLING();
			break;
		case 4:
			CH_AL();
			BEMF_B_RISING();
			break;
		case 5:
			CH_BL();
			BEMF_A_FALLING();
			break;
			
	}
	return;
}

void pwm_timer0B(unsigned char duty){
	OCR0B = 0xFF - duty;
}
void pwm_timer1A(unsigned char duty){
	//OCR1A = (0x00FF & duty);  //Use this if you want to make sure the high byte register is always 0
	OCR1AL = 0xFF - duty;
}
void pwm_timer1B(unsigned char duty){
	OCR1BL = 0xFF - duty;
}
void pwm_all(unsigned char duty){
	pwm_timer0B(duty);
	pwm_timer1A(duty);
	pwm_timer1B(duty);
}

inline unsigned char limit_deg(unsigned char deg){
  //deg = (0<deg) ? ( (deg<60) ? deg : 60 ) : 0;
	// -30 to 30 degrees offset 30 to avoid negative values - 0 to 60
	// 0 to 60 in PWM:	PWM = 16 + (degrees/4)
	//return (unsigned char) servoMin+(deg/4);
  
  return deg = (0<deg) ? ( (deg<80) ? deg : 80 ) : 0;
}

void servo1(unsigned char deg){
  //Right elevon - invert for proper function
	OCR2A = 33-(limit_deg(deg)/3);
}
void servo2(unsigned char deg){
  //Left elevon
	OCR2B = 11+(limit_deg(deg)/3);
}

void ramp_up(unsigned char mPWM, int endSpeed){
	//Ramp-up loop
	pwm_all(mPWM);  //High current start PWM
	state = 0; //Known location
	comutate(state);    //Get to known location	
	delayMicroS(500000);	//500mS delay
	int i = 2000;	//2mS
	while(i > endSpeed){
		delayMicroS(i);
		state++;
		state %= 6;	//Roll-over when state=6
		set_comp_trigger_mode(state);
		//comutate(state);
		i -= 10;
	}
	pwm_all(minPWM);
	for(int j = 0; j<6000; j++){
		state++;
		state %= 6;
		set_comp_trigger_mode(state);
		delayMicroS(endSpeed);
	}
}

void openloop_comutation(int speed){
	delayMicroS(speed);
	set_comp_trigger_mode(state);
	//comutate(state);
	state++;
	state %= 6;
	
}

int main(){	
	cli();  //Clear global interrupt
	initSerial();
	portSetup();
	initAnalogComp();
	initTimer0();
	initTimer1();
	initTimer2(); //For servo's
	//initWDT();
	sei();  //Set global interrupt flag
	
	//comutate(neutralState);	//Start with all outputs off

	while(1){	//Initial loop
		if( (isSpinning != true) ){
			isSpinning = true;
			ramp_up(startPWM, minSpeed);
			break;
		}
		
	}
	ACSR |= (1<<ACIE);

	while(1){	//Running loop
		//if(!closedLoop){
			//openloop_comutation(minSpeed);
		//}


		if(newData == true){
        
			switch(dataArray[0]){
				case 0x01:
					pwm_all(dataArray[1]);
					newData = false;
					break;
				case 0x02:
					//if(dataArray[1] > 60) dataArray[1] = 60;	//Limit
					servo2(dataArray[1]);
					newData = false;
					break;
				case 0x03:
					//if(dataArray[1] > 60) dataArray[1] = 60;	//Limit
					servo1(dataArray[1]);
					newData = false;
					break;
			}
		}
	}
 
	return 0;
}
