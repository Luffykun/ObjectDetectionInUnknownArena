#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include<math.h>

//Initializing global variable

int global_orientatn = 0;
/*global orientatn specifies the orientatn of the bot in the arena.
i.e. whether the bot is facing forward or right or left or backward.*/
int flag_goes_into_interrupt=0;
int pulkit_ans;
int net_angle=0;
unsigned char data=0x00;

int counter;
int forward = 0;
//stores the displacement of the bot ONLY in the forward direction.
int right = 0;
//stores the displacement of the bot ONLY in the rightward/leftward directn.

unsigned long int ShaftCountLeft = 0;
//to keep track of left position encoder
unsigned long int ShaftCountRight = 0;
//to keep track of right position encoder

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

void adc_pin_config (void)
{
 DDRF = 0x00; //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00; //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void port_init()
{
 motion_pin_config();
 adc_pin_config();
 left_encoder_pin_config(); //left encoder pin config
 right_encoder_pin_config(); //right encoder pin config
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt
}

ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}

void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}

//rotates the bot by a specified angle
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.5; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0;
 ShaftCountLeft = 0;

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 motion_set(0x00); //Stop robot
}

//Function used for moving robot forward by specified distance
void linear_distance_mm(unsigned int DistanceInMM)
{
	motion_set(0x06);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}

	/*updates the value of "forward" and "right" according to the orientation.
	ie updates the displacement of the bot	*/
	int mod = global_orientatn % 4;
	switch(mod)
	{
		case 0 :	forward = forward + DistanceInMM; //in this case the bot is facing forward
					break;
		case 1 :    right = right +  DistanceInMM; //in this case bot is facing rightward
					break;
		case 2 :    forward = forward - DistanceInMM; //in this case bot is facing backward
					break;
		case 3 :    right = right - DistanceInMM; //in this case bot is facing leftward
					break;
	}

	motion_set(0x00);
	_delay_ms(10);

}

void linear_distance_mm1(unsigned int DistanceInMM)
{
	motion_set(0x06);
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}

	motion_set(0x00);
	_delay_ms(10);

}


//turns left as well as updates the global orientation
void turn_left(void)
{
	motion_set(0x05);
	angle_rotate(90);
	global_orientatn--;
	motion_set(0x00);
}
//turn right as well as updates the global orientation
void turn_right(void)
{
	motion_set(0x0A);
	angle_rotate(90);
	global_orientatn++;
	motion_set(0x00);
}


void turn_right1(void)
{
	motion_set(0x0A);
	angle_rotate(88);
	global_orientatn++;
	motion_set(0x00);
}

void turn_left1(void)
{
	motion_set(0x05);
	angle_rotate(95);
	global_orientatn--;
	motion_set(0x00);
}

unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}


void move_towards_ball()
{ double proportionality_factor=45.0/64.0;
  int reqd_angle;
  unsigned int value;
  unsigned char sharp;
  	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);
  while(value>=200)
  {

  flag_goes_into_interrupt=0;
  reqd_angle = pulkit_ans * proportionality_factor;
  rotate_to_centre(reqd_angle);
  linear_distance_mm1(200);
  update_coordinates(reqd_angle);
  //_delay_ms(2000);

  send_data(0x00);//send byte to laptop
  while(!flag_goes_into_interrupt);

  	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);

  }
  motion_set(0x00);
  while(1);
}

SIGNAL(SIG_USART0_RECV)
{ 
  pulkit_ans = 0;
  data=UDR0;
  if(data!=0x03)
   flag_goes_into_interrupt=1;
  int first_bit = (data & 0x80)/128; //Check first bit to see if ball was found
  //if(data != 0xFF)
  //{
  if(first_bit == 1)
  {
 	int second_bit = (data & 0x40)/64;
	pulkit_ans = data & 0x3F;
	if(second_bit == 1)
	{
		pulkit_ans *= (-1);
	}
  }
  //}
}

void send_data(unsigned char send)
{
	while(data != 0x03)
	{
		UDR0 = send;	
		//_delay_ms(1000);
	}
}

//this function scans 360 degrees looking for the specified object
void rotate_360 (int rotatn_angle)
 {	counter=0;
    int first_bit;
	 for (int i = 0 ; i < 360 / rotatn_angle ; i++)
		 {
		  flag_goes_into_interrupt=0;
		  motion_set(0x0A);
		  angle_rotate(rotatn_angle);
		  motion_set(0x00);		  
		  send_data(0x00);//send byte to laptop

		  while(!flag_goes_into_interrupt); //Will wait until data is received
		  
		  //goes into interrupt
		  flag_goes_into_interrupt = 0;
		  first_bit = (data & 0x80)/128;
		  if(first_bit == 1)
		  {
			 	net_angle = (((global_orientatn % 4)*90 + (counter*rotatn_angle)) % 360 );
				move_towards_ball();
		  }
		  while(data != 0x00);

		  _delay_ms(500);
		  counter++;
		  /*check pulkits ans
		  if yes pura loop break*/
		 }
 }




int check_obstacles(unsigned int check_value)
{
	unsigned int value;
	unsigned char sharp;
	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);
	if(value<check_value) return 1;
	return 0;
}

void restore_orientation(int initial)
{
	int curr_orientatn = global_orientatn;
	for(int i = 0; i < (curr_orientatn - initial) % 4; i++)
	{
		linear_distance_mm(300);
		turn_left();
	}
}

void obstacle_detected(void)
{
	int init_orientatn = global_orientatn;
	rotate_360(90);

	do
	{

		while(check_obstacles(200)==1)
		{
			turn_right();
			motion_set(0x00);
			_delay_ms(500);
		}

		linear_distance_mm(100);
		motion_set(0x00);
		_delay_ms(500);

		turn_left();
		motion_set(0x00);
		_delay_ms(500);
	}
	while (check_obstacles(200)==1);

    rotate_360(90);
	//distance move karo
	restore_orientation(init_orientatn);
	//to set back the orientatn to forward

}

void move_forward(unsigned int distance)
{
	//int final_forward = forward;
	int number_of_iters = distance/60;
	for(int i  = 0; i < number_of_iters; i++)
	{
		if(check_obstacles(200))
		{
			obstacle_detected();

		}

		//if(forward > Y_limit)
		//break;

		linear_distance_mm(60);
	}
}

void update_coordinates (int angle)
{
	
	net_angle = net_angle + angle;
	//remember right side positive
	//forward = forward + 200*cos(net_angle);
	//right = right + 200*sin(net_angle);

}

void rotate_to_centre( int angle )
{

	if(angle >= 0)
	motion_set(0x0A);

	else
	{
	motion_set(0x05);
	angle = angle*(-1);
	}

	angle_rotate(angle);
	motion_set(0x00);
}


void init_devices (void)
{
 cli();
 port_init();
 adc_init();
 uart0_init();
 left_position_encoder_interrupt_init();
 right_position_encoder_interrupt_init();
 sei();
}

int main()
{ init_devices();
  //unsigned int forward_count = 0;
  //unsigned int right_count = 0;
  unsigned int value;
  unsigned char sharp;

  unsigned int rotatn_angle = 90;
  unsigned int arena_forward=900,arena_right=900;
  //unsigned int init_forward=forward, init_right=right;
  unsigned int move = 300, side = 300;
  unsigned int left_or_right = 0;

  while(1)
  {
  	 // forward =0;
	 // while(((forward + move <= arena_forward) && global_orientatn == 0) || ((forward <= move) && global_orientatn == 2))
	 while((forward + move <= arena_forward && global_orientatn ==0) || (forward >= move && global_orientatn ==2 ))
	  {
		  rotate_360(rotatn_angle);
		  //motion_set(0x00);
		  move_forward(move);
		  motion_set(0x00);
		  _delay_ms(1000);
	  }
	  if(left_or_right == 0)
	  {
		  turn_right1();
		  _delay_ms(1000);
		  move_forward(side);
		  motion_set(0x00);
		  _delay_ms(1000);
		  turn_right();
		  // motion_set(0x00);
		  _delay_ms(1000);
		  left_or_right = 1;

	  }
	  else
	  {
		  turn_left();
		  // motion_set(0x00);
		  _delay_ms(1000);
		  move_forward(side);
		  motion_set(0x00);
		  _delay_ms(1000);
		  turn_left();
		  // motion_set(0x00);
		  _delay_ms(1000);
		  left_or_right = 0;
	  }

	  
}
}
