/********************************************//**
 * Written by : Group 6, CS101 Embedded Systems Project
 *
 * AVR Studio Version 4.17, Build 666
 *
 * Date : 18-04-15
 *
 * This code demonstrates the motion of the bot inside
 * the arena looking for the object. The bot will capture
 * images with the help of a webcam at various instants from
 * different positions and sent it to the laptop for
 * image processing. Once the ball is detected the bot
 * will move towards the ball and stop at a certain distance
 * from it.
 *
 * Note:
 * 1. Make sure that in the configuration options following settings are
 * done for proper operation of the code
 * Microcontroller: atmega2560
 * Frequency: 14745600
 * Optimization: -O0
 ***********************************************/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//int ARENA_LENGTH_IN_CM = 145;
//int ARENA_WIDTH_IN_CM = 145;

/** Global orientatn specifies the orientatn of the bot in the arena.
  *
  *i.e. whether the bot is facing forward or x_coordinate_of_bot or left or backward.  */
int global_orientation_of_bot = 0;

///Flag goes high when code goes into interrupt.
int flag_goes_into_interrupt = 0;

///Stores the distance of the centre of the ball from the centre of the image.
int distance_of_the_centre_of_ball_from_centre_of_image;

///Keeps storage of the net angle rotated by the bot,with respect to y-axis, after detecting the ball.
int net_angle_rotated_after_detecting_ball=0;

///Stores the data received by the bot from the laptop.
unsigned char data_received_by_bot_from_laptop=0x00;

///Keeps count of the turn,while rotating, from global orientation at which ball was detected.
int turn_from_global_orientation_at_which_ball_was_detected=0;

///Stores the displacement of the bot ONLY in the Y direction.
int y_coordinate_of_bot = 0;

///Stores the displacement of the bot ONLY in the X direction.
int x_coordinate_of_bot = 0;

///Stores the Y co-ordinate of the bot when it sees the ball for the first ball.
int y_coordinate_of_bot_at_which_ball_was_detected = 0;

///Stores the X co-ordinate of the bot when it sees the ball for the first ball.
int x_coordinate_of_bot_at_which_ball_was_detected = 0;

///To keep track of left position encoder.
unsigned long int ShaftCountLeft = 0;

///To keep track of right position encoder.
unsigned long int ShaftCountRight = 0;

///Function to initialize UART0
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

///Function to confifure the motion pins
void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F;   //set direction of the PORTA 3 to PORTA 0 pins as output
 PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}

///Function to confifure the adc pins
void adc_pin_config (void)
{
 DDRF = 0x00;  //set PORTF direction as input
 PORTF = 0x00; //set PORTF pins floating
 DDRK = 0x00;  //set PORTK direction as input
 PORTK = 0x00; //set PORTK pins floating
}

///Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

///Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}

///Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

///Function to Initialize PORTS
void port_init()
{
 motion_pin_config();
 adc_pin_config();
 left_encoder_pin_config();  //left encoder pin configuration
 right_encoder_pin_config(); //right encoder pin configuration
}

///Interrupt 4 enable
void left_position_encoder_interrupt_init (void)
{
 cli();                 //Clears the global interrupt
 EICRB = EICRB | 0x02;  // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10;  // Enable Interrupt INT4 for left position encoder
 sei();                 // Enables the global interrupt
}

///Interrupt 5 enable
void right_position_encoder_interrupt_init (void)
{
 cli();                 //Clears the global interrupt
 EICRB = EICRB | 0x08;  // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20;  // Enable Interrupt INT5 for right position encoder
 sei();                 // Enables the global interrupt
}

///ISR for right position encoder
ISR(INT5_vect)
{
 ShaftCountRight++;  //increment right shaft position count
}


///ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


/**Precondition: "Direction" takes the following values for corresponding motion
             *
             * 0x06 - Forward
             *
             * 0x09 - Backward
             *
             * 0x05 - Left
             *
             * 0x0A - Right
             *
             * 0x00 - Stop
             *
*Function used for setting motor's direction */
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


/**Precondition : "Degrees" specifies the angle in degrees through which the bot is to be rotated
*
*Function to rotate the bot by a specified angle */
void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.5;             // division by resolution to get shaft count
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

/**Precondition : "DistanceInMM" gives the distance to be travelled by the bot in mm
*
*Function used for moving robot forward by specified distance and updating the coordinates of the bot in the arena */
void linear_distance_with_update_of_coordinates(unsigned int DistanceInMM)
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

	/*
	Updates the value of "y_coordinate_of_bot" and "x_coordinate_of_bot" according to the orientation.
	ie updates the displacement of the bot	*/

	int mod = global_orientation_of_bot % 4;
	switch(mod)
	{
		case 0 :	y_coordinate_of_bot = y_coordinate_of_bot + DistanceInMM; //in this case the bot is facing forward
					break;
		case 1 :    x_coordinate_of_bot = x_coordinate_of_bot +  DistanceInMM; //in this case bot is facing rightward
					break;
		case 2 :    y_coordinate_of_bot = y_coordinate_of_bot - DistanceInMM; //in this case bot is facing backward
					break;
		case 3 :    x_coordinate_of_bot = x_coordinate_of_bot - DistanceInMM; //in this case bot is facing leftward
					break;
	}

	motion_set(0x00);
	_delay_ms(10);

}


/**Precondition : "DistanceInMM" gives the distance to be travelled by the bot in mm
*
*Function used for moving robot forward by specified distance without uppdating the coordinates of the bot in the arena */
void linear_distance_without_update_of_coordinates(unsigned int DistanceInMM)
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


///Turns left through 90 degrees as well as updates the global orientation.
void turn_left_with_update_of_coordinates(void)
{
	motion_set(0x05);
	angle_rotate(90);
	global_orientation_of_bot--;
	motion_set(0x00);
}

///Turns right through 90 degrees as well as updates the global orientation.
void turn_right_with_update_of_coordinates(void)
{
	motion_set(0x0A);
	angle_rotate(90);
	global_orientation_of_bot++;
	motion_set(0x00);
}

///Turns right through 88 degrees as well as updates the global orientation.
void turn_right1(void)
{
	motion_set(0x0A);
	angle_rotate(88);
	global_orientation_of_bot++;
	motion_set(0x00);
}

///Turns left through 95 degrees as well as updates the global orientation.
void turn_left1(void)
{
	motion_set(0x05);
	angle_rotate(95);
	global_orientation_of_bot--;
	motion_set(0x00);
}

/**Precondition : "Ch" is the channel number
*
*This Function accepts the Channel Number and returns the corresponding Analog Value */
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


/**Precondition : "adc_reading" is the analog value of the sharp sensor
*
*This Function calculates the actual distance in millimeters(mm) from the input analog value of Sharp Sensor. */
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


/**Precondition : This function is called when the bot is moving towards the detected object
*
 *              but it turns out that it is not the required object due to ineffieciency in image processing
 *
*This function takes the bot back to its initial position from which the object was first detected.
*/
void retrace_back()
{
    if(net_angle_rotated_after_detecting_ball >= 0)
    motion_set(0x05);

    else
    {
        motion_set(0x0A);
        net_angle_rotated_after_detecting_ball = net_angle_rotated_after_detecting_ball*(-1);
    }

    angle_rotate(net_angle_rotated_after_detecting_ball);
	_delay_ms(2000);
	motion_set(0x05);

    double y = (y_coordinate_of_bot - y_coordinate_of_bot_at_which_ball_was_detected);
	double x = (x_coordinate_of_bot - y_coordinate_of_bot_at_which_ball_was_detected);
    double z = atan2( y , x );
	double degrees = ((z / 3.142 + 0.5)*180);
	angle_rotate( degrees );

    linear_distance_without_update_of_coordinates( sqrt((x*x)+(y*y)) );
	motion_set(0x0A);
    angle_rotate(degrees);

}


/**Precondition : This function is called when the object is detected
*
*The function makes the bot move towards the detected object  */
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
  reqd_angle = distance_of_the_centre_of_ball_from_centre_of_image * proportionality_factor;
  rotate_to_centre_of_the_ball_in_the_image(reqd_angle);
  linear_distance_without_update_of_coordinates(200);
  update_coordinates(reqd_angle);

  send_data(0x00);
  while(!flag_goes_into_interrupt);
  if(data_received_by_bot_from_laptop==0x00)
    { retrace_back();
	  y_coordinate_of_bot=y_coordinate_of_bot_at_which_ball_was_detected;
	  x_coordinate_of_bot=y_coordinate_of_bot_at_which_ball_was_detected;
	  return;
     }

  	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);

  }
  motion_set(0x00);
  while(1);
}

/**ISR which is trigerred when data is returned to the received buffer
*
*The data received from the laptop is distinguised as follows:
*
*    0x03 - Confirmation that the sent byte has been received by the laptop
*
*    0x80 - To relay to the laptop that the sent byte has been received by the bot
*
*    First bit is 0 - object not found
*
*    First bit is 1 - object found
*
*    If first bit is 1 ,
*
*     second bit is 0 - Centre of ball is towards the right of the centre of the image
*
*     second bit is 1 - Centre of ball is towards the left of the centre of the image   */
SIGNAL(SIG_USART0_RECV)
{
  distance_of_the_centre_of_ball_from_centre_of_image = 0;
  data_received_by_bot_from_laptop=UDR0;
  if(data_received_by_bot_from_laptop!=0x03)
   { flag_goes_into_interrupt=1;
     _delay_ms(10);
     UDR0 = 0x80;
     _delay_ms(10);
   }
  int first_bit = (data_received_by_bot_from_laptop & 0x80)/128; //Check first bit to see if ball was found

  if(first_bit == 1)
  {
 	int second_bit = (data_received_by_bot_from_laptop & 0x40)/64;
	distance_of_the_centre_of_ball_from_centre_of_image = data_received_by_bot_from_laptop & 0x3F;
	if(second_bit == 1)
	{
		distance_of_the_centre_of_ball_from_centre_of_image *= (-1);
	}
  }

}

/**Precondition : "send" is the data to be sent to the laptop
*
*Sends 0x00 which tells the laptop to click an image and process it  */
void send_data(unsigned char send)
{
	while(data_received_by_bot_from_laptop != 0x03)
	{
		UDR0 = send;
	}
}

/**Precondition : "angle_rotated_in_each_turn" gives the angle through which the bot rotates in each turn
*
*This function scans 360 degrees looking for the specified object */
void rotate_360_scanning_for_ball (int angle_rotated_in_each_turn)
 {	turn_from_global_orientation_at_which_ball_was_detected=0;
    int first_bit;
	 for (int i = 0 ; i < 360 / angle_rotated_in_each_turn ; i++)
		 {
		  flag_goes_into_interrupt=0;
		  motion_set(0x0A);
		  angle_rotate(angle_rotated_in_each_turn);
		  motion_set(0x00);
		  send_data(0x00);

		  while(!flag_goes_into_interrupt); //Will wait until data_received_by_bot_from_laptop is received

		  //Goes into interrupt
		  flag_goes_into_interrupt = 0;
		  first_bit = (data_received_by_bot_from_laptop & 0x80)/128;
		  if(first_bit == 1)
		  {
			 	net_angle_rotated_after_detecting_ball = (((global_orientation_of_bot % 4)*90 + (turn_from_global_orientation_at_which_ball_was_detected*angle_rotated_in_each_turn)) % 360 );
				y_coordinate_of_bot_at_which_ball_was_detected = y_coordinate_of_bot;
				y_coordinate_of_bot_at_which_ball_was_detected = x_coordinate_of_bot;
				move_towards_ball();
				turn_from_global_orientation_at_which_ball_was_detected++;
				continue;
		  }
		  while(data_received_by_bot_from_laptop != 0x00);

		  _delay_ms(500);
		  turn_from_global_orientation_at_which_ball_was_detected++;

		 }
 }


/**Precondition : "check_value" takes the minimum distance of the bot from the obstacle for
*                the obstacle to be detected.
*
*This function returns 1 when the distance of the obstacle from the bot is less than "check_value"
*and returns 0 otherwise.  */
int check_obstacles(unsigned int check_value)
{
	unsigned int value;
	unsigned char sharp;
	sharp = ADC_Conversion(11);
	value = Sharp_GP2D12_estimation(sharp);
	if(value<check_value)
	 {  motion_set(0x00);
	    _delay_ms(1000);
	    return 1;
     }
	return 0;
}

/**Precondition : "initial" takes the value of the initial orientation of the bot before the
*                obstacle was detected.
*
*This function restores the orientation of the bot after the obstacle is overcome. */
void restore_orientation_after_overcoming_obstacle(int initial)
{
	int curr_orientatn = global_orientation_of_bot;
	for(int i = 0; i < (curr_orientatn - initial) % 4; i++)
	{
		linear_distance_with_update_of_coordinates(300);
		_delay_ms(500);
		turn_left_with_update_of_coordinates();
		_delay_ms(500);
	}
}


/**Precondition : This function is called when an obstacle is detected.
*
*This function overcomes the obstacles in the way of the bot and then continues its normal motion */
void overcome_obstacle(void)
{
	int init_orientatn = global_orientation_of_bot;
	_delay_ms(200);
	rotate_360_scanning_for_ball(90);

	do
	{

		while(check_obstacles(200)==1)
		{
			turn_right_with_update_of_coordinates();
			motion_set(0x00);
			_delay_ms(500);
		}

		linear_distance_with_update_of_coordinates(200);
		motion_set(0x00);
		_delay_ms(700);

		turn_left_with_update_of_coordinates();
		motion_set(0x00);
		_delay_ms(500);
	}
	while (check_obstacles(200)==1);

	_delay_ms(500);
	turn_right_with_update_of_coordinates();
	_delay_ms(500);
    linear_distance_with_update_of_coordinates(50);
    turn_left_with_update_of_coordinates();
    _delay_ms(1000);

	int k = (global_orientation_of_bot - init_orientatn)%4;
	restore_orientation_after_overcoming_obstacle(init_orientatn);

	if(k>=2)
	 rotate_360_scanning_for_ball(90);

}

/**Precondition : "distance" specifies the total distance to be moved
*
*This function moves the bot forward by a specific distance, avoiding all obstacles. */
void move_forward_by_specific_distance_avoiding_obstacles(unsigned int distance)
{
	int number_of_iters = distance/60;
	for(int i  = 0; i < number_of_iters; i++)
	{
		if(check_obstacles(200))
		{
			overcome_obstacle();

		}

		linear_distance_with_update_of_coordinates(60);
	}
}


/**Precondition : "angle" specifies the angle rotated by the bot towards the centre of the detected ball
*
*This function updates the coordinates of the bot during its motion towards the ball  */
void update_coordinates (int angle)
{

	net_angle_rotated_after_detecting_ball = net_angle_rotated_after_detecting_ball + angle;  //Right side positive
    y_coordinate_of_bot = y_coordinate_of_bot + 200*cos(net_angle_rotated_after_detecting_ball*3.14/180);
	x_coordinate_of_bot = x_coordinate_of_bot + 200*sin(net_angle_rotated_after_detecting_ball*3.14/180);


}


/**Precondition : "angle" specifies the angle rotated by the bot towards the centre of the detected ball
*
*This function rotates the bot such that it faces the centre of the detected ball in the image  */
void rotate_to_centre_of_the_ball_in_the_image( int angle )
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

///This function initializes all ports, adc, UART ports and the left and right position encoders
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

///Main function
int main()
{
  init_devices();

  unsigned int value;
  unsigned char sharp;

  unsigned int angle_rotated_in_each_turn = 90;
  unsigned int arena_breadth= 930 , arena_length= 930 ;
  unsigned int distance_moved_forward_in_one_go = arena_breadth/4, distance_moved_sideways_in_one_go = arena_length/3;

  unsigned int left_or_right = 0;

  while(1)
  {
  	 while((y_coordinate_of_bot + distance_moved_forward_in_one_go <= arena_breadth && global_orientation_of_bot ==0) || (y_coordinate_of_bot >= distance_moved_forward_in_one_go && global_orientation_of_bot ==2 ))
	  {
		  rotate_360_scanning_for_ball(angle_rotated_in_each_turn);
		  move_forward_by_specific_distance_avoiding_obstacles(distance_moved_forward_in_one_go);
		  motion_set(0x00);
		  _delay_ms(1000);
	  }

	  if( (x_coordinate_of_bot + distance_moved_sideways_in_one_go) >= arena_length )
      {
          motion_set(0x00);
          while(1);
      }

	  if(left_or_right == 0)
	  {
		  turn_right1();
		  _delay_ms(1000);
		  move_forward_by_specific_distance_avoiding_obstacles(distance_moved_sideways_in_one_go);
		  motion_set(0x00);
		  _delay_ms(1000);
		  turn_right_with_update_of_coordinates();
		  _delay_ms(1000);
		  left_or_right = 1;

	  }
	  else
	  {
		  turn_left_with_update_of_coordinates();
		  _delay_ms(1000);
		  move_forward_by_specific_distance_avoiding_obstacles(distance_moved_sideways_in_one_go);
		  motion_set(0x00);
		  _delay_ms(1000);
		  turn_left_with_update_of_coordinates();
		  _delay_ms(1000);
		  left_or_right = 0;
	  }


  }
}
