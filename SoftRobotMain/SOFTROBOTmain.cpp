

/* KNOWN INTERRUPTS **

ISR(TCC0_OVF_vect)
	library: SOFTROBOTmain.cpp
	frequency: 1000 Hz
	purpose: milliseconds counter

ISR(TCE0_OVF_vect)
	library: SRpressure_control.c
	frequency: 125 Hz
	purpose: service actuators, measure central pressure
	
ISR(RTC_OVF_vect)	**AVAILABLE**
	library: software_clock.c	(commented out) 

ISR(PORTB_INT0_vect)
	library: SRaccelerometer.c
	purpose: interrupt function for PCB mounted accelerometer

ISR(PORTC_INT0_vect)
	library: SRbuttons.c
	purpose: interrupt function for button presses

*/


#include "SOFTROBOTmain.h"

#include "xgrid_maintenance.h"

#include <stdio.h>
#include <stdlib.h> // for srand

// USART
#define USART_TX_BUF_SIZE 64
#define USART_RX_BUF_SIZE 64
char usart_txbuf[USART_TX_BUF_SIZE];
char usart_rxbuf[USART_RX_BUF_SIZE];
CREATE_USART(usart, UART_DEVICE_PORT);
FILE usart_stream;

#define NODE_TX_BUF_SIZE 32
#define NODE_RX_BUF_SIZE 64
char usart_n0_txbuf[NODE_TX_BUF_SIZE];
char usart_n0_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n0, USART_N0_DEVICE_PORT);
char usart_n1_txbuf[NODE_TX_BUF_SIZE];
char usart_n1_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n1, USART_N1_DEVICE_PORT);
char usart_n2_txbuf[NODE_TX_BUF_SIZE];
char usart_n2_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n2, USART_N2_DEVICE_PORT);
char usart_n3_txbuf[NODE_TX_BUF_SIZE];
char usart_n3_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n3, USART_N3_DEVICE_PORT);
char usart_n4_txbuf[NODE_TX_BUF_SIZE];
char usart_n4_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n4, USART_N4_DEVICE_PORT);
char usart_n5_txbuf[NODE_TX_BUF_SIZE];
char usart_n5_rxbuf[NODE_RX_BUF_SIZE];
CREATE_USART(usart_n5, USART_N5_DEVICE_PORT);


Xgrid xgrid;


void init(void);

// note: MAIN_LOOP_TIMEOUT_ms defined in software_clock.h, governs the time it takes for the 'main' loop


//uint8_t exp_timer_number;	// for catching the expired timer number, [1,... ]





void rx_pkt(Xgrid::Packet *pkt);

void swarm_communication_send();

void swarm_command_handler();

void init_PLL(void);

void init(void);

void calculate_swarm_id();


int main(void)
{
    stdout = &usart_stream;	// connects printf to usart (serial) out
	
	uint32_t main_loop_settime = 0; // for debugging overtime fails (this variable used to be xgrid's 'j')
        
	uint8_t input_char = 0;
	uint8_t input_int;
	uint8_t button_check = 0;
	uint8_t touch_diff;
	uint8_t untouch_diff;

	//uint8_t CDT_bits = 1;
	float CDT_time_us;
	//uint8_t CDC_bits = 1;
	uint8_t CDC_current_uA;

	uint8_t test_iterator = 0;

	uint8_t act_choice = 1;

	uint8_t loop_count = 0;

	// initial delay, allow devices to power up
    _delay_ms(50);
        
    init();

	// Init xgrid packet handler
    xgrid.rx_pkt = &rx_pkt;

	_delay_ms(50);

    fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);
	_delay_ms(10);	
	
	// DATA RECORD INIT //

	/* setup for data recording (automatic select)
	for(uint8_t act_num = 1; act_num < 7; act_num++)
	{
		if(is_ADC_pressure_calibrated(act_num))
			data_record_channel_on(act_num);
	}*/
	
	// setup for data recording (manual select)
	data_record_channel_on(WALK_ACT_1);
	data_record_channel_on(WALK_ACT_2);
	data_record_channel_on(WALK_ACT_3);
	data_record_channel_on(WALK_ACT_4);
	
	// set up data record for cap touch (automatic select)
	for(uint8_t pad_num = 1; pad_num <= 8; pad_num++)
	{
		data_record_cap_pad_on(pad_num);
	}	

	// set up data record for cap touch (manual select)
	//data_record_cap_pad_on(7);
	//data_record_cap_pad_on(8);


	// ------------------------------------------------------------------------------- //

    while (1)	// MAIN LOOP ==>
    {        	
		main_loop_settime = get_milliseconds(); // for debugging
		set_main_countdown_timer(MAIN_LOOP_TIMEOUT_ms); // from here, you have MAIN_LOOP_TIMEOUT_ms ms to reach the bottom of this loop

	// CONTROLLER (input phase)	/////////////////////////////////////////////////// (BEGIN)
		
		// get input from user (serial - terminal):
		if (usart.available())
			input_char = usart.get();
					
		// get input from user (peripheral buttons):
		button_check = get_button_press_event();
		
		// get input from devices:
		//touch_diff = get_touch_status_change();
		//untouch_diff = get_untouch_status_change();
		get_touch_status_change(&touch_diff, &untouch_diff);

	// CONTROLLER (input phase)	/////////////////////////////////////////////////// (END)
	//
	//
	// MODEL (state transitions - process input)  ///////////////////////////////// (BEGIN)

		// MODEL (process input)
		
		// handle input from user (serial - terminal):
		if(input_char)
		{
			printf(">%c\r\n", input_char);	// trying DEBUG unknown problem

			if (input_char == 0x1b)	// ESC key
				xboot_reset();

			else if ((input_char >= '1')&&(input_char <= '6'))
			{
				input_int =  input_char - '1' + 1;	// convert char numeral to integer value
				
				if(input_int == 1)	act_choice = WALK_ACT_1;
				if(input_int == 2)	act_choice = WALK_ACT_2;
				if(input_int == 3)	act_choice = WALK_ACT_3;
				if(input_int == 4)	act_choice = WALK_ACT_4;
				
				fprintf_P(&usart_stream, PSTR("actuator: %u\r\n"), act_choice);
				
				float act_pressure = get_actuator_pressure_PSI(act_choice);
				
				fprintf_P(&usart_stream, PSTR("@ %.4f PSI\r\n"), act_pressure);
				_delay_ms(10);
			}

			else if (input_char == 'a')		// calibrate to atmosphere
			{
				fprintf_P(&usart_stream, PSTR("\r\nCALIBRATE TO ATMOSPHERE\r\n"));
				_delay_ms(5);

				request_overtime_permission();

				// open all valves so that all actuators and reservoir may equilibrate to atmospheric pressure
				valve_thru_open_ALL();
				
				float main_pressure;
				
				do{
					main_pressure = get_digital_pressure();
					printf("MP: %.4f\r\n", (double)main_pressure);
									
					_delay_ms(500);
					
				}while(main_pressure > 0);

				valve_hold_ALL();
			}

			else if (input_char == 'A')		// auto calibration routine
			{
				fprintf_P(&usart_stream, PSTR("known actuators: "));
				for(uint8_t act_num = 1; act_num < 7; act_num++)
				{
					if(is_ADC_pressure_calibrated(act_num))
					fprintf_P(&usart_stream, PSTR("%i "), act_num);
				}
				
				fprintf_P(&usart_stream, PSTR("\r\n"));
				
				fprintf_P(&usart_stream, PSTR("build new calibration? "));
				if(type_yes_or_no_blocking(&usart))
				build_entire_new_calibration();
			}
			
			else if (input_char == 'b')		// barometer
			{
				fprintf_P(&usart_stream, PSTR("-BPS-\r\n"));
				
				float BPS_temp;
				BPS_temp = get_BPS_temperatue_C();
				fprintf_P(&usart_stream, PSTR("T: %.2f øC\r\n"), BPS_temp);
				
				_delay_ms(5);

				
				float BPS_pressure;
				BPS_pressure = get_BPS_pressure_mbar();
				fprintf_P(&usart_stream, PSTR("P: %.2f mbar\r\n"), BPS_pressure);

				_delay_ms(5);

			}

			else if (input_char == 'B')
			{
				BPS_reset_sequence();
			}

			else if (input_char == 'c')
			{
				printf("current:  ");	print_cap_values_allpads(CAP_VALUE_TYPE_ADC, 0);
				printf("baseline: ");	print_cap_values_allpads(CAP_VALUE_TYPE_ADC, 1);
			}
			
			else if (input_char == 'C')
			{
				print_electrode_values();
			}

			else if (input_char == 'd')
			{
				// DEBUG BLOCK **
				
				

			
			}

			else if (input_char == 'D')
				load_default_ADCtoPSI_calibration();

			else if (input_char == 'e')
			{
				edit_record_parameters();
			}
			
			else if (input_char == 'E')
			{
				fprintf_P(&usart_stream, PSTR("clear all calibration? "));
				if(type_yes_or_no_blocking(&usart))
				clear_all_ADCtoPSI_calibration();
			}
			else if (input_char == 'f')
				valve_fill(act_choice);

			else if (input_char == 'F')
				valve_fill_ALL();
			
			else if (input_char == 'g')
			{
				float gyr[3];

				get_9axis_gyro_deg(gyr,1);

				printf("GYR1: [%f, ", gyr[0]);	_delay_ms(5);
				printf("%f, ", gyr[1]);		_delay_ms(5);
				printf("%f]\r\n", gyr[2]);	_delay_ms(5);
				
				get_9axis_gyro_deg(gyr,2);

				printf("GYR2: [%f, ", gyr[0]);	_delay_ms(5);
				printf("%f, ", gyr[1]);		_delay_ms(5);
				printf("%f]\r\n", gyr[2]);	_delay_ms(5);

			}
			
			else if (input_char == 'G')
			{
				register_read_print_9axis_gyro(1);

				/*
				float goal_pressure;
				
				request_overtime_permission();
				fprintf_P(&usart_stream, PSTR("act %i goto: "), act_choice);
				goal_pressure = type_a_number_blocking(&usart);
				actuator_set_and_forget(act_choice, goal_pressure);
				*/
			}

			else if (input_char == 'h')
				valve_hold(act_choice);

			else if (input_char == 'H')
				valve_hold_ALL();

			else if (input_char == 'i')
				check_all_of_I2C();

			else if (input_char == 'j')
				calibrate_pressure_sensor_debug(act_choice);

			else if (input_char == 'k')
				touch_soft_reset();
			
			else if (input_char == 'l')
			{
				printf("load PROM\r\n");
				BPS_read_calibration_PROM();
			}

			else if (input_char == 'm')
			{
				int16_t mag_vals[3];
				get_9axis_mag_bytes(mag_vals, 1);

				printf("MAG1: [%i, ", mag_vals[0]);	_delay_ms(5);
				printf("%i, ", mag_vals[1]);		_delay_ms(5);
				printf("%i]\r\n", mag_vals[2]);		_delay_ms(5);

				get_9axis_mag_bytes(mag_vals, 2);

				printf("MAG2: [%i, ", mag_vals[0]);	_delay_ms(5);
				printf("%i, ", mag_vals[1]);		_delay_ms(5);
				printf("%i]\r\n", mag_vals[2]);		_delay_ms(5);
			}

			else if (input_char == 'M')
			{
				//register_read_print_9axis_mag(1);
			
				float mag[3];

				get_9axis_mag_uT(mag, 1);
				_delay_ms(5);
				/*
				printf("fMAG1: [%.1f, ", mag[0]);	_delay_ms(5);
				printf("%.1f, ", mag[1]);		_delay_ms(5);
				printf("%.1f]\r\n", mag[2]);	_delay_ms(5);
				*/

				get_9axis_mag_uT(mag, 2);
				_delay_ms(5);
				/*
				printf("fMAG2: [%.1f, ", mag[0]);	_delay_ms(5);
				printf("%.1f, ", mag[1]);		_delay_ms(5);
				printf("%.1f]\r\n", mag[2]);	_delay_ms(5);
				*/
			}

			else if (input_char == 'n')
			{
				actuator_set_and_forget(GRASP_ACT_1, GRASP_PRESSURE_psi);
				actuator_set_and_forget(GRASP_ACT_2, GRASP_PRESSURE_psi);
			}
			
			else if (input_char == 'N')
			{
				actuator_set_and_forget(GRASP_ACT_1, GRASP_RELAX_psi);
				actuator_set_and_forget(GRASP_ACT_2, GRASP_RELAX_psi);
			}
			
			else if (input_char == 'o')
				valve_thru_open(act_choice);
			
			else if (input_char == 'O')
				valve_thru_open_ALL();

			else if (input_char == 'p')
			{
				toggle_pump_mode();
			}
			
			else if (input_char == 'P')
			{
				float goal_pressure;
				
				request_overtime_permission();
				
				fprintf_P(&usart_stream, PSTR("m goto: "), act_choice);
				
				goal_pressure = type_a_number_blocking(&usart);
				
				actuator_set_and_forget(GRASP_ACT_1, goal_pressure);
				actuator_set_and_forget(GRASP_ACT_2, goal_pressure);
			}
			
			else if (input_char == 'r')
			{
				record_data(RECORD_MODE_FULL_HEADER);
			}
			
			else if (input_char == 'R')
			{
				record_data(RECORD_MODE_SIMPLE);
			}

			else if (input_char == 's')
			{
				snapshot_data_record();
			}

			else if (input_char == 't')			// touch settings
			{

				CDT_time_us = get_MPR121_CDT_us();

				printf("CDT: %.2f us\r\n", CDT_time_us);
				_delay_ms(10);

				CDC_current_uA = get_MPR121_CDC_uA();

				printf("CDC: %u uA\r\n", CDC_current_uA);
				_delay_ms(10);

				//MPR121_debug_printout();
			}

			else if (input_char == 'u')
			{
				enable_MPR121_individual_electrodes_bits(4);
				//save_baseline_touch_values();
			}

			else if (input_char == 'U')
			{
				touch_soft_reset();
			}

			else if (input_char == 'v')
			{
				valve_vent(act_choice);
			}

			else if (input_char == 'V')
			{
				valve_vent_ALL();
			}

			else if (input_char == 'w')
			{
				//request_overtime_permission();
				sweep_MPR121_AFE_calibration();
			}

			else if (input_char == 'x')
			{
				float acc[3];
				get_9axis_accel_g(acc, 0, 1);

				printf("ACC: [%f, ", acc[0]);	_delay_ms(5);
				printf("%f, ", acc[1]);			_delay_ms(5);
				printf("%f]\r\n", acc[2]);		_delay_ms(5);
			}

			else if (input_char == 'X')
			{
				register_read_print_9axis_accel(1);
			}

			else if (input_char == 'y')
			{
				begin_motion(YOUNG_MOTION);
			}

			else if((input_char == '!')||(input_char == '@')||(input_char == '#')||(input_char == '$')||
			(input_char == '%')||(input_char == '^')||(input_char == '&')||(input_char == '*')||
			(input_char == '(')||(input_char == ')'))
			{
				switch(input_char)
				{
					case ')': actuator_set_and_forget(act_choice, 0); break;
					case '!': actuator_set_and_forget(act_choice, 1); break;
					case '@': actuator_set_and_forget(act_choice, 2); break;
					case '#': actuator_set_and_forget(act_choice, 3); break;
					case '$': actuator_set_and_forget(act_choice, 4); break;
					case '%': actuator_set_and_forget(act_choice, 5); break;
					case '^': actuator_set_and_forget(act_choice, 6); break;
					case '&': actuator_set_and_forget(act_choice, 7); break;
					case '*': actuator_set_and_forget(act_choice, 8); break;
					case '(': actuator_set_and_forget(act_choice, 9); break;
					default: break;
				}
			}
			
			else
			{
				printf("x--%c [%u]\r\n", input_char, input_char);	// trying DEBUG unknown problem
				_delay_ms(1000);
				fprintf_P(&usart_stream, PSTR("no command for '%c'\r\n"), input_char);
			}

			input_char = 0x00;	// clear the most recent computer input
		}

		// handle input from user (peripheral buttons):
		if(button_check != BUTTON_EVENT_NONE)
		{
			switch(button_check)
			{
				case BUTTON_EVENT_A_DOWN:
					valve_vent_ALL();
					break;
				case BUTTON_EVENT_A_UP:
					valve_hold_ALL();
					break;
				case BUTTON_EVENT_B_DOWN:
					valve_fill_ALL();
					set_pump_speed_manual(PUMP_SPEED_HI);
					break;
				case BUTTON_EVENT_B_UP:
					valve_hold_ALL();
					set_pump_speed_manual(0);
					break;
			}
		}

		touch_controller(touch_diff, untouch_diff);
		//if(!is_data_recording())	touch_controller(touch_diff, untouch_diff);	
		

	// MODEL (state transitions - process input)  ///////////////////////////////// (END)
	//
	// MODEL (automatic state transitions)  /////////////////////////////////////// (BEGIN)

		motion_handler();

	// MODEL (automatic state transitions)  /////////////////////////////////////// (END)
	//
	//
	// VIEW (output phase)	/////////////////////////////////////////////////////// (BEGIN)
		
		data_service();	// record data to terminal, if applicable

		toggle_LED(2);
        
	// VIEW (output phase)	/////////////////////////////////////////////////////// (END)
	//
	//
	// INFREQUENT FUNCTION CALLS ////////////////////////////////////////////////// (BEGIN)

		loop_count++;
		
		// DECA-LOOP	(do once every 10 loops)
		if(loop_count % 10 == 0)
		{
			
			
		}

		// HECTA-LOOP	(do once every 100 loops)
		if(loop_count == 100)	// 0.25 Hz
		{
			//printf("+");
			loop_count = 0;
		}

	// INFREQUENT FUNCTION CALLS ////////////////////////////////////////////////// (END)
	//
	//
	// TEST CODE ONLY! //////////////////////////////////////////////////////////// (BEGIN)

		//get_9axis_accel(M_accel);	// test position

	// TEST CODE ONLY! //////////////////////////////////////////////////////////// (END)
	//
	//
	//	////// <== MAIN LOOP [END]	>>>	>>>

		if(main_countdown_timer_expired())
		{
			// if TRUE, then we took too long to get here, halt program
			_delay_ms(10);
			fprintf_P(&usart_stream, PSTR("OVERTIME: %i\r\n"), compute_tdiff_ms(main_loop_settime, get_milliseconds()) );	// debug
		}

		while(!main_countdown_timer_expired())
		{/* killing time here */};
    }  
}

/*------------------------- OLD COMMANDS --------------------------------------


------------------------- OLD COMMANDS --------------------------------------*/

// Init everything
void init(void)
{
	// clock
	OSC.CTRL |= OSC_RC32MEN_bm; // turn on 32 MHz oscillator
    while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }; // wait for it to start
    CCP = CCP_IOREG_gc;
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch osc
    DFLLRC32M.CTRL = DFLL_ENABLE_bm; // turn on DFLL
       
	//init_PLL();
	    
    // disable JTAG
    CCP = CCP_IOREG_gc;
    MCU.MCUCR = 1;
        
	//init_uarts(); // usart manager

	// UARTs
    usart.set_tx_buffer(usart_txbuf, USART_TX_BUF_SIZE);
    usart.set_rx_buffer(usart_rxbuf, USART_RX_BUF_SIZE);
    usart.begin(UART_BAUD_RATE);
    usart.setup_stream(&usart_stream);
        
    usart_n0.set_tx_buffer(usart_n0_txbuf, NODE_TX_BUF_SIZE);
    usart_n0.set_rx_buffer(usart_n0_rxbuf, NODE_RX_BUF_SIZE);
    usart_n0.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n0);
        
	usart_n1.set_tx_buffer(usart_n1_txbuf, NODE_TX_BUF_SIZE);
    usart_n1.set_rx_buffer(usart_n1_rxbuf, NODE_RX_BUF_SIZE);
    usart_n1.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n1);
        
	usart_n2.set_tx_buffer(usart_n2_txbuf, NODE_TX_BUF_SIZE);
    usart_n2.set_rx_buffer(usart_n2_rxbuf, NODE_RX_BUF_SIZE);
    usart_n2.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n2);
    
	/*    
	usart_n3.set_tx_buffer(usart_n3_txbuf, NODE_TX_BUF_SIZE);
    usart_n3.set_rx_buffer(usart_n3_rxbuf, NODE_RX_BUF_SIZE);
    usart_n3.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n3);
        
	usart_n4.set_tx_buffer(usart_n4_txbuf, NODE_TX_BUF_SIZE);
    usart_n4.set_rx_buffer(usart_n4_rxbuf, NODE_RX_BUF_SIZE);
    usart_n4.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n4);
        
	usart_n5.set_tx_buffer(usart_n5_txbuf, NODE_TX_BUF_SIZE);
    usart_n5.set_rx_buffer(usart_n5_rxbuf, NODE_RX_BUF_SIZE);
    usart_n5.begin(NODE_BAUD_RATE);
    xgrid.add_node(&usart_n5);
	*/

    // TCC0
    TCC0.CTRLA = TC_CLKSEL_DIV256_gc;
    TCC0.CTRLB = 0;
    TCC0.CTRLC = 0;
    TCC0.CTRLD = 0;
    TCC0.CTRLE = 0;
    TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc;		// this is an overflow interrupt (ref: doc8331.pdf, pg 181)
    //TCC0.INTCTRLA = TC_OVFINTLVL_HI_gc;
	TCC0.INTCTRLB = 0;						// this is for clock compare interrupts (there are 4 avail)
    TCC0.CNT = 0;
    TCC0.PER = 125;
        
    // Interrupts
    //PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;		// ref: doc8331.pdf, pg 138
        
    sei();

	// INITIALIZATION PRINT [BEGIN] ==>											---------------
	 
	fprintf_P(&usart_stream, PSTR("\r\n= SoftRobot V1 =\r\n"));
	_delay_ms(10);

	// Swarm ID
	calculate_swarm_id();
	_delay_ms(10);

	fprintf_P(&usart_stream, PSTR("chip ID: %02x\r\n"), get_swarm_id());
	// Use swarm ID to seed random numbers
	srand(get_swarm_id());


	/* INITIALIZE EXTERNAL COMPONENTS (ANALOG) */

	// Initialize the status LEDs:
	fprintf_P(&usart_stream, PSTR("STATUS LED INIT\r\n"));
	_delay_ms(10);
	status_LED_init();

	// Initialize the pump:
	fprintf_P(&usart_stream, PSTR("PUMP INIT\r\n"));
	_delay_ms(10);
	pump_init();		
	
	// Initialize the valve(s):
	fprintf_P(&usart_stream, PSTR("VALVES INIT\r\n"));
	_delay_ms(10);
	valves_init();
	
	// Initialize the analog pressure sensor(s):
	fprintf_P(&usart_stream, PSTR("ANALOG PRESSURE INIT\r\n"));
	_delay_ms(10);
	APS_init();

	// Initialize the strain sensor:
	fprintf_P(&usart_stream, PSTR("STRAIN SENSOR INIT\r\n"));
	_delay_ms(10);
	SS_init();
	
	/*
	// Initialize the external buttons, if applicable (on NODE 1 serial port)
	fprintf_P(&usart_stream, PSTR("BUTTONS INIT\r\n"));
	_delay_ms(10);
	init_buttons();
	*/
	
	/* INITIALIZE I2C COMPONENTS */

	uint8_t IC_ready;

	// Initialize the I2C bus:
	fprintf_P(&usart_stream, PSTR("I2C INIT\t--\r\n"));
	_delay_ms(10);
	init_I2C();		_delay_ms(30);
	
	// Initialize the digital pressure sensor (I2C main device):
	//fprintf_P(&usart_stream, PSTR("DIGITAL PRESSURE INIT\r\n"));
	IC_ready = init_DPS();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("DIGITAL PRESSURE INIT: %u\r\n"), IC_ready);
	_delay_ms(30);
	
	// Initialize the barometric pressure sensor (I2C peripheral device):
	IC_ready = init_BPS();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("BPS INIT: %u\r\n"), IC_ready);
	_delay_ms(30);

	// Initialize the capacitive touch sensor (I2C peripheral device):
	IC_ready = init_touch();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("CAP TOUCH INIT: %u\r\n"), IC_ready);
	_delay_ms(30);
	
	// Initialize the 9-axis accelerometer (I2C peripheral device):
	IC_ready = init_9axis_accel();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("ACCELEROMETER INIT: %u\r\n"), IC_ready);
	_delay_ms(30);

	// Initialize the gyroscope (I2C peripheral device):
	IC_ready = init_9axis_gyro();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("GYROSCOPE INIT: %u\r\n"), IC_ready);
	_delay_ms(30);

	// Initialize the magnetometer (I2C peripheral device):
	IC_ready = init_9axis_mag();
	_delay_ms(10);
	fprintf_P(&usart_stream, PSTR("MAGNETOMETER INIT: %u\r\n"), IC_ready);
	_delay_ms(30);


	/* INITIALIZE CONTROLS */
	fprintf_P(&usart_stream, PSTR("Configuring...\r\n"));
	_delay_ms(30);
	
	// Initialize the pressure control (control loop):
	fprintf_P(&usart_stream, PSTR("PRESSURE CONTROL:"));
	_delay_ms(10);
	init_pressure_control();
	_delay_ms(10);


	/*
	// Initialize the reservoir:
	fprintf_P(&usart_stream, PSTR("RESERVOIR CONTROL INIT\r\n"));
	_delay_ms(10);
	reservoir_init();
	*/

	// Initialize the pressure control (control loop):
	fprintf_P(&usart_stream, PSTR("TOUCH CONTROL:"));
	_delay_ms(10);
	init_touch_control();
	_delay_ms(10);
	


	/* INITIALIZE CALIBRAION VALUES */
	
	// Set Actuator-PCB connections in software
	fprintf_P(&usart_stream, PSTR("\r\nLOAD DEFAULT ACTUATOR CONNECTIONS\r\n"));
	_delay_ms(10);
	load_default_ADCtoPSI_calibration();

	// <== [END] INITIALIZATION PRINT											---------------

	_delay_ms(50);	// final delay
}


void rx_pkt(Xgrid::Packet *pkt)
{
	// optional data fields
	//uint8_t message_source_node = pkt->rx_node;
	//uint8_t* data_ptr = pkt->data;
	
	// if counting messages
	//tally_message(message_source_node);

	fprintf_P(&usart_stream, PSTR("packet type received: %i\n\r"), pkt->type);
}


// Timer tick ISR (1 kHz)
ISR(TCC0_OVF_vect)	
{
	// Timers
	//milliseconds++;
	increment_milliseconds();

	xgrid.process();
}


/*	// ----- UNUSED DEBUG / DEVELOPMENT CODE BLOCKS ----- //

{
	print_actuator_pressure_equation(act_choice);
	fprintf_P(&usart_stream, PSTR("calibrate actuator %u? "), act_choice);
	if(type_yes_or_no_blocking(&usart))
	calibrate_pressure_sensor(act_choice);
}

{
	begin_motion(MOTION_STEP);
}

{
	begin_motion(MOTION_STAND_UP);
}





 */