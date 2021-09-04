// SRPC_com.c

#include "SRPC_com.h"

//extern Usart usart;
//extern Usart* usart_addr = &usart;

//extern Usart usart;

float type_a_number_blocking(Usart* usart)
{
	printf("#");

	float number = 0;

	char number_char = 0;
	uint8_t digit;
	float decimal_place = 0;
	float decimal_part = 0;

	do 
	{					
		number_char = usart->get();
				
		// input is a number and we have not yet seen a decimal point
		if((number_char >= '0')&&(number_char <= '9')&&(decimal_place == 0))
		{
			digit = number_char-'0';
						
			number *= 10;
			number += digit;

			printf("%i", digit);
			//printf("\n%f\n", number);
		}

		// if input is a decimal point
		else if(number_char == '.')
		{
			if(decimal_place == 0)
			{
				decimal_place = 10;
				printf(".");
				//printf("\n%f\n", number);
			}
		}

		else if((number_char >= '0')&&(number_char <= '9')&&(decimal_place > 0))
		{
			digit = number_char-'0';
						
			decimal_part += ((float)digit/decimal_place);
			decimal_place *= 10;

			printf("%i", digit);
		}

		else if(number_char == '\b')	// backspace
		{
			printf("\r\n#");
			
			// start over
			number = 0;
			decimal_place = 0;
			decimal_part = 0;
		}
				
	} while(number_char != 13);	// 13 = return key
			
	printf("\r\n");
	//printf("int %f", number);
				
	number += decimal_part; 	// number now contains the typed in float

	//printf("full %f", number);

	return number;
}


float type_yes_or_no_blocking(Usart* usart)
{
	printf("[y/n]>");

	char response_char;

	uint8_t answer = 2;
	
	do
	{
		response_char = usart->get();
		
		if((response_char == 'y')||(response_char == 'Y'))
		{
			if(answer != 2)
				printf("\b");
			
			answer = 1;
			printf("%c", response_char);
		}
		
		else if((response_char == 'n')||(response_char == 'N'))
		{
			if(answer != 2)
				printf("\b");
			
			answer = 0;
			printf("%c", response_char);
		}

	} while((response_char != 13)||(answer == 2));	// 13 = return key
	
	printf("\r\n");

	return answer;
}