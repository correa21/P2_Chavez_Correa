/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#include "MK66F18.h"
#include "GPIO.h"
#include "Bits.h"

static void (*gpio_A_callback)(void) = 0;
static void (*gpio_B_callback)(void) = 0;
static void (*gpio_C_callback)(void) = 0;
static void (*gpio_D_callback)(void) = 0;
static void (*gpio_E_callback)(void) = 0;

static gpio_interrupt_flags_t g_intr_status_flag = {0};



void GPIO_callback_init(gpio_port_name_t port_name,void (*handler)(void))
{
	switch(port_name)
	{

		case GPIO_A:
			gpio_A_callback = handler;
		break;

		case GPIO_B:
			gpio_B_callback = handler;
		break;

		case GPIO_C:
			gpio_C_callback = handler;
		break;

		case GPIO_D:
			gpio_D_callback = handler;
		break;

		case GPIO_E:
			gpio_E_callback = handler;
		break;

		default:
		break;
	}
}

/** ************ Interrupt Handlers for Port Interrupts*************** */

void PORTA_IRQHandler(void)
{
	if(gpio_A_callback)
	{
		gpio_A_callback();
	}

	GPIO_clear_interrupt(GPIO_A);
}


void PORTB_IRQHandler(void)
{
	if(gpio_B_callback)
	{
		gpio_B_callback();
	}

	GPIO_clear_interrupt(GPIO_B);
}


void PORTC_IRQHandler(void)
{
	if(gpio_C_callback)
	{
		gpio_C_callback();
	}

	GPIO_clear_interrupt(GPIO_C);

}

void PORTD_IRQHandler(void)
{
	if(gpio_D_callback)
	{
		gpio_D_callback();
	}

	GPIO_clear_interrupt(GPIO_D);
}

void PORTE_IRQHandler(void)
{
	if(gpio_E_callback)
	{
		gpio_E_callback();
	}

	GPIO_clear_interrupt(GPIO_E);
}
/** ************ EOF Interrupt Handlers for Port Interrupts*************** */

void GPIO_clear_irq_status(gpio_port_name_t gpio)
{
	switch(gpio)
	{

		case GPIO_A:
			g_intr_status_flag.flag_port_a = FALSE;
			GPIO_clear_interrupt(GPIO_A);
		break;

		case GPIO_B:
			g_intr_status_flag.flag_port_b = FALSE;
			GPIO_clear_interrupt(GPIO_B);
		break;

		case GPIO_C:
			g_intr_status_flag.flag_port_c = FALSE;
			GPIO_clear_interrupt(GPIO_C);
		break;

		case GPIO_D:
			g_intr_status_flag.flag_port_d = FALSE;
			GPIO_clear_interrupt(GPIO_D);
		break;

		case GPIO_E:
			g_intr_status_flag.flag_port_e = FALSE;
			GPIO_clear_interrupt(GPIO_E);
		break;

		default:
		break;
	}

}

uint8_t GPIO_get_irq_status(gpio_port_name_t gpio)
{
	uint8_t status = 0;

	switch(gpio)
	{

		case GPIO_A:
			status = g_intr_status_flag.flag_port_a;
		break;

		case GPIO_B:
			status = g_intr_status_flag.flag_port_b;
		break;

		case GPIO_C:
			status = g_intr_status_flag.flag_port_c;
		break;

		case GPIO_D:
			status = g_intr_status_flag.flag_port_d;
		break;

		case GPIO_E:
			status = g_intr_status_flag.flag_port_e;
		break;

		default:
		break;
	}

	return(status);
}



void GPIO_clear_interrupt(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
		break;

	}// end switch
}

uint8_t GPIO_clock_gating(gpio_port_name_t port_name)
{

	switch (port_name)/** Selecting the GPIO for clock enabling*/
	{

		case GPIO_A: /** GPIO A is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
		break;

		case GPIO_B: /** GPIO B is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
		break;

		case GPIO_C: /** GPIO C is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
		break;

		case GPIO_D: /** GPIO D is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
		break;

		case GPIO_E: /** GPIO E is selected*/
			SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
		break;

		default: /**If doesn't exist the option*/
			return(FALSE);

		}// end switch
	/**Successful configuration*/
	return (TRUE);
}// end function

uint8_t GPIO_pin_control_register(gpio_port_name_t port_name,
								  uint8_t pin,
								  const gpio_pin_control_register_t*  pin_control_register)
{

	switch (port_name)
	{

		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pin_control_register;
		break;

		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pin_control_register;
		break;

		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pin_control_register;
		break;

		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pin_control_register;
		break;

		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin] = *pin_control_register;
		break;

		default:/**If doesn't exist the option*/
			return (FALSE);
		break;

	}
	/**Successful configuration*/
	return (TRUE);
}

void GPIO_write_port(gpio_port_name_t portName, uint32_t data)
{

	switch (portName)
	{

		case GPIO_A: /** GPIO A is selected*/
			GPIOA->PDOR = data;
		break;

		case GPIO_B: /** GPIO B is selected*/
			GPIOB->PDOR = data;
		break;

		case GPIO_C: /** GPIO C is selected*/
			GPIOC->PDOR = data;
		break;

		case GPIO_D: /** GPIO D is selected*/
			GPIOD->PDOR = data;
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDOR = data;
		break;

		default:/**If doesn't exist the option*/
		break;

	}
}

uint32_t GPIO_read_port(gpio_port_name_t portName)
{
	uint32_t port_data = 0;

	switch (portName)
	{

		case GPIO_A: /** GPIO A is selected*/
			port_data = GPIOA->PDIR;
		break;

		case GPIO_B: /** GPIO B is selected*/
			port_data = GPIOB->PDIR;
		break;

		case GPIO_C: /** GPIO C is selected*/
			port_data = GPIOC->PDIR;
		break;

		case GPIO_D: /** GPIO D is selected*/
			port_data = GPIOD->PDIR;
		break;

		case GPIO_E: /** GPIO E is selected*/
			port_data = GPIOE->PDIR;
		break;

		default:/**If doesn't exist the option*/
			port_data = 0;
		break;
	}

	return (port_data);
}

uint8_t GPIO_read_pin(gpio_port_name_t portName, uint8_t pin)
{
	uint32_t portdata = 0;

	portdata = GPIO_read_port(portName);
	portdata = portdata & (1<<pin);

	if (FALSE == portdata)
	{
		return (FALSE);
	}
	else
	{
		return (TRUE);
	}

}

void GPIO_set_pin(gpio_port_name_t portName, uint8_t pin)
{

	switch (portName)
	{
		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PSOR = TRUE << pin;
		break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PSOR = TRUE << pin;
		break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PSOR = TRUE << pin;
		break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PSOR = TRUE << pin;
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PSOR = TRUE << pin;
		break;

		default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_clear_pin(gpio_port_name_t portName, uint8_t pin)
{

	switch (portName)
	{

		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PCOR = TRUE << pin;
		break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PCOR = TRUE << pin;
		break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PCOR = TRUE << pin;
		break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PCOR = TRUE << pin;
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PCOR = TRUE << pin;
		break;

		default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_toogle_pin(gpio_port_name_t portName, uint8_t pin)
{

	switch (portName)
	{

		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PTOR = TRUE << pin;
		break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PTOR = TRUE << pin;
		break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PTOR = TRUE << pin;
		break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PTOR = TRUE << pin;
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PTOR = TRUE << pin;
		break;

		default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_data_direction_port(gpio_port_name_t portName, uint32_t direction)
{

	switch (portName)
	{

		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR = (GPIO_OUTPUT == direction) ? PORT_OUT : PORT_INPUT;
		break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR = (GPIO_OUTPUT == direction) ? PORT_OUT : PORT_INPUT;
		break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR = (GPIO_OUTPUT == direction) ? PORT_OUT : PORT_INPUT;
		break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR = (GPIO_OUTPUT == direction) ? PORT_OUT : PORT_INPUT;
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR = (GPIO_OUTPUT == direction) ? PORT_OUT : PORT_INPUT;
		break;

		default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_data_direction_pin(gpio_port_name_t portName,
							 uint8_t state, uint8_t pin)
{

	switch (portName)
	{

		case GPIO_A:/** GPIO A is selected*/
			GPIOA->PDDR = ((GPIOA->PDDR & (~(1 << pin))) | (state << pin));
		break;

		case GPIO_B:/** GPIO B is selected*/
			GPIOB->PDDR = ((GPIOB->PDDR & (~(1 << pin))) | (state << pin));
		break;

		case GPIO_C:/** GPIO C is selected*/
			GPIOC->PDDR = ((GPIOC->PDDR & (~(1 << pin))) | (state << pin));
		break;

		case GPIO_D:/** GPIO D is selected*/
			GPIOD->PDDR = ((GPIOD->PDDR & (~(1 << pin))) | (state << pin));
		break;

		case GPIO_E: /** GPIO E is selected*/
			GPIOE->PDDR = ((GPIOE->PDDR & (~(1 << pin))) | (state << pin));
		break;

		default:/**If doesn't exist the option*/
		break;
	}
}
///////////////////////////////////////////////////////////////////////////////
///EOF
///////////////////////////////////////////////////////////////////////////////
