/*
 * I2C Slave build
 * Made by Felix Jochems
 * Written for an STM32F1 bluepill
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>

static void
my_delay_1( void )
{
   int i = 72e6/2/4;
   //int i = 100000;
   while( i > 0 )
     {
        i--;
        __asm__( "nop" );
     }
}


#define MYSLAVE_ADDRESS 0x33

//Commands
#define MYSLAVE_GUID	          0xA1
#define MYSLAVE_CLAIM          0xA2
#define MYSLAVE_ENDCLAIM       0xA3
#define MYSLAVE_UNLOCK         0xA4
static void
i2c_slave_init(uint8_t ownaddress)
{
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_I2C1);

   nvic_enable_irq(NVIC_I2C1_EV_IRQ);

   // configure i2c pins
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                 GPIO_I2C1_SDA); //PB7
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                 GPIO_I2C1_SCL); //PB6

   i2c_reset(I2C1);
   i2c_peripheral_disable(I2C1);

   i2c_set_speed(I2C1, i2c_speed_sm_100k, I2C_CR2_FREQ_36MHZ);
   i2c_set_own_7bit_slave_address(I2C1, ownaddress);
   i2c_enable_interrupt(I2C1, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
   i2c_peripheral_enable(I2C1);

   // slave needs to acknowledge on receiving bytes
   // set it after enabling Peripheral i.e. PE = 1
   i2c_enable_ack(I2C1);
}

static void
button_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO9);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO6);
	gpio_set(GPIOA,GPIO9);
	gpio_clear(GPIOA,GPIO6);
}

volatile uint8_t *read_p;
volatile uint8_t *write_p;
volatile uint8_t writing;
volatile uint8_t reading;
volatile uint8_t readbuf[4];
volatile uint8_t writebuf[4];
volatile uint8_t buf[3];
volatile uint16_t val;

volatile uint8_t reserved = 0;
volatile uint8_t writesize = 0;
volatile uint8_t unlock = 0;


//i2c1 event ISR, triggers on I2C pins
void i2c1_ev_isr(void)
{
   uint32_t sr1, sr2;

   sr1 = I2C_SR1(I2C1);
   // Address matched (Slave)
   if (sr1 & I2C_SR1_ADDR)
     {
        reading = 0;
        read_p = buf;
        write_p = ((volatile uint8_t *)(&val) + 1);
        writing = 3;
        //Clear the ADDR sequence by reading SR2.
        sr2 = I2C_SR2(I2C1);
        (void) sr2;
     }

   // Receive buffer not empty
   else if (sr1 & I2C_SR1_RxNE)
     {
	if(reading > 4)
	    return;
        //read bytes from slave
        readbuf[reading] = i2c_get_data(I2C1);
        reading++;
     }

   // Transmit buffer empty & Data byte transfer not finished
   else if ((sr1 & I2C_SR1_TxE) && !(sr1 & I2C_SR1_BTF))
     {
        //send data to master in MSB order
        //i2c_send_data(I2C1, *write_p--);
	i2c_send_data(I2C1, writebuf[writesize]);
        writesize--;
     }

   // done by master by sending STOP
   //this event happens when slave is in Recv mode at the end of communication
   else if (sr1 & I2C_SR1_STOPF)
     {
        i2c_peripheral_enable(I2C1);

        if (readbuf[0] == MYSLAVE_GUID)
	{
		writebuf[0] = 0xFF;
		writebuf[1] = 0xAA;
		writebuf[2] = 0xFF;
		writesize = 2;
	}
        else if (readbuf[0] == MYSLAVE_CLAIM)
        {
	    reserved = 1;
	    writebuf[0] = 0xAC;
	    writesize=0;
	}
        else if (readbuf[0] == MYSLAVE_ENDCLAIM)
	{
	    reserved = 0;
	    writebuf[0] = 0xAC;
	    writesize = 0;
	}
		else if(readbuf[0] == MYSLAVE_UNLOCK)
		{
			unlock = 1;
			writebuf[0] = 0xAC;
			writesize = 0;
			
		}
     }
   //this event happens when slave is in transmit mode at the end of communication
   else if (sr1 & I2C_SR1_AF)
     {
        //(void) I2C_SR1(I2C1);
        I2C_SR1(I2C1) &= ~(I2C_SR1_AF);
     }
}

int main( void )
{
   //set STM32 to 72 MHz
   rcc_clock_setup_in_hse_8mhz_out_72mhz();

   // Enable GPIOC clock
   rcc_periph_clock_enable(RCC_GPIOC);

   //Set GPIO13, F1 is special snowflake
   gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
   //switch led off
   gpio_set(GPIOC, GPIO13);

   //initialize i2c slave
   i2c_slave_init(MYSLAVE_ADDRESS);

	//init button
	button_setup();
   while( 1 )
     {
	//gpio_set(GPIOA,GPIO3);
       	my_delay_1();
			if(gpio_get(GPIOA,GPIO9))
			{
				if(reserved)
				{
					if(unlock)
					{
						while(gpio_get(GPIOA,GPIO9))
						{
							gpio_set(GPIOA,GPIO6);
							gpio_clear(GPIOC,GPIO13);
							my_delay_1();
						}
						unlock = 0;
					}
					gpio_clear(GPIOA,GPIO6);
					gpio_set(GPIOC,GPIO13);
				}
				else
				{
					gpio_set(GPIOA,GPIO6);
					gpio_clear(GPIOC,GPIO13);
				}
			}
			else
			{
				gpio_clear(GPIOA,GPIO6);
				gpio_set(GPIOC, GPIO13);
			}
     }
}
