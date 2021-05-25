#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "include.h"

#define SAMPLE_PERIOD_MS		200

#define I2C_SCL_IO				13			/*!< gpio number for I2C master clock */
#define I2C_SDA_IO				12			/*!< gpio number for I2C master data  */
#define I2C_ACK_IO				38			/*!< gpio number for I2C ack          */
#define I2C_FREQ_HZ				20000      	/*!< I2C master clock frequency */
#define I2C_PORT_NUM			I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE  	0           /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE  	0           /*!< I2C master do not need buffer */

// I2C common protocol defines
#define WRITE_BIT               I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN            0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0              /*!< I2C ack value */
#define NACK_VAL                0x1              /*!< I2C nack value */

#define VEML7700_I2CADDR_DEFAULT 	0x10  ///< I2C address

#define VEML7700_ALS_CONFIG         0x00  ///< Light configuration register
#define VEML7700_ALS_THREHOLD_HIGH  0x01  ///< Light high threshold for irq
#define VEML7700_ALS_THREHOLD_LOW   0x02  ///< Light low threshold for irq
#define VEML7700_ALS_POWER_SAVE     0x03  ///< Power save regiester
#define VEML7700_ALS_DATA           0x04  ///< The light data output
#define VEML7700_WHITE_DATA         0x05  ///< The white light data output
#define VEML7700_INTERRUPTSTATUS   	0x06  ///< What IRQ (if any)

#define VEML7700_INTERRUPT_HIGH     0x4000 ///< Interrupt status for high threshold
#define VEML7700_INTERRUPT_LOW      0x8000 ///< Interrupt status for low threshold

#define VEML7700_GAIN_1             0x00  ///< ALS gain 1x
#define VEML7700_GAIN_2             0x01  ///< ALS gain 2x
#define VEML7700_GAIN_1_8           0x02  ///< ALS gain 1/8x
#define VEML7700_GAIN_1_4           0x03  ///< ALS gain 1/4x

#define VEML7700_IT_100MS           0x00  ///< ALS intetgration time 100ms
#define VEML7700_IT_200MS           0x01  ///< ALS intetgration time 200ms
#define VEML7700_IT_400MS           0x02  ///< ALS intetgration time 400ms
#define VEML7700_IT_800MS           0x03  ///< ALS intetgration time 800ms
#define VEML7700_IT_50MS            0x08  ///< ALS intetgration time 50ms
#define VEML7700_IT_25MS            0x0C  ///< ALS intetgration time 25ms

#define VEML7700_PERS_1             0x00  ///< ALS irq persisance 1 sample
#define VEML7700_PERS_2             0x01  ///< ALS irq persisance 2 samples
#define VEML7700_PERS_4             0x02  ///< ALS irq persisance 4 samples
#define VEML7700_PERS_8             0x03  ///< ALS irq persisance 8 samples

#define VEML7700_POWERSAVE_MODE1    0x00  ///< Power saving mode 1
#define VEML7700_POWERSAVE_MODE2    0x01  ///< Power saving mode 2
#define VEML7700_POWERSAVE_MODE3    0x02  ///< Power saving mode 3
#define VEML7700_POWERSAVE_MODE4    0x03  ///< Power saving mode 4
/* Configuration Register #0 (REG_ALS_CONF) 
bit[15:13] (reserved)
Set 000b
 
bit[12:11] ALS_GAIN Gain Selection
00 = ALS gain x 1
01 = ALS gain x 2
10 = ALS gain x (1/8)
11 = ALS gain x (1/4)
 
bit[10]    (reserved)
set 0b
 
bit[9:6]   ALS_IT ALS integration time settings
1100 =  25 ms
1000 =  50 ms
0000 = 100 ms
0001 = 200 ms
0010 = 400 ms
0011 = 800 ms
 
bit[5:4]   ALS_PERS ALS Persistence protect number setting
00 = 1
01 = 2
10 = 4
11 = 8
 
bit[3:2]   (reserved)
Set 00b
 
bit[1]     ALS_INT_EN ALS interrupt enable setting
0 = ALS INT disable
1 = ALS INT enable
 
bit[0]     ALS_SD ALS shut down setting
0 = ALS power on
1 = ALS shut down
*/
 
/* High Threshold Windows Setting #1 
bit[15:8] ALS high threshold window setting (15:8 MSB 8 bits of whole 16 bits)
bit[7:0]  ALS high threshold window setting (7:0 LSB 8 bits of whole 16 bits)
*/
 
/* Low Threshold WIndows Setting #2
bit[15:8] ALS Low threshold window setting (15:8 MSB 8 bits of whole 16 bits)
bit[7:0]  ALS Low threshold window setting (7:0 LSB 8 bits of whole 16 bits)
*/
 
/* Power Saving Mode 
bit[15:3] (reserved)
bit[2:1] PSM Power saving mode; see table "Refresh time"
*/
 
/* ALS High Resolution Output data #4
bit[15:8] ALS high resolution output data (15:8 MSB 8 bits of whole 16 bits)
bit[7:0]  ALS high resolution output data (7:0 LSB 8 bits of whole 16 bits)
*/
 
/* WHITE Channel Output data #5
bit[15:8] WHITE output data (15:8 MSB 8 bits of whole 16 bits)
bit[7:0] WHITE output data (7:0 MSB 8 bits of whole 16 bits)
*/
 
/* Interrupt status #6
bit[15] int_th_low Read bit. Indicate a low threshold exceed.
bit[14] int_th_high Read bit. Indicate a high threshold exceed.
bit[13:0] (reserved)
*/
#define ASLP_RATE_20MS			0x00
#define ACTIVE_MASK				0x01
#define DATA_RATE_80MS        	0x28
#define FULL_SCALE_2G         	0x00
#define MODS_MASK             	0x03
#define MODS1_MASK            	0x02
#define MODS0_MASK            	0x01
#define PP_OD_MASK            	0x01
#define INT_EN_DRDY_MASK      	0x01
#define INT_CFG_DRDY_MASK     	x01

float powerPanel[]=
{ 
	PANEL1980PW, PANEL1990PW, PANEL2010PW, 
	PANEL_WHITEPW, PANEL_KALEOPW, PANEL_TERRACOTAPW
};

/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);

    i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
}

/**
 * @brief test code to read i2c slave device with registered interface
 * ________________________________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | command + ack | start | slave_addr + rd_bit + ack | LSB + ack | MSB + nack | stop |
 * |-------|---------------------------|---------------|-------|---------------------------|-----------|------------|------|
 *
 */
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // Send repeated start
    i2c_master_start(cmd);
    // now send device address (indicating read) & read data
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * _______________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | command + ack | write LSB + ack | write MSB + ack | stop |
 * --------|---------------------------|---------------|-----------------|-----------------|------|
 *
 */
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read contents of a VEML7700 register
---------------------------------------------------------------------------*/
esp_err_t VEML7700read( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_read_slave_reg( I2C_PORT_NUM, VEML7700_I2CADDR_DEFAULT,  reg, pdata, count ) );
}

/* Write value to specified VEML7700 register
---------------------------------------------------------------------------*/
esp_err_t VEML7700write( uint8_t reg, uint8_t *pdata, uint8_t count )
{
	return( i2c_master_write_slave_reg( I2C_PORT_NUM, VEML7700_I2CADDR_DEFAULT,  reg, pdata, count ) );
}
/*--------------------------------------------------------------------------*/
uint16_t byte_swap( uint16_t data )
{
	return( (data >> 8) | (data << 8));
}

/**
 * @brief VEML7700 initialization
 */
void VEML7700_init()
{
	uint8_t data[2];

	i2c_master_init();
    // Pour mesurer la luminosité du soleil il faut un faible temps d'exposition et
    // l'interval de mesure maximum
	
	// configuration du gain (1/8) et du temps d'exposition 25ms -> max range
	data[0]=0x00;data[1]=0x13;
	VEML7700write(VEML7700_ALS_CONFIG, data, 2);
	// als_WH, interrupt_high
	data[0]=0x00;data[1]=0x10;
	VEML7700write(VEML7700_ALS_THREHOLD_HIGH, data, 2);
	// als_WH, interrupt_low
	data[0]=0x00;data[1]=0x00;
	VEML7700write(VEML7700_ALS_THREHOLD_LOW, data, 2);
	// pow_sav, power_save_mode
	data[0]=0x00;data[1]=0x00;
	VEML7700write(VEML7700_ALS_POWER_SAVE, data, 2);
}

/**
 * @brief VEML7700 read luminosity
 */
uint32_t VEML7700_getValue(uint8_t panel)
{
	uint8_t data[2];
	uint16_t *val=(uint16_t *)data;
	float lux=1.8432;
	float Pmax=powerPanel[panel]*2; // deux panneaux
	float power;

    // Lit la valeur du capteur en lux (corrigé) et retourne un int
	esp_err_t ret=VEML7700read(VEML7700_ALS_DATA, data, 2);
	if(ret!=ESP_OK)
	{
		return 11111;
	}
	lux*=(float)(*val);
	// lux sur une surface standard de 2 panneaux solaire (1.6 m^2)
    power = lux * (Pmax / 26000);
    // puissance du panneau solaire specifique
    //printf("produced power : %.0f W", power);
    if(power > Pmax) power = Pmax;
	
	return (uint32_t)power;	
}
