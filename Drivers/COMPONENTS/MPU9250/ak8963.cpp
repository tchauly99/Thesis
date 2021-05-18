//Edited by tchauly99 -- 2021
#include "ak8963.h"

#define AK8963_WHO_AM_I             0x00 //should return 0x48
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_XOUT_L               0x03
#define AK8963_XOUT_H               0x04
#define AK8963_YOUT_L               0x05
#define AK8963_YOUT_H               0x06
#define AK8963_ZOUT_L               0x07
#define AK8963_ZOUT_H               0x08
#define AK8963_ST2                  0x09
#define AK8963_CNTL                 0x0A
#define AK8963_RSV                  0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
#define AK8963_ADDR                 (0x0C<<1)
#define INT_PIN_CFG 				0x37

#define TIMEOUT_MS_DEFAULT          100         /*!< Default MPU9250 I2C communiation timeout */
#define BUFFER_CALIB_DEFAULT        1000        /*!< Default the number of sample data when calibrate */

static I2C_HandleTypeDef ak8963_i2cHandler;
//static ak8963_handle global_ak8963_handle;


typedef void (*read_func)(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);
typedef void (*write_func)(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms);

typedef struct ak8963 {
    ak8963_mode_t               opr_mode;               /*!< AK8963 operatkion mode */
    ak8963_mfs_sel_t            mfs_sel;                /*!< AK8963 magnetometer full scale range */
    ak8963_if_protocol_t        if_protocol;            /*!< AK8963 interface protocol */
    ak8963_hard_iron_bias_t     hard_iron_bias;         /*!< AK8963 magnetometer bias data */
    ak8963_sens_adj_t           asa;                    /*!< AK8963 magnetometer sensitive adjust data */
    ak8963_soft_iron_corr_t     soft_iron_corr;         /*!< AK8963 magnetometer scale */
    ak8963_soft_iron_scale_t	soft_iron_scale;
    float                       mag_scaling_factor;     /*!< AK8963 magnetometer scaling factor */
    read_func                   _read;                  /*!< AK8963 read function */
    write_func                  _write;                 /*!< AK8963 write function */
} ak8963_t;
static ak8963_t global_ak8963_handle;

static void _i2c_write_func(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buf_send[len + 1];
    buf_send[0] = reg_addr;
    for (uint8_t i = 0; i < len; i++)
    {
        buf_send[i + 1] = buf[i];
    }
    HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, buf_send, len + 1, timeout_ms);

}

static void _i2c_read_func(ak8963_handle_t handle, uint8_t reg_addr, uint8_t *buf, uint16_t len, uint32_t timeout_ms)
{
    uint8_t buffer[1];
    buffer[0] = reg_addr;
    HAL_I2C_Master_Transmit(&hi2c1, AK8963_ADDR, buffer, 1, 10);
    HAL_I2C_Master_Receive(&hi2c1, AK8963_ADDR, buf, len, timeout_ms);
}

static void _ak8963_set_func(ak8963_handle_t handle, read_func _read, write_func _write)
{
    handle->_read = _read;
    handle->_write = _write;
}

ak8963_handle_t ak8963_init(ak8963_cfg_t *config, I2C_HandleTypeDef *I2Chnd)
{

    /* Allocate memory for handle structure */
    ak8963_handle_t handle;
    handle = &global_ak8963_handle;

    if (config->if_protocol == AK8963_IF_I2C)
    {
        _ak8963_set_func(handle, _i2c_read_func, _i2c_write_func);
    }
    uint8_t buffer = 0;

	buffer = 0x01;
	handle->_write(handle, AK8963_RSV, &buffer, 1, TIMEOUT_MS_DEFAULT);

    handle->_read(handle, AK8963_WHO_AM_I, &buffer, 1, TIMEOUT_MS_DEFAULT);
    if(buffer != 0x48){
    	return NULL;
    }

	/* Power down AK8963 magnetic sensor */
	buffer = 0x00;
	handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT);
	HAL_Delay(10);

	/* Set fuse ROM access mode */
	//buffer = 0x0F;
    buffer = 0x1F;
	handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT);
	HAL_Delay(10);

	/* Read magnetic sensitivity adjustment */
	uint8_t mag_raw_data[3];
	handle->_read(handle, AK8963_ASAX, mag_raw_data, 3, TIMEOUT_MS_DEFAULT);

	handle->asa.x_axis = (float)(mag_raw_data[0] - 128) / 256.0f + 1.0f;
	handle->asa.y_axis = (float)(mag_raw_data[1] - 128) / 256.0f + 1.0f;
	handle->asa.z_axis = (float)(mag_raw_data[2] - 128) / 256.0f + 1.0f;

	/* Power down AK8963 magnetic sensor */
	buffer = 0x00;
	handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT);
	HAL_Delay(10);

	/* Configure magnetic operation mode and range */
	buffer = 0;
	buffer = (config->opr_mode) & 0x0F;
	buffer |= (config->mfs_sel << 4) & 0x10;
	handle->_write(handle, AK8963_CNTL, &buffer, 1, TIMEOUT_MS_DEFAULT);
	HAL_Delay(10);

	/* Update magnetometer scaling factor */
	switch (config->mfs_sel)
	{
	case AK8963_MFS_14BIT:
		handle->mag_scaling_factor = 10.0f * 4912.0f / 8190.0f;
		break;

	case AK8963_MFS_16BIT:
		handle->mag_scaling_factor = 10.0f * 4912.0f / 32760.0f;
		break;

	default:
		break;
	}

	/* Update handle structure */
	handle->opr_mode = config->opr_mode;
	handle->mfs_sel = config->mfs_sel;
	handle->hard_iron_bias.x_axis = 0;
	handle->hard_iron_bias.y_axis = 0;
	handle->hard_iron_bias.z_axis = 0;
	handle->if_protocol = config->if_protocol;

	return handle;

}

void ak8963_get_mag_raw(ak8963_handle_t handle, ak8963_raw_data_t *raw_data)
{
    uint8_t mag_raw_data[7];
    uint8_t buffer=0;

	/*Check DRDY bit in the status 1 reg if it is ready to read*/
	handle->_read(handle, AK8963_ST1, &buffer, 1, TIMEOUT_MS_DEFAULT);
    if((buffer&0x01) == 0x01){
		handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);
		//Check if the sensor overflows
		handle->_read(handle, AK8963_ST2, &buffer, 1, TIMEOUT_MS_DEFAULT);
			if( (buffer&0x08) == 0x00 ){
			raw_data->x_axis = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
			raw_data->y_axis = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
			raw_data->z_axis = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);
			raw_data->x_axis *= handle->asa.x_axis * handle->mag_scaling_factor;
			raw_data->y_axis *= handle->asa.y_axis * handle->mag_scaling_factor;
			raw_data->z_axis *= handle->asa.z_axis * handle->mag_scaling_factor;
			}
    }
}

void ak8963_get_mag_cali(ak8963_handle_t handle, ak8963_cali_data_t *cali_data)
{
    uint8_t mag_raw_data[7];

    handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);

    int16_t temp_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
    int16_t temp_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
    int16_t temp_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

    cali_data->x_axis = ((float)temp_x * handle->asa.x_axis - handle->hard_iron_bias.x_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.x_axis;
    cali_data->y_axis = ((float)temp_y * handle->asa.y_axis - handle->hard_iron_bias.y_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.y_axis;
    cali_data->z_axis = ((float)temp_z * handle->asa.z_axis - handle->hard_iron_bias.z_axis / handle->mag_scaling_factor) * handle->soft_iron_corr.z_axis;
}

void ak8963_get_mag_scale(ak8963_handle_t handle, ak8963_scale_data_t *scale_data)
{
    uint8_t mag_raw_data[7];
    uint8_t buffer=0;

    /*Check DRDY bit in the status 1 reg if it is ready to read*/
    handle->_read(handle, AK8963_ST1, &buffer, 1, TIMEOUT_MS_DEFAULT);
   if((buffer&0x01) == 0x01){
		handle->_read(handle, AK8963_XOUT_L, mag_raw_data, 7, TIMEOUT_MS_DEFAULT);
		//Check if the sensor overflows
		handle->_read(handle, AK8963_ST2, &buffer, 1, TIMEOUT_MS_DEFAULT);
		if( (buffer&0x08) == 0x00 ){
			int16_t temp_x = (int16_t)((int16_t)(mag_raw_data[1] << 8) | mag_raw_data[0]);
			int16_t temp_y = (int16_t)((int16_t)(mag_raw_data[3] << 8) | mag_raw_data[2]);
			int16_t temp_z = (int16_t)((int16_t)(mag_raw_data[5] << 8) | mag_raw_data[4]);

			int16_t temp_x_= ((float)temp_x * handle->asa.x_axis * handle->mag_scaling_factor - handle->hard_iron_bias.x_axis);
			int16_t temp_y_ = ((float)temp_y * handle->asa.y_axis * handle->mag_scaling_factor - handle->hard_iron_bias.y_axis);
			int16_t temp_z_ = ((float)temp_z * handle->asa.z_axis * handle->mag_scaling_factor - handle->hard_iron_bias.z_axis);

			float soft_iron_x[3]={handle->soft_iron_scale.x_axis[0], handle->soft_iron_scale.x_axis[1], handle->soft_iron_scale.x_axis[2]};
			float soft_iron_y[3]={handle->soft_iron_scale.y_axis[0], handle->soft_iron_scale.y_axis[1], handle->soft_iron_scale.y_axis[2]};
			float soft_iron_z[3]={handle->soft_iron_scale.z_axis[0], handle->soft_iron_scale.z_axis[1], handle->soft_iron_scale.z_axis[2]};

			scale_data->x_axis = soft_iron_x[0]*temp_x_ + soft_iron_x[1]*temp_y_ +soft_iron_x[2]*temp_z_;
			scale_data->y_axis = soft_iron_y[0]*temp_x_ + soft_iron_y[1]*temp_y_ +soft_iron_y[2]*temp_z_;
			scale_data->z_axis = soft_iron_z[0]*temp_x_ + soft_iron_z[1]*temp_y_ +soft_iron_z[2]*temp_z_;
		}

//		buffer=0x75;
//		HAL_I2C_Master_Transmit(&ak8963_i2cHandler, (0x68<<1), &buffer, 1, 10);
//		HAL_I2C_Master_Receive(&ak8963_i2cHandler, (0x68<<1), &buffer, 1, 100);
//		if(buffer!=0x71){
//			buffer=buffer;
//		}
//		else
//			buffer=0x71;
   }
}

void ak8963_set_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t hard_iron_bias)
{
    handle->hard_iron_bias.x_axis = hard_iron_bias.x_axis;
    handle->hard_iron_bias.y_axis = hard_iron_bias.y_axis;
    handle->hard_iron_bias.z_axis = hard_iron_bias.z_axis;
}

void ak8963_get_hard_iron_bias(ak8963_handle_t handle, ak8963_hard_iron_bias_t *hard_iron_bias)
{
    hard_iron_bias->x_axis = handle->hard_iron_bias.x_axis;
    hard_iron_bias->y_axis = handle->hard_iron_bias.y_axis;
    hard_iron_bias->z_axis = handle->hard_iron_bias.z_axis;
}

void ak8963_set_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t soft_iron_corr)
{
    handle->soft_iron_corr.x_axis = soft_iron_corr.x_axis;
    handle->soft_iron_corr.y_axis = soft_iron_corr.y_axis;
    handle->soft_iron_corr.z_axis = soft_iron_corr.z_axis;
}
void ak8963_set_soft_iron_scale(ak8963_handle_t handle, ak8963_soft_iron_scale_t* soft_iron_scale){
	memcpy(&(handle->soft_iron_scale), soft_iron_scale, sizeof(*soft_iron_scale));
}

void ak8963_get_soft_iron_corr(ak8963_handle_t handle, ak8963_soft_iron_corr_t *soft_iron_corr)
{
    soft_iron_corr->x_axis = handle->soft_iron_corr.x_axis;
    soft_iron_corr->y_axis = handle->soft_iron_corr.y_axis;
    soft_iron_corr->z_axis = handle->soft_iron_corr.z_axis;
}

void ak8963_get_mag_sens_adj(ak8963_handle_t handle, ak8963_sens_adj_t *asa)
{
    asa->x_axis = handle->asa.x_axis;
    asa->y_axis = handle->asa.y_axis;
    asa->z_axis = handle->asa.z_axis;
}

void ak8963_auto_calib(ak8963_handle_t handle)
{
    uint32_t i;
    int16_t mag_min[3] = {32767, 32767, 32767}, mag_max[3] = {-32768, -32768, -32768};
    uint32_t sample_count;

    if (handle->mfs_sel == AK8963_MFS_14BIT)
        sample_count = 50;
    if (handle->mfs_sel == AK8963_MFS_16BIT)
        sample_count = 1500;

    for (i = 0; i < sample_count + 100; i++)            /*!< Dismiss 100 first value */
    {
        if (i > 100 && i <= (sample_count + 100))
        {
            ak8963_raw_data_t mag_raw;
            ak8963_get_mag_raw(handle, &mag_raw);

            if (mag_raw.x_axis > mag_max[0])
                mag_max[0] = mag_raw.x_axis;
            if (mag_raw.x_axis < mag_min[0])
                mag_min[0] = mag_raw.x_axis;

            if (mag_raw.y_axis > mag_max[1])
                mag_max[1] = mag_raw.y_axis;
            if (mag_raw.y_axis < mag_min[1])
                mag_min[1] = mag_raw.y_axis;

            if (mag_raw.z_axis > mag_max[2])
                mag_max[2] = mag_raw.z_axis;
            if (mag_raw.z_axis < mag_min[2])
                mag_min[2] = mag_raw.z_axis;
        }
        if (handle->mfs_sel == AK8963_MFS_14BIT)
            HAL_Delay(150);
        if (handle->mfs_sel == AK8963_MFS_16BIT)
            HAL_Delay(15);
    }

    handle->hard_iron_bias.x_axis = (float)((mag_max[0] + mag_min[0]) / 2) * handle->mag_scaling_factor * handle->asa.x_axis;
    handle->hard_iron_bias.y_axis = (float)((mag_max[1] + mag_min[1]) / 2) * handle->mag_scaling_factor * handle->asa.y_axis;
    handle->hard_iron_bias.z_axis = (float)((mag_max[2] + mag_min[2]) / 2) * handle->mag_scaling_factor * handle->asa.z_axis;

    float scale_temp[3];

    scale_temp[0] = (mag_max[0] - mag_min[0]) / 2;
    scale_temp[1] = (mag_max[1] - mag_min[1]) / 2;
    scale_temp[2] = (mag_max[2] - mag_min[2]) / 2;

    float mag_scale_avg = (scale_temp[0] + scale_temp[1] + scale_temp[2]) / 3.0f;

    handle->soft_iron_corr.x_axis = mag_scale_avg / ((float)scale_temp[0]);
    handle->soft_iron_corr.y_axis = mag_scale_avg / ((float)scale_temp[1]);
    handle->soft_iron_corr.z_axis = mag_scale_avg / ((float)scale_temp[2]);
}
