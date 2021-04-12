/*
 * mainpp.cpp
 *
 *  Created on: Feb 2, 2021
 *      Author: chau
 */
#define CODE_CPP
#include "mainpp.h"
#define IMU_600USD_SIZE 80
#define IMU_600USD
//#define IMU_CALIB
#define ROS
//#define DUTY_DB
Encoder Enc_L((13*4*28.8461), LEFT, 3, 1, 0.01);
Encoder Enc_R((13*4*28.8461), RIGHT, 3, 1, 0.01); //24.0384
uint8_t imu_receive_buffer[IMU_600USD_SIZE*2];
uint8_t imu_temp[IMU_600USD_SIZE*2];
uint8_t imu_cache[IMU_600USD_SIZE];
IMU_value imu_value;
IMU_prevalue imu_prevalue;
IMU_sensor_prevalue IMU_premag;
IMU_sensor_value IMU_mag;
uint8_t IMU_index=0;
//float Encoder::test = 2;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	 if(htim->Instance== htim9.Instance)
	 {
		 cnt++;
		 cnt_1++;

		 if(cnt_1%5==0){
			 get_odom();
#ifdef ROS
			 prepub_odom();
#endif
			 flag_b[ONE_HUNDRED]=true;
		 }
		 if(cnt_1==10){
			 cnt_1=0;
			 flag_b[TWO_HUNDRED] = true;
		 }
		 if (cnt == 3){
			 cnt = 0;
			 flag_b[SIXTY] = true;
			Get_encoder();
			robot_speed = (Enc_L.encoder_speed_inmeter + Enc_R.encoder_speed_inmeter)*100/2.0;
			//yaw_msg.data = odom_pose[0];
			yaw_msg.data = robot_speed;
#ifndef DUTY_DB
			Control_motor();
#endif
#ifndef IMU_600USD
			update_quaternion();
#endif
			//MPU9250_Get_Scaled(&Accel_scaled_disp, &Gyro_scaled_disp);
			//MPU9250_AK8963_GetMag_Scaled(&Mag_scaled_disp);
		 }
		 flag_b[TWENTY]=true;
	}
}
void setup(void){

#ifndef IMU_600USD
	//Choose if to calib IMU:
	//MPU9250_AG_Calibrate(&Accel_bias_disp, &Gyro_bias_disp);
	IMU_Config();
	//MPU9250_AK8963_AutoCalib();
#endif

#ifdef ROS
	ros_setup();
#endif
#ifdef IMU_CALIB
	ros_setup();
#endif
#ifdef IMU_600USD
	HAL_UART_Receive_DMA(&huart5, imu_receive_buffer, IMU_600USD_SIZE*2);
	//IMU_600USD_GetSample();
#endif
#ifndef IMU_600USD
	//Get magnetometer samples for calib, rotate IMU
	//MPU9250_AK8963_GetSample();
#endif
	//HAL_Delay(2000);
	//str_msg.data = hello;
	HAL_TIM_Base_Start_IT(&htim9);

	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
//
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

#ifdef DUTY_DB
	Enc_L.duty=0;
	Enc_R.duty=0;
#endif



	for(int i=0; i<=NUMBER_OF_FLAG; i++){
		flag_b[i]=0;
	}

	//nh.loginfo("Advertise and Subcribe successfully!");
}
void loop(void){
#ifdef ROS
	update_Variable(nh.connected());     /*!< Update variable */
	update_TFPrefix(nh.connected());
#endif
	/*!< Update TF */

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
#ifdef DUTY_DB
	Enc_L.set_PWM();
	Enc_R.set_PWM();
#endif

#ifdef ROS
	if (flag_b[SIXTY]){
		//chatter.publish(&str_msg);
		publish_IMU();
		yaw_pub.publish(&yaw_msg);
		flag_b[SIXTY] = false;
	}

	if (flag_b[ONE_HUNDRED]){
		publish_driveinfo();
		flag_b[ONE_HUNDRED] = false;
	}

	if (flag_b[TWO_HUNDRED]){
		flag_b[TWO_HUNDRED] = false;
	}

	//send_logmsg();
		nh.spinOnce();
	waitfor_SerialLink(nh.connected());
#endif
#ifdef IMU_CALIB
#ifdef IMU_600USD
	IMU_600USD_GetSample();
#else
	MPU9250_AK8963_GetSample();
#endif
#endif
}

void ros_setup(void)
{
	nh.initNode();
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
#ifndef IMU_CALIB
	while(!nh.connected())
	  {
		nh.spinOnce();
	  }
#endif
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	nh.loginfo("Init node successfully!");
	nh.loginfo("Write Led Red successfully!");

	/*advertise*/
//	nh.advertise(chatter);
	nh.advertise(yaw_pub);
	nh.advertise(imu_pub);              /*!< Register the publisher to "imu" topic */
//    nh.advertise(cmd_vel_motor_pub);    /*!< Register the publisher to "cmd_vel_motor" topic */
    nh.advertise(odom_pub);             /*!< Register the publisher to "odom" topic */

	/*subcribe*/
	nh.subscribe(toggle_sub);
	nh.subscribe(cmd_vel_sub);          /*!< Subscribe "cmd_vel" topic to get motor cmd */
    //nh.subscribe(reset_sub);            /*!< Subscribe "reset" topic */


    tf_broadcaster.init(nh);            /*!< Init TransformBroadcaster */
    HAL_Delay(500);
    nh.negotiateTopics();

}
void IMU_600USD_Read(void){
//	float cache[3];

	memcpy(imu_cache, &imu_temp[IMU_index], sizeof(imu_cache));
	memcpy(imu_prevalue.roll, &imu_cache[1], 6);
	memcpy(imu_prevalue.pitch, &imu_cache[8], 6);
	memcpy(imu_prevalue.yaw, &imu_cache[15], 6);

	imu_value.roll = -atof(imu_prevalue.roll)/100.0;
	imu_value.pitch = atof(imu_prevalue.pitch)/100.0;
	imu_value.yaw = -atof(imu_prevalue.yaw)/100.0;

	memcpy(IMU_premag.x, &imu_cache[61], 5);
	memcpy(IMU_premag.y, &imu_cache[67], 5);
	memcpy(IMU_premag.z, &imu_cache[73], 5);

	IMU_mag.x = atof(IMU_premag.x);
	IMU_mag.y = atof(IMU_premag.y);
	IMU_mag.z = atof(IMU_premag.z);

}
void IMU_600USD_GetSample(void){
//	int i, sample_count;
//	sample_count = 2000;
//	IMU_sensor_prevalue IMU_magsample;
//	char buffer[50];
//	for (i = 0; i < sample_count + 100; i++)            /*!< Dismiss 100 first value */
//	    {
//	        if (i > 100 && i <= (sample_count + 100))
//	        {
//	        	memcpy(&IMU_magsample, &IMU_premag, sizeof(IMU_premag));
//	        	memcpy(buffer, IMU_magsample.x, sizeof(IMU_magsample.x));
//	        	buffer[5] = buffer[12] = ',';
//	        	buffer[18] = '\n';
//	        	memcpy(&buffer[6], IMU_magsample.y, sizeof(IMU_magsample.y));
//	        	memcpy(&buffer[13], IMU_magsample.z, sizeof(IMU_magsample.z));
//	            //n = sprintf(buffer,"%10.2f, %10.2f, %10.2f \n", Sample[0], Sample[1], Sample[2]);
//	            HAL_UART_Transmit(&huart4, (uint8_t*)buffer, 19, 100);
//	        }
//	        HAL_Delay(20);
//	    }

	static int i;
	IMU_sensor_prevalue IMU_magsample;
	char buffer[50];
	i++;
	if (i > 100)
	{
		memcpy(&IMU_magsample, &IMU_premag, sizeof(IMU_premag));
		memcpy(buffer, IMU_magsample.x, sizeof(IMU_magsample.x));
		buffer[5] = buffer[11] = ',';
		buffer[17] = '\n';
		memcpy(&buffer[6], IMU_magsample.y, sizeof(IMU_magsample.y));
		memcpy(&buffer[12], IMU_magsample.z, sizeof(IMU_magsample.z));
		HAL_UART_Transmit(&huart4, (uint8_t*)buffer, 18, 100);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance== huart5.Instance){
	HAL_UART_DMAStop(&huart5);
	 uint8_t i=0;
	 memcpy(imu_temp, imu_receive_buffer, IMU_600USD_SIZE*2);
	 for(i=0; i<(IMU_600USD_SIZE*2); i++){
		 if (imu_temp[i]=='\n'){
			 if(imu_temp[i+IMU_600USD_SIZE-1]=='\r'){
				 memcpy(imu_cache, &imu_temp[i], IMU_600USD_SIZE);
				 break;
			 }
		 }
	 }
	 //nh.spinOnce();
	 IMU_index=i;
	 IMU_600USD_Read();
	 HAL_UART_Receive_DMA(&huart5, imu_receive_buffer, IMU_600USD_SIZE*2);
	}
}
void IMU_Config(void){
	MPU9250_Config();
	MPU9250_AK8963_Config();
	MPU9250_AK8963_Set_Hard_Soft_Iron(&Soft_iron, Hard_iron);
}

void init_Odom(void){
	for (int index = 0; index < 3; index++)
	{
		odom_pose[index] = 0.0;
		odom_vel[index]  = 0.0;
	}

	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;

	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 0.0;

	odom.twist.twist.linear.x  = 0.0;
	odom.twist.twist.angular.z = 0.0;
}
void toggle_callback(const geometry_msgs::Twist &cmd_msg)
{
	led_toggle();
}
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg){
    goal_cmd_vel[LINEAR] = cmd_vel_msg.linear.x;
    goal_cmd_vel[ANGULAR] = cmd_vel_msg.angular.z;
}
void reset_callback(const std_msgs::Empty &reset_msg){
	char log_msg[50];
	(void)(reset_msg);
	sprintf(log_msg, "Start Calibration of Gyro");
	nh.loginfo(log_msg);
	init_Odom();
	sprintf(log_msg, "Reset Odometry");
	nh.loginfo(log_msg);
}
void Low_pass_filter(float* Current, float* Pre, float Alpha){
	float current = *Current;
	float pre = *Pre;
	current = Alpha*current + (1-Alpha)*pre;
	*Current = current;
	*Pre = current;
}
//void set_goal_vel(uint8_t Wheel){
//	float delta_vel=0;
//	delta_vel=goal_wheel_vel[Wheel] - goal_wheel_vel_PID[Wheel];
//	if(((delta_vel <= 0.1) && (delta_vel >= 0.0)) || ((delta_vel >= -0.1) && (delta_vel <= 0.0))){
//		goal_wheel_vel_PID[Wheel] = goal_wheel_vel[Wheel];
//	}
//	else{
//		if(delta_vel > 0.1)
//			goal_wheel_vel_PID[Wheel] += 0.1;
//		else
//			goal_wheel_vel_PID[Wheel] -= 0.1;
//	}
//}

/*Control each wheel velo from robot goal linear velo and angular velo*/
void Control_motor(void){
	float linear_vel, angular_vel;
	linear_vel = goal_cmd_vel[LINEAR];
	angular_vel = goal_cmd_vel[ANGULAR];

	/*Differential drive for 2 wheels*/
	Enc_L.goal_wheel_vel =(linear_vel -(angular_vel*WHEEL_SEPERATION/2))*(60/(2*PI*WHEEL_RADIUS));
	Enc_R.goal_wheel_vel =(linear_vel +(angular_vel*WHEEL_SEPERATION/2))*(60/(2*PI*WHEEL_RADIUS));

	/*Lowpass filter for goal velocity*/
	Low_pass_filter(&Enc_L.goal_wheel_vel, &Enc_L.pre_goal_wheel_vel, alpha);
	Low_pass_filter(&Enc_R.goal_wheel_vel, &Enc_R.pre_goal_wheel_vel, alpha);

	/*PID controller for velocity control*/
	Enc_L.PID_control();
	Enc_R.PID_control();
}
void Get_encoder(void){
	Enc_L.get_encoder();
	Enc_R.get_encoder();
}
void Encoder::PID_control(void){
	double out;
	ek_2 = ek_1;
	ek_1 = ek;
	ek = (double)goal_wheel_vel - (double)encoder_speed;

	if(goal_wheel_vel<0.7 && goal_wheel_vel>-0.7)
		uk=0;
	else uk += Kp*(ek-ek_1) +Ki*(T/2)*(ek+ek_1) + (Kd/T)*(ek-2*ek_1+ek_2);

	out = uk;
	if(uk>u_max) out=u_max;
	if(ek<(-u_max)) out=-u_max;

	duty = (float)(out);


	Encoder::set_PWM();
}
/*
 * Set PWM duty for each wheel
 * Wheel left: Timer4 Channel 1
 * Wheel right: Timer4 Channel 2*/
void Encoder::set_PWM(void){
	float duty_set=duty;
	if (duty>=0){
		__HAL_TIM_SET_COMPARE(&htim4, pwm_channel, duty_set);
		Encoder::set_dir(FORWARD);
	}
	else{
		duty_set = -duty;
		__HAL_TIM_SET_COMPARE(&htim4, pwm_channel, duty_set);
		Encoder::set_dir(REVERSE);
	}
}
/*
 * Set direction for each wheel
 * Wheel left: PB11 PB12
 * Wheel right: PC11, PC10*/
void Encoder::set_dir(uint8_t dir){
	if (wheel == LEFT){
		if (dir == FORWARD){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		}
	}
	else{
		if(dir == FORWARD){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		}
	}
}
void Encoder::get_speed(void){
//	if (delta_encoder_counter[Wheel]<30000){
//		Dir[Wheel]=true;
//		delta_encoder_counter_real[Wheel] = delta_encoder_counter[Wheel];
//		round_encoder[Wheel]=(float)((delta_encoder_counter_real[Wheel])/(pulse_per_round[Wheel]));
//	}
//	else{
//		Dir[Wheel]=false;
//		delta_encoder_counter_real[Wheel] = 0xFFFF - delta_encoder_counter[Wheel];
//		round_encoder[Wheel]=(float)((-delta_encoder_counter_real[Wheel])/(pulse_per_round[Wheel]));
//	}
	//static float pre_encoder = 0.0;
	encoder= (float)delta_encoder_counter;
	// Low Pass Filter for encoder
	Low_pass_filter(&encoder, &pre_encoder, beta);

	round_encoder=(float)((encoder)/(pulse_per_round));

	encoder_speed=(round_encoder*60.0)/0.06;

	encoder_speed_inmeter=(encoder_speed*2*PI*WHEEL_RADIUS)/60;

}
void Encoder::get_encoder(void){
	//static uint16_t pre_counter=0.0;
	if(wheel==LEFT){
		delta_encoder_counter = (int16_t)(TIM2->CNT - pre_counter);
		pre_counter=TIM2->CNT;
	}
	else{
		delta_encoder_counter = (int16_t)(TIM3->CNT - pre_counter);
		pre_counter=TIM3->CNT;
	}
	Encoder::get_speed();

}
void get_speed_100ms(float* encoder_inrad_100){
	static uint16_t pre_counter_100[2]={0, 0};
	int16_t delta_counter_100[2];

	delta_counter_100[LEFT] = (int16_t)(TIM3->CNT - pre_counter_100[LEFT]);
	delta_counter_100[RIGHT] = (int16_t)(TIM2->CNT - pre_counter_100[RIGHT]);
	encoder_inrad_100[LEFT]=(float)((2*PI*delta_counter_100[LEFT])/Enc_L.pulse_per_round);
	encoder_inrad_100[RIGHT]=(float)((2*PI*delta_counter_100[RIGHT])/Enc_R.pulse_per_round);
	pre_counter_100[LEFT]=TIM3->CNT;
	pre_counter_100[RIGHT]=TIM2->CNT;
}
void get_odom(void){
	float delta_s, roll, pitch, yaw;
	float quat_data[4];
	float v,w;
	float encoder_inrad[2];
	static uint8_t delay=0;

	get_speed_100ms(encoder_inrad);
	delta_s = WHEEL_RADIUS*(encoder_inrad[LEFT] + encoder_inrad[RIGHT])/2.0;
#ifndef IMU_600USD
	MADGWICK_Get_Quaternion(quat_data);
	quat[0] = quat_data[0];
	quat[1] = quat_data[1];
	quat[2] = quat_data[2];
	quat[3] = quat_data[3];
	theta = atan2f(quat_data[0] * quat_data[3] + quat_data[1] * quat_data[2], 0.5f - quat_data[2] * quat_data[2] - quat_data[3] * quat_data[3]);
	//Wait 5s for quaternion to get stable at the beginning
	if (delay<100){
		delay++;
		last_theta=theta;
	}
	else{

		roll = atan2f(2.0*(quat_data[0]*quat_data[1]+quat_data[2]*quat_data[3]), quat_data[0]*quat_data[0]-quat_data[1]*quat_data[1]-quat_data[2]*quat_data[2]+quat_data[3]*quat_data[3]);
		pitch = asinf(-2.0*(quat_data[0]*quat_data[2]-quat_data[1]*quat_data[3]));
		yaw=theta;

		delta_theta = theta - last_theta;
		if ( delta_theta>=6.0 || delta_theta<= -6.0)
				delta_theta = 0;

		odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
		odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
		odom_pose[2] += delta_theta;

		v = delta_s/0.1;
		w = DEG2RAD(Gyro_scaled_disp.z)*(cosf(roll)/cosf(pitch));
		//w = delta_theta/0.1;
	}
#else
	theta = DEG2RAD(imu_value.yaw);
	if (delay<30){
		delay++;
		last_theta=theta;
	}
	else{
		delta_theta = theta - last_theta;
		if ( delta_theta>=6.0 | delta_theta<= -6.0)
			delta_theta = 0;
		odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
		odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
		odom_pose[2] += delta_theta;

		v = delta_s/0.1;
		w = delta_theta/0.1;
	}

#endif
	odom_vel[0] = v;
	odom_vel[1] = 0.0;
	odom_vel[2] = w;

	last_theta = theta;

}

void prepub_odom(void){
	odom.header.frame_id = odom_header_frame_id;
	odom.child_frame_id  = odom_child_frame_id;

	odom.pose.pose.position.x = odom_pose[0];
	odom.pose.pose.position.y = odom_pose[1];
	odom.pose.pose.position.z = 0;
	odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

	odom.twist.twist.linear.x  = odom_vel[0];
	odom.twist.twist.angular.z = odom_vel[2];
}
void update_TF(geometry_msgs::TransformStamped& odom_tf)
{
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;
}
void update_quaternion(void){
	MADGWICK_Quaternion_Update();
}
void publish_driveinfo(void){
	ros::Time stamp_now = nh.now();
	odom.header.stamp = stamp_now;
	odom_pub.publish(&odom);

	update_TF(odom_tf);
	odom_tf.header.stamp = stamp_now;
	tf_broadcaster.sendTransform(odom_tf);

//	yaw_pub.publish(&yaw_msg);
}
void led_toggle (void)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
}

void update_IMU(sensor_msgs::Imu& imu_msg_, float* quat, ScaledData_Def* accel_scaled, ScaledData_Def* gyro_scaled)
{
	imu_msg_.angular_velocity.x = (double)gyro_scaled->x;
	imu_msg_.angular_velocity.y = (double)gyro_scaled->y;
	imu_msg_.angular_velocity.z = (double)gyro_scaled->z;

	imu_msg_.linear_acceleration.x = (double)accel_scaled->x;
	imu_msg_.linear_acceleration.y = (double)accel_scaled->y;
	imu_msg_.linear_acceleration.z = (double)accel_scaled->z;

	imu_msg_.orientation.w = quat[0];
	imu_msg_.orientation.x = quat[1];
	imu_msg_.orientation.y = quat[2];
	imu_msg_.orientation.z = quat[3];
}
void publish_IMU(void)
{
	ScaledData_Def Accel_scaled, Gyro_scaled;
	float quat_data[4];
	MPU9250_Get_Scaled(&Accel_scaled, &Gyro_scaled);
	MADGWICK_Get_Quaternion(quat_data);
	update_IMU(imu_msg, quat_data, &Accel_scaled, &Gyro_scaled);
	imu_pub.publish(&imu_msg);
}
void update_TFPrefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[50];

    if (isConnected)
    {
        if (isChecked == false)
        {
            nh.getParam("~tf_prefix", &get_tf_prefix);

            if (!strcmp(get_tf_prefix, ""))
            {
                sprintf(odom_header_frame_id, "odom");
                sprintf(odom_child_frame_id, "base_footprint");

                sprintf(imu_frame_id, "imu_link");
            }
            else
            {
                strcpy(odom_header_frame_id, get_tf_prefix);
                strcpy(odom_child_frame_id, get_tf_prefix);

                strcpy(imu_frame_id, get_tf_prefix);

                strcat(odom_header_frame_id, "/odom");
                strcat(odom_child_frame_id, "/base_footprint");

                strcat(imu_frame_id, "/imu_link");
            }

            sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
            nh.loginfo(log_msg);

            sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
            nh.loginfo(log_msg);

            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}
void waitfor_SerialLink(bool isConnected)
{
    static bool wait_flag = false;

    if (isConnected)
    {
        if (wait_flag == false)
        {
            HAL_Delay(10);

            wait_flag = true;
        }
    }
    else
    {
        wait_flag = false;
    }
}
void send_logmsg(void)
{
    static bool log_flag = false;

    if (nh.connected())
    {
        if (log_flag == false)
        {
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            sprintf(log_msg, "Connected to openSTM32-Board");
            nh.loginfo(log_msg);

            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);

            log_flag = true;
        }
    }
    else
    {
        log_flag = false;
    }
}
void update_Variable(bool isConnected)
{
    static bool variable_flag = false;

    if (isConnected)
    {
        if (variable_flag == false)
        {
//            initIMU();
            init_Odom();

            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}


