#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./systick/bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#include "./exti/bsp_exti.h"
#include "./key/bsp_key.h"
#include "./lcd/bsp_ili9341_lcd.h"

#include "mltypes.h"

#include "inv_mpu_dmp_motion_driver.h"
#include "log.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mpu.h"
#include "inv_mpu.h"
#include "log.h"
#include "packet.h"


#define WINDOW_WIDTH 240
#define WINDOW_HEIGHT 320
#define N 20
#define M 20
#define ROOM_SIZE 10
#define WALL_SIZE 2
struct edge
{
	int ux, uy, vx, vy;
	int dir;
};

int fa[N * M]; //并查集
char maze_map[N * 2 + 1][M * 2 + 1];
struct edge edges[N * (M - 1) + M * (N - 1)];
int cnt_e = 0;
int getfa(int x)
{
	if (fa[x] == x)
		return x;
	return fa[x] = getfa(fa[x]);
}

void maze_init()
{
	memset(maze_map, '#', sizeof(maze_map));
	for (int i = 0; i < N; i++)
		for (int j = 0; j < M; j++)
		{
			fa[i * M + j] = i * M + j;
			maze_map[i * 2 + 1][j * 2 + 1] = ' ';
		}
	for (int i = 0; i < N; i++)
		for (int j = 0; j < M - 1; j++)
		{
			edges[cnt_e].ux = i;
			edges[cnt_e].uy = j;
			edges[cnt_e].vx = i;
			edges[cnt_e].vy = j + 1;
			maze_map[i * 2 + 1][j * 2 + 2] = '#';
			edges[cnt_e].dir = 0; //横向
			cnt_e++;
		}
	for (int j = 0; j < M; j++)
		for (int i = 0; i < N - 1; i++)
		{
			edges[cnt_e].ux = i;
			edges[cnt_e].uy = j;
			edges[cnt_e].vx = i + 1;
			edges[cnt_e].vy = j;
			maze_map[i * 2 + 2][j * 2 + 1] = '#';
			edges[cnt_e].dir = 1; //纵向
			cnt_e++;
		}

	while (cnt_e)
	{
		int x = rand() % cnt_e;
		int fa_v = getfa(edges[x].vx * M + edges[x].vy);
		int fa_u = getfa(edges[x].ux * M + edges[x].uy);
		if (fa_v != fa_u)
		{
			fa[fa_v] = fa_u;
			if (edges[x].dir == 0)
				maze_map[edges[x].ux * 2 + 1][edges[x].uy * 2 + 2] = ' ';
			else
				maze_map[edges[x].ux * 2 + 2][edges[x].uy * 2 + 1] = ' ';
		}
		for (int i = x + 1; i < cnt_e; i++)
			edges[i - 1] = edges[i];
		cnt_e--;
	}
}
#define POINT_SIZE 4
int now_x = POINT_SIZE + WALL_SIZE;
int now_y = POINT_SIZE + WALL_SIZE;
int dx[] = {0, 1, 0, -1}; //右、下、左、上
int dy[] = {1, 0, -1, 0};
int dir = 0;

void draw_map()
{
	int clr_x = now_x - POINT_SIZE-1;
	int clr_y = now_y - POINT_SIZE-1;
	ILI9341_Draw_Rec_Color(clr_x > 0 ? clr_x : 0, clr_y > 0 ? clr_y : 0, POINT_SIZE * 2+2, POINT_SIZE * 2+2, 0, 0, 0);
	LCD_SetColors(RED, BLACK);
	ILI9341_DrawCircle(now_x, now_y, POINT_SIZE / 2, 1);
	for (int i = 0; i < N * 2 + 1; i++)
	{
		int dir = (i + 1) & 1;
		for (int j = dir; j < M * 2 + 1; j += 2)
		{
			if (maze_map[i][j] == '#')
			{
				if (dir == 1)
					ILI9341_Draw_Rec_Color(ROOM_SIZE * (j / 2), ROOM_SIZE * (i / 2), ROOM_SIZE, WALL_SIZE, 255, 255, 255);
				else
					ILI9341_Draw_Rec_Color(ROOM_SIZE * (j / 2), ROOM_SIZE * (i / 2), WALL_SIZE, ROOM_SIZE, 255, 255, 255);
			}
		}
	}
}
int dirs[4][5][5] = {{0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0}, {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0}, {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0}, {0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0}};
#define DIR_SIZE 4
void show_dir()
{
	for (int i = 0; i < 5; i++)
		for (int j = 0; j < 5; j++)
		{
			uint8_t c = dirs[dir][i][j] == 1 ? 255 : 0;
			ILI9341_Draw_Rec_Color(50 + i * DIR_SIZE, 290 + j * DIR_SIZE, DIR_SIZE, DIR_SIZE, c, c, c);
		}
}

void move(int dir)
{
	if (now_x + dx[dir] >= 0 && now_x + dx[dir] <= WINDOW_WIDTH && now_y + dy[dir] >= 0 && now_y + dy[dir] <= WINDOW_HEIGHT)
	{
		printf("%d %d %d\n", now_x + dx[dir], now_y + dy[dir], ILI9341_GetPointPixel(now_x + dx[dir], now_y + dy[dir]));
		if (ILI9341_GetPointPixel(now_x + dx[dir], now_y + dy[dir]) != WHITE)
		{
			now_x = now_x + dx[dir];
			now_y = now_y + dy[dir];
			draw_map();
			show_dir();
		}
	}
}

#define TASK_ENABLE 0
extern unsigned int Task_Delay[NumOfTask];

/* Private typedef -----------------------------------------------------------*/
/* Data read from MPL. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define PRINT_COMPASS (0x08)
#define PRINT_EULER (0x10)
#define PRINT_ROT_MAT (0x20)
#define PRINT_HEADING (0x40)
#define PRINT_PEDO (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

#define MOTION (0)
#define NO_MOTION (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ (20)

#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)
#define COMPASS_READ_MS (100)

struct rx_s
{
    unsigned char header[3];
    unsigned char cmd;
};

struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
struct platform_data_s
{
    signed char orientation[9];
};


static struct platform_data_s gyro_pdata = {
    .orientation = {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1}};


/**
  * @brief  控制串口发送1个字符
  * @param  c:要发送的字符
  * @retval none
  */
void usart1_send_char(uint8_t c)
{
    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET)
        ; //循环发送,直到发送完毕
    USART_SendData(DEBUG_USARTx, c);
}

/**
  * @brief  按“匿名四轴”上位机软件的协议发送数据（v2.6版本）
  * @param  c:要发送的字符
  * @param fun:功能字. 0XA0~0XAF
  * @param data:数据缓存区,最多28字节
  * @param  len:data区有效数据个数
  * @retval none
  */
void usart1_niming_report(uint8_t fun, uint8_t *data, uint8_t len)
{
    uint8_t send_buf[32];
    uint8_t i;
    if (len > 28)
        return;            //最多28字节数据
    send_buf[len + 3] = 0; //校验数置零
    send_buf[0] = 0X88;    //帧头
    send_buf[1] = fun;     //功能字
    send_buf[2] = len;     //数据长度
    for (i = 0; i < len; i++)
        send_buf[3 + i] = data[i]; //复制数据
    for (i = 0; i < len + 3; i++)
        send_buf[len + 3] += send_buf[i]; //计算校验和
    for (i = 0; i < len + 4; i++)
        usart1_send_char(send_buf[i]); //发送数据到串口1
}

/**
  * @brief  发送加速度传感器数据和陀螺仪原始数据
  * @param  aacx,aacy,aacz:x,y,z三个方向上面的加速度值
  * @param gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
  * @retval none
  */
void mpu6050_send_data(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz)
{
    uint8_t tbuf[12];
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    usart1_niming_report(0XA1, tbuf, 12); //自定义帧,0XA1
}

/**
  * @brief  发送姿态数据给上位机
  * @param aacx,aacy,aacz:x,y,z三个方向上面的加速度值
  * @param gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
  * @param roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
  * @param pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
  * @param yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
  * @retval none
  */
void usart1_report_imu(short aacx, short aacy, short aacz, short gyrox, short gyroy, short gyroz, short roll, short pitch, short yaw)
{
    uint8_t tbuf[28];
    uint8_t i;
    for (i = 0; i < 28; i++)
        tbuf[i] = 0; //清0
    tbuf[0] = (aacx >> 8) & 0XFF;
    tbuf[1] = aacx & 0XFF;
    tbuf[2] = (aacy >> 8) & 0XFF;
    tbuf[3] = aacy & 0XFF;
    tbuf[4] = (aacz >> 8) & 0XFF;
    tbuf[5] = aacz & 0XFF;
    tbuf[6] = (gyrox >> 8) & 0XFF;
    tbuf[7] = gyrox & 0XFF;
    tbuf[8] = (gyroy >> 8) & 0XFF;
    tbuf[9] = gyroy & 0XFF;
    tbuf[10] = (gyroz >> 8) & 0XFF;
    tbuf[11] = gyroz & 0XFF;
    tbuf[18] = (roll >> 8) & 0XFF;
    tbuf[19] = roll & 0XFF;
    tbuf[20] = (pitch >> 8) & 0XFF;
    tbuf[21] = pitch & 0XFF;
    tbuf[22] = (yaw >> 8) & 0XFF;
    tbuf[23] = yaw & 0XFF;
    usart1_niming_report(0XAF, tbuf, 28); //飞控显示帧,0XAF
}

extern struct inv_sensor_cal_t sensors;

/* Get data from MPL.
 * TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
 * between new and stale data.
 */
static void read_from_mpl(void)
{
    long data[9];
    int8_t accuracy;

    if (1)
    {
        unsigned long timestamp;

        /*获取欧拉角*/
        if (inv_get_sensor_type_euler(data, &accuracy, (inv_time_t *)&timestamp))
        {
			float pitch =  data[0] * 1.0 / (1 << 16);
			float roll = data[1] * 1.0 / (1 << 16);
			float yaw =  data[2] * 1.0 / (1 << 16);
			if(pitch>20.0)
				move(0);
			else if(pitch<-20.0)
				move(2);
			if(roll>20.0)
				move(1);
			else if(roll<-20.0)
				move(3);
            printf( "Pitch :  %.4f  \n",pitch); //inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            printf( "Roll :  %.4f  \n", roll); //inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            printf( "Yaw :  %.4f  \n", yaw); //inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
            /*温度*/
            mpu_get_temperature(data, (inv_time_t *)&timestamp);
            printf( "Temperature :  %.2f  \n", data[0] * 1.0 / (1 << 16)); //inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
        }
    }

}


static void tap_cb(unsigned char direction, unsigned char count)
{
    MPU_DEBUG_FUNC();
    switch (direction)
    {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
    MPU_DEBUG_FUNC();
    switch (orientation)
    {
    case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
    case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
    case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
    case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
    default:
        return;
    }
}

void gyro_data_ready_cb(void)
{

    hal.new_gyro = 1;
}



/**
  * @brief  主函数
  * @param  无  
  * @retval 无
  */
int main(void)
{

    inv_error_t result;
    unsigned char accel_fsr, new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;
	
	Key_GPIO_Config();
	
    ILI9341_Init(); //LCD 初始化
    ILI9341_GramScan(6);
    LCD_SetFont(&Font8x16);
    LCD_SetColors(RED, BLACK);

    SysTick_Init();
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    /* LED 端口初始化 */
    LED_GPIO_Config();
    LED_BLUE;

    /* 串口通信初始化 */
    USART_Config();

    //MPU6050中断引脚
    EXTI_Pxy_Config();
    //I2C初始化
    I2C_Bus_Init();
    ILI9341_Clear(0, 0, 240, 320);
    printf("mpu 6050 test start");
    result = mpu_init(&int_param);
    if (result)
    {
        LED_RED;
        printf("Could not initialize gyro.result =  %d\n", result);
    }
    else
    {
        LED_GREEN;
    }
    result = inv_init_mpl();
    if (result)
    {
        MPL_LOGE("Could not initialize MPL.\n");
    }
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    inv_enable_fast_nomot();
    inv_enable_gyro_tc();
    inv_enable_eMPL_outputs();
    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
            MPL_LOGE("Not authorized.\n");
        }
    }
    if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
    }

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);
    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);

    hal.sensors = ACCEL_ON | GYRO_ON;

    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    get_tick_count(&timestamp);

    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);

    dmp_register_android_orient_cb(android_orient_cb);

    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
	
	maze_init();
	draw_map();
	show_dir();
	
    while (1)
    {
		if (Key_Scan(KEY1_GPIO_PORT, KEY1_GPIO_PIN) == KEY_ON)
		{
			dir = (dir + 1) % 4;
			show_dir();
		}
		if (Key_Scan(KEY2_GPIO_PORT, KEY2_GPIO_PIN) == KEY_ON)
		{
			move(dir);
		}
		
        unsigned long sensor_timestamp;
        int new_data = 0;
        if (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_RXNE))
        {
            USART_ClearFlag(DEBUG_USARTx, USART_FLAG_RXNE);
        }
        get_tick_count(&timestamp);

        if (timestamp > hal.next_temp_ms)
        {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }

        if (hal.motion_int_mode)
        {
            /* Enable motion interrupt. */
            mpu_lp_motion_interrupt(500, 1, 5);
            /* Notify the MPL that contiguity was broken. */
            inv_accel_was_turned_off();
            inv_gyro_was_turned_off();
            inv_compass_was_turned_off();
            inv_quaternion_sensor_was_turned_off();
            /* Wait for the MPU interrupt. */
            while (!hal.new_gyro);
           
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro)
        {
            continue;
        }
        if (hal.new_gyro && hal.lp_accel_mode)
        {
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            hal.new_gyro = 0;
        }
        else if (hal.new_gyro && hal.dmp_on)
        {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;

            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO)
            {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp)
                {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT)
            {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        }
        else if (hal.new_gyro)
        {
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                          &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO)
            {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp)
                {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }

        if (new_data)
        {
            inv_execute_on_data();

            read_from_mpl();
        }
    }
}

