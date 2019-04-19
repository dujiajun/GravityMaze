#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "./usart/bsp_usart.h"
#include "./led/bsp_led.h"
#include "./lcd/bsp_ili9341_lcd.h"
#include "./key/bsp_key.h"
#include "./Exti/bsp_exti.h"
#include "./i2c/bsp_i2c.h"
#include "./Systick/bsp_Systick.h"
#include "./mpu6050/mpu6050.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

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
	int clr_x = now_x - POINT_SIZE * 2;
	int clr_y = now_y - POINT_SIZE * 2;
	ILI9341_Draw_Rec_Color(clr_x > 0 ? clr_x : 0, clr_y > 0 ? clr_y : 0, POINT_SIZE * 4, POINT_SIZE * 4, 0, 0, 0);
	LCD_SetColors(RED, BLACK);
	ILI9341_DrawCircle(now_x, now_y, POINT_SIZE / 2, 1);
	//ILI9341_SetPointPixel(now_x,now_y);
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

int main(void)
{
	//LCD屏初始化
	ILI9341_Init();
	//LED灯初始化
	LED_GPIO_Config();
	LED_BLUE;
	//串口初始化
	USART_Config();
	//按键初始化
	Key_GPIO_Config();
	//I2C初始化
	i2c_GPIO_Config();
	//MPU6050初始化
	MPU6050_Init();
	//初始化systick
	SysTick_Init();
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	
	ILI9341_Clear_Window();
	maze_init();

	draw_map();
	show_dir();

	/* 轮询按键状态 */
	int id = MPU6050ReadID();
	short Accel[3];
	short Gyro[3];
	float Temp;
	
	while (1)
	{
		if(id==1)
		{
			if(Task_Delay[0]==TASK_ENABLE)
			{
				LED2_TOGGLE;
				Task_Delay[0]=1000;
			}
			
			if(Task_Delay[1]==0)
			{
				MPU6050ReadAcc(Accel);			
				printf("\r\n加速度： %8d%8d%8d    ",Accel[0],Accel[1],Accel[2]);
				MPU6050ReadGyro(Gyro);
				printf("陀螺仪： %8d%8d%8d    ",Gyro[0],Gyro[1],Gyro[2]);	
				MPU6050_ReturnTemp(&Temp); 
				printf("温度： %8.2f",Temp);
				Task_Delay[1]=500;
			}
		}
		else
		{
			printf("\r\n没有检测到MPU6050传感器！\r\n");
			LED_RED; 
		}
		if (Key_Scan(KEY1_GPIO_PORT, KEY1_GPIO_PIN) == KEY_ON)
		{
			dir = (dir + 1) % 4;
			show_dir();
		}
		if (Key_Scan(KEY2_GPIO_PORT, KEY2_GPIO_PIN) == KEY_ON)
		{
			move(dir);
		}
	}
	//return 0;
}
