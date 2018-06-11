/*
 * test.h
 *
 * Created: 12/8/2017 4:45:50 PM
 *  Author: raghu
 */ 


#ifndef TEST_H_
#define TEST_H_

#include <asf.h>
#include "ORB_control.h"
#include "super_servo_control.h"
#include "test.h"
#include "delay.h"


void test_ORB(){
	int i =0;
	update_ORB_single(0x31,0,255,0);
	delay_cycles_ms(100);
	update_ORB_single(0x32,0,255,0);
	delay_cycles_ms(100);
	/*
	//LEDS
	update_LEDS_single(0x31,250);
	delay_cycles_ms(2000);
	update_LEDS_single(0x32,250);
	delay_cycles_ms(2000);
	update_LEDS_single(0x33,250);
	delay_cycles_ms(2000);
	update_LEDS_single(0x31,0);
	delay_cycles_ms(2000);
	update_LEDS_single(0x32,0);
	delay_cycles_ms(2000);
	update_LEDS_single(0x34,0);
	delay_cycles_ms(2000);
	*/
	
	/*
	//ORB 
	update_ORB_single(0x31,0,0,250);
	delay_cycles_ms(2000);
	update_ORB_single(0x31,0,250,0);
	delay_cycles_ms(2000);
	update_ORB_single(0x31,2500,0,0);
	delay_cycles_ms(2000);
	*/
	
	/*
	//Super Servo
	update_super_servo_single(0x34,0);
	delay_cycles_ms(2000);
	update_super_servo_single(0x34,120);
	delay_cycles_ms(2000);
	update_super_servo_single(0x34,254);
	delay_cycles_ms(2000);
	*/

	//
	/*
	for(i=0;i<=254;i++)
	{
		update_LEDS_single(0x31,i);
		update_LEDS_single(0x32,i);
		update_ORB_single(0x32,i,0,0);
		delay_cycles_ms(20);
	}
	for(i=254;i>=0;i--)
	{
		update_LEDS_single(0x31,i);
		update_LEDS_single(0x32,i);
		update_ORB_single(0x32,i,0,0);
		delay_cycles_ms(20);
	}
	*/
	/*
	for(i=0;i<=254;i++)
	{
		update_ORB(i,0,0,i,0,0);
		update_LEDS(i,0,0,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	for(i=254;i>=0;i--)
	{
		update_ORB(i,0,0,i,0,0);
		update_LEDS(i,0,0,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	//
	for(i=0;i<=254;i++)
	{
		update_ORB(0,i,0,0,i,0);
		update_LEDS(0,i,0,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	for(i=254;i>=0;i--)
	{
		update_ORB(0,i,0,0,i,0);
		update_LEDS(0,i,0,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	//
	for(i=0;i<=254;i++)
	{
		update_ORB(0,0,i,0,0,i);
		update_LEDS(0,0,i,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	for(i=254;i>=0;i--)
	{
		update_ORB(0,0,i,0,0,i);
		update_LEDS(0,0,i,0);
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	*/
}

/*
void test_LED()
{
	update_LEDS(0,0,10);
}
*/
void test_servos()
{
	static int i =0;
	/*
	for(i=0;i<=254;i++)
	{
		update_super_servo(0,0,0,i);
		delay_cycles_ms(20);
	}
	*/
	if(i==0)
	{
		i=254;
	}
	else
	{
		i=0;
	}
	update_super_servo(i,i,i,i);
	delay_cycles_ms(10);
	
}

#endif /* TEST_H_ */