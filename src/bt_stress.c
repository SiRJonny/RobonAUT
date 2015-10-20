/*
 * bt_stress.c
 *
 *  Created on: 2015. okt. 18.
 *      Author: Gabor
 */
#include "bt_stress.h"


extern QueueHandle_t xQueue_BT;

void test() {


	struct BT_MSG msg_int;
	struct BT_MSG * msg_int_ptr = &msg_int;

	struct BT_MSG msg_int_arr[5];

	for (int i = 0; i < 5; i++) {
		msg_int_ptr = &msg_int_arr[i];
		int2msg(msg_int_ptr, i, "iii_int32\n");
		xQueueSend( xQueue_BT, (void*) &msg_int_ptr, osWaitForever);

	}




}







