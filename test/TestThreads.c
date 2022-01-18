#include "../unity/src/unity.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Threads.h"

struct sched_param param[3];
pthread_attr_t attr[3];
pthread_t thread[3];


void setUp (void) 
{

} /* Is run before every test, put unit init calls here. */


void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_initThread(void)
{	

	TEST_ASSERT_TRUE(initThread(&attr[0], &param[0], 97));\
	pthread_attr_getschedparam(&attr[0], &param[0]);
	TEST_ASSERT_EQUAL_INT(97, param[0].sched_priority);
}

int main(void)
{
	UNITY_BEGIN();
	RUN_TEST(test_initThread);
	return UNITY_END();
}