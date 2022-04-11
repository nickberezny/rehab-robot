
#include "../unity/src/unity.h"
#include <pthread.h>
#include "./include/Parameters.h"
#include "./include/TimeUtilities.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_stepTime(void)
{
	struct timespec ts;
	struct timespec tf;

	ts.tv_nsec = 2;
	tf.tv_nsec = 5;
	ts.tv_sec = 0;
	tf.tv_sec = 0;

	int dt = 0;

	timeStep(&ts,&tf, &dt);

	TEST_ASSERT_EQUAL(3, dt);

	ts.tv_sec = 2;
	ts.tv_nsec = 0;
	tf.tv_sec = 5;
	tf.tv_nsec = 0;

	timeStep(&ts,&tf, &dt);

	TEST_ASSERT_EQUAL(3*NSEC_IN_SEC, dt);

}

void test_getTimeToSleep(void)
{
	struct timespec ts;
	struct timespec tf;

	ts.tv_nsec = 2;
	tf.tv_nsec = 5;
	ts.tv_sec = 0;
	tf.tv_sec = 0;

	getTimeToSleep(&ts,&tf);

	//TEST_ASSERT_EQUAL(2,3);
}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_stepTime);
RUN_TEST(test_getTimeToSleep);
return UNITY_END();
}