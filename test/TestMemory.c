#include "../unity/src/unity.h"

#include <pthread.h>

#include "./include/Memory.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_initMutex(void)
{
	pthread_mutex_t lock;
	TEST_ASSERT_EQUAL_INT(initMutex(&lock),0);
}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_initMutex);
return UNITY_END();
}