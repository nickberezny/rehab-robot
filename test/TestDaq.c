#include "../unity/src/unity.h"
#include <stdbool.h>

#include <LabJackM.h>

#include "./include/Daq.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_initDaq(void)
{
	TEST_ASSERT_NOT_EQUAL(initDaq(),0);
	TEST_ASSERT_TRUE(closeAllDaqs());
}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_initDaq);
return UNITY_END();
}