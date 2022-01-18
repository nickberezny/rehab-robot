#include "../unity/src/unity.h"

#include "./include/Controller.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_controllerThread(void)
{

}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_controllerThread);
return UNITY_END();
}