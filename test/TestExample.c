#include "../unity/src/unity.h"
#include "Example.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void test_exampleFn(void)
{
TEST_ASSERT_EQUAL_INT(6, examplFn(3));
TEST_ASSERT_EQUAL_INT(4, examplFn(2));
}

int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_exampleFn);
return UNITY_END();
}