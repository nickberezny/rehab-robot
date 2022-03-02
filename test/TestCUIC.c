#include "../unity/src/unity.h"

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Structures.h"
#include "./include/CUIC.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

struct Test {
	double t;
};

void test_virtualTractory(void)
{
	
	struct States s = {0};
	struct Params p = {0};
	
	p.Md = 2;

	p.Bd = 3;
	p.Kd = 4;

	s.ddxv = 0;
	s.dxv = 0;
	s.xv = 1;
	s.Fext = 1;
	s.x = 1;
	s.dx = 0;
	s.ddx = 0;
	s.x0 = 2;
	s.dx0 = 1;
	s.ddx0 = 1;
	s.dt = 0.001;

	VirtualTrajectory(&s,&p);
	
	TEST_ASSERT_EQUAL_DOUBLE(5, s.ddxv);
	TEST_ASSERT_EQUAL_DOUBLE(0.005, s.dxv);
	TEST_ASSERT_EQUAL_DOUBLE(1.000005, s.xv);
}

void test_GetCommand(void)
{
	TEST_ASSERT_TRUE(false);
}



int main(void)
{
UNITY_BEGIN();
RUN_TEST(test_virtualTractory);
RUN_TEST(test_GetCommand);
return UNITY_END();
}