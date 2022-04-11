/**
 * @file Home.c
 * @author Nick Berezny
 * @date 3 Mar 2022
 * @Homing the robot
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"

void VelocityControl(struct States * s, double * vel_des, double * P)
{

    s->cmd = (*P) * (*vel_des - s->dx);

}