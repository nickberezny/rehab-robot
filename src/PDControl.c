/**
 * @file PDControl.c
 * @author Nick Berezny
 * @date 13 Apr 2022
 * @brief basic position PD controller
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