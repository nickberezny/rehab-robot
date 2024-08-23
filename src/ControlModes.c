/****************************************************************************
 * Copyright (C) 2012 by Matteo Franchin                                    *
 *                                                                          *
 * This file is part of Box.                                                *
 *                                                                          *
 *   Box is free software: you can redistribute it and/or modify it         *
 *   under the terms of the GNU Lesser General Public License as published  *
 *   by the Free Software Foundation, either version 3 of the License, or   *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   Box is distributed in the hope that it will be useful,                 *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU Lesser General Public License for more details.                    *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with Box.  If not, see <http://www.gnu.org/licenses/>.   *
 ****************************************************************************/

/**
 * @file ControlModes.c
 * @author Nick Berezny
 * @date 26 Apr 2023
 * @
 *
 * Test test
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/ControlModes.h"
#include "./include/CUIC.h"

void PositionMode(struct States * s, struct ControlParams * p)
{

    BasicPD(s,p);
    return;
}


void AdmittanceMode(struct States * s, struct ControlParams * p)
{

    VirtualTrajectory(s,p);
    BasicPDxv(s,p);
    return;
}

void AdmittanceZeroStiffnessMode(struct States * s, struct ControlParams * p)
{
    VirtualTrajectoryZeroStiffness(s,p);
    BasicPDxv(s,p);
    return;
}

void ImpedanceMode(struct States * s, struct ControlParams * p)
{
    //TODO: Test
    ComputedTorqueImp(s,p);
    return;
}

void UICMode(struct States * s, struct ControlParams * p)
{
    VirtualTrajectory(s,p);
    ComputedTorque(s,p);
    return;
}

void UICZeroStiffnessMode(struct States * s, struct ControlParams * p)
{
    VirtualTrajectoryZeroStiffness(s,p);
    ComputedTorque(s,p);
    return;
}

void ImpFFWMode(struct States * s, struct ControlParams * p)
{
    //kp is the feedforward gain
    s->cmd = ((1.0+p->kp)*(- p->Dd*(s->dx-s->dx0) - p->Kd*(s->x-s->x0)) + p->kp*(s->Fext))/420.7;
}

void ImpAccelMode(struct States * s, struct ControlParams * p)
{
    //note we are subtracting Md and Bd; Kd is added as normal 
    s->cmd = (p->Md*s->ddx + p->Dd*(s->dx-s->dx0) - p->Kd*(s->x-s->x0))/420.7;
}

//Imp Accel
//Imp FFW
//Adm CT

//Post-sensor mass compensation? 

