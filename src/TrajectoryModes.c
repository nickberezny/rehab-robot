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
 * @file Example.c
 * @author Nick Berezny
 * @date 04 May 2023
 * @Exaaaaample
 *
 * Test test
 */
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <limits.h>
#include <math.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"

#include "./include/TrajectoryModes.h"

void BackAndForth(struct States * s, struct ControlParams * p)
{
	//get c
}

void RandomStaticPosition(struct States * s, struct ControlParams * p)
{
	//
	p->x0 = ((double)rand()/(double)RAND_MAX)*p->xend;
}

void GoTo(struct States * s, struct ControlParams * p, double * x0, double vel)
{
	if(*x0 > p->xend) p->x0 = p->xend;
	if(*x0 < 0.0) p->x0 = 0.0;	
	if(p->x0 < *x0)
	{
		p->x0 = p->x0 + 0.001*vel;
		p->dx0 = vel;
	}
	else if(p->x0 > *x0)
	{
		p->x0 = p->x0 - 0.001*vel;
		p->dx0 = -vel;

	}
	if(fabs(p->x0 - *x0) < 0.01)
	{
		p->x0 = *x0;
		p->dx0 = 0.0;
		
	}
	
}

void SineWave(struct States * s, struct ControlParams * p)
{
	p->x0 = p->amplitude*sin(p->frequency*(s->t-p->t_traj_start)) +  p->offset;
	p->dx0 = p->frequency*p->amplitude*sin(p->frequency*(s->t-p->t_traj_start));

	if(p->x0 > p->xend)
	{
		p->x0 = p->xend;
		p->dx0 = 0.0;
	}
	else if(p->x0 < 0)
	{
		p->x0 = 0.0;
		p->dx0 = 0.0;
	}
	
}