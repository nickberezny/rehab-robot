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
	//v(x)
	s->dx0 = 2.0*p->vmax/p->xend * s->x;
	s->x0 += s->dx0 * s->dt;
}

void RandomStaticPosition(struct States * s, struct ControlParams * p)
{
	//
	p->x0 = (double)(rand()/(double)RAND_MAX)*10.0;
}

void SineWave(struct States * s, struct ControlParams * p, double A, double w)
{
	//A: amplitude (percent of xend)
	//w: freq
	if(A < 0 || A > 1) 
	{
		printf("Amplitude out of range: %f\n", A);
		A = 0;
	}

	p->x0 = A*(p->xend/2.0)*sin(w*s->t) + (p->xend/2.0) ;
	p->dx0 = w*A*(p->xend/2.0)*cos(w*s->t);
}