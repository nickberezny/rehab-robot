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
	ControlParams->x0 = (double)(rand()/(double)RAND_MAX)*10.0;
}