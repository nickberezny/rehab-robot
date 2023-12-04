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
 * @file ReadProcessFile.c
 * @author Nick Berezny
 * @date 4 Dec 2023
 * @Exaaaaample
 *
 * Test test
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "./include/ReadProcessFile.h"

const char* getfield(char* line, int num)
{
    const char* tok;
    for (tok = strtok(line, ";");
            tok && *tok;
            tok = strtok(NULL, ";\n"))
    {
        if (!--num)
            return tok;
    }
    return NULL;
}

void ReadProcessFile(char * fullpath, struct ControlParams * p)
{
	File* stream = fopen(fullpath,"r");
    char line[1024];
    j = 0;
    ctlNum = -1;

    while (fgets(line, 1024, stream))
    {

        char* tmp = strdup(line);
        char *eptr;
        p->process[j] = atoi(getfield(tmp, 2));
        p->ctl[j] = atoi(getfield(tmp, 3));
        if(p->ctl[j] == 5) ctlNum = ctlNum + 1;

        p->t[ctlNum][j] = strtod(getfield(tmp, 4), &eptr);
        p->x[ctlNum][j] = strtod(getfield(tmp, 5), &eptr);
        if(j == 0) p->dx[ctlNum][j] = 0;
        else p->dx[ctlNum][j] = (p->x[ctlNum][j] - p->x[ctlNum][j-1])/(p->t[ctlNum][j] - p->t[ctlNum][j-1]);
        p->cmd[ctlNum][j] = strtod(getfield(tmp, 6), &eptr);
        
        j = j + 1;
        free(tmp);
        
    }
}




