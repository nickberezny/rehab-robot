/**
 * @file MathUtilities.c
 * @author Nick Berezny
 * @date 3 Aug 2022
 * @brief 
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

#include "include/Structures.h"
#include "include/Parameters.h"

void MatrixSquare(double A[2][2], double C[2][2])
{

/*------------------------------------------------------------------------
	Squares a 2x2 matrices
		C = A*A
------------------------------------------------------------------------*/

	double B[2][2] = {0};
	memcpy(B, C, 4*sizeof(double));

	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			C[i][j] = (A[i][i]) * (B[i][j]) + A[i][i+1-2*i]*B[i+1-2*i][j];
		}
		
	}

	return;
}

void Factorial(double n, double * ans)
{

/*------------------------------------------------------------------------
	Factorial of number n 
		ans = n!
------------------------------------------------------------------------*/

	for(double i = 0.0; i < n; i++)
		*ans = *ans * (i + 1.0);
	
	return;
}

void DiscretizeMatrix(double A[2][2], double B[2][2])
{

/*------------------------------------------------------------------------
Approximates matrix exponentiation for discrete to continuous conversion
Uses MAT_EXP_ITERATIONS iterations for numerical estimate
	B = exp(A) = I + A + A^2/fact(2) + A^3/fact(3) ...
------------------------------------------------------------------------*/

	
	//B = {0,0,0,0};
	double C[2][2] = {0};
	double D[2][2] = {0};
	double k = 0.0;

	C[0][0] = 0.001*STEP_SIZE_MS* A[0][0];
	C[1][0] = 0.001*STEP_SIZE_MS * A[1][0];
	C[0][1] = 0.001*STEP_SIZE_MS * A[0][1];
	C[1][1] = 0.001*STEP_SIZE_MS* A[1][1];

	memcpy(D, C, 4*sizeof(double));
	memcpy(B, C, 4*sizeof(double));


	for(int i = 1; i<MAT_EXP_ITERATIONS; i++)
	{
		k = 1.0;
		Factorial(i+1, &k);
		MatrixSquare(D, C);
	
		for(int j = 0; j<2; j++) 
		{
			B[j][0] += C[j][0]/k;
			B[j][1] += C[j][1]/k;
		}
	}

	B[0][0] += 1.0; //add unity matrix
	B[1][1] += 1.0;

	return;
}

void InvertMatrix(double A[2][2], double B[2][2])
{

/*------------------------------------------------------------------------
	Invert 2x2 matrix 
		B = A^-1
------------------------------------------------------------------------*/
	//printf("a to invert: %.2f,%.2f,%.2f,%.2f\n", A[0][0], A[0][1], A[1][0], A[1][1]);

	if(A[0][0]*A[1][1] - A[0][1]*A[1][0] == 0)
	{
		printf("Matrix is singular \n");
		return;
	}

	B[0][0] = A[1][1] / ( A[0][0]*A[1][1] - A[0][1]*A[1][0]);
	B[1][0] = -A[1][0] / ( A[0][0]*A[1][1] - A[0][1]*A[1][0]);
	B[0][1] = -A[0][1] / ( A[0][0]*A[1][1] - A[0][1]*A[1][0]);
	B[1][1] = A[0][0] / ( A[0][0]*A[1][1] - A[0][1]*A[1][0]);

	printf("Ainv: %.4f, %.4f, %.4f, %.4f\n", B[0][0], B[0][1], B[1][0], B[1][1]);

	return;
}

void DicretizeInput(double Ad[2][2], double A[2][2], double B[2], double Bd[2])
{

/*------------------------------------------------------------------------
	Approximates discrete B matrix 
		Bd = A^-1 *(Ad - I)*B
------------------------------------------------------------------------*/

	double Ainv[2][2] = {0.0};
	double temp[2][2] = {0.0};

	//printf("a to invert 1: %.2f,%.2f,%.2f,%.2f\n", A[0][0], A[0][1], A[1][0], A[1][1]);

	InvertMatrix(A, Ainv);

	temp[0][0] = Ainv[0][0]*(Ad[0][0] - 1.0) + Ainv[0][1]*Ad[1][0];
	temp[0][1] = Ainv[0][0]*Ad[0][1] + Ainv[0][1]*(Ad[1][1] - 1.0);
	temp[1][0] = Ainv[1][0]*(Ad[0][0] - 1.0) + Ainv[1][1]*(Ad[1][0]);
	temp[1][1] = Ainv[1][0]*Ad[0][1] + Ainv[1][1]*(Ad[1][1] - 1.0);

	 //printf("temp: %.4f, %.4f, %.4f, %.4f\n", temp[0][0], temp[0][1], temp[1][0], temp[1][1]);

	Bd[0] = temp[0][0]*B[0] + temp[0][1]*B[1];
	Bd[1] = temp[1][0]*B[0] + temp[1][1]*B[1];

	return;
}

void FIR_FILTER(double * array, double * output, int * order)
{ 

/*------------------------------------------------------------------------
    Moving average fitler with set order
------------------------------------------------------------------------*/

    //moving average FIR filter of adjustable order

    array[0] = *output;

    for(int i = 1; i < *order; i++)
    {
        *output += array[i];
        array[i] = array[i+1];
    }
    
    array[*order] = array[0];
    //*output += *output;
    *output = *output / (double) *order;
}

void Butterworth10(double * x0, double * y0, double * x, double * y, double * a, double * b)
{
	//3rd order Butterworth, cutoff Freq = 10Hz

	//shift vectors
	for(int i = 3; i > 0; i--)
	{
		x[i] = x[i-1];
		y[i] = y[i-1];
	}

	

	x[0] = *x0;
	y[0] = b[0]*x[0];

	//discrete filter
	for(int i = 1; i < 4; i++)
	{
		y[0] = y[0] + b[i]*x[i] - a[i]*y[i];
	}

	//printf("filter x: %f, %f, %f, %f\n", x[0], x[1], x[2], x[3]);
	//printf("filter y: %f, %f, %f, %f\n", y[0], y[1], y[2], y[3]);

	*y0 = y[0];

	return;

}

void Interpolation(double * t, double * x, double *ti, double *xi, int *index, int n)
{
	int i = (int)(1000*(*ti));

	printf("intep index init: %f, %f\n",t[i], *ti);

	if(t[i] == *ti)
	{
		*xi = x[i];
		return;
	} 
	else if(t[i] > *ti)
	{
		while(t[i] > *ti)
		{
			i = i - 1;
			if(i < 0)
			{
				printf("Interpolation error: time is below lowest time in trajectory vector\n");
				i = 0;
				break;
			}
		}
	}
	else if(t[i] < *ti)
	{
		while(t[i] < *ti)
		{
			i = i + 1;
			if(i > n)
			{
				printf("Interpolation error: time exceeds trajectory vector\n");
				i = n;
				break;
			}
		}
	}

	*xi = x[i];
	*index = i;
	return;

}


void AverageVector(double * x, double * avg, int n)
{
	for(int i = 0; i < n; i++)
	{
		*avg = *avg + x[i];
	}

	*avg = *avg/(double)n;
}