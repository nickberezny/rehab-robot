/**
 * @file TensorFlow.c
 * @author Nick Berezny
 * @date 22 Jun 2023
 * @brief Initializing and running tensorflow c api
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include "tensorflow/c/c_api.h"
#include <time.h>

#include "./include/Parameters.h"
#include "./include/Structures.h"
#include "./include/Tensorflow.h"

void NoOpDeallocator(void* data, size_t a, void* b) {}

void runModel(struct tensorFlowVars * tf)
{
    clock_t start, end;
    double execution_time;

    start = clock();

    float data[1*10] = {0.0};
  
    data[0]=3.0; 
    data[1]=1.5;
    data[2]=10000.0;
    data[3]=2.0;
    data[4]=0.0;
    data[5]=0.0;
    data[6]=1.0;
    data[7]=20000.0;
    data[8]=1979.0;
    data[9]=1999.0;

    int ndims = 2;
    int64_t dims[] = {1,10};
    int ndata = sizeof(float)*1*10 ;

    TF_Tensor* int_tensor = TF_NewTensor(TF_FLOAT, dims, ndims, data, ndata, &NoOpDeallocator, 0);
    tf->InputValues[0] = int_tensor;

    //Run the Session
    TF_SessionRun(tf->Session, NULL, tf->Input, tf->InputValues, tf->NumInputs, tf->Output, tf->OutputValues, tf->NumOutputs, NULL, 0,NULL , tf->Status);

    void* buff = TF_TensorData(tf->OutputValues[0]);
    float* offsets = (float*)buff;

    end = clock();
    execution_time = ((double)(end - start))/CLOCKS_PER_SEC;
    printf("Execution time (s) %f\n",execution_time);

    printf("%f\n",offsets[0]);

}

void initModel(struct tensorFlowVars * tf)
{
    //********* Read model
    TF_Graph* Graph = TF_NewGraph();
    tf->Status = TF_NewStatus();
    
    TF_SessionOptions* SessionOpts = TF_NewSessionOptions();
    TF_Buffer* RunOpts = NULL;

    const char* saved_model_dir = "./src/test_model/";
    const char* tags = "serve"; // default model serving tag; can change in future
    int ntags = 1;
 
    tf->Session = TF_LoadSessionFromSavedModel(SessionOpts, RunOpts, saved_model_dir, &tags, ntags, Graph, NULL, tf->Status);
    if(TF_GetCode(tf->Status) == TF_OK)
    {
        printf("TF_LoadSessionFromSavedModel OK\n");
    }
    else
    {
        printf("%s",TF_Message(tf->Status));
    }

    //****** Get input tensor
    //TODO : need to use saved_model_cli to read saved_model arch
    tf->NumInputs = 1;
    tf->Input = (TF_Output*)malloc(sizeof(TF_Output) * tf->NumInputs);

    TF_Output t0 = {TF_GraphOperationByName(Graph, "serving_default_input_1"), 0};
    if(t0.oper == NULL)
        printf("ERROR: Failed TF_GraphOperationByName serving_default_input_1\n");
    else
    printf("TF_GraphOperationByName serving_default_input_1 is OK\n");
    
    tf->Input[0] = t0;
    
    //********* Get Output tensor
    tf->NumOutputs = 1;
    tf->Output = (TF_Output*)malloc(sizeof(TF_Output) * tf->NumOutputs);

    TF_Output t2 = {TF_GraphOperationByName(Graph, "StatefulPartitionedCall"), 0};
    if(t2.oper == NULL)
        printf("ERROR: Failed TF_GraphOperationByName StatefulPartitionedCall\n");
    else    
    printf("TF_GraphOperationByName StatefulPartitionedCall is OK\n");
    
    tf->Output[0] = t2;

    //********* Allocate data for inputs & outputs
    tf->InputValues = (TF_Tensor**)malloc(sizeof(TF_Tensor*)*tf->NumInputs);
    tf->OutputValues = (TF_Tensor**)malloc(sizeof(TF_Tensor*)*tf->NumOutputs);

    int ndims = 2;
    int64_t dims[] = {1,10};
    float data[1*10] = {0.0};

    int ndata = sizeof(float)*1*10 ;// This is tricky, it number of bytes not number of element

    TF_Tensor* int_tensor = TF_NewTensor(TF_FLOAT, dims, ndims, data, ndata, &NoOpDeallocator, 0);
    if (int_tensor != NULL)
    {
        printf("TF_NewTensor is OK\n");
    }
    else
    printf("ERROR: Failed TF_NewTensor\n");
    
    tf->InputValues[0] = int_tensor;
    printf("TF_NewTensor is OK\n");
    return;
}
