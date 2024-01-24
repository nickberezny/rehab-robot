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

void NoOpDeallocator(void* data, size_t a, void* b) {return;}

void runModel(struct tensorFlowVars * tf)
{
 
    printf("TF test 0\n");
    float data[1*tf->NumInputs];
    
    for(int i = 0; i < tf->NumInputs; i++)
    {
        data[i] = tf->inputVals[i];
    }
    printf("TF test 1\n");   

    int ndims = 2;
    int64_t dims[] = {1,tf->NumInputs};
    int ndata = sizeof(float)*1*tf->NumInputs ;

    TF_Tensor* int_tensor = TF_NewTensor(TF_FLOAT, dims, ndims, data, ndata, &NoOpDeallocator, 0);
    tf->InputValues[0] = int_tensor;
    printf("TF test 2\n");
    //Run the Session
    TF_SessionRun(tf->Session, NULL, tf->Input, tf->InputValues, tf->NumInputs, tf->Output, tf->OutputValues, tf->NumOutputs, NULL, 0,NULL , tf->Status);
    printf("TF test 3\n");
    void* buff = TF_TensorData(tf->OutputValues[0]);
    tf->outputVals = (double*)buff;
    printf("TF test 4\n");
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
    //tf->NumInputs = 1;
    tf->Input = (TF_Output*)malloc(sizeof(TF_Output) * tf->NumInputs);

    TF_Output t0 = {TF_GraphOperationByName(Graph, "serving_default_input_1"), 0};
    if(t0.oper == NULL)
        printf("ERROR: Failed TF_GraphOperationByName serving_default_input_1\n");
    else
    printf("TF_GraphOperationByName serving_default_input_1 is OK\n");
    
    tf->Input[0] = t0;
    
    //********* Get Output tensor
   // tf->NumOutputs = 1;
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
    int64_t dims[] = {1,tf->NumInputs};
    float data[1*tf->NumInputs];

    for(int i = 0; i < tf->NumInputs; i++)
    {
        data[i] = 0.0;
    }
    int ndata = sizeof(float)*1*tf->NumInputs ;// This is tricky, it number of bytes not number of element

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
