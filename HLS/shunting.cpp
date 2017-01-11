#include <stdio.h>
#include <stdlib.h>

#include "shunting.h"

/**
 * Non-Linear Above Threshold Function
 * @param  x : Neuron activity
 * @return   
 */
float UpperBound(float x){
#pragma HLS INLINE
	float output;
	if (x > 0) {
		output = x;
	}else{
		output = 0;
	}
	return output;
}

/**
 * Non-linear Below Threshold Function
 * @param  x : Target/obstacle representive by number
 * @return   
 */
float LowerBound(float x){
#pragma HLS INLINE
	float output;
	if (x == 4) {
		output = 100;
	}else{
		output = 0;
	}
	return output;
}

/**
 * External Input to i-th Neuron
 * @param  i         : Vertical position
 * @param  j         : Horizontal position
 * @param  workspace : Workspace representived by matrix containing obscacle/target information
 * @return           
 */
float ExternalInput(float m){
#pragma HLS INLINE
    int output;
    if (m == 0) {
        output = 0;
    }else if(m == 7) {
    	output = 0;
    }else if (m == 4){
        output = -100;
    }else if (m == 1){
        output = 100;
    }
    return output;
}

/**
 * Check Invalid Input
 * @param  x : Vertical position
 * @param  y : Horizontal position
 * @return   : Input valid return 1, otherwise 0
 */
int boundary_check(int x, int y){
#pragma HLS INLINE
    if (x < 0 || x > (ELEMENTS-1) || y < 0 || y > (ELEMENTS-1)) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * Handle Most of the Path Calculation Task
 * @param c_act   : Current neural activity in workspace
 * @param work_info   : Workspace information
 * @param out : Updated neural activity in workspace
 */
void ComputeCore_hw(float c_act[LEN][LEN], float work_info[LEN][LEN], float out[LEN][LEN])
{
#pragma HLS INLINE

	float dx;
	for (int p = 0; p < LEN; ++p)
		for (int q = 0; q < LEN; ++q){
#pragma HLS PIPELINE II=2
			float sum = 0;
			for(int i = -1; i < 2; i++){
				for(int j = -1; j < 2; j++){
#pragma HLS PIPELINE II=2
					if(boundary_check(i+p, q+j) == 1){
						if (i == 0 || j == 0) {
							if ((i+j) !=0) {
								sum += UpperBound(c_act[i+p][q+j]);
							}
						} else {
							sum += 0.707107*UpperBound(c_act[i+p][j+q]);
						}
					}
				}
			}
			dx = -(10 * c_act[p][q]) + (1 - c_act[p][q])*(UpperBound(ExternalInput(work_info[p][q]))
				+ sum) - (1 + c_act[p][q])*LowerBound(work_info[p][q]);
			out[p][q] = c_act[p][q] + (dx * 0.01);
		}
	return;
}

/**
 * Packing data&Converting data
 * @param in_stream  : Input data stream
 * @param out_stream : Output data stream
 */
void wrapped_ComputCore_hw (
	AXI_S in_stream[2*ELEMENTS],
	AXI_S out_stream[ELEMENTS])
{
#pragma HLS INLINE

	float c_act[LEN][LEN];
	float work_info[LEN][LEN];
	float out[LEN][LEN];

	assert(sizeof(float)*8 == 32);

	// stream in first matrix
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
#pragma HLS PIPELINE II=1
			int k = i*LEN+j;
			c_act[i][j] = stream_in<float,4,5,5>(in_stream[k]);
		}

	// stream in second matrix
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
#pragma HLS PIPELINE II=1
			int k = i*LEN+j+ELEMENTS;
			work_info[i][j] = stream_in<float,4,5,5>(in_stream[k]);
		}

	// do HW Computation
	ComputeCore_hw(c_act,work_info,out);

	// stream out result matrix
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
#pragma HLS PIPELINE II=1
			int k = i*LEN+j;
			out_stream[k] = stream_out<float,4,5,5>(out[i][j],k == (ELEMENTS-1));
		}
	return;
}

/**
 * Top Function for HLS
 * @param INPUT_STREAM  : Input stream
 * @param OUTPUT_STREAM : Output stream
 */
void HLS_accel (AXI_S INPUT_STREAM[2*ELEMENTS], AXI_S OUTPUT_STREAM[ELEMENTS])
{
#pragma HLS INTERFACE s_axilite port=return     bundle=CONTROL_BUS
#pragma HLS INTERFACE axis      port=OUTPUT_STREAM
#pragma HLS INTERFACE axis      port=INPUT_STREAM

	wrapped_ComputCore_hw (INPUT_STREAM, OUTPUT_STREAM);

	return;
}

