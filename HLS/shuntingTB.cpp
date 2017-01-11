#include <stdio.h>
#include <stdlib.h>

#include "shunting.h"

/**
 * Non-Linear Above Threshold Function
 * @param  x : Neuron activity
 * @return   
 */
float UpperBound2(float x){
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
float LowerBound2(float x){
	float output;
	if (x == 4) {
		output = 100;
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
float ExternalInput2(float m){
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
int boundary_check2(int x, int y){
    if (x < 0 || x > (ELEMENTS-1) || y < 0 || y > (ELEMENTS-1)) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * Handle Most of the Path Calculation Task
 * @param c_act : Current neural activity in workspace
 * @param work_info : Workspace information
 */
void ComputeCore_sw(float c_act[LEN][LEN],float work_info[LEN][LEN], float out[LEN][LEN])
{
	float dx;
	for (int p = 0; p < LEN; ++p)
		for (int q = 0; q < LEN; ++q){
			float sum = 0;
			for(int i = -1; i < 2; i++){
				for(int j = -1; j < 2; j++){
					if(boundary_check2(i+p, q+j) == 1){
						if (i == 0 || j == 0) {
							if ((i+j) !=0) {
								sum += UpperBound2(c_act[i+p][q+j]);
							}
						} else {
							sum += 0.707107*UpperBound2(c_act[i+p][j+q]);
						}
					}
				}
			}
			dx = -(10 * c_act[p][q]) + (1 - c_act[p][q])*(UpperBound2(ExternalInput2(work_info[p][q]))
				+ sum) - (1 + c_act[p][q])*LowerBound2(work_info[p][q]);
			out[p][q] = c_act[p][q] + (dx * 0.01);
		}
	return;
}


int main(void)
{
	int err;
	float activity[LEN][LEN]; 
	float workspace_float[LEN][LEN];
	float result_sw[LEN][LEN];
	float result_hw[LEN][LEN];

	int workspace[32][32] ={
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 1, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 7, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 4, 4, 4, 4, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
		};

	/** Matrix Initiation */
	for(int i = 0; i<LEN; i++){
		for(int j = 0; j<LEN; j++){
			activity[i][j] = 0.0;
			workspace_float[i][j] = (float)(workspace[i][j]);
		}
	}

	// prepare data
	AXI_S inp_stream[2*ELEMENTS];
	AXI_S out_stream[ELEMENTS];

	assert(sizeof(float)*8 == 32);
	// stream in the first input  matrix
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
			int k = i*LEN+j;
			inp_stream[k] = stream_out<float,4,5,5>(activity[i][j],0);
		}

	// stream in the second input  matrix
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
			int k = i*LEN+j;
			inp_stream[k+ELEMENTS] = stream_out<float,4,5,5>(workspace_float[i][j],k == (ELEMENTS-1));
		}

	HLS_accel(inp_stream, out_stream);

	// extract the output matrix from the out stream
	for(int i=0; i<LEN; i++)
		for(int j=0; j<LEN; j++)
		{
			int k = i*LEN+j;
			result_hw[i][j] = stream_in<float,4,5,5>(out_stream[k]);
		}


	/* reference software ComputeCore */
	ComputeCore_sw(activity, workspace_float, result_sw);

	/** Result comparison */
	err = 0;
	for (int i = 0; (i<LEN); i++){
		for (int j = 0; (j<LEN); j++){
			printf("%f, ", result_sw[i][j]);
			if (result_sw[i][j] - result_hw[i][j] > 0.000002 || result_sw[i][j] - result_hw[i][j] < -0.000002)
				err++;
		}printf("\n");
	}printf("\n");
	if (err == 0)
		printf("Test successful!\r\n");
	else
		printf("Test failed!\r\n");

	return err;
}


