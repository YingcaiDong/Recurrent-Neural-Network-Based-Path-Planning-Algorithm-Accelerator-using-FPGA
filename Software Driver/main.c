#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#include "xtmrctr.h"
#include "xaxidma.h"
#include "shunting_drive.h"
#include "xil_cache.h"

#define XPAR_AXI_TIMER_DEVICE_ID 		(XPAR_AXI_TIMER_0_DEVICE_ID)

// TIMER Instance
XTmrCtr timer_dev;

// AXI DMA Instance
XAxiDma AxiDma;


int init_dma(){
	XAxiDma_Config *CfgPtr;
	int status;

	CfgPtr = XAxiDma_LookupConfig( (XPAR_AXI_DMA_0_DEVICE_ID) );
	if(!CfgPtr){
		print("Error looking for AXI DMA config\n\r");
		return XST_FAILURE;
	}
	status = XAxiDma_CfgInitialize(&AxiDma,CfgPtr);
	if(status != XST_SUCCESS){
		print("Error initializing DMA\n\r");
		return XST_FAILURE;
	}
	//check for scatter gather mode
	if(XAxiDma_HasSg(&AxiDma)){
		print("Error DMA configured in SG mode\n\r");
		return XST_FAILURE;
	}
	/* Disable interrupts, we use polling mode */
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DEVICE_TO_DMA);
	XAxiDma_IntrDisable(&AxiDma, XAXIDMA_IRQ_ALL_MASK, XAXIDMA_DMA_TO_DEVICE);

	// Reset DMA
	XAxiDma_Reset(&AxiDma);
	while (!XAxiDma_ResetIsDone(&AxiDma)) {}

	return XST_SUCCESS;
}

int boundary_check(int x, int y){
    if (x < 0 || x > (LEN-1) || y < 0 || y > (LEN-1)) {
        return 0;
    } else {
        return 1;
    }
}


int main(int argc, char **argv)
{
	int i, j, k;
	int err=0;
	int status;
	float c_act[LEN][LEN];
	float work_info[LEN][LEN];
	int res_hw[LEN][LEN];
	int res_sw[LEN][LEN];

	int workspace[LEN][LEN] = 
	{		{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,4,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,4,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,4,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,4,4,4,0,0,0,0,4,0,0,0,0,4,4,4,0,4,4,4,4,4,4,4,4,4,0,0,4},
			{4,0,0,0,0,4,4,0,0,0,0,4,0,0,0,0,4,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,4,4,0,0,0,4,0,0,0,0,4,0,4,4,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,4,4,0,0,0,4,0,0,0,0,4,0,4,1,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,4,4,0,0,4,0,0,0,0,4,0,4,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,4,4,0,0,4,0,0,0,0,4,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,4,4,0,4,4,4,4,4,4,4,4,4,4,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,4,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,4,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,4,4,0,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,4,0,7,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,4,4,0,0,0,4,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
			{4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4}};

	unsigned int dma_size = SIZE * sizeof(float);

    float acc_factor;
	unsigned int init_time, curr_time, calibration;
	unsigned int begin_time;
	unsigned int end_time;
	unsigned int run_time_sw = 0;
	unsigned int run_time_hw = 0;

	init_platform();

	xil_printf("\r ***************************************************************\n\r");
	xil_printf("\r 32x32 MATRIX workspace shunting model path planning -> AXI DMA -> ARM ACP \n\r");
	xil_printf("\r ***************************************************************\n\n\r");

	/* ******************************************************************************* */
	// Init DMA
	status = init_dma();
	if(status != XST_SUCCESS){
		print("\rError: DMA init failed\n");
		return XST_FAILURE;
	}
	print("\r\nDMA Init done\n\r");

	/* ******************************************************************************* */
	// Setup HW timer
	status = XTmrCtr_Initialize(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	if(status != XST_SUCCESS)
	{
		print("\rError: timer setup failed\n");
		//return XST_FAILURE;
	}
	XTmrCtr_SetOptions(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID, XTC_ENABLE_ALL_OPTION);

	// Calibrate HW timer
	XTmrCtr_Reset(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	init_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	curr_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	calibration = curr_time - init_time;

	// Loop measurement
	XTmrCtr_Reset(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	begin_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	end_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	run_time_sw = end_time - begin_time - calibration;
	xil_printf("\rLoop 1 time is %d cycles.\r\n", run_time_sw);

	/* ******************************************************************************* */
	// input data Initiation
	 for(i = 0; i<LEN; i++)
	 	for(j = 0; j<LEN; j++)
	 	{
	 		res_sw[i][j] = 0;
	 		res_hw[i][j] = 0;
	 	}
	/** End of Initiation */
	/* ******************************************************************************* */
	// call the software version of the function
	xil_printf("\rRunning shunting model in SW\n");
	XTmrCtr_Reset(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	begin_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	
	generatePath(workspace,res_sw);

	end_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	run_time_sw = end_time - begin_time - calibration;
	xil_printf("\r\nTotal run time for SW on Processor is %d cycles.\r\n",
			run_time_sw);

	/* ******************************************************************************* */
	// call the HW accelerator
	XTmrCtr_Reset(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	begin_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	// Setup the HW Accelerator
	int p,q;
	int r_x, r_y, t_x, t_y, s_x, s_y;
	for (p = 0;p < 32;p++) {
		for (q = 0;q < 32;q++) {
			if (workspace[p][q] == 7) {
				r_x = p; r_y = q; s_x = p; s_y = q;
			}
			if (workspace[p][q] == 1) {
				t_x = p; t_y = q;
			}
			c_act[p][q] = 0;
			work_info[p][q] = (float)workspace[p][q];
		}
	}
	Setup_HW_Accelerator(c_act, work_info, c_act, dma_size);

	while(r_x != t_x || r_y != t_y) {
		int speed;
		for(speed = 0; speed < SPEED; speed++){
			Start_HW_Accelerator();
			Run_HW_Accelerator(c_act, work_info, c_act, dma_size);}
		Xil_DCacheFlushRange((unsigned int)c_act,dma_size);

		float max = c_act[r_x][r_y];
		int temp_x = r_x; int temp_y = r_y;
		for(p = -1; p < 2; p++){
			for(q = -1; q < 2; q++){
				if(boundary_check(r_x+p, r_y+q)){
					if(c_act[r_x+p][r_y+q] > max){
						max = c_act[r_x+p][r_y+q];
						temp_x = r_x+p; temp_y = r_y+q;
					}
				}
			}
		}
		r_x = temp_x; r_y = temp_y;
		res_hw[temp_x][temp_y] = 7;

	}
	for (p = 0; p < LEN; p += 1) {
		for (q = 0; q < LEN; q += 1) {
			if(workspace[p][q] == 4){
				res_hw[p][q] = workspace[p][q];
			}
		}
	}
	res_hw[s_x][s_y] = 9;
	res_hw[t_x][t_y] = 1;
	end_time = XTmrCtr_GetValue(&timer_dev, XPAR_AXI_TIMER_DEVICE_ID);
	run_time_hw = end_time - begin_time - calibration;
	xil_printf(
			"\rTotal run time for AXI DMA + HW accelerator is %d cycles.\r\n",
			run_time_hw);

	/* ******************************************************************************* */
	//Compare the results from sw and hw

	for (i = 0; i < LEN; i++)
		for (j = 0; j < LEN; j++)
			if (res_hw[i][j] != res_sw[i][j]) {
				err += 1;
				printf("\nposition: x = %d, y = %d\n", i, j);
			}

	// HW vs. SW speedup factor
	acc_factor = (float) run_time_sw / (float) run_time_hw;
	xil_printf("Acceleration factor: %d.%d\n\n",
			(int) acc_factor, (int) (acc_factor * 1000) % 1000);

	if (err == 0){
		print("\rSW and HW results match!\n\r");
	}else{
		printf("ERROR: results mismatch %d\n", err);
	}

	int last_i, last_j;

	printf("\nres_sw = \n");
	for(last_i = 0; last_i < LEN; last_i++)
	{
		for(last_j = 0; last_j < LEN; last_j++)
		{
			printf("%d",res_sw[last_i][last_j]);
		}
		printf("\n");
	}
	printf("\n\n");

	printf("\nres_hw = \n");
	for(last_i = 0; last_i < LEN; last_i++)
	{
		for(last_j = 0; last_j < LEN; last_j++)
		{
			printf("%d",res_hw[last_i][last_j]);
		}
		printf("\n");
	}
	printf("\n\n");
    cleanup_platform();
    return 0;
}

