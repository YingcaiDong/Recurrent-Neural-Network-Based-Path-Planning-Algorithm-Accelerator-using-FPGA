#include <stdio.h>
#include <stdlib.h>
#include "platform.h"
#include "xparameters.h"
#include "xscugic.h"
#include "xaxidma.h"
#include "xhls_accel.h"
#include "shunting_drive.h"
#include "xil_printf.h"


volatile static int RunExample = 0;
volatile static int ResultExample = 0;

XHls_accel shunting_dev;

XHls_accel_Config shunting_config = {
	0,
	XPAR_HLS_ACCEL_0_S_AXI_CONTROL_BUS_BASEADDR
};

//Interrupt Controller Instance
XScuGic ScuGic;

// AXI DMA Instance
extern XAxiDma AxiDma;


int XMmultSetup(){
	return XHls_accel_CfgInitialize(&shunting_dev,&shunting_config);
}

void XShuntingStart(void *InstancePtr){
	XHls_accel *pExample = (XHls_accel *)InstancePtr;
	XHls_accel_InterruptEnable(pExample,1);
	XHls_accel_InterruptGlobalEnable(pExample);
	XHls_accel_Start(pExample);
}


void XShuntingIsr(void *InstancePtr){
	XHls_accel *pExample = (XHls_accel *)InstancePtr;

	//Disable the global interrupt
	XHls_accel_InterruptGlobalDisable(pExample);
	//Disable the local interrupt
	XHls_accel_InterruptDisable(pExample,0xffffffff);

	// clear the local interrupt
	XHls_accel_InterruptClear(pExample,1);

	ResultExample = 1;
	// restart the core if it should run again
	if(RunExample){
		XShuntingStart(pExample);
	}
}

int XShuntingSetupInterrupt()
{
	//This functions sets up the interrupt on the ARM
	int result;
	XScuGic_Config *pCfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (pCfg == NULL){
		print("Interrupt Configuration Lookup Failed\n\r");
		return XST_FAILURE;
	}
	result = XScuGic_CfgInitialize(&ScuGic,pCfg,pCfg->CpuBaseAddress);
	if(result != XST_SUCCESS){
		return result;
	}
	// self test
	result = XScuGic_SelfTest(&ScuGic);
	if(result != XST_SUCCESS){
		return result;
	}
	// Initialize the exception handler
	Xil_ExceptionInit();
	// Register the exception handler
	//print("Register the exception handler\n\r");
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler,&ScuGic);
	//Enable the exception handler
	Xil_ExceptionEnable();
	// Connect the Adder ISR to the exception table
	//print("Connect the Adder ISR to the Exception handler table\n\r");
	result = XScuGic_Connect(&ScuGic,XPAR_FABRIC_HLS_ACCEL_0_INTERRUPT_INTR,(Xil_InterruptHandler)XShuntingIsr,&shunting_dev);
	if(result != XST_SUCCESS){
		return result;
	}
	//print("Enable the Adder ISR\n\r");
	XScuGic_Enable(&ScuGic,XPAR_FABRIC_HLS_ACCEL_0_INTERRUPT_INTR);
	return XST_SUCCESS;
}

int Setup_HW_Accelerator(float c_act[LEN][LEN], float work_info[LEN][LEN], float res_hw[LEN][LEN], int dma_size)
//Setup the Vivado HLS Block
{
	int status = XMmultSetup();
	if(status != XST_SUCCESS){
		print("Error: example setup failed\n");
		return XST_FAILURE;
	}
	status =  XShuntingSetupInterrupt();
	if(status != XST_SUCCESS){
		print("Error: interrupt setup failed\n");
		return XST_FAILURE;
	}

	XShuntingStart(&shunting_dev);

	//flush the cache
	Xil_DCacheFlushRange((unsigned int)c_act,dma_size);
	Xil_DCacheFlushRange((unsigned int)work_info,dma_size);
	Xil_DCacheFlushRange((unsigned int)res_hw,dma_size);

	return 0;
}

//==================================================================================
//===================code for software result part1
float UpperBound2(float x){
	float output;
	if (x > 0) {
		output = x;
	}else{
		output = 0;
	}
	return output;
}


float LowerBound2(float x){
	float output;
	if (x == 4) {
		output = 100;
	}else{
		output = 0;
	}
	return output;
}

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

int boundary_check2(int x, int y){
    if (x < 0 || x > (LEN-1) || y < 0 || y > (LEN-1)) {
        return 0;
    } else {
        return 1;
    }
}

void shunting_sf_ref(float a[LEN][LEN], float b[LEN][LEN]) {
	int p,q,i,j;
	float dx = 0;
	for (p = 0; p < LEN; ++p){
		for (q = 0; q < LEN; ++q){
			float sum = 0;
			for(i = -1; i < 2; ++i){
				for(j = -1; j < 2; ++j){
					if(boundary_check2(i+p, q+j) == 1){
						if (i == 0 || j == 0) {
							if ((i+j) !=0) {
								sum += UpperBound2(a[i+p][q+j]);
							}
						} else {
							sum += 0.707107*UpperBound2(a[i+p][j+q]);
						}
					}
				}
			}
			dx = -(10 * a[p][q]) + (1 - a[p][q])*(UpperBound2(ExternalInput2(b[p][q]))
				+ sum) - (1 + a[p][q])*LowerBound2(b[p][q]);
			a[p][q] += (dx * 0.01);
		}
	}
return;
}
//===================code for software result part1 ends here
//==================================================================================

void Start_HW_Accelerator()
{
	int status = XMmultSetup();
	if(status != XST_SUCCESS){
		print("Error: example setup failed\n");
		return XST_FAILURE;
	}
	status =  XShuntingSetupInterrupt();
	if(status != XST_SUCCESS){
		print("Error: interrupt setup failed\n");
		return XST_FAILURE;
	}

	XShuntingStart(&shunting_dev);
}

int Run_HW_Accelerator(float c_act[LEN][LEN], float work_info[LEN][LEN], float res_hw[LEN][LEN], int dma_size)
{
	//transfer c_act to the Vivado HLS block
	int status = XAxiDma_SimpleTransfer(&AxiDma, (unsigned int) c_act, dma_size, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		//print("Error: DMA transfer to Vivado HLS block failed\n");
		return XST_FAILURE;
	}
	/* Wait for transfer to be done */
	while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) ;

	status = XAxiDma_SimpleTransfer(&AxiDma, (unsigned int) work_info, dma_size, XAXIDMA_DMA_TO_DEVICE);
	if (status != XST_SUCCESS) {
		//print("Error: DMA transfer to Vivado HLS block failed\n");
		return XST_FAILURE;
	}
	/* Wait for transfer to be done */
	while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) ;

	//get results from the Vivado HLS block
	status = XAxiDma_SimpleTransfer(&AxiDma, (unsigned int) res_hw, dma_size,
			XAXIDMA_DEVICE_TO_DMA);
	if (status != XST_SUCCESS) {
		//print("Error: DMA transfer from Vivado HLS block failed\n");
		return XST_FAILURE;
	}
	/* Wait for transfer to be done */
	while (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE)) ;

	//poll the DMA engine to verify transfers are complete
	/* Waiting for data processing */
	/* While this wait operation, the following action would be done
	 * First: Second matrix will be sent.
	 * After: Multiplication will be compute.
	 * Then: Output matrix will be sent from the accelerator to DDR and
	 * it will be stored at the base address that you set in the first SimpleTransfer
	 */
	while ((XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA)) || (XAxiDma_Busy(&AxiDma, XAXIDMA_DMA_TO_DEVICE))) ;

	return 0;
}

//==================================================================================
//===================code for software result part2
void generatePath(int input_workspace[LEN][LEN],int SWRESULT[LEN][LEN]){
	// init HW_c_neural_activity
	float c_activity[LEN][LEN], fworkspace[LEN][LEN];
	int r_x, r_y, t_x, t_y, s_x, s_y;
	int i,j;
	for (i = 0;i < LEN;i++) {
		for (j = 0;j < LEN;j++) {
			if (input_workspace[i][j] == 7) {
				r_x = i;
				r_y = j;
				s_x = i;
				s_y = j;
			}
			if (input_workspace[i][j] == 1) {
				t_x = i;
				t_y = j;
			}
		}
	}
	for(i = 0; i < LEN; i++) {
		for(j = 0; j < LEN; j++){
			c_activity[i][j] = 0;
			fworkspace[i][j] = (float)input_workspace[i][j];
		}
	}
	while (r_x != t_x || r_y != t_y) {
		// Caculate all the neurals in the work space
		int speed;
		for(speed = 0; speed < SPEED; speed++){
			shunting_sf_ref(c_activity, fworkspace);}
		// update neural activity
		float max = c_activity[r_x][r_y];
		int tempX = r_x; int tempY = r_y;
		int i,j;
        for (i = -1;i < 2; i++) {
            for (j = -1;j < 2; j++) {
                if (boundary_check2(r_x+i, r_y+j)) {
                    if (c_activity[r_x+i][r_y+j] > max) {
                        max = c_activity[r_x+i][r_y+j];
                        tempX = r_x+i; tempY = r_y+j;
                    }
                }
            }
        }
        r_x = tempX; r_y = tempY;
        SWRESULT[r_x][r_y] = 7;

	}
	int s_i,s_j;
		for (s_i = 0; s_i < LEN; s_i += 1) {
			for (s_j = 0; s_j < LEN; s_j += 1) {
				if(input_workspace[s_i][s_j]==4){
					SWRESULT[s_i][s_j] = input_workspace[s_i][s_j];
				}
			}
		}
		SWRESULT[s_x][s_y] = 9;
		SWRESULT[t_x][t_y] = 1;
	return;
}
