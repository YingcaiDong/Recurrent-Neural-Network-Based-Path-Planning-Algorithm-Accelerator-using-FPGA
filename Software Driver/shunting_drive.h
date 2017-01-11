
#ifndef SHUNTING_DIVE_H
#define SHUNTING_DIVE_H

#define LEN    32
#define SIZE  ((LEN)*(LEN))
#define SPEED 10

int Setup_HW_Accelerator(float c_act[LEN][LEN],float work_info[LEN][LEN], float res_hw[LEN][LEN], int dma_size);

int Run_HW_Accelerator(float c_act[LEN][LEN], float work_info[LEN][LEN], float res_hw[LEN][LEN], int dma_size);

void Start_HW_Accelerator();

void generatePath(int input_workspace[LEN][LEN],int SWRESULT[LEN][LEN]);

#endif
