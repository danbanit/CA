/* 046267 Computer Architecture - Spring 2017 - HW #1 */
/* This file should hold your implementation of the CPU pipeline core simulator */

#include "sim_api.h"

// Global variables
bool forwarding;
bool split_reg;


static SIM_coreState scs;
static int ALU_res = 0; //saving the result of the ALU at the MEM latch
static int MEM_in = 0; //saving the input of the memory stage (from the MEM latch, after tick)
static int MEM_out = 0; //saving the output of the MEM stage at the WB latch
static int WB_in = 0; //saving the input of the WB stage (from the WB latch, after tick)
static int DECODED_dst[2] = { 0 }; // saving the values of the dest the DECODE and EXECUTE stages respectively.
static bool is_branch  = false; // a flag indicating an identification of a branch cmd at the memory stage.
static bool mem_dly    = false; // a flag indicating the MEM stage hasn't finished reading from the memory.  


void flush(pipeStage stage)
{
	scs.pipeStageState[stage].cmd.src1 = 0;
	scs.pipeStageState[stage].cmd.src2 = 0;
	scs.pipeStageState[stage].cmd.dst = 0;
	scs.pipeStageState[stage].cmd.isSrc2Imm = false;
	scs.pipeStageState[stage].cmd.opcode = CMD_NOP;
	scs.pipeStageState[stage].src1Val = 0;
	scs.pipeStageState[stage].src2Val = 0;
}


/*! SIM_CoreReset: Reset the processor core simulator machine to start new simulation
  Use this API to initialize the processor core simulator's data structures.
  The simulator machine must complete this call with these requirements met:
  - PC = 0  (entry point for a program is at address 0)
  - All the register file is cleared (all registers hold 0)
  - The value of IF is the instuction in address 0x0
  \returns 0 on success. <0 in case of initialization failure.
*/
int SIM_CoreReset(void) {
	int i;
	for (i = 0; i < SIM_REGFILE_SIZE; i++)
	{
		scs.regFile[i] = 0;
	}
	for (i = 0; i < SIM_PIPELINE_DEPTH; i++)
	{
		flush(i);
	}
	scs.pc = 0;
	SIM_MemInstRead(scs.pc,&scs.pipeStageState[FETCH].cmd);
	return 0;
}


bool RAWHazard_aux(SIM_cmd dec_cmd, SIM_cmd prev_cmd) // auxilary func for detecting RAW Hazards
{
	int prev_dest = prev_cmd.dst;
	bool fwd_cond = (forwarding && prev_cmd.opcode == CMD_LOAD);
	bool non_fwd_cond = ((!forwarding) && (prev_cmd.opcode == CMD_ADD || prev_cmd.opcode == CMD_SUB || prev_cmd.opcode == CMD_LOAD));
	
	if (fwd_cond || non_fwd_cond)
	{
		switch (dec_cmd.opcode)
		{
			case CMD_ADD:
			{
				if (dec_cmd.src1 == prev_dest || dec_cmd.src2==prev_dest) return true;
				break;
			}
			case CMD_SUB:
			{
				if (dec_cmd.src1 == prev_dest || dec_cmd.src2 == prev_dest) return true;
				break;
			}
			case CMD_LOAD:
			{
				if (dec_cmd.isSrc2Imm)
				{
					if (dec_cmd.src1 == prev_dest) return true;
					break;
				}
				else if ((dec_cmd.src1 == prev_dest) || dec_cmd.src2 == prev_dest) return true;
				break;
			}
			case CMD_STORE:
			{
				if (dec_cmd.isSrc2Imm)
				{
					if (dec_cmd.src1 == prev_dest || dec_cmd.dst == prev_dest) return true;
					break;
				}
				else if ((dec_cmd.src1 == prev_dest) || dec_cmd.src2 == prev_dest || dec_cmd.dst == prev_dest) return true;
				break;
			}
			case CMD_BR:
			{
				if (dec_cmd.dst == prev_dest) return true;
			}
			case CMD_BREQ:
			{
				if ((dec_cmd.src1 == prev_dest) || dec_cmd.src2 == prev_dest || dec_cmd.dst == prev_dest) return true;
			}
			case CMD_BRNEQ:
			{
				if ((dec_cmd.src1 == prev_dest) || dec_cmd.src2 == prev_dest || dec_cmd.dst == prev_dest) return true;
			}
		}
	}
	return false;
}

bool isBubble()
{
	SIM_cmd wb_cmd = scs.pipeStageState[WRITEBACK].cmd;
	SIM_cmd mem_cmd = scs.pipeStageState[MEMORY].cmd;
	SIM_cmd exe_cmd = scs.pipeStageState[EXECUTE].cmd;
	SIM_cmd dec_cmd = scs.pipeStageState[DECODE].cmd;

	
	if (forwarding)
	{
		if (RAWHazard_aux(dec_cmd, exe_cmd)) return true;
	}
	else if (split_reg)
	{
		if (RAWHazard_aux(dec_cmd, mem_cmd) || RAWHazard_aux(dec_cmd, exe_cmd)) return true;
	}
	else
	{
		if (RAWHazard_aux(dec_cmd, wb_cmd) || RAWHazard_aux(dec_cmd, mem_cmd) || RAWHazard_aux(dec_cmd, exe_cmd)) return true;
	}
	return false;
}

void MemStage()
{
	switch (scs.pipeStageState[MEMORY].cmd.opcode)
	{
		case CMD_LOAD:
		{
			if (!SIM_MemDataRead(ALU_res, &MEM_out)) mem_dly = true;
			else mem_dly = false;
			break;
		}
		case CMD_BR:
		case CMD_BREQ:
		case CMD_BRNEQ:
		{
			if (is_branch) //flush the pipe and update pc
			{
				flush(EXECUTE);
				flush(DECODE);
				flush(FETCH);
				scs.pc = MEM_in;
			}
			break;
		}
		case CMD_STORE:
		{
			SIM_MemDataWrite(MEM_in, scs.pipeStageState[MEMORY].src1Val);
			break;
		}

		default: MEM_out = MEM_in;
	}

	if (!is_branch) scs.pc += 4;
	is_branch = false;
}

void HDU()
{
	SIM_cmd_opcode mem_opcode = scs.pipeStageState[MEMORY].cmd.opcode;
	SIM_cmd_opcode wb_opcode = scs.pipeStageState[WRITEBACK].cmd.opcode;
	bool hdu_mem_cond = (mem_opcode == CMD_ADD || mem_opcode == CMD_SUB);
	bool hdu_wb_cond = (wb_opcode == CMD_ADD || wb_opcode == CMD_SUB || wb_opcode == CMD_LOAD);
	int mem_dst = scs.pipeStageState[MEMORY].cmd.dst;
	int wb_dst = scs.pipeStageState[WRITEBACK].cmd.dst;
	int exe_src1 = scs.pipeStageState[EXECUTE].cmd.src1;
	int exe_src2 = scs.pipeStageState[EXECUTE].cmd.src2;
	int exe_dst = scs.pipeStageState[EXECUTE].cmd.dst;
	
	if (hdu_mem_cond || hdu_wb_cond)
	{
		// if mem and wb has hazard for ex stage in one of the srcs
		if (mem_dst == exe_src1)
			scs.pipeStageState[EXECUTE].src1Val = ALU_res;
		else if (wb_dst == exe_src1)
			scs.pipeStageState[EXECUTE].src1Val = MEM_out;
		if (mem_dst == exe_src2)
			scs.pipeStageState[EXECUTE].src2Val = ALU_res;
		else if (mem_dst == exe_src2)
			scs.pipeStageState[EXECUTE].src2Val = MEM_out;

		if (mem_dst == exe_dst)
			DECODED_dst[1] = ALU_res;
		else if (wb_dst == exe_src1)
			DECODED_dst[1] = MEM_out;
	}
}

void ExeStage()
{
	if (forwarding) HDU();
	switch (scs.pipeStageState[EXECUTE].cmd.opcode)
	{
		case CMD_NOP:
		{
			ALU_res = 0;
			break;
		}
		case CMD_ADD:
		{
			ALU_res = scs.pipeStageState[EXECUTE].src1Val + scs.pipeStageState[EXECUTE].src2Val;
			break;
		}

		case CMD_SUB:
		{
			ALU_res = scs.pipeStageState[EXECUTE].src1Val - scs.pipeStageState[EXECUTE].src2Val;
			break;
		}
		case CMD_LOAD:
		{
			ALU_res = scs.pipeStageState[EXECUTE].src1Val + scs.pipeStageState[EXECUTE].src2Val;
			break;
		}
		case CMD_STORE:
		{
			ALU_res = DECODED_dst[1] + scs.pipeStageState[EXECUTE].src2Val;
			break;
		}
		case CMD_BR:
		{
			is_branch = true;
			ALU_res = DECODED_dst[1] + scs.pc - 4;
			break;
		}
		case CMD_BREQ:
		{
			if (scs.pipeStageState[EXECUTE].src1Val == scs.pipeStageState[EXECUTE].src2Val) is_branch = true; // if src1==src2
			ALU_res = DECODED_dst[1] + scs.pc - 4;
			break;
		}
		case CMD_BRNEQ:
		{
			if (scs.pipeStageState[EXECUTE].src1Val != scs.pipeStageState[EXECUTE].src2Val) is_branch = true; // if src1!=src2
			ALU_res = DECODED_dst[1] + scs.pc - 4;
			break;
		}
		case CMD_HALT:
		{
			ALU_res = 0;
			break;
		}
		default:
		{
			ALU_res = 0;
			break;
		}
	}
}

void DecStage()
{
	int src1_addr = scs.pipeStageState[FETCH].cmd.src1;
	int src2_addr = scs.pipeStageState[FETCH].cmd.src2;
	int dst_addr = scs.pipeStageState[DECODE].cmd.dst;

	scs.pipeStageState[FETCH].src1Val = scs.regFile[src1_addr];
	DECODED_dst[0] = scs.regFile[dst_addr];
	if (!scs.pipeStageState[FETCH].cmd.isSrc2Imm)
	{
		scs.pipeStageState[FETCH].src2Val = scs.regFile[src2_addr];
	}
	else scs.pipeStageState[FETCH].src2Val = scs.pipeStageState[FETCH].cmd.src2;
}

void WbStage()
{
	switch (scs.pipeStageState[WRITEBACK].cmd.opcode)
	{
		case CMD_ADD:
		case CMD_SUB:
		case CMD_LOAD:
		{
			scs.regFile[scs.pipeStageState[WRITEBACK].cmd.dst] = WB_in;
			break;
		}
		default: break;
	}
}

void UpdtPipe()
{

	int i;
	bool is_bubble = isBubble();

	for (i = SIM_PIPELINE_DEPTH - 1; i > 0; i--) 	// Update pipe
	{
		if (i == 2 && is_bubble)
		{
			flush(i);
			break;
		}
		memcpy(&scs.pipeStageState[i] , &scs.pipeStageState[i - 1],sizeof(scs.pipeStageState[i]));
	}

	DECODED_dst[1] = DECODED_dst[0]; // moving the decoded destination to the execution stage

	// shifting the data outputs to the next stages' inputs
	MEM_in = ALU_res;
	WB_in  = MEM_out;

}

void FetStage()
{
	SIM_MemInstRead(scs.pc, &scs.pipeStageState[FETCH].cmd);
	scs.pipeStageState[FETCH].src1Val = 0;
	scs.pipeStageState[FETCH].src2Val = 0;
}


/*! SIM_CoreClkTick: Update the core simulator's state given one clock cycle.
  This function is expected to update the core pipeline given a clock cycle event.
*/
void SIM_CoreClkTick() {
	

	MemStage(); // MEMORY
	if (!mem_dly)
	{
		ExeStage(); // EXECUTION
		if (split_reg || forwarding)
		{
			WbStage();  // WRITEBACK 
			DecStage(); // DECODE
		}
		else
		{
			DecStage(); // DECODE
			WbStage();  // WRITEBACK
		}
		UpdtPipe(); // Update the pipeline
		FetStage(); // FETCH
	}
	else WbStage();  // WRITEBACK
}

/*! SIM_CoreGetState: Return the current core (pipeline) internal state
    curState: The returned current pipeline state
    The function will return the state of the pipe at the end of a cycle
*/
void SIM_CoreGetState(SIM_coreState *curState) {

	memcpy(curState, &scs, sizeof(scs));
}

