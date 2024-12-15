# PipelinedProcessor Details:
### pipelineCPU.sv: Top-level module that instantiates all the necessary submodules and pipeline registers to generate the 5 stages of the pipeline.
### cntrl_main.sv: Slightly altered version of the control logic module from the Single Cycle Processor which removes BrTaken, Reg2Loc, and UncondBr.
### PC_control.sv: Handles the the BrTaken, Reg2Loc, and UncondBr control logic in a seperate module that occurs prior to the cntrl_main logic in the pipeline to quickly update PC.
### forwarding_unit.sv: Detects data forwarding hazards between Memory and Execute stages of the pipeline and will forward data via two MUXs to the ALU.
### zero_equal.sv: Used by CBZ ARM Assembly operations to quickly check the Branch status required if the ouput of the ALU is zero.

## ! All other files are adopted from RegFile and ALU projects!
