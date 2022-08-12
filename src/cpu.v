//CPU FINAL


`include "datamem.v"
`include "datacache.v"
`include "inst_mem.v"
`include "inst_cache.v"

`timescale  1ns/100ps


// Test bench for the cpu
module stimulus;

    wire [31:0] PC;
    wire [31:0] INST;
    //reg [7:0] INST_MEM[1023:0];
    reg CLK, RESET;
    integer i;
    
    wire [7:0] ADDRESS, WRITEDATA, READDATA;
    wire [0:0] READ, WRITE, BUSYWAIT;
    
    wire [31:0] MEM_READDATA, MEM_WRITEDATA;
    wire [5:0] MEM_ADDRESS, inst_mem_address;
    wire [0:0] MEM_READ, MEM_WRITE;
    wire [0:0] MEM_BUSYWAIT;
    
    wire [0:0]inst_mem_read, inst_mem_busywait, inst_busywait;
    wire [127:0] inst_mem_readinst;
    

    //CPU
    cpu MY_CPU(PC, INST, CLK, RESET, ADDRESS, WRITEDATA, READ, WRITE, BUSYWAIT, inst_busywait, READDATA);///**********************NEW-->HANDLE INST_BUSYWAIT
    
    //DATA CACHE
    dcache DC(CLK, RESET, BUSYWAIT, READ, WRITE, WRITEDATA, READDATA, ADDRESS, MEM_BUSYWAIT, MEM_READ, MEM_WRITE, MEM_WRITEDATA, MEM_READDATA, MEM_ADDRESS);
    
    //DATA MEMORY
    data_memory DM(CLK, RESET, MEM_READ, MEM_WRITE, MEM_ADDRESS, MEM_WRITEDATA, MEM_READDATA, MEM_BUSYWAIT);
    
    //INSTRUCTION CACHE
    instcache IC(inst_busywait, INST, PC[9:0], inst_mem_busywait, inst_mem_read, inst_mem_readinst, inst_mem_address);
    
    //INSTRUCTION MEMORY
    instruction_memory IM(CLK, inst_mem_read, inst_mem_address, inst_mem_readinst, inst_mem_busywait);
    
    
/*
	// To create the proper instruction from the bytes read from the .mem file
    always @(PC)
    begin
    	// Delay for the loading of the instruction
        INST = #2 {INST_MEM[PC+3],INST_MEM[PC+2],INST_MEM[PC+1],INST_MEM[PC]}; 
    end
*/

	
	initial //to generate the clock pulses
    begin
        CLK = 1'b0;
        forever #4 CLK = ~CLK;
    end


	initial // To reset the cpu at the begining
    begin
        RESET = 1'b1;
        #5 RESET = 1'b0;
    end

    initial
    begin
    	// Finish after 300 time units
        #2000 $finish;
    end
	
/*	// Reading from the instruction memory and loading to array
    initial
    begin
        $readmemb("instr_mem.mem", INST_MEM);
    end
*/


	// To create the gtk wave
    initial
    begin
        $dumpfile("cpu_dump.vcd");
        $dumpvars(0, stimulus);
        
    end
endmodule



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// CPU

// 1 : Alu module and its sub-modules

// For the FORWARD function
module FORWARD(DATA2, RESULT);

	input [7:0] DATA2;		// For the input DATA2 of the alu
	output [7:0] RESULT;	// For the output
	
	// RESULT gets DATA2 after a time delay of 1
	assign #1 RESULT = DATA2;
	
endmodule

// For the ADD function
module ADD(DATA1, DATA2, RESULT);

	input [7:0] DATA1;		// For the input DATA1 of the alu
	input [7:0] DATA2;		// For the input DATA2 of the alu
	output [7:0] RESULT;	// For the output
	
	// RESULT gets DATA1 + DATA2 after a time delay of 2
	assign #2 RESULT = (DATA1 + DATA2);
	
endmodule

// For the AND function
module AND(DATA1, DATA2, RESULT);

	input [7:0] DATA1;		// For the input DATA1 of the alu
	input [7:0] DATA2;		// For the input DATA2 of the alu
	output [7:0] RESULT;	// For the output
	
	// RESULT gets DATA1 & DATA2 after a time delay of 1
	assign #1 RESULT = (DATA1 & DATA2);
	
endmodule

// For the OR function
module OR(DATA1, DATA2, RESULT);

	input [7:0] DATA1;		// For the input DATA1 of the alu
	input [7:0] DATA2;		// For the input DATA2 of the alu
	output [7:0] RESULT;	// For the output
	
	// RESULT gets DATA1 | DATA2 after a time delay of 1
	assign #1 RESULT = (DATA1 | DATA2);

endmodule





//ALU
module alu(DATA1, DATA2, RESULT, SELECT, ZERO);

	input [7:0] DATA1;		// For the input DATA1 of the alu (8 bits)
	input [7:0] DATA2;		// For the input DATA2 of the alu (8 bits)
	input [2:0] SELECT;		// For the select line (3 bits)
	output [7:0] RESULT;	// For the final output of the alu (8 bits)
	output reg [0:0] ZERO;		// For beq instructions ; a select signal --->new
	
	
	reg [7:0] RESULT;		// Making the output type of the alu reg
	
	wire [7:0] RESULT1;		// Result from the module FORWARD (8 bits)
	wire [7:0] RESULT2;		// Result from the module ADD (8 bits)
	wire [7:0] RESULT3;		// Result from the module AND (8 bits)
	wire [7:0] RESULT4;		// Result from the module OR (8 bits)
	
	// Instantiating the modules which need to be fed into the MUX inputs
	FORWARD f_1(DATA2, RESULT1);
	ADD f_2(DATA1, DATA2, RESULT2);
	AND f_3(DATA1, DATA2, RESULT3);
	OR f_4(DATA1, DATA2, RESULT4);
	
	// A MUX is implemented using a case structure
	// This always block will get triggered only if SELECT, RESULT1, RESULT2, RESULT3, RESULT4 are changed
	always @(SELECT, RESULT1, RESULT2, RESULT3, RESULT4)
	begin
	// Selecting the relevant output based on the 3 bit aluop
	case(SELECT)
		3'b000 : // Output of the FORWARD module is selected
		begin
		RESULT = RESULT1;
		ZERO = 1'b0;
		end			
		
		3'b001 : // Output of the ADD module is selected
		begin
		RESULT = RESULT2;
			if(RESULT == 8'd0)
				ZERO = 1'b1;	//ZERO will be made high if DATA1 - DATA2 == 0
			else
				ZERO = 1'b0;	//ZERO will be made high if DATA1 - DATA2 == 0
		end			
		
		3'b010 : // Output of the AND module is selected
		begin
		RESULT = RESULT3;
		ZERO = 1'b0;			
		end
		
		3'b011 : // Output of the OR module is selected
		begin
		RESULT = RESULT4;
		ZERO = 1'b0;			
		end
		
		
		// The following outputs are made zero and will be dealt with later
		3'b100 : RESULT = 8'b00000000; 
		
		3'b101 : RESULT = 8'b00000000; 
		
		3'b110 : RESULT = 8'b00000000; 
		
		3'b111 : RESULT = 8'b00000000;  
		
	endcase
	
	end

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 2 : Register file module

module reg_file(IN, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITE, CLK, RESET);
	
	input [7:0] IN;					// 8-bit data to be written into a given register
	input [2:0] INADDRESS;			// 3-bit register address of the input register
	input [2:0] OUT1ADDRESS;		// 3-bit output register address 1
	input [2:0] OUT2ADDRESS;		// 3-bit output register address 2
	input CLK, RESET, WRITE;		// Other input values including the clock, reset value and the enable value
	
	output reg [7:0] OUT1;			// 8-bit data from output register 1
	output reg [7:0] OUT2;			// 8-bit data from output register 2
	
	reg [7:0] regs [7:0];			// Array of registers (8 registers that can store 8-bit data each)
	
	
	// This module has three functions it can perform; reading from two registers, writing to a register at a time and the ability to reset the values stored in the registers
	
	// Function 1: Reading from the registers (given two registers)
	// Reading process is done asynchronously with a delay of two time units
	// The following always block is triggered only when regs[OUT1ADDRESS], regs[OUT2ADDRESS], OUT1ADDRESS, OUT2ADDRESS are changed
	always @(regs[OUT1ADDRESS], regs[OUT2ADDRESS], OUT1ADDRESS, OUT2ADDRESS)
	begin
		
		// Reading from the relevant register and giving the output through the relevant port
		// Non-blocking assignment is being used with a delay of two time units (The delays do not get accumulated since it's on RHS)
		OUT1 <= #2 regs[OUT1ADDRESS];		
		OUT2 <= #2 regs[OUT2ADDRESS];
	end
	
	
	// Function 2: Writing into the given register
	// Writing process is done synchronously with a delay of one time unit
	// The following always block is triggered only at the positive edge of the clock  
	always @(posedge CLK)
	begin
		
		// Writing into the register should only happen when (WRITE == 1 AND RESET == 0)
		if(WRITE == 1'b1  &&  RESET == 1'b0)
		begin 
			#1 regs[INADDRESS] <= IN;		// Writing into the register after a delay of one time unit (non-blocking assingment is used)
		end
	end
	
	
	// Function 3: Resetting all the values stored in the 8 registers 
	// Resetting is done synchronously with a delay of one time unit
	integer counter;			// This variable is used for the counter
	
	// The following always block is triggered only at the positive edge of the clock 
	always @(posedge CLK)
	begin
	
		// Making sure RESET == 1
		if(RESET == 1'b1)
		begin
			// For loop to access the registers
			for(counter = 0; counter < 8; counter = counter + 1)
				regs[counter] <= #1 8'b00000000;		// Making each register value zero after one time delay
		end
	end
	
	// End of functions
	
	

	
	
	// This part is for monitering purposes
	// To moniter the values stored in the values using gtkwave
	// Ignore the warning
	
	/*always @(*)
		$display("%d %d %d %d %d %d %d %d %d", $time, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);*/
		
	//For debugging	
		
	initial
	begin
		#5
		$display("\n\t\t\t==================================================================");
		$display("\t\t\t Change of Register Content Starting from Time #5");
		$display("\t\t\t==================================================================\n");
		$display("\t\ttime\tregs0\tregs1\tregs2\tregs3\tregs4\tregs5\tregs6\tregs7");
		$display("\t\t-------------------------------------------------------------------------------------");
		$monitor("   	   	",$realtime, "\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d", regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
	end	
		
		
		
		
	//For the gtkwave	
	
	initial
	begin
		$dumpfile("cpu_dump.vcd");
		for(counter = 0; counter < 8; counter = counter + 1)
			$dumpvars(1, regs[counter]);
	end  

	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 3 : MUX(type 1 for 8 bits)

module mux(INPUT1, INPUT2, SELECT, OUTPUT);
	
	input [7:0] INPUT1;
	input [7:0] INPUT2;
	input SELECT;
	output reg [7:0] OUTPUT;
	
	// This always block gets triggered if INPUT1, INPUT2, SELECT changes
	always @(*)
	begin
		if(SELECT == 0)
			OUTPUT = INPUT1;	// INPUT1 for SELECT = 0
		else if(SELECT == 1)
			OUTPUT = INPUT2;	// INPUT2 for SELECT = 1
	end	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 4 : MUX(type 2 for 32 bits)(Implementing this to avoid warnings)(Used for MUX3 - To update the PC value)

module muxt2(INPUT1, INPUT2, SELECT, OUTPUT);
	
	input [31:0] INPUT1;
	input [31:0] INPUT2;
	input SELECT;
	output reg [31:0] OUTPUT;
	
	// This always block gets triggered if INPUT1, INPUT2, SELECT changes
	always @(*)
	begin
		if(SELECT == 0)
			OUTPUT = INPUT1;	// INPUT1 for SELECT = 0
		else if(SELECT == 1)
			OUTPUT = INPUT2;	// INPUT2 for SELECT = 1
	end	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 5 : 2's compliment

module comp(INPUT, OUTPUT);
	
	input [7:0] INPUT;
	output [7:0] OUTPUT;
	
	// This can output the 2's compliment of an 8 bit number
	
	// The input gets inverted and 1 is added after a delay of one	
	assign #1 OUTPUT = (~INPUT + 8'b00000001);	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 6 : PC (Program counter)

module pc(PCIN, RESET, CLK, PCOUT);
	
	input RESET, CLK;
	input [31:0] PCIN;
	output reg [31:0] PCOUT;	// PC value after the update
	
	
	// Initializing the value of PCIN is done in the cpu
	
	// A synchronous process
	// To assign a value to PCOUT
	// There is an RHS delay of 1
	always @(posedge CLK)
	begin
		// If RESET is high then PCOUT is zero
		if(RESET == 1'b1)
			#1 PCOUT = 32'd0;
		else
		// If not PCOUT gets PCIN
			#1 PCOUT = PCIN;
	end
	
	

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 7 : PC+4 adder

module pc_4(PC, PCNEXT);

	input [31:0] PC;
	output reg [31:0] PCNEXT;	// PCNEXT is the value PC+4
	
	
	// This block gets triggered whenever PC changes
	// PCNEXT gets updated after a delay of 1 time unit
	always @(PC)
	begin
		PCNEXT = #1 (PC + 32'd4);
	end
	

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 8 : Branch/Jump Target Adder

module jbeq_adder(PCNEXT, OFFSET, PCNEW);

	input [31:0] PCNEXT;	// This is the value of PC+4
	input [31:0] OFFSET;	// Modified offset value of 32 bits
	output [31:0] PCNEW;	// Next pc value for j and beq instructions
	
	// PCNEW gets updated after a delay of 2 time units
	assign #2 PCNEW = (PCNEXT + OFFSET);
	

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 9 : Modifying the offset (shifting and sign extension)

module modify(OFFSET, N_OFFSET);

	input [7:0] OFFSET;				//Original 8 bit offset from RD
	output reg[31:0] N_OFFSET;		// Final offset of 32 bits
	reg [9:0] SHIFTED;				//Offset after multiplying by 4 
	
	// For shifting to the left by 2
	always @(OFFSET)
	begin
		SHIFTED[9:2] = OFFSET;
		SHIFTED[1:0] = 2'b0;
	end
	
	// Sign extension
	always @(SHIFTED)
	begin
		N_OFFSET[9:0] = SHIFTED;		//Getting the first ten bits
		if(N_OFFSET[9] == 0)
			N_OFFSET[31:10] = 22'b0000000000000000000000;	//For positive numbers
		else
			N_OFFSET[31:10] = 22'b1111111111111111111111;	//For negative numbers
	end
	
	
	

endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// 10 : Decoder

module decode(INST, OPCODE, RD, RT, RS);
	
	input [31:0] INST;
	output reg [7:0] OPCODE, RD, RT, RS;
	

	
	// This always block is to decode several parts of the fetched instruction
	// The relevants parts are directed as needed
	// Non blocking assignment was used
	always @(*)
	begin
		OPCODE <= INST[31:24];		// For the opcode
		RD <= INST[23:16];			// Destination reg
		RT <= INST[15:8];			// Rt
		RS <= INST[7:0];    		// Source reg
		
	end
	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 11 : Combinational logic unit (To generate MUX3SEL signal)

module comb(ZERO, J, BEQ, OUTSIGNAL);

	input [0:0] ZERO, J, BEQ;		// These are coming from the alu and the cu
	output [0:0] OUTSIGNAL;		// Given to MUX3SEL
	wire [0:0] ANDOUT;
	
/*	always @(*)
	begin
		// ZERO is ANDed with BEQ. Then the result is ORed with J
		// No delays
		OUTSIGNAL = (ZERO * BEQ) + J; 
	end*/
	
	and AND_G1(ANDOUT, BEQ, ZERO);
	or OR_G1(OUTSIGNAL, J, ANDOUT);
	
	

endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// 12 : CU (Control Unit)

module cu(INST, IMM, OUT1ADDRESS, OUT2ADDRESS, INADDRESS, WRITEEN, MUX1SEL, MUX2SEL, MUX4SEL, ALUOP, OFFSET, J, BEQ, READ, WRITE, BUSYWAIT, inst_busywait);
	
	input [31:0] INST;
	input BUSYWAIT, inst_busywait;//new
	output reg [7:0] IMM;
	output reg [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS;
	output reg [0:0] WRITEEN, MUX1SEL, MUX2SEL, MUX4SEL, J, BEQ, READ, WRITE;//new
	output reg [2:0] ALUOP;
	output reg [7:0] OFFSET;
	
	wire [7:0] OPCODE, RD, RT, RS;
	
	
	// Instantiating the decoder inside the cu
	decode D1(INST, OPCODE, RD, RT, RS);
	
	
	// This block stalls the processor by keeping the pc value unchaged. 
    always @(BUSYWAIT, inst_busywait)
    begin
        if (BUSYWAIT || inst_busywait) 
        begin
            J <= 1'b1;
            OFFSET <= 8'b11111111; //setting the jump value to -1.    
        end
        else
        begin
            J <= 1'b0; //if a delay is added here, the pc will not be sensitive to the pcIn value which is updated exactly at the time the new pc value is updated               
            //if the following 2 are not set, BUSYWAIT will not be sensitive to consecutive load/store instructions
            WRITE = 1'b0;
            READ = 1'b0;
        end
    end
	

	
	// The control signals are generated according to the relevant opcode with a delay of one time unit
	// Control signals include ALUOP, MUX1SEL, MUX2SEL, MUX4SEL
	always @(OPCODE, RD, RT, RS)
	begin
		if (BUSYWAIT || inst_busywait) 
        begin //this is to avoid any unexpected writing to the regfile/data_memory
            WRITEEN <= #1 1'b0;    
        end
        else 
        begin
			case (OPCODE)
		
			// For loadi
			8'b00000000 :
			begin
				ALUOP <= #1 3'b000;
				INADDRESS <= RD[2:0];
				IMM <= RS;
				MUX1SEL <= #1 0; 
				MUX2SEL <= #1 0;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1; 
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
				
			end
			
			// For mov
			8'b00000001 :
			begin
				ALUOP <= #1 3'b000;
				INADDRESS <= RD[2:0];
				OUT2ADDRESS <= RS[2:0];
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1;
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
				
			end
			
			// For add
			8'b00000010 :
			begin
				ALUOP <= #1 3'b001;
				OUT1ADDRESS <= RT[2:0];
				OUT2ADDRESS <= RS[2:0];
				INADDRESS <= RD[2:0];
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1;
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
			end
			
			// For sub
			8'b00000011 :
			begin
				ALUOP <= #1 3'b001;
				OUT1ADDRESS <= RT[2:0];
				OUT2ADDRESS <= RS[2:0];
				INADDRESS <= RD[2:0];
				MUX1SEL <= #1 1;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1;
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
				
			end
			
			// For and
			8'b00000100 :
			begin
				ALUOP <= #1 3'b010;
				OUT1ADDRESS <= RT[2:0];
				OUT2ADDRESS <= RS[2:0];
				INADDRESS <= RD[2:0];
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1;
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
				
			end
			
			// For or
			8'b00000101 :
			begin
				ALUOP <= #1 3'b011;
				OUT1ADDRESS <= RT[2:0];
				OUT2ADDRESS <= RS[2:0];
				INADDRESS <= RD[2:0];
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b1;
				//OFFSET <= RD[7:0];
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
				
			end
			
			//For j											
			8'b00000110 :
			begin
				WRITEEN <= #1 1'b0;
				OFFSET <= RD;
				J <= #1 1'b1;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;

			end
			
			// For beq										
			8'b00000111 :
			begin
				ALUOP <= #1 3'b001;
				OUT1ADDRESS <= RT[2:0];
				OUT2ADDRESS <= RS[2:0];
				MUX1SEL <= #1 1;
				MUX2SEL <= #1 1;
				//MUX4SEL <= #1 1;
				WRITEEN <= #1 1'b0;
				OFFSET <= RD;
				J <= #1 1'b0;
				BEQ <= #1 1'b1;
				READ <= #1 1'b0;
				WRITE <= #1 1'b0;
			end
			
			// For lwd
			8'b00001110 :
			begin
				ALUOP <= #1 3'b000;
				OUT2ADDRESS <= RS[2:0];
				INADDRESS <= RD[2:0];
				WRITEEN <= #1 1'b1;
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 0;
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b1;
				WRITE <= #1 1'b0;
			end
			
			// For lwi
			8'b00001111 :
			begin
				ALUOP <= #1 3'b000;
				IMM <= RS;
				INADDRESS <= RD[2:0];
				WRITEEN <= #1 1'b1;
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 0;
				MUX4SEL <= #1 0;
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b1;
				WRITE <= #1 1'b0;
			end
			
			// For swd
			8'b00010000 :
			begin
				ALUOP <= #1 3'b000;
				OUT2ADDRESS <= RS[2:0];
				OUT1ADDRESS <= RT[2:0];
				WRITEEN <= #1 1'b0;
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 1;
				MUX4SEL <= #1 0;
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b1;
			end
			
			// For swi
			8'b00010001 :
			begin
				ALUOP <= #1 3'b000;
				IMM <= RS;
				OUT1ADDRESS <= RT[2:0];
				WRITEEN <= #1 1'b0;
				MUX1SEL <= #1 0;
				MUX2SEL <= #1 0;
				MUX4SEL <= #1 0;
				J <= #1 1'b0;
				BEQ <= #1 1'b0;
				READ <= #1 1'b0;
				WRITE <= #1 1'b1;
			end
			
			endcase	
		end			 
	end
	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 13 : CPU

module cpu(PC, INSTRUCTION, CLK, RESET, ADDRESS, WRITEDATA, READ, WRITE, BUSYWAIT, inst_busywait, READDATA);
	
	input CLK, RESET;
	input [31:0] INSTRUCTION;
	output [31:0] PC;	// from the pc unit to the PC+4 ADDER and the instruction mem
	
	wire [7:0] IMM, OUT1, OUT2;
	wire [2:0] OUT1ADDRESS, OUT2ADDRESS, INADDRESS, ALUOP;
	wire WRITEEN, MUX1SEL, MUX2SEL, MUX3SEL, MUX4SEL;//new
	wire [7:0] COMPOUT;
	wire [7:0] IN;
	wire [7:0] MUX1OUT, MUX2OUT, MUX4OUT;
	wire [31:0] MUX3OUT; 
	wire [31:0] PC1, PC2;	//outputs of the adders  PC2-->NEW VALUE for j and beq instructions
	wire [31:0] N_OFFSET; 	//new modified offset 32 bits
	wire [7:0] OFFSET;		//coming from the cu (RD) 8 bits
	wire ZERO;				//coming from the alu
	wire J, BEQ;			//coming from the cu
	
	//new
	output reg [7:0] ADDRESS, WRITEDATA;
	output [0:0] READ, WRITE;
	input BUSYWAIT, inst_busywait; //Coming from the data memory
	input [7:0] READDATA;	//Coming from the data memory
	
	
	always @(IN)
	begin
		ADDRESS = IN;
	end
	
	always @(OUT1)
	begin
		WRITEDATA = OUT1;
	end
	
	
	
	
	//Control unit
	cu CU1(INSTRUCTION, IMM, OUT1ADDRESS, OUT2ADDRESS, INADDRESS, WRITEEN, MUX1SEL, MUX2SEL, MUX4SEL, ALUOP, OFFSET, J, BEQ, READ, WRITE, BUSYWAIT, inst_busywait);
	
	//PC
	pc P1(MUX3OUT, RESET, CLK, PC);
	
	//Offset modifier (8 to 32 bits)
	modify M1(OFFSET, N_OFFSET);
	
	//Pc+4 adder
	pc_4 P2(PC, PC1);
	
	//Jump/beq adder
	jbeq_adder P3(PC1, N_OFFSET, PC2);
	
	//Register file and 2's comp
	reg_file RF1(MUX4OUT, OUT1, OUT2, INADDRESS, OUT1ADDRESS, OUT2ADDRESS, WRITEEN, CLK, RESET);
	comp COMP1(OUT2, COMPOUT);
	
	
	//First two muxes - 8 bits
	mux MX1(OUT2, COMPOUT, MUX1SEL, MUX1OUT);
	mux MX2(IMM, MUX1OUT, MUX2SEL, MUX2OUT);
	
	//Combinational logic unit and the relevant mux (mux 3) (To choose the relevant PC value)
	comb CB1(ZERO, J, BEQ, MUX3SEL);
	muxt2 MX3(PC1, PC2, MUX3SEL, MUX3OUT);		//32 bits
	
	//New mux
	mux MX4(READDATA, IN, MUX4SEL, MUX4OUT);
	
	
	//ALU
	alu ALU1(OUT1, MUX2OUT, IN, ALUOP, ZERO);
	
	
	
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



















