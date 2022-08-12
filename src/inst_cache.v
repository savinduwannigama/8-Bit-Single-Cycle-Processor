// Instruction cache module

`timescale  1ns/100ps

module instcache (busywait, instruction, address, mem_busywait, mem_read, mem_read_inst, mem_address);

	//Inputs and outputs
	//Ports connected to the CPU
	
	input [9:0] address;
	output reg [31:0] instruction;
	output reg [0:0] busywait;
	
	//Ports connected to the Instruction Memory 
	input mem_busywait;
	input [127:0] mem_read_inst;
	output reg [0:0] mem_read;
	output reg [5:0] mem_address;
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	//Structure inside the Instruction Cache
	reg [7:0] INST_CACHE_ARRAY [127:0];	//128x8 bits
    reg [2:0] INST_TAG_ARRAY [7:0];
    reg [0:0] INST_VALID_BIT_ARRAY [7:0];
    
    reg [31:0] LOAD_CACHE_INST;
	
	//Components of the address of 10 bits
    reg [2:0] tag, index;
    reg [1:0] offset;
    
    //HIT signal
    reg [0:0] hit;
    
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
     //To generate the dump file
    integer i;
    initial
    begin
        $dumpfile("cpu_dump.vcd");
        for(i=0;i<128;i++)
            $dumpvars(1,INST_CACHE_ARRAY[i]);
        for(i=0;i<8;i++)
        begin
            $dumpvars(1,INST_TAG_ARRAY[i]);
            $dumpvars(1,INST_VALID_BIT_ARRAY[i]);
        end
    end
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	//Breaking the address (from PC) into three components
    always @(address)
    begin
    	tag <= #1 address [9:7];	//3 bits
    	index <= #1 address [6:4];	//3 bits
    	offset <= #1 address [3:2];	//2 bits
    end
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    //Tag comparison to generate the HIT signal before a memory read
    always @(tag, index, offset)
    begin
    	if((tag == INST_TAG_ARRAY[index])  &&  (INST_VALID_BIT_ARRAY[index])  && !mem_busywait )
    		begin
    		hit <= #0.9 1'b1;
    		
    		end
    	else
    		begin
    		#0.9;
    		busywait = 1'b1;
    		mem_read = 1'b1;            
            mem_address = {tag, index};           
            
            end
    	
    end
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    
    //Tag comparison to generate the HIT signal after a memory read
    always @(mem_busywait)
    begin
    	#2;
        if ((tag == INST_TAG_ARRAY[index])  &&  (INST_VALID_BIT_ARRAY[index])  && !mem_busywait) 
        begin
            hit <= #0.9 1'b1;
        end
    	
    end
    
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    //Read-hit 
    //Reading from the cache is done asynchronously
    always @(tag, index, offset)
    begin
    	
        #1 LOAD_CACHE_INST = {INST_CACHE_ARRAY[{index, offset, 2'b11}], INST_CACHE_ARRAY[{index, offset, 2'b10}], INST_CACHE_ARRAY[{index, offset, 2'b01}],INST_CACHE_ARRAY[{index, offset, 2'b00}]}; //Tag comparison delay may overlap with this delay
        
        if(hit)                                                  
        begin                                                  
            instruction = LOAD_CACHE_INST;
            busywait = 1'b0;
            hit = 1'b0;
            
        end
        
      
    end
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Read-miss
    //Reading from the cache after updating
    always @(mem_busywait) 
    begin
        if(!mem_busywait) //not necessary
        begin
        	mem_read = 1'b0; //to set the mem_read to 0 after a memory read
            #3 LOAD_CACHE_INST = {INST_CACHE_ARRAY[{index, offset, 2'b11}], INST_CACHE_ARRAY[{index, offset, 2'b10}], INST_CACHE_ARRAY[{index, offset, 2'b01}],INST_CACHE_ARRAY[{index, offset, 2'b00}]};//tag comparison delay may overlap with this delay
            
            if(hit)                                                   
            begin                                                  
                instruction = LOAD_CACHE_INST;
            	busywait = 1'b0;
            	hit = 1'b0;
            end
        end
    end
    
   
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    
    //To write to the cache after a memory read
    always @(mem_busywait)
    begin
        if(!mem_busywait)
        begin
            {INST_CACHE_ARRAY[{index, 4'b0011}], INST_CACHE_ARRAY[{index, 4'b0010}], INST_CACHE_ARRAY[{index, 4'b0001}], INST_CACHE_ARRAY[{index, 4'b0000}]} <= #1 mem_read_inst[31:0];
            {INST_CACHE_ARRAY[{index, 4'b0111}], INST_CACHE_ARRAY[{index, 4'b0110}], INST_CACHE_ARRAY[{index, 4'b0101}], INST_CACHE_ARRAY[{index, 4'b0100}]} <= #1 mem_read_inst[63:32];
            {INST_CACHE_ARRAY[{index, 4'b1011}], INST_CACHE_ARRAY[{index, 4'b1010}], INST_CACHE_ARRAY[{index, 4'b1001}], INST_CACHE_ARRAY[{index, 4'b1000}]} <= #1 mem_read_inst[95:64];
            {INST_CACHE_ARRAY[{index, 4'b1111}], INST_CACHE_ARRAY[{index, 4'b1110}], INST_CACHE_ARRAY[{index, 4'b1101}], INST_CACHE_ARRAY[{index, 4'b1100}]} <= #1 mem_read_inst[127:96];

            INST_VALID_BIT_ARRAY[index] = 1'b1;
            INST_TAG_ARRAY[index] = tag;	
            
          
        end
        

    end
    
    
      
    

endmodule
