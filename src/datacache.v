//Group 14

/*
Module  : Data Cache 
Author  : Isuru Nawinne, Kisaru Liyanage
Date    : 25/05/2020


*/

`timescale  1ns/100ps

module dcache (clk, reset, busywait, read, write, writedata, readdata, address, mem_busywait, mem_read, mem_write, mem_writedata, mem_readdata, mem_address);

	//Inputs and outputs
	input clk, reset;
	
	//Ports coonected to the CPU
	input [0:0] read, write;
	input [7:0] writedata, address;
	output reg [7:0] readdata;
	output reg [0:0] busywait;
	
	//Ports connected to the Memory Storage
	input [0:0] mem_busywait;
	input [31:0] mem_readdata;
	output reg [0:0] mem_read, mem_write;
	output reg [31:0] mem_writedata;
	output reg [5:0] mem_address;
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    /*
    Combinational part for indexing, tag comparison for hit deciding, etc.
    ...
    ...
    */
    
    //Structure inside the Cache Storage
    reg [7:0] CACHE_MEM_ARRAY [31:0];	//32x8 bits
    reg [2:0] TAG_ARRAY [7:0];
    reg [0:0] VALID_BIT_ARRAY [7:0];
    reg [0:0] DIRTY_BIT_ARRAY [7:0];
    reg [7:0] LOAD_CACHE_DATA;
    
    //Components of the address of 8 bits
    reg [2:0] tag, index;
    reg [1:0] offset;
    
    //HIT signal
    reg [0:0] hit, sen;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //For testing purposes
    initial
    begin
        $dumpfile("cpu_dump.vcd");
        for(i=0;i<32;i++)
            $dumpvars(1,CACHE_MEM_ARRAY[i]);
        for(i=0;i<8;i++)
        begin
            $dumpvars(1,TAG_ARRAY[i]);
            $dumpvars(1,VALID_BIT_ARRAY[i]);
            $dumpvars(1,DIRTY_BIT_ARRAY[i]);
        end
    end
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    //Breaking the ADDRESS into three components
    always @(address)
    begin
    	tag <= #1 address [7:5];	//3 bits
    	index <= #1 address [4:2];	//3 bits
    	offset <= #1 address [1:0];	//2 bits
    end
    
    //to get the busywait signal
    always @(read, write)
    begin
        busywait = (read || write)? 1 : 0;
        sen = #3 ~sen; //used to trigger the blocks for "tag comparison", and "read from cache after a hit", when we get similar consecutive addresses.
    end
    
    
    //Tag comparison to generate the HIT signal
    always @(tag, index, offset, state, sen)//sensitive to state to invoke the block after a memory read
    begin
    	if((tag == TAG_ARRAY[index])  &&  (VALID_BIT_ARRAY[index] == 1'b1)  && (read || write)  &&  (state==IDLE))
    		begin
    		hit <= #0.9 1'b1;
    		//busywait = 1'b0; //busywait will be already 0 when invoked after a memory read
    		end
    	else
    		hit <= #0.9 1'b0;
    	
    end
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //If it is a hit
    
    //Write-hit
    //Writing to the cache is done synchronously
    always @(posedge clk)
    begin

    	if(hit  &&  !read  &&  write)
    	begin
    		CACHE_MEM_ARRAY[{index, offset}] <= #1 writedata; 
    		hit = 1'b0;
    		DIRTY_BIT_ARRAY[index] = 1'b1;
    		//VALID_BIT_ARRAY[index] = 1'b1;
    		busywait = 1'b0;
    	end
    	
    end
    
    
    //Read-hit
    //Reading from the cache is done asynchronously
    always @(hit, index, offset, sen)
    begin
    	if(read && !write)
        begin
            #1 LOAD_CACHE_DATA = CACHE_MEM_ARRAY[{index, offset}]; //Tag comparison delay may overlap with this delay
            if(hit)                                                   //tag comparison will complete before the end of this delay
            begin                                                  
                readdata = LOAD_CACHE_DATA;
                busywait = 1'b0;
                hit = 1'b0;
            end
        end
    end
    
  
  	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  	
  	//Reset cache
  	integer i;
    always @(posedge reset)
    begin
        if (reset)
        begin
            for (i=0;i<32; i=i+1)
                CACHE_MEM_ARRAY[i] = 8'd0;
            for (i=0;i<8; i=i+1)
            begin
                TAG_ARRAY[i] = 1'bx;
                VALID_BIT_ARRAY[i] = 0;
                DIRTY_BIT_ARRAY[i] = 0;
            end
            busywait = 0;
            mem_read = 0;
            mem_write = 0;
            mem_address = 6'dx;
            mem_writedata = 32'dx;
            hit = 0;
            sen = 0;
        end
    end
  	
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
     //To write to the cache after a memory read
    always @(state)
    begin
        if(state == IDLE)
        begin
            CACHE_MEM_ARRAY[{index, 2'b00}] <= #1 mem_readdata[7:0];
            CACHE_MEM_ARRAY[{index, 2'b01}] <= #1 mem_readdata[15:8];
            CACHE_MEM_ARRAY[{index, 2'b10}] <= #1 mem_readdata[23:16];
            CACHE_MEM_ARRAY[{index, 2'b11}] <= #1 mem_readdata[31:24];

            VALID_BIT_ARRAY[index] = 1'b1;
            DIRTY_BIT_ARRAY[index] = 1'b0;
            TAG_ARRAY[index] = tag;	
        end   
    end
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    

    /* Cache Controller FSM Start */

    parameter IDLE = 3'b000, MEM_READ = 3'b001, MEM_WRITE = 3'b010;
    reg [2:0] state, next_state;

    // combinational next state logic
    always @(state, hit, read, write, index, mem_busywait)
    begin
        case (state)
            IDLE:
                if ((read || write) && !DIRTY_BIT_ARRAY[index] && !hit)  
                    next_state = MEM_READ;
                else if ((read || write) && DIRTY_BIT_ARRAY[index] && !hit)
                    next_state = MEM_WRITE;
                else
                    next_state = IDLE;
            
            MEM_READ:
                if (!mem_busywait)
                	begin
                    next_state = IDLE;
                    
                    //Update 
                    VALID_BIT_ARRAY[index] = 1'b1; //If these are not updated here
                    DIRTY_BIT_ARRAY[index] = 1'b0; //the hit signal wont be set to 1
                    TAG_ARRAY[index] = tag;        // on time after the mem read and i dont know why

                    end
                    
                else    
                    next_state = MEM_READ;
                    
            MEM_WRITE:
            	if (!mem_busywait)
            		next_state = MEM_READ;
            	else
            		next_state = MEM_WRITE;
            
        endcase
    end
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // combinational output logic
    always @(state)
    begin
        case(state)
            IDLE:
            begin
                mem_read = 0;
                mem_write = 0;
                mem_address = 8'dx;
                mem_writedata = 8'dx;
            end
         
            MEM_READ: 
            begin
                mem_read = 1;
                mem_write = 0;
                mem_address = {tag, index};
                mem_writedata = 32'dx;
                busywait = 1;   
            end
            
            MEM_WRITE:
            begin
            	mem_read = 0;
            	mem_write = 1;
            	mem_address = {TAG_ARRAY[index], index};
            	
            	mem_writedata[7:0] = CACHE_MEM_ARRAY[{index, 2'b00}];
                mem_writedata[15:8] = CACHE_MEM_ARRAY[{index, 2'b01}];
                mem_writedata[23:16] = CACHE_MEM_ARRAY[{index, 2'b10}];
                mem_writedata[31:24] = CACHE_MEM_ARRAY[{index, 2'b11}];
                
                busywait = 1;
            end
            
        endcase
    end

    // sequential logic for state transitioning 
    always @(posedge clk, reset)
    begin
        if(reset)
            state = IDLE;
        else
            state = next_state;
    end

    /* Cache Controller FSM End */

endmodule