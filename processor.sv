`timescale 1ns /1 ns
/************************** Control path **************************************************/
module control_path(
    input logic clk,
    input logic reset, 
    input logic run, 
    input logic [15:0] INSTRin,
    output logic R0in, R1in, Ain, Rin, IRin, 
    output logic [1:0] select, ALUOP,
    output logic done
); 

/* OPCODE format: II M X DDDDDDDDDDDD, where 
    *     II = instruction, M = Immediate, X = rX; X = (rX==0) ? r0:r1
    *     If M = 0, DDDDDDDDDDDD = 00000000000Y = rY; Y = (rY==0) r0:r1
    *     If M = 1, DDDDDDDDDDDD = #D is the immediate operand 
    *
    *  II M  Instruction   Description
    *  -- -  -----------   -----------
    *  00 0: mv    rX,rY    rX <- rY
    *  00 1: mv    rX,#D    rX <- D (sign extended)
    *  01 0: add   rX,rY    rX <- rX + rY
    *  01 1: add   rX,#D    rX <- rX + D
    *  10 0: sub   rX,rY    rX <- rX - rY
    *  10 1: sub   rX,#D    rX <- rX - D
    *  11 0: mult  rX,rY    rX <- rX * rY
    *  11 1: mult  rX,#D    rX <- rX * D 
*/

parameter mv = 2'b00, add = 2'b01, sub = 2'b10, mult = 2'b11; // Opcode constants

logic [1:0] II; // Instruction Type
logic M, rX, rY; // Immediate flag and register selection

assign II = INSTRin[15:14]; // Extract instruction type from INSTRin
assign M =  INSTRin[13]; // Extract immediate flag
assign rX = INSTRin[12]; // Extract target register
assign rY = INSTRin[0]; // Extract source register

// control FSM states
typedef enum logic[1:0]
{
    C0 = 'd0, // Fetch state
    C1 = 'd1, // Decode / prepare ALU operands
    C2 = 'd2, // Execute ALU operation
    C3 = 'd3 // Writeback
} statetype;

statetype current_state, next_state;


// control FSM state table
always_comb begin
    case(current_state)
	C0: next_state = run? C1:C0; // If run is high, move to decode
        C1: next_state = done? C0:C2; // If done, return to fetch, else execute
        C2: next_state = C3; // Execute goes to writeback
        C3: next_state = C0; // Writeback returns to fetch
    endcase
end

// output logic i.e: datapath control signals
always_comb begin
    // by default, make all our signals 0
    R0in = 1'b0; R1in = 1'b0;
    Ain = 1'b0; Rin = 1'b0; IRin = 1'b0;
    select = 2'bxx; 
    ALUOP = 2'bxx;
    done = 1'b0;

    case(current_state)
        C0: IRin = 1; // Load instructions into IR
        C1: 
            begin
            case(II): // Decode logic
            begin
                2'b00: // mv instruction
                    if (!M) select = 2'b11; // Register move
                    else select = (rY ? 2'b01:2'b10); // Immediate move
                    if (rX) R1in = 1; // Enable target register
                    else R0in = 1;
                    done = 1; // mv complete
            end
                default: begin
                    select = (rX?2'b10:2'b01); // ALU input select
                    Ain = 1; // Load register
                end
            endcase
            end
        C2: 
            Rin = 1; // Enable result register
            if (rY)
            select = (M?2'b11:2'b10); // Select the operand
            else select = (M?2'b11:2'b01);
            // set ALU operation
            case (II)
                add:  ALUOP = 2'b00;
                sub:  ALUOP = 2'b01;
                mult: ALUOP = 2'b10;
                default: ALUOP = 2'b00; // mv or fallback
            endcase
        C3: 
            select = 0; // Reset select
            if (rX) r1in = 1; // Enable target register
            else r0in = 1;
            done = 1; // Instr complete
    endcase 
end


// control FSM FlipFlop
always_ff @(posedge clk) begin
    if(reset)
        current_state <= C0; // Reset State
    else
       current_state <= next_state; // Update state
end

endmodule


/************************** Datapath **************************************************/
module datapath(
    input logic clk, 
    input logic reset,
    input logic [15:0] INSTRin,
    input logic IRin, R0in, R1in, Ain, Rin,
    input logic [1:0] select, ALUOP,
    output logic [15:0] r0, r1, a, r // for testing purposes these are outputs
);
// Internal registers of the datapath
logic [15:0] IR, R0, R1, A, R_reg, MUXout, ALUout;

// Output assignments
assign r0 = R0;
assign r1 = R1;
assign a  = A;
assign r  = R_reg;

// MUX: 00->R, 01->R0, 10->R1, 11->IR(immediate)
always_comb begin
    case (select)
        2'b00: MUXout = R_reg; // Use prev ALU result
        2'b01: MUXout = R0; // Use R0 as input
        2'b10: MUXout = R1; // Use R1 as input
        2'b11: MUXout = {{4{IR[11]}}, IR[11:0]}; // sign-extend 12-bit immediate from IR as input
        default: MUXout = 16'b0; // Default to 0 for safety
    endcase
end

// ALU: A op MUXout (A is left operand)
always_comb begin
    case (ALUOP)
        2'b00: ALUout = A + MUXout; // add
        2'b01: ALUout = A - MUXout; // sub
        2'b10: ALUout = A * MUXout; // mult
        default: ALUout = MUXout; // mv
    endcase
end

// Registers
always_ff @(posedge clk) begin
    if (reset) begin
        R0    <= 16'b0;
        R1    <= 16'b0;
        A     <= 16'b0;
        R_reg <= 16'b0;
        IR    <= 16'b0;
    end
    else begin
        // Load instr into the respective register if the control signal is high
        if (IRin)
            IR <= INSTRin;
        if (R0in)
            R0 <= MUXout;
        if (R1in)
            R1 <= MUXout;
        if (Ain)
            A <= MUXout;
        if (Rin)
            R_reg <= ALUout;
    end
end
endmodule


/************************** processor  **************************************************/
module part2(
    input logic [15:0] INSTRin,
    input logic reset, 
    input logic clk,
    input logic run,
    output logic done,
    output logic[15:0] r0_out,r1_out, a_out, r_out
);

// intermediate logic 
logic r0in, r1in, ain, rin, irin;
logic[1:0] select, aluop;

control_path control(
   .clk(clk),
   .reset(reset), 
   .run(run), 
   .INSTRin(INSTRin),
   .R0in(r0in), 
   .R1in(r1in), 
   .Ain(ain), 
   .Rin(rin), 
   .IRin(irin), 
   .select(select), 
   .ALUOP(aluop),
   .done(done)
);

datapath data(
    .clk(clk), 
    .reset(reset),
    .INSTRin(INSTRin),
    .IRin(irin), 
    .R0in(r0in),
    .R1in(r1in), 
    .Ain(ain),
    .Rin(rin),
    .select(select), 
    .ALUOP(aluop),
    .r0(r0_out), 
    .r1(r1_out),
    .a(a_out),
    .r(r_out)
);

endmodule
