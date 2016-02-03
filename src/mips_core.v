/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// The MIPS standalone processor module
////
////   clk          (input)  - The clock
////   inst_addr    (output) - Address of instruction to load
////   inst         (input)  - Instruction from memory
////   inst_excpt   (input)  - inst_addr not valid
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_data_out (input)  - Data from memory load
////   mem_write_en (output) - Memory write mask
////   mem_excpt    (input)  - mem_addr not valid
////   halted       (output) - Processor halted
////   reset        (input)  - Reset the processor
////   

module mips_core(/*AUTOARG*/
   // Outputs
   inst_addr, mem_addr, mem_data_in, mem_write_en, halted,
   // Inputs
   clk, inst_excpt, mem_excpt, inst, mem_data_out, rst_b
   );
   
   parameter text_start  = 32'h00400000; /* Initial value of $pc */

   // Core Interface
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0] mem_addr;
   input  [31:0] inst, mem_data_out;
   output [31:0] mem_data_in;
   output [3:0]  mem_write_en;
   output        halted;
   input         rst_b;

   // Forced interface signals -- required for synthesis to work OK.
   // This is probably not what you want!
   assign        mem_addr = 0;
   assign        mem_data_in = mem_data_out;
   assign        mem_write_en = 4'b0;

   // Internal signals
   wire [31:0]   pc, nextpc, nextnextpc;
   wire          exception_halt, syscall_halt, internal_halt;
   wire          load_epc, load_bva, load_bva_sel;
   wire [31:0]   rt_data, rs_data, rd_data, alu__out, r_v0;
   //wire [31:0]   rt_data, rs_data, rd_data, r_v0;
   //reg  [31:0]   alu__out;
   wire [31:0]   epc, cause, bad_v_addr;
   wire [4:0]    cause_code;

   // Decode signals
   wire [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
   wire [5:0]    dcd_op, dcd_funct2;
   wire [4:0]    dcd_rs, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
   wire [15:0]   dcd_offset, dcd_imm;
   wire [25:0]   dcd_target;
   wire [19:0]   dcd_code;
   wire          dcd_bczft;
   reg           op_sel;
   reg [31:0]    op1;
   reg [31:0]    op2; 
   reg [31:0]    op_dst;
   reg 	         alu_mux_sel;
   reg [7:0]     regsrc;
   reg           regdst;
   reg           sign_ext;
   reg           shamt;
   
   // PC Management
   register #(32, text_start) PCReg(pc, nextpc, clk, ~internal_halt, rst_b);
   register #(32, text_start+4) PCReg2(nextpc, nextnextpc, clk,
                                       ~internal_halt, rst_b);
   add_const #(4) NextPCAdder(nextnextpc, nextpc);
   assign        inst_addr = pc[31:2];

   // Instruction decoding
   assign        dcd_op = inst[31:26];    // Opcode
   assign        dcd_rs = inst[25:21];    // rs field
   assign        dcd_rt = inst[20:16];    // rt field
   assign        dcd_rd = inst[15:11];    // rd field
   assign        dcd_shamt = inst[10:6];  // Shift amount
   assign        dcd_bczft = inst[16];    // bczt or bczf?
   assign        dcd_funct1 = inst[4:0];  // Coprocessor 0 function field
   assign        dcd_funct2 = inst[5:0];  // funct field; secondary opcode
   assign        dcd_offset = inst[15:0]; // offset field
        // Sign-extended offset for branches
   assign        dcd_se_offset = { {14{dcd_offset[15]}}, dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        dcd_se_mem_offset = { {16{dcd_offset[15]}}, dcd_offset };
   assign        dcd_imm = inst[15:0];        // immediate field
   assign        dcd_e_imm = { 16'h0, dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
   assign        dcd_target = inst[25:0];     // target field
   assign        dcd_code = inst[25:6];       // Breakpoint code

   // synthesis translate_off
   always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "=== Simulation Cycle %d ===", $time );
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   pc, inst, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted);
     end
   end
   // synthesis translate_on



   always @(*) begin
       op1 = op_sel ? rs_data : rt_data; //TODO - reg[dcd_rs]
       op_dst = regdst ? rt_data : rs_data;
       op2 = regsrc[7] ? dcd_rs :
             regsrc[6] ? {2'h0, dcd_imm} : //18_u
             regsrc[5] ? dcd_shamt : //SLL
             regsrc[4] ? { {2{dcd_imm[15]}}, dcd_imm } : //18_s
             regsrc[3] ? dcd_imm : //16_u
             regsrc[2] ? $signed(dcd_offset) :
             regsrc[1] ? dcd_e_imm : //32_u
             regsrc[0] ? dcd_se_imm : rt_data;
   end
   //always @(*) begin
   //     //ALU   
   //     op1     = dcd_rs;
   //     op2     = dcd_rt;
   //     op_dst  = dcd_rd;
   //     case(dcd_op)
   //         `OP_OTHER0:
   //             begin
   //                 case(dcd_funct2)
   //                     //`OP0_SYSCALL:
   //                     //    begin
   //                     //    end
   //                     //`OP0_BREAK:
   //                     //    begin
   //                     //    end
   //                     //`OP0_SLL:
   //                     //    begin
   //     		//	op2 = dcd_shamt;
   //                     //    end
   //                     //`OP0_SRL:
   //                     //    begin 	
   //     		//	op2 = dcd_shamt;
   //                     //    end
   //                     //`OP0_SRA:
   //                     //    begin
   //     		//	op1 = $signed(dcd_rs);
   //     		//	op2 = dcd_shamt;
   //                     //    end
   //                     //`OP0_SLLV:
   //                     //    begin
   //                     //    end
   //                     //`OP0_SRLV:
   //                     //    begin
   //                     //    end
   //                     //`OP0_SRAV:
   //                     //    begin
   //                     //    end
   //                     //`OP0_SLT:
   //                     //    begin
   //     		//	op1 = $signed(dcd_rs);
   //     		//	op2 = $signed(dcd_rt);
   //                     //    end
   //                     //`OP0_MFHI:
   //                     //    begin
   //                     //    end
   //                     //`OP0_MFLO:
   //                     //    begin
   //                     //    end
   //                     //`OP0_MTLO:
   //                     //    begin
   //                     //    end
   //                     //`OP0_MTHI:
   //                     //    begin
   //                     //    end
   //                     //default:
   //                     //    begin
   //                     //    end
   //                 endcase
   //             end
   //         `OP_ADDIU:
   //             begin
   //                 op2 = dcd_se_imm;
   //             end
   //         `OP_ADDI:
   //             begin
   //                 op2 = dcd_se_imm;
   // 	        end
   //         `OP_ANDI:
   // 	        begin
   //                 op2 = dcd_se_imm;
   // 	        end
   //         `OP_ORI:
   //             begin
   //                 op2 = dcd_se_imm;
   //             end
   //         `OP_XORI:
   //             begin
   //                 op2 = dcd_se_imm;
   //             end
   //         `OP_SLTI:
   //             begin
   //     	    op1 = $signed(dcd_rs);
   //     	    op2 = dcd_se_imm;
   //             end
   //         `OP_SLTIU:
   //             begin
   //     	    op2 = dcd_se_imm;
   //             end
   //         //default:
   //         //    begin
   //         //    end
   //     endcase
   //end

   //always @(posedge clk or negedge reset) begin
   //      //Branch   
   //     case(dcd_op)
   //         //`OP_OTHER0:
   //         //    begin
   //         //        case(dcd_funct2)
   //         //            `OP0_JR:
   //         //                begin
   //         //                end
   //         //            `OP0_JALR:
   //         //                begin
   //         //                end
   //         //        default:
   //         //            begin
   //         //            end
   //         //        endcase
   //         //    end
   //         //`OP_OTHER1:
   //         //    begin
   //         //        case(dcd_rt)
   //         //            `OP1_BLTZ:
   //         //                begin
   //         //                end
   //         //            `OP1_BGEZ:
   //         //                begin
   //         //                end
   //         //            `OP1_BLTZAL:
   //         //                begin
   //         //                end
   //         //            `OP1_BGEZAL:
   //         //                begin
   //         //                end
   //         //        default:
   //         //            begin
   //         //            end
   //         //        endcase
   //         //`OP_BEQ:
   //         //    begin
   //         //    end
   //         //`OP_BNE:
   //         //    begin
   //         //    end
   //         //`OP_BLEZ:
   //         //    begin
   //         //    end
   //         //`OP_BGTZ:
   //         //    begin
   //         //    end
   //         //`OP_J:
   //         //    begin
   //         //    end
   //         //`OP_JAL:
   //         //    begin
   //         //    end
   //         default:
   //             begin
   //             end
   //     endcase
   //end

   //always @(posedge clk or negedge reset) begin
   //      //Load/Store   
   //     case(dcd_op)
   //         `OP_LB:
   //             begin
   //             end
   //         `OP_LH:
   //             begin
   //             end
   //         `OP_LW:
   //             begin
   //             end
   //         `OP_LBU:
   //             begin
   //             end
   //         `OP_LHU:
   //             begin
   //             end
   //         `OP_SB:
   //             begin
   //             end
   //         `OP_SH:
   //             begin
   //             end
   //         `OP_SW:
   //             begin
   //             end
   //         default:
   //             begin
   //             end
   //     endcase
   //end

   // of Verilog-Mode's power -- undeclared nets get AUTOWIREd up when we
   // run 'make auto'.
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [3:0]		alu__sel;		// From Decoder of mips_decode.v
   wire			ctrl_RI;		// From Decoder of mips_decode.v
   wire			ctrl_Sys;		// From Decoder of mips_decode.v
   wire			ctrl_we;		// From Decoder of mips_decode.v
   // End of automatics

   // Generate control signals
   mips_decode Decoder(/*AUTOINST*/
		       // Outputs
		       .ctrl_we		(ctrl_we),
		       .ctrl_Sys	(ctrl_Sys),
		       .ctrl_RI		(ctrl_RI),
		       .alu__sel	(alu__sel[3:0]),
		       // Inputs
		       .dcd_op		(dcd_op[5:0]),
                       .alu_mux_sel     (alu_mux_sel),
                       .regsrc          (regsrc),
                       .regdst          (regdst),
                       .op_sel          (op_sel),
                       .sign_ext        (sign_ext),
                       .shamt           (shamt),
		       .dcd_funct2	(dcd_funct2[5:0]));
 
   // Register File
   // Instantiate the register file from regfile.v here.
   // Don't forget to hookup the "halted" signal to trigger the register dump 
   regfile Regfile(
                    //Outputs
                    .rs_data            (rs_data[31:0]),
                    .rt_data            (rt_data[31:0]),
                    //Inputs
                    .rs_num             (dcd_rs[4:0]),
                    .rt_num             (dcd_rt[4:0]),
                    .rd_num             (dcd_rd[4:0]),
                    .rd_data            (rd_data[31:0]),
                    .rd_we              (ctrl_we),
                    .clk                (clk),
                    .rst_b              (rst_b),
                    .halted             (halted));
 
   // synthesis translate_off
   initial begin
     // Delete this block when you are ready to try for real
     $display(""); 
     $display(""); 
     $display(""); 
     $display(""); 
     $display(">>>>> This works much better after you have hooked up the reg file. <<<<<");
     $display(""); 
     $display(""); 
     $display(""); 
     $display(""); 
     $finish;
   end
   // synthesis translate_on

   // Execute
   mips_ALU ALU(.alu__out(op_dst), 
                .alu__op1(op1),
                .alu__op2(op2),
                .alu__sel(alu__sel));
 
   // Miscellaneous stuff (Exceptions, syscalls, and halt)
   exception_unit EU(.exception_halt(exception_halt), .pc(pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(load_ex_regs),
                     .load_bva(load_bva), .load_bva_sel(load_bva_sel),
                     .cause(cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   assign r_v0 = 32'h0a; // Good enough for now. To support syscall for real,
                         // you should read the syscall
                         // argument from $v0 of the register file 

   syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc), .clk(clk), .Sys(ctrl_Sys),
                   .r_v0(r_v0), .rst_b(rst_b));
   assign        internal_halt = exception_halt | syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   register #(32, 0) EPCReg(epc, pc, clk, load_ex_regs, rst_b);
   register #(32, 0) CauseReg(cause,
                              {25'b0, cause_code, 2'b0}, 
                              clk, load_ex_regs, rst_b);
   register #(32, 0) BadVAddrReg(bad_v_addr, pc, clk, load_bva, rst_b);

endmodule // mips_core


////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed
////
module mips_ALU(alu__out, alu__op1, alu__op2, alu__sel);

   output [31:0] alu__out;
   input [31:0]  alu__op1, alu__op2;
   input [3:0]   alu__sel;
   //reg [31:0]    aluop;
   
   always @(*) begin
     case(alu__sel)
         4'b0000: //ADD
              //adder AdderUnit(alu__out, alu__op1, alu__op2, alu__sel[0]);
              alu__out = alu__op1 + alu__op2;
         4'b0001: //SUB
              //adder1 AdderUnit(alu__out, alu__op1, alu__op2, alu__sel[0]);
              alu__out = alu__op1 - alu__op2;
         4'b0010: //SR
              alu__out = alu__op1>>alu__op2;
         4'b0010: //SRA
              alu__out = alu__op1>>>alu__op2;
         4'b0110: //SL
              alu__out = alu__op1<<alu__op2;
         4'b1000: //AND
              alu__out = alu__op1 & alu__op2;
         4'b1001: //OR
              alu__out = alu__op1 | alu__op2;
         4'b1010: //NOR
              alu__out = ~(alu__op1 | alu__op2);
         4'b1011: //XOR
              alu__out = alu__op1 ^ alu__op2;
         4'b1111: //SLT
              alu__out = ((alu__op1 < alu__op2) ? 1 : 0); 
     endcase
     //alu__out = aluop;
   end

endmodule //mips_ALU

//module Branch(pc_out, pc_in, rd_data, rt_data, rs_data, dcd_se_offset);
// 
//   output reg [31:0] pc_out;
//   input reg [31:0] pc_in

//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable)
       q <= d;

endmodule // register


////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(out, in1, in2, sub);
   output [31:0] out;
   input [31:0]  in1, in2;
   input         sub;

   assign        out = sub?(in1 - in2):(in1 + in2);

endmodule // adder


////
//// add_const: An adder that adds a fixed constant value
////
//// out (output) - adder result
//// in  (input)  - Operand
////
module add_const(out, in);

   parameter add_value = 1;

   output   [31:0] out;
   input    [31:0] in;

   assign   out = in + add_value;

endmodule // adder

// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
