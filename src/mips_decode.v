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

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// mips_decode: Decode MIPS instructions
////
//// op      (input)  - Instruction opcode
//// funct2  (input)  - Instruction minor opcode
//// rt      (input)  - Instruction minor opcode
//// alu_sel (output) - Selects the ALU function
//// we      (output) - Write to the register file
//// Sys     (output) - System call exception
//// RI      (output) - Reserved instruction exception
////
module mips_decode(/*AUTOARG*/
   // Outputs
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, alu_mux_sel, reg_sel
   // Inputs
   dcd_op, dcd_funct2
   );

   input       [5:0] dcd_op, dcd_funct2;
   output reg        ctrl_we, ctrl_Sys, ctrl_RI;
   output reg  [3:0] alu__sel;
   output reg        alu_mux_sel;  
   output reg        reg_sel;

   always @(*) begin
     alu__sel = 4'hx;
     ctrl_we = 1'b0;
     ctrl_Sys = 1'b0;
     ctrl_RI = 1'b0;
     alu_mux_sel = 1'b0;
     reg_sel = 1'b0;// GPR[rd] by default
     case(dcd_op)
       `OP_OTHER0:
         case(dcd_funct2)
           `OP0_SYSCALL:
                ctrl_Sys = 1'b1;
	   `OP0_ADD:
                alu__sel = `ALU_ADD;
                alu_mux_sel = 1'b1;
                ctrl_we = 1'b1;
	   `OP0_ADDU:
		alu__sel = `ALU_ADD;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SUB:
		alu__sel = `ALU_SUB;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SUBU:
		alu__sel = `ALU_SUB;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_AND:
		alu__sel = `ALU_AND;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_OR:
		alu__sel = `ALU_OR;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_XOR:
		alu__sel = `ALU_XOR;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_NOR:
		alu__sel = `ALU_NOR;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SLT:
		alu__sel = `ALU_SLT;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SLTU:
		alu__sel = `ALU_SLTU;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SLL:
		alu__sel  = `ALU_SLL;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SLLV:
		alu__sel = `ALU_SLLV;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SRL:
		alu__sel = `ALU_SRL;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SRA:
		alu__sel = `ALU_SRA;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_SRAV:
		alu__sel = `ALU_SRAV;
		alu_mux_sel = 1'b1;
		ctrl_we = 1'b1;
	   `OP0_JR:
		ctrl_we = 1'b0;
	   `OP0_JALR:
		ctrl_we = 1'b0;
	   `OP0_MFHI:
		ctrl_we = 1'b1
	   `OP0_MTHI:
	   `OP0_MFLO:
	   `OP0_MTLO:
           default:
                ctrl_RI = 1'b1;
         endcase
       `OP_ADDIU:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1; //GPR[rt] selected as dest. register
         end
       `OP_ADDI:
         begin
	    alu__sel = `ALU_ADD;
	    ctrl_we = 1'b1;
 	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1; //GPR[rt] selected as dest. register
	 end
       `OP_ANDI:
	 begin
	    alu__sel = `ALU_AND;
	    ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1;
	 end
	`OP_ORI:
	  begin
 	    alu__sel = `ALU_OR;
            ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1;
	  end
	`OP_XORI:
	  begin
	    alu__sel = `ALU_XOR;
	    ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1;
	  end
	`OP_SLTI:
	  begin
	    alu__sel = `ALU_SLT;
	    ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1;
	  end
	`OP_SLTIU:
	  begin
	    alu__sel = `ALU_SLTU;
	    ctrl_we = 1'b1;
	    alu_mux_sel = 1'b0;
	    reg_sel = 1'b1;
	  end
	`OP_
       default:
         begin
            ctrl_RI = 1'b1;
         end
     endcase // case(op)
   end

endmodule
