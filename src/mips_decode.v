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
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, alu_mux_sel, regsrc, regdst, sign_ext, shamt, op_sel,
   // Inputs
   dcd_op, dcd_funct2
   );

   input       [5:0] dcd_op, dcd_funct2;
   output reg        ctrl_we, ctrl_Sys, ctrl_RI;
   output reg  [3:0] alu__sel;
   output reg 	     alu_mux_sel;
   output reg  [7:0] regsrc;
   output reg        regdst;
   output reg        sign_ext;
   output reg        shamt;
   output reg        op_sel;
    

   always @(*) begin
        //ALU   
        alu__sel = 4'hx;
        ctrl_we = 1'b0;
        ctrl_Sys = 1'b0;
        ctrl_RI = 1'b0;
        alu_mux_sel = 1'b0;
        regsrc = 8'b0;
        sign_ext = 1'b0;
        shamt = 1'b0;
        regdst = 1'b0;
        op_sel = 1'b1;

        case(dcd_op)
	    `OP_OTHER0:
	        begin
                    case(dcd_funct2)
                        `OP0_SYSCALL:
                            ctrl_Sys = 1'b1;
                        //`OP0_BREAK:
                        //    begin
                        //    end
                        `OP0_ADD:
                            begin
                                alu__sel = `ALU_ADD;
                                ctrl_we = 1'b1;
                            end
                        `OP0_SLL:
                            begin
		                alu__sel  = `ALU_SL;
		                ctrl_we = 1'b1;
			        regsrc = 8'b00100000;
                                op_sel = 1'b0;
                            end
                        `OP0_SRL:
                            begin
		                alu__sel = `ALU_SR;
		                ctrl_we = 1'b1;
			        regsrc = 8'b00100000;
			        op_sel = 1'b0;
                            end
                        `OP0_SRA:
                            begin
		                alu__sel = `ALU_SRA;
		                ctrl_we = 1'b1;
			        regsrc = 8'b00100000;
                                op_sel = 1'b0;
                            end
                        `OP0_SLLV:
                            begin
		                alu__sel = `ALU_SL;
		                ctrl_we = 1'b1;
                                regsrc = 8'b10000000;
                                op_sel = 1'b0;
                            end
                        `OP0_SRLV:
                            begin
                                alu__sel = `ALU_SR;
                                ctrl_we = 1'b1;
                                op_sel = 1'b0;
                            end
                        `OP0_SRAV:
                            begin
		                alu__sel = `ALU_SRA;
		                ctrl_we = 1'b1;
                                op_sel = 1'b0;
                            end
                        `OP0_ADDU:
                            begin
		                alu__sel = `ALU_ADD;
		                ctrl_we = 1'b1;
                            end
                        `OP0_SUB:
                            begin
		                alu__sel = `ALU_SUB;
		                ctrl_we = 1'b1;
                            end
                        `OP0_SUBU:
                            begin
		                alu__sel = `ALU_SUB;
		                ctrl_we = 1'b1;
                            end
                        `OP0_AND:
                            begin
		                alu__sel = `ALU_AND;
		                ctrl_we = 1'b1;
                            end
                        `OP0_OR:
                            begin
		                alu__sel = `ALU_OR;
		                ctrl_we = 1'b1;
                            end
                        `OP0_NOR:
                            begin
		                alu__sel = `ALU_NOR;
		                ctrl_we = 1'b1;
                            end
                        `OP0_SLT:
                            begin
		                alu__sel = `ALU_SL;
		                ctrl_we = 1'b1;
                            end
                        `OP0_MFHI:
		            ctrl_we = 1'b1;
                        //`OP0_MFLO:
                        //    begin
                        //    end
                        //`OP0_MTLO:
                        //    begin
                        //    end
                        //`OP0_MTHI:
                        //    begin
                        //    end
                        `OP0_SLTU:
                            begin
		                alu__sel = `ALU_SL;
		                ctrl_we = 1'b1;
                            end
                        `OP0_XOR:
                            begin
		                alu__sel = `ALU_XOR;
		                ctrl_we = 1'b1;
                            end
                        `OP0_JR:
		            ctrl_we = 1'b0;
                        `OP0_JALR:
		            ctrl_we = 1'b0;
                    default:
                        ctrl_RI = 1'b1;
                    endcase
	        end
	    //`OP_OTHER1:
	    //    begin
            //        case(dcd_rt)
            //            `OP0_BLTZ:
            //                begin
            //                end
            //            `OP0_BGEZ:
            //                begin
            //                end
            //            `OP0_BLTZAL:
            //                begin
            //                end
            //            `OP0_BGEZAL:
            //                begin
            //                end
            //        default:
            //            begin
            //            end
            //        endcase
            `OP_ADDIU:
                begin
                    alu__sel = `ALU_ADD;
                    ctrl_we = 1'b1;
                    regsrc = 8'b00000001;
                end
            `OP_ADDI:
                begin
	            alu__sel = `ALU_ADD;
	            ctrl_we = 1'b1;
                    regsrc = 8'b00000001;
    	        end
            `OP_ANDI:
    	        begin
	            alu__sel = `ALU_AND;
	            ctrl_we = 1'b1;
                    regsrc = 8'b00000010;
                    regdst = 1'b1;
    	        end
	    `OP_ORI:
	        begin
 	            alu__sel = `ALU_OR;
                    ctrl_we = 1'b1;
                    regsrc = 8'b00000010;
                    regdst = 1'b1;
	        end
	    `OP_XORI:
	        begin
	            alu__sel = `ALU_XOR;
	            ctrl_we = 1'b1;
                    regsrc = 8'b00000010;
                    regdst = 1'b1;
	        end
	    `OP_SLTI:
	        begin
	            alu__sel = `ALU_SL;
	            ctrl_we = 1'b1;
	        end
	    `OP_SLTIU:
	        begin
	            alu__sel = `ALU_SL;
	            ctrl_we = 1'b1;
                    regsrc = 8'b00000001;
                    regdst = 1'b1;
	        end
	    //`OP_LB:
	    //    begin
	    //    end
	    //`OP_LH:
	    //    begin
	    //    end
	    //`OP_LW:
	    //    begin
	    //    end
	    //`OP_LBU:
	    //    begin
	    //    end
	    //`OP_LHU:
	    //    begin
	    //    end
	    //`OP_SB:
	    //    begin
	    //    end
	    //`OP_SH:
	    //    begin
	    //    end
	    //`OP_SW:
	    //    begin
	    //    end
	    //default:
	    default:
	        begin
                    ctrl_RI = 1'b1;
	        end
        endcase
   end

endmodule
