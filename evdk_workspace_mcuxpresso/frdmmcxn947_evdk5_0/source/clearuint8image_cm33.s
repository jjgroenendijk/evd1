/**
  ******************************************************************************
  * File    clearuint8image_cm33.s
  * Authors ...
  * Version V4.0
  * Date    09 October 2024
  * Brief   ARM Cortex-M33 assembler routine to set a byte sequence to 0.
  *         Assumptions:
  *             - aligned data
  *             - uint8_t image type
  *             - number of data bytes in the image is divisible by 32
  *
  *         ARM Cortex-M33 instruction set can be found here:
  *         https://developer.arm.com/documentation/100235/0100
  *
  *         Note: For using this function declare the following function
  *               prototype external in the project:
  *
  *               extern void clearuint8image_cm33(const image_t *img);
  *
  *         Example usage:
  *
  *               clearuint8image_cm33(img);
  *
  *
  * Copyright (C) 2024 HAN University of Applied Sciences. All Rights Reserved.
  *
  * Permission is hereby granted, free of charge, to any person obtaining a
  * copy of this software and associated documentation files (the "Software"),
  * to deal in the Software without restriction, including without limitation
  * the rights to use, copy, modify, merge, publish, distribute, sublicense,
  * and/or sell copies of the Software, and to permit persons to whom the
  * Software is furnished to do so, subject to the following conditions:
  *
  * The above copyright notice and this permission notice shall be included in
  * all copies or substantial portions of the Software.
  *
  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
  * IN THE SOFTWARE.
  ******************************************************************************
  */
        .syntax unified
        .cpu cortex-m33
        .thumb

        .section .text.clearUint8Image_cm33
        .weak clearUint8Image_cm33
        .type clearUint8Image_cm33, %function

clearUint8Image_cm33:

        // r0-r3 : function arguments and scratch registers
        //         caller-saved: must be saved before this procedure is called
        //         r0-r1 are also the result registers
        // r4-r11: local variables
        //         callee-saved: must be saved by this procedure
        // r12   : scratch register
        //         caller-saved: must be saved before this procedure is called
        // r13   : Stack Pointer
        // r14   : Link Register
        // r15   : Program Counter


		

		// Save callee-saved registers
		PUSH    {r4-r11, lr}

	    // r0 = pointer to image_t
	    // image_t layout:
	    //   offset  0: cols (int32_t)
	    //   offset  4: rows (int32_t)
	    //   offset  8: type (eImageType) (unused here)
	    //   offset 12: data pointer (uint8_t*)

	    // Load cols from offset 0 into register r4
	    // r4 = image->cols
	    LDR     r4, [r0, #0]

	    // Load rows from offset 4 into register r1
	    // r1 = image->rows
	    LDR     r1, [r0, #4]

	    // Multiply cols * rows = amount of pixels. Save in r1
	    // r1 = r4 * r1
	    MUL     r1, r4, r1

	    // Load data pointer from offset 12 into register r0
	    // r0 = image->data
	    LDR     r0, [r0, #12]

	    // Clear registers r4-r11
	    MOV     r4,  #0
	    MOV     r5,  #0
	    MOV     r6,  #0
	    MOV     r7,  #0
	    MOV     r8,  #0
	    MOV     r9,  #0
	    MOV     r10, #0
	    MOV     r11, #0

	LOOP_START:

		// STMIA = "Store Multiple, Increment After."
     	// Takes the address in r0
     	// Stores the contents of r4, r5, r6, r7, r8, r9, r10, and r11 at that address
		// After each register is stored, r0 is incremented by 4 bytes.
		// So after 8 registers, r0 has moved forward by 32 bytes.

		// Store r4-r11 at r0
	    STMIA   r0!, {r4-r11}

		// Subtract 32 from r1 (number of written pixels)
	    SUBS    r1, r1, #32

	    // If r1 is not zero, jump to LOOP_START
	    BNE     LOOP_START

	    // Restore callee-saved registers
	    POP     {r4-r11, lr}

	    // Return
	    BX      lr
