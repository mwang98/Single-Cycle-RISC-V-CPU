.data
n: .word 10 
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    

    recur:
        addi sp, sp, -8
        sw   x1, 4(sp)      #store return address
        sw   x10, 0(sp)     #store n
        slti  x8, x10, 2    #x8 = x10 < 2) ? 1:0
        beq   x8, x0, L1    #x8 = 0 (n>=2) , go to L1
        addi x10, x0, 4     #else, return value = 4 
        addi sp, sp, 8      #pop stack
        jalr x0, 0(x1)      #return
    L1:
        srli x10, x10, 1    # n = n/2
        jal  x1,  recur     # call T(n/2)
        addi x5, x10, 0
        lw   x29, 0(sp)     # n
        lw   x1, 4(sp)
        addi sp, sp, 8
        slli  x6, x5, 1     # 2T(n/2)
        slli  x7, x29, 3    # 8n
        add  x10, x6, x7    # T(n) = 2T(n/2) + 8n
        addi  x10, x10, 5   # T(n) = 2T(n/2) + 8n +5
        jalr x0, 0(x1)      # return

    addi x5, x10, 0         #dummy code, branch instruction will not jump to here. 
    
# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   x10, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   x10, 4(t0)
    addi a0,x0,10
    ecall