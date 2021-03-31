.data
 
.align 2

array:
.word 5, -12, -3, 0, 9, -4, 8

.text 

#   $a0: base address; $a1: N
#   $t0: i
#   $t1: j
#   $t2: value of A[i] 
#   $t3: value of A[j]
#   $t8: address of A[i]
#   $t9: address of A[j]

beq $0, $0, main
 
sort:
    add $t0, $0, $0  # i = 0
    add $t8, $0, $a0 # *a[i]

for1:
    beq $t0, $a1, end # i = N -> end
    addi $t1, $t0, 1  # j = i + 1
    addi $t9, $t8, 0x4 # *a[j]

for2:
    beq $t1, $a1, endJfor # j = N -> endJfor
    lw $t2, ($t8) # a[i]
    lw $t3, ($t9) # a[j]
    slt $t4, $t3, $t2 # a[i] > a[j] => $t4 = 1 => swap
    beq $t4, $0, next # no
    sw $t2, ($t9) #swap
    sw $t3, ($t8)

next:
    addi $t1, $t1, 1   # j++
    addi $t9, $t9, 0x4 # next elem in array
    beq $0, $0, for2   # always

endJfor:
    addi $t0, $t0, 1   # i++
    addi $t8, $t8, 0x4 # 
    beq $0, $0, for1

end:
    jr $ra

main:
     lw $a0, 0x0000000C
     lw $a1, 0x00000008
     #addi $a1, $0, 7
     #la $a0, array 
     jal sort
     
#0x00000008 = N
#0x0000000C = address