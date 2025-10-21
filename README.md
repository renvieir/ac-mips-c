fazer uma funcao produto escalar, usar como examplo a funcao leaf_example do slide

f = (g*h) + (i*j) 

[g h]   [i i]
[g h] . [j j]

```asm
produto_scalar:             ; label para chamar a funcao
    addi    $sp, $sp, -4    ; avança o stack poiner
    sw      $s0, 0($sp)     ; empilha o valor de $s0

    mult    $t0, $a0, $a1   ; $t0 = g*h
    mult    $t1, $a2, $a3   ; $t1 = i*j
    add     $s0, $t0, $t1   ; f   = $t0 + $t1

    add     $v0, $s0, $zero ; coloca resultado em $v0

    lw         $s0, 0($sp)  ; restaura $s0
    addi    $sp, $sp, 4
    jr      $ra
```