.extern RAM_END
.extern main

.section .vector_table
initial_sp: .word RAM_END
reset_handler: .word main
