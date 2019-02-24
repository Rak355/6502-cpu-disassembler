struct CPU{
  u8 opcode; //current opcode
  u16 opcode_address; //its address
  u16 fetch_address; //operand fetch address as provided by the addressing mode
  uint cpuIsRunning;
  uint inst_count;
  uint cycles; //cycles needed for this opcode
  uint stall; //stall the cpu
  u16 pc; //program counter
  u8 sp; //stack pointer
  u8 a,x,y; //GRP
  u8 C,Z,I,D,B,U,V,N; //carry,zero,interrupt-disabled,decimal,break,unused,overflow,negative
  u8 interrupt; //interrupt info
}cpu;
