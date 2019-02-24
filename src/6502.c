#include<types.h>
#include<6502.h>
#include<mem.h>
#include<file.h>
extern void init_files();
extern void mapper();
FILE *ROM;
FILE *LOG;
struct opcode{
  char name[4];
};
struct opcode name_arr[256]={
  /*     0,     1,     2,     3,     4,     5,     6,     7,     8,     9,     A,     B,     C,     D,     E,     F */
  /*0*/  "BRK", "ORA", "KIL", "SLO", "NOP", "ORA", "ASL", "SLO", "PHP", "ORA", "ASL", "ANC", "NOP", "ORA", "ASL", "SLO",
	/*1*/  "BPL", "ORA", "KIL", "SLO", "NOP", "ORA", "ASL", "SLO", "CLC", "ORA", "NOP", "SLO", "NOP", "ORA", "ASL", "SLO",
	/*2*/  "JSR", "AND", "KIL", "RLA", "BIT", "AND", "ROL", "RLA", "PLP", "AND", "ROL", "ANC", "BIT", "AND", "ROL", "RLA",
	/*3*/  "BMI", "AND", "KIL", "RLA", "NOP", "AND", "ROL", "RLA", "SEC", "AND", "NOP", "RLA", "NOP", "AND", "ROL", "RLA",
	/*4*/  "RTI", "EOR", "KIL", "SRE", "NOP", "EOR", "LSR", "SRE", "PHA", "EOR", "LSR", "ALR", "JMP", "EOR", "LSR", "SRE",
	/*5*/  "BVC", "EOR", "KIL", "SRE", "NOP", "EOR", "LSR", "SRE", "CLI", "EOR", "NOP", "SRE", "NOP", "EOR", "LSR", "SRE",
	/*6*/  "RTS", "ADC", "KIL", "RRA", "NOP", "ADC", "ROR", "RRA", "PLA", "ADC", "ROR", "ARR", "JMP", "ADC", "ROR", "RRA",
	/*7*/  "BVS", "ADC", "KIL", "RRA", "NOP", "ADC", "ROR", "RRA", "SEI", "ADC", "NOP", "RRA", "NOP", "ADC", "ROR", "RRA",
	/*8*/  "NOP", "STA", "NOP", "SAX", "STY", "STA", "STX", "SAX", "DEY", "NOP", "TXA", "XAA", "STY", "STA", "STX", "SAX",
	/*9*/  "BCC", "STA", "KIL", "AHX", "STY", "STA", "STX", "SAX", "TYA", "STA", "TXS", "TAS", "SHY", "STA", "SHX", "AHX",
	/*A*/  "LDY", "LDA", "LDX", "LAX", "LDY", "LDA", "LDX", "LAX", "TAY", "LDA", "TAX", "LAX", "LDY", "LDA", "LDX", "LAX",
	/*B*/  "BCS", "LDA", "KIL", "LAX", "LDY", "LDA", "LDX", "LAX", "CLV", "LDA", "TSX", "LAS", "LDY", "LDA", "LDX", "LAX",
	/*C*/  "CPY", "CMP", "NOP", "DCP", "CPY", "CMP", "DEC", "DCP", "INY", "CMP", "DEX", "AXS", "CPY", "CMP", "DEC", "DCP",
	/*D*/  "BNE", "CMP", "KIL", "DCP", "NOP", "CMP", "DEC", "DCP", "CLD", "CMP", "NOP", "DCP", "NOP", "CMP", "DEC", "DCP",
	/*E*/  "CPX", "SBC", "NOP", "ISC", "CPX", "SBC", "INC", "ISC", "INX", "SBC", "NOP", "SBC", "CPX", "SBC", "INC", "ISC",
	/*F*/  "BEQ", "SBC", "KIL", "ISC", "NOP", "SBC", "INC", "ISC", "SED", "SBC", "NOP", "ISC", "NOP", "SBC", "INC", "ISC",
};

struct Disassembler{
  char *op_name;
  u8 op1;
  u8 op2;
  uint mode;
  uint size;
  uint fetch_address;
};

long op_addr[];
struct Curr_step{
  u16 address;
  u16 pc;
  u8 mode;
};
enum interrupt{
  int_none,
  int_NMI,
  int_IRQ
};
enum addr_mode {//types of addressing modes used by the cpu
  invalid_mode, //0
  absolute, //1
  absolute_X, //2
  absolute_Y, //3
  accumulator, //4
  immediate, //5
  implied, //6
  indexed_indirect, //7
  indirect, //8
  indirect_indexed, //9
  relative, //10
  zero_page, //11
  zero_page_X, //12
  zero_page_Y //13
};
uint inst_modes[256] = { //Addressing modes for the instructions
  /*     0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F */
  /*0*/  6,7,6,7,11,11,11,11,6,5,4,5,1,1,1,1,
  /*1*/	 10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2,
  /*2*/	 1,7,6,7,11,11,11,11,6,5,4,5,1,1,1,1,
  /*3*/	 10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2,
  /*4*/	 6,7,6,7,11,11,11,11,6,5,4,5,1,1,1,1,
  /*5*/	 10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2,
  /*6*/	 6,7,6,7,11,11,11,11,6,5,4,5,8,1,1,1,
  /*7*/	 10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2,
  /*8*/	 5,7,5,7,11,11,11,11,6,5,6,5,1,1,1,1,
  /*9*/	 10,9,6,9,12,12,13,13,6,3,6,3,2,2,3,3,
  /*A*/	 5,7,5,7,11,11,11,11,6,5,6,5,1,1,1,1,
  /*B*/	 10,9,6,9,12,12,13,13,6,3,6,3,2,2,3,3,
  /*C*/	 5,7,5,7,11,11,11,11,6,5,6,5,1,1,1,1,
  /*D*/	 10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2,
  /*E*/	 5,7,5,7,11,11,11,11,6,5,6,5,1,1,1,1,
  /*F*/  10,9,6,9,12,12,12,12,6,3,6,3,2,2,2,2
};
uint inst_size[256] = { //Instruction sizes
  /*     0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F */
  /*0*/  1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*1*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*2*/  3,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*3*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*4*/  1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*5*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*6*/  1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*7*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*8*/  2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*9*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*A*/  2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*B*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*C*/  2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*D*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
  /*E*/  2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
  /*F*/  2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
};
uint inst_cycles[256] = { //Instructions cycles without conditional cycles
  /*     0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F */
  /*0*/  7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6,
  /*1*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  /*2*/  6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6,
  /*3*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  /*4*/  6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6,
  /*5*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  /*6*/  6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6,
  /*7*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  /*8*/  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
  /*9*/  2,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,
  /*A*/  2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
  /*B*/  2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
  /*C*/  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
  /*D*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
  /*E*/  2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
  /*F*/  2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
};
uint inst_Page_cycles[256] = { //Instructions cycles with page boundary crossed
  /*     0,1,2,3,4,5,6,7,8,9,A,B,C,D,E,F */
  /*0*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*1*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
  /*2*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*3*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
  /*4*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*5*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
  /*6*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*7*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
  /*8*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*9*/  1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*A*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*B*/  1,1,0,1,0,0,0,0,0,1,0,1,1,1,1,1,
  /*C*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*D*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
  /*E*/  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  /*F*/  1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
};
u8 main_mem[0xFFFF];
u8 GetFlags(){
  u8 flags = 0x0000;
	flags |= (cpu.C << 0);
	flags |= (cpu.Z << 1);
	flags |= (cpu.I << 2);
	flags |= (cpu.D << 3);
	flags |= (cpu.B << 4);
	flags |= (cpu.U << 5);
	flags |= (cpu.V << 6);
	flags |= (cpu.N << 7);
	return flags;
}
void SetFlags(u8 flags){
  cpu.C = (flags >> 0) & 1;
  cpu.Z = (flags >> 1) & 1;
	cpu.I = (flags >> 2) & 1;
	cpu.D = (flags >> 3) & 1;
	cpu.B = (flags >> 4) & 1;
	cpu.U = (flags >> 5) & 1;
	cpu.V = (flags >> 6) & 1;
  cpu.N = (flags >> 7) & 1;
}
u8 cread(u16 address){
  return main_mem[address];
}
void cwrite(u16 address, u8 value){
  main_mem[address]=value;
}
u16 cread16(u16 address){
  u8 low = (u8)cread(address);
  u8 high = (u8)cread(address+1);
  return (u16)((high<<8)|low);
}
u16 cread16bug(u16 address){
  u16 a = address;
  u16 b = ((u16)(address&0x00FF) == 0x00FF)?((u16)(address&0xFF00)):((u16)(address+1));
  u8 low = (u8)cread(a);
  u8 high = (u8)cread(b);
  return (u16)((high<<8)|low);
}
void push(u8 value){
  cwrite((u16)(0x100|cpu.sp),value);
  cpu.sp--;
}
u8 pull(){
  cpu.sp++;
  return cread(0x100|cpu.sp);
}
void push16(u16 value){
  u8 high = value>>8;
  u8 low = value&0xFF;
  push(high);
  push(low);
}
u16 pull16(){
  u8 low = pull();
  u8 high = pull();
  return (u16)((high<<8)|low);
}
uint diff_Page(u16 addr1,u16 addr2){
  if((addr1&0xFF00)!=(addr2&0xFF00))
  return true;
  else
  return false;
}
void add_branch_cycles(struct Curr_step step_data){
  cpu.cycles++;
  if(diff_Page(step_data.pc,step_data.address) == 1){
    cpu.cycles++;
  }
}
void SetZ(u8 value){
  cpu.Z = (value==0)?1:0;
}
void SetN(u8 value){
  cpu.N = (value&0x80)?1:0;
}
void SetZN(u8 value){
  cpu.Z = (value==0)?1:0;
  cpu.N = (value&0x80)?1:0;
}
void triggerNMI(){
  cpu.interrupt = int_NMI;
}
void triggerIRQ(){
  if(cpu.I == 0)
  cpu.interrupt = int_IRQ;
}

void compare(u8 a,u8 b){
  SetZN((u8)a-(u8)b);
  cpu.C = (a>=b)?1:0;
}

static u8 a,b,c,value;
void decode(u8,struct Curr_step);
typedef void execute(struct Curr_step step_data);
void adc(struct Curr_step step_data){//01 ADC
  a = cpu.a;
  b = cread(step_data.address);
  c = cpu.C;
  cpu.a = a+b+c;
  SetZN(cpu.a);
  cpu.C = (a+b+c > 0xFF)?1:0;
  cpu.V = (((a^b)&0x80)==0 && ((a^cpu.a)&0x80)!=0)?1:0;
}
void and(struct Curr_step step_data){//02 AND
  cpu.a = cpu.a & cread(step_data.address);
  SetZN(cpu.a);
}
void asl(struct Curr_step step_data){//03 ASL
  if(step_data.mode == accumulator){
    cpu.C = (cpu.a >> 7) & 1;
    cpu.a <<= 1;
    SetZN(cpu.a);
  }
  else{
    value = cread(step_data.address);
    cpu.C = (value >> 7) & 1;
    value <<= 1;
    cwrite(step_data.address, value);
    SetZN(value);
  }
}
void bcc(struct Curr_step step_data){//04 BCC
  if(cpu.C == 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void bcs(struct Curr_step step_data){//05 BCS
  if(cpu.C != 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void beq(struct Curr_step step_data){//06 BEQ
  if(cpu.Z !=0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void bit(struct Curr_step step_data){//07 BIT
  value = cread(step_data.address);
  cpu.V = (value >> 6) & 1;
  SetZ(value & cpu.a);
  SetN(value);
}
void bmi(struct Curr_step step_data){//08 BMI
  if(cpu.N != 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void bne(struct Curr_step step_data){//09 BNE
  if(cpu.Z == 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void bpl(struct Curr_step step_data){//10 BPL
  if(cpu.N == 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void brk(struct Curr_step step_data){//11 BRK
  push16(cpu.pc);
  decode(0x08,step_data); //php
  decode(0x78,step_data); //sei
  cpu.pc = cread16(0xFFFE);
}
void bvc(struct Curr_step step_data){//12 BVC
  if(cpu.V == 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void bvs(struct Curr_step step_data){//12 BVS
  if(cpu.V != 0){
    cpu.pc = step_data.address;
    add_branch_cycles(step_data);
  }
}
void clc(struct Curr_step step_data){//13 CLC
  cpu.C = 0;
}
void cld(struct Curr_step step_data){//14 CLD
  cpu.D = 0;
}
void cli(struct Curr_step step_data){//15 CLI
  cpu.I = 0;
}
void clv(struct Curr_step step_data){//16 CLV
  cpu.V = 0;
}
void cmp(struct Curr_step step_data){//17 CMP
  value = (u8)cread(step_data.address);
  compare(cpu.a, value);
}
void cpx(struct Curr_step step_data){//18 CPX
  value = cread(step_data.address);
  compare(cpu.x,value);
}
void cpy(struct Curr_step step_data){//19 CPY
  value = cread(step_data.address);
  compare(cpu.y,value);
}
void dec(struct Curr_step step_data){//20 DEC
  value = cread(step_data.address) - 1;
  cwrite(step_data.address,value);
  SetZN(value);
}
void dex(struct Curr_step step_data){//21 DEX
  cpu.x--;
  SetZN(cpu.x);
}
void dey(struct Curr_step step_data){//22 DEY
  cpu.y--;
  SetZN(cpu.y);
}
void eor(struct Curr_step step_data){//23 EOR
  cpu.a = cpu.a^cread(step_data.address);
  SetZN(cpu.a);
}
void inc(struct Curr_step step_data){//24 INC
  value = cread(step_data.address) + 1;
  cwrite(step_data.address,value);
  SetZN(value);
}
void inx(struct Curr_step step_data){//25 INX
  cpu.x++;
  SetZN(cpu.x);
}
void iny(struct Curr_step step_data){//26 INY
  cpu.y++;
  SetZN(cpu.y);
}
void jmp(struct Curr_step step_data){//27 JMP
  cpu.pc = step_data.address;
}
void jsr(struct Curr_step step_data){//28 JSR
  push16(cpu.pc - 1);
  cpu.pc = step_data.address;
}
void lda(struct Curr_step step_data){//29 LDA
  cpu.a = (u8)cread(step_data.address);
  SetZN(cpu.a);
}
void ldx(struct Curr_step step_data){//30 LDX
  cpu.x = cread(step_data.address);
  SetZN(cpu.x);
}
void ldy(struct Curr_step step_data){//31 LDY
  cpu.y = cread(step_data.address);
  SetZN(cpu.y);
}
void lsr(struct Curr_step step_data){//32 LSR
  if(step_data.mode == accumulator){
    cpu.C = cpu.a & 1;
    cpu.a >>=(u8)1;
    SetZN(cpu.a);
  }
  else{
    value = cread(step_data.address);
    cpu.C = value & 1;
    value >>=(u8)1;
    cwrite(step_data.address,value);
    SetZN(value);
  }
}
void nop(struct Curr_step step_data){//33 NOP

}
void ora(struct Curr_step step_data){//34 ORA
  cpu.a = cpu.a | cread(step_data.address);
  SetZN(cpu.a);
}
void pha(struct Curr_step step_data){//35 PHA
  push(cpu.a);
}
void php(struct Curr_step step_data){//36 PHP
  push(GetFlags() | 0x10); //TODO review this and plp
}
void pla(struct Curr_step step_data){//37 PLA
  cpu.a = pull();
  SetZN(cpu.a);
}
void plp(struct Curr_step step_data){//38 PLP
  SetFlags((pull()&0xEF) | 0x20); //TODO modified
}
void rol(struct Curr_step step_data){//39 ROL
  if(step_data.mode == accumulator){
    c = cpu.C;
    cpu.C = (u8)((cpu.a >> 7) & 1);
    cpu.a = (u8)((cpu.a << 1) | c);
    SetZN(cpu.a);
  }
  else{
    c = cpu.C;
    value = cread(step_data.address);
    cpu.C = (u8)((value >> 7) & 1);
    value = (u8)((value << 1) | c);
    cwrite(step_data.address,value);
    SetZN(value);
  }
}
void ror(struct Curr_step step_data){//40 ROR
  if(step_data.mode == accumulator){
    c = cpu.C;
    cpu.C = cpu.a & 0x0001;
    cpu.a = (u8)((u8)(cpu.a >> 1) | (u8)(c << 7));
    SetZN(cpu.a);
  }
  else{
    c = cpu.C;
    value = cread(step_data.address);
    cpu.C = value & 1;
    value = (u8)((u8)(value >> 1) | (u8)(c << 7));
    cwrite(step_data.address,value);
    SetZN(value);
  }
}
void rti(struct Curr_step step_data){//41 RTI
  SetFlags((pull() & 0xEF) | 0x20);
  cpu.pc = pull16();
}
void rts(struct Curr_step step_data){//42 RTS
  cpu.pc = (u16)(pull16() + 1);
}
void sbc(struct Curr_step step_data){//43 SBC
  a = cpu.a;
  b = cread(step_data.address);
  c = cpu.C;
  cpu.a = a - b - (1 - c);
  SetZN(cpu.a);
  cpu.C = (a-b-(1-c) >= 0)?1:0;
  cpu.V = (((a^b) & 0x80) != 0 && ((a^cpu.a) & 0x80) != 0)?1:0; //TODO might be wrong
}
void sec(struct Curr_step step_data){//44 SEC
  cpu.C = 1;
}
void sed(struct Curr_step step_data){//45 SED
  cpu.D = 1;
}
void sei(struct Curr_step step_data){//46 SEI
  cpu.I = 1;
}
void sta(struct Curr_step step_data){//47 STA
  cwrite(step_data.address,cpu.a);
}
void stx(struct Curr_step step_data){//48 STX
  cwrite(step_data.address,cpu.x);
}
void sty(struct Curr_step step_data){//49 STY
  cwrite(step_data.address,cpu.y);
}
void tax(struct Curr_step step_data){//50 TAX
  cpu.x = cpu.a;
  SetZN(cpu.x);
}
void tay(struct Curr_step step_data){//51 TAY
  cpu.y = cpu.a;
  SetZN(cpu.y);
}
void tsx(struct Curr_step step_data){//52 TSX
  cpu.x = cpu.sp;
  SetZN(cpu.x);
}
void txa(struct Curr_step step_data){//53 TXA
  cpu.a = cpu.x;
  SetZN(cpu.a);
}
void txs(struct Curr_step step_data){//54 TXS
  cpu.sp = cpu.x;
}
void tya(struct Curr_step step_data){//55 TYA
  cpu.a = cpu.y;
  SetZN(cpu.a);
}

//UNOFFICIAL OPCODES
void lax(struct Curr_step step_data){//LAX
  cpu.a = cread(step_data.address);
  cpu.x = cread(step_data.address);
  SetZ(cpu.a & cpu.x);
  SetN(cpu.a | cpu.x);
}
void sax(struct Curr_step step_data){//SAX
  value = (u8)((u8)cpu.a & (u8)cpu.x);
  cwrite(step_data.address,value);
}
void dcp(struct Curr_step step_data){//DCP
  value = cread(step_data.address) - 1; //DEC
  cwrite(step_data.address,value);
  SetZN(value);

  value = (u8)cread(step_data.address); //CMP
  compare(cpu.a, value);
}
void isc(struct Curr_step step_data){//ISB/ISC
  value = cread(step_data.address) + 1; //INC
  cwrite(step_data.address,value);
  SetZN(value);

  a = cpu.a; //SBC
  b = cread(step_data.address);
  c = cpu.C;
  cpu.a = a - b - (1 - c);
  //cpu.a = a - b;
  SetZN(cpu.a);
  cpu.C = (a-b >= 0)?1:0;
  cpu.V = (((a^b) & 0x80) != 0 && ((a^cpu.a) & 0x80) != 0)?1:0; //TODO might be wrong
}
void slo(struct Curr_step step_data){//SLO/ASO
  if(step_data.mode == accumulator){ //ASL
    cpu.C = (cpu.a >> 7) & 1;
    cpu.a <<= 1;
    SetZN(cpu.a);
  }
  else{
    value = cread(step_data.address);
    cpu.C = (value >> 7) & 1;
    value <<= 1;
    cwrite(step_data.address, value);
    SetZN(value);
  }

  cpu.a = cpu.a | cread(step_data.address); //ORA
  SetZN(cpu.a);
}
void rla(struct Curr_step step_data){//RLA
  if(step_data.mode == accumulator){ //ROL
    c = cpu.C;
    cpu.C = (cpu.a >> 7) & 1;
    cpu.a = (cpu.a << 1) | c;
    SetZN(cpu.a);
  }
  else{
    c = cpu.C;
    value = cread(step_data.address);
    cpu.C = (value >> 7) & 1;
    value = (value << 1) | c;
    cwrite(step_data.address,value);
    SetZN(value);
  }

  cpu.a = cpu.a & cread(step_data.address); //AND
  SetZN(cpu.a);
}
void sre(struct Curr_step step_data){//SRE
  if(step_data.mode == accumulator){ //LSR
    cpu.C = cpu.a & 1;
    cpu.a >>= 1;
    SetZN(cpu.a);
  }
  else{
    value = cread(step_data.address);
    cpu.C = value & 1;
    value >>=1;
    cwrite(step_data.address,value);
    SetZN(value);
  }

  cpu.a = cpu.a^cread(step_data.address); //EOR
  SetZN(cpu.a);
}
void rra(struct Curr_step step_data){//RRA
  if(step_data.mode == accumulator){ //ROR
    c = cpu.C;
    cpu.C = cpu.a & 1;
    cpu.a = (cpu.a >> 1) | (c << 7);
    SetZN(cpu.a);
  }
  else{
    c = cpu.C;
    value = cread(step_data.address);
    cpu.C = value & 1;
    value = (value >> 1) | (c << 7);
    cwrite(step_data.address,value);
    SetZN(value);
  }

  a = cpu.a; //ADC
  b = cread(step_data.address);
  c = cpu.C;
  cpu.a = a+b+c;
  SetZN(cpu.a);
  if(a+b+c > 0xFF) cpu.C = 1;
  else cpu.C = 0;
  if( ((a^b)&0x80) == 0 && ((a^cpu.a)&0x80) != 0) cpu.V = 1;
  else cpu.V = 0;
}

void kil(struct Curr_step step_data){}
void anc(struct Curr_step step_data){}
void alr(struct Curr_step step_data){}
void tas(struct Curr_step step_data){}
void axs(struct Curr_step step_data){}
void ahx(struct Curr_step step_data){}
void arr(struct Curr_step step_data){}
void shy(struct Curr_step step_data){}
void xaa(struct Curr_step step_data){}
void las(struct Curr_step step_data){}
void shx(struct Curr_step step_data){}
/*un-official opcodes
  ahx
  alr
  anc
  arr
  axs
  dcp // done remove
  isc // done remove
  kil
  las
  lax // done remove
  rla // done remove
  rra // done remove
  sax // done remove
  shx
  shy
  slo // done remove
  sre // done remove
  tas
  xaa
*/
//interrupt handlers
void NMI(){
  struct Curr_step step_data = {0,0};
  push16(cpu.pc);
  decode(0x08,step_data); //php
  cpu.pc = cread16(0xFFFA);
  cpu.I = 1;
  cpu.cycles += 7;
}
void IRQ(){
  struct Curr_step step_data = {0,0};
  push16(cpu.pc);
  decode(0x08,step_data); //php
  cpu.pc = cread16(0xFFFE);
  cpu.I = 1;
  cpu.cycles += 7;
}
void RESET(){
  cpu.cpuIsRunning = true;
  cpu.inst_count = 1;
  cpu.cycles = 7;
  cpu.stall = 0;
  //cpu.pc = cread16(0xFFFC);
  cpu.pc = 0xC000; //uncomment this,comment above line and test nestest.nes file
  cpu.sp = 0xFD;
  cpu.a = 0x00;
  cpu.x = 0x00;
  cpu.y = 0x00;
  SetFlags(0x24);
}
long op_addr[265] = {//opcode address
    (long)&brk, (long)&ora, (long)&kil, (long)&slo, (long)&nop, (long)&ora, (long)&asl, (long)&slo, (long)&php, (long)&ora, (long)&asl, (long)&anc, (long)&nop, (long)&ora, (long)&asl, (long)&slo,
		(long)&bpl, (long)&ora, (long)&kil, (long)&slo, (long)&nop, (long)&ora, (long)&asl, (long)&slo, (long)&clc, (long)&ora, (long)&nop, (long)&slo, (long)&nop, (long)&ora, (long)&asl, (long)&slo,
		(long)&jsr, (long)&and, (long)&kil, (long)&rla, (long)&bit, (long)&and, (long)&rol, (long)&rla, (long)&plp, (long)&and, (long)&rol, (long)&anc, (long)&bit, (long)&and, (long)&rol, (long)&rla,
		(long)&bmi, (long)&and, (long)&kil, (long)&rla, (long)&nop, (long)&and, (long)&rol, (long)&rla, (long)&sec, (long)&and, (long)&nop, (long)&rla, (long)&nop, (long)&and, (long)&rol, (long)&rla,
		(long)&rti, (long)&eor, (long)&kil, (long)&sre, (long)&nop, (long)&eor, (long)&lsr, (long)&sre, (long)&pha, (long)&eor, (long)&lsr, (long)&alr, (long)&jmp, (long)&eor, (long)&lsr, (long)&sre,
		(long)&bvc, (long)&eor, (long)&kil, (long)&sre, (long)&nop, (long)&eor, (long)&lsr, (long)&sre, (long)&cli, (long)&eor, (long)&nop, (long)&sre, (long)&nop, (long)&eor, (long)&lsr, (long)&sre,
		(long)&rts, (long)&adc, (long)&kil, (long)&rra, (long)&nop, (long)&adc, (long)&ror, (long)&rra, (long)&pla, (long)&adc, (long)&ror, (long)&arr, (long)&jmp, (long)&adc, (long)&ror, (long)&rra,
		(long)&bvs, (long)&adc, (long)&kil, (long)&rra, (long)&nop, (long)&adc, (long)&ror, (long)&rra, (long)&sei, (long)&adc, (long)&nop, (long)&rra, (long)&nop, (long)&adc, (long)&ror, (long)&rra,
		(long)&nop, (long)&sta, (long)&nop, (long)&sax, (long)&sty, (long)&sta, (long)&stx, (long)&sax, (long)&dey, (long)&nop, (long)&txa, (long)&xaa, (long)&sty, (long)&sta, (long)&stx, (long)&sax,
		(long)&bcc, (long)&sta, (long)&kil, (long)&ahx, (long)&sty, (long)&sta, (long)&stx, (long)&sax, (long)&tya, (long)&sta, (long)&txs, (long)&tas, (long)&shy, (long)&sta, (long)&shx, (long)&ahx,
		(long)&ldy, (long)&lda, (long)&ldx, (long)&lax, (long)&ldy, (long)&lda, (long)&ldx, (long)&lax, (long)&tay, (long)&lda, (long)&tax, (long)&lax, (long)&ldy, (long)&lda, (long)&ldx, (long)&lax,
		(long)&bcs, (long)&lda, (long)&kil, (long)&lax, (long)&ldy, (long)&lda, (long)&ldx, (long)&lax, (long)&clv, (long)&lda, (long)&tsx, (long)&las, (long)&ldy, (long)&lda, (long)&ldx, (long)&lax,
		(long)&cpy, (long)&cmp, (long)&nop, (long)&dcp, (long)&cpy, (long)&cmp, (long)&dec, (long)&dcp, (long)&iny, (long)&cmp, (long)&dex, (long)&axs, (long)&cpy, (long)&cmp, (long)&dec, (long)&dcp,
		(long)&bne, (long)&cmp, (long)&kil, (long)&dcp, (long)&nop, (long)&cmp, (long)&dec, (long)&dcp, (long)&cld, (long)&cmp, (long)&nop, (long)&dcp, (long)&nop, (long)&cmp, (long)&dec, (long)&dcp,
		(long)&cpx, (long)&sbc, (long)&nop, (long)&isc, (long)&cpx, (long)&sbc, (long)&inc, (long)&isc, (long)&inx, (long)&sbc, (long)&nop, (long)&sbc, (long)&cpx, (long)&sbc, (long)&inc, (long)&isc,
		(long)&beq, (long)&sbc, (long)&kil, (long)&isc, (long)&nop, (long)&sbc, (long)&inc, (long)&isc, (long)&sed, (long)&sbc, (long)&nop, (long)&isc, (long)&nop, (long)&sbc, (long)&inc, (long)&isc,
};
void decode(u8 opcode,struct Curr_step step_data){ //decode and execute are simultaneous
  execute *e = (execute*)(long)op_addr[opcode];
  e(step_data);
}
void mode_addressing(uint mode,u16 *address,uint *pageCrossed){
  switch(mode){
    case absolute:
      *address = (uint16_t)cread16(cpu.pc + 1);
      break;
    case absolute_X:{
      *address = cread16(cpu.pc + 1) + (uint16_t)cpu.x;
      *pageCrossed = diff_Page(*address - cpu.x, *address);
      break;
    }
    case absolute_Y:{
      *address = cread16(cpu.pc + 1) + (uint16_t)cpu.y;
      *pageCrossed = diff_Page(*address - cpu.y, *address);
      break;
    }
    case accumulator: *address = 0;break;
    case immediate: *address = (cpu.pc + 1);break;
    case implied: *address = (cpu.pc + 0);break;
    case indexed_indirect:{
      *address = (uint16_t)(cread(cpu.pc+1)+cpu.x) & 0x00FF;
      *address = (uint16_t)cread16bug(*address);
      break;
    }
    case indirect: *address = cread16bug(cread16(cpu.pc+1));break;
    case indirect_indexed:{
      *address = cread16bug(cread(cpu.pc+1)) + cpu.y;
      *pageCrossed = diff_Page(*address - cpu.y, *address);
      break;
    }
    case relative:{ // PC moves to next inst and relative offset is applied to the new PC address
      uint8_t offset = cread(cpu.pc + 1);
      if(offset < 0x80) *address = cpu.pc + 2 + offset;
      else *address = cpu.pc + 2 + offset - 0x100;
      break;
    }
    case zero_page: *address = (uint8_t)(cread(cpu.pc + 1))&0x00FF;break;
    case zero_page_X: *address = (uint8_t)(cread(cpu.pc + 1) + cpu.x)&0x00FF;break;
    case zero_page_Y: *address = (uint8_t)(cread(cpu.pc + 1) + cpu.y)&0x00FF;break;
  }
  cpu.fetch_address = *address;
}
void check_interrupt(){
  switch(cpu.interrupt){
    case int_NMI: NMI(); break;
    case int_IRQ: IRQ(); break;
  }
}
char line[500];
void disassembler(){
  struct Disassembler dis;
  dis.op_name = name_arr[cpu.opcode].name;
  dis.op1 = cread(cpu.pc+1);
  dis.op2 = cread(cpu.pc+2);
  dis.mode = inst_modes[cpu.opcode];
  dis.size = inst_size[cpu.opcode];
  dis.fetch_address = cpu.fetch_address;
  switch(dis.mode){
    case absolute:
      sprintf(line,"%x> %s $%x%x",cpu.pc,dis.op_name,dis.op2,dis.op1);
      break;
    case absolute_X:
      sprintf(line,"%x> %s $%x%x,X",cpu.pc,dis.op_name,dis.op2,dis.op1);
      break;
    case absolute_Y:
      sprintf(line,"%x> %s $%x%x,Y",cpu.pc,dis.op_name,dis.op2,dis.op1);
      break;
    case accumulator:
      sprintf(line,"%x> %s A",cpu.pc,dis.op_name);
      break;
    case immediate:
      sprintf(line,"%x> %s #$%x",cpu.pc,dis.op_name,dis.op1);
      break;
    case implied:
      sprintf(line,"%x> %s",cpu.pc,dis.op_name);
      break;
    case indexed_indirect:
      sprintf(line,"%x> %s (X,$%x)",cpu.pc,dis.op_name,dis.op1);
      break;
    case indirect:
      sprintf(line,"%x> %s ($%x%x)",cpu.pc,dis.op_name,dis.op2,dis.op1);
      break;
    case indirect_indexed:
      sprintf(line,"%x> %s ($%x,Y)",cpu.pc,dis.op_name,dis.op1);
      break;
  }
  printf("%s\n",line);
  fprintf(LOG,"%s\n",line);
}
uint cpu_step(){
  if(cpu.stall>0){ //Stall cpu (SYNC)
    cpu.stall--;
    cpu.cycles++;
  }
  uint cycles = cpu.cycles;
  u8 opcode = cread(cpu.pc);
  cpu.opcode = opcode;
  cpu.opcode_address = cpu.pc;

  uint mode = inst_modes[opcode];
  u16 address = 0;
  uint page_crossed = false;

  mode_addressing(mode,&address,&page_crossed);
  disassembler();
  check_interrupt();
  if(cpu.interrupt!=int_none){
    cpu.interrupt = int_none;
    return cpu.cycles - cycles;
  }
  cpu.interrupt = int_none;
  cpu.inst_count++;
  cpu.pc += inst_size[opcode];
  cpu.cycles += inst_cycles[opcode];
  if(page_crossed == 1) cpu.cycles += inst_Page_cycles[opcode];
  struct Curr_step step_data = {address,cpu.pc,mode};
  decode(opcode,step_data);

  return cpu.cycles - cycles;
}

int cpu_run(){
  init_files();
  mapper();
  RESET();
  while(cpu.cpuIsRunning){ //main execution loop
    cpu_step();
  }
}
