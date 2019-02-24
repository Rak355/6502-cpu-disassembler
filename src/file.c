#include<types.h>
#include<mem.h>
#include<Rom_header.h>
#include<file.h>
u8 main_mem[];
u8 ppu_mem[];
struct Rom_header *header;
FILE *ROM;
FILE *LOG;

u8 *rom_buffer;
void init_files(){
  ROM = fopen("./ROM/nestest.nes","rb"); //default file
  LOG = fopen("log.txt","a");
  if(ROM == NULL){
    printf("Could not open ROM file");
    exit(0);
  }
  if(ROM == NULL){
    printf("Could not open LOG file");
    exit(0);
  }
  rom_buffer = (u8*)malloc(sizeof(u8)*5000000);

  //initializing rom_buffer with ROM data
  int c,offset=0;
  while(true){
    c = fgetc(ROM);
    if(feof(ROM)){
       break;
    }
    *(rom_buffer + offset++)=(int)c;
  }
}

void print_meta(){
  header = (struct Rom_header*)rom_buffer;
  printf("string:%c\n",header->string[0]);
  printf("string:%c\n",header->string[1]);
  printf("string:%c\n",header->string[2]);
  printf("prg_count:%x\n",header->rom_prg_nos);
  printf("chr_count:%x\n",header->rom_chr_nos);
}

//Implementing the mapper here for now.
void mapper(){ //MAPPER 0
    print_meta();
  //mapping prg code
  unsigned int i,j;
  for(i=16,j=0x8000;j<=0xBFFF;i++,j++){
    main_mem[j] = rom_buffer[i];
  }

  if(header->rom_prg_nos == 1){
    for(i=16,j=0xC000;j<=0xFFFF;i++,j++){
      main_mem[j] = rom_buffer[i];
    }
    //mapping chr code
    for(i=0x0000,j=16+1*16384;i<=0x1FFF;i++,j++){
      ppu_mem[i] = rom_buffer[j];
    }
  }
  else if(header->rom_prg_nos == 2){
    for(i=16+1*16384,j=0xC000;j<=0xFFFF;i++,j++){
      main_mem[j] = rom_buffer[i];
    }
    //mapping chr code
    for(i=0x0000,j=16+2*16384;i<=0x1FFF;i++,j++){
      ppu_mem[i] = rom_buffer[j];
    }
  }
  else printf("Muahaha, mapper not implemented!!"); // :(
}
