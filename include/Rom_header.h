struct Rom_header{
  char string[4];
  char rom_prg_nos; //Number of 16384 byte program rom pages
  char rom_chr_nos; //Number of 8192 byte character rom pages
  char bitfield1; /* bit 0     1 for vertical mirroring, 0 for horizontal mirroring.
                     bit 1     1 for battery-backed RAM at $6000-$7FFF.
                     bit 2     1 for a 512-byte trainer at $7000-$71FF.
                     bit 3     1 for a four-screen VRAM layout.
                     bit 4-7   Four lower bits of ROM Mapper Type. */
  char bitfield2; /* bit 0     1 for VS-System cartridges.
                     bit 1-3   Reserved, must be zeroes!
                     bit 4-7   Four higher bits of ROM Mapper Type.*/
  char unused[8]; // must be 0
};
extern struct Rom_header *header;
