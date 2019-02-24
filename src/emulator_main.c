#include<types.h>
extern void init_files();
extern void cpu_run();
int main(){
  init_files();
  cpu_run();
  return 0;
}
