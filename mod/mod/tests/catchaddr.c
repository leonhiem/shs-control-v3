#include <stdio.h>
#include <stdint.h>
main()
{
  int a;
  char data=0;
  char in=0x78;
  for(a=0;a<8;a++) {

      char t=in&(1<<(7-a));
      printf("bit=%d\n",t!=0);

      data = (data << 1);
      if(t!=0) {
        data |= 1;
      }

  }
  printf("in=0x%x data=0x%x\n",in,data);
}
