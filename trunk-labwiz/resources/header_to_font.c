#include "stdint.h"
#include "font_v2.h"
#include "stdio.h"
#include "string.h"

// This takes a header produced by GIMP and turns it into a 
// column based font for the LabWiz

int main()
{
    int x,i;
    uint8_t d,val;
    uint8_t data[1][479];
    int limit = width;

    memset(data,0,sizeof(data));

#if 1
    
    for(i=0;i<limit;i++)
    {
        val = 0;
        if(!(limit+1)%5) continue;
        for(x=0;x<8;x++)
        {
            d = header_data[i+(width*x)];
            val |= (d<<(7-x));
            //printf("x%02X,",d);
        }
        //printf("\n");
        data[0][i] = val;
    }
#endif
#if 1
    char c=' ';
    for(i=0;i<limit;i++)
    {
        if(((i+1)%5))
            printf("0x%02X,",data[0][i]);
        if(!((i+1)%5)) printf("// '%c'\n",(i/5)+' ');
    }
#endif
    return 0;
}
