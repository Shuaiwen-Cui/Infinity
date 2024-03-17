# CH02 - Enlight LED

## Enlight LED

### Project File Structure

```bash
-- Target 1
    |-- Source Group 1
        |-- main.c
```

### Project Code

```c
#include <REGX52.H>

void main()
{
    P2 = 0xFE; // 1111 1110
    while(1)
    {
    
    }
}
```

## LED Blinking

### Project File Structure

```bash
-- Target 1
    |-- Source Group 1
        |-- main.c
```

### Project Code

```c
#include <REGX52.H>
#include <INTRINS.H>

void Delay500ms()		//@11.0592MHz
{
	unsigned char i, j, k;

	_nop_();
	i = 4;
	j = 129;
	k = 119;
	do
	{
		do
		{
			while (--k);
		} while (--j);
	} while (--i);
}

int main()
{
	while(1)
	{
		P2 = 0xFE; // 1111 1110 only the last one is on
		Delay500ms();
	    P2 = 0xFF; // 1111 1111 all off
		Delay500ms();
	}
}
```

## LED Running Light

### Project File Structure

```bash
-- Target 1
    |-- Source Group 1
        |-- main.c
```

```c
#include "reg52.h"
#include "intrins.h"

typedef unsigned int u16;	
typedef unsigned char u8;

#define LED_PORT	P2	

void delay_10us(u16 ten_us)
{
	while(ten_us--);	
}

void main()
{	
   	u8 i=0;

	LED_PORT=~0x01;
	delay_10us(50000);
	while(1)
	{
		for(i=0;i<8;i++)
		{
			LED_PORT=~(0x01<<i);	
			delay_10us(50000);
		}
		
		//
//		for(i=0;i<7;i++)	 //
//		{									  
//			LED_PORT=_crol_(LED_PORT,1);
//			delay_10us(50000); 	
//		}
//		for(i=0;i<7;i++)	//
//		{
//			LED_PORT=_cror_(LED_PORT,1);
//			delay_10us(50000);	
//		}	
	}		
}

```