# CH03 BUTTON CONTROL

## 3-1 Control LED On/Off

```c
#include <REGX52.H>

void main()
{
    P2_0 = 0;
	while(1)
	{
		if(P3_1 == 0)
        {
            P2_0 = 0;
        }
        else
        {
            P2_0 = 1;
        }
	}

}
```

## 3-2 LED Status

```c
#include <REGX52.H>

void Delay(unsigned int xms)		//@11.0592MHz
{
	unsigned char i, j;
    while(xms)
    {
        //_nop_();
        i = 2;
        j = 199;
        do
        {
            while (--j);
        } while (--i);
        xms--;
    }

}

int main()
{
    while(1)
    {
        if(P3_1==0)
        {   // to remove jitter
            Delay(20);
            while(P3_1==0);
            Delay(20);
            P2_0 = ~P2_0;
        }
    }
}
```

## 3-3 Binary LED Control

```c
#include <REGX52.H>

void Delay(unsigned int xms)		//@11.0592MHz
{
	unsigned char i, j;
    while(xms--)
    {
        //_nop_();
        i = 2;
        j = 199;
        do
        {
            while (--j);
        } while (--i);
    }

}

int main()
{
    unsigned char LEDNum=0;
    while(1)
    {
        if(P3_1==0)
        {
            Delay(20);
            while(P3_1==0);
            Delay(20);
            
            LEDNum++;
            P2 = ~LEDNum;
        }
    }
}
```

## 3-4 Bit Control LED

```c
#include <REGX52.H>

unsigned char LEDNum=0;

void Delay(unsigned int xms)		//@11.0592MHz
{
	unsigned char i, j;
    while(xms--)
    {
        //_nop_();
        i = 2;
        j = 199;
        do
        {
            while (--j);
        } while (--i);
    }

}

int main()
{
    P2 = ~0x01;
    while(1)
    {
        if(P3_1==0) // one direction
        {
            Delay(20);
            while(P3_1==0);
            Delay(20);
            
            LEDNum++;
            if(LEDNum>=8)
                LEDNum=0;
            P2 = ~(0x01<<LEDNum);
        }
        if(P3_0==0) // the other direction
        {
            Delay(20);
            while(P3_0==0);
            Delay(20);
            
            if(LEDNum == 0)
                LEDNum = 7;
            else
                LEDNum--;
            
            P2 = ~(0x01<<LEDNum);
        }
    }
}
```