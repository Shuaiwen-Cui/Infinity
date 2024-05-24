:
/*-----------------------------------------------------------------
 thrSGN: Signal LED to change 
*----------------------------------------------------------------*/
__NO_RETURN static void thrSGN(void *argument)
{
    (void)argument;
    uint32_t last;
    for (;;)
    {
        osDelay(500U); // Run delay for 500 ticks
        while (!GPIO_PinRead(BOARD_USER_BUTTON_GPIO,
                             BOARD_USER_BUTTON_GPIO_PIN))
        {
            osDelay(10); // Delay further while SW8 button is pressed
        }
        osThreadFlagsSet(tid_thrLED, 1U); // Set flag to thrLED
    }
}
: