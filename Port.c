/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description:  Source file for TM4C123GH6PM Microcontroller - Port Driver 
 *
 * Author: Mohamed Magdy Mohamed
 ******************************************************************************/
#include "Port.h"
#include "Det.h"
#include "SchM_port.h"

#if (Port_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Port Modules */
#if ((DET_AR_MAJOR_VERSION != Port_AR_RELEASE_MAJOR_VERSION) || \
     (DET_AR_MINOR_VERSION != Port_AR_RELEASE_MINOR_VERSION) || \
     (DET_AR_PATCH_VERSION != Port_AR_RELEASE_PATCH_VERSION))
#error "The AR version of Det.h does not match the expected version"
#endif

#endif

/***************************************************************/
/*           Private Functions Declarations                    */
/****************************************************************/
boolean STATIC Port_InitDevErrorCheck(uint8 Service_Id);

STATIC volatile uint32 *Port_GetPortReg(uint8 pin_index);

/*********************************************************/
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;

/* This point to the pin config */
STATIC const Port_PinConfig *Port_Pins = NULL_PTR;

/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/

void Port_Init(const Port_ConfigType *ConfigPtr)
{
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON)
    if (ConfigPtr == NULL_PTR)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_INIT_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* nothing to do */
    }
#endif

    if (error == FALSE)
    {
        /* changing the state of module */
        Port_Status = PORT_INITIALIZED;

        /* variables */
        volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
        volatile uint32 delay = 0;
        uint8 pin_index = 0;

        /* Assign the pin config to pointer */
        Port_Pins = (ConfigPtr->Pins);

        for (pin_index = 0; pin_index < PORT_CONFIGURED_PINS; pin_index++)
        {
            /* Get the address of Port regester*/
            PortGpio_Ptr = (Port_GetPortReg(pin_index));

            /* Enable clock for PORT and allow time for clock to start*/
            SYSCTL_REGCGC2_REG |= (1 << Port_Pins[pin_index].Port_Num);
            delay = SYSCTL_REGCGC2_REG;

            /* check if the pin is special pins or need unlock */
            if (((Port_Pins[pin_index].Port_Num == 3) && (Port_Pins[pin_index].Pin_Num == 7)) || ((Port_Pins[pin_index].Port_Num == 5) && (Port_Pins[pin_index].Pin_Num == 0))) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                             /* Unlock the GPIOCR register */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            else if ((Port_Pins[pin_index].Port_Num == 2) && (Port_Pins[pin_index].Pin_Num <= 3)) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
                return;
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }

            if (Port_Pins[pin_index].PinMode == PORT_PIN_MODE_ADC)
            {
                /* Set the corresponding bit in the GPIOAMSEL register to Enable analog functionality on this pin */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[pin_index].Pin_Num);
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */
            }
            else
            {
                /* Clear the corresponding bit in the GPIOAMSEL register to disable analog functionality on this pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_Pins[pin_index].Pin_Num);

                switch (Port_Pins[pin_index].PinMode)
                {
                case PORT_PIN_MODE_DIO:

                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_MODE_DIO_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* Clear the PMCx bits for this pin */
                    break;

                case PORT_PIN_MODE_PWM:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_PWM_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;
                case PORT_PIN_MODE_USB:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_USB_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;

                case PORT_PIN_MODE_USART:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_USART_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;

                case PORT_PIN_MODE_CAN:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_CAN_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;

                case PORT_PIN_MODE_I2C:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_I2C_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;

                case PORT_PIN_MODE_SSI:

                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

                    *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_SSI_MASK << (Port_Pins[pin_index].Pin_Num * 4)); /* choosing the pin mode */
                    break;
                }
            }

            if (Port_Pins[pin_index].PinDirection == PORT_PIN_OUT)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                if (Port_Pins[pin_index].initial_value == STD_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                }
            }
            else if (Port_Pins[pin_index].PinDirection == PORT_PIN_IN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                if (Port_Pins[pin_index].PinResistorType == PORT_PULL_UP)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                }
                else if (Port_Pins[pin_index].PinResistorType == PORT_PULL_DOWN)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[pin_index].Pin_Num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                }
            }
            else
            {
                /* Do Nothing */
            }

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIODEN register to enable digital functionality on this pin */
        }
    }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number
*                  Direction - Port Pin Direction
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction.
************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction)
{
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON) /* Report Errors */
    error = Port_InitDevErrorCheck(PORT_SET_PIN_DIRECTION_SID);

    if (PORT_CONFIGURED_PINS <= Pin)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    if (FALSE == error)
    {
        volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
                                                  /* choosing the right port */
        PortGpio_Ptr = Port_GetPortReg(Pin);

        /* set the direction */
        if (Direction == PORT_PIN_OUT)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

            if (Port_Pins[Pin].initial_value == STD_HIGH)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
            }
        }
        else if (Direction == PORT_PIN_IN)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

            if (Port_Pins[Pin].PinResistorType == PORT_PULL_UP)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
            }
            else if (Port_Pins[Pin].PinResistorType == PORT_PULL_DOWN)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
            }
            else
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[Pin].Pin_Num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
            }
        }
        else
        {
            /* Do Nothing */
        }

    } /* (FALSE == error) */
    else
    {
        /* No Action Required */
    }
}

#endif
/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
void Port_RefreshPortDirection(void)
{
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON) /* Report Errors */
    error = Port_InitDevErrorCheck(PORT_REFRESH_PORT_Direction_SID);
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */

    if (error == FALSE)
    {
        Port_PinType pin_index = 0;
        volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

        for (pin_index = 0; pin_index < PORT_CONFIGURED_PINS; pin_index++)
        {
            /* choosing the write port */
            PortGpio_Ptr = Port_GetPortReg(pin_index);

            /* set the direction of the pin  */
            if (Port_Pins[pin_index].PinDirection == PORT_PIN_OUT)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIODIR register to configure it as output pin */

                if (Port_Pins[pin_index].initial_value == STD_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIODATA register to provide initial value 1 */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIODATA register to provide initial value 0 */
                }
            }
            else if (Port_Pins[pin_index].PinDirection == PORT_PIN_IN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */

                if (Port_Pins[pin_index].PinResistorType == PORT_PULL_UP)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIOPUR register to enable the internal pull up pin */
                }
                else if (Port_Pins[pin_index].PinResistorType == PORT_PULL_DOWN)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Set the corresponding bit in the GPIOPDR register to enable the internal pull down pin */
                }
                else
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_Pins[pin_index].Pin_Num);   /* Clear the corresponding bit in the GPIOPUR register to disable the internal pull up pin */
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_Pins[pin_index].Pin_Num); /* Clear the corresponding bit in the GPIOPDR register to disable the internal pull down pin */
                }
            }
            else
            {
                /* Do Nothing */
            }
        }
    }
    else
    {
        /* Do Nothing */
    }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module.
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if input pointer is not Null pointer */
    if (NULL_PTR == versioninfo)
    {
        /* Report to DET  */
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */
    {
        /* Copy the vendor Id */
        versioninfo->vendorID = (uint16)Port_VENDOR_ID;
        /* Copy the module Id */
        versioninfo->moduleID = (uint16)Port_MODULE_ID;
        /* Copy Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* Copy Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* Copy Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number
*                  Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)

void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
    boolean error = FALSE;

#if (PORT_DEV_ERROR_DETECT == STD_ON) /* Report Errors */
    error = Port_InitDevErrorCheck(PORT_SET_PIN_MODE_SID);

    if (PORT_CONFIGURED_PINS <= Pin)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else
    {
        /* No Action Required */
    }
    if (PORT_CONFIG_MODE_NUM <= Mode)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }

    /* Return immediately if the pin mode is not changeable */
    if (PORT_PIN_MODE_CHANGEABLE == STD_OFF)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
        return;
    }
#endif /* (PORT_DEV_ERROR_DETECT == STD_ON) */

    if (FALSE == error)
    {
        volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */

        PortGpio_Ptr = Port_GetPortReg(Pin);
        switch (Mode)
        {
        case PORT_PIN_MODE_DIO:

            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Disable Alternative function for this pin by clear the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(PORT_MODE_DIO_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* Clear the PMCx bits for this pin */
            break;

        case PORT_PIN_MODE_PWM:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_PWM_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;
        case PORT_PIN_MODE_USB:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_USB_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;

        case PORT_PIN_MODE_USART:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_USART_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;

        case PORT_PIN_MODE_CAN:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_CAN_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;

        case PORT_PIN_MODE_I2C:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_I2C_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;

        case PORT_PIN_MODE_SSI:

            SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_Pins[Pin].Pin_Num); /* Enable Alternative function for this pin by SET the corresponding bit in GPIOAFSEL register */

            *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (PORT_MODE_SSI_MASK << (Port_Pins[Pin].Pin_Num * 4)); /* choosing the pin mode */
            break;
        }
    }
}

#endif

/************************************************************************************
 *                                Private function 
 * *********************************************************************************/

/************************************************************************************
* Service Name: Port_InitDevErrorCheck
* Parameters (in): Service_Id - just take the fuction id that used this function
* Parameters (inout): None
* Parameters (out): None
* Return value: boolean about the error
* Description: this function used to check the state of the Port  
************************************************************************************/
boolean STATIC Port_InitDevErrorCheck(uint8 Service_Id)
{
    if (PORT_NOT_INITIALIZED == Port_Status)
    {
        Det_ReportError(Port_MODULE_ID, Port_INSTANCE_ID,
                        Service_Id, PORT_E_UNINIT);
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/************************************************************************************
* Service Name: Port_GetPortReg
* Parameters (in): pin_index - just take index of the pin 
* Parameters (inout): None
* Parameters (out): None
* Return value: pointer to the addres of the pin
* Description: private function return the address of Port regester  
************************************************************************************/
STATIC volatile uint32 *Port_GetPortReg(uint8 pin_index)
{
    volatile uint32 *PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    switch (Port_Pins[pin_index].Port_Num)
    {
    case 0:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
        break;
    case 1:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
        break;
    case 2:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
        break;
    case 3:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
        break;
    case 4:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
        break;
    case 5:
        PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
        break;
    }
    return PortGpio_Ptr;
}