/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Magdy Mohamed
 ******************************************************************************/

#ifndef Port_H
#define Port_H


/* vendor ID number */
#define Port_VENDOR_ID    (1000U)

/* Port Module Id */
#define Port_MODULE_ID    (124U)

/* Port Instance Id */
#define Port_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)



#include "Std_Types.h"
/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif



/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif

/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_CFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_CFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif

/* Non AUTOSAR files */
#include "Common_Macros.h"

#include "Port_reg.h"

#include "tm4c123gh6pm_registers.h"

/*
 * Macros for Port Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* Service ID for init Port*/
#define PORT_INIT_SID                       (uint8)0x00

/* Service ID for Port Set Direction */
#define PORT_SET_PIN_DIRECTION_SID          (uint8) 0x01

/* Service ID for Refresh Port Direction */
#define PORT_REFRESH_PORT_Direction_SID     (uint8)0x02

/* Service ID for Port GetVersionInfo */
#define PORT_GET_VERSION_INFO_SID           (uint8)0x03

/* Service ID  for setting the pin mode of port */
#define PORT_SET_PIN_MODE_SID               (uint8)0x04


/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/
/* Invalid Port Pin ID requested  */
#define PORT_E_PARAM_PIN                (uint8)0x0A

/*  Port Pin not configured as changeable  */
#define PORT_E_DIRECTION_UNCHANGEABLE   (uint8)0x0B

/*  API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG             (uint8)0x0C

/*  API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_PARAM_INVALID_MODE       (uint8)0x0D

/*  API Port_SetPinMode service called when mode is unchangeable */
#define PORT_E_MODE_UNCHANGEABLE        (uint8)0x0E

/*  API service called without module initialization  */
#define PORT_E_UNINIT                   (uint8)0x0F

/*  APIs called with a Null Pointer  */
#define PORT_E_PARAM_POINTER            (uint8)0x10



/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* cover all available port pins */
typedef uint8 Port_PinType;

/* cover all available ports */
typedef uint8 Port_PortType ;

/* initial values for output pins*/
typedef uint8 Port_InitPinLevelType;

/* Possible directions of a port pin */
typedef enum 
{
    PORT_PIN_IN , 
    PORT_PIN_OUT
}Port_PinDirectionType;


/* Activation of internal pull-ups */
typedef enum 
{
    PORT_PULL_UP , 
    PORT_PULL_DOWN,
    PORT_RESISTOR_OFF
}Port_PinResistorType;


/*Number of Pin modes */
#define PORT_CONFIG_MODE_NUM                (8U)

/* As several port pin modes shall be configurable on one pin */
typedef uint8 Port_PinModeType;

#define PORT_PIN_MODE_DIO       ((Port_PinModeType)0)
#define PORT_PIN_MODE_ADC       ((Port_PinModeType)1)
#define PORT_PIN_MODE_PWM       ((Port_PinModeType)2)
#define PORT_PIN_MODE_USB       ((Port_PinModeType)3)
#define PORT_PIN_MODE_USART     ((Port_PinModeType)4)
#define PORT_PIN_MODE_CAN       ((Port_PinModeType)6)
#define PORT_PIN_MODE_I2C       ((Port_PinModeType)7)
#define PORT_PIN_MODE_SSI       ((Port_PinModeType)8)


#define PORT_MODE_DIO_MASK      ((uint32) 0x0000000F)
#define PORT_MODE_PWM_MASK      ((uint32) 0x00000005)
#define PORT_MODE_USB_MASK      ((uint32) 0x00000008)
#define PORT_MODE_USART_MASK    ((uint32) 0x00000001)
#define PORT_MODE_CAN_MASK      ((uint32) 0x00000008)
#define PORT_MODE_SSI_MASK      ((uint32) 0x00000002)
#define PORT_MODE_I2C_MASK      ((uint32) 0x00000003)


typedef struct 
{
  Port_PinType Pin_Num;
  Port_PortType Port_Num;
  Port_PinDirectionType PinDirection ;

#if (PORT_PIN_INITIAL_MODE == STD_ON)
  Port_PinModeType PinMode;
#endif
  Port_InitPinLevelType initial_value;
  Port_PinResistorType PinResistorType ;
  
}Port_PinConfig;


/*
* Type of the external data structure 
* containing the initialization data for this module
*/
typedef struct 
{
  Port_PinConfig Pins[PORT_CONFIGURED_PINS];
}Port_ConfigType;


/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* function for Port init  Initialization API */
void Port_Init(const Port_ConfigType *ConfigPtr);


/* function for Seting the port pin direction*/
#if(PORT_SET_PIN_DIRECTION_API == STD_ON)
  void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );
#endif

/* fuction Refreshes port direction*/
void Port_RefreshPortDirection( void );

/* Returns the version information of this module*/
#if (PORT_VERSION_INFO_API == STD_ON)
  void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );
#endif

/* Sets the port pin mode*/
#if(PORT_SET_PIN_MODE_API == STD_ON)
  void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );
#endif


/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by port and other modules */
extern const  Port_ConfigType Port_Config ;

#endif