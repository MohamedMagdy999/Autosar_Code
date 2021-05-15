/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Magdy Mohamed
 ******************************************************************************/

#ifndef PORT_CFG_H
#define PORT_CFG_H

/*
 * Module Version 1.0.0
 */
#define PORT_CFG_SW_MAJOR_VERSION           (1U)
#define PORT_CFG_SW_MINOR_VERSION           (0U)
#define PORT_CFG_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION   (3U)

/*******************************************************************************/

/* The number of Pins in mcu */
#define PORT_NUMBER_OF_PORT_PINS             (42)


/* Number of the configured Ports pins */
#define PORT_CONFIGURED_PINS                (2U)


/* Number of configured Ports */
#define PORT_CONFIG_PORT_NUM                (6U)


/*******************************************************************************
 *                             Pre-compile options
*******************************************************************************/

/* Pre-compile option for Version Info API */
#define PORT_VERSION_INFO_API               (STD_OFF)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT               (STD_ON)

/* Pre-compile option for setting the direction druing run time */
#define PORT_SET_PIN_DIRECTION_API          (STD_ON)

/* Pre-processor switch to enable / disable the use of the function Port_SetPinMode */
#define PORT_SET_PIN_MODE_API               (STD_ON)

/* Precompile and Post Build option for Port pin mode changeability */
#define PORT_PIN_MODE_CHANGEABLE            (STD_OFF)

/* Precompile and Post Build option for setting Pin mode
to be ADC, DIO, LIN, .. */
#define PORT_PIN_MODE                         (STD_ON)


/* Pre-compile and Post Build option for setting the Pin direction,
   according to its mode E.g. a pin used for an ADC must be configured
   to be an in port. */
#define PORT_PIN_DIRECTION                    (STD_ON)

/* Pre-compile and Post Build option for changing a Pin 
   direction during runtime */
#define PORT_PIN_DIRECTION_CHANGEABLE         (STD_ON)

/* Precompile and Post Build option for setting initial Pin mode 
   to be ADC, DIO, LIN, .. */
#define PORT_PIN_INITIAL_MODE                 (STD_ON)

/* Precompile and Post Build option for setting 
   Port Pin Level value from Port pin list. */
#define PORT_PIN_LEVEL_VALUE                  (STD_OFF)



/* Port Pin Level value from Port pin list */
#if (PORT_PIN_LEVEL_VALUE == STD_ON)
    #define PORT_PIN_LEVEL_HIGH                   (STD_HIGH)
    #define PORT_PIN_LEVEL_LOW                    (STD_LOW)
#endif /* (PORT_PIN_LEVEL_VALUE == STD_ON) */



/*******************************************************************************/

/* Pin Index in the array of structures in Port_PBcfg.c */
#define PortConf_LED1_PIN_ID_INDEX            (uint8)0x00
#define PortConf_SW1_PIN_ID_INDEX             (uint8)0x01

/* Port Configured Port ID's  */
#define PortConf_LED1_PORT_NUM                (Port_PortType)5 /* PORTF */
#define PortConf_SW1_PORT_NUM                 (Port_PortType)5 /* PORTF */

/* Port Configured Pin ID's */
#define PortConf_LED1_PIN_NUM                (Port_PinType)1 /* Pin 1 in PORTF */
#define PortConf_SW1_PIN_NUM                 (Port_PinType)4 /* Pin 4 in PORTF */

#define PortConf_LED1_PIN_DIRECTION          (PORT_PIN_OUT)
#define PortConf_SW1_PIN_DIRECTION           (PORT_PIN_IN)

#define PortConf_LED1_PIN_MODE               (PORT_PIN_MODE_DIO)
#define PortConf_SW1_PIN_MODE                (PORT_PIN_MODE_DIO)

#define PortConf_LED1_PIN_INIT_LEVEL        (STD_LOW)
#define PortConf_SW1_PIN_INIT_LEVEL         (STD_LOW)


#define PortConf_LED1_PIN_RESISTOR          (PORT_RESISTOR_OFF)
#define PortConf_SW1_PIN_RESISTOR           (PORT_PULL_UP)



#endif