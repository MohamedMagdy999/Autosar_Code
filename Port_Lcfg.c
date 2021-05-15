/******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Lcfg.c
 *
 * Description: source file for TM4C123GH6PM Microcontroller - Port Driver
 *
 * Author: Mohamed Magdy Mohamed
 ******************************************************************************/

#include "port.h"


/*
 * Module Version 1.0.0
 */
#define  PORT_PBCFG_SW_MAJOR_VERSION              (1U)
#define  PORT_PBCFG_SW_MINOR_VERSION              (0U)
#define  PORT_PBCFG_SW_PATCH_VERSION              (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define  PORT_PBCFG_AR_RELEASE_MAJOR_VERSION     (4U)
#define  PORT_PBCFG_AR_RELEASE_MINOR_VERSION     (0U)
#define  PORT_PBCFG_AR_RELEASE_PATCH_VERSION     (3U)


/* 
* Check compatibility of Port.h AUTOSAR version with
* Port_PBcfg.c AUTOSAR version.
*/
#if ((PORT_PBCFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_PBCFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
	#error "The AR version of Port.h does not match the expected version"
#endif

/* 
* Check compatibility of Port.h SW version with
* Port_PBcfg.c SW version.
*/
#if ((PORT_PBCFG_SW_MAJOR_VERSION != PORT_SW_MAJOR_VERSION)\
 ||  (PORT_PBCFG_SW_MINOR_VERSION != PORT_SW_MINOR_VERSION)\
 ||  (PORT_PBCFG_SW_PATCH_VERSION != PORT_SW_PATCH_VERSION))
	#error "The AR version of Port.h does not match the expected version"
#endif


