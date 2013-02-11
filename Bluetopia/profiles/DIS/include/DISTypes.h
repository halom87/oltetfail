/*****< distypes.h >***********************************************************/
/*      Copyright 2011 - 2012 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  DISTypes - Stonestreet One Bluetooth Stack Device Information Service     */
/*             Types.                                                         */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   10/04/11  T. Cook        Initial creation.                               */
/******************************************************************************/
#ifndef __DISTYPESH__
#define __DISTYPESH__

#include "SS1BTGAT.h"     /* Bluetooth Stack GATT API Prototypes/Constants.   */
#include "BTPSKRNL.h"     /* BTPS Kernel Prototypes/Constants.                */

   /* The following MACRO is a utility MACRO that assigns the Device    */
   /* Information Service 16 bit UUID to the specified UUID_16_t        */
   /* variable.  This MACRO accepts one parameter which is a pointer to */
   /* a UUID_16_t variable that is to receive the DIS UUID Constant     */
   /* value.                                                            */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_DIS_SERVICE_UUID_16(_x)               ASSIGN_UUID_16(*((UUID_16_t *)(_x)), 0x0A, 0x18)
   
   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Service UUID in UUID16 form.  This     */
   /* MACRO only returns whether the UUID_16_t variable is equal to the */
   /* DIS Service UUID (MACRO returns boolean result) NOT less          */
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Service UUID.                               */
#define DIS_COMPARE_DIS_SERVICE_UUID_TO_UUID_16(_x)      COMPARE_UUID_16_TO_CONSTANT((_x), 0x0A, 0x18)

   /* The following defines the Device Information Service UUID that is */
   /* used when building the DIS Service Table.                         */
#define DIS_SERVICE_UUID_CONSTANT                        { 0x0A, 0x18 }

   /* The following MACRO is a utility MACRO that assigns the DIS       */
   /* Manufacturer Name Characteristic 16 bit UUID to the specified     */
   /* UUID_16_t variable.  This MACRO accepts one parameter which is the*/
   /* UUID_16_t variable that is to receive the DIS Manufacturer Name   */
   /* UUID Constant value.                                              */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_MANUFACTURER_NAME_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x29, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Manufacturer Name UUID in UUID16 form. */
   /* This MACRO only returns whether the UUID_16_t variable is equal to*/
   /* the Manufacturer Name UUID (MACRO returns boolean result) NOT less*/
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Manufacturer Name UUID.                     */
#define DIS_COMPARE_DIS_MANUFACTURER_NAME_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x29, 0x2A)

   /* The following defines the DIS Manufacturer Name Characteristic    */
   /* UUID that is used when building the DIS Service Table.            */
#define DIS_MANUFACTURER_NAME_CHARACTERISTIC_UUID_CONSTANT    { 0x29, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS Model */
   /* Number Characteristic 16 bit UUID to the specified UUID_16_t      */
   /* variable.  This MACRO accepts one parameter which is the UUID_16_t*/
   /* variable that is to receive the DIS Model Number UUID Constant    */
   /* value.                                                            */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_MODEL_NUMBER_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x24, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Model Number UUID in UUID16 form.  This*/
   /* MACRO only returns whether the UUID_16_t variable is equal to the */
   /* Model Number UUID (MACRO returns boolean result) NOT less         */
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Model Number UUID.                          */
#define DIS_COMPARE_DIS_MODEL_NUMBER_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x24, 0x2A)

   /* The following defines the DIS Model Number Characteristic UUID    */
   /* that is used when building the DIS Service Table.                 */
#define DIS_MODEL_NUMBER_CHARACTERISTIC_UUID_CONSTANT    { 0x24, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS Serial*/
   /* Number Characteristic 16 bit UUID to the specified UUID_16_t      */
   /* variable.  This MACRO accepts one parameter which is the UUID_16_t*/
   /* variable that is to receive the DIS Serial Number UUID Constant   */
   /* value.                                                            */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_SERIAL_NUMBER_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x25, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Serial Number UUID in UUID16 form.     */
   /* This MACRO only returns whether the UUID_16_t variable is equal to*/
   /* the Serial Number UUID (MACRO returns boolean result) NOT less    */
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Serial Number UUID.                         */
#define DIS_COMPARE_DIS_SERIAL_NUMBER_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x25, 0x2A)

   /* The following defines the DIS Serial Number Characteristic UUID   */
   /* that is used when building the DIS Service Table.                 */
#define DIS_SERIAL_NUMBER_CHARACTERISTIC_UUID_CONSTANT    { 0x25, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS       */
   /* Hardware Revision Characteristic 16 bit UUID to the specified     */
   /* UUID_16_t variable.  This MACRO accepts one parameter which is the*/
   /* UUID_16_t variable that is to receive the DIS Hardware Revision   */
   /* UUID Constant value.                                              */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_HARDWARE_REVISION_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x27, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Hardware Revision UUID in UUID16 form. */
   /* This MACRO only returns whether the UUID_16_t variable is equal to*/
   /* the Hardware Revision UUID (MACRO returns boolean result) NOT less*/
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Hardware Revision UUID.                     */
#define DIS_COMPARE_DIS_HARDWARE_REVISION_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x27, 0x2A)

   /* The following defines the DIS Hardware Revision Characteristic    */
   /* UUID that is used when building the DIS Service Table.            */
#define DIS_HARDWARE_REVISION_CHARACTERISTIC_UUID_CONSTANT    { 0x27, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS       */
   /* Firmware Revision Characteristic 16 bit UUID to the specified     */
   /* UUID_16_t variable.  This MACRO accepts one parameter which is the*/
   /* UUID_16_t variable that is to receive the DIS Firmware Revision   */
   /* UUID Constant value.                                              */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_FIRMWARE_REVISION_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x26, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Firmware Revision UUID in UUID16 form. */
   /* This MACRO only returns whether the UUID_16_t variable is equal to*/
   /* the Firmware Revision UUID (MACRO returns boolean result) NOT less*/
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Firmware Revision UUID.                     */
#define DIS_COMPARE_DIS_FIRMWARE_REVISION_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x26, 0x2A)

   /* The following defines the DIS Firmware Revision Characteristic    */
   /* UUID that is used when building the DIS Service Table.            */
#define DIS_FIRMWARE_REVISION_CHARACTERISTIC_UUID_CONSTANT    { 0x26, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS       */
   /* Software Revision Characteristic 16 bit UUID to the specified     */
   /* UUID_16_t variable.  This MACRO accepts one parameter which is the*/
   /* UUID_16_t variable that is to receive the DIS Software Revision   */
   /* UUID Constant value.                                              */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_SOFTWARE_REVISION_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x28, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS Software Revision UUID in UUID16 form. */
   /* This MACRO only returns whether the UUID_16_t variable is equal to*/
   /* the Software Revision UUID (MACRO returns boolean result) NOT less*/
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS Software Revision UUID.                     */
#define DIS_COMPARE_DIS_SOFTWARE_REVISION_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x28, 0x2A)

   /* The following defines the DIS Software Revision Characteristic    */
   /* UUID that is used when building the DIS Service Table.            */
#define DIS_SOFTWARE_REVISION_CHARACTERISTIC_UUID_CONSTANT    { 0x28, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS System*/
   /* ID Characteristic 16 bit UUID to the specified UUID_16_t variable.*/
   /* This MACRO accepts one parameter which is the UUID_16_t variable  */
   /* that is to receive the DIS System ID UUID Constant value.         */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_SYSTEM_ID_UUID_16(_x)                 ASSIGN_UUID_16((_x), 0x23, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS System ID UUID in UUID16 form.  This   */
   /* MACRO only returns whether the UUID_16_t variable is equal to the */
   /* System ID UUID (MACRO returns boolean result) NOT less            */
   /* than/greater than.  The first parameter is the UUID_16_t variable */
   /* to compare to the DIS System ID UUID.                             */
#define DIS_COMPARE_DIS_SYSTEM_ID_UUID_TO_UUID_16(_x)    COMPARE_UUID_16_TO_CONSTANT((_x), 0x23, 0x2A)

   /* The following defines the DIS System ID Characteristic UUID that  */
   /* is used when building the DIS Service Table.                      */
#define DIS_SYSTEM_ID_CHARACTERISTIC_UUID_CONSTANT       { 0x23, 0x2A }

   /* The following MACRO is a utility MACRO that assigns the DIS IEEE  */
   /* Certification Data Characteristic 16 bit UUID to the specified    */
   /* UUID_16_t variable.  This MACRO accepts one parameter which is the*/
   /* UUID_16_t variable that is to receive the DIS IEEE Certification  */
   /* Data UUID Constant value.                                         */
   /* * NOTE * The UUID will be assigned into the UUID_16_t variable in */
   /*          Little-Endian format.                                    */
#define DIS_ASSIGN_IEEE_CERTIFICATION_DATA_UUID_16(_x)         ASSIGN_UUID_16((_x), 0x2A, 0x2A)

   /* The following MACRO is a utility MACRO that exist to compare a    */
   /* UUID 16 to the defined DIS IEEE Certification Data UUID in UUID16 */
   /* form.  This MACRO only returns whether the UUID_16_t variable is  */
   /* equal to the IEEE Certification Data UUID (MACRO returns boolean  */
   /* result) NOT less than/greater than.  The first parameter is the   */
   /* UUID_16_t variable to compare to the DIS IEEE Certification Data  */
   /* UUID.                                                             */
#define DIS_COMPARE_DIS_IEEE_CERTIFICATION_DATA_UUID_TO_UUID_16(_x) COMPARE_UUID_16_TO_CONSTANT((_x), 0x2A, 0x2A)

   /* The following defines the DIS IEEE Certification Data             */
   /* Characteristic UUID that is used when building the DIS Service    */
   /* Table.                                                            */
#define DIS_IEEE_CERTIFICATION_DATA_CHARACTERISTIC_UUID_CONSTANT    { 0x2A, 0x2A }

#endif
