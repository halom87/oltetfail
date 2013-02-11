/*****< disapi.h >*************************************************************/
/*      Copyright 2011 - 2012 Stonestreet One.                                */
/*      All Rights Reserved.                                                  */
/*                                                                            */
/*  DISAPI - Stonestreet One Bluetooth Device Information Service (GATT       */
/*          based) API Type Definitions, Constants, and Prototypes.           */
/*                                                                            */
/*  Author:  Tim Cook                                                         */
/*                                                                            */
/*** MODIFICATION HISTORY *****************************************************/
/*                                                                            */
/*   mm/dd/yy  F. Lastname    Description of Modification                     */
/*   --------  -----------    ------------------------------------------------*/
/*   10/04/11  T. Cook        Initial creation.                               */
/******************************************************************************/
#ifndef __DISAPIH__
#define __DISAPIH__

#include "SS1BTPS.h"        /* Bluetooth Stack API Prototypes/Constants.      */
#include "SS1BTGAT.h"       /* Bluetooth Stack GATT API Prototypes/Constants. */
#include "DISTypes.h"       /* Device Information Service Types/Constants.    */

   /* Error Return Codes.                                               */

   /* Error Codes that are smaller than these (less than -1000) are     */
   /* related to the Bluetooth Protocol Stack itself (see BTERRORS.H).  */
#define DIS_ERROR_INVALID_PARAMETER                      (-1000)
#define DIS_ERROR_INSUFFICIENT_RESOURCES                 (-1001)
#define DIS_ERROR_SERVICE_ALREADY_REGISTERED             (-1003)
#define DIS_ERROR_INVALID_INSTANCE_ID                    (-1004)
#define DIS_ERROR_MALFORMATTED_DATA                      (-1005)
#define DIS_ERROR_UNKNOWN_ERROR                          (-1006)

   /* The following controls the maximum length of a static string.     */
#define DIS_MAXIMUM_SUPPORTED_STRING                     (BTPS_CONFIGURATION_DIS_MAXIMUM_SUPPORTED_STRING_LENGTH)

   /* The following structure defines the format of the System ID       */
   /* characteristic value.  The OUI is the IEEE defined                */
   /* Organizationally Unique Identifier.  The Manufacturer Identifier  */
   /* is a manufacturer defined value.  Both fields are Little-Endian.  */
typedef __PACKED_STRUCT_BEGIN__ struct _tagDIS_System_ID_t
{
   NonAlignedByte_t Manufacturer_Identifier[5];
   NonAlignedByte_t Organizationally_Unique_Identifier[3];
} __PACKED_STRUCT_END__ DIS_System_ID_t;

#define DIS_SYSTEM_ID_DATA_SIZE                          (sizeof(DIS_System_ID_t))

   /* The following function is responsible for opening a DIS Server.   */
   /* The first parameter is the Bluetooth Stack ID on which to open the*/
   /* server.  The final parameter is a pointer to store the GATT       */
   /* Service ID of the registered DIS service.  This can be used to    */
   /* include the service registered by this call.  This function       */
   /* returns the positive, non-zero, Instance ID or a negative error   */
   /* code.                                                             */
   /* * NOTE * Only 1 DIS Server may be open at a time, per Bluetooth   */
   /*          Stack ID.                                                */
BTPSAPI_DECLARATION int BTPSAPI DIS_Initialize_Service(unsigned int BluetoothStackID, unsigned int *ServiceID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_DIS_Initialize_Service_t)(unsigned int BluetoothStackID, unsigned int *ServiceID);
#endif

   /* The following function is responsible for closing a previously DIS*/
   /* Server.  The first parameter is the Bluetooth Stack ID on which to*/
   /* close the server.  The second parameter is the InstanceID that was*/
   /* returned from a successfull call to DIS_Initialize_Service().     */
   /* This function returns a zero if successful or a negative return   */
   /* error code if an error occurs.                                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Cleanup_Service(unsigned int BluetoothStackID, unsigned int InstanceID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES
   typedef int (BTPSAPI *PFN_DIS_Cleanup_Service_t)(unsigned int BluetoothStackID, unsigned int InstanceID);
#endif

   /* The following function is responsible for setting the Manufacturer*/
   /* Name characteristic on the specified Device Information Service   */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is the Manufacturer Name to set as the current          */
   /* Manufacturer Name for the specified Device Information Service    */
   /* Instance.  The Manufacturer Name parameter must be a pointer to a */
   /* NULL terminated ASCII String of at most                           */
   /* DIS_MAXIMUM_SUPPORTED_STRING (not counting the trailing NULL      */
   /* terminator).  This function returns a zero if successful or a     */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Manufacturer_Name(unsigned int BluetoothStackID, unsigned int InstanceID, char *ManufacturerName);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Manufacturer_Name_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *ManufacturerName);
#endif
   
   /* The following function is responsible for querying the current    */
   /* Manufacturer Name characteristic value on the specified DIS       */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is a pointer to a structure to return the current       */
   /* Manufacturer Name for the specified DIS Service Instance.  The    */
   /* Manufacturer Name Length should be at least                       */
   /* (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Manufacturer_Name(unsigned int BluetoothStackID, unsigned int InstanceID, char *ManufacturerName);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Manufacturer_Name_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *ManufacturerName);
#endif
  
   /* The following function is responsible for setting the Model Number*/
   /* characteristic on the specified Device Information Service        */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is the Model Number to set as the current Model Number  */
   /* for the specified Device Information Service Instance.  The Model */
   /* Number parameter must be a pointer to a NULL terminated ASCII     */
   /* String of at most DIS_MAXIMUM_SUPPORTED_STRING (not counting the  */
   /* trailing NULL terminator).  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Model_Number(unsigned int BluetoothStackID, unsigned int InstanceID, char *ModelNumber);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Model_Number_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *ModelNumber);
#endif
   
   /* The following function is responsible for querying the current    */
   /* Model Number characteristic value on the specified DIS instance.  */
   /* The first parameter is the Bluetooth Stack ID of the Bluetooth    */
   /* Device.  The second parameter is the InstanceID returned from a   */
   /* successful call to DIS_Initialize_Server().  The final parameter  */
   /* is a pointer to a structure to return the current Model Number for*/
   /* the specified DIS Service Instance.  The Model Number Length      */
   /* should be at least (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the   */
   /* Maximum allowable string (plus a single character to hold the NULL*/
   /* terminator) This function returns a zero if successful or a       */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Model_Number(unsigned int BluetoothStackID, unsigned int InstanceID, char *ModelNumber);
                                                                                                
#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Model_Number_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *ModelNumber);
#endif

   /* The following function is responsible for setting the Serial      */
   /* Number characteristic on the specified Device Information Service */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is the Serial Number to set as the current Serial Number*/
   /* for the specified Device Information Service Instance.  The Serial*/
   /* Number parameter must be a pointer to a NULL terminated ASCII     */
   /* String of at most DIS_MAXIMUM_SUPPORTED_STRING (not counting the  */
   /* trailing NULL terminator).  This function returns a zero if       */
   /* successful or a negative return error code if an error occurs.    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Serial_Number(unsigned int BluetoothStackID, unsigned int InstanceID, char *SerialNumber);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Serial_Number_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *SerialNumber);
#endif

   /* The following function is responsible for querying the current    */
   /* Serial Number characteristic value on the specified DIS instance. */
   /* The first parameter is the Bluetooth Stack ID of the Bluetooth    */
   /* Device.  The second parameter is the InstanceID returned from a   */
   /* successful call to DIS_Initialize_Server().  The final parameter  */
   /* is a pointer to a structure to return the current Serial Number   */
   /* for the specified DIS Service Instance.  The Serial Number Length */
   /* should be at least (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the   */
   /* Maximum allowable string (plus a single character to hold the NULL*/
   /* terminator) This function returns a zero if successful or a       */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Serial_Number(unsigned int BluetoothStackID, unsigned int InstanceID, char *SerialNumber);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Serial_Number_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *SerialNumber);
#endif

   /* The following function is responsible for setting the Hardware    */
   /* Revision characteristic on the specified Device Information       */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to DIS_Initialize_Server().  The  */
   /* final parameter is the Hardware Revision to set as the current    */
   /* Hardware Revision for the specified Device Information Service    */
   /* Instance.  The Hardware Revision parameter must be a pointer to a */
   /* NULL terminated ASCII String of at most                           */
   /* DIS_MAXIMUM_SUPPORTED_STRING (not counting the trailing NULL      */
   /* terminator).  This function returns a zero if successful or a     */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Hardware_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *Hardware_Revision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Hardware_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *Hardware_Revision);
#endif

   /* The following function is responsible for querying the current    */
   /* Hardware Revision characteristic value on the specified DIS       */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is a pointer to a structure to return the current       */
   /* Hardware Revision for the specified DIS Service Instance.  The    */
   /* Hardware Revision Length should be at least                       */
   /* (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Hardware_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *Hardware_Revision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Hardware_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *Hardware_Revision);
#endif

   /* The following function is responsible for setting the Firmware    */
   /* Revision characteristic on the specified Device Information       */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to DIS_Initialize_Server().  The  */
   /* final parameter is the Firmware Revision to set as the current    */
   /* Firmware Revision for the specified Device Information Service    */
   /* Instance.  The Firmware Revision parameter must be a pointer to a */
   /* NULL terminated ASCII String of at most                           */
   /* DIS_MAXIMUM_SUPPORTED_STRING (not counting the trailing NULL      */
   /* terminator).  This function returns a zero if successful or a     */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Firmware_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *FirmwareRevision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Firmware_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *FirmwareRevision);
#endif

   /* The following function is responsible for querying the current    */
   /* Firmware Revision characteristic value on the specified DIS       */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is a pointer to a structure to return the current       */
   /* Firmware Revision for the specified DIS Service Instance.  The    */
   /* Firmware Revision Length should be at least                       */
   /* (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Firmware_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *FirmwareRevision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Firmware_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *FirmwareRevision);
#endif

   /* The following function is responsible for setting the Software    */
   /* Revision characteristic on the specified Device Information       */
   /* Service instance.  The first parameter is the Bluetooth Stack ID  */
   /* of the Bluetooth Device.  The second parameter is the InstanceID  */
   /* returned from a successful call to DIS_Initialize_Server().  The  */
   /* final parameter is the Software Revision to set as the current    */
   /* Software Revision for the specified Device Information Service    */
   /* Instance.  The Software Revision parameter must be a pointer to a */
   /* NULL terminated ASCII String of at most                           */
   /* DIS_MAXIMUM_SUPPORTED_STRING (not counting the trailing NULL      */
   /* terminator).  This function returns a zero if successful or a     */
   /* negative return error code if an error occurs.                    */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_Software_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *SoftwareRevision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_Software_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *SoftwareRevision);
#endif

   /* The following function is responsible for querying the current    */
   /* Software Revision characteristic value on the specified DIS       */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is a pointer to a structure to return the current       */
   /* Software Revision for the specified DIS Service Instance.  The    */
   /* Software Revision Length should be at least                       */
   /* (DIS_MAXIMUM_SUPPORTED_STRING+1) to hold the Maximum allowable    */
   /* string (plus a single character to hold the NULL terminator) This */
   /* function returns a zero if successful or a negative return error  */
   /* code if an error occurs.                                          */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_Software_Revision(unsigned int BluetoothStackID, unsigned int InstanceID, char *SoftwareRevision);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_Software_Revision_t)(unsigned int BluetoothStackID, unsigned int InstanceID, char *SoftwareRevision);
#endif

   /* The following function is responsible for setting the System ID   */
   /* characteristic on the specified Device Information Service        */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is the System ID to set as the current System ID for the*/
   /* specified Device Information Service Instance.  This function     */
   /* returns a zero if successful or a negative return error code if an*/
   /* error occurs.                                                     */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_System_ID(unsigned int BluetoothStackID, unsigned int InstanceID, DIS_System_ID_t *SystemID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_System_ID_t)(unsigned int BluetoothStackID, unsigned int InstanceID, DIS_System_ID_t *SystemID);
#endif

   /* The following function is responsible for querying the current    */
   /* System ID characteristic value on the specified DIS instance.  The*/
   /* first parameter is the Bluetooth Stack ID of the Bluetooth Device.*/
   /* The second parameter is the InstanceID returned from a successful */
   /* call to DIS_Initialize_Server().  The final parameter is a pointer*/
   /* to a structure to return the current System ID for the specified  */
   /* DIS Service Instance.  This function returns a zero if successful */
   /* or a negative return error code if an error occurs.               */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_System_ID(unsigned int BluetoothStackID, unsigned int InstanceID, DIS_System_ID_t *SystemID);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_System_ID_t)(unsigned int BluetoothStackID, unsigned int InstanceID, DIS_System_ID_t *SystemID);
#endif

   /* The following function is responsible for setting the IEEE        */
   /* Certification Data characteristic on the specified Device         */
   /* Information Service instance.  The first parameter is the         */
   /* Bluetooth Stack ID of the Bluetooth Device.  The second parameter */
   /* is the InstanceID returned from a successful call to              */
   /* DIS_Initialize_Server().  The third parameter is the length of the*/
   /* IEEE Certification Data.  The final parameter is the IEEE         */
   /* Certification Data to set as the current IEEE Certification Data  */
   /* for the specified Device Information Service Instance.  The IEEE  */
   /* Certification Data parameter must be a pointer to a array of at   */
   /* most DIS_MAXIMUM_SUPPORTED_STRING characters.  This function      */
   /* returns a zero if successful or a negative return error code if an*/
   /* error occurs.                                                     */
BTPSAPI_DECLARATION int BTPSAPI DIS_Set_IEEE_Certification_Data(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int Length, Byte_t *IEEE_Certification_Data);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Set_IEEE_Certification_Data_t)(unsigned int BluetoothStackID, unsigned int InstanceID, unsigned int Length, Byte_t *IEEE_Certification_Data);
#endif

   /* The following function is responsible for querying the current    */
   /* IEEE Certification Data characteristic value on the specified DIS */
   /* instance.  The first parameter is the Bluetooth Stack ID of the   */
   /* Bluetooth Device.  The second parameter is the InstanceID returned*/
   /* from a successful call to DIS_Initialize_Server().  The final     */
   /* parameter is a pointer to a structure to return the current IEEE  */
   /* Certification Data for the specified DIS Service Instance.  The   */
   /* IEEE Certification Data Length should be at least                 */
   /* (DIS_MAXIMUM_SUPPORTED_STRING) to hold the Maximum allowable IEEE */
   /* Certification Data.  This function the length of the IEEE         */
   /* Certification Data if successful or a negative return error code  */
   /* if an error occurs.                                               */
BTPSAPI_DECLARATION int BTPSAPI DIS_Query_IEEE_Certification_Data(unsigned int BluetoothStackID, unsigned int InstanceID, Byte_t *IEEE_Certification_Data);

#ifdef INCLUDE_BLUETOOTH_API_PROTOTYPES                                                         
   typedef int (BTPSAPI *PFN_DIS_Query_IEEE_Certification_Data_t)(unsigned int BluetoothStackID, unsigned int InstanceID, Byte_t *IEEE_Certification_Data);
#endif


#endif
