#include "extcode.h"
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * AboutADC
 */
void __cdecl AboutADC(int32_t DevOut);
/*!
 * InitADC
 */
int32_t __cdecl InitADC(uint32_t VRange, int32_t *DeviceID);
/*!
 * ScanADC
 */
void __cdecl ScanADC(uint32_t Handles, uint32_t ByteBuf, uint16_t ArrSize[], 
	int32_t lenArr, uint16_t OutArr[], int32_t lenOut, int32_t *lenOut2, 
	char ADCStat[], int32_t lenStat, int32_t *lenStatOut);
/*!
 * StartCap
 */
void __cdecl StartCap(int32_t DevIn, uint32_t *Handles, float Lv_65, 
	uint32_t SampRec, uint32_t *ByteLen, LVBoolean SaveDat, char SaveName[], 
	LVBoolean *ErrBool, char ADCStat[], int32_t len, int32_t *len2);

MgErr __cdecl LVDLLStatus(char *errStr, int errStrLen, void *module);

#ifdef __cplusplus
} // extern "C"
#endif

