#include "user_mb_app.h"

#include "mb_app_data.h"

#if (MB_SLAVE_ASCII_ENABLED > 0 || MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_TCP_ENABLED > 0)

/*------------------------Slave mode use these variables----------------------*/

//Slave mode:DiscreteInputs variables
#if S_DISCRETE_INPUT_NDISCRETES > 0
USHORT   usSDiscInStart                               = S_DISCRETE_INPUT_START;
#if S_DISCRETE_INPUT_NDISCRETES%8
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8+1];
#else
UCHAR    ucSDiscInBuf[S_DISCRETE_INPUT_NDISCRETES/8]  ;
#endif
#endif

//Slave mode:Coils variables
#if S_COIL_NCOILS > 0
USHORT   usSCoilStart                                 = S_COIL_START;
#if S_COIL_NCOILS%8
extern UCHAR    ucSCoilBuf[]; //UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
extern UCHAR    ucSCoilBuf[]; //UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif
#endif

//Slave mode:InputRegister variables
#if S_REG_INPUT_NREGS > 0
USHORT   usSRegInStart                                = S_REG_INPUT_START;
extern SHORT usSRegInBuf[]; //USHORT   usSRegInBuf[S_REG_INPUT_NREGS]               ;
#endif

//Slave mode:HoldingRegister variables
#if S_REG_HOLDING_NREGS > 0
USHORT   usSRegHoldStart                              = S_REG_HOLDING_START;
extern SHORT usSRegHoldBuf[] ;//	USHORT   usSRegHoldBuf[S_REG_HOLDING_NREGS]           ;
UCHAR	ucSRegHoldChangedMask[S_REG_HOLDING_NREGS];
#endif
/*------------------------Slave user code----------------------*/

void (*eMBRegHoldingWriteCB)(USHORT RegType, USHORT iRegIndex, USHORT usNRegs);

/*------------------------Slave registers callback function----------------------*/

/**
 * Modbus slave input register callback function.
 *
 * @param pucRegBuffer input register buffer
 * @param usAddress input register address
 * @param usNRegs input register number
 *
 * @return result
 */
eMBErrorCode eMBRegInputCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
#if S_REG_INPUT_NREGS > 0
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    SHORT *        pusRegInputBuf;
    USHORT          REG_INPUT_START;
    USHORT          REG_INPUT_NREGS;
    USHORT          usRegInStart;

    pusRegInputBuf = usSRegInBuf;
    REG_INPUT_START = S_REG_INPUT_START;
    REG_INPUT_NREGS = S_REG_INPUT_NREGS;
    usRegInStart = usSRegInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = usAddress - usRegInStart;
        while (usNRegs > 0)
        {
        	/*if(!app_mb_inreg_exists(usNRegs))
        		continue;
			*/

        	MBRegsTableNote_t *reg = app_mb_inreg_note_find(iRegIndex);

			if(reg == NULL) {
				eStatus = MB_ENOREG;
				break;
			}

			int16_t value = *(reg->pMBRegValue);

            *pucRegBuffer++ = (UCHAR) (value >> 8);
            *pucRegBuffer++ = (UCHAR) (value & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
#else
	return MB_ENOREG;
#endif
}

/**
 * Modbus slave holding register callback function.
 *
 * @param pucRegBuffer holding register buffer
 * @param usAddress holding register address
 * @param usNRegs holding register number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegHoldingCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
#if S_REG_HOLDING_NREGS > 0
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex;
    SHORT *        pusRegHoldingBuf;
    USHORT          REG_HOLDING_START;
    USHORT          REG_HOLDING_NREGS;
    USHORT          usRegHoldStart;

    pusRegHoldingBuf = usSRegHoldBuf;
    REG_HOLDING_START = S_REG_HOLDING_START;
    REG_HOLDING_NREGS = S_REG_HOLDING_NREGS;
    usRegHoldStart = usSRegHoldStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = usAddress - usRegHoldStart;
        switch (eMode)
        {
        /* read current register values from the protocol stack. */
        case MB_REG_READ:
            while (usNRegs > 0)
            {

                /**pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (UCHAR) (pusRegHoldingBuf[iRegIndex] & 0xFF); */

            	MBRegsTableNote_t *reg = app_mb_note_find(iRegIndex);

            	if(reg == NULL) {
            		eStatus = MB_ENOREG;
            		break;
            	}

            	int16_t value = *(reg->pMBRegValue);
            	*pucRegBuffer++ = (UCHAR) (value >> 8);
            	*pucRegBuffer++ = (UCHAR) (value & 0xFF);

                iRegIndex++;
                usNRegs--;
            }
            break;

        /* write current register values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (usNRegs > 0)
            {
                //pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                //pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;

            	MBRegsTableNote_t *reg = app_mb_note_find(iRegIndex);
            	if(reg == NULL) {
					eStatus = MB_ENOREG;
					break;
				}

            	int16_t val = *pucRegBuffer++ << 8;
            	val |= *pucRegBuffer++;

				*(reg->pMBRegValue) = val;

                /* add request to queue */
                if(eMBRegHoldingWriteCB != 0)
                	eMBRegHoldingWriteCB(MB_RegType_HoldReg, iRegIndex, val);

                iRegIndex++;
                usNRegs--;
            }

            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
#else
	return MB_ENOREG;
#endif
}

/**
 * Modbus slave coils callback function.
 *
 * @param pucRegBuffer coils buffer
 * @param usAddress coils address
 * @param usNCoils coils number
 * @param eMode read or write
 *
 * @return result
 */
eMBErrorCode eMBRegCoilsCB(UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
#if S_COIL_NCOILS > 0
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucCoilBuf;
    USHORT          COIL_START;
    USHORT          COIL_NCOILS;
    USHORT          usCoilStart;
    iNReg =  usNCoils;// / 8 + 1;

    pucCoilBuf = ucSCoilBuf;
    COIL_START = S_COIL_START;
    COIL_NCOILS = S_COIL_NCOILS;
    usCoilStart = usSCoilStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= COIL_START ) && ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - usCoilStart);// / 8;
        //iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (usNCoils > 0)
            {
                //*pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8);
                //iNReg--;
            	MBRegsTableNote_t *reg = app_mb_coil_note_find(iRegIndex);

				if(reg == NULL) {
					eStatus = MB_ENOREG;
					break;
				}

				int16_t value = *(reg->pMBRegValue);
				*pucRegBuffer++ = (UCHAR)value;

				iRegIndex++;
				usNCoils--;
            }
            //pucRegBuffer--;
            /* last coils */
            //usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            //*pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            //*pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
        	while (usNCoils > 0)
			{
				//pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
				//pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;

				MBRegsTableNote_t *reg = app_mb_coil_note_find(iRegIndex);
				if(reg == NULL) {
					eStatus = MB_ENOREG;
					break;
				}

				int16_t val = *pucRegBuffer++;

				*(reg->pMBRegValue) = val;

				/* add request to queue */
				if(eMBRegHoldingWriteCB != 0)
					eMBRegHoldingWriteCB(MB_RegType_Coil, iRegIndex, val);

				iRegIndex++;
				usNCoils--;
			}

			break;
//            while (iNReg > 1)
//            {
//                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
//                        *pucRegBuffer++);
//                iNReg--;
//            }
//            /* last coils */
//            usNCoils = usNCoils % 8;
//            /* xMBUtilSetBits has bug when ucNBits is zero */
//            if (usNCoils != 0)
//            {
//                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
//                        *pucRegBuffer++);
//            }
//            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
#else
	return MB_ENOREG;
#endif
}

/**
 * Modbus slave discrete callback function.
 *
 * @param pucRegBuffer discrete buffer
 * @param usAddress discrete address
 * @param usNDiscrete discrete number
 *
 * @return result
 */
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
#if S_DISCRETE_INPUT_NDISCRETES > 0
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucDiscreteInputBuf;
    USHORT          DISCRETE_INPUT_START;
    USHORT          DISCRETE_INPUT_NDISCRETES;
    USHORT          usDiscreteInputStart;
    iNReg =  usNDiscrete / 8 + 1;

    pucDiscreteInputBuf = ucSDiscInBuf;
    DISCRETE_INPUT_START = S_DISCRETE_INPUT_START;
    DISCRETE_INPUT_NDISCRETES = S_DISCRETE_INPUT_NDISCRETES;
    usDiscreteInputStart = usSDiscInStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if ((usAddress >= DISCRETE_INPUT_START) && (usAddress + usNDiscrete    <= DISCRETE_INPUT_START + DISCRETE_INPUT_NDISCRETES))
    {
        iRegIndex = (USHORT) (usAddress - usDiscreteInputStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usDiscreteInputStart) % 8;

        if(!app_mb_din_exists(iRegIndex, iRegBitIndex)) {
        	return MB_ENOREG;
        }


        while (iNReg > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteInputBuf[iRegIndex++],
                    iRegBitIndex, 8);
            iNReg--;
        }
        pucRegBuffer--;
        /* last discrete */
        usNDiscrete = usNDiscrete % 8;
        /* filling zero to high bit */
        *pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
        *pucRegBuffer = *pucRegBuffer >> (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
#else
	return MB_ENOREG;
#endif
}

#endif
