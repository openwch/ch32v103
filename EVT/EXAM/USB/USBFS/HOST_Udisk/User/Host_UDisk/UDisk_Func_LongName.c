/********************************** (C) COPYRIGHT *******************************
* File Name          : Udisk_Func_longname.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/11/22
* Description        : USB full-speed port host operation functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*******************************************************************************/
/* Header File */
#include "UDisk_Operation.h"

/*******************************************************************************/
/* Variable Definition */
uint8_t    LongNameBuf[ LONG_NAME_BUF_LEN ];
/*
Long file name example(UNICODEthe endianness of the encoding must match theUNICODE_ENDIAN definition)
The following is the encoded content inLongName:
Create a long file name with two input parameters: 1.use(unicode  big-endian), terminate the string with two 00s0 to mark the end;2.ANSIencode the short file name.TXT
*/
uint8_t LongName[ ] =
#if UNICODE_ENDIAN == 1
{
    0x5E, 0xFA, 0x7A, 0xCB, 0x95, 0x7F, 0x65, 0x87, 0x4E, 0xF6, 0x54, 0x0D, 0xFF, 0x0C, 0x8F, 0x93,
    0x51, 0x65, 0x4E, 0x24, 0x4E, 0x2A, 0x53, 0xC2, 0x65, 0x70, 0xFF, 0x1A, 0x00, 0x20, 0x00, 0x31,
    0x00, 0x2E, 0x91, 0xC7, 0x75, 0x28, 0x00, 0x28, 0x00, 0x75, 0x00, 0x6E, 0x00, 0x69, 0x00, 0x63,
    0x00, 0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x59, 0x27, 0x7A, 0xEF, 0x00, 0x29, 0xFF, 0x0C,
    0x5B, 0x57, 0x7B, 0x26, 0x4E, 0x32, 0x67, 0x2B, 0x5C, 0x3E, 0x75, 0x28, 0x4E, 0x24, 0x4E, 0x2A,
    0x00, 0x30, 0x88, 0x68, 0x79, 0x3A, 0x7E, 0xD3, 0x67, 0x5F, 0x00, 0x3B, 0x00, 0x32, 0x00, 0x2E,
    0x00, 0x41, 0x00, 0x4E, 0x00, 0x53, 0x00, 0x49, 0x7F, 0x16, 0x78, 0x01, 0x77, 0xED, 0x65, 0x87,
    0x4E, 0xF6, 0x54, 0x0D, 0x00, 0x2E, 0x00, 0x54, 0x00, 0x58, 0x00, 0x54
};
#else
{
    0xFA, 0x5E, 0xCB, 0x7A, 0x7F, 0x95, 0x87, 0x65, 0xF6, 0x4E, 0x0D, 0x54, 0x0C, 0xFF, 0x93, 0x8F,
    0x65, 0x51, 0x24, 0x4E, 0x2A, 0x4E, 0xC2, 0x53, 0x70, 0x65, 0x1A, 0xFF, 0x20, 0x00, 0x31, 0x00,
    0x2E, 0x00, 0xC7, 0x91, 0x28, 0x75, 0x28, 0x00, 0x75, 0x00, 0x6E, 0x00, 0x69, 0x00, 0x63, 0x00,
    0x6F, 0x00, 0x64, 0x00, 0x65, 0x00, 0x20, 0x00, 0x27, 0x59, 0xEF, 0x7A, 0x29, 0x00, 0x0C, 0xFF,
    0x57, 0x5B, 0x26, 0x7B, 0x32, 0x4E, 0x2B, 0x67, 0x3E, 0x5C, 0x28, 0x75, 0x24, 0x4E, 0x2A, 0x4E,
    0x30, 0x00, 0x68, 0x88, 0x3A, 0x79, 0xD3, 0x7E, 0x5F, 0x67, 0x3B, 0x00, 0x32, 0x00, 0x2E, 0x00,
    0x41, 0x00, 0x4E, 0x00, 0x53, 0x00, 0x49, 0x00, 0x16, 0x7F, 0x01, 0x78, 0xED, 0x77, 0x87, 0x65,
    0xF6, 0x4E, 0x0D, 0x54, 0x2E, 0x00, 0x54, 0x00, 0x58, 0x00, 0x54, 0x00
};
#endif

/*********************************************************************
 * @fn      UDisk_USBH_Longname
 *
 * @brief   Demo Function For UDisk long-name Operation(EXAM 13)
 *
 * @return  none
 */
void UDisk_USBH_Longname( void )
{
    uint8_t  ret, i, len;
    uint16_t j;

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        /*==================== The following demonstrates creating and reading long file names ============================*/
        // Copy the long file name(UNICODE big-endian) toLongNameBuf
        len = LongName_Len;
        memcpy( LongNameBuf, LongName, len );
        // Terminate with two0 to mark the end
        LongNameBuf[len] = 0x00;
        LongNameBuf[len + 1] = 0x00;
        // TheANSI encoded short file name of this long file name(8+3 format)
        strcpy( mCmdParam.Create.mPathName, "\\long file name.TXT" );
        i = CHRV3CreateLongName( );
        if( i == ERR_SUCCESS )
        {
            DUG_PRINTF( "Created Long Name OK!\r\n" );
        }
        else
        {
            /* Error code defined in "udisk_operaion.h" */
            DUG_PRINTF( "Error Code: %02X\r\n", (uint16_t)i );
        }

        DUG_PRINTF( "Get long Name#\r\n" );
        strcpy( mCmdParam.Open.mPathName, "\\long file name.TXT" );
        // The full path of the file name must be input above
        i = CHRV3GetLongName( );
        if( i == ERR_SUCCESS )
        {
            // Long file name collection completed,inUNICODE encoding(as per theUNICODE_ENDIAN definition)
            // Stored in theLongNameBuf buffer,, the long file name is terminated with two 0s at the end0.
            // Display all data in the buffer below
            DUG_PRINTF( "LongNameBuf: " );
            for( j=0; j!=LONG_NAME_BUF_LEN; j++ )
            {
                DUG_PRINTF( "%02X ", (uint16_t)LongNameBuf[j] );
            }
            DUG_PRINTF( "\r\n" );
        }
        else
        {
            /* Error code defined in "udisk_operaion.h" */
            DUG_PRINTF( "Error Code: %02X\r\n", (uint16_t)i );
        }
    }
}

/*********************************************************************
 * @fn      CheckNameSum
 *
 * @brief   Check the short file name checksum of the long file name
 *
 * @return  The calculated checksum
 */
uint8_t CheckNameSum( uint8_t *p )
{
uint8_t FcbNameLen;
uint8_t Sum;

    Sum = 0;
    for (FcbNameLen=0; FcbNameLen!=11; FcbNameLen++)
        Sum = ((Sum & 1) ? 0x80 : 0) + (Sum >> 1) + *p++;
    return Sum;
}

/*********************************************************************
 * @fn      AnalyzeLongName
 *
 * @brief   Organize the long file name return the number of26-byte units
 *
 * @return  return the number of26-byte units
 */
uint8_t AnalyzeLongName( void )
{
uint8_t   i, j;
uint16_t  index;

    i = FALSE;
    for( index=0; index!=LONG_NAME_BUF_LEN; index = index + 2 )
    {
        if( ( LongNameBuf[index] == 0 ) && ( LongNameBuf[index+1] == 0 ) )
        {
            i = TRUE;
            break;
        }
    }
    if( ( i == FALSE ) || ( index == 0) )
        return 0;                   // Return0 to indicate a wrong long file name

    i = index % 26;
    if( i != 0 )
    {
        index += 2;
        if( index % 26 != 0 )       // Add0 to terminate exactly
        {
            for( j=i+2; j!=26; j++ )// Fill the remaining data with0XFF
                LongNameBuf[index++] = 0xff;
        }
    }
    return  (index / 26);
}

/*********************************************************************
 * @fn      CHRV3CreateLongName
 *
 * @brief   Create a long file name,the full path of the short file name must be input
 *
 * @return  Operation status
 */
uint8_t CHRV3CreateLongName( void )
{
// Analyze preserve the file path create an empty file get theFDT offset and its sector delete the file
// Shift backward by sectors may fail e.g.FAT12/16when located in the root directory re-create the file after padding
uint8_t   i;
uint8_t   len;                                // Length of the stored path
uint16_t  index;                              // Long file offset index
uint16_t  indexBak;                           // Backup of the long file offset index
uint32_t  Secoffset;                          // Sector offset

uint8_t   Fbit;                               // First entry: write sector
uint8_t   Mult;                               // The long file name length is a multiple of26
uint8_t   MultBak;                            // Backup: the long file name length is a multiple of26

uint16_t  Backoffset;                         // Save the backup of the file offset
uint16_t  BackoffsetBak;                      // Save the backup of the offset backup
uint32_t  BackFdtSector;                      // Save the offset of the previous sector
uint8_t   sum;                                // Save the checksum of the long file name

uint8_t   BackPathBuf[MAX_PATH_LEN];    // Save the file path

    Mult = AnalyzeLongName( );              // Save whether the long file name is a multiple of26
    if( Mult == 0 )
        return ERR_LONG_NAME;
    MultBak = Mult;

    i = CHRV3FileOpen();                    // Return an error if the short file name exists
    if( i == ERR_SUCCESS )
        return ERR_NAME_EXIST;

    i = CHRV3FileCreate( );
    if( i == ERR_SUCCESS )
    {
        Backoffset = CHRV3vFdtOffset;
        BackoffsetBak = Backoffset;
        BackFdtSector = CHRV3vFdtLba;
        sum = CheckNameSum( &DISK_BASE_BUF[Backoffset ] );
        for( i=0; i!=MAX_PATH_LEN; i++ )    // Back up the file path
            BackPathBuf[i] = mCmdParam.Open.mPathName[i];
        CHRV3FileErase( );                  // Delete this file

        Secoffset   = 0;                    // Offset starting from0
        index       = Mult*26;              // Get the length of the long file name
        indexBak    = index;
        Fbit        = FALSE;                // Not entered by default
        // Open the parent  to fill data
        P_RETRY:
        for(len=0; len!=MAX_PATH_LEN; len++)
        {
            if(mCmdParam.Open.mPathName[len] == 0)
                break;                      // Get the string length
        }

        for(i=len-1; i!=0xff; i--)          // Get the parent directory position
        {
            if((mCmdParam.Open.mPathName[i] == '\\') || (mCmdParam.Open.mPathName[i] == '/'))
                break;
        }
        mCmdParam.Open.mPathName[i] = 0x00;

        if( i==0 )                          // Note when opening a first-level directory:the special case starting at the root directory
        {
            mCmdParam.Open.mPathName[0] = '/';
            mCmdParam.Open.mPathName[1] = 0;
        }

        i = CHRV3FileOpen();                // Open the parent directory
        if( i == ERR_OPEN_DIR )
        {
            while( 1 )                      // Fill in a loop until completed
            {
                mCmdParam.Locate.mSectorOffset = Secoffset;
                i = CHRV3FileLocate( );
                if( i == ERR_SUCCESS )
                {
                    if( Fbit )             // Second entry: write sector
                    {
                        if( mCmdParam.Locate.mSectorOffset != 0x0FFFFFFFF )
                        {
                            BackFdtSector = mCmdParam.Locate.mSectorOffset;
                            Backoffset = 0;
                        }
                        else
                        {
                            for( i=0; i!=MAX_PATH_LEN; i++ )// Restore the file path
                                mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                            i = CHRV3FileCreate( );         // Expand the space
                            if( i != ERR_SUCCESS )
                                return i;
                            CHRV3FileErase( );
                            goto P_RETRY;                   // Reopen the parent directory
                        }
                    }

                    if( BackFdtSector == mCmdParam.Locate.mSectorOffset )
                    {
                        mCmdParam.Read.mSectorCount = 1;   // Read one sector into the disk buffer
                        mCmdParam.Read.mDataBuffer = &DISK_BASE_BUF[0];
                        i = CHRV3FileRead( );
                        CHRV3DirtyBuffer( );                // Clear the disk buffer
                        if( i!= ERR_SUCCESS )
                            return i;

                        i = ( CHRV3vSectorSize - Backoffset ) / 32;
                        if( Mult > i )
                            Mult = Mult - i;                // Remaining multiple of
                        else
                        {
                            i = Mult;
                            Mult = 0;
                        }

                        for( len=i; len!=0; len-- )
                        {
                            indexBak -= 26;
                            index = indexBak;
                            for( i=0; i!=5; i++)            // 1-5 characters of the long file name
                            {                               // On the diskUNICODE stored in little-endian
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + i*2 + 2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + i*2 + 2 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            for( i =0; i!=6; i++)           // 6-11 characters of the long file name
                            {
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + 14 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 14 + i*2 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + 14 + i*2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 14 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            for( i=0; i!=2; i++)            // 12-13 characters of the long file name
                            {
                                #if UNICODE_ENDIAN == 1
                                DISK_BASE_BUF[Backoffset + 28 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 28 + i*2 ] =
                                    LongNameBuf[index++];
                                #else
                                DISK_BASE_BUF[Backoffset + 28 + i*2 ] =
                                    LongNameBuf[index++];
                                DISK_BASE_BUF[Backoffset + 28 + i*2 + 1 ] =
                                    LongNameBuf[index++];
                                #endif
                            }

                            DISK_BASE_BUF[Backoffset + 0x0b] = 0x0f;
                            DISK_BASE_BUF[Backoffset + 0x0c] = 0x00;
                            DISK_BASE_BUF[Backoffset + 0x0d] = sum;
                            DISK_BASE_BUF[Backoffset + 0x1a] = 0x00;
                            DISK_BASE_BUF[Backoffset + 0x1b] = 0x00;
                            DISK_BASE_BUF[Backoffset] = MultBak--;
                            Backoffset += 32;
                        }

                        if( !Fbit )
                        {
                            Fbit = TRUE;
                            DISK_BASE_BUF[ BackoffsetBak ] |= 0x40;
                        }
                        CHRV3vLbaCurrent = BackFdtSector;
                        i = CHRV3WriteSector( 1, DISK_BASE_BUF );
                        if( i!= ERR_SUCCESS )
                            return i;

                        if( Mult==0 )
                        {   // Restore the file path
					        CHRV3FileClose( );
                            for( i=0; i!=MAX_PATH_LEN; i++ )
                                mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                            i = CHRV3FileCreate( );
                            return i;
                        }
                    }
                }
                else
                    return i;
                Secoffset++;
            }
        }
    }
    return i;
}

/*********************************************************************
 * @fn      GetUpSectorData
 *
 * @brief   Get the data of the previous sector from the current sector, put it in the disk buffer
 *
 * @return  Operation status
 */
uint8_t GetUpSectorData( uint32_t *NowSector )
{
uint8_t  i;
uint8_t  len;             // Length of the stored path
uint32_t index;           // Directory sector offset sector count

    index = 0;
    for(len=0; len!=MAX_PATH_LEN; len++)
    {
        if(mCmdParam.Open.mPathName[len] == 0)          // Get the string length
            break;
    }

    for(i=len-1; i!=0xff; i--)                          // Get the parent directory position
    {
        if((mCmdParam.Open.mPathName[i] == '\\') || (mCmdParam.Open.mPathName[i] == '/'))
            break;
    }
    mCmdParam.Open.mPathName[i] = 0x00;

    if( i==0 )  // Note when opening a first-level directory:the special case starting at the root directory
    {
        mCmdParam.Open.mPathName[0] = '/';
        mCmdParam.Open.mPathName[1] = 0;
        i = CHRV3FileOpen();
        if ( i == ERR_OPEN_DIR )
            goto P_NEXT0;
    }
    else
    {
        i = CHRV3FileOpen();
        if ( i == ERR_OPEN_DIR )
        {
            while( 1 )
            {
                P_NEXT0:
                mCmdParam.Locate.mSectorOffset = index;
                i = CHRV3FileLocate( );
                if( i == ERR_SUCCESS )
                {
                    if( *NowSector == mCmdParam.Locate.mSectorOffset )
                    {
                        if( index==0 )                          // At the start of the root directory sector
                            return ERR_NO_NAME;
                        mCmdParam.Locate.mSectorOffset = --index;
                        i = CHRV3FileLocate( );                 // Read the data of the previous sector
                        if( i == ERR_SUCCESS )
                        {                                       // Save the current sector count below
                            *NowSector = mCmdParam.Locate.mSectorOffset;
                            mCmdParam.Read.mSectorCount = 1;   // Read one sector into the disk buffer
                            mCmdParam.Read.mDataBuffer = &DISK_BASE_BUF[0];
                            i = CHRV3FileRead( );
                            CHRV3DirtyBuffer( );                // Clear the disk buffer
                            return i;
                        }
                        else
                            return i;
                    }
                }
                else
                    return i;
                index++;
            }
        }
    }
    return i;
}

/*********************************************************************
 * @fn      CHRV3GetLongName
 *
 * @brief   From the full short file name path(which can be a file or a folder)get the corresponding long file name
 *
 * @return  Operation status
 */
uint8_t CHRV3GetLongName( void )
{
// Requires variable sector size
// Step 1: check whether the file is found when opening it,analyze whether the file exists,and get theFDT offset in this sector and the sector it is in
// Step 2: analyze the above info for long file name existence and whether it is at the start of the first sector of the directory
// Step 3: shift backward by one sector?read the long file name ((sector512-byteUdisk)
uint8_t   i;
uint16_t  index;          // Index within the long file name buffer
uint32_t  BackFdtSector;  // Save the offset of the previous sector
uint8_t   sum;            // Save the checksum of the long file name
//uint16_t  Backoffset;     // Save the backup of the file offset
uint16_t  offset;         // File offset within the sector is a multiple of32
uint8_t   FirstBit;       // Flag: long file name spans two sectors
uint8_t   BackPathBuf[MAX_PATH_LEN]; // Save the file path

    i = CHRV3FileOpen( );
    if( ( i == ERR_SUCCESS ) || ( i == ERR_OPEN_DIR ) )
    {
        for( i=0; i!=MAX_PATH_LEN; i++ )
            BackPathBuf[i] = mCmdParam.Open.mPathName[i];
        // Backup of the path completed above

        sum = CheckNameSum( &DISK_BASE_BUF[CHRV3vFdtOffset] );
        index = 0;
        FirstBit = FALSE;
//        Backoffset = CHRV3vFdtOffset;
        BackFdtSector = CHRV3vFdtLba;
        if( CHRV3vFdtOffset == 0 )
        {
            // First check whether it is at the start of a sector whether it is at the start of the root directory , otherwise shift backward
            if( FirstBit == FALSE )
                FirstBit = TRUE;
            i = GetUpSectorData( &BackFdtSector );
            if( i == ERR_SUCCESS )
            {
                CHRV3vFdtOffset = CHRV3vSectorSize;
                goto P_NEXT1;
            }
        }
        else
        {
            // Read the offset data until the end. If not enough, shift backward
            P_NEXT1:
            offset = CHRV3vFdtOffset;
            while( 1 )
            {
                if( offset != 0 )
                {
                    offset = offset - 32;
                    if( ( DISK_BASE_BUF[offset + 11] == ATTR_LONG_NAME )
                        && (  DISK_BASE_BUF[offset + 13] == sum ) )
                    {
                        if( (index + 26) > LONG_NAME_BUF_LEN )
                            return ERR_BUF_OVER;

                        for( i=0; i!=5; i++)            // 1-5 characters of the long file name
                        {                               // On the diskUNICODE stored in little-endian
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 2];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 1];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + i*2 + 2];
                            #endif
                        }

                        for( i =0; i!=6; i++)           // 6-11 characters of the long file name
                        {
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 14 + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + + 14 + i*2 ];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + + 14 + i*2 ];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 14 + i*2 + 1];
                            #endif

                        }

                        for( i=0; i!=2; i++)            // 12-13 characters of the long file name
                        {
                            #if UNICODE_ENDIAN == 1
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 + 1];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 ];
                            #else
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 ];
                            LongNameBuf[index++] =
                                DISK_BASE_BUF[offset + 28 + i*2 + 1];
                            #endif
                        }

                        if( DISK_BASE_BUF[offset] & 0X40 )
                        {
                            if( ! (((LongNameBuf[index -1] ==0x00)
                                && (LongNameBuf[index -2] ==0x00))
                                || ((LongNameBuf[index -1] ==0xFF)
                                && (LongNameBuf[index -2 ] ==0xFF))))
                            {                           // Handle file names with length exactly26 bytes
                                if(index + 52 >LONG_NAME_BUF_LEN )
                                    return ERR_BUF_OVER;
                                LongNameBuf[ index ] = 0x00;
                                LongNameBuf[ index + 1] = 0x00;
                            }
                            return ERR_SUCCESS;         // Long file name collection completed successfully
                        }
                    }
                    else
                        return ERR_NO_NAME;             // Wrong long file name,the program returns
                }
                else
                {
                    if( FirstBit == FALSE )
                        FirstBit = TRUE;
                    else                                // Otherwise enter a second time
                    {
                        for( i=0; i!=MAX_PATH_LEN; i++ )// Restore the path
                            mCmdParam.Open.mPathName[i] = BackPathBuf[i];
                    }
                    i = GetUpSectorData( &BackFdtSector );
                    if( i == ERR_SUCCESS )
                    {
                        CHRV3vFdtOffset = CHRV3vSectorSize;
                        goto P_NEXT1;
                    }
                    else
                        return i;
                    // Shift backward by sectors
                }
            }
        }
    }
    return i;                // Return an error
}

