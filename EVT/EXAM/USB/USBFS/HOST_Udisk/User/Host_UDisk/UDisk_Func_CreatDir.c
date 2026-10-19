/********************************** (C) COPYRIGHT *******************************
* File Name          : UDisk_Func_CreatDir.c
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

/*******************************************************************************
* Function Name  : CreateDirectory
* Description    : Create a directory and open it,if the directory already exists open it directly, the directory name is inmCmdParam.Create.mPathName,following the same rules as file names
* Input          :
* Output         : None
* Return         : ERR_SUCCESS = Directory opened or created successfully,
                   ERR_FOUND_NAME = A file with the same name already exists,
                   ERR_MISS_DIR = Invalid path name or the parent directory does not exist
*******************************************************************************/
uint8_t CreateDirectory( void )
{
    uint8_t   i, j;
    uint32_t  UpDirCluster;
    uint8_t * DirXramBuf;
    uint8_t  *DirConstData;
    j = 0xFF;
    for ( i = 0; i != sizeof( mCmdParam.Create.mPathName ); i ++ )    //Check the directory path
    {
        if ( mCmdParam.Create.mPathName[ i ] == 0 )
        {
            break;
        }
        if ( mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR1 || mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR2 )
        {
            j = i;                                                     //Record the parent directory
        }
    }
    i = ERR_SUCCESS;
    if ( j == 0 || (j == 2 && mCmdParam.Create.mPathName[1] == ':') )
    {
        UpDirCluster = 0;                                              //Create the subdirectory under the root directory
    }
    else
    {
        if ( j != 0xFF )                                               //For an absolute path, get the start cluster number of the parent directory
        {
            mCmdParam.Create.mPathName[ j ] = 0;
            i = CHRV3FileOpen( );                                      //Open the parent directory
            if ( i == ERR_SUCCESS )
            {
                i = ERR_MISS_DIR;                                      //It is a file rather than a directory
            }
            else if ( i == ERR_OPEN_DIR )
            {
                i = ERR_SUCCESS;                                       //Parent directory opened successfully
            }
            mCmdParam.Create.mPathName[ j ] = PATH_SEPAR_CHAR1;        //Restore the directory separator
        }
        UpDirCluster = CHRV3vStartCluster;                             //Save the start cluster number of the parent directory
    }
    if ( i == ERR_SUCCESS )                                            //Start cluster number of the parent directory obtained successfully
    {
        i = CHRV3FileOpen( );                                          //Open the current level subdirectory
        if ( i == ERR_SUCCESS )
        {
            i = ERR_FOUND_NAME;                                        //It is a file rather than a directory
        }
        else if ( i == ERR_OPEN_DIR )
        {
            i = ERR_SUCCESS;                                           //The directory already exists
        }
        else if ( i == ERR_MISS_FILE )                                 //The directory does not exist,it can be created
        {
            i = CHRV3FileCreate( );                                    //Create the directory the same way as creating a file
            if ( i == ERR_SUCCESS )
            {
                if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                {
                    memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);     //IfFILE_DATA_BUF andDISK_BASE_BUF share the same buffer, the disk buffer must be cleared
                }
                DirXramBuf = pDISK_FAT_BUF;                            //File data buffer
                DirConstData = ".          \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0..         \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0";
                for ( i = 0x40; i != 0; i -- )                         //Reserved entries of the directory,pointing to itself and the parent directory respectively
                {
                    *DirXramBuf = *DirConstData;
                    DirXramBuf ++;
                    DirConstData ++;
                }
                *(pDISK_FAT_BUF+0x1A) = ( (uint8_t *)&CHRV3vStartCluster )[3];//Its own start cluster number
                *(pDISK_FAT_BUF+0x1B) = ( (uint8_t *)&CHRV3vStartCluster )[2];
                *(pDISK_FAT_BUF+0x14) = ( (uint8_t *)&CHRV3vStartCluster )[1];
                *(pDISK_FAT_BUF+0x15) = ( (uint8_t *)&CHRV3vStartCluster )[0];
                *(pDISK_FAT_BUF+0x20+0x1A) = ( (uint8_t *)&UpDirCluster )[3];//Start cluster number of the parent directory
                *(pDISK_FAT_BUF+0x20+0x1B) = ( (uint8_t *)&UpDirCluster )[2];
                *(pDISK_FAT_BUF+0x20+0x14) = ( (uint8_t *)&UpDirCluster )[1];
                *(pDISK_FAT_BUF+0x20+0x15) = ( (uint8_t *)&UpDirCluster )[0];
//              for ( count = 0x40; count != CHRV3vSectorSizeH*256; count ++ ) {  /* Clear the remaining part of the directory area */
//                  *DirXramBuf = 0;
//                  DirXramBuf ++;
//              }
                mCmdParam.Write.mSectorCount = 1;
                mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;                //Point to the start address of the file data buffer
                i = CHRV3FileWrite( );                                      //Write data to the file
                if ( i == ERR_SUCCESS )
                {
                    DirXramBuf = pDISK_FAT_BUF;
                    for ( i = 0x40; i != 0; i -- )                          //Clear the directory area
                    {
                        *DirXramBuf = 0;
                        DirXramBuf ++;
                    }
                    for ( j = 1; j != CHRV3vSecPerClus; j ++ )
                    {
                        if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                        {
                            memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);   //IfFILE_DATA_BUF andDISK_BASE_BUF share the same buffer, the disk buffer must be cleared
                        }
                        mCmdParam.Write.mSectorCount = 1;
                        mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;         //Point to the start address of the file data buffer
                        i = CHRV3FileWrite( );                               //Clear the remaining sectors of the directory
                        if ( i != ERR_SUCCESS )
                        {
                            break;
                        }
                    }
                    if ( j == CHRV3vSecPerClus )                              //Directory cleared successfully
                    {
                        mCmdParam.Modify.mFileSize = 0;                       //The length of a directory is always0
                        mCmdParam.Modify.mFileDate = 0xFFFF;
                        mCmdParam.Modify.mFileTime = 0xFFFF;
                        mCmdParam.Modify.mFileAttr = 0x10;                    //Set the directory attribute
                        i = CHRV3FileModify( );                               //Change the file information to a directory
                    }
                }
            }
        }
    }
    return( i );
}

/*********************************************************************
 * @fn      UDisk_USBH_CreatDirectory
 *
 * @brief   Demo Function For UDisk Create Directory (EXAM9)
 *
 * @return  none
 */
void UDisk_USBH_CreatDirectory( void )
{
    uint8_t  i;
    uint8_t  ret;

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        printf("CHRV3DiskStatus:%02x\r\n",CHRV3DiskStatus);
        printf( "Create Level 1 Directory /YEAR2004 \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004" );             //Directory name,this directory is created under the root directory
        ret = CreateDirectory( );                                      //Create or open the directory
        mStopIfError( ret );
        /* Directory created or opened successfully,now create a demo file in this subdirectory */
        printf( "Create New File /YEAR2004/DEMO2004.TXT \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/DEMO2004.TXT" );//File name
        ret = CHRV3FileCreate( );                                      //Create and open the file,if the file already exists, delete it first and then create a new one
        mStopIfError( ret );
        printf( "Write some data to file DEMO2004.TXT \r\n" );
        i = sprintf( Com_Buffer, "Demo file\xd\xa" );
        mCmdParam.ByteWrite.mByteCount = i;                            //Specify the number of bytes to write this time,a single read/write must not exceedMAX_BYTE_IO
        mCmdParam.ByteWrite.mByteBuffer = Com_Buffer;                  //Point to the buffer
        ret = CHRV3ByteWrite( );                                       //Write data to the file in bytes,a single read/write must not exceedMAX_BYTE_IO
        mStopIfError( ret );
        printf( "Close file DEMO2004.TXT \r\n" );
        mCmdParam.Close.mUpdateLen = 1;                                //Automatically calculate the file length,for byte-based file writing,it is recommended to let the library close the file so the file length is updated automatically
        ret = CHRV3FileClose( );
        mStopIfError( ret );
        /* Create a second-level subdirectory below,same method as the first-level subdirectory above */
        printf( "Create Level 2 Directory /YEAR2004/MONTH05 \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/MONTH05" );    //Directory name,this directory is created under theYEAR2004subdirectory,YEAR2004the YEAR2004 directory must already exist
        ret = CreateDirectory( );                                     //Create or open the directory
        mStopIfError( ret );
        printf( "Close\r\n" );
        mCmdParam.Close.mUpdateLen = 0;                               //No automatic file length update is needed for directories
        ret = CHRV3FileClose( );                                      //Close the directory,directories do not need to be closed,closing here is only to prevent misoperation below
        mStopIfError( ret );
    }
}


