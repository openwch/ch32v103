/********************************** (C) COPYRIGHT *******************************
* File Name          : Udisk_Func_BasicOp.c
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
__attribute__((aligned(4)))  uint8_t  MY_DATA_BUF[ DISK_BASE_BUF_LEN ];   /* MY_DATA_BUFpoints to the externalRAM disk data buffer,with a length of at least one sector,used for user data caching */
uint8_t  *pCodeStr;

/*********************************************************************
 * @fn      UDisk_USBH_ByteOperation
 *
 * @brief   Demo Function For UDisk File Byte-operation
 *          including Create\Modify\Read\Erase (EXAM1)
 *
 * @return  none
 */
void UDisk_USBH_ByteOperation( void )
{
    uint8_t ret;
    uint8_t i,t;
    uint16_t TotalCount = 0;

    UDisk_USBH_DiskReady( );
    if( (CHRV3DiskStatus >= DISK_MOUNTED)&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        printf("CHRV3DiskStatus:%02x\r\n",CHRV3DiskStatus);
        /* Read file */
        strcpy( (char *)mCmdParam.Open.mPathName, "/NEWFILE.C" ); //Set the file path and name to operate on/NEWFILE.C
        ret = CHRV3FileOpen( );                                       //Open the file
        if ( ret == ERR_MISS_DIR || ret == ERR_MISS_FILE )            //File not found
        {
            //File creation demo
            printf( "Find No File And Create\r\n" );
            strcpy( (char *)mCmdParam.Create.mPathName, "/NEWFILE.C" );  //New file name,in the root directory,Chinese file name
            ret = CHRV3FileCreate( );                                        //Create and open the file,if the file already exists, delete it first and then create a new one
            mStopIfError( ret );
            printf( "ByteWrite\r\n" );
            //Should check whether the data length matches the defined buffer length; write multiple times if it exceeds the buffer length
            i = sprintf( (char *)Com_Buffer,"Note: \xd\xaThis program performsU disk file read/write in bytes,simple demo only.\xd\xa");
            for(t=0; t<10; t++)
            {
                mCmdParam.ByteWrite.mByteCount = i;                           //Specify the number of bytes to write this time
                mCmdParam.ByteWrite.mByteBuffer = Com_Buffer;                 //Point to the buffer
                ret = CHRV3ByteWrite( );                                      //Write data to the file in bytes
                mStopIfError( ret );
                printf("Successfully wrote %02X times\r\n",(uint16_t)t);
            }

            //Demo: modify file attributes
            printf( "Modify\r\n" );
            mCmdParam.Modify.mFileAttr = 0xff;   //Input parameter: new file attribute,if0FFH, do not modify
            mCmdParam.Modify.mFileTime = 0xffff;   //Input parameter: new file time,if0FFFFH, do not modify,use the default time created with the new file
            mCmdParam.Modify.mFileDate = MAKE_FILE_DATE( 2015, 5, 18 );  //Input parameter: new file date: 2015.05.18
            mCmdParam.Modify.mFileSize = 0xffffffff;  // Input parameter: new file length,for byte-based writes the library should update the length automatically when closing the file,so do not modify it here
            i = CHRV3FileModify( );   //Modify the information of the current file,modify the date
            mStopIfError( i );
            printf( "Close\r\n" );
            mCmdParam.Close.mUpdateLen = 1;     //Automatically calculate the file length,for byte-based file writing,it is recommended to let the library close the file so the file length is updated automatically
            i = CHRV3FileClose( );
            mStopIfError( i );

            /* Delete a file */
//            printf( "Erase\n" );
//            strcpy( (char *)mCmdParam.Create.mPathName, "/OLD" );  //The file name to be deleted,under the root directory
//            i = CHRV3FileErase( );  //Delete the file and close it
//            if ( i != ERR_SUCCESS ) printf( "Error: %02X\n", (uint16_t)i );  //Display error
        }
        else
        {
            /* 1. Write file */
            printf( "ByteWrite\r\n" ); //To append new data to the end of the original file,the file pointer can be moved
            mCmdParam.ByteLocate.mByteOffset = 0xffffffff;  //Move to the end of the file
            CHRV3ByteLocate( );           //Should check whether the data length matches the defined buffer length; write multiple times if it exceeds the buffer length
            i = sprintf( (char *)Com_Buffer,"Note: \xd\xaThis program performsU disk file read/write in bytes,simple demo only.\xd\xa"); //Demo text
            for(t=0; t<10; t++)
            {
                mCmdParam.ByteWrite.mByteCount = i;               //Specify the number of bytes to write this time
                mCmdParam.ByteWrite.mByteBuffer = Com_Buffer;     // Point to the buffer
                ret = CHRV3ByteWrite( );                          // Write data to the file in bytes
                mStopIfError( ret );
                printf("Successfully wrote %02X times\r\n",(uint16_t)t);
            }

            //Demo: modify file attributes
            printf( "Modify\r\n" );
            mCmdParam.Modify.mFileAttr = 0xff;   //Input parameter: new file attribute,if0FFH, do not modify
            mCmdParam.Modify.mFileTime = 0xffff;   //Input parameter: new file time,if0FFFFH, do not modify,use the default time created with the new file
            mCmdParam.Modify.mFileDate = MAKE_FILE_DATE( 2015, 5, 18 );  //Input parameter: new file date: 2015.05.18
            mCmdParam.Modify.mFileSize = 0xffffffff;  // Input parameter: new file length,for byte-based writes the library should update the length automatically when closing the file,so do not modify it here
            i = CHRV3FileModify( );   //Modify the information of the current file,modify the date
            mStopIfError( i );
            printf( "Close\r\n" );
            mCmdParam.Close.mUpdateLen = 1;     //Automatically calculate the file length,for byte-based file writing,it is recommended to let the library close the file so the file length is updated automatically
            i = CHRV3FileClose( );
            mStopIfError( i );

            /* 2. Read the firstN bytes of the file */
            TotalCount = 60;                                                  //Set the total length prepared for reading100 bytes
            strcpy( (char *)mCmdParam.Open.mPathName, "/NEWFILE.C" );     //Set the file path and name to operate on/NEWFILE.C
            CHRV3FileOpen( );                                                 //Open the file
            printf( "The first%d characters read out are:\r\n",TotalCount );
            while ( TotalCount )
            {
                //If the file is large,if one read cannot finish it,callCH103ByteRead to continue reading,the file pointer moves backward automatically
                if ( TotalCount > (MAX_PATH_LEN-1) )
                {
                    t = MAX_PATH_LEN-1; // Much remaining data,limit the length of a single read/write to not exceed sizeof( mCmdParam.Other.mBuffer )
                }
                else
                {
                    t = TotalCount; //Number of remaining bytes at the end
                }
                mCmdParam.ByteRead.mByteCount = t;                   //Request to read dozens of bytes of data
                mCmdParam.ByteRead.mByteBuffer= &Com_Buffer[0];
                ret = CHRV3ByteRead( );                              //Read a data block in bytes,a single read/write must not exceed MAX_BYTE_IOMAX_BYTE_IO,on the second call continue reading right after the previous position
                TotalCount -= mCmdParam.ByteRead.mByteCount;         //Count,subtract the number of characters actually read
                for ( i=0; i!=mCmdParam.ByteRead.mByteCount; i++ )
                {
                    printf( "%c", mCmdParam.ByteRead.mByteBuffer[i] ); //Display the read character
                }
                printf( "\r\n" );

                if ( mCmdParam.ByteRead.mByteCount < t ) //The actually read character count is less than requested,meaning the end of the file has been reached
                {
                    printf( "\r\n" );
                    printf( "End of file\r\n" );
                    break;
                }
            }
            i = CHRV3FileClose( ); //Close the file
            mStopIfError( i );
        }
    }
}

/*********************************************************************
 * @fn      UDisk_USBH_SectorOperation
 *
 * @brief   Demo Function For UDisk File Sector-operation
 *          including Create\Modify\Read\Erase (EXAM6)
 *
 * @return  none
 */
void UDisk_USBH_SectorOperation( void )
{
    uint8_t  ret, SecCount, s;
    uint8_t  i;
    uint16_t tmp;
    uint8_t  tmpbuf[64];

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        /* Query the physical capacity of the disk */
        printf( "DiskSize\r\n" );
        i = CHRV3DiskQuery( );
        mStopIfError( i );
        printf( "TotalSize = %u MB \n", (unsigned int)( mCmdParam.Query.mTotalSector * CHRV3vSectorSizeB / 2 ) );  //Displayed as capacity in units ofMB

        /* Read the original file */
        printf( "Open\r\n" );
        strcpy( mCmdParam.Open.mPathName, "/NEWFILE.TXT" );//File name,this file is in theC51 subdirectory
        s = CHRV3FileOpen( );                       //Open the file
        if ( s == ERR_MISS_DIR || s == ERR_MISS_FILE )//File not found
        {
            printf( "File not found\r\n" );
        }
        else                                        //File found or error occurred
        {
            printf( "Query\r\n" );
            i = CHRV3FileQuery( );                  //Query the information of the current file
            mStopIfError( i );
            printf( "Read\r\n" );
            CHRV3vFileSize = CHRV3vFileSize+(sizeof( MY_DATA_BUF )-1);    //Length of the original file
            SecCount = CHRV3vFileSize/ sizeof( MY_DATA_BUF )  ;//Calculate the sector count of the file,since read/write is sector-based,first addCHRV3vSectorSize-1, in order to read the trailing part of the file less than1 sector
            printf( "Size=%ld, Sec=%d\r\n", CHRV3vFileSize, (uint16_t)SecCount );
            while(SecCount--)
            {
                mCmdParam.Read.mSectorCount = sizeof( MY_DATA_BUF )/512;  //Read all data,if exceeding2 sectors, only read2 sectors
                mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0];//Point to the start address of the file data buffer
                i = CHRV3FileRead( );                    //Read data from the file
                mStopIfError( i );
                if(SecCount == 0) break;
                /*
                for(tmp=0; tmp<sizeof( MY_DATA_BUF ); tmp++)
                {
                printf("%02X ",(uint16_t)MY_DATA_BUF[tmp]);
                }
                printf("\n");
                */
            }
            tmp = (CHRV3vFileSize-(sizeof( MY_DATA_BUF )-1))%sizeof( MY_DATA_BUF );
            if((tmp == 0)&&(CHRV3vFileSize != 0)) tmp = sizeof( MY_DATA_BUF );
            CHRV3vFileSize = CHRV3vFileSize-(sizeof( MY_DATA_BUF )-1);    //Restore the length of the original file
            /*
            for(i=0; i<tmp; i++)
            {
              printf("%02X ",(uint16_t)MY_DATA_BUF[i]);
            }
            printf("\n");
            */
            /*
                         If the file is large,if one read cannot finish it,callCHRV3FileRead to continue reading,the file pointer moves backward automatically
             while ( 1 )
             {
               c = 4;   Read4 sectors each time,the larger the defined buffer, the more sectors per read
               mCmdParam.Read.mSectorCount = c;   Specify the number of sectors to read
               mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0];  Point to the start address of the file data buffer
               CHRV3FileRead();   After reading, the file pointer moves backward automatically process the data
               if ( mCmdParam.Read.mSectorCount < c ) break;   A smaller actually read sector count means the end of the file
             }
                                               To read/write from a specified position,the file pointer can be moved
              mCmdParam.Locate.mSectorOffset = 3;  Skip the first3 sectors of the file and start read/write
              i = CHRV3FileLocate( );
              mCmdParam.Read.mSectorCount = 10;
              mCmdParam.Read.mDataBuffer = &MY_DATA_BUF[0];  Point to the start address of the file data buffer
              CHRV3FileRead();   Directly read data starting from byte(CHRV3vSectorSizeH*256*3)of the file,the first3 sectors are skipped
                                               To append new data to the end of the original file,the file pointer can be moved
              i = CHRV3FileOpen( );
              mCmdParam.Locate.mSectorOffset = 0xffffffff;  Move to the end of the file,in sectors,if the original file is3 bytes,then start appending from byteCHRV3vSectorSizeH onwards
              i = CHRV3FileLocate( );
              mCmdParam.Write.mSectorCount = 10;
              mCmdParam.Write.mDataBuffer = &MY_DATA_BUF[0];
              CHRV3FileWrite();   Append data after the original file
                                               UsingCHRV3FileRead, the start address of the data buffer can be defined by the user
              mCmdParam.Read.mSectorCount = 2;
              mCmdParam.Read.mDataBuffer = 0x50;  Place the read data into the buffer starting at50Hthe start address of the buffer must be specified
              CHRV3FileRead();   Read2 sectors from the file into the specified buffer
                                                UsingCHRV3FileWrite, the start address of the data buffer can be defined by the user
              mCmdParam.Wiite.mSectorCount = 2;
              mCmdParam.Write.mDataBuffer = 0x50;  Write the data in the buffer starting at50H
              CHRV3FileWrite();   Write the data in the specified buffer to2 sectors of the file
            */
            printf( "Close\r\n" );
            i = CHRV3FileClose( );                            //Close the file
            mStopIfError( i );
        }
        printf( "Create\r\n" );
        strcpy( mCmdParam.Create.mPathName, "/NEWFILE.TXT" );//New file name,in the root directory,Chinese file name
        s = CHRV3FileCreate( );                               //Create and open the file,if the file already exists, delete it first and then create a new one */
        mStopIfError( s );
        printf( "Write\r\n" );
        strcpy( tmpbuf, "0000ABCDEFGHIJKLMNOPQRSTUVWXYZ\xd\xa" );//Prepare the file data to write
        for(i=0; i<(DISK_BASE_BUF_LEN/sizeof(tmpbuf)); i++)
        {
            tmp=i*sizeof(tmpbuf);
            strcpy(&MY_DATA_BUF[tmp],tmpbuf);
        }
        for(tmp=0; tmp<sizeof(MY_DATA_BUF); tmp++)
        {
            printf("%02X",(uint16_t)MY_DATA_BUF[tmp]);
        }
        printf("\r\n");
        for(s=0; s<10; s++)
        {
            mCmdParam.Write.mSectorCount = 1;                 //Write the data of all sectors
            mCmdParam.Write.mDataBuffer = &MY_DATA_BUF[0];    //Point to the start address of the file data buffer
            i = CHRV3FileWrite( );                            //Write data to the file
            mStopIfError( i );
            printf("Successfully wrote %02X times\r\n",(uint16_t)s);
        }
        /* printf( "Modify\n" );
           mCmdParam.Modify.mFileAttr = 0xff;   Input parameter: new file attribute,if0FFH, do not modify
           mCmdParam.Modify.mFileTime = 0xffff;   Input parameter: new file time,if0FFFFH, do not modify,use the default time created with the new file
           mCmdParam.Modify.mFileDate = MAKE_FILE_DATE( 2015, 5, 18 );  Input parameter: new file date: 2015.05.18
           mCmdParam.Modify.mFileSize = 0xffffffff;   Input parameter: new file length,for byte-based writes the library should update the length automatically when closing the file,so do not modify it here
           i = CHRV3FileModify( );   Modify the information of the current file,modify the date
           mStopIfError( i );
        */
        printf( "Close\r\n" );
        mCmdParam.Close.mUpdateLen = 1;                        //Automatically calculate the file length,for byte-based file writing,it is recommended to let the library close the file so the file length is updated automatically
        i = CHRV3FileClose( );
        mStopIfError( i );
        /* Delete a file */
        /*printf( "Erase\n" );
          strcpy( mCmdParam.Create.mPathName, "/OLD.TXT" );  The file name to be deleted,under the root directory
          i = CHRV3FileErase( );  Delete the file and close it
          if ( i != ERR_SUCCESS ) printf( "Error File not exist: %02X\n", (uint16_t)i );  Display error
        */
        printf( "Udisk demo completed\r\n" );
    }
}


/*********************************************************************
 * @fn      UDisk_USBH_EnumFiles
 *
 * @brief   Demo Function For Enumerating files in UDisk(EXAM11)
 *
 * @return  none
 */
void UDisk_USBH_EnumFiles( void )
{
    uint8_t  i, s, ret;
    uint16_t j;

    ret = UDisk_USBH_DiskReady( );
    if( ( ret == DISK_READY )&&( UDisk_Opeation_Flag == 1 ) )
    {
        UDisk_Opeation_Flag = 0;
        /* Read the original file */
        printf( "Open\r\n" );
        strcpy( mCmdParam.Open.mPathName, "/C51/CHRV3HFT.C" );//File name,this file is in theC51 subdirectory
        s = CHRV3FileOpen( );                        //Open the file
        /* List files */
        if ( s == ERR_MISS_DIR )
        {
            printf("List all files if the file does not exist\r\n");  //C51 subdirectory does not exist, list all files under the root directory
            pCodeStr = "/*";
        }
        else
        {
            pCodeStr = "/C51/*";                     //CHRV3HFT.C file is not found, list files under the\C51 directory starting withCHRV3
        }
        printf( "List file %s\r\n", pCodeStr );
        for ( j = 0; j < 10000; j ++ )               //Search at most the first10000 files,there is actually no limit
        {
            strcpy( (char *)mCmdParam.Open.mPathName, pCodeStr );//Search the file name,* is the wildcard,applicable to all files or subdirectories
            i = strlen( mCmdParam.Open.mPathName );
            mCmdParam.Open.mPathName[ i ] = 0xFF;    //Replace the terminator with the search index according to the string length,from0to254,, if it is0xFFi.e.255, it means the search index is in theCHRV3vFileSizevariable
            CHRV3vFileSize = j;                      //Specify the search/enumeration index
            i = CHRV3FileOpen( );                    //Open the file,if the file name contains the wildcard **,only search instead of opening
            /* CHRV3FileEnum  and CHRV3FileOpen  is: when the latter returnsERR_FOUND_NAME then the former returnsERR_SUCCESS */
            if ( i == ERR_MISS_FILE )
            {
                break;                                //No more matching files can be found,there are no matching file names anymore
            }
            if ( i == ERR_FOUND_NAME )
            {
                /* Found the file name matching the wildcard,the file name and its full path are in the command buffer */
                printf( "  match file %04d#: %s\r\n", (unsigned int)j, mCmdParam.Open.mPathName );//Display the index and the found matching file name or subdirectory name
                continue;                             //Continue searching for the next matching file name,the index will increment by 1 on the next search1
            }
            else
            {
                /* Error occurred */
                mStopIfError( i );
                break;
            }
        }
        printf( "Close\r\n" );
        CHRV3FileClose( );                            //Close the file
        printf( "Udisk demo completed\r\n" );
    }
}
