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
#include "Udisk_Operation.h"

/*******************************************************************************
* Function Name  : CreateDirectory
* Description    : �½�Ŀ¼����,���Ŀ¼�Ѿ�������ֱ�Ӵ򿪣�Ŀ¼����mCmdParam.Create.mPathName��,���ļ���������ͬ
* Input          :
* Output         : None
* Return         : ERR_SUCCESS = ��Ŀ¼�ɹ����ߴ���Ŀ¼�ɹ�,
                   ERR_FOUND_NAME = �Ѿ�����ͬ���ļ�,
                   ERR_MISS_DIR = ·������Ч�����ϼ�Ŀ¼������
*******************************************************************************/
uint8_t CreateDirectory( void )
{
    uint8_t   i, j;
    uint32_t  UpDirCluster;
    uint8_t * DirXramBuf;
    uint8_t  *DirConstData;
    j = 0xFF;
    for ( i = 0; i != sizeof( mCmdParam.Create.mPathName ); i ++ )    //���Ŀ¼·��
    {
        if ( mCmdParam.Create.mPathName[ i ] == 0 )
        {
            break;
        }
        if ( mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR1 || mCmdParam.Create.mPathName[ i ] == PATH_SEPAR_CHAR2 )
        {
            j = i;                                                     //��¼�ϼ�Ŀ¼
        }
    }
    i = ERR_SUCCESS;
    if ( j == 0 || (j == 2 && mCmdParam.Create.mPathName[1] == ':') )
    {
        UpDirCluster = 0;                                              //�ڸ�Ŀ¼�´�����Ŀ¼
    }
    else
    {
        if ( j != 0xFF )                                               //���ھ���·��Ӧ�û�ȡ�ϼ�Ŀ¼����ʼ�غ�
        {
            mCmdParam.Create.mPathName[ j ] = 0;
            i = CHRV3FileOpen( );                                      //���ϼ�Ŀ¼
            if ( i == ERR_SUCCESS )
            {
                i = ERR_MISS_DIR;                                      //���ļ�����Ŀ¼
            }
            else if ( i == ERR_OPEN_DIR )
            {
                i = ERR_SUCCESS;                                       //�ɹ����ϼ�Ŀ¼
            }
            mCmdParam.Create.mPathName[ j ] = PATH_SEPAR_CHAR1;        //�ָ�Ŀ¼�ָ���
        }
        UpDirCluster = CHRV3vStartCluster;                             //�����ϼ�Ŀ¼����ʼ�غ�
    }
    if ( i == ERR_SUCCESS )                                            //�ɹ���ȡ�ϼ�Ŀ¼����ʼ�غ�
    {
        i = CHRV3FileOpen( );                                          //�򿪱�����Ŀ¼
        if ( i == ERR_SUCCESS )
        {
            i = ERR_FOUND_NAME;                                        //���ļ�����Ŀ¼
        }
        else if ( i == ERR_OPEN_DIR )
        {
            i = ERR_SUCCESS;                                           //Ŀ¼�Ѿ�����
        }
        else if ( i == ERR_MISS_FILE )                                 //Ŀ¼������,�����½�
        {
            i = CHRV3FileCreate( );                                    //�Դ����ļ��ķ�������Ŀ¼
            if ( i == ERR_SUCCESS )
            {
                if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                {
                    memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);     //���FILE_DATA_BUF��DISK_BASE_BUF���������������̻�����
                }
                DirXramBuf = pDISK_FAT_BUF;                            //�ļ����ݻ�����
                DirConstData = ".          \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0..         \x10\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x21\x30\x0\x0\x0\x0\x0\x0";
                for ( i = 0x40; i != 0; i -- )                         //Ŀ¼�ı�����Ԫ,�ֱ�ָ��������ϼ�Ŀ¼
                {
                    *DirXramBuf = *DirConstData;
                    DirXramBuf ++;
                    DirConstData ++;
                }
                *(pDISK_FAT_BUF+0x1A) = ( (uint8_t *)&CHRV3vStartCluster )[3];//�������ʼ�غ�
                *(pDISK_FAT_BUF+0x1B) = ( (uint8_t *)&CHRV3vStartCluster )[2];
                *(pDISK_FAT_BUF+0x14) = ( (uint8_t *)&CHRV3vStartCluster )[1];
                *(pDISK_FAT_BUF+0x15) = ( (uint8_t *)&CHRV3vStartCluster )[0];
                *(pDISK_FAT_BUF+0x20+0x1A) = ( (uint8_t *)&UpDirCluster )[3];//�ϼ�Ŀ¼����ʼ�غ�
                *(pDISK_FAT_BUF+0x20+0x1B) = ( (uint8_t *)&UpDirCluster )[2];
                *(pDISK_FAT_BUF+0x20+0x14) = ( (uint8_t *)&UpDirCluster )[1];
                *(pDISK_FAT_BUF+0x20+0x15) = ( (uint8_t *)&UpDirCluster )[0];
//              for ( count = 0x40; count != CHRV3vSectorSizeH*256; count ++ ) {  /* ���Ŀ¼��ʣ�ಿ�� */
//                  *DirXramBuf = 0;
//                  DirXramBuf ++;
//              }
                mCmdParam.Write.mSectorCount = 1;
                mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;                //ָ���ļ����ݻ���������ʼ��ַ
                i = CHRV3FileWrite( );                                      //���ļ�д������
                if ( i == ERR_SUCCESS )
                {
                    DirXramBuf = pDISK_FAT_BUF;
                    for ( i = 0x40; i != 0; i -- )                          //���Ŀ¼��
                    {
                        *DirXramBuf = 0;
                        DirXramBuf ++;
                    }
                    for ( j = 1; j != CHRV3vSecPerClus; j ++ )
                    {
                        if ( pDISK_FAT_BUF == pDISK_BASE_BUF )
                        {
                            memset(pDISK_FAT_BUF,0,CHRV3vSectorSize);   //���FILE_DATA_BUF��DISK_BASE_BUF���������������̻�����
                        }
                        mCmdParam.Write.mSectorCount = 1;
                        mCmdParam.Write.mDataBuffer = pDISK_FAT_BUF;         //ָ���ļ����ݻ���������ʼ��ַ
                        i = CHRV3FileWrite( );                               //���Ŀ¼��ʣ������
                        if ( i != ERR_SUCCESS )
                        {
                            break;
                        }
                    }
                    if ( j == CHRV3vSecPerClus )                              //�ɹ����Ŀ¼
                    {
                        mCmdParam.Modify.mFileSize = 0;                       //Ŀ¼�ĳ�������0
                        mCmdParam.Modify.mFileDate = 0xFFFF;
                        mCmdParam.Modify.mFileTime = 0xFFFF;
                        mCmdParam.Modify.mFileAttr = 0x10;                    //��Ŀ¼����
                        i = CHRV3FileModify( );                               //���ļ���Ϣ�޸�ΪĿ¼
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
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004" );             //Ŀ¼��,��Ŀ¼���ڸ�Ŀ¼��
        ret = CreateDirectory( );                                      //�½����ߴ�Ŀ¼
        mStopIfError( ret );
        /* Ŀ¼�½����ߴ򿪳ɹ�,�����������Ŀ¼���½�һ����ʾ�ļ� */
        printf( "Create New File /YEAR2004/DEMO2004.TXT \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/DEMO2004.TXT" );//�ļ���
        ret = CHRV3FileCreate( );                                      //�½��ļ�����,����ļ��Ѿ���������ɾ�������½�
        mStopIfError( ret );
        printf( "Write some data to file DEMO2004.TXT \r\n" );
        i = sprintf( Com_Buffer, "��ʾ�ļ�\xd\xa" );
        mCmdParam.ByteWrite.mByteCount = i;                            //ָ������д����ֽ���,���ζ�д�ĳ��Ȳ��ܳ���MAX_BYTE_IO
        mCmdParam.ByteWrite.mByteBuffer = Com_Buffer;                  //ָ�򻺳���
        ret = CHRV3ByteWrite( );                                       //���ֽ�Ϊ��λ���ļ�д������,���ζ�д�ĳ��Ȳ��ܳ���MAX_BYTE_IO
        mStopIfError( ret );
        printf( "Close file DEMO2004.TXT \r\n" );
        mCmdParam.Close.mUpdateLen = 1;                                //�Զ������ļ�����,���ֽ�Ϊ��λд�ļ�,�����ó����ر��ļ��Ա��Զ������ļ�����
        ret = CHRV3FileClose( );
        mStopIfError( ret );
        /* �����½�������Ŀ¼,������ǰ���һ����Ŀ¼��ȫ��ͬ */
        printf( "Create Level 2 Directory /YEAR2004/MONTH05 \r\n" );
        strcpy( mCmdParam.Create.mPathName, "/YEAR2004/MONTH05" );    //Ŀ¼��,��Ŀ¼����YEAR2004��Ŀ¼��,YEAR2004Ŀ¼�������ȴ���
        ret = CreateDirectory( );                                     //�½����ߴ�Ŀ¼
        mStopIfError( ret );
        printf( "Close\r\n" );
        mCmdParam.Close.mUpdateLen = 0;                               //����Ŀ¼����Ҫ�Զ������ļ�����
        ret = CHRV3FileClose( );                                      //�ر�Ŀ¼,Ŀ¼����Ҫ�ر�,�ر�ֻ��Ϊ�˷�ֹ���������
        mStopIfError( ret );
    }
}


