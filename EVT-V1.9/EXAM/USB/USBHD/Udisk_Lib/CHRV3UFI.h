/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH32V103UFI.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2014/09/09
 * Description        : USB-flash File Interface for CHRV3.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 *******************************************************************************/
/* CHRV3 U�������ļ�ϵͳ�ӿ�, ֧��: FAT12/FAT16/FAT32 */

//#include "CHRV3BAS.H"

#ifndef __CHRV3UFI_H__
#define __CHRV3UFI_H__

#define CHRV3_LIB_VER                0x10

//#define DISK_BASE_BUF_LEN		512	/* Ĭ�ϵĴ������ݻ�������СΪ512�ֽ�(����ѡ��Ϊ2048����4096��֧��ĳЩ��������U��),Ϊ0���ֹ�ڱ��ļ��ж��建��������Ӧ�ó�����pDISK_BASE_BUF��ָ�� */
/* �����Ҫ���ô������ݻ������Խ�ԼRAM,��ô�ɽ�DISK_BASE_BUF_LEN����Ϊ0�Խ�ֹ�ڱ��ļ��ж��建����,����Ӧ�ó����ڵ���CHRV3LibInit֮ǰ��������������õĻ�������ʼ��ַ����pDISK_BASE_BUF���� */

//#define NO_DEFAULT_ACCESS_SECTOR	    1		/* ��ֹĬ�ϵĴ���������д�ӳ���,���������б�д�ĳ�������� */
//#define NO_DEFAULT_DISK_CONNECT		1		/* ��ֹĬ�ϵļ����������ӳ���,���������б�д�ĳ�������� */
//#define NO_DEFAULT_FILE_ENUMER		1		/* ��ֹĬ�ϵ��ļ���ö�ٻص�����,���������б�д�ĳ�������� */
#define FOR_ROOT_UDISK_ONLY           1
#ifdef __cplusplus
extern "C" {
#endif

/* ********************************************************************************************************************* */
/* FILE: CHRV3UF.H */

/* ������ */
#ifndef ERR_SUCCESS
#define ERR_SUCCESS				0x00	/* �����ɹ� */
#endif
#ifndef ERR_DISK_DISCON
#define ERR_CHRV3_ERROR			0x81	/* CHRV3Ӳ������,������Ҫ��λCHRV3 */
//#define ERR_DISK_DISCON			0x82	/* ������δ����,���ܴ����Ѿ��Ͽ� */
#define ERR_STATUS_ERR			0x83	/* ����״̬����,�����������ӻ��߶Ͽ����� */
#define ERR_HUB_PORT_FREE		0x84	/* USB-HUB�Ѿ����ӵ���HUB�˿���δ���Ӵ���,���ܴ����Ѿ��Ͽ� */
#define ERR_MBR_ERROR			0x91	/* ���̵���������¼��Ч,���ܴ�����δ����������δ��ʽ�� */
#define ERR_TYPE_ERROR			0x92	/* ���̷������Ͳ�֧��,ֻ֧��FAT12/FAT16/BigDOS/FAT32,��Ҫ�ɴ��̹��������·��� */
#define ERR_BPB_ERROR			0xA1	/* ������δ��ʽ��,���߲�������,��Ҫ��WINDOWS����Ĭ�ϲ������¸�ʽ�� */
#define ERR_TOO_LARGE			0xA2	/* ���̷�������ʽ��������������4GB,������������250GB,��Ҫ��WINDOWS����Ĭ�ϲ������¸�ʽ�� */
#define ERR_FAT_ERROR			0xA3	/* ���̵��ļ�ϵͳ��֧��,ֻ֧��FAT12/FAT16/FAT32,��Ҫ��WINDOWS����Ĭ�ϲ������¸�ʽ�� */
#define ERR_DISK_FULL			0xB1	/* �����ļ�̫��,ʣ��ռ�̫�ٻ����Ѿ�û��,��Ҫ�������� */
#define ERR_FDT_OVER			0xB2	/* Ŀ¼���ļ�̫��,û�п��е�Ŀ¼��,FAT12/FAT16��Ŀ¼�µ��ļ���Ӧ������500��,��Ҫ�������� */
#define ERR_MISS_DIR			0xB3	/* ָ��·����ĳ����Ŀ¼û���ҵ�,������Ŀ¼���ƴ��� */
#define ERR_FILE_CLOSE			0xB4	/* �ļ��Ѿ��ر�,�����Ҫʹ��,Ӧ�����´��ļ� */
#define ERR_OPEN_DIR			0x41	/* ָ��·����Ŀ¼���� */
#define ERR_MISS_FILE			0x42	/* ָ��·�����ļ�û���ҵ�,�������ļ����ƴ��� */
#define ERR_FOUND_NAME			0x43	/* ��������ͨ�����ƥ����ļ���,�ļ�����������·�������������,�����Ҫʹ��,Ӧ�ô򿪸��ļ� */
#endif
/* ����2XH-3XH����USB������ʽ��ͨѶʧ�ܴ���,��CHRV3�ӳ���ģ��CH375�ķ��� */
/* ����1XH����USB������ʽ�Ĳ���״̬����,��CHRV3�ӳ���ģ��CH375�ķ��� */
#ifndef ERR_USB_CONNECT
#define	ERR_USB_CONNECT_LS		0x13	/* ��⵽����USB�豸�����¼� */
#define	ERR_USB_CONNECT			0x15	/* ��⵽USB�豸�����¼�,�����Ѿ����� */
#define	ERR_USB_DISCON			0x16	/* ��⵽USB�豸�Ͽ��¼�,�����Ѿ��Ͽ� */
#define	ERR_USB_BUF_OVER		0x17	/* USB��������������������̫�໺������� */
#define	ERR_USB_DISK_ERR		0x1F	/* USB�洢������ʧ��,�ڳ�ʼ��ʱ������USB�洢����֧��,�ڶ�д�����п����Ǵ����𻵻����Ѿ��Ͽ� */
#define	ERR_USB_TRANSFER		0x20	/* NAK/STALL�ȸ����������0x20~0x2F */
#endif

/* ���̼��ļ�״̬ */
#define DISK_UNKNOWN			0x00	/* ��δ��ʼ��,δ֪״̬ */
#define DISK_DISCONNECT			0x01	/* ����û�����ӻ����Ѿ��Ͽ� */
#define DISK_CONNECT			0x02	/* �����Ѿ�����,������δ��ʼ�������޷�ʶ��ô��� */
#define DISK_USB_ADDR			0x04	/* �����Ѿ�����USB�豸��ַ,������δ����USB�ͳ�ʼ������ */
#define DISK_MOUNTED			0x05	/* �����Ѿ���ʼ���ɹ�,������δ�����ļ�ϵͳ�����ļ�ϵͳ��֧�� */
#define DISK_READY				0x10	/* �Ѿ��������̵��ļ�ϵͳ�����ܹ�֧�� */
#define DISK_OPEN_ROOT			0x12	/* �Ѿ��򿪸�Ŀ¼,����ģʽ,ֻ��������Ϊ��λ��дĿ¼������,ʹ�ú����ر�,ע��FAT12/FAT16��Ŀ¼�ǹ̶����� */
#define DISK_OPEN_DIR			0x13	/* �Ѿ�����Ŀ¼,����ģʽ,ֻ��������Ϊ��λ��дĿ¼������ */
#define DISK_OPEN_FILE			0x14	/* �Ѿ����ļ�,����ģʽ,����������Ϊ��λ�������ݶ�д */
#define DISK_OPEN_FILE_B		0x15	/* �Ѿ����ļ�,�ֽ�ģʽ,�������ֽ�Ϊ��λ�������ݶ�д */

/* FAT���ͱ�־ */
#ifndef DISK_FAT16
#define DISK_FS_UNKNOWN			0		/* δ֪���ļ�ϵͳ */
#define DISK_FAT12				1		/* FAT12�ļ�ϵͳ */
#define DISK_FAT16				2		/* FAT16�ļ�ϵͳ */
#define DISK_FAT32				3		/* FAT32�ļ�ϵͳ */
#endif

/* FAT���������ļ�Ŀ¼��Ϣ */
typedef struct _FAT_DIR_INFO
{
	UINT8	DIR_Name[11];				/* 00H,�ļ���,��11�ֽ�,���㴦��ո� */
	UINT8	DIR_Attr;					/* 0BH,�ļ�����,�ο������˵�� */
	UINT8	DIR_NTRes;					/* 0CH */
	UINT8	DIR_CrtTimeTenth;			/* 0DH,�ļ�������ʱ��,��0.1�뵥λ���� */
	UINT16	DIR_CrtTime;				/* 0EH,�ļ�������ʱ�� */
	UINT16	DIR_CrtDate;				/* 10H,�ļ����������� */
	UINT16	DIR_LstAccDate;				/* 12H,���һ�δ�ȡ���������� */
	UINT16	DIR_FstClusHI;				/* 14H */
	UINT16	DIR_WrtTime;				/* 16H,�ļ��޸�ʱ��,�ο���MAKE_FILE_TIME */
	UINT16	DIR_WrtDate;				/* 18H,�ļ��޸�����,�ο���MAKE_FILE_DATA */
	UINT16	DIR_FstClusLO;				/* 1AH */
	UINT32	DIR_FileSize;				/* 1CH,�ļ����� */
} FAT_DIR_INFO;							/* 20H */

typedef FAT_DIR_INFO *PX_FAT_DIR_INFO;

/* �ļ����� */
#define ATTR_READ_ONLY			0x01	/* �ļ�Ϊֻ������ */
#define ATTR_HIDDEN				0x02	/* �ļ�Ϊ�������� */
#define ATTR_SYSTEM				0x04	/* �ļ�Ϊϵͳ���� */
#define ATTR_VOLUME_ID			0x08	/* ��� */
#define ATTR_DIRECTORY			0x10	/* ��Ŀ¼ */
#define ATTR_ARCHIVE			0x20	/* �ļ�Ϊ�浵���� */
#define ATTR_LONG_NAME			( ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID )
/* �ļ����� UINT8 */
/* bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7 */
/*  ֻ   ��   ϵ   ��   Ŀ   ��   δ����   */
/*  ��   ��   ͳ   ��   ¼   ��            */
/* �ļ�ʱ�� UINT16 */
/* Time = (Hour<<11) + (Minute<<5) + (Second>>1) */
#define MAKE_FILE_TIME( h, m, s )	( (h<<11) + (m<<5) + (s>>1) )	/* ����ָ��ʱ������ļ�ʱ������ */
/* �ļ����� UINT16 */
/* Date = ((Year-1980)<<9) + (Month<<5) + Day */
#define MAKE_FILE_DATE( y, m, d )	( ((y-1980)<<9) + (m<<5) + d )	/* ����ָ�������յ��ļ��������� */

/* �ļ��� */
#define PATH_WILDCARD_CHAR		0x2A	/* ·������ͨ��� '*' */
#define PATH_SEPAR_CHAR1		0x5C	/* ·�����ķָ��� '\' */
#define PATH_SEPAR_CHAR2		0x2F	/* ·�����ķָ��� '/' */
#ifndef MAX_PATH_LEN
#define MAX_PATH_LEN			64		/* ���·������,������б�ָܷ�����С���������Լ�·��������00H */
#endif

/* �ⲿ������� */
typedef union _CMD_PARAM {
	struct {
		UINT8	mBuffer[ MAX_PATH_LEN ];
	} Other;
	struct {
		UINT32	mTotalSector;			/* ����: ��ǰ�߼��̵��������� */
		UINT32	mFreeSector;			/* ����: ��ǰ�߼��̵�ʣ�������� */
		UINT32	mSaveValue;
	} Query;							/* CMD_DiskQuery, ��ѯ������Ϣ */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* �������: ·��: [�̷�,ð��,б��,Ŀ¼�������ļ�������չ��...,������00H], �����̷���ð�ſ���ʡ��, ����"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Open;								/* CMD_FileOpen, ���ļ� */
//	struct {
//		UINT8	mPathName[ MAX_PATH_LEN ];	/* �������: ·��: [�̷�,ð��,б��,Ŀ¼�������ļ�������չ��(��ͨ���*)...,������00H], �����̷���ð�ſ���ʡ��, ����"C:\DIR1.EXT\DIR2\FILE*",00H */
//	} Open;								/* CMD_FileOpen, ö���ļ�, CHRV3vFileSize���λΪ1�������xFileNameEnumer,Ϊ0�򷵻�ָ����ŵ��ļ��� */
	struct {
		UINT8	mUpdateLen;				/* �������: �Ƿ�������³���: 0��ֹ,1���� */
	} Close;							/* CMD_FileClose, �رյ�ǰ�ļ� */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* �������: ·��: [�̷�,ð��,б��,Ŀ¼�������ļ�������չ��...,������00H], �����̷���ð�ſ���ʡ��, ����"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Create;							/* CMD_FileCreate, �½��ļ�����,����ļ��Ѿ���������ɾ�������½� */
	struct {
		UINT8	mPathName[ MAX_PATH_LEN ];	/* �������: ·��: [�̷�,ð��,б��,Ŀ¼�������ļ�������չ��...,������00H], �����̷���ð�ſ���ʡ��, ����"C:\DIR1.EXT\DIR2\FILENAME.EXT",00H */
	} Erase;							/* CMD_FileErase, ɾ���ļ����ر� */
	struct {
		UINT32	mFileSize;				/* �������: �µ��ļ�����,Ϊ0FFFFFFFFH���޸�, ����: ԭ���� */
		UINT16	mFileDate;				/* �������: �µ��ļ�����,Ϊ0FFFFH���޸�, ����: ԭ���� */
		UINT16	mFileTime;				/* �������: �µ��ļ�ʱ��,Ϊ0FFFFH���޸�, ����: ԭʱ�� */
		UINT8	mFileAttr;				/* �������: �µ��ļ�����,Ϊ0FFH���޸�, ����: ԭ���� */
	} Modify;							/* CMD_FileQuery, ��ѯ��ǰ�ļ�����Ϣ; CMD_FileModify, ��ѯ�����޸ĵ�ǰ�ļ�����Ϣ */
	struct {
		UINT32	mSaveCurrClus;
		UINT32	mSaveLastClus;
	} Alloc;							/* CMD_FileAlloc, �����ļ����ȵ���Ϊ�ļ�����Ĵ��̿ռ� */
	struct {
		UINT32	mSectorOffset;			/* �������: ����ƫ��,0���ƶ����ļ�ͷ,0FFFFFFFFH���ƶ����ļ�β, ����: ��ǰ�ļ�ָ���Ӧ�ľ�������������, 0FFFFFFFFH���ѵ��ļ�β */
		UINT32	mLastOffset;
	} Locate;							/* CMD_FileLocate, �ƶ���ǰ�ļ�ָ�� */
	struct {
		UINT8	mSectorCount;			/* �������: ��ȡ������, ����: ʵ�ʶ�ȡ������ */
		UINT8	mActCnt;
		UINT8	mLbaCount;
		UINT8	mRemainCnt;
		PUINT8	mDataBuffer;			/* �������: ��������ʼ��ַ, ����: ��������ǰ��ַ */
		UINT32	mLbaStart;
	} Read;								/* CMD_FileRead, �ӵ�ǰ�ļ���ȡ���� */
	struct {
		UINT8	mSectorCount;			/* �������: д��������, ����: ʵ��д�������� */
		UINT8	mActCnt;
		UINT8	mLbaCount;
		UINT8	mAllocCnt;
		PUINT8	mDataBuffer;			/* �������: ��������ʼ��ַ, ����: ��������ǰ��ַ */
		UINT32	mLbaStart;
		UINT32	mSaveValue;
	} Write;							/* CMD_FileWrite, ��ǰ�ļ�д������ */
	struct {
		UINT32	mDiskSizeSec;			/* ����: ����������̵���������, ���״ε���ʱ���� */
	} DiskReady;						/* CMD_DiskReady, ��ѯ���̾��� */
	struct {
		UINT32	mByteOffset;			/* �������: ���ֽ�Ϊ��λ��ƫ����, ���ֽ�Ϊ��λ���ļ�ָ��, ����: ��ǰ�ļ�ָ���Ӧ�ľ�������������, 0FFFFFFFFH���ѵ��ļ�β */
		UINT32	mLastOffset;
	} ByteLocate;						/* CMD_ByteLocate, ���ֽ�Ϊ��λ�ƶ���ǰ�ļ�ָ�� */
	struct {
		UINT16	mByteCount;				/* �������: ׼����ȡ���ֽ���, ����: ʵ�ʶ������ֽ��� */
		PUINT8	mByteBuffer;			/* �������: ָ���Ŷ������ݿ�Ļ����� */
		UINT16	mActCnt;
	} ByteRead;							/* CMD_ByteRead, ���ֽ�Ϊ��λ�ӵ�ǰ�ļ���ȡ���ݿ� */
	struct {
		UINT16	mByteCount;				/* �������: ׼��д����ֽ���, ����: ʵ��д����ֽ��� */
		PUINT8	mByteBuffer;			/* �������: ָ���Ŷ������ݿ�Ļ����� */
		UINT16	mActCnt;
	} ByteWrite;						/* CMD_ByteWrite, ���ֽ�Ϊ��λ��ǰ�ļ�д�����ݿ� */
	struct {
		UINT8	mSaveVariable;			/* �������: Ϊ0��ָ�����U�̵ı���,Ϊ0x80��ָ����U�̵ı���,����ֵ�򱸷�/������� */
		UINT8	mReserved[3];
		PUINT8	mBuffer;				/* �������: ָ���ӳ����ı����ı��ݻ�����,���Ȳ�С��80���ֽ� */
	} SaveVariable;						/* CMD_SaveVariable, ����/����/�ָ��ӳ����ı��� */
} CMD_PARAM;

typedef CMD_PARAM CMD_PARAM_I;
//typedef CMD_PARAM *P_CMD_PARAM;

/* SCSI������ */
#ifndef SPC_CMD_INQUIRY
#define SPC_CMD_INQUIRY			0x12
#define SPC_CMD_READ_CAPACITY	0x25
#define SPC_CMD_READ10			0x28
#define SPC_CMD_WRITE10			0x2A
#define SPC_CMD_TEST_READY		0x00
#define SPC_CMD_REQUEST_SENSE	0x03
#define SPC_CMD_MODESENSE6		0x1A
#define SPC_CMD_MODESENSE10		0x5A
#define SPC_CMD_START_STOP		0x1B
#endif

/* FILE: CHRV3UFI.C */

#define EN_DISK_WRITE			1

#ifndef DISK_BASE_BUF_LEN
#define DISK_BASE_BUF_LEN		512		/* Ĭ�ϵĴ������ݻ�������СΪ512�ֽ�,����ѡ��Ϊ2048����4096��֧��ĳЩ��������U��,Ϊ0���ֹ��.H�ļ��ж��建��������Ӧ�ó�����pDISK_BASE_BUF��ָ�� */
#endif

/* �ӳ�������ṩ�ı��� */
extern	UINT8V	CHRV3IntStatus;				/* CHRV3�������ж�״̬ */
extern	UINT8V	CHRV3DiskStatus;			/* ���̼��ļ�״̬ */
extern	UINT8	CHRV3vDiskFat;				/* �߼��̵�FAT��־:1=FAT12,2=FAT16,3=FAT32 */
extern	UINT8	CHRV3vSecPerClus;			/* �߼��̵�ÿ�������� */
extern	UINT8	CHRV3vSectorSizeB;			/* log2(CHRV3vSectorSize) */
extern	UINT32	CHRV3vStartLba;				/* �߼��̵���ʼ����������LBA */
extern	UINT32	CHRV3vDiskRoot;				/* ����FAT16��Ϊ��Ŀ¼ռ��������,����FAT32��Ϊ��Ŀ¼��ʼ�غ� */
extern	UINT32	CHRV3vDataStart;			/* �߼��̵������������ʼLBA */
extern	UINT32	CHRV3vStartCluster;			/* ��ǰ�ļ�����Ŀ¼����ʼ�غ� */
extern	UINT32	CHRV3vFileSize;				/* ��ǰ�ļ��ĳ��� */
extern	UINT32	CHRV3vCurrentOffset;		/* ��ǰ�ļ�ָ��,��ǰ��дλ�õ��ֽ�ƫ�� */
extern	UINT32	CHRV3vFdtLba;				/* ��ǰFDT���ڵ�LBA��ַ */
extern	UINT32	CHRV3vLbaCurrent;			/* ��ǰ��д�Ĵ�����ʼLBA��ַ */
extern	UINT16	CHRV3vFdtOffset;			/* ��ǰFDT�������ڵ�ƫ�Ƶ�ַ */
extern	UINT16	CHRV3vSectorSize;			/* ���̵�������С */
extern	UINT8	CHRV3vCurrentLun;			/* ���̵�ǰ�����߼���Ԫ�� */
extern	BOOL	CHRV3vSubClassIs6;			/* USB�洢���豸������Ϊ6,0���6 */
extern	PUINT8	pDISK_BASE_BUF;		/* ָ���ⲿRAM�Ĵ������ݻ�����,���������Ȳ�С��CHRV3vSectorSize,��Ӧ�ó����ʼ�� */
extern	PUINT8	pDISK_FAT_BUF;		/* ָ���ⲿRAM�Ĵ���FAT���ݻ�����,���������Ȳ�С��CHRV3vSectorSize,��Ӧ�ó����ʼ�� */
extern	UINT16	CHRV3vPacketSize;			/* USB�洢���豸����������:64@FS,512@HS/SS,��Ӧ�ó����ʼ�� */
extern	PUINT32	pTX_DMA_A_REG;				/* ָ����DMA��ַ�Ĵ���,��Ӧ�ó����ʼ�� */
extern	PUINT32	pRX_DMA_A_REG;				/* ָ�����DMA��ַ�Ĵ���,��Ӧ�ó����ʼ�� */
extern	PUINT16	pTX_LEN_REG;				/* ָ���ͳ��ȼĴ���,��Ӧ�ó����ʼ�� */
extern	PUINT16	pRX_LEN_REG;				/* ָ����ճ��ȼĴ���,��Ӧ�ó����ʼ�� */

extern	CMD_PARAM_I	mCmdParam;				/* ������� */

extern	__attribute__((aligned(4))) uint8_t RxBuffer[ MAX_PACKET_SIZE ];  // IN, must even address
extern	__attribute__((aligned(4))) uint8_t	TxBuffer[ MAX_PACKET_SIZE ];  // OUT, must even address


#ifndef	pSetupReq
#define	pSetupReq	((PUSB_SETUP_REQ)TxBuffer)
#endif
#ifndef	pCBW
#define	pCBW		((PXUDISK_BOC_CBW)TxBuffer)
#define	pCSW		((PXUDISK_BOC_CSW)RxBuffer)
#endif
#ifndef	pBOC_buf
#define	pBOC_buf	(TxBuffer+((USB_BO_CBW_SIZE+4)&0xFE))
#endif

#if		DISK_BASE_BUF_LEN > 0
extern	UINT8	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ];	/* �ⲿRAM�Ĵ������ݻ�����,����������Ϊһ�������ĳ��� */
#endif
extern	UINT8	CHRV3ReadSector( UINT8 SectCount, PUINT8 DataBuf );	/* �Ӵ��̶�ȡ������������ݵ��������� */
#ifdef	EN_DISK_WRITE
extern	UINT8	CHRV3WriteSector( UINT8 SectCount, PUINT8 DataBuf );	/* ���������еĶ�����������ݿ�д����� */
#endif

extern	UINT8	CHRV3DiskConnect( void );	/* �������Ƿ����Ӳ����´���״̬ */
extern	void	xFileNameEnumer( void );	/* �����ⲿ������ӳ���,�ļ���ö�ٻص��ӳ��� */

extern	UINT8	CHRV3LibInit( void );		/* ��ʼ��CHRV3�����,�����ɹ�����0 */

/* �ӳ�������ṩ���ӳ��� */
/* �����ӳ�����, �ļ������ӳ���CHRV3File*�ʹ��̲�ѯ�ӳ���CHRV3DiskQuery�����ܻ��õ��������ݻ�����pDISK_BASE_BUF,
   �����п�����pDISK_BASE_BUF�б����˴�����Ϣ, ���Ա��뱣֤pDISK_BASE_BUF��������������;,
   ���RAM����, Ҫ��pDISK_BASE_BUF��ʱ����������;, ��ô����ʱ�����������CHRV3DirtyBuffer������̻����� */
extern	UINT8	CHRV3GetVer( void );		/* ��ȡ��ǰ�ӳ����İ汾�� */
extern	void	CHRV3DirtyBuffer( void );	/* ������̻����� */
extern	UINT8	CHRV3BulkOnlyCmd( PUINT8 DataBuf );	/* ִ�л���BulkOnlyЭ������� */
extern	UINT8	CHRV3DiskReady( void );		/* ��ѯ�����Ƿ�׼���� */
extern	UINT8	CHRV3AnalyzeError( UINT8 iMode );	/* USB����ʧ�ܷ���CHRV3IntStatus���ش���״̬ */
extern	UINT8	CHRV3FileOpen( void );		/* ���ļ�����ö���ļ� */
extern	UINT8	CHRV3FileClose( void );		/* �رյ�ǰ�ļ� */
#ifdef	EN_DISK_WRITE
extern	UINT8	CHRV3FileErase( void );		/* ɾ���ļ����ر� */
extern	UINT8	CHRV3FileCreate( void );	/* �½��ļ�����,����ļ��Ѿ���������ɾ�������½� */
extern	UINT8	CHRV3FileAlloc( void );		/* �����ļ����ȵ���Ϊ�ļ�����Ĵ��̿ռ� */
#endif
extern	UINT8	CHRV3FileModify( void );	/* ��ѯ�����޸ĵ�ǰ�ļ�����Ϣ */
extern	UINT8	CHRV3FileQuery( void );		/* ��ѯ��ǰ�ļ�����Ϣ */
extern	UINT8	CHRV3FileLocate( void );	/* �ƶ���ǰ�ļ�ָ�� */
extern	UINT8	CHRV3FileRead( void );		/* �ӵ�ǰ�ļ���ȡ���ݵ�ָ�������� */
#ifdef	EN_DISK_WRITE
extern	UINT8	CHRV3FileWrite( void );		/* ��ǰ�ļ�д��ָ�������������� */
#endif
extern	UINT8	CHRV3ByteLocate( void );	/* ���ֽ�Ϊ��λ�ƶ���ǰ�ļ�ָ�� */
extern	UINT8	CHRV3ByteRead( void );		/* ���ֽ�Ϊ��λ�ӵ�ǰλ�ö�ȡ���ݿ� */
#ifdef	EN_DISK_WRITE
extern	UINT8	CHRV3ByteWrite( void );		/* ���ֽ�Ϊ��λ��ǰλ��д�����ݿ� */
#endif
extern	UINT8	CHRV3DiskQuery( void );		/* ��ѯ������Ϣ */
extern	void	CHRV3SaveVariable( void );	/* ����/����/�ָ��ӳ����ı���,�����ӳ�����ڶ��оƬ����U��֮������л� */

extern	void	mDelayuS( UINT16 n );		// ��uSΪ��λ��ʱ
extern	void	mDelaymS( UINT16 n );		// ��mSΪ��λ��ʱ
extern	UINT8	USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout );	// CHRV3��������,����Ŀ�Ķ˵��ַ/PID����,ͬ����־,NAK����ʱ��,����0�ɹ�,��ʱ/��������
extern	UINT8	HostCtrlTransfer( PUINT8 DataBuf, PUINT8 RetLen );	// ִ�п��ƴ���,8�ֽ���������pSetupReq��,DataBufΪ��ѡ���շ�������,ʵ���շ����ȷ�����ReqLenָ��ı�����
extern	void	CopySetupReqPkg( PCCHAR pReqPkt );  // ���ƿ��ƴ���������
extern	UINT8	CtrlGetDeviceDescrTB( void );  // ��ȡ�豸������,������TxBuffer��
extern	UINT8	CtrlGetConfigDescrTB( void );  // ��ȡ����������,������TxBuffer��
extern	UINT8	CtrlSetUsbAddress( UINT8 addr );  // ����USB�豸��ַ
extern	UINT8	CtrlSetUsbConfig( UINT8 cfg );  // ����USB�豸����
extern	UINT8	CtrlClearEndpStall( UINT8 endp );  // ����˵�STALL
#ifndef	FOR_ROOT_UDISK_ONLY
extern	UINT8	CtrlGetHubDescr( void );  // ��ȡHUB������,������TxBuffer��
extern	UINT8	HubGetPortStatus( UINT8 HubPortIndex );  // ��ѯHUB�˿�״̬,������TxBuffer��
extern	UINT8	HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt );  // ����HUB�˿�����
extern	UINT8	HubClearPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt );  // ���HUB�˿�����
#endif

#ifdef __cplusplus
}
#endif

#endif
