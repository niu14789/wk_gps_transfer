// gps_transferDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "gps_transfer.h"
#include "gps_transferDlg.h"

CString strFilePath;
static unsigned char openflag = 0;
unsigned char file_buffer[1024*1024*2];//2mB
static unsigned int cnt = 0;

static FILE * fp = NULL;
static FILE * fp_creater_rt = NULL;
//static FILE * fp_creater_timestamp = NULL;
//static FILE * fp_creater_bin = NULL;
static FILE * fp_creater_nav = NULL;
static FILE * fp_creater_txt = NULL;
/*-------------------------------*/
static FILE * fp_creater_pde = NULL;
static FILE * fp_creater_pde_txt = NULL;
static FILE * fp_creater_ephemeris_gps = NULL;
static FILE * fp_creater_almance_gps = NULL;
static FILE * fp_creater_ephemeris_glonass = NULL;
static FILE * fp_creater_almance_glonass = NULL;
static FILE * fp_creater_event_txt = NULL;
/*-------------------------------*/
static FILE * fp_creater_imu = NULL;
/*-------------------------------*/
static FILE * fp_nmea_bin = NULL;
static FILE * fp_nmea_txt = NULL;
/*-------------------------------*/
unsigned short 					EventNumber = 0;
unsigned char 					EventSource = 0, EventPort=0;
double 							EventTime = 0.0;
unsigned short 					Week_number;

static unsigned int GPS_TIME_LAST_PDE = 0;

/*-------------------------------*/
static unsigned long long last_times = 0;
static unsigned int reduce = 0;
/* static */
static unsigned char step = 0;

static unsigned char switch_one = 0;

static unsigned char raw_test = 0 , nav_test = 0 , icm20609_test = 0;

const unsigned char head[3][4]={{0xAA,0x55,0xC3,0x3C},{0xAA,0x55,0xCC,0x33},{0xAA,0x55,0xF0,0x0F}}; 

static unsigned char data_buffer_decode[1024];

unsigned long long timestamps;
unsigned int pps_num;

int com2_new_version;

typedef struct gps_nav_struct_s{
	unsigned long long system_timestamps;//size 8
	unsigned int system_pps_num;//size 4
	/* from ID 0x01 */
	unsigned int GPS_time_ms;//size 4
	unsigned short GPS_week_num;//size 2
	unsigned char svn;//size 1
	unsigned char position_flag1;//size 1
	unsigned char position_flag2;//size 1
	unsigned char init_num;//size 1
	unsigned short rev1;//size 2
	/* from ID 0x02 */
	double lat;//size 8
	double lon;//size 8
	double height;//size 8
	/* from ID 0x08 */
	unsigned char rev_a[7];//size 2
	unsigned char Velocity_flags;//size 1
	float Speed;//size 4
	float Heading;//size 4
	float Vertical_velocity;//size 4
	float Local_heading;//size 4
	/* from ID 0x09 */
	float PDOP;//size 4Heading
	float HDOP;//size 4
	float VDOP;//size 4
	float TDOP;//size 4
	unsigned char rev_b[8];//size 8
}gps_nav_def;

/* gps new version */
typedef struct{ // 28 byte
	unsigned int Position_type_information;
	unsigned int correction_age;
	unsigned int RTX_STD_SUB_Minutes_Left;
	unsigned int Pole_Wobble_Distance;
	unsigned short ITRF_Epoch;
	unsigned char output_record_type;
	unsigned char record_length;
	unsigned char solution_flags;
	unsigned char rtk_condition;
	unsigned char network_flags;
	unsigned char network_flags2;
	unsigned char frame_flag;
	unsigned char techtonic_plate;
	unsigned char Pole_Wobble_status_Flag;
	unsigned char rev;
}gps_new_version_def;
/* gps new version */


static gps_nav_def gps_nav;

typedef struct gps_psdu_doppler_s{
    unsigned long long system_timestamps;//size 8//ϵͳͬ��ʱ���?
	unsigned int system_pps_num;//size 4//��Ӧpps��
	/* from ID 0x01 */
	unsigned int GPS_receive_ms;//size 4//gps time
    /*  */
	double PSEUDORANGE;//8α��
    float doppler;//4������
	unsigned char num;//1
	unsigned char id;//1
	unsigned short week_number;//2
	double phase;//8
	double SNR;//8
}gps_psdu_doppler_def;

static gps_psdu_doppler_def gps_psdu_doppler;

typedef struct gps_ephemeris_data_s{
unsigned long long system_timestamps;//size 8
unsigned int system_pps_num;//size 4
unsigned short ephemeris_week;//2
unsigned short IODC;//2
unsigned char  PRN;//1
unsigned char  RESERVED1[2];//2
unsigned char  IODE;//1
unsigned int   TOW;//4
unsigned int   TOC;//4
unsigned int   TOE;//4
double TGD;//8
double AF2;//8
double AF1;//8
double AF0;//8
double CRS;//8
double DELTAN;//8
double MSUB0;//8
double CUC;//8
double ECCENTRICITY;//8
double CUS;//8
double SQRTA;//8
double CIC;//8
double OMEGASUB0;//8
double CIS;//8
double ISUB0;//8
double CRC;//8
double OMEGA;//8
double OMEGADOT;//8
double IDOT;//8
unsigned int FLAGS;//4
unsigned char  RESERVED2[4];//4
}gps_ephemeris_data_def;

static gps_ephemeris_data_def gps_ephemeris_data;

typedef struct gps_Almanac_data_s{
unsigned long long system_timestamps;//size 8
unsigned int system_pps_num;//size 4
/* start */
unsigned int ALM_DECODE_TIME;//4
unsigned short AWN;//2
unsigned char ALM_HEALTH;//1
unsigned char PRN;//1
unsigned int TOA;//4
double SQRTA;//8
double ECCENT;//8
double ISUBO;//8
double OMEGADOT;//8
double OMEGSUBO;//8
double OMEGA;//8
double MSUBO;//8
}gps_Almanac_data_def;

static gps_Almanac_data_def gps_Almanac_data;
/* subtype 09 */
typedef struct glonass_ephemeris_data_s{
unsigned long long system_timestamps;//size 8
unsigned int system_pps_num;//size 4
unsigned short GPS_WEEK_EPH_VALID_REF_TIME;//2
unsigned short GPS_WEEK_EPH_DECODE_REF_TIME;//2
unsigned int GPS_TIME_EPH_VALID_REF_TIME;//4
unsigned int GPS_TIME_EPH_DECODE_REF_TIME;//4
unsigned short GLONASS_DAY_NUMBER;//2
unsigned char  REF_TIME_OF_EPHEMERIS;//1
unsigned char  LEAP_SECONDS;//1
unsigned int   FRAME_START_TIME;//4
unsigned char  FLAGS;//1
unsigned char  ARG_OF_DATA;//1
unsigned char  EPHEMERIS_SOURCE;//1
unsigned char  FDMA;//1
unsigned char  HEALTH;//1
unsigned char  GENERATION;//1
unsigned char  UDRE;//1
unsigned char PRN;//1
double X;//8
double X_VELOCITY;//8
double X_ACCELERATION;//8
double Y;//8
double Y_VELOCITY;//8
double Y_ACCELERATION;//8
double Z;//8
double Z_VELOCITY;//8
double Z_ACCELERATION;//8
double A0_UTC;//8
double A0;//8
double A1;//8
double TAU_GPS;//8
double DELTA_TAU_N;//8
}glonass_ephemeris_data_def;

static glonass_ephemeris_data_def glonass_ephemeris_data;

typedef struct glonass_almance_data_s{
	unsigned long long system_timestamps;//size 8
	unsigned int system_pps_num;//size 4
    unsigned short DAY_NUMBER;//2
    unsigned char FDMA_NUMBER;//1
    unsigned char HEALTH;//1
    unsigned char rev[7];//7
    unsigned char PRN;//1
    double ECCENTRICITY;//8
    double ARG_OF_PERIGEE;//8
    double ORBIT_PERIOD;//8
    double ORBITAL_PREIOD_CORRECTION;//8
    double LONG_FIRST_ASCENDING_NODE;//8
    double TIME_ASCENDING_NODE;//8
    double INCLINATION;//8
    double A0;//8
}glonass_almance_data_def ;

static glonass_almance_data_def glonass_almance_data;

typedef struct { // 55_21----176
unsigned short EPHEMERIS_WEEK;
unsigned short IODC;
unsigned char RESERVED;
unsigned char IODE;
unsigned char res1[2];
unsigned int TOW;
unsigned int TOC;
unsigned int TOE;
float flags;
double TGD;
double AF2;
double AF1;
double AF0;
double CRS;
double DELTAN;
double M_SUB_0;
double CUC;
double ECCENTRICITY;
double CUS;
double SQRTA;
double CIC;
double OMEGA_SUB_0;
double CIS;
double I_SUB_0;
double CRC;
double OMEGA;
double OMEGA_DOT;
double I_DOT;
}beidou_ephemeris_def;

typedef struct{
unsigned int ALMDECODE_TIME;
unsigned short AWN;
unsigned int TOA;
double SQRTA;
double ECCENT;
double ISUBO;
double OMEGADOT;
double OMEGSUBO;
double OMEGA;
double MSUBO;
unsigned short ALM_HEALTH;
double ASUBF0;
double ASUBF1;
unsigned char ALM_SRC_FOR_BeiDou;
}beidou_almanac_def;

//const unsigned char test_code[] = {
//0x01,0x11,0x07,0xA1,0x00,0x58,0x00,0x58,0x00,0x04,0x65,0x12,0x00,0x04,0x81,0x20,0x00,0x04,0x81,0x20,0xBE,0x48,0x00,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3D,0x81,0x00,0x00,0x00,0x00,0x00,0x00,0xBF,0x26,0xE8,0x68,0x00,0x00,
//0x00,0x00,0xC0,0x3D,0x08,0x00,0x00,0x00,0x00,0x00,0x3E,0x16,0x32,0x00,0x00,0x00,0x00,0x00,0xBF,0xE5,0xD5,0x0F,0x0F,0x80,0x00,0x00,0xBE,
//0x9E,0x66,0x0A,0x43,0xD3,0x19,0x33,0x3F,0x87,0xD1,0xE7,0x94,0x00,0x00,0x00,0x3E,0xAE,0x23,0xD4,0xE6,0x6E,0x49,0x68,0x40,0xB4,0x21,0xA4,
//0xA1,0x00,0x00,0x00,0x3E,0x72,0x76,0x43,0xE3,0x7E,0xDD,0xB7,0x3F,0xE9,0x92,0xE3,0x2C,0x80,0x00,0x00,0xBE,0x53,0xBC,0x36,0xEA,0x5B,0x7A,
//0x3F,0x3F,0xD3,0xF8,0x0E,0xA2,0x00,0x00,0x00,0x40,0x75,0x02,0x80,0x00,0x00,0x00,0x00,0xBF,0xE2,0xD3,0x93,0xD0,0xC0,0x00,0x00,0xBE,0x25,
//0x8A,0xC0,0x00,0x00,0x00,0x00,0xBD,0xA2,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x0A,0x48,0x03
//};
//unsigned char almance_code[] = {
//0x02,0x28,0x55,0x45,0x02,0x05,0x00,0x03,0xDE,0xD2,0x07,0xA1,0x00,0x07,0xB0,0x00,0x40,0xB4,0x21,0xB6,0xE0,0x00,0x00,
//0x00,0x3F,0x74,0x7A,0x00,0x00,0x00,0x00,0x00,0x3F,0xD3,0x48,0x3B,0x49,0x9F,0x2B,0xA3,0xBE,0x25,0xB0,0x00,0x0D,0x3A,
//0x8E,0x47,0xBF,0xE1,0xC3,0xD5,0x11,0x32,0x10,0xA8,0x3F,0xC5,0xFE,0x0C,0xFF,0xF6,0x2F,0xEE,0xBF,0xE7,0x96,0xB2,0x56,
//0xF7,0xD7,0xDD,0x00,0xEF,0x03};
//unsigned char glonass_ephemeris[] = {
//0x02,0x28,0x55,0x8D,0x09,0x08,0x07,0xA1,0x00,0x04,0x7D,0xAE,0x07,0xA1,0x00,0x04,0x7D,0x54,0x02,0x13,
//0x33,0x12,0x76,0x00,0x00,0xB2,0xF2,0x00,0x00,0x06,0x00,0x01,0x03,0xC1,0x58,0x75,0x4F,0x82,0x00,0x00,
//0x00,0x40,0xA1,0x4E,0x59,0x6F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC1,0x61,0x61,
//0xF3,0xCE,0x20,0x00,0x00,0xC0,0xA1,0x4F,0xAD,0x47,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
//0x00,0x41,0x75,0xF0,0x00,0x78,0xE0,0x00,0x00,0xC0,0x70,0x4E,0x12,0x80,0x00,0x00,0x00,0xBE,0xC7,0x70,
//0x00,0x00,0x00,0x00,0x00,0xBE,0x2C,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0xFC,0x13,0x80,0x00,0x00,0x00,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xBE,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0xBE,0x30,0x00,
//0x00,0x00,0x00,0x00,0x00,0xC1,0x03};
//unsigned char glonass_almance_code[] = {
//0x02,0x28,0x55,0x46,0x08,0x01,0x02,0x13,0x01,0x3F,0x40,0x20,0x00,0x00,0x00,0x00,0x00,0xBF,0xE7,0x3B,0xD1,
//0xFE,0x9E,0x26,0x50,0x40,0xE3,0xCB,0xF8,0x10,0x00,0x00,0x00,0xBF,0x4E,0x00,0x00,0x00,0x00,0x00,0x00,0x40,
//0x06,0x44,0x4D,0xA4,0x3E,0x33,0x23,0x40,0xE3,0x7F,0x80,0x00,0x00,0x00,0x00,0x3F,0xF1,0xEA,0x70,0x28,0x2D,
//0x10,0x78,0xBE,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x03};

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ����Ӧ�ó��򡰹��ڡ��˵����?CAboutDlg �Ի���

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// Cgps_transferDlg �Ի���




Cgps_transferDlg::Cgps_transferDlg(CWnd* pParent /*=NULL*/)
	: CDialog(Cgps_transferDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void Cgps_transferDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_switch);
	DDX_Control(pDX, IDC_CHECK1, m_check_raw_enable);
	DDX_Control(pDX, IDC_CHECK2, m_check_nav_enable);
	DDX_Control(pDX, IDC_CHECK3, m_20609_enable);
	DDX_Control(pDX, IDC_CHECK4, m_rt27);
	DDX_Control(pDX, IDC_CHECK_NMEA, m_check_nmea);
	DDX_Control(pDX, IDC_CHECK5, m_new_version);
}

BEGIN_MESSAGE_MAP(Cgps_transferDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON1, &Cgps_transferDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &Cgps_transferDlg::OnBnClickedButton2)
END_MESSAGE_MAP()


// Cgps_transferDlg ��Ϣ�������?

BOOL Cgps_transferDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�?

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	// TODO: �ڴ���Ӷ���ĳ�ʼ������

	m_switch.SetCurSel(0);
		
	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

void Cgps_transferDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void Cgps_transferDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ��������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù����ʾ��?
//
HCURSOR Cgps_transferDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void Cgps_transferDlg::OnBnClickedButton1()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������?

   // TODO: Add your control notification handler code here   
    // ���ù�����   
    TCHAR szFilter[] = _T("all_people(*.*)|*.*||");   
    // ������ļ��Ի���?  
    CFileDialog fileDlg(TRUE, _T("open file"), NULL, 0, szFilter, this);   
   
  
    // ��ʾ���ļ��Ի���   
    if (IDOK == fileDlg.DoModal())   
    {   
        // ���������ļ��Ի����ϵġ��򿪡���ť����ѡ����ļ�·����ʾ���༭����?  
        strFilePath = fileDlg.GetPathName();   
        SetDlgItemText(IDC_EDIT1, strFilePath);
		openflag = 1;
    }
}

void Cgps_transferDlg::OnBnClickedButton2()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������?

	   static char * file_name;
	   unsigned int len;
	   char creater_buffer[200];


	   com2_new_version = m_new_version.GetCheck();

	   switch_one = m_switch.GetCurSel();

	   raw_test = m_check_raw_enable.GetCheck();
       nav_test = m_check_nav_enable.GetCheck();
	   icm20609_test = m_20609_enable.GetCheck();

	   USES_CONVERSION;

	   file_name = T2A(strFilePath);

	   fp = fopen(file_name,"rb");

       if(fp == NULL)
	   {
		   MessageBox(_T("open file error"),_T("tips"),0);
		   return;
	   }

	   char name_buffer[100][1024];

	   memset(name_buffer,0,sizeof(name_buffer));

	   int len2 = sscanf(file_name,"%s.%s",name_buffer[0],name_buffer[1]);

	   file_name = name_buffer[0];

	   if( switch_one == 0 )
	   {
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s_rt27.rt27",file_name);

		   fp_creater_rt = fopen(creater_buffer,"wb+");

		   if(fp_creater_rt == NULL)
		   {
			   MessageBox(_T("creater rt file error"),_T("tips"),0);
			   return;
		   }
           //fp_creater_pde
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.pse",file_name); 

		   fp_creater_pde = fopen(creater_buffer,"wb+");

		   if(fp_creater_pde == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_pde error"),_T("tips"),0);
			   return;
		   }
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.txt",file_name); 

		   fp_creater_event_txt = fopen(creater_buffer,"wb+");

		   if(fp_creater_event_txt == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_pde error"),_T("tips"),0);
			   return;
		   }

		   if(raw_test)
		   {
			   /* TEST */
			   memset(creater_buffer,0,sizeof(creater_buffer));

			   sprintf(creater_buffer,"%s.txt",file_name);

			   fp_creater_pde_txt = fopen(creater_buffer,"wb+");

			   if(fp_creater_pde_txt == NULL)
			   {
				   MessageBox(_T("creater rt fp_creater_pde error"),_T("tips"),0);
				   return;
			   }
		   }
           /*--fp_creater_pde_txt--- fp_creater_ephemeris_gps */
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.eph",file_name);

		   fp_creater_ephemeris_gps = fopen(creater_buffer,"wb+");

		   if(fp_creater_ephemeris_gps == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_pde error"),_T("tips"),0);
			   return;
		   }
		   /*fp_creater_almance_gps*/
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.alm",file_name);

		   fp_creater_almance_gps = fopen(creater_buffer,"wb+");

		   if(fp_creater_almance_gps == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_pde error"),_T("tips"),0);
			   return;
		   }
		   /* fp_creater_ephemeris_glonass */
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.eme",file_name);

		   fp_creater_ephemeris_glonass = fopen(creater_buffer,"wb+");

		   if(fp_creater_ephemeris_glonass == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_ephemeris_glonass error"),_T("tips"),0);
			   return;
		   }
		   /* fp_creater_almance_glonass */
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.galm",file_name);

		   fp_creater_almance_glonass = fopen(creater_buffer,"wb+");

		   if(fp_creater_almance_glonass == NULL)
		   {
			   MessageBox(_T("creater rt fp_creater_ephemeris_glonass error"),_T("tips"),0);
			   return;
		   }

	   }else  if( switch_one == 1 )
	   {
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.nraw",file_name);

		   fp_creater_rt = fopen(creater_buffer,"wb+");

		   if(fp_creater_rt == NULL)
		   {
			   MessageBox(_T("creater rt file error"),_T("tips"),0);
			   return;
		   }
	   }else if( switch_one == 2 )/*imu test */
	   {
           memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s.txt",file_name);

		   fp_creater_imu = fopen(creater_buffer,"wb+");

		   if( fp_creater_imu == NULL)
		   {
			   MessageBox(_T("creater txt file error"),_T("tips"),0);
			   return;
		   }
	   }
     /*  memset(creater_buffer,0,sizeof(creater_buffer));

	   sprintf(creater_buffer,"%s_times.txt",file_name);

	   fp_creater_timestamp = fopen(creater_buffer,"wb+");

       if(fp_creater_timestamp == NULL)
	   {
		   MessageBox(_T("creater rt file error"),_T("tips"),0);
		   return;
	   }

	   memset(creater_buffer,0,sizeof(creater_buffer));

	   sprintf(creater_buffer,"%s_times.bin",file_name);

	   fp_creater_bin = fopen(creater_buffer,"wb+");

       if(fp_creater_bin == NULL)
	   {
		   MessageBox(_T("creater rt bin file error"),_T("tips"),0);
		   return;
	   }*/
/*  if creater file_name switch_one */
	   if( switch_one == 1 )
	   {
		   memset(creater_buffer,0,sizeof(creater_buffer));

		   sprintf(creater_buffer,"%s_nav.bin",file_name);

		   fp_creater_nav = fopen(creater_buffer,"wb+");

		   if(fp_creater_nav == NULL)
		   {
			   MessageBox(_T("creater rt bin file error"),_T("tips"),0);
			   return;
		   }
		   if(nav_test)
		   {
			   memset(creater_buffer,0,sizeof(creater_buffer));

			   sprintf(creater_buffer,"%s_nav.txt",file_name);

			   fp_creater_txt = fopen(creater_buffer,"wb+");

			   if(fp_creater_txt == NULL)
			   {
				   MessageBox(_T("creater rt bin file error"),_T("tips"),0);
				   return;
			   }
		   }
	   }
/* nav end */ 
       cnt  = 0;
	   while(1)
	   {
			/* read */
			len = fread(file_buffer,1,sizeof(file_buffer),fp);
			if( m_rt27.GetCheck() && switch_one == 0 )
			{
               	/* decode the gps raw and nav data  */   
				raw_gps_parse(file_buffer,len);
			}else if( m_check_nmea.GetCheck() && switch_one == 0 )
			{
			   memset(creater_buffer,0,sizeof(creater_buffer));

			   sprintf(creater_buffer,"%s_nmea.txt",file_name);

			   fp_nmea_txt = fopen(creater_buffer,"wb+");

			   if(fp_nmea_txt == NULL)
			   {
				   MessageBox(_T("creater fp_nmea_txt file error"),_T("tips"),0);
				   return;
			   }

			   memset(creater_buffer,0,sizeof(creater_buffer));

			   sprintf(creater_buffer,"%s_nmea.bin",file_name);

			   fp_nmea_bin = fopen(creater_buffer,"wb+");

			   if(fp_nmea_bin == NULL)
			   {
				   MessageBox(_T("creater fp_nmea_bin file error"),_T("tips"),0);
				   return;
			   }
			   nmea_decode((char *)file_buffer,len);
			}
			else if( com2_new_version && switch_one == 0 )
			{               	
				/* decode the gps raw and nav data  */   
				raw_gps_parse(file_buffer,len);
                
			}else
			{
                data_parse(file_buffer,len);
			}
			

			if(len < sizeof(file_buffer))
			{
				MessageBox(_T("ok"),_T("tips"),0);
				fclose(fp);
				fclose(fp_creater_event_txt);
				if( switch_one == 1 )
				{
					fclose(fp_creater_nav);
					fclose(fp_creater_rt);
					if(nav_test)
					{
						fclose(fp_creater_txt);
					}
				}else if( switch_one == 0)
				{
					fclose(fp_creater_pde);
					if(raw_test)
					{
						fclose(fp_creater_pde_txt);
					}
					fclose(fp_creater_ephemeris_gps);
					fclose(fp_creater_almance_gps);
					fclose(fp_creater_ephemeris_glonass);
					fclose(fp_creater_almance_glonass);
					fclose(fp_creater_rt);
				}else if(switch_one == 2)
				{
					fclose(fp_creater_imu);	
				}else
				{

				}

				if( m_check_nmea.GetCheck())
				{
				  fclose(fp_nmea_bin);
				  fclose(fp_nmea_txt);
				}
				break;
			}
	   }
}

void Cgps_transferDlg::data_parse(unsigned char *buffer,unsigned int len)
{
	static unsigned int i;
	for(i=0;i<len;i++)
	{
       data_step(buffer[i]);
	}
}

void Cgps_transferDlg::data_step(unsigned char d)
{
	static unsigned short len,data_len = 0;
	static unsigned char data_buffer[4096];

	unsigned short sum = 0;

	unsigned char da_save[4096];

	switch(step)
	{
		case 0:
			if( d == head[switch_one][0] )
			{
				step = 1;
			}else
			{
				step = 0;
			}
			break;
		case 1:
			if( d == head[switch_one][1] )
			{
				step = 2;
			}else
			{
				step = 0;
			}
			break;
		case 2:
			if( d == head[switch_one][2] )
			{
				step = 3;
			}else
			{
				step = 0;
			}
			break;
		case 3:
			if( d == head[switch_one][3] )
			{
				step = 4;
			}else
			{
				step = 0;
			}
			break;
		case 4:
            len = d<<8;
			step = 5;
			break;
		case 5:
            len |= d;
			step = 6;
			data_len = 0;
			break;
		case 6:
            data_buffer[data_len] = d;
			data_len++;
			if( data_len >= ( len - 6 ) )
			{
				for(int i=0;i< len - 8 ;i++)
				{
					sum += (unsigned short)data_buffer[i];
				}

				sum += 0xAA;
				sum += 0x55;
				sum += 0xC3;
				sum += 0x3C;
				sum += (len >> 8)&0xff;
				sum += len & 0xff;
	            
				if((sum >> 8) == data_buffer[data_len - 2] && (sum & 0xff ) == data_buffer[data_len - 1])
				{
					if( switch_one == 0 | switch_one == 1 )
					{
						memcpy(da_save,data_buffer+12,data_len-14);
						fwrite(da_save,1,data_len - 14,fp_creater_rt);

						timestamps = 0;
						pps_num = 0;
						timestamps |= data_buffer[0] << 56;
						timestamps |= data_buffer[1] << 48;
						timestamps |= data_buffer[2] << 40;
						timestamps |= data_buffer[3] << 32;
						timestamps |= data_buffer[4] << 24;
						timestamps |= data_buffer[5] << 16;
						timestamps |= data_buffer[6] << 8;
						timestamps |= data_buffer[7] << 0;
						/* pps num */
						pps_num |= data_buffer[8] << 24;
						pps_num |= data_buffer[9] << 16;
						pps_num |= data_buffer[10] << 8;
						pps_num |= data_buffer[11] << 0;

						if(timestamps == 0x934EFE0)
						{
							printf("test");
						}
						/* decode the gps raw and nav data  */   
						raw_gps_parse(da_save,data_len - 14);
					}else
					{
						if(icm20609_test)//20609 for test
						{
							icm_20609_handle(data_buffer,data_len - 14);
						}else
						{
							/* for stim300 */	   
						}
					}
					cnt++;
				}else
				{
                  step = 0;
				}
				 step = 0;
			}
           
			break;
		default:
			step = 0;
			break;
	}
}


void Cgps_transferDlg::raw_gps_parse(unsigned char *buffer,unsigned int len)
{
   	static unsigned int i;
	for(i=0;i<len;i++)
	{
       raw_gps_decode(buffer[i]);
	}
}

void Cgps_transferDlg::raw_gps_decode(unsigned char d)
{
	static unsigned char step;
	static unsigned char len,data_len = 0;
	static unsigned char status = 0;
	static unsigned char type=0;
	static unsigned int cnt = 0;
	long long timestamps;

	unsigned char sum = 0;

	unsigned char da_save[1024];

    switch(step)
	{
		case 0:
			if( d == 0x02 )
			{
				step = 1;
			}else
			{
				step = 0;
			}
			break;
		case 1:
			if( d == 0x28 || d == 0x68 || d == 0xE8 || 0xA8 )
			{
				step = 2;
				status = d;
			}else
			{
				step = 0;
			}
			break;
		case 2:
			if( d == 0x55 || d == 0x57 ||  d == 0x40 )
			{
				step = 3;
			}else
			{
				step = 0;
			}
            type = d;
			break;
        case 3:
			len = d;
			step = 4;
			cnt = 0;
			break;
		case 4:
            data_buffer_decode[cnt] = d;
			cnt++;
			if(cnt >= len + 2 )
			{ 
               for(int i=0;i<len;i++)
			   {
                  sum += data_buffer_decode[i];
			   }
			   sum += status;
               sum += type;
			   sum += len;

               if(sum == data_buffer_decode[len] && 0x3 == data_buffer_decode[len+1])
			   {
				   /* check ok */
				   gps_raw_detail(type,data_buffer_decode,len);
			   }else
			   {
				//   gps_raw_detail(type,data_buffer_decode,len);
                   step = 0;
			   }
			   step = 0;
			   status = 0;
			}
			break;
	}
}



void Cgps_transferDlg::gps_raw_detail( unsigned char type,unsigned char *data,unsigned char len )
{
	static unsigned char svn = 0;
	static double lat,lon,height;
	unsigned char len_package = 0;
	unsigned char len_package_single;
	unsigned char *nav_data;
	static char buffer[2048];
	static char buffer_tmp[8];
	static unsigned int len_write;
	static unsigned int sizeof_test;
    
	static unsigned char buffer_page[20*256];//this buffer max supplys 20 pages.
	static unsigned int buffer_len_page = 0;

	static unsigned char total_page = 0;
	static unsigned char current_page = 0;

	static unsigned char step = 0;

	static unsigned char page_cnt = 0;
    unsigned int len_gps_ephemeris;
	unsigned int len_gps_almance;
	unsigned int len_glonass_ephemeris;
	unsigned int len_glonass_almance;
	//static unsigned char page_buffer[1024 * 1024];
 //   static unsigned int page_buffer_cnt = 0;
	char event_buffer[200];

  switch(type)
  {
	  case 0x57:
        
		  switch(data[0])//raw survey data
		  {
		  case 0:
			  svn = data[21]; 
			  break;
		  case 1: 
			  break;
		  case 2:
				EventSource = data[4];
				EventPort = data[5];
				big2little(data + 6, (char *)&EventNumber, 2);
				big2little(data + 8, (char *)&EventTime, 8);
				/*------------------------------------*/
				memset(event_buffer,0,sizeof(event_buffer));
				sprintf(event_buffer,"%d %lldd\r\n",EventNumber,EventTime);
				/*------------------------------------*/
				fwrite(event_buffer,1,strlen(event_buffer),fp_creater_event_txt);
				/*------------------------------------*/
			  break;
		  case 6:

			  //page_buffer[page_buffer_cnt++] = data[1];

			  if( (( data[1] & 0xf0 ) >> 4) == ( data[1] & 0x0f ) && ( data[1] & 0x0f ) == 0x1 ) // this command only has one page 
			  {
                 real_time_gnss_survey_data(data,len);
			  }else
			  { 
				  switch(step)
				  {
				  case 0:
					 if( (( data[1] & 0xf0 ) >> 4 ) == 0x1 ) //first page
					 {
						 buffer_len_page = len;
						 memcpy(buffer_page,data,buffer_len_page);

						 total_page = data[1] & 0xf;//total such as 2 3 6 8

						 current_page = ( data[1] & 0xf0 ) >> 4;//1
						 page_cnt = 0;
						 step = 1;
					 }
					 break;
				  case 1:
					 if( (( data[1] & 0xf0 ) >> 4 ) == ( data[1] & 0x0f ) )
					 {
						/* ok */
						 if( total_page == ( data[1] & 0x0f ) && total_page == (page_cnt + 2) )
						 { 
							 memcpy(buffer_page + buffer_len_page ,data + 4 ,len - 4 );
							 buffer_len_page += len - 4;
							 real_time_gnss_survey_data(buffer_page,buffer_len_page);
							 buffer_len_page = 0;
						 }
						 page_cnt = 0;
						 step = 0;
					 }else if( (( data[1] & 0xf0 ) >> 4) < ( data[1] & 0x0f )  )
					 {
						 if( total_page ==  ( data[1] & 0x0f ) && (current_page + 1) == (( data[1] & 0xf0 ) >> 4) )
						 {
							/* normal page */
							 memcpy(buffer_page + buffer_len_page ,data + 4 ,len - 4 );
							 buffer_len_page += len - 4;
							 current_page = ( data[1] & 0xf0 ) >> 4;//1
							 page_cnt++;

						 }else
						 {
							 step = 0;
							 buffer_len_page = 0;
							 page_cnt = 0;
						 } 
					 }else
					 {
					   /* bad data */
						printf("this is a bad data\n");
						step = 0;
						buffer_len_page = 0;
						page_cnt = 0; 
					 }
					 break;
				  default :
					  break;
				  }
			  }
			  
			  break;
		  default:
			  break;
		  }
		  break;
	  case 0x55:

          switch(data[0])
		  {
			  case 0:
				  printf("1");
				  break;
			  case 1:

				  gps_ephemeris_data.system_timestamps = timestamps;
				  gps_ephemeris_data.system_pps_num = pps_num; 

				  gps_ephemeris_data.PRN = data[1];
 
				  big2little(data+2,(char *)&gps_ephemeris_data.ephemeris_week,2); //2
				  big2little(data+4,(char *)&gps_ephemeris_data.IODC,2); //2

				  gps_ephemeris_data.IODE = data[7];

				  big2little(data + 8,(char *)&gps_ephemeris_data.TOW,4);   // 4
				  big2little(data + 12,(char *)&gps_ephemeris_data.TOC,4);  // 4
				  big2little(data + 16 ,(char *)&gps_ephemeris_data.TOE,4); // 4

				  big2little(data + 20 ,(char *)&gps_ephemeris_data.TGD, 8 ); // 8 
				  big2little(data + 28 ,(char *)&gps_ephemeris_data.AF2, 8 ); // 8
                  big2little(data + 36 ,(char *)&gps_ephemeris_data.AF1, 8 ); // 8
				  big2little(data + 44 ,(char *)&gps_ephemeris_data.AF0, 8 ); // 8
				  big2little(data + 52 ,(char *)&gps_ephemeris_data.CRS, 8 ); // 8
				  big2little(data + 60 ,(char *)&gps_ephemeris_data.DELTAN, 8 ); // 8
				  big2little(data + 68 ,(char *)&gps_ephemeris_data.MSUB0, 8 ); // 8
				  big2little(data + 76 ,(char *)&gps_ephemeris_data.CUC, 8 ); // 8
				  big2little(data + 84 ,(char *)&gps_ephemeris_data.ECCENTRICITY, 8 ); // 8
				  big2little(data + 92 ,(char *)&gps_ephemeris_data.CUS, 8 ); // 8
				  big2little(data + 100 ,(char *)&gps_ephemeris_data.SQRTA, 8 ); // 8
				  big2little(data + 108 ,(char *)&gps_ephemeris_data.CIC, 8 ); // 8
				  big2little(data + 116 ,(char *)&gps_ephemeris_data.OMEGASUB0, 8 ); // 8
				  big2little(data + 124 ,(char *)&gps_ephemeris_data.CIS, 8 ); // 8
				  big2little(data + 132 ,(char *)&gps_ephemeris_data.ISUB0, 8 ); // 8
				  big2little(data + 140 ,(char *)&gps_ephemeris_data.CRC, 8 ); // 8
				  big2little(data + 148 ,(char *)&gps_ephemeris_data.OMEGA, 8 ); // 8
				  big2little(data + 156 ,(char *)&gps_ephemeris_data.OMEGADOT, 8 ); // 8
                  big2little(data + 164 ,(char *)&gps_ephemeris_data.IDOT, 8 ); // 8

				  big2little(data + 172 ,(char *)&gps_ephemeris_data.FLAGS,4); // 4

                  len_gps_ephemeris = sizeof(gps_ephemeris_data);

				  fwrite(&gps_ephemeris_data,1,len_gps_ephemeris,fp_creater_ephemeris_gps);

				  break;
			  case 2:

				  gps_Almanac_data.system_timestamps = timestamps;
				  gps_Almanac_data.system_pps_num = pps_num;

				  gps_Almanac_data.PRN = data[1];

				  big2little(data + 2 ,(char *)&gps_Almanac_data.ALM_DECODE_TIME, 4 ); // 4
				  big2little(data + 6 ,(char *)&gps_Almanac_data.AWN, 2 ); // 2
				  big2little(data + 8 ,(char *)&gps_Almanac_data.TOA, 4 ); // 4
				  /* double */
				  big2little(data + 12 ,(char *)&gps_Almanac_data.SQRTA, 8 ); // 8
				  big2little(data + 20 ,(char *)&gps_Almanac_data.ECCENT, 8 ); // 8
				  big2little(data + 28 ,(char *)&gps_Almanac_data.ISUBO, 8 ); // 8
				  big2little(data + 36 ,(char *)&gps_Almanac_data.OMEGADOT, 8 ); // 8
				  big2little(data + 44 ,(char *)&gps_Almanac_data.OMEGSUBO, 8 ); // 8
				  big2little(data + 52 ,(char *)&gps_Almanac_data.OMEGA, 8 ); // 8
				  big2little(data + 60 ,(char *)&gps_Almanac_data.MSUBO, 8 ); // 8

				  gps_Almanac_data.ALM_HEALTH = data[68];//1

				  len_gps_almance = sizeof(gps_Almanac_data);

				  fwrite(&gps_Almanac_data,1,len_gps_almance,fp_creater_almance_gps);

				  break;
			  case 3:
				  printf("1");
				  break;
			  case 4:
				  printf("1");
				  break;
			  case 5:
				  printf("1"); 
				  break;
			  case 7:
				  printf("1");
				  break;
			  case 8:

				  glonass_almance_data.system_timestamps = timestamps;
				  glonass_almance_data.system_pps_num = pps_num;

				  glonass_almance_data.PRN = data[1];

				  big2little(data + 2 ,(char *)&glonass_almance_data.DAY_NUMBER, 2 ); // 2

				  glonass_almance_data.FDMA_NUMBER = data[4];

				  for(int i = 0 ; i < 8 ; i ++ )
				  {
					  big2little(data + 5 + i * 8 ,(char *)&glonass_almance_data.ECCENTRICITY + i * 8 , 8 ); // 8
				  }

				  glonass_almance_data.HEALTH = data[69]; 
				  /* fp_creater_almance_glonass */

				  len_glonass_almance = sizeof(glonass_almance_data);

                  fwrite(&glonass_almance_data,1,len_glonass_almance,fp_creater_almance_glonass); 
				  break; 
			  case 9:

				  glonass_ephemeris_data.system_timestamps = timestamps;
				  glonass_ephemeris_data.system_pps_num = pps_num;

				  glonass_ephemeris_data.PRN = data[1];

				  big2little(data + 2 ,(char *)&glonass_ephemeris_data.GPS_WEEK_EPH_VALID_REF_TIME, 2 ); // 2
				  big2little(data + 4 ,(char *)&glonass_ephemeris_data.GPS_TIME_EPH_VALID_REF_TIME, 4 ); // 4
				  big2little(data + 8 ,(char *)&glonass_ephemeris_data.GPS_WEEK_EPH_DECODE_REF_TIME, 2 ); // 2
				  big2little(data + 10 ,(char *)&glonass_ephemeris_data.GPS_TIME_EPH_DECODE_REF_TIME, 4 ); // 4
				  big2little(data + 14 ,(char *)&glonass_ephemeris_data.GLONASS_DAY_NUMBER, 2 ); // 2

				  glonass_ephemeris_data.REF_TIME_OF_EPHEMERIS = data[16];//1
				  glonass_ephemeris_data.LEAP_SECONDS = data[17];//1
				  glonass_ephemeris_data.FLAGS = data[18];//1

				  big2little(data + 19 ,(char *)&glonass_ephemeris_data.FRAME_START_TIME, 4 ); // 2

				  glonass_ephemeris_data.ARG_OF_DATA = data[23];//1
				  glonass_ephemeris_data.EPHEMERIS_SOURCE = data[24];//1
				  glonass_ephemeris_data.FDMA = data[25];//1
				  glonass_ephemeris_data.HEALTH = data[26];//1
				  glonass_ephemeris_data.GENERATION = data[27];
				  glonass_ephemeris_data.UDRE = data[28];

				  for(int i = 0 ; i < 14 ; i ++ )
				  {
					  big2little(data + 29 + i * 8 ,(char *)&glonass_ephemeris_data.X + i * 8 , 8 ); // 2
				  }

				  len_glonass_ephemeris = sizeof(glonass_ephemeris_data);

				  fwrite(&glonass_ephemeris_data,1,len_glonass_ephemeris,fp_creater_ephemeris_glonass);

				  break;
			  case 11:
				  printf("1");
				  break;
			  case 12:
				  printf("1");
				  break;
			  case 14:
				  printf("1");
				  break;
			  case 16:
				  printf("1");
				  break;
			  case 20:
				  printf("1");
				  break;
			  case 21:
				  printf("1");
				  break;
   		      case 22:
				  printf("1");
				  break;
			  default:
				  printf("1");
				  break;
		  }

		  break;
	  case 0x40:
           nav_data = &data[3];

           len_package = nav_data[1];//cmd len
           len_package_single = len_package;

		   sizeof_test = sizeof(gps_nav);

		   while( 1 )
		   {
			   switch(*nav_data)
			   {
				  case 0x01:
                        big2little(nav_data+2,(char *)&gps_nav.GPS_time_ms,sizeof(gps_nav.GPS_time_ms)); //4
                        big2little(nav_data+6,(char *)&gps_nav.GPS_week_num,sizeof(gps_nav.GPS_week_num));//2
						gps_nav.svn = nav_data[8];//1
						gps_nav.position_flag1 = nav_data[9];//1
						gps_nav.position_flag2 = nav_data[10];//1
						gps_nav.init_num = nav_data[11];//1

					  break;
				  case 0x02:
					    big2little(nav_data+2,(char *)&gps_nav.lat,8);
						big2little(nav_data+10,(char *)&gps_nav.lon,8);
						big2little(nav_data+18,(char *)&gps_nav.height,8);		                        

					  break;
				  case 0x08:
					  gps_nav.Velocity_flags = nav_data[2];//1
					  big2little(nav_data+3,(char *)&gps_nav.Speed,sizeof(gps_nav.Speed));//4
					  big2little(nav_data+7,(char *)&gps_nav.Heading,sizeof(gps_nav.Heading));//4
					  big2little(nav_data+11,(char *)&gps_nav.Vertical_velocity,sizeof(gps_nav.Vertical_velocity));//4
					  big2little(nav_data+15,(char *)&gps_nav.Local_heading,sizeof(gps_nav.Local_heading));//4
					  break;
				  case 0x09:
					  big2little(nav_data+2,(char *)&gps_nav.PDOP,sizeof(gps_nav.PDOP));//4 
					  big2little(nav_data+6,(char *)&gps_nav.HDOP,sizeof(gps_nav.HDOP));//4
                      big2little(nav_data+10,(char *)&gps_nav.VDOP,sizeof(gps_nav.VDOP));//4
					  big2little(nav_data+14,(char *)&gps_nav.TDOP,sizeof(gps_nav.TDOP));//4
					  break;
				  default:
					   break;
			   }
               if( com2_new_version )
			   {
				   if( len_package == 0x7f )
				   {
					   break;
				   }
			   }else
			   {
				   if( len_package == 0x3f )
				   {
					   break;
				   }
			   }
               nav_data = &nav_data[ len_package_single + 2 ];
               len_package += nav_data[1];//cmd len
			   len_package_single = nav_data[1];//cmd len
  		   }
   //    
		   gps_nav.system_pps_num = pps_num;
		   gps_nav.system_timestamps = timestamps;

           fwrite(&gps_nav,1,sizeof(gps_nav) ,fp_creater_nav );

		   if(nav_test)
			{
				//static unsigned long long last_times = 0;
				//static unsigned int reduce = 0;
				 //reduce = gps_nav.system_timestamps - last_times;
				 sprintf(buffer,"st : %d \r\n", gps_nav.system_timestamps );//PDOP;//size 4Heading
				 // last_times = gps_nav.system_timestamps;
				  // gps_nav.system_timestamps,
				  // gps_nav.system_pps_num,
				  // gps_nav.GPS_time_ms,
				  // gps_nav.GPS_week_num,
				  // gps_nav.svn,
				  // gps_nav.lat,
				  // gps_nav.lon,
				  // gps_nav.height,
				  // gps_nav.Velocity_flags,
				  // gps_nav.Speed,
				  // gps_nav.Heading,
				  // gps_nav.Vertical_velocity,
				  // gps_nav.PDOP);

			   len_write = strlen(buffer);

			   fwrite(buffer,1,len_write ,fp_creater_txt );
			}

		  break;
	  default:break;
  }
}

void Cgps_transferDlg::big2little(unsigned char * src,char * dst,unsigned char len)
{
	for(int i=0;i<len;i++)
	{
		dst[i] = src[ len - 1 - i ];
	}
}

void Cgps_transferDlg::real_time_gnss_survey_data(unsigned char * data/*beggin from the length */,unsigned int len)//subtype 6
{
   unsigned char record_type = 0;
   unsigned char page_number = 0;
   unsigned char reply_number = 0;
   unsigned char record_interpretation_flags = 0;
   unsigned char block_length = 0;
   unsigned short week_number;
   unsigned int receiver_time;
   unsigned char number_SVs;
   unsigned char number_of_sv;
   unsigned char satellite_id[2];//gps or glosnass , whitch number
   unsigned char EPOCH;
   unsigned int PSEUDORANGE;//
   static double pseu;
   unsigned int doppler_int = 0;
   static float doppler_f;
   
   unsigned char block_type;
   unsigned char track_type;
   unsigned short SNR;
   long long phase;
   unsigned char cycle_slip_count;
   unsigned char measurement_flags;
   unsigned char measurement_flags2;
   unsigned char measurement_flags3;
   unsigned char * rt_p = NULL;
   char buffer_txt[256];
   unsigned int length_of_total = 0;

   record_type = data[0];//4

   if( record_type == 0x06 )
   {
	   page_number = data[1];//5
	   reply_number = data[2];//6
	   record_interpretation_flags = data[3];//7

	   rt_p = &data[4];
	   block_length = rt_p[0];

       length_of_total += block_length;

	   big2little( rt_p + 1 , (char *)&week_number ,  2);
	   big2little( rt_p + 3 , (char *)&receiver_time ,4);

	   gps_psdu_doppler.GPS_receive_ms = receiver_time;
	   gps_psdu_doppler.week_number = week_number;

	   number_SVs = rt_p[10];
	   EPOCH = rt_p[11];
       /* ------------------------------------ reply ------------------------------------- */
   
	   rt_p += block_length;//0b
	   block_length = rt_p[0];
	   length_of_total += block_length;
	   while( length_of_total < len - 4 )
	   {
		   /* *************************satellite message id oc*************************** */
		   rt_p += block_length;
		   block_length = rt_p[0];
           
		   length_of_total += block_length;

		   satellite_id[0] = rt_p[1];//num
		   satellite_id[1] = rt_p[2];//gps
	       
		   gps_psdu_doppler.num = satellite_id[0];
		   gps_psdu_doppler.id  = satellite_id[1];

		   number_of_sv =  rt_p[4];

		   if(number_of_sv > 2)
		   {
			   return;
		   }

		   for(int i = 0 ; i < number_of_sv ; i++ )
		   {
	/* decode PSEUDORANGE */
			   rt_p += block_length;
			   block_length = rt_p[0];

			   if(rt_p[0] == 0)
			   {
				   return;
			   }

			   length_of_total += block_length;

			   if( i == 0 )
			   {
				   phase = 0;
				   block_type = rt_p[1];
				   track_type = rt_p[2];
				   big2little( rt_p + 3 , (char *)&SNR , 2);
				   big2little( rt_p + 5 , (char *)&PSEUDORANGE  ,4);
				   big2little( rt_p + 9 , (char *)&phase  ,6);

                   if( phase & 0x800000000000 )
				   {
                      phase = 0xffffffffffff - phase + 1;

					  gps_psdu_doppler.phase = ((double)phase / 32768.0f) * (-1);
				   }else
				   {
                      gps_psdu_doppler.phase = ((double)phase / 32768.0f);
				   }

				   gps_psdu_doppler.SNR = (double)SNR / 10.0f;

				   cycle_slip_count = rt_p[15];
				   measurement_flags = rt_p[16];

				   pseu = (double)PSEUDORANGE / 128;

				   gps_psdu_doppler.PSEUDORANGE = pseu;

				   if( measurement_flags & 0x80 )
				   {
					   /* measurement flags2 */
					   measurement_flags2 = rt_p[17];

					   if( measurement_flags2 & 0x80 )
					   {
                           measurement_flags3 = rt_p[18]; 
						   if( measurement_flags & 0x04 )
						   {
							   big2little( rt_p + 19 , (char *)&doppler_int ,  3);

							   if( doppler_int & 0x800000 )
							   {
									doppler_int = 0xffffff - doppler_int + 1;
									doppler_f = ((float)doppler_int / 256) * (-1);
							   }else
							   {
									doppler_f = (float)doppler_int / 256;
							   }
							   
							   gps_psdu_doppler.doppler = doppler_f;
						   }
					   }else
					   {
						   if( measurement_flags & 0x04 )
						   {
							   big2little( rt_p + 18 , (char *)&doppler_int ,  3);

							   if( doppler_int & 0x800000 )
							   {
									doppler_int = 0xffffff - doppler_int + 1;
									doppler_f = ((float)doppler_int / 256) * (-1);
							   }else
							   {
									doppler_f = (float)doppler_int / 256;
							   }
							   
							   gps_psdu_doppler.doppler = doppler_f;
						   }
					   }
				   }else
				   {  
					   if( measurement_flags & 0x04 )
					   {
						   big2little( rt_p + 17 , (char *)&doppler_int ,  3);
                    
						   if( doppler_int & 0x800000 )
						   {
								doppler_int = 0xffffff - doppler_int + 1;
								doppler_f = ((float)doppler_int / 256) * (-1);
						   }else
						   {
                                doppler_f = (float)doppler_int / 256;
						   }
						   
						   gps_psdu_doppler.doppler = doppler_f;
					   }
				   }
			   }else
			   {

			   }
		   }
		   /* printf and save data fp_creater_pde */
		   gps_psdu_doppler.system_timestamps = timestamps;

		   gps_psdu_doppler.system_pps_num = gps_psdu_doppler.GPS_receive_ms - GPS_TIME_LAST_PDE;

		   if( gps_psdu_doppler.system_pps_num > 300 )
		   {
			   gps_psdu_doppler.system_pps_num = 300;
		   }

		   GPS_TIME_LAST_PDE = gps_psdu_doppler.GPS_receive_ms;

           fwrite(&gps_psdu_doppler,1,sizeof(gps_psdu_doppler),fp_creater_pde);
		   /* --sdf-s-dsf-s-dfs-df-sd-fs-fds-f-sdf*/
		    if(raw_test)
			{
			   int len_str;
			   sprintf(buffer_txt,"pps_num:%d gps_rec_time:%d num:%d id:%d gps_week:%d ",//pseud:%10.3f doppler:%10.3f\r\n",
				   //gps_psdu_doppler.system_timestamps,
				   gps_psdu_doppler.system_pps_num,
				   gps_psdu_doppler.GPS_receive_ms,
				   gps_psdu_doppler.num,
				   gps_psdu_doppler.id,
				   gps_psdu_doppler.week_number);
	/*			   gps_psdu_doppler.PSEUDORANGE,
				   gps_psdu_doppler.doppler*/

				   len_str = strlen(buffer_txt);

				   sprintf(buffer_txt + len_str,"st:%d ",gps_psdu_doppler.system_timestamps);

				   len_str = strlen(buffer_txt);

				   sprintf(buffer_txt + len_str,"pseud:%10.3f ",gps_psdu_doppler.PSEUDORANGE);

				   len_str = strlen(buffer_txt);

				   sprintf(buffer_txt + len_str,"doppler:%10.3f",gps_psdu_doppler.doppler);

				   len_str = strlen(buffer_txt);

				   sprintf(buffer_txt + len_str,"phase:%10.3f",gps_psdu_doppler.phase);

				   len_str = strlen(buffer_txt);

				   sprintf(buffer_txt + len_str,"snr:%10.3f\r\n",gps_psdu_doppler.SNR);

				   len_str = strlen(buffer_txt);

				   fwrite(&buffer_txt,1,len_str,fp_creater_pde_txt);
			}
		   /*-----------------------------------------*/
	   }
	   /* ------------------------------------ reply ------------------------------------- */

   }
}

void Cgps_transferDlg::icm_20609_handle(unsigned char *buffer,unsigned int len)
{
	unsigned char *p;
	unsigned int len_tmp = 0;
	float gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z;
	long long system_timestamps;
	unsigned short tt;
	unsigned int pps_num;
	char put_buffer[256];
	unsigned int len_str;
	const float gyro_sensitivity = 1 / 32.8f / 180.0f * 3.1415926f;
	const float accel_sensitivity = 1 / 4096.0f * 9.788f;
	p = buffer;
	/* data handle */
	system_timestamps = 0;
	pps_num = 0;
	tt = 0;
	tt |= p[0]  << 8;
	tt |= p[1]  << 0;
	system_timestamps |= p[2]  << 40;
	system_timestamps |= p[3]  << 32;
	system_timestamps |= p[4] << 24;
	system_timestamps |= p[5] << 16;
	system_timestamps |= p[6] << 8;
	system_timestamps |= p[7] << 0;
	/* pps num */
	pps_num |= p[8] << 24;
	pps_num |= p[9] << 16;
	pps_num |= p[10] << 8;
	pps_num |= p[11] << 0;

	/* sensor */
	gyro_x = (float)((short)(p[14] * 256 + p[15])) * gyro_sensitivity;
	gyro_y = (float)((short)(p[17] * 256 + p[18])) * gyro_sensitivity;
	gyro_z = (float)((short)(p[20] * 256 + p[21])) * gyro_sensitivity;

	acc_x =  (float)((short)(p[24] * 256 + p[25])) * accel_sensitivity;
	acc_y =  (float)((short)(p[27] * 256 + p[28])) * accel_sensitivity;
	acc_z =  (float)((short)(p[30] * 256 + p[31])) * accel_sensitivity;

	sprintf(put_buffer,"systime:%d tt:%d pps:%d g_x:%f g_y:%f g_z:%f a_x:%f a_y:%f a_z:%f \r\n",
					(unsigned int)system_timestamps,
					tt,
					pps_num,
					gyro_x,
					gyro_y,
					gyro_z,
					acc_x,
					acc_y,
					acc_z);

	len_str = strlen(put_buffer);

	fwrite(put_buffer,1,len_str,fp_creater_imu);
}



void Cgps_transferDlg::nmea_decode(char * data ,unsigned int len)
{
	unsigned int bufferdef[7];
	unsigned short tmp;
	char show[200];
   for( unsigned int i = 0 ; i < len ; i++)
   {
     if( data[i] == '$' && data[i+1] == 'G' && data[i+2] == 'P' && data[i+3] == 'G' && data[i+4] == 'S' && data[i+5] == 'V' )
	 {
        if(sscanf(&data[i],"$GPGSV,%d,%d,%d,%d,%d,%d,%d,",&bufferdef[0],&bufferdef[1],&bufferdef[2],&bufferdef[3],&bufferdef[4],&bufferdef[5],&bufferdef[6]) == 7)
		{
            tmp = (unsigned char)bufferdef[3] << 8 | (unsigned char)bufferdef[6] ;
			fwrite(&tmp,1,2,fp_nmea_bin);
			sprintf(show,"sv_num:%d SNR:%d\r\n", bufferdef[3], bufferdef[6]);
            fwrite(show,1,strlen(show),fp_nmea_txt);
		}
	 }
   }
}




