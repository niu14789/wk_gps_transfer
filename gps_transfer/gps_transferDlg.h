// gps_transferDlg.h : ͷ�ļ�
//

#pragma once
#include "afxwin.h"


// Cgps_transferDlg �Ի���
class Cgps_transferDlg : public CDialog
{
// ����
public:
	Cgps_transferDlg(CWnd* pParent = NULL);	// ��׼���캯��
	void Cgps_transferDlg::data_parse(unsigned char *buffer,unsigned int len);
	void Cgps_transferDlg::data_step(unsigned char d);
	void Cgps_transferDlg::raw_gps_decode(unsigned char d);
	void Cgps_transferDlg::raw_gps_parse(unsigned char *buffer,unsigned int len);
	void Cgps_transferDlg::gps_raw_detail( unsigned char type,unsigned char *data,unsigned char len );
	void Cgps_transferDlg::big2little(unsigned char * src,char * dst,unsigned char len);
	void Cgps_transferDlg::real_time_gnss_survey_data(unsigned char * data/*beggin from the length */,unsigned int len);//subtype 6
	void Cgps_transferDlg::icm_20609_handle(unsigned char *buffer,unsigned int len);
	void Cgps_transferDlg::nmea_decode(char * data ,unsigned int len);
// �Ի�������
	enum { IDD = IDD_GPS_TRANSFER_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButton1();
public:
	afx_msg void OnBnClickedButton2();
public:
	CComboBox m_switch;
public:
	CButton m_check_raw_enable;
public:
	CButton m_check_nav_enable;
public:
	CButton m_20609_enable;
public:
	CButton m_rt27;
public:
	CButton m_check_nmea;
public:
	CButton m_new_version;
};
