 /* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef WLAN_PLATFORM_SERVICE_V01_H
#define WLAN_PLATFORM_SERVICE_V01_H

#define WPLS_SERVICE_ID_V01 0x2001
#define WPLS_SERVICE_VERS_V01 0x01

#define QMI_WPLS_CAP_RESP_V01 0x0023
#define QMI_WPLS_CAP_REQ_V01 0x0023
#define QMI_WPLS_CAL_DOWNLOAD_RESP_V01 0x0026
#define QMI_WPLS_CAL_DOWNLOAD_REQ_V01 0x0026
#define QMI_WPLS_FW_READY_IND_V01 0x0021
#define QMI_WPLS_CAL_UPDATE_IND_V01 0x0027
#define QMI_WPLS_CAL_REPORT_RESP_V01 0x0025
#define QMI_WPLS_WLAN_INFO_RESP_V01 0x0022
#define QMI_WPLS_CAL_REPORT_REQ_V01 0x0025
#define QMI_WPLS_BDF_DOWNLOAD_RESP_V01 0x0024
#define QMI_WPLS_HANDSHAKE_REQ_V01 0x0020
#define QMI_WPLS_WLAN_INFO_REQ_V01 0x0022
#define QMI_WPLS_BDF_DOWNLOAD_REQ_V01 0x0024
#define QMI_WPLS_HANDSHAKE_RESP_V01 0x0020
#define QMI_WPLS_CAL_DOWNLOAD_IND_V01 0x0026
#define QMI_WPLS_CAL_UPDATE_REQ_V01 0x0027
#define QMI_WPLS_CAL_UPDATE_RESP_V01 0x0027

#define QMI_WPLS_MAX_DATA_SIZE_V01 6144
#define QMI_WPLS_MAX_NUM_MSG_V01 10
#define QMI_WPLS_MAX_NUM_CE_V01 12
#define QMI_WPLS_MAX_STR_LEN_V01 16
#define QMI_WPLS_MAX_NUM_CAL_V01 10
#define QMI_WPLS_MAX_NUM_SVC_V01 24

enum wpls_driver_mode_t_v01 {
	WPLS_DRIVER_MODE_T_MIN_VAL_V01 = INT_MIN,
	QMI_WPLS_MISSION_V01 = 0,
	QMI_WPLS_FTM_V01 = 1,
	QMI_WPLS_EPPING_V01 = 2,
	WPLS_DRIVER_MODE_T_MAX_VAL_V01 = INT_MAX,
};

enum wpls_cal_temp_id_t_v01 {
	WPLS_CAL_TEMP_ID_T_MIN_VAL_V01 = INT_MIN,
	QMI_WPLS_CAL_TEMP_IDX_0_V01 = 0,
	QMI_WPLS_CAL_TEMP_IDX_1_V01 = 1,
	QMI_WPLS_CAL_TEMP_IDX_2_V01 = 2,
	QMI_WPLS_CAL_TEMP_IDX_3_V01 = 3,
	QMI_WPLS_CAL_TEMP_IDX_4_V01 = 4,
	WPLS_CAL_TEMP_ID_T_MAX_VAL_V01 = INT_MAX,
};

struct wpls_data_s_v01 {
	enum wpls_cal_temp_id_t_v01 file_id;
	uint32_t total_size;
	uint32_t seg_id;
	uint32_t data_len;
	uint8_t data[QMI_WPLS_MAX_DATA_SIZE_V01];
	uint8_t end;
};

struct wpls_ce_tgt_pipe_cfg_s_v01 {
	uint32_t pipe_num;
	uint32_t pipe_dir;
	uint32_t nentries;
	uint32_t nbytes_max;
	uint32_t flags;
	uint32_t reserved;
};

struct wpls_ce_svc_pipe_cfg_s_v01 {
	uint32_t service_id;
	uint32_t pipedir;
	uint32_t pipenum;
};

struct wpls_handshake_req_msg_v01 {
	uint32_t msg_len;
	uint32_t msg[QMI_WPLS_MAX_NUM_MSG_V01];
};
#define WPLS_HANDSHAKE_REQ_MSG_V01_MAX_MSG_LEN 44
extern struct elem_info wpls_handshake_req_msg_v01_ei[];

struct wpls_handshake_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};
#define WPLS_HANDSHAKE_RESP_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_handshake_resp_msg_v01_ei[];

struct wpls_fw_ready_ind_msg_v01 {
	char placeholder;
};
#define WPLS_FW_READY_IND_MSG_V01_MAX_MSG_LEN 0
extern struct elem_info wpls_fw_ready_ind_msg_v01_ei[];

struct wpls_wlan_info_req_msg_v01 {
	uint8_t enable;
	enum wpls_driver_mode_t_v01 mode;
	uint8_t host_version_valid;
	char host_version[QMI_WPLS_MAX_STR_LEN_V01];
	uint8_t tgt_cfg_valid;
	uint32_t tgt_cfg_len;
	struct wpls_ce_tgt_pipe_cfg_s_v01 tgt_cfg[QMI_WPLS_MAX_NUM_CE_V01];
	uint8_t svc_cfg_valid;
	uint32_t svc_cfg_len;
	struct wpls_ce_svc_pipe_cfg_s_v01 svc_cfg[QMI_WPLS_MAX_NUM_SVC_V01];
};
#define WPLS_WLAN_INFO_REQ_MSG_V01_MAX_MSG_LEN 614
extern struct elem_info wpls_wlan_info_req_msg_v01_ei[];

struct wpls_wlan_info_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};
#define WPLS_WLAN_INFO_RESP_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_wlan_info_resp_msg_v01_ei[];

struct wpls_cap_req_msg_v01 {
	char placeholder;
};
#define WPLS_CAP_REQ_MSG_V01_MAX_MSG_LEN 0
extern struct elem_info wpls_cap_req_msg_v01_ei[];

struct wpls_cap_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
	uint32_t board_id;
	uint32_t num_peers;
	uint32_t mac_version;
	char fw_version[QMI_WPLS_MAX_STR_LEN_V01];
};
#define WPLS_CAP_RESP_MSG_V01_MAX_MSG_LEN 47
extern struct elem_info wpls_cap_resp_msg_v01_ei[];

struct wpls_bdf_download_req_msg_v01 {
	uint8_t bdf_data_valid;
	struct wpls_data_s_v01 bdf_data;
};
#define WPLS_BDF_DOWNLOAD_REQ_MSG_V01_MAX_MSG_LEN 6162
extern struct elem_info wpls_bdf_download_req_msg_v01_ei[];

struct wpls_bdf_download_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};
#define WPLS_BDF_DOWNLOAD_RESP_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_bdf_download_resp_msg_v01_ei[];

struct wpls_cal_report_req_msg_v01 {
	uint32_t meta_data_len;
	uint32_t meta_data[QMI_WPLS_MAX_NUM_CAL_V01];
};
#define WPLS_CAL_REPORT_REQ_MSG_V01_MAX_MSG_LEN 44
extern struct elem_info wpls_cal_report_req_msg_v01_ei[];

struct wpls_cal_report_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};
#define WPLS_CAL_REPORT_RESP_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_cal_report_resp_msg_v01_ei[];

struct wpls_cal_download_ind_msg_v01 {
	enum wpls_cal_temp_id_t_v01 cal_id;
};
#define WPLS_CAL_DOWNLOAD_IND_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_cal_download_ind_msg_v01_ei[];

struct wpls_cal_download_req_msg_v01 {
	uint8_t cal_data_valid;
	struct wpls_data_s_v01 cal_data;
};
#define WPLS_CAL_DOWNLOAD_REQ_MSG_V01_MAX_MSG_LEN 6162
extern struct elem_info wpls_cal_download_req_msg_v01_ei[];

struct wpls_cal_download_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
};
#define WPLS_CAL_DOWNLOAD_RESP_MSG_V01_MAX_MSG_LEN 7
extern struct elem_info wpls_cal_download_resp_msg_v01_ei[];

struct wpls_cal_update_ind_msg_v01 {
	enum wpls_cal_temp_id_t_v01 cal_id;
	uint32_t total_size;
};
#define WPLS_CAL_UPDATE_IND_MSG_V01_MAX_MSG_LEN 14
extern struct elem_info wpls_cal_update_ind_msg_v01_ei[];

struct wpls_cal_update_req_msg_v01 {
	enum wpls_cal_temp_id_t_v01 cal_id;
	uint32_t seg_id;
};
#define WPLS_CAL_UPDATE_REQ_MSG_V01_MAX_MSG_LEN 14
extern struct elem_info wpls_cal_update_req_msg_v01_ei[];

struct wpls_cal_update_resp_msg_v01 {
	struct qmi_response_type_v01 resp;
	struct wpls_data_s_v01 cal_data;
};
#define WPLS_CAL_UPDATE_RESP_MSG_V01_MAX_MSG_LEN 6169
extern struct elem_info wpls_cal_update_resp_msg_v01_ei[];

#endif
