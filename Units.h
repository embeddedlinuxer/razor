/* This Information is proprietary to Phase Dynamics Inc, Richardson, Texas
* and MAY NOT be copied by any method or incorporated into another program
* without the express written consent of Phase Dynamics Inc. This information
* or any portion thereof remains the property of Phase Dynamics Inc.
* The information contained herein is believed to be accurate and Phase
* Dynamics Inc assumes no responsibility or liability for its use in any way
* and conveys no license or title under any patent or copyright and makes
* no representation or warranty that this Information is free from patent
* or copyright infringement.
*------------------------------------------------------------------------

*------------------------------------------------------------------------
* Units.h 
*-------------------------------------------------------------------------
* Contains all the code relevant to the LCD menu system. The menu system
* uses a state-machine architecture, which is defined in Menu.h by a state
* table and a state transition table. This is the only Razor code that is
* run in the context of a Task (SYS-BIOS module) ?everything else is
* interrupt-based.
* Most "end-node" states of the menu tree have a function associated with
* them. Each MNU_xxxx menu calls the corresponding FXN_xxxx function.
* Currently, we cycle through the mnu
* code with a maximum frequency of about 6.67 times per second
* (minimum period = 0.15 seconds).
*-------------------------------------------------------------------------
* HISTORY:
*       Aug-20-2019 : Daniel Koh : Created Ver 1.0.0
*------------------------------------------------------------------------*/

#ifndef _UNITS_H
#define _UNITS_H

/*
 *
 * Integer Codes
 * refer to HART - "Common Tables HFC_SPEC-183, Rev 13.0"
 * (note: some unit codes are added for PDI use)
 */

/* generic unit codes */
#define var_not_used						250
#define u_generic_not_used					250
#define u_generic_none						251
#define u_generic_unknown					252
#define u_generic_special					253

/* mfgr specific units = 240...249 */
#define u_mfgr_specific_none				240
#define u_mfgr_specific_water_phase			241
#define u_mfgr_specific_bps					242
#define u_mfgr_specific_ct					243
#define u_mfgr_specific_MHz					244
#define u_mfgr_specific_V_per_MHz			245
#define u_mfgr_specific_perc_per_API_60F	246
#define u_mfgr_specific_perc_per_kgm3_15C	247
#define u_mfgr_specific_u_v_mscf			248
#define u_mfgr_specific_u_vfr_mscf_day		249
#define u_alt_percent						57		/* more compatible with HART 5.2 */

/* temperature unit codes */
#define c_temperature						64
#define u_temp_C							32
#define u_temp_F							33
#define u_temp_R							34
#define u_temp_K							35

/* pressure unit codes */
#define c_pressure							65
#define u_p_in_H2O_68F						1
#define u_p_in_Hg_0C						2
#define u_p_ft_H2O_68F						3
#define u_p_mm_H2O_68F						4
#define u_p_mm_Hg_0C						5
#define u_p_psi								6
#define u_p_lbs_si							6
#define u_p_bar								7
#define u_p_mbar							8
#define u_p_g_scm							9
#define u_p_kg_scm							10
#define u_p_Pa								11
#define u_p_kPa								12
#define u_p_torr							13
#define u_p_atm								14
#define u_p_in_H2O_60F						145
#define u_p_cm_H2O_4C						170
#define u_p_m_H2O_4C						171
#define u_p_cm_Hg_0C						172
#define u_p_lbs_sf							173
#define u_p_hectoPa							174
#define u_p_psia							175
#define u_p_kg_sm							176
#define u_p_ft_H2O_4C						177
#define u_p_ft_H2O_60F						178
#define u_p_m_Hg_0C							179
#define u_p_MPa								237
#define u_p_in_H2O_4C						238
#define u_p_mm_H2O_4C						239

/* velocity unit codes */
#define c_velocity							67
#define u_vel_feet_sec						20
#define u_vel_meter_sec						21
#define u_vel_in_sec						114
#define u_vel_in_min						115
#define u_vel_feet_min						116
#define u_vel_meter_hr						120

/* volume totalizer/inventory unit codes */
#define c_volume							68
#define u_v_gal								40
#define u_v_L								41
#define u_v_imp_gal							42
#define u_v_cm								43
#define u_v_barrel							46
#define u_v_bushel							110
#define u_v_cyard							111
#define u_v_cf								112
#define u_v_ci								113
#define u_v_bbl_liq							124
#define u_v_hectoliter						236

/*STP*/
#define u_v_N_cm							166
#define u_v_N_L								167
#define u_v_scf								168
#define u_v_mscf							u_mfgr_specific_u_v_mscf

/* length unit codes */
#define c_length							69
#define u_l_ft								44
#define u_l_m								45
#define u_l_in								47
#define u_l_cm								48
#define u_l_mm								49
#define u_l_ft16							151

/* time unit codes */
#define c_time								70
#define u_time_min							50
#define u_time_sec							51
#define u_time_hr							52
#define u_time_day							53

/* mass totalizer/inventory unit codes */
#define c_mass								71
#define u_m_g								60
#define u_m_kg								61
#define u_m_metric_tons						62
#define u_m_lbs								63
#define u_m_short_tons						64
#define u_m_long_tons						65
#define u_m_ounce							125

/* volume flow rate unit codes */
#define c_volumetric_flow					66
#define u_vfr_cf_min						15
#define u_vfr_gal_min						16
#define u_vfr_L_min							17
#define u_vfr_imp_gal_min		    		18
#define u_vfr_cm_hr							19
#define u_vfr_gal_sec						22
#define u_vfr_Mgal_day						23
#define u_vfr_L_sec							24
#define u_vfr_ML_day						25
#define u_vfr_cf_sec						26
#define u_vfr_cf_day						27
#define u_vfr_cm_sec						28
#define u_vfr_cm_day						29
#define u_vfr_imp_gal_hr					30
#define u_vfr_imp_gal_day		   			31
#define u_vfr_cf_hr							130
#define u_vfr_cm_min						131
#define u_vfr_barrel_sec					132
#define u_vfr_barrel_min					133
#define u_vfr_barrel_hr			       		134
#define u_vfr_barrel_day					135
#define u_vfr_gal_hr						136
#define u_vfr_imp_gal_sec		    		137
#define u_vfr_L_hr							138
#define u_vfr_gal_day						235

/*STP*/
#define u_vfr_N_cm_hr						121
#define u_vfr_N_L_hr						122
#define u_vfr_scf_min						123
#define u_vfr_mscf_day						u_mfgr_specific_u_vfr_mscf_day
#define u_vfr_scf_day						184
#define u_vfr_scf_sec						186
#define u_vfr_N_cm_sec						190
#define u_vfr_N_cm_day						191

/* mass flow rate unit codes */
#define c_mass_flow							72
#define u_mfr_g_sec							70
#define u_mfr_g_min							71
#define u_mfr_g_hr							72
#define u_mfr_kg_sec						73
#define u_mfr_kg_min			     		74
#define u_mfr_kg_hr							75
#define u_mfr_kg_day						76
#define u_mfr_metric_tons_min				77
#define u_mfr_metric_tons_hr				78
#define u_mfr_metric_tons_day				79
#define u_mfr_lbs_sec						80
#define u_mfr_lbs_min						81
#define u_mfr_lbs_hr						82
#define u_mfr_lbs_day						83
#define u_mfr_short_tons_min				84
#define u_mfr_short_tons_hr					85
#define u_mfr_short_tons_day				86
#define u_mfr_long_tons_hr					87
#define u_mfr_long_tons_day					88

/* mass per volume unit codes */
#define c_mass_per_volume					73
#define u_mpv_sg							90
#define u_mpv_g_cc							91
#define u_mpv_kg_cm							92
#define u_mpv_lbs_gal						93
#define u_mpv_lbs_cf						94
#define u_mpv_lbs_cf_60F					941
#define u_mpv_g_mL							95
#define u_mpv_kg_L							96
#define u_mpv_g_L							97
#define u_mpv_lbs_ci						98
#define u_mpv_short_tons_cyard				99
#define u_mpv_deg_twaddell					100
#define u_mpv_deg_baume_heavy				102
#define u_mpv_deg_baume_light				103
#define u_mpv_deg_API						104
#define	u_mpv_deg_API_60F					105
#define	u_mpv_deg_API_15C					106
#define	u_mpv_kg_cm_15C						107
#define	u_mpv_kg_cm_60F						108
#define u_mpv_sg_60F						109
#define u_mpv_sg_15C						110
#define u_mpv_ug_L							146
#define u_mpv_ug_cm							147
#define u_mpv_percent_consistency			148

/* viscosity unit codes */
#define c_viscosity							74
#define u_visc_centistokes		       		54
#define u_visc_centipoise					55

/* angular velocity unit codes */
#define c_angular_velocity					75
#define u_ang_deg_sec						117
#define u_ang_RPS							118
#define u_ang_RPM							117

/* area unit codes */
#define c_area								76

/* work/energy unit codes */
#define c_energy_work						77
#define u_nrg_Nm					 		69
#define u_nrg_deka_therm					89
#define u_nrg_ft_lb							126
#define u_nrg_kWh							128
#define u_nrg_Mc							162
#define u_nrg_MJ							164
#define u_nrg_BTU							165

/* force unit codes */
#define c_force								78
#define u_force_newton						68

/* power unit codes */
#define c_power								79
#define u_pwr_kW							127
#define u_pwr_HP							129
#define u_pwr_Mc_hr							140
#define u_pwr_MJ_hr							141
#define u_pwr_BTU_hr						142

/* frequency unit codes */
#define c_frequency						80
#define u_freq_hertz					38

/* analytical unit codes */
#define c_analytical					81
#define u_ana_percent					57
#define u_ana_pH					 	59
#define u_ana_perc_steam_qual			150
#define u_ana_perc_plato				160
#define u_ana_percent_lower_explosion_level	161

/* capacitance unit codes */
#define c_capacitance					82
#define u_cap_pF					 	153

/* emf unit codes */
#define c_emf							83
#define u_emf_mV						36
#define u_emf_V							58

/* current unit codes */
#define c_current						84
#define u_cur_mA						39

/* resistance unit codes */
#define c_resistance					85
#define u_res_ohm						37
#define u_res_kohm						163

/* angle unit codes */
#define c_angle							86
#define u_ang_deg						143
#define u_ang_rad						144

/* conductance unit codes */
#define c_conductance					87
#define u_cond_microsiemens				56
#define u_cond_m_siem_cm				66
#define u_cond_u_siem_cm				67

/* volume per volume unit codes */
#define c_volume_per_volume				88
#define u_vpv_volume_percent			149
#define u_vpv_mL_L						154
#define u_vpv_uL_L						155

/* volume per mass unit codes */
#define c_volume_per_mass				89
#define u_misc_deg_balling				107
#define u_misc_cf_lb					152

/* concentration unit codes */
#define c_concentration					90
#define u_conc_deg_brix					101
#define u_conc_perc_solid_per_weight	105
#define u_conc_perc_solid_per_vol		106
#define u_conc_proof_vol				108
#define u_conc_proof_mass				109
#define u_conc_ppm					 	139
#define u_conc_ppb						169

/* misc unit codes */
#define c_not_classified				0
#define c_not_used						250
#define c_none							251
#define c_unknown						252
#define c_special						253
#define u_misc_newton					68
#define u_misc_deg					 	143
#define u_misc_rad					 	144
#define u_misc_percent_consistency		148
#define u_misc_feet_16ths				151

/* process variable codes */
#define u_proc_mass_flow_rate	   		0
#define u_proc_temp						1
#define u_proc_mass_totalizer	   		2
#define u_proc_density					3
#define u_proc_mass_inventory	   		4
#define u_proc_volume_flow_rate	   		5
#define u_proc_volume_totalizer	   		6
#define u_proc_volume_inventory	   		7
#define u_proc_viscosity		       	8
#define u_proc_pressure					9
#define u_proc_event1					10
#define u_proc_event2					11

/* sensor flange codes */
#define u_flange_ansi150		   		0
#define u_flange_ansi300		   		1
#define u_flange_ansi600		   		2
#define u_flange_pn40					5
#define u_flange_jis10k					7
#define u_flange_jis20k					8
#define u_flange_ansi900		   		9
#define u_flange_sanitary_clamp	   		10
#define u_flange_union					11
#define u_flange_unknown		       	252
#define u_flange_special		   		253

/* sensor flow tube construction material codes */
#define u_mat_hastelloy_c22				3
#define u_mat_monel						4
#define u_mat_tantalum					5
#define u_mat_316L_ss					19
#define u_mat_unknown					252
#define u_mat_special					253

/* sensor flow tube liner material codes */
#define u_mat_PTFE						10
#define u_mat_teflon					10
#define u_mat_halar						11
#define u_mat_tefzel					16
#define u_mat_none						251
#define u_mat_unknown					252
#define u_mat_special					253

/* control output variable codes */
#define u_ctrl_for_rev_flow				0
#define u_ctrl_zero_in_progress	   		1
#define u_ctrl_faults					2
#define u_ctrl_event1					3
#define u_ctrl_event2					4

/* flow direction codes */
#define u_flow_forward					0
#define u_flow_reverse					1
#define u_flow_bidirectional	  		2

/* fault limit codes */
#define u_fault_upscale					0
#define u_fault_downscale		   		1
#define u_fault_last_measured_val  		2
#define u_fault_internal_zero	  		3

/* operating mode codes */
#define u_operating_mode_normal			0
#define u_operating_mode_not_used		250
#define u_operating_mode_none			251
#define u_operating_mode_unknown		252
#define u_operating_mode_special		253

/* variable status - upper 2 bits of upper nibble */
#define h_var_stat_bad					0x00
#define h_var_stat_good					0xC0
#define h_var_stat_poor_accuracy		0x40
#define h_var_stat_fixed				0x80
#define h_var_stat_manual				0x80

/* variable status - lower 2 bits of upper nibble */
#define h_var_stat_constant				0x30
#define h_var_stat_low_limit			0x10
#define h_var_stat_high_limit			0x10
#define h_var_stat_no_limit				0x00

#define h_var_stat_more					0x08

#define c_valve_actuator				91

/* transfer function codes */
/* mfgr specific tf = 240...249 */
#define tf_linear						0
#define tf_sqrt							1
#define tf_sqrt3						2
#define tf_sqrt5						3
#define tf_special_curve				4
#define tf_sq							5
#define tf_discrete						230
#define tf_sqrt_special					231
#define tf_sqrt3_special				232
#define tf_sqrt5_special				233
#define tf_not_used						250
#define tf_none							251
#define tf_unknown						252
#define tf_special						253

#ifndef UNITS_H
extern const float* MASTER_UNITS;
#endif

#ifdef UNITS_H
//#pragma DATA_SECTION(MASTER_UNITS,"TABLES")
const float MASTER_UNITS[]={
								c_volume,				0,0,
								u_v_barrel,				1.0, 0.0,
								u_v_bbl_liq,			1.0, 0.0,
								u_v_gal,				42.0, 0.0,
								u_v_L,					158.987294928, 0.0,
								u_v_imp_gal,			34.972315754418, 0.0,
								u_v_cm,					0.158987294928, 0.0,
								u_v_bushel,				4.5116767887198, 0.0,
								u_v_cyard,				0.207947530864206, 0.0,
								u_v_cf,					5.6145833333352, 0.0,
								u_v_ci,					9702.0, 0.0,
								u_v_N_cm,				0.158987294928, 0.0,
								u_v_N_L,				158.987294928, 0.0,
								u_v_scf,				5.6145833333352, 0.0,
								u_v_hectoliter,			1.58987294928, 0.0,
								u_v_mscf,				5.6145833333352/1e3, 0.0,

								c_volumetric_flow,		0,0,
								u_vfr_barrel_day,		1.0, 0.0,
								u_vfr_Mgal_day,			42.0/1e3, 0.0,
								u_vfr_ML_day,			158.987294928/1e3, 0.0,
								u_vfr_cf_day,			5.6145833333352, 0.0,
								u_vfr_cm_day,			0.158987294928, 0.0,
								u_vfr_imp_gal_day,		34.972315754418, 0.0,
								u_vfr_gal_day,			42.0, 0.0,
								u_vfr_mscf_day,			5.6145833333352/1e3, 0.0,
								u_vfr_scf_sec,			5.6145833333352/86400.0, 0.0,
								u_vfr_scf_day,			5.6145833333352, 0.0,
								u_vfr_barrel_min,		1.0/1440.0, 0.0,
								u_vfr_cm_min,			0.158987294928/1440.0, 0.0,
								u_vfr_cf_min,			5.6145833333352/1440.0, 0.0,
								u_vfr_gal_min,			42.0/1440.0, 0.0,
								u_vfr_L_min,			158.987294928/1440.0, 0.0,
								u_vfr_imp_gal_min,		34.972315754418/1440.0, 0.0,
								u_vfr_scf_min,			5.6145833333352/1440.0, 0.0,
								u_vfr_cm_hr,			0.158987294928/24.0, 0.0,
								u_vfr_imp_gal_hr,		34.972315754418/24.0, 0.0,
								u_vfr_N_cm_hr,			0.158987294928/24.0, 0.0,
								u_vfr_N_cm_sec,			0.158987294928/86400.0, 0.0,
								u_vfr_N_cm_day,			0.158987294928, 0.0,
								u_vfr_N_L_hr,			158.987294928/24.0, 0.0,
								u_vfr_cf_hr,			5.6145833333352/24.0, 0.0,
								u_vfr_barrel_hr,		1.0/24.0, 0.0,
								u_vfr_gal_hr,			42.0/24.0, 0.0,
								u_vfr_L_hr,				158.987294928/24.0, 0.0,
								u_vfr_gal_sec,			42.0/86400.0, 0.0,
								u_vfr_L_sec,			158.987294928/86400.0, 0.0,
								u_vfr_cf_sec,			5.6145833333352/86400.0, 0.0,
								u_vfr_cm_sec,			0.158987294928/86400.0, 0.0,
								u_vfr_barrel_sec,		1.0/86400.0, 0.0,
								u_vfr_imp_gal_sec,		34.972315754418/86400.0, 0.0,

								c_temperature,			0,0,
								u_temp_C,				1.0, 0.0,
								u_temp_F,				1.8, 32.0,
								u_temp_R,				1.8, 32.0+491.67,
								u_temp_K,				1.0, 273.15,

								c_concentration,				0,0,
								u_conc_perc_solid_per_vol,		1.0, 0.0,
								u_conc_perc_solid_per_weight,	1.0, 0.0,
								u_conc_ppm,						1e4, 0.0,
								u_conc_ppb,						1e7, 0.0,

								c_frequency,			0,0,
								u_mfgr_specific_MHz,	1.0, 0.0,
								u_freq_hertz,			1e6, 0.0,

								c_pressure,				0,0,
								u_p_atm,				1.0, 0.0,
								u_p_Pa,					101325.0, 0.0,
								u_p_kPa,				101.325, 0.0,
								u_p_hectoPa,			1013.25, 0.0,
								u_p_MPa,				0.101325, 0.0,
								u_p_psi,				14.69596432068, 0.0,
								u_p_psia,				14.69596432068, 0.0,
								u_p_lbs_sf,				2116.218862178, 0.0,
								u_p_bar,				1.01325, 0.0,
								u_p_mbar,				1013.25, 0.0,
								u_p_g_scm,				1033.2274528 , 0.0,
								u_p_kg_scm,				1.0332274528, 0.0,
								u_p_kg_sm,				10332.274528, 0.0,
								u_p_torr,				760.0, 0.0,
								u_p_in_H2O_68F,			406.7824617322385, 0.0,
								u_p_ft_H2O_68F,			33.898538477686536, 0.0,
								u_p_mm_H2O_68F,			10332.27452799886, 0.0,
								u_p_in_H2O_60F,			407.1893586240, 0.0,
								u_p_ft_H2O_60F,			33.932446552, 0.0,
								u_p_mm_H2O_4C,			10332.274527998856, 0.0,
								u_p_cm_H2O_4C,			1033.2274527998856, 0.0,
								u_p_m_H2O_4C,			103.32274527998856, 0.0,
								u_p_in_H2O_4C,			406.7824617322385, 0.0,
								u_p_ft_H2O_4C,			33.898538477686536, 0.0,
								u_p_in_Hg_0C,			760.0/25.4, 0.0,
								u_p_mm_Hg_0C,			760.0, 0.0,
								u_p_cm_Hg_0C,			76.0, 0.0,
								u_p_m_Hg_0C,			7.6, 0.0,

								c_emf,					0,0,
								u_emf_V,				1.0, 0.0,
								u_emf_mV,				1000.0, 0.0,

								c_current,				0,0,
								u_cur_mA,				1.0, 0.0,

								c_analytical,			0,0,
								u_ana_percent,			1.0, 0.0,

								c_volume_per_volume,	0,0,
								u_vpv_volume_percent,	1.0, 0.0,

								c_mass,					0,0,
								u_m_kg,					1.0, 0.0,
								u_m_lbs,				2.204622621849, 0.0,
								u_m_g,					1000.0, 0.0,
								u_m_metric_tons,		0.001, 0.0,
								u_m_short_tons,			0.001102311310924, 0.0,
								u_m_long_tons,			0.0009842065276111, 0.0,
								u_m_ounce,				35.27396194958, 0.0,

								c_mass_flow,			0,0,
								u_mfr_kg_sec,			1.0, 0.0,
								u_mfr_kg_min,			60.0, 0.0,
								u_mfr_kg_hr,			3600.0, 0.0,
								u_mfr_kg_day,			86400.0, 0.0,
								u_mfr_g_sec,			1000.0, 0.0,
								u_mfr_g_min,			60000.0, 0.0,
								u_mfr_g_hr,				3600000.0, 0.0,
								u_mfr_metric_tons_min,	60.0*0.001, 0.0,
								u_mfr_metric_tons_hr,	3600.0*0.001, 0.0,
								u_mfr_metric_tons_day,	86400.0*0.001, 0.0,
								u_mfr_lbs_sec,			2.204622621849, 0.0,
								u_mfr_lbs_min,			60.0*2.204622621849, 0.0,
								u_mfr_lbs_hr,			3600.0*2.204622621849, 0.0,
								u_mfr_lbs_day,			86400.0*2.204622621849, 0.0,
								u_mfr_short_tons_min,	60.0*0.001102311310924, 0.0,
								u_mfr_short_tons_hr,	3600.0*0.001102311310924, 0.0,
								u_mfr_short_tons_day,	86400.0*0.001102311310924, 0.0,
								u_mfr_long_tons_hr,		3600.0*0.0009842065276111, 0.0,
								u_mfr_long_tons_day,	86400.0*0.0009842065276111, 0.0,

								c_mass_per_volume,		0,0,
								u_mpv_kg_cm,			1.0, 0.0,
								u_mpv_g_cc,				1.0/1000.0, 0.0,
								u_mpv_g_mL,				1.0/1000.0, 0.0,
								u_mpv_lbs_gal,			8.345404474089843/1000.0, 0.0,
								u_mpv_lbs_cf,			62.427962033560895/1000.0, 0.0,
								u_mpv_kg_L,				1.0/1000.0, 0.0,
								u_mpv_g_L,				1000.0/1000.0, 0.0,
								u_mpv_lbs_ci,			0.03612729162184624/1000.0, 0.0,
								u_mpv_short_tons_cyard,	0.8427774674338713/1000.0, 0.0,

								c_time,					0,0,
								u_time_sec,				1.0, 0.0,
								u_time_min,				60.0, 0.0,
								u_time_hr,				3600.0, 0.0,
								u_time_day,				86400.0, 0.0,

								0,0,0 };


//#pragma DATA_SECTION(MASTER_UNITS_STR,"TABLES")
const Uint16 MASTER_UNITS_STR[]={
						0x100|	c_volume,				'V','O','L','U','M','E',' ',0,
								u_v_barrel,				'b','b','l',' ',' ',' ',' ',0,
								u_v_bbl_liq,			'b','b','l',' ',' ',' ',' ',0,
								u_v_gal,				'g','a','l',' ',' ',' ',' ',0,
								u_v_L,			    	'L',' ',' ',' ',' ',' ',' ',0,
								u_v_imp_gal,			'I','g','a','l',' ',' ',' ',0,
								u_v_cm,					'm','3',' ',' ',' ',' ',' ',0,
								u_v_bushel,				'b','s','h',' ',' ',' ',' ',0,
								u_v_cyard,				'y','d','3',' ',' ',' ',' ',0,
								u_v_cf,					'f','t','3',' ',' ',' ',' ',0,
								u_v_ci,					'i','n','3',' ',' ',' ',' ',0,
								u_v_N_cm,				'N','m','3',' ',' ',' ',' ',0,
								u_v_N_L,				'N','L',' ',' ',' ',' ',' ',0,
								u_v_scf,				'S','c','f',' ',' ',' ',' ',0,
								u_v_hectoliter,			'h','L',' ',' ',' ',' ',' ',0,
								u_v_mscf,				'M','S','c','f',' ',' ',' ',0,

						0x100|	c_volumetric_flow,		'V','F','R',' ',' ',' ',' ',0,
								u_vfr_barrel_day,		'b','b','l','/','d','y',' ',0,
								u_vfr_Mgal_day,			'M','g','a','l','/','d','y',0,
								u_vfr_ML_day,			'M','L','/','d','y',' ',' ',0,
								u_vfr_cf_day,			'c','f','/','d','y',' ',' ',0,
								u_vfr_cm_day,			'm','3','/','d','y',' ',' ',0,
								u_vfr_imp_gal_day,		'I','g','a','l','/','d','y',0,
								u_vfr_gal_day,			'g','a','l','/','d','y',' ',0,
								u_vfr_mscf_day,			'M','S','c','f','/','d','y',0,
								u_vfr_scf_day,			'S','c','f','/','d','y',' ',0,
								u_vfr_scf_sec,			'S','c','f','/','s',' ',' ',0,
								u_vfr_barrel_min,		'b','b','l','/','m','i','n',0,
								u_vfr_cm_min,			'm','3','/','m','i','n',' ',0,
								u_vfr_cf_min,			'c','f','/','m','i','n',' ',0,
								u_vfr_imp_gal_min,		'I','g','l','/','m','i','n',0,
								u_vfr_gal_min,			'g','a','l','/','m','i','n',0,
								u_vfr_L_min,			'L','/','m','i','n',' ',' ',0,
								u_vfr_scf_min,			'S','c','f','/','m','i','n',0,
								u_vfr_cm_hr,			'm','3','/','h','r',' ',' ',0,
								u_vfr_imp_gal_hr,		'I','g','a','l','/','h','r',0,
								u_vfr_N_cm_hr,			'N','m','3','/','h','r',' ',0,
								u_vfr_N_cm_sec,			'N','m','3','/','s',' ',' ',0,
								u_vfr_N_cm_day,			'N','m','3','/','d','a','y',0,
								u_vfr_N_L_hr,			'N','L','/','h','r',' ',' ',0,
								u_vfr_cf_hr,			'c','f','/','h','r',' ',' ',0,
								u_vfr_barrel_hr,		'b','b','l','/','h','r',' ',0,
								u_vfr_gal_hr,			'g','a','l','/','h','r',' ',0,
								u_vfr_L_hr,				'L','/','h','r',' ',' ',' ',0,
								u_vfr_gal_sec,			'g','a','l','/','s',' ',' ',0,
								u_vfr_L_sec,			'L','/','s',' ',' ',' ',' ',0,
								u_vfr_cf_sec,			'c','f','/','s',' ',' ',' ',0,
								u_vfr_cm_sec,			'm','3','/','s',' ',' ',' ',0,
								u_vfr_barrel_sec,		'b','b','l','/','s',' ',' ',0,
								u_vfr_imp_gal_sec,		'I','g','a','l','/','s',' ',0,

						0x100|	c_temperature,			'T','E','M','P',' ',' ',' ',0,
								u_temp_C,				0xDF,'C',' ',' ',' ',' ',' ',0,
								u_temp_F,				0xDF,'F',' ',' ',' ',' ',' ',0,
								u_temp_R,				0xDF,'R',' ',' ',' ',' ',' ',0,
								u_temp_K,				' ','K',' ',' ',' ',' ',' ',0,

						0x100|	c_concentration,		'C','O','N','C',' ',' ',' ',0,
								u_conc_perc_solid_per_weight,	'%',' ',' ',' ',' ',' ',' ',0,
								u_conc_perc_solid_per_vol,		'%',' ',' ',' ',' ',' ',' ',0,
								u_conc_ppm,				'p','p','m',' ',' ',' ',' ',0,
								u_conc_ppb,				'p','p','b',' ',' ',' ',' ',0,

						0x100|	c_frequency,			'F','R','E','Q',' ',' ',' ',0,
								u_freq_hertz,			'H','z',' ',' ',' ',' ',' ',0,
								u_mfgr_specific_MHz,	'M','H','z',' ',' ',' ',' ',0,

						0x100|	c_pressure,				'P','R','E','S','S',' ',' ',0,
								u_p_atm,				'a','t','m',' ',' ',' ',' ',0,
								u_p_in_H2O_68F,			'i','n','H','2','O','6','8',0,
								u_p_in_Hg_0C,			'i','n','H','g','0','C',' ',0,
								u_p_ft_H2O_68F,			'f','t','H','2','O','6','8',0,
								u_p_mm_H2O_68F,			'm','m','H','2','O','6','8',0,
								u_p_mm_Hg_0C,			'm','m','H','g',' ',' ',' ',0,
								u_p_psi,				'p','s','i',' ',' ',' ',' ',0,
								u_p_bar,				'b',' ',' ',' ',' ',' ',' ',0,
								u_p_mbar,				'm','b',' ',' ',' ',' ',' ',0,
								u_p_g_scm,				'g','/','c','m','2',' ',' ',0,
								u_p_kg_scm,				'k','g','/','c','m','2',' ',0,
								u_p_Pa,					'P','a',' ',' ',' ',' ',' ',0,
								u_p_kPa,				'k','P','a',' ',' ',' ',' ',0,
								u_p_torr,				't','o','r','r',' ',' ',' ',0,
								u_p_in_H2O_60F,			'i','n','H','2','O','6','0',0,
								u_p_cm_H2O_4C,			'c','m','H','2','O','4','C',0,
								u_p_m_H2O_4C,			'm','H','2','O','4','C',' ',0,
								u_p_cm_Hg_0C,			'c','m','H','g','0','C',' ',0,
								u_p_lbs_sf,				'p','s','f',' ',' ',' ',' ',0,
								u_p_hectoPa,			'h','P','a',' ',' ',' ',' ',0,
								u_p_psia,				'p','s','i','a',' ',' ',' ',0,
								u_p_kg_sm,				'k','g','/','m','2',' ',' ',0,
								u_p_ft_H2O_4C,			'f','t','H','2','O','4','C',0,
								u_p_ft_H2O_60F,			'f','t','H','2','O','6','0',0,
								u_p_m_Hg_0C,			'm','H','g','0','C',' ',' ',0,
								u_p_MPa,				'M','P','a',' ',' ',' ',' ',0,
								u_p_in_H2O_4C,			'i','n','H','2','O','4','C',0,
								u_p_mm_H2O_4C,			'm','m','H','2','O','4','C',0,

						0x100|	c_emf,					'E','M','F',' ',' ',' ',' ',0,
								u_emf_mV,				'm','V',' ',' ',' ',' ',' ',0,
								u_emf_V,				'V',' ',' ',' ',' ',' ',' ',0,

						0x100|	c_current,				'C','U','R','R','E','N','T',0,
								u_cur_mA,				'm','A',' ',' ',' ',' ',' ',0,

						0x100|	c_analytical,			'A','N','A',' ',' ',' ',' ',0,
								u_ana_percent,			'%',' ',' ',' ',' ',' ',' ',0,
					u_mfgr_specific_perc_per_API_60F,	'%','/','A','P','I',' ',' ',0,
					u_mfgr_specific_perc_per_kgm3_15C,	'%','/','k','g','/','m','3',0,

						0x100|	c_volume_per_volume,	'V','P','V',' ',' ',' ',' ',0,
								u_vpv_volume_percent,	'%',' ',' ',' ',' ',' ',' ',0,

						0x100|	c_mass,					'M','A','S','S',' ',' ',' ',0,
								u_m_kg,					'k','g',' ',' ',' ',' ',' ',0,
								u_m_lbs,				'l','b','s',' ',' ',' ',' ',0,
								u_m_g,					'g',' ',' ',' ',' ',' ',' ',0,
								u_m_metric_tons,		't',' ',' ',' ',' ',' ',' ',0,
								u_m_short_tons,			't','s',' ',' ',' ',' ',' ',0,
								u_m_long_tons,			't','l',' ',' ',' ',' ',' ',0,
								u_m_ounce,				'o','z',' ',' ',' ',' ',' ',0,

						0x100|	c_mass_flow,			'M','F','R',' ',' ',' ',' ',0,
								u_mfr_g_sec,			'g','/','s',' ',' ',' ',' ',0,
								u_mfr_g_min,			'g','/','m','i','n',' ',' ',0,
								u_mfr_g_hr,				'g','/','h','r',' ',' ',' ',0,
								u_mfr_kg_sec,			'k','g','/','s',' ',' ',' ',0,
								u_mfr_kg_min,			'k','g','/','m','i','n',' ',0,
								u_mfr_kg_hr,			'k','g','/','h','r',' ',' ',0,
								u_mfr_kg_day,			'k','g','/','d','y',' ',' ',0,
								u_mfr_metric_tons_min,	't','/','m','i','n',' ',' ',0,
								u_mfr_metric_tons_hr,	't','/','h','r',' ',' ',' ',0,
								u_mfr_metric_tons_day,	't','/','d','y',' ',' ',' ',0,
								u_mfr_lbs_sec,			'l','b','s','/','s',' ',' ',0,
								u_mfr_lbs_min,			'l','b','s','/','m','i','n',0,
								u_mfr_lbs_hr,			'l','b','s','/','h','r',' ',0,
								u_mfr_lbs_day,			'l','b','s','/','d','y',' ',0,
								u_mfr_short_tons_min,	't','s','/','m','i','n',' ',0,
								u_mfr_short_tons_hr,	't','s','/','h','r',' ',' ',0,
								u_mfr_short_tons_day,	't','s','/','d','y',' ',' ',0,
								u_mfr_long_tons_hr,		't','l','/','h','r',' ',' ',0,
								u_mfr_long_tons_day,	't','l','/','d','y',' ',' ',0,

						0x100|	c_mass_per_volume,		'M','P','V',' ',' ',' ',' ',0,
								u_mpv_kg_cm,			'k','g','/','m','3',' ',' ',0,
								u_mpv_kg_cm_15C,		'k','g','/','m','3','1','5',0,
								u_mpv_kg_cm_60F,		'k','g','/','m','3','6','0',0,
								u_mpv_deg_API,			0xDF,'A','P','I',' ',' ',' ',0,
								u_mpv_deg_API_60F,		0xDF,'A','P','I','6','0','F',0,
								u_mpv_deg_API_15C,		0xDF,'A','P','I','1','5','C',0,
								u_mpv_sg,				's','g',' ',' ',' ',' ',' ',0,
								u_mpv_sg_15C,			's','g','1','5','C',' ',' ',0,
								u_mpv_sg_60F,			's','g','6','0','F',' ',' ',0,
								u_mpv_g_cc,				'g','/','c','c',' ',' ',' ',0,
								u_mpv_g_mL,				'g','/','m','L',' ',' ',' ',0,
								u_mpv_g_L,				'g','/','L',' ',' ',' ',' ',0,
								u_mpv_kg_L,				'k','g','/','L',' ',' ',' ',0,
								u_mpv_lbs_gal,			'l','b','s','/','g','a','l',0,
								u_mpv_lbs_cf,			'l','b','s','/','c','f',' ',0,
								u_mpv_lbs_cf_60F,		'l','b','s','/','c','f','6',0,
								u_mpv_lbs_ci,			'l','b','s','/','i','n','3',0,
								u_mpv_short_tons_cyard,	't','s','/','y','d','3',' ',0,

						0x100|	c_time,					'T','I','M','E',' ',' ',' ',0,
								u_time_sec,				's',' ',' ',' ',' ',' ',' ',0,
								u_time_min,				'm','i','n',' ',' ',' ',' ',0,
								u_time_hr,				'h','r',' ',' ',' ',' ',' ',0,
								u_time_day,				'd','y',' ',' ',' ',' ',' ',0,
								u_mfgr_specific_ct,		'c','t',' ',' ',' ',' ',' ',0,

						0x100|	c_not_classified,		'M','I','S','C',' ',' ',' ',0,
								u_ana_percent,			'%',' ',' ',' ',' ',' ',' ',0,
								u_mfgr_specific_bps,	'B','d',' ',' ',' ',' ',' ',0,
								u_mfgr_specific_V_per_MHz,	'V','/','M','H','z',' ',' ',0,

						0x100|	c_none,					' ',' ',' ',' ',' ',' ',' ',0};


#endif
#endif
