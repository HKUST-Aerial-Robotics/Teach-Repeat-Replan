#ifndef MOSEK_H
#define MOSEK_H

/******************************************************************************
 ** Module : mosek.h
 **
 ** Generated 2016
 **
 ** Copyright (c) 1998-2016 MOSEK ApS, Denmark.
 **
 ** All rights reserved
 **
 ******************************************************************************/
/* 
 The content of this file is subject to copyright. However, it may free
 of charge be redistributed in identical form --- i.e. with no changes of
 the wording --- for any legitimate purpose.
*/ 


#include <stdlib.h>
#include <stdio.h>

#define MSK_VERSION_MAJOR    7
#define MSK_VERSION_MINOR    1
#define MSK_VERSION_BUILD    0
#define MSK_VERSION_REVISION 51
#define MSK_VERSION_STATE    ""


#ifndef MSKCONST
#define MSKCONST const
#endif


#define MSK_INFINITY 1.0e30

/* BEGIN PLATFORM SPECIFIC DEFINITIONS (linux64x86) */
#define MSKAPI   
#define MSKAPIVA 
/* END   PLATFORM SPECIFIC DEFINITIONS (linux64x86) */


/* Enums and constants */
/* namespace mosek { */
enum MSKsolveform_enum {
  MSK_SOLVE_BEGIN  = 0,
  MSK_SOLVE_END    = 3,

  MSK_SOLVE_FREE   = 0,
  MSK_SOLVE_PRIMAL = 1,
  MSK_SOLVE_DUAL   = 2
};

enum MSKproblemitem_enum {
  MSK_PI_BEGIN = 0,
  MSK_PI_END  = 3,

  MSK_PI_VAR  = 0,
  MSK_PI_CON  = 1,
  MSK_PI_CONE = 2
};

enum MSKaccmode_enum {
  MSK_ACC_BEGIN = 0,
  MSK_ACC_END = 2,

  MSK_ACC_VAR = 0,
  MSK_ACC_CON = 1
};

enum MSKsensitivitytype_enum {
  MSK_SENSITIVITY_TYPE_BEGIN             = 0,
  MSK_SENSITIVITY_TYPE_END               = 2,

  MSK_SENSITIVITY_TYPE_BASIS             = 0,
  MSK_SENSITIVITY_TYPE_OPTIMAL_PARTITION = 1
};

enum MSKuplo_enum {
  MSK_UPLO_BEGIN = 0,
  MSK_UPLO_END = 2,

  MSK_UPLO_LO = 0,
  MSK_UPLO_UP = 1
};

enum MSKintpnthotstart_enum {
  MSK_INTPNT_HOTSTART_BEGIN       = 0,
  MSK_INTPNT_HOTSTART_END         = 4,

  MSK_INTPNT_HOTSTART_NONE        = 0,
  MSK_INTPNT_HOTSTART_PRIMAL      = 1,
  MSK_INTPNT_HOTSTART_DUAL        = 2,
  MSK_INTPNT_HOTSTART_PRIMAL_DUAL = 3
};

enum MSKsparam_enum {
  MSK_SPAR_BEGIN                     = 0,
  MSK_SPAR_END                       = 26,

  MSK_SPAR_BAS_SOL_FILE_NAME         = 0,
  MSK_SPAR_DATA_FILE_NAME            = 1,
  MSK_SPAR_DEBUG_FILE_NAME           = 2,
  MSK_SPAR_FEASREPAIR_NAME_PREFIX    = 3,
  MSK_SPAR_FEASREPAIR_NAME_SEPARATOR = 4,
  MSK_SPAR_FEASREPAIR_NAME_WSUMVIOL  = 5,
  MSK_SPAR_INT_SOL_FILE_NAME         = 6,
  MSK_SPAR_ITR_SOL_FILE_NAME         = 7,
  MSK_SPAR_MIO_DEBUG_STRING          = 8,
  MSK_SPAR_PARAM_COMMENT_SIGN        = 9,
  MSK_SPAR_PARAM_READ_FILE_NAME      = 10,
  MSK_SPAR_PARAM_WRITE_FILE_NAME     = 11,
  MSK_SPAR_READ_MPS_BOU_NAME         = 12,
  MSK_SPAR_READ_MPS_OBJ_NAME         = 13,
  MSK_SPAR_READ_MPS_RAN_NAME         = 14,
  MSK_SPAR_READ_MPS_RHS_NAME         = 15,
  MSK_SPAR_SENSITIVITY_FILE_NAME     = 16,
  MSK_SPAR_SENSITIVITY_RES_FILE_NAME = 17,
  MSK_SPAR_SOL_FILTER_XC_LOW         = 18,
  MSK_SPAR_SOL_FILTER_XC_UPR         = 19,
  MSK_SPAR_SOL_FILTER_XX_LOW         = 20,
  MSK_SPAR_SOL_FILTER_XX_UPR         = 21,
  MSK_SPAR_STAT_FILE_NAME            = 22,
  MSK_SPAR_STAT_KEY                  = 23,
  MSK_SPAR_STAT_NAME                 = 24,
  MSK_SPAR_WRITE_LP_GEN_VAR_NAME     = 25
};

enum MSKiparam_enum {
  MSK_IPAR_BEGIN                                 = 0,
  MSK_IPAR_END                                   = 207,

  MSK_IPAR_ALLOC_ADD_QNZ                         = 0,
  MSK_IPAR_ANA_SOL_BASIS                         = 1,
  MSK_IPAR_ANA_SOL_PRINT_VIOLATED                = 2,
  MSK_IPAR_AUTO_SORT_A_BEFORE_OPT                = 3,
  MSK_IPAR_AUTO_UPDATE_SOL_INFO                  = 4,
  MSK_IPAR_BASIS_SOLVE_USE_PLUS_ONE              = 5,
  MSK_IPAR_BI_CLEAN_OPTIMIZER                    = 6,
  MSK_IPAR_BI_IGNORE_MAX_ITER                    = 7,
  MSK_IPAR_BI_IGNORE_NUM_ERROR                   = 8,
  MSK_IPAR_BI_MAX_ITERATIONS                     = 9,
  MSK_IPAR_CACHE_LICENSE                         = 10,
  MSK_IPAR_CHECK_CONVEXITY                       = 11,
  MSK_IPAR_COMPRESS_STATFILE                     = 12,
  MSK_IPAR_CONCURRENT_NUM_OPTIMIZERS             = 13,
  MSK_IPAR_CONCURRENT_PRIORITY_DUAL_SIMPLEX      = 14,
  MSK_IPAR_CONCURRENT_PRIORITY_FREE_SIMPLEX      = 15,
  MSK_IPAR_CONCURRENT_PRIORITY_INTPNT            = 16,
  MSK_IPAR_CONCURRENT_PRIORITY_PRIMAL_SIMPLEX    = 17,
  MSK_IPAR_FEASREPAIR_OPTIMIZE                   = 18,
  MSK_IPAR_INFEAS_GENERIC_NAMES                  = 19,
  MSK_IPAR_INFEAS_PREFER_PRIMAL                  = 20,
  MSK_IPAR_INFEAS_REPORT_AUTO                    = 21,
  MSK_IPAR_INFEAS_REPORT_LEVEL                   = 22,
  MSK_IPAR_INTPNT_BASIS                          = 23,
  MSK_IPAR_INTPNT_DIFF_STEP                      = 24,
  MSK_IPAR_INTPNT_FACTOR_DEBUG_LVL               = 25,
  MSK_IPAR_INTPNT_FACTOR_METHOD                  = 26,
  MSK_IPAR_INTPNT_HOTSTART                       = 27,
  MSK_IPAR_INTPNT_MAX_ITERATIONS                 = 28,
  MSK_IPAR_INTPNT_MAX_NUM_COR                    = 29,
  MSK_IPAR_INTPNT_MAX_NUM_REFINEMENT_STEPS       = 30,
  MSK_IPAR_INTPNT_OFF_COL_TRH                    = 31,
  MSK_IPAR_INTPNT_ORDER_METHOD                   = 32,
  MSK_IPAR_INTPNT_REGULARIZATION_USE             = 33,
  MSK_IPAR_INTPNT_SCALING                        = 34,
  MSK_IPAR_INTPNT_SOLVE_FORM                     = 35,
  MSK_IPAR_INTPNT_STARTING_POINT                 = 36,
  MSK_IPAR_LIC_TRH_EXPIRY_WRN                    = 37,
  MSK_IPAR_LICENSE_DEBUG                         = 38,
  MSK_IPAR_LICENSE_PAUSE_TIME                    = 39,
  MSK_IPAR_LICENSE_SUPPRESS_EXPIRE_WRNS          = 40,
  MSK_IPAR_LICENSE_WAIT                          = 41,
  MSK_IPAR_LOG                                   = 42,
  MSK_IPAR_LOG_BI                                = 43,
  MSK_IPAR_LOG_BI_FREQ                           = 44,
  MSK_IPAR_LOG_CHECK_CONVEXITY                   = 45,
  MSK_IPAR_LOG_CONCURRENT                        = 46,
  MSK_IPAR_LOG_CUT_SECOND_OPT                    = 47,
  MSK_IPAR_LOG_EXPAND                            = 48,
  MSK_IPAR_LOG_FACTOR                            = 49,
  MSK_IPAR_LOG_FEAS_REPAIR                       = 50,
  MSK_IPAR_LOG_FILE                              = 51,
  MSK_IPAR_LOG_HEAD                              = 52,
  MSK_IPAR_LOG_INFEAS_ANA                        = 53,
  MSK_IPAR_LOG_INTPNT                            = 54,
  MSK_IPAR_LOG_MIO                               = 55,
  MSK_IPAR_LOG_MIO_FREQ                          = 56,
  MSK_IPAR_LOG_NONCONVEX                         = 57,
  MSK_IPAR_LOG_OPTIMIZER                         = 58,
  MSK_IPAR_LOG_ORDER                             = 59,
  MSK_IPAR_LOG_PARAM                             = 60,
  MSK_IPAR_LOG_PRESOLVE                          = 61,
  MSK_IPAR_LOG_RESPONSE                          = 62,
  MSK_IPAR_LOG_SENSITIVITY                       = 63,
  MSK_IPAR_LOG_SENSITIVITY_OPT                   = 64,
  MSK_IPAR_LOG_SIM                               = 65,
  MSK_IPAR_LOG_SIM_FREQ                          = 66,
  MSK_IPAR_LOG_SIM_MINOR                         = 67,
  MSK_IPAR_LOG_SIM_NETWORK_FREQ                  = 68,
  MSK_IPAR_LOG_STORAGE                           = 69,
  MSK_IPAR_MAX_NUM_WARNINGS                      = 70,
  MSK_IPAR_MIO_BRANCH_DIR                        = 71,
  MSK_IPAR_MIO_BRANCH_PRIORITIES_USE             = 72,
  MSK_IPAR_MIO_CONSTRUCT_SOL                     = 73,
  MSK_IPAR_MIO_CONT_SOL                          = 74,
  MSK_IPAR_MIO_CUT_CG                            = 75,
  MSK_IPAR_MIO_CUT_CMIR                          = 76,
  MSK_IPAR_MIO_CUT_LEVEL_ROOT                    = 77,
  MSK_IPAR_MIO_CUT_LEVEL_TREE                    = 78,
  MSK_IPAR_MIO_FEASPUMP_LEVEL                    = 79,
  MSK_IPAR_MIO_HEURISTIC_LEVEL                   = 80,
  MSK_IPAR_MIO_HOTSTART                          = 81,
  MSK_IPAR_MIO_KEEP_BASIS                        = 82,
  MSK_IPAR_MIO_LOCAL_BRANCH_NUMBER               = 83,
  MSK_IPAR_MIO_MAX_NUM_BRANCHES                  = 84,
  MSK_IPAR_MIO_MAX_NUM_RELAXS                    = 85,
  MSK_IPAR_MIO_MAX_NUM_SOLUTIONS                 = 86,
  MSK_IPAR_MIO_MODE                              = 87,
  MSK_IPAR_MIO_MT_USER_CB                        = 88,
  MSK_IPAR_MIO_NODE_OPTIMIZER                    = 89,
  MSK_IPAR_MIO_NODE_SELECTION                    = 90,
  MSK_IPAR_MIO_OPTIMIZER_MODE                    = 91,
  MSK_IPAR_MIO_PRESOLVE_AGGREGATE                = 92,
  MSK_IPAR_MIO_PRESOLVE_PROBING                  = 93,
  MSK_IPAR_MIO_PRESOLVE_USE                      = 94,
  MSK_IPAR_MIO_PROBING_LEVEL                     = 95,
  MSK_IPAR_MIO_RINS_MAX_NODES                    = 96,
  MSK_IPAR_MIO_ROOT_OPTIMIZER                    = 97,
  MSK_IPAR_MIO_STRONG_BRANCH                     = 98,
  MSK_IPAR_MIO_USE_MULTITHREADED_OPTIMIZER       = 99,
  MSK_IPAR_MT_SPINCOUNT                          = 100,
  MSK_IPAR_NONCONVEX_MAX_ITERATIONS              = 101,
  MSK_IPAR_NUM_THREADS                           = 102,
  MSK_IPAR_OPF_MAX_TERMS_PER_LINE                = 103,
  MSK_IPAR_OPF_WRITE_HEADER                      = 104,
  MSK_IPAR_OPF_WRITE_HINTS                       = 105,
  MSK_IPAR_OPF_WRITE_PARAMETERS                  = 106,
  MSK_IPAR_OPF_WRITE_PROBLEM                     = 107,
  MSK_IPAR_OPF_WRITE_SOL_BAS                     = 108,
  MSK_IPAR_OPF_WRITE_SOL_ITG                     = 109,
  MSK_IPAR_OPF_WRITE_SOL_ITR                     = 110,
  MSK_IPAR_OPF_WRITE_SOLUTIONS                   = 111,
  MSK_IPAR_OPTIMIZER                             = 112,
  MSK_IPAR_PARAM_READ_CASE_NAME                  = 113,
  MSK_IPAR_PARAM_READ_IGN_ERROR                  = 114,
  MSK_IPAR_PRESOLVE_ELIM_FILL                    = 115,
  MSK_IPAR_PRESOLVE_ELIMINATOR_MAX_NUM_TRIES     = 116,
  MSK_IPAR_PRESOLVE_ELIMINATOR_USE               = 117,
  MSK_IPAR_PRESOLVE_LEVEL                        = 118,
  MSK_IPAR_PRESOLVE_LINDEP_ABS_WORK_TRH          = 119,
  MSK_IPAR_PRESOLVE_LINDEP_REL_WORK_TRH          = 120,
  MSK_IPAR_PRESOLVE_LINDEP_USE                   = 121,
  MSK_IPAR_PRESOLVE_MAX_NUM_REDUCTIONS           = 122,
  MSK_IPAR_PRESOLVE_USE                          = 123,
  MSK_IPAR_PRIMAL_REPAIR_OPTIMIZER               = 124,
  MSK_IPAR_QO_SEPARABLE_REFORMULATION            = 125,
  MSK_IPAR_READ_ANZ                              = 126,
  MSK_IPAR_READ_CON                              = 127,
  MSK_IPAR_READ_CONE                             = 128,
  MSK_IPAR_READ_DATA_COMPRESSED                  = 129,
  MSK_IPAR_READ_DATA_FORMAT                      = 130,
  MSK_IPAR_READ_DEBUG                            = 131,
  MSK_IPAR_READ_KEEP_FREE_CON                    = 132,
  MSK_IPAR_READ_LP_DROP_NEW_VARS_IN_BOU          = 133,
  MSK_IPAR_READ_LP_QUOTED_NAMES                  = 134,
  MSK_IPAR_READ_MPS_FORMAT                       = 135,
  MSK_IPAR_READ_MPS_KEEP_INT                     = 136,
  MSK_IPAR_READ_MPS_OBJ_SENSE                    = 137,
  MSK_IPAR_READ_MPS_RELAX                        = 138,
  MSK_IPAR_READ_MPS_WIDTH                        = 139,
  MSK_IPAR_READ_QNZ                              = 140,
  MSK_IPAR_READ_TASK_IGNORE_PARAM                = 141,
  MSK_IPAR_READ_VAR                              = 142,
  MSK_IPAR_SENSITIVITY_ALL                       = 143,
  MSK_IPAR_SENSITIVITY_OPTIMIZER                 = 144,
  MSK_IPAR_SENSITIVITY_TYPE                      = 145,
  MSK_IPAR_SIM_BASIS_FACTOR_USE                  = 146,
  MSK_IPAR_SIM_DEGEN                             = 147,
  MSK_IPAR_SIM_DUAL_CRASH                        = 148,
  MSK_IPAR_SIM_DUAL_PHASEONE_METHOD              = 149,
  MSK_IPAR_SIM_DUAL_RESTRICT_SELECTION           = 150,
  MSK_IPAR_SIM_DUAL_SELECTION                    = 151,
  MSK_IPAR_SIM_EXPLOIT_DUPVEC                    = 152,
  MSK_IPAR_SIM_HOTSTART                          = 153,
  MSK_IPAR_SIM_HOTSTART_LU                       = 154,
  MSK_IPAR_SIM_INTEGER                           = 155,
  MSK_IPAR_SIM_MAX_ITERATIONS                    = 156,
  MSK_IPAR_SIM_MAX_NUM_SETBACKS                  = 157,
  MSK_IPAR_SIM_NON_SINGULAR                      = 158,
  MSK_IPAR_SIM_PRIMAL_CRASH                      = 159,
  MSK_IPAR_SIM_PRIMAL_PHASEONE_METHOD            = 160,
  MSK_IPAR_SIM_PRIMAL_RESTRICT_SELECTION         = 161,
  MSK_IPAR_SIM_PRIMAL_SELECTION                  = 162,
  MSK_IPAR_SIM_REFACTOR_FREQ                     = 163,
  MSK_IPAR_SIM_REFORMULATION                     = 164,
  MSK_IPAR_SIM_SAVE_LU                           = 165,
  MSK_IPAR_SIM_SCALING                           = 166,
  MSK_IPAR_SIM_SCALING_METHOD                    = 167,
  MSK_IPAR_SIM_SOLVE_FORM                        = 168,
  MSK_IPAR_SIM_STABILITY_PRIORITY                = 169,
  MSK_IPAR_SIM_SWITCH_OPTIMIZER                  = 170,
  MSK_IPAR_SOL_FILTER_KEEP_BASIC                 = 171,
  MSK_IPAR_SOL_FILTER_KEEP_RANGED                = 172,
  MSK_IPAR_SOL_READ_NAME_WIDTH                   = 173,
  MSK_IPAR_SOL_READ_WIDTH                        = 174,
  MSK_IPAR_SOLUTION_CALLBACK                     = 175,
  MSK_IPAR_TIMING_LEVEL                          = 176,
  MSK_IPAR_WARNING_LEVEL                         = 177,
  MSK_IPAR_WRITE_BAS_CONSTRAINTS                 = 178,
  MSK_IPAR_WRITE_BAS_HEAD                        = 179,
  MSK_IPAR_WRITE_BAS_VARIABLES                   = 180,
  MSK_IPAR_WRITE_DATA_COMPRESSED                 = 181,
  MSK_IPAR_WRITE_DATA_FORMAT                     = 182,
  MSK_IPAR_WRITE_DATA_PARAM                      = 183,
  MSK_IPAR_WRITE_FREE_CON                        = 184,
  MSK_IPAR_WRITE_GENERIC_NAMES                   = 185,
  MSK_IPAR_WRITE_GENERIC_NAMES_IO                = 186,
  MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_CONIC_ITEMS = 187,
  MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_ITEMS       = 188,
  MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_NL_ITEMS    = 189,
  MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_PSD_ITEMS   = 190,
  MSK_IPAR_WRITE_INT_CONSTRAINTS                 = 191,
  MSK_IPAR_WRITE_INT_HEAD                        = 192,
  MSK_IPAR_WRITE_INT_VARIABLES                   = 193,
  MSK_IPAR_WRITE_LP_LINE_WIDTH                   = 194,
  MSK_IPAR_WRITE_LP_QUOTED_NAMES                 = 195,
  MSK_IPAR_WRITE_LP_STRICT_FORMAT                = 196,
  MSK_IPAR_WRITE_LP_TERMS_PER_LINE               = 197,
  MSK_IPAR_WRITE_MPS_INT                         = 198,
  MSK_IPAR_WRITE_PRECISION                       = 199,
  MSK_IPAR_WRITE_SOL_BARVARIABLES                = 200,
  MSK_IPAR_WRITE_SOL_CONSTRAINTS                 = 201,
  MSK_IPAR_WRITE_SOL_HEAD                        = 202,
  MSK_IPAR_WRITE_SOL_IGNORE_INVALID_NAMES        = 203,
  MSK_IPAR_WRITE_SOL_VARIABLES                   = 204,
  MSK_IPAR_WRITE_TASK_INC_SOL                    = 205,
  MSK_IPAR_WRITE_XML_MODE                        = 206
};

enum MSKsolsta_enum {
  MSK_SOL_STA_BEGIN                   = 0,
  MSK_SOL_STA_END                     = 16,

  MSK_SOL_STA_UNKNOWN                 = 0,
  MSK_SOL_STA_OPTIMAL                 = 1,
  MSK_SOL_STA_PRIM_FEAS               = 2,
  MSK_SOL_STA_DUAL_FEAS               = 3,
  MSK_SOL_STA_PRIM_AND_DUAL_FEAS      = 4,
  MSK_SOL_STA_PRIM_INFEAS_CER         = 5,
  MSK_SOL_STA_DUAL_INFEAS_CER         = 6,
  MSK_SOL_STA_NEAR_OPTIMAL            = 8,
  MSK_SOL_STA_NEAR_PRIM_FEAS          = 9,
  MSK_SOL_STA_NEAR_DUAL_FEAS          = 10,
  MSK_SOL_STA_NEAR_PRIM_AND_DUAL_FEAS = 11,
  MSK_SOL_STA_NEAR_PRIM_INFEAS_CER    = 12,
  MSK_SOL_STA_NEAR_DUAL_INFEAS_CER    = 13,
  MSK_SOL_STA_INTEGER_OPTIMAL         = 14,
  MSK_SOL_STA_NEAR_INTEGER_OPTIMAL    = 15
};

enum MSKobjsense_enum {
  MSK_OBJECTIVE_SENSE_BEGIN    = 0,
  MSK_OBJECTIVE_SENSE_END      = 2,

  MSK_OBJECTIVE_SENSE_MINIMIZE = 0,
  MSK_OBJECTIVE_SENSE_MAXIMIZE = 1
};

enum MSKsolitem_enum {
  MSK_SOL_ITEM_BEGIN = 0,
  MSK_SOL_ITEM_END = 8,

  MSK_SOL_ITEM_XC  = 0,
  MSK_SOL_ITEM_XX  = 1,
  MSK_SOL_ITEM_Y   = 2,
  MSK_SOL_ITEM_SLC = 3,
  MSK_SOL_ITEM_SUC = 4,
  MSK_SOL_ITEM_SLX = 5,
  MSK_SOL_ITEM_SUX = 6,
  MSK_SOL_ITEM_SNX = 7
};

enum MSKboundkey_enum {
  MSK_BK_BEGIN = 0,
  MSK_BK_END = 5,

  MSK_BK_LO = 0,
  MSK_BK_UP = 1,
  MSK_BK_FX = 2,
  MSK_BK_FR = 3,
  MSK_BK_RA = 4
};

enum MSKbasindtype_enum {
  MSK_BI_BEGIN       = 0,
  MSK_BI_END         = 5,

  MSK_BI_NEVER       = 0,
  MSK_BI_ALWAYS      = 1,
  MSK_BI_NO_ERROR    = 2,
  MSK_BI_IF_FEASIBLE = 3,
  MSK_BI_RESERVERED  = 4
};

enum MSKbranchdir_enum {
  MSK_BRANCH_DIR_BEGIN = 0,
  MSK_BRANCH_DIR_END  = 3,

  MSK_BRANCH_DIR_FREE = 0,
  MSK_BRANCH_DIR_UP   = 1,
  MSK_BRANCH_DIR_DOWN = 2
};

enum MSKliinfitem_enum {
  MSK_LIINF_BEGIN                         = 0,
  MSK_LIINF_END                           = 14,

  MSK_LIINF_BI_CLEAN_DUAL_DEG_ITER        = 0,
  MSK_LIINF_BI_CLEAN_DUAL_ITER            = 1,
  MSK_LIINF_BI_CLEAN_PRIMAL_DEG_ITER      = 2,
  MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_DEG_ITER = 3,
  MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_ITER     = 4,
  MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_SUB_ITER = 5,
  MSK_LIINF_BI_CLEAN_PRIMAL_ITER          = 6,
  MSK_LIINF_BI_DUAL_ITER                  = 7,
  MSK_LIINF_BI_PRIMAL_ITER                = 8,
  MSK_LIINF_INTPNT_FACTOR_NUM_NZ          = 9,
  MSK_LIINF_MIO_INTPNT_ITER               = 10,
  MSK_LIINF_MIO_SIMPLEX_ITER              = 11,
  MSK_LIINF_RD_NUMANZ                     = 12,
  MSK_LIINF_RD_NUMQNZ                     = 13
};

enum MSKstreamtype_enum {
  MSK_STREAM_BEGIN = 0,
  MSK_STREAM_END = 4,

  MSK_STREAM_LOG = 0,
  MSK_STREAM_MSG = 1,
  MSK_STREAM_ERR = 2,
  MSK_STREAM_WRN = 3
};

enum MSKsimhotstart_enum {
  MSK_SIM_HOTSTART_BEGIN       = 0,
  MSK_SIM_HOTSTART_END         = 3,

  MSK_SIM_HOTSTART_NONE        = 0,
  MSK_SIM_HOTSTART_FREE        = 1,
  MSK_SIM_HOTSTART_STATUS_KEYS = 2
};

enum MSKcallbackcode_enum {
  MSK_CALLBACK_BEGIN                         = 0,
  MSK_CALLBACK_END                           = 114,

  MSK_CALLBACK_BEGIN_BI                      = 0,
  MSK_CALLBACK_BEGIN_CONCURRENT              = 1,
  MSK_CALLBACK_BEGIN_CONIC                   = 2,
  MSK_CALLBACK_BEGIN_DUAL_BI                 = 3,
  MSK_CALLBACK_BEGIN_DUAL_SENSITIVITY        = 4,
  MSK_CALLBACK_BEGIN_DUAL_SETUP_BI           = 5,
  MSK_CALLBACK_BEGIN_DUAL_SIMPLEX            = 6,
  MSK_CALLBACK_BEGIN_DUAL_SIMPLEX_BI         = 7,
  MSK_CALLBACK_BEGIN_FULL_CONVEXITY_CHECK    = 8,
  MSK_CALLBACK_BEGIN_INFEAS_ANA              = 9,
  MSK_CALLBACK_BEGIN_INTPNT                  = 10,
  MSK_CALLBACK_BEGIN_LICENSE_WAIT            = 11,
  MSK_CALLBACK_BEGIN_MIO                     = 12,
  MSK_CALLBACK_BEGIN_NETWORK_DUAL_SIMPLEX    = 13,
  MSK_CALLBACK_BEGIN_NETWORK_PRIMAL_SIMPLEX  = 14,
  MSK_CALLBACK_BEGIN_NETWORK_SIMPLEX         = 15,
  MSK_CALLBACK_BEGIN_NONCONVEX               = 16,
  MSK_CALLBACK_BEGIN_OPTIMIZER               = 17,
  MSK_CALLBACK_BEGIN_PRESOLVE                = 18,
  MSK_CALLBACK_BEGIN_PRIMAL_BI               = 19,
  MSK_CALLBACK_BEGIN_PRIMAL_DUAL_SIMPLEX     = 20,
  MSK_CALLBACK_BEGIN_PRIMAL_DUAL_SIMPLEX_BI  = 21,
  MSK_CALLBACK_BEGIN_PRIMAL_REPAIR           = 22,
  MSK_CALLBACK_BEGIN_PRIMAL_SENSITIVITY      = 23,
  MSK_CALLBACK_BEGIN_PRIMAL_SETUP_BI         = 24,
  MSK_CALLBACK_BEGIN_PRIMAL_SIMPLEX          = 25,
  MSK_CALLBACK_BEGIN_PRIMAL_SIMPLEX_BI       = 26,
  MSK_CALLBACK_BEGIN_QCQO_REFORMULATE        = 27,
  MSK_CALLBACK_BEGIN_READ                    = 28,
  MSK_CALLBACK_BEGIN_SIMPLEX                 = 29,
  MSK_CALLBACK_BEGIN_SIMPLEX_BI              = 30,
  MSK_CALLBACK_BEGIN_SIMPLEX_NETWORK_DETECT  = 31,
  MSK_CALLBACK_BEGIN_WRITE                   = 32,
  MSK_CALLBACK_CONIC                         = 33,
  MSK_CALLBACK_DUAL_SIMPLEX                  = 34,
  MSK_CALLBACK_END_BI                        = 35,
  MSK_CALLBACK_END_CONCURRENT                = 36,
  MSK_CALLBACK_END_CONIC                     = 37,
  MSK_CALLBACK_END_DUAL_BI                   = 38,
  MSK_CALLBACK_END_DUAL_SENSITIVITY          = 39,
  MSK_CALLBACK_END_DUAL_SETUP_BI             = 40,
  MSK_CALLBACK_END_DUAL_SIMPLEX              = 41,
  MSK_CALLBACK_END_DUAL_SIMPLEX_BI           = 42,
  MSK_CALLBACK_END_FULL_CONVEXITY_CHECK      = 43,
  MSK_CALLBACK_END_INFEAS_ANA                = 44,
  MSK_CALLBACK_END_INTPNT                    = 45,
  MSK_CALLBACK_END_LICENSE_WAIT              = 46,
  MSK_CALLBACK_END_MIO                       = 47,
  MSK_CALLBACK_END_NETWORK_DUAL_SIMPLEX      = 48,
  MSK_CALLBACK_END_NETWORK_PRIMAL_SIMPLEX    = 49,
  MSK_CALLBACK_END_NETWORK_SIMPLEX           = 50,
  MSK_CALLBACK_END_NONCONVEX                 = 51,
  MSK_CALLBACK_END_OPTIMIZER                 = 52,
  MSK_CALLBACK_END_PRESOLVE                  = 53,
  MSK_CALLBACK_END_PRIMAL_BI                 = 54,
  MSK_CALLBACK_END_PRIMAL_DUAL_SIMPLEX       = 55,
  MSK_CALLBACK_END_PRIMAL_DUAL_SIMPLEX_BI    = 56,
  MSK_CALLBACK_END_PRIMAL_REPAIR             = 57,
  MSK_CALLBACK_END_PRIMAL_SENSITIVITY        = 58,
  MSK_CALLBACK_END_PRIMAL_SETUP_BI           = 59,
  MSK_CALLBACK_END_PRIMAL_SIMPLEX            = 60,
  MSK_CALLBACK_END_PRIMAL_SIMPLEX_BI         = 61,
  MSK_CALLBACK_END_QCQO_REFORMULATE          = 62,
  MSK_CALLBACK_END_READ                      = 63,
  MSK_CALLBACK_END_SIMPLEX                   = 64,
  MSK_CALLBACK_END_SIMPLEX_BI                = 65,
  MSK_CALLBACK_END_SIMPLEX_NETWORK_DETECT    = 66,
  MSK_CALLBACK_END_WRITE                     = 67,
  MSK_CALLBACK_IM_BI                         = 68,
  MSK_CALLBACK_IM_CONIC                      = 69,
  MSK_CALLBACK_IM_DUAL_BI                    = 70,
  MSK_CALLBACK_IM_DUAL_SENSIVITY             = 71,
  MSK_CALLBACK_IM_DUAL_SIMPLEX               = 72,
  MSK_CALLBACK_IM_FULL_CONVEXITY_CHECK       = 73,
  MSK_CALLBACK_IM_INTPNT                     = 74,
  MSK_CALLBACK_IM_LICENSE_WAIT               = 75,
  MSK_CALLBACK_IM_LU                         = 76,
  MSK_CALLBACK_IM_MIO                        = 77,
  MSK_CALLBACK_IM_MIO_DUAL_SIMPLEX           = 78,
  MSK_CALLBACK_IM_MIO_INTPNT                 = 79,
  MSK_CALLBACK_IM_MIO_PRESOLVE               = 80,
  MSK_CALLBACK_IM_MIO_PRIMAL_SIMPLEX         = 81,
  MSK_CALLBACK_IM_NETWORK_DUAL_SIMPLEX       = 82,
  MSK_CALLBACK_IM_NETWORK_PRIMAL_SIMPLEX     = 83,
  MSK_CALLBACK_IM_NONCONVEX                  = 84,
  MSK_CALLBACK_IM_ORDER                      = 85,
  MSK_CALLBACK_IM_PRESOLVE                   = 86,
  MSK_CALLBACK_IM_PRIMAL_BI                  = 87,
  MSK_CALLBACK_IM_PRIMAL_DUAL_SIMPLEX        = 88,
  MSK_CALLBACK_IM_PRIMAL_SENSIVITY           = 89,
  MSK_CALLBACK_IM_PRIMAL_SIMPLEX             = 90,
  MSK_CALLBACK_IM_QO_REFORMULATE             = 91,
  MSK_CALLBACK_IM_READ                       = 92,
  MSK_CALLBACK_IM_SIMPLEX                    = 93,
  MSK_CALLBACK_IM_SIMPLEX_BI                 = 94,
  MSK_CALLBACK_INTPNT                        = 95,
  MSK_CALLBACK_NEW_INT_MIO                   = 96,
  MSK_CALLBACK_NONCOVEX                      = 97,
  MSK_CALLBACK_PRIMAL_SIMPLEX                = 98,
  MSK_CALLBACK_READ_OPF                      = 99,
  MSK_CALLBACK_READ_OPF_SECTION              = 100,
  MSK_CALLBACK_UPDATE_DUAL_BI                = 101,
  MSK_CALLBACK_UPDATE_DUAL_SIMPLEX           = 102,
  MSK_CALLBACK_UPDATE_DUAL_SIMPLEX_BI        = 103,
  MSK_CALLBACK_UPDATE_NETWORK_DUAL_SIMPLEX   = 104,
  MSK_CALLBACK_UPDATE_NETWORK_PRIMAL_SIMPLEX = 105,
  MSK_CALLBACK_UPDATE_NONCONVEX              = 106,
  MSK_CALLBACK_UPDATE_PRESOLVE               = 107,
  MSK_CALLBACK_UPDATE_PRIMAL_BI              = 108,
  MSK_CALLBACK_UPDATE_PRIMAL_DUAL_SIMPLEX    = 109,
  MSK_CALLBACK_UPDATE_PRIMAL_DUAL_SIMPLEX_BI = 110,
  MSK_CALLBACK_UPDATE_PRIMAL_SIMPLEX         = 111,
  MSK_CALLBACK_UPDATE_PRIMAL_SIMPLEX_BI      = 112,
  MSK_CALLBACK_WRITE_OPF                     = 113
};

enum MSKsymmattype_enum {
  MSK_SYMMAT_TYPE_BEGIN  = 0,
  MSK_SYMMAT_TYPE_END    = 1,

  MSK_SYMMAT_TYPE_SPARSE = 0
};

enum MSKfeature_enum {
  MSK_FEATURE_BEGIN = 0,
  MSK_FEATURE_END  = 4,

  MSK_FEATURE_PTS  = 0,
  MSK_FEATURE_PTON = 1,
  MSK_FEATURE_PTOM = 2,
  MSK_FEATURE_PTOX = 3
};

enum MSKmark_enum {
  MSK_MARK_BEGIN = 0,
  MSK_MARK_END = 2,

  MSK_MARK_LO = 0,
  MSK_MARK_UP = 1
};

enum MSKconetype_enum {
  MSK_CT_BEGIN = 0,
  MSK_CT_END   = 2,

  MSK_CT_QUAD  = 0,
  MSK_CT_RQUAD = 1
};

enum MSKfeasrepairtype_enum {
  MSK_FEASREPAIR_BEGIN             = 0,
  MSK_FEASREPAIR_END               = 3,

  MSK_FEASREPAIR_OPTIMIZE_NONE     = 0,
  MSK_FEASREPAIR_OPTIMIZE_PENALTY  = 1,
  MSK_FEASREPAIR_OPTIMIZE_COMBINED = 2
};

enum MSKiomode_enum {
  MSK_IOMODE_BEGIN     = 0,
  MSK_IOMODE_END       = 3,

  MSK_IOMODE_READ      = 0,
  MSK_IOMODE_WRITE     = 1,
  MSK_IOMODE_READWRITE = 2
};

enum MSKsimseltype_enum {
  MSK_SIM_SELECTION_BEGIN   = 0,
  MSK_SIM_SELECTION_END     = 6,

  MSK_SIM_SELECTION_FREE    = 0,
  MSK_SIM_SELECTION_FULL    = 1,
  MSK_SIM_SELECTION_ASE     = 2,
  MSK_SIM_SELECTION_DEVEX   = 3,
  MSK_SIM_SELECTION_SE      = 4,
  MSK_SIM_SELECTION_PARTIAL = 5
};

enum MSKmsgkey_enum {
  MSK_MSG_READING_FILE = 1000,
  MSK_MSG_WRITING_FILE = 1001,
  MSK_MSG_MPS_SELECTED = 1100
};

enum MSKmiomode_enum {
  MSK_MIO_MODE_BEGIN     = 0,
  MSK_MIO_MODE_END       = 3,

  MSK_MIO_MODE_IGNORED   = 0,
  MSK_MIO_MODE_SATISFIED = 1,
  MSK_MIO_MODE_LAZY      = 2
};

enum MSKdinfitem_enum {
  MSK_DINF_BEGIN                         = 0,
  MSK_DINF_END                           = 70,

  MSK_DINF_BI_CLEAN_DUAL_TIME            = 0,
  MSK_DINF_BI_CLEAN_PRIMAL_DUAL_TIME     = 1,
  MSK_DINF_BI_CLEAN_PRIMAL_TIME          = 2,
  MSK_DINF_BI_CLEAN_TIME                 = 3,
  MSK_DINF_BI_DUAL_TIME                  = 4,
  MSK_DINF_BI_PRIMAL_TIME                = 5,
  MSK_DINF_BI_TIME                       = 6,
  MSK_DINF_CONCURRENT_TIME               = 7,
  MSK_DINF_INTPNT_DUAL_FEAS              = 8,
  MSK_DINF_INTPNT_DUAL_OBJ               = 9,
  MSK_DINF_INTPNT_FACTOR_NUM_FLOPS       = 10,
  MSK_DINF_INTPNT_OPT_STATUS             = 11,
  MSK_DINF_INTPNT_ORDER_TIME             = 12,
  MSK_DINF_INTPNT_PRIMAL_FEAS            = 13,
  MSK_DINF_INTPNT_PRIMAL_OBJ             = 14,
  MSK_DINF_INTPNT_TIME                   = 15,
  MSK_DINF_MIO_CG_SEPERATION_TIME        = 16,
  MSK_DINF_MIO_CMIR_SEPERATION_TIME      = 17,
  MSK_DINF_MIO_CONSTRUCT_SOLUTION_OBJ    = 18,
  MSK_DINF_MIO_DUAL_BOUND_AFTER_PRESOLVE = 19,
  MSK_DINF_MIO_HEURISTIC_TIME            = 20,
  MSK_DINF_MIO_OBJ_ABS_GAP               = 21,
  MSK_DINF_MIO_OBJ_BOUND                 = 22,
  MSK_DINF_MIO_OBJ_INT                   = 23,
  MSK_DINF_MIO_OBJ_REL_GAP               = 24,
  MSK_DINF_MIO_OPTIMIZER_TIME            = 25,
  MSK_DINF_MIO_PROBING_TIME              = 26,
  MSK_DINF_MIO_ROOT_CUTGEN_TIME          = 27,
  MSK_DINF_MIO_ROOT_OPTIMIZER_TIME       = 28,
  MSK_DINF_MIO_ROOT_PRESOLVE_TIME        = 29,
  MSK_DINF_MIO_TIME                      = 30,
  MSK_DINF_MIO_USER_OBJ_CUT              = 31,
  MSK_DINF_OPTIMIZER_TIME                = 32,
  MSK_DINF_PRESOLVE_ELI_TIME             = 33,
  MSK_DINF_PRESOLVE_LINDEP_TIME          = 34,
  MSK_DINF_PRESOLVE_TIME                 = 35,
  MSK_DINF_PRIMAL_REPAIR_PENALTY_OBJ     = 36,
  MSK_DINF_QCQO_REFORMULATE_TIME         = 37,
  MSK_DINF_RD_TIME                       = 38,
  MSK_DINF_SIM_DUAL_TIME                 = 39,
  MSK_DINF_SIM_FEAS                      = 40,
  MSK_DINF_SIM_NETWORK_DUAL_TIME         = 41,
  MSK_DINF_SIM_NETWORK_PRIMAL_TIME       = 42,
  MSK_DINF_SIM_NETWORK_TIME              = 43,
  MSK_DINF_SIM_OBJ                       = 44,
  MSK_DINF_SIM_PRIMAL_DUAL_TIME          = 45,
  MSK_DINF_SIM_PRIMAL_TIME               = 46,
  MSK_DINF_SIM_TIME                      = 47,
  MSK_DINF_SOL_BAS_DUAL_OBJ              = 48,
  MSK_DINF_SOL_BAS_DVIOLCON              = 49,
  MSK_DINF_SOL_BAS_DVIOLVAR              = 50,
  MSK_DINF_SOL_BAS_PRIMAL_OBJ            = 51,
  MSK_DINF_SOL_BAS_PVIOLCON              = 52,
  MSK_DINF_SOL_BAS_PVIOLVAR              = 53,
  MSK_DINF_SOL_ITG_PRIMAL_OBJ            = 54,
  MSK_DINF_SOL_ITG_PVIOLBARVAR           = 55,
  MSK_DINF_SOL_ITG_PVIOLCON              = 56,
  MSK_DINF_SOL_ITG_PVIOLCONES            = 57,
  MSK_DINF_SOL_ITG_PVIOLITG              = 58,
  MSK_DINF_SOL_ITG_PVIOLVAR              = 59,
  MSK_DINF_SOL_ITR_DUAL_OBJ              = 60,
  MSK_DINF_SOL_ITR_DVIOLBARVAR           = 61,
  MSK_DINF_SOL_ITR_DVIOLCON              = 62,
  MSK_DINF_SOL_ITR_DVIOLCONES            = 63,
  MSK_DINF_SOL_ITR_DVIOLVAR              = 64,
  MSK_DINF_SOL_ITR_PRIMAL_OBJ            = 65,
  MSK_DINF_SOL_ITR_PVIOLBARVAR           = 66,
  MSK_DINF_SOL_ITR_PVIOLCON              = 67,
  MSK_DINF_SOL_ITR_PVIOLCONES            = 68,
  MSK_DINF_SOL_ITR_PVIOLVAR              = 69
};

enum MSKparametertype_enum {
  MSK_PAR_BEGIN        = 0,
  MSK_PAR_END          = 4,

  MSK_PAR_INVALID_TYPE = 0,
  MSK_PAR_DOU_TYPE     = 1,
  MSK_PAR_INT_TYPE     = 2,
  MSK_PAR_STR_TYPE     = 3
};

enum MSKrescodetype_enum {
  MSK_RESPONSE_BEGIN = 0,
  MSK_RESPONSE_END = 5,

  MSK_RESPONSE_OK  = 0,
  MSK_RESPONSE_WRN = 1,
  MSK_RESPONSE_TRM = 2,
  MSK_RESPONSE_ERR = 3,
  MSK_RESPONSE_UNK = 4
};

enum MSKprosta_enum {
  MSK_PRO_STA_BEGIN                    = 0,
  MSK_PRO_STA_END                      = 12,

  MSK_PRO_STA_UNKNOWN                  = 0,
  MSK_PRO_STA_PRIM_AND_DUAL_FEAS       = 1,
  MSK_PRO_STA_PRIM_FEAS                = 2,
  MSK_PRO_STA_DUAL_FEAS                = 3,
  MSK_PRO_STA_PRIM_INFEAS              = 4,
  MSK_PRO_STA_DUAL_INFEAS              = 5,
  MSK_PRO_STA_PRIM_AND_DUAL_INFEAS     = 6,
  MSK_PRO_STA_ILL_POSED                = 7,
  MSK_PRO_STA_NEAR_PRIM_AND_DUAL_FEAS  = 8,
  MSK_PRO_STA_NEAR_PRIM_FEAS           = 9,
  MSK_PRO_STA_NEAR_DUAL_FEAS           = 10,
  MSK_PRO_STA_PRIM_INFEAS_OR_UNBOUNDED = 11
};

enum MSKscalingtype_enum {
  MSK_SCALING_BEGIN      = 0,
  MSK_SCALING_END        = 4,

  MSK_SCALING_FREE       = 0,
  MSK_SCALING_NONE       = 1,
  MSK_SCALING_MODERATE   = 2,
  MSK_SCALING_AGGRESSIVE = 3
};

enum MSKrescode_enum {
  MSK_RES_OK                                      = 0,
  MSK_RES_WRN_OPEN_PARAM_FILE                     = 50,
  MSK_RES_WRN_LARGE_BOUND                         = 51,
  MSK_RES_WRN_LARGE_LO_BOUND                      = 52,
  MSK_RES_WRN_LARGE_UP_BOUND                      = 53,
  MSK_RES_WRN_LARGE_CON_FX                        = 54,
  MSK_RES_WRN_LARGE_CJ                            = 57,
  MSK_RES_WRN_LARGE_AIJ                           = 62,
  MSK_RES_WRN_ZERO_AIJ                            = 63,
  MSK_RES_WRN_NAME_MAX_LEN                        = 65,
  MSK_RES_WRN_SPAR_MAX_LEN                        = 66,
  MSK_RES_WRN_MPS_SPLIT_RHS_VECTOR                = 70,
  MSK_RES_WRN_MPS_SPLIT_RAN_VECTOR                = 71,
  MSK_RES_WRN_MPS_SPLIT_BOU_VECTOR                = 72,
  MSK_RES_WRN_LP_OLD_QUAD_FORMAT                  = 80,
  MSK_RES_WRN_LP_DROP_VARIABLE                    = 85,
  MSK_RES_WRN_NZ_IN_UPR_TRI                       = 200,
  MSK_RES_WRN_DROPPED_NZ_QOBJ                     = 201,
  MSK_RES_WRN_IGNORE_INTEGER                      = 250,
  MSK_RES_WRN_NO_GLOBAL_OPTIMIZER                 = 251,
  MSK_RES_WRN_MIO_INFEASIBLE_FINAL                = 270,
  MSK_RES_WRN_SOL_FILTER                          = 300,
  MSK_RES_WRN_UNDEF_SOL_FILE_NAME                 = 350,
  MSK_RES_WRN_SOL_FILE_IGNORED_CON                = 351,
  MSK_RES_WRN_SOL_FILE_IGNORED_VAR                = 352,
  MSK_RES_WRN_TOO_FEW_BASIS_VARS                  = 400,
  MSK_RES_WRN_TOO_MANY_BASIS_VARS                 = 405,
  MSK_RES_WRN_NO_NONLINEAR_FUNCTION_WRITE         = 450,
  MSK_RES_WRN_LICENSE_EXPIRE                      = 500,
  MSK_RES_WRN_LICENSE_SERVER                      = 501,
  MSK_RES_WRN_EMPTY_NAME                          = 502,
  MSK_RES_WRN_USING_GENERIC_NAMES                 = 503,
  MSK_RES_WRN_LICENSE_FEATURE_EXPIRE              = 505,
  MSK_RES_WRN_PARAM_NAME_DOU                      = 510,
  MSK_RES_WRN_PARAM_NAME_INT                      = 511,
  MSK_RES_WRN_PARAM_NAME_STR                      = 512,
  MSK_RES_WRN_PARAM_STR_VALUE                     = 515,
  MSK_RES_WRN_PARAM_IGNORED_CMIO                  = 516,
  MSK_RES_WRN_ZEROS_IN_SPARSE_ROW                 = 705,
  MSK_RES_WRN_ZEROS_IN_SPARSE_COL                 = 710,
  MSK_RES_WRN_TOO_MANY_THREADS_CONCURRENT         = 750,
  MSK_RES_WRN_INCOMPLETE_LINEAR_DEPENDENCY_CHECK  = 800,
  MSK_RES_WRN_ELIMINATOR_SPACE                    = 801,
  MSK_RES_WRN_PRESOLVE_OUTOFSPACE                 = 802,
  MSK_RES_WRN_WRITE_CHANGED_NAMES                 = 803,
  MSK_RES_WRN_WRITE_DISCARDED_CFIX                = 804,
  MSK_RES_WRN_CONSTRUCT_SOLUTION_INFEAS           = 805,
  MSK_RES_WRN_CONSTRUCT_INVALID_SOL_ITG           = 807,
  MSK_RES_WRN_CONSTRUCT_NO_SOL_ITG                = 810,
  MSK_RES_WRN_DUPLICATE_CONSTRAINT_NAMES          = 850,
  MSK_RES_WRN_DUPLICATE_VARIABLE_NAMES            = 851,
  MSK_RES_WRN_DUPLICATE_BARVARIABLE_NAMES         = 852,
  MSK_RES_WRN_DUPLICATE_CONE_NAMES                = 853,
  MSK_RES_WRN_ANA_LARGE_BOUNDS                    = 900,
  MSK_RES_WRN_ANA_C_ZERO                          = 901,
  MSK_RES_WRN_ANA_EMPTY_COLS                      = 902,
  MSK_RES_WRN_ANA_CLOSE_BOUNDS                    = 903,
  MSK_RES_WRN_ANA_ALMOST_INT_BOUNDS               = 904,
  MSK_RES_WRN_QUAD_CONES_WITH_ROOT_FIXED_AT_ZERO  = 930,
  MSK_RES_WRN_RQUAD_CONES_WITH_ROOT_FIXED_AT_ZERO = 931,
  MSK_RES_WRN_NO_DUALIZER                         = 950,
  MSK_RES_ERR_LICENSE                             = 1000,
  MSK_RES_ERR_LICENSE_EXPIRED                     = 1001,
  MSK_RES_ERR_LICENSE_VERSION                     = 1002,
  MSK_RES_ERR_SIZE_LICENSE                        = 1005,
  MSK_RES_ERR_PROB_LICENSE                        = 1006,
  MSK_RES_ERR_FILE_LICENSE                        = 1007,
  MSK_RES_ERR_MISSING_LICENSE_FILE                = 1008,
  MSK_RES_ERR_SIZE_LICENSE_CON                    = 1010,
  MSK_RES_ERR_SIZE_LICENSE_VAR                    = 1011,
  MSK_RES_ERR_SIZE_LICENSE_INTVAR                 = 1012,
  MSK_RES_ERR_OPTIMIZER_LICENSE                   = 1013,
  MSK_RES_ERR_FLEXLM                              = 1014,
  MSK_RES_ERR_LICENSE_SERVER                      = 1015,
  MSK_RES_ERR_LICENSE_MAX                         = 1016,
  MSK_RES_ERR_LICENSE_MOSEKLM_DAEMON              = 1017,
  MSK_RES_ERR_LICENSE_FEATURE                     = 1018,
  MSK_RES_ERR_PLATFORM_NOT_LICENSED               = 1019,
  MSK_RES_ERR_LICENSE_CANNOT_ALLOCATE             = 1020,
  MSK_RES_ERR_LICENSE_CANNOT_CONNECT              = 1021,
  MSK_RES_ERR_LICENSE_INVALID_HOSTID              = 1025,
  MSK_RES_ERR_LICENSE_SERVER_VERSION              = 1026,
  MSK_RES_ERR_LICENSE_NO_SERVER_SUPPORT           = 1027,
  MSK_RES_ERR_LICENSE_NO_SERVER_LINE              = 1028,
  MSK_RES_ERR_OPEN_DL                             = 1030,
  MSK_RES_ERR_OLDER_DLL                           = 1035,
  MSK_RES_ERR_NEWER_DLL                           = 1036,
  MSK_RES_ERR_LINK_FILE_DLL                       = 1040,
  MSK_RES_ERR_THREAD_MUTEX_INIT                   = 1045,
  MSK_RES_ERR_THREAD_MUTEX_LOCK                   = 1046,
  MSK_RES_ERR_THREAD_MUTEX_UNLOCK                 = 1047,
  MSK_RES_ERR_THREAD_CREATE                       = 1048,
  MSK_RES_ERR_THREAD_COND_INIT                    = 1049,
  MSK_RES_ERR_UNKNOWN                             = 1050,
  MSK_RES_ERR_SPACE                               = 1051,
  MSK_RES_ERR_FILE_OPEN                           = 1052,
  MSK_RES_ERR_FILE_READ                           = 1053,
  MSK_RES_ERR_FILE_WRITE                          = 1054,
  MSK_RES_ERR_DATA_FILE_EXT                       = 1055,
  MSK_RES_ERR_INVALID_FILE_NAME                   = 1056,
  MSK_RES_ERR_INVALID_SOL_FILE_NAME               = 1057,
  MSK_RES_ERR_END_OF_FILE                         = 1059,
  MSK_RES_ERR_NULL_ENV                            = 1060,
  MSK_RES_ERR_NULL_TASK                           = 1061,
  MSK_RES_ERR_INVALID_STREAM                      = 1062,
  MSK_RES_ERR_NO_INIT_ENV                         = 1063,
  MSK_RES_ERR_INVALID_TASK                        = 1064,
  MSK_RES_ERR_NULL_POINTER                        = 1065,
  MSK_RES_ERR_LIVING_TASKS                        = 1066,
  MSK_RES_ERR_BLANK_NAME                          = 1070,
  MSK_RES_ERR_DUP_NAME                            = 1071,
  MSK_RES_ERR_INVALID_OBJ_NAME                    = 1075,
  MSK_RES_ERR_INVALID_CON_NAME                    = 1076,
  MSK_RES_ERR_INVALID_VAR_NAME                    = 1077,
  MSK_RES_ERR_INVALID_CONE_NAME                   = 1078,
  MSK_RES_ERR_INVALID_BARVAR_NAME                 = 1079,
  MSK_RES_ERR_SPACE_LEAKING                       = 1080,
  MSK_RES_ERR_SPACE_NO_INFO                       = 1081,
  MSK_RES_ERR_READ_FORMAT                         = 1090,
  MSK_RES_ERR_MPS_FILE                            = 1100,
  MSK_RES_ERR_MPS_INV_FIELD                       = 1101,
  MSK_RES_ERR_MPS_INV_MARKER                      = 1102,
  MSK_RES_ERR_MPS_NULL_CON_NAME                   = 1103,
  MSK_RES_ERR_MPS_NULL_VAR_NAME                   = 1104,
  MSK_RES_ERR_MPS_UNDEF_CON_NAME                  = 1105,
  MSK_RES_ERR_MPS_UNDEF_VAR_NAME                  = 1106,
  MSK_RES_ERR_MPS_INV_CON_KEY                     = 1107,
  MSK_RES_ERR_MPS_INV_BOUND_KEY                   = 1108,
  MSK_RES_ERR_MPS_INV_SEC_NAME                    = 1109,
  MSK_RES_ERR_MPS_NO_OBJECTIVE                    = 1110,
  MSK_RES_ERR_MPS_SPLITTED_VAR                    = 1111,
  MSK_RES_ERR_MPS_MUL_CON_NAME                    = 1112,
  MSK_RES_ERR_MPS_MUL_QSEC                        = 1113,
  MSK_RES_ERR_MPS_MUL_QOBJ                        = 1114,
  MSK_RES_ERR_MPS_INV_SEC_ORDER                   = 1115,
  MSK_RES_ERR_MPS_MUL_CSEC                        = 1116,
  MSK_RES_ERR_MPS_CONE_TYPE                       = 1117,
  MSK_RES_ERR_MPS_CONE_OVERLAP                    = 1118,
  MSK_RES_ERR_MPS_CONE_REPEAT                     = 1119,
  MSK_RES_ERR_MPS_NON_SYMMETRIC_Q                 = 1120,
  MSK_RES_ERR_MPS_DUPLICATE_Q_ELEMENT             = 1121,
  MSK_RES_ERR_MPS_INVALID_OBJSENSE                = 1122,
  MSK_RES_ERR_MPS_TAB_IN_FIELD2                   = 1125,
  MSK_RES_ERR_MPS_TAB_IN_FIELD3                   = 1126,
  MSK_RES_ERR_MPS_TAB_IN_FIELD5                   = 1127,
  MSK_RES_ERR_MPS_INVALID_OBJ_NAME                = 1128,
  MSK_RES_ERR_ORD_INVALID_BRANCH_DIR              = 1130,
  MSK_RES_ERR_ORD_INVALID                         = 1131,
  MSK_RES_ERR_LP_INCOMPATIBLE                     = 1150,
  MSK_RES_ERR_LP_EMPTY                            = 1151,
  MSK_RES_ERR_LP_DUP_SLACK_NAME                   = 1152,
  MSK_RES_ERR_WRITE_MPS_INVALID_NAME              = 1153,
  MSK_RES_ERR_LP_INVALID_VAR_NAME                 = 1154,
  MSK_RES_ERR_LP_FREE_CONSTRAINT                  = 1155,
  MSK_RES_ERR_WRITE_OPF_INVALID_VAR_NAME          = 1156,
  MSK_RES_ERR_LP_FILE_FORMAT                      = 1157,
  MSK_RES_ERR_WRITE_LP_FORMAT                     = 1158,
  MSK_RES_ERR_READ_LP_MISSING_END_TAG             = 1159,
  MSK_RES_ERR_LP_FORMAT                           = 1160,
  MSK_RES_ERR_WRITE_LP_NON_UNIQUE_NAME            = 1161,
  MSK_RES_ERR_READ_LP_NONEXISTING_NAME            = 1162,
  MSK_RES_ERR_LP_WRITE_CONIC_PROBLEM              = 1163,
  MSK_RES_ERR_LP_WRITE_GECO_PROBLEM               = 1164,
  MSK_RES_ERR_WRITING_FILE                        = 1166,
  MSK_RES_ERR_OPF_FORMAT                          = 1168,
  MSK_RES_ERR_OPF_NEW_VARIABLE                    = 1169,
  MSK_RES_ERR_INVALID_NAME_IN_SOL_FILE            = 1170,
  MSK_RES_ERR_LP_INVALID_CON_NAME                 = 1171,
  MSK_RES_ERR_OPF_PREMATURE_EOF                   = 1172,
  MSK_RES_ERR_ARGUMENT_LENNEQ                     = 1197,
  MSK_RES_ERR_ARGUMENT_TYPE                       = 1198,
  MSK_RES_ERR_NR_ARGUMENTS                        = 1199,
  MSK_RES_ERR_IN_ARGUMENT                         = 1200,
  MSK_RES_ERR_ARGUMENT_DIMENSION                  = 1201,
  MSK_RES_ERR_INDEX_IS_TOO_SMALL                  = 1203,
  MSK_RES_ERR_INDEX_IS_TOO_LARGE                  = 1204,
  MSK_RES_ERR_PARAM_NAME                          = 1205,
  MSK_RES_ERR_PARAM_NAME_DOU                      = 1206,
  MSK_RES_ERR_PARAM_NAME_INT                      = 1207,
  MSK_RES_ERR_PARAM_NAME_STR                      = 1208,
  MSK_RES_ERR_PARAM_INDEX                         = 1210,
  MSK_RES_ERR_PARAM_IS_TOO_LARGE                  = 1215,
  MSK_RES_ERR_PARAM_IS_TOO_SMALL                  = 1216,
  MSK_RES_ERR_PARAM_VALUE_STR                     = 1217,
  MSK_RES_ERR_PARAM_TYPE                          = 1218,
  MSK_RES_ERR_INF_DOU_INDEX                       = 1219,
  MSK_RES_ERR_INF_INT_INDEX                       = 1220,
  MSK_RES_ERR_INDEX_ARR_IS_TOO_SMALL              = 1221,
  MSK_RES_ERR_INDEX_ARR_IS_TOO_LARGE              = 1222,
  MSK_RES_ERR_INF_LINT_INDEX                      = 1225,
  MSK_RES_ERR_ARG_IS_TOO_SMALL                    = 1226,
  MSK_RES_ERR_ARG_IS_TOO_LARGE                    = 1227,
  MSK_RES_ERR_INVALID_WHICHSOL                    = 1228,
  MSK_RES_ERR_INF_DOU_NAME                        = 1230,
  MSK_RES_ERR_INF_INT_NAME                        = 1231,
  MSK_RES_ERR_INF_TYPE                            = 1232,
  MSK_RES_ERR_INF_LINT_NAME                       = 1234,
  MSK_RES_ERR_INDEX                               = 1235,
  MSK_RES_ERR_WHICHSOL                            = 1236,
  MSK_RES_ERR_SOLITEM                             = 1237,
  MSK_RES_ERR_WHICHITEM_NOT_ALLOWED               = 1238,
  MSK_RES_ERR_MAXNUMCON                           = 1240,
  MSK_RES_ERR_MAXNUMVAR                           = 1241,
  MSK_RES_ERR_MAXNUMBARVAR                        = 1242,
  MSK_RES_ERR_MAXNUMQNZ                           = 1243,
  MSK_RES_ERR_TOO_SMALL_MAX_NUM_NZ                = 1245,
  MSK_RES_ERR_INVALID_IDX                         = 1246,
  MSK_RES_ERR_INVALID_MAX_NUM                     = 1247,
  MSK_RES_ERR_NUMCONLIM                           = 1250,
  MSK_RES_ERR_NUMVARLIM                           = 1251,
  MSK_RES_ERR_TOO_SMALL_MAXNUMANZ                 = 1252,
  MSK_RES_ERR_INV_APTRE                           = 1253,
  MSK_RES_ERR_MUL_A_ELEMENT                       = 1254,
  MSK_RES_ERR_INV_BK                              = 1255,
  MSK_RES_ERR_INV_BKC                             = 1256,
  MSK_RES_ERR_INV_BKX                             = 1257,
  MSK_RES_ERR_INV_VAR_TYPE                        = 1258,
  MSK_RES_ERR_SOLVER_PROBTYPE                     = 1259,
  MSK_RES_ERR_OBJECTIVE_RANGE                     = 1260,
  MSK_RES_ERR_FIRST                               = 1261,
  MSK_RES_ERR_LAST                                = 1262,
  MSK_RES_ERR_NEGATIVE_SURPLUS                    = 1263,
  MSK_RES_ERR_NEGATIVE_APPEND                     = 1264,
  MSK_RES_ERR_UNDEF_SOLUTION                      = 1265,
  MSK_RES_ERR_BASIS                               = 1266,
  MSK_RES_ERR_INV_SKC                             = 1267,
  MSK_RES_ERR_INV_SKX                             = 1268,
  MSK_RES_ERR_INV_SK_STR                          = 1269,
  MSK_RES_ERR_INV_SK                              = 1270,
  MSK_RES_ERR_INV_CONE_TYPE_STR                   = 1271,
  MSK_RES_ERR_INV_CONE_TYPE                       = 1272,
  MSK_RES_ERR_INV_SKN                             = 1274,
  MSK_RES_ERR_INVALID_SURPLUS                     = 1275,
  MSK_RES_ERR_INV_NAME_ITEM                       = 1280,
  MSK_RES_ERR_PRO_ITEM                            = 1281,
  MSK_RES_ERR_INVALID_FORMAT_TYPE                 = 1283,
  MSK_RES_ERR_FIRSTI                              = 1285,
  MSK_RES_ERR_LASTI                               = 1286,
  MSK_RES_ERR_FIRSTJ                              = 1287,
  MSK_RES_ERR_LASTJ                               = 1288,
  MSK_RES_ERR_MAX_LEN_IS_TOO_SMALL                = 1289,
  MSK_RES_ERR_NONLINEAR_EQUALITY                  = 1290,
  MSK_RES_ERR_NONCONVEX                           = 1291,
  MSK_RES_ERR_NONLINEAR_RANGED                    = 1292,
  MSK_RES_ERR_CON_Q_NOT_PSD                       = 1293,
  MSK_RES_ERR_CON_Q_NOT_NSD                       = 1294,
  MSK_RES_ERR_OBJ_Q_NOT_PSD                       = 1295,
  MSK_RES_ERR_OBJ_Q_NOT_NSD                       = 1296,
  MSK_RES_ERR_ARGUMENT_PERM_ARRAY                 = 1299,
  MSK_RES_ERR_CONE_INDEX                          = 1300,
  MSK_RES_ERR_CONE_SIZE                           = 1301,
  MSK_RES_ERR_CONE_OVERLAP                        = 1302,
  MSK_RES_ERR_CONE_REP_VAR                        = 1303,
  MSK_RES_ERR_MAXNUMCONE                          = 1304,
  MSK_RES_ERR_CONE_TYPE                           = 1305,
  MSK_RES_ERR_CONE_TYPE_STR                       = 1306,
  MSK_RES_ERR_CONE_OVERLAP_APPEND                 = 1307,
  MSK_RES_ERR_REMOVE_CONE_VARIABLE                = 1310,
  MSK_RES_ERR_SOL_FILE_INVALID_NUMBER             = 1350,
  MSK_RES_ERR_HUGE_C                              = 1375,
  MSK_RES_ERR_HUGE_AIJ                            = 1380,
  MSK_RES_ERR_LOWER_BOUND_IS_A_NAN                = 1390,
  MSK_RES_ERR_UPPER_BOUND_IS_A_NAN                = 1391,
  MSK_RES_ERR_INFINITE_BOUND                      = 1400,
  MSK_RES_ERR_INV_QOBJ_SUBI                       = 1401,
  MSK_RES_ERR_INV_QOBJ_SUBJ                       = 1402,
  MSK_RES_ERR_INV_QOBJ_VAL                        = 1403,
  MSK_RES_ERR_INV_QCON_SUBK                       = 1404,
  MSK_RES_ERR_INV_QCON_SUBI                       = 1405,
  MSK_RES_ERR_INV_QCON_SUBJ                       = 1406,
  MSK_RES_ERR_INV_QCON_VAL                        = 1407,
  MSK_RES_ERR_QCON_SUBI_TOO_SMALL                 = 1408,
  MSK_RES_ERR_QCON_SUBI_TOO_LARGE                 = 1409,
  MSK_RES_ERR_QOBJ_UPPER_TRIANGLE                 = 1415,
  MSK_RES_ERR_QCON_UPPER_TRIANGLE                 = 1417,
  MSK_RES_ERR_FIXED_BOUND_VALUES                  = 1425,
  MSK_RES_ERR_NONLINEAR_FUNCTIONS_NOT_ALLOWED     = 1428,
  MSK_RES_ERR_USER_FUNC_RET                       = 1430,
  MSK_RES_ERR_USER_FUNC_RET_DATA                  = 1431,
  MSK_RES_ERR_USER_NLO_FUNC                       = 1432,
  MSK_RES_ERR_USER_NLO_EVAL                       = 1433,
  MSK_RES_ERR_USER_NLO_EVAL_HESSUBI               = 1440,
  MSK_RES_ERR_USER_NLO_EVAL_HESSUBJ               = 1441,
  MSK_RES_ERR_INVALID_OBJECTIVE_SENSE             = 1445,
  MSK_RES_ERR_UNDEFINED_OBJECTIVE_SENSE           = 1446,
  MSK_RES_ERR_Y_IS_UNDEFINED                      = 1449,
  MSK_RES_ERR_NAN_IN_DOUBLE_DATA                  = 1450,
  MSK_RES_ERR_NAN_IN_BLC                          = 1461,
  MSK_RES_ERR_NAN_IN_BUC                          = 1462,
  MSK_RES_ERR_NAN_IN_C                            = 1470,
  MSK_RES_ERR_NAN_IN_BLX                          = 1471,
  MSK_RES_ERR_NAN_IN_BUX                          = 1472,
  MSK_RES_ERR_INVALID_AIJ                         = 1473,
  MSK_RES_ERR_INV_PROBLEM                         = 1500,
  MSK_RES_ERR_MIXED_PROBLEM                       = 1501,
  MSK_RES_ERR_INV_CONIC_PROBLEM                   = 1502,
  MSK_RES_ERR_GLOBAL_INV_CONIC_PROBLEM            = 1503,
  MSK_RES_ERR_INVALID_NETWORK_PROBLEM             = 1504,
  MSK_RES_ERR_INV_OPTIMIZER                       = 1550,
  MSK_RES_ERR_MIO_NO_OPTIMIZER                    = 1551,
  MSK_RES_ERR_NO_OPTIMIZER_VAR_TYPE               = 1552,
  MSK_RES_ERR_MIO_NOT_LOADED                      = 1553,
  MSK_RES_ERR_POSTSOLVE                           = 1580,
  MSK_RES_ERR_OVERFLOW                            = 1590,
  MSK_RES_ERR_NO_BASIS_SOL                        = 1600,
  MSK_RES_ERR_BASIS_FACTOR                        = 1610,
  MSK_RES_ERR_BASIS_SINGULAR                      = 1615,
  MSK_RES_ERR_FACTOR                              = 1650,
  MSK_RES_ERR_FEASREPAIR_CANNOT_RELAX             = 1700,
  MSK_RES_ERR_FEASREPAIR_SOLVING_RELAXED          = 1701,
  MSK_RES_ERR_FEASREPAIR_INCONSISTENT_BOUND       = 1702,
  MSK_RES_ERR_REPAIR_INVALID_PROBLEM              = 1710,
  MSK_RES_ERR_REPAIR_OPTIMIZATION_FAILED          = 1711,
  MSK_RES_ERR_NAME_MAX_LEN                        = 1750,
  MSK_RES_ERR_NAME_IS_NULL                        = 1760,
  MSK_RES_ERR_INVALID_COMPRESSION                 = 1800,
  MSK_RES_ERR_INVALID_IOMODE                      = 1801,
  MSK_RES_ERR_NO_PRIMAL_INFEAS_CER                = 2000,
  MSK_RES_ERR_NO_DUAL_INFEAS_CER                  = 2001,
  MSK_RES_ERR_NO_SOLUTION_IN_CALLBACK             = 2500,
  MSK_RES_ERR_INV_MARKI                           = 2501,
  MSK_RES_ERR_INV_MARKJ                           = 2502,
  MSK_RES_ERR_INV_NUMI                            = 2503,
  MSK_RES_ERR_INV_NUMJ                            = 2504,
  MSK_RES_ERR_CANNOT_CLONE_NL                     = 2505,
  MSK_RES_ERR_CANNOT_HANDLE_NL                    = 2506,
  MSK_RES_ERR_INVALID_ACCMODE                     = 2520,
  MSK_RES_ERR_MBT_INCOMPATIBLE                    = 2550,
  MSK_RES_ERR_MBT_INVALID                         = 2551,
  MSK_RES_ERR_TASK_INCOMPATIBLE                   = 2560,
  MSK_RES_ERR_TASK_INVALID                        = 2561,
  MSK_RES_ERR_LU_MAX_NUM_TRIES                    = 2800,
  MSK_RES_ERR_INVALID_UTF8                        = 2900,
  MSK_RES_ERR_INVALID_WCHAR                       = 2901,
  MSK_RES_ERR_NO_DUAL_FOR_ITG_SOL                 = 2950,
  MSK_RES_ERR_NO_SNX_FOR_BAS_SOL                  = 2953,
  MSK_RES_ERR_INTERNAL                            = 3000,
  MSK_RES_ERR_API_ARRAY_TOO_SMALL                 = 3001,
  MSK_RES_ERR_API_CB_CONNECT                      = 3002,
  MSK_RES_ERR_API_FATAL_ERROR                     = 3005,
  MSK_RES_ERR_SEN_FORMAT                          = 3050,
  MSK_RES_ERR_SEN_UNDEF_NAME                      = 3051,
  MSK_RES_ERR_SEN_INDEX_RANGE                     = 3052,
  MSK_RES_ERR_SEN_BOUND_INVALID_UP                = 3053,
  MSK_RES_ERR_SEN_BOUND_INVALID_LO                = 3054,
  MSK_RES_ERR_SEN_INDEX_INVALID                   = 3055,
  MSK_RES_ERR_SEN_INVALID_REGEXP                  = 3056,
  MSK_RES_ERR_SEN_SOLUTION_STATUS                 = 3057,
  MSK_RES_ERR_SEN_NUMERICAL                       = 3058,
  MSK_RES_ERR_CONCURRENT_OPTIMIZER                = 3059,
  MSK_RES_ERR_SEN_UNHANDLED_PROBLEM_TYPE          = 3080,
  MSK_RES_ERR_TOO_MANY_CONCURRENT_TASKS           = 3090,
  MSK_RES_ERR_UNB_STEP_SIZE                       = 3100,
  MSK_RES_ERR_IDENTICAL_TASKS                     = 3101,
  MSK_RES_ERR_AD_INVALID_CODELIST                 = 3102,
  MSK_RES_ERR_AD_INVALID_OPERATOR                 = 3103,
  MSK_RES_ERR_AD_INVALID_OPERAND                  = 3104,
  MSK_RES_ERR_AD_MISSING_OPERAND                  = 3105,
  MSK_RES_ERR_AD_MISSING_RETURN                   = 3106,
  MSK_RES_ERR_INVALID_BRANCH_DIRECTION            = 3200,
  MSK_RES_ERR_INVALID_BRANCH_PRIORITY             = 3201,
  MSK_RES_ERR_NO_DUAL_INFO_FOR_ITG_SOL            = 3300,
  MSK_RES_ERR_INTERNAL_TEST_FAILED                = 3500,
  MSK_RES_ERR_XML_INVALID_PROBLEM_TYPE            = 3600,
  MSK_RES_ERR_INVALID_AMPL_STUB                   = 3700,
  MSK_RES_ERR_INT64_TO_INT32_CAST                 = 3800,
  MSK_RES_ERR_SIZE_LICENSE_NUMCORES               = 3900,
  MSK_RES_ERR_INFEAS_UNDEFINED                    = 3910,
  MSK_RES_ERR_NO_BARX_FOR_SOLUTION                = 3915,
  MSK_RES_ERR_NO_BARS_FOR_SOLUTION                = 3916,
  MSK_RES_ERR_BAR_VAR_DIM                         = 3920,
  MSK_RES_ERR_SYM_MAT_INVALID_ROW_INDEX           = 3940,
  MSK_RES_ERR_SYM_MAT_INVALID_COL_INDEX           = 3941,
  MSK_RES_ERR_SYM_MAT_NOT_LOWER_TRINGULAR         = 3942,
  MSK_RES_ERR_SYM_MAT_INVALID_VALUE               = 3943,
  MSK_RES_ERR_SYM_MAT_DUPLICATE                   = 3944,
  MSK_RES_ERR_INVALID_SYM_MAT_DIM                 = 3950,
  MSK_RES_ERR_API_INTERNAL                        = 3999,
  MSK_RES_ERR_INVALID_FILE_FORMAT_FOR_SYM_MAT     = 4000,
  MSK_RES_ERR_INVALID_FILE_FORMAT_FOR_CONES       = 4005,
  MSK_RES_ERR_INVALID_FILE_FORMAT_FOR_GENERAL_NL  = 4010,
  MSK_RES_ERR_DUPLICATE_CONSTRAINT_NAMES          = 4500,
  MSK_RES_ERR_DUPLICATE_VARIABLE_NAMES            = 4501,
  MSK_RES_ERR_DUPLICATE_BARVARIABLE_NAMES         = 4502,
  MSK_RES_ERR_DUPLICATE_CONE_NAMES                = 4503,
  MSK_RES_ERR_NON_UNIQUE_ARRAY                    = 5000,
  MSK_RES_ERR_ARGUMENT_IS_TOO_LARGE               = 5005,
  MSK_RES_ERR_MIO_INTERNAL                        = 5010,
  MSK_RES_ERR_INVALID_PROBLEM_TYPE                = 6000,
  MSK_RES_ERR_UNHANDLED_SOLUTION_STATUS           = 6010,
  MSK_RES_ERR_UPPER_TRIANGLE                      = 6020,
  MSK_RES_ERR_LAU_SINGULAR_MATRIX                 = 7000,
  MSK_RES_ERR_LAU_UNKNOWN                         = 7001,
  MSK_RES_ERR_LAU_ARG_M                           = 7002,
  MSK_RES_ERR_LAU_ARG_N                           = 7003,
  MSK_RES_ERR_LAU_ARG_K                           = 7004,
  MSK_RES_ERR_LAU_ARG_TRANSA                      = 7005,
  MSK_RES_ERR_LAU_ARG_TRANSB                      = 7006,
  MSK_RES_ERR_LAU_ARG_UPLO                        = 7007,
  MSK_RES_ERR_LAU_ARG_TRANS                       = 7008,
  MSK_RES_ERR_CBF_PARSE                           = 7100,
  MSK_RES_ERR_CBF_OBJ_SENSE                       = 7101,
  MSK_RES_ERR_CBF_NO_VARIABLES                    = 7102,
  MSK_RES_ERR_CBF_TOO_MANY_CONSTRAINTS            = 7103,
  MSK_RES_ERR_CBF_TOO_MANY_VARIABLES              = 7104,
  MSK_RES_ERR_CBF_NO_VERSION_SPECIFIED            = 7105,
  MSK_RES_ERR_CBF_SYNTAX                          = 7106,
  MSK_RES_ERR_CBF_DUPLICATE_OBJ                   = 7107,
  MSK_RES_ERR_CBF_DUPLICATE_CON                   = 7108,
  MSK_RES_ERR_CBF_DUPLICATE_VAR                   = 7109,
  MSK_RES_ERR_CBF_DUPLICATE_INT                   = 7110,
  MSK_RES_ERR_CBF_INVALID_VAR_TYPE                = 7111,
  MSK_RES_ERR_CBF_INVALID_CON_TYPE                = 7112,
  MSK_RES_ERR_CBF_INVALID_DOMAIN_DIMENSION        = 7113,
  MSK_RES_ERR_CBF_DUPLICATE_OBJACOORD             = 7114,
  MSK_RES_ERR_CBF_DUPLICATE_BCOORD                = 7115,
  MSK_RES_ERR_CBF_DUPLICATE_ACOORD                = 7116,
  MSK_RES_ERR_CBF_TOO_FEW_VARIABLES               = 7117,
  MSK_RES_ERR_CBF_TOO_FEW_CONSTRAINTS             = 7118,
  MSK_RES_ERR_CBF_TOO_FEW_INTS                    = 7119,
  MSK_RES_ERR_CBF_TOO_MANY_INTS                   = 7120,
  MSK_RES_ERR_CBF_INVALID_INT_INDEX               = 7121,
  MSK_RES_ERR_CBF_UNSUPPORTED                     = 7122,
  MSK_RES_ERR_MIO_INVALID_ROOT_OPTIMIZER          = 7130,
  MSK_RES_ERR_MIO_INVALID_NODE_OPTIMIZER          = 7131,
  MSK_RES_ERR_TOCONIC_CONVERSION_FAIL             = 7200,
  MSK_RES_TRM_MAX_ITERATIONS                      = 10000,
  MSK_RES_TRM_MAX_TIME                            = 10001,
  MSK_RES_TRM_OBJECTIVE_RANGE                     = 10002,
  MSK_RES_TRM_MIO_NEAR_REL_GAP                    = 10003,
  MSK_RES_TRM_MIO_NEAR_ABS_GAP                    = 10004,
  MSK_RES_TRM_STALL                               = 10006,
  MSK_RES_TRM_USER_CALLBACK                       = 10007,
  MSK_RES_TRM_MIO_NUM_RELAXS                      = 10008,
  MSK_RES_TRM_MIO_NUM_BRANCHES                    = 10009,
  MSK_RES_TRM_NUM_MAX_NUM_INT_SOLUTIONS           = 10015,
  MSK_RES_TRM_MAX_NUM_SETBACKS                    = 10020,
  MSK_RES_TRM_NUMERICAL_PROBLEM                   = 10025,
  MSK_RES_TRM_INTERNAL                            = 10030,
  MSK_RES_TRM_INTERNAL_STOP                       = 10031
};

enum MSKmionodeseltype_enum {
  MSK_MIO_NODE_SELECTION_BEGIN  = 0,
  MSK_MIO_NODE_SELECTION_END    = 6,

  MSK_MIO_NODE_SELECTION_FREE   = 0,
  MSK_MIO_NODE_SELECTION_FIRST  = 1,
  MSK_MIO_NODE_SELECTION_BEST   = 2,
  MSK_MIO_NODE_SELECTION_WORST  = 3,
  MSK_MIO_NODE_SELECTION_HYBRID = 4,
  MSK_MIO_NODE_SELECTION_PSEUDO = 5
};

enum MSKtranspose_enum {
  MSK_TRANSPOSE_BEGIN = 0,
  MSK_TRANSPOSE_END = 2,

  MSK_TRANSPOSE_NO  = 0,
  MSK_TRANSPOSE_YES = 1
};

enum MSKonoffkey_enum {
  MSK_BEGIN = 0,
  MSK_END = 2,

  MSK_OFF = 0,
  MSK_ON  = 1
};

enum MSKsimdegen_enum {
  MSK_SIM_DEGEN_BEGIN      = 0,
  MSK_SIM_DEGEN_END        = 5,

  MSK_SIM_DEGEN_NONE       = 0,
  MSK_SIM_DEGEN_FREE       = 1,
  MSK_SIM_DEGEN_AGGRESSIVE = 2,
  MSK_SIM_DEGEN_MODERATE   = 3,
  MSK_SIM_DEGEN_MINIMUM    = 4
};

enum MSKdataformat_enum {
  MSK_DATA_FORMAT_BEGIN     = 0,
  MSK_DATA_FORMAT_END       = 8,

  MSK_DATA_FORMAT_EXTENSION = 0,
  MSK_DATA_FORMAT_MPS       = 1,
  MSK_DATA_FORMAT_LP        = 2,
  MSK_DATA_FORMAT_OP        = 3,
  MSK_DATA_FORMAT_XML       = 4,
  MSK_DATA_FORMAT_FREE_MPS  = 5,
  MSK_DATA_FORMAT_TASK      = 6,
  MSK_DATA_FORMAT_CB        = 7
};

enum MSKorderingtype_enum {
  MSK_ORDER_METHOD_BEGIN          = 0,
  MSK_ORDER_METHOD_END            = 6,

  MSK_ORDER_METHOD_FREE           = 0,
  MSK_ORDER_METHOD_APPMINLOC      = 1,
  MSK_ORDER_METHOD_EXPERIMENTAL   = 2,
  MSK_ORDER_METHOD_TRY_GRAPHPAR   = 3,
  MSK_ORDER_METHOD_FORCE_GRAPHPAR = 4,
  MSK_ORDER_METHOD_NONE           = 5
};

enum MSKproblemtype_enum {
  MSK_PROBTYPE_BEGIN = 0,
  MSK_PROBTYPE_END   = 6,

  MSK_PROBTYPE_LO    = 0,
  MSK_PROBTYPE_QO    = 1,
  MSK_PROBTYPE_QCQO  = 2,
  MSK_PROBTYPE_GECO  = 3,
  MSK_PROBTYPE_CONIC = 4,
  MSK_PROBTYPE_MIXED = 5
};

enum MSKinftype_enum {
  MSK_INF_BEGIN     = 0,
  MSK_INF_END       = 3,

  MSK_INF_DOU_TYPE  = 0,
  MSK_INF_INT_TYPE  = 1,
  MSK_INF_LINT_TYPE = 2
};

enum MSKdparam_enum {
  MSK_DPAR_BEGIN                              = 0,
  MSK_DPAR_END                                = 70,

  MSK_DPAR_ANA_SOL_INFEAS_TOL                 = 0,
  MSK_DPAR_BASIS_REL_TOL_S                    = 1,
  MSK_DPAR_BASIS_TOL_S                        = 2,
  MSK_DPAR_BASIS_TOL_X                        = 3,
  MSK_DPAR_CHECK_CONVEXITY_REL_TOL            = 4,
  MSK_DPAR_DATA_TOL_AIJ                       = 5,
  MSK_DPAR_DATA_TOL_AIJ_HUGE                  = 6,
  MSK_DPAR_DATA_TOL_AIJ_LARGE                 = 7,
  MSK_DPAR_DATA_TOL_BOUND_INF                 = 8,
  MSK_DPAR_DATA_TOL_BOUND_WRN                 = 9,
  MSK_DPAR_DATA_TOL_C_HUGE                    = 10,
  MSK_DPAR_DATA_TOL_CJ_LARGE                  = 11,
  MSK_DPAR_DATA_TOL_QIJ                       = 12,
  MSK_DPAR_DATA_TOL_X                         = 13,
  MSK_DPAR_FEASREPAIR_TOL                     = 14,
  MSK_DPAR_INTPNT_CO_TOL_DFEAS                = 15,
  MSK_DPAR_INTPNT_CO_TOL_INFEAS               = 16,
  MSK_DPAR_INTPNT_CO_TOL_MU_RED               = 17,
  MSK_DPAR_INTPNT_CO_TOL_NEAR_REL             = 18,
  MSK_DPAR_INTPNT_CO_TOL_PFEAS                = 19,
  MSK_DPAR_INTPNT_CO_TOL_REL_GAP              = 20,
  MSK_DPAR_INTPNT_NL_MERIT_BAL                = 21,
  MSK_DPAR_INTPNT_NL_TOL_DFEAS                = 22,
  MSK_DPAR_INTPNT_NL_TOL_MU_RED               = 23,
  MSK_DPAR_INTPNT_NL_TOL_NEAR_REL             = 24,
  MSK_DPAR_INTPNT_NL_TOL_PFEAS                = 25,
  MSK_DPAR_INTPNT_NL_TOL_REL_GAP              = 26,
  MSK_DPAR_INTPNT_NL_TOL_REL_STEP             = 27,
  MSK_DPAR_INTPNT_TOL_DFEAS                   = 28,
  MSK_DPAR_INTPNT_TOL_DSAFE                   = 29,
  MSK_DPAR_INTPNT_TOL_INFEAS                  = 30,
  MSK_DPAR_INTPNT_TOL_MU_RED                  = 31,
  MSK_DPAR_INTPNT_TOL_PATH                    = 32,
  MSK_DPAR_INTPNT_TOL_PFEAS                   = 33,
  MSK_DPAR_INTPNT_TOL_PSAFE                   = 34,
  MSK_DPAR_INTPNT_TOL_REL_GAP                 = 35,
  MSK_DPAR_INTPNT_TOL_REL_STEP                = 36,
  MSK_DPAR_INTPNT_TOL_STEP_SIZE               = 37,
  MSK_DPAR_LOWER_OBJ_CUT                      = 38,
  MSK_DPAR_LOWER_OBJ_CUT_FINITE_TRH           = 39,
  MSK_DPAR_MIO_DISABLE_TERM_TIME              = 40,
  MSK_DPAR_MIO_HEURISTIC_TIME                 = 41,
  MSK_DPAR_MIO_MAX_TIME                       = 42,
  MSK_DPAR_MIO_MAX_TIME_APRX_OPT              = 43,
  MSK_DPAR_MIO_NEAR_TOL_ABS_GAP               = 44,
  MSK_DPAR_MIO_NEAR_TOL_REL_GAP               = 45,
  MSK_DPAR_MIO_REL_ADD_CUT_LIMITED            = 46,
  MSK_DPAR_MIO_REL_GAP_CONST                  = 47,
  MSK_DPAR_MIO_TOL_ABS_GAP                    = 48,
  MSK_DPAR_MIO_TOL_ABS_RELAX_INT              = 49,
  MSK_DPAR_MIO_TOL_FEAS                       = 50,
  MSK_DPAR_MIO_TOL_MAX_CUT_FRAC_RHS           = 51,
  MSK_DPAR_MIO_TOL_MIN_CUT_FRAC_RHS           = 52,
  MSK_DPAR_MIO_TOL_REL_DUAL_BOUND_IMPROVEMENT = 53,
  MSK_DPAR_MIO_TOL_REL_GAP                    = 54,
  MSK_DPAR_MIO_TOL_REL_RELAX_INT              = 55,
  MSK_DPAR_MIO_TOL_X                          = 56,
  MSK_DPAR_NONCONVEX_TOL_FEAS                 = 57,
  MSK_DPAR_NONCONVEX_TOL_OPT                  = 58,
  MSK_DPAR_OPTIMIZER_MAX_TIME                 = 59,
  MSK_DPAR_PRESOLVE_TOL_ABS_LINDEP            = 60,
  MSK_DPAR_PRESOLVE_TOL_AIJ                   = 61,
  MSK_DPAR_PRESOLVE_TOL_REL_LINDEP            = 62,
  MSK_DPAR_PRESOLVE_TOL_S                     = 63,
  MSK_DPAR_PRESOLVE_TOL_X                     = 64,
  MSK_DPAR_QCQO_REFORMULATE_REL_DROP_TOL      = 65,
  MSK_DPAR_SIM_LU_TOL_REL_PIV                 = 66,
  MSK_DPAR_SIMPLEX_ABS_TOL_PIV                = 67,
  MSK_DPAR_UPPER_OBJ_CUT                      = 68,
  MSK_DPAR_UPPER_OBJ_CUT_FINITE_TRH           = 69
};

enum MSKsimdupvec_enum {
  MSK_SIM_EXPLOIT_DUPVEC_BEGIN = 0,
  MSK_SIM_EXPLOIT_DUPVEC_END  = 3,

  MSK_SIM_EXPLOIT_DUPVEC_OFF  = 0,
  MSK_SIM_EXPLOIT_DUPVEC_ON   = 1,
  MSK_SIM_EXPLOIT_DUPVEC_FREE = 2
};

enum MSKcompresstype_enum {
  MSK_COMPRESS_BEGIN = 0,
  MSK_COMPRESS_END  = 3,

  MSK_COMPRESS_NONE = 0,
  MSK_COMPRESS_FREE = 1,
  MSK_COMPRESS_GZIP = 2
};

enum MSKnametype_enum {
  MSK_NAME_TYPE_BEGIN = 0,
  MSK_NAME_TYPE_END = 3,

  MSK_NAME_TYPE_GEN = 0,
  MSK_NAME_TYPE_MPS = 1,
  MSK_NAME_TYPE_LP  = 2
};

enum MSKmpsformat_enum {
  MSK_MPS_FORMAT_BEGIN   = 0,
  MSK_MPS_FORMAT_END     = 3,

  MSK_MPS_FORMAT_STRICT  = 0,
  MSK_MPS_FORMAT_RELAXED = 1,
  MSK_MPS_FORMAT_FREE    = 2
};

enum MSKvariabletype_enum {
  MSK_VAR_BEGIN     = 0,
  MSK_VAR_END       = 2,

  MSK_VAR_TYPE_CONT = 0,
  MSK_VAR_TYPE_INT  = 1
};

enum MSKcheckconvexitytype_enum {
  MSK_CHECK_CONVEXITY_BEGIN  = 0,
  MSK_CHECK_CONVEXITY_END    = 3,

  MSK_CHECK_CONVEXITY_NONE   = 0,
  MSK_CHECK_CONVEXITY_SIMPLE = 1,
  MSK_CHECK_CONVEXITY_FULL   = 2
};

enum MSKlanguage_enum {
  MSK_LANG_BEGIN = 0,
  MSK_LANG_END = 2,

  MSK_LANG_ENG = 0,
  MSK_LANG_DAN = 1
};

enum MSKstartpointtype_enum {
  MSK_STARTING_POINT_BEGIN          = 0,
  MSK_STARTING_POINT_END            = 4,

  MSK_STARTING_POINT_FREE           = 0,
  MSK_STARTING_POINT_GUESS          = 1,
  MSK_STARTING_POINT_CONSTANT       = 2,
  MSK_STARTING_POINT_SATISFY_BOUNDS = 3
};

enum MSKsoltype_enum {
  MSK_SOL_BEGIN = 0,
  MSK_SOL_END = 3,

  MSK_SOL_ITR = 0,
  MSK_SOL_BAS = 1,
  MSK_SOL_ITG = 2
};

enum MSKscalingmethod_enum {
  MSK_SCALING_METHOD_BEGIN = 0,
  MSK_SCALING_METHOD_END  = 2,

  MSK_SCALING_METHOD_POW2 = 0,
  MSK_SCALING_METHOD_FREE = 1
};

enum MSKvalue_enum {
  MSK_LICENSE_BUFFER_LENGTH = 20,
  MSK_MAX_STR_LEN           = 1024
};

enum MSKstakey_enum {
  MSK_SK_BEGIN  = 0,
  MSK_SK_END    = 7,

  MSK_SK_UNK    = 0,
  MSK_SK_BAS    = 1,
  MSK_SK_SUPBAS = 2,
  MSK_SK_LOW    = 3,
  MSK_SK_UPR    = 4,
  MSK_SK_FIX    = 5,
  MSK_SK_INF    = 6
};

enum MSKsimreform_enum {
  MSK_SIM_REFORMULATION_BEGIN      = 0,
  MSK_SIM_REFORMULATION_END        = 4,

  MSK_SIM_REFORMULATION_OFF        = 0,
  MSK_SIM_REFORMULATION_ON         = 1,
  MSK_SIM_REFORMULATION_FREE       = 2,
  MSK_SIM_REFORMULATION_AGGRESSIVE = 3
};

enum MSKiinfitem_enum {
  MSK_IINF_BEGIN                          = 0,
  MSK_IINF_END                            = 97,

  MSK_IINF_ANA_PRO_NUM_CON                = 0,
  MSK_IINF_ANA_PRO_NUM_CON_EQ             = 1,
  MSK_IINF_ANA_PRO_NUM_CON_FR             = 2,
  MSK_IINF_ANA_PRO_NUM_CON_LO             = 3,
  MSK_IINF_ANA_PRO_NUM_CON_RA             = 4,
  MSK_IINF_ANA_PRO_NUM_CON_UP             = 5,
  MSK_IINF_ANA_PRO_NUM_VAR                = 6,
  MSK_IINF_ANA_PRO_NUM_VAR_BIN            = 7,
  MSK_IINF_ANA_PRO_NUM_VAR_CONT           = 8,
  MSK_IINF_ANA_PRO_NUM_VAR_EQ             = 9,
  MSK_IINF_ANA_PRO_NUM_VAR_FR             = 10,
  MSK_IINF_ANA_PRO_NUM_VAR_INT            = 11,
  MSK_IINF_ANA_PRO_NUM_VAR_LO             = 12,
  MSK_IINF_ANA_PRO_NUM_VAR_RA             = 13,
  MSK_IINF_ANA_PRO_NUM_VAR_UP             = 14,
  MSK_IINF_CONCURRENT_FASTEST_OPTIMIZER   = 15,
  MSK_IINF_INTPNT_FACTOR_DIM_DENSE        = 16,
  MSK_IINF_INTPNT_ITER                    = 17,
  MSK_IINF_INTPNT_NUM_THREADS             = 18,
  MSK_IINF_INTPNT_SOLVE_DUAL              = 19,
  MSK_IINF_MIO_CONSTRUCT_NUM_ROUNDINGS    = 20,
  MSK_IINF_MIO_CONSTRUCT_SOLUTION         = 21,
  MSK_IINF_MIO_INITIAL_SOLUTION           = 22,
  MSK_IINF_MIO_NUM_ACTIVE_NODES           = 23,
  MSK_IINF_MIO_NUM_BASIS_CUTS             = 24,
  MSK_IINF_MIO_NUM_BRANCH                 = 25,
  MSK_IINF_MIO_NUM_CARDGUB_CUTS           = 26,
  MSK_IINF_MIO_NUM_CLIQUE_CUTS            = 27,
  MSK_IINF_MIO_NUM_COEF_REDC_CUTS         = 28,
  MSK_IINF_MIO_NUM_CONTRA_CUTS            = 29,
  MSK_IINF_MIO_NUM_DISAGG_CUTS            = 30,
  MSK_IINF_MIO_NUM_FLOW_COVER_CUTS        = 31,
  MSK_IINF_MIO_NUM_GCD_CUTS               = 32,
  MSK_IINF_MIO_NUM_GOMORY_CUTS            = 33,
  MSK_IINF_MIO_NUM_GUB_COVER_CUTS         = 34,
  MSK_IINF_MIO_NUM_INT_SOLUTIONS          = 35,
  MSK_IINF_MIO_NUM_KNAPSUR_COVER_CUTS     = 36,
  MSK_IINF_MIO_NUM_LATTICE_CUTS           = 37,
  MSK_IINF_MIO_NUM_LIFT_CUTS              = 38,
  MSK_IINF_MIO_NUM_OBJ_CUTS               = 39,
  MSK_IINF_MIO_NUM_PLAN_LOC_CUTS          = 40,
  MSK_IINF_MIO_NUM_RELAX                  = 41,
  MSK_IINF_MIO_NUMCON                     = 42,
  MSK_IINF_MIO_NUMINT                     = 43,
  MSK_IINF_MIO_NUMVAR                     = 44,
  MSK_IINF_MIO_OBJ_BOUND_DEFINED          = 45,
  MSK_IINF_MIO_TOTAL_NUM_CUTS             = 46,
  MSK_IINF_MIO_USER_OBJ_CUT               = 47,
  MSK_IINF_OPT_NUMCON                     = 48,
  MSK_IINF_OPT_NUMVAR                     = 49,
  MSK_IINF_OPTIMIZE_RESPONSE              = 50,
  MSK_IINF_RD_NUMBARVAR                   = 51,
  MSK_IINF_RD_NUMCON                      = 52,
  MSK_IINF_RD_NUMCONE                     = 53,
  MSK_IINF_RD_NUMINTVAR                   = 54,
  MSK_IINF_RD_NUMQ                        = 55,
  MSK_IINF_RD_NUMVAR                      = 56,
  MSK_IINF_RD_PROTYPE                     = 57,
  MSK_IINF_SIM_DUAL_DEG_ITER              = 58,
  MSK_IINF_SIM_DUAL_HOTSTART              = 59,
  MSK_IINF_SIM_DUAL_HOTSTART_LU           = 60,
  MSK_IINF_SIM_DUAL_INF_ITER              = 61,
  MSK_IINF_SIM_DUAL_ITER                  = 62,
  MSK_IINF_SIM_NETWORK_DUAL_DEG_ITER      = 63,
  MSK_IINF_SIM_NETWORK_DUAL_HOTSTART      = 64,
  MSK_IINF_SIM_NETWORK_DUAL_HOTSTART_LU   = 65,
  MSK_IINF_SIM_NETWORK_DUAL_INF_ITER      = 66,
  MSK_IINF_SIM_NETWORK_DUAL_ITER          = 67,
  MSK_IINF_SIM_NETWORK_PRIMAL_DEG_ITER    = 68,
  MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART    = 69,
  MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART_LU = 70,
  MSK_IINF_SIM_NETWORK_PRIMAL_INF_ITER    = 71,
  MSK_IINF_SIM_NETWORK_PRIMAL_ITER        = 72,
  MSK_IINF_SIM_NUMCON                     = 73,
  MSK_IINF_SIM_NUMVAR                     = 74,
  MSK_IINF_SIM_PRIMAL_DEG_ITER            = 75,
  MSK_IINF_SIM_PRIMAL_DUAL_DEG_ITER       = 76,
  MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART       = 77,
  MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART_LU    = 78,
  MSK_IINF_SIM_PRIMAL_DUAL_INF_ITER       = 79,
  MSK_IINF_SIM_PRIMAL_DUAL_ITER           = 80,
  MSK_IINF_SIM_PRIMAL_HOTSTART            = 81,
  MSK_IINF_SIM_PRIMAL_HOTSTART_LU         = 82,
  MSK_IINF_SIM_PRIMAL_INF_ITER            = 83,
  MSK_IINF_SIM_PRIMAL_ITER                = 84,
  MSK_IINF_SIM_SOLVE_DUAL                 = 85,
  MSK_IINF_SOL_BAS_PROSTA                 = 86,
  MSK_IINF_SOL_BAS_SOLSTA                 = 87,
  MSK_IINF_SOL_INT_PROSTA                 = 88,
  MSK_IINF_SOL_INT_SOLSTA                 = 89,
  MSK_IINF_SOL_ITG_PROSTA                 = 90,
  MSK_IINF_SOL_ITG_SOLSTA                 = 91,
  MSK_IINF_SOL_ITR_PROSTA                 = 92,
  MSK_IINF_SOL_ITR_SOLSTA                 = 93,
  MSK_IINF_STO_NUM_A_CACHE_FLUSHES        = 94,
  MSK_IINF_STO_NUM_A_REALLOC              = 95,
  MSK_IINF_STO_NUM_A_TRANSPOSES           = 96
};

enum MSKxmlwriteroutputtype_enum {
  MSK_WRITE_XML_MODE_BEGIN = 0,
  MSK_WRITE_XML_MODE_END = 2,

  MSK_WRITE_XML_MODE_ROW = 0,
  MSK_WRITE_XML_MODE_COL = 1
};

enum MSKoptimizertype_enum {
  MSK_OPTIMIZER_BEGIN                  = 0,
  MSK_OPTIMIZER_END                    = 12,

  MSK_OPTIMIZER_FREE                   = 0,
  MSK_OPTIMIZER_INTPNT                 = 1,
  MSK_OPTIMIZER_CONIC                  = 2,
  MSK_OPTIMIZER_PRIMAL_SIMPLEX         = 3,
  MSK_OPTIMIZER_DUAL_SIMPLEX           = 4,
  MSK_OPTIMIZER_PRIMAL_DUAL_SIMPLEX    = 5,
  MSK_OPTIMIZER_FREE_SIMPLEX           = 6,
  MSK_OPTIMIZER_NETWORK_PRIMAL_SIMPLEX = 7,
  MSK_OPTIMIZER_MIXED_INT_CONIC        = 8,
  MSK_OPTIMIZER_MIXED_INT              = 9,
  MSK_OPTIMIZER_CONCURRENT             = 10,
  MSK_OPTIMIZER_NONCONVEX              = 11
};

enum MSKpresolvemode_enum {
  MSK_PRESOLVE_MODE_BEGIN = 0,
  MSK_PRESOLVE_MODE_END  = 3,

  MSK_PRESOLVE_MODE_OFF  = 0,
  MSK_PRESOLVE_MODE_ON   = 1,
  MSK_PRESOLVE_MODE_FREE = 2
};

enum MSKmiocontsoltype_enum {
  MSK_MIO_CONT_SOL_BEGIN   = 0,
  MSK_MIO_CONT_SOL_END     = 4,

  MSK_MIO_CONT_SOL_NONE    = 0,
  MSK_MIO_CONT_SOL_ROOT    = 1,
  MSK_MIO_CONT_SOL_ITG     = 2,
  MSK_MIO_CONT_SOL_ITG_REL = 3
};

/* } namespace mosek; */
/**************************************************/
#define MSK_FIRST_ERR_CODE 1000 
#define MSK_LAST_ERR_CODE  9999 
/**************************************************/



#define MSK_SPAR_BAS_SOL_FILE_NAME_                         "MSK_SPAR_BAS_SOL_FILE_NAME"
#define MSK_SPAR_DATA_FILE_NAME_                            "MSK_SPAR_DATA_FILE_NAME"
#define MSK_SPAR_DEBUG_FILE_NAME_                           "MSK_SPAR_DEBUG_FILE_NAME"
#define MSK_SPAR_FEASREPAIR_NAME_PREFIX_                    "MSK_SPAR_FEASREPAIR_NAME_PREFIX"
#define MSK_SPAR_FEASREPAIR_NAME_SEPARATOR_                 "MSK_SPAR_FEASREPAIR_NAME_SEPARATOR"
#define MSK_SPAR_FEASREPAIR_NAME_WSUMVIOL_                  "MSK_SPAR_FEASREPAIR_NAME_WSUMVIOL"
#define MSK_SPAR_INT_SOL_FILE_NAME_                         "MSK_SPAR_INT_SOL_FILE_NAME"
#define MSK_SPAR_ITR_SOL_FILE_NAME_                         "MSK_SPAR_ITR_SOL_FILE_NAME"
#define MSK_SPAR_MIO_DEBUG_STRING_                          "MSK_SPAR_MIO_DEBUG_STRING"
#define MSK_SPAR_PARAM_COMMENT_SIGN_                        "MSK_SPAR_PARAM_COMMENT_SIGN"
#define MSK_SPAR_PARAM_READ_FILE_NAME_                      "MSK_SPAR_PARAM_READ_FILE_NAME"
#define MSK_SPAR_PARAM_WRITE_FILE_NAME_                     "MSK_SPAR_PARAM_WRITE_FILE_NAME"
#define MSK_SPAR_READ_MPS_BOU_NAME_                         "MSK_SPAR_READ_MPS_BOU_NAME"
#define MSK_SPAR_READ_MPS_OBJ_NAME_                         "MSK_SPAR_READ_MPS_OBJ_NAME"
#define MSK_SPAR_READ_MPS_RAN_NAME_                         "MSK_SPAR_READ_MPS_RAN_NAME"
#define MSK_SPAR_READ_MPS_RHS_NAME_                         "MSK_SPAR_READ_MPS_RHS_NAME"
#define MSK_SPAR_SENSITIVITY_FILE_NAME_                     "MSK_SPAR_SENSITIVITY_FILE_NAME"
#define MSK_SPAR_SENSITIVITY_RES_FILE_NAME_                 "MSK_SPAR_SENSITIVITY_RES_FILE_NAME"
#define MSK_SPAR_SOL_FILTER_XC_LOW_                         "MSK_SPAR_SOL_FILTER_XC_LOW"
#define MSK_SPAR_SOL_FILTER_XC_UPR_                         "MSK_SPAR_SOL_FILTER_XC_UPR"
#define MSK_SPAR_SOL_FILTER_XX_LOW_                         "MSK_SPAR_SOL_FILTER_XX_LOW"
#define MSK_SPAR_SOL_FILTER_XX_UPR_                         "MSK_SPAR_SOL_FILTER_XX_UPR"
#define MSK_SPAR_STAT_FILE_NAME_                            "MSK_SPAR_STAT_FILE_NAME"
#define MSK_SPAR_STAT_KEY_                                  "MSK_SPAR_STAT_KEY"
#define MSK_SPAR_STAT_NAME_                                 "MSK_SPAR_STAT_NAME"
#define MSK_SPAR_WRITE_LP_GEN_VAR_NAME_                     "MSK_SPAR_WRITE_LP_GEN_VAR_NAME"

#define MSK_DPAR_ANA_SOL_INFEAS_TOL_                        "MSK_DPAR_ANA_SOL_INFEAS_TOL"
#define MSK_DPAR_BASIS_REL_TOL_S_                           "MSK_DPAR_BASIS_REL_TOL_S"
#define MSK_DPAR_BASIS_TOL_S_                               "MSK_DPAR_BASIS_TOL_S"
#define MSK_DPAR_BASIS_TOL_X_                               "MSK_DPAR_BASIS_TOL_X"
#define MSK_DPAR_CHECK_CONVEXITY_REL_TOL_                   "MSK_DPAR_CHECK_CONVEXITY_REL_TOL"
#define MSK_DPAR_DATA_TOL_AIJ_                              "MSK_DPAR_DATA_TOL_AIJ"
#define MSK_DPAR_DATA_TOL_AIJ_HUGE_                         "MSK_DPAR_DATA_TOL_AIJ_HUGE"
#define MSK_DPAR_DATA_TOL_AIJ_LARGE_                        "MSK_DPAR_DATA_TOL_AIJ_LARGE"
#define MSK_DPAR_DATA_TOL_BOUND_INF_                        "MSK_DPAR_DATA_TOL_BOUND_INF"
#define MSK_DPAR_DATA_TOL_BOUND_WRN_                        "MSK_DPAR_DATA_TOL_BOUND_WRN"
#define MSK_DPAR_DATA_TOL_C_HUGE_                           "MSK_DPAR_DATA_TOL_C_HUGE"
#define MSK_DPAR_DATA_TOL_CJ_LARGE_                         "MSK_DPAR_DATA_TOL_CJ_LARGE"
#define MSK_DPAR_DATA_TOL_QIJ_                              "MSK_DPAR_DATA_TOL_QIJ"
#define MSK_DPAR_DATA_TOL_X_                                "MSK_DPAR_DATA_TOL_X"
#define MSK_DPAR_FEASREPAIR_TOL_                            "MSK_DPAR_FEASREPAIR_TOL"
#define MSK_DPAR_INTPNT_CO_TOL_DFEAS_                       "MSK_DPAR_INTPNT_CO_TOL_DFEAS"
#define MSK_DPAR_INTPNT_CO_TOL_INFEAS_                      "MSK_DPAR_INTPNT_CO_TOL_INFEAS"
#define MSK_DPAR_INTPNT_CO_TOL_MU_RED_                      "MSK_DPAR_INTPNT_CO_TOL_MU_RED"
#define MSK_DPAR_INTPNT_CO_TOL_NEAR_REL_                    "MSK_DPAR_INTPNT_CO_TOL_NEAR_REL"
#define MSK_DPAR_INTPNT_CO_TOL_PFEAS_                       "MSK_DPAR_INTPNT_CO_TOL_PFEAS"
#define MSK_DPAR_INTPNT_CO_TOL_REL_GAP_                     "MSK_DPAR_INTPNT_CO_TOL_REL_GAP"
#define MSK_DPAR_INTPNT_NL_MERIT_BAL_                       "MSK_DPAR_INTPNT_NL_MERIT_BAL"
#define MSK_DPAR_INTPNT_NL_TOL_DFEAS_                       "MSK_DPAR_INTPNT_NL_TOL_DFEAS"
#define MSK_DPAR_INTPNT_NL_TOL_MU_RED_                      "MSK_DPAR_INTPNT_NL_TOL_MU_RED"
#define MSK_DPAR_INTPNT_NL_TOL_NEAR_REL_                    "MSK_DPAR_INTPNT_NL_TOL_NEAR_REL"
#define MSK_DPAR_INTPNT_NL_TOL_PFEAS_                       "MSK_DPAR_INTPNT_NL_TOL_PFEAS"
#define MSK_DPAR_INTPNT_NL_TOL_REL_GAP_                     "MSK_DPAR_INTPNT_NL_TOL_REL_GAP"
#define MSK_DPAR_INTPNT_NL_TOL_REL_STEP_                    "MSK_DPAR_INTPNT_NL_TOL_REL_STEP"
#define MSK_DPAR_INTPNT_TOL_DFEAS_                          "MSK_DPAR_INTPNT_TOL_DFEAS"
#define MSK_DPAR_INTPNT_TOL_DSAFE_                          "MSK_DPAR_INTPNT_TOL_DSAFE"
#define MSK_DPAR_INTPNT_TOL_INFEAS_                         "MSK_DPAR_INTPNT_TOL_INFEAS"
#define MSK_DPAR_INTPNT_TOL_MU_RED_                         "MSK_DPAR_INTPNT_TOL_MU_RED"
#define MSK_DPAR_INTPNT_TOL_PATH_                           "MSK_DPAR_INTPNT_TOL_PATH"
#define MSK_DPAR_INTPNT_TOL_PFEAS_                          "MSK_DPAR_INTPNT_TOL_PFEAS"
#define MSK_DPAR_INTPNT_TOL_PSAFE_                          "MSK_DPAR_INTPNT_TOL_PSAFE"
#define MSK_DPAR_INTPNT_TOL_REL_GAP_                        "MSK_DPAR_INTPNT_TOL_REL_GAP"
#define MSK_DPAR_INTPNT_TOL_REL_STEP_                       "MSK_DPAR_INTPNT_TOL_REL_STEP"
#define MSK_DPAR_INTPNT_TOL_STEP_SIZE_                      "MSK_DPAR_INTPNT_TOL_STEP_SIZE"
#define MSK_DPAR_LOWER_OBJ_CUT_                             "MSK_DPAR_LOWER_OBJ_CUT"
#define MSK_DPAR_LOWER_OBJ_CUT_FINITE_TRH_                  "MSK_DPAR_LOWER_OBJ_CUT_FINITE_TRH"
#define MSK_DPAR_MIO_DISABLE_TERM_TIME_                     "MSK_DPAR_MIO_DISABLE_TERM_TIME"
#define MSK_DPAR_MIO_HEURISTIC_TIME_                        "MSK_DPAR_MIO_HEURISTIC_TIME"
#define MSK_DPAR_MIO_MAX_TIME_                              "MSK_DPAR_MIO_MAX_TIME"
#define MSK_DPAR_MIO_MAX_TIME_APRX_OPT_                     "MSK_DPAR_MIO_MAX_TIME_APRX_OPT"
#define MSK_DPAR_MIO_NEAR_TOL_ABS_GAP_                      "MSK_DPAR_MIO_NEAR_TOL_ABS_GAP"
#define MSK_DPAR_MIO_NEAR_TOL_REL_GAP_                      "MSK_DPAR_MIO_NEAR_TOL_REL_GAP"
#define MSK_DPAR_MIO_REL_ADD_CUT_LIMITED_                   "MSK_DPAR_MIO_REL_ADD_CUT_LIMITED"
#define MSK_DPAR_MIO_REL_GAP_CONST_                         "MSK_DPAR_MIO_REL_GAP_CONST"
#define MSK_DPAR_MIO_TOL_ABS_GAP_                           "MSK_DPAR_MIO_TOL_ABS_GAP"
#define MSK_DPAR_MIO_TOL_ABS_RELAX_INT_                     "MSK_DPAR_MIO_TOL_ABS_RELAX_INT"
#define MSK_DPAR_MIO_TOL_FEAS_                              "MSK_DPAR_MIO_TOL_FEAS"
#define MSK_DPAR_MIO_TOL_MAX_CUT_FRAC_RHS_                  "MSK_DPAR_MIO_TOL_MAX_CUT_FRAC_RHS"
#define MSK_DPAR_MIO_TOL_MIN_CUT_FRAC_RHS_                  "MSK_DPAR_MIO_TOL_MIN_CUT_FRAC_RHS"
#define MSK_DPAR_MIO_TOL_REL_DUAL_BOUND_IMPROVEMENT_        "MSK_DPAR_MIO_TOL_REL_DUAL_BOUND_IMPROVEMENT"
#define MSK_DPAR_MIO_TOL_REL_GAP_                           "MSK_DPAR_MIO_TOL_REL_GAP"
#define MSK_DPAR_MIO_TOL_REL_RELAX_INT_                     "MSK_DPAR_MIO_TOL_REL_RELAX_INT"
#define MSK_DPAR_MIO_TOL_X_                                 "MSK_DPAR_MIO_TOL_X"
#define MSK_DPAR_NONCONVEX_TOL_FEAS_                        "MSK_DPAR_NONCONVEX_TOL_FEAS"
#define MSK_DPAR_NONCONVEX_TOL_OPT_                         "MSK_DPAR_NONCONVEX_TOL_OPT"
#define MSK_DPAR_OPTIMIZER_MAX_TIME_                        "MSK_DPAR_OPTIMIZER_MAX_TIME"
#define MSK_DPAR_PRESOLVE_TOL_ABS_LINDEP_                   "MSK_DPAR_PRESOLVE_TOL_ABS_LINDEP"
#define MSK_DPAR_PRESOLVE_TOL_AIJ_                          "MSK_DPAR_PRESOLVE_TOL_AIJ"
#define MSK_DPAR_PRESOLVE_TOL_REL_LINDEP_                   "MSK_DPAR_PRESOLVE_TOL_REL_LINDEP"
#define MSK_DPAR_PRESOLVE_TOL_S_                            "MSK_DPAR_PRESOLVE_TOL_S"
#define MSK_DPAR_PRESOLVE_TOL_X_                            "MSK_DPAR_PRESOLVE_TOL_X"
#define MSK_DPAR_QCQO_REFORMULATE_REL_DROP_TOL_             "MSK_DPAR_QCQO_REFORMULATE_REL_DROP_TOL"
#define MSK_DPAR_SIM_LU_TOL_REL_PIV_                        "MSK_DPAR_SIM_LU_TOL_REL_PIV"
#define MSK_DPAR_SIMPLEX_ABS_TOL_PIV_                       "MSK_DPAR_SIMPLEX_ABS_TOL_PIV"
#define MSK_DPAR_UPPER_OBJ_CUT_                             "MSK_DPAR_UPPER_OBJ_CUT"
#define MSK_DPAR_UPPER_OBJ_CUT_FINITE_TRH_                  "MSK_DPAR_UPPER_OBJ_CUT_FINITE_TRH"

#define MSK_IPAR_ALLOC_ADD_QNZ_                             "MSK_IPAR_ALLOC_ADD_QNZ"
#define MSK_IPAR_ANA_SOL_BASIS_                             "MSK_IPAR_ANA_SOL_BASIS"
#define MSK_IPAR_ANA_SOL_PRINT_VIOLATED_                    "MSK_IPAR_ANA_SOL_PRINT_VIOLATED"
#define MSK_IPAR_AUTO_SORT_A_BEFORE_OPT_                    "MSK_IPAR_AUTO_SORT_A_BEFORE_OPT"
#define MSK_IPAR_AUTO_UPDATE_SOL_INFO_                      "MSK_IPAR_AUTO_UPDATE_SOL_INFO"
#define MSK_IPAR_BASIS_SOLVE_USE_PLUS_ONE_                  "MSK_IPAR_BASIS_SOLVE_USE_PLUS_ONE"
#define MSK_IPAR_BI_CLEAN_OPTIMIZER_                        "MSK_IPAR_BI_CLEAN_OPTIMIZER"
#define MSK_IPAR_BI_IGNORE_MAX_ITER_                        "MSK_IPAR_BI_IGNORE_MAX_ITER"
#define MSK_IPAR_BI_IGNORE_NUM_ERROR_                       "MSK_IPAR_BI_IGNORE_NUM_ERROR"
#define MSK_IPAR_BI_MAX_ITERATIONS_                         "MSK_IPAR_BI_MAX_ITERATIONS"
#define MSK_IPAR_CACHE_LICENSE_                             "MSK_IPAR_CACHE_LICENSE"
#define MSK_IPAR_CHECK_CONVEXITY_                           "MSK_IPAR_CHECK_CONVEXITY"
#define MSK_IPAR_COMPRESS_STATFILE_                         "MSK_IPAR_COMPRESS_STATFILE"
#define MSK_IPAR_CONCURRENT_NUM_OPTIMIZERS_                 "MSK_IPAR_CONCURRENT_NUM_OPTIMIZERS"
#define MSK_IPAR_CONCURRENT_PRIORITY_DUAL_SIMPLEX_          "MSK_IPAR_CONCURRENT_PRIORITY_DUAL_SIMPLEX"
#define MSK_IPAR_CONCURRENT_PRIORITY_FREE_SIMPLEX_          "MSK_IPAR_CONCURRENT_PRIORITY_FREE_SIMPLEX"
#define MSK_IPAR_CONCURRENT_PRIORITY_INTPNT_                "MSK_IPAR_CONCURRENT_PRIORITY_INTPNT"
#define MSK_IPAR_CONCURRENT_PRIORITY_PRIMAL_SIMPLEX_        "MSK_IPAR_CONCURRENT_PRIORITY_PRIMAL_SIMPLEX"
#define MSK_IPAR_FEASREPAIR_OPTIMIZE_                       "MSK_IPAR_FEASREPAIR_OPTIMIZE"
#define MSK_IPAR_INFEAS_GENERIC_NAMES_                      "MSK_IPAR_INFEAS_GENERIC_NAMES"
#define MSK_IPAR_INFEAS_PREFER_PRIMAL_                      "MSK_IPAR_INFEAS_PREFER_PRIMAL"
#define MSK_IPAR_INFEAS_REPORT_AUTO_                        "MSK_IPAR_INFEAS_REPORT_AUTO"
#define MSK_IPAR_INFEAS_REPORT_LEVEL_                       "MSK_IPAR_INFEAS_REPORT_LEVEL"
#define MSK_IPAR_INTPNT_BASIS_                              "MSK_IPAR_INTPNT_BASIS"
#define MSK_IPAR_INTPNT_DIFF_STEP_                          "MSK_IPAR_INTPNT_DIFF_STEP"
#define MSK_IPAR_INTPNT_FACTOR_DEBUG_LVL_                   "MSK_IPAR_INTPNT_FACTOR_DEBUG_LVL"
#define MSK_IPAR_INTPNT_FACTOR_METHOD_                      "MSK_IPAR_INTPNT_FACTOR_METHOD"
#define MSK_IPAR_INTPNT_HOTSTART_                           "MSK_IPAR_INTPNT_HOTSTART"
#define MSK_IPAR_INTPNT_MAX_ITERATIONS_                     "MSK_IPAR_INTPNT_MAX_ITERATIONS"
#define MSK_IPAR_INTPNT_MAX_NUM_COR_                        "MSK_IPAR_INTPNT_MAX_NUM_COR"
#define MSK_IPAR_INTPNT_MAX_NUM_REFINEMENT_STEPS_           "MSK_IPAR_INTPNT_MAX_NUM_REFINEMENT_STEPS"
#define MSK_IPAR_INTPNT_OFF_COL_TRH_                        "MSK_IPAR_INTPNT_OFF_COL_TRH"
#define MSK_IPAR_INTPNT_ORDER_METHOD_                       "MSK_IPAR_INTPNT_ORDER_METHOD"
#define MSK_IPAR_INTPNT_REGULARIZATION_USE_                 "MSK_IPAR_INTPNT_REGULARIZATION_USE"
#define MSK_IPAR_INTPNT_SCALING_                            "MSK_IPAR_INTPNT_SCALING"
#define MSK_IPAR_INTPNT_SOLVE_FORM_                         "MSK_IPAR_INTPNT_SOLVE_FORM"
#define MSK_IPAR_INTPNT_STARTING_POINT_                     "MSK_IPAR_INTPNT_STARTING_POINT"
#define MSK_IPAR_LIC_TRH_EXPIRY_WRN_                        "MSK_IPAR_LIC_TRH_EXPIRY_WRN"
#define MSK_IPAR_LICENSE_DEBUG_                             "MSK_IPAR_LICENSE_DEBUG"
#define MSK_IPAR_LICENSE_PAUSE_TIME_                        "MSK_IPAR_LICENSE_PAUSE_TIME"
#define MSK_IPAR_LICENSE_SUPPRESS_EXPIRE_WRNS_              "MSK_IPAR_LICENSE_SUPPRESS_EXPIRE_WRNS"
#define MSK_IPAR_LICENSE_WAIT_                              "MSK_IPAR_LICENSE_WAIT"
#define MSK_IPAR_LOG_                                       "MSK_IPAR_LOG"
#define MSK_IPAR_LOG_BI_                                    "MSK_IPAR_LOG_BI"
#define MSK_IPAR_LOG_BI_FREQ_                               "MSK_IPAR_LOG_BI_FREQ"
#define MSK_IPAR_LOG_CHECK_CONVEXITY_                       "MSK_IPAR_LOG_CHECK_CONVEXITY"
#define MSK_IPAR_LOG_CONCURRENT_                            "MSK_IPAR_LOG_CONCURRENT"
#define MSK_IPAR_LOG_CUT_SECOND_OPT_                        "MSK_IPAR_LOG_CUT_SECOND_OPT"
#define MSK_IPAR_LOG_EXPAND_                                "MSK_IPAR_LOG_EXPAND"
#define MSK_IPAR_LOG_FACTOR_                                "MSK_IPAR_LOG_FACTOR"
#define MSK_IPAR_LOG_FEAS_REPAIR_                           "MSK_IPAR_LOG_FEAS_REPAIR"
#define MSK_IPAR_LOG_FILE_                                  "MSK_IPAR_LOG_FILE"
#define MSK_IPAR_LOG_HEAD_                                  "MSK_IPAR_LOG_HEAD"
#define MSK_IPAR_LOG_INFEAS_ANA_                            "MSK_IPAR_LOG_INFEAS_ANA"
#define MSK_IPAR_LOG_INTPNT_                                "MSK_IPAR_LOG_INTPNT"
#define MSK_IPAR_LOG_MIO_                                   "MSK_IPAR_LOG_MIO"
#define MSK_IPAR_LOG_MIO_FREQ_                              "MSK_IPAR_LOG_MIO_FREQ"
#define MSK_IPAR_LOG_NONCONVEX_                             "MSK_IPAR_LOG_NONCONVEX"
#define MSK_IPAR_LOG_OPTIMIZER_                             "MSK_IPAR_LOG_OPTIMIZER"
#define MSK_IPAR_LOG_ORDER_                                 "MSK_IPAR_LOG_ORDER"
#define MSK_IPAR_LOG_PARAM_                                 "MSK_IPAR_LOG_PARAM"
#define MSK_IPAR_LOG_PRESOLVE_                              "MSK_IPAR_LOG_PRESOLVE"
#define MSK_IPAR_LOG_RESPONSE_                              "MSK_IPAR_LOG_RESPONSE"
#define MSK_IPAR_LOG_SENSITIVITY_                           "MSK_IPAR_LOG_SENSITIVITY"
#define MSK_IPAR_LOG_SENSITIVITY_OPT_                       "MSK_IPAR_LOG_SENSITIVITY_OPT"
#define MSK_IPAR_LOG_SIM_                                   "MSK_IPAR_LOG_SIM"
#define MSK_IPAR_LOG_SIM_FREQ_                              "MSK_IPAR_LOG_SIM_FREQ"
#define MSK_IPAR_LOG_SIM_MINOR_                             "MSK_IPAR_LOG_SIM_MINOR"
#define MSK_IPAR_LOG_SIM_NETWORK_FREQ_                      "MSK_IPAR_LOG_SIM_NETWORK_FREQ"
#define MSK_IPAR_LOG_STORAGE_                               "MSK_IPAR_LOG_STORAGE"
#define MSK_IPAR_MAX_NUM_WARNINGS_                          "MSK_IPAR_MAX_NUM_WARNINGS"
#define MSK_IPAR_MIO_BRANCH_DIR_                            "MSK_IPAR_MIO_BRANCH_DIR"
#define MSK_IPAR_MIO_BRANCH_PRIORITIES_USE_                 "MSK_IPAR_MIO_BRANCH_PRIORITIES_USE"
#define MSK_IPAR_MIO_CONSTRUCT_SOL_                         "MSK_IPAR_MIO_CONSTRUCT_SOL"
#define MSK_IPAR_MIO_CONT_SOL_                              "MSK_IPAR_MIO_CONT_SOL"
#define MSK_IPAR_MIO_CUT_CG_                                "MSK_IPAR_MIO_CUT_CG"
#define MSK_IPAR_MIO_CUT_CMIR_                              "MSK_IPAR_MIO_CUT_CMIR"
#define MSK_IPAR_MIO_CUT_LEVEL_ROOT_                        "MSK_IPAR_MIO_CUT_LEVEL_ROOT"
#define MSK_IPAR_MIO_CUT_LEVEL_TREE_                        "MSK_IPAR_MIO_CUT_LEVEL_TREE"
#define MSK_IPAR_MIO_FEASPUMP_LEVEL_                        "MSK_IPAR_MIO_FEASPUMP_LEVEL"
#define MSK_IPAR_MIO_HEURISTIC_LEVEL_                       "MSK_IPAR_MIO_HEURISTIC_LEVEL"
#define MSK_IPAR_MIO_HOTSTART_                              "MSK_IPAR_MIO_HOTSTART"
#define MSK_IPAR_MIO_KEEP_BASIS_                            "MSK_IPAR_MIO_KEEP_BASIS"
#define MSK_IPAR_MIO_LOCAL_BRANCH_NUMBER_                   "MSK_IPAR_MIO_LOCAL_BRANCH_NUMBER"
#define MSK_IPAR_MIO_MAX_NUM_BRANCHES_                      "MSK_IPAR_MIO_MAX_NUM_BRANCHES"
#define MSK_IPAR_MIO_MAX_NUM_RELAXS_                        "MSK_IPAR_MIO_MAX_NUM_RELAXS"
#define MSK_IPAR_MIO_MAX_NUM_SOLUTIONS_                     "MSK_IPAR_MIO_MAX_NUM_SOLUTIONS"
#define MSK_IPAR_MIO_MODE_                                  "MSK_IPAR_MIO_MODE"
#define MSK_IPAR_MIO_MT_USER_CB_                            "MSK_IPAR_MIO_MT_USER_CB"
#define MSK_IPAR_MIO_NODE_OPTIMIZER_                        "MSK_IPAR_MIO_NODE_OPTIMIZER"
#define MSK_IPAR_MIO_NODE_SELECTION_                        "MSK_IPAR_MIO_NODE_SELECTION"
#define MSK_IPAR_MIO_OPTIMIZER_MODE_                        "MSK_IPAR_MIO_OPTIMIZER_MODE"
#define MSK_IPAR_MIO_PRESOLVE_AGGREGATE_                    "MSK_IPAR_MIO_PRESOLVE_AGGREGATE"
#define MSK_IPAR_MIO_PRESOLVE_PROBING_                      "MSK_IPAR_MIO_PRESOLVE_PROBING"
#define MSK_IPAR_MIO_PRESOLVE_USE_                          "MSK_IPAR_MIO_PRESOLVE_USE"
#define MSK_IPAR_MIO_PROBING_LEVEL_                         "MSK_IPAR_MIO_PROBING_LEVEL"
#define MSK_IPAR_MIO_RINS_MAX_NODES_                        "MSK_IPAR_MIO_RINS_MAX_NODES"
#define MSK_IPAR_MIO_ROOT_OPTIMIZER_                        "MSK_IPAR_MIO_ROOT_OPTIMIZER"
#define MSK_IPAR_MIO_STRONG_BRANCH_                         "MSK_IPAR_MIO_STRONG_BRANCH"
#define MSK_IPAR_MIO_USE_MULTITHREADED_OPTIMIZER_           "MSK_IPAR_MIO_USE_MULTITHREADED_OPTIMIZER"
#define MSK_IPAR_MT_SPINCOUNT_                              "MSK_IPAR_MT_SPINCOUNT"
#define MSK_IPAR_NONCONVEX_MAX_ITERATIONS_                  "MSK_IPAR_NONCONVEX_MAX_ITERATIONS"
#define MSK_IPAR_NUM_THREADS_                               "MSK_IPAR_NUM_THREADS"
#define MSK_IPAR_OPF_MAX_TERMS_PER_LINE_                    "MSK_IPAR_OPF_MAX_TERMS_PER_LINE"
#define MSK_IPAR_OPF_WRITE_HEADER_                          "MSK_IPAR_OPF_WRITE_HEADER"
#define MSK_IPAR_OPF_WRITE_HINTS_                           "MSK_IPAR_OPF_WRITE_HINTS"
#define MSK_IPAR_OPF_WRITE_PARAMETERS_                      "MSK_IPAR_OPF_WRITE_PARAMETERS"
#define MSK_IPAR_OPF_WRITE_PROBLEM_                         "MSK_IPAR_OPF_WRITE_PROBLEM"
#define MSK_IPAR_OPF_WRITE_SOL_BAS_                         "MSK_IPAR_OPF_WRITE_SOL_BAS"
#define MSK_IPAR_OPF_WRITE_SOL_ITG_                         "MSK_IPAR_OPF_WRITE_SOL_ITG"
#define MSK_IPAR_OPF_WRITE_SOL_ITR_                         "MSK_IPAR_OPF_WRITE_SOL_ITR"
#define MSK_IPAR_OPF_WRITE_SOLUTIONS_                       "MSK_IPAR_OPF_WRITE_SOLUTIONS"
#define MSK_IPAR_OPTIMIZER_                                 "MSK_IPAR_OPTIMIZER"
#define MSK_IPAR_PARAM_READ_CASE_NAME_                      "MSK_IPAR_PARAM_READ_CASE_NAME"
#define MSK_IPAR_PARAM_READ_IGN_ERROR_                      "MSK_IPAR_PARAM_READ_IGN_ERROR"
#define MSK_IPAR_PRESOLVE_ELIM_FILL_                        "MSK_IPAR_PRESOLVE_ELIM_FILL"
#define MSK_IPAR_PRESOLVE_ELIMINATOR_MAX_NUM_TRIES_         "MSK_IPAR_PRESOLVE_ELIMINATOR_MAX_NUM_TRIES"
#define MSK_IPAR_PRESOLVE_ELIMINATOR_USE_                   "MSK_IPAR_PRESOLVE_ELIMINATOR_USE"
#define MSK_IPAR_PRESOLVE_LEVEL_                            "MSK_IPAR_PRESOLVE_LEVEL"
#define MSK_IPAR_PRESOLVE_LINDEP_ABS_WORK_TRH_              "MSK_IPAR_PRESOLVE_LINDEP_ABS_WORK_TRH"
#define MSK_IPAR_PRESOLVE_LINDEP_REL_WORK_TRH_              "MSK_IPAR_PRESOLVE_LINDEP_REL_WORK_TRH"
#define MSK_IPAR_PRESOLVE_LINDEP_USE_                       "MSK_IPAR_PRESOLVE_LINDEP_USE"
#define MSK_IPAR_PRESOLVE_MAX_NUM_REDUCTIONS_               "MSK_IPAR_PRESOLVE_MAX_NUM_REDUCTIONS"
#define MSK_IPAR_PRESOLVE_USE_                              "MSK_IPAR_PRESOLVE_USE"
#define MSK_IPAR_PRIMAL_REPAIR_OPTIMIZER_                   "MSK_IPAR_PRIMAL_REPAIR_OPTIMIZER"
#define MSK_IPAR_QO_SEPARABLE_REFORMULATION_                "MSK_IPAR_QO_SEPARABLE_REFORMULATION"
#define MSK_IPAR_READ_ANZ_                                  "MSK_IPAR_READ_ANZ"
#define MSK_IPAR_READ_CON_                                  "MSK_IPAR_READ_CON"
#define MSK_IPAR_READ_CONE_                                 "MSK_IPAR_READ_CONE"
#define MSK_IPAR_READ_DATA_COMPRESSED_                      "MSK_IPAR_READ_DATA_COMPRESSED"
#define MSK_IPAR_READ_DATA_FORMAT_                          "MSK_IPAR_READ_DATA_FORMAT"
#define MSK_IPAR_READ_DEBUG_                                "MSK_IPAR_READ_DEBUG"
#define MSK_IPAR_READ_KEEP_FREE_CON_                        "MSK_IPAR_READ_KEEP_FREE_CON"
#define MSK_IPAR_READ_LP_DROP_NEW_VARS_IN_BOU_              "MSK_IPAR_READ_LP_DROP_NEW_VARS_IN_BOU"
#define MSK_IPAR_READ_LP_QUOTED_NAMES_                      "MSK_IPAR_READ_LP_QUOTED_NAMES"
#define MSK_IPAR_READ_MPS_FORMAT_                           "MSK_IPAR_READ_MPS_FORMAT"
#define MSK_IPAR_READ_MPS_KEEP_INT_                         "MSK_IPAR_READ_MPS_KEEP_INT"
#define MSK_IPAR_READ_MPS_OBJ_SENSE_                        "MSK_IPAR_READ_MPS_OBJ_SENSE"
#define MSK_IPAR_READ_MPS_RELAX_                            "MSK_IPAR_READ_MPS_RELAX"
#define MSK_IPAR_READ_MPS_WIDTH_                            "MSK_IPAR_READ_MPS_WIDTH"
#define MSK_IPAR_READ_QNZ_                                  "MSK_IPAR_READ_QNZ"
#define MSK_IPAR_READ_TASK_IGNORE_PARAM_                    "MSK_IPAR_READ_TASK_IGNORE_PARAM"
#define MSK_IPAR_READ_VAR_                                  "MSK_IPAR_READ_VAR"
#define MSK_IPAR_SENSITIVITY_ALL_                           "MSK_IPAR_SENSITIVITY_ALL"
#define MSK_IPAR_SENSITIVITY_OPTIMIZER_                     "MSK_IPAR_SENSITIVITY_OPTIMIZER"
#define MSK_IPAR_SENSITIVITY_TYPE_                          "MSK_IPAR_SENSITIVITY_TYPE"
#define MSK_IPAR_SIM_BASIS_FACTOR_USE_                      "MSK_IPAR_SIM_BASIS_FACTOR_USE"
#define MSK_IPAR_SIM_DEGEN_                                 "MSK_IPAR_SIM_DEGEN"
#define MSK_IPAR_SIM_DUAL_CRASH_                            "MSK_IPAR_SIM_DUAL_CRASH"
#define MSK_IPAR_SIM_DUAL_PHASEONE_METHOD_                  "MSK_IPAR_SIM_DUAL_PHASEONE_METHOD"
#define MSK_IPAR_SIM_DUAL_RESTRICT_SELECTION_               "MSK_IPAR_SIM_DUAL_RESTRICT_SELECTION"
#define MSK_IPAR_SIM_DUAL_SELECTION_                        "MSK_IPAR_SIM_DUAL_SELECTION"
#define MSK_IPAR_SIM_EXPLOIT_DUPVEC_                        "MSK_IPAR_SIM_EXPLOIT_DUPVEC"
#define MSK_IPAR_SIM_HOTSTART_                              "MSK_IPAR_SIM_HOTSTART"
#define MSK_IPAR_SIM_HOTSTART_LU_                           "MSK_IPAR_SIM_HOTSTART_LU"
#define MSK_IPAR_SIM_INTEGER_                               "MSK_IPAR_SIM_INTEGER"
#define MSK_IPAR_SIM_MAX_ITERATIONS_                        "MSK_IPAR_SIM_MAX_ITERATIONS"
#define MSK_IPAR_SIM_MAX_NUM_SETBACKS_                      "MSK_IPAR_SIM_MAX_NUM_SETBACKS"
#define MSK_IPAR_SIM_NON_SINGULAR_                          "MSK_IPAR_SIM_NON_SINGULAR"
#define MSK_IPAR_SIM_PRIMAL_CRASH_                          "MSK_IPAR_SIM_PRIMAL_CRASH"
#define MSK_IPAR_SIM_PRIMAL_PHASEONE_METHOD_                "MSK_IPAR_SIM_PRIMAL_PHASEONE_METHOD"
#define MSK_IPAR_SIM_PRIMAL_RESTRICT_SELECTION_             "MSK_IPAR_SIM_PRIMAL_RESTRICT_SELECTION"
#define MSK_IPAR_SIM_PRIMAL_SELECTION_                      "MSK_IPAR_SIM_PRIMAL_SELECTION"
#define MSK_IPAR_SIM_REFACTOR_FREQ_                         "MSK_IPAR_SIM_REFACTOR_FREQ"
#define MSK_IPAR_SIM_REFORMULATION_                         "MSK_IPAR_SIM_REFORMULATION"
#define MSK_IPAR_SIM_SAVE_LU_                               "MSK_IPAR_SIM_SAVE_LU"
#define MSK_IPAR_SIM_SCALING_                               "MSK_IPAR_SIM_SCALING"
#define MSK_IPAR_SIM_SCALING_METHOD_                        "MSK_IPAR_SIM_SCALING_METHOD"
#define MSK_IPAR_SIM_SOLVE_FORM_                            "MSK_IPAR_SIM_SOLVE_FORM"
#define MSK_IPAR_SIM_STABILITY_PRIORITY_                    "MSK_IPAR_SIM_STABILITY_PRIORITY"
#define MSK_IPAR_SIM_SWITCH_OPTIMIZER_                      "MSK_IPAR_SIM_SWITCH_OPTIMIZER"
#define MSK_IPAR_SOL_FILTER_KEEP_BASIC_                     "MSK_IPAR_SOL_FILTER_KEEP_BASIC"
#define MSK_IPAR_SOL_FILTER_KEEP_RANGED_                    "MSK_IPAR_SOL_FILTER_KEEP_RANGED"
#define MSK_IPAR_SOL_READ_NAME_WIDTH_                       "MSK_IPAR_SOL_READ_NAME_WIDTH"
#define MSK_IPAR_SOL_READ_WIDTH_                            "MSK_IPAR_SOL_READ_WIDTH"
#define MSK_IPAR_SOLUTION_CALLBACK_                         "MSK_IPAR_SOLUTION_CALLBACK"
#define MSK_IPAR_TIMING_LEVEL_                              "MSK_IPAR_TIMING_LEVEL"
#define MSK_IPAR_WARNING_LEVEL_                             "MSK_IPAR_WARNING_LEVEL"
#define MSK_IPAR_WRITE_BAS_CONSTRAINTS_                     "MSK_IPAR_WRITE_BAS_CONSTRAINTS"
#define MSK_IPAR_WRITE_BAS_HEAD_                            "MSK_IPAR_WRITE_BAS_HEAD"
#define MSK_IPAR_WRITE_BAS_VARIABLES_                       "MSK_IPAR_WRITE_BAS_VARIABLES"
#define MSK_IPAR_WRITE_DATA_COMPRESSED_                     "MSK_IPAR_WRITE_DATA_COMPRESSED"
#define MSK_IPAR_WRITE_DATA_FORMAT_                         "MSK_IPAR_WRITE_DATA_FORMAT"
#define MSK_IPAR_WRITE_DATA_PARAM_                          "MSK_IPAR_WRITE_DATA_PARAM"
#define MSK_IPAR_WRITE_FREE_CON_                            "MSK_IPAR_WRITE_FREE_CON"
#define MSK_IPAR_WRITE_GENERIC_NAMES_                       "MSK_IPAR_WRITE_GENERIC_NAMES"
#define MSK_IPAR_WRITE_GENERIC_NAMES_IO_                    "MSK_IPAR_WRITE_GENERIC_NAMES_IO"
#define MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_CONIC_ITEMS_     "MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_CONIC_ITEMS"
#define MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_ITEMS_           "MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_ITEMS"
#define MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_NL_ITEMS_        "MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_NL_ITEMS"
#define MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_PSD_ITEMS_       "MSK_IPAR_WRITE_IGNORE_INCOMPATIBLE_PSD_ITEMS"
#define MSK_IPAR_WRITE_INT_CONSTRAINTS_                     "MSK_IPAR_WRITE_INT_CONSTRAINTS"
#define MSK_IPAR_WRITE_INT_HEAD_                            "MSK_IPAR_WRITE_INT_HEAD"
#define MSK_IPAR_WRITE_INT_VARIABLES_                       "MSK_IPAR_WRITE_INT_VARIABLES"
#define MSK_IPAR_WRITE_LP_LINE_WIDTH_                       "MSK_IPAR_WRITE_LP_LINE_WIDTH"
#define MSK_IPAR_WRITE_LP_QUOTED_NAMES_                     "MSK_IPAR_WRITE_LP_QUOTED_NAMES"
#define MSK_IPAR_WRITE_LP_STRICT_FORMAT_                    "MSK_IPAR_WRITE_LP_STRICT_FORMAT"
#define MSK_IPAR_WRITE_LP_TERMS_PER_LINE_                   "MSK_IPAR_WRITE_LP_TERMS_PER_LINE"
#define MSK_IPAR_WRITE_MPS_INT_                             "MSK_IPAR_WRITE_MPS_INT"
#define MSK_IPAR_WRITE_PRECISION_                           "MSK_IPAR_WRITE_PRECISION"
#define MSK_IPAR_WRITE_SOL_BARVARIABLES_                    "MSK_IPAR_WRITE_SOL_BARVARIABLES"
#define MSK_IPAR_WRITE_SOL_CONSTRAINTS_                     "MSK_IPAR_WRITE_SOL_CONSTRAINTS"
#define MSK_IPAR_WRITE_SOL_HEAD_                            "MSK_IPAR_WRITE_SOL_HEAD"
#define MSK_IPAR_WRITE_SOL_IGNORE_INVALID_NAMES_            "MSK_IPAR_WRITE_SOL_IGNORE_INVALID_NAMES"
#define MSK_IPAR_WRITE_SOL_VARIABLES_                       "MSK_IPAR_WRITE_SOL_VARIABLES"
#define MSK_IPAR_WRITE_TASK_INC_SOL_                        "MSK_IPAR_WRITE_TASK_INC_SOL"
#define MSK_IPAR_WRITE_XML_MODE_                            "MSK_IPAR_WRITE_XML_MODE"

#define MSK_IINF_ANA_PRO_NUM_CON_                           "MSK_IINF_ANA_PRO_NUM_CON"
#define MSK_IINF_ANA_PRO_NUM_CON_EQ_                        "MSK_IINF_ANA_PRO_NUM_CON_EQ"
#define MSK_IINF_ANA_PRO_NUM_CON_FR_                        "MSK_IINF_ANA_PRO_NUM_CON_FR"
#define MSK_IINF_ANA_PRO_NUM_CON_LO_                        "MSK_IINF_ANA_PRO_NUM_CON_LO"
#define MSK_IINF_ANA_PRO_NUM_CON_RA_                        "MSK_IINF_ANA_PRO_NUM_CON_RA"
#define MSK_IINF_ANA_PRO_NUM_CON_UP_                        "MSK_IINF_ANA_PRO_NUM_CON_UP"
#define MSK_IINF_ANA_PRO_NUM_VAR_                           "MSK_IINF_ANA_PRO_NUM_VAR"
#define MSK_IINF_ANA_PRO_NUM_VAR_BIN_                       "MSK_IINF_ANA_PRO_NUM_VAR_BIN"
#define MSK_IINF_ANA_PRO_NUM_VAR_CONT_                      "MSK_IINF_ANA_PRO_NUM_VAR_CONT"
#define MSK_IINF_ANA_PRO_NUM_VAR_EQ_                        "MSK_IINF_ANA_PRO_NUM_VAR_EQ"
#define MSK_IINF_ANA_PRO_NUM_VAR_FR_                        "MSK_IINF_ANA_PRO_NUM_VAR_FR"
#define MSK_IINF_ANA_PRO_NUM_VAR_INT_                       "MSK_IINF_ANA_PRO_NUM_VAR_INT"
#define MSK_IINF_ANA_PRO_NUM_VAR_LO_                        "MSK_IINF_ANA_PRO_NUM_VAR_LO"
#define MSK_IINF_ANA_PRO_NUM_VAR_RA_                        "MSK_IINF_ANA_PRO_NUM_VAR_RA"
#define MSK_IINF_ANA_PRO_NUM_VAR_UP_                        "MSK_IINF_ANA_PRO_NUM_VAR_UP"
#define MSK_IINF_CONCURRENT_FASTEST_OPTIMIZER_              "MSK_IINF_CONCURRENT_FASTEST_OPTIMIZER"
#define MSK_IINF_INTPNT_FACTOR_DIM_DENSE_                   "MSK_IINF_INTPNT_FACTOR_DIM_DENSE"
#define MSK_IINF_INTPNT_ITER_                               "MSK_IINF_INTPNT_ITER"
#define MSK_IINF_INTPNT_NUM_THREADS_                        "MSK_IINF_INTPNT_NUM_THREADS"
#define MSK_IINF_INTPNT_SOLVE_DUAL_                         "MSK_IINF_INTPNT_SOLVE_DUAL"
#define MSK_IINF_MIO_CONSTRUCT_NUM_ROUNDINGS_               "MSK_IINF_MIO_CONSTRUCT_NUM_ROUNDINGS"
#define MSK_IINF_MIO_CONSTRUCT_SOLUTION_                    "MSK_IINF_MIO_CONSTRUCT_SOLUTION"
#define MSK_IINF_MIO_INITIAL_SOLUTION_                      "MSK_IINF_MIO_INITIAL_SOLUTION"
#define MSK_IINF_MIO_NUM_ACTIVE_NODES_                      "MSK_IINF_MIO_NUM_ACTIVE_NODES"
#define MSK_IINF_MIO_NUM_BASIS_CUTS_                        "MSK_IINF_MIO_NUM_BASIS_CUTS"
#define MSK_IINF_MIO_NUM_BRANCH_                            "MSK_IINF_MIO_NUM_BRANCH"
#define MSK_IINF_MIO_NUM_CARDGUB_CUTS_                      "MSK_IINF_MIO_NUM_CARDGUB_CUTS"
#define MSK_IINF_MIO_NUM_CLIQUE_CUTS_                       "MSK_IINF_MIO_NUM_CLIQUE_CUTS"
#define MSK_IINF_MIO_NUM_COEF_REDC_CUTS_                    "MSK_IINF_MIO_NUM_COEF_REDC_CUTS"
#define MSK_IINF_MIO_NUM_CONTRA_CUTS_                       "MSK_IINF_MIO_NUM_CONTRA_CUTS"
#define MSK_IINF_MIO_NUM_DISAGG_CUTS_                       "MSK_IINF_MIO_NUM_DISAGG_CUTS"
#define MSK_IINF_MIO_NUM_FLOW_COVER_CUTS_                   "MSK_IINF_MIO_NUM_FLOW_COVER_CUTS"
#define MSK_IINF_MIO_NUM_GCD_CUTS_                          "MSK_IINF_MIO_NUM_GCD_CUTS"
#define MSK_IINF_MIO_NUM_GOMORY_CUTS_                       "MSK_IINF_MIO_NUM_GOMORY_CUTS"
#define MSK_IINF_MIO_NUM_GUB_COVER_CUTS_                    "MSK_IINF_MIO_NUM_GUB_COVER_CUTS"
#define MSK_IINF_MIO_NUM_INT_SOLUTIONS_                     "MSK_IINF_MIO_NUM_INT_SOLUTIONS"
#define MSK_IINF_MIO_NUM_KNAPSUR_COVER_CUTS_                "MSK_IINF_MIO_NUM_KNAPSUR_COVER_CUTS"
#define MSK_IINF_MIO_NUM_LATTICE_CUTS_                      "MSK_IINF_MIO_NUM_LATTICE_CUTS"
#define MSK_IINF_MIO_NUM_LIFT_CUTS_                         "MSK_IINF_MIO_NUM_LIFT_CUTS"
#define MSK_IINF_MIO_NUM_OBJ_CUTS_                          "MSK_IINF_MIO_NUM_OBJ_CUTS"
#define MSK_IINF_MIO_NUM_PLAN_LOC_CUTS_                     "MSK_IINF_MIO_NUM_PLAN_LOC_CUTS"
#define MSK_IINF_MIO_NUM_RELAX_                             "MSK_IINF_MIO_NUM_RELAX"
#define MSK_IINF_MIO_NUMCON_                                "MSK_IINF_MIO_NUMCON"
#define MSK_IINF_MIO_NUMINT_                                "MSK_IINF_MIO_NUMINT"
#define MSK_IINF_MIO_NUMVAR_                                "MSK_IINF_MIO_NUMVAR"
#define MSK_IINF_MIO_OBJ_BOUND_DEFINED_                     "MSK_IINF_MIO_OBJ_BOUND_DEFINED"
#define MSK_IINF_MIO_TOTAL_NUM_CUTS_                        "MSK_IINF_MIO_TOTAL_NUM_CUTS"
#define MSK_IINF_MIO_USER_OBJ_CUT_                          "MSK_IINF_MIO_USER_OBJ_CUT"
#define MSK_IINF_OPT_NUMCON_                                "MSK_IINF_OPT_NUMCON"
#define MSK_IINF_OPT_NUMVAR_                                "MSK_IINF_OPT_NUMVAR"
#define MSK_IINF_OPTIMIZE_RESPONSE_                         "MSK_IINF_OPTIMIZE_RESPONSE"
#define MSK_IINF_RD_NUMBARVAR_                              "MSK_IINF_RD_NUMBARVAR"
#define MSK_IINF_RD_NUMCON_                                 "MSK_IINF_RD_NUMCON"
#define MSK_IINF_RD_NUMCONE_                                "MSK_IINF_RD_NUMCONE"
#define MSK_IINF_RD_NUMINTVAR_                              "MSK_IINF_RD_NUMINTVAR"
#define MSK_IINF_RD_NUMQ_                                   "MSK_IINF_RD_NUMQ"
#define MSK_IINF_RD_NUMVAR_                                 "MSK_IINF_RD_NUMVAR"
#define MSK_IINF_RD_PROTYPE_                                "MSK_IINF_RD_PROTYPE"
#define MSK_IINF_SIM_DUAL_DEG_ITER_                         "MSK_IINF_SIM_DUAL_DEG_ITER"
#define MSK_IINF_SIM_DUAL_HOTSTART_                         "MSK_IINF_SIM_DUAL_HOTSTART"
#define MSK_IINF_SIM_DUAL_HOTSTART_LU_                      "MSK_IINF_SIM_DUAL_HOTSTART_LU"
#define MSK_IINF_SIM_DUAL_INF_ITER_                         "MSK_IINF_SIM_DUAL_INF_ITER"
#define MSK_IINF_SIM_DUAL_ITER_                             "MSK_IINF_SIM_DUAL_ITER"
#define MSK_IINF_SIM_NETWORK_DUAL_DEG_ITER_                 "MSK_IINF_SIM_NETWORK_DUAL_DEG_ITER"
#define MSK_IINF_SIM_NETWORK_DUAL_HOTSTART_                 "MSK_IINF_SIM_NETWORK_DUAL_HOTSTART"
#define MSK_IINF_SIM_NETWORK_DUAL_HOTSTART_LU_              "MSK_IINF_SIM_NETWORK_DUAL_HOTSTART_LU"
#define MSK_IINF_SIM_NETWORK_DUAL_INF_ITER_                 "MSK_IINF_SIM_NETWORK_DUAL_INF_ITER"
#define MSK_IINF_SIM_NETWORK_DUAL_ITER_                     "MSK_IINF_SIM_NETWORK_DUAL_ITER"
#define MSK_IINF_SIM_NETWORK_PRIMAL_DEG_ITER_               "MSK_IINF_SIM_NETWORK_PRIMAL_DEG_ITER"
#define MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART_               "MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART"
#define MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART_LU_            "MSK_IINF_SIM_NETWORK_PRIMAL_HOTSTART_LU"
#define MSK_IINF_SIM_NETWORK_PRIMAL_INF_ITER_               "MSK_IINF_SIM_NETWORK_PRIMAL_INF_ITER"
#define MSK_IINF_SIM_NETWORK_PRIMAL_ITER_                   "MSK_IINF_SIM_NETWORK_PRIMAL_ITER"
#define MSK_IINF_SIM_NUMCON_                                "MSK_IINF_SIM_NUMCON"
#define MSK_IINF_SIM_NUMVAR_                                "MSK_IINF_SIM_NUMVAR"
#define MSK_IINF_SIM_PRIMAL_DEG_ITER_                       "MSK_IINF_SIM_PRIMAL_DEG_ITER"
#define MSK_IINF_SIM_PRIMAL_DUAL_DEG_ITER_                  "MSK_IINF_SIM_PRIMAL_DUAL_DEG_ITER"
#define MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART_                  "MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART"
#define MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART_LU_               "MSK_IINF_SIM_PRIMAL_DUAL_HOTSTART_LU"
#define MSK_IINF_SIM_PRIMAL_DUAL_INF_ITER_                  "MSK_IINF_SIM_PRIMAL_DUAL_INF_ITER"
#define MSK_IINF_SIM_PRIMAL_DUAL_ITER_                      "MSK_IINF_SIM_PRIMAL_DUAL_ITER"
#define MSK_IINF_SIM_PRIMAL_HOTSTART_                       "MSK_IINF_SIM_PRIMAL_HOTSTART"
#define MSK_IINF_SIM_PRIMAL_HOTSTART_LU_                    "MSK_IINF_SIM_PRIMAL_HOTSTART_LU"
#define MSK_IINF_SIM_PRIMAL_INF_ITER_                       "MSK_IINF_SIM_PRIMAL_INF_ITER"
#define MSK_IINF_SIM_PRIMAL_ITER_                           "MSK_IINF_SIM_PRIMAL_ITER"
#define MSK_IINF_SIM_SOLVE_DUAL_                            "MSK_IINF_SIM_SOLVE_DUAL"
#define MSK_IINF_SOL_BAS_PROSTA_                            "MSK_IINF_SOL_BAS_PROSTA"
#define MSK_IINF_SOL_BAS_SOLSTA_                            "MSK_IINF_SOL_BAS_SOLSTA"
#define MSK_IINF_SOL_INT_PROSTA_                            "MSK_IINF_SOL_INT_PROSTA"
#define MSK_IINF_SOL_INT_SOLSTA_                            "MSK_IINF_SOL_INT_SOLSTA"
#define MSK_IINF_SOL_ITG_PROSTA_                            "MSK_IINF_SOL_ITG_PROSTA"
#define MSK_IINF_SOL_ITG_SOLSTA_                            "MSK_IINF_SOL_ITG_SOLSTA"
#define MSK_IINF_SOL_ITR_PROSTA_                            "MSK_IINF_SOL_ITR_PROSTA"
#define MSK_IINF_SOL_ITR_SOLSTA_                            "MSK_IINF_SOL_ITR_SOLSTA"
#define MSK_IINF_STO_NUM_A_CACHE_FLUSHES_                   "MSK_IINF_STO_NUM_A_CACHE_FLUSHES"
#define MSK_IINF_STO_NUM_A_REALLOC_                         "MSK_IINF_STO_NUM_A_REALLOC"
#define MSK_IINF_STO_NUM_A_TRANSPOSES_                      "MSK_IINF_STO_NUM_A_TRANSPOSES"

#define MSK_DINF_BI_CLEAN_DUAL_TIME_                        "MSK_DINF_BI_CLEAN_DUAL_TIME"
#define MSK_DINF_BI_CLEAN_PRIMAL_DUAL_TIME_                 "MSK_DINF_BI_CLEAN_PRIMAL_DUAL_TIME"
#define MSK_DINF_BI_CLEAN_PRIMAL_TIME_                      "MSK_DINF_BI_CLEAN_PRIMAL_TIME"
#define MSK_DINF_BI_CLEAN_TIME_                             "MSK_DINF_BI_CLEAN_TIME"
#define MSK_DINF_BI_DUAL_TIME_                              "MSK_DINF_BI_DUAL_TIME"
#define MSK_DINF_BI_PRIMAL_TIME_                            "MSK_DINF_BI_PRIMAL_TIME"
#define MSK_DINF_BI_TIME_                                   "MSK_DINF_BI_TIME"
#define MSK_DINF_CONCURRENT_TIME_                           "MSK_DINF_CONCURRENT_TIME"
#define MSK_DINF_INTPNT_DUAL_FEAS_                          "MSK_DINF_INTPNT_DUAL_FEAS"
#define MSK_DINF_INTPNT_DUAL_OBJ_                           "MSK_DINF_INTPNT_DUAL_OBJ"
#define MSK_DINF_INTPNT_FACTOR_NUM_FLOPS_                   "MSK_DINF_INTPNT_FACTOR_NUM_FLOPS"
#define MSK_DINF_INTPNT_OPT_STATUS_                         "MSK_DINF_INTPNT_OPT_STATUS"
#define MSK_DINF_INTPNT_ORDER_TIME_                         "MSK_DINF_INTPNT_ORDER_TIME"
#define MSK_DINF_INTPNT_PRIMAL_FEAS_                        "MSK_DINF_INTPNT_PRIMAL_FEAS"
#define MSK_DINF_INTPNT_PRIMAL_OBJ_                         "MSK_DINF_INTPNT_PRIMAL_OBJ"
#define MSK_DINF_INTPNT_TIME_                               "MSK_DINF_INTPNT_TIME"
#define MSK_DINF_MIO_CG_SEPERATION_TIME_                    "MSK_DINF_MIO_CG_SEPERATION_TIME"
#define MSK_DINF_MIO_CMIR_SEPERATION_TIME_                  "MSK_DINF_MIO_CMIR_SEPERATION_TIME"
#define MSK_DINF_MIO_CONSTRUCT_SOLUTION_OBJ_                "MSK_DINF_MIO_CONSTRUCT_SOLUTION_OBJ"
#define MSK_DINF_MIO_DUAL_BOUND_AFTER_PRESOLVE_             "MSK_DINF_MIO_DUAL_BOUND_AFTER_PRESOLVE"
#define MSK_DINF_MIO_HEURISTIC_TIME_                        "MSK_DINF_MIO_HEURISTIC_TIME"
#define MSK_DINF_MIO_OBJ_ABS_GAP_                           "MSK_DINF_MIO_OBJ_ABS_GAP"
#define MSK_DINF_MIO_OBJ_BOUND_                             "MSK_DINF_MIO_OBJ_BOUND"
#define MSK_DINF_MIO_OBJ_INT_                               "MSK_DINF_MIO_OBJ_INT"
#define MSK_DINF_MIO_OBJ_REL_GAP_                           "MSK_DINF_MIO_OBJ_REL_GAP"
#define MSK_DINF_MIO_OPTIMIZER_TIME_                        "MSK_DINF_MIO_OPTIMIZER_TIME"
#define MSK_DINF_MIO_PROBING_TIME_                          "MSK_DINF_MIO_PROBING_TIME"
#define MSK_DINF_MIO_ROOT_CUTGEN_TIME_                      "MSK_DINF_MIO_ROOT_CUTGEN_TIME"
#define MSK_DINF_MIO_ROOT_OPTIMIZER_TIME_                   "MSK_DINF_MIO_ROOT_OPTIMIZER_TIME"
#define MSK_DINF_MIO_ROOT_PRESOLVE_TIME_                    "MSK_DINF_MIO_ROOT_PRESOLVE_TIME"
#define MSK_DINF_MIO_TIME_                                  "MSK_DINF_MIO_TIME"
#define MSK_DINF_MIO_USER_OBJ_CUT_                          "MSK_DINF_MIO_USER_OBJ_CUT"
#define MSK_DINF_OPTIMIZER_TIME_                            "MSK_DINF_OPTIMIZER_TIME"
#define MSK_DINF_PRESOLVE_ELI_TIME_                         "MSK_DINF_PRESOLVE_ELI_TIME"
#define MSK_DINF_PRESOLVE_LINDEP_TIME_                      "MSK_DINF_PRESOLVE_LINDEP_TIME"
#define MSK_DINF_PRESOLVE_TIME_                             "MSK_DINF_PRESOLVE_TIME"
#define MSK_DINF_PRIMAL_REPAIR_PENALTY_OBJ_                 "MSK_DINF_PRIMAL_REPAIR_PENALTY_OBJ"
#define MSK_DINF_QCQO_REFORMULATE_TIME_                     "MSK_DINF_QCQO_REFORMULATE_TIME"
#define MSK_DINF_RD_TIME_                                   "MSK_DINF_RD_TIME"
#define MSK_DINF_SIM_DUAL_TIME_                             "MSK_DINF_SIM_DUAL_TIME"
#define MSK_DINF_SIM_FEAS_                                  "MSK_DINF_SIM_FEAS"
#define MSK_DINF_SIM_NETWORK_DUAL_TIME_                     "MSK_DINF_SIM_NETWORK_DUAL_TIME"
#define MSK_DINF_SIM_NETWORK_PRIMAL_TIME_                   "MSK_DINF_SIM_NETWORK_PRIMAL_TIME"
#define MSK_DINF_SIM_NETWORK_TIME_                          "MSK_DINF_SIM_NETWORK_TIME"
#define MSK_DINF_SIM_OBJ_                                   "MSK_DINF_SIM_OBJ"
#define MSK_DINF_SIM_PRIMAL_DUAL_TIME_                      "MSK_DINF_SIM_PRIMAL_DUAL_TIME"
#define MSK_DINF_SIM_PRIMAL_TIME_                           "MSK_DINF_SIM_PRIMAL_TIME"
#define MSK_DINF_SIM_TIME_                                  "MSK_DINF_SIM_TIME"
#define MSK_DINF_SOL_BAS_DUAL_OBJ_                          "MSK_DINF_SOL_BAS_DUAL_OBJ"
#define MSK_DINF_SOL_BAS_DVIOLCON_                          "MSK_DINF_SOL_BAS_DVIOLCON"
#define MSK_DINF_SOL_BAS_DVIOLVAR_                          "MSK_DINF_SOL_BAS_DVIOLVAR"
#define MSK_DINF_SOL_BAS_PRIMAL_OBJ_                        "MSK_DINF_SOL_BAS_PRIMAL_OBJ"
#define MSK_DINF_SOL_BAS_PVIOLCON_                          "MSK_DINF_SOL_BAS_PVIOLCON"
#define MSK_DINF_SOL_BAS_PVIOLVAR_                          "MSK_DINF_SOL_BAS_PVIOLVAR"
#define MSK_DINF_SOL_ITG_PRIMAL_OBJ_                        "MSK_DINF_SOL_ITG_PRIMAL_OBJ"
#define MSK_DINF_SOL_ITG_PVIOLBARVAR_                       "MSK_DINF_SOL_ITG_PVIOLBARVAR"
#define MSK_DINF_SOL_ITG_PVIOLCON_                          "MSK_DINF_SOL_ITG_PVIOLCON"
#define MSK_DINF_SOL_ITG_PVIOLCONES_                        "MSK_DINF_SOL_ITG_PVIOLCONES"
#define MSK_DINF_SOL_ITG_PVIOLITG_                          "MSK_DINF_SOL_ITG_PVIOLITG"
#define MSK_DINF_SOL_ITG_PVIOLVAR_                          "MSK_DINF_SOL_ITG_PVIOLVAR"
#define MSK_DINF_SOL_ITR_DUAL_OBJ_                          "MSK_DINF_SOL_ITR_DUAL_OBJ"
#define MSK_DINF_SOL_ITR_DVIOLBARVAR_                       "MSK_DINF_SOL_ITR_DVIOLBARVAR"
#define MSK_DINF_SOL_ITR_DVIOLCON_                          "MSK_DINF_SOL_ITR_DVIOLCON"
#define MSK_DINF_SOL_ITR_DVIOLCONES_                        "MSK_DINF_SOL_ITR_DVIOLCONES"
#define MSK_DINF_SOL_ITR_DVIOLVAR_                          "MSK_DINF_SOL_ITR_DVIOLVAR"
#define MSK_DINF_SOL_ITR_PRIMAL_OBJ_                        "MSK_DINF_SOL_ITR_PRIMAL_OBJ"
#define MSK_DINF_SOL_ITR_PVIOLBARVAR_                       "MSK_DINF_SOL_ITR_PVIOLBARVAR"
#define MSK_DINF_SOL_ITR_PVIOLCON_                          "MSK_DINF_SOL_ITR_PVIOLCON"
#define MSK_DINF_SOL_ITR_PVIOLCONES_                        "MSK_DINF_SOL_ITR_PVIOLCONES"
#define MSK_DINF_SOL_ITR_PVIOLVAR_                          "MSK_DINF_SOL_ITR_PVIOLVAR"

#define MSK_LIINF_BI_CLEAN_DUAL_DEG_ITER_                   "MSK_LIINF_BI_CLEAN_DUAL_DEG_ITER"
#define MSK_LIINF_BI_CLEAN_DUAL_ITER_                       "MSK_LIINF_BI_CLEAN_DUAL_ITER"
#define MSK_LIINF_BI_CLEAN_PRIMAL_DEG_ITER_                 "MSK_LIINF_BI_CLEAN_PRIMAL_DEG_ITER"
#define MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_DEG_ITER_            "MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_DEG_ITER"
#define MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_ITER_                "MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_ITER"
#define MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_SUB_ITER_            "MSK_LIINF_BI_CLEAN_PRIMAL_DUAL_SUB_ITER"
#define MSK_LIINF_BI_CLEAN_PRIMAL_ITER_                     "MSK_LIINF_BI_CLEAN_PRIMAL_ITER"
#define MSK_LIINF_BI_DUAL_ITER_                             "MSK_LIINF_BI_DUAL_ITER"
#define MSK_LIINF_BI_PRIMAL_ITER_                           "MSK_LIINF_BI_PRIMAL_ITER"
#define MSK_LIINF_INTPNT_FACTOR_NUM_NZ_                     "MSK_LIINF_INTPNT_FACTOR_NUM_NZ"
#define MSK_LIINF_MIO_INTPNT_ITER_                          "MSK_LIINF_MIO_INTPNT_ITER"
#define MSK_LIINF_MIO_SIMPLEX_ITER_                         "MSK_LIINF_MIO_SIMPLEX_ITER"
#define MSK_LIINF_RD_NUMANZ_                                "MSK_LIINF_RD_NUMANZ"
#define MSK_LIINF_RD_NUMQNZ_                                "MSK_LIINF_RD_NUMQNZ"



/* Typedefs */

typedef char       MSKchart;
typedef void     * MSKvoid_t;
#define MSKintt  MSKint32t 
#define MSKidxt  MSKint32t 
#define MSKlidxt MSKint32t 
#define MSKlintt MSKint32t 

#ifdef  MSKUINT64
typedef MSKUINT64 __mskuint64;
#else
typedef unsigned long long __mskuint64;
#endif
#ifdef  MSKINT64
typedef MSKINT64 __mskint64;
#else
typedef long long __mskint64;
#endif

#if defined(LLONG_MAX) && LLONG_MAX <= INT_MAX
#warning "Expected (long long) to be a 64bit type. MOSEK API functions may not work."
#endif
typedef int          __mskint32;
typedef unsigned int __mskuint32;


/* Enumeration typedefs */
#ifndef MSK_NO_ENUMS
typedef int                     MSKsolveforme;
typedef enum MSKproblemitem_enum     MSKproblemiteme;
typedef enum MSKaccmode_enum         MSKaccmodee;
typedef int                     MSKsensitivitytypee;
typedef enum MSKuplo_enum            MSKuploe;
typedef enum MSKintpnthotstart_enum  MSKintpnthotstarte;
typedef enum MSKsparam_enum          MSKsparame;
typedef enum MSKiparam_enum          MSKiparame;
typedef enum MSKsolsta_enum          MSKsolstae;
typedef enum MSKobjsense_enum        MSKobjsensee;
typedef enum MSKsolitem_enum         MSKsoliteme;
typedef enum MSKboundkey_enum        MSKboundkeye;
typedef int                     MSKbasindtypee;
typedef int                     MSKbranchdire;
typedef enum MSKliinfitem_enum       MSKliinfiteme;
typedef enum MSKstreamtype_enum      MSKstreamtypee;
typedef enum MSKsimhotstart_enum     MSKsimhotstarte;
typedef enum MSKcallbackcode_enum    MSKcallbackcodee;
typedef enum MSKsymmattype_enum      MSKsymmattypee;
typedef enum MSKfeature_enum         MSKfeaturee;
typedef enum MSKmark_enum            MSKmarke;
typedef enum MSKconetype_enum        MSKconetypee;
typedef int                     MSKfeasrepairtypee;
typedef int                     MSKiomodee;
typedef int                     MSKsimseltypee;
typedef enum MSKmsgkey_enum          MSKmsgkeye;
typedef int                     MSKmiomodee;
typedef enum MSKdinfitem_enum        MSKdinfiteme;
typedef enum MSKparametertype_enum   MSKparametertypee;
typedef enum MSKrescodetype_enum     MSKrescodetypee;
typedef enum MSKprosta_enum          MSKprostae;
typedef int                     MSKscalingtypee;
typedef enum MSKrescode_enum         MSKrescodee;
typedef int                     MSKmionodeseltypee;
typedef enum MSKtranspose_enum       MSKtransposee;
typedef int                     MSKonoffkeye;
typedef enum MSKsimdegen_enum        MSKsimdegene;
typedef int                     MSKdataformate;
typedef int                     MSKorderingtypee;
typedef enum MSKproblemtype_enum     MSKproblemtypee;
typedef enum MSKinftype_enum         MSKinftypee;
typedef enum MSKdparam_enum          MSKdparame;
typedef enum MSKsimdupvec_enum       MSKsimdupvece;
typedef int                     MSKcompresstypee;
typedef enum MSKnametype_enum        MSKnametypee;
typedef int                     MSKmpsformate;
typedef enum MSKvariabletype_enum    MSKvariabletypee;
typedef int                     MSKcheckconvexitytypee;
typedef enum MSKlanguage_enum        MSKlanguagee;
typedef int                     MSKstartpointtypee;
typedef enum MSKsoltype_enum         MSKsoltypee;
typedef int                     MSKscalingmethode;
typedef int                     MSKvaluee;
typedef enum MSKstakey_enum          MSKstakeye;
typedef enum MSKsimreform_enum       MSKsimreforme;
typedef enum MSKiinfitem_enum        MSKiinfiteme;
typedef enum MSKxmlwriteroutputtype_enum MSKxmlwriteroutputtypee;
typedef int                     MSKoptimizertypee;
typedef int                     MSKpresolvemodee;
typedef int                     MSKmiocontsoltypee;
#else
typedef int                     MSKsolveforme;
typedef int                     MSKproblemiteme;
typedef int                     MSKaccmodee;
typedef int                     MSKsensitivitytypee;
typedef int                     MSKuploe;
typedef int                     MSKintpnthotstarte;
typedef int                     MSKsparame;
typedef int                     MSKiparame;
typedef int                     MSKsolstae;
typedef int                     MSKobjsensee;
typedef int                     MSKsoliteme;
typedef int                     MSKboundkeye;
typedef int                     MSKbasindtypee;
typedef int                     MSKbranchdire;
typedef int                     MSKliinfiteme;
typedef int                     MSKstreamtypee;
typedef int                     MSKsimhotstarte;
typedef int                     MSKcallbackcodee;
typedef int                     MSKsymmattypee;
typedef int                     MSKfeaturee;
typedef int                     MSKmarke;
typedef int                     MSKconetypee;
typedef int                     MSKfeasrepairtypee;
typedef int                     MSKiomodee;
typedef int                     MSKsimseltypee;
typedef int                     MSKmsgkeye;
typedef int                     MSKmiomodee;
typedef int                     MSKdinfiteme;
typedef int                     MSKparametertypee;
typedef int                     MSKrescodetypee;
typedef int                     MSKprostae;
typedef int                     MSKscalingtypee;
typedef int                     MSKrescodee;
typedef int                     MSKmionodeseltypee;
typedef int                     MSKtransposee;
typedef int                     MSKonoffkeye;
typedef int                     MSKsimdegene;
typedef int                     MSKdataformate;
typedef int                     MSKorderingtypee;
typedef int                     MSKproblemtypee;
typedef int                     MSKinftypee;
typedef int                     MSKdparame;
typedef int                     MSKsimdupvece;
typedef int                     MSKcompresstypee;
typedef int                     MSKnametypee;
typedef int                     MSKmpsformate;
typedef int                     MSKvariabletypee;
typedef int                     MSKcheckconvexitytypee;
typedef int                     MSKlanguagee;
typedef int                     MSKstartpointtypee;
typedef int                     MSKsoltypee;
typedef int                     MSKscalingmethode;
typedef int                     MSKvaluee;
typedef int                     MSKstakeye;
typedef int                     MSKsimreforme;
typedef int                     MSKiinfiteme;
typedef int                     MSKxmlwriteroutputtypee;
typedef int                     MSKoptimizertypee;
typedef int                     MSKpresolvemodee;
typedef int                     MSKmiocontsoltypee;
#endif

/* Simple typedefs */
typedef void * MSKenv_t;

typedef void * MSKtask_t;

typedef void * MSKuserhandle_t;

typedef int MSKbooleant;

typedef __mskint32 MSKint32t;

typedef __mskint64 MSKint64t;

typedef wchar_t MSKwchart;

typedef double MSKrealt;

typedef char * MSKstring_t;

/* Function typedefs */
typedef MSKint32t  (MSKAPI * MSKcallbackfunc) (
	MSKtask_t task,
	MSKuserhandle_t usrptr,
	MSKcallbackcodee caller,
	MSKCONST MSKrealt * douinf,
	MSKCONST MSKint32t * intinf,
	MSKCONST MSKint64t * lintinf);

typedef void  (MSKAPI * MSKexitfunc) (
	MSKuserhandle_t usrptr,
	MSKCONST char * file,
	MSKint32t line,
	MSKCONST char * msg);

typedef void  (MSKAPI * MSKfreefunc) (
	MSKuserhandle_t usrptr,
	void * buffer);

typedef void *  (MSKAPI * MSKmallocfunc) (
	MSKuserhandle_t usrptr,
	MSKCONST size_t size);

typedef void *  (MSKAPI * MSKcallocfunc) (
	MSKuserhandle_t usrptr,
	MSKCONST size_t num,
	MSKCONST size_t size);

typedef void *  (MSKAPI * MSKreallocfunc) (
	MSKuserhandle_t usrptr,
	void * ptr,
	MSKCONST size_t size);

typedef MSKint32t  (MSKAPI * MSKnlgetspfunc) (
	MSKuserhandle_t nlhandle,
	MSKint32t * numgrdobjnz,
	MSKint32t * grdobjsub,
	MSKint32t i,
	MSKbooleant * convali,
	MSKint32t * grdconinz,
	MSKint32t * grdconisub,
	MSKint32t yo,
	MSKint32t numycnz,
	MSKCONST MSKint32t * ycsub,
	MSKint32t maxnumhesnz,
	MSKint32t * numhesnz,
	MSKint32t * hessubi,
	MSKint32t * hessubj);

typedef MSKint32t  (MSKAPI * MSKnlgetvafunc) (
	MSKuserhandle_t nlhandle,
	MSKCONST MSKrealt * xx,
	MSKrealt yo,
	MSKCONST MSKrealt * yc,
	MSKrealt * objval,
	MSKint32t * numgrdobjnz,
	MSKint32t * grdobjsub,
	MSKrealt * grdobjval,
	MSKint32t numi,
	MSKCONST MSKint32t * subi,
	MSKrealt * conval,
	MSKCONST MSKint32t * grdconptrb,
	MSKCONST MSKint32t * grdconptre,
	MSKCONST MSKint32t * grdconsub,
	MSKrealt * grdconval,
	MSKrealt * grdlag,
	MSKint32t maxnumhesnz,
	MSKint32t * numhesnz,
	MSKint32t * hessubi,
	MSKint32t * hessubj,
	MSKrealt * hesval);

typedef void  (MSKAPI * MSKstreamfunc) (
	MSKuserhandle_t handle,
	MSKCONST char * str);

typedef MSKrescodee  (MSKAPI * MSKresponsefunc) (
	MSKuserhandle_t handle,
	MSKrescodee r,
	MSKCONST char * msg);




/* Functions */

/* using __cplusplus */
#ifdef __cplusplus
extern "C" {
#endif

/* MSK_analyzeproblem */
MSKrescodee (MSKAPI MSK_analyzeproblem) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_analyzenames */
MSKrescodee (MSKAPI MSK_analyzenames) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKnametypee nametype);

/* MSK_analyzesolution */
MSKrescodee (MSKAPI MSK_analyzesolution) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKsoltypee whichsol);

/* MSK_initbasissolve */
MSKrescodee (MSKAPI MSK_initbasissolve) (	MSKtask_t task,	MSKint32t * basis);

/* MSK_solvewithbasis */
MSKrescodee (MSKAPI MSK_solvewithbasis) (	MSKtask_t task,	MSKint32t transp,	MSKint32t * numnz,	MSKint32t * sub,	MSKrealt * val);

/* MSK_basiscond */
MSKrescodee (MSKAPI MSK_basiscond) (	MSKtask_t task,	MSKrealt * nrmbasis,	MSKrealt * nrminvbasis);

/* MSK_appendcons */
MSKrescodee (MSKAPI MSK_appendcons) (	MSKtask_t task,	MSKint32t num);

/* MSK_appendvars */
MSKrescodee (MSKAPI MSK_appendvars) (	MSKtask_t task,	MSKint32t num);

/* MSK_removecons */
MSKrescodee (MSKAPI MSK_removecons) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subset);

/* MSK_removevars */
MSKrescodee (MSKAPI MSK_removevars) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subset);

/* MSK_removebarvars */
MSKrescodee (MSKAPI MSK_removebarvars) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subset);

/* MSK_removecones */
MSKrescodee (MSKAPI MSK_removecones) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subset);

/* MSK_appendbarvars */
MSKrescodee (MSKAPI MSK_appendbarvars) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * dim);

/* MSK_appendcone */
MSKrescodee (MSKAPI MSK_appendcone) (	MSKtask_t task,	MSKconetypee conetype,	MSKrealt conepar,	MSKint32t nummem,	MSKCONST MSKint32t * submem);

/* MSK_appendconeseq */
MSKrescodee (MSKAPI MSK_appendconeseq) (	MSKtask_t task,	MSKconetypee conetype,	MSKrealt conepar,	MSKint32t nummem,	MSKint32t j);

/* MSK_appendconesseq */
MSKrescodee (MSKAPI MSK_appendconesseq) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKconetypee * conetype,	MSKCONST MSKrealt * conepar,	MSKCONST MSKint32t * nummem,	MSKint32t j);

/* MSK_bktostr */
MSKrescodee (MSKAPI MSK_bktostr) (	MSKtask_t task,	MSKboundkeye bk,	char * str);

/* MSK_callbackcodetostr */
MSKrescodee (MSKAPI MSK_callbackcodetostr) (	MSKcallbackcodee code,	char * callbackcodestr);

/* MSK_calloctask */
void * (MSKAPI MSK_calloctask) (	MSKtask_t task,	MSKCONST size_t number,	MSKCONST size_t size);

/* MSK_callocdbgtask */
void * (MSKAPI MSK_callocdbgtask) (	MSKtask_t task,	MSKCONST size_t number,	MSKCONST size_t size,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_chgbound */
MSKrescodee (MSKAPI MSK_chgbound) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t i,	MSKint32t lower,	MSKint32t finite,	MSKrealt value);

/* MSK_conetypetostr */
MSKrescodee (MSKAPI MSK_conetypetostr) (	MSKtask_t task,	MSKconetypee conetype,	char * str);

/* MSK_deletetask */
MSKrescodee (MSKAPI MSK_deletetask) (	MSKtask_t * task);

/* MSK_echotask */
MSKrescodee (MSKAPIVA MSK_echotask) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKCONST char * format,	...);

/* MSK_freetask */
void (MSKAPI MSK_freetask) (	MSKtask_t task,	MSKCONST void * buffer);

/* MSK_freedbgtask */
void (MSKAPI MSK_freedbgtask) (	MSKtask_t task,	MSKCONST void * buffer,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_getaij */
MSKrescodee (MSKAPI MSK_getaij) (	MSKtask_t task,	MSKint32t i,	MSKint32t j,	MSKrealt * aij);

/* MSK_getapiecenumnz */
MSKrescodee (MSKAPI MSK_getapiecenumnz) (	MSKtask_t task,	MSKint32t firsti,	MSKint32t lasti,	MSKint32t firstj,	MSKint32t lastj,	MSKint32t * numnz);

/* MSK_getacolnumnz */
MSKrescodee (MSKAPI MSK_getacolnumnz) (	MSKtask_t task,	MSKint32t i,	MSKint32t * nzj);

/* MSK_getacol */
MSKrescodee (MSKAPI MSK_getacol) (	MSKtask_t task,	MSKint32t j,	MSKint32t * nzj,	MSKint32t * subj,	MSKrealt * valj);

/* MSK_getarownumnz */
MSKrescodee (MSKAPI MSK_getarownumnz) (	MSKtask_t task,	MSKint32t i,	MSKint32t * nzi);

/* MSK_getarow */
MSKrescodee (MSKAPI MSK_getarow) (	MSKtask_t task,	MSKint32t i,	MSKint32t * nzi,	MSKint32t * subi,	MSKrealt * vali);

/* MSK_getaslicenumnz */
MSKrescodee (MSKAPI MSK_getaslicenumnz) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t first,	MSKint32t last,	MSKint32t * numnz);

/* MSK_getaslicenumnz64 */
MSKrescodee (MSKAPI MSK_getaslicenumnz64) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t first,	MSKint32t last,	MSKint64t * numnz);

/* MSK_getaslice */
MSKrescodee (MSKAPI MSK_getaslice) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t first,	MSKint32t last,	MSKint32t maxnumnz,	MSKint32t * surp,	MSKint32t * ptrb,	MSKint32t * ptre,	MSKint32t * sub,	MSKrealt * val);

/* MSK_getaslice64 */
MSKrescodee (MSKAPI MSK_getaslice64) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t first,	MSKint32t last,	MSKint64t maxnumnz,	MSKint64t * surp,	MSKint64t * ptrb,	MSKint64t * ptre,	MSKint32t * sub,	MSKrealt * val);

/* MSK_getarowslicetrip */
MSKrescodee (MSKAPI MSK_getarowslicetrip) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKint64t maxnumnz,	MSKint64t * surp,	MSKint32t * subi,	MSKint32t * subj,	MSKrealt * val);

/* MSK_getacolslicetrip */
MSKrescodee (MSKAPI MSK_getacolslicetrip) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKint64t maxnumnz,	MSKint64t * surp,	MSKint32t * subi,	MSKint32t * subj,	MSKrealt * val);

/* MSK_getconbound */
MSKrescodee (MSKAPI MSK_getconbound) (	MSKtask_t task,	MSKint32t i,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_getvarbound */
MSKrescodee (MSKAPI MSK_getvarbound) (	MSKtask_t task,	MSKint32t i,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_getbound */
MSKrescodee (MSKAPI MSK_getbound) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t i,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_getconboundslice */
MSKrescodee (MSKAPI MSK_getconboundslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_getvarboundslice */
MSKrescodee (MSKAPI MSK_getvarboundslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_getboundslice */
MSKrescodee (MSKAPI MSK_getboundslice) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t first,	MSKint32t last,	MSKboundkeye * bk,	MSKrealt * bl,	MSKrealt * bu);

/* MSK_putboundslice */
MSKrescodee (MSKAPI MSK_putboundslice) (	MSKtask_t task,	MSKaccmodee con,	MSKint32t first,	MSKint32t last,	MSKCONST MSKboundkeye * bk,	MSKCONST MSKrealt * bl,	MSKCONST MSKrealt * bu);

/* MSK_getcj */
MSKrescodee (MSKAPI MSK_getcj) (	MSKtask_t task,	MSKint32t j,	MSKrealt * cj);

/* MSK_getc */
MSKrescodee (MSKAPI MSK_getc) (	MSKtask_t task,	MSKrealt * c);

/* MSK_getcallbackfunc */
MSKrescodee (MSKAPI MSK_getcallbackfunc) (	MSKtask_t task,	MSKcallbackfunc * func,	MSKuserhandle_t * handle);

/* MSK_getsolutionincallback */
MSKrescodee (MSKAPI MSK_getsolutionincallback) (	MSKtask_t task,	MSKcallbackcodee where,	MSKsoltypee whichsol,	MSKprostae * prosta,	MSKsolstae * solsta,	MSKstakeye * skc,	MSKstakeye * skx,	MSKstakeye * skn,	MSKrealt * xc,	MSKrealt * xx,	MSKrealt * y,	MSKrealt * slc,	MSKrealt * suc,	MSKrealt * slx,	MSKrealt * sux,	MSKrealt * snx);

/* MSK_getcfix */
MSKrescodee (MSKAPI MSK_getcfix) (	MSKtask_t task,	MSKrealt * cfix);

/* MSK_getcone */
MSKrescodee (MSKAPI MSK_getcone) (	MSKtask_t task,	MSKint32t k,	MSKconetypee * conetype,	MSKrealt * conepar,	MSKint32t * nummem,	MSKint32t * submem);

/* MSK_getconeinfo */
MSKrescodee (MSKAPI MSK_getconeinfo) (	MSKtask_t task,	MSKint32t k,	MSKconetypee * conetype,	MSKrealt * conepar,	MSKint32t * nummem);

/* MSK_getcslice */
MSKrescodee (MSKAPI MSK_getcslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKrealt * c);

/* MSK_getdouinf */
MSKrescodee (MSKAPI MSK_getdouinf) (	MSKtask_t task,	MSKdinfiteme whichdinf,	MSKrealt * dvalue);

/* MSK_getdouparam */
MSKrescodee (MSKAPI MSK_getdouparam) (	MSKtask_t task,	MSKdparame param,	MSKrealt * parvalue);

/* MSK_getdualobj */
MSKrescodee (MSKAPI MSK_getdualobj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * dualobj);

/* MSK_getenv */
MSKrescodee (MSKAPI MSK_getenv) (	MSKtask_t task,	MSKenv_t * env);

/* MSK_getinfindex */
MSKrescodee (MSKAPI MSK_getinfindex) (	MSKtask_t task,	MSKinftypee inftype,	MSKCONST char * infname,	MSKint32t * infindex);

/* MSK_getinfmax */
MSKrescodee (MSKAPI MSK_getinfmax) (	MSKtask_t task,	MSKinftypee inftype,	MSKint32t * infmax);

/* MSK_getinfname */
MSKrescodee (MSKAPI MSK_getinfname) (	MSKtask_t task,	MSKinftypee inftype,	MSKint32t whichinf,	char * infname);

/* MSK_getintinf */
MSKrescodee (MSKAPI MSK_getintinf) (	MSKtask_t task,	MSKiinfiteme whichiinf,	MSKint32t * ivalue);

/* MSK_getlintinf */
MSKrescodee (MSKAPI MSK_getlintinf) (	MSKtask_t task,	MSKliinfiteme whichliinf,	MSKint64t * ivalue);

/* MSK_getintparam */
MSKrescodee (MSKAPI MSK_getintparam) (	MSKtask_t task,	MSKiparame param,	MSKint32t * parvalue);

/* MSK_getmaxnamelen */
MSKrescodee (MSKAPI MSK_getmaxnamelen) (	MSKtask_t task,	MSKint32t * maxlen);

/* MSK_getmaxnumanz */
MSKrescodee (MSKAPI MSK_getmaxnumanz) (	MSKtask_t task,	MSKint32t * maxnumanz);

/* MSK_getmaxnumanz64 */
MSKrescodee (MSKAPI MSK_getmaxnumanz64) (	MSKtask_t task,	MSKint64t * maxnumanz);

/* MSK_getmaxnumcon */
MSKrescodee (MSKAPI MSK_getmaxnumcon) (	MSKtask_t task,	MSKint32t * maxnumcon);

/* MSK_getmaxnumvar */
MSKrescodee (MSKAPI MSK_getmaxnumvar) (	MSKtask_t task,	MSKint32t * maxnumvar);

/* MSK_getnadouinf */
MSKrescodee (MSKAPI MSK_getnadouinf) (	MSKtask_t task,	MSKCONST char * whichdinf,	MSKrealt * dvalue);

/* MSK_getnadouparam */
MSKrescodee (MSKAPI MSK_getnadouparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKrealt * parvalue);

/* MSK_getnaintinf */
MSKrescodee (MSKAPI MSK_getnaintinf) (	MSKtask_t task,	MSKCONST char * infitemname,	MSKint32t * ivalue);

/* MSK_getnaintparam */
MSKrescodee (MSKAPI MSK_getnaintparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKint32t * parvalue);

/* MSK_getbarvarnamelen */
MSKrescodee (MSKAPI MSK_getbarvarnamelen) (	MSKtask_t task,	MSKint32t i,	MSKint32t * len);

/* MSK_getbarvarname */
MSKrescodee (MSKAPI MSK_getbarvarname) (	MSKtask_t task,	MSKint32t i,	MSKint32t maxlen,	char * name);

/* MSK_getbarvarnameindex */
MSKrescodee (MSKAPI MSK_getbarvarnameindex) (	MSKtask_t task,	MSKCONST char * somename,	MSKint32t * asgn,	MSKint32t * index);

/* MSK_putconname */
MSKrescodee (MSKAPI MSK_putconname) (	MSKtask_t task,	MSKint32t i,	MSKCONST char * name);

/* MSK_putvarname */
MSKrescodee (MSKAPI MSK_putvarname) (	MSKtask_t task,	MSKint32t j,	MSKCONST char * name);

/* MSK_putconename */
MSKrescodee (MSKAPI MSK_putconename) (	MSKtask_t task,	MSKint32t j,	MSKCONST char * name);

/* MSK_putbarvarname */
MSKrescodee (MSKAPI MSK_putbarvarname) (	MSKtask_t task,	MSKint32t j,	MSKCONST char * name);

/* MSK_getvarnamelen */
MSKrescodee (MSKAPI MSK_getvarnamelen) (	MSKtask_t task,	MSKint32t i,	MSKint32t * len);

/* MSK_getvarname */
MSKrescodee (MSKAPI MSK_getvarname) (	MSKtask_t task,	MSKint32t j,	MSKint32t maxlen,	char * name);

/* MSK_getconnamelen */
MSKrescodee (MSKAPI MSK_getconnamelen) (	MSKtask_t task,	MSKint32t i,	MSKint32t * len);

/* MSK_getconname */
MSKrescodee (MSKAPI MSK_getconname) (	MSKtask_t task,	MSKint32t i,	MSKint32t maxlen,	char * name);

/* MSK_getconnameindex */
MSKrescodee (MSKAPI MSK_getconnameindex) (	MSKtask_t task,	MSKCONST char * somename,	MSKint32t * asgn,	MSKint32t * index);

/* MSK_getvarnameindex */
MSKrescodee (MSKAPI MSK_getvarnameindex) (	MSKtask_t task,	MSKCONST char * somename,	MSKint32t * asgn,	MSKint32t * index);

/* MSK_getconenamelen */
MSKrescodee (MSKAPI MSK_getconenamelen) (	MSKtask_t task,	MSKint32t i,	MSKint32t * len);

/* MSK_getconename */
MSKrescodee (MSKAPI MSK_getconename) (	MSKtask_t task,	MSKint32t i,	MSKint32t maxlen,	char * name);

/* MSK_getconenameindex */
MSKrescodee (MSKAPI MSK_getconenameindex) (	MSKtask_t task,	MSKCONST char * somename,	MSKint32t * asgn,	MSKint32t * index);

/* MSK_getnastrparam */
MSKrescodee (MSKAPI MSK_getnastrparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKint32t maxlen,	MSKint32t * len,	char * parvalue);

/* MSK_getnumanz */
MSKrescodee (MSKAPI MSK_getnumanz) (	MSKtask_t task,	MSKint32t * numanz);

/* MSK_getnumanz64 */
MSKrescodee (MSKAPI MSK_getnumanz64) (	MSKtask_t task,	MSKint64t * numanz);

/* MSK_getnumcon */
MSKrescodee (MSKAPI MSK_getnumcon) (	MSKtask_t task,	MSKint32t * numcon);

/* MSK_getnumcone */
MSKrescodee (MSKAPI MSK_getnumcone) (	MSKtask_t task,	MSKint32t * numcone);

/* MSK_getnumconemem */
MSKrescodee (MSKAPI MSK_getnumconemem) (	MSKtask_t task,	MSKint32t k,	MSKint32t * nummem);

/* MSK_getnumintvar */
MSKrescodee (MSKAPI MSK_getnumintvar) (	MSKtask_t task,	MSKint32t * numintvar);

/* MSK_getnumparam */
MSKrescodee (MSKAPI MSK_getnumparam) (	MSKtask_t task,	MSKparametertypee partype,	MSKint32t * numparam);

/* MSK_getnumqconknz */
MSKrescodee (MSKAPI MSK_getnumqconknz) (	MSKtask_t task,	MSKint32t k,	MSKint32t * numqcnz);

/* MSK_getnumqconknz64 */
MSKrescodee (MSKAPI MSK_getnumqconknz64) (	MSKtask_t task,	MSKint32t k,	MSKint64t * numqcnz);

/* MSK_getnumqobjnz */
MSKrescodee (MSKAPI MSK_getnumqobjnz) (	MSKtask_t task,	MSKint32t * numqonz);

/* MSK_getnumqobjnz64 */
MSKrescodee (MSKAPI MSK_getnumqobjnz64) (	MSKtask_t task,	MSKint64t * numqonz);

/* MSK_getnumvar */
MSKrescodee (MSKAPI MSK_getnumvar) (	MSKtask_t task,	MSKint32t * numvar);

/* MSK_getnumbarvar */
MSKrescodee (MSKAPI MSK_getnumbarvar) (	MSKtask_t task,	MSKint32t * numbarvar);

/* MSK_getmaxnumbarvar */
MSKrescodee (MSKAPI MSK_getmaxnumbarvar) (	MSKtask_t task,	MSKint32t * maxnumbarvar);

/* MSK_getdimbarvarj */
MSKrescodee (MSKAPI MSK_getdimbarvarj) (	MSKtask_t task,	MSKint32t j,	MSKint32t * dimbarvarj);

/* MSK_getlenbarvarj */
MSKrescodee (MSKAPI MSK_getlenbarvarj) (	MSKtask_t task,	MSKint32t j,	MSKint64t * lenbarvarj);

/* MSK_getobjname */
MSKrescodee (MSKAPI MSK_getobjname) (	MSKtask_t task,	MSKint32t maxlen,	char * objname);

/* MSK_getobjnamelen */
MSKrescodee (MSKAPI MSK_getobjnamelen) (	MSKtask_t task,	MSKint32t * len);

/* MSK_getparamname */
MSKrescodee (MSKAPI MSK_getparamname) (	MSKtask_t task,	MSKparametertypee partype,	MSKint32t param,	char * parname);

/* MSK_getparammax */
MSKrescodee (MSKAPI MSK_getparammax) (	MSKtask_t task,	MSKparametertypee partype,	MSKint32t * parammax);

/* MSK_getprimalobj */
MSKrescodee (MSKAPI MSK_getprimalobj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * primalobj);

/* MSK_getprobtype */
MSKrescodee (MSKAPI MSK_getprobtype) (	MSKtask_t task,	MSKproblemtypee * probtype);

/* MSK_getqconk64 */
MSKrescodee (MSKAPI MSK_getqconk64) (	MSKtask_t task,	MSKint32t k,	MSKint64t maxnumqcnz,	MSKint64t * qcsurp,	MSKint64t * numqcnz,	MSKint32t * qcsubi,	MSKint32t * qcsubj,	MSKrealt * qcval);

/* MSK_getqconk */
MSKrescodee (MSKAPI MSK_getqconk) (	MSKtask_t task,	MSKint32t k,	MSKint32t maxnumqcnz,	MSKint32t * qcsurp,	MSKint32t * numqcnz,	MSKint32t * qcsubi,	MSKint32t * qcsubj,	MSKrealt * qcval);

/* MSK_getqobj */
MSKrescodee (MSKAPI MSK_getqobj) (	MSKtask_t task,	MSKint32t maxnumqonz,	MSKint32t * qosurp,	MSKint32t * numqonz,	MSKint32t * qosubi,	MSKint32t * qosubj,	MSKrealt * qoval);

/* MSK_getqobj64 */
MSKrescodee (MSKAPI MSK_getqobj64) (	MSKtask_t task,	MSKint64t maxnumqonz,	MSKint64t * qosurp,	MSKint64t * numqonz,	MSKint32t * qosubi,	MSKint32t * qosubj,	MSKrealt * qoval);

/* MSK_getqobjij */
MSKrescodee (MSKAPI MSK_getqobjij) (	MSKtask_t task,	MSKint32t i,	MSKint32t j,	MSKrealt * qoij);

/* MSK_getsolution */
MSKrescodee (MSKAPI MSK_getsolution) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKprostae * prosta,	MSKsolstae * solsta,	MSKstakeye * skc,	MSKstakeye * skx,	MSKstakeye * skn,	MSKrealt * xc,	MSKrealt * xx,	MSKrealt * y,	MSKrealt * slc,	MSKrealt * suc,	MSKrealt * slx,	MSKrealt * sux,	MSKrealt * snx);

/* MSK_getpbi */
MSKrescodee (MSKAPI MSK_getpbi) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKaccmodee accmode,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * pbi,	MSKint32t normalize);

/* MSK_getdbi */
MSKrescodee (MSKAPI MSK_getdbi) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKaccmodee accmode,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * dbi);

/* MSK_getdeqi */
MSKrescodee (MSKAPI MSK_getdeqi) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKaccmodee accmode,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * deqi,	MSKint32t normalize);

/* MSK_getpeqi */
MSKrescodee (MSKAPI MSK_getpeqi) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * peqi,	MSKint32t normalize);

/* MSK_getinti */
MSKrescodee (MSKAPI MSK_getinti) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * inti);

/* MSK_getpcni */
MSKrescodee (MSKAPI MSK_getpcni) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * pcni);

/* MSK_getdcni */
MSKrescodee (MSKAPI MSK_getdcni) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKint32t * sub,	MSKint32t len,	MSKrealt * dcni);

/* MSK_getsolutioni */
MSKrescodee (MSKAPI MSK_getsolutioni) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t i,	MSKsoltypee whichsol,	MSKstakeye * sk,	MSKrealt * x,	MSKrealt * sl,	MSKrealt * su,	MSKrealt * sn);

/* MSK_getsolsta */
MSKrescodee (MSKAPI MSK_getsolsta) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKsolstae * solsta);

/* MSK_getprosta */
MSKrescodee (MSKAPI MSK_getprosta) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKprostae * prosta);

/* MSK_getskc */
MSKrescodee (MSKAPI MSK_getskc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKstakeye * skc);

/* MSK_getskx */
MSKrescodee (MSKAPI MSK_getskx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKstakeye * skx);

/* MSK_getxc */
MSKrescodee (MSKAPI MSK_getxc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * xc);

/* MSK_getxx */
MSKrescodee (MSKAPI MSK_getxx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * xx);

/* MSK_gety */
MSKrescodee (MSKAPI MSK_gety) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * y);

/* MSK_getslc */
MSKrescodee (MSKAPI MSK_getslc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * slc);

/* MSK_getsuc */
MSKrescodee (MSKAPI MSK_getsuc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * suc);

/* MSK_getslx */
MSKrescodee (MSKAPI MSK_getslx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * slx);

/* MSK_getsux */
MSKrescodee (MSKAPI MSK_getsux) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * sux);

/* MSK_getsnx */
MSKrescodee (MSKAPI MSK_getsnx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * snx);

/* MSK_getskcslice */
MSKrescodee (MSKAPI MSK_getskcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKstakeye * skc);

/* MSK_getskxslice */
MSKrescodee (MSKAPI MSK_getskxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKstakeye * skx);

/* MSK_getxcslice */
MSKrescodee (MSKAPI MSK_getxcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * xc);

/* MSK_getxxslice */
MSKrescodee (MSKAPI MSK_getxxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * xx);

/* MSK_getyslice */
MSKrescodee (MSKAPI MSK_getyslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * y);

/* MSK_getslcslice */
MSKrescodee (MSKAPI MSK_getslcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * slc);

/* MSK_getsucslice */
MSKrescodee (MSKAPI MSK_getsucslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * suc);

/* MSK_getslxslice */
MSKrescodee (MSKAPI MSK_getslxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * slx);

/* MSK_getsuxslice */
MSKrescodee (MSKAPI MSK_getsuxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * sux);

/* MSK_getsnxslice */
MSKrescodee (MSKAPI MSK_getsnxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * snx);

/* MSK_getbarxj */
MSKrescodee (MSKAPI MSK_getbarxj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t j,	MSKrealt * barxj);

/* MSK_getbarsj */
MSKrescodee (MSKAPI MSK_getbarsj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t j,	MSKrealt * barsj);

/* MSK_putskc */
MSKrescodee (MSKAPI MSK_putskc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKstakeye * skc);

/* MSK_putskx */
MSKrescodee (MSKAPI MSK_putskx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKstakeye * skx);

/* MSK_putxc */
MSKrescodee (MSKAPI MSK_putxc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * xc);

/* MSK_putxx */
MSKrescodee (MSKAPI MSK_putxx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * xx);

/* MSK_puty */
MSKrescodee (MSKAPI MSK_puty) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * y);

/* MSK_putslc */
MSKrescodee (MSKAPI MSK_putslc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * slc);

/* MSK_putsuc */
MSKrescodee (MSKAPI MSK_putsuc) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * suc);

/* MSK_putslx */
MSKrescodee (MSKAPI MSK_putslx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * slx);

/* MSK_putsux */
MSKrescodee (MSKAPI MSK_putsux) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * sux);

/* MSK_putsnx */
MSKrescodee (MSKAPI MSK_putsnx) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKrealt * sux);

/* MSK_putskcslice */
MSKrescodee (MSKAPI MSK_putskcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKstakeye * skc);

/* MSK_putskxslice */
MSKrescodee (MSKAPI MSK_putskxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKstakeye * skx);

/* MSK_putxcslice */
MSKrescodee (MSKAPI MSK_putxcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * xc);

/* MSK_putxxslice */
MSKrescodee (MSKAPI MSK_putxxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * xx);

/* MSK_putyslice */
MSKrescodee (MSKAPI MSK_putyslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * y);

/* MSK_putslcslice */
MSKrescodee (MSKAPI MSK_putslcslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * slc);

/* MSK_putsucslice */
MSKrescodee (MSKAPI MSK_putsucslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * suc);

/* MSK_putslxslice */
MSKrescodee (MSKAPI MSK_putslxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * slx);

/* MSK_putsuxslice */
MSKrescodee (MSKAPI MSK_putsuxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * sux);

/* MSK_putsnxslice */
MSKrescodee (MSKAPI MSK_putsnxslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * snx);

/* MSK_putbarxj */
MSKrescodee (MSKAPI MSK_putbarxj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t j,	MSKCONST MSKrealt * barxj);

/* MSK_putbarsj */
MSKrescodee (MSKAPI MSK_putbarsj) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t j,	MSKCONST MSKrealt * barsj);

/* MSK_getsolutioninf */
MSKrescodee (MSKAPI MSK_getsolutioninf) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKprostae * prosta,	MSKsolstae * solsta,	MSKrealt * primalobj,	MSKrealt * maxpbi,	MSKrealt * maxpcni,	MSKrealt * maxpeqi,	MSKrealt * maxinti,	MSKrealt * dualobj,	MSKrealt * maxdbi,	MSKrealt * maxdcni,	MSKrealt * maxdeqi);

/* MSK_getpviolcon */
MSKrescodee (MSKAPI MSK_getpviolcon) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getpviolvar */
MSKrescodee (MSKAPI MSK_getpviolvar) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getpviolbarvar */
MSKrescodee (MSKAPI MSK_getpviolbarvar) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getpviolcones */
MSKrescodee (MSKAPI MSK_getpviolcones) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getdviolcon */
MSKrescodee (MSKAPI MSK_getdviolcon) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getdviolvar */
MSKrescodee (MSKAPI MSK_getdviolvar) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getdviolbarvar */
MSKrescodee (MSKAPI MSK_getdviolbarvar) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getdviolcones */
MSKrescodee (MSKAPI MSK_getdviolcones) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKrealt * viol);

/* MSK_getsolutioninfo */
MSKrescodee (MSKAPI MSK_getsolutioninfo) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKrealt * pobj,	MSKrealt * pviolcon,	MSKrealt * pviolvar,	MSKrealt * pviolbarvar,	MSKrealt * pviolcone,	MSKrealt * pviolitg,	MSKrealt * dobj,	MSKrealt * dviolcon,	MSKrealt * dviolvar,	MSKrealt * dviolbarvar,	MSKrealt * dviolcone);

/* MSK_getsolutionslice */
MSKrescodee (MSKAPI MSK_getsolutionslice) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKsoliteme solitem,	MSKint32t first,	MSKint32t last,	MSKrealt * values);

/* MSK_getreducedcosts */
MSKrescodee (MSKAPI MSK_getreducedcosts) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKint32t first,	MSKint32t last,	MSKrealt * redcosts);

/* MSK_getstrparam */
MSKrescodee (MSKAPI MSK_getstrparam) (	MSKtask_t task,	MSKsparame param,	MSKint32t maxlen,	MSKint32t * len,	char * parvalue);

/* MSK_getstrparamlen */
MSKrescodee (MSKAPI MSK_getstrparamlen) (	MSKtask_t task,	MSKsparame param,	MSKint32t * len);

/* MSK_getstrparamal */
MSKrescodee (MSKAPI MSK_getstrparamal) (	MSKtask_t task,	MSKsparame param,	MSKint32t numaddchr,	MSKstring_t * value);

/* MSK_getnastrparamal */
MSKrescodee (MSKAPI MSK_getnastrparamal) (	MSKtask_t task,	MSKCONST char * paramname,	MSKint32t numaddchr,	MSKstring_t * value);

/* MSK_getsymbcon */
MSKrescodee (MSKAPI MSK_getsymbcon) (	MSKtask_t task,	MSKint32t i,	MSKint32t maxlen,	char * name,	MSKint32t * value);

/* MSK_gettasknamelen */
MSKrescodee (MSKAPI MSK_gettasknamelen) (	MSKtask_t task,	MSKint32t * len);

/* MSK_gettaskname */
MSKrescodee (MSKAPI MSK_gettaskname) (	MSKtask_t task,	MSKint32t maxlen,	char * taskname);

/* MSK_getvartype */
MSKrescodee (MSKAPI MSK_getvartype) (	MSKtask_t task,	MSKint32t j,	MSKvariabletypee * vartype);

/* MSK_getvartypelist */
MSKrescodee (MSKAPI MSK_getvartypelist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subj,	MSKvariabletypee * vartype);

/* MSK_inputdata */
MSKrescodee (MSKAPI MSK_inputdata) (	MSKtask_t task,	MSKint32t maxnumcon,	MSKint32t maxnumvar,	MSKint32t numcon,	MSKint32t numvar,	MSKCONST MSKrealt * c,	MSKrealt cfix,	MSKCONST MSKint32t * aptrb,	MSKCONST MSKint32t * aptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval,	MSKCONST MSKboundkeye * bkc,	MSKCONST MSKrealt * blc,	MSKCONST MSKrealt * buc,	MSKCONST MSKboundkeye * bkx,	MSKCONST MSKrealt * blx,	MSKCONST MSKrealt * bux);

/* MSK_inputdata64 */
MSKrescodee (MSKAPI MSK_inputdata64) (	MSKtask_t task,	MSKint32t maxnumcon,	MSKint32t maxnumvar,	MSKint32t numcon,	MSKint32t numvar,	MSKCONST MSKrealt * c,	MSKrealt cfix,	MSKCONST MSKint64t * aptrb,	MSKCONST MSKint64t * aptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval,	MSKCONST MSKboundkeye * bkc,	MSKCONST MSKrealt * blc,	MSKCONST MSKrealt * buc,	MSKCONST MSKboundkeye * bkx,	MSKCONST MSKrealt * blx,	MSKCONST MSKrealt * bux);

/* MSK_isdouparname */
MSKrescodee (MSKAPI MSK_isdouparname) (	MSKtask_t task,	MSKCONST char * parname,	MSKdparame * param);

/* MSK_isintparname */
MSKrescodee (MSKAPI MSK_isintparname) (	MSKtask_t task,	MSKCONST char * parname,	MSKiparame * param);

/* MSK_isstrparname */
MSKrescodee (MSKAPI MSK_isstrparname) (	MSKtask_t task,	MSKCONST char * parname,	MSKsparame * param);

/* MSK_linkfiletotaskstream */
MSKrescodee (MSKAPI MSK_linkfiletotaskstream) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKCONST char * filename,	MSKint32t append);

/* MSK_linkfunctotaskstream */
MSKrescodee (MSKAPI MSK_linkfunctotaskstream) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKuserhandle_t handle,	MSKstreamfunc func);

/* MSK_unlinkfuncfromtaskstream */
MSKrescodee (MSKAPI MSK_unlinkfuncfromtaskstream) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_clonetask */
MSKrescodee (MSKAPI MSK_clonetask) (	MSKtask_t task,	MSKtask_t * clonedtask);

/* MSK_relaxprimal */
MSKrescodee (MSKAPI MSK_relaxprimal) (	MSKtask_t task,	MSKtask_t * relaxedtask,	MSKrealt * wlc,	MSKrealt * wuc,	MSKrealt * wlx,	MSKrealt * wux);

/* MSK_primalrepair */
MSKrescodee (MSKAPI MSK_primalrepair) (	MSKtask_t task,	MSKCONST MSKrealt * wlc,	MSKCONST MSKrealt * wuc,	MSKCONST MSKrealt * wlx,	MSKCONST MSKrealt * wux);

/* MSK_toconic */
MSKrescodee (MSKAPI MSK_toconic) (	MSKtask_t task);

/* MSK_reformqcqotosocp */
MSKrescodee (MSKAPI MSK_reformqcqotosocp) (	MSKtask_t task,	MSKtask_t relaxedtask);

/* MSK_optimizeconcurrent */
MSKrescodee (MSKAPI MSK_optimizeconcurrent) (	MSKtask_t task,	MSKCONST MSKtask_t * taskarray,	MSKint32t num);

/* MSK_optimize */
MSKrescodee (MSKAPI MSK_optimize) (	MSKtask_t task);

/* MSK_optimizetrm */
MSKrescodee (MSKAPI MSK_optimizetrm) (	MSKtask_t task,	MSKrescodee * trmcode);

/* MSK_printdata */
MSKrescodee (MSKAPI MSK_printdata) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKint32t firsti,	MSKint32t lasti,	MSKint32t firstj,	MSKint32t lastj,	MSKint32t firstk,	MSKint32t lastk,	MSKint32t c,	MSKint32t qo,	MSKint32t a,	MSKint32t qc,	MSKint32t bc,	MSKint32t bx,	MSKint32t vartype,	MSKint32t cones);

/* MSK_printparam */
MSKrescodee (MSKAPI MSK_printparam) (	MSKtask_t task);

/* MSK_probtypetostr */
MSKrescodee (MSKAPI MSK_probtypetostr) (	MSKtask_t task,	MSKproblemtypee probtype,	char * str);

/* MSK_prostatostr */
MSKrescodee (MSKAPI MSK_prostatostr) (	MSKtask_t task,	MSKprostae prosta,	char * str);

/* MSK_putresponsefunc */
MSKrescodee (MSKAPI MSK_putresponsefunc) (	MSKtask_t task,	MSKresponsefunc responsefunc,	MSKuserhandle_t handle);

/* MSK_commitchanges */
MSKrescodee (MSKAPI MSK_commitchanges) (	MSKtask_t task);

/* MSK_putaij */
MSKrescodee (MSKAPI MSK_putaij) (	MSKtask_t task,	MSKint32t i,	MSKint32t j,	MSKrealt aij);

/* MSK_putaijlist */
MSKrescodee (MSKAPI MSK_putaijlist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subi,	MSKCONST MSKint32t * subj,	MSKCONST MSKrealt * valij);

/* MSK_putaijlist64 */
MSKrescodee (MSKAPI MSK_putaijlist64) (	MSKtask_t task,	MSKint64t num,	MSKCONST MSKint32t * subi,	MSKCONST MSKint32t * subj,	MSKCONST MSKrealt * valij);

/* MSK_putacol */
MSKrescodee (MSKAPI MSK_putacol) (	MSKtask_t task,	MSKint32t j,	MSKint32t nzj,	MSKCONST MSKint32t * subj,	MSKCONST MSKrealt * valj);

/* MSK_putarow */
MSKrescodee (MSKAPI MSK_putarow) (	MSKtask_t task,	MSKint32t i,	MSKint32t nzi,	MSKCONST MSKint32t * subi,	MSKCONST MSKrealt * vali);

/* MSK_putarowslice */
MSKrescodee (MSKAPI MSK_putarowslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKint32t * ptrb,	MSKCONST MSKint32t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putarowslice64 */
MSKrescodee (MSKAPI MSK_putarowslice64) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKint64t * ptrb,	MSKCONST MSKint64t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putarowlist */
MSKrescodee (MSKAPI MSK_putarowlist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKint32t * aptrb,	MSKCONST MSKint32t * aptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putarowlist64 */
MSKrescodee (MSKAPI MSK_putarowlist64) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKint64t * ptrb,	MSKCONST MSKint64t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putacolslice */
MSKrescodee (MSKAPI MSK_putacolslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKint32t * ptrb,	MSKCONST MSKint32t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putacolslice64 */
MSKrescodee (MSKAPI MSK_putacolslice64) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKint64t * ptrb,	MSKCONST MSKint64t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putacollist */
MSKrescodee (MSKAPI MSK_putacollist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKint32t * ptrb,	MSKCONST MSKint32t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putacollist64 */
MSKrescodee (MSKAPI MSK_putacollist64) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKint64t * ptrb,	MSKCONST MSKint64t * ptre,	MSKCONST MSKint32t * asub,	MSKCONST MSKrealt * aval);

/* MSK_putbaraij */
MSKrescodee (MSKAPI MSK_putbaraij) (	MSKtask_t task,	MSKint32t i,	MSKint32t j,	MSKint64t num,	MSKCONST MSKint64t * sub,	MSKCONST MSKrealt * weights);

/* MSK_getnumbarcnz */
MSKrescodee (MSKAPI MSK_getnumbarcnz) (	MSKtask_t task,	MSKint64t * nz);

/* MSK_getnumbaranz */
MSKrescodee (MSKAPI MSK_getnumbaranz) (	MSKtask_t task,	MSKint64t * nz);

/* MSK_getbarcsparsity */
MSKrescodee (MSKAPI MSK_getbarcsparsity) (	MSKtask_t task,	MSKint64t maxnumnz,	MSKint64t * numnz,	MSKint64t * idxj);

/* MSK_getbarasparsity */
MSKrescodee (MSKAPI MSK_getbarasparsity) (	MSKtask_t task,	MSKint64t maxnumnz,	MSKint64t * numnz,	MSKint64t * idxij);

/* MSK_getbarcidxinfo */
MSKrescodee (MSKAPI MSK_getbarcidxinfo) (	MSKtask_t task,	MSKint64t idx,	MSKint64t * num);

/* MSK_getbarcidxj */
MSKrescodee (MSKAPI MSK_getbarcidxj) (	MSKtask_t task,	MSKint64t idx,	MSKint32t * j);

/* MSK_getbarcidx */
MSKrescodee (MSKAPI MSK_getbarcidx) (	MSKtask_t task,	MSKint64t idx,	MSKint64t maxnum,	MSKint32t * j,	MSKint64t * num,	MSKint64t * sub,	MSKrealt * weights);

/* MSK_getbaraidxinfo */
MSKrescodee (MSKAPI MSK_getbaraidxinfo) (	MSKtask_t task,	MSKint64t idx,	MSKint64t * num);

/* MSK_getbaraidxij */
MSKrescodee (MSKAPI MSK_getbaraidxij) (	MSKtask_t task,	MSKint64t idx,	MSKint32t * i,	MSKint32t * j);

/* MSK_getbaraidx */
MSKrescodee (MSKAPI MSK_getbaraidx) (	MSKtask_t task,	MSKint64t idx,	MSKint64t maxnum,	MSKint32t * i,	MSKint32t * j,	MSKint64t * num,	MSKint64t * sub,	MSKrealt * weights);

/* MSK_putbarcblocktriplet */
MSKrescodee (MSKAPI MSK_putbarcblocktriplet) (	MSKtask_t task,	MSKint64t num,	MSKCONST MSKint32t * subj,	MSKCONST MSKint32t * subk,	MSKCONST MSKint32t * subl,	MSKCONST MSKrealt * valjkl);

/* MSK_putbarablocktriplet */
MSKrescodee (MSKAPI MSK_putbarablocktriplet) (	MSKtask_t task,	MSKint64t num,	MSKCONST MSKint32t * subi,	MSKCONST MSKint32t * subj,	MSKCONST MSKint32t * subk,	MSKCONST MSKint32t * subl,	MSKCONST MSKrealt * valijkl);

/* MSK_getnumbarcblocktriplets */
MSKrescodee (MSKAPI MSK_getnumbarcblocktriplets) (	MSKtask_t task,	MSKint64t * num);

/* MSK_getbarcblocktriplet */
MSKrescodee (MSKAPI MSK_getbarcblocktriplet) (	MSKtask_t task,	MSKint64t maxnum,	MSKint64t * num,	MSKint32t * subj,	MSKint32t * subk,	MSKint32t * subl,	MSKrealt * valijkl);

/* MSK_getnumbarablocktriplets */
MSKrescodee (MSKAPI MSK_getnumbarablocktriplets) (	MSKtask_t task,	MSKint64t * num);

/* MSK_getbarablocktriplet */
MSKrescodee (MSKAPI MSK_getbarablocktriplet) (	MSKtask_t task,	MSKint64t maxnum,	MSKint64t * num,	MSKint32t * subi,	MSKint32t * subj,	MSKint32t * subk,	MSKint32t * subl,	MSKrealt * valijkl);

/* MSK_putbound */
MSKrescodee (MSKAPI MSK_putbound) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t i,	MSKboundkeye bk,	MSKrealt bl,	MSKrealt bu);

/* MSK_putboundlist */
MSKrescodee (MSKAPI MSK_putboundlist) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKboundkeye * bk,	MSKCONST MSKrealt * bl,	MSKCONST MSKrealt * bu);

/* MSK_putconbound */
MSKrescodee (MSKAPI MSK_putconbound) (	MSKtask_t task,	MSKint32t i,	MSKboundkeye bk,	MSKrealt bl,	MSKrealt bu);

/* MSK_putconboundlist */
MSKrescodee (MSKAPI MSK_putconboundlist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKboundkeye * bkc,	MSKCONST MSKrealt * blc,	MSKCONST MSKrealt * buc);

/* MSK_putconboundslice */
MSKrescodee (MSKAPI MSK_putconboundslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKboundkeye * bk,	MSKCONST MSKrealt * bl,	MSKCONST MSKrealt * bu);

/* MSK_putvarbound */
MSKrescodee (MSKAPI MSK_putvarbound) (	MSKtask_t task,	MSKint32t j,	MSKboundkeye bk,	MSKrealt bl,	MSKrealt bu);

/* MSK_putvarboundlist */
MSKrescodee (MSKAPI MSK_putvarboundlist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * sub,	MSKCONST MSKboundkeye * bkx,	MSKCONST MSKrealt * blx,	MSKCONST MSKrealt * bux);

/* MSK_putvarboundslice */
MSKrescodee (MSKAPI MSK_putvarboundslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKboundkeye * bk,	MSKCONST MSKrealt * bl,	MSKCONST MSKrealt * bu);

/* MSK_putcallbackfunc */
MSKrescodee (MSKAPI MSK_putcallbackfunc) (	MSKtask_t task,	MSKcallbackfunc func,	MSKuserhandle_t handle);

/* MSK_putcfix */
MSKrescodee (MSKAPI MSK_putcfix) (	MSKtask_t task,	MSKrealt cfix);

/* MSK_putcj */
MSKrescodee (MSKAPI MSK_putcj) (	MSKtask_t task,	MSKint32t j,	MSKrealt cj);

/* MSK_putobjsense */
MSKrescodee (MSKAPI MSK_putobjsense) (	MSKtask_t task,	MSKobjsensee sense);

/* MSK_getobjsense */
MSKrescodee (MSKAPI MSK_getobjsense) (	MSKtask_t task,	MSKobjsensee * sense);

/* MSK_putclist */
MSKrescodee (MSKAPI MSK_putclist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subj,	MSKCONST MSKrealt * val);

/* MSK_putcslice */
MSKrescodee (MSKAPI MSK_putcslice) (	MSKtask_t task,	MSKint32t first,	MSKint32t last,	MSKCONST MSKrealt * slice);

/* MSK_putbarcj */
MSKrescodee (MSKAPI MSK_putbarcj) (	MSKtask_t task,	MSKint32t j,	MSKint64t num,	MSKCONST MSKint64t * sub,	MSKCONST MSKrealt * weights);

/* MSK_putcone */
MSKrescodee (MSKAPI MSK_putcone) (	MSKtask_t task,	MSKint32t k,	MSKconetypee conetype,	MSKrealt conepar,	MSKint32t nummem,	MSKCONST MSKint32t * submem);

/* MSK_appendsparsesymmat */
MSKrescodee (MSKAPI MSK_appendsparsesymmat) (	MSKtask_t task,	MSKint32t dim,	MSKint64t nz,	MSKCONST MSKint32t * subi,	MSKCONST MSKint32t * subj,	MSKCONST MSKrealt * valij,	MSKint64t * idx);

/* MSK_getsymmatinfo */
MSKrescodee (MSKAPI MSK_getsymmatinfo) (	MSKtask_t task,	MSKint64t idx,	MSKint32t * dim,	MSKint64t * nz,	MSKsymmattypee * type);

/* MSK_getnumsymmat */
MSKrescodee (MSKAPI MSK_getnumsymmat) (	MSKtask_t task,	MSKint64t * num);

/* MSK_getsparsesymmat */
MSKrescodee (MSKAPI MSK_getsparsesymmat) (	MSKtask_t task,	MSKint64t idx,	MSKint64t maxlen,	MSKint32t * subi,	MSKint32t * subj,	MSKrealt * valij);

/* MSK_putdouparam */
MSKrescodee (MSKAPI MSK_putdouparam) (	MSKtask_t task,	MSKdparame param,	MSKrealt parvalue);

/* MSK_putintparam */
MSKrescodee (MSKAPI MSK_putintparam) (	MSKtask_t task,	MSKiparame param,	MSKint32t parvalue);

/* MSK_putmaxnumcon */
MSKrescodee (MSKAPI MSK_putmaxnumcon) (	MSKtask_t task,	MSKint32t maxnumcon);

/* MSK_putmaxnumcone */
MSKrescodee (MSKAPI MSK_putmaxnumcone) (	MSKtask_t task,	MSKint32t maxnumcone);

/* MSK_getmaxnumcone */
MSKrescodee (MSKAPI MSK_getmaxnumcone) (	MSKtask_t task,	MSKint32t * maxnumcone);

/* MSK_putmaxnumvar */
MSKrescodee (MSKAPI MSK_putmaxnumvar) (	MSKtask_t task,	MSKint32t maxnumvar);

/* MSK_putmaxnumbarvar */
MSKrescodee (MSKAPI MSK_putmaxnumbarvar) (	MSKtask_t task,	MSKint32t maxnumbarvar);

/* MSK_putmaxnumanz */
MSKrescodee (MSKAPI MSK_putmaxnumanz) (	MSKtask_t task,	MSKint64t maxnumanz);

/* MSK_putmaxnumqnz */
MSKrescodee (MSKAPI MSK_putmaxnumqnz) (	MSKtask_t task,	MSKint64t maxnumqnz);

/* MSK_getmaxnumqnz */
MSKrescodee (MSKAPI MSK_getmaxnumqnz) (	MSKtask_t task,	MSKint32t * maxnumqnz);

/* MSK_getmaxnumqnz64 */
MSKrescodee (MSKAPI MSK_getmaxnumqnz64) (	MSKtask_t task,	MSKint64t * maxnumqnz);

/* MSK_putnadouparam */
MSKrescodee (MSKAPI MSK_putnadouparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKrealt parvalue);

/* MSK_putnaintparam */
MSKrescodee (MSKAPI MSK_putnaintparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKint32t parvalue);

/* MSK_putnastrparam */
MSKrescodee (MSKAPI MSK_putnastrparam) (	MSKtask_t task,	MSKCONST char * paramname,	MSKCONST char * parvalue);

/* MSK_putnlfunc */
MSKrescodee (MSKAPI MSK_putnlfunc) (	MSKtask_t task,	MSKuserhandle_t nlhandle,	MSKnlgetspfunc nlgetsp,	MSKnlgetvafunc nlgetva);

/* MSK_getnlfunc */
MSKrescodee (MSKAPI MSK_getnlfunc) (	MSKtask_t task,	MSKuserhandle_t * nlhandle,	MSKnlgetspfunc * nlgetsp,	MSKnlgetvafunc * nlgetva);

/* MSK_putobjname */
MSKrescodee (MSKAPI MSK_putobjname) (	MSKtask_t task,	MSKCONST char * objname);

/* MSK_putparam */
MSKrescodee (MSKAPI MSK_putparam) (	MSKtask_t task,	MSKCONST char * parname,	MSKCONST char * parvalue);

/* MSK_putqcon */
MSKrescodee (MSKAPI MSK_putqcon) (	MSKtask_t task,	MSKint32t numqcnz,	MSKCONST MSKint32t * qcsubk,	MSKCONST MSKint32t * qcsubi,	MSKCONST MSKint32t * qcsubj,	MSKCONST MSKrealt * qcval);

/* MSK_putqconk */
MSKrescodee (MSKAPI MSK_putqconk) (	MSKtask_t task,	MSKint32t k,	MSKint32t numqcnz,	MSKCONST MSKint32t * qcsubi,	MSKCONST MSKint32t * qcsubj,	MSKCONST MSKrealt * qcval);

/* MSK_putqobj */
MSKrescodee (MSKAPI MSK_putqobj) (	MSKtask_t task,	MSKint32t numqonz,	MSKCONST MSKint32t * qosubi,	MSKCONST MSKint32t * qosubj,	MSKCONST MSKrealt * qoval);

/* MSK_putqobjij */
MSKrescodee (MSKAPI MSK_putqobjij) (	MSKtask_t task,	MSKint32t i,	MSKint32t j,	MSKrealt qoij);

/* MSK_putsolution */
MSKrescodee (MSKAPI MSK_putsolution) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST MSKstakeye * skc,	MSKCONST MSKstakeye * skx,	MSKCONST MSKstakeye * skn,	MSKCONST MSKrealt * xc,	MSKCONST MSKrealt * xx,	MSKCONST MSKrealt * y,	MSKCONST MSKrealt * slc,	MSKCONST MSKrealt * suc,	MSKCONST MSKrealt * slx,	MSKCONST MSKrealt * sux,	MSKCONST MSKrealt * snx);

/* MSK_putsolutioni */
MSKrescodee (MSKAPI MSK_putsolutioni) (	MSKtask_t task,	MSKaccmodee accmode,	MSKint32t i,	MSKsoltypee whichsol,	MSKstakeye sk,	MSKrealt x,	MSKrealt sl,	MSKrealt su,	MSKrealt sn);

/* MSK_putsolutionyi */
MSKrescodee (MSKAPI MSK_putsolutionyi) (	MSKtask_t task,	MSKint32t i,	MSKsoltypee whichsol,	MSKrealt y);

/* MSK_putstrparam */
MSKrescodee (MSKAPI MSK_putstrparam) (	MSKtask_t task,	MSKsparame param,	MSKCONST char * parvalue);

/* MSK_puttaskname */
MSKrescodee (MSKAPI MSK_puttaskname) (	MSKtask_t task,	MSKCONST char * taskname);

/* MSK_putvartype */
MSKrescodee (MSKAPI MSK_putvartype) (	MSKtask_t task,	MSKint32t j,	MSKvariabletypee vartype);

/* MSK_putvartypelist */
MSKrescodee (MSKAPI MSK_putvartypelist) (	MSKtask_t task,	MSKint32t num,	MSKCONST MSKint32t * subj,	MSKCONST MSKvariabletypee * vartype);

/* MSK_putvarbranchorder */
MSKrescodee (MSKAPI MSK_putvarbranchorder) (	MSKtask_t task,	MSKint32t j,	MSKint32t priority,	int direction);

/* MSK_getvarbranchorder */
MSKrescodee (MSKAPI MSK_getvarbranchorder) (	MSKtask_t task,	MSKint32t j,	MSKint32t * priority,	MSKbranchdire * direction);

/* MSK_getvarbranchpri */
MSKrescodee (MSKAPI MSK_getvarbranchpri) (	MSKtask_t task,	MSKint32t j,	MSKint32t * priority);

/* MSK_getvarbranchdir */
MSKrescodee (MSKAPI MSK_getvarbranchdir) (	MSKtask_t task,	MSKint32t j,	MSKbranchdire * direction);

/* MSK_readdata */
MSKrescodee (MSKAPI MSK_readdata) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_readdataformat */
MSKrescodee (MSKAPI MSK_readdataformat) (	MSKtask_t task,	MSKCONST char * filename,	int format,	int compress);

/* MSK_readdataautoformat */
MSKrescodee (MSKAPI MSK_readdataautoformat) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_readparamfile */
MSKrescodee (MSKAPI MSK_readparamfile) (	MSKtask_t task);

/* MSK_readsolution */
MSKrescodee (MSKAPI MSK_readsolution) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST char * filename);

/* MSK_readsummary */
MSKrescodee (MSKAPI MSK_readsummary) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_resizetask */
MSKrescodee (MSKAPI MSK_resizetask) (	MSKtask_t task,	MSKint32t maxnumcon,	MSKint32t maxnumvar,	MSKint32t maxnumcone,	MSKint64t maxnumanz,	MSKint64t maxnumqnz);

/* MSK_checkmemtask */
MSKrescodee (MSKAPI MSK_checkmemtask) (	MSKtask_t task,	MSKCONST char * file,	MSKint32t line);

/* MSK_getmemusagetask */
MSKrescodee (MSKAPI MSK_getmemusagetask) (	MSKtask_t task,	MSKint64t * meminuse,	MSKint64t * maxmemuse);

/* MSK_setdefaults */
MSKrescodee (MSKAPI MSK_setdefaults) (	MSKtask_t task);

/* MSK_sktostr */
MSKrescodee (MSKAPI MSK_sktostr) (	MSKtask_t task,	MSKstakeye sk,	char * str);

/* MSK_solstatostr */
MSKrescodee (MSKAPI MSK_solstatostr) (	MSKtask_t task,	MSKsolstae solsta,	char * str);

/* MSK_solutiondef */
MSKrescodee (MSKAPI MSK_solutiondef) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKbooleant * isdef);

/* MSK_deletesolution */
MSKrescodee (MSKAPI MSK_deletesolution) (	MSKtask_t task,	MSKsoltypee whichsol);

/* MSK_startstat */
MSKrescodee (MSKAPI MSK_startstat) (	MSKtask_t task);

/* MSK_stopstat */
MSKrescodee (MSKAPI MSK_stopstat) (	MSKtask_t task);

/* MSK_appendstat */
MSKrescodee (MSKAPI MSK_appendstat) (	MSKtask_t task);

/* MSK_onesolutionsummary */
MSKrescodee (MSKAPI MSK_onesolutionsummary) (	MSKtask_t task,	MSKstreamtypee whichstream,	MSKsoltypee whichsol);

/* MSK_solutionsummary */
MSKrescodee (MSKAPI MSK_solutionsummary) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_updatesolutioninfo */
MSKrescodee (MSKAPI MSK_updatesolutioninfo) (	MSKtask_t task,	MSKsoltypee whichsol);

/* MSK_optimizersummary */
MSKrescodee (MSKAPI MSK_optimizersummary) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_strduptask */
char * (MSKAPI MSK_strduptask) (	MSKtask_t task,	MSKCONST char * str);

/* MSK_strdupdbgtask */
char * (MSKAPI MSK_strdupdbgtask) (	MSKtask_t task,	MSKCONST char * str,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_strtoconetype */
MSKrescodee (MSKAPI MSK_strtoconetype) (	MSKtask_t task,	MSKCONST char * str,	MSKconetypee * conetype);

/* MSK_strtosk */
MSKrescodee (MSKAPI MSK_strtosk) (	MSKtask_t task,	MSKCONST char * str,	MSKint32t * sk);

/* MSK_whichparam */
MSKrescodee (MSKAPI MSK_whichparam) (	MSKtask_t task,	MSKCONST char * parname,	MSKparametertypee * partype,	MSKint32t * param);

/* MSK_writedata */
MSKrescodee (MSKAPI MSK_writedata) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_writetask */
MSKrescodee (MSKAPI MSK_writetask) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_readtask */
MSKrescodee (MSKAPI MSK_readtask) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_readbranchpriorities */
MSKrescodee (MSKAPI MSK_readbranchpriorities) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_writebranchpriorities */
MSKrescodee (MSKAPI MSK_writebranchpriorities) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_writeparamfile */
MSKrescodee (MSKAPI MSK_writeparamfile) (	MSKtask_t task,	MSKCONST char * filename);

/* MSK_getinfeasiblesubproblem */
MSKrescodee (MSKAPI MSK_getinfeasiblesubproblem) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKtask_t * inftask);

/* MSK_writesolution */
MSKrescodee (MSKAPI MSK_writesolution) (	MSKtask_t task,	MSKsoltypee whichsol,	MSKCONST char * filename);

/* MSK_primalsensitivity */
MSKrescodee (MSKAPI MSK_primalsensitivity) (	MSKtask_t task,	MSKint32t numi,	MSKCONST MSKint32t * subi,	MSKCONST MSKmarke * marki,	MSKint32t numj,	MSKCONST MSKint32t * subj,	MSKCONST MSKmarke * markj,	MSKrealt * leftpricei,	MSKrealt * rightpricei,	MSKrealt * leftrangei,	MSKrealt * rightrangei,	MSKrealt * leftpricej,	MSKrealt * rightpricej,	MSKrealt * leftrangej,	MSKrealt * rightrangej);

/* MSK_sensitivityreport */
MSKrescodee (MSKAPI MSK_sensitivityreport) (	MSKtask_t task,	MSKstreamtypee whichstream);

/* MSK_dualsensitivity */
MSKrescodee (MSKAPI MSK_dualsensitivity) (	MSKtask_t task,	MSKint32t numj,	MSKCONST MSKint32t * subj,	MSKrealt * leftpricej,	MSKrealt * rightpricej,	MSKrealt * leftrangej,	MSKrealt * rightrangej);

/* MSK_checkconvexity */
MSKrescodee (MSKAPI MSK_checkconvexity) (	MSKtask_t task);

/* MSK_getlasterror */
MSKrescodee (MSKAPI MSK_getlasterror) (	MSKtask_t task,	MSKrescodee * lastrescode,	MSKint32t maxlen,	MSKint32t * lastmsglen,	char * lastmsg);

/* MSK_getlasterror64 */
MSKrescodee (MSKAPI MSK_getlasterror64) (	MSKtask_t task,	MSKrescodee * lastrescode,	MSKint64t maxlen,	MSKint64t * lastmsglen,	char * lastmsg);

/* MSK_isinfinity */
MSKbooleant (MSKAPI MSK_isinfinity) (	MSKrealt value);

/* MSK_checkoutlicense */
MSKrescodee (MSKAPI MSK_checkoutlicense) (	MSKenv_t env,	MSKfeaturee feature);

/* MSK_checkinlicense */
MSKrescodee (MSKAPI MSK_checkinlicense) (	MSKenv_t env,	MSKfeaturee feature);

/* MSK_getbuildinfo */
MSKrescodee (MSKAPI MSK_getbuildinfo) (	char * buildstate,	char * builddate,	char * buildtool);

/* MSK_getresponseclass */
MSKrescodee (MSKAPI MSK_getresponseclass) (	MSKrescodee r,	MSKrescodetypee * rc);

/* MSK_callocenv */
void * (MSKAPI MSK_callocenv) (	MSKenv_t env,	MSKCONST size_t number,	MSKCONST size_t size);

/* MSK_callocdbgenv */
void * (MSKAPI MSK_callocdbgenv) (	MSKenv_t env,	MSKCONST size_t number,	MSKCONST size_t size,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_deleteenv */
MSKrescodee (MSKAPI MSK_deleteenv) (	MSKenv_t * env);

/* MSK_echoenv */
MSKrescodee (MSKAPIVA MSK_echoenv) (	MSKenv_t env,	MSKstreamtypee whichstream,	MSKCONST char * format,	...);

/* MSK_echointro */
MSKrescodee (MSKAPI MSK_echointro) (	MSKenv_t env,	MSKint32t longver);

/* MSK_freeenv */
void (MSKAPI MSK_freeenv) (	MSKenv_t env,	MSKCONST void * buffer);

/* MSK_freedbgenv */
void (MSKAPI MSK_freedbgenv) (	MSKenv_t env,	MSKCONST void * buffer,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_getcodedesc */
MSKrescodee (MSKAPI MSK_getcodedesc) (	MSKrescodee code,	char * symname,	char * str);

/* MSK_getsymbcondim */
MSKrescodee (MSKAPI MSK_getsymbcondim) (	MSKenv_t env,	MSKint32t * num,	size_t * maxlen);

/* MSK_getversion */
MSKrescodee (MSKAPI MSK_getversion) (	MSKint32t * major,	MSKint32t * minor,	MSKint32t * build,	MSKint32t * revision);

/* MSK_checkversion */
MSKrescodee (MSKAPI MSK_checkversion) (	MSKenv_t env,	MSKint32t major,	MSKint32t minor,	MSKint32t build,	MSKint32t revision);

/* MSK_iparvaltosymnam */
MSKrescodee (MSKAPI MSK_iparvaltosymnam) (	MSKenv_t env,	MSKiparame whichparam,	MSKint32t whichvalue,	char * symbolicname);

/* MSK_linkfiletoenvstream */
MSKrescodee (MSKAPI MSK_linkfiletoenvstream) (	MSKenv_t env,	MSKstreamtypee whichstream,	MSKCONST char * filename,	MSKint32t append);

/* MSK_linkfunctoenvstream */
MSKrescodee (MSKAPI MSK_linkfunctoenvstream) (	MSKenv_t env,	MSKstreamtypee whichstream,	MSKuserhandle_t handle,	MSKstreamfunc func);

/* MSK_unlinkfuncfromenvstream */
MSKrescodee (MSKAPI MSK_unlinkfuncfromenvstream) (	MSKenv_t env,	MSKstreamtypee whichstream);

/* MSK_makeenv */
MSKrescodee (MSKAPI MSK_makeenv) (	MSKenv_t * env,	MSKCONST char * dbgfile);

/* MSK_makeenvalloc */
MSKrescodee (MSKAPI MSK_makeenvalloc) (	MSKenv_t * env,	MSKuserhandle_t usrptr,	MSKmallocfunc usrmalloc,	MSKcallocfunc usrcalloc,	MSKreallocfunc usrrealloc,	MSKfreefunc usrfree,	MSKCONST char * dbgfile);

/* MSK_initenv */
MSKrescodee (MSKAPI MSK_initenv) (	MSKenv_t env);

/* MSK_getglbdllname */
MSKrescodee (MSKAPI MSK_getglbdllname) (	MSKenv_t env,	MSKCONST size_t sizedllname,	char * dllname);

/* MSK_putdllpath */
MSKrescodee (MSKAPI MSK_putdllpath) (	MSKenv_t env,	MSKCONST char * dllpath);

/* MSK_putlicensedebug */
MSKrescodee (MSKAPI MSK_putlicensedebug) (	MSKenv_t env,	MSKint32t licdebug);

/* MSK_putlicensecode */
MSKrescodee (MSKAPI MSK_putlicensecode) (	MSKenv_t env,	MSKCONST MSKint32t * code);

/* MSK_putlicensewait */
MSKrescodee (MSKAPI MSK_putlicensewait) (	MSKenv_t env,	MSKint32t licwait);

/* MSK_putlicensepath */
MSKrescodee (MSKAPI MSK_putlicensepath) (	MSKenv_t env,	MSKCONST char * licensepath);

/* MSK_putkeepdlls */
MSKrescodee (MSKAPI MSK_putkeepdlls) (	MSKenv_t env,	MSKint32t keepdlls);

/* MSK_maketask */
MSKrescodee (MSKAPI MSK_maketask) (	MSKenv_t env,	MSKint32t maxnumcon,	MSKint32t maxnumvar,	MSKtask_t * task);

/* MSK_makeemptytask */
MSKrescodee (MSKAPI MSK_makeemptytask) (	MSKenv_t env,	MSKtask_t * task);

/* MSK_putexitfunc */
MSKrescodee (MSKAPI MSK_putexitfunc) (	MSKenv_t env,	MSKexitfunc exitfunc,	MSKuserhandle_t handle);

/* MSK_utf8towchar */
MSKrescodee (MSKAPI MSK_utf8towchar) (	MSKCONST size_t outputlen,	size_t * len,	size_t * conv,	MSKwchart * output,	MSKCONST char * input);

/* MSK_wchartoutf8 */
MSKrescodee (MSKAPI MSK_wchartoutf8) (	MSKCONST size_t outputlen,	size_t * len,	size_t * conv,	char * output,	MSKCONST MSKwchart * input);

/* MSK_checkmemenv */
MSKrescodee (MSKAPI MSK_checkmemenv) (	MSKenv_t env,	MSKCONST char * file,	MSKint32t line);

/* MSK_strdupenv */
char * (MSKAPI MSK_strdupenv) (	MSKenv_t env,	MSKCONST char * str);

/* MSK_strdupdbgenv */
char * (MSKAPI MSK_strdupdbgenv) (	MSKenv_t env,	MSKCONST char * str,	MSKCONST char * file,	MSKCONST unsigned line);

/* MSK_symnamtovalue */
MSKbooleant (MSKAPI MSK_symnamtovalue) (	MSKCONST char * name,	char * value);

/* MSK_axpy */
MSKrescodee (MSKAPI MSK_axpy) (	MSKenv_t env,	MSKint32t n,	MSKrealt alpha,	MSKCONST MSKrealt * x,	MSKrealt * y);

/* MSK_dot */
MSKrescodee (MSKAPI MSK_dot) (	MSKenv_t env,	MSKint32t n,	MSKCONST MSKrealt * x,	MSKCONST MSKrealt * y,	MSKrealt * xty);

/* MSK_gemv */
MSKrescodee (MSKAPI MSK_gemv) (	MSKenv_t env,	MSKtransposee transa,	MSKint32t m,	MSKint32t n,	MSKrealt alpha,	MSKCONST MSKrealt * a,	MSKCONST MSKrealt * x,	MSKrealt beta,	MSKrealt * y);

/* MSK_gemm */
MSKrescodee (MSKAPI MSK_gemm) (	MSKenv_t env,	MSKtransposee transa,	MSKtransposee transb,	MSKint32t m,	MSKint32t n,	MSKint32t k,	MSKrealt alpha,	MSKCONST MSKrealt * a,	MSKCONST MSKrealt * b,	MSKrealt beta,	MSKrealt * c);

/* MSK_syrk */
MSKrescodee (MSKAPI MSK_syrk) (	MSKenv_t env,	MSKuploe uplo,	MSKtransposee trans,	MSKint32t n,	MSKint32t k,	MSKrealt alpha,	MSKCONST MSKrealt * a,	MSKrealt beta,	MSKrealt * c);

/* MSK_potrf */
MSKrescodee (MSKAPI MSK_potrf) (	MSKenv_t env,	MSKuploe uplo,	MSKint32t n,	MSKrealt * a);

/* MSK_syeig */
MSKrescodee (MSKAPI MSK_syeig) (	MSKenv_t env,	MSKuploe uplo,	MSKint32t n,	MSKCONST MSKrealt * a,	MSKrealt * w);

/* MSK_syevd */
MSKrescodee (MSKAPI MSK_syevd) (	MSKenv_t env,	MSKuploe uplo,	MSKint32t n,	MSKrealt * a,	MSKrealt * w);

/* MSK_licensecleanup */
MSKrescodee (MSKAPI MSK_licensecleanup) (void);



#ifdef __cplusplus
}
#endif


#endif


