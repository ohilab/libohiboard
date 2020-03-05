/*
 * MK64F12_portability.h
 *
 *  Created on: 23 lug 2018
 *      Author: m.piersantelli
 */

#ifndef LIBOHIBOARD_INCLUDE_PLATFORMS_MK64F12_PORTABILITY_H_
#define LIBOHIBOARD_INCLUDE_PLATFORMS_MK64F12_PORTABILITY_H_

#include "MK64F12.h"

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register accessors */
#define ADC_SC1_REG(base,index)                  ((base)->SC1[index])
#define ADC_CFG1_REG(base)                       ((base)->CFG1)
#define ADC_CFG2_REG(base)                       ((base)->CFG2)
#define ADC_R_REG(base,index)                    ((base)->R[index])
#define ADC_CV1_REG(base)                        ((base)->CV1)
#define ADC_CV2_REG(base)                        ((base)->CV2)
#define ADC_SC2_REG(base)                        ((base)->SC2)
#define ADC_SC3_REG(base)                        ((base)->SC3)
#define ADC_OFS_REG(base)                        ((base)->OFS)
#define ADC_PG_REG(base)                         ((base)->PG)
#define ADC_MG_REG(base)                         ((base)->MG)
#define ADC_CLPD_REG(base)                       ((base)->CLPD)
#define ADC_CLPS_REG(base)                       ((base)->CLPS)
#define ADC_CLP4_REG(base)                       ((base)->CLP4)
#define ADC_CLP3_REG(base)                       ((base)->CLP3)
#define ADC_CLP2_REG(base)                       ((base)->CLP2)
#define ADC_CLP1_REG(base)                       ((base)->CLP1)
#define ADC_CLP0_REG(base)                       ((base)->CLP0)
#define ADC_CLMD_REG(base)                       ((base)->CLMD)
#define ADC_CLMS_REG(base)                       ((base)->CLMS)
#define ADC_CLM4_REG(base)                       ((base)->CLM4)
#define ADC_CLM3_REG(base)                       ((base)->CLM3)
#define ADC_CLM2_REG(base)                       ((base)->CLM2)
#define ADC_CLM1_REG(base)                       ((base)->CLM1)
#define ADC_CLM0_REG(base)                       ((base)->CLM0)

/* ADC - Peripheral instance base addresses */
#define ADC0_BASE_PTR                            (ADC0)
#define ADC1_BASE_PTR                            (ADC1)


/* ADC - Register instance definitions */
/* ADC0 */
#define ADC0_SC1A                                ADC_SC1_REG(ADC0,0)
#define ADC0_SC1B                                ADC_SC1_REG(ADC0,1)
#define ADC0_CFG1                                ADC_CFG1_REG(ADC0)
#define ADC0_CFG2                                ADC_CFG2_REG(ADC0)
#define ADC0_RA                                  ADC_R_REG(ADC0,0)
#define ADC0_RB                                  ADC_R_REG(ADC0,1)
#define ADC0_CV1                                 ADC_CV1_REG(ADC0)
#define ADC0_CV2                                 ADC_CV2_REG(ADC0)
#define ADC0_SC2                                 ADC_SC2_REG(ADC0)
#define ADC0_SC3                                 ADC_SC3_REG(ADC0)
#define ADC0_OFS                                 ADC_OFS_REG(ADC0)
#define ADC0_PG                                  ADC_PG_REG(ADC0)
#define ADC0_MG                                  ADC_MG_REG(ADC0)
#define ADC0_CLPD                                ADC_CLPD_REG(ADC0)
#define ADC0_CLPS                                ADC_CLPS_REG(ADC0)
#define ADC0_CLP4                                ADC_CLP4_REG(ADC0)
#define ADC0_CLP3                                ADC_CLP3_REG(ADC0)
#define ADC0_CLP2                                ADC_CLP2_REG(ADC0)
#define ADC0_CLP1                                ADC_CLP1_REG(ADC0)
#define ADC0_CLP0                                ADC_CLP0_REG(ADC0)
#define ADC0_CLMD                                ADC_CLMD_REG(ADC0)
#define ADC0_CLMS                                ADC_CLMS_REG(ADC0)
#define ADC0_CLM4                                ADC_CLM4_REG(ADC0)
#define ADC0_CLM3                                ADC_CLM3_REG(ADC0)
#define ADC0_CLM2                                ADC_CLM2_REG(ADC0)
#define ADC0_CLM1                                ADC_CLM1_REG(ADC0)
#define ADC0_CLM0                                ADC_CLM0_REG(ADC0)
/* ADC1 */
#define ADC1_SC1A                                ADC_SC1_REG(ADC1,0)
#define ADC1_SC1B                                ADC_SC1_REG(ADC1,1)
#define ADC1_CFG1                                ADC_CFG1_REG(ADC1)
#define ADC1_CFG2                                ADC_CFG2_REG(ADC1)
#define ADC1_RA                                  ADC_R_REG(ADC1,0)
#define ADC1_RB                                  ADC_R_REG(ADC1,1)
#define ADC1_CV1                                 ADC_CV1_REG(ADC1)
#define ADC1_CV2                                 ADC_CV2_REG(ADC1)
#define ADC1_SC2                                 ADC_SC2_REG(ADC1)
#define ADC1_SC3                                 ADC_SC3_REG(ADC1)
#define ADC1_OFS                                 ADC_OFS_REG(ADC1)
#define ADC1_PG                                  ADC_PG_REG(ADC1)
#define ADC1_MG                                  ADC_MG_REG(ADC1)
#define ADC1_CLPD                                ADC_CLPD_REG(ADC1)
#define ADC1_CLPS                                ADC_CLPS_REG(ADC1)
#define ADC1_CLP4                                ADC_CLP4_REG(ADC1)
#define ADC1_CLP3                                ADC_CLP3_REG(ADC1)
#define ADC1_CLP2                                ADC_CLP2_REG(ADC1)
#define ADC1_CLP1                                ADC_CLP1_REG(ADC1)
#define ADC1_CLP0                                ADC_CLP0_REG(ADC1)
#define ADC1_CLMD                                ADC_CLMD_REG(ADC1)
#define ADC1_CLMS                                ADC_CLMS_REG(ADC1)
#define ADC1_CLM4                                ADC_CLM4_REG(ADC1)
#define ADC1_CLM3                                ADC_CLM3_REG(ADC1)
#define ADC1_CLM2                                ADC_CLM2_REG(ADC1)
#define ADC1_CLM1                                ADC_CLM1_REG(ADC1)
#define ADC1_CLM0                                ADC_CLM0_REG(ADC1)

/* ADC - Register array accessors */
#define ADC0_SC1(index)                          ADC_SC1_REG(ADC0,index)
#define ADC1_SC1(index)                          ADC_SC1_REG(ADC1,index)
#define ADC0_R(index)                            ADC_R_REG(ADC0,index)
#define ADC1_R(index)                            ADC_R_REG(ADC1,index)


/* ----------------------------------------------------------------------------
   -- AIPS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AIPS_Register_Accessor_Macros AIPS - Register accessor macros
 * @{
 */


/* AIPS - Register accessors */
#define AIPS_MPRA_REG(base)                      ((base)->MPRA)
#define AIPS_PACRA_REG(base)                     ((base)->PACRA)
#define AIPS_PACRB_REG(base)                     ((base)->PACRB)
#define AIPS_PACRC_REG(base)                     ((base)->PACRC)
#define AIPS_PACRD_REG(base)                     ((base)->PACRD)
#define AIPS_PACRE_REG(base)                     ((base)->PACRE)
#define AIPS_PACRF_REG(base)                     ((base)->PACRF)
#define AIPS_PACRG_REG(base)                     ((base)->PACRG)
#define AIPS_PACRH_REG(base)                     ((base)->PACRH)
#define AIPS_PACRI_REG(base)                     ((base)->PACRI)
#define AIPS_PACRJ_REG(base)                     ((base)->PACRJ)
#define AIPS_PACRK_REG(base)                     ((base)->PACRK)
#define AIPS_PACRL_REG(base)                     ((base)->PACRL)
#define AIPS_PACRM_REG(base)                     ((base)->PACRM)
#define AIPS_PACRN_REG(base)                     ((base)->PACRN)
#define AIPS_PACRO_REG(base)                     ((base)->PACRO)
#define AIPS_PACRP_REG(base)                     ((base)->PACRP)
#define AIPS_PACRU_REG(base)                     ((base)->PACRU)

/* AIPS - Peripheral instance base addresses */
#define AIPS0_BASE_PTR                           (AIPS0)
#define AIPS1_BASE_PTR                           (AIPS1)

/* AIPS - Register instance definitions */
/* AIPS0 */
#define AIPS0_MPRA                               AIPS_MPRA_REG(AIPS0)
#define AIPS0_PACRA                              AIPS_PACRA_REG(AIPS0)
#define AIPS0_PACRB                              AIPS_PACRB_REG(AIPS0)
#define AIPS0_PACRC                              AIPS_PACRC_REG(AIPS0)
#define AIPS0_PACRD                              AIPS_PACRD_REG(AIPS0)
#define AIPS0_PACRE                              AIPS_PACRE_REG(AIPS0)
#define AIPS0_PACRF                              AIPS_PACRF_REG(AIPS0)
#define AIPS0_PACRG                              AIPS_PACRG_REG(AIPS0)
#define AIPS0_PACRH                              AIPS_PACRH_REG(AIPS0)
#define AIPS0_PACRI                              AIPS_PACRI_REG(AIPS0)
#define AIPS0_PACRJ                              AIPS_PACRJ_REG(AIPS0)
#define AIPS0_PACRK                              AIPS_PACRK_REG(AIPS0)
#define AIPS0_PACRL                              AIPS_PACRL_REG(AIPS0)
#define AIPS0_PACRM                              AIPS_PACRM_REG(AIPS0)
#define AIPS0_PACRN                              AIPS_PACRN_REG(AIPS0)
#define AIPS0_PACRO                              AIPS_PACRO_REG(AIPS0)
#define AIPS0_PACRP                              AIPS_PACRP_REG(AIPS0)
#define AIPS0_PACRU                              AIPS_PACRU_REG(AIPS0)
/* AIPS1 */
#define AIPS1_MPRA                               AIPS_MPRA_REG(AIPS1)
#define AIPS1_PACRA                              AIPS_PACRA_REG(AIPS1)
#define AIPS1_PACRB                              AIPS_PACRB_REG(AIPS1)
#define AIPS1_PACRC                              AIPS_PACRC_REG(AIPS1)
#define AIPS1_PACRD                              AIPS_PACRD_REG(AIPS1)
#define AIPS1_PACRE                              AIPS_PACRE_REG(AIPS1)
#define AIPS1_PACRF                              AIPS_PACRF_REG(AIPS1)
#define AIPS1_PACRG                              AIPS_PACRG_REG(AIPS1)
#define AIPS1_PACRH                              AIPS_PACRH_REG(AIPS1)
#define AIPS1_PACRI                              AIPS_PACRI_REG(AIPS1)
#define AIPS1_PACRJ                              AIPS_PACRJ_REG(AIPS1)
#define AIPS1_PACRK                              AIPS_PACRK_REG(AIPS1)
#define AIPS1_PACRL                              AIPS_PACRL_REG(AIPS1)
#define AIPS1_PACRM                              AIPS_PACRM_REG(AIPS1)
#define AIPS1_PACRN                              AIPS_PACRN_REG(AIPS1)
#define AIPS1_PACRO                              AIPS_PACRO_REG(AIPS1)
#define AIPS1_PACRP                              AIPS_PACRP_REG(AIPS1)
#define AIPS1_PACRU                              AIPS_PACRU_REG(AIPS1)


/* ----------------------------------------------------------------------------
   -- AXBS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup AXBS_Register_Accessor_Macros AXBS - Register accessor macros
 * @{
 */


/* AXBS - Register accessors */
#define AXBS_PRS_REG(base,index)                 ((base)->SLAVE[index].PRS)
#define AXBS_CRS_REG(base,index)                 ((base)->SLAVE[index].CRS)
#define AXBS_MGPCR0_REG(base)                    ((base)->MGPCR0)
#define AXBS_MGPCR1_REG(base)                    ((base)->MGPCR1)
#define AXBS_MGPCR2_REG(base)                    ((base)->MGPCR2)
#define AXBS_MGPCR3_REG(base)                    ((base)->MGPCR3)
#define AXBS_MGPCR4_REG(base)                    ((base)->MGPCR4)
#define AXBS_MGPCR5_REG(base)                    ((base)->MGPCR5)

/* AXBS - Peripheral instance base addresses */
#define AXBS_BASE_PTR                            (AXBS)


/* AXBS - Register instance definitions */
/* AXBS */
#define AXBS_PRS0                                AXBS_PRS_REG(AXBS,0)
#define AXBS_CRS0                                AXBS_CRS_REG(AXBS,0)
#define AXBS_PRS1                                AXBS_PRS_REG(AXBS,1)
#define AXBS_CRS1                                AXBS_CRS_REG(AXBS,1)
#define AXBS_PRS2                                AXBS_PRS_REG(AXBS,2)
#define AXBS_CRS2                                AXBS_CRS_REG(AXBS,2)
#define AXBS_PRS3                                AXBS_PRS_REG(AXBS,3)
#define AXBS_CRS3                                AXBS_CRS_REG(AXBS,3)
#define AXBS_PRS4                                AXBS_PRS_REG(AXBS,4)
#define AXBS_CRS4                                AXBS_CRS_REG(AXBS,4)
#define AXBS_MGPCR0                              AXBS_MGPCR0_REG(AXBS)
#define AXBS_MGPCR1                              AXBS_MGPCR1_REG(AXBS)
#define AXBS_MGPCR2                              AXBS_MGPCR2_REG(AXBS)
#define AXBS_MGPCR3                              AXBS_MGPCR3_REG(AXBS)
#define AXBS_MGPCR4                              AXBS_MGPCR4_REG(AXBS)
#define AXBS_MGPCR5                              AXBS_MGPCR5_REG(AXBS)

/* AXBS - Register array accessors */
#define AXBS_PRS(index)                          AXBS_PRS_REG(AXBS,index)
#define AXBS_CRS(index)                          AXBS_CRS_REG(AXBS,index)



/* ----------------------------------------------------------------------------
   -- CAN - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAN_Register_Accessor_Macros CAN - Register accessor macros
 * @{
 */


/* CAN - Register accessors */
#define CAN_MCR_REG(base)                        ((base)->MCR)
#define CAN_CTRL1_REG(base)                      ((base)->CTRL1)
#define CAN_TIMER_REG(base)                      ((base)->TIMER)
#define CAN_RXMGMASK_REG(base)                   ((base)->RXMGMASK)
#define CAN_RX14MASK_REG(base)                   ((base)->RX14MASK)
#define CAN_RX15MASK_REG(base)                   ((base)->RX15MASK)
#define CAN_ECR_REG(base)                        ((base)->ECR)
#define CAN_ESR1_REG(base)                       ((base)->ESR1)
#define CAN_IMASK1_REG(base)                     ((base)->IMASK1)
#define CAN_IFLAG1_REG(base)                     ((base)->IFLAG1)
#define CAN_CTRL2_REG(base)                      ((base)->CTRL2)
#define CAN_ESR2_REG(base)                       ((base)->ESR2)
#define CAN_CRCR_REG(base)                       ((base)->CRCR)
#define CAN_RXFGMASK_REG(base)                   ((base)->RXFGMASK)
#define CAN_RXFIR_REG(base)                      ((base)->RXFIR)
#define CAN_CS_REG(base,index)                   ((base)->MB[index].CS)
#define CAN_ID_REG(base,index)                   ((base)->MB[index].ID)
#define CAN_WORD0_REG(base,index)                ((base)->MB[index].WORD0)
#define CAN_WORD1_REG(base,index)                ((base)->MB[index].WORD1)
#define CAN_RXIMR_REG(base,index)                ((base)->RXIMR[index])

/* CAN - Peripheral instance base addresses */
#define CAN0_BASE_PTR                            (CAN0)


/* CAN - Register instance definitions */
/* CAN0 */
#define CAN0_MCR                                 CAN_MCR_REG(CAN0)
#define CAN0_CTRL1                               CAN_CTRL1_REG(CAN0)
#define CAN0_TIMER                               CAN_TIMER_REG(CAN0)
#define CAN0_RXMGMASK                            CAN_RXMGMASK_REG(CAN0)
#define CAN0_RX14MASK                            CAN_RX14MASK_REG(CAN0)
#define CAN0_RX15MASK                            CAN_RX15MASK_REG(CAN0)
#define CAN0_ECR                                 CAN_ECR_REG(CAN0)
#define CAN0_ESR1                                CAN_ESR1_REG(CAN0)
#define CAN0_IMASK1                              CAN_IMASK1_REG(CAN0)
#define CAN0_IFLAG1                              CAN_IFLAG1_REG(CAN0)
#define CAN0_CTRL2                               CAN_CTRL2_REG(CAN0)
#define CAN0_ESR2                                CAN_ESR2_REG(CAN0)
#define CAN0_CRCR                                CAN_CRCR_REG(CAN0)
#define CAN0_RXFGMASK                            CAN_RXFGMASK_REG(CAN0)
#define CAN0_RXFIR                               CAN_RXFIR_REG(CAN0)
#define CAN0_CS0                                 CAN_CS_REG(CAN0,0)
#define CAN0_ID0                                 CAN_ID_REG(CAN0,0)
#define CAN0_WORD00                              CAN_WORD0_REG(CAN0,0)
#define CAN0_WORD10                              CAN_WORD1_REG(CAN0,0)
#define CAN0_CS1                                 CAN_CS_REG(CAN0,1)
#define CAN0_ID1                                 CAN_ID_REG(CAN0,1)
#define CAN0_WORD01                              CAN_WORD0_REG(CAN0,1)
#define CAN0_WORD11                              CAN_WORD1_REG(CAN0,1)
#define CAN0_CS2                                 CAN_CS_REG(CAN0,2)
#define CAN0_ID2                                 CAN_ID_REG(CAN0,2)
#define CAN0_WORD02                              CAN_WORD0_REG(CAN0,2)
#define CAN0_WORD12                              CAN_WORD1_REG(CAN0,2)
#define CAN0_CS3                                 CAN_CS_REG(CAN0,3)
#define CAN0_ID3                                 CAN_ID_REG(CAN0,3)
#define CAN0_WORD03                              CAN_WORD0_REG(CAN0,3)
#define CAN0_WORD13                              CAN_WORD1_REG(CAN0,3)
#define CAN0_CS4                                 CAN_CS_REG(CAN0,4)
#define CAN0_ID4                                 CAN_ID_REG(CAN0,4)
#define CAN0_WORD04                              CAN_WORD0_REG(CAN0,4)
#define CAN0_WORD14                              CAN_WORD1_REG(CAN0,4)
#define CAN0_CS5                                 CAN_CS_REG(CAN0,5)
#define CAN0_ID5                                 CAN_ID_REG(CAN0,5)
#define CAN0_WORD05                              CAN_WORD0_REG(CAN0,5)
#define CAN0_WORD15                              CAN_WORD1_REG(CAN0,5)
#define CAN0_CS6                                 CAN_CS_REG(CAN0,6)
#define CAN0_ID6                                 CAN_ID_REG(CAN0,6)
#define CAN0_WORD06                              CAN_WORD0_REG(CAN0,6)
#define CAN0_WORD16                              CAN_WORD1_REG(CAN0,6)
#define CAN0_CS7                                 CAN_CS_REG(CAN0,7)
#define CAN0_ID7                                 CAN_ID_REG(CAN0,7)
#define CAN0_WORD07                              CAN_WORD0_REG(CAN0,7)
#define CAN0_WORD17                              CAN_WORD1_REG(CAN0,7)
#define CAN0_CS8                                 CAN_CS_REG(CAN0,8)
#define CAN0_ID8                                 CAN_ID_REG(CAN0,8)
#define CAN0_WORD08                              CAN_WORD0_REG(CAN0,8)
#define CAN0_WORD18                              CAN_WORD1_REG(CAN0,8)
#define CAN0_CS9                                 CAN_CS_REG(CAN0,9)
#define CAN0_ID9                                 CAN_ID_REG(CAN0,9)
#define CAN0_WORD09                              CAN_WORD0_REG(CAN0,9)
#define CAN0_WORD19                              CAN_WORD1_REG(CAN0,9)
#define CAN0_CS10                                CAN_CS_REG(CAN0,10)
#define CAN0_ID10                                CAN_ID_REG(CAN0,10)
#define CAN0_WORD010                             CAN_WORD0_REG(CAN0,10)
#define CAN0_WORD110                             CAN_WORD1_REG(CAN0,10)
#define CAN0_CS11                                CAN_CS_REG(CAN0,11)
#define CAN0_ID11                                CAN_ID_REG(CAN0,11)
#define CAN0_WORD011                             CAN_WORD0_REG(CAN0,11)
#define CAN0_WORD111                             CAN_WORD1_REG(CAN0,11)
#define CAN0_CS12                                CAN_CS_REG(CAN0,12)
#define CAN0_ID12                                CAN_ID_REG(CAN0,12)
#define CAN0_WORD012                             CAN_WORD0_REG(CAN0,12)
#define CAN0_WORD112                             CAN_WORD1_REG(CAN0,12)
#define CAN0_CS13                                CAN_CS_REG(CAN0,13)
#define CAN0_ID13                                CAN_ID_REG(CAN0,13)
#define CAN0_WORD013                             CAN_WORD0_REG(CAN0,13)
#define CAN0_WORD113                             CAN_WORD1_REG(CAN0,13)
#define CAN0_CS14                                CAN_CS_REG(CAN0,14)
#define CAN0_ID14                                CAN_ID_REG(CAN0,14)
#define CAN0_WORD014                             CAN_WORD0_REG(CAN0,14)
#define CAN0_WORD114                             CAN_WORD1_REG(CAN0,14)
#define CAN0_CS15                                CAN_CS_REG(CAN0,15)
#define CAN0_ID15                                CAN_ID_REG(CAN0,15)
#define CAN0_WORD015                             CAN_WORD0_REG(CAN0,15)
#define CAN0_WORD115                             CAN_WORD1_REG(CAN0,15)
#define CAN0_RXIMR0                              CAN_RXIMR_REG(CAN0,0)
#define CAN0_RXIMR1                              CAN_RXIMR_REG(CAN0,1)
#define CAN0_RXIMR2                              CAN_RXIMR_REG(CAN0,2)
#define CAN0_RXIMR3                              CAN_RXIMR_REG(CAN0,3)
#define CAN0_RXIMR4                              CAN_RXIMR_REG(CAN0,4)
#define CAN0_RXIMR5                              CAN_RXIMR_REG(CAN0,5)
#define CAN0_RXIMR6                              CAN_RXIMR_REG(CAN0,6)
#define CAN0_RXIMR7                              CAN_RXIMR_REG(CAN0,7)
#define CAN0_RXIMR8                              CAN_RXIMR_REG(CAN0,8)
#define CAN0_RXIMR9                              CAN_RXIMR_REG(CAN0,9)
#define CAN0_RXIMR10                             CAN_RXIMR_REG(CAN0,10)
#define CAN0_RXIMR11                             CAN_RXIMR_REG(CAN0,11)
#define CAN0_RXIMR12                             CAN_RXIMR_REG(CAN0,12)
#define CAN0_RXIMR13                             CAN_RXIMR_REG(CAN0,13)
#define CAN0_RXIMR14                             CAN_RXIMR_REG(CAN0,14)
#define CAN0_RXIMR15                             CAN_RXIMR_REG(CAN0,15)

/* CAN - Register array accessors */
#define CAN0_CS(index)                           CAN_CS_REG(CAN0,index)
#define CAN0_ID(index)                           CAN_ID_REG(CAN0,index)
#define CAN0_WORD0(index)                        CAN_WORD0_REG(CAN0,index)
#define CAN0_WORD1(index)                        CAN_WORD1_REG(CAN0,index)
#define CAN0_RXIMR(index)                        CAN_RXIMR_REG(CAN0,index)



/* ----------------------------------------------------------------------------
   -- CAU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CAU_Register_Accessor_Macros CAU - Register accessor macros
 * @{
 */


/* CAU - Register accessors */
#define CAU_DIRECT_REG(base,index)               ((base)->DIRECT[index])
#define CAU_LDR_CASR_REG(base)                   ((base)->LDR_CASR)
#define CAU_LDR_CAA_REG(base)                    ((base)->LDR_CAA)
#define CAU_LDR_CA_REG(base,index)               ((base)->LDR_CA[index])
#define CAU_STR_CASR_REG(base)                   ((base)->STR_CASR)
#define CAU_STR_CAA_REG(base)                    ((base)->STR_CAA)
#define CAU_STR_CA_REG(base,index)               ((base)->STR_CA[index])
#define CAU_ADR_CASR_REG(base)                   ((base)->ADR_CASR)
#define CAU_ADR_CAA_REG(base)                    ((base)->ADR_CAA)
#define CAU_ADR_CA_REG(base,index)               ((base)->ADR_CA[index])
#define CAU_RADR_CASR_REG(base)                  ((base)->RADR_CASR)
#define CAU_RADR_CAA_REG(base)                   ((base)->RADR_CAA)
#define CAU_RADR_CA_REG(base,index)              ((base)->RADR_CA[index])
#define CAU_XOR_CASR_REG(base)                   ((base)->XOR_CASR)
#define CAU_XOR_CAA_REG(base)                    ((base)->XOR_CAA)
#define CAU_XOR_CA_REG(base,index)               ((base)->XOR_CA[index])
#define CAU_ROTL_CASR_REG(base)                  ((base)->ROTL_CASR)
#define CAU_ROTL_CAA_REG(base)                   ((base)->ROTL_CAA)
#define CAU_ROTL_CA_REG(base,index)              ((base)->ROTL_CA[index])
#define CAU_AESC_CASR_REG(base)                  ((base)->AESC_CASR)
#define CAU_AESC_CAA_REG(base)                   ((base)->AESC_CAA)
#define CAU_AESC_CA_REG(base,index)              ((base)->AESC_CA[index])
#define CAU_AESIC_CASR_REG(base)                 ((base)->AESIC_CASR)
#define CAU_AESIC_CAA_REG(base)                  ((base)->AESIC_CAA)
#define CAU_AESIC_CA_REG(base,index)             ((base)->AESIC_CA[index])

/* CAU - Peripheral instance base addresses */
#define CAU_BASE_PTR                             (CAU)

/* CAU - Register instance definitions */
/* CAU */
#define CAU_DIRECT0                              CAU_DIRECT_REG(CAU,0)
#define CAU_DIRECT1                              CAU_DIRECT_REG(CAU,1)
#define CAU_DIRECT2                              CAU_DIRECT_REG(CAU,2)
#define CAU_DIRECT3                              CAU_DIRECT_REG(CAU,3)
#define CAU_DIRECT4                              CAU_DIRECT_REG(CAU,4)
#define CAU_DIRECT5                              CAU_DIRECT_REG(CAU,5)
#define CAU_DIRECT6                              CAU_DIRECT_REG(CAU,6)
#define CAU_DIRECT7                              CAU_DIRECT_REG(CAU,7)
#define CAU_DIRECT8                              CAU_DIRECT_REG(CAU,8)
#define CAU_DIRECT9                              CAU_DIRECT_REG(CAU,9)
#define CAU_DIRECT10                             CAU_DIRECT_REG(CAU,10)
#define CAU_DIRECT11                             CAU_DIRECT_REG(CAU,11)
#define CAU_DIRECT12                             CAU_DIRECT_REG(CAU,12)
#define CAU_DIRECT13                             CAU_DIRECT_REG(CAU,13)
#define CAU_DIRECT14                             CAU_DIRECT_REG(CAU,14)
#define CAU_DIRECT15                             CAU_DIRECT_REG(CAU,15)
#define CAU_LDR_CASR                             CAU_LDR_CASR_REG(CAU)
#define CAU_LDR_CAA                              CAU_LDR_CAA_REG(CAU)
#define CAU_LDR_CA0                              CAU_LDR_CA_REG(CAU,0)
#define CAU_LDR_CA1                              CAU_LDR_CA_REG(CAU,1)
#define CAU_LDR_CA2                              CAU_LDR_CA_REG(CAU,2)
#define CAU_LDR_CA3                              CAU_LDR_CA_REG(CAU,3)
#define CAU_LDR_CA4                              CAU_LDR_CA_REG(CAU,4)
#define CAU_LDR_CA5                              CAU_LDR_CA_REG(CAU,5)
#define CAU_LDR_CA6                              CAU_LDR_CA_REG(CAU,6)
#define CAU_LDR_CA7                              CAU_LDR_CA_REG(CAU,7)
#define CAU_LDR_CA8                              CAU_LDR_CA_REG(CAU,8)
#define CAU_STR_CASR                             CAU_STR_CASR_REG(CAU)
#define CAU_STR_CAA                              CAU_STR_CAA_REG(CAU)
#define CAU_STR_CA0                              CAU_STR_CA_REG(CAU,0)
#define CAU_STR_CA1                              CAU_STR_CA_REG(CAU,1)
#define CAU_STR_CA2                              CAU_STR_CA_REG(CAU,2)
#define CAU_STR_CA3                              CAU_STR_CA_REG(CAU,3)
#define CAU_STR_CA4                              CAU_STR_CA_REG(CAU,4)
#define CAU_STR_CA5                              CAU_STR_CA_REG(CAU,5)
#define CAU_STR_CA6                              CAU_STR_CA_REG(CAU,6)
#define CAU_STR_CA7                              CAU_STR_CA_REG(CAU,7)
#define CAU_STR_CA8                              CAU_STR_CA_REG(CAU,8)
#define CAU_ADR_CASR                             CAU_ADR_CASR_REG(CAU)
#define CAU_ADR_CAA                              CAU_ADR_CAA_REG(CAU)
#define CAU_ADR_CA0                              CAU_ADR_CA_REG(CAU,0)
#define CAU_ADR_CA1                              CAU_ADR_CA_REG(CAU,1)
#define CAU_ADR_CA2                              CAU_ADR_CA_REG(CAU,2)
#define CAU_ADR_CA3                              CAU_ADR_CA_REG(CAU,3)
#define CAU_ADR_CA4                              CAU_ADR_CA_REG(CAU,4)
#define CAU_ADR_CA5                              CAU_ADR_CA_REG(CAU,5)
#define CAU_ADR_CA6                              CAU_ADR_CA_REG(CAU,6)
#define CAU_ADR_CA7                              CAU_ADR_CA_REG(CAU,7)
#define CAU_ADR_CA8                              CAU_ADR_CA_REG(CAU,8)
#define CAU_RADR_CASR                            CAU_RADR_CASR_REG(CAU)
#define CAU_RADR_CAA                             CAU_RADR_CAA_REG(CAU)
#define CAU_RADR_CA0                             CAU_RADR_CA_REG(CAU,0)
#define CAU_RADR_CA1                             CAU_RADR_CA_REG(CAU,1)
#define CAU_RADR_CA2                             CAU_RADR_CA_REG(CAU,2)
#define CAU_RADR_CA3                             CAU_RADR_CA_REG(CAU,3)
#define CAU_RADR_CA4                             CAU_RADR_CA_REG(CAU,4)
#define CAU_RADR_CA5                             CAU_RADR_CA_REG(CAU,5)
#define CAU_RADR_CA6                             CAU_RADR_CA_REG(CAU,6)
#define CAU_RADR_CA7                             CAU_RADR_CA_REG(CAU,7)
#define CAU_RADR_CA8                             CAU_RADR_CA_REG(CAU,8)
#define CAU_XOR_CASR                             CAU_XOR_CASR_REG(CAU)
#define CAU_XOR_CAA                              CAU_XOR_CAA_REG(CAU)
#define CAU_XOR_CA0                              CAU_XOR_CA_REG(CAU,0)
#define CAU_XOR_CA1                              CAU_XOR_CA_REG(CAU,1)
#define CAU_XOR_CA2                              CAU_XOR_CA_REG(CAU,2)
#define CAU_XOR_CA3                              CAU_XOR_CA_REG(CAU,3)
#define CAU_XOR_CA4                              CAU_XOR_CA_REG(CAU,4)
#define CAU_XOR_CA5                              CAU_XOR_CA_REG(CAU,5)
#define CAU_XOR_CA6                              CAU_XOR_CA_REG(CAU,6)
#define CAU_XOR_CA7                              CAU_XOR_CA_REG(CAU,7)
#define CAU_XOR_CA8                              CAU_XOR_CA_REG(CAU,8)
#define CAU_ROTL_CASR                            CAU_ROTL_CASR_REG(CAU)
#define CAU_ROTL_CAA                             CAU_ROTL_CAA_REG(CAU)
#define CAU_ROTL_CA0                             CAU_ROTL_CA_REG(CAU,0)
#define CAU_ROTL_CA1                             CAU_ROTL_CA_REG(CAU,1)
#define CAU_ROTL_CA2                             CAU_ROTL_CA_REG(CAU,2)
#define CAU_ROTL_CA3                             CAU_ROTL_CA_REG(CAU,3)
#define CAU_ROTL_CA4                             CAU_ROTL_CA_REG(CAU,4)
#define CAU_ROTL_CA5                             CAU_ROTL_CA_REG(CAU,5)
#define CAU_ROTL_CA6                             CAU_ROTL_CA_REG(CAU,6)
#define CAU_ROTL_CA7                             CAU_ROTL_CA_REG(CAU,7)
#define CAU_ROTL_CA8                             CAU_ROTL_CA_REG(CAU,8)
#define CAU_AESC_CASR                            CAU_AESC_CASR_REG(CAU)
#define CAU_AESC_CAA                             CAU_AESC_CAA_REG(CAU)
#define CAU_AESC_CA0                             CAU_AESC_CA_REG(CAU,0)
#define CAU_AESC_CA1                             CAU_AESC_CA_REG(CAU,1)
#define CAU_AESC_CA2                             CAU_AESC_CA_REG(CAU,2)
#define CAU_AESC_CA3                             CAU_AESC_CA_REG(CAU,3)
#define CAU_AESC_CA4                             CAU_AESC_CA_REG(CAU,4)
#define CAU_AESC_CA5                             CAU_AESC_CA_REG(CAU,5)
#define CAU_AESC_CA6                             CAU_AESC_CA_REG(CAU,6)
#define CAU_AESC_CA7                             CAU_AESC_CA_REG(CAU,7)
#define CAU_AESC_CA8                             CAU_AESC_CA_REG(CAU,8)
#define CAU_AESIC_CASR                           CAU_AESIC_CASR_REG(CAU)
#define CAU_AESIC_CAA                            CAU_AESIC_CAA_REG(CAU)
#define CAU_AESIC_CA0                            CAU_AESIC_CA_REG(CAU,0)
#define CAU_AESIC_CA1                            CAU_AESIC_CA_REG(CAU,1)
#define CAU_AESIC_CA2                            CAU_AESIC_CA_REG(CAU,2)
#define CAU_AESIC_CA3                            CAU_AESIC_CA_REG(CAU,3)
#define CAU_AESIC_CA4                            CAU_AESIC_CA_REG(CAU,4)
#define CAU_AESIC_CA5                            CAU_AESIC_CA_REG(CAU,5)
#define CAU_AESIC_CA6                            CAU_AESIC_CA_REG(CAU,6)
#define CAU_AESIC_CA7                            CAU_AESIC_CA_REG(CAU,7)
#define CAU_AESIC_CA8                            CAU_AESIC_CA_REG(CAU,8)

/* CAU - Register array accessors */
#define CAU_DIRECT(index)                        CAU_DIRECT_REG(CAU,index)
#define CAU_LDR_CA(index)                        CAU_LDR_CA_REG(CAU,index)
#define CAU_STR_CA(index)                        CAU_STR_CA_REG(CAU,index)
#define CAU_ADR_CA(index)                        CAU_ADR_CA_REG(CAU,index)
#define CAU_RADR_CA(index)                       CAU_RADR_CA_REG(CAU,index)
#define CAU_XOR_CA(index)                        CAU_XOR_CA_REG(CAU,index)
#define CAU_ROTL_CA(index)                       CAU_ROTL_CA_REG(CAU,index)
#define CAU_AESC_CA(index)                       CAU_AESC_CA_REG(CAU,index)
#define CAU_AESIC_CA(index)                      CAU_AESIC_CA_REG(CAU,index)


/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register accessors */
#define CMP_CR0_REG(base)                        ((base)->CR0)
#define CMP_CR1_REG(base)                        ((base)->CR1)
#define CMP_FPR_REG(base)                        ((base)->FPR)
#define CMP_SCR_REG(base)                        ((base)->SCR)
#define CMP_DACCR_REG(base)                      ((base)->DACCR)
#define CMP_MUXCR_REG(base)                      ((base)->MUXCR)

/* CMP - Peripheral instance base addresses */
#define CMP0_BASE_PTR                            (CMP0)
#define CMP1_BASE_PTR                            (CMP1)
#define CMP2_BASE_PTR                            (CMP2)


/* CMP - Register instance definitions */
/* CMP0 */
#define CMP0_CR0                                 CMP_CR0_REG(CMP0)
#define CMP0_CR1                                 CMP_CR1_REG(CMP0)
#define CMP0_FPR                                 CMP_FPR_REG(CMP0)
#define CMP0_SCR                                 CMP_SCR_REG(CMP0)
#define CMP0_DACCR                               CMP_DACCR_REG(CMP0)
#define CMP0_MUXCR                               CMP_MUXCR_REG(CMP0)
/* CMP1 */
#define CMP1_CR0                                 CMP_CR0_REG(CMP1)
#define CMP1_CR1                                 CMP_CR1_REG(CMP1)
#define CMP1_FPR                                 CMP_FPR_REG(CMP1)
#define CMP1_SCR                                 CMP_SCR_REG(CMP1)
#define CMP1_DACCR                               CMP_DACCR_REG(CMP1)
#define CMP1_MUXCR                               CMP_MUXCR_REG(CMP1)
/* CMP2 */
#define CMP2_CR0                                 CMP_CR0_REG(CMP2)
#define CMP2_CR1                                 CMP_CR1_REG(CMP2)
#define CMP2_FPR                                 CMP_FPR_REG(CMP2)
#define CMP2_SCR                                 CMP_SCR_REG(CMP2)
#define CMP2_DACCR                               CMP_DACCR_REG(CMP2)
#define CMP2_MUXCR                               CMP_MUXCR_REG(CMP2)



/* ----------------------------------------------------------------------------
   -- CMT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMT_Register_Accessor_Macros CMT - Register accessor macros
 * @{
 */


/* CMT - Register accessors */
#define CMT_CGH1_REG(base)                       ((base)->CGH1)
#define CMT_CGL1_REG(base)                       ((base)->CGL1)
#define CMT_CGH2_REG(base)                       ((base)->CGH2)
#define CMT_CGL2_REG(base)                       ((base)->CGL2)
#define CMT_OC_REG(base)                         ((base)->OC)
#define CMT_MSC_REG(base)                        ((base)->MSC)
#define CMT_CMD1_REG(base)                       ((base)->CMD1)
#define CMT_CMD2_REG(base)                       ((base)->CMD2)
#define CMT_CMD3_REG(base)                       ((base)->CMD3)
#define CMT_CMD4_REG(base)                       ((base)->CMD4)
#define CMT_PPS_REG(base)                        ((base)->PPS)
#define CMT_DMA_REG(base)                        ((base)->DMA)

/* CMT - Peripheral instance base addresses */
#define CMT_BASE_PTR                             (CMT)


/* CMT - Register instance definitions */
/* CMT */
#define CMT_CGH1                                 CMT_CGH1_REG(CMT)
#define CMT_CGL1                                 CMT_CGL1_REG(CMT)
#define CMT_CGH2                                 CMT_CGH2_REG(CMT)
#define CMT_CGL2                                 CMT_CGL2_REG(CMT)
#define CMT_OC                                   CMT_OC_REG(CMT)
#define CMT_MSC                                  CMT_MSC_REG(CMT)
#define CMT_CMD1                                 CMT_CMD1_REG(CMT)
#define CMT_CMD2                                 CMT_CMD2_REG(CMT)
#define CMT_CMD3                                 CMT_CMD3_REG(CMT)
#define CMT_CMD4                                 CMT_CMD4_REG(CMT)
#define CMT_PPS                                  CMT_PPS_REG(CMT)
#define CMT_DMA                                  CMT_DMA_REG(CMT)



/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register accessors */
#define CRC_DATAL_REG(base)                      ((base)->ACCESS16BIT.DATAL)
#define CRC_DATAH_REG(base)                      ((base)->ACCESS16BIT.DATAH)
#define CRC_DATA_REG(base)                       ((base)->DATA)
#define CRC_DATALL_REG(base)                     ((base)->ACCESS8BIT.DATALL)
#define CRC_DATALU_REG(base)                     ((base)->ACCESS8BIT.DATALU)
#define CRC_DATAHL_REG(base)                     ((base)->ACCESS8BIT.DATAHL)
#define CRC_DATAHU_REG(base)                     ((base)->ACCESS8BIT.DATAHU)
#define CRC_GPOLYL_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYL)
#define CRC_GPOLYH_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYH)
#define CRC_GPOLY_REG(base)                      ((base)->GPOLY)
#define CRC_GPOLYLL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLL)
#define CRC_GPOLYLU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLU)
#define CRC_GPOLYHL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHL)
#define CRC_GPOLYHU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHU)
#define CRC_CTRL_REG(base)                       ((base)->CTRL)
#define CRC_CTRLHU_REG(base)                     ((base)->CTRL_ACCESS8BIT.CTRLHU)

/* CRC - Peripheral instance base addresses */
#define CRC_BASE_PTR                             (CRC0)


/* CRC - Register instance definitions */
/* CRC */
#define CRC_DATA                                 CRC_DATA_REG(CRC0)
#define CRC_DATAL                                CRC_DATAL_REG(CRC0)
#define CRC_DATALL                               CRC_DATALL_REG(CRC0)
#define CRC_DATALU                               CRC_DATALU_REG(CRC0)
#define CRC_DATAH                                CRC_DATAH_REG(CRC0)
#define CRC_DATAHL                               CRC_DATAHL_REG(CRC0)
#define CRC_DATAHU                               CRC_DATAHU_REG(CRC0)
#define CRC_GPOLY                                CRC_GPOLY_REG(CRC0)
#define CRC_GPOLYL                               CRC_GPOLYL_REG(CRC0)
#define CRC_GPOLYLL                              CRC_GPOLYLL_REG(CRC0)
#define CRC_GPOLYLU                              CRC_GPOLYLU_REG(CRC0)
#define CRC_GPOLYH                               CRC_GPOLYH_REG(CRC0)
#define CRC_GPOLYHL                              CRC_GPOLYHL_REG(CRC0)
#define CRC_GPOLYHU                              CRC_GPOLYHU_REG(CRC0)
#define CRC_CTRL                                 CRC_CTRL_REG(CRC0)
#define CRC_CTRLHU                               CRC_CTRLHU_REG(CRC0)

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register accessors */
#define DAC_DATL_REG(base,index)                 ((base)->DAT[index].DATL)
#define DAC_DATH_REG(base,index)                 ((base)->DAT[index].DATH)
#define DAC_SR_REG(base)                         ((base)->SR)
#define DAC_C0_REG(base)                         ((base)->C0)
#define DAC_C1_REG(base)                         ((base)->C1)
#define DAC_C2_REG(base)                         ((base)->C2)

#define DAC0_BASE_PTR                            (DAC0)
#define DAC1_BASE_PTR                            (DAC1)


/* DAC - Register instance definitions */
/* DAC0 */
#define DAC0_DAT0L                               DAC_DATL_REG(DAC0,0)
#define DAC0_DAT0H                               DAC_DATH_REG(DAC0,0)
#define DAC0_DAT1L                               DAC_DATL_REG(DAC0,1)
#define DAC0_DAT1H                               DAC_DATH_REG(DAC0,1)
#define DAC0_DAT2L                               DAC_DATL_REG(DAC0,2)
#define DAC0_DAT2H                               DAC_DATH_REG(DAC0,2)
#define DAC0_DAT3L                               DAC_DATL_REG(DAC0,3)
#define DAC0_DAT3H                               DAC_DATH_REG(DAC0,3)
#define DAC0_DAT4L                               DAC_DATL_REG(DAC0,4)
#define DAC0_DAT4H                               DAC_DATH_REG(DAC0,4)
#define DAC0_DAT5L                               DAC_DATL_REG(DAC0,5)
#define DAC0_DAT5H                               DAC_DATH_REG(DAC0,5)
#define DAC0_DAT6L                               DAC_DATL_REG(DAC0,6)
#define DAC0_DAT6H                               DAC_DATH_REG(DAC0,6)
#define DAC0_DAT7L                               DAC_DATL_REG(DAC0,7)
#define DAC0_DAT7H                               DAC_DATH_REG(DAC0,7)
#define DAC0_DAT8L                               DAC_DATL_REG(DAC0,8)
#define DAC0_DAT8H                               DAC_DATH_REG(DAC0,8)
#define DAC0_DAT9L                               DAC_DATL_REG(DAC0,9)
#define DAC0_DAT9H                               DAC_DATH_REG(DAC0,9)
#define DAC0_DAT10L                              DAC_DATL_REG(DAC0,10)
#define DAC0_DAT10H                              DAC_DATH_REG(DAC0,10)
#define DAC0_DAT11L                              DAC_DATL_REG(DAC0,11)
#define DAC0_DAT11H                              DAC_DATH_REG(DAC0,11)
#define DAC0_DAT12L                              DAC_DATL_REG(DAC0,12)
#define DAC0_DAT12H                              DAC_DATH_REG(DAC0,12)
#define DAC0_DAT13L                              DAC_DATL_REG(DAC0,13)
#define DAC0_DAT13H                              DAC_DATH_REG(DAC0,13)
#define DAC0_DAT14L                              DAC_DATL_REG(DAC0,14)
#define DAC0_DAT14H                              DAC_DATH_REG(DAC0,14)
#define DAC0_DAT15L                              DAC_DATL_REG(DAC0,15)
#define DAC0_DAT15H                              DAC_DATH_REG(DAC0,15)
#define DAC0_SR                                  DAC_SR_REG(DAC0)
#define DAC0_C0                                  DAC_C0_REG(DAC0)
#define DAC0_C1                                  DAC_C1_REG(DAC0)
#define DAC0_C2                                  DAC_C2_REG(DAC0)
/* DAC1 */
#define DAC1_DAT0L                               DAC_DATL_REG(DAC1,0)
#define DAC1_DAT0H                               DAC_DATH_REG(DAC1,0)
#define DAC1_DAT1L                               DAC_DATL_REG(DAC1,1)
#define DAC1_DAT1H                               DAC_DATH_REG(DAC1,1)
#define DAC1_DAT2L                               DAC_DATL_REG(DAC1,2)
#define DAC1_DAT2H                               DAC_DATH_REG(DAC1,2)
#define DAC1_DAT3L                               DAC_DATL_REG(DAC1,3)
#define DAC1_DAT3H                               DAC_DATH_REG(DAC1,3)
#define DAC1_DAT4L                               DAC_DATL_REG(DAC1,4)
#define DAC1_DAT4H                               DAC_DATH_REG(DAC1,4)
#define DAC1_DAT5L                               DAC_DATL_REG(DAC1,5)
#define DAC1_DAT5H                               DAC_DATH_REG(DAC1,5)
#define DAC1_DAT6L                               DAC_DATL_REG(DAC1,6)
#define DAC1_DAT6H                               DAC_DATH_REG(DAC1,6)
#define DAC1_DAT7L                               DAC_DATL_REG(DAC1,7)
#define DAC1_DAT7H                               DAC_DATH_REG(DAC1,7)
#define DAC1_DAT8L                               DAC_DATL_REG(DAC1,8)
#define DAC1_DAT8H                               DAC_DATH_REG(DAC1,8)
#define DAC1_DAT9L                               DAC_DATL_REG(DAC1,9)
#define DAC1_DAT9H                               DAC_DATH_REG(DAC1,9)
#define DAC1_DAT10L                              DAC_DATL_REG(DAC1,10)
#define DAC1_DAT10H                              DAC_DATH_REG(DAC1,10)
#define DAC1_DAT11L                              DAC_DATL_REG(DAC1,11)
#define DAC1_DAT11H                              DAC_DATH_REG(DAC1,11)
#define DAC1_DAT12L                              DAC_DATL_REG(DAC1,12)
#define DAC1_DAT12H                              DAC_DATH_REG(DAC1,12)
#define DAC1_DAT13L                              DAC_DATL_REG(DAC1,13)
#define DAC1_DAT13H                              DAC_DATH_REG(DAC1,13)
#define DAC1_DAT14L                              DAC_DATL_REG(DAC1,14)
#define DAC1_DAT14H                              DAC_DATH_REG(DAC1,14)
#define DAC1_DAT15L                              DAC_DATL_REG(DAC1,15)
#define DAC1_DAT15H                              DAC_DATH_REG(DAC1,15)
#define DAC1_SR                                  DAC_SR_REG(DAC1)
#define DAC1_C0                                  DAC_C0_REG(DAC1)
#define DAC1_C1                                  DAC_C1_REG(DAC1)
#define DAC1_C2                                  DAC_C2_REG(DAC1)

/* DAC - Register array accessors */
#define DAC0_DATL(index)                         DAC_DATL_REG(DAC0,index)
#define DAC1_DATL(index)                         DAC_DATL_REG(DAC1,index)
#define DAC0_DATH(index)                         DAC_DATH_REG(DAC0,index)
#define DAC1_DATH(index)                         DAC_DATH_REG(DAC1,index)



/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register accessors */
#define DMA_CR_REG(base)                         ((base)->CR)
#define DMA_ES_REG(base)                         ((base)->ES)
#define DMA_ERQ_REG(base)                        ((base)->ERQ)
#define DMA_EEI_REG(base)                        ((base)->EEI)
#define DMA_CEEI_REG(base)                       ((base)->CEEI)
#define DMA_SEEI_REG(base)                       ((base)->SEEI)
#define DMA_CERQ_REG(base)                       ((base)->CERQ)
#define DMA_SERQ_REG(base)                       ((base)->SERQ)
#define DMA_CDNE_REG(base)                       ((base)->CDNE)
#define DMA_SSRT_REG(base)                       ((base)->SSRT)
#define DMA_CERR_REG(base)                       ((base)->CERR)
#define DMA_CINT_REG(base)                       ((base)->CINT)
#define DMA_INT_REG(base)                        ((base)->INT)
#define DMA_ERR_REG(base)                        ((base)->ERR)
#define DMA_HRS_REG(base)                        ((base)->HRS)
#define DMA_DCHPRI3_REG(base)                    ((base)->DCHPRI3)
#define DMA_DCHPRI2_REG(base)                    ((base)->DCHPRI2)
#define DMA_DCHPRI1_REG(base)                    ((base)->DCHPRI1)
#define DMA_DCHPRI0_REG(base)                    ((base)->DCHPRI0)
#define DMA_DCHPRI7_REG(base)                    ((base)->DCHPRI7)
#define DMA_DCHPRI6_REG(base)                    ((base)->DCHPRI6)
#define DMA_DCHPRI5_REG(base)                    ((base)->DCHPRI5)
#define DMA_DCHPRI4_REG(base)                    ((base)->DCHPRI4)
#define DMA_DCHPRI11_REG(base)                   ((base)->DCHPRI11)
#define DMA_DCHPRI10_REG(base)                   ((base)->DCHPRI10)
#define DMA_DCHPRI9_REG(base)                    ((base)->DCHPRI9)
#define DMA_DCHPRI8_REG(base)                    ((base)->DCHPRI8)
#define DMA_DCHPRI15_REG(base)                   ((base)->DCHPRI15)
#define DMA_DCHPRI14_REG(base)                   ((base)->DCHPRI14)
#define DMA_DCHPRI13_REG(base)                   ((base)->DCHPRI13)
#define DMA_DCHPRI12_REG(base)                   ((base)->DCHPRI12)
#define DMA_SADDR_REG(base,index)                ((base)->TCD[index].SADDR)
#define DMA_SOFF_REG(base,index)                 ((base)->TCD[index].SOFF)
#define DMA_ATTR_REG(base,index)                 ((base)->TCD[index].ATTR)
#define DMA_NBYTES_MLNO_REG(base,index)          ((base)->TCD[index].NBYTES_MLNO)
#define DMA_NBYTES_MLOFFNO_REG(base,index)       ((base)->TCD[index].NBYTES_MLOFFNO)
#define DMA_NBYTES_MLOFFYES_REG(base,index)      ((base)->TCD[index].NBYTES_MLOFFYES)
#define DMA_SLAST_REG(base,index)                ((base)->TCD[index].SLAST)
#define DMA_DADDR_REG(base,index)                ((base)->TCD[index].DADDR)
#define DMA_DOFF_REG(base,index)                 ((base)->TCD[index].DOFF)
#define DMA_CITER_ELINKNO_REG(base,index)        ((base)->TCD[index].CITER_ELINKNO)
#define DMA_CITER_ELINKYES_REG(base,index)       ((base)->TCD[index].CITER_ELINKYES)
#define DMA_DLAST_SGA_REG(base,index)            ((base)->TCD[index].DLAST_SGA)
#define DMA_CSR_REG(base,index)                  ((base)->TCD[index].CSR)
#define DMA_BITER_ELINKNO_REG(base,index)        ((base)->TCD[index].BITER_ELINKNO)
#define DMA_BITER_ELINKYES_REG(base,index)       ((base)->TCD[index].BITER_ELINKYES)


/* DMA - Peripheral instance base addresses */
#define DMA_BASE_PTR                             (DMA0)


/* DMA - Register instance definitions */
/* DMA */
#define DMA_CR                                   DMA_CR_REG(DMA0)
#define DMA_ES                                   DMA_ES_REG(DMA0)
#define DMA_ERQ                                  DMA_ERQ_REG(DMA0)
#define DMA_EEI                                  DMA_EEI_REG(DMA0)
#define DMA_CEEI                                 DMA_CEEI_REG(DMA0)
#define DMA_SEEI                                 DMA_SEEI_REG(DMA0)
#define DMA_CERQ                                 DMA_CERQ_REG(DMA0)
#define DMA_SERQ                                 DMA_SERQ_REG(DMA0)
#define DMA_CDNE                                 DMA_CDNE_REG(DMA0)
#define DMA_SSRT                                 DMA_SSRT_REG(DMA0)
#define DMA_CERR                                 DMA_CERR_REG(DMA0)
#define DMA_CINT                                 DMA_CINT_REG(DMA0)
#define DMA_INT                                  DMA_INT_REG(DMA0)
#define DMA_ERR                                  DMA_ERR_REG(DMA0)
#define DMA_HRS                                  DMA_HRS_REG(DMA0)
#define DMA_DCHPRI3                              DMA_DCHPRI3_REG(DMA0)
#define DMA_DCHPRI2                              DMA_DCHPRI2_REG(DMA0)
#define DMA_DCHPRI1                              DMA_DCHPRI1_REG(DMA0)
#define DMA_DCHPRI0                              DMA_DCHPRI0_REG(DMA0)
#define DMA_DCHPRI7                              DMA_DCHPRI7_REG(DMA0)
#define DMA_DCHPRI6                              DMA_DCHPRI6_REG(DMA0)
#define DMA_DCHPRI5                              DMA_DCHPRI5_REG(DMA0)
#define DMA_DCHPRI4                              DMA_DCHPRI4_REG(DMA0)
#define DMA_DCHPRI11                             DMA_DCHPRI11_REG(DMA0)
#define DMA_DCHPRI10                             DMA_DCHPRI10_REG(DMA0)
#define DMA_DCHPRI9                              DMA_DCHPRI9_REG(DMA0)
#define DMA_DCHPRI8                              DMA_DCHPRI8_REG(DMA0)
#define DMA_DCHPRI15                             DMA_DCHPRI15_REG(DMA0)
#define DMA_DCHPRI14                             DMA_DCHPRI14_REG(DMA0)
#define DMA_DCHPRI13                             DMA_DCHPRI13_REG(DMA0)
#define DMA_DCHPRI12                             DMA_DCHPRI12_REG(DMA0)
#define DMA_TCD0_SADDR                           DMA_SADDR_REG(DMA0,0)
#define DMA_TCD0_SOFF                            DMA_SOFF_REG(DMA0,0)
#define DMA_TCD0_ATTR                            DMA_ATTR_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,0)
#define DMA_TCD0_SLAST                           DMA_SLAST_REG(DMA0,0)
#define DMA_TCD0_DADDR                           DMA_DADDR_REG(DMA0,0)
#define DMA_TCD0_DOFF                            DMA_DOFF_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD0_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,0)
#define DMA_TCD0_CSR                             DMA_CSR_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD1_SADDR                           DMA_SADDR_REG(DMA0,1)
#define DMA_TCD1_SOFF                            DMA_SOFF_REG(DMA0,1)
#define DMA_TCD1_ATTR                            DMA_ATTR_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,1)
#define DMA_TCD1_SLAST                           DMA_SLAST_REG(DMA0,1)
#define DMA_TCD1_DADDR                           DMA_DADDR_REG(DMA0,1)
#define DMA_TCD1_DOFF                            DMA_DOFF_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD1_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,1)
#define DMA_TCD1_CSR                             DMA_CSR_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD2_SADDR                           DMA_SADDR_REG(DMA0,2)
#define DMA_TCD2_SOFF                            DMA_SOFF_REG(DMA0,2)
#define DMA_TCD2_ATTR                            DMA_ATTR_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,2)
#define DMA_TCD2_SLAST                           DMA_SLAST_REG(DMA0,2)
#define DMA_TCD2_DADDR                           DMA_DADDR_REG(DMA0,2)
#define DMA_TCD2_DOFF                            DMA_DOFF_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD2_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,2)
#define DMA_TCD2_CSR                             DMA_CSR_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD3_SADDR                           DMA_SADDR_REG(DMA0,3)
#define DMA_TCD3_SOFF                            DMA_SOFF_REG(DMA0,3)
#define DMA_TCD3_ATTR                            DMA_ATTR_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,3)
#define DMA_TCD3_SLAST                           DMA_SLAST_REG(DMA0,3)
#define DMA_TCD3_DADDR                           DMA_DADDR_REG(DMA0,3)
#define DMA_TCD3_DOFF                            DMA_DOFF_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD3_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,3)
#define DMA_TCD3_CSR                             DMA_CSR_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD4_SADDR                           DMA_SADDR_REG(DMA0,4)
#define DMA_TCD4_SOFF                            DMA_SOFF_REG(DMA0,4)
#define DMA_TCD4_ATTR                            DMA_ATTR_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,4)
#define DMA_TCD4_SLAST                           DMA_SLAST_REG(DMA0,4)
#define DMA_TCD4_DADDR                           DMA_DADDR_REG(DMA0,4)
#define DMA_TCD4_DOFF                            DMA_DOFF_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD4_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,4)
#define DMA_TCD4_CSR                             DMA_CSR_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD5_SADDR                           DMA_SADDR_REG(DMA0,5)
#define DMA_TCD5_SOFF                            DMA_SOFF_REG(DMA0,5)
#define DMA_TCD5_ATTR                            DMA_ATTR_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,5)
#define DMA_TCD5_SLAST                           DMA_SLAST_REG(DMA0,5)
#define DMA_TCD5_DADDR                           DMA_DADDR_REG(DMA0,5)
#define DMA_TCD5_DOFF                            DMA_DOFF_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD5_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,5)
#define DMA_TCD5_CSR                             DMA_CSR_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD6_SADDR                           DMA_SADDR_REG(DMA0,6)
#define DMA_TCD6_SOFF                            DMA_SOFF_REG(DMA0,6)
#define DMA_TCD6_ATTR                            DMA_ATTR_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,6)
#define DMA_TCD6_SLAST                           DMA_SLAST_REG(DMA0,6)
#define DMA_TCD6_DADDR                           DMA_DADDR_REG(DMA0,6)
#define DMA_TCD6_DOFF                            DMA_DOFF_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD6_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,6)
#define DMA_TCD6_CSR                             DMA_CSR_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD7_SADDR                           DMA_SADDR_REG(DMA0,7)
#define DMA_TCD7_SOFF                            DMA_SOFF_REG(DMA0,7)
#define DMA_TCD7_ATTR                            DMA_ATTR_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,7)
#define DMA_TCD7_SLAST                           DMA_SLAST_REG(DMA0,7)
#define DMA_TCD7_DADDR                           DMA_DADDR_REG(DMA0,7)
#define DMA_TCD7_DOFF                            DMA_DOFF_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD7_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,7)
#define DMA_TCD7_CSR                             DMA_CSR_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD8_SADDR                           DMA_SADDR_REG(DMA0,8)
#define DMA_TCD8_SOFF                            DMA_SOFF_REG(DMA0,8)
#define DMA_TCD8_ATTR                            DMA_ATTR_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,8)
#define DMA_TCD8_SLAST                           DMA_SLAST_REG(DMA0,8)
#define DMA_TCD8_DADDR                           DMA_DADDR_REG(DMA0,8)
#define DMA_TCD8_DOFF                            DMA_DOFF_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD8_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,8)
#define DMA_TCD8_CSR                             DMA_CSR_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD9_SADDR                           DMA_SADDR_REG(DMA0,9)
#define DMA_TCD9_SOFF                            DMA_SOFF_REG(DMA0,9)
#define DMA_TCD9_ATTR                            DMA_ATTR_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,9)
#define DMA_TCD9_SLAST                           DMA_SLAST_REG(DMA0,9)
#define DMA_TCD9_DADDR                           DMA_DADDR_REG(DMA0,9)
#define DMA_TCD9_DOFF                            DMA_DOFF_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD9_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,9)
#define DMA_TCD9_CSR                             DMA_CSR_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD10_SADDR                          DMA_SADDR_REG(DMA0,10)
#define DMA_TCD10_SOFF                           DMA_SOFF_REG(DMA0,10)
#define DMA_TCD10_ATTR                           DMA_ATTR_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,10)
#define DMA_TCD10_SLAST                          DMA_SLAST_REG(DMA0,10)
#define DMA_TCD10_DADDR                          DMA_DADDR_REG(DMA0,10)
#define DMA_TCD10_DOFF                           DMA_DOFF_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD10_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,10)
#define DMA_TCD10_CSR                            DMA_CSR_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD11_SADDR                          DMA_SADDR_REG(DMA0,11)
#define DMA_TCD11_SOFF                           DMA_SOFF_REG(DMA0,11)
#define DMA_TCD11_ATTR                           DMA_ATTR_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,11)
#define DMA_TCD11_SLAST                          DMA_SLAST_REG(DMA0,11)
#define DMA_TCD11_DADDR                          DMA_DADDR_REG(DMA0,11)
#define DMA_TCD11_DOFF                           DMA_DOFF_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD11_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,11)
#define DMA_TCD11_CSR                            DMA_CSR_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD12_SADDR                          DMA_SADDR_REG(DMA0,12)
#define DMA_TCD12_SOFF                           DMA_SOFF_REG(DMA0,12)
#define DMA_TCD12_ATTR                           DMA_ATTR_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,12)
#define DMA_TCD12_SLAST                          DMA_SLAST_REG(DMA0,12)
#define DMA_TCD12_DADDR                          DMA_DADDR_REG(DMA0,12)
#define DMA_TCD12_DOFF                           DMA_DOFF_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD12_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,12)
#define DMA_TCD12_CSR                            DMA_CSR_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD13_SADDR                          DMA_SADDR_REG(DMA0,13)
#define DMA_TCD13_SOFF                           DMA_SOFF_REG(DMA0,13)
#define DMA_TCD13_ATTR                           DMA_ATTR_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,13)
#define DMA_TCD13_SLAST                          DMA_SLAST_REG(DMA0,13)
#define DMA_TCD13_DADDR                          DMA_DADDR_REG(DMA0,13)
#define DMA_TCD13_DOFF                           DMA_DOFF_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD13_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,13)
#define DMA_TCD13_CSR                            DMA_CSR_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD14_SADDR                          DMA_SADDR_REG(DMA0,14)
#define DMA_TCD14_SOFF                           DMA_SOFF_REG(DMA0,14)
#define DMA_TCD14_ATTR                           DMA_ATTR_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,14)
#define DMA_TCD14_SLAST                          DMA_SLAST_REG(DMA0,14)
#define DMA_TCD14_DADDR                          DMA_DADDR_REG(DMA0,14)
#define DMA_TCD14_DOFF                           DMA_DOFF_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD14_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,14)
#define DMA_TCD14_CSR                            DMA_CSR_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD15_SADDR                          DMA_SADDR_REG(DMA0,15)
#define DMA_TCD15_SOFF                           DMA_SOFF_REG(DMA0,15)
#define DMA_TCD15_ATTR                           DMA_ATTR_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,15)
#define DMA_TCD15_SLAST                          DMA_SLAST_REG(DMA0,15)
#define DMA_TCD15_DADDR                          DMA_DADDR_REG(DMA0,15)
#define DMA_TCD15_DOFF                           DMA_DOFF_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,15)
#define DMA_TCD15_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,15)
#define DMA_TCD15_CSR                            DMA_CSR_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,15)

/* DMA - Register array accessors */
#define DMA_SADDR(index)                         DMA_SADDR_REG(DMA0,index)
#define DMA_SOFF(index)                          DMA_SOFF_REG(DMA0,index)
#define DMA_ATTR(index)                          DMA_ATTR_REG(DMA0,index)
#define DMA_NBYTES_MLNO(index)                   DMA_NBYTES_MLNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFNO(index)                DMA_NBYTES_MLOFFNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFYES(index)               DMA_NBYTES_MLOFFYES_REG(DMA0,index)
#define DMA_SLAST(index)                         DMA_SLAST_REG(DMA0,index)
#define DMA_DADDR(index)                         DMA_DADDR_REG(DMA0,index)
#define DMA_DOFF(index)                          DMA_DOFF_REG(DMA0,index)
#define DMA_CITER_ELINKNO(index)                 DMA_CITER_ELINKNO_REG(DMA0,index)
#define DMA_CITER_ELINKYES(index)                DMA_CITER_ELINKYES_REG(DMA0,index)
#define DMA_DLAST_SGA(index)                     DMA_DLAST_SGA_REG(DMA0,index)
#define DMA_CSR(index)                           DMA_CSR_REG(DMA0,index)
#define DMA_BITER_ELINKNO(index)                 DMA_BITER_ELINKNO_REG(DMA0,index)
#define DMA_BITER_ELINKYES(index)                DMA_BITER_ELINKYES_REG(DMA0,index)



/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register accessors */
#define DMAMUX_CHCFG_REG(base,index)             ((base)->CHCFG[index])


/* DMAMUX - Peripheral instance base addresses */
#define DMAMUX_BASE_PTR                          (DMAMUX)


/* DMAMUX - Register instance definitions */
/* DMAMUX */
#define DMAMUX_CHCFG0                            DMAMUX_CHCFG_REG(DMAMUX,0)
#define DMAMUX_CHCFG1                            DMAMUX_CHCFG_REG(DMAMUX,1)
#define DMAMUX_CHCFG2                            DMAMUX_CHCFG_REG(DMAMUX,2)
#define DMAMUX_CHCFG3                            DMAMUX_CHCFG_REG(DMAMUX,3)
#define DMAMUX_CHCFG4                            DMAMUX_CHCFG_REG(DMAMUX,4)
#define DMAMUX_CHCFG5                            DMAMUX_CHCFG_REG(DMAMUX,5)
#define DMAMUX_CHCFG6                            DMAMUX_CHCFG_REG(DMAMUX,6)
#define DMAMUX_CHCFG7                            DMAMUX_CHCFG_REG(DMAMUX,7)
#define DMAMUX_CHCFG8                            DMAMUX_CHCFG_REG(DMAMUX,8)
#define DMAMUX_CHCFG9                            DMAMUX_CHCFG_REG(DMAMUX,9)
#define DMAMUX_CHCFG10                           DMAMUX_CHCFG_REG(DMAMUX,10)
#define DMAMUX_CHCFG11                           DMAMUX_CHCFG_REG(DMAMUX,11)
#define DMAMUX_CHCFG12                           DMAMUX_CHCFG_REG(DMAMUX,12)
#define DMAMUX_CHCFG13                           DMAMUX_CHCFG_REG(DMAMUX,13)
#define DMAMUX_CHCFG14                           DMAMUX_CHCFG_REG(DMAMUX,14)
#define DMAMUX_CHCFG15                           DMAMUX_CHCFG_REG(DMAMUX,15)

/* DMAMUX - Register array accessors */
#define DMAMUX_CHCFG(index)                      DMAMUX_CHCFG_REG(DMAMUX,index)

/* ----------------------------------------------------------------------------
   -- ENET - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ENET_Register_Accessor_Macros ENET - Register accessor macros
 * @{
 */


/* ENET - Register accessors */
#define ENET_EIR_REG(base)                       ((base)->EIR)
#define ENET_EIMR_REG(base)                      ((base)->EIMR)
#define ENET_RDAR_REG(base)                      ((base)->RDAR)
#define ENET_TDAR_REG(base)                      ((base)->TDAR)
#define ENET_ECR_REG(base)                       ((base)->ECR)
#define ENET_MMFR_REG(base)                      ((base)->MMFR)
#define ENET_MSCR_REG(base)                      ((base)->MSCR)
#define ENET_MIBC_REG(base)                      ((base)->MIBC)
#define ENET_RCR_REG(base)                       ((base)->RCR)
#define ENET_TCR_REG(base)                       ((base)->TCR)
#define ENET_PALR_REG(base)                      ((base)->PALR)
#define ENET_PAUR_REG(base)                      ((base)->PAUR)
#define ENET_OPD_REG(base)                       ((base)->OPD)
#define ENET_IAUR_REG(base)                      ((base)->IAUR)
#define ENET_IALR_REG(base)                      ((base)->IALR)
#define ENET_GAUR_REG(base)                      ((base)->GAUR)
#define ENET_GALR_REG(base)                      ((base)->GALR)
#define ENET_TFWR_REG(base)                      ((base)->TFWR)
#define ENET_RDSR_REG(base)                      ((base)->RDSR)
#define ENET_TDSR_REG(base)                      ((base)->TDSR)
#define ENET_MRBR_REG(base)                      ((base)->MRBR)
#define ENET_RSFL_REG(base)                      ((base)->RSFL)
#define ENET_RSEM_REG(base)                      ((base)->RSEM)
#define ENET_RAEM_REG(base)                      ((base)->RAEM)
#define ENET_RAFL_REG(base)                      ((base)->RAFL)
#define ENET_TSEM_REG(base)                      ((base)->TSEM)
#define ENET_TAEM_REG(base)                      ((base)->TAEM)
#define ENET_TAFL_REG(base)                      ((base)->TAFL)
#define ENET_TIPG_REG(base)                      ((base)->TIPG)
#define ENET_FTRL_REG(base)                      ((base)->FTRL)
#define ENET_TACC_REG(base)                      ((base)->TACC)
#define ENET_RACC_REG(base)                      ((base)->RACC)
#define ENET_RMON_T_PACKETS_REG(base)            ((base)->RMON_T_PACKETS)
#define ENET_RMON_T_BC_PKT_REG(base)             ((base)->RMON_T_BC_PKT)
#define ENET_RMON_T_MC_PKT_REG(base)             ((base)->RMON_T_MC_PKT)
#define ENET_RMON_T_CRC_ALIGN_REG(base)          ((base)->RMON_T_CRC_ALIGN)
#define ENET_RMON_T_UNDERSIZE_REG(base)          ((base)->RMON_T_UNDERSIZE)
#define ENET_RMON_T_OVERSIZE_REG(base)           ((base)->RMON_T_OVERSIZE)
#define ENET_RMON_T_FRAG_REG(base)               ((base)->RMON_T_FRAG)
#define ENET_RMON_T_JAB_REG(base)                ((base)->RMON_T_JAB)
#define ENET_RMON_T_COL_REG(base)                ((base)->RMON_T_COL)
#define ENET_RMON_T_P64_REG(base)                ((base)->RMON_T_P64)
#define ENET_RMON_T_P65TO127_REG(base)           ((base)->RMON_T_P65TO127)
#define ENET_RMON_T_P128TO255_REG(base)          ((base)->RMON_T_P128TO255)
#define ENET_RMON_T_P256TO511_REG(base)          ((base)->RMON_T_P256TO511)
#define ENET_RMON_T_P512TO1023_REG(base)         ((base)->RMON_T_P512TO1023)
#define ENET_RMON_T_P1024TO2047_REG(base)        ((base)->RMON_T_P1024TO2047)
#define ENET_RMON_T_P_GTE2048_REG(base)          ((base)->RMON_T_P_GTE2048)
#define ENET_RMON_T_OCTETS_REG(base)             ((base)->RMON_T_OCTETS)
#define ENET_IEEE_T_FRAME_OK_REG(base)           ((base)->IEEE_T_FRAME_OK)
#define ENET_IEEE_T_1COL_REG(base)               ((base)->IEEE_T_1COL)
#define ENET_IEEE_T_MCOL_REG(base)               ((base)->IEEE_T_MCOL)
#define ENET_IEEE_T_DEF_REG(base)                ((base)->IEEE_T_DEF)
#define ENET_IEEE_T_LCOL_REG(base)               ((base)->IEEE_T_LCOL)
#define ENET_IEEE_T_EXCOL_REG(base)              ((base)->IEEE_T_EXCOL)
#define ENET_IEEE_T_MACERR_REG(base)             ((base)->IEEE_T_MACERR)
#define ENET_IEEE_T_CSERR_REG(base)              ((base)->IEEE_T_CSERR)
#define ENET_IEEE_T_FDXFC_REG(base)              ((base)->IEEE_T_FDXFC)
#define ENET_IEEE_T_OCTETS_OK_REG(base)          ((base)->IEEE_T_OCTETS_OK)
#define ENET_RMON_R_PACKETS_REG(base)            ((base)->RMON_R_PACKETS)
#define ENET_RMON_R_BC_PKT_REG(base)             ((base)->RMON_R_BC_PKT)
#define ENET_RMON_R_MC_PKT_REG(base)             ((base)->RMON_R_MC_PKT)
#define ENET_RMON_R_CRC_ALIGN_REG(base)          ((base)->RMON_R_CRC_ALIGN)
#define ENET_RMON_R_UNDERSIZE_REG(base)          ((base)->RMON_R_UNDERSIZE)
#define ENET_RMON_R_OVERSIZE_REG(base)           ((base)->RMON_R_OVERSIZE)
#define ENET_RMON_R_FRAG_REG(base)               ((base)->RMON_R_FRAG)
#define ENET_RMON_R_JAB_REG(base)                ((base)->RMON_R_JAB)
#define ENET_RMON_R_P64_REG(base)                ((base)->RMON_R_P64)
#define ENET_RMON_R_P65TO127_REG(base)           ((base)->RMON_R_P65TO127)
#define ENET_RMON_R_P128TO255_REG(base)          ((base)->RMON_R_P128TO255)
#define ENET_RMON_R_P256TO511_REG(base)          ((base)->RMON_R_P256TO511)
#define ENET_RMON_R_P512TO1023_REG(base)         ((base)->RMON_R_P512TO1023)
#define ENET_RMON_R_P1024TO2047_REG(base)        ((base)->RMON_R_P1024TO2047)
#define ENET_RMON_R_P_GTE2048_REG(base)          ((base)->RMON_R_P_GTE2048)
#define ENET_RMON_R_OCTETS_REG(base)             ((base)->RMON_R_OCTETS)
#define ENET_IEEE_R_DROP_REG(base)               ((base)->IEEE_R_DROP)
#define ENET_IEEE_R_FRAME_OK_REG(base)           ((base)->IEEE_R_FRAME_OK)
#define ENET_IEEE_R_CRC_REG(base)                ((base)->IEEE_R_CRC)
#define ENET_IEEE_R_ALIGN_REG(base)              ((base)->IEEE_R_ALIGN)
#define ENET_IEEE_R_MACERR_REG(base)             ((base)->IEEE_R_MACERR)
#define ENET_IEEE_R_FDXFC_REG(base)              ((base)->IEEE_R_FDXFC)
#define ENET_IEEE_R_OCTETS_OK_REG(base)          ((base)->IEEE_R_OCTETS_OK)
#define ENET_ATCR_REG(base)                      ((base)->ATCR)
#define ENET_ATVR_REG(base)                      ((base)->ATVR)
#define ENET_ATOFF_REG(base)                     ((base)->ATOFF)
#define ENET_ATPER_REG(base)                     ((base)->ATPER)
#define ENET_ATCOR_REG(base)                     ((base)->ATCOR)
#define ENET_ATINC_REG(base)                     ((base)->ATINC)
#define ENET_ATSTMP_REG(base)                    ((base)->ATSTMP)
#define ENET_TGSR_REG(base)                      ((base)->TGSR)
#define ENET_TCSR_REG(base,index)                ((base)->CHANNEL[index].TCSR)
#define ENET_TCCR_REG(base,index)                ((base)->CHANNEL[index].TCCR)


#define ENET_BASE_PTR                            (ENET)



/* ENET - Register instance definitions */
/* ENET */
#define ENET_EIR                                 ENET_EIR_REG(ENET)
#define ENET_EIMR                                ENET_EIMR_REG(ENET)
#define ENET_RDAR                                ENET_RDAR_REG(ENET)
#define ENET_TDAR                                ENET_TDAR_REG(ENET)
#define ENET_ECR                                 ENET_ECR_REG(ENET)
#define ENET_MMFR                                ENET_MMFR_REG(ENET)
#define ENET_MSCR                                ENET_MSCR_REG(ENET)
#define ENET_MIBC                                ENET_MIBC_REG(ENET)
#define ENET_RCR                                 ENET_RCR_REG(ENET)
#define ENET_TCR                                 ENET_TCR_REG(ENET)
#define ENET_PALR                                ENET_PALR_REG(ENET)
#define ENET_PAUR                                ENET_PAUR_REG(ENET)
#define ENET_OPD                                 ENET_OPD_REG(ENET)
#define ENET_IAUR                                ENET_IAUR_REG(ENET)
#define ENET_IALR                                ENET_IALR_REG(ENET)
#define ENET_GAUR                                ENET_GAUR_REG(ENET)
#define ENET_GALR                                ENET_GALR_REG(ENET)
#define ENET_TFWR                                ENET_TFWR_REG(ENET)
#define ENET_RDSR                                ENET_RDSR_REG(ENET)
#define ENET_TDSR                                ENET_TDSR_REG(ENET)
#define ENET_MRBR                                ENET_MRBR_REG(ENET)
#define ENET_RSFL                                ENET_RSFL_REG(ENET)
#define ENET_RSEM                                ENET_RSEM_REG(ENET)
#define ENET_RAEM                                ENET_RAEM_REG(ENET)
#define ENET_RAFL                                ENET_RAFL_REG(ENET)
#define ENET_TSEM                                ENET_TSEM_REG(ENET)
#define ENET_TAEM                                ENET_TAEM_REG(ENET)
#define ENET_TAFL                                ENET_TAFL_REG(ENET)
#define ENET_TIPG                                ENET_TIPG_REG(ENET)
#define ENET_FTRL                                ENET_FTRL_REG(ENET)
#define ENET_TACC                                ENET_TACC_REG(ENET)
#define ENET_RACC                                ENET_RACC_REG(ENET)
#define ENET_RMON_T_PACKETS                      ENET_RMON_T_PACKETS_REG(ENET)
#define ENET_RMON_T_BC_PKT                       ENET_RMON_T_BC_PKT_REG(ENET)
#define ENET_RMON_T_MC_PKT                       ENET_RMON_T_MC_PKT_REG(ENET)
#define ENET_RMON_T_CRC_ALIGN                    ENET_RMON_T_CRC_ALIGN_REG(ENET)
#define ENET_RMON_T_UNDERSIZE                    ENET_RMON_T_UNDERSIZE_REG(ENET)
#define ENET_RMON_T_OVERSIZE                     ENET_RMON_T_OVERSIZE_REG(ENET)
#define ENET_RMON_T_FRAG                         ENET_RMON_T_FRAG_REG(ENET)
#define ENET_RMON_T_JAB                          ENET_RMON_T_JAB_REG(ENET)
#define ENET_RMON_T_COL                          ENET_RMON_T_COL_REG(ENET)
#define ENET_RMON_T_P64                          ENET_RMON_T_P64_REG(ENET)
#define ENET_RMON_T_P65TO127                     ENET_RMON_T_P65TO127_REG(ENET)
#define ENET_RMON_T_P128TO255                    ENET_RMON_T_P128TO255_REG(ENET)
#define ENET_RMON_T_P256TO511                    ENET_RMON_T_P256TO511_REG(ENET)
#define ENET_RMON_T_P512TO1023                   ENET_RMON_T_P512TO1023_REG(ENET)
#define ENET_RMON_T_P1024TO2047                  ENET_RMON_T_P1024TO2047_REG(ENET)
#define ENET_RMON_T_P_GTE2048                    ENET_RMON_T_P_GTE2048_REG(ENET)
#define ENET_RMON_T_OCTETS                       ENET_RMON_T_OCTETS_REG(ENET)
#define ENET_IEEE_T_FRAME_OK                     ENET_IEEE_T_FRAME_OK_REG(ENET)
#define ENET_IEEE_T_1COL                         ENET_IEEE_T_1COL_REG(ENET)
#define ENET_IEEE_T_MCOL                         ENET_IEEE_T_MCOL_REG(ENET)
#define ENET_IEEE_T_DEF                          ENET_IEEE_T_DEF_REG(ENET)
#define ENET_IEEE_T_LCOL                         ENET_IEEE_T_LCOL_REG(ENET)
#define ENET_IEEE_T_EXCOL                        ENET_IEEE_T_EXCOL_REG(ENET)
#define ENET_IEEE_T_MACERR                       ENET_IEEE_T_MACERR_REG(ENET)
#define ENET_IEEE_T_CSERR                        ENET_IEEE_T_CSERR_REG(ENET)
#define ENET_IEEE_T_FDXFC                        ENET_IEEE_T_FDXFC_REG(ENET)
#define ENET_IEEE_T_OCTETS_OK                    ENET_IEEE_T_OCTETS_OK_REG(ENET)
#define ENET_RMON_R_PACKETS                      ENET_RMON_R_PACKETS_REG(ENET)
#define ENET_RMON_R_BC_PKT                       ENET_RMON_R_BC_PKT_REG(ENET)
#define ENET_RMON_R_MC_PKT                       ENET_RMON_R_MC_PKT_REG(ENET)
#define ENET_RMON_R_CRC_ALIGN                    ENET_RMON_R_CRC_ALIGN_REG(ENET)
#define ENET_RMON_R_UNDERSIZE                    ENET_RMON_R_UNDERSIZE_REG(ENET)
#define ENET_RMON_R_OVERSIZE                     ENET_RMON_R_OVERSIZE_REG(ENET)
#define ENET_RMON_R_FRAG                         ENET_RMON_R_FRAG_REG(ENET)
#define ENET_RMON_R_JAB                          ENET_RMON_R_JAB_REG(ENET)
#define ENET_RMON_R_P64                          ENET_RMON_R_P64_REG(ENET)
#define ENET_RMON_R_P65TO127                     ENET_RMON_R_P65TO127_REG(ENET)
#define ENET_RMON_R_P128TO255                    ENET_RMON_R_P128TO255_REG(ENET)
#define ENET_RMON_R_P256TO511                    ENET_RMON_R_P256TO511_REG(ENET)
#define ENET_RMON_R_P512TO1023                   ENET_RMON_R_P512TO1023_REG(ENET)
#define ENET_RMON_R_P1024TO2047                  ENET_RMON_R_P1024TO2047_REG(ENET)
#define ENET_RMON_R_P_GTE2048                    ENET_RMON_R_P_GTE2048_REG(ENET)
#define ENET_RMON_R_OCTETS                       ENET_RMON_R_OCTETS_REG(ENET)
#define ENET_IEEE_R_DROP                         ENET_IEEE_R_DROP_REG(ENET)
#define ENET_IEEE_R_FRAME_OK                     ENET_IEEE_R_FRAME_OK_REG(ENET)
#define ENET_IEEE_R_CRC                          ENET_IEEE_R_CRC_REG(ENET)
#define ENET_IEEE_R_ALIGN                        ENET_IEEE_R_ALIGN_REG(ENET)
#define ENET_IEEE_R_MACERR                       ENET_IEEE_R_MACERR_REG(ENET)
#define ENET_IEEE_R_FDXFC                        ENET_IEEE_R_FDXFC_REG(ENET)
#define ENET_IEEE_R_OCTETS_OK                    ENET_IEEE_R_OCTETS_OK_REG(ENET)
#define ENET_ATCR                                ENET_ATCR_REG(ENET)
#define ENET_ATVR                                ENET_ATVR_REG(ENET)
#define ENET_ATOFF                               ENET_ATOFF_REG(ENET)
#define ENET_ATPER                               ENET_ATPER_REG(ENET)
#define ENET_ATCOR                               ENET_ATCOR_REG(ENET)
#define ENET_ATINC                               ENET_ATINC_REG(ENET)
#define ENET_ATSTMP                              ENET_ATSTMP_REG(ENET)
#define ENET_TGSR                                ENET_TGSR_REG(ENET)
#define ENET_TCSR0                               ENET_TCSR_REG(ENET,0)
#define ENET_TCCR0                               ENET_TCCR_REG(ENET,0)
#define ENET_TCSR1                               ENET_TCSR_REG(ENET,1)
#define ENET_TCCR1                               ENET_TCCR_REG(ENET,1)
#define ENET_TCSR2                               ENET_TCSR_REG(ENET,2)
#define ENET_TCCR2                               ENET_TCCR_REG(ENET,2)
#define ENET_TCSR3                               ENET_TCSR_REG(ENET,3)
#define ENET_TCCR3                               ENET_TCCR_REG(ENET,3)

/* ENET - Register array accessors */
#define ENET_TCSR(index)                         ENET_TCSR_REG(ENET,index)
#define ENET_TCCR(index)                         ENET_TCCR_REG(ENET,index)



/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register accessors */
#define EWM_CTRL_REG(base)                       ((base)->CTRL)
#define EWM_SERV_REG(base)                       ((base)->SERV)
#define EWM_CMPL_REG(base)                       ((base)->CMPL)
#define EWM_CMPH_REG(base)                       ((base)->CMPH)

/* EWM - Peripheral instance base addresses */
#define EWM_BASE_PTR                             (EWM)

/* EWM - Register instance definitions */
/* EWM */
#define EWM_CTRL                                 EWM_CTRL_REG(EWM)
#define EWM_SERV                                 EWM_SERV_REG(EWM)
#define EWM_CMPL                                 EWM_CMPL_REG(EWM)
#define EWM_CMPH                                 EWM_CMPH_REG(EWM)



/* ----------------------------------------------------------------------------
   -- FB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Register_Accessor_Macros FB - Register accessor macros
 * @{
 */


/* FB - Register accessors */
#define FB_CSAR_REG(base,index)                  ((base)->CS[index].CSAR)
#define FB_CSMR_REG(base,index)                  ((base)->CS[index].CSMR)
#define FB_CSCR_REG(base,index)                  ((base)->CS[index].CSCR)
#define FB_CSPMCR_REG(base)                      ((base)->CSPMCR)


/* FB - Peripheral instance base addresses */
#define FB_BASE_PTR                              (FB)


/* FB - Register instance definitions */
/* FB */
#define FB_CSAR0                                 FB_CSAR_REG(FB,0)
#define FB_CSMR0                                 FB_CSMR_REG(FB,0)
#define FB_CSCR0                                 FB_CSCR_REG(FB,0)
#define FB_CSAR1                                 FB_CSAR_REG(FB,1)
#define FB_CSMR1                                 FB_CSMR_REG(FB,1)
#define FB_CSCR1                                 FB_CSCR_REG(FB,1)
#define FB_CSAR2                                 FB_CSAR_REG(FB,2)
#define FB_CSMR2                                 FB_CSMR_REG(FB,2)
#define FB_CSCR2                                 FB_CSCR_REG(FB,2)
#define FB_CSAR3                                 FB_CSAR_REG(FB,3)
#define FB_CSMR3                                 FB_CSMR_REG(FB,3)
#define FB_CSCR3                                 FB_CSCR_REG(FB,3)
#define FB_CSAR4                                 FB_CSAR_REG(FB,4)
#define FB_CSMR4                                 FB_CSMR_REG(FB,4)
#define FB_CSCR4                                 FB_CSCR_REG(FB,4)
#define FB_CSAR5                                 FB_CSAR_REG(FB,5)
#define FB_CSMR5                                 FB_CSMR_REG(FB,5)
#define FB_CSCR5                                 FB_CSCR_REG(FB,5)
#define FB_CSPMCR                                FB_CSPMCR_REG(FB)

/* FB - Register array accessors */
#define FB_CSAR(index)                           FB_CSAR_REG(FB,index)
#define FB_CSMR(index)                           FB_CSMR_REG(FB,index)
#define FB_CSCR(index)                           FB_CSCR_REG(FB,index)



/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register accessors */
#define FMC_PFAPR_REG(base)                      ((base)->PFAPR)
#define FMC_PFB0CR_REG(base)                     ((base)->PFB0CR)
#define FMC_PFB1CR_REG(base)                     ((base)->PFB1CR)
#define FMC_TAGVDW0S_REG(base,index)             ((base)->TAGVDW0S[index])
#define FMC_TAGVDW1S_REG(base,index)             ((base)->TAGVDW1S[index])
#define FMC_TAGVDW2S_REG(base,index)             ((base)->TAGVDW2S[index])
#define FMC_TAGVDW3S_REG(base,index)             ((base)->TAGVDW3S[index])
#define FMC_DATA_U_REG(base,index,index2)        ((base)->SET[index][index2].DATA_U)
#define FMC_DATA_L_REG(base,index,index2)        ((base)->SET[index][index2].DATA_L)


/* FMC - Peripheral instance base addresses */
#define FMC_BASE_PTR                             (FMC)


/* FMC - Register instance definitions */
/* FMC */
#define FMC_PFAPR                                FMC_PFAPR_REG(FMC)
#define FMC_PFB0CR                               FMC_PFB0CR_REG(FMC)
#define FMC_PFB1CR                               FMC_PFB1CR_REG(FMC)
#define FMC_TAGVDW0S0                            FMC_TAGVDW0S_REG(FMC,0)
#define FMC_TAGVDW0S1                            FMC_TAGVDW0S_REG(FMC,1)
#define FMC_TAGVDW0S2                            FMC_TAGVDW0S_REG(FMC,2)
#define FMC_TAGVDW0S3                            FMC_TAGVDW0S_REG(FMC,3)
#define FMC_TAGVDW1S0                            FMC_TAGVDW1S_REG(FMC,0)
#define FMC_TAGVDW1S1                            FMC_TAGVDW1S_REG(FMC,1)
#define FMC_TAGVDW1S2                            FMC_TAGVDW1S_REG(FMC,2)
#define FMC_TAGVDW1S3                            FMC_TAGVDW1S_REG(FMC,3)
#define FMC_TAGVDW2S0                            FMC_TAGVDW2S_REG(FMC,0)
#define FMC_TAGVDW2S1                            FMC_TAGVDW2S_REG(FMC,1)
#define FMC_TAGVDW2S2                            FMC_TAGVDW2S_REG(FMC,2)
#define FMC_TAGVDW2S3                            FMC_TAGVDW2S_REG(FMC,3)
#define FMC_TAGVDW3S0                            FMC_TAGVDW3S_REG(FMC,0)
#define FMC_TAGVDW3S1                            FMC_TAGVDW3S_REG(FMC,1)
#define FMC_TAGVDW3S2                            FMC_TAGVDW3S_REG(FMC,2)
#define FMC_TAGVDW3S3                            FMC_TAGVDW3S_REG(FMC,3)
#define FMC_DATAW0S0U                            FMC_DATA_U_REG(FMC,0,0)
#define FMC_DATAW0S0L                            FMC_DATA_L_REG(FMC,0,0)
#define FMC_DATAW0S1U                            FMC_DATA_U_REG(FMC,0,1)
#define FMC_DATAW0S1L                            FMC_DATA_L_REG(FMC,0,1)
#define FMC_DATAW0S2U                            FMC_DATA_U_REG(FMC,0,2)
#define FMC_DATAW0S2L                            FMC_DATA_L_REG(FMC,0,2)
#define FMC_DATAW0S3U                            FMC_DATA_U_REG(FMC,0,3)
#define FMC_DATAW0S3L                            FMC_DATA_L_REG(FMC,0,3)
#define FMC_DATAW1S0U                            FMC_DATA_U_REG(FMC,1,0)
#define FMC_DATAW1S0L                            FMC_DATA_L_REG(FMC,1,0)
#define FMC_DATAW1S1U                            FMC_DATA_U_REG(FMC,1,1)
#define FMC_DATAW1S1L                            FMC_DATA_L_REG(FMC,1,1)
#define FMC_DATAW1S2U                            FMC_DATA_U_REG(FMC,1,2)
#define FMC_DATAW1S2L                            FMC_DATA_L_REG(FMC,1,2)
#define FMC_DATAW1S3U                            FMC_DATA_U_REG(FMC,1,3)
#define FMC_DATAW1S3L                            FMC_DATA_L_REG(FMC,1,3)
#define FMC_DATAW2S0U                            FMC_DATA_U_REG(FMC,2,0)
#define FMC_DATAW2S0L                            FMC_DATA_L_REG(FMC,2,0)
#define FMC_DATAW2S1U                            FMC_DATA_U_REG(FMC,2,1)
#define FMC_DATAW2S1L                            FMC_DATA_L_REG(FMC,2,1)
#define FMC_DATAW2S2U                            FMC_DATA_U_REG(FMC,2,2)
#define FMC_DATAW2S2L                            FMC_DATA_L_REG(FMC,2,2)
#define FMC_DATAW2S3U                            FMC_DATA_U_REG(FMC,2,3)
#define FMC_DATAW2S3L                            FMC_DATA_L_REG(FMC,2,3)
#define FMC_DATAW3S0U                            FMC_DATA_U_REG(FMC,3,0)
#define FMC_DATAW3S0L                            FMC_DATA_L_REG(FMC,3,0)
#define FMC_DATAW3S1U                            FMC_DATA_U_REG(FMC,3,1)
#define FMC_DATAW3S1L                            FMC_DATA_L_REG(FMC,3,1)
#define FMC_DATAW3S2U                            FMC_DATA_U_REG(FMC,3,2)
#define FMC_DATAW3S2L                            FMC_DATA_L_REG(FMC,3,2)
#define FMC_DATAW3S3U                            FMC_DATA_U_REG(FMC,3,3)
#define FMC_DATAW3S3L                            FMC_DATA_L_REG(FMC,3,3)

/* FMC - Register array accessors */
#define FMC_TAGVDW0S(index)                      FMC_TAGVDW0S_REG(FMC,index)
#define FMC_TAGVDW1S(index)                      FMC_TAGVDW1S_REG(FMC,index)
#define FMC_TAGVDW2S(index)                      FMC_TAGVDW2S_REG(FMC,index)
#define FMC_TAGVDW3S(index)                      FMC_TAGVDW3S_REG(FMC,index)
#define FMC_DATA_U(index,index2)                 FMC_DATA_U_REG(FMC,index,index2)
#define FMC_DATA_L(index,index2)                 FMC_DATA_L_REG(FMC,index,index2)

/* ----------------------------------------------------------------------------
   -- FTFE - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFE_Register_Accessor_Macros FTFE - Register accessor macros
 * @{
 */


/* FTFE - Register accessors */
#define FTFE_FSTAT_REG(base)                     ((base)->FSTAT)
#define FTFE_FCNFG_REG(base)                     ((base)->FCNFG)
#define FTFE_FSEC_REG(base)                      ((base)->FSEC)
#define FTFE_FOPT_REG(base)                      ((base)->FOPT)
#define FTFE_FCCOB3_REG(base)                    ((base)->FCCOB3)
#define FTFE_FCCOB2_REG(base)                    ((base)->FCCOB2)
#define FTFE_FCCOB1_REG(base)                    ((base)->FCCOB1)
#define FTFE_FCCOB0_REG(base)                    ((base)->FCCOB0)
#define FTFE_FCCOB7_REG(base)                    ((base)->FCCOB7)
#define FTFE_FCCOB6_REG(base)                    ((base)->FCCOB6)
#define FTFE_FCCOB5_REG(base)                    ((base)->FCCOB5)
#define FTFE_FCCOB4_REG(base)                    ((base)->FCCOB4)
#define FTFE_FCCOBB_REG(base)                    ((base)->FCCOBB)
#define FTFE_FCCOBA_REG(base)                    ((base)->FCCOBA)
#define FTFE_FCCOB9_REG(base)                    ((base)->FCCOB9)
#define FTFE_FCCOB8_REG(base)                    ((base)->FCCOB8)
#define FTFE_FPROT3_REG(base)                    ((base)->FPROT3)
#define FTFE_FPROT2_REG(base)                    ((base)->FPROT2)
#define FTFE_FPROT1_REG(base)                    ((base)->FPROT1)
#define FTFE_FPROT0_REG(base)                    ((base)->FPROT0)
#define FTFE_FEPROT_REG(base)                    ((base)->FEPROT)
#define FTFE_FDPROT_REG(base)                    ((base)->FDPROT)

/* FTFE - Peripheral instance base addresses */
#define FTFE_BASE_PTR                            (FTFE)
/* FTFE - Register instance definitions */
/* FTFE */
#define FTFE_FSTAT                               FTFE_FSTAT_REG(FTFE)
#define FTFE_FCNFG                               FTFE_FCNFG_REG(FTFE)
#define FTFE_FSEC                                FTFE_FSEC_REG(FTFE)
#define FTFE_FOPT                                FTFE_FOPT_REG(FTFE)
#define FTFE_FCCOB3                              FTFE_FCCOB3_REG(FTFE)
#define FTFE_FCCOB2                              FTFE_FCCOB2_REG(FTFE)
#define FTFE_FCCOB1                              FTFE_FCCOB1_REG(FTFE)
#define FTFE_FCCOB0                              FTFE_FCCOB0_REG(FTFE)
#define FTFE_FCCOB7                              FTFE_FCCOB7_REG(FTFE)
#define FTFE_FCCOB6                              FTFE_FCCOB6_REG(FTFE)
#define FTFE_FCCOB5                              FTFE_FCCOB5_REG(FTFE)
#define FTFE_FCCOB4                              FTFE_FCCOB4_REG(FTFE)
#define FTFE_FCCOBB                              FTFE_FCCOBB_REG(FTFE)
#define FTFE_FCCOBA                              FTFE_FCCOBA_REG(FTFE)
#define FTFE_FCCOB9                              FTFE_FCCOB9_REG(FTFE)
#define FTFE_FCCOB8                              FTFE_FCCOB8_REG(FTFE)
#define FTFE_FPROT3                              FTFE_FPROT3_REG(FTFE)
#define FTFE_FPROT2                              FTFE_FPROT2_REG(FTFE)
#define FTFE_FPROT1                              FTFE_FPROT1_REG(FTFE)
#define FTFE_FPROT0                              FTFE_FPROT0_REG(FTFE)
#define FTFE_FEPROT                              FTFE_FEPROT_REG(FTFE)
#define FTFE_FDPROT                              FTFE_FDPROT_REG(FTFE)

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register accessors */
#define FTM_SC_REG(base)                         ((base)->SC)
#define FTM_CNT_REG(base)                        ((base)->CNT)
#define FTM_MOD_REG(base)                        ((base)->MOD)
#define FTM_CnSC_REG(base,index)                 ((base)->CONTROLS[index].CnSC)
#define FTM_CnV_REG(base,index)                  ((base)->CONTROLS[index].CnV)
#define FTM_CNTIN_REG(base)                      ((base)->CNTIN)
#define FTM_STATUS_REG(base)                     ((base)->STATUS)
#define FTM_MODE_REG(base)                       ((base)->MODE)
#define FTM_SYNC_REG(base)                       ((base)->SYNC)
#define FTM_OUTINIT_REG(base)                    ((base)->OUTINIT)
#define FTM_OUTMASK_REG(base)                    ((base)->OUTMASK)
#define FTM_COMBINE_REG(base)                    ((base)->COMBINE)
#define FTM_DEADTIME_REG(base)                   ((base)->DEADTIME)
#define FTM_EXTTRIG_REG(base)                    ((base)->EXTTRIG)
#define FTM_POL_REG(base)                        ((base)->POL)
#define FTM_FMS_REG(base)                        ((base)->FMS)
#define FTM_FILTER_REG(base)                     ((base)->FILTER)
#define FTM_FLTCTRL_REG(base)                    ((base)->FLTCTRL)
#define FTM_QDCTRL_REG(base)                     ((base)->QDCTRL)
#define FTM_CONF_REG(base)                       ((base)->CONF)
#define FTM_FLTPOL_REG(base)                     ((base)->FLTPOL)
#define FTM_SYNCONF_REG(base)                    ((base)->SYNCONF)
#define FTM_INVCTRL_REG(base)                    ((base)->INVCTRL)
#define FTM_SWOCTRL_REG(base)                    ((base)->SWOCTRL)
#define FTM_PWMLOAD_REG(base)                    ((base)->PWMLOAD)


#define FTM0_BASE_PTR                            (FTM0)
#define FTM1_BASE_PTR                            (FTM1)
#define FTM2_BASE_PTR                            (FTM2)
#define FTM3_BASE_PTR                            (FTM3)


/* FTM - Register instance definitions */
/* FTM0 */
#define FTM0_SC                                  FTM_SC_REG(FTM0)
#define FTM0_CNT                                 FTM_CNT_REG(FTM0)
#define FTM0_MOD                                 FTM_MOD_REG(FTM0)
#define FTM0_C0SC                                FTM_CnSC_REG(FTM0,0)
#define FTM0_C0V                                 FTM_CnV_REG(FTM0,0)
#define FTM0_C1SC                                FTM_CnSC_REG(FTM0,1)
#define FTM0_C1V                                 FTM_CnV_REG(FTM0,1)
#define FTM0_C2SC                                FTM_CnSC_REG(FTM0,2)
#define FTM0_C2V                                 FTM_CnV_REG(FTM0,2)
#define FTM0_C3SC                                FTM_CnSC_REG(FTM0,3)
#define FTM0_C3V                                 FTM_CnV_REG(FTM0,3)
#define FTM0_C4SC                                FTM_CnSC_REG(FTM0,4)
#define FTM0_C4V                                 FTM_CnV_REG(FTM0,4)
#define FTM0_C5SC                                FTM_CnSC_REG(FTM0,5)
#define FTM0_C5V                                 FTM_CnV_REG(FTM0,5)
#define FTM0_C6SC                                FTM_CnSC_REG(FTM0,6)
#define FTM0_C6V                                 FTM_CnV_REG(FTM0,6)
#define FTM0_C7SC                                FTM_CnSC_REG(FTM0,7)
#define FTM0_C7V                                 FTM_CnV_REG(FTM0,7)
#define FTM0_CNTIN                               FTM_CNTIN_REG(FTM0)
#define FTM0_STATUS                              FTM_STATUS_REG(FTM0)
#define FTM0_MODE                                FTM_MODE_REG(FTM0)
#define FTM0_SYNC                                FTM_SYNC_REG(FTM0)
#define FTM0_OUTINIT                             FTM_OUTINIT_REG(FTM0)
#define FTM0_OUTMASK                             FTM_OUTMASK_REG(FTM0)
#define FTM0_COMBINE                             FTM_COMBINE_REG(FTM0)
#define FTM0_DEADTIME                            FTM_DEADTIME_REG(FTM0)
#define FTM0_EXTTRIG                             FTM_EXTTRIG_REG(FTM0)
#define FTM0_POL                                 FTM_POL_REG(FTM0)
#define FTM0_FMS                                 FTM_FMS_REG(FTM0)
#define FTM0_FILTER                              FTM_FILTER_REG(FTM0)
#define FTM0_FLTCTRL                             FTM_FLTCTRL_REG(FTM0)
#define FTM0_QDCTRL                              FTM_QDCTRL_REG(FTM0)
#define FTM0_CONF                                FTM_CONF_REG(FTM0)
#define FTM0_FLTPOL                              FTM_FLTPOL_REG(FTM0)
#define FTM0_SYNCONF                             FTM_SYNCONF_REG(FTM0)
#define FTM0_INVCTRL                             FTM_INVCTRL_REG(FTM0)
#define FTM0_SWOCTRL                             FTM_SWOCTRL_REG(FTM0)
#define FTM0_PWMLOAD                             FTM_PWMLOAD_REG(FTM0)
/* FTM1 */
#define FTM1_SC                                  FTM_SC_REG(FTM1)
#define FTM1_CNT                                 FTM_CNT_REG(FTM1)
#define FTM1_MOD                                 FTM_MOD_REG(FTM1)
#define FTM1_C0SC                                FTM_CnSC_REG(FTM1,0)
#define FTM1_C0V                                 FTM_CnV_REG(FTM1,0)
#define FTM1_C1SC                                FTM_CnSC_REG(FTM1,1)
#define FTM1_C1V                                 FTM_CnV_REG(FTM1,1)
#define FTM1_CNTIN                               FTM_CNTIN_REG(FTM1)
#define FTM1_STATUS                              FTM_STATUS_REG(FTM1)
#define FTM1_MODE                                FTM_MODE_REG(FTM1)
#define FTM1_SYNC                                FTM_SYNC_REG(FTM1)
#define FTM1_OUTINIT                             FTM_OUTINIT_REG(FTM1)
#define FTM1_OUTMASK                             FTM_OUTMASK_REG(FTM1)
#define FTM1_COMBINE                             FTM_COMBINE_REG(FTM1)
#define FTM1_DEADTIME                            FTM_DEADTIME_REG(FTM1)
#define FTM1_EXTTRIG                             FTM_EXTTRIG_REG(FTM1)
#define FTM1_POL                                 FTM_POL_REG(FTM1)
#define FTM1_FMS                                 FTM_FMS_REG(FTM1)
#define FTM1_FILTER                              FTM_FILTER_REG(FTM1)
#define FTM1_FLTCTRL                             FTM_FLTCTRL_REG(FTM1)
#define FTM1_QDCTRL                              FTM_QDCTRL_REG(FTM1)
#define FTM1_CONF                                FTM_CONF_REG(FTM1)
#define FTM1_FLTPOL                              FTM_FLTPOL_REG(FTM1)
#define FTM1_SYNCONF                             FTM_SYNCONF_REG(FTM1)
#define FTM1_INVCTRL                             FTM_INVCTRL_REG(FTM1)
#define FTM1_SWOCTRL                             FTM_SWOCTRL_REG(FTM1)
#define FTM1_PWMLOAD                             FTM_PWMLOAD_REG(FTM1)
/* FTM2 */
#define FTM2_SC                                  FTM_SC_REG(FTM2)
#define FTM2_CNT                                 FTM_CNT_REG(FTM2)
#define FTM2_MOD                                 FTM_MOD_REG(FTM2)
#define FTM2_C0SC                                FTM_CnSC_REG(FTM2,0)
#define FTM2_C0V                                 FTM_CnV_REG(FTM2,0)
#define FTM2_C1SC                                FTM_CnSC_REG(FTM2,1)
#define FTM2_C1V                                 FTM_CnV_REG(FTM2,1)
#define FTM2_CNTIN                               FTM_CNTIN_REG(FTM2)
#define FTM2_STATUS                              FTM_STATUS_REG(FTM2)
#define FTM2_MODE                                FTM_MODE_REG(FTM2)
#define FTM2_SYNC                                FTM_SYNC_REG(FTM2)
#define FTM2_OUTINIT                             FTM_OUTINIT_REG(FTM2)
#define FTM2_OUTMASK                             FTM_OUTMASK_REG(FTM2)
#define FTM2_COMBINE                             FTM_COMBINE_REG(FTM2)
#define FTM2_DEADTIME                            FTM_DEADTIME_REG(FTM2)
#define FTM2_EXTTRIG                             FTM_EXTTRIG_REG(FTM2)
#define FTM2_POL                                 FTM_POL_REG(FTM2)
#define FTM2_FMS                                 FTM_FMS_REG(FTM2)
#define FTM2_FILTER                              FTM_FILTER_REG(FTM2)
#define FTM2_FLTCTRL                             FTM_FLTCTRL_REG(FTM2)
#define FTM2_QDCTRL                              FTM_QDCTRL_REG(FTM2)
#define FTM2_CONF                                FTM_CONF_REG(FTM2)
#define FTM2_FLTPOL                              FTM_FLTPOL_REG(FTM2)
#define FTM2_SYNCONF                             FTM_SYNCONF_REG(FTM2)
#define FTM2_INVCTRL                             FTM_INVCTRL_REG(FTM2)
#define FTM2_SWOCTRL                             FTM_SWOCTRL_REG(FTM2)
#define FTM2_PWMLOAD                             FTM_PWMLOAD_REG(FTM2)
/* FTM3 */
#define FTM3_SC                                  FTM_SC_REG(FTM3)
#define FTM3_CNT                                 FTM_CNT_REG(FTM3)
#define FTM3_MOD                                 FTM_MOD_REG(FTM3)
#define FTM3_C0SC                                FTM_CnSC_REG(FTM3,0)
#define FTM3_C0V                                 FTM_CnV_REG(FTM3,0)
#define FTM3_C1SC                                FTM_CnSC_REG(FTM3,1)
#define FTM3_C1V                                 FTM_CnV_REG(FTM3,1)
#define FTM3_C2SC                                FTM_CnSC_REG(FTM3,2)
#define FTM3_C2V                                 FTM_CnV_REG(FTM3,2)
#define FTM3_C3SC                                FTM_CnSC_REG(FTM3,3)
#define FTM3_C3V                                 FTM_CnV_REG(FTM3,3)
#define FTM3_C4SC                                FTM_CnSC_REG(FTM3,4)
#define FTM3_C4V                                 FTM_CnV_REG(FTM3,4)
#define FTM3_C5SC                                FTM_CnSC_REG(FTM3,5)
#define FTM3_C5V                                 FTM_CnV_REG(FTM3,5)
#define FTM3_C6SC                                FTM_CnSC_REG(FTM3,6)
#define FTM3_C6V                                 FTM_CnV_REG(FTM3,6)
#define FTM3_C7SC                                FTM_CnSC_REG(FTM3,7)
#define FTM3_C7V                                 FTM_CnV_REG(FTM3,7)
#define FTM3_CNTIN                               FTM_CNTIN_REG(FTM3)
#define FTM3_STATUS                              FTM_STATUS_REG(FTM3)
#define FTM3_MODE                                FTM_MODE_REG(FTM3)
#define FTM3_SYNC                                FTM_SYNC_REG(FTM3)
#define FTM3_OUTINIT                             FTM_OUTINIT_REG(FTM3)
#define FTM3_OUTMASK                             FTM_OUTMASK_REG(FTM3)
#define FTM3_COMBINE                             FTM_COMBINE_REG(FTM3)
#define FTM3_DEADTIME                            FTM_DEADTIME_REG(FTM3)
#define FTM3_EXTTRIG                             FTM_EXTTRIG_REG(FTM3)
#define FTM3_POL                                 FTM_POL_REG(FTM3)
#define FTM3_FMS                                 FTM_FMS_REG(FTM3)
#define FTM3_FILTER                              FTM_FILTER_REG(FTM3)
#define FTM3_FLTCTRL                             FTM_FLTCTRL_REG(FTM3)
#define FTM3_QDCTRL                              FTM_QDCTRL_REG(FTM3)
#define FTM3_CONF                                FTM_CONF_REG(FTM3)
#define FTM3_FLTPOL                              FTM_FLTPOL_REG(FTM3)
#define FTM3_SYNCONF                             FTM_SYNCONF_REG(FTM3)
#define FTM3_INVCTRL                             FTM_INVCTRL_REG(FTM3)
#define FTM3_SWOCTRL                             FTM_SWOCTRL_REG(FTM3)
#define FTM3_PWMLOAD                             FTM_PWMLOAD_REG(FTM3)

/* FTM - Register array accessors */
#define FTM0_CnSC(index)                         FTM_CnSC_REG(FTM0,index)
#define FTM1_CnSC(index)                         FTM_CnSC_REG(FTM1,index)
#define FTM2_CnSC(index)                         FTM_CnSC_REG(FTM2,index)
#define FTM3_CnSC(index)                         FTM_CnSC_REG(FTM3,index)
#define FTM0_CnV(index)                          FTM_CnV_REG(FTM0,index)
#define FTM1_CnV(index)                          FTM_CnV_REG(FTM1,index)
#define FTM2_CnV(index)                          FTM_CnV_REG(FTM2,index)
#define FTM3_CnV(index)                          FTM_CnV_REG(FTM3,index)


/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register accessors */
#define GPIO_PDOR_REG(base)                      ((base)->PDOR)
#define GPIO_PSOR_REG(base)                      ((base)->PSOR)
#define GPIO_PCOR_REG(base)                      ((base)->PCOR)
#define GPIO_PTOR_REG(base)                      ((base)->PTOR)
#define GPIO_PDIR_REG(base)                      ((base)->PDIR)
#define GPIO_PDDR_REG(base)                      ((base)->PDDR)

/* GPIO - Peripheral instance base addresses */
#define PTA_BASE_PTR                             (PTA)
#define PTB_BASE_PTR                             (PTB)
#define PTC_BASE_PTR                             (PTC)
#define PTD_BASE_PTR                             (PTD)
#define PTE_BASE_PTR                             (PTE)


/* GPIO - Register instance definitions */
/* PTA */
#define GPIOA_PDOR                               GPIO_PDOR_REG(PTA)
#define GPIOA_PSOR                               GPIO_PSOR_REG(PTA)
#define GPIOA_PCOR                               GPIO_PCOR_REG(PTA)
#define GPIOA_PTOR                               GPIO_PTOR_REG(PTA)
#define GPIOA_PDIR                               GPIO_PDIR_REG(PTA)
#define GPIOA_PDDR                               GPIO_PDDR_REG(PTA)
/* PTB */
#define GPIOB_PDOR                               GPIO_PDOR_REG(PTB)
#define GPIOB_PSOR                               GPIO_PSOR_REG(PTB)
#define GPIOB_PCOR                               GPIO_PCOR_REG(PTB)
#define GPIOB_PTOR                               GPIO_PTOR_REG(PTB)
#define GPIOB_PDIR                               GPIO_PDIR_REG(PTB)
#define GPIOB_PDDR                               GPIO_PDDR_REG(PTB)
/* PTC */
#define GPIOC_PDOR                               GPIO_PDOR_REG(PTC)
#define GPIOC_PSOR                               GPIO_PSOR_REG(PTC)
#define GPIOC_PCOR                               GPIO_PCOR_REG(PTC)
#define GPIOC_PTOR                               GPIO_PTOR_REG(PTC)
#define GPIOC_PDIR                               GPIO_PDIR_REG(PTC)
#define GPIOC_PDDR                               GPIO_PDDR_REG(PTC)
/* PTD */
#define GPIOD_PDOR                               GPIO_PDOR_REG(PTD)
#define GPIOD_PSOR                               GPIO_PSOR_REG(PTD)
#define GPIOD_PCOR                               GPIO_PCOR_REG(PTD)
#define GPIOD_PTOR                               GPIO_PTOR_REG(PTD)
#define GPIOD_PDIR                               GPIO_PDIR_REG(PTD)
#define GPIOD_PDDR                               GPIO_PDDR_REG(PTD)
/* PTE */
#define GPIOE_PDOR                               GPIO_PDOR_REG(PTE)
#define GPIOE_PSOR                               GPIO_PSOR_REG(PTE)
#define GPIOE_PCOR                               GPIO_PCOR_REG(PTE)
#define GPIOE_PTOR                               GPIO_PTOR_REG(PTE)
#define GPIOE_PDIR                               GPIO_PDIR_REG(PTE)
#define GPIOE_PDDR                               GPIO_PDDR_REG(PTE)


/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register accessors */
#define I2C_A1_REG(base)                         ((base)->A1)
#define I2C_F_REG(base)                          ((base)->F)
#define I2C_C1_REG(base)                         ((base)->C1)
#define I2C_S_REG(base)                          ((base)->S)
#define I2C_D_REG(base)                          ((base)->D)
#define I2C_C2_REG(base)                         ((base)->C2)
#define I2C_FLT_REG(base)                        ((base)->FLT)
#define I2C_RA_REG(base)                         ((base)->RA)
#define I2C_SMB_REG(base)                        ((base)->SMB)
#define I2C_A2_REG(base)                         ((base)->A2)
#define I2C_SLTH_REG(base)                       ((base)->SLTH)
#define I2C_SLTL_REG(base)                       ((base)->SLTL)

/* I2C - Peripheral instance base addresses */
#define I2C0_BASE_PTR                            (I2C0)
#define I2C1_BASE_PTR                            (I2C1)
#define I2C2_BASE_PTR                            (I2C2)



/* I2C - Register instance definitions */
/* I2C0 */
#define I2C0_A1                                  I2C_A1_REG(I2C0)
#define I2C0_F                                   I2C_F_REG(I2C0)
#define I2C0_C1                                  I2C_C1_REG(I2C0)
#define I2C0_S                                   I2C_S_REG(I2C0)
#define I2C0_D                                   I2C_D_REG(I2C0)
#define I2C0_C2                                  I2C_C2_REG(I2C0)
#define I2C0_FLT                                 I2C_FLT_REG(I2C0)
#define I2C0_RA                                  I2C_RA_REG(I2C0)
#define I2C0_SMB                                 I2C_SMB_REG(I2C0)
#define I2C0_A2                                  I2C_A2_REG(I2C0)
#define I2C0_SLTH                                I2C_SLTH_REG(I2C0)
#define I2C0_SLTL                                I2C_SLTL_REG(I2C0)
/* I2C1 */
#define I2C1_A1                                  I2C_A1_REG(I2C1)
#define I2C1_F                                   I2C_F_REG(I2C1)
#define I2C1_C1                                  I2C_C1_REG(I2C1)
#define I2C1_S                                   I2C_S_REG(I2C1)
#define I2C1_D                                   I2C_D_REG(I2C1)
#define I2C1_C2                                  I2C_C2_REG(I2C1)
#define I2C1_FLT                                 I2C_FLT_REG(I2C1)
#define I2C1_RA                                  I2C_RA_REG(I2C1)
#define I2C1_SMB                                 I2C_SMB_REG(I2C1)
#define I2C1_A2                                  I2C_A2_REG(I2C1)
#define I2C1_SLTH                                I2C_SLTH_REG(I2C1)
#define I2C1_SLTL                                I2C_SLTL_REG(I2C1)
/* I2C2 */
#define I2C2_A1                                  I2C_A1_REG(I2C2)
#define I2C2_F                                   I2C_F_REG(I2C2)
#define I2C2_C1                                  I2C_C1_REG(I2C2)
#define I2C2_S                                   I2C_S_REG(I2C2)
#define I2C2_D                                   I2C_D_REG(I2C2)
#define I2C2_C2                                  I2C_C2_REG(I2C2)
#define I2C2_FLT                                 I2C_FLT_REG(I2C2)
#define I2C2_RA                                  I2C_RA_REG(I2C2)
#define I2C2_SMB                                 I2C_SMB_REG(I2C2)
#define I2C2_A2                                  I2C_A2_REG(I2C2)
#define I2C2_SLTH                                I2C_SLTH_REG(I2C2)
#define I2C2_SLTL                                I2C_SLTL_REG(I2C2)



/* ----------------------------------------------------------------------------
   -- I2S - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Accessor_Macros I2S - Register accessor macros
 * @{
 */


/* I2S - Register accessors */
#define I2S_TCSR_REG(base)                       ((base)->TCSR)
#define I2S_TCR1_REG(base)                       ((base)->TCR1)
#define I2S_TCR2_REG(base)                       ((base)->TCR2)
#define I2S_TCR3_REG(base)                       ((base)->TCR3)
#define I2S_TCR4_REG(base)                       ((base)->TCR4)
#define I2S_TCR5_REG(base)                       ((base)->TCR5)
#define I2S_TDR_REG(base,index)                  ((base)->TDR[index])
#define I2S_TFR_REG(base,index)                  ((base)->TFR[index])
#define I2S_TMR_REG(base)                        ((base)->TMR)
#define I2S_RCSR_REG(base)                       ((base)->RCSR)
#define I2S_RCR1_REG(base)                       ((base)->RCR1)
#define I2S_RCR2_REG(base)                       ((base)->RCR2)
#define I2S_RCR3_REG(base)                       ((base)->RCR3)
#define I2S_RCR4_REG(base)                       ((base)->RCR4)
#define I2S_RCR5_REG(base)                       ((base)->RCR5)
#define I2S_RDR_REG(base,index)                  ((base)->RDR[index])
#define I2S_RFR_REG(base,index)                  ((base)->RFR[index])
#define I2S_RMR_REG(base)                        ((base)->RMR)
#define I2S_MCR_REG(base)                        ((base)->MCR)
#define I2S_MDR_REG(base)                        ((base)->MDR)


/* I2S - Peripheral instance base addresses */
#define I2S0_BASE_PTR                            (I2S0)


/* I2S - Register instance definitions */
/* I2S0 */
#define I2S0_TCSR                                I2S_TCSR_REG(I2S0)
#define I2S0_TCR1                                I2S_TCR1_REG(I2S0)
#define I2S0_TCR2                                I2S_TCR2_REG(I2S0)
#define I2S0_TCR3                                I2S_TCR3_REG(I2S0)
#define I2S0_TCR4                                I2S_TCR4_REG(I2S0)
#define I2S0_TCR5                                I2S_TCR5_REG(I2S0)
#define I2S0_TDR0                                I2S_TDR_REG(I2S0,0)
#define I2S0_TDR1                                I2S_TDR_REG(I2S0,1)
#define I2S0_TFR0                                I2S_TFR_REG(I2S0,0)
#define I2S0_TFR1                                I2S_TFR_REG(I2S0,1)
#define I2S0_TMR                                 I2S_TMR_REG(I2S0)
#define I2S0_RCSR                                I2S_RCSR_REG(I2S0)
#define I2S0_RCR1                                I2S_RCR1_REG(I2S0)
#define I2S0_RCR2                                I2S_RCR2_REG(I2S0)
#define I2S0_RCR3                                I2S_RCR3_REG(I2S0)
#define I2S0_RCR4                                I2S_RCR4_REG(I2S0)
#define I2S0_RCR5                                I2S_RCR5_REG(I2S0)
#define I2S0_RDR0                                I2S_RDR_REG(I2S0,0)
#define I2S0_RDR1                                I2S_RDR_REG(I2S0,1)
#define I2S0_RFR0                                I2S_RFR_REG(I2S0,0)
#define I2S0_RFR1                                I2S_RFR_REG(I2S0,1)
#define I2S0_RMR                                 I2S_RMR_REG(I2S0)
#define I2S0_MCR                                 I2S_MCR_REG(I2S0)
#define I2S0_MDR                                 I2S_MDR_REG(I2S0)

/* I2S - Register array accessors */
#define I2S0_TDR(index)                          I2S_TDR_REG(I2S0,index)
#define I2S0_TFR(index)                          I2S_TFR_REG(I2S0,index)
#define I2S0_RDR(index)                          I2S_RDR_REG(I2S0,index)
#define I2S0_RFR(index)                          I2S_RFR_REG(I2S0,index)


/* ----------------------------------------------------------------------------
   -- LLWU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Accessor_Macros LLWU - Register accessor macros
 * @{
 */


/* LLWU - Register accessors */
#define LLWU_PE1_REG(base)                       ((base)->PE1)
#define LLWU_PE2_REG(base)                       ((base)->PE2)
#define LLWU_PE3_REG(base)                       ((base)->PE3)
#define LLWU_PE4_REG(base)                       ((base)->PE4)
#define LLWU_ME_REG(base)                        ((base)->ME)
#define LLWU_F1_REG(base)                        ((base)->F1)
#define LLWU_F2_REG(base)                        ((base)->F2)
#define LLWU_F3_REG(base)                        ((base)->F3)
#define LLWU_FILT1_REG(base)                     ((base)->FILT1)
#define LLWU_FILT2_REG(base)                     ((base)->FILT2)
#define LLWU_RST_REG(base)                       ((base)->RST)

/* LLWU - Peripheral instance base addresses */
#define LLWU_BASE_PTR                            (LLWU)


/* LLWU - Register instance definitions */
/* LLWU */
#define LLWU_PE1                                 LLWU_PE1_REG(LLWU)
#define LLWU_PE2                                 LLWU_PE2_REG(LLWU)
#define LLWU_PE3                                 LLWU_PE3_REG(LLWU)
#define LLWU_PE4                                 LLWU_PE4_REG(LLWU)
#define LLWU_ME                                  LLWU_ME_REG(LLWU)
#define LLWU_F1                                  LLWU_F1_REG(LLWU)
#define LLWU_F2                                  LLWU_F2_REG(LLWU)
#define LLWU_F3                                  LLWU_F3_REG(LLWU)
#define LLWU_FILT1                               LLWU_FILT1_REG(LLWU)
#define LLWU_FILT2                               LLWU_FILT2_REG(LLWU)
#define LLWU_RST                                 LLWU_RST_REG(LLWU)



/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register accessors */
#define LPTMR_CSR_REG(base)                      ((base)->CSR)
#define LPTMR_PSR_REG(base)                      ((base)->PSR)
#define LPTMR_CMR_REG(base)                      ((base)->CMR)
#define LPTMR_CNR_REG(base)                      ((base)->CNR)

/* LPTMR - Peripheral instance base addresses */
#define LPTMR0_BASE_PTR                          (LPTMR0)


/* LPTMR - Register instance definitions */
/* LPTMR0 */
#define LPTMR0_CSR                               LPTMR_CSR_REG(LPTMR0)
#define LPTMR0_PSR                               LPTMR_PSR_REG(LPTMR0)
#define LPTMR0_CMR                               LPTMR_CMR_REG(LPTMR0)
#define LPTMR0_CNR                               LPTMR_CNR_REG(LPTMR0)


/* ----------------------------------------------------------------------------
   -- MCG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Accessor_Macros MCG - Register accessor macros
 * @{
 */


/* MCG - Register accessors */
#define MCG_C1_REG(base)                         ((base)->C1)
#define MCG_C2_REG(base)                         ((base)->C2)
#define MCG_C3_REG(base)                         ((base)->C3)
#define MCG_C4_REG(base)                         ((base)->C4)
#define MCG_C5_REG(base)                         ((base)->C5)
#define MCG_C6_REG(base)                         ((base)->C6)
#define MCG_S_REG(base)                          ((base)->S)
#define MCG_SC_REG(base)                         ((base)->SC)
#define MCG_ATCVH_REG(base)                      ((base)->ATCVH)
#define MCG_ATCVL_REG(base)                      ((base)->ATCVL)
#define MCG_C7_REG(base)                         ((base)->C7)
#define MCG_C8_REG(base)                         ((base)->C8)

/* MCG - Peripheral instance base addresses */
#define MCG_BASE_PTR                             (MCG)


/* MCG - Register instance definitions */
/* MCG */
#define MCG_C1                                   MCG_C1_REG(MCG)
#define MCG_C2                                   MCG_C2_REG(MCG)
#define MCG_C3                                   MCG_C3_REG(MCG)
#define MCG_C4                                   MCG_C4_REG(MCG)
#define MCG_C5                                   MCG_C5_REG(MCG)
#define MCG_C6                                   MCG_C6_REG(MCG)
#define MCG_S                                    MCG_S_REG(MCG)
#define MCG_SC                                   MCG_SC_REG(MCG)
#define MCG_ATCVH                                MCG_ATCVH_REG(MCG)
#define MCG_ATCVL                                MCG_ATCVL_REG(MCG)
#define MCG_C7                                   MCG_C7_REG(MCG)
#define MCG_C8                                   MCG_C8_REG(MCG)



/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register accessors */
#define MCM_PLASC_REG(base)                      ((base)->PLASC)
#define MCM_PLAMC_REG(base)                      ((base)->PLAMC)
#define MCM_CR_REG(base)                         ((base)->CR)
#define MCM_ISCR_REG(base)                       ((base)->ISCR)
#define MCM_ETBCC_REG(base)                      ((base)->ETBCC)
#define MCM_ETBRL_REG(base)                      ((base)->ETBRL)
#define MCM_ETBCNT_REG(base)                     ((base)->ETBCNT)
#define MCM_PID_REG(base)                        ((base)->PID)

/* MCM - Peripheral instance base addresses */
#define MCM_BASE_PTR                             (MCM)


/* MCM - Register instance definitions */
/* MCM */
#define MCM_PLASC                                MCM_PLASC_REG(MCM)
#define MCM_PLAMC                                MCM_PLAMC_REG(MCM)
#define MCM_CR                                   MCM_CR_REG(MCM)
#define MCM_ISCR                                 MCM_ISCR_REG(MCM)
#define MCM_ETBCC                                MCM_ETBCC_REG(MCM)
#define MCM_ETBRL                                MCM_ETBRL_REG(MCM)
#define MCM_ETBCNT                               MCM_ETBCNT_REG(MCM)
#define MCM_PID                                  MCM_PID_REG(MCM)

/* ----------------------------------------------------------------------------
   -- SYSMPU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MPU_Register_Accessor_Macros MPU - Register accessor macros
 * @{
 */

/* MPU - Register accessors */
#define MPU_CESR_REG(base)                       ((base)->CESR)
#define MPU_EAR_REG(base,index)                  ((base)->SP[index].EAR)
#define MPU_EAR_COUNT                            5
#define MPU_EDR_REG(base,index)                  ((base)->SP[index].EDR)
#define MPU_EDR_COUNT                            5
#define MPU_WORD_REG(base,index,index2)          ((base)->WORD[index][index2])
#define MPU_WORD_COUNT                           12
#define MPU_WORD_COUNT2                          4
#define MPU_RGDAAC_REG(base,index)               ((base)->RGDAAC[index])
#define MPU_RGDAAC_COUNT                         12

#define MPU_BASE_PTR                             (SYSMPU)

/* MPU - Register instance definitions */
/* MPU */
#define MPU_CESR                                 MPU_CESR_REG(MPU_BASE_PTR)
#define MPU_EAR0                                 MPU_EAR_REG(MPU_BASE_PTR,0)
#define MPU_EDR0                                 MPU_EDR_REG(MPU_BASE_PTR,0)
#define MPU_EAR1                                 MPU_EAR_REG(MPU_BASE_PTR,1)
#define MPU_EDR1                                 MPU_EDR_REG(MPU_BASE_PTR,1)
#define MPU_EAR2                                 MPU_EAR_REG(MPU_BASE_PTR,2)
#define MPU_EDR2                                 MPU_EDR_REG(MPU_BASE_PTR,2)
#define MPU_EAR3                                 MPU_EAR_REG(MPU_BASE_PTR,3)
#define MPU_EDR3                                 MPU_EDR_REG(MPU_BASE_PTR,3)
#define MPU_EAR4                                 MPU_EAR_REG(MPU_BASE_PTR,4)
#define MPU_EDR4                                 MPU_EDR_REG(MPU_BASE_PTR,4)
#define MPU_RGD0_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,0,0)
#define MPU_RGD0_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,0,1)
#define MPU_RGD0_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,0,2)
#define MPU_RGD0_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,0,3)
#define MPU_RGD1_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,1,0)
#define MPU_RGD1_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,1,1)
#define MPU_RGD1_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,1,2)
#define MPU_RGD1_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,1,3)
#define MPU_RGD2_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,2,0)
#define MPU_RGD2_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,2,1)
#define MPU_RGD2_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,2,2)
#define MPU_RGD2_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,2,3)
#define MPU_RGD3_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,3,0)
#define MPU_RGD3_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,3,1)
#define MPU_RGD3_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,3,2)
#define MPU_RGD3_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,3,3)
#define MPU_RGD4_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,4,0)
#define MPU_RGD4_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,4,1)
#define MPU_RGD4_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,4,2)
#define MPU_RGD4_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,4,3)
#define MPU_RGD5_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,5,0)
#define MPU_RGD5_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,5,1)
#define MPU_RGD5_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,5,2)
#define MPU_RGD5_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,5,3)
#define MPU_RGD6_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,6,0)
#define MPU_RGD6_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,6,1)
#define MPU_RGD6_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,6,2)
#define MPU_RGD6_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,6,3)
#define MPU_RGD7_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,7,0)
#define MPU_RGD7_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,7,1)
#define MPU_RGD7_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,7,2)
#define MPU_RGD7_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,7,3)
#define MPU_RGD8_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,8,0)
#define MPU_RGD8_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,8,1)
#define MPU_RGD8_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,8,2)
#define MPU_RGD8_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,8,3)
#define MPU_RGD9_WORD0                           MPU_WORD_REG(MPU_BASE_PTR,9,0)
#define MPU_RGD9_WORD1                           MPU_WORD_REG(MPU_BASE_PTR,9,1)
#define MPU_RGD9_WORD2                           MPU_WORD_REG(MPU_BASE_PTR,9,2)
#define MPU_RGD9_WORD3                           MPU_WORD_REG(MPU_BASE_PTR,9,3)
#define MPU_RGD10_WORD0                          MPU_WORD_REG(MPU_BASE_PTR,10,0)
#define MPU_RGD10_WORD1                          MPU_WORD_REG(MPU_BASE_PTR,10,1)
#define MPU_RGD10_WORD2                          MPU_WORD_REG(MPU_BASE_PTR,10,2)
#define MPU_RGD10_WORD3                          MPU_WORD_REG(MPU_BASE_PTR,10,3)
#define MPU_RGD11_WORD0                          MPU_WORD_REG(MPU_BASE_PTR,11,0)
#define MPU_RGD11_WORD1                          MPU_WORD_REG(MPU_BASE_PTR,11,1)
#define MPU_RGD11_WORD2                          MPU_WORD_REG(MPU_BASE_PTR,11,2)
#define MPU_RGD11_WORD3                          MPU_WORD_REG(MPU_BASE_PTR,11,3)
#define MPU_RGDAAC0                              MPU_RGDAAC_REG(MPU_BASE_PTR,0)
#define MPU_RGDAAC1                              MPU_RGDAAC_REG(MPU_BASE_PTR,1)
#define MPU_RGDAAC2                              MPU_RGDAAC_REG(MPU_BASE_PTR,2)
#define MPU_RGDAAC3                              MPU_RGDAAC_REG(MPU_BASE_PTR,3)
#define MPU_RGDAAC4                              MPU_RGDAAC_REG(MPU_BASE_PTR,4)
#define MPU_RGDAAC5                              MPU_RGDAAC_REG(MPU_BASE_PTR,5)
#define MPU_RGDAAC6                              MPU_RGDAAC_REG(MPU_BASE_PTR,6)
#define MPU_RGDAAC7                              MPU_RGDAAC_REG(MPU_BASE_PTR,7)
#define MPU_RGDAAC8                              MPU_RGDAAC_REG(MPU_BASE_PTR,8)
#define MPU_RGDAAC9                              MPU_RGDAAC_REG(MPU_BASE_PTR,9)
#define MPU_RGDAAC10                             MPU_RGDAAC_REG(MPU_BASE_PTR,10)
#define MPU_RGDAAC11                             MPU_RGDAAC_REG(MPU_BASE_PTR,11)

/* MPU_BASE_PTR - Register array accessors */
#define MPU_EAR(index)                           MPU_EAR_REG(MPU_BASE_PTR,index)
#define MPU_EDR(index)                           MPU_EDR_REG(MPU_BASE_PTR,index)
#define MPU_WORD(index,index2)                   MPU_WORD_REG(MPU_BASE_PTR,index,index2)
#define MPU_RGDAAC(index)                        MPU_RGDAAC_REG(MPU_BASE_PTR,index)



/* ----------------------------------------------------------------------------
   -- NV - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Accessor_Macros NV - Register accessor macros
 * @{
 */


/* NV - Register accessors */
#define NV_BACKKEY3_REG(base)                    ((base)->BACKKEY3)
#define NV_BACKKEY2_REG(base)                    ((base)->BACKKEY2)
#define NV_BACKKEY1_REG(base)                    ((base)->BACKKEY1)
#define NV_BACKKEY0_REG(base)                    ((base)->BACKKEY0)
#define NV_BACKKEY7_REG(base)                    ((base)->BACKKEY7)
#define NV_BACKKEY6_REG(base)                    ((base)->BACKKEY6)
#define NV_BACKKEY5_REG(base)                    ((base)->BACKKEY5)
#define NV_BACKKEY4_REG(base)                    ((base)->BACKKEY4)
#define NV_FPROT3_REG(base)                      ((base)->FPROT3)
#define NV_FPROT2_REG(base)                      ((base)->FPROT2)
#define NV_FPROT1_REG(base)                      ((base)->FPROT1)
#define NV_FPROT0_REG(base)                      ((base)->FPROT0)
#define NV_FSEC_REG(base)                        ((base)->FSEC)
#define NV_FOPT_REG(base)                        ((base)->FOPT)
#define NV_FEPROT_REG(base)                      ((base)->FEPROT)
#define NV_FDPROT_REG(base)                      ((base)->FDPROT)


/* NV - Register instance definitions */
/* FTFE_FlashConfig */
#define NV_BACKKEY3                              NV_BACKKEY3_REG(FTFE_FlashConfig)
#define NV_BACKKEY2                              NV_BACKKEY2_REG(FTFE_FlashConfig)
#define NV_BACKKEY1                              NV_BACKKEY1_REG(FTFE_FlashConfig)
#define NV_BACKKEY0                              NV_BACKKEY0_REG(FTFE_FlashConfig)
#define NV_BACKKEY7                              NV_BACKKEY7_REG(FTFE_FlashConfig)
#define NV_BACKKEY6                              NV_BACKKEY6_REG(FTFE_FlashConfig)
#define NV_BACKKEY5                              NV_BACKKEY5_REG(FTFE_FlashConfig)
#define NV_BACKKEY4                              NV_BACKKEY4_REG(FTFE_FlashConfig)
#define NV_FPROT3                                NV_FPROT3_REG(FTFE_FlashConfig)
#define NV_FPROT2                                NV_FPROT2_REG(FTFE_FlashConfig)
#define NV_FPROT1                                NV_FPROT1_REG(FTFE_FlashConfig)
#define NV_FPROT0                                NV_FPROT0_REG(FTFE_FlashConfig)
#define NV_FSEC                                  NV_FSEC_REG(FTFE_FlashConfig)
#define NV_FOPT                                  NV_FOPT_REG(FTFE_FlashConfig)
#define NV_FEPROT                                NV_FEPROT_REG(FTFE_FlashConfig)
#define NV_FDPROT                                NV_FDPROT_REG(FTFE_FlashConfig)



/* ----------------------------------------------------------------------------
   -- OSC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Accessor_Macros OSC - Register accessor macros
 * @{
 */


/* OSC - Register accessors */
#define OSC_CR_REG(base)                         ((base)->CR)

/* OSC - Peripheral instance base addresses */
#define OSC_BASE_PTR                             (OSC)


/* OSC - Register instance definitions */
/* OSC */
#define OSC_CR                                   OSC_CR_REG(OSC)



/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register accessors */
#define PDB_SC_REG(base)                         ((base)->SC)
#define PDB_MOD_REG(base)                        ((base)->MOD)
#define PDB_CNT_REG(base)                        ((base)->CNT)
#define PDB_IDLY_REG(base)                       ((base)->IDLY)
#define PDB_C1_REG(base,index)                   ((base)->CH[index].C1)
#define PDB_S_REG(base,index)                    ((base)->CH[index].S)
#define PDB_DLY_REG(base,index,index2)           ((base)->CH[index].DLY[index2])
#define PDB_INTC_REG(base,index)                 ((base)->DAC[index].INTC)
#define PDB_INT_REG(base,index)                  ((base)->DAC[index].INT)
#define PDB_POEN_REG(base)                       ((base)->POEN)
#define PDB_PODLY_REG(base,index)                ((base)->PODLY[index])


/* PDB - Peripheral instance base addresses */
#define PDB0_BASE_PTR                            (PDB0)


/* PDB - Register instance definitions */
/* PDB0 */
#define PDB0_SC                                  PDB_SC_REG(PDB0)
#define PDB0_MOD                                 PDB_MOD_REG(PDB0)
#define PDB0_CNT                                 PDB_CNT_REG(PDB0)
#define PDB0_IDLY                                PDB_IDLY_REG(PDB0)
#define PDB0_CH0C1                               PDB_C1_REG(PDB0,0)
#define PDB0_CH0S                                PDB_S_REG(PDB0,0)
#define PDB0_CH0DLY0                             PDB_DLY_REG(PDB0,0,0)
#define PDB0_CH0DLY1                             PDB_DLY_REG(PDB0,0,1)
#define PDB0_CH1C1                               PDB_C1_REG(PDB0,1)
#define PDB0_CH1S                                PDB_S_REG(PDB0,1)
#define PDB0_CH1DLY0                             PDB_DLY_REG(PDB0,1,0)
#define PDB0_CH1DLY1                             PDB_DLY_REG(PDB0,1,1)
#define PDB0_DACINTC0                            PDB_INTC_REG(PDB0,0)
#define PDB0_DACINT0                             PDB_INT_REG(PDB0,0)
#define PDB0_DACINTC1                            PDB_INTC_REG(PDB0,1)
#define PDB0_DACINT1                             PDB_INT_REG(PDB0,1)
#define PDB0_POEN                                PDB_POEN_REG(PDB0)
#define PDB0_PO0DLY                              PDB_PODLY_REG(PDB0,0)
#define PDB0_PO1DLY                              PDB_PODLY_REG(PDB0,1)
#define PDB0_PO2DLY                              PDB_PODLY_REG(PDB0,2)

/* PDB - Register array accessors */
#define PDB0_C1(index)                           PDB_C1_REG(PDB0,index)
#define PDB0_S(index)                            PDB_S_REG(PDB0,index)
#define PDB0_DLY(index,index2)                   PDB_DLY_REG(PDB0,index,index2)
#define PDB0_INTC(index)                         PDB_INTC_REG(PDB0,index)
#define PDB0_INT(index)                          PDB_INT_REG(PDB0,index)
#define PDB0_PODLY(index)                        PDB_PODLY_REG(PDB0,index)

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register accessors */
#define PIT_MCR_REG(base)                        ((base)->MCR)
#define PIT_LDVAL_REG(base,index)                ((base)->CHANNEL[index].LDVAL)
#define PIT_CVAL_REG(base,index)                 ((base)->CHANNEL[index].CVAL)
#define PIT_TCTRL_REG(base,index)                ((base)->CHANNEL[index].TCTRL)
#define PIT_TFLG_REG(base,index)                 ((base)->CHANNEL[index].TFLG)


/* PIT - Peripheral instance base addresses */
#define PIT_BASE_PTR                             (PIT)


/* PIT - Register instance definitions */
/* PIT */
#define PIT_MCR                                  PIT_MCR_REG(PIT)
#define PIT_LDVAL0                               PIT_LDVAL_REG(PIT,0)
#define PIT_CVAL0                                PIT_CVAL_REG(PIT,0)
#define PIT_TCTRL0                               PIT_TCTRL_REG(PIT,0)
#define PIT_TFLG0                                PIT_TFLG_REG(PIT,0)
#define PIT_LDVAL1                               PIT_LDVAL_REG(PIT,1)
#define PIT_CVAL1                                PIT_CVAL_REG(PIT,1)
#define PIT_TCTRL1                               PIT_TCTRL_REG(PIT,1)
#define PIT_TFLG1                                PIT_TFLG_REG(PIT,1)
#define PIT_LDVAL2                               PIT_LDVAL_REG(PIT,2)
#define PIT_CVAL2                                PIT_CVAL_REG(PIT,2)
#define PIT_TCTRL2                               PIT_TCTRL_REG(PIT,2)
#define PIT_TFLG2                                PIT_TFLG_REG(PIT,2)
#define PIT_LDVAL3                               PIT_LDVAL_REG(PIT,3)
#define PIT_CVAL3                                PIT_CVAL_REG(PIT,3)
#define PIT_TCTRL3                               PIT_TCTRL_REG(PIT,3)
#define PIT_TFLG3                                PIT_TFLG_REG(PIT,3)

/* PIT - Register array accessors */
#define PIT_LDVAL(index)                         PIT_LDVAL_REG(PIT,index)
#define PIT_CVAL(index)                          PIT_CVAL_REG(PIT,index)
#define PIT_TCTRL(index)                         PIT_TCTRL_REG(PIT,index)
#define PIT_TFLG(index)                          PIT_TFLG_REG(PIT,index)



/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register accessors */
#define PMC_LVDSC1_REG(base)                     ((base)->LVDSC1)
#define PMC_LVDSC2_REG(base)                     ((base)->LVDSC2)
#define PMC_REGSC_REG(base)                      ((base)->REGSC)

/* PMC - Peripheral instance base addresses */
#define PMC_BASE_PTR                             (PMC)


/* PMC - Register instance definitions */
/* PMC */
#define PMC_LVDSC1                               PMC_LVDSC1_REG(PMC)
#define PMC_LVDSC2                               PMC_LVDSC2_REG(PMC)
#define PMC_REGSC                                PMC_REGSC_REG(PMC)

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register accessors */
#define PORT_PCR_REG(base,index)                 ((base)->PCR[index])
//#define PORT_PCR_COUNT                           32
#define PORT_GPCLR_REG(base)                     ((base)->GPCLR)
#define PORT_GPCHR_REG(base)                     ((base)->GPCHR)
#define PORT_ISFR_REG(base)                      ((base)->ISFR)
#define PORT_DFER_REG(base)                      ((base)->DFER)
#define PORT_DFCR_REG(base)                      ((base)->DFCR)
#define PORT_DFWR_REG(base)                      ((base)->DFWR)

/* PORT - Peripheral instance base addresses */
#define PORTA_BASE_PTR                           (PORTA)
#define PORTB_BASE_PTR                           (PORTB)
#define PORTC_BASE_PTR                           (PORTC)
#define PORTD_BASE_PTR                           (PORTD)
#define PORTE_BASE_PTR                           (PORTE)


/* PORT - Register instance definitions */
/* PORTA */
#define PORTA_PCR0                               PORT_PCR_REG(PORTA,0)
#define PORTA_PCR1                               PORT_PCR_REG(PORTA,1)
#define PORTA_PCR2                               PORT_PCR_REG(PORTA,2)
#define PORTA_PCR3                               PORT_PCR_REG(PORTA,3)
#define PORTA_PCR4                               PORT_PCR_REG(PORTA,4)
#define PORTA_PCR5                               PORT_PCR_REG(PORTA,5)
#define PORTA_PCR6                               PORT_PCR_REG(PORTA,6)
#define PORTA_PCR7                               PORT_PCR_REG(PORTA,7)
#define PORTA_PCR8                               PORT_PCR_REG(PORTA,8)
#define PORTA_PCR9                               PORT_PCR_REG(PORTA,9)
#define PORTA_PCR10                              PORT_PCR_REG(PORTA,10)
#define PORTA_PCR11                              PORT_PCR_REG(PORTA,11)
#define PORTA_PCR12                              PORT_PCR_REG(PORTA,12)
#define PORTA_PCR13                              PORT_PCR_REG(PORTA,13)
#define PORTA_PCR14                              PORT_PCR_REG(PORTA,14)
#define PORTA_PCR15                              PORT_PCR_REG(PORTA,15)
#define PORTA_PCR16                              PORT_PCR_REG(PORTA,16)
#define PORTA_PCR17                              PORT_PCR_REG(PORTA,17)
#define PORTA_PCR18                              PORT_PCR_REG(PORTA,18)
#define PORTA_PCR19                              PORT_PCR_REG(PORTA,19)
#define PORTA_PCR20                              PORT_PCR_REG(PORTA,20)
#define PORTA_PCR21                              PORT_PCR_REG(PORTA,21)
#define PORTA_PCR22                              PORT_PCR_REG(PORTA,22)
#define PORTA_PCR23                              PORT_PCR_REG(PORTA,23)
#define PORTA_PCR24                              PORT_PCR_REG(PORTA,24)
#define PORTA_PCR25                              PORT_PCR_REG(PORTA,25)
#define PORTA_PCR26                              PORT_PCR_REG(PORTA,26)
#define PORTA_PCR27                              PORT_PCR_REG(PORTA,27)
#define PORTA_PCR28                              PORT_PCR_REG(PORTA,28)
#define PORTA_PCR29                              PORT_PCR_REG(PORTA,29)
#define PORTA_PCR30                              PORT_PCR_REG(PORTA,30)
#define PORTA_PCR31                              PORT_PCR_REG(PORTA,31)
#define PORTA_GPCLR                              PORT_GPCLR_REG(PORTA)
#define PORTA_GPCHR                              PORT_GPCHR_REG(PORTA)
#define PORTA_ISFR                               PORT_ISFR_REG(PORTA)
/* PORTB */
#define PORTB_PCR0                               PORT_PCR_REG(PORTB,0)
#define PORTB_PCR1                               PORT_PCR_REG(PORTB,1)
#define PORTB_PCR2                               PORT_PCR_REG(PORTB,2)
#define PORTB_PCR3                               PORT_PCR_REG(PORTB,3)
#define PORTB_PCR4                               PORT_PCR_REG(PORTB,4)
#define PORTB_PCR5                               PORT_PCR_REG(PORTB,5)
#define PORTB_PCR6                               PORT_PCR_REG(PORTB,6)
#define PORTB_PCR7                               PORT_PCR_REG(PORTB,7)
#define PORTB_PCR8                               PORT_PCR_REG(PORTB,8)
#define PORTB_PCR9                               PORT_PCR_REG(PORTB,9)
#define PORTB_PCR10                              PORT_PCR_REG(PORTB,10)
#define PORTB_PCR11                              PORT_PCR_REG(PORTB,11)
#define PORTB_PCR12                              PORT_PCR_REG(PORTB,12)
#define PORTB_PCR13                              PORT_PCR_REG(PORTB,13)
#define PORTB_PCR14                              PORT_PCR_REG(PORTB,14)
#define PORTB_PCR15                              PORT_PCR_REG(PORTB,15)
#define PORTB_PCR16                              PORT_PCR_REG(PORTB,16)
#define PORTB_PCR17                              PORT_PCR_REG(PORTB,17)
#define PORTB_PCR18                              PORT_PCR_REG(PORTB,18)
#define PORTB_PCR19                              PORT_PCR_REG(PORTB,19)
#define PORTB_PCR20                              PORT_PCR_REG(PORTB,20)
#define PORTB_PCR21                              PORT_PCR_REG(PORTB,21)
#define PORTB_PCR22                              PORT_PCR_REG(PORTB,22)
#define PORTB_PCR23                              PORT_PCR_REG(PORTB,23)
#define PORTB_PCR24                              PORT_PCR_REG(PORTB,24)
#define PORTB_PCR25                              PORT_PCR_REG(PORTB,25)
#define PORTB_PCR26                              PORT_PCR_REG(PORTB,26)
#define PORTB_PCR27                              PORT_PCR_REG(PORTB,27)
#define PORTB_PCR28                              PORT_PCR_REG(PORTB,28)
#define PORTB_PCR29                              PORT_PCR_REG(PORTB,29)
#define PORTB_PCR30                              PORT_PCR_REG(PORTB,30)
#define PORTB_PCR31                              PORT_PCR_REG(PORTB,31)
#define PORTB_GPCLR                              PORT_GPCLR_REG(PORTB)
#define PORTB_GPCHR                              PORT_GPCHR_REG(PORTB)
#define PORTB_ISFR                               PORT_ISFR_REG(PORTB)
/* PORTC */
#define PORTC_PCR0                               PORT_PCR_REG(PORTC,0)
#define PORTC_PCR1                               PORT_PCR_REG(PORTC,1)
#define PORTC_PCR2                               PORT_PCR_REG(PORTC,2)
#define PORTC_PCR3                               PORT_PCR_REG(PORTC,3)
#define PORTC_PCR4                               PORT_PCR_REG(PORTC,4)
#define PORTC_PCR5                               PORT_PCR_REG(PORTC,5)
#define PORTC_PCR6                               PORT_PCR_REG(PORTC,6)
#define PORTC_PCR7                               PORT_PCR_REG(PORTC,7)
#define PORTC_PCR8                               PORT_PCR_REG(PORTC,8)
#define PORTC_PCR9                               PORT_PCR_REG(PORTC,9)
#define PORTC_PCR10                              PORT_PCR_REG(PORTC,10)
#define PORTC_PCR11                              PORT_PCR_REG(PORTC,11)
#define PORTC_PCR12                              PORT_PCR_REG(PORTC,12)
#define PORTC_PCR13                              PORT_PCR_REG(PORTC,13)
#define PORTC_PCR14                              PORT_PCR_REG(PORTC,14)
#define PORTC_PCR15                              PORT_PCR_REG(PORTC,15)
#define PORTC_PCR16                              PORT_PCR_REG(PORTC,16)
#define PORTC_PCR17                              PORT_PCR_REG(PORTC,17)
#define PORTC_PCR18                              PORT_PCR_REG(PORTC,18)
#define PORTC_PCR19                              PORT_PCR_REG(PORTC,19)
#define PORTC_PCR20                              PORT_PCR_REG(PORTC,20)
#define PORTC_PCR21                              PORT_PCR_REG(PORTC,21)
#define PORTC_PCR22                              PORT_PCR_REG(PORTC,22)
#define PORTC_PCR23                              PORT_PCR_REG(PORTC,23)
#define PORTC_PCR24                              PORT_PCR_REG(PORTC,24)
#define PORTC_PCR25                              PORT_PCR_REG(PORTC,25)
#define PORTC_PCR26                              PORT_PCR_REG(PORTC,26)
#define PORTC_PCR27                              PORT_PCR_REG(PORTC,27)
#define PORTC_PCR28                              PORT_PCR_REG(PORTC,28)
#define PORTC_PCR29                              PORT_PCR_REG(PORTC,29)
#define PORTC_PCR30                              PORT_PCR_REG(PORTC,30)
#define PORTC_PCR31                              PORT_PCR_REG(PORTC,31)
#define PORTC_GPCLR                              PORT_GPCLR_REG(PORTC)
#define PORTC_GPCHR                              PORT_GPCHR_REG(PORTC)
#define PORTC_ISFR                               PORT_ISFR_REG(PORTC)
/* PORTD */
#define PORTD_PCR0                               PORT_PCR_REG(PORTD,0)
#define PORTD_PCR1                               PORT_PCR_REG(PORTD,1)
#define PORTD_PCR2                               PORT_PCR_REG(PORTD,2)
#define PORTD_PCR3                               PORT_PCR_REG(PORTD,3)
#define PORTD_PCR4                               PORT_PCR_REG(PORTD,4)
#define PORTD_PCR5                               PORT_PCR_REG(PORTD,5)
#define PORTD_PCR6                               PORT_PCR_REG(PORTD,6)
#define PORTD_PCR7                               PORT_PCR_REG(PORTD,7)
#define PORTD_PCR8                               PORT_PCR_REG(PORTD,8)
#define PORTD_PCR9                               PORT_PCR_REG(PORTD,9)
#define PORTD_PCR10                              PORT_PCR_REG(PORTD,10)
#define PORTD_PCR11                              PORT_PCR_REG(PORTD,11)
#define PORTD_PCR12                              PORT_PCR_REG(PORTD,12)
#define PORTD_PCR13                              PORT_PCR_REG(PORTD,13)
#define PORTD_PCR14                              PORT_PCR_REG(PORTD,14)
#define PORTD_PCR15                              PORT_PCR_REG(PORTD,15)
#define PORTD_PCR16                              PORT_PCR_REG(PORTD,16)
#define PORTD_PCR17                              PORT_PCR_REG(PORTD,17)
#define PORTD_PCR18                              PORT_PCR_REG(PORTD,18)
#define PORTD_PCR19                              PORT_PCR_REG(PORTD,19)
#define PORTD_PCR20                              PORT_PCR_REG(PORTD,20)
#define PORTD_PCR21                              PORT_PCR_REG(PORTD,21)
#define PORTD_PCR22                              PORT_PCR_REG(PORTD,22)
#define PORTD_PCR23                              PORT_PCR_REG(PORTD,23)
#define PORTD_PCR24                              PORT_PCR_REG(PORTD,24)
#define PORTD_PCR25                              PORT_PCR_REG(PORTD,25)
#define PORTD_PCR26                              PORT_PCR_REG(PORTD,26)
#define PORTD_PCR27                              PORT_PCR_REG(PORTD,27)
#define PORTD_PCR28                              PORT_PCR_REG(PORTD,28)
#define PORTD_PCR29                              PORT_PCR_REG(PORTD,29)
#define PORTD_PCR30                              PORT_PCR_REG(PORTD,30)
#define PORTD_PCR31                              PORT_PCR_REG(PORTD,31)
#define PORTD_GPCLR                              PORT_GPCLR_REG(PORTD)
#define PORTD_GPCHR                              PORT_GPCHR_REG(PORTD)
#define PORTD_ISFR                               PORT_ISFR_REG(PORTD)
#define PORTD_DFER                               PORT_DFER_REG(PORTD)
#define PORTD_DFCR                               PORT_DFCR_REG(PORTD)
#define PORTD_DFWR                               PORT_DFWR_REG(PORTD)
/* PORTE */
#define PORTE_PCR0                               PORT_PCR_REG(PORTE,0)
#define PORTE_PCR1                               PORT_PCR_REG(PORTE,1)
#define PORTE_PCR2                               PORT_PCR_REG(PORTE,2)
#define PORTE_PCR3                               PORT_PCR_REG(PORTE,3)
#define PORTE_PCR4                               PORT_PCR_REG(PORTE,4)
#define PORTE_PCR5                               PORT_PCR_REG(PORTE,5)
#define PORTE_PCR6                               PORT_PCR_REG(PORTE,6)
#define PORTE_PCR7                               PORT_PCR_REG(PORTE,7)
#define PORTE_PCR8                               PORT_PCR_REG(PORTE,8)
#define PORTE_PCR9                               PORT_PCR_REG(PORTE,9)
#define PORTE_PCR10                              PORT_PCR_REG(PORTE,10)
#define PORTE_PCR11                              PORT_PCR_REG(PORTE,11)
#define PORTE_PCR12                              PORT_PCR_REG(PORTE,12)
#define PORTE_PCR13                              PORT_PCR_REG(PORTE,13)
#define PORTE_PCR14                              PORT_PCR_REG(PORTE,14)
#define PORTE_PCR15                              PORT_PCR_REG(PORTE,15)
#define PORTE_PCR16                              PORT_PCR_REG(PORTE,16)
#define PORTE_PCR17                              PORT_PCR_REG(PORTE,17)
#define PORTE_PCR18                              PORT_PCR_REG(PORTE,18)
#define PORTE_PCR19                              PORT_PCR_REG(PORTE,19)
#define PORTE_PCR20                              PORT_PCR_REG(PORTE,20)
#define PORTE_PCR21                              PORT_PCR_REG(PORTE,21)
#define PORTE_PCR22                              PORT_PCR_REG(PORTE,22)
#define PORTE_PCR23                              PORT_PCR_REG(PORTE,23)
#define PORTE_PCR24                              PORT_PCR_REG(PORTE,24)
#define PORTE_PCR25                              PORT_PCR_REG(PORTE,25)
#define PORTE_PCR26                              PORT_PCR_REG(PORTE,26)
#define PORTE_PCR27                              PORT_PCR_REG(PORTE,27)
#define PORTE_PCR28                              PORT_PCR_REG(PORTE,28)
#define PORTE_PCR29                              PORT_PCR_REG(PORTE,29)
#define PORTE_PCR30                              PORT_PCR_REG(PORTE,30)
#define PORTE_PCR31                              PORT_PCR_REG(PORTE,31)
#define PORTE_GPCLR                              PORT_GPCLR_REG(PORTE)
#define PORTE_GPCHR                              PORT_GPCHR_REG(PORTE)
#define PORTE_ISFR                               PORT_ISFR_REG(PORTE)

/* PORT - Register array accessors */
#define PORTA_PCR(index)                         PORT_PCR_REG(PORTA,index)
#define PORTB_PCR(index)                         PORT_PCR_REG(PORTB,index)
#define PORTC_PCR(index)                         PORT_PCR_REG(PORTC,index)
#define PORTD_PCR(index)                         PORT_PCR_REG(PORTD,index)
#define PORTE_PCR(index)                         PORT_PCR_REG(PORTE,index)



/* ----------------------------------------------------------------------------
   -- RCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Accessor_Macros RCM - Register accessor macros
 * @{
 */


/* RCM - Register accessors */
#define RCM_SRS0_REG(base)                       ((base)->SRS0)
#define RCM_SRS1_REG(base)                       ((base)->SRS1)
#define RCM_RPFC_REG(base)                       ((base)->RPFC)
#define RCM_RPFW_REG(base)                       ((base)->RPFW)
#define RCM_MR_REG(base)                         ((base)->MR)

/* RCM - Peripheral instance base addresses */
#define RCM_BASE_PTR                             (RCM)


/* RCM - Register instance definitions */
/* RCM */
#define RCM_SRS0                                 RCM_SRS0_REG(RCM)
#define RCM_SRS1                                 RCM_SRS1_REG(RCM)
#define RCM_RPFC                                 RCM_RPFC_REG(RCM)
#define RCM_RPFW                                 RCM_RPFW_REG(RCM)
#define RCM_MR                                   RCM_MR_REG(RCM)



/* ----------------------------------------------------------------------------
   -- RFSYS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Register_Accessor_Macros RFSYS - Register accessor macros
 * @{
 */


/* RFSYS - Register accessors */
#define RFSYS_REG_REG(base,index)                ((base)->REG[index])


/* RFSYS - Peripheral instance base addresses */
#define RFSYS_BASE_PTR                           (RFSYS)

/* RFSYS - Register instance definitions */
/* RFSYS */
#define RFSYS_REG0                               RFSYS_REG_REG(RFSYS,0)
#define RFSYS_REG1                               RFSYS_REG_REG(RFSYS,1)
#define RFSYS_REG2                               RFSYS_REG_REG(RFSYS,2)
#define RFSYS_REG3                               RFSYS_REG_REG(RFSYS,3)
#define RFSYS_REG4                               RFSYS_REG_REG(RFSYS,4)
#define RFSYS_REG5                               RFSYS_REG_REG(RFSYS,5)
#define RFSYS_REG6                               RFSYS_REG_REG(RFSYS,6)
#define RFSYS_REG7                               RFSYS_REG_REG(RFSYS,7)

/* RFSYS - Register array accessors */
#define RFSYS_REG(index)                         RFSYS_REG_REG(RFSYS,index)


/* ----------------------------------------------------------------------------
   -- RFVBAT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Register_Accessor_Macros RFVBAT - Register accessor macros
 * @{
 */


/* RFVBAT - Register accessors */
#define RFVBAT_REG_REG(base,index)               ((base)->REG[index])

/* RFVBAT - Peripheral instance base addresses */
#define RFVBAT_BASE_PTR                          (RFVBAT)

/* RFVBAT - Register instance definitions */
/* RFVBAT */
#define RFVBAT_REG0                              RFVBAT_REG_REG(RFVBAT,0)
#define RFVBAT_REG1                              RFVBAT_REG_REG(RFVBAT,1)
#define RFVBAT_REG2                              RFVBAT_REG_REG(RFVBAT,2)
#define RFVBAT_REG3                              RFVBAT_REG_REG(RFVBAT,3)
#define RFVBAT_REG4                              RFVBAT_REG_REG(RFVBAT,4)
#define RFVBAT_REG5                              RFVBAT_REG_REG(RFVBAT,5)
#define RFVBAT_REG6                              RFVBAT_REG_REG(RFVBAT,6)
#define RFVBAT_REG7                              RFVBAT_REG_REG(RFVBAT,7)

/* RFVBAT - Register array accessors */
#define RFVBAT_REG(index)                        RFVBAT_REG_REG(RFVBAT,index)


/* ----------------------------------------------------------------------------
   -- RNG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RNG_Register_Accessor_Macros RNG - Register accessor macros
 * @{
 */


/* RNG - Register accessors */
#define RNG_CR_REG(base)                         ((base)->CR)
#define RNG_SR_REG(base)                         ((base)->SR)
#define RNG_ER_REG(base)                         ((base)->ER)
#define RNG_OR_REG(base)                         ((base)->OR)

/* RNG - Peripheral instance base addresses */
#define RNG_BASE_PTR                             (RNG)


/* RNG - Register instance definitions */
/* RNG */
#define RNG_CR                                   RNG_CR_REG(RNG)
#define RNG_SR                                   RNG_SR_REG(RNG)
#define RNG_ER                                   RNG_ER_REG(RNG)
#define RNG_OR                                   RNG_OR_REG(RNG)



/* ----------------------------------------------------------------------------
   -- RTC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Accessor_Macros RTC - Register accessor macros
 * @{
 */


/* RTC - Register accessors */
#define RTC_TSR_REG(base)                        ((base)->TSR)
#define RTC_TPR_REG(base)                        ((base)->TPR)
#define RTC_TAR_REG(base)                        ((base)->TAR)
#define RTC_TCR_REG(base)                        ((base)->TCR)
#define RTC_CR_REG(base)                         ((base)->CR)
#define RTC_SR_REG(base)                         ((base)->SR)
#define RTC_LR_REG(base)                         ((base)->LR)
#define RTC_IER_REG(base)                        ((base)->IER)
#define RTC_WAR_REG(base)                        ((base)->WAR)
#define RTC_RAR_REG(base)                        ((base)->RAR)


/* RTC - Peripheral instance base addresses */
#define RTC_BASE_PTR                             (RTC)

/* RTC - Register instance definitions */
/* RTC */
#define RTC_TSR                                  RTC_TSR_REG(RTC)
#define RTC_TPR                                  RTC_TPR_REG(RTC)
#define RTC_TAR                                  RTC_TAR_REG(RTC)
#define RTC_TCR                                  RTC_TCR_REG(RTC)
#define RTC_CR                                   RTC_CR_REG(RTC)
#define RTC_SR                                   RTC_SR_REG(RTC)
#define RTC_LR                                   RTC_LR_REG(RTC)
#define RTC_IER                                  RTC_IER_REG(RTC)
#define RTC_WAR                                  RTC_WAR_REG(RTC)
#define RTC_RAR                                  RTC_RAR_REG(RTC)



/* ----------------------------------------------------------------------------
   -- SDHC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SDHC_Register_Accessor_Macros SDHC - Register accessor macros
 * @{
 */


/* SDHC - Register accessors */
#define SDHC_DSADDR_REG(base)                    ((base)->DSADDR)
#define SDHC_BLKATTR_REG(base)                   ((base)->BLKATTR)
#define SDHC_CMDARG_REG(base)                    ((base)->CMDARG)
#define SDHC_XFERTYP_REG(base)                   ((base)->XFERTYP)
#define SDHC_CMDRSP_REG(base,index)              ((base)->CMDRSP[index])
#define SDHC_DATPORT_REG(base)                   ((base)->DATPORT)
#define SDHC_PRSSTAT_REG(base)                   ((base)->PRSSTAT)
#define SDHC_PROCTL_REG(base)                    ((base)->PROCTL)
#define SDHC_SYSCTL_REG(base)                    ((base)->SYSCTL)
#define SDHC_IRQSTAT_REG(base)                   ((base)->IRQSTAT)
#define SDHC_IRQSTATEN_REG(base)                 ((base)->IRQSTATEN)
#define SDHC_IRQSIGEN_REG(base)                  ((base)->IRQSIGEN)
#define SDHC_AC12ERR_REG(base)                   ((base)->AC12ERR)
#define SDHC_HTCAPBLT_REG(base)                  ((base)->HTCAPBLT)
#define SDHC_WML_REG(base)                       ((base)->WML)
#define SDHC_FEVT_REG(base)                      ((base)->FEVT)
#define SDHC_ADMAES_REG(base)                    ((base)->ADMAES)
#define SDHC_ADSADDR_REG(base)                   ((base)->ADSADDR)
#define SDHC_VENDOR_REG(base)                    ((base)->VENDOR)
#define SDHC_MMCBOOT_REG(base)                   ((base)->MMCBOOT)
#define SDHC_HOSTVER_REG(base)                   ((base)->HOSTVER)

/* SDHC - Peripheral instance base addresses */
#define SDHC_BASE_PTR                            (SDHC)

/* SDHC - Register instance definitions */
/* SDHC */
#define SDHC_DSADDR                              SDHC_DSADDR_REG(SDHC)
#define SDHC_BLKATTR                             SDHC_BLKATTR_REG(SDHC)
#define SDHC_CMDARG                              SDHC_CMDARG_REG(SDHC)
#define SDHC_XFERTYP                             SDHC_XFERTYP_REG(SDHC)
#define SDHC_CMDRSP0                             SDHC_CMDRSP_REG(SDHC,0)
#define SDHC_CMDRSP1                             SDHC_CMDRSP_REG(SDHC,1)
#define SDHC_CMDRSP2                             SDHC_CMDRSP_REG(SDHC,2)
#define SDHC_CMDRSP3                             SDHC_CMDRSP_REG(SDHC,3)
#define SDHC_DATPORT                             SDHC_DATPORT_REG(SDHC)
#define SDHC_PRSSTAT                             SDHC_PRSSTAT_REG(SDHC)
#define SDHC_PROCTL                              SDHC_PROCTL_REG(SDHC)
#define SDHC_SYSCTL                              SDHC_SYSCTL_REG(SDHC)
#define SDHC_IRQSTAT                             SDHC_IRQSTAT_REG(SDHC)
#define SDHC_IRQSTATEN                           SDHC_IRQSTATEN_REG(SDHC)
#define SDHC_IRQSIGEN                            SDHC_IRQSIGEN_REG(SDHC)
#define SDHC_AC12ERR                             SDHC_AC12ERR_REG(SDHC)
#define SDHC_HTCAPBLT                            SDHC_HTCAPBLT_REG(SDHC)
#define SDHC_WML                                 SDHC_WML_REG(SDHC)
#define SDHC_FEVT                                SDHC_FEVT_REG(SDHC)
#define SDHC_ADMAES                              SDHC_ADMAES_REG(SDHC)
#define SDHC_ADSADDR                             SDHC_ADSADDR_REG(SDHC)
#define SDHC_VENDOR                              SDHC_VENDOR_REG(SDHC)
#define SDHC_MMCBOOT                             SDHC_MMCBOOT_REG(SDHC)
#define SDHC_HOSTVER                             SDHC_HOSTVER_REG(SDHC)

/* SDHC - Register array accessors */
#define SDHC_CMDRSP(index)                       SDHC_CMDRSP_REG(SDHC,index)

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */
#define SIM_BASE_PTR                             (SIM)

/* SIM - Register accessors */
#define SIM_SOPT1_REG(base)                      ((base)->SOPT1)
#define SIM_SOPT1CFG_REG(base)                   ((base)->SOPT1CFG)
#define SIM_SOPT2_REG(base)                      ((base)->SOPT2)
#define SIM_SOPT4_REG(base)                      ((base)->SOPT4)
#define SIM_SOPT5_REG(base)                      ((base)->SOPT5)
#define SIM_SOPT7_REG(base)                      ((base)->SOPT7)
#define SIM_SDID_REG(base)                       ((base)->SDID)
#define SIM_SCGC1_REG(base)                      ((base)->SCGC1)
#define SIM_SCGC2_REG(base)                      ((base)->SCGC2)
#define SIM_SCGC3_REG(base)                      ((base)->SCGC3)
#define SIM_SCGC4_REG(base)                      ((base)->SCGC4)
#define SIM_SCGC5_REG(base)                      ((base)->SCGC5)
#define SIM_SCGC6_REG(base)                      ((base)->SCGC6)
#define SIM_SCGC7_REG(base)                      ((base)->SCGC7)
#define SIM_CLKDIV1_REG(base)                    ((base)->CLKDIV1)
#define SIM_CLKDIV2_REG(base)                    ((base)->CLKDIV2)
#define SIM_FCFG1_REG(base)                      ((base)->FCFG1)
#define SIM_FCFG2_REG(base)                      ((base)->FCFG2)
#define SIM_UIDH_REG(base)                       ((base)->UIDH)
#define SIM_UIDMH_REG(base)                      ((base)->UIDMH)
#define SIM_UIDML_REG(base)                      ((base)->UIDML)
#define SIM_UIDL_REG(base)                       ((base)->UIDL)


/* SIM - Register instance definitions */
/* SIM */
#define SIM_SOPT1                                SIM_SOPT1_REG(SIM)
#define SIM_SOPT1CFG                             SIM_SOPT1CFG_REG(SIM)
#define SIM_SOPT2                                SIM_SOPT2_REG(SIM)
#define SIM_SOPT4                                SIM_SOPT4_REG(SIM)
#define SIM_SOPT5                                SIM_SOPT5_REG(SIM)
#define SIM_SOPT7                                SIM_SOPT7_REG(SIM)
#define SIM_SDID                                 SIM_SDID_REG(SIM)
#define SIM_SCGC1                                SIM_SCGC1_REG(SIM)
#define SIM_SCGC2                                SIM_SCGC2_REG(SIM)
#define SIM_SCGC3                                SIM_SCGC3_REG(SIM)
#define SIM_SCGC4                                SIM_SCGC4_REG(SIM)
#define SIM_SCGC5                                SIM_SCGC5_REG(SIM)
#define SIM_SCGC6                                SIM_SCGC6_REG(SIM)
#define SIM_SCGC7                                SIM_SCGC7_REG(SIM)
#define SIM_CLKDIV1                              SIM_CLKDIV1_REG(SIM)
#define SIM_CLKDIV2                              SIM_CLKDIV2_REG(SIM)
#define SIM_FCFG1                                SIM_FCFG1_REG(SIM)
#define SIM_FCFG2                                SIM_FCFG2_REG(SIM)
#define SIM_UIDH                                 SIM_UIDH_REG(SIM)
#define SIM_UIDMH                                SIM_UIDMH_REG(SIM)
#define SIM_UIDML                                SIM_UIDML_REG(SIM)
#define SIM_UIDL                                 SIM_UIDL_REG(SIM)


/* ----------------------------------------------------------------------------
   -- SMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Accessor_Macros SMC - Register accessor macros
 * @{
 */


/* SMC - Register accessors */
#define SMC_PMPROT_REG(base)                     ((base)->PMPROT)
#define SMC_PMCTRL_REG(base)                     ((base)->PMCTRL)
#define SMC_VLLSCTRL_REG(base)                   ((base)->VLLSCTRL)
#define SMC_PMSTAT_REG(base)                     ((base)->PMSTAT)

/* SMC - Peripheral instance base addresses */
#define SMC_BASE_PTR                             (SMC)


/* SMC - Register instance definitions */
/* SMC */
#define SMC_PMPROT                               SMC_PMPROT_REG(SMC)
#define SMC_PMCTRL                               SMC_PMCTRL_REG(SMC)
#define SMC_VLLSCTRL                             SMC_VLLSCTRL_REG(SMC)
#define SMC_PMSTAT                               SMC_PMSTAT_REG(SMC)

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register accessors */
#define SPI_MCR_REG(base)                        ((base)->MCR)
#define SPI_TCR_REG(base)                        ((base)->TCR)
#define SPI_CTAR_REG(base,index2)                ((base)->CTAR[index2])
#define SPI_CTAR_SLAVE_REG(base,index2)          ((base)->CTAR_SLAVE[index2])
#define SPI_SR_REG(base)                         ((base)->SR)
#define SPI_RSER_REG(base)                       ((base)->RSER)
#define SPI_PUSHR_REG(base)                      ((base)->PUSHR)
#define SPI_PUSHR_SLAVE_REG(base)                ((base)->PUSHR_SLAVE)
#define SPI_POPR_REG(base)                       ((base)->POPR)
#define SPI_TXFR0_REG(base)                      ((base)->TXFR0)
#define SPI_TXFR1_REG(base)                      ((base)->TXFR1)
#define SPI_TXFR2_REG(base)                      ((base)->TXFR2)
#define SPI_TXFR3_REG(base)                      ((base)->TXFR3)
#define SPI_RXFR0_REG(base)                      ((base)->RXFR0)
#define SPI_RXFR1_REG(base)                      ((base)->RXFR1)
#define SPI_RXFR2_REG(base)                      ((base)->RXFR2)
#define SPI_RXFR3_REG(base)                      ((base)->RXFR3)


/* SPI - Peripheral instance base addresses */
#define SPI0_BASE_PTR                            (SPI0)
#define SPI1_BASE_PTR                            (SPI1)
#define SPI2_BASE_PTR                            (SPI2)


/* SPI - Register instance definitions */
/* SPI0 */
#define SPI0_MCR                                 SPI_MCR_REG(SPI0)
#define SPI0_TCR                                 SPI_TCR_REG(SPI0)
#define SPI0_CTAR0                               SPI_CTAR_REG(SPI0,0)
#define SPI0_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI0,0)
#define SPI0_CTAR1                               SPI_CTAR_REG(SPI0,1)
#define SPI0_SR                                  SPI_SR_REG(SPI0)
#define SPI0_RSER                                SPI_RSER_REG(SPI0)
#define SPI0_PUSHR                               SPI_PUSHR_REG(SPI0)
#define SPI0_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI0)
#define SPI0_POPR                                SPI_POPR_REG(SPI0)
#define SPI0_TXFR0                               SPI_TXFR0_REG(SPI0)
#define SPI0_TXFR1                               SPI_TXFR1_REG(SPI0)
#define SPI0_TXFR2                               SPI_TXFR2_REG(SPI0)
#define SPI0_TXFR3                               SPI_TXFR3_REG(SPI0)
#define SPI0_RXFR0                               SPI_RXFR0_REG(SPI0)
#define SPI0_RXFR1                               SPI_RXFR1_REG(SPI0)
#define SPI0_RXFR2                               SPI_RXFR2_REG(SPI0)
#define SPI0_RXFR3                               SPI_RXFR3_REG(SPI0)
/* SPI1 */
#define SPI1_MCR                                 SPI_MCR_REG(SPI1)
#define SPI1_TCR                                 SPI_TCR_REG(SPI1)
#define SPI1_CTAR0                               SPI_CTAR_REG(SPI1,0)
#define SPI1_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI1,0)
#define SPI1_CTAR1                               SPI_CTAR_REG(SPI1,1)
#define SPI1_SR                                  SPI_SR_REG(SPI1)
#define SPI1_RSER                                SPI_RSER_REG(SPI1)
#define SPI1_PUSHR                               SPI_PUSHR_REG(SPI1)
#define SPI1_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI1)
#define SPI1_POPR                                SPI_POPR_REG(SPI1)
#define SPI1_TXFR0                               SPI_TXFR0_REG(SPI1)
#define SPI1_TXFR1                               SPI_TXFR1_REG(SPI1)
#define SPI1_TXFR2                               SPI_TXFR2_REG(SPI1)
#define SPI1_TXFR3                               SPI_TXFR3_REG(SPI1)
#define SPI1_RXFR0                               SPI_RXFR0_REG(SPI1)
#define SPI1_RXFR1                               SPI_RXFR1_REG(SPI1)
#define SPI1_RXFR2                               SPI_RXFR2_REG(SPI1)
#define SPI1_RXFR3                               SPI_RXFR3_REG(SPI1)
/* SPI2 */
#define SPI2_MCR                                 SPI_MCR_REG(SPI2)
#define SPI2_TCR                                 SPI_TCR_REG(SPI2)
#define SPI2_CTAR0                               SPI_CTAR_REG(SPI2,0)
#define SPI2_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI2,0)
#define SPI2_CTAR1                               SPI_CTAR_REG(SPI2,1)
#define SPI2_SR                                  SPI_SR_REG(SPI2)
#define SPI2_RSER                                SPI_RSER_REG(SPI2)
#define SPI2_PUSHR                               SPI_PUSHR_REG(SPI2)
#define SPI2_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI2)
#define SPI2_POPR                                SPI_POPR_REG(SPI2)
#define SPI2_TXFR0                               SPI_TXFR0_REG(SPI2)
#define SPI2_TXFR1                               SPI_TXFR1_REG(SPI2)
#define SPI2_TXFR2                               SPI_TXFR2_REG(SPI2)
#define SPI2_TXFR3                               SPI_TXFR3_REG(SPI2)
#define SPI2_RXFR0                               SPI_RXFR0_REG(SPI2)
#define SPI2_RXFR1                               SPI_RXFR1_REG(SPI2)
#define SPI2_RXFR2                               SPI_RXFR2_REG(SPI2)
#define SPI2_RXFR3                               SPI_RXFR3_REG(SPI2)

/* SPI - Register array accessors */
#define SPI0_CTAR(index2)                        SPI_CTAR_REG(SPI0,index2)
#define SPI1_CTAR(index2)                        SPI_CTAR_REG(SPI1,index2)
#define SPI2_CTAR(index2)                        SPI_CTAR_REG(SPI2,index2)
#define SPI0_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI0,index2)
#define SPI1_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI1,index2)
#define SPI2_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI2,index2)



/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */


/* UART - Register accessors */
#define UART_BDH_REG(base)                       ((base)->BDH)
#define UART_BDL_REG(base)                       ((base)->BDL)
#define UART_C1_REG(base)                        ((base)->C1)
#define UART_C2_REG(base)                        ((base)->C2)
#define UART_S1_REG(base)                        ((base)->S1)
#define UART_S2_REG(base)                        ((base)->S2)
#define UART_C3_REG(base)                        ((base)->C3)
#define UART_D_REG(base)                         ((base)->D)
#define UART_MA1_REG(base)                       ((base)->MA1)
#define UART_MA2_REG(base)                       ((base)->MA2)
#define UART_C4_REG(base)                        ((base)->C4)
#define UART_C5_REG(base)                        ((base)->C5)
#define UART_ED_REG(base)                        ((base)->ED)
#define UART_MODEM_REG(base)                     ((base)->MODEM)
#define UART_IR_REG(base)                        ((base)->IR)
#define UART_PFIFO_REG(base)                     ((base)->PFIFO)
#define UART_CFIFO_REG(base)                     ((base)->CFIFO)
#define UART_SFIFO_REG(base)                     ((base)->SFIFO)
#define UART_TWFIFO_REG(base)                    ((base)->TWFIFO)
#define UART_TCFIFO_REG(base)                    ((base)->TCFIFO)
#define UART_RWFIFO_REG(base)                    ((base)->RWFIFO)
#define UART_RCFIFO_REG(base)                    ((base)->RCFIFO)
#define UART_C7816_REG(base)                     ((base)->C7816)
#define UART_IE7816_REG(base)                    ((base)->IE7816)
#define UART_IS7816_REG(base)                    ((base)->IS7816)
#define UART_WP7816T0_REG(base)                  ((base)->WP7816T0)
#define UART_WP7816T1_REG(base)                  ((base)->WP7816T1)
#define UART_WN7816_REG(base)                    ((base)->WN7816)
#define UART_WF7816_REG(base)                    ((base)->WF7816)
#define UART_ET7816_REG(base)                    ((base)->ET7816)
#define UART_TL7816_REG(base)                    ((base)->TL7816)

/* UART - Peripheral instance base addresses */
#define UART0_BASE_PTR                           (UART0)
#define UART1_BASE_PTR                           (UART1)
#define UART2_BASE_PTR                           (UART2)
#define UART3_BASE_PTR                           (UART3)
#define UART4_BASE_PTR                           (UART4)
#define UART5_BASE_PTR                           (UART5)


/* UART - Register instance definitions */
/* UART0 */
#define UART0_BDH                                UART_BDH_REG(UART0)
#define UART0_BDL                                UART_BDL_REG(UART0)
#define UART0_C1                                 UART_C1_REG(UART0)
#define UART0_C2                                 UART_C2_REG(UART0)
#define UART0_S1                                 UART_S1_REG(UART0)
#define UART0_S2                                 UART_S2_REG(UART0)
#define UART0_C3                                 UART_C3_REG(UART0)
#define UART0_D                                  UART_D_REG(UART0)
#define UART0_MA1                                UART_MA1_REG(UART0)
#define UART0_MA2                                UART_MA2_REG(UART0)
#define UART0_C4                                 UART_C4_REG(UART0)
#define UART0_C5                                 UART_C5_REG(UART0)
#define UART0_ED                                 UART_ED_REG(UART0)
#define UART0_MODEM                              UART_MODEM_REG(UART0)
#define UART0_IR                                 UART_IR_REG(UART0)
#define UART0_PFIFO                              UART_PFIFO_REG(UART0)
#define UART0_CFIFO                              UART_CFIFO_REG(UART0)
#define UART0_SFIFO                              UART_SFIFO_REG(UART0)
#define UART0_TWFIFO                             UART_TWFIFO_REG(UART0)
#define UART0_TCFIFO                             UART_TCFIFO_REG(UART0)
#define UART0_RWFIFO                             UART_RWFIFO_REG(UART0)
#define UART0_RCFIFO                             UART_RCFIFO_REG(UART0)
#define UART0_C7816                              UART_C7816_REG(UART0)
#define UART0_IE7816                             UART_IE7816_REG(UART0)
#define UART0_IS7816                             UART_IS7816_REG(UART0)
#define UART0_WP7816T0                           UART_WP7816T0_REG(UART0)
#define UART0_WP7816T1                           UART_WP7816T1_REG(UART0)
#define UART0_WN7816                             UART_WN7816_REG(UART0)
#define UART0_WF7816                             UART_WF7816_REG(UART0)
#define UART0_ET7816                             UART_ET7816_REG(UART0)
#define UART0_TL7816                             UART_TL7816_REG(UART0)
/* UART1 */
#define UART1_BDH                                UART_BDH_REG(UART1)
#define UART1_BDL                                UART_BDL_REG(UART1)
#define UART1_C1                                 UART_C1_REG(UART1)
#define UART1_C2                                 UART_C2_REG(UART1)
#define UART1_S1                                 UART_S1_REG(UART1)
#define UART1_S2                                 UART_S2_REG(UART1)
#define UART1_C3                                 UART_C3_REG(UART1)
#define UART1_D                                  UART_D_REG(UART1)
#define UART1_MA1                                UART_MA1_REG(UART1)
#define UART1_MA2                                UART_MA2_REG(UART1)
#define UART1_C4                                 UART_C4_REG(UART1)
#define UART1_C5                                 UART_C5_REG(UART1)
#define UART1_ED                                 UART_ED_REG(UART1)
#define UART1_MODEM                              UART_MODEM_REG(UART1)
#define UART1_IR                                 UART_IR_REG(UART1)
#define UART1_PFIFO                              UART_PFIFO_REG(UART1)
#define UART1_CFIFO                              UART_CFIFO_REG(UART1)
#define UART1_SFIFO                              UART_SFIFO_REG(UART1)
#define UART1_TWFIFO                             UART_TWFIFO_REG(UART1)
#define UART1_TCFIFO                             UART_TCFIFO_REG(UART1)
#define UART1_RWFIFO                             UART_RWFIFO_REG(UART1)
#define UART1_RCFIFO                             UART_RCFIFO_REG(UART1)
/* UART2 */
#define UART2_BDH                                UART_BDH_REG(UART2)
#define UART2_BDL                                UART_BDL_REG(UART2)
#define UART2_C1                                 UART_C1_REG(UART2)
#define UART2_C2                                 UART_C2_REG(UART2)
#define UART2_S1                                 UART_S1_REG(UART2)
#define UART2_S2                                 UART_S2_REG(UART2)
#define UART2_C3                                 UART_C3_REG(UART2)
#define UART2_D                                  UART_D_REG(UART2)
#define UART2_MA1                                UART_MA1_REG(UART2)
#define UART2_MA2                                UART_MA2_REG(UART2)
#define UART2_C4                                 UART_C4_REG(UART2)
#define UART2_C5                                 UART_C5_REG(UART2)
#define UART2_ED                                 UART_ED_REG(UART2)
#define UART2_MODEM                              UART_MODEM_REG(UART2)
#define UART2_IR                                 UART_IR_REG(UART2)
#define UART2_PFIFO                              UART_PFIFO_REG(UART2)
#define UART2_CFIFO                              UART_CFIFO_REG(UART2)
#define UART2_SFIFO                              UART_SFIFO_REG(UART2)
#define UART2_TWFIFO                             UART_TWFIFO_REG(UART2)
#define UART2_TCFIFO                             UART_TCFIFO_REG(UART2)
#define UART2_RWFIFO                             UART_RWFIFO_REG(UART2)
#define UART2_RCFIFO                             UART_RCFIFO_REG(UART2)
/* UART3 */
#define UART3_BDH                                UART_BDH_REG(UART3)
#define UART3_BDL                                UART_BDL_REG(UART3)
#define UART3_C1                                 UART_C1_REG(UART3)
#define UART3_C2                                 UART_C2_REG(UART3)
#define UART3_S1                                 UART_S1_REG(UART3)
#define UART3_S2                                 UART_S2_REG(UART3)
#define UART3_C3                                 UART_C3_REG(UART3)
#define UART3_D                                  UART_D_REG(UART3)
#define UART3_MA1                                UART_MA1_REG(UART3)
#define UART3_MA2                                UART_MA2_REG(UART3)
#define UART3_C4                                 UART_C4_REG(UART3)
#define UART3_C5                                 UART_C5_REG(UART3)
#define UART3_ED                                 UART_ED_REG(UART3)
#define UART3_MODEM                              UART_MODEM_REG(UART3)
#define UART3_IR                                 UART_IR_REG(UART3)
#define UART3_PFIFO                              UART_PFIFO_REG(UART3)
#define UART3_CFIFO                              UART_CFIFO_REG(UART3)
#define UART3_SFIFO                              UART_SFIFO_REG(UART3)
#define UART3_TWFIFO                             UART_TWFIFO_REG(UART3)
#define UART3_TCFIFO                             UART_TCFIFO_REG(UART3)
#define UART3_RWFIFO                             UART_RWFIFO_REG(UART3)
#define UART3_RCFIFO                             UART_RCFIFO_REG(UART3)
/* UART4 */
#define UART4_BDH                                UART_BDH_REG(UART4)
#define UART4_BDL                                UART_BDL_REG(UART4)
#define UART4_C1                                 UART_C1_REG(UART4)
#define UART4_C2                                 UART_C2_REG(UART4)
#define UART4_S1                                 UART_S1_REG(UART4)
#define UART4_S2                                 UART_S2_REG(UART4)
#define UART4_C3                                 UART_C3_REG(UART4)
#define UART4_D                                  UART_D_REG(UART4)
#define UART4_MA1                                UART_MA1_REG(UART4)
#define UART4_MA2                                UART_MA2_REG(UART4)
#define UART4_C4                                 UART_C4_REG(UART4)
#define UART4_C5                                 UART_C5_REG(UART4)
#define UART4_ED                                 UART_ED_REG(UART4)
#define UART4_MODEM                              UART_MODEM_REG(UART4)
#define UART4_IR                                 UART_IR_REG(UART4)
#define UART4_PFIFO                              UART_PFIFO_REG(UART4)
#define UART4_CFIFO                              UART_CFIFO_REG(UART4)
#define UART4_SFIFO                              UART_SFIFO_REG(UART4)
#define UART4_TWFIFO                             UART_TWFIFO_REG(UART4)
#define UART4_TCFIFO                             UART_TCFIFO_REG(UART4)
#define UART4_RWFIFO                             UART_RWFIFO_REG(UART4)
#define UART4_RCFIFO                             UART_RCFIFO_REG(UART4)
/* UART5 */
#define UART5_BDH                                UART_BDH_REG(UART5)
#define UART5_BDL                                UART_BDL_REG(UART5)
#define UART5_C1                                 UART_C1_REG(UART5)
#define UART5_C2                                 UART_C2_REG(UART5)
#define UART5_S1                                 UART_S1_REG(UART5)
#define UART5_S2                                 UART_S2_REG(UART5)
#define UART5_C3                                 UART_C3_REG(UART5)
#define UART5_D                                  UART_D_REG(UART5)
#define UART5_MA1                                UART_MA1_REG(UART5)
#define UART5_MA2                                UART_MA2_REG(UART5)
#define UART5_C4                                 UART_C4_REG(UART5)
#define UART5_C5                                 UART_C5_REG(UART5)
#define UART5_ED                                 UART_ED_REG(UART5)
#define UART5_MODEM                              UART_MODEM_REG(UART5)
#define UART5_IR                                 UART_IR_REG(UART5)
#define UART5_PFIFO                              UART_PFIFO_REG(UART5)
#define UART5_CFIFO                              UART_CFIFO_REG(UART5)
#define UART5_SFIFO                              UART_SFIFO_REG(UART5)
#define UART5_TWFIFO                             UART_TWFIFO_REG(UART5)
#define UART5_TCFIFO                             UART_TCFIFO_REG(UART5)
#define UART5_RWFIFO                             UART_RWFIFO_REG(UART5)
#define UART5_RCFIFO                             UART_RCFIFO_REG(UART5)

/* ----------------------------------------------------------------------------
   -- USB - Register accessor macros
   ---------------------------------------------------------------------------- */
/*!
 * @addtogroup USB_Register_Accessor_Macros USB - Register accessor macros
 * @{
 */


/* USB - Register accessors */
#define USB_PERID_REG(base)                      ((base)->PERID)
#define USB_IDCOMP_REG(base)                     ((base)->IDCOMP)
#define USB_REV_REG(base)                        ((base)->REV)
#define USB_ADDINFO_REG(base)                    ((base)->ADDINFO)
#define USB_OTGISTAT_REG(base)                   ((base)->OTGISTAT)
#define USB_OTGICR_REG(base)                     ((base)->OTGICR)
#define USB_OTGSTAT_REG(base)                    ((base)->OTGSTAT)
#define USB_OTGCTL_REG(base)                     ((base)->OTGCTL)
#define USB_ISTAT_REG(base)                      ((base)->ISTAT)
#define USB_INTEN_REG(base)                      ((base)->INTEN)
#define USB_ERRSTAT_REG(base)                    ((base)->ERRSTAT)
#define USB_ERREN_REG(base)                      ((base)->ERREN)
#define USB_STAT_REG(base)                       ((base)->STAT)
#define USB_CTL_REG(base)                        ((base)->CTL)
#define USB_ADDR_REG(base)                       ((base)->ADDR)
#define USB_BDTPAGE1_REG(base)                   ((base)->BDTPAGE1)
#define USB_FRMNUML_REG(base)                    ((base)->FRMNUML)
#define USB_FRMNUMH_REG(base)                    ((base)->FRMNUMH)
#define USB_TOKEN_REG(base)                      ((base)->TOKEN)
#define USB_SOFTHLD_REG(base)                    ((base)->SOFTHLD)
#define USB_BDTPAGE2_REG(base)                   ((base)->BDTPAGE2)
#define USB_BDTPAGE3_REG(base)                   ((base)->BDTPAGE3)
#define USB_ENDPT_REG(base,index)                ((base)->ENDPOINT[index].ENDPT)
//#define USB_ENDPT_COUNT                          16
#define USB_USBCTRL_REG(base)                    ((base)->USBCTRL)
#define USB_OBSERVE_REG(base)                    ((base)->OBSERVE)
#define USB_CONTROL_REG(base)                    ((base)->CONTROL)
#define USB_USBTRC0_REG(base)                    ((base)->USBTRC0)
#define USB_USBFRMADJUST_REG(base)               ((base)->USBFRMADJUST)
#define USB_CLK_RECOVER_CTRL_REG(base)           ((base)->CLK_RECOVER_CTRL)
#define USB_CLK_RECOVER_IRC_EN_REG(base)         ((base)->CLK_RECOVER_IRC_EN)
#define USB_CLK_RECOVER_INT_STATUS_REG(base)     ((base)->CLK_RECOVER_INT_STATUS)

/*!
 * @addtogroup USB_Register_Accessor_Macros USB - Register accessor macros
 * @{
 */


/* USB - Register instance definitions */
/* USB0 */
#define USB0_PERID                               USB_PERID_REG(USB0)
#define USB0_IDCOMP                              USB_IDCOMP_REG(USB0)
#define USB0_REV                                 USB_REV_REG(USB0)
#define USB0_ADDINFO                             USB_ADDINFO_REG(USB0)
#define USB0_OTGISTAT                            USB_OTGISTAT_REG(USB0)
#define USB0_OTGICR                              USB_OTGICR_REG(USB0)
#define USB0_OTGSTAT                             USB_OTGSTAT_REG(USB0)
#define USB0_OTGCTL                              USB_OTGCTL_REG(USB0)
#define USB0_ISTAT                               USB_ISTAT_REG(USB0)
#define USB0_INTEN                               USB_INTEN_REG(USB0)
#define USB0_ERRSTAT                             USB_ERRSTAT_REG(USB0)
#define USB0_ERREN                               USB_ERREN_REG(USB0)
#define USB0_STAT                                USB_STAT_REG(USB0)
#define USB0_CTL                                 USB_CTL_REG(USB0)
#define USB0_ADDR                                USB_ADDR_REG(USB0)
#define USB0_BDTPAGE1                            USB_BDTPAGE1_REG(USB0)
#define USB0_FRMNUML                             USB_FRMNUML_REG(USB0)
#define USB0_FRMNUMH                             USB_FRMNUMH_REG(USB0)
#define USB0_TOKEN                               USB_TOKEN_REG(USB0)
#define USB0_SOFTHLD                             USB_SOFTHLD_REG(USB0)
#define USB0_BDTPAGE2                            USB_BDTPAGE2_REG(USB0)
#define USB0_BDTPAGE3                            USB_BDTPAGE3_REG(USB0)
#define USB0_ENDPT0                              USB_ENDPT_REG(USB0,0)
#define USB0_ENDPT1                              USB_ENDPT_REG(USB0,1)
#define USB0_ENDPT2                              USB_ENDPT_REG(USB0,2)
#define USB0_ENDPT3                              USB_ENDPT_REG(USB0,3)
#define USB0_ENDPT4                              USB_ENDPT_REG(USB0,4)
#define USB0_ENDPT5                              USB_ENDPT_REG(USB0,5)
#define USB0_ENDPT6                              USB_ENDPT_REG(USB0,6)
#define USB0_ENDPT7                              USB_ENDPT_REG(USB0,7)
#define USB0_ENDPT8                              USB_ENDPT_REG(USB0,8)
#define USB0_ENDPT9                              USB_ENDPT_REG(USB0,9)
#define USB0_ENDPT10                             USB_ENDPT_REG(USB0,10)
#define USB0_ENDPT11                             USB_ENDPT_REG(USB0,11)
#define USB0_ENDPT12                             USB_ENDPT_REG(USB0,12)
#define USB0_ENDPT13                             USB_ENDPT_REG(USB0,13)
#define USB0_ENDPT14                             USB_ENDPT_REG(USB0,14)
#define USB0_ENDPT15                             USB_ENDPT_REG(USB0,15)
#define USB0_USBCTRL                             USB_USBCTRL_REG(USB0)
#define USB0_OBSERVE                             USB_OBSERVE_REG(USB0)
#define USB0_CONTROL                             USB_CONTROL_REG(USB0)
#define USB0_USBTRC0                             USB_USBTRC0_REG(USB0)
#define USB0_USBFRMADJUST                        USB_USBFRMADJUST_REG(USB0)
#define USB0_CLK_RECOVER_CTRL                    USB_CLK_RECOVER_CTRL_REG(USB0)
#define USB0_CLK_RECOVER_IRC_EN                  USB_CLK_RECOVER_IRC_EN_REG(USB0)
#define USB0_CLK_RECOVER_INT_STATUS              USB_CLK_RECOVER_INT_STATUS_REG(USB0)

/* USB - Register array accessors */
#define USB0_ENDPT(index)                        USB_ENDPT_REG(USB0,index)



/* ----------------------------------------------------------------------------
   -- USBDCD - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USBDCD_Register_Accessor_Macros USBDCD - Register accessor macros
 * @{
 */


/* USBDCD - Register accessors */
#define USBDCD_CONTROL_REG(base)                 ((base)->CONTROL)
#define USBDCD_CLOCK_REG(base)                   ((base)->CLOCK)
#define USBDCD_STATUS_REG(base)                  ((base)->STATUS)
#define USBDCD_TIMER0_REG(base)                  ((base)->TIMER0)
#define USBDCD_TIMER1_REG(base)                  ((base)->TIMER1)
#define USBDCD_TIMER2_BC11_REG(base)             ((base)->TIMER2_BC11)
#define USBDCD_TIMER2_BC12_REG(base)             ((base)->TIMER2_BC12)

/* USBDCD - Peripheral instance base addresses */
#define USBDCD_BASE_PTR                          (USBDCD)


/* USBDCD - Register instance definitions */
/* USBDCD */
#define USBDCD_CONTROL                           USBDCD_CONTROL_REG(USBDCD)
#define USBDCD_CLOCK                             USBDCD_CLOCK_REG(USBDCD)
#define USBDCD_STATUS                            USBDCD_STATUS_REG(USBDCD)
#define USBDCD_TIMER0                            USBDCD_TIMER0_REG(USBDCD)
#define USBDCD_TIMER1                            USBDCD_TIMER1_REG(USBDCD)
#define USBDCD_TIMER2_BC11                       USBDCD_TIMER2_BC11_REG(USBDCD)
#define USBDCD_TIMER2_BC12                       USBDCD_TIMER2_BC12_REG(USBDCD)



/* ----------------------------------------------------------------------------
   -- VREF - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Register_Accessor_Macros VREF - Register accessor macros
 * @{
 */


/* VREF - Register accessors */
#define VREF_TRM_REG(base)                       ((base)->TRM)
#define VREF_SC_REG(base)                        ((base)->SC)

/* VREF - Peripheral instance base addresses */
#define VREF_BASE_PTR                            (VREF)

/* VREF - Register instance definitions */
/* VREF */
#define VREF_TRM                                 VREF_TRM_REG(VREF)
#define VREF_SC                                  VREF_SC_REG(VREF)



/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register accessors */
#define WDOG_STCTRLH_REG(base)                   ((base)->STCTRLH)
#define WDOG_STCTRLL_REG(base)                   ((base)->STCTRLL)
#define WDOG_TOVALH_REG(base)                    ((base)->TOVALH)
#define WDOG_TOVALL_REG(base)                    ((base)->TOVALL)
#define WDOG_WINH_REG(base)                      ((base)->WINH)
#define WDOG_WINL_REG(base)                      ((base)->WINL)
#define WDOG_REFRESH_REG(base)                   ((base)->REFRESH)
#define WDOG_UNLOCK_REG(base)                    ((base)->UNLOCK)
#define WDOG_TMROUTH_REG(base)                   ((base)->TMROUTH)
#define WDOG_TMROUTL_REG(base)                   ((base)->TMROUTL)
#define WDOG_RSTCNT_REG(base)                    ((base)->RSTCNT)
#define WDOG_PRESC_REG(base)                     ((base)->PRESC)

/* WDOG - Peripheral instance base addresses */
#define WDOG_BASE_PTR                            (WDOG)


/* WDOG - Register instance definitions */
/* WDOG */
#define WDOG_STCTRLH                             WDOG_STCTRLH_REG(WDOG)
#define WDOG_STCTRLL                             WDOG_STCTRLL_REG(WDOG)
#define WDOG_TOVALH                              WDOG_TOVALH_REG(WDOG)
#define WDOG_TOVALL                              WDOG_TOVALL_REG(WDOG)
#define WDOG_WINH                                WDOG_WINH_REG(WDOG)
#define WDOG_WINL                                WDOG_WINL_REG(WDOG)
#define WDOG_REFRESH                             WDOG_REFRESH_REG(WDOG)
#define WDOG_UNLOCK                              WDOG_UNLOCK_REG(WDOG)
#define WDOG_TMROUTH                             WDOG_TMROUTH_REG(WDOG)
#define WDOG_TMROUTL                             WDOG_TMROUTL_REG(WDOG)
#define WDOG_RSTCNT                              WDOG_RSTCNT_REG(WDOG)
#define WDOG_PRESC                               WDOG_PRESC_REG(WDOG)

#endif /* LIBOHIBOARD_INCLUDE_PLATFORMS_MK64F12_PORTABILITY_H_ */
