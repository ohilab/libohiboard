;==========================================================================
;  PIC24FJ1024GA606 Standard Assembly Include File
; 
;  (c) Copyright 2019 Microchip Technology, All rights reserved
;  Part support version 1.36.B(25-Jan-2019)
; 
 ; This software is developed by Microchip Technology Inc. and its
 ; subsidiaries ("Microchip").
 ; 
 ; Redistribution and use in source and binary forms, with or without
 ; modification, are permitted provided that the following conditions are 
 ; met:
 ; 
 ; 1.      Redistributions of source code must retain the above copyright
 ;         notice, this list of conditions and the following disclaimer.
 ; 2.      Redistributions in binary form must reproduce the above 
 ;         copyright notice, this list of conditions and the following 
 ;         disclaimer in the documentation and/or other materials provided 
 ;         with the distribution.
 ; 3.      Microchip's name may not be used to endorse or promote products
 ;         derived from this software without specific prior written 
 ;         permission.
 ; 
 ; THIS SOFTWARE IS PROVIDED BY MICROCHIP "AS IS" AND ANY EXPRESS OR IMPLIED
 ; WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 ; MERCHANTABILITY AND FITNESS FOR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 ; SHALL MICROCHIP BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 ; EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT LIMITED TO
 ; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA OR PROFITS;
 ; OR BUSINESS INTERRUPTION) HOWSOEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 ; WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 ; OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ; ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ; 
 ;

.NOLIST

;==========================================================================
;  This header file defines configurations, registers, and other useful
;  bits of information for the PIC24FJ1024GA606 microcontroller.  These names
;  are taken to match the data sheets as closely as possible.
;
;  Note that the processor must be selected before this file is included.
;  The processor may be selected the following ways:
;
;       1. Command line switch:
;               C:\> pic30-as file.s -p24FJ1024GA606
;       2. Placing a ".equ" directive before the ".include":
;               .equ __24FJ1024GA606, 1
;               .include "p24FJ1024GA606.inc"
;       3. Setting the processor in the MPLAB IDE Project Dialog
;==========================================================================

;==========================================================================
;
;       Verify Processor
;
;==========================================================================

.IFNDEF __24FJ1024GA606
   .ERROR "Include file does not match processor setting."
.ENDIF


;==========================================================================
;
;       Register Definitions
;
;==========================================================================

;----- Register Equates (Low Byte, High Byte) -----------------------------

        .extern WREG0
        .equiv  WREG0L, _WREG0
        .equiv  WREG0H, _WREG0+1

        .extern WREG1
        .equiv  WREG1L, _WREG1
        .equiv  WREG1H, _WREG1+1

        .extern WREG2
        .equiv  WREG2L, _WREG2
        .equiv  WREG2H, _WREG2+1

        .extern WREG3
        .equiv  WREG3L, _WREG3
        .equiv  WREG3H, _WREG3+1

        .extern WREG4
        .equiv  WREG4L, _WREG4
        .equiv  WREG4H, _WREG4+1

        .extern WREG5
        .equiv  WREG5L, _WREG5
        .equiv  WREG5H, _WREG5+1

        .extern WREG6
        .equiv  WREG6L, _WREG6
        .equiv  WREG6H, _WREG6+1

        .extern WREG7
        .equiv  WREG7L, _WREG7
        .equiv  WREG7H, _WREG7+1

        .extern WREG8
        .equiv  WREG8L, _WREG8
        .equiv  WREG8H, _WREG8+1

        .extern WREG9
        .equiv  WREG9L, _WREG9
        .equiv  WREG9H, _WREG9+1

        .extern WREG10
        .equiv  WREG10L, _WREG10
        .equiv  WREG10H, _WREG10+1

        .extern WREG11
        .equiv  WREG11L, _WREG11
        .equiv  WREG11H, _WREG11+1

        .extern WREG12
        .equiv  WREG12L, _WREG12
        .equiv  WREG12H, _WREG12+1

        .extern WREG13
        .equiv  WREG13L, _WREG13
        .equiv  WREG13H, _WREG13+1

        .extern WREG14
        .equiv  WREG14L, _WREG14
        .equiv  WREG14H, _WREG14+1

        .extern WREG15
        .equiv  WREG15L, _WREG15
        .equiv  WREG15H, _WREG15+1

        .extern SPLIM
        .equiv  SPLIML, _SPLIM
        .equiv  SPLIMH, _SPLIM+1

        .extern PCL
        .equiv  PCLL, _PCL
        .equiv  PCLH, _PCL+1

        .extern PCH
        .equiv  PCHL, _PCH
        .equiv  PCHH, _PCH+1

        .extern DSRPAG
        .equiv  DSRPAGL, _DSRPAG
        .equiv  DSRPAGH, _DSRPAG+1

        .extern DSWPAG
        .equiv  DSWPAGL, _DSWPAG
        .equiv  DSWPAGH, _DSWPAG+1

        .extern RCOUNT
        .equiv  RCOUNTL, _RCOUNT
        .equiv  RCOUNTH, _RCOUNT+1

        .extern SR
        .equiv  SRL, _SR
        .equiv  SRH, _SR+1

        .extern CORCON
        .equiv  CORCONL, _CORCON
        .equiv  CORCONH, _CORCON+1

        .extern DISICNT
        .equiv  DISICNTL, _DISICNT
        .equiv  DISICNTH, _DISICNT+1

        .extern TBLPAG
        .equiv  TBLPAGL, _TBLPAG
        .equiv  TBLPAGH, _TBLPAG+1

        .extern INTCON1
        .equiv  INTCON1L, _INTCON1
        .equiv  INTCON1H, _INTCON1+1

        .extern INTCON2
        .equiv  INTCON2L, _INTCON2
        .equiv  INTCON2H, _INTCON2+1

        .extern INTCON4
        .equiv  INTCON4L, _INTCON4
        .equiv  INTCON4H, _INTCON4+1

        .extern IFS0
        .equiv  IFS0L, _IFS0
        .equiv  IFS0H, _IFS0+1

        .extern IFS1
        .equiv  IFS1L, _IFS1
        .equiv  IFS1H, _IFS1+1

        .extern IFS2
        .equiv  IFS2L, _IFS2
        .equiv  IFS2H, _IFS2+1

        .extern IFS3
        .equiv  IFS3L, _IFS3
        .equiv  IFS3H, _IFS3+1

        .extern IFS4
        .equiv  IFS4L, _IFS4
        .equiv  IFS4H, _IFS4+1

        .extern IFS5
        .equiv  IFS5L, _IFS5
        .equiv  IFS5H, _IFS5+1

        .extern IFS6
        .equiv  IFS6L, _IFS6
        .equiv  IFS6H, _IFS6+1

        .extern IFS7
        .equiv  IFS7L, _IFS7
        .equiv  IFS7H, _IFS7+1

        .extern IEC0
        .equiv  IEC0L, _IEC0
        .equiv  IEC0H, _IEC0+1

        .extern IEC1
        .equiv  IEC1L, _IEC1
        .equiv  IEC1H, _IEC1+1

        .extern IEC2
        .equiv  IEC2L, _IEC2
        .equiv  IEC2H, _IEC2+1

        .extern IEC3
        .equiv  IEC3L, _IEC3
        .equiv  IEC3H, _IEC3+1

        .extern IEC4
        .equiv  IEC4L, _IEC4
        .equiv  IEC4H, _IEC4+1

        .extern IEC5
        .equiv  IEC5L, _IEC5
        .equiv  IEC5H, _IEC5+1

        .extern IEC6
        .equiv  IEC6L, _IEC6
        .equiv  IEC6H, _IEC6+1

        .extern IEC7
        .equiv  IEC7L, _IEC7
        .equiv  IEC7H, _IEC7+1

        .extern IPC0
        .equiv  IPC0L, _IPC0
        .equiv  IPC0H, _IPC0+1

        .extern IPC1
        .equiv  IPC1L, _IPC1
        .equiv  IPC1H, _IPC1+1

        .extern IPC2
        .equiv  IPC2L, _IPC2
        .equiv  IPC2H, _IPC2+1

        .extern IPC3
        .equiv  IPC3L, _IPC3
        .equiv  IPC3H, _IPC3+1

        .extern IPC4
        .equiv  IPC4L, _IPC4
        .equiv  IPC4H, _IPC4+1

        .extern IPC5
        .equiv  IPC5L, _IPC5
        .equiv  IPC5H, _IPC5+1

        .extern IPC6
        .equiv  IPC6L, _IPC6
        .equiv  IPC6H, _IPC6+1

        .extern IPC7
        .equiv  IPC7L, _IPC7
        .equiv  IPC7H, _IPC7+1

        .extern IPC8
        .equiv  IPC8L, _IPC8
        .equiv  IPC8H, _IPC8+1

        .extern IPC9
        .equiv  IPC9L, _IPC9
        .equiv  IPC9H, _IPC9+1

        .extern IPC10
        .equiv  IPC10L, _IPC10
        .equiv  IPC10H, _IPC10+1

        .extern IPC11
        .equiv  IPC11L, _IPC11
        .equiv  IPC11H, _IPC11+1

        .extern IPC12
        .equiv  IPC12L, _IPC12
        .equiv  IPC12H, _IPC12+1

        .extern IPC13
        .equiv  IPC13L, _IPC13
        .equiv  IPC13H, _IPC13+1

        .extern IPC14
        .equiv  IPC14L, _IPC14
        .equiv  IPC14H, _IPC14+1

        .extern IPC15
        .equiv  IPC15L, _IPC15
        .equiv  IPC15H, _IPC15+1

        .extern IPC16
        .equiv  IPC16L, _IPC16
        .equiv  IPC16H, _IPC16+1

        .extern IPC17
        .equiv  IPC17L, _IPC17
        .equiv  IPC17H, _IPC17+1

        .extern IPC18
        .equiv  IPC18L, _IPC18
        .equiv  IPC18H, _IPC18+1

        .extern IPC19
        .equiv  IPC19L, _IPC19
        .equiv  IPC19H, _IPC19+1

        .extern IPC20
        .equiv  IPC20L, _IPC20
        .equiv  IPC20H, _IPC20+1

        .extern IPC21
        .equiv  IPC21L, _IPC21
        .equiv  IPC21H, _IPC21+1

        .extern IPC22
        .equiv  IPC22L, _IPC22
        .equiv  IPC22H, _IPC22+1

        .extern IPC23
        .equiv  IPC23L, _IPC23
        .equiv  IPC23H, _IPC23+1

        .extern IPC24
        .equiv  IPC24L, _IPC24
        .equiv  IPC24H, _IPC24+1

        .extern IPC25
        .equiv  IPC25L, _IPC25
        .equiv  IPC25H, _IPC25+1

        .extern IPC26
        .equiv  IPC26L, _IPC26
        .equiv  IPC26H, _IPC26+1

        .extern IPC27
        .equiv  IPC27L, _IPC27
        .equiv  IPC27H, _IPC27+1

        .extern IPC28
        .equiv  IPC28L, _IPC28
        .equiv  IPC28H, _IPC28+1

        .extern IPC29
        .equiv  IPC29L, _IPC29
        .equiv  IPC29H, _IPC29+1

        .extern INTTREG
        .equiv  INTTREGL, _INTTREG
        .equiv  INTTREGH, _INTTREG+1

        .extern OSCCON

        .extern OSCCONL
        .equiv  OSCCONLL, _OSCCONL
        .equiv  OSCCONLH, _OSCCONL+1

        .extern OSCCONH
        .equiv  OSCCONHL, _OSCCONH
        .equiv  OSCCONHH, _OSCCONH+1

        .extern CLKDIV
        .equiv  CLKDIVL, _CLKDIV
        .equiv  CLKDIVH, _CLKDIV+1

        .extern OSCTUN
        .equiv  OSCTUNL, _OSCTUN
        .equiv  OSCTUNH, _OSCTUN+1

        .extern DCOTUN
        .equiv  DCOTUNL, _DCOTUN
        .equiv  DCOTUNH, _DCOTUN+1

        .extern DCOCON
        .equiv  DCOCONL, _DCOCON
        .equiv  DCOCONH, _DCOCON+1

        .extern OSCDIV
        .equiv  OSCDIVL, _OSCDIV
        .equiv  OSCDIVH, _OSCDIV+1

        .extern OSCFDIV
        .equiv  OSCFDIVL, _OSCFDIV
        .equiv  OSCFDIVH, _OSCFDIV+1

        .extern RCON
        .equiv  RCONL, _RCON
        .equiv  RCONH, _RCON+1

        .extern HLVDCON
        .equiv  HLVDCONL, _HLVDCON
        .equiv  HLVDCONH, _HLVDCON+1

        .extern PMCON1
        .equiv  PMCON1L, _PMCON1
        .equiv  PMCON1H, _PMCON1+1

        .extern PMCON2
        .equiv  PMCON2L, _PMCON2
        .equiv  PMCON2H, _PMCON2+1

        .extern PMCON3
        .equiv  PMCON3L, _PMCON3
        .equiv  PMCON3H, _PMCON3+1

        .extern PMCON4
        .equiv  PMCON4L, _PMCON4
        .equiv  PMCON4H, _PMCON4+1

        .extern PMCS1CF
        .equiv  PMCS1CFL, _PMCS1CF
        .equiv  PMCS1CFH, _PMCS1CF+1

        .extern PMCS1BS
        .equiv  PMCS1BSL, _PMCS1BS
        .equiv  PMCS1BSH, _PMCS1BS+1

        .extern PMCS1MD
        .equiv  PMCS1MDL, _PMCS1MD
        .equiv  PMCS1MDH, _PMCS1MD+1

        .extern PMCS2CF
        .equiv  PMCS2CFL, _PMCS2CF
        .equiv  PMCS2CFH, _PMCS2CF+1

        .extern PMCS2BS
        .equiv  PMCS2BSL, _PMCS2BS
        .equiv  PMCS2BSH, _PMCS2BS+1

        .extern PMCS2MD
        .equiv  PMCS2MDL, _PMCS2MD
        .equiv  PMCS2MDH, _PMCS2MD+1

        .extern PMDOUT1
        .equiv  PMDOUT1L, _PMDOUT1
        .equiv  PMDOUT1H, _PMDOUT1+1

        .extern PMDOUT2
        .equiv  PMDOUT2L, _PMDOUT2
        .equiv  PMDOUT2H, _PMDOUT2+1

        .extern PMDIN1
        .equiv  PMDIN1L, _PMDIN1
        .equiv  PMDIN1H, _PMDIN1+1

        .extern PMDIN2
        .equiv  PMDIN2L, _PMDIN2
        .equiv  PMDIN2H, _PMDIN2+1

        .extern PMSTAT
        .equiv  PMSTATL, _PMSTAT
        .equiv  PMSTATH, _PMSTAT+1

        .extern CRCCON1
        .equiv  CRCCON1L, _CRCCON1
        .equiv  CRCCON1H, _CRCCON1+1

        .extern CRCCON2
        .equiv  CRCCON2L, _CRCCON2
        .equiv  CRCCON2H, _CRCCON2+1

        .extern CRCXORL
        .equiv  CRCXORLL, _CRCXORL
        .equiv  CRCXORLH, _CRCXORL+1

        .extern CRCXORH
        .equiv  CRCXORHL, _CRCXORH
        .equiv  CRCXORHH, _CRCXORH+1

        .extern CRCDAT

        .extern CRCDATL
        .equiv  CRCDATLL, _CRCDATL
        .equiv  CRCDATLH, _CRCDATL+1

        .extern CRCDATH
        .equiv  CRCDATHL, _CRCDATH
        .equiv  CRCDATHH, _CRCDATH+1

        .extern CRCWDAT

        .extern CRCWDATL
        .equiv  CRCWDATLL, _CRCWDATL
        .equiv  CRCWDATLH, _CRCWDATL+1

        .extern CRCWDATH
        .equiv  CRCWDATHL, _CRCWDATH
        .equiv  CRCWDATHH, _CRCWDATH+1

        .extern REFOCON1
        .equiv  REFOCON1L, _REFOCON1
        .equiv  REFOCON1H, _REFOCON1+1

        .extern REFOCONL
        .equiv  REFOCONLL, _REFOCONL
        .equiv  REFOCONLH, _REFOCONL+1

        .extern REFOCON2
        .equiv  REFOCON2L, _REFOCON2
        .equiv  REFOCON2H, _REFOCON2+1

        .extern REFOCONH
        .equiv  REFOCONHL, _REFOCONH
        .equiv  REFOCONHH, _REFOCONH+1

        .extern PMD1
        .equiv  PMD1L, _PMD1
        .equiv  PMD1H, _PMD1+1

        .extern PMD2
        .equiv  PMD2L, _PMD2
        .equiv  PMD2H, _PMD2+1

        .extern PMD3
        .equiv  PMD3L, _PMD3
        .equiv  PMD3H, _PMD3+1

        .extern PMD4
        .equiv  PMD4L, _PMD4
        .equiv  PMD4H, _PMD4+1

        .extern PMD5
        .equiv  PMD5L, _PMD5
        .equiv  PMD5H, _PMD5+1

        .extern PMD6
        .equiv  PMD6L, _PMD6
        .equiv  PMD6H, _PMD6+1

        .extern PMD7
        .equiv  PMD7L, _PMD7
        .equiv  PMD7H, _PMD7+1

        .extern PMD8
        .equiv  PMD8L, _PMD8
        .equiv  PMD8H, _PMD8+1

        .extern TMR1
        .equiv  TMR1L, _TMR1
        .equiv  TMR1H, _TMR1+1

        .extern PR1
        .equiv  PR1L, _PR1
        .equiv  PR1H, _PR1+1

        .extern T1CON
        .equiv  T1CONL, _T1CON
        .equiv  T1CONH, _T1CON+1

        .extern TMR2
        .equiv  TMR2L, _TMR2
        .equiv  TMR2H, _TMR2+1

        .extern TMR3HLD
        .equiv  TMR3HLDL, _TMR3HLD
        .equiv  TMR3HLDH, _TMR3HLD+1

        .extern TMR3
        .equiv  TMR3L, _TMR3
        .equiv  TMR3H, _TMR3+1

        .extern PR2
        .equiv  PR2L, _PR2
        .equiv  PR2H, _PR2+1

        .extern PR3
        .equiv  PR3L, _PR3
        .equiv  PR3H, _PR3+1

        .extern T2CON
        .equiv  T2CONL, _T2CON
        .equiv  T2CONH, _T2CON+1

        .extern T3CON
        .equiv  T3CONL, _T3CON
        .equiv  T3CONH, _T3CON+1

        .extern TMR4
        .equiv  TMR4L, _TMR4
        .equiv  TMR4H, _TMR4+1

        .extern TMR5HLD
        .equiv  TMR5HLDL, _TMR5HLD
        .equiv  TMR5HLDH, _TMR5HLD+1

        .extern TMR5
        .equiv  TMR5L, _TMR5
        .equiv  TMR5H, _TMR5+1

        .extern PR4
        .equiv  PR4L, _PR4
        .equiv  PR4H, _PR4+1

        .extern PR5
        .equiv  PR5L, _PR5
        .equiv  PR5H, _PR5+1

        .extern T4CON
        .equiv  T4CONL, _T4CON
        .equiv  T4CONH, _T4CON+1

        .extern T5CON
        .equiv  T5CONL, _T5CON
        .equiv  T5CONH, _T5CON+1

        .extern CTMUCON1L
        .equiv  CTMUCON1LL, _CTMUCON1L
        .equiv  CTMUCON1LH, _CTMUCON1L+1

        .extern CTMUCON1H
        .equiv  CTMUCON1HL, _CTMUCON1H
        .equiv  CTMUCON1HH, _CTMUCON1H+1

        .extern CTMUCON2L
        .equiv  CTMUCON2LL, _CTMUCON2L
        .equiv  CTMUCON2LH, _CTMUCON2L+1

        .extern RTCCON1L
        .equiv  RTCCON1LL, _RTCCON1L
        .equiv  RTCCON1LH, _RTCCON1L+1

        .extern RTCCON1H
        .equiv  RTCCON1HL, _RTCCON1H
        .equiv  RTCCON1HH, _RTCCON1H+1

        .extern RTCCON2L
        .equiv  RTCCON2LL, _RTCCON2L
        .equiv  RTCCON2LH, _RTCCON2L+1

        .extern RTCCON2H
        .equiv  RTCCON2HL, _RTCCON2H
        .equiv  RTCCON2HH, _RTCCON2H+1

        .extern RTCCON3L
        .equiv  RTCCON3LL, _RTCCON3L
        .equiv  RTCCON3LH, _RTCCON3L+1

        .extern RTCSTATL
        .equiv  RTCSTATLL, _RTCSTATL
        .equiv  RTCSTATLH, _RTCSTATL+1

        .extern TIMEL
        .equiv  TIMELL, _TIMEL
        .equiv  TIMELH, _TIMEL+1

        .extern TIMEH
        .equiv  TIMEHL, _TIMEH
        .equiv  TIMEHH, _TIMEH+1

        .extern DATEL
        .equiv  DATELL, _DATEL
        .equiv  DATELH, _DATEL+1

        .extern DATEH
        .equiv  DATEHL, _DATEH
        .equiv  DATEHH, _DATEH+1

        .extern ALMTIMEL
        .equiv  ALMTIMELL, _ALMTIMEL
        .equiv  ALMTIMELH, _ALMTIMEL+1

        .extern ALMTIMEH
        .equiv  ALMTIMEHL, _ALMTIMEH
        .equiv  ALMTIMEHH, _ALMTIMEH+1

        .extern ALMDATEL
        .equiv  ALMDATELL, _ALMDATEL
        .equiv  ALMDATELH, _ALMDATEL+1

        .extern ALMDATEH
        .equiv  ALMDATEHL, _ALMDATEH
        .equiv  ALMDATEHH, _ALMDATEH+1

        .extern TSATIMEL
        .equiv  TSATIMELL, _TSATIMEL
        .equiv  TSATIMELH, _TSATIMEL+1

        .extern TSATIMEH
        .equiv  TSATIMEHL, _TSATIMEH
        .equiv  TSATIMEHH, _TSATIMEH+1

        .extern TSADATEL
        .equiv  TSADATELL, _TSADATEL
        .equiv  TSADATELH, _TSADATEL+1

        .extern TSADATEH
        .equiv  TSADATEHL, _TSADATEH
        .equiv  TSADATEHH, _TSADATEH+1

        .extern IC1CON1
        .equiv  IC1CON1L, _IC1CON1
        .equiv  IC1CON1H, _IC1CON1+1

        .extern IC1CON2
        .equiv  IC1CON2L, _IC1CON2
        .equiv  IC1CON2H, _IC1CON2+1

        .extern IC1BUF
        .equiv  IC1BUFL, _IC1BUF
        .equiv  IC1BUFH, _IC1BUF+1

        .extern IC1TMR
        .equiv  IC1TMRL, _IC1TMR
        .equiv  IC1TMRH, _IC1TMR+1

        .extern IC2CON1
        .equiv  IC2CON1L, _IC2CON1
        .equiv  IC2CON1H, _IC2CON1+1

        .extern IC2CON2
        .equiv  IC2CON2L, _IC2CON2
        .equiv  IC2CON2H, _IC2CON2+1

        .extern IC2BUF
        .equiv  IC2BUFL, _IC2BUF
        .equiv  IC2BUFH, _IC2BUF+1

        .extern IC2TMR
        .equiv  IC2TMRL, _IC2TMR
        .equiv  IC2TMRH, _IC2TMR+1

        .extern IC3CON1
        .equiv  IC3CON1L, _IC3CON1
        .equiv  IC3CON1H, _IC3CON1+1

        .extern IC3CON2
        .equiv  IC3CON2L, _IC3CON2
        .equiv  IC3CON2H, _IC3CON2+1

        .extern IC3BUF
        .equiv  IC3BUFL, _IC3BUF
        .equiv  IC3BUFH, _IC3BUF+1

        .extern IC3TMR
        .equiv  IC3TMRL, _IC3TMR
        .equiv  IC3TMRH, _IC3TMR+1

        .extern IC4CON1
        .equiv  IC4CON1L, _IC4CON1
        .equiv  IC4CON1H, _IC4CON1+1

        .extern IC4CON2
        .equiv  IC4CON2L, _IC4CON2
        .equiv  IC4CON2H, _IC4CON2+1

        .extern IC4BUF
        .equiv  IC4BUFL, _IC4BUF
        .equiv  IC4BUFH, _IC4BUF+1

        .extern IC4TMR
        .equiv  IC4TMRL, _IC4TMR
        .equiv  IC4TMRH, _IC4TMR+1

        .extern IC5CON1
        .equiv  IC5CON1L, _IC5CON1
        .equiv  IC5CON1H, _IC5CON1+1

        .extern IC5CON2
        .equiv  IC5CON2L, _IC5CON2
        .equiv  IC5CON2H, _IC5CON2+1

        .extern IC5BUF
        .equiv  IC5BUFL, _IC5BUF
        .equiv  IC5BUFH, _IC5BUF+1

        .extern IC5TMR
        .equiv  IC5TMRL, _IC5TMR
        .equiv  IC5TMRH, _IC5TMR+1

        .extern IC6CON1
        .equiv  IC6CON1L, _IC6CON1
        .equiv  IC6CON1H, _IC6CON1+1

        .extern IC6CON2
        .equiv  IC6CON2L, _IC6CON2
        .equiv  IC6CON2H, _IC6CON2+1

        .extern IC6BUF
        .equiv  IC6BUFL, _IC6BUF
        .equiv  IC6BUFH, _IC6BUF+1

        .extern IC6TMR
        .equiv  IC6TMRL, _IC6TMR
        .equiv  IC6TMRH, _IC6TMR+1

        .extern OC1CON1
        .equiv  OC1CON1L, _OC1CON1
        .equiv  OC1CON1H, _OC1CON1+1

        .extern OC1CON2
        .equiv  OC1CON2L, _OC1CON2
        .equiv  OC1CON2H, _OC1CON2+1

        .extern OC1RS
        .equiv  OC1RSL, _OC1RS
        .equiv  OC1RSH, _OC1RS+1

        .extern OC1R
        .equiv  OC1RL, _OC1R
        .equiv  OC1RH, _OC1R+1

        .extern OC1TMR
        .equiv  OC1TMRL, _OC1TMR
        .equiv  OC1TMRH, _OC1TMR+1

        .extern OC2CON1
        .equiv  OC2CON1L, _OC2CON1
        .equiv  OC2CON1H, _OC2CON1+1

        .extern OC2CON2
        .equiv  OC2CON2L, _OC2CON2
        .equiv  OC2CON2H, _OC2CON2+1

        .extern OC2RS
        .equiv  OC2RSL, _OC2RS
        .equiv  OC2RSH, _OC2RS+1

        .extern OC2R
        .equiv  OC2RL, _OC2R
        .equiv  OC2RH, _OC2R+1

        .extern OC2TMR
        .equiv  OC2TMRL, _OC2TMR
        .equiv  OC2TMRH, _OC2TMR+1

        .extern OC3CON1
        .equiv  OC3CON1L, _OC3CON1
        .equiv  OC3CON1H, _OC3CON1+1

        .extern OC3CON2
        .equiv  OC3CON2L, _OC3CON2
        .equiv  OC3CON2H, _OC3CON2+1

        .extern OC3RS
        .equiv  OC3RSL, _OC3RS
        .equiv  OC3RSH, _OC3RS+1

        .extern OC3R
        .equiv  OC3RL, _OC3R
        .equiv  OC3RH, _OC3R+1

        .extern OC3TMR
        .equiv  OC3TMRL, _OC3TMR
        .equiv  OC3TMRH, _OC3TMR+1

        .extern OC4CON1
        .equiv  OC4CON1L, _OC4CON1
        .equiv  OC4CON1H, _OC4CON1+1

        .extern OC4CON2
        .equiv  OC4CON2L, _OC4CON2
        .equiv  OC4CON2H, _OC4CON2+1

        .extern OC4RS
        .equiv  OC4RSL, _OC4RS
        .equiv  OC4RSH, _OC4RS+1

        .extern OC4R
        .equiv  OC4RL, _OC4R
        .equiv  OC4RH, _OC4R+1

        .extern OC4TMR
        .equiv  OC4TMRL, _OC4TMR
        .equiv  OC4TMRH, _OC4TMR+1

        .extern OC5CON1
        .equiv  OC5CON1L, _OC5CON1
        .equiv  OC5CON1H, _OC5CON1+1

        .extern OC5CON2
        .equiv  OC5CON2L, _OC5CON2
        .equiv  OC5CON2H, _OC5CON2+1

        .extern OC5RS
        .equiv  OC5RSL, _OC5RS
        .equiv  OC5RSH, _OC5RS+1

        .extern OC5R
        .equiv  OC5RL, _OC5R
        .equiv  OC5RH, _OC5R+1

        .extern OC5TMR
        .equiv  OC5TMRL, _OC5TMR
        .equiv  OC5TMRH, _OC5TMR+1

        .extern OC6CON1
        .equiv  OC6CON1L, _OC6CON1
        .equiv  OC6CON1H, _OC6CON1+1

        .extern OC6CON2
        .equiv  OC6CON2L, _OC6CON2
        .equiv  OC6CON2H, _OC6CON2+1

        .extern OC6RS
        .equiv  OC6RSL, _OC6RS
        .equiv  OC6RSH, _OC6RS+1

        .extern OC6R
        .equiv  OC6RL, _OC6R
        .equiv  OC6RH, _OC6R+1

        .extern OC6TMR
        .equiv  OC6TMRL, _OC6TMR
        .equiv  OC6TMRH, _OC6TMR+1

        .extern CCP1CON1L
        .equiv  CCP1CON1LL, _CCP1CON1L
        .equiv  CCP1CON1LH, _CCP1CON1L+1

        .extern CCP1CON1H
        .equiv  CCP1CON1HL, _CCP1CON1H
        .equiv  CCP1CON1HH, _CCP1CON1H+1

        .extern CCP1CON2L
        .equiv  CCP1CON2LL, _CCP1CON2L
        .equiv  CCP1CON2LH, _CCP1CON2L+1

        .extern CCP1CON2H
        .equiv  CCP1CON2HL, _CCP1CON2H
        .equiv  CCP1CON2HH, _CCP1CON2H+1

        .extern CCP1CON3L
        .equiv  CCP1CON3LL, _CCP1CON3L
        .equiv  CCP1CON3LH, _CCP1CON3L+1

        .extern CCP1CON3H
        .equiv  CCP1CON3HL, _CCP1CON3H
        .equiv  CCP1CON3HH, _CCP1CON3H+1

        .extern CCP1STAT

        .extern CCP1STATL
        .equiv  CCP1STATLL, _CCP1STATL
        .equiv  CCP1STATLH, _CCP1STATL+1

        .extern CCP1STATH
        .equiv  CCP1STATHL, _CCP1STATH
        .equiv  CCP1STATHH, _CCP1STATH+1

        .extern CCP1TMRL
        .equiv  CCP1TMRLL, _CCP1TMRL
        .equiv  CCP1TMRLH, _CCP1TMRL+1

        .extern CCP1TMRH
        .equiv  CCP1TMRHL, _CCP1TMRH
        .equiv  CCP1TMRHH, _CCP1TMRH+1

        .extern CCP1PRL
        .equiv  CCP1PRLL, _CCP1PRL
        .equiv  CCP1PRLH, _CCP1PRL+1

        .extern CCP1PRH
        .equiv  CCP1PRHL, _CCP1PRH
        .equiv  CCP1PRHH, _CCP1PRH+1

        .extern CCP1RA

        .extern CCP1RAL
        .equiv  CCP1RALL, _CCP1RAL
        .equiv  CCP1RALH, _CCP1RAL+1

        .extern CCP1RAH
        .equiv  CCP1RAHL, _CCP1RAH
        .equiv  CCP1RAHH, _CCP1RAH+1

        .extern CCP1RB

        .extern CCP1RBL
        .equiv  CCP1RBLL, _CCP1RBL
        .equiv  CCP1RBLH, _CCP1RBL+1

        .extern CCP1RBH
        .equiv  CCP1RBHL, _CCP1RBH
        .equiv  CCP1RBHH, _CCP1RBH+1

        .extern CCP1BUFL
        .equiv  CCP1BUFLL, _CCP1BUFL
        .equiv  CCP1BUFLH, _CCP1BUFL+1

        .extern CCP1BUFH
        .equiv  CCP1BUFHL, _CCP1BUFH
        .equiv  CCP1BUFHH, _CCP1BUFH+1

        .extern CCP2CON1L
        .equiv  CCP2CON1LL, _CCP2CON1L
        .equiv  CCP2CON1LH, _CCP2CON1L+1

        .extern CCP2CON1H
        .equiv  CCP2CON1HL, _CCP2CON1H
        .equiv  CCP2CON1HH, _CCP2CON1H+1

        .extern CCP2CON2L
        .equiv  CCP2CON2LL, _CCP2CON2L
        .equiv  CCP2CON2LH, _CCP2CON2L+1

        .extern CCP2CON2H
        .equiv  CCP2CON2HL, _CCP2CON2H
        .equiv  CCP2CON2HH, _CCP2CON2H+1

        .extern CCP2CON3L
        .equiv  CCP2CON3LL, _CCP2CON3L
        .equiv  CCP2CON3LH, _CCP2CON3L+1

        .extern CCP2CON3H
        .equiv  CCP2CON3HL, _CCP2CON3H
        .equiv  CCP2CON3HH, _CCP2CON3H+1

        .extern CCP2STAT

        .extern CCP2STATL
        .equiv  CCP2STATLL, _CCP2STATL
        .equiv  CCP2STATLH, _CCP2STATL+1

        .extern CCP2STATH
        .equiv  CCP2STATHL, _CCP2STATH
        .equiv  CCP2STATHH, _CCP2STATH+1

        .extern CCP2TMRL
        .equiv  CCP2TMRLL, _CCP2TMRL
        .equiv  CCP2TMRLH, _CCP2TMRL+1

        .extern CCP2TMRH
        .equiv  CCP2TMRHL, _CCP2TMRH
        .equiv  CCP2TMRHH, _CCP2TMRH+1

        .extern CCP2PRL
        .equiv  CCP2PRLL, _CCP2PRL
        .equiv  CCP2PRLH, _CCP2PRL+1

        .extern CCP2PRH
        .equiv  CCP2PRHL, _CCP2PRH
        .equiv  CCP2PRHH, _CCP2PRH+1

        .extern CCP2RA

        .extern CCP2RAL
        .equiv  CCP2RALL, _CCP2RAL
        .equiv  CCP2RALH, _CCP2RAL+1

        .extern CCP2RAH
        .equiv  CCP2RAHL, _CCP2RAH
        .equiv  CCP2RAHH, _CCP2RAH+1

        .extern CCP2RB

        .extern CCP2RBL
        .equiv  CCP2RBLL, _CCP2RBL
        .equiv  CCP2RBLH, _CCP2RBL+1

        .extern CCP2RBH
        .equiv  CCP2RBHL, _CCP2RBH
        .equiv  CCP2RBHH, _CCP2RBH+1

        .extern CCP2BUFL
        .equiv  CCP2BUFLL, _CCP2BUFL
        .equiv  CCP2BUFLH, _CCP2BUFL+1

        .extern CCP2BUFH
        .equiv  CCP2BUFHL, _CCP2BUFH
        .equiv  CCP2BUFHH, _CCP2BUFH+1

        .extern CCP3CON1L
        .equiv  CCP3CON1LL, _CCP3CON1L
        .equiv  CCP3CON1LH, _CCP3CON1L+1

        .extern CCP3CON1H
        .equiv  CCP3CON1HL, _CCP3CON1H
        .equiv  CCP3CON1HH, _CCP3CON1H+1

        .extern CCP3CON2L
        .equiv  CCP3CON2LL, _CCP3CON2L
        .equiv  CCP3CON2LH, _CCP3CON2L+1

        .extern CCP3CON2H
        .equiv  CCP3CON2HL, _CCP3CON2H
        .equiv  CCP3CON2HH, _CCP3CON2H+1

        .extern CCP3CON3L
        .equiv  CCP3CON3LL, _CCP3CON3L
        .equiv  CCP3CON3LH, _CCP3CON3L+1

        .extern CCP3CON3H
        .equiv  CCP3CON3HL, _CCP3CON3H
        .equiv  CCP3CON3HH, _CCP3CON3H+1

        .extern CCP3STAT

        .extern CCP3STATL
        .equiv  CCP3STATLL, _CCP3STATL
        .equiv  CCP3STATLH, _CCP3STATL+1

        .extern CCP3STATH
        .equiv  CCP3STATHL, _CCP3STATH
        .equiv  CCP3STATHH, _CCP3STATH+1

        .extern CCP3TMRL
        .equiv  CCP3TMRLL, _CCP3TMRL
        .equiv  CCP3TMRLH, _CCP3TMRL+1

        .extern CCP3TMRH
        .equiv  CCP3TMRHL, _CCP3TMRH
        .equiv  CCP3TMRHH, _CCP3TMRH+1

        .extern CCP3PRL
        .equiv  CCP3PRLL, _CCP3PRL
        .equiv  CCP3PRLH, _CCP3PRL+1

        .extern CCP3PRH
        .equiv  CCP3PRHL, _CCP3PRH
        .equiv  CCP3PRHH, _CCP3PRH+1

        .extern CCP3RA

        .extern CCP3RAL
        .equiv  CCP3RALL, _CCP3RAL
        .equiv  CCP3RALH, _CCP3RAL+1

        .extern CCP3RAH
        .equiv  CCP3RAHL, _CCP3RAH
        .equiv  CCP3RAHH, _CCP3RAH+1

        .extern CCP3RB

        .extern CCP3RBL
        .equiv  CCP3RBLL, _CCP3RBL
        .equiv  CCP3RBLH, _CCP3RBL+1

        .extern CCP3RBH
        .equiv  CCP3RBHL, _CCP3RBH
        .equiv  CCP3RBHH, _CCP3RBH+1

        .extern CCP3BUFL
        .equiv  CCP3BUFLL, _CCP3BUFL
        .equiv  CCP3BUFLH, _CCP3BUFL+1

        .extern CCP3BUFH
        .equiv  CCP3BUFHL, _CCP3BUFH
        .equiv  CCP3BUFHH, _CCP3BUFH+1

        .extern CMSTAT
        .equiv  CMSTATL, _CMSTAT
        .equiv  CMSTATH, _CMSTAT+1

        .extern CVRCON
        .equiv  CVRCONL, _CVRCON
        .equiv  CVRCONH, _CVRCON+1

        .extern CM1CON
        .equiv  CM1CONL, _CM1CON
        .equiv  CM1CONH, _CM1CON+1

        .extern CM2CON
        .equiv  CM2CONL, _CM2CON
        .equiv  CM2CONH, _CM2CON+1

        .extern CM3CON
        .equiv  CM3CONL, _CM3CON
        .equiv  CM3CONH, _CM3CON+1

        .extern ANCFG
        .equiv  ANCFGL, _ANCFG
        .equiv  ANCFGH, _ANCFG+1

        .extern CCP4CON1L
        .equiv  CCP4CON1LL, _CCP4CON1L
        .equiv  CCP4CON1LH, _CCP4CON1L+1

        .extern CCP4CON1H
        .equiv  CCP4CON1HL, _CCP4CON1H
        .equiv  CCP4CON1HH, _CCP4CON1H+1

        .extern CCP4CON2L
        .equiv  CCP4CON2LL, _CCP4CON2L
        .equiv  CCP4CON2LH, _CCP4CON2L+1

        .extern CCP4CON2H
        .equiv  CCP4CON2HL, _CCP4CON2H
        .equiv  CCP4CON2HH, _CCP4CON2H+1

        .extern CCP4CON3L
        .equiv  CCP4CON3LL, _CCP4CON3L
        .equiv  CCP4CON3LH, _CCP4CON3L+1

        .extern CCP4CON3H
        .equiv  CCP4CON3HL, _CCP4CON3H
        .equiv  CCP4CON3HH, _CCP4CON3H+1

        .extern CCP4STAT

        .extern CCP4STATL
        .equiv  CCP4STATLL, _CCP4STATL
        .equiv  CCP4STATLH, _CCP4STATL+1

        .extern CCP4STATH
        .equiv  CCP4STATHL, _CCP4STATH
        .equiv  CCP4STATHH, _CCP4STATH+1

        .extern CCP4TMRL
        .equiv  CCP4TMRLL, _CCP4TMRL
        .equiv  CCP4TMRLH, _CCP4TMRL+1

        .extern CCP4TMRH
        .equiv  CCP4TMRHL, _CCP4TMRH
        .equiv  CCP4TMRHH, _CCP4TMRH+1

        .extern CCP4PRL
        .equiv  CCP4PRLL, _CCP4PRL
        .equiv  CCP4PRLH, _CCP4PRL+1

        .extern CCP4PRH
        .equiv  CCP4PRHL, _CCP4PRH
        .equiv  CCP4PRHH, _CCP4PRH+1

        .extern CCP4RA

        .extern CCP4RAL
        .equiv  CCP4RALL, _CCP4RAL
        .equiv  CCP4RALH, _CCP4RAL+1

        .extern CCP4RAH
        .equiv  CCP4RAHL, _CCP4RAH
        .equiv  CCP4RAHH, _CCP4RAH+1

        .extern CCP4RB

        .extern CCP4RBL
        .equiv  CCP4RBLL, _CCP4RBL
        .equiv  CCP4RBLH, _CCP4RBL+1

        .extern CCP4RBH
        .equiv  CCP4RBHL, _CCP4RBH
        .equiv  CCP4RBHH, _CCP4RBH+1

        .extern CCP4BUFL
        .equiv  CCP4BUFLL, _CCP4BUFL
        .equiv  CCP4BUFLH, _CCP4BUFL+1

        .extern CCP4BUFH
        .equiv  CCP4BUFHL, _CCP4BUFH
        .equiv  CCP4BUFHH, _CCP4BUFH+1

        .extern CCP5CON1L
        .equiv  CCP5CON1LL, _CCP5CON1L
        .equiv  CCP5CON1LH, _CCP5CON1L+1

        .extern CCP5CON1H
        .equiv  CCP5CON1HL, _CCP5CON1H
        .equiv  CCP5CON1HH, _CCP5CON1H+1

        .extern CCP5CON2L
        .equiv  CCP5CON2LL, _CCP5CON2L
        .equiv  CCP5CON2LH, _CCP5CON2L+1

        .extern CCP5CON2H
        .equiv  CCP5CON2HL, _CCP5CON2H
        .equiv  CCP5CON2HH, _CCP5CON2H+1

        .extern CCP5CON3L
        .equiv  CCP5CON3LL, _CCP5CON3L
        .equiv  CCP5CON3LH, _CCP5CON3L+1

        .extern CCP5CON3H
        .equiv  CCP5CON3HL, _CCP5CON3H
        .equiv  CCP5CON3HH, _CCP5CON3H+1

        .extern CCP5STAT

        .extern CCP5STATL
        .equiv  CCP5STATLL, _CCP5STATL
        .equiv  CCP5STATLH, _CCP5STATL+1

        .extern CCP5STATH
        .equiv  CCP5STATHL, _CCP5STATH
        .equiv  CCP5STATHH, _CCP5STATH+1

        .extern CCP5TMRL
        .equiv  CCP5TMRLL, _CCP5TMRL
        .equiv  CCP5TMRLH, _CCP5TMRL+1

        .extern CCP5TMRH
        .equiv  CCP5TMRHL, _CCP5TMRH
        .equiv  CCP5TMRHH, _CCP5TMRH+1

        .extern CCP5PRL
        .equiv  CCP5PRLL, _CCP5PRL
        .equiv  CCP5PRLH, _CCP5PRL+1

        .extern CCP5PRH
        .equiv  CCP5PRHL, _CCP5PRH
        .equiv  CCP5PRHH, _CCP5PRH+1

        .extern CCP5RA

        .extern CCP5RAL
        .equiv  CCP5RALL, _CCP5RAL
        .equiv  CCP5RALH, _CCP5RAL+1

        .extern CCP5RAH
        .equiv  CCP5RAHL, _CCP5RAH
        .equiv  CCP5RAHH, _CCP5RAH+1

        .extern CCP5RB

        .extern CCP5RBL
        .equiv  CCP5RBLL, _CCP5RBL
        .equiv  CCP5RBLH, _CCP5RBL+1

        .extern CCP5RBH
        .equiv  CCP5RBHL, _CCP5RBH
        .equiv  CCP5RBHH, _CCP5RBH+1

        .extern CCP5BUFL
        .equiv  CCP5BUFLL, _CCP5BUFL
        .equiv  CCP5BUFLH, _CCP5BUFL+1

        .extern CCP5BUFH
        .equiv  CCP5BUFHL, _CCP5BUFH
        .equiv  CCP5BUFHH, _CCP5BUFH+1

        .extern CCP6CON1L
        .equiv  CCP6CON1LL, _CCP6CON1L
        .equiv  CCP6CON1LH, _CCP6CON1L+1

        .extern CCP6CON1H
        .equiv  CCP6CON1HL, _CCP6CON1H
        .equiv  CCP6CON1HH, _CCP6CON1H+1

        .extern CCP6CON2L
        .equiv  CCP6CON2LL, _CCP6CON2L
        .equiv  CCP6CON2LH, _CCP6CON2L+1

        .extern CCP6CON2H
        .equiv  CCP6CON2HL, _CCP6CON2H
        .equiv  CCP6CON2HH, _CCP6CON2H+1

        .extern CCP6CON3L
        .equiv  CCP6CON3LL, _CCP6CON3L
        .equiv  CCP6CON3LH, _CCP6CON3L+1

        .extern CCP6CON3H
        .equiv  CCP6CON3HL, _CCP6CON3H
        .equiv  CCP6CON3HH, _CCP6CON3H+1

        .extern CCP6STAT

        .extern CCP6STATL
        .equiv  CCP6STATLL, _CCP6STATL
        .equiv  CCP6STATLH, _CCP6STATL+1

        .extern CCP6STATH
        .equiv  CCP6STATHL, _CCP6STATH
        .equiv  CCP6STATHH, _CCP6STATH+1

        .extern CCP6TMRL
        .equiv  CCP6TMRLL, _CCP6TMRL
        .equiv  CCP6TMRLH, _CCP6TMRL+1

        .extern CCP6TMRH
        .equiv  CCP6TMRHL, _CCP6TMRH
        .equiv  CCP6TMRHH, _CCP6TMRH+1

        .extern CCP6PRL
        .equiv  CCP6PRLL, _CCP6PRL
        .equiv  CCP6PRLH, _CCP6PRL+1

        .extern CCP6PRH
        .equiv  CCP6PRHL, _CCP6PRH
        .equiv  CCP6PRHH, _CCP6PRH+1

        .extern CCP6RA

        .extern CCP6RAL
        .equiv  CCP6RALL, _CCP6RAL
        .equiv  CCP6RALH, _CCP6RAL+1

        .extern CCP6RAH
        .equiv  CCP6RAHL, _CCP6RAH
        .equiv  CCP6RAHH, _CCP6RAH+1

        .extern CCP6RB

        .extern CCP6RBL
        .equiv  CCP6RBLL, _CCP6RBL
        .equiv  CCP6RBLH, _CCP6RBL+1

        .extern CCP6RBH
        .equiv  CCP6RBHL, _CCP6RBH
        .equiv  CCP6RBHH, _CCP6RBH+1

        .extern CCP6BUFL
        .equiv  CCP6BUFLL, _CCP6BUFL
        .equiv  CCP6BUFLH, _CCP6BUFL+1

        .extern CCP6BUFH
        .equiv  CCP6BUFHL, _CCP6BUFH
        .equiv  CCP6BUFHH, _CCP6BUFH+1

        .extern CCP7CON1L
        .equiv  CCP7CON1LL, _CCP7CON1L
        .equiv  CCP7CON1LH, _CCP7CON1L+1

        .extern CCP7CON1H
        .equiv  CCP7CON1HL, _CCP7CON1H
        .equiv  CCP7CON1HH, _CCP7CON1H+1

        .extern CCP7CON2L
        .equiv  CCP7CON2LL, _CCP7CON2L
        .equiv  CCP7CON2LH, _CCP7CON2L+1

        .extern CCP7CON2H
        .equiv  CCP7CON2HL, _CCP7CON2H
        .equiv  CCP7CON2HH, _CCP7CON2H+1

        .extern CCP7CON3L
        .equiv  CCP7CON3LL, _CCP7CON3L
        .equiv  CCP7CON3LH, _CCP7CON3L+1

        .extern CCP7CON3H
        .equiv  CCP7CON3HL, _CCP7CON3H
        .equiv  CCP7CON3HH, _CCP7CON3H+1

        .extern CCP7STAT

        .extern CCP7STATL
        .equiv  CCP7STATLL, _CCP7STATL
        .equiv  CCP7STATLH, _CCP7STATL+1

        .extern CCP7STATH
        .equiv  CCP7STATHL, _CCP7STATH
        .equiv  CCP7STATHH, _CCP7STATH+1

        .extern CCP7TMRL
        .equiv  CCP7TMRLL, _CCP7TMRL
        .equiv  CCP7TMRLH, _CCP7TMRL+1

        .extern CCP7TMRH
        .equiv  CCP7TMRHL, _CCP7TMRH
        .equiv  CCP7TMRHH, _CCP7TMRH+1

        .extern CCP7PRL
        .equiv  CCP7PRLL, _CCP7PRL
        .equiv  CCP7PRLH, _CCP7PRL+1

        .extern CCP7PRH
        .equiv  CCP7PRHL, _CCP7PRH
        .equiv  CCP7PRHH, _CCP7PRH+1

        .extern CCP7RA

        .extern CCP7RAL
        .equiv  CCP7RALL, _CCP7RAL
        .equiv  CCP7RALH, _CCP7RAL+1

        .extern CCP7RAH
        .equiv  CCP7RAHL, _CCP7RAH
        .equiv  CCP7RAHH, _CCP7RAH+1

        .extern CCP7RB

        .extern CCP7RBL
        .equiv  CCP7RBLL, _CCP7RBL
        .equiv  CCP7RBLH, _CCP7RBL+1

        .extern CCP7RBH
        .equiv  CCP7RBHL, _CCP7RBH
        .equiv  CCP7RBHH, _CCP7RBH+1

        .extern CCP7BUFL
        .equiv  CCP7BUFLL, _CCP7BUFL
        .equiv  CCP7BUFLH, _CCP7BUFL+1

        .extern CCP7BUFH
        .equiv  CCP7BUFHL, _CCP7BUFH
        .equiv  CCP7BUFHH, _CCP7BUFH+1

        .extern U1MODE
        .equiv  U1MODEL, _U1MODE
        .equiv  U1MODEH, _U1MODE+1

        .extern U1STA
        .equiv  U1STAL, _U1STA
        .equiv  U1STAH, _U1STA+1

        .extern U1TXREG
        .equiv  U1TXREGL, _U1TXREG
        .equiv  U1TXREGH, _U1TXREG+1

        .extern U1RXREG
        .equiv  U1RXREGL, _U1RXREG
        .equiv  U1RXREGH, _U1RXREG+1

        .extern U1BRG
        .equiv  U1BRGL, _U1BRG
        .equiv  U1BRGH, _U1BRG+1

        .extern U1ADMD
        .equiv  U1ADMDL, _U1ADMD
        .equiv  U1ADMDH, _U1ADMD+1

        .extern U2MODE
        .equiv  U2MODEL, _U2MODE
        .equiv  U2MODEH, _U2MODE+1

        .extern U2STA
        .equiv  U2STAL, _U2STA
        .equiv  U2STAH, _U2STA+1

        .extern U2TXREG
        .equiv  U2TXREGL, _U2TXREG
        .equiv  U2TXREGH, _U2TXREG+1

        .extern U2RXREG
        .equiv  U2RXREGL, _U2RXREG
        .equiv  U2RXREGH, _U2RXREG+1

        .extern U2BRG
        .equiv  U2BRGL, _U2BRG
        .equiv  U2BRGH, _U2BRG+1

        .extern U2ADMD
        .equiv  U2ADMDL, _U2ADMD
        .equiv  U2ADMDH, _U2ADMD+1

        .extern U3MODE
        .equiv  U3MODEL, _U3MODE
        .equiv  U3MODEH, _U3MODE+1

        .extern U3STA
        .equiv  U3STAL, _U3STA
        .equiv  U3STAH, _U3STA+1

        .extern U3TXREG
        .equiv  U3TXREGL, _U3TXREG
        .equiv  U3TXREGH, _U3TXREG+1

        .extern U3RXREG
        .equiv  U3RXREGL, _U3RXREG
        .equiv  U3RXREGH, _U3RXREG+1

        .extern U3BRG
        .equiv  U3BRGL, _U3BRG
        .equiv  U3BRGH, _U3BRG+1

        .extern U3ADMD
        .equiv  U3ADMDL, _U3ADMD
        .equiv  U3ADMDH, _U3ADMD+1

        .extern U4MODE
        .equiv  U4MODEL, _U4MODE
        .equiv  U4MODEH, _U4MODE+1

        .extern U4STA
        .equiv  U4STAL, _U4STA
        .equiv  U4STAH, _U4STA+1

        .extern U4TXREG
        .equiv  U4TXREGL, _U4TXREG
        .equiv  U4TXREGH, _U4TXREG+1

        .extern U4RXREG
        .equiv  U4RXREGL, _U4RXREG
        .equiv  U4RXREGH, _U4RXREG+1

        .extern U4BRG
        .equiv  U4BRGL, _U4BRG
        .equiv  U4BRGH, _U4BRG+1

        .extern U4ADMD
        .equiv  U4ADMDL, _U4ADMD
        .equiv  U4ADMDH, _U4ADMD+1

        .extern U5MODE
        .equiv  U5MODEL, _U5MODE
        .equiv  U5MODEH, _U5MODE+1

        .extern U5STA
        .equiv  U5STAL, _U5STA
        .equiv  U5STAH, _U5STA+1

        .extern U5TXREG
        .equiv  U5TXREGL, _U5TXREG
        .equiv  U5TXREGH, _U5TXREG+1

        .extern U5RXREG
        .equiv  U5RXREGL, _U5RXREG
        .equiv  U5RXREGH, _U5RXREG+1

        .extern U5BRG
        .equiv  U5BRGL, _U5BRG
        .equiv  U5BRGH, _U5BRG+1

        .extern U5ADMD
        .equiv  U5ADMDL, _U5ADMD
        .equiv  U5ADMDH, _U5ADMD+1

        .extern U6MODE
        .equiv  U6MODEL, _U6MODE
        .equiv  U6MODEH, _U6MODE+1

        .extern U6STA
        .equiv  U6STAL, _U6STA
        .equiv  U6STAH, _U6STA+1

        .extern U6TXREG
        .equiv  U6TXREGL, _U6TXREG
        .equiv  U6TXREGH, _U6TXREG+1

        .extern U6RXREG
        .equiv  U6RXREGL, _U6RXREG
        .equiv  U6RXREGH, _U6RXREG+1

        .extern U6BRG
        .equiv  U6BRGL, _U6BRG
        .equiv  U6BRGH, _U6BRG+1

        .extern U6ADMD
        .equiv  U6ADMDL, _U6ADMD
        .equiv  U6ADMDH, _U6ADMD+1

        .extern SPI1CON1

        .extern SPI1CON1L
        .equiv  SPI1CON1LL, _SPI1CON1L
        .equiv  SPI1CON1LH, _SPI1CON1L+1

        .extern SPI1CON1H
        .equiv  SPI1CON1HL, _SPI1CON1H
        .equiv  SPI1CON1HH, _SPI1CON1H+1

        .extern SPI1CON2

        .extern SPI1CON2L
        .equiv  SPI1CON2LL, _SPI1CON2L
        .equiv  SPI1CON2LH, _SPI1CON2L+1

        .extern SPI1CON3
        .equiv  SPI1CON3L, _SPI1CON3
        .equiv  SPI1CON3H, _SPI1CON3+1

        .extern SPI1STATL
        .equiv  SPI1STATLL, _SPI1STATL
        .equiv  SPI1STATLH, _SPI1STATL+1

        .extern SPI1STATH
        .equiv  SPI1STATHL, _SPI1STATH
        .equiv  SPI1STATHH, _SPI1STATH+1

        .extern SPI1BUFL
        .equiv  SPI1BUFLL, _SPI1BUFL
        .equiv  SPI1BUFLH, _SPI1BUFL+1

        .extern SPI1BUFH
        .equiv  SPI1BUFHL, _SPI1BUFH
        .equiv  SPI1BUFHH, _SPI1BUFH+1

        .extern SPI1BRGL
        .equiv  SPI1BRGLL, _SPI1BRGL
        .equiv  SPI1BRGLH, _SPI1BRGL+1

        .extern SPI1IMSK1
        .equiv  SPI1IMSK1L, _SPI1IMSK1
        .equiv  SPI1IMSK1H, _SPI1IMSK1+1

        .extern SPI1IMSKL
        .equiv  SPI1IMSKLL, _SPI1IMSKL
        .equiv  SPI1IMSKLH, _SPI1IMSKL+1

        .extern SPI1IMSK2
        .equiv  SPI1IMSK2L, _SPI1IMSK2
        .equiv  SPI1IMSK2H, _SPI1IMSK2+1

        .extern SPI1IMSKH
        .equiv  SPI1IMSKHL, _SPI1IMSKH
        .equiv  SPI1IMSKHH, _SPI1IMSKH+1

        .extern SPI1URDTL
        .equiv  SPI1URDTLL, _SPI1URDTL
        .equiv  SPI1URDTLH, _SPI1URDTL+1

        .extern SPI1URDTH
        .equiv  SPI1URDTHL, _SPI1URDTH
        .equiv  SPI1URDTHH, _SPI1URDTH+1

        .extern SPI2CON1

        .extern SPI2CON1L
        .equiv  SPI2CON1LL, _SPI2CON1L
        .equiv  SPI2CON1LH, _SPI2CON1L+1

        .extern SPI2CON1H
        .equiv  SPI2CON1HL, _SPI2CON1H
        .equiv  SPI2CON1HH, _SPI2CON1H+1

        .extern SPI2CON2

        .extern SPI2CON2L
        .equiv  SPI2CON2LL, _SPI2CON2L
        .equiv  SPI2CON2LH, _SPI2CON2L+1

        .extern SPI2CON3
        .equiv  SPI2CON3L, _SPI2CON3
        .equiv  SPI2CON3H, _SPI2CON3+1

        .extern SPI2STATL
        .equiv  SPI2STATLL, _SPI2STATL
        .equiv  SPI2STATLH, _SPI2STATL+1

        .extern SPI2STATH
        .equiv  SPI2STATHL, _SPI2STATH
        .equiv  SPI2STATHH, _SPI2STATH+1

        .extern SPI2BUFL
        .equiv  SPI2BUFLL, _SPI2BUFL
        .equiv  SPI2BUFLH, _SPI2BUFL+1

        .extern SPI2BUFH
        .equiv  SPI2BUFHL, _SPI2BUFH
        .equiv  SPI2BUFHH, _SPI2BUFH+1

        .extern SPI2BRGL
        .equiv  SPI2BRGLL, _SPI2BRGL
        .equiv  SPI2BRGLH, _SPI2BRGL+1

        .extern SPI2IMSK1
        .equiv  SPI2IMSK1L, _SPI2IMSK1
        .equiv  SPI2IMSK1H, _SPI2IMSK1+1

        .extern SPI2IMSKL
        .equiv  SPI2IMSKLL, _SPI2IMSKL
        .equiv  SPI2IMSKLH, _SPI2IMSKL+1

        .extern SPI2IMSK2
        .equiv  SPI2IMSK2L, _SPI2IMSK2
        .equiv  SPI2IMSK2H, _SPI2IMSK2+1

        .extern SPI2IMSKH
        .equiv  SPI2IMSKHL, _SPI2IMSKH
        .equiv  SPI2IMSKHH, _SPI2IMSKH+1

        .extern SPI2URDTL
        .equiv  SPI2URDTLL, _SPI2URDTL
        .equiv  SPI2URDTLH, _SPI2URDTL+1

        .extern SPI2URDTH
        .equiv  SPI2URDTHL, _SPI2URDTH
        .equiv  SPI2URDTHH, _SPI2URDTH+1

        .extern SPI3CON1

        .extern SPI3CON1L
        .equiv  SPI3CON1LL, _SPI3CON1L
        .equiv  SPI3CON1LH, _SPI3CON1L+1

        .extern SPI3CON1H
        .equiv  SPI3CON1HL, _SPI3CON1H
        .equiv  SPI3CON1HH, _SPI3CON1H+1

        .extern SPI3CON2

        .extern SPI3CON2L
        .equiv  SPI3CON2LL, _SPI3CON2L
        .equiv  SPI3CON2LH, _SPI3CON2L+1

        .extern SPI3CON3
        .equiv  SPI3CON3L, _SPI3CON3
        .equiv  SPI3CON3H, _SPI3CON3+1

        .extern SPI3STATL
        .equiv  SPI3STATLL, _SPI3STATL
        .equiv  SPI3STATLH, _SPI3STATL+1

        .extern SPI3STATH
        .equiv  SPI3STATHL, _SPI3STATH
        .equiv  SPI3STATHH, _SPI3STATH+1

        .extern SPI3BUFL
        .equiv  SPI3BUFLL, _SPI3BUFL
        .equiv  SPI3BUFLH, _SPI3BUFL+1

        .extern SPI3BUFH
        .equiv  SPI3BUFHL, _SPI3BUFH
        .equiv  SPI3BUFHH, _SPI3BUFH+1

        .extern SPI3BRGL
        .equiv  SPI3BRGLL, _SPI3BRGL
        .equiv  SPI3BRGLH, _SPI3BRGL+1

        .extern SPI3IMSK1
        .equiv  SPI3IMSK1L, _SPI3IMSK1
        .equiv  SPI3IMSK1H, _SPI3IMSK1+1

        .extern SPI3IMSKL
        .equiv  SPI3IMSKLL, _SPI3IMSKL
        .equiv  SPI3IMSKLH, _SPI3IMSKL+1

        .extern SPI3IMSK2
        .equiv  SPI3IMSK2L, _SPI3IMSK2
        .equiv  SPI3IMSK2H, _SPI3IMSK2+1

        .extern SPI3IMSKH
        .equiv  SPI3IMSKHL, _SPI3IMSKH
        .equiv  SPI3IMSKHH, _SPI3IMSKH+1

        .extern SPI3URDTL
        .equiv  SPI3URDTLL, _SPI3URDTL
        .equiv  SPI3URDTLH, _SPI3URDTL+1

        .extern SPI3URDTH
        .equiv  SPI3URDTHL, _SPI3URDTH
        .equiv  SPI3URDTHH, _SPI3URDTH+1

        .extern CLC1CONL
        .equiv  CLC1CONLL, _CLC1CONL
        .equiv  CLC1CONLH, _CLC1CONL+1

        .extern CLC1CONH
        .equiv  CLC1CONHL, _CLC1CONH
        .equiv  CLC1CONHH, _CLC1CONH+1

        .extern CLC1SEL

        .extern CLC1SELL
        .equiv  CLC1SELLL, _CLC1SELL
        .equiv  CLC1SELLH, _CLC1SELL+1

        .extern CLC1GLSL
        .equiv  CLC1GLSLL, _CLC1GLSL
        .equiv  CLC1GLSLH, _CLC1GLSL+1

        .extern CLC1GLSH
        .equiv  CLC1GLSHL, _CLC1GLSH
        .equiv  CLC1GLSHH, _CLC1GLSH+1

        .extern CLC2CONL
        .equiv  CLC2CONLL, _CLC2CONL
        .equiv  CLC2CONLH, _CLC2CONL+1

        .extern CLC2CONH
        .equiv  CLC2CONHL, _CLC2CONH
        .equiv  CLC2CONHH, _CLC2CONH+1

        .extern CLC2SEL

        .extern CLC2SELL
        .equiv  CLC2SELLL, _CLC2SELL
        .equiv  CLC2SELLH, _CLC2SELL+1

        .extern CLC2GLSL
        .equiv  CLC2GLSLL, _CLC2GLSL
        .equiv  CLC2GLSLH, _CLC2GLSL+1

        .extern CLC2GLSH
        .equiv  CLC2GLSHL, _CLC2GLSH
        .equiv  CLC2GLSHH, _CLC2GLSH+1

        .extern CLC3CONL
        .equiv  CLC3CONLL, _CLC3CONL
        .equiv  CLC3CONLH, _CLC3CONL+1

        .extern CLC3CONH
        .equiv  CLC3CONHL, _CLC3CONH
        .equiv  CLC3CONHH, _CLC3CONH+1

        .extern CLC3SEL

        .extern CLC3SELL
        .equiv  CLC3SELLL, _CLC3SELL
        .equiv  CLC3SELLH, _CLC3SELL+1

        .extern CLC3GLSL
        .equiv  CLC3GLSLL, _CLC3GLSL
        .equiv  CLC3GLSLH, _CLC3GLSL+1

        .extern CLC3GLSH
        .equiv  CLC3GLSHL, _CLC3GLSH
        .equiv  CLC3GLSHH, _CLC3GLSH+1

        .extern CLC4CONL
        .equiv  CLC4CONLL, _CLC4CONL
        .equiv  CLC4CONLH, _CLC4CONL+1

        .extern CLC4CONH
        .equiv  CLC4CONHL, _CLC4CONH
        .equiv  CLC4CONHH, _CLC4CONH+1

        .extern CLC4SEL

        .extern CLC4SELL
        .equiv  CLC4SELLL, _CLC4SELL
        .equiv  CLC4SELLH, _CLC4SELL+1

        .extern CLC4GLSL
        .equiv  CLC4GLSLL, _CLC4GLSL
        .equiv  CLC4GLSLH, _CLC4GLSL+1

        .extern CLC4GLSH
        .equiv  CLC4GLSHL, _CLC4GLSH
        .equiv  CLC4GLSHH, _CLC4GLSH+1

        .extern I2C1RCV
        .equiv  I2C1RCVL, _I2C1RCV
        .equiv  I2C1RCVH, _I2C1RCV+1

        .extern I2C1TRN
        .equiv  I2C1TRNL, _I2C1TRN
        .equiv  I2C1TRNH, _I2C1TRN+1

        .extern I2C1BRG
        .equiv  I2C1BRGL, _I2C1BRG
        .equiv  I2C1BRGH, _I2C1BRG+1

        .extern I2C1CON1
        .equiv  I2C1CON1L, _I2C1CON1
        .equiv  I2C1CON1H, _I2C1CON1+1

        .extern I2C1CONL
        .equiv  I2C1CONLL, _I2C1CONL
        .equiv  I2C1CONLH, _I2C1CONL+1

        .extern I2C1CON2
        .equiv  I2C1CON2L, _I2C1CON2
        .equiv  I2C1CON2H, _I2C1CON2+1

        .extern I2C1CONH
        .equiv  I2C1CONHL, _I2C1CONH
        .equiv  I2C1CONHH, _I2C1CONH+1

        .extern I2C1STAT
        .equiv  I2C1STATL, _I2C1STAT
        .equiv  I2C1STATH, _I2C1STAT+1

        .extern I2C1ADD
        .equiv  I2C1ADDL, _I2C1ADD
        .equiv  I2C1ADDH, _I2C1ADD+1

        .extern I2C1MSK
        .equiv  I2C1MSKL, _I2C1MSK
        .equiv  I2C1MSKH, _I2C1MSK+1

        .extern I2C2RCV
        .equiv  I2C2RCVL, _I2C2RCV
        .equiv  I2C2RCVH, _I2C2RCV+1

        .extern I2C2TRN
        .equiv  I2C2TRNL, _I2C2TRN
        .equiv  I2C2TRNH, _I2C2TRN+1

        .extern I2C2BRG
        .equiv  I2C2BRGL, _I2C2BRG
        .equiv  I2C2BRGH, _I2C2BRG+1

        .extern I2C2CON1
        .equiv  I2C2CON1L, _I2C2CON1
        .equiv  I2C2CON1H, _I2C2CON1+1

        .extern I2C2CONL
        .equiv  I2C2CONLL, _I2C2CONL
        .equiv  I2C2CONLH, _I2C2CONL+1

        .extern I2C2CON2
        .equiv  I2C2CON2L, _I2C2CON2
        .equiv  I2C2CON2H, _I2C2CON2+1

        .extern I2C2CONH
        .equiv  I2C2CONHL, _I2C2CONH
        .equiv  I2C2CONHH, _I2C2CONH+1

        .extern I2C2STAT
        .equiv  I2C2STATL, _I2C2STAT
        .equiv  I2C2STATH, _I2C2STAT+1

        .extern I2C2ADD
        .equiv  I2C2ADDL, _I2C2ADD
        .equiv  I2C2ADDH, _I2C2ADD+1

        .extern I2C2MSK
        .equiv  I2C2MSKL, _I2C2MSK
        .equiv  I2C2MSKH, _I2C2MSK+1

        .extern I2C3RCV
        .equiv  I2C3RCVL, _I2C3RCV
        .equiv  I2C3RCVH, _I2C3RCV+1

        .extern I2C3TRN
        .equiv  I2C3TRNL, _I2C3TRN
        .equiv  I2C3TRNH, _I2C3TRN+1

        .extern I2C3BRG
        .equiv  I2C3BRGL, _I2C3BRG
        .equiv  I2C3BRGH, _I2C3BRG+1

        .extern I2C3CON1
        .equiv  I2C3CON1L, _I2C3CON1
        .equiv  I2C3CON1H, _I2C3CON1+1

        .extern I2C3CONL
        .equiv  I2C3CONLL, _I2C3CONL
        .equiv  I2C3CONLH, _I2C3CONL+1

        .extern I2C3CON2
        .equiv  I2C3CON2L, _I2C3CON2
        .equiv  I2C3CON2H, _I2C3CON2+1

        .extern I2C3CONH
        .equiv  I2C3CONHL, _I2C3CONH
        .equiv  I2C3CONHH, _I2C3CONH+1

        .extern I2C3STAT
        .equiv  I2C3STATL, _I2C3STAT
        .equiv  I2C3STATH, _I2C3STAT+1

        .extern I2C3ADD
        .equiv  I2C3ADDL, _I2C3ADD
        .equiv  I2C3ADDH, _I2C3ADD+1

        .extern I2C3MSK
        .equiv  I2C3MSKL, _I2C3MSK
        .equiv  I2C3MSKH, _I2C3MSK+1

        .extern DMACON
        .equiv  DMACONL, _DMACON
        .equiv  DMACONH, _DMACON+1

        .extern DMABUF
        .equiv  DMABUFL, _DMABUF
        .equiv  DMABUFH, _DMABUF+1

        .extern DMAL
        .equiv  DMALL, _DMAL
        .equiv  DMALH, _DMAL+1

        .extern DMAH
        .equiv  DMAHL, _DMAH
        .equiv  DMAHH, _DMAH+1

        .extern DMACH0
        .equiv  DMACH0L, _DMACH0
        .equiv  DMACH0H, _DMACH0+1

        .extern DMAINT0
        .equiv  DMAINT0L, _DMAINT0
        .equiv  DMAINT0H, _DMAINT0+1

        .extern DMASRC0
        .equiv  DMASRC0L, _DMASRC0
        .equiv  DMASRC0H, _DMASRC0+1

        .extern DMADST0
        .equiv  DMADST0L, _DMADST0
        .equiv  DMADST0H, _DMADST0+1

        .extern DMACNT0
        .equiv  DMACNT0L, _DMACNT0
        .equiv  DMACNT0H, _DMACNT0+1

        .extern DMACH1
        .equiv  DMACH1L, _DMACH1
        .equiv  DMACH1H, _DMACH1+1

        .extern DMAINT1
        .equiv  DMAINT1L, _DMAINT1
        .equiv  DMAINT1H, _DMAINT1+1

        .extern DMASRC1
        .equiv  DMASRC1L, _DMASRC1
        .equiv  DMASRC1H, _DMASRC1+1

        .extern DMADST1
        .equiv  DMADST1L, _DMADST1
        .equiv  DMADST1H, _DMADST1+1

        .extern DMACNT1
        .equiv  DMACNT1L, _DMACNT1
        .equiv  DMACNT1H, _DMACNT1+1

        .extern DMACH2
        .equiv  DMACH2L, _DMACH2
        .equiv  DMACH2H, _DMACH2+1

        .extern DMAINT2
        .equiv  DMAINT2L, _DMAINT2
        .equiv  DMAINT2H, _DMAINT2+1

        .extern DMASRC2
        .equiv  DMASRC2L, _DMASRC2
        .equiv  DMASRC2H, _DMASRC2+1

        .extern DMADST2
        .equiv  DMADST2L, _DMADST2
        .equiv  DMADST2H, _DMADST2+1

        .extern DMACNT2
        .equiv  DMACNT2L, _DMACNT2
        .equiv  DMACNT2H, _DMACNT2+1

        .extern DMACH3
        .equiv  DMACH3L, _DMACH3
        .equiv  DMACH3H, _DMACH3+1

        .extern DMAINT3
        .equiv  DMAINT3L, _DMAINT3
        .equiv  DMAINT3H, _DMAINT3+1

        .extern DMASRC3
        .equiv  DMASRC3L, _DMASRC3
        .equiv  DMASRC3H, _DMASRC3+1

        .extern DMADST3
        .equiv  DMADST3L, _DMADST3
        .equiv  DMADST3H, _DMADST3+1

        .extern DMACNT3
        .equiv  DMACNT3L, _DMACNT3
        .equiv  DMACNT3H, _DMACNT3+1

        .extern DMACH4
        .equiv  DMACH4L, _DMACH4
        .equiv  DMACH4H, _DMACH4+1

        .extern DMAINT4
        .equiv  DMAINT4L, _DMAINT4
        .equiv  DMAINT4H, _DMAINT4+1

        .extern DMASRC4
        .equiv  DMASRC4L, _DMASRC4
        .equiv  DMASRC4H, _DMASRC4+1

        .extern DMADST4
        .equiv  DMADST4L, _DMADST4
        .equiv  DMADST4H, _DMADST4+1

        .extern DMACNT4
        .equiv  DMACNT4L, _DMACNT4
        .equiv  DMACNT4H, _DMACNT4+1

        .extern DMACH5
        .equiv  DMACH5L, _DMACH5
        .equiv  DMACH5H, _DMACH5+1

        .extern DMAINT5
        .equiv  DMAINT5L, _DMAINT5
        .equiv  DMAINT5H, _DMAINT5+1

        .extern DMASRC5
        .equiv  DMASRC5L, _DMASRC5
        .equiv  DMASRC5H, _DMASRC5+1

        .extern DMADST5
        .equiv  DMADST5L, _DMADST5
        .equiv  DMADST5H, _DMADST5+1

        .extern DMACNT5
        .equiv  DMACNT5L, _DMACNT5
        .equiv  DMACNT5H, _DMACNT5+1

        .extern DMACH6
        .equiv  DMACH6L, _DMACH6
        .equiv  DMACH6H, _DMACH6+1

        .extern DMAINT6
        .equiv  DMAINT6L, _DMAINT6
        .equiv  DMAINT6H, _DMAINT6+1

        .extern DMASRC6
        .equiv  DMASRC6L, _DMASRC6
        .equiv  DMASRC6H, _DMASRC6+1

        .extern DMADST6
        .equiv  DMADST6L, _DMADST6
        .equiv  DMADST6H, _DMADST6+1

        .extern DMACNT6
        .equiv  DMACNT6L, _DMACNT6
        .equiv  DMACNT6H, _DMACNT6+1

        .extern DMACH7
        .equiv  DMACH7L, _DMACH7
        .equiv  DMACH7H, _DMACH7+1

        .extern DMAINT7
        .equiv  DMAINT7L, _DMAINT7
        .equiv  DMAINT7H, _DMAINT7+1

        .extern DMASRC7
        .equiv  DMASRC7L, _DMASRC7
        .equiv  DMASRC7H, _DMASRC7+1

        .extern DMADST7
        .equiv  DMADST7L, _DMADST7
        .equiv  DMADST7H, _DMADST7+1

        .extern DMACNT7
        .equiv  DMACNT7L, _DMACNT7
        .equiv  DMACNT7H, _DMACNT7+1

        .extern PADCON
        .equiv  PADCONL, _PADCON
        .equiv  PADCONH, _PADCON+1

        .extern IOCSTAT
        .equiv  IOCSTATL, _IOCSTAT
        .equiv  IOCSTATH, _IOCSTAT+1

        .extern TRISB
        .equiv  TRISBL, _TRISB
        .equiv  TRISBH, _TRISB+1

        .extern PORTB
        .equiv  PORTBL, _PORTB
        .equiv  PORTBH, _PORTB+1

        .extern LATB
        .equiv  LATBL, _LATB
        .equiv  LATBH, _LATB+1

        .extern ODCB
        .equiv  ODCBL, _ODCB
        .equiv  ODCBH, _ODCB+1

        .extern ANSB
        .equiv  ANSBL, _ANSB
        .equiv  ANSBH, _ANSB+1

        .extern ANSELB
        .equiv  ANSELBL, _ANSELB
        .equiv  ANSELBH, _ANSELB+1

        .extern IOCPB
        .equiv  IOCPBL, _IOCPB
        .equiv  IOCPBH, _IOCPB+1

        .extern IOCNB
        .equiv  IOCNBL, _IOCNB
        .equiv  IOCNBH, _IOCNB+1

        .extern IOCFB
        .equiv  IOCFBL, _IOCFB
        .equiv  IOCFBH, _IOCFB+1

        .extern CNPUB
        .equiv  CNPUBL, _CNPUB
        .equiv  CNPUBH, _CNPUB+1

        .extern IOCPUB
        .equiv  IOCPUBL, _IOCPUB
        .equiv  IOCPUBH, _IOCPUB+1

        .extern CNPDB
        .equiv  CNPDBL, _CNPDB
        .equiv  CNPDBH, _CNPDB+1

        .extern IOCPDB
        .equiv  IOCPDBL, _IOCPDB
        .equiv  IOCPDBH, _IOCPDB+1

        .extern TRISC
        .equiv  TRISCL, _TRISC
        .equiv  TRISCH, _TRISC+1

        .extern PORTC
        .equiv  PORTCL, _PORTC
        .equiv  PORTCH, _PORTC+1

        .extern LATC
        .equiv  LATCL, _LATC
        .equiv  LATCH, _LATC+1

        .extern ODCC
        .equiv  ODCCL, _ODCC
        .equiv  ODCCH, _ODCC+1

        .extern ANSC
        .equiv  ANSCL, _ANSC
        .equiv  ANSCH, _ANSC+1

        .extern ANSELC
        .equiv  ANSELCL, _ANSELC
        .equiv  ANSELCH, _ANSELC+1

        .extern IOCPC
        .equiv  IOCPCL, _IOCPC
        .equiv  IOCPCH, _IOCPC+1

        .extern IOCNC
        .equiv  IOCNCL, _IOCNC
        .equiv  IOCNCH, _IOCNC+1

        .extern IOCFC
        .equiv  IOCFCL, _IOCFC
        .equiv  IOCFCH, _IOCFC+1

        .extern CNPUC
        .equiv  CNPUCL, _CNPUC
        .equiv  CNPUCH, _CNPUC+1

        .extern IOCPUC
        .equiv  IOCPUCL, _IOCPUC
        .equiv  IOCPUCH, _IOCPUC+1

        .extern CNPDC
        .equiv  CNPDCL, _CNPDC
        .equiv  CNPDCH, _CNPDC+1

        .extern IOCPDC
        .equiv  IOCPDCL, _IOCPDC
        .equiv  IOCPDCH, _IOCPDC+1

        .extern TRISD
        .equiv  TRISDL, _TRISD
        .equiv  TRISDH, _TRISD+1

        .extern PORTD
        .equiv  PORTDL, _PORTD
        .equiv  PORTDH, _PORTD+1

        .extern LATD
        .equiv  LATDL, _LATD
        .equiv  LATDH, _LATD+1

        .extern ODCD
        .equiv  ODCDL, _ODCD
        .equiv  ODCDH, _ODCD+1

        .extern ANSD
        .equiv  ANSDL, _ANSD
        .equiv  ANSDH, _ANSD+1

        .extern ANSELD
        .equiv  ANSELDL, _ANSELD
        .equiv  ANSELDH, _ANSELD+1

        .extern IOCPD
        .equiv  IOCPDL, _IOCPD
        .equiv  IOCPDH, _IOCPD+1

        .extern IOCND
        .equiv  IOCNDL, _IOCND
        .equiv  IOCNDH, _IOCND+1

        .extern IOCFD
        .equiv  IOCFDL, _IOCFD
        .equiv  IOCFDH, _IOCFD+1

        .extern CNPUD
        .equiv  CNPUDL, _CNPUD
        .equiv  CNPUDH, _CNPUD+1

        .extern IOCPUD
        .equiv  IOCPUDL, _IOCPUD
        .equiv  IOCPUDH, _IOCPUD+1

        .extern CNPDD
        .equiv  CNPDDL, _CNPDD
        .equiv  CNPDDH, _CNPDD+1

        .extern IOCPDD
        .equiv  IOCPDDL, _IOCPDD
        .equiv  IOCPDDH, _IOCPDD+1

        .extern TRISE
        .equiv  TRISEL, _TRISE
        .equiv  TRISEH, _TRISE+1

        .extern PORTE
        .equiv  PORTEL, _PORTE
        .equiv  PORTEH, _PORTE+1

        .extern LATE
        .equiv  LATEL, _LATE
        .equiv  LATEH, _LATE+1

        .extern ODCE
        .equiv  ODCEL, _ODCE
        .equiv  ODCEH, _ODCE+1

        .extern ANSE
        .equiv  ANSEL, _ANSE
        .equiv  ANSEH, _ANSE+1

        .extern ANSELE
        .equiv  ANSELEL, _ANSELE
        .equiv  ANSELEH, _ANSELE+1

        .extern IOCPE
        .equiv  IOCPEL, _IOCPE
        .equiv  IOCPEH, _IOCPE+1

        .extern IOCNE
        .equiv  IOCNEL, _IOCNE
        .equiv  IOCNEH, _IOCNE+1

        .extern IOCFE
        .equiv  IOCFEL, _IOCFE
        .equiv  IOCFEH, _IOCFE+1

        .extern CNPUE
        .equiv  CNPUEL, _CNPUE
        .equiv  CNPUEH, _CNPUE+1

        .extern IOCPUE
        .equiv  IOCPUEL, _IOCPUE
        .equiv  IOCPUEH, _IOCPUE+1

        .extern CNPDE
        .equiv  CNPDEL, _CNPDE
        .equiv  CNPDEH, _CNPDE+1

        .extern IOCPDE
        .equiv  IOCPDEL, _IOCPDE
        .equiv  IOCPDEH, _IOCPDE+1

        .extern TRISF
        .equiv  TRISFL, _TRISF
        .equiv  TRISFH, _TRISF+1

        .extern PORTF
        .equiv  PORTFL, _PORTF
        .equiv  PORTFH, _PORTF+1

        .extern LATF
        .equiv  LATFL, _LATF
        .equiv  LATFH, _LATF+1

        .extern ODCF
        .equiv  ODCFL, _ODCF
        .equiv  ODCFH, _ODCF+1

        .extern ANSELF
        .equiv  ANSELFL, _ANSELF
        .equiv  ANSELFH, _ANSELF+1

        .extern ANSF
        .equiv  ANSFL, _ANSF
        .equiv  ANSFH, _ANSF+1

        .extern IOCPF
        .equiv  IOCPFL, _IOCPF
        .equiv  IOCPFH, _IOCPF+1

        .extern IOCNF
        .equiv  IOCNFL, _IOCNF
        .equiv  IOCNFH, _IOCNF+1

        .extern IOCFF
        .equiv  IOCFFL, _IOCFF
        .equiv  IOCFFH, _IOCFF+1

        .extern CNPUF
        .equiv  CNPUFL, _CNPUF
        .equiv  CNPUFH, _CNPUF+1

        .extern IOCPUF
        .equiv  IOCPUFL, _IOCPUF
        .equiv  IOCPUFH, _IOCPUF+1

        .extern CNPDF
        .equiv  CNPDFL, _CNPDF
        .equiv  CNPDFH, _CNPDF+1

        .extern IOCPDF
        .equiv  IOCPDFL, _IOCPDF
        .equiv  IOCPDFH, _IOCPDF+1

        .extern TRISG
        .equiv  TRISGL, _TRISG
        .equiv  TRISGH, _TRISG+1

        .extern PORTG
        .equiv  PORTGL, _PORTG
        .equiv  PORTGH, _PORTG+1

        .extern LATG
        .equiv  LATGL, _LATG
        .equiv  LATGH, _LATG+1

        .extern ODCG
        .equiv  ODCGL, _ODCG
        .equiv  ODCGH, _ODCG+1

        .extern ANSELG
        .equiv  ANSELGL, _ANSELG
        .equiv  ANSELGH, _ANSELG+1

        .extern ANSG
        .equiv  ANSGL, _ANSG
        .equiv  ANSGH, _ANSG+1

        .extern IOCPG
        .equiv  IOCPGL, _IOCPG
        .equiv  IOCPGH, _IOCPG+1

        .extern IOCNG
        .equiv  IOCNGL, _IOCNG
        .equiv  IOCNGH, _IOCNG+1

        .extern IOCFG
        .equiv  IOCFGL, _IOCFG
        .equiv  IOCFGH, _IOCFG+1

        .extern CNPUG
        .equiv  CNPUGL, _CNPUG
        .equiv  CNPUGH, _CNPUG+1

        .extern IOCPUG
        .equiv  IOCPUGL, _IOCPUG
        .equiv  IOCPUGH, _IOCPUG+1

        .extern CNPDG
        .equiv  CNPDGL, _CNPDG
        .equiv  CNPDGH, _CNPDG+1

        .extern IOCPDG
        .equiv  IOCPDGL, _IOCPDG
        .equiv  IOCPDGH, _IOCPDG+1

        .extern ADC1BUF0
        .equiv  ADC1BUF0L, _ADC1BUF0
        .equiv  ADC1BUF0H, _ADC1BUF0+1

        .extern ADC1BUF1
        .equiv  ADC1BUF1L, _ADC1BUF1
        .equiv  ADC1BUF1H, _ADC1BUF1+1

        .extern ADC1BUF2
        .equiv  ADC1BUF2L, _ADC1BUF2
        .equiv  ADC1BUF2H, _ADC1BUF2+1

        .extern ADC1BUF3
        .equiv  ADC1BUF3L, _ADC1BUF3
        .equiv  ADC1BUF3H, _ADC1BUF3+1

        .extern ADC1BUF4
        .equiv  ADC1BUF4L, _ADC1BUF4
        .equiv  ADC1BUF4H, _ADC1BUF4+1

        .extern ADC1BUF5
        .equiv  ADC1BUF5L, _ADC1BUF5
        .equiv  ADC1BUF5H, _ADC1BUF5+1

        .extern ADC1BUF6
        .equiv  ADC1BUF6L, _ADC1BUF6
        .equiv  ADC1BUF6H, _ADC1BUF6+1

        .extern ADC1BUF7
        .equiv  ADC1BUF7L, _ADC1BUF7
        .equiv  ADC1BUF7H, _ADC1BUF7+1

        .extern ADC1BUF8
        .equiv  ADC1BUF8L, _ADC1BUF8
        .equiv  ADC1BUF8H, _ADC1BUF8+1

        .extern ADC1BUF9
        .equiv  ADC1BUF9L, _ADC1BUF9
        .equiv  ADC1BUF9H, _ADC1BUF9+1

        .extern ADC1BUF10
        .equiv  ADC1BUF10L, _ADC1BUF10
        .equiv  ADC1BUF10H, _ADC1BUF10+1

        .extern ADC1BUF11
        .equiv  ADC1BUF11L, _ADC1BUF11
        .equiv  ADC1BUF11H, _ADC1BUF11+1

        .extern ADC1BUF12
        .equiv  ADC1BUF12L, _ADC1BUF12
        .equiv  ADC1BUF12H, _ADC1BUF12+1

        .extern ADC1BUF13
        .equiv  ADC1BUF13L, _ADC1BUF13
        .equiv  ADC1BUF13H, _ADC1BUF13+1

        .extern ADC1BUF14
        .equiv  ADC1BUF14L, _ADC1BUF14
        .equiv  ADC1BUF14H, _ADC1BUF14+1

        .extern ADC1BUF15
        .equiv  ADC1BUF15L, _ADC1BUF15
        .equiv  ADC1BUF15H, _ADC1BUF15+1

        .extern ADC1BUF16
        .equiv  ADC1BUF16L, _ADC1BUF16
        .equiv  ADC1BUF16H, _ADC1BUF16+1

        .extern ADC1BUF17
        .equiv  ADC1BUF17L, _ADC1BUF17
        .equiv  ADC1BUF17H, _ADC1BUF17+1

        .extern ADC1BUF18
        .equiv  ADC1BUF18L, _ADC1BUF18
        .equiv  ADC1BUF18H, _ADC1BUF18+1

        .extern ADC1BUF19
        .equiv  ADC1BUF19L, _ADC1BUF19
        .equiv  ADC1BUF19H, _ADC1BUF19+1

        .extern ADC1BUF20
        .equiv  ADC1BUF20L, _ADC1BUF20
        .equiv  ADC1BUF20H, _ADC1BUF20+1

        .extern ADC1BUF21
        .equiv  ADC1BUF21L, _ADC1BUF21
        .equiv  ADC1BUF21H, _ADC1BUF21+1

        .extern ADC1BUF22
        .equiv  ADC1BUF22L, _ADC1BUF22
        .equiv  ADC1BUF22H, _ADC1BUF22+1

        .extern ADC1BUF23
        .equiv  ADC1BUF23L, _ADC1BUF23
        .equiv  ADC1BUF23H, _ADC1BUF23+1

        .extern ADC1BUF24
        .equiv  ADC1BUF24L, _ADC1BUF24
        .equiv  ADC1BUF24H, _ADC1BUF24+1

        .extern ADC1BUF25
        .equiv  ADC1BUF25L, _ADC1BUF25
        .equiv  ADC1BUF25H, _ADC1BUF25+1

        .extern AD1CON1
        .equiv  AD1CON1L, _AD1CON1
        .equiv  AD1CON1H, _AD1CON1+1

        .extern AD1CON2
        .equiv  AD1CON2L, _AD1CON2
        .equiv  AD1CON2H, _AD1CON2+1

        .extern AD1CON3
        .equiv  AD1CON3L, _AD1CON3
        .equiv  AD1CON3H, _AD1CON3+1

        .extern AD1CHS
        .equiv  AD1CHSL, _AD1CHS
        .equiv  AD1CHSH, _AD1CHS+1

        .extern AD1CHS0
        .equiv  AD1CHS0L, _AD1CHS0
        .equiv  AD1CHS0H, _AD1CHS0+1

        .extern AD1CSSH
        .equiv  AD1CSSHL, _AD1CSSH
        .equiv  AD1CSSHH, _AD1CSSH+1

        .extern AD1CSSL
        .equiv  AD1CSSLL, _AD1CSSL
        .equiv  AD1CSSLH, _AD1CSSL+1

        .extern AD1CON4
        .equiv  AD1CON4L, _AD1CON4
        .equiv  AD1CON4H, _AD1CON4+1

        .extern AD1CON5
        .equiv  AD1CON5L, _AD1CON5
        .equiv  AD1CON5H, _AD1CON5+1

        .extern AD1CHITH
        .equiv  AD1CHITHL, _AD1CHITH
        .equiv  AD1CHITHH, _AD1CHITH+1

        .extern AD1CHITL
        .equiv  AD1CHITLL, _AD1CHITL
        .equiv  AD1CHITLH, _AD1CHITL+1

        .extern AD1CTMENH
        .equiv  AD1CTMENHL, _AD1CTMENH
        .equiv  AD1CTMENHH, _AD1CTMENH+1

        .extern AD1CTMENL
        .equiv  AD1CTMENLL, _AD1CTMENL
        .equiv  AD1CTMENLH, _AD1CTMENL+1

        .extern AD1RESDMA
        .equiv  AD1RESDMAL, _AD1RESDMA
        .equiv  AD1RESDMAH, _AD1RESDMA+1

        .extern NVMCON
        .equiv  NVMCONL, _NVMCON
        .equiv  NVMCONH, _NVMCON+1

        .extern NVMADR

        .extern NVMADRL
        .equiv  NVMADRLL, _NVMADRL
        .equiv  NVMADRLH, _NVMADRL+1

        .extern NVMADRH
        .equiv  NVMADRHL, _NVMADRH
        .equiv  NVMADRHH, _NVMADRH+1

        .extern NVMADRU
        .equiv  NVMADRUL, _NVMADRU
        .equiv  NVMADRUH, _NVMADRU+1

        .extern NVMKEY
        .equiv  NVMKEYL, _NVMKEY
        .equiv  NVMKEYH, _NVMKEY+1

        .extern RPINR0
        .equiv  RPINR0L, _RPINR0
        .equiv  RPINR0H, _RPINR0+1

        .extern RPINR1
        .equiv  RPINR1L, _RPINR1
        .equiv  RPINR1H, _RPINR1+1

        .extern RPINR2
        .equiv  RPINR2L, _RPINR2
        .equiv  RPINR2H, _RPINR2+1

        .extern RPINR3
        .equiv  RPINR3L, _RPINR3
        .equiv  RPINR3H, _RPINR3+1

        .extern RPINR4
        .equiv  RPINR4L, _RPINR4
        .equiv  RPINR4H, _RPINR4+1

        .extern RPINR7
        .equiv  RPINR7L, _RPINR7
        .equiv  RPINR7H, _RPINR7+1

        .extern RPINR8
        .equiv  RPINR8L, _RPINR8
        .equiv  RPINR8H, _RPINR8+1

        .extern RPINR11
        .equiv  RPINR11L, _RPINR11
        .equiv  RPINR11H, _RPINR11+1

        .extern RPINR12
        .equiv  RPINR12L, _RPINR12
        .equiv  RPINR12H, _RPINR12+1

        .extern RPINR17
        .equiv  RPINR17L, _RPINR17
        .equiv  RPINR17H, _RPINR17+1

        .extern RPINR18
        .equiv  RPINR18L, _RPINR18
        .equiv  RPINR18H, _RPINR18+1

        .extern RPINR19
        .equiv  RPINR19L, _RPINR19
        .equiv  RPINR19H, _RPINR19+1

        .extern RPINR20
        .equiv  RPINR20L, _RPINR20
        .equiv  RPINR20H, _RPINR20+1

        .extern RPINR21
        .equiv  RPINR21L, _RPINR21
        .equiv  RPINR21H, _RPINR21+1

        .extern RPINR22
        .equiv  RPINR22L, _RPINR22
        .equiv  RPINR22H, _RPINR22+1

        .extern RPINR23
        .equiv  RPINR23L, _RPINR23
        .equiv  RPINR23H, _RPINR23+1

        .extern RPINR25
        .equiv  RPINR25L, _RPINR25
        .equiv  RPINR25H, _RPINR25+1

        .extern RPINR27
        .equiv  RPINR27L, _RPINR27
        .equiv  RPINR27H, _RPINR27+1

        .extern RPINR28
        .equiv  RPINR28L, _RPINR28
        .equiv  RPINR28H, _RPINR28+1

        .extern RPINR29
        .equiv  RPINR29L, _RPINR29
        .equiv  RPINR29H, _RPINR29+1

        .extern RPOR0
        .equiv  RPOR0L, _RPOR0
        .equiv  RPOR0H, _RPOR0+1

        .extern RPOR1
        .equiv  RPOR1L, _RPOR1
        .equiv  RPOR1H, _RPOR1+1

        .extern RPOR2
        .equiv  RPOR2L, _RPOR2
        .equiv  RPOR2H, _RPOR2+1

        .extern RPOR3
        .equiv  RPOR3L, _RPOR3
        .equiv  RPOR3H, _RPOR3+1

        .extern RPOR4
        .equiv  RPOR4L, _RPOR4
        .equiv  RPOR4H, _RPOR4+1

        .extern RPOR5
        .equiv  RPOR5L, _RPOR5
        .equiv  RPOR5H, _RPOR5+1

        .extern RPOR6
        .equiv  RPOR6L, _RPOR6
        .equiv  RPOR6H, _RPOR6+1

        .extern RPOR7
        .equiv  RPOR7L, _RPOR7
        .equiv  RPOR7H, _RPOR7+1

        .extern RPOR8
        .equiv  RPOR8L, _RPOR8
        .equiv  RPOR8H, _RPOR8+1

        .extern RPOR9
        .equiv  RPOR9L, _RPOR9
        .equiv  RPOR9H, _RPOR9+1

        .extern RPOR10
        .equiv  RPOR10L, _RPOR10
        .equiv  RPOR10H, _RPOR10+1

        .extern RPOR11
        .equiv  RPOR11L, _RPOR11
        .equiv  RPOR11H, _RPOR11+1

        .extern RPOR12
        .equiv  RPOR12L, _RPOR12
        .equiv  RPOR12H, _RPOR12+1

        .extern RPOR13
        .equiv  RPOR13L, _RPOR13
        .equiv  RPOR13H, _RPOR13+1

        .extern RPOR14
        .equiv  RPOR14L, _RPOR14
        .equiv  RPOR14H, _RPOR14+1

        .extern RPOR15
        .equiv  RPOR15L, _RPOR15
        .equiv  RPOR15H, _RPOR15+1

;----- SR Bits -----------------------------------------------------
        .equiv C,       0x0000
        .equiv Z,       0x0001
        .equiv OV,      0x0002
        .equiv N,       0x0003
        .equiv RA,      0x0004
        .equiv IPL0,    0x0005
        .equiv IPL1,    0x0006
        .equiv IPL2,    0x0007
        .equiv DC,      0x0008



;----- CORCON Bits -----------------------------------------------------
        .equiv IPL3,    0x0003


;----- INTCON1 Bits -----------------------------------------------------
        .equiv OSCFAIL, 0x0001
        .equiv STKERR,  0x0002
        .equiv ADDRERR, 0x0003
        .equiv MATHERR, 0x0004
        .equiv NSTDIS,  0x000F


;----- INTCON2 Bits -----------------------------------------------------
        .equiv INT0EP,  0x0000
        .equiv INT1EP,  0x0001
        .equiv INT2EP,  0x0002
        .equiv INT3EP,  0x0003
        .equiv INT4EP,  0x0004
        .equiv AIVTEN,  0x0008
        .equiv SWTRAP,  0x000D
        .equiv DISI,    0x000E
        .equiv GIE,     0x000F


;----- INTCON4 Bits -----------------------------------------------------
        .equiv SGHT,    0x0000
        .equiv ECCDBE,  0x0001


;----- IFS0 Bits -----------------------------------------------------
        .equiv INT0IF,  0x0000
        .equiv IC1IF,   0x0001
        .equiv OC1IF,   0x0002
        .equiv T1IF,    0x0003
        .equiv DMA0IF,  0x0004
        .equiv IC2IF,   0x0005
        .equiv OC2IF,   0x0006
        .equiv T2IF,    0x0007
        .equiv T3IF,    0x0008
        .equiv SPI1IF,  0x0009
        .equiv SPI1TXIF, 0x000A
        .equiv U1RXIF,  0x000B
        .equiv U1TXIF,  0x000C
        .equiv AD1IF,   0x000D
        .equiv DMA1IF,  0x000E
        .equiv NVMIF,   0x000F


;----- IFS1 Bits -----------------------------------------------------
        .equiv SI2C1IF, 0x0000
        .equiv MI2C1IF, 0x0001
        .equiv CMIF,    0x0002
        .equiv IOCIF,   0x0003
        .equiv INT1IF,  0x0004
        .equiv CCP5IF,  0x0006
        .equiv CCP6IF,  0x0007
        .equiv DMA2IF,  0x0008
        .equiv OC3IF,   0x0009
        .equiv OC4IF,   0x000A
        .equiv T4IF,    0x000B
        .equiv T5IF,    0x000C
        .equiv INT2IF,  0x000D
        .equiv U2RXIF,  0x000E
        .equiv U2TXIF,  0x000F


;----- IFS2 Bits -----------------------------------------------------
        .equiv SPI2IF,  0x0000
        .equiv SPI2TXIF, 0x0001
        .equiv DMA3IF,  0x0004
        .equiv IC3IF,   0x0005
        .equiv IC4IF,   0x0006
        .equiv IC5IF,   0x0007
        .equiv IC6IF,   0x0008
        .equiv OC5IF,   0x0009
        .equiv OC6IF,   0x000A
        .equiv CCT3IF,  0x000B
        .equiv CCT4IF,  0x000C
        .equiv PMPIF,   0x000D
        .equiv DMA4IF,  0x000E
        .equiv CCT5IF,  0x000F


;----- IFS3 Bits -----------------------------------------------------
        .equiv CCT6IF,  0x0000
        .equiv SI2C2IF, 0x0001
        .equiv MI2C2IF, 0x0002
        .equiv CCT7IF,  0x0003
        .equiv INT3IF,  0x0005
        .equiv INT4IF,  0x0006
        .equiv SPI1RXIF, 0x000A
        .equiv SPI2RXIF, 0x000B
        .equiv SPI3RXIF, 0x000C
        .equiv DMA5IF,  0x000D
        .equiv RTCIF,   0x000E
        .equiv CCP1IF,  0x000F


;----- IFS4 Bits -----------------------------------------------------
        .equiv CCP2IF,  0x0000
        .equiv U1ERIF,  0x0001
        .equiv U2ERIF,  0x0002
        .equiv CRCIF,   0x0003
        .equiv DMA6IF,  0x0004
        .equiv DMA7IF,  0x0005
        .equiv SI2C3IF, 0x0006
        .equiv MI2C3IF, 0x0007
        .equiv LVDIF,   0x0008
        .equiv CCP7IF,  0x0009
        .equiv CTMUIF,  0x000D


;----- IFS5 Bits -----------------------------------------------------
        .equiv U3ERIF,  0x0001
        .equiv U3RXIF,  0x0002
        .equiv U3TXIF,  0x0003
        .equiv I2C1BCIF, 0x0004
        .equiv I2C2BCIF, 0x0005
        .equiv U4ERIF,  0x0007
        .equiv U4RXIF,  0x0008
        .equiv U4TXIF,  0x0009
        .equiv SPI3IF,  0x000A
        .equiv SPI3TXIF, 0x000B
        .equiv CCP3IF,  0x000E
        .equiv CCP4IF,  0x000F


;----- IFS6 Bits -----------------------------------------------------
        .equiv CLC1IF,  0x0000
        .equiv CLC2IF,  0x0001
        .equiv CLC3IF,  0x0002
        .equiv CLC4IF,  0x0003
        .equiv CCT1IF,  0x0005
        .equiv CCT2IF,  0x0006
        .equiv FSTIF,   0x000A
        .equiv I2C3BCIF, 0x000D
        .equiv RTCCTSIF, 0x000E
        .equiv U5RXIF,  0x000F


;----- IFS7 Bits -----------------------------------------------------
        .equiv U5TXIF,  0x0000
        .equiv U5ERIF,  0x0001
        .equiv U6RXIF,  0x0002
        .equiv U6TXIF,  0x0003
        .equiv U6ERIF,  0x0004
        .equiv JTAGIF,  0x0005


;----- IEC0 Bits -----------------------------------------------------
        .equiv INT0IE,  0x0000
        .equiv IC1IE,   0x0001
        .equiv OC1IE,   0x0002
        .equiv T1IE,    0x0003
        .equiv DMA0IE,  0x0004
        .equiv IC2IE,   0x0005
        .equiv OC2IE,   0x0006
        .equiv T2IE,    0x0007
        .equiv T3IE,    0x0008
        .equiv SPI1IE,  0x0009
        .equiv SPI1TXIE, 0x000A
        .equiv U1RXIE,  0x000B
        .equiv U1TXIE,  0x000C
        .equiv AD1IE,   0x000D
        .equiv DMA1IE,  0x000E
        .equiv NVMIE,   0x000F


;----- IEC1 Bits -----------------------------------------------------
        .equiv SI2C1IE, 0x0000
        .equiv MI2C1IE, 0x0001
        .equiv CMIE,    0x0002
        .equiv IOCIE,   0x0003
        .equiv INT1IE,  0x0004
        .equiv CCP5IE,  0x0006
        .equiv CCP6IE,  0x0007
        .equiv DMA2IE,  0x0008
        .equiv OC3IE,   0x0009
        .equiv OC4IE,   0x000A
        .equiv T4IE,    0x000B
        .equiv T5IE,    0x000C
        .equiv INT2IE,  0x000D
        .equiv U2RXIE,  0x000E
        .equiv U2TXIE,  0x000F


;----- IEC2 Bits -----------------------------------------------------
        .equiv SPI2IE,  0x0000
        .equiv SPI2TXIE, 0x0001
        .equiv DMA3IE,  0x0004
        .equiv IC3IE,   0x0005
        .equiv IC4IE,   0x0006
        .equiv IC5IE,   0x0007
        .equiv IC6IE,   0x0008
        .equiv OC5IE,   0x0009
        .equiv OC6IE,   0x000A
        .equiv CCT3IE,  0x000B
        .equiv CCT4IE,  0x000C
        .equiv PMPIE,   0x000D
        .equiv DMA4IE,  0x000E
        .equiv CCT5IE,  0x000F


;----- IEC3 Bits -----------------------------------------------------
        .equiv CCT6IE,  0x0000
        .equiv SI2C2IE, 0x0001
        .equiv MI2C2IE, 0x0002
        .equiv CCT7IE,  0x0003
        .equiv INT3IE,  0x0005
        .equiv INT4IE,  0x0006
        .equiv SPI1RXIE, 0x000A
        .equiv SPI2RXIE, 0x000B
        .equiv SPI3RXIE, 0x000C
        .equiv DMA5IE,  0x000D
        .equiv RTCIE,   0x000E
        .equiv CCP1IE,  0x000F


;----- IEC4 Bits -----------------------------------------------------
        .equiv CCP2IE,  0x0000
        .equiv U1ERIE,  0x0001
        .equiv U2ERIE,  0x0002
        .equiv CRCIE,   0x0003
        .equiv DMA6IE,  0x0004
        .equiv DMA7IE,  0x0005
        .equiv SI2C3IE, 0x0006
        .equiv MI2C3IE, 0x0007
        .equiv LVDIE,   0x0008
        .equiv CCP7IE,  0x0009
        .equiv CTMUIE,  0x000D


;----- IEC5 Bits -----------------------------------------------------
        .equiv U3ERIE,  0x0001
        .equiv U3RXIE,  0x0002
        .equiv U3TXIE,  0x0003
        .equiv I2C1BCIE, 0x0004
        .equiv I2C2BCIE, 0x0005
        .equiv U4ERIE,  0x0007
        .equiv U4RXIE,  0x0008
        .equiv U4TXIE,  0x0009
        .equiv SPI3IE,  0x000A
        .equiv SPI3TXIE, 0x000B
        .equiv CCP3IE,  0x000E
        .equiv CCP4IE,  0x000F


;----- IEC6 Bits -----------------------------------------------------
        .equiv CLC1IE,  0x0000
        .equiv CLC2IE,  0x0001
        .equiv CLC3IE,  0x0002
        .equiv CLC4IE,  0x0003
        .equiv CCT1IE,  0x0005
        .equiv CCT2IE,  0x0006
        .equiv FSTIE,   0x000A
        .equiv I2C3BCIE, 0x000D
        .equiv RTCCTSIE, 0x000E
        .equiv U5RXIE,  0x000F


;----- IEC7 Bits -----------------------------------------------------
        .equiv U5TXIE,  0x0000
        .equiv U5ERIE,  0x0001
        .equiv U6RXIE,  0x0002
        .equiv U6TXIE,  0x0003
        .equiv U6ERIE,  0x0004
        .equiv JTAGIE,  0x0005


;----- IPC0 Bits -----------------------------------------------------
        .equiv INT0IP0, 0x0000
        .equiv INT0IP1, 0x0001
        .equiv INT0IP2, 0x0002
        .equiv IC1IP0,  0x0004
        .equiv IC1IP1,  0x0005
        .equiv IC1IP2,  0x0006
        .equiv OC1IP0,  0x0008
        .equiv OC1IP1,  0x0009
        .equiv OC1IP2,  0x000A
        .equiv T1IP0,   0x000C
        .equiv T1IP1,   0x000D
        .equiv T1IP2,   0x000E


;----- IPC1 Bits -----------------------------------------------------
        .equiv DMA0IP0, 0x0000
        .equiv DMA0IP1, 0x0001
        .equiv DMA0IP2, 0x0002
        .equiv IC2IP0,  0x0004
        .equiv IC2IP1,  0x0005
        .equiv IC2IP2,  0x0006
        .equiv OC2IP0,  0x0008
        .equiv OC2IP1,  0x0009
        .equiv OC2IP2,  0x000A
        .equiv T2IP0,   0x000C
        .equiv T2IP1,   0x000D
        .equiv T2IP2,   0x000E


;----- IPC2 Bits -----------------------------------------------------
        .equiv T3IP0,   0x0000
        .equiv T3IP1,   0x0001
        .equiv T3IP2,   0x0002
        .equiv SPI1IP0, 0x0004
        .equiv SPI1IP1, 0x0005
        .equiv SPI1IP2, 0x0006
        .equiv SPI1TXIP0, 0x0008
        .equiv SPI1TXIP1, 0x0009
        .equiv SPI1TXIP2, 0x000A
        .equiv U1RXIP0, 0x000C
        .equiv U1RXIP1, 0x000D
        .equiv U1RXIP2, 0x000E


;----- IPC3 Bits -----------------------------------------------------
        .equiv U1TXIP0, 0x0000
        .equiv U1TXIP1, 0x0001
        .equiv U1TXIP2, 0x0002
        .equiv AD1IP0,  0x0004
        .equiv AD1IP1,  0x0005
        .equiv AD1IP2,  0x0006
        .equiv DMA1IP0, 0x0008
        .equiv DMA1IP1, 0x0009
        .equiv DMA1IP2, 0x000A
        .equiv NVMIP0,  0x000C
        .equiv NVMIP1,  0x000D
        .equiv NVMIP2,  0x000E


;----- IPC4 Bits -----------------------------------------------------
        .equiv SI2C1IP0, 0x0000
        .equiv SI2C1IP1, 0x0001
        .equiv SI2C1IP2, 0x0002
        .equiv MI2C1IP0, 0x0004
        .equiv MI2C1IP1, 0x0005
        .equiv MI2C1IP2, 0x0006
        .equiv CMIP0,   0x0008
        .equiv CMIP1,   0x0009
        .equiv CMIP2,   0x000A
        .equiv CNIP0,   0x000C
        .equiv CNIP1,   0x000D
        .equiv CNIP2,   0x000E


        .equiv SI2C1P0, 0x0000
        .equiv SI2C1P1, 0x0001
        .equiv SI2C1P2, 0x0002
        .equiv MI2C1P0, 0x0004
        .equiv MI2C1P1, 0x0005
        .equiv MI2C1P2, 0x0006


;----- IPC5 Bits -----------------------------------------------------
        .equiv INT1IP0, 0x0000
        .equiv INT1IP1, 0x0001
        .equiv INT1IP2, 0x0002
        .equiv CCP5IP0, 0x0008
        .equiv CCP5IP1, 0x0009
        .equiv CCP5IP2, 0x000A
        .equiv CCP6IP0, 0x000C
        .equiv CCP6IP1, 0x000D
        .equiv CCP6IP2, 0x000E


;----- IPC6 Bits -----------------------------------------------------
        .equiv DMA2IP0, 0x0000
        .equiv DMA2IP1, 0x0001
        .equiv DMA2IP2, 0x0002
        .equiv OC3IP0,  0x0004
        .equiv OC3IP1,  0x0005
        .equiv OC3IP2,  0x0006
        .equiv OC4IP0,  0x0008
        .equiv OC4IP1,  0x0009
        .equiv OC4IP2,  0x000A
        .equiv T4IP0,   0x000C
        .equiv T4IP1,   0x000D
        .equiv T4IP2,   0x000E


;----- IPC7 Bits -----------------------------------------------------
        .equiv T5IP0,   0x0000
        .equiv T5IP1,   0x0001
        .equiv T5IP2,   0x0002
        .equiv INT2IP0, 0x0004
        .equiv INT2IP1, 0x0005
        .equiv INT2IP2, 0x0006
        .equiv U2RXIP0, 0x0008
        .equiv U2RXIP1, 0x0009
        .equiv U2RXIP2, 0x000A
        .equiv U2TXIP0, 0x000C
        .equiv U2TXIP1, 0x000D
        .equiv U2TXIP2, 0x000E


;----- IPC8 Bits -----------------------------------------------------
        .equiv SPI2IP0, 0x0000
        .equiv SPI2IP1, 0x0001
        .equiv SPI2IP2, 0x0002
        .equiv SPI2TXIP0, 0x0004
        .equiv SPI2TXIP1, 0x0005
        .equiv SPI2TXIP2, 0x0006


;----- IPC9 Bits -----------------------------------------------------
        .equiv DMA3IP0, 0x0000
        .equiv DMA3IP1, 0x0001
        .equiv DMA3IP2, 0x0002
        .equiv IC3IP0,  0x0004
        .equiv IC3IP1,  0x0005
        .equiv IC3IP2,  0x0006
        .equiv IC4IP0,  0x0008
        .equiv IC4IP1,  0x0009
        .equiv IC4IP2,  0x000A
        .equiv IC5IP0,  0x000C
        .equiv IC5IP1,  0x000D
        .equiv IC5IP2,  0x000E


;----- IPC10 Bits -----------------------------------------------------
        .equiv IC6IP0,  0x0000
        .equiv IC6IP1,  0x0001
        .equiv IC6IP2,  0x0002
        .equiv OC5IP0,  0x0004
        .equiv OC5IP1,  0x0005
        .equiv OC5IP2,  0x0006
        .equiv OC6IP0,  0x0008
        .equiv OC6IP1,  0x0009
        .equiv OC6IP2,  0x000A
        .equiv CCT3IP0, 0x000C
        .equiv CCT3IP1, 0x000D
        .equiv CCT3IP2, 0x000E


;----- IPC11 Bits -----------------------------------------------------
        .equiv CCT4IP0, 0x0000
        .equiv CCT4IP1, 0x0001
        .equiv CCT4IP2, 0x0002
        .equiv PMPIP0,  0x0004
        .equiv PMPIP1,  0x0005
        .equiv PMPIP2,  0x0006
        .equiv DMA4IP0, 0x0008
        .equiv DMA4IP1, 0x0009
        .equiv DMA4IP2, 0x000A
        .equiv CCT5IP0, 0x000C
        .equiv CCT5IP1, 0x000D
        .equiv CCT5IP2, 0x000E


;----- IPC12 Bits -----------------------------------------------------
        .equiv CCT6IP0, 0x0000
        .equiv CCT6IP1, 0x0001
        .equiv CCT6IP2, 0x0002
        .equiv SI2C2IP0, 0x0004
        .equiv SI2C2IP1, 0x0005
        .equiv SI2C2IP2, 0x0006
        .equiv MI2C2IP0, 0x0008
        .equiv MI2C2IP1, 0x0009
        .equiv MI2C2IP2, 0x000A
        .equiv CCT7IP0, 0x000C
        .equiv CCT7IP1, 0x000D
        .equiv CCT7IP2, 0x000E

        .equiv SI2C2P0, 0x0004
        .equiv SI2C2P1, 0x0005
        .equiv SI2C2P2, 0x0006
        .equiv MI2C2P0, 0x0008
        .equiv MI2C2P1, 0x0009
        .equiv MI2C2P2, 0x000A



;----- IPC13 Bits -----------------------------------------------------
        .equiv INT3IP0, 0x0004
        .equiv INT3IP1, 0x0005
        .equiv INT3IP2, 0x0006
        .equiv INT4IP0, 0x0008
        .equiv INT4IP1, 0x0009
        .equiv INT4IP2, 0x000A


;----- IPC14 Bits -----------------------------------------------------
        .equiv SPI1RXIP0, 0x0008
        .equiv SPI1RXIP1, 0x0009
        .equiv SPI1RXIP2, 0x000A
        .equiv SPI2RXIP0, 0x000C
        .equiv SPI2RXIP1, 0x000D
        .equiv SPI2RXIP2, 0x000E


;----- IPC15 Bits -----------------------------------------------------
        .equiv SPI3RXIP0, 0x0000
        .equiv SPI3RXIP1, 0x0001
        .equiv SPI3RXIP2, 0x0002
        .equiv DMA5IP0, 0x0004
        .equiv DMA5IP1, 0x0005
        .equiv DMA5IP2, 0x0006
        .equiv RTCIP0,  0x0008
        .equiv RTCIP1,  0x0009
        .equiv RTCIP2,  0x000A
        .equiv CCP1IP0, 0x000C
        .equiv CCP1IP1, 0x000D
        .equiv CCP1IP2, 0x000E


;----- IPC16 Bits -----------------------------------------------------
        .equiv CCP2IP0, 0x0000
        .equiv CCP2IP1, 0x0001
        .equiv CCP2IP2, 0x0002
        .equiv U1ERIP0, 0x0004
        .equiv U1ERIP1, 0x0005
        .equiv U1ERIP2, 0x0006
        .equiv U2ERIP0, 0x0008
        .equiv U2ERIP1, 0x0009
        .equiv U2ERIP2, 0x000A
        .equiv CRCIP0,  0x000C
        .equiv CRCIP1,  0x000D
        .equiv CRCIP2,  0x000E


;----- IPC17 Bits -----------------------------------------------------
        .equiv DMA6IP0, 0x0000
        .equiv DMA6IP1, 0x0001
        .equiv DMA6IP2, 0x0002
        .equiv DMA7IP0, 0x0004
        .equiv DMA7IP1, 0x0005
        .equiv DMA7IP2, 0x0006
        .equiv SI2C3IP0, 0x0008
        .equiv SI2C3IP1, 0x0009
        .equiv SI2C3IP2, 0x000A
        .equiv MI2C3IP0, 0x000C
        .equiv MI2C3IP1, 0x000D
        .equiv MI2C3IP2, 0x000E


;----- IPC18 Bits -----------------------------------------------------
        .equiv LVDIP0,  0x0000
        .equiv LVDIP1,  0x0001
        .equiv LVDIP2,  0x0002
        .equiv CCP7IP0, 0x0004
        .equiv CCP7IP1, 0x0005
        .equiv CCP7IP2, 0x0006


;----- IPC19 Bits -----------------------------------------------------
        .equiv CTMUIP0, 0x0004
        .equiv CTMUIP1, 0x0005
        .equiv CTMUIP2, 0x0006


;----- IPC20 Bits -----------------------------------------------------
        .equiv U3ERIP0, 0x0004
        .equiv U3ERIP1, 0x0005
        .equiv U3ERIP2, 0x0006
        .equiv U3RXIP0, 0x0008
        .equiv U3RXIP1, 0x0009
        .equiv U3RXIP2, 0x000A
        .equiv U3TXIP0, 0x000C
        .equiv U3TXIP1, 0x000D
        .equiv U3TXIP2, 0x000E


;----- IPC21 Bits -----------------------------------------------------
        .equiv U4ERIP0, 0x000C
        .equiv U4ERIP1, 0x000D
        .equiv U4ERIP2, 0x000E


;----- IPC22 Bits -----------------------------------------------------
        .equiv U4RXIP0, 0x0000
        .equiv U4RXIP1, 0x0001
        .equiv U4RXIP2, 0x0002
        .equiv U4TXIP0, 0x0004
        .equiv U4TXIP1, 0x0005
        .equiv U4TXIP2, 0x0006
        .equiv SPI3IP0, 0x0008
        .equiv SPI3IP1, 0x0009
        .equiv SPI3IP2, 0x000A
        .equiv SPI3TXIP0, 0x000C
        .equiv SPI3TXIP1, 0x000D
        .equiv SPI3TXIP2, 0x000E


;----- IPC23 Bits -----------------------------------------------------
        .equiv CCP3IP0, 0x0008
        .equiv CCP3IP1, 0x0009
        .equiv CCP3IP2, 0x000A
        .equiv CCP4IP0, 0x000C
        .equiv CCP4IP1, 0x000D
        .equiv CCP4IP2, 0x000E


;----- IPC24 Bits -----------------------------------------------------
        .equiv CLC1IP0, 0x0000
        .equiv CLC1IP1, 0x0001
        .equiv CLC1IP2, 0x0002
        .equiv CLC2IP0, 0x0004
        .equiv CLC2IP1, 0x0005
        .equiv CLC2IP2, 0x0006
        .equiv CLC3IP0, 0x0008
        .equiv CLC3IP1, 0x0009
        .equiv CLC3IP2, 0x000A
        .equiv CLC4IP0, 0x000C
        .equiv CLC4IP1, 0x000D
        .equiv CLC4IP2, 0x000E


;----- IPC25 Bits -----------------------------------------------------
        .equiv CCT1IP0, 0x0004
        .equiv CCT1IP1, 0x0005
        .equiv CCT1IP2, 0x0006
        .equiv CCT2IP0, 0x0008
        .equiv CCT2IP1, 0x0009
        .equiv CCT2IP2, 0x000A


;----- IPC26 Bits -----------------------------------------------------
        .equiv FSTIP0,  0x0008
        .equiv FSTIP1,  0x0009
        .equiv FSTIP2,  0x000A


;----- IPC27 Bits -----------------------------------------------------
        .equiv I2C3BCIP0, 0x0004
        .equiv I2C3BCIP1, 0x0005
        .equiv I2C3BCIP2, 0x0006
        .equiv RTCCTCIP0, 0x0008
        .equiv RTCCTSIP1, 0x0009
        .equiv RTCCTSIP2, 0x000A
        .equiv U5RXIP0, 0x000C
        .equiv U5RXIP1, 0x000D
        .equiv U5RXIP2, 0x000E


;----- IPC28 Bits -----------------------------------------------------
        .equiv U5TXIP0, 0x0000
        .equiv U5TXIP1, 0x0001
        .equiv U5TXIP2, 0x0002
        .equiv U5ERIP0, 0x0004
        .equiv U5ERIP1, 0x0005
        .equiv U5ERIP2, 0x0006
        .equiv U6RXIP0, 0x0008
        .equiv U6RXIP1, 0x0009
        .equiv U6RXIP2, 0x000A
        .equiv U6TXIP0, 0x000C
        .equiv U6TXIP1, 0x000D
        .equiv U6TXIP2, 0x000E


;----- IPC29 Bits -----------------------------------------------------
        .equiv U6ERIP0, 0x0000
        .equiv U6ERIP1, 0x0001
        .equiv U6ERIP2, 0x0002
        .equiv JTAGIP0, 0x0004
        .equiv JTAGIP1, 0x0005
        .equiv JTAGIP2, 0x0006


;----- INTTREG Bits -----------------------------------------------------
        .equiv VHOLD,   0x000D
        .equiv CPUIRQ,  0x000F

        .equiv VECNUM0, 0x0000
        .equiv VECNUM1, 0x0001
        .equiv VECNUM2, 0x0002
        .equiv VECNUM3, 0x0003
        .equiv VECNUM4, 0x0004
        .equiv VECNUM5, 0x0005
        .equiv VECNUM6, 0x0006
        .equiv VECNUM7, 0x0007
        .equiv ILR0,    0x0008
        .equiv ILR1,    0x0009
        .equiv ILR2,    0x000A
        .equiv ILR3,    0x000B


;----- OSCCON Bits -----------------------------------------------------
        .equiv OSWEN,   0x0000
        .equiv SOSCEN,  0x0001
        .equiv POSCEN,  0x0002
        .equiv CF,      0x0003
        .equiv LOCK,    0x0005
        .equiv IOLOCK,  0x0006
        .equiv CLKLOCK, 0x0007

        .equiv NOSC0,   0x0008
        .equiv NOSC1,   0x0009
        .equiv NOSC2,   0x000A
        .equiv COSC0,   0x000C
        .equiv COSC1,   0x000D
        .equiv COSC2,   0x000E


;----- CLKDIV Bits -----------------------------------------------------
        .equiv PLLEN,   0x0005
        .equiv DOZEN,   0x000B
        .equiv ROI,     0x000F

        .equiv CPDIV0,  0x0006
        .equiv CPDIV1,  0x0007
        .equiv RCDIV0,  0x0008
        .equiv RCDIV1,  0x0009
        .equiv RCDIV2,  0x000A
        .equiv DOZE0,   0x000C
        .equiv DOZE1,   0x000D
        .equiv DOZE2,   0x000E


;----- OSCTUN Bits -----------------------------------------------------
        .equiv STORPOL, 0x0008
        .equiv STOR,    0x0009
        .equiv STLPOL,  0x000A
        .equiv STLOCK,  0x000B
        .equiv STSRC,   0x000C
        .equiv STSIDL,  0x000D
        .equiv STEN,    0x000F

        .equiv TUN0,    0x0000
        .equiv TUN1,    0x0001
        .equiv TUN2,    0x0002
        .equiv TUN3,    0x0003
        .equiv TUN4,    0x0004
        .equiv TUN5,    0x0005


;----- DCOTUN Bits -----------------------------------------------------
       ;.equiv TUN0,    0x0000
       ;.equiv TUN1,    0x0001
       ;.equiv TUN2,    0x0002
       ;.equiv TUN3,    0x0003
       ;.equiv TUN4,    0x0004
       ;.equiv TUN5,    0x0005


;----- DCOCON Bits -----------------------------------------------------
        .equiv DCOEN,   0x000D

        .equiv DCOFSEL0, 0x0008
        .equiv DCOFSEL1, 0x0009
        .equiv DCOFSEL2, 0x000A
        .equiv DCOFSEL3, 0x000B


;----- OSCDIV Bits -----------------------------------------------------
        .equiv DIV0,    0x0000
        .equiv DIV1,    0x0001
        .equiv DIV2,    0x0002
        .equiv DIV3,    0x0003
        .equiv DIV4,    0x0004
        .equiv DIV5,    0x0005
        .equiv DIV6,    0x0006
        .equiv DIV7,    0x0007
        .equiv DIV8,    0x0008
        .equiv DIV9,    0x0009
        .equiv DIV10,   0x000A
        .equiv DIV11,   0x000B
        .equiv DIV12,   0x000C
        .equiv DIV13,   0x000D
        .equiv DIV14,   0x000E


;----- OSCFDIV Bits -----------------------------------------------------
        .equiv TRIM8,   0x0007
        .equiv TRIM7,   0x0008
        .equiv TRIM6,   0x0009
        .equiv TRIM5,   0x000A
        .equiv TRIM4,   0x000B
        .equiv TRIM3,   0x000C
        .equiv TRIM2,   0x000D
        .equiv TRIM1,   0x000E
        .equiv TRIM0,   0x000F


;----- RCON Bits -----------------------------------------------------
        .equiv POR,     0x0000
        .equiv BOR,     0x0001
        .equiv IDLE,    0x0002
        .equiv SLEEP,   0x0003
        .equiv WDTO,    0x0004
        .equiv SWDTEN,  0x0005
        .equiv SWR,     0x0006
        .equiv EXTR,    0x0007
        .equiv VREGS,   0x0008
        .equiv CM_RCON, 0x0009
        .equiv RETEN,   0x000C
        .equiv SBOREN,  0x000D
        .equiv IOPUWR,  0x000E
        .equiv TRAPR,   0x000F

        .equiv PMSLP,   0x0008


;----- HLVDCON Bits -----------------------------------------------------
        .equiv LVDEVT,  0x0008
        .equiv IRVST,   0x0009
        .equiv BGVST,   0x000A
        .equiv VDIR,    0x000B
        .equiv LSIDL,   0x000D
        .equiv HLVDEN,  0x000F

        .equiv HLVDL0,  0x0000
        .equiv HLVDL1,  0x0001
        .equiv HLVDL2,  0x0002
        .equiv HLVDL3,  0x0003


;----- PMCON1 Bits -----------------------------------------------------
        .equiv BUSKEEP, 0x0002
        .equiv ALMODE,  0x0004
        .equiv ALP,     0x0005
        .equiv PSIDL,   0x000D
        .equiv PMPEN,   0x000F

        .equiv IRQM0,   0x0000
        .equiv IRQM1,   0x0001
        .equiv CSF0,    0x0006
        .equiv CSF1,    0x0007
        .equiv MODE0_PMCON1, 0x0008
        .equiv MODE1_PMCON1, 0x0009
        .equiv ADRMUX0, 0x000B
        .equiv ADRMUX1, 0x000C


;----- PMCON2 Bits -----------------------------------------------------
        .equiv TIMEOUT, 0x000C
        .equiv ERROR,   0x000D
        .equiv BUSY,    0x000F

        .equiv RADDR16, 0x0000
        .equiv RADDR17, 0x0001
        .equiv RADDR18, 0x0002
        .equiv RADDR19, 0x0003
        .equiv RADDR20, 0x0004
        .equiv RADDR21, 0x0005
        .equiv RADDR22, 0x0006
        .equiv RADDR23, 0x0007


;----- PMCON3 Bits -----------------------------------------------------
        .equiv AWAITE,  0x0008
        .equiv PTBE0EN, 0x000C
        .equiv PTBE1EN, 0x000D
        .equiv PTRDEN,  0x000E
        .equiv PTWREN,  0x000F

        .equiv AWAITM0, 0x0009
        .equiv AWAITM1, 0x000A


;----- PMCON4 Bits -----------------------------------------------------
        .equiv PTEN0,   0x0000
        .equiv PTEN1,   0x0001
        .equiv PTEN2,   0x0002
        .equiv PTEN3,   0x0003
        .equiv PTEN4,   0x0004
        .equiv PTEN5,   0x0005
        .equiv PTEN6,   0x0006
        .equiv PTEN7,   0x0007
        .equiv PTEN8,   0x0008
        .equiv PTEN9,   0x0009
        .equiv PTEN10,  0x000A
        .equiv PTEN11,  0x000B
        .equiv PTEN12,  0x000C
        .equiv PTEN13,  0x000D
        .equiv PTEN14,  0x000E
        .equiv PTEN15,  0x000F


;----- PMCS1CF Bits -----------------------------------------------------
        .equiv ACKP,    0x0007
        .equiv SM,      0x0008
        .equiv RDSP,    0x0009
        .equiv WRSP,    0x000A
        .equiv BEP,     0x000C
        .equiv CSPTEN,  0x000D
        .equiv CSP,     0x000E
        .equiv CSDIS,   0x000F

        .equiv PTSZ0,   0x0005
        .equiv PTSZ1,   0x0006


;----- PMCS1BS Bits -----------------------------------------------------
        .equiv BASE11,  0x0003
        .equiv BASE15,  0x0007
        .equiv BASE16,  0x0008
        .equiv BASE17,  0x0009
        .equiv BASE18,  0x000A
        .equiv BASE19,  0x000B
        .equiv BASE20,  0x000C
        .equiv BASE21,  0x000D
        .equiv BASE22,  0x000E
        .equiv BASE23,  0x000F


;----- PMCS1MD Bits -----------------------------------------------------
        .equiv DWAITE0, 0x0000
        .equiv DWAITE1, 0x0001
        .equiv DWAITM0, 0x0002
        .equiv DWAITM1, 0x0003
        .equiv DWAITM2, 0x0004
        .equiv DWAITM3, 0x0005
        .equiv DWAITB0, 0x0006
        .equiv DWAITB1, 0x0007
        .equiv AMWAIT0, 0x000B
        .equiv AMWAIT1, 0x000C
        .equiv AMWAIT2, 0x000D
        .equiv ACKM0,   0x000E
        .equiv ACKM1,   0x000F


;----- PMCS2CF Bits -----------------------------------------------------
       ;.equiv ACKP,    0x0007
       ;.equiv SM,      0x0008
       ;.equiv RDSP,    0x0009
       ;.equiv WRSP,    0x000A
       ;.equiv BEP,     0x000C
       ;.equiv CSPTEN,  0x000D
       ;.equiv CSP,     0x000E
       ;.equiv CSDIS,   0x000F

       ;.equiv PTSZ0,   0x0005
       ;.equiv PTSZ1,   0x0006


;----- PMCS2BS Bits -----------------------------------------------------
       ;.equiv BASE11,  0x0003
       ;.equiv BASE15,  0x0007
       ;.equiv BASE16,  0x0008
       ;.equiv BASE17,  0x0009
       ;.equiv BASE18,  0x000A
       ;.equiv BASE19,  0x000B
       ;.equiv BASE20,  0x000C
       ;.equiv BASE21,  0x000D
       ;.equiv BASE22,  0x000E
       ;.equiv BASE23,  0x000F


;----- PMCS2MD Bits -----------------------------------------------------
       ;.equiv DWAITE0, 0x0000
       ;.equiv DWAITE1, 0x0001
       ;.equiv DWAITM0, 0x0002
       ;.equiv DWAITM1, 0x0003
       ;.equiv DWAITM2, 0x0004
       ;.equiv DWAITM3, 0x0005
       ;.equiv DWAITB0, 0x0006
       ;.equiv DWAITB1, 0x0007
       ;.equiv AMWAIT0, 0x000B
       ;.equiv AMWAIT1, 0x000C
       ;.equiv AMWAIT2, 0x000D
       ;.equiv ACKM0,   0x000E
       ;.equiv ACKM1,   0x000F


;----- PMSTAT Bits -----------------------------------------------------
        .equiv OB0E,    0x0000
        .equiv OB1E,    0x0001
        .equiv OB2E,    0x0002
        .equiv OB3E,    0x0003
        .equiv OBUF,    0x0006
        .equiv OBE,     0x0007
        .equiv IB0F,    0x0008
        .equiv IB1F,    0x0009
        .equiv IB2F,    0x000A
        .equiv IB3F,    0x000B
        .equiv IBOV,    0x000E
        .equiv IBF,     0x000F


;----- CRCCON1 Bits -----------------------------------------------------
        .equiv LENDIAN, 0x0003
        .equiv CRCGO,   0x0004
        .equiv CRCISEL, 0x0005
        .equiv CRCMPT,  0x0006
        .equiv CRCFUL,  0x0007
        .equiv CSIDL,   0x000D
        .equiv CRCEN,   0x000F

        .equiv VWORD0,  0x0008
        .equiv VWORD1,  0x0009
        .equiv VWORD2,  0x000A
        .equiv VWORD3,  0x000B
        .equiv VWORD4,  0x000C


;----- CRCCON2 Bits -----------------------------------------------------
        .equiv PLEN0,   0x0000
        .equiv PLEN1,   0x0001
        .equiv PLEN2,   0x0002
        .equiv PLEN3,   0x0003
        .equiv PLEN4,   0x0004
        .equiv DWIDTH0, 0x0008
        .equiv DWIDTH1, 0x0009
        .equiv DWIDTH2, 0x000A
        .equiv DWIDTH3, 0x000B
        .equiv DWIDTH4, 0x000C


;----- CRCXORL Bits -----------------------------------------------------
        .equiv X1,      0x0001
        .equiv X2,      0x0002
        .equiv X3,      0x0003
        .equiv X4,      0x0004
        .equiv X5,      0x0005
        .equiv X6,      0x0006
        .equiv X7,      0x0007
        .equiv X8,      0x0008
        .equiv X9,      0x0009
        .equiv X10,     0x000A
        .equiv X11,     0x000B
        .equiv X12,     0x000C
        .equiv X13,     0x000D
        .equiv X14,     0x000E
        .equiv X15,     0x000F


;----- CRCXORH Bits -----------------------------------------------------
        .equiv X16,     0x0000
        .equiv X17,     0x0001
        .equiv X18,     0x0002
        .equiv X19,     0x0003
        .equiv X20,     0x0004
        .equiv X21,     0x0005
        .equiv X22,     0x0006
        .equiv X23,     0x0007
        .equiv X24,     0x0008
        .equiv X25,     0x0009
        .equiv X26,     0x000A
        .equiv X27,     0x000B
        .equiv X28,     0x000C
        .equiv X29,     0x000D
        .equiv X30,     0x000E
        .equiv X31,     0x000F


;----- REFOCON1 Bits -----------------------------------------------------
        .equiv ROACTIVE, 0x0008
        .equiv ROSWEN,  0x0009
        .equiv ROSLP,   0x000B
        .equiv ROOUT,   0x000C
        .equiv ROSIDL,  0x000D
        .equiv ROEN,    0x000F

        .equiv ROSEL0,  0x0000
        .equiv ROSEL1,  0x0001
        .equiv ROSEL2,  0x0002
        .equiv ROSEL3,  0x0003


;----- REFOCONL Bits -----------------------------------------------------
       ;.equiv ROACTIVE, 0x0008
       ;.equiv ROSWEN,  0x0009
       ;.equiv ROSLP,   0x000B
       ;.equiv ROOUT,   0x000C
       ;.equiv ROSIDL,  0x000D
       ;.equiv ROEN,    0x000F

       ;.equiv ROSEL0,  0x0000
       ;.equiv ROSEL1,  0x0001
       ;.equiv ROSEL2,  0x0002
       ;.equiv ROSEL3,  0x0003


;----- REFOCON2 Bits -----------------------------------------------------
        .equiv RODIV0,  0x0000
        .equiv RODIV1,  0x0001
        .equiv RODIV2,  0x0002
        .equiv RODIV3,  0x0003
        .equiv RODIV4,  0x0004
        .equiv RODIV5,  0x0005
        .equiv RODIV6,  0x0006
        .equiv RODIV7,  0x0007
        .equiv RODIV8,  0x0008
        .equiv RODIV9,  0x0009
        .equiv RODIV10, 0x000A
        .equiv RODIV11, 0x000B
        .equiv RODIV12, 0x000C
        .equiv RODIV13, 0x000D
        .equiv RODIV14, 0x000E


;----- REFOCONH Bits -----------------------------------------------------
       ;.equiv RODIV0,  0x0000
       ;.equiv RODIV1,  0x0001
       ;.equiv RODIV2,  0x0002
       ;.equiv RODIV3,  0x0003
       ;.equiv RODIV4,  0x0004
       ;.equiv RODIV5,  0x0005
       ;.equiv RODIV6,  0x0006
       ;.equiv RODIV7,  0x0007
       ;.equiv RODIV8,  0x0008
       ;.equiv RODIV9,  0x0009
       ;.equiv RODIV10, 0x000A
       ;.equiv RODIV11, 0x000B
       ;.equiv RODIV12, 0x000C
       ;.equiv RODIV13, 0x000D
       ;.equiv RODIV14, 0x000E


;----- PMD1 Bits -----------------------------------------------------
        .equiv AD1MD,   0x0000
        .equiv SPI1MD,  0x0003
        .equiv SPI2MD,  0x0004
        .equiv U1MD,    0x0005
        .equiv U2MD,    0x0006
        .equiv I2C1MD,  0x0007
        .equiv T1MD,    0x000B
        .equiv T2MD,    0x000C
        .equiv T3MD,    0x000D
        .equiv T4MD,    0x000E
        .equiv T5MD,    0x000F


;----- PMD2 Bits -----------------------------------------------------
        .equiv OC1MD,   0x0000
        .equiv OC2MD,   0x0001
        .equiv OC3MD,   0x0002
        .equiv OC4MD,   0x0003
        .equiv OC5MD,   0x0004
        .equiv OC6MD,   0x0005
        .equiv OC7MD,   0x0006
        .equiv OC8MD,   0x0007
        .equiv IC1MD,   0x0008
        .equiv IC2MD,   0x0009
        .equiv IC3MD,   0x000A
        .equiv IC4MD,   0x000B
        .equiv IC5MD,   0x000C
        .equiv IC6MD,   0x000D
        .equiv IC7MD,   0x000E
        .equiv IC8MD,   0x000F


;----- PMD3 Bits -----------------------------------------------------
        .equiv I2C2MD,  0x0001
        .equiv I2C3MD,  0x0002
        .equiv U3MD,    0x0003
        .equiv CRCMD,   0x0007
        .equiv PMPMD,   0x0008
        .equiv RTCCMD,  0x0009
        .equiv CMPMD,   0x000A


;----- PMD4 Bits -----------------------------------------------------
        .equiv USB1MD,  0x0000
        .equiv LVDMD,   0x0001
        .equiv CTMUMD,  0x0002
        .equiv REFOMD,  0x0003
        .equiv U4MD,    0x0005


;----- PMD5 Bits -----------------------------------------------------
        .equiv OC9MD,   0x0000
        .equiv IC9MD,   0x0008


;----- PMD6 Bits -----------------------------------------------------
        .equiv SPI3MD,  0x0000


;----- PMD7 Bits -----------------------------------------------------
        .equiv DMA0MD,  0x0004
        .equiv DMA1MD,  0x0005


;----- PMD8 Bits -----------------------------------------------------
        .equiv CLC1MD,  0x0002
        .equiv CLC2MD,  0x0003
        .equiv CLC3MD,  0x0004
        .equiv CLC4MD,  0x0005
        .equiv U5MD,    0x0006
        .equiv U6MD,    0x0007


;----- T1CON Bits -----------------------------------------------------
        .equiv TCS,     0x0001
        .equiv TSYNC,   0x0002
        .equiv TGATE,   0x0006
        .equiv TSIDL,   0x000D
        .equiv TON,     0x000F

        .equiv TCKPS0,  0x0004
        .equiv TCKPS1,  0x0005
        .equiv T1ECS0,  0x0008
        .equiv T1ECS1,  0x0009


        .equiv TECS0,   0x0008
        .equiv TECS1,   0x0009


        .equiv TIECS0,  0x0008
        .equiv TIECS1,  0x0009


;----- T2CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
        .equiv T32_T2CON, 0x0003
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005
       ;.equiv TECS0,   0x0008
       ;.equiv TECS1,   0x0009


       ;.equiv TIECS0,  0x0008
       ;.equiv TIECS1,  0x0009

        .equiv T45,     0x0003


;----- T3CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005
       ;.equiv TECS0,   0x0008
       ;.equiv TECS1,   0x0009


       ;.equiv TIECS0,  0x0008
       ;.equiv TIECS1,  0x0009


;----- T4CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
        .equiv T32_T4CON, 0x0003
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005
       ;.equiv TECS0,   0x0008
       ;.equiv TECS1,   0x0009


       ;.equiv TIECS0,  0x0008
       ;.equiv TIECS1,  0x0009

       ;.equiv T45,     0x0003


;----- T5CON Bits -----------------------------------------------------
       ;.equiv TCS,     0x0001
       ;.equiv TGATE,   0x0006
       ;.equiv TSIDL,   0x000D
       ;.equiv TON,     0x000F

       ;.equiv TCKPS0,  0x0004
       ;.equiv TCKPS1,  0x0005
       ;.equiv TECS0,   0x0008
       ;.equiv TECS1,   0x0009


       ;.equiv TIECS0,  0x0008
       ;.equiv TIECS1,  0x0009


;----- CTMUCON1L Bits -----------------------------------------------------
        .equiv CTTRIG,  0x0008
        .equiv IDISSEN, 0x0009
        .equiv EDGSEQEN, 0x000A
        .equiv EDGEN,   0x000B
        .equiv TGEN,    0x000C
        .equiv CTMUSIDL, 0x000D
        .equiv CTMUEN,  0x000F

        .equiv IRNG0,   0x0000
        .equiv IRNG1,   0x0001
        .equiv ITRIM0,  0x0002
        .equiv ITRIM1,  0x0003
        .equiv ITRIM2,  0x0004
        .equiv ITRIM3,  0x0005
        .equiv ITRIM4,  0x0006
        .equiv ITRIM5,  0x0007


;----- CTMUCON1H Bits -----------------------------------------------------
        .equiv IRNGH,   0x0000
        .equiv EDG2POL, 0x0006
        .equiv EDG2MOD, 0x0007
        .equiv EDG1STAT, 0x0008
        .equiv EDG2STAT, 0x0009
        .equiv EDG1POL, 0x000E
        .equiv EDG1MOD, 0x000F

        .equiv EDG2SEL0, 0x0002
        .equiv EDG2SEL1, 0x0003
        .equiv EDG2SEL2, 0x0004
        .equiv EDG2SEL3, 0x0005
        .equiv EDG1SEL0, 0x000A
        .equiv EDG1SEL1, 0x000B
        .equiv EDG1SEL2, 0x000C
        .equiv EDG1SEL3, 0x000D


;----- CTMUCON2L Bits -----------------------------------------------------
        .equiv IRSTEN,  0x0004

        .equiv DSCHS0,  0x0000
        .equiv DSCHS1,  0x0001
        .equiv DSCHS2,  0x0002


;----- RTCCON1L Bits -----------------------------------------------------
        .equiv TSAEN,   0x0000
        .equiv RTCOE,   0x0007
        .equiv PWCPOE,  0x0008
        .equiv PWCPOL,  0x0009
        .equiv PWCEN,   0x000A
        .equiv WRLOCK,  0x000B
        .equiv RTCEN,   0x000F


;----- RTCCON1H Bits -----------------------------------------------------
        .equiv CHIME,   0x000E
        .equiv ALRMEN,  0x000F


;----- RTCSTATL Bits -----------------------------------------------------
        .equiv HALFSEC, 0x0000
        .equiv ALMSYNC, 0x0001
        .equiv SYNC_RTCSTATL, 0x0002
        .equiv TSAEVT,  0x0003
        .equiv ALMEVT,  0x0005


;----- DATEH Bits -----------------------------------------------------
        .equiv MTHTEN,  0x0004


;----- ALMDATEH Bits -----------------------------------------------------
       ;.equiv MTHTEN,  0x0004


;----- TSADATEH Bits -----------------------------------------------------
       ;.equiv MTHTEN,  0x0004


;----- IC1CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC1CON1, 0x0003
        .equiv ICOV_IC1CON1, 0x0004
        .equiv ICSIDL,  0x000D

        .equiv ICM0,    0x0000
        .equiv ICM1,    0x0001
        .equiv ICM2,    0x0002
        .equiv ICI0,    0x0005
        .equiv ICI1,    0x0006
        .equiv ICTSEL0, 0x000A
        .equiv ICTSEL1, 0x000B
        .equiv ICTSEL2, 0x000C


;----- IC1CON2 Bits -----------------------------------------------------
        .equiv TRIGSTAT, 0x0006
        .equiv ICTRIG,  0x0007
        .equiv IC32,    0x0008

        .equiv SYNCSEL0, 0x0000
        .equiv SYNCSEL1, 0x0001
        .equiv SYNCSEL2, 0x0002
        .equiv SYNCSEL3, 0x0003
        .equiv SYNCSEL4, 0x0004


;----- IC2CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC2CON1, 0x0003
        .equiv ICOV_IC2CON1, 0x0004
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006
       ;.equiv ICTSEL0, 0x000A
       ;.equiv ICTSEL1, 0x000B
       ;.equiv ICTSEL2, 0x000C


;----- IC2CON2 Bits -----------------------------------------------------
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv ICTRIG,  0x0007
       ;.equiv IC32,    0x0008

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004


;----- IC3CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC3CON1, 0x0003
        .equiv ICOV_IC3CON1, 0x0004
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006
       ;.equiv ICTSEL0, 0x000A
       ;.equiv ICTSEL1, 0x000B
       ;.equiv ICTSEL2, 0x000C


;----- IC3CON2 Bits -----------------------------------------------------
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv ICTRIG,  0x0007
       ;.equiv IC32,    0x0008

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004


;----- IC4CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC4CON1, 0x0003
        .equiv ICOV_IC4CON1, 0x0004
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006
       ;.equiv ICTSEL0, 0x000A
       ;.equiv ICTSEL1, 0x000B
       ;.equiv ICTSEL2, 0x000C


;----- IC4CON2 Bits -----------------------------------------------------
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv ICTRIG,  0x0007
       ;.equiv IC32,    0x0008

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004


;----- IC5CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC5CON1, 0x0003
        .equiv ICOV_IC5CON1, 0x0004
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006
       ;.equiv ICTSEL0, 0x000A
       ;.equiv ICTSEL1, 0x000B
       ;.equiv ICTSEL2, 0x000C


;----- IC5CON2 Bits -----------------------------------------------------
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv ICTRIG,  0x0007
       ;.equiv IC32,    0x0008

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004


;----- IC6CON1 Bits -----------------------------------------------------
        .equiv ICBNE_IC6CON1, 0x0003
        .equiv ICOV_IC6CON1, 0x0004
       ;.equiv ICSIDL,  0x000D

       ;.equiv ICM0,    0x0000
       ;.equiv ICM1,    0x0001
       ;.equiv ICM2,    0x0002
       ;.equiv ICI0,    0x0005
       ;.equiv ICI1,    0x0006
       ;.equiv ICTSEL0, 0x000A
       ;.equiv ICTSEL1, 0x000B
       ;.equiv ICTSEL2, 0x000C


;----- IC6CON2 Bits -----------------------------------------------------
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv ICTRIG,  0x0007
       ;.equiv IC32,    0x0008

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004


;----- OC1CON1 Bits -----------------------------------------------------
        .equiv TRIGMODE, 0x0003
        .equiv OCFLT0,  0x0004
        .equiv OCFLT1,  0x0005
        .equiv OCFLT2,  0x0006
        .equiv ENFLT0,  0x0007
        .equiv ENFLT1,  0x0008
        .equiv ENFLT2,  0x0009
        .equiv OCSIDL,  0x000D

        .equiv OCM0,    0x0000
        .equiv OCM1,    0x0001
        .equiv OCM2,    0x0002
        .equiv OCTSEL0, 0x000A
        .equiv OCTSEL1, 0x000B
        .equiv OCTSEL2, 0x000C

        .equiv OCFLTA,  0x0004
        .equiv OCFLTB,  0x0005
        .equiv OCFLTC,  0x0006
        .equiv ENFLTA,  0x0007
        .equiv ENFLTB,  0x0008
        .equiv ENFLTC,  0x0009


;----- OC1CON2 Bits -----------------------------------------------------
        .equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
        .equiv OCTRIG,  0x0007
        .equiv OC32,    0x0008
        .equiv OCINV,   0x000C
        .equiv FLTTRIEN, 0x000D
        .equiv FLTOUT,  0x000E
        .equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
        .equiv DCB0,    0x0009
        .equiv DCB1,    0x000A
        .equiv FLTMODE, 0x000E


;----- OC2CON1 Bits -----------------------------------------------------
       ;.equiv TRIGMODE, 0x0003
       ;.equiv OCFLT0,  0x0004
       ;.equiv OCFLT1,  0x0005
       ;.equiv OCFLT2,  0x0006
       ;.equiv ENFLT0,  0x0007
       ;.equiv ENFLT1,  0x0008
       ;.equiv ENFLT2,  0x0009
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002
       ;.equiv OCTSEL0, 0x000A
       ;.equiv OCTSEL1, 0x000B
       ;.equiv OCTSEL2, 0x000C

       ;.equiv OCFLTA,  0x0004
       ;.equiv OCFLTB,  0x0005
       ;.equiv OCFLTC,  0x0006
       ;.equiv ENFLTA,  0x0007
       ;.equiv ENFLTB,  0x0008
       ;.equiv ENFLTC,  0x0009


;----- OC2CON2 Bits -----------------------------------------------------
       ;.equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv OCTRIG,  0x0007
       ;.equiv OC32,    0x0008
       ;.equiv OCINV,   0x000C
       ;.equiv FLTTRIEN, 0x000D
       ;.equiv FLTOUT,  0x000E
       ;.equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
       ;.equiv DCB0,    0x0009
       ;.equiv DCB1,    0x000A
       ;.equiv FLTMODE, 0x000E


;----- OC3CON1 Bits -----------------------------------------------------
       ;.equiv TRIGMODE, 0x0003
       ;.equiv OCFLT0,  0x0004
       ;.equiv OCFLT1,  0x0005
       ;.equiv OCFLT2,  0x0006
       ;.equiv ENFLT0,  0x0007
       ;.equiv ENFLT1,  0x0008
       ;.equiv ENFLT2,  0x0009
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002
       ;.equiv OCTSEL0, 0x000A
       ;.equiv OCTSEL1, 0x000B
       ;.equiv OCTSEL2, 0x000C

       ;.equiv OCFLTA,  0x0004
       ;.equiv OCFLTB,  0x0005
       ;.equiv OCFLTC,  0x0006
       ;.equiv ENFLTA,  0x0007
       ;.equiv ENFLTB,  0x0008
       ;.equiv ENFLTC,  0x0009


;----- OC3CON2 Bits -----------------------------------------------------
       ;.equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv OCTRIG,  0x0007
       ;.equiv OC32,    0x0008
       ;.equiv OCINV,   0x000C
       ;.equiv FLTTRIEN, 0x000D
       ;.equiv FLTOUT,  0x000E
       ;.equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
       ;.equiv DCB0,    0x0009
       ;.equiv DCB1,    0x000A
       ;.equiv FLTMODE, 0x000E


;----- OC4CON1 Bits -----------------------------------------------------
       ;.equiv TRIGMODE, 0x0003
       ;.equiv OCFLT0,  0x0004
       ;.equiv OCFLT1,  0x0005
       ;.equiv OCFLT2,  0x0006
       ;.equiv ENFLT0,  0x0007
       ;.equiv ENFLT1,  0x0008
       ;.equiv ENFLT2,  0x0009
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002
       ;.equiv OCTSEL0, 0x000A
       ;.equiv OCTSEL1, 0x000B
       ;.equiv OCTSEL2, 0x000C

       ;.equiv OCFLTA,  0x0004
       ;.equiv OCFLTB,  0x0005
       ;.equiv OCFLTC,  0x0006
       ;.equiv ENFLTA,  0x0007
       ;.equiv ENFLTB,  0x0008
       ;.equiv ENFLTC,  0x0009


;----- OC4CON2 Bits -----------------------------------------------------
       ;.equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv OCTRIG,  0x0007
       ;.equiv OC32,    0x0008
       ;.equiv OCINV,   0x000C
       ;.equiv FLTTRIEN, 0x000D
       ;.equiv FLTOUT,  0x000E
       ;.equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
       ;.equiv DCB0,    0x0009
       ;.equiv DCB1,    0x000A
       ;.equiv FLTMODE, 0x000E


;----- OC5CON1 Bits -----------------------------------------------------
       ;.equiv TRIGMODE, 0x0003
       ;.equiv OCFLT0,  0x0004
       ;.equiv OCFLT1,  0x0005
       ;.equiv OCFLT2,  0x0006
       ;.equiv ENFLT0,  0x0007
       ;.equiv ENFLT1,  0x0008
       ;.equiv ENFLT2,  0x0009
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002
       ;.equiv OCTSEL0, 0x000A
       ;.equiv OCTSEL1, 0x000B
       ;.equiv OCTSEL2, 0x000C

       ;.equiv OCFLTA,  0x0004
       ;.equiv OCFLTB,  0x0005
       ;.equiv OCFLTC,  0x0006
       ;.equiv ENFLTA,  0x0007
       ;.equiv ENFLTB,  0x0008
       ;.equiv ENFLTC,  0x0009


;----- OC5CON2 Bits -----------------------------------------------------
       ;.equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv OCTRIG,  0x0007
       ;.equiv OC32,    0x0008
       ;.equiv OCINV,   0x000C
       ;.equiv FLTTRIEN, 0x000D
       ;.equiv FLTOUT,  0x000E
       ;.equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
       ;.equiv DCB0,    0x0009
       ;.equiv DCB1,    0x000A
       ;.equiv FLTMODE, 0x000E


;----- OC6CON1 Bits -----------------------------------------------------
       ;.equiv TRIGMODE, 0x0003
       ;.equiv OCFLT0,  0x0004
       ;.equiv OCFLT1,  0x0005
       ;.equiv OCFLT2,  0x0006
       ;.equiv ENFLT0,  0x0007
       ;.equiv ENFLT1,  0x0008
       ;.equiv ENFLT2,  0x0009
       ;.equiv OCSIDL,  0x000D

       ;.equiv OCM0,    0x0000
       ;.equiv OCM1,    0x0001
       ;.equiv OCM2,    0x0002
       ;.equiv OCTSEL0, 0x000A
       ;.equiv OCTSEL1, 0x000B
       ;.equiv OCTSEL2, 0x000C

       ;.equiv OCFLTA,  0x0004
       ;.equiv OCFLTB,  0x0005
       ;.equiv OCFLTC,  0x0006
       ;.equiv ENFLTA,  0x0007
       ;.equiv ENFLTB,  0x0008
       ;.equiv ENFLTC,  0x0009


;----- OC6CON2 Bits -----------------------------------------------------
       ;.equiv OCTRIS,  0x0005
       ;.equiv TRIGSTAT, 0x0006
       ;.equiv OCTRIG,  0x0007
       ;.equiv OC32,    0x0008
       ;.equiv OCINV,   0x000C
       ;.equiv FLTTRIEN, 0x000D
       ;.equiv FLTOUT,  0x000E
       ;.equiv FLTMD,   0x000F

       ;.equiv SYNCSEL0, 0x0000
       ;.equiv SYNCSEL1, 0x0001
       ;.equiv SYNCSEL2, 0x0002
       ;.equiv SYNCSEL3, 0x0003
       ;.equiv SYNCSEL4, 0x0004
       ;.equiv DCB0,    0x0009
       ;.equiv DCB1,    0x000A
       ;.equiv FLTMODE, 0x000E


;----- CCP1CON1L Bits -----------------------------------------------------
        .equiv CCSEL,   0x0004
        .equiv T32_CCP1CON1L, 0x0005
        .equiv TMRSYNC, 0x000B
        .equiv CCPSLP,  0x000C
        .equiv CCPSIDL, 0x000D
        .equiv CCPON,   0x000F

        .equiv MOD0,    0x0000
        .equiv MOD1,    0x0001
        .equiv MOD2,    0x0002
        .equiv MOD3,    0x0003
        .equiv TMR32,   0x0005
        .equiv TMRPS0,  0x0006
        .equiv TMRPS1,  0x0007
        .equiv CLKSEL0, 0x0008
        .equiv CLKSEL1, 0x0009
        .equiv CLKSEL2, 0x000A


        .equiv CCPMOD0, 0x0000
        .equiv CCPMOD1, 0x0001
        .equiv CCPMOD2, 0x0002
        .equiv CCPMOD3, 0x0003


;----- CCP1CON1H Bits -----------------------------------------------------
        .equiv ALTSYNC, 0x0005
        .equiv ONESHOT, 0x0006
        .equiv TRIGEN,  0x0007
        .equiv RTRGEN,  0x000E
        .equiv OPSSRC,  0x000F

        .equiv SYNC0,   0x0000
        .equiv SYNC1,   0x0001
        .equiv SYNC2,   0x0002
        .equiv SYNC3,   0x0003
        .equiv SYNC4,   0x0004
        .equiv OPS0,    0x0008
        .equiv OPS1,    0x0009
        .equiv OPS2,    0x000A
        .equiv OPS3,    0x000B
        .equiv OPSRC,   0x000F


        .equiv IOPS0,   0x0008
        .equiv IOPS1,   0x0009
        .equiv IOPS2,   0x000A
        .equiv IOPS3,   0x000B


;----- CCP1CON2L Bits -----------------------------------------------------
        .equiv SSDG,    0x000C
        .equiv ASDGM,   0x000E
        .equiv PWMRSEN, 0x000F

        .equiv ASDG0,   0x0000
        .equiv ASDG1,   0x0001
        .equiv ASDG2,   0x0002
        .equiv ASDG3,   0x0003
        .equiv ASDG4,   0x0004
        .equiv ASDG5,   0x0005
        .equiv ASDG6,   0x0006
        .equiv ASDG7,   0x0007


;----- CCP1CON2H Bits -----------------------------------------------------
        .equiv OCAEN,   0x0008
        .equiv OCBEN,   0x0009
        .equiv OENSYNC, 0x000F

        .equiv ICS0,    0x0000
        .equiv ICS1,    0x0001
        .equiv ICS2,    0x0002
        .equiv AUXOUT0, 0x0003
        .equiv AUXOUT1, 0x0004
        .equiv ICGSM0,  0x0006
        .equiv ICGSM1,  0x0007


        .equiv ICSEL0,  0x0000
        .equiv ICSEL1,  0x0001
        .equiv ICSEL2,  0x0002


;----- CCP1CON3L Bits -----------------------------------------------------
        .equiv DT0,     0x0000
        .equiv DT1,     0x0001
        .equiv DT2,     0x0002
        .equiv DT3,     0x0003
        .equiv DT4,     0x0004
        .equiv DT5,     0x0005


;----- CCP1CON3H Bits -----------------------------------------------------
        .equiv POLBDF,  0x0004
        .equiv POLACE,  0x0005
        .equiv OETRIG,  0x000F

        .equiv PSSBDF0, 0x0000
        .equiv PSSBDF1, 0x0001
        .equiv PSSACE0, 0x0002
        .equiv PSSACE1, 0x0003
        .equiv OUTM0,   0x0008
        .equiv OUTM1,   0x0009
        .equiv OUTM2,   0x000A
        .equiv OSCNT0,  0x000C
        .equiv OSCNT1,  0x000D
        .equiv OSCNT2,  0x000E


;----- CCP1STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP1STAT, 0x0000
        .equiv ICOV_CCP1STAT, 0x0001
        .equiv ICDIS,   0x0002
        .equiv SCEVT,   0x0003
        .equiv ASEVT,   0x0004
        .equiv TRCLR,   0x0005
        .equiv TRSET,   0x0006
        .equiv CCPTRIG, 0x0007
        .equiv ICGARM,  0x000A

        .equiv TRIG,    0x0007


;----- CCP1STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP1STATL, 0x0000
        .equiv ICOV_CCP1STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP1STATH Bits -----------------------------------------------------
        .equiv RAWIP,   0x0000
        .equiv RBWIP,   0x0001
        .equiv TMRLWIP, 0x0002
        .equiv TMRHWIP, 0x0003
        .equiv PRLWIP,  0x0004


;----- CCP2CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP2CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP2CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP2CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP2CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OCBEN,   0x0009
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP2CON3L Bits -----------------------------------------------------
       ;.equiv DT0,     0x0000
       ;.equiv DT1,     0x0001
       ;.equiv DT2,     0x0002
       ;.equiv DT3,     0x0003
       ;.equiv DT4,     0x0004
       ;.equiv DT5,     0x0005


;----- CCP2CON3H Bits -----------------------------------------------------
       ;.equiv POLBDF,  0x0004
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSBDF0, 0x0000
       ;.equiv PSSBDF1, 0x0001
       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OUTM0,   0x0008
       ;.equiv OUTM1,   0x0009
       ;.equiv OUTM2,   0x000A
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP2STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP2STAT, 0x0000
        .equiv ICOV_CCP2STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP2STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP2STATL, 0x0000
        .equiv ICOV_CCP2STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP2STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- CCP3CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP3CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP3CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP3CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP3CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OCBEN,   0x0009
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP3CON3L Bits -----------------------------------------------------
       ;.equiv DT0,     0x0000
       ;.equiv DT1,     0x0001
       ;.equiv DT2,     0x0002
       ;.equiv DT3,     0x0003
       ;.equiv DT4,     0x0004
       ;.equiv DT5,     0x0005


;----- CCP3CON3H Bits -----------------------------------------------------
       ;.equiv POLBDF,  0x0004
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSBDF0, 0x0000
       ;.equiv PSSBDF1, 0x0001
       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OUTM0,   0x0008
       ;.equiv OUTM1,   0x0009
       ;.equiv OUTM2,   0x000A
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP3STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP3STAT, 0x0000
        .equiv ICOV_CCP3STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP3STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP3STATL, 0x0000
        .equiv ICOV_CCP3STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP3STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- CMSTAT Bits -----------------------------------------------------
        .equiv C1OUT,   0x0000
        .equiv C2OUT,   0x0001
        .equiv C3OUT,   0x0002
        .equiv C1EVT,   0x0008
        .equiv C2EVT,   0x0009
        .equiv C3EVT,   0x000A
        .equiv CMIDL,   0x000F

        .equiv CMSIDL,  0x000F


;----- CVRCON Bits -----------------------------------------------------
        .equiv CVRSS,   0x0005
        .equiv CVROE,   0x0006
        .equiv CVREN,   0x0007
        .equiv CVREFP,  0x000A

        .equiv CVR0,    0x0000
        .equiv CVR1,    0x0001
        .equiv CVR2,    0x0002
        .equiv CVR3,    0x0003
        .equiv CVR4,    0x0004
        .equiv CVREFM0, 0x0008
        .equiv CVREFM1, 0x0009


;----- CM1CON Bits -----------------------------------------------------
        .equiv CREF,    0x0004
        .equiv COUT,    0x0008
        .equiv CEVT,    0x0009
        .equiv CPOL,    0x000D
        .equiv COE,     0x000E
        .equiv CON,     0x000F

        .equiv CCH0,    0x0000
        .equiv CCH1,    0x0001
        .equiv EVPOL0,  0x0006
        .equiv EVPOL1,  0x0007
        .equiv CEN,     0x000F


;----- CM2CON Bits -----------------------------------------------------
       ;.equiv CREF,    0x0004
       ;.equiv COUT,    0x0008
       ;.equiv CEVT,    0x0009
       ;.equiv CPOL,    0x000D
       ;.equiv COE,     0x000E
       ;.equiv CON,     0x000F

       ;.equiv CCH0,    0x0000
       ;.equiv CCH1,    0x0001
       ;.equiv EVPOL0,  0x0006
       ;.equiv EVPOL1,  0x0007
       ;.equiv CEN,     0x000F


;----- CM3CON Bits -----------------------------------------------------
       ;.equiv CREF,    0x0004
       ;.equiv COUT,    0x0008
       ;.equiv CEVT,    0x0009
       ;.equiv CPOL,    0x000D
       ;.equiv COE,     0x000E
       ;.equiv CON,     0x000F

       ;.equiv CCH0,    0x0000
       ;.equiv CCH1,    0x0001
       ;.equiv EVPOL0,  0x0006
       ;.equiv EVPOL1,  0x0007
       ;.equiv CEN,     0x000F


;----- ANCFG Bits -----------------------------------------------------
        .equiv VBGEN,   0x0000
        .equiv VBGCMP,  0x0001
        .equiv VBGADC,  0x0002
        .equiv VBGUSB,  0x0003


;----- CCP4CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP4CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP4CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP4CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP4CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP4CON3H Bits -----------------------------------------------------
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP4STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP4STAT, 0x0000
        .equiv ICOV_CCP4STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP4STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP4STATL, 0x0000
        .equiv ICOV_CCP4STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP4STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- CCP5CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP5CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP5CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP5CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP5CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP5CON3H Bits -----------------------------------------------------
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP5STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP5STAT, 0x0000
        .equiv ICOV_CCP5STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP5STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP5STATL, 0x0000
        .equiv ICOV_CCP5STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP5STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- CCP6CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP6CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP6CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP6CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP6CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP6CON3H Bits -----------------------------------------------------
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP6STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP6STAT, 0x0000
        .equiv ICOV_CCP6STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP6STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP6STATL, 0x0000
        .equiv ICOV_CCP6STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP6STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- CCP7CON1L Bits -----------------------------------------------------
       ;.equiv CCSEL,   0x0004
        .equiv T32_CCP7CON1L, 0x0005
       ;.equiv TMRSYNC, 0x000B
       ;.equiv CCPSLP,  0x000C
       ;.equiv CCPSIDL, 0x000D
       ;.equiv CCPON,   0x000F

       ;.equiv MOD0,    0x0000
       ;.equiv MOD1,    0x0001
       ;.equiv MOD2,    0x0002
       ;.equiv MOD3,    0x0003
       ;.equiv TMR32,   0x0005
       ;.equiv TMRPS0,  0x0006
       ;.equiv TMRPS1,  0x0007
       ;.equiv CLKSEL0, 0x0008
       ;.equiv CLKSEL1, 0x0009
       ;.equiv CLKSEL2, 0x000A


       ;.equiv CCPMOD0, 0x0000
       ;.equiv CCPMOD1, 0x0001
       ;.equiv CCPMOD2, 0x0002
       ;.equiv CCPMOD3, 0x0003


;----- CCP7CON1H Bits -----------------------------------------------------
       ;.equiv ALTSYNC, 0x0005
       ;.equiv ONESHOT, 0x0006
       ;.equiv TRIGEN,  0x0007
       ;.equiv RTRGEN,  0x000E
       ;.equiv OPSSRC,  0x000F

       ;.equiv SYNC0,   0x0000
       ;.equiv SYNC1,   0x0001
       ;.equiv SYNC2,   0x0002
       ;.equiv SYNC3,   0x0003
       ;.equiv SYNC4,   0x0004
       ;.equiv OPS0,    0x0008
       ;.equiv OPS1,    0x0009
       ;.equiv OPS2,    0x000A
       ;.equiv OPS3,    0x000B
       ;.equiv OPSRC,   0x000F


       ;.equiv IOPS0,   0x0008
       ;.equiv IOPS1,   0x0009
       ;.equiv IOPS2,   0x000A
       ;.equiv IOPS3,   0x000B


;----- CCP7CON2L Bits -----------------------------------------------------
       ;.equiv SSDG,    0x000C
       ;.equiv ASDGM,   0x000E
       ;.equiv PWMRSEN, 0x000F

       ;.equiv ASDG0,   0x0000
       ;.equiv ASDG1,   0x0001
       ;.equiv ASDG2,   0x0002
       ;.equiv ASDG3,   0x0003
       ;.equiv ASDG4,   0x0004
       ;.equiv ASDG5,   0x0005
       ;.equiv ASDG6,   0x0006
       ;.equiv ASDG7,   0x0007


;----- CCP7CON2H Bits -----------------------------------------------------
       ;.equiv OCAEN,   0x0008
       ;.equiv OENSYNC, 0x000F

       ;.equiv ICS0,    0x0000
       ;.equiv ICS1,    0x0001
       ;.equiv ICS2,    0x0002
       ;.equiv AUXOUT0, 0x0003
       ;.equiv AUXOUT1, 0x0004
       ;.equiv ICGSM0,  0x0006
       ;.equiv ICGSM1,  0x0007


       ;.equiv ICSEL0,  0x0000
       ;.equiv ICSEL1,  0x0001
       ;.equiv ICSEL2,  0x0002


;----- CCP7CON3H Bits -----------------------------------------------------
       ;.equiv POLACE,  0x0005
       ;.equiv OETRIG,  0x000F

       ;.equiv PSSACE0, 0x0002
       ;.equiv PSSACE1, 0x0003
       ;.equiv OSCNT0,  0x000C
       ;.equiv OSCNT1,  0x000D
       ;.equiv OSCNT2,  0x000E


;----- CCP7STAT Bits -----------------------------------------------------
        .equiv ICBNE_CCP7STAT, 0x0000
        .equiv ICOV_CCP7STAT, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP7STATL Bits -----------------------------------------------------
        .equiv ICBNE_CCP7STATL, 0x0000
        .equiv ICOV_CCP7STATL, 0x0001
       ;.equiv ICDIS,   0x0002
       ;.equiv SCEVT,   0x0003
       ;.equiv ASEVT,   0x0004
       ;.equiv TRCLR,   0x0005
       ;.equiv TRSET,   0x0006
       ;.equiv CCPTRIG, 0x0007
       ;.equiv ICGARM,  0x000A

       ;.equiv TRIG,    0x0007


;----- CCP7STATH Bits -----------------------------------------------------
       ;.equiv RAWIP,   0x0000
       ;.equiv RBWIP,   0x0001
       ;.equiv TMRLWIP, 0x0002
       ;.equiv TMRHWIP, 0x0003
       ;.equiv PRLWIP,  0x0004


;----- U1MODE Bits -----------------------------------------------------
        .equiv STSEL,   0x0000
        .equiv BRGH,    0x0003
        .equiv URXINV,  0x0004
        .equiv ABAUD,   0x0005
        .equiv LPBACK,  0x0006
        .equiv WAKE,    0x0007
        .equiv RTSMD,   0x000B
        .equiv IREN,    0x000C
        .equiv USIDL,   0x000D
        .equiv UARTEN,  0x000F

        .equiv PDSEL0,  0x0001
        .equiv PDSEL1,  0x0002
        .equiv RXINV,   0x0004
        .equiv UEN0,    0x0008
        .equiv UEN1,    0x0009


;----- U1STA Bits -----------------------------------------------------
        .equiv URXDA,   0x0000
        .equiv OERR,    0x0001
        .equiv FERR,    0x0002
        .equiv PERR,    0x0003
        .equiv RIDLE,   0x0004
        .equiv ADDEN,   0x0005
        .equiv TRMT,    0x0008
        .equiv UTXBF,   0x0009
        .equiv UTXEN,   0x000A
        .equiv UTXBRK,  0x000B
        .equiv URXEN,   0x000C
        .equiv UTXISEL0, 0x000D
        .equiv UTXINV,  0x000E
        .equiv UTXISEL1, 0x000F

        .equiv URXISEL0, 0x0006
        .equiv URXISEL1, 0x0007


;----- U1TXREG Bits -----------------------------------------------------
        .equiv LAST,    0x000F


;----- U1ADMD Bits -----------------------------------------------------
        .equiv ADMADDR0, 0x0000
        .equiv ADMADDR1, 0x0001
        .equiv ADMADDR2, 0x0002
        .equiv ADMADDR3, 0x0003
        .equiv ADMADDR4, 0x0004
        .equiv ADMADDR5, 0x0005
        .equiv ADMADDR6, 0x0006
        .equiv ADMADDR7, 0x0007
        .equiv ADMMASK0, 0x0008
        .equiv ADMMASK1, 0x0009
        .equiv ADMMASK2, 0x000A
        .equiv ADMMASK3, 0x000B
        .equiv ADMMASK4, 0x000C
        .equiv ADMMASK5, 0x000D
        .equiv ADMMASK6, 0x000E
        .equiv ADMMASK7, 0x000F


;----- U2MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv BRGH,    0x0003
       ;.equiv URXINV,  0x0004
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv RTSMD,   0x000B
       ;.equiv IREN,    0x000C
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002
       ;.equiv RXINV,   0x0004
       ;.equiv UEN0,    0x0008
       ;.equiv UEN1,    0x0009


;----- U2STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv URXEN,   0x000C
       ;.equiv UTXISEL0, 0x000D
       ;.equiv UTXINV,  0x000E
       ;.equiv UTXISEL1, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U2TXREG Bits -----------------------------------------------------
       ;.equiv LAST,    0x000F


;----- U2ADMD Bits -----------------------------------------------------
       ;.equiv ADMADDR0, 0x0000
       ;.equiv ADMADDR1, 0x0001
       ;.equiv ADMADDR2, 0x0002
       ;.equiv ADMADDR3, 0x0003
       ;.equiv ADMADDR4, 0x0004
       ;.equiv ADMADDR5, 0x0005
       ;.equiv ADMADDR6, 0x0006
       ;.equiv ADMADDR7, 0x0007
       ;.equiv ADMMASK0, 0x0008
       ;.equiv ADMMASK1, 0x0009
       ;.equiv ADMMASK2, 0x000A
       ;.equiv ADMMASK3, 0x000B
       ;.equiv ADMMASK4, 0x000C
       ;.equiv ADMMASK5, 0x000D
       ;.equiv ADMMASK6, 0x000E
       ;.equiv ADMMASK7, 0x000F


;----- U3MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv BRGH,    0x0003
       ;.equiv URXINV,  0x0004
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv RTSMD,   0x000B
       ;.equiv IREN,    0x000C
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002
       ;.equiv RXINV,   0x0004
       ;.equiv UEN0,    0x0008
       ;.equiv UEN1,    0x0009


;----- U3STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv URXEN,   0x000C
       ;.equiv UTXISEL0, 0x000D
       ;.equiv UTXINV,  0x000E
       ;.equiv UTXISEL1, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U3TXREG Bits -----------------------------------------------------
       ;.equiv LAST,    0x000F


;----- U3ADMD Bits -----------------------------------------------------
       ;.equiv ADMADDR0, 0x0000
       ;.equiv ADMADDR1, 0x0001
       ;.equiv ADMADDR2, 0x0002
       ;.equiv ADMADDR3, 0x0003
       ;.equiv ADMADDR4, 0x0004
       ;.equiv ADMADDR5, 0x0005
       ;.equiv ADMADDR6, 0x0006
       ;.equiv ADMADDR7, 0x0007
       ;.equiv ADMMASK0, 0x0008
       ;.equiv ADMMASK1, 0x0009
       ;.equiv ADMMASK2, 0x000A
       ;.equiv ADMMASK3, 0x000B
       ;.equiv ADMMASK4, 0x000C
       ;.equiv ADMMASK5, 0x000D
       ;.equiv ADMMASK6, 0x000E
       ;.equiv ADMMASK7, 0x000F


;----- U4MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv BRGH,    0x0003
       ;.equiv URXINV,  0x0004
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv RTSMD,   0x000B
       ;.equiv IREN,    0x000C
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002
       ;.equiv RXINV,   0x0004
       ;.equiv UEN0,    0x0008
       ;.equiv UEN1,    0x0009


;----- U4STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv URXEN,   0x000C
       ;.equiv UTXISEL0, 0x000D
       ;.equiv UTXINV,  0x000E
       ;.equiv UTXISEL1, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U4TXREG Bits -----------------------------------------------------
       ;.equiv LAST,    0x000F


;----- U4ADMD Bits -----------------------------------------------------
       ;.equiv ADMADDR0, 0x0000
       ;.equiv ADMADDR1, 0x0001
       ;.equiv ADMADDR2, 0x0002
       ;.equiv ADMADDR3, 0x0003
       ;.equiv ADMADDR4, 0x0004
       ;.equiv ADMADDR5, 0x0005
       ;.equiv ADMADDR6, 0x0006
       ;.equiv ADMADDR7, 0x0007
       ;.equiv ADMMASK0, 0x0008
       ;.equiv ADMMASK1, 0x0009
       ;.equiv ADMMASK2, 0x000A
       ;.equiv ADMMASK3, 0x000B
       ;.equiv ADMMASK4, 0x000C
       ;.equiv ADMMASK5, 0x000D
       ;.equiv ADMMASK6, 0x000E
       ;.equiv ADMMASK7, 0x000F


;----- U5MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv BRGH,    0x0003
       ;.equiv URXINV,  0x0004
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv RTSMD,   0x000B
       ;.equiv IREN,    0x000C
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002
       ;.equiv RXINV,   0x0004
       ;.equiv UEN0,    0x0008
       ;.equiv UEN1,    0x0009


;----- U5STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv URXEN,   0x000C
       ;.equiv UTXISEL0, 0x000D
       ;.equiv UTXINV,  0x000E
       ;.equiv UTXISEL1, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U5TXREG Bits -----------------------------------------------------
       ;.equiv LAST,    0x000F


;----- U5ADMD Bits -----------------------------------------------------
       ;.equiv ADMADDR0, 0x0000
       ;.equiv ADMADDR1, 0x0001
       ;.equiv ADMADDR2, 0x0002
       ;.equiv ADMADDR3, 0x0003
       ;.equiv ADMADDR4, 0x0004
       ;.equiv ADMADDR5, 0x0005
       ;.equiv ADMADDR6, 0x0006
       ;.equiv ADMADDR7, 0x0007
       ;.equiv ADMMASK0, 0x0008
       ;.equiv ADMMASK1, 0x0009
       ;.equiv ADMMASK2, 0x000A
       ;.equiv ADMMASK3, 0x000B
       ;.equiv ADMMASK4, 0x000C
       ;.equiv ADMMASK5, 0x000D
       ;.equiv ADMMASK6, 0x000E
       ;.equiv ADMMASK7, 0x000F


;----- U6MODE Bits -----------------------------------------------------
       ;.equiv STSEL,   0x0000
       ;.equiv BRGH,    0x0003
       ;.equiv URXINV,  0x0004
       ;.equiv ABAUD,   0x0005
       ;.equiv LPBACK,  0x0006
       ;.equiv WAKE,    0x0007
       ;.equiv RTSMD,   0x000B
       ;.equiv IREN,    0x000C
       ;.equiv USIDL,   0x000D
       ;.equiv UARTEN,  0x000F

       ;.equiv PDSEL0,  0x0001
       ;.equiv PDSEL1,  0x0002
       ;.equiv RXINV,   0x0004
       ;.equiv UEN0,    0x0008
       ;.equiv UEN1,    0x0009


;----- U6STA Bits -----------------------------------------------------
       ;.equiv URXDA,   0x0000
       ;.equiv OERR,    0x0001
       ;.equiv FERR,    0x0002
       ;.equiv PERR,    0x0003
       ;.equiv RIDLE,   0x0004
       ;.equiv ADDEN,   0x0005
       ;.equiv TRMT,    0x0008
       ;.equiv UTXBF,   0x0009
       ;.equiv UTXEN,   0x000A
       ;.equiv UTXBRK,  0x000B
       ;.equiv URXEN,   0x000C
       ;.equiv UTXISEL0, 0x000D
       ;.equiv UTXINV,  0x000E
       ;.equiv UTXISEL1, 0x000F

       ;.equiv URXISEL0, 0x0006
       ;.equiv URXISEL1, 0x0007


;----- U6TXREG Bits -----------------------------------------------------
       ;.equiv LAST,    0x000F


;----- U6ADMD Bits -----------------------------------------------------
       ;.equiv ADMADDR0, 0x0000
       ;.equiv ADMADDR1, 0x0001
       ;.equiv ADMADDR2, 0x0002
       ;.equiv ADMADDR3, 0x0003
       ;.equiv ADMADDR4, 0x0004
       ;.equiv ADMADDR5, 0x0005
       ;.equiv ADMADDR6, 0x0006
       ;.equiv ADMADDR7, 0x0007
       ;.equiv ADMMASK0, 0x0008
       ;.equiv ADMMASK1, 0x0009
       ;.equiv ADMMASK2, 0x000A
       ;.equiv ADMMASK3, 0x000B
       ;.equiv ADMMASK4, 0x000C
       ;.equiv ADMMASK5, 0x000D
       ;.equiv ADMMASK6, 0x000E
       ;.equiv ADMMASK7, 0x000F


;----- SPI1CON1 Bits -----------------------------------------------------
        .equiv ENHBUF,  0x0000
        .equiv SPIFE,   0x0001
        .equiv MCLKEN,  0x0002
        .equiv DISSCK,  0x0003
        .equiv DISSDI,  0x0004
        .equiv MSTEN,   0x0005
        .equiv CKP,     0x0006
        .equiv SSEN,    0x0007
        .equiv CKE,     0x0008
        .equiv SMP,     0x0009
        .equiv MODE16,  0x000A
        .equiv MODE32,  0x000B
        .equiv DISSDO,  0x000C
        .equiv SPISIDL, 0x000D
        .equiv SPIEN,   0x000F



;----- SPI1CON1L Bits -----------------------------------------------------
       ;.equiv ENHBUF,  0x0000
       ;.equiv SPIFE,   0x0001
       ;.equiv MCLKEN,  0x0002
       ;.equiv DISSCK,  0x0003
       ;.equiv DISSDI,  0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv MODE32,  0x000B
       ;.equiv DISSDO,  0x000C
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F



;----- SPI1CON1H Bits -----------------------------------------------------
        .equiv FRMSYPW, 0x0003
        .equiv MSSEN,   0x0004
        .equiv FRMPOL,  0x0005
        .equiv FRMSYNC, 0x0006
        .equiv FRMEN,   0x0007
        .equiv URDTEN,  0x000A
        .equiv AUDMONO, 0x000B
        .equiv IGNTUR,  0x000C
        .equiv IGNROV,  0x000D
        .equiv SPISGNEXT, 0x000E
        .equiv AUDEN,   0x000F

        .equiv FRMCNT0, 0x0000
        .equiv FRMCNT1, 0x0001
        .equiv FRMCNT2, 0x0002
        .equiv AUDMOD0, 0x0008
        .equiv AUDMOD1, 0x0009


;----- SPI1CON2 Bits -----------------------------------------------------
       ;.equiv FRMSYPW, 0x0003
       ;.equiv MSSEN,   0x0004
       ;.equiv FRMPOL,  0x0005
       ;.equiv FRMSYNC, 0x0006
       ;.equiv FRMEN,   0x0007
       ;.equiv URDTEN,  0x000A
       ;.equiv AUDMONO, 0x000B
       ;.equiv IGNTUR,  0x000C
       ;.equiv IGNROV,  0x000D
       ;.equiv SPISGNEXT, 0x000E
       ;.equiv AUDEN,   0x000F

       ;.equiv FRMCNT0, 0x0000
       ;.equiv FRMCNT1, 0x0001
       ;.equiv FRMCNT2, 0x0002
       ;.equiv AUDMOD0, 0x0008
       ;.equiv AUDMOD1, 0x0009


;----- SPI1STATL Bits -----------------------------------------------------
        .equiv SPIRBF,  0x0000
        .equiv SPITBF,  0x0001
        .equiv SPITBE,  0x0003
        .equiv SPIRBE,  0x0005
        .equiv SPIROV,  0x0006
        .equiv SRMT,    0x0007
        .equiv SPITUR,  0x0008
        .equiv SPIBUSY, 0x000B
        .equiv FRMERR,  0x000C


;----- SPI1STATH Bits -----------------------------------------------------
        .equiv TXELM0,  0x0000
        .equiv TXELM1,  0x0001
        .equiv TXELM2,  0x0002
        .equiv TXELM3,  0x0003
        .equiv TXELM4,  0x0004
        .equiv TXELM5,  0x0005
        .equiv RXELM0,  0x0008
        .equiv RXELM1,  0x0009
        .equiv RXELM2,  0x000A
        .equiv RXELM3,  0x000B
        .equiv RXELM4,  0x000C
        .equiv RXELM5,  0x000D


;----- SPI1IMSK1 Bits -----------------------------------------------------
        .equiv SPIRBFEN, 0x0000
        .equiv SPITBFEN, 0x0001
        .equiv SPITBEN, 0x0003
        .equiv SPIRBEN, 0x0005
        .equiv SPIROVEN, 0x0006
        .equiv SRMTEN,  0x0007
        .equiv SPITUREN, 0x0008
        .equiv BUSYEN,  0x000B
        .equiv FRMERREN, 0x000C


;----- SPI1IMSKL Bits -----------------------------------------------------
       ;.equiv SPIRBFEN, 0x0000
       ;.equiv SPITBFEN, 0x0001
       ;.equiv SPITBEN, 0x0003
       ;.equiv SPIRBEN, 0x0005
       ;.equiv SPIROVEN, 0x0006
       ;.equiv SRMTEN,  0x0007
       ;.equiv SPITUREN, 0x0008
       ;.equiv BUSYEN,  0x000B
       ;.equiv FRMERREN, 0x000C


;----- SPI1IMSK2 Bits -----------------------------------------------------
        .equiv TXWIEN,  0x0007
        .equiv RXWIEN,  0x000F

        .equiv TXMSK0,  0x0000
        .equiv TXMSK1,  0x0001
        .equiv TXMSK2,  0x0002
        .equiv TXMSK3,  0x0003
        .equiv TXMSK4,  0x0004
        .equiv TXMSK5,  0x0005
        .equiv RXMSK0,  0x0008
        .equiv RXMSK1,  0x0009
        .equiv RXMSK2,  0x000A
        .equiv RXMSK3,  0x000B
        .equiv RXMSK4,  0x000C
        .equiv RXMSK5,  0x000D


;----- SPI1IMSKH Bits -----------------------------------------------------
       ;.equiv TXWIEN,  0x0007
       ;.equiv RXWIEN,  0x000F

       ;.equiv TXMSK0,  0x0000
       ;.equiv TXMSK1,  0x0001
       ;.equiv TXMSK2,  0x0002
       ;.equiv TXMSK3,  0x0003
       ;.equiv TXMSK4,  0x0004
       ;.equiv TXMSK5,  0x0005
       ;.equiv RXMSK0,  0x0008
       ;.equiv RXMSK1,  0x0009
       ;.equiv RXMSK2,  0x000A
       ;.equiv RXMSK3,  0x000B
       ;.equiv RXMSK4,  0x000C
       ;.equiv RXMSK5,  0x000D


;----- SPI2CON1 Bits -----------------------------------------------------
       ;.equiv ENHBUF,  0x0000
       ;.equiv SPIFE,   0x0001
       ;.equiv MCLKEN,  0x0002
       ;.equiv DISSCK,  0x0003
       ;.equiv DISSDI,  0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv MODE32,  0x000B
       ;.equiv DISSDO,  0x000C
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F



;----- SPI2CON1L Bits -----------------------------------------------------
       ;.equiv ENHBUF,  0x0000
       ;.equiv SPIFE,   0x0001
       ;.equiv MCLKEN,  0x0002
       ;.equiv DISSCK,  0x0003
       ;.equiv DISSDI,  0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv MODE32,  0x000B
       ;.equiv DISSDO,  0x000C
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F



;----- SPI2CON1H Bits -----------------------------------------------------
       ;.equiv FRMSYPW, 0x0003
       ;.equiv MSSEN,   0x0004
       ;.equiv FRMPOL,  0x0005
       ;.equiv FRMSYNC, 0x0006
       ;.equiv FRMEN,   0x0007
       ;.equiv URDTEN,  0x000A
       ;.equiv AUDMONO, 0x000B
       ;.equiv IGNTUR,  0x000C
       ;.equiv IGNROV,  0x000D
       ;.equiv SPISGNEXT, 0x000E
       ;.equiv AUDEN,   0x000F

       ;.equiv FRMCNT0, 0x0000
       ;.equiv FRMCNT1, 0x0001
       ;.equiv FRMCNT2, 0x0002
       ;.equiv AUDMOD0, 0x0008
       ;.equiv AUDMOD1, 0x0009


;----- SPI2CON2 Bits -----------------------------------------------------
       ;.equiv FRMSYPW, 0x0003
       ;.equiv MSSEN,   0x0004
       ;.equiv FRMPOL,  0x0005
       ;.equiv FRMSYNC, 0x0006
       ;.equiv FRMEN,   0x0007
       ;.equiv URDTEN,  0x000A
       ;.equiv AUDMONO, 0x000B
       ;.equiv IGNTUR,  0x000C
       ;.equiv IGNROV,  0x000D
       ;.equiv SPISGNEXT, 0x000E
       ;.equiv AUDEN,   0x000F

       ;.equiv FRMCNT0, 0x0000
       ;.equiv FRMCNT1, 0x0001
       ;.equiv FRMCNT2, 0x0002
       ;.equiv AUDMOD0, 0x0008
       ;.equiv AUDMOD1, 0x0009


;----- SPI2STATL Bits -----------------------------------------------------
       ;.equiv SPIRBF,  0x0000
       ;.equiv SPITBF,  0x0001
       ;.equiv SPITBE,  0x0003
       ;.equiv SPIRBE,  0x0005
       ;.equiv SPIROV,  0x0006
       ;.equiv SRMT,    0x0007
       ;.equiv SPITUR,  0x0008
       ;.equiv SPIBUSY, 0x000B
       ;.equiv FRMERR,  0x000C


;----- SPI2STATH Bits -----------------------------------------------------
       ;.equiv TXELM0,  0x0000
       ;.equiv TXELM1,  0x0001
       ;.equiv TXELM2,  0x0002
       ;.equiv TXELM3,  0x0003
       ;.equiv TXELM4,  0x0004
       ;.equiv TXELM5,  0x0005
       ;.equiv RXELM0,  0x0008
       ;.equiv RXELM1,  0x0009
       ;.equiv RXELM2,  0x000A
       ;.equiv RXELM3,  0x000B
       ;.equiv RXELM4,  0x000C
       ;.equiv RXELM5,  0x000D


;----- SPI2IMSK1 Bits -----------------------------------------------------
       ;.equiv SPIRBFEN, 0x0000
       ;.equiv SPITBFEN, 0x0001
       ;.equiv SPITBEN, 0x0003
       ;.equiv SPIRBEN, 0x0005
       ;.equiv SPIROVEN, 0x0006
       ;.equiv SRMTEN,  0x0007
       ;.equiv SPITUREN, 0x0008
       ;.equiv BUSYEN,  0x000B
       ;.equiv FRMERREN, 0x000C


;----- SPI2IMSKL Bits -----------------------------------------------------
       ;.equiv SPIRBFEN, 0x0000
       ;.equiv SPITBFEN, 0x0001
       ;.equiv SPITBEN, 0x0003
       ;.equiv SPIRBEN, 0x0005
       ;.equiv SPIROVEN, 0x0006
       ;.equiv SRMTEN,  0x0007
       ;.equiv SPITUREN, 0x0008
       ;.equiv BUSYEN,  0x000B
       ;.equiv FRMERREN, 0x000C


;----- SPI2IMSK2 Bits -----------------------------------------------------
       ;.equiv TXWIEN,  0x0007
       ;.equiv RXWIEN,  0x000F

       ;.equiv TXMSK0,  0x0000
       ;.equiv TXMSK1,  0x0001
       ;.equiv TXMSK2,  0x0002
       ;.equiv TXMSK3,  0x0003
       ;.equiv TXMSK4,  0x0004
       ;.equiv TXMSK5,  0x0005
       ;.equiv RXMSK0,  0x0008
       ;.equiv RXMSK1,  0x0009
       ;.equiv RXMSK2,  0x000A
       ;.equiv RXMSK3,  0x000B
       ;.equiv RXMSK4,  0x000C
       ;.equiv RXMSK5,  0x000D


;----- SPI2IMSKH Bits -----------------------------------------------------
       ;.equiv TXWIEN,  0x0007
       ;.equiv RXWIEN,  0x000F

       ;.equiv TXMSK0,  0x0000
       ;.equiv TXMSK1,  0x0001
       ;.equiv TXMSK2,  0x0002
       ;.equiv TXMSK3,  0x0003
       ;.equiv TXMSK4,  0x0004
       ;.equiv TXMSK5,  0x0005
       ;.equiv RXMSK0,  0x0008
       ;.equiv RXMSK1,  0x0009
       ;.equiv RXMSK2,  0x000A
       ;.equiv RXMSK3,  0x000B
       ;.equiv RXMSK4,  0x000C
       ;.equiv RXMSK5,  0x000D


;----- SPI3CON1 Bits -----------------------------------------------------
       ;.equiv ENHBUF,  0x0000
       ;.equiv SPIFE,   0x0001
       ;.equiv MCLKEN,  0x0002
       ;.equiv DISSCK,  0x0003
       ;.equiv DISSDI,  0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv MODE32,  0x000B
       ;.equiv DISSDO,  0x000C
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F



;----- SPI3CON1L Bits -----------------------------------------------------
       ;.equiv ENHBUF,  0x0000
       ;.equiv SPIFE,   0x0001
       ;.equiv MCLKEN,  0x0002
       ;.equiv DISSCK,  0x0003
       ;.equiv DISSDI,  0x0004
       ;.equiv MSTEN,   0x0005
       ;.equiv CKP,     0x0006
       ;.equiv SSEN,    0x0007
       ;.equiv CKE,     0x0008
       ;.equiv SMP,     0x0009
       ;.equiv MODE16,  0x000A
       ;.equiv MODE32,  0x000B
       ;.equiv DISSDO,  0x000C
       ;.equiv SPISIDL, 0x000D
       ;.equiv SPIEN,   0x000F



;----- SPI3CON1H Bits -----------------------------------------------------
       ;.equiv FRMSYPW, 0x0003
       ;.equiv MSSEN,   0x0004
       ;.equiv FRMPOL,  0x0005
       ;.equiv FRMSYNC, 0x0006
       ;.equiv FRMEN,   0x0007
       ;.equiv URDTEN,  0x000A
       ;.equiv AUDMONO, 0x000B
       ;.equiv IGNTUR,  0x000C
       ;.equiv IGNROV,  0x000D
       ;.equiv SPISGNEXT, 0x000E
       ;.equiv AUDEN,   0x000F

       ;.equiv FRMCNT0, 0x0000
       ;.equiv FRMCNT1, 0x0001
       ;.equiv FRMCNT2, 0x0002
       ;.equiv AUDMOD0, 0x0008
       ;.equiv AUDMOD1, 0x0009


;----- SPI3CON2 Bits -----------------------------------------------------
       ;.equiv FRMSYPW, 0x0003
       ;.equiv MSSEN,   0x0004
       ;.equiv FRMPOL,  0x0005
       ;.equiv FRMSYNC, 0x0006
       ;.equiv FRMEN,   0x0007
       ;.equiv URDTEN,  0x000A
       ;.equiv AUDMONO, 0x000B
       ;.equiv IGNTUR,  0x000C
       ;.equiv IGNROV,  0x000D
       ;.equiv SPISGNEXT, 0x000E
       ;.equiv AUDEN,   0x000F

       ;.equiv FRMCNT0, 0x0000
       ;.equiv FRMCNT1, 0x0001
       ;.equiv FRMCNT2, 0x0002
       ;.equiv AUDMOD0, 0x0008
       ;.equiv AUDMOD1, 0x0009


;----- SPI3STATL Bits -----------------------------------------------------
       ;.equiv SPIRBF,  0x0000
       ;.equiv SPITBF,  0x0001
       ;.equiv SPITBE,  0x0003
       ;.equiv SPIRBE,  0x0005
       ;.equiv SPIROV,  0x0006
       ;.equiv SRMT,    0x0007
       ;.equiv SPITUR,  0x0008
       ;.equiv SPIBUSY, 0x000B
       ;.equiv FRMERR,  0x000C


;----- SPI3STATH Bits -----------------------------------------------------
       ;.equiv TXELM0,  0x0000
       ;.equiv TXELM1,  0x0001
       ;.equiv TXELM2,  0x0002
       ;.equiv TXELM3,  0x0003
       ;.equiv TXELM4,  0x0004
       ;.equiv TXELM5,  0x0005
       ;.equiv RXELM0,  0x0008
       ;.equiv RXELM1,  0x0009
       ;.equiv RXELM2,  0x000A
       ;.equiv RXELM3,  0x000B
       ;.equiv RXELM4,  0x000C
       ;.equiv RXELM5,  0x000D


;----- SPI3IMSK1 Bits -----------------------------------------------------
       ;.equiv SPIRBFEN, 0x0000
       ;.equiv SPITBFEN, 0x0001
       ;.equiv SPITBEN, 0x0003
       ;.equiv SPIRBEN, 0x0005
       ;.equiv SPIROVEN, 0x0006
       ;.equiv SRMTEN,  0x0007
       ;.equiv SPITUREN, 0x0008
       ;.equiv BUSYEN,  0x000B
       ;.equiv FRMERREN, 0x000C


;----- SPI3IMSKL Bits -----------------------------------------------------
       ;.equiv SPIRBFEN, 0x0000
       ;.equiv SPITBFEN, 0x0001
       ;.equiv SPITBEN, 0x0003
       ;.equiv SPIRBEN, 0x0005
       ;.equiv SPIROVEN, 0x0006
       ;.equiv SRMTEN,  0x0007
       ;.equiv SPITUREN, 0x0008
       ;.equiv BUSYEN,  0x000B
       ;.equiv FRMERREN, 0x000C


;----- SPI3IMSK2 Bits -----------------------------------------------------
       ;.equiv TXWIEN,  0x0007
       ;.equiv RXWIEN,  0x000F

       ;.equiv TXMSK0,  0x0000
       ;.equiv TXMSK1,  0x0001
       ;.equiv TXMSK2,  0x0002
       ;.equiv TXMSK3,  0x0003
       ;.equiv TXMSK4,  0x0004
       ;.equiv TXMSK5,  0x0005
       ;.equiv RXMSK0,  0x0008
       ;.equiv RXMSK1,  0x0009
       ;.equiv RXMSK2,  0x000A
       ;.equiv RXMSK3,  0x000B
       ;.equiv RXMSK4,  0x000C
       ;.equiv RXMSK5,  0x000D


;----- SPI3IMSKH Bits -----------------------------------------------------
       ;.equiv TXWIEN,  0x0007
       ;.equiv RXWIEN,  0x000F

       ;.equiv TXMSK0,  0x0000
       ;.equiv TXMSK1,  0x0001
       ;.equiv TXMSK2,  0x0002
       ;.equiv TXMSK3,  0x0003
       ;.equiv TXMSK4,  0x0004
       ;.equiv TXMSK5,  0x0005
       ;.equiv RXMSK0,  0x0008
       ;.equiv RXMSK1,  0x0009
       ;.equiv RXMSK2,  0x000A
       ;.equiv RXMSK3,  0x000B
       ;.equiv RXMSK4,  0x000C
       ;.equiv RXMSK5,  0x000D


;----- CLC1CONL Bits -----------------------------------------------------
        .equiv LCPOL,   0x0005
        .equiv LCOUT,   0x0006
        .equiv LCOE,    0x0007
        .equiv INTN,    0x000A
        .equiv INTP,    0x000B
        .equiv LCEN,    0x000F

        .equiv MODE0_CLC1CONL, 0x0000
        .equiv MODE1_CLC1CONL, 0x0001
        .equiv MODE2,   0x0002


;----- CLC1CONH Bits -----------------------------------------------------
        .equiv G1POL,   0x0000
        .equiv G2POL,   0x0001
        .equiv G3POL,   0x0002
        .equiv G4POL,   0x0003


;----- CLC1SEL Bits -----------------------------------------------------
        .equiv DS10,    0x0000
        .equiv DS11,    0x0001
        .equiv DS12,    0x0002
        .equiv DS20,    0x0004
        .equiv DS21,    0x0005
        .equiv DS22,    0x0006
        .equiv DS30,    0x0008
        .equiv DS31,    0x0009
        .equiv DS32,    0x000A
        .equiv DS40,    0x000C
        .equiv DS41,    0x000D
        .equiv DS42,    0x000E


;----- CLC1SELL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC1GLSL Bits -----------------------------------------------------
        .equiv G1D1N,   0x0000
        .equiv G1D1T,   0x0001
        .equiv G1D2N,   0x0002
        .equiv G1D2T,   0x0003
        .equiv G1D3N,   0x0004
        .equiv G1D3T,   0x0005
        .equiv G1D4N,   0x0006
        .equiv G1D4T,   0x0007
        .equiv G2D1N,   0x0008
        .equiv G2D1T,   0x0009
        .equiv G2D2N,   0x000A
        .equiv G2D2T,   0x000B
        .equiv G2D3N,   0x000C
        .equiv G2D3T,   0x000D
        .equiv G2D4N,   0x000E
        .equiv G2D4T,   0x000F


;----- CLC1GLSH Bits -----------------------------------------------------
        .equiv G3D1N,   0x0000
        .equiv G3D1T,   0x0001
        .equiv G3D2N,   0x0002
        .equiv G3D2T,   0x0003
        .equiv G3D3N,   0x0004
        .equiv G3D3T,   0x0005
        .equiv G3D4N,   0x0006
        .equiv G3D4T,   0x0007
        .equiv G4D1N,   0x0008
        .equiv G4D1T,   0x0009
        .equiv G4D2N,   0x000A
        .equiv G4D2T,   0x000B
        .equiv G4D3N,   0x000C
        .equiv G4D3T,   0x000D
        .equiv G4D4N,   0x000E
        .equiv G4D4T,   0x000F


;----- CLC2CONL Bits -----------------------------------------------------
       ;.equiv LCPOL,   0x0005
       ;.equiv LCOUT,   0x0006
       ;.equiv LCOE,    0x0007
       ;.equiv INTN,    0x000A
       ;.equiv INTP,    0x000B
       ;.equiv LCEN,    0x000F

        .equiv MODE0_CLC2CONL, 0x0000
        .equiv MODE1_CLC2CONL, 0x0001
       ;.equiv MODE2,   0x0002


;----- CLC2CONH Bits -----------------------------------------------------
       ;.equiv G1POL,   0x0000
       ;.equiv G2POL,   0x0001
       ;.equiv G3POL,   0x0002
       ;.equiv G4POL,   0x0003


;----- CLC2SEL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC2SELL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC2GLSL Bits -----------------------------------------------------
       ;.equiv G1D1N,   0x0000
       ;.equiv G1D1T,   0x0001
       ;.equiv G1D2N,   0x0002
       ;.equiv G1D2T,   0x0003
       ;.equiv G1D3N,   0x0004
       ;.equiv G1D3T,   0x0005
       ;.equiv G1D4N,   0x0006
       ;.equiv G1D4T,   0x0007
       ;.equiv G2D1N,   0x0008
       ;.equiv G2D1T,   0x0009
       ;.equiv G2D2N,   0x000A
       ;.equiv G2D2T,   0x000B
       ;.equiv G2D3N,   0x000C
       ;.equiv G2D3T,   0x000D
       ;.equiv G2D4N,   0x000E
       ;.equiv G2D4T,   0x000F


;----- CLC2GLSH Bits -----------------------------------------------------
       ;.equiv G3D1N,   0x0000
       ;.equiv G3D1T,   0x0001
       ;.equiv G3D2N,   0x0002
       ;.equiv G3D2T,   0x0003
       ;.equiv G3D3N,   0x0004
       ;.equiv G3D3T,   0x0005
       ;.equiv G3D4N,   0x0006
       ;.equiv G3D4T,   0x0007
       ;.equiv G4D1N,   0x0008
       ;.equiv G4D1T,   0x0009
       ;.equiv G4D2N,   0x000A
       ;.equiv G4D2T,   0x000B
       ;.equiv G4D3N,   0x000C
       ;.equiv G4D3T,   0x000D
       ;.equiv G4D4N,   0x000E
       ;.equiv G4D4T,   0x000F


;----- CLC3CONL Bits -----------------------------------------------------
       ;.equiv LCPOL,   0x0005
       ;.equiv LCOUT,   0x0006
       ;.equiv LCOE,    0x0007
       ;.equiv INTN,    0x000A
       ;.equiv INTP,    0x000B
       ;.equiv LCEN,    0x000F

        .equiv MODE0_CLC3CONL, 0x0000
        .equiv MODE1_CLC3CONL, 0x0001
       ;.equiv MODE2,   0x0002


;----- CLC3CONH Bits -----------------------------------------------------
       ;.equiv G1POL,   0x0000
       ;.equiv G2POL,   0x0001
       ;.equiv G3POL,   0x0002
       ;.equiv G4POL,   0x0003


;----- CLC3SEL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC3SELL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC3GLSL Bits -----------------------------------------------------
       ;.equiv G1D1N,   0x0000
       ;.equiv G1D1T,   0x0001
       ;.equiv G1D2N,   0x0002
       ;.equiv G1D2T,   0x0003
       ;.equiv G1D3N,   0x0004
       ;.equiv G1D3T,   0x0005
       ;.equiv G1D4N,   0x0006
       ;.equiv G1D4T,   0x0007
       ;.equiv G2D1N,   0x0008
       ;.equiv G2D1T,   0x0009
       ;.equiv G2D2N,   0x000A
       ;.equiv G2D2T,   0x000B
       ;.equiv G2D3N,   0x000C
       ;.equiv G2D3T,   0x000D
       ;.equiv G2D4N,   0x000E
       ;.equiv G2D4T,   0x000F


;----- CLC3GLSH Bits -----------------------------------------------------
       ;.equiv G3D1N,   0x0000
       ;.equiv G3D1T,   0x0001
       ;.equiv G3D2N,   0x0002
       ;.equiv G3D2T,   0x0003
       ;.equiv G3D3N,   0x0004
       ;.equiv G3D3T,   0x0005
       ;.equiv G3D4N,   0x0006
       ;.equiv G3D4T,   0x0007
       ;.equiv G4D1N,   0x0008
       ;.equiv G4D1T,   0x0009
       ;.equiv G4D2N,   0x000A
       ;.equiv G4D2T,   0x000B
       ;.equiv G4D3N,   0x000C
       ;.equiv G4D3T,   0x000D
       ;.equiv G4D4N,   0x000E
       ;.equiv G4D4T,   0x000F


;----- CLC4CONL Bits -----------------------------------------------------
       ;.equiv LCPOL,   0x0005
       ;.equiv LCOUT,   0x0006
       ;.equiv LCOE,    0x0007
       ;.equiv INTN,    0x000A
       ;.equiv INTP,    0x000B
       ;.equiv LCEN,    0x000F

        .equiv MODE0_CLC4CONL, 0x0000
        .equiv MODE1_CLC4CONL, 0x0001
       ;.equiv MODE2,   0x0002


;----- CLC4CONH Bits -----------------------------------------------------
       ;.equiv G1POL,   0x0000
       ;.equiv G2POL,   0x0001
       ;.equiv G3POL,   0x0002
       ;.equiv G4POL,   0x0003


;----- CLC4SEL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC4SELL Bits -----------------------------------------------------
       ;.equiv DS10,    0x0000
       ;.equiv DS11,    0x0001
       ;.equiv DS12,    0x0002
       ;.equiv DS20,    0x0004
       ;.equiv DS21,    0x0005
       ;.equiv DS22,    0x0006
       ;.equiv DS30,    0x0008
       ;.equiv DS31,    0x0009
       ;.equiv DS32,    0x000A
       ;.equiv DS40,    0x000C
       ;.equiv DS41,    0x000D
       ;.equiv DS42,    0x000E


;----- CLC4GLSL Bits -----------------------------------------------------
       ;.equiv G1D1N,   0x0000
       ;.equiv G1D1T,   0x0001
       ;.equiv G1D2N,   0x0002
       ;.equiv G1D2T,   0x0003
       ;.equiv G1D3N,   0x0004
       ;.equiv G1D3T,   0x0005
       ;.equiv G1D4N,   0x0006
       ;.equiv G1D4T,   0x0007
       ;.equiv G2D1N,   0x0008
       ;.equiv G2D1T,   0x0009
       ;.equiv G2D2N,   0x000A
       ;.equiv G2D2T,   0x000B
       ;.equiv G2D3N,   0x000C
       ;.equiv G2D3T,   0x000D
       ;.equiv G2D4N,   0x000E
       ;.equiv G2D4T,   0x000F


;----- CLC4GLSH Bits -----------------------------------------------------
       ;.equiv G3D1N,   0x0000
       ;.equiv G3D1T,   0x0001
       ;.equiv G3D2N,   0x0002
       ;.equiv G3D2T,   0x0003
       ;.equiv G3D3N,   0x0004
       ;.equiv G3D3T,   0x0005
       ;.equiv G3D4N,   0x0006
       ;.equiv G3D4T,   0x0007
       ;.equiv G4D1N,   0x0008
       ;.equiv G4D1T,   0x0009
       ;.equiv G4D2N,   0x000A
       ;.equiv G4D2T,   0x000B
       ;.equiv G4D3N,   0x000C
       ;.equiv G4D3T,   0x000D
       ;.equiv G4D4N,   0x000E
       ;.equiv G4D4T,   0x000F


;----- I2C1CON1 Bits -----------------------------------------------------
        .equiv SEN,     0x0000
        .equiv RSEN,    0x0001
        .equiv PEN,     0x0002
        .equiv RCEN,    0x0003
        .equiv ACKEN,   0x0004
        .equiv ACKDT,   0x0005
        .equiv STREN,   0x0006
        .equiv GCEN,    0x0007
        .equiv SMEN,    0x0008
        .equiv DISSLW,  0x0009
        .equiv A10M,    0x000A
        .equiv STRICT,  0x000B
        .equiv SCLREL,  0x000C
        .equiv I2CSIDL, 0x000D
        .equiv I2CEN,   0x000F

        .equiv SIDL,    0x000D


;----- I2C1CONL Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv STRICT,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F

       ;.equiv SIDL,    0x000D


;----- I2C1CON2 Bits -----------------------------------------------------
        .equiv DHEN,    0x0000
        .equiv AHEN,    0x0001
        .equiv SBCDE,   0x0002
        .equiv SDAHT,   0x0003
        .equiv BOEN,    0x0004
        .equiv SCIE,    0x0005
        .equiv PCIE,    0x0006


;----- I2C1CONH Bits -----------------------------------------------------
       ;.equiv DHEN,    0x0000
       ;.equiv AHEN,    0x0001
       ;.equiv SBCDE,   0x0002
       ;.equiv SDAHT,   0x0003
       ;.equiv BOEN,    0x0004
       ;.equiv SCIE,    0x0005
       ;.equiv PCIE,    0x0006


;----- I2C1STAT Bits -----------------------------------------------------
        .equiv TBF,     0x0000
        .equiv RBF,     0x0001
        .equiv R_NOT_W, 0x0002
        .equiv S,       0x0003
        .equiv P,       0x0004
        .equiv D_NOT_A, 0x0005
        .equiv I2COV,   0x0006
        .equiv IWCOL,   0x0007
        .equiv ADD10,   0x0008
        .equiv GCSTAT,  0x0009
        .equiv BCL,     0x000A
        .equiv ACKTIM,  0x000D
        .equiv TRSTAT,  0x000E
        .equiv ACKSTAT, 0x000F

        .equiv R_W,     0x0002
        .equiv D_A,     0x0005

        .equiv RW,      0x0002
        .equiv DA,      0x0005


;----- I2C2CON1 Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv STRICT,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F

       ;.equiv SIDL,    0x000D


;----- I2C2CONL Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv STRICT,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F

       ;.equiv SIDL,    0x000D


;----- I2C2CON2 Bits -----------------------------------------------------
       ;.equiv DHEN,    0x0000
       ;.equiv AHEN,    0x0001
       ;.equiv SBCDE,   0x0002
       ;.equiv SDAHT,   0x0003
       ;.equiv BOEN,    0x0004
       ;.equiv SCIE,    0x0005
       ;.equiv PCIE,    0x0006


;----- I2C2CONH Bits -----------------------------------------------------
       ;.equiv DHEN,    0x0000
       ;.equiv AHEN,    0x0001
       ;.equiv SBCDE,   0x0002
       ;.equiv SDAHT,   0x0003
       ;.equiv BOEN,    0x0004
       ;.equiv SCIE,    0x0005
       ;.equiv PCIE,    0x0006


;----- I2C2STAT Bits -----------------------------------------------------
       ;.equiv TBF,     0x0000
       ;.equiv RBF,     0x0001
       ;.equiv R_NOT_W, 0x0002
       ;.equiv S,       0x0003
       ;.equiv P,       0x0004
       ;.equiv D_NOT_A, 0x0005
       ;.equiv I2COV,   0x0006
       ;.equiv IWCOL,   0x0007
       ;.equiv ADD10,   0x0008
       ;.equiv GCSTAT,  0x0009
       ;.equiv BCL,     0x000A
       ;.equiv ACKTIM,  0x000D
       ;.equiv TRSTAT,  0x000E
       ;.equiv ACKSTAT, 0x000F

       ;.equiv R_W,     0x0002
       ;.equiv D_A,     0x0005

       ;.equiv RW,      0x0002
       ;.equiv DA,      0x0005


;----- I2C3CON1 Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv STRICT,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F

       ;.equiv SIDL,    0x000D


;----- I2C3CONL Bits -----------------------------------------------------
       ;.equiv SEN,     0x0000
       ;.equiv RSEN,    0x0001
       ;.equiv PEN,     0x0002
       ;.equiv RCEN,    0x0003
       ;.equiv ACKEN,   0x0004
       ;.equiv ACKDT,   0x0005
       ;.equiv STREN,   0x0006
       ;.equiv GCEN,    0x0007
       ;.equiv SMEN,    0x0008
       ;.equiv DISSLW,  0x0009
       ;.equiv A10M,    0x000A
       ;.equiv STRICT,  0x000B
       ;.equiv SCLREL,  0x000C
       ;.equiv I2CSIDL, 0x000D
       ;.equiv I2CEN,   0x000F

       ;.equiv SIDL,    0x000D


;----- I2C3CON2 Bits -----------------------------------------------------
       ;.equiv DHEN,    0x0000
       ;.equiv AHEN,    0x0001
       ;.equiv SBCDE,   0x0002
       ;.equiv SDAHT,   0x0003
       ;.equiv BOEN,    0x0004
       ;.equiv SCIE,    0x0005
       ;.equiv PCIE,    0x0006


;----- I2C3CONH Bits -----------------------------------------------------
       ;.equiv DHEN,    0x0000
       ;.equiv AHEN,    0x0001
       ;.equiv SBCDE,   0x0002
       ;.equiv SDAHT,   0x0003
       ;.equiv BOEN,    0x0004
       ;.equiv SCIE,    0x0005
       ;.equiv PCIE,    0x0006


;----- I2C3STAT Bits -----------------------------------------------------
       ;.equiv TBF,     0x0000
       ;.equiv RBF,     0x0001
       ;.equiv R_NOT_W, 0x0002
       ;.equiv S,       0x0003
       ;.equiv P,       0x0004
       ;.equiv D_NOT_A, 0x0005
       ;.equiv I2COV,   0x0006
       ;.equiv IWCOL,   0x0007
       ;.equiv ADD10,   0x0008
       ;.equiv GCSTAT,  0x0009
       ;.equiv BCL,     0x000A
       ;.equiv ACKTIM,  0x000D
       ;.equiv TRSTAT,  0x000E
       ;.equiv ACKSTAT, 0x000F

       ;.equiv R_W,     0x0002
       ;.equiv D_A,     0x0005

       ;.equiv RW,      0x0002
       ;.equiv DA,      0x0005


;----- DMACON Bits -----------------------------------------------------
        .equiv PRSSEL,  0x0000
        .equiv DMASIDL, 0x000D
        .equiv DMAEN_DMACON, 0x000F


;----- DMACH0 Bits -----------------------------------------------------
        .equiv CHEN,    0x0000
        .equiv SIZE,    0x0001
        .equiv CHREQ,   0x0008
        .equiv RELOAD,  0x0009
        .equiv NULLW,   0x000A

        .equiv TRMODE0, 0x0002
        .equiv TRMODE1, 0x0003
        .equiv DAMODE0, 0x0004
        .equiv DAMODE1, 0x0005
        .equiv SAMODE0, 0x0006
        .equiv SAMODE1, 0x0007


;----- DMAINT0 Bits -----------------------------------------------------
        .equiv HALFEN,  0x0000
        .equiv OVRUNIF, 0x0003
        .equiv HALFIF,  0x0004
        .equiv DONEIF,  0x0005
        .equiv LOWIF,   0x0006
        .equiv HIGHIF,  0x0007
        .equiv DBUFWF,  0x000F

        .equiv CHSEL0,  0x0008
        .equiv CHSEL1,  0x0009
        .equiv CHSEL2,  0x000A
        .equiv CHSEL3,  0x000B
        .equiv CHSEL4,  0x000C
        .equiv CHSEL5,  0x000D
        .equiv CHSEL6,  0x000E


;----- DMACH1 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT1 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH2 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT2 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH3 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT3 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH4 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT4 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH5 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT5 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH6 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT6 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- DMACH7 Bits -----------------------------------------------------
       ;.equiv CHEN,    0x0000
       ;.equiv SIZE,    0x0001
       ;.equiv CHREQ,   0x0008
       ;.equiv RELOAD,  0x0009
       ;.equiv NULLW,   0x000A

       ;.equiv TRMODE0, 0x0002
       ;.equiv TRMODE1, 0x0003
       ;.equiv DAMODE0, 0x0004
       ;.equiv DAMODE1, 0x0005
       ;.equiv SAMODE0, 0x0006
       ;.equiv SAMODE1, 0x0007


;----- DMAINT7 Bits -----------------------------------------------------
       ;.equiv HALFEN,  0x0000
       ;.equiv OVRUNIF, 0x0003
       ;.equiv HALFIF,  0x0004
       ;.equiv DONEIF,  0x0005
       ;.equiv LOWIF,   0x0006
       ;.equiv HIGHIF,  0x0007
       ;.equiv DBUFWF,  0x000F

       ;.equiv CHSEL0,  0x0008
       ;.equiv CHSEL1,  0x0009
       ;.equiv CHSEL2,  0x000A
       ;.equiv CHSEL3,  0x000B
       ;.equiv CHSEL4,  0x000C
       ;.equiv CHSEL5,  0x000D
       ;.equiv CHSEL6,  0x000E


;----- PADCON Bits -----------------------------------------------------
        .equiv PMPTTL,  0x0000
        .equiv IOCON,   0x000F


;----- IOCSTAT Bits -----------------------------------------------------
        .equiv IOCPBF,  0x0001
        .equiv IOCPCF,  0x0002
        .equiv IOCPDF,  0x0003
        .equiv IOCPEF,  0x0004
        .equiv IOCPFF,  0x0005
        .equiv IOCPGF,  0x0006


;----- TRISB Bits -----------------------------------------------------
        .equiv TRISB0,  0x0000
        .equiv TRISB1,  0x0001
        .equiv TRISB2,  0x0002
        .equiv TRISB3,  0x0003
        .equiv TRISB4,  0x0004
        .equiv TRISB5,  0x0005
        .equiv TRISB6,  0x0006
        .equiv TRISB7,  0x0007
        .equiv TRISB8,  0x0008
        .equiv TRISB9,  0x0009
        .equiv TRISB10, 0x000A
        .equiv TRISB11, 0x000B
        .equiv TRISB12, 0x000C
        .equiv TRISB13, 0x000D
        .equiv TRISB14, 0x000E
        .equiv TRISB15, 0x000F


;----- PORTB Bits -----------------------------------------------------
        .equiv RB0,     0x0000
        .equiv RB1,     0x0001
        .equiv RB2,     0x0002
        .equiv RB3,     0x0003
        .equiv RB4,     0x0004
        .equiv RB5,     0x0005
        .equiv RB6,     0x0006
        .equiv RB7,     0x0007
        .equiv RB8,     0x0008
        .equiv RB9,     0x0009
        .equiv RB10,    0x000A
        .equiv RB11,    0x000B
        .equiv RB12,    0x000C
        .equiv RB13,    0x000D
        .equiv RB14,    0x000E
        .equiv RB15,    0x000F


;----- LATB Bits -----------------------------------------------------
        .equiv LATB0,   0x0000
        .equiv LATB1,   0x0001
        .equiv LATB2,   0x0002
        .equiv LATB3,   0x0003
        .equiv LATB4,   0x0004
        .equiv LATB5,   0x0005
        .equiv LATB6,   0x0006
        .equiv LATB7,   0x0007
        .equiv LATB8,   0x0008
        .equiv LATB9,   0x0009
        .equiv LATB10,  0x000A
        .equiv LATB11,  0x000B
        .equiv LATB12,  0x000C
        .equiv LATB13,  0x000D
        .equiv LATB14,  0x000E
        .equiv LATB15,  0x000F


;----- ODCB Bits -----------------------------------------------------
        .equiv ODCB0,   0x0000
        .equiv ODCB1,   0x0001
        .equiv ODCB2,   0x0002
        .equiv ODCB3,   0x0003
        .equiv ODCB4,   0x0004
        .equiv ODCB5,   0x0005
        .equiv ODCB6,   0x0006
        .equiv ODCB7,   0x0007
        .equiv ODCB8,   0x0008
        .equiv ODCB9,   0x0009
        .equiv ODCB10,  0x000A
        .equiv ODCB11,  0x000B
        .equiv ODCB12,  0x000C
        .equiv ODCB13,  0x000D
        .equiv ODCB14,  0x000E
        .equiv ODCB15,  0x000F


;----- ANSB Bits -----------------------------------------------------
        .equiv ANSB0,   0x0000
        .equiv ANSB1,   0x0001
        .equiv ANSB2,   0x0002
        .equiv ANSB3,   0x0003
        .equiv ANSB4,   0x0004
        .equiv ANSB5,   0x0005
        .equiv ANSB6,   0x0006
        .equiv ANSB7,   0x0007
        .equiv ANSB8,   0x0008
        .equiv ANSB9,   0x0009
        .equiv ANSB10,  0x000A
        .equiv ANSB11,  0x000B
        .equiv ANSB12,  0x000C
        .equiv ANSB13,  0x000D
        .equiv ANSB14,  0x000E
        .equiv ANSB15,  0x000F

        .equiv ANSELB0, 0x0000
        .equiv ANSELB1, 0x0001
        .equiv ANSELB2, 0x0002
        .equiv ANSELB3, 0x0003
        .equiv ANSELB4, 0x0004
        .equiv ANSELB5, 0x0005
        .equiv ANSELB6, 0x0006
        .equiv ANSELB7, 0x0007
        .equiv ANSELB8, 0x0008
        .equiv ANSELB9, 0x0009
        .equiv ANSELB10, 0x000A
        .equiv ANSELB11, 0x000B
        .equiv ANSELB12, 0x000C
        .equiv ANSELB13, 0x000D
        .equiv ANSELB14, 0x000E
        .equiv ANSELB15, 0x000F


;----- ANSELB Bits -----------------------------------------------------
       ;.equiv ANSB0,   0x0000
       ;.equiv ANSB1,   0x0001
       ;.equiv ANSB2,   0x0002
       ;.equiv ANSB3,   0x0003
       ;.equiv ANSB4,   0x0004
       ;.equiv ANSB5,   0x0005
       ;.equiv ANSB6,   0x0006
       ;.equiv ANSB7,   0x0007
       ;.equiv ANSB8,   0x0008
       ;.equiv ANSB9,   0x0009
       ;.equiv ANSB10,  0x000A
       ;.equiv ANSB11,  0x000B
       ;.equiv ANSB12,  0x000C
       ;.equiv ANSB13,  0x000D
       ;.equiv ANSB14,  0x000E
       ;.equiv ANSB15,  0x000F

       ;.equiv ANSELB0, 0x0000
       ;.equiv ANSELB1, 0x0001
       ;.equiv ANSELB2, 0x0002
       ;.equiv ANSELB3, 0x0003
       ;.equiv ANSELB4, 0x0004
       ;.equiv ANSELB5, 0x0005
       ;.equiv ANSELB6, 0x0006
       ;.equiv ANSELB7, 0x0007
       ;.equiv ANSELB8, 0x0008
       ;.equiv ANSELB9, 0x0009
       ;.equiv ANSELB10, 0x000A
       ;.equiv ANSELB11, 0x000B
       ;.equiv ANSELB12, 0x000C
       ;.equiv ANSELB13, 0x000D
       ;.equiv ANSELB14, 0x000E
       ;.equiv ANSELB15, 0x000F


;----- IOCPB Bits -----------------------------------------------------
        .equiv IOCPB0,  0x0000
        .equiv IOCPB1,  0x0001
        .equiv IOCPB2,  0x0002
        .equiv IOCPB3,  0x0003
        .equiv IOCPB4,  0x0004
        .equiv IOCPB5,  0x0005
        .equiv IOCPB6,  0x0006
        .equiv IOCPB7,  0x0007
        .equiv IOCPB8,  0x0008
        .equiv IOCPB9,  0x0009
        .equiv IOCPB10, 0x000A
        .equiv IOCPB11, 0x000B
        .equiv IOCPB12, 0x000C
        .equiv IOCPB13, 0x000D
        .equiv IOCPB14, 0x000E
        .equiv IOCPB15, 0x000F


;----- IOCNB Bits -----------------------------------------------------
        .equiv IOCNB0,  0x0000
        .equiv IOCNB1,  0x0001
        .equiv IOCNB2,  0x0002
        .equiv IOCNB3,  0x0003
        .equiv IOCNB4,  0x0004
        .equiv IOCNB5,  0x0005
        .equiv IOCNB6,  0x0006
        .equiv IOCNB7,  0x0007
        .equiv IOCNB8,  0x0008
        .equiv IOCNB9,  0x0009
        .equiv IOCNB10, 0x000A
        .equiv IOCNB11, 0x000B
        .equiv IOCNB12, 0x000C
        .equiv IOCNB13, 0x000D
        .equiv IOCNB14, 0x000E
        .equiv IOCNB15, 0x000F


;----- IOCFB Bits -----------------------------------------------------
        .equiv IOCFB0,  0x0000
        .equiv IOCFB1,  0x0001
        .equiv IOCFB2,  0x0002
        .equiv IOCFB3,  0x0003
        .equiv IOCFB4,  0x0004
        .equiv IOCFB5,  0x0005
        .equiv IOCFB6,  0x0006
        .equiv IOCFB7,  0x0007
        .equiv IOCFB8,  0x0008
        .equiv IOCFB9,  0x0009
        .equiv IOCFB10, 0x000A
        .equiv IOCFB11, 0x000B
        .equiv IOCFB12, 0x000C
        .equiv IOCFB13, 0x000D
        .equiv IOCFB14, 0x000E
        .equiv IOCFB15, 0x000F


;----- CNPUB Bits -----------------------------------------------------
       ;.equiv IOCPB0,  0x0000
       ;.equiv IOCPB1,  0x0001
       ;.equiv IOCPB2,  0x0002
       ;.equiv IOCPB3,  0x0003
       ;.equiv IOCPB4,  0x0004
       ;.equiv IOCPB5,  0x0005
       ;.equiv IOCPB6,  0x0006
       ;.equiv IOCPB7,  0x0007
       ;.equiv IOCPB8,  0x0008
       ;.equiv IOCPB9,  0x0009
       ;.equiv IOCPB10, 0x000A
       ;.equiv IOCPB11, 0x000B
       ;.equiv IOCPB12, 0x000C
       ;.equiv IOCPB13, 0x000D
       ;.equiv IOCPB14, 0x000E
       ;.equiv IOCPB15, 0x000F

        .equiv IOCPUB0, 0x0000
        .equiv IOCPUB1, 0x0001
        .equiv IOCPUB2, 0x0002
        .equiv IOCPUB3, 0x0003
        .equiv IOCPUB4, 0x0004
        .equiv IOCPUB5, 0x0005
        .equiv IOCPUB6, 0x0006
        .equiv IOCPUB7, 0x0007
        .equiv IOCPUB8, 0x0008
        .equiv IOCPUB9, 0x0009
        .equiv IOCPUB10, 0x000A
        .equiv IOCPUB11, 0x000B
        .equiv IOCPUB12, 0x000C
        .equiv IOCPUB13, 0x000D
        .equiv IOCPUB14, 0x000E
        .equiv IOCPUB15, 0x000F

        .equiv CNPUB0,  0x0000
        .equiv CNPUB1,  0x0001
        .equiv CNPUB2,  0x0002
        .equiv CNPUB3,  0x0003
        .equiv CNPUB4,  0x0004
        .equiv CNPUB5,  0x0005
        .equiv CNPUB6,  0x0006
        .equiv CNPUB7,  0x0007
        .equiv CNPUB8,  0x0008
        .equiv CNPUB9,  0x0009
        .equiv CNPUB10, 0x000A
        .equiv CNPUB11, 0x000B
        .equiv CNPUB12, 0x000C
        .equiv CNPUB13, 0x000D
        .equiv CNPUB14, 0x000E
        .equiv CNPUB15, 0x000F


;----- IOCPUB Bits -----------------------------------------------------
       ;.equiv IOCPB0,  0x0000
       ;.equiv IOCPB1,  0x0001
       ;.equiv IOCPB2,  0x0002
       ;.equiv IOCPB3,  0x0003
       ;.equiv IOCPB4,  0x0004
       ;.equiv IOCPB5,  0x0005
       ;.equiv IOCPB6,  0x0006
       ;.equiv IOCPB7,  0x0007
       ;.equiv IOCPB8,  0x0008
       ;.equiv IOCPB9,  0x0009
       ;.equiv IOCPB10, 0x000A
       ;.equiv IOCPB11, 0x000B
       ;.equiv IOCPB12, 0x000C
       ;.equiv IOCPB13, 0x000D
       ;.equiv IOCPB14, 0x000E
       ;.equiv IOCPB15, 0x000F

       ;.equiv IOCPUB0, 0x0000
       ;.equiv IOCPUB1, 0x0001
       ;.equiv IOCPUB2, 0x0002
       ;.equiv IOCPUB3, 0x0003
       ;.equiv IOCPUB4, 0x0004
       ;.equiv IOCPUB5, 0x0005
       ;.equiv IOCPUB6, 0x0006
       ;.equiv IOCPUB7, 0x0007
       ;.equiv IOCPUB8, 0x0008
       ;.equiv IOCPUB9, 0x0009
       ;.equiv IOCPUB10, 0x000A
       ;.equiv IOCPUB11, 0x000B
       ;.equiv IOCPUB12, 0x000C
       ;.equiv IOCPUB13, 0x000D
       ;.equiv IOCPUB14, 0x000E
       ;.equiv IOCPUB15, 0x000F

       ;.equiv CNPUB0,  0x0000
       ;.equiv CNPUB1,  0x0001
       ;.equiv CNPUB2,  0x0002
       ;.equiv CNPUB3,  0x0003
       ;.equiv CNPUB4,  0x0004
       ;.equiv CNPUB5,  0x0005
       ;.equiv CNPUB6,  0x0006
       ;.equiv CNPUB7,  0x0007
       ;.equiv CNPUB8,  0x0008
       ;.equiv CNPUB9,  0x0009
       ;.equiv CNPUB10, 0x000A
       ;.equiv CNPUB11, 0x000B
       ;.equiv CNPUB12, 0x000C
       ;.equiv CNPUB13, 0x000D
       ;.equiv CNPUB14, 0x000E
       ;.equiv CNPUB15, 0x000F


;----- CNPDB Bits -----------------------------------------------------
        .equiv IOCPDB0, 0x0000
        .equiv IOCPDB1, 0x0001
        .equiv IOCPDB2, 0x0002
        .equiv IOCPDB3, 0x0003
        .equiv IOCPDB4, 0x0004
        .equiv IOCPDB5, 0x0005
        .equiv IOCPDB6, 0x0006
        .equiv IOCPDB7, 0x0007
        .equiv IOCPDB8, 0x0008
        .equiv IOCPDB9, 0x0009
        .equiv IOCPDB10, 0x000A
        .equiv IOCPDB11, 0x000B
        .equiv IOCPDB12, 0x000C
        .equiv IOCPDB13, 0x000D
        .equiv IOCPDB14, 0x000E
        .equiv IOCPDB15, 0x000F

        .equiv CNPDB0,  0x0000
        .equiv CNPDB1,  0x0001
        .equiv CNPDB2,  0x0002
        .equiv CNPDB3,  0x0003
        .equiv CNPDB4,  0x0004
        .equiv CNPDB5,  0x0005
        .equiv CNPDB6,  0x0006
        .equiv CNPDB7,  0x0007
        .equiv CNPDB8,  0x0008
        .equiv CNPDB9,  0x0009
        .equiv CNPDB10, 0x000A
        .equiv CNPDB11, 0x000B
        .equiv CNPDB12, 0x000C
        .equiv CNPDB13, 0x000D
        .equiv CNPDB14, 0x000E
        .equiv CNPDB15, 0x000F


;----- IOCPDB Bits -----------------------------------------------------
       ;.equiv IOCPDB0, 0x0000
       ;.equiv IOCPDB1, 0x0001
       ;.equiv IOCPDB2, 0x0002
       ;.equiv IOCPDB3, 0x0003
       ;.equiv IOCPDB4, 0x0004
       ;.equiv IOCPDB5, 0x0005
       ;.equiv IOCPDB6, 0x0006
       ;.equiv IOCPDB7, 0x0007
       ;.equiv IOCPDB8, 0x0008
       ;.equiv IOCPDB9, 0x0009
       ;.equiv IOCPDB10, 0x000A
       ;.equiv IOCPDB11, 0x000B
       ;.equiv IOCPDB12, 0x000C
       ;.equiv IOCPDB13, 0x000D
       ;.equiv IOCPDB14, 0x000E
       ;.equiv IOCPDB15, 0x000F

       ;.equiv CNPDB0,  0x0000
       ;.equiv CNPDB1,  0x0001
       ;.equiv CNPDB2,  0x0002
       ;.equiv CNPDB3,  0x0003
       ;.equiv CNPDB4,  0x0004
       ;.equiv CNPDB5,  0x0005
       ;.equiv CNPDB6,  0x0006
       ;.equiv CNPDB7,  0x0007
       ;.equiv CNPDB8,  0x0008
       ;.equiv CNPDB9,  0x0009
       ;.equiv CNPDB10, 0x000A
       ;.equiv CNPDB11, 0x000B
       ;.equiv CNPDB12, 0x000C
       ;.equiv CNPDB13, 0x000D
       ;.equiv CNPDB14, 0x000E
       ;.equiv CNPDB15, 0x000F


;----- TRISC Bits -----------------------------------------------------
        .equiv TRISC12, 0x000C
        .equiv TRISC13, 0x000D
        .equiv TRISC14, 0x000E
        .equiv TRISC15, 0x000F


;----- PORTC Bits -----------------------------------------------------
        .equiv RC12,    0x000C
        .equiv RC13,    0x000D
        .equiv RC14,    0x000E
        .equiv RC15,    0x000F


;----- LATC Bits -----------------------------------------------------
        .equiv LATC12,  0x000C
        .equiv LATC13,  0x000D
        .equiv LATC14,  0x000E
        .equiv LATC15,  0x000F


;----- ODCC Bits -----------------------------------------------------
        .equiv ODCC12,  0x000C
        .equiv ODCC13,  0x000D
        .equiv ODCC14,  0x000E
        .equiv ODCC15,  0x000F


;----- ANSC Bits -----------------------------------------------------
        .equiv ANSC13,  0x000D
        .equiv ANSC14,  0x000E

        .equiv ANSELC13, 0x000D
        .equiv ANSELC14, 0x000E


;----- ANSELC Bits -----------------------------------------------------
       ;.equiv ANSC13,  0x000D
       ;.equiv ANSC14,  0x000E

       ;.equiv ANSELC13, 0x000D
       ;.equiv ANSELC14, 0x000E


;----- IOCPC Bits -----------------------------------------------------
        .equiv IOCPC12, 0x000C
        .equiv IOCPC13, 0x000D
        .equiv IOCPC14, 0x000E
        .equiv IOCPC15, 0x000F


;----- IOCNC Bits -----------------------------------------------------
        .equiv IOCNC12, 0x000C
        .equiv IOCNC13, 0x000D
        .equiv IOCNC14, 0x000E
        .equiv IOCNC15, 0x000F


;----- IOCFC Bits -----------------------------------------------------
        .equiv IOCFC12, 0x000C
        .equiv IOCFC13, 0x000D
        .equiv IOCFC14, 0x000E
        .equiv IOCFC15, 0x000F


;----- CNPUC Bits -----------------------------------------------------
       ;.equiv IOCPC12, 0x000C
       ;.equiv IOCPC13, 0x000D
       ;.equiv IOCPC14, 0x000E
       ;.equiv IOCPC15, 0x000F

        .equiv IOCPUC12, 0x000C
        .equiv IOCPUC13, 0x000D
        .equiv IOCPUC14, 0x000E
        .equiv IOCPUC15, 0x000F

        .equiv CNPUC12, 0x000C
        .equiv CNPUC13, 0x000D
        .equiv CNPUC14, 0x000E
        .equiv CNPUC15, 0x000F


;----- IOCPUC Bits -----------------------------------------------------
       ;.equiv IOCPC12, 0x000C
       ;.equiv IOCPC13, 0x000D
       ;.equiv IOCPC14, 0x000E
       ;.equiv IOCPC15, 0x000F

       ;.equiv IOCPUC12, 0x000C
       ;.equiv IOCPUC13, 0x000D
       ;.equiv IOCPUC14, 0x000E
       ;.equiv IOCPUC15, 0x000F

       ;.equiv CNPUC12, 0x000C
       ;.equiv CNPUC13, 0x000D
       ;.equiv CNPUC14, 0x000E
       ;.equiv CNPUC15, 0x000F


;----- CNPDC Bits -----------------------------------------------------
        .equiv IOCPDC12, 0x000C
        .equiv IOCPDC13, 0x000D
        .equiv IOCPDC14, 0x000E
        .equiv IOCPDC15, 0x000F

        .equiv CNPDC12, 0x000C
        .equiv CNPDC13, 0x000D
        .equiv CNPDC14, 0x000E
        .equiv CNPDC15, 0x000F


;----- IOCPDC Bits -----------------------------------------------------
       ;.equiv IOCPDC12, 0x000C
       ;.equiv IOCPDC13, 0x000D
       ;.equiv IOCPDC14, 0x000E
       ;.equiv IOCPDC15, 0x000F

       ;.equiv CNPDC12, 0x000C
       ;.equiv CNPDC13, 0x000D
       ;.equiv CNPDC14, 0x000E
       ;.equiv CNPDC15, 0x000F


;----- TRISD Bits -----------------------------------------------------
        .equiv TRISD0,  0x0000
        .equiv TRISD1,  0x0001
        .equiv TRISD2,  0x0002
        .equiv TRISD3,  0x0003
        .equiv TRISD4,  0x0004
        .equiv TRISD5,  0x0005
        .equiv TRISD6,  0x0006
        .equiv TRISD7,  0x0007
        .equiv TRISD8,  0x0008
        .equiv TRISD9,  0x0009
        .equiv TRISD10, 0x000A
        .equiv TRISD11, 0x000B


;----- PORTD Bits -----------------------------------------------------
        .equiv RD0,     0x0000
        .equiv RD1,     0x0001
        .equiv RD2,     0x0002
        .equiv RD3,     0x0003
        .equiv RD4,     0x0004
        .equiv RD5,     0x0005
        .equiv RD6,     0x0006
        .equiv RD7,     0x0007
        .equiv RD8,     0x0008
        .equiv RD9,     0x0009
        .equiv RD10,    0x000A
        .equiv RD11,    0x000B


;----- LATD Bits -----------------------------------------------------
        .equiv LATD0,   0x0000
        .equiv LATD1,   0x0001
        .equiv LATD2,   0x0002
        .equiv LATD3,   0x0003
        .equiv LATD4,   0x0004
        .equiv LATD5,   0x0005
        .equiv LATD6,   0x0006
        .equiv LATD7,   0x0007
        .equiv LATD8,   0x0008
        .equiv LATD9,   0x0009
        .equiv LATD10,  0x000A
        .equiv LATD11,  0x000B


;----- ODCD Bits -----------------------------------------------------
        .equiv ODCD0,   0x0000
        .equiv ODCD1,   0x0001
        .equiv ODCD2,   0x0002
        .equiv ODCD3,   0x0003
        .equiv ODCD4,   0x0004
        .equiv ODCD5,   0x0005
        .equiv ODCD6,   0x0006
        .equiv ODCD7,   0x0007
        .equiv ODCD8,   0x0008
        .equiv ODCD9,   0x0009
        .equiv ODCD10,  0x000A
        .equiv ODCD11,  0x000B


;----- ANSD Bits -----------------------------------------------------
        .equiv ANSD6,   0x0006
        .equiv ANSD7,   0x0007

        .equiv ANSELD6, 0x0006
        .equiv ANSELD7, 0x0007


;----- ANSELD Bits -----------------------------------------------------
       ;.equiv ANSD6,   0x0006
       ;.equiv ANSD7,   0x0007

       ;.equiv ANSELD6, 0x0006
       ;.equiv ANSELD7, 0x0007


;----- IOCPD Bits -----------------------------------------------------
        .equiv IOCPD0,  0x0000
        .equiv IOCPD1,  0x0001
        .equiv IOCPD2,  0x0002
        .equiv IOCPD3,  0x0003
        .equiv IOCPD4,  0x0004
        .equiv IOCPD5,  0x0005
        .equiv IOCPD6,  0x0006
        .equiv IOCPD7,  0x0007
        .equiv IOCPD8,  0x0008
        .equiv IOCPD9,  0x0009
        .equiv IOCPD10, 0x000A
        .equiv IOCPD11, 0x000B


;----- IOCND Bits -----------------------------------------------------
        .equiv IOCND0,  0x0000
        .equiv IOCND1,  0x0001
        .equiv IOCND2,  0x0002
        .equiv IOCND3,  0x0003
        .equiv IOCND4,  0x0004
        .equiv IOCND5,  0x0005
        .equiv IOCND6,  0x0006
        .equiv IOCND7,  0x0007
        .equiv IOCND8,  0x0008
        .equiv IOCND9,  0x0009
        .equiv IOCND10, 0x000A
        .equiv IOCND11, 0x000B


;----- IOCFD Bits -----------------------------------------------------
        .equiv IOCFD0,  0x0000
        .equiv IOCFD1,  0x0001
        .equiv IOCFD2,  0x0002
        .equiv IOCFD3,  0x0003
        .equiv IOCFD4,  0x0004
        .equiv IOCFD5,  0x0005
        .equiv IOCFD6,  0x0006
        .equiv IOCFD7,  0x0007
        .equiv IOCFD8,  0x0008
        .equiv IOCFD9,  0x0009
        .equiv IOCFD10, 0x000A
        .equiv IOCFD11, 0x000B


;----- CNPUD Bits -----------------------------------------------------
       ;.equiv IOCPD0,  0x0000
       ;.equiv IOCPD1,  0x0001
       ;.equiv IOCPD2,  0x0002
       ;.equiv IOCPD3,  0x0003
       ;.equiv IOCPD4,  0x0004
       ;.equiv IOCPD5,  0x0005
       ;.equiv IOCPD6,  0x0006
       ;.equiv IOCPD7,  0x0007
       ;.equiv IOCPD8,  0x0008
       ;.equiv IOCPD9,  0x0009
       ;.equiv IOCPD10, 0x000A
       ;.equiv IOCPD11, 0x000B

        .equiv IOCPUD0, 0x0000
        .equiv IOCPUD1, 0x0001
        .equiv IOCPUD2, 0x0002
        .equiv IOCPUD3, 0x0003
        .equiv IOCPUD4, 0x0004
        .equiv IOCPUD5, 0x0005
        .equiv IOCPUD6, 0x0006
        .equiv IOCPUD7, 0x0007
        .equiv IOCPUD8, 0x0008
        .equiv IOCPUD9, 0x0009
        .equiv IOCPUD10, 0x000A
        .equiv IOCPUD11, 0x000B

        .equiv CNPUD0,  0x0000
        .equiv CNPUD1,  0x0001
        .equiv CNPUD2,  0x0002
        .equiv CNPUD3,  0x0003
        .equiv CNPUD4,  0x0004
        .equiv CNPUD5,  0x0005
        .equiv CNPUD6,  0x0006
        .equiv CNPUD7,  0x0007
        .equiv CNPUD8,  0x0008
        .equiv CNPUD9,  0x0009
        .equiv CNPUD10, 0x000A
        .equiv CNPUD11, 0x000B


;----- IOCPUD Bits -----------------------------------------------------
       ;.equiv IOCPD0,  0x0000
       ;.equiv IOCPD1,  0x0001
       ;.equiv IOCPD2,  0x0002
       ;.equiv IOCPD3,  0x0003
       ;.equiv IOCPD4,  0x0004
       ;.equiv IOCPD5,  0x0005
       ;.equiv IOCPD6,  0x0006
       ;.equiv IOCPD7,  0x0007
       ;.equiv IOCPD8,  0x0008
       ;.equiv IOCPD9,  0x0009
       ;.equiv IOCPD10, 0x000A
       ;.equiv IOCPD11, 0x000B

       ;.equiv IOCPUD0, 0x0000
       ;.equiv IOCPUD1, 0x0001
       ;.equiv IOCPUD2, 0x0002
       ;.equiv IOCPUD3, 0x0003
       ;.equiv IOCPUD4, 0x0004
       ;.equiv IOCPUD5, 0x0005
       ;.equiv IOCPUD6, 0x0006
       ;.equiv IOCPUD7, 0x0007
       ;.equiv IOCPUD8, 0x0008
       ;.equiv IOCPUD9, 0x0009
       ;.equiv IOCPUD10, 0x000A
       ;.equiv IOCPUD11, 0x000B

       ;.equiv CNPUD0,  0x0000
       ;.equiv CNPUD1,  0x0001
       ;.equiv CNPUD2,  0x0002
       ;.equiv CNPUD3,  0x0003
       ;.equiv CNPUD4,  0x0004
       ;.equiv CNPUD5,  0x0005
       ;.equiv CNPUD6,  0x0006
       ;.equiv CNPUD7,  0x0007
       ;.equiv CNPUD8,  0x0008
       ;.equiv CNPUD9,  0x0009
       ;.equiv CNPUD10, 0x000A
       ;.equiv CNPUD11, 0x000B


;----- CNPDD Bits -----------------------------------------------------
        .equiv IOCPDD0, 0x0000
        .equiv IOCPDD1, 0x0001
        .equiv IOCPDD2, 0x0002
        .equiv IOCPDD3, 0x0003
        .equiv IOCPDD4, 0x0004
        .equiv IOCPDD5, 0x0005
        .equiv IOCPDD6, 0x0006
        .equiv IOCPDD7, 0x0007
        .equiv IOCPDD8, 0x0008
        .equiv IOCPDD9, 0x0009
        .equiv IOCPDD10, 0x000A
        .equiv IOCPDD11, 0x000B

        .equiv CNPDD0,  0x0000
        .equiv CNPDD1,  0x0001
        .equiv CNPDD2,  0x0002
        .equiv CNPDD3,  0x0003
        .equiv CNPDD4,  0x0004
        .equiv CNPDD5,  0x0005
        .equiv CNPDD6,  0x0006
        .equiv CNPDD7,  0x0007
        .equiv CNPDD8,  0x0008
        .equiv CNPDD9,  0x0009
        .equiv CNPDD10, 0x000A
        .equiv CNPDD11, 0x000B


;----- IOCPDD Bits -----------------------------------------------------
       ;.equiv IOCPDD0, 0x0000
       ;.equiv IOCPDD1, 0x0001
       ;.equiv IOCPDD2, 0x0002
       ;.equiv IOCPDD3, 0x0003
       ;.equiv IOCPDD4, 0x0004
       ;.equiv IOCPDD5, 0x0005
       ;.equiv IOCPDD6, 0x0006
       ;.equiv IOCPDD7, 0x0007
       ;.equiv IOCPDD8, 0x0008
       ;.equiv IOCPDD9, 0x0009
       ;.equiv IOCPDD10, 0x000A
       ;.equiv IOCPDD11, 0x000B

       ;.equiv CNPDD0,  0x0000
       ;.equiv CNPDD1,  0x0001
       ;.equiv CNPDD2,  0x0002
       ;.equiv CNPDD3,  0x0003
       ;.equiv CNPDD4,  0x0004
       ;.equiv CNPDD5,  0x0005
       ;.equiv CNPDD6,  0x0006
       ;.equiv CNPDD7,  0x0007
       ;.equiv CNPDD8,  0x0008
       ;.equiv CNPDD9,  0x0009
       ;.equiv CNPDD10, 0x000A
       ;.equiv CNPDD11, 0x000B


;----- TRISE Bits -----------------------------------------------------
        .equiv TRISE0,  0x0000
        .equiv TRISE1,  0x0001
        .equiv TRISE2,  0x0002
        .equiv TRISE3,  0x0003
        .equiv TRISE4,  0x0004
        .equiv TRISE5,  0x0005
        .equiv TRISE6,  0x0006
        .equiv TRISE7,  0x0007


;----- PORTE Bits -----------------------------------------------------
        .equiv RE0,     0x0000
        .equiv RE1,     0x0001
        .equiv RE2,     0x0002
        .equiv RE3,     0x0003
        .equiv RE4,     0x0004
        .equiv RE5,     0x0005
        .equiv RE6,     0x0006
        .equiv RE7,     0x0007


;----- LATE Bits -----------------------------------------------------
        .equiv LATE0,   0x0000
        .equiv LATE1,   0x0001
        .equiv LATE2,   0x0002
        .equiv LATE3,   0x0003
        .equiv LATE4,   0x0004
        .equiv LATE5,   0x0005
        .equiv LATE6,   0x0006
        .equiv LATE7,   0x0007


;----- ODCE Bits -----------------------------------------------------
        .equiv ODCE0,   0x0000
        .equiv ODCE1,   0x0001
        .equiv ODCE2,   0x0002
        .equiv ODCE3,   0x0003
        .equiv ODCE4,   0x0004
        .equiv ODCE5,   0x0005
        .equiv ODCE6,   0x0006
        .equiv ODCE7,   0x0007


;----- ANSE Bits -----------------------------------------------------
        .equiv ANSE4,   0x0004

        .equiv ANSELE4, 0x0004


;----- ANSELE Bits -----------------------------------------------------
       ;.equiv ANSE4,   0x0004

       ;.equiv ANSELE4, 0x0004


;----- IOCPE Bits -----------------------------------------------------
        .equiv IOCPE0,  0x0000
        .equiv IOCPE1,  0x0001
        .equiv IOCPE2,  0x0002
        .equiv IOCPE3,  0x0003
        .equiv IOCPE4,  0x0004
        .equiv IOCPE5,  0x0005
        .equiv IOCPE6,  0x0006
        .equiv IOCPE7,  0x0007


;----- IOCNE Bits -----------------------------------------------------
        .equiv IOCNE0,  0x0000
        .equiv IOCNE1,  0x0001
        .equiv IOCNE2,  0x0002
        .equiv IOCNE3,  0x0003
        .equiv IOCNE4,  0x0004
        .equiv IOCNE5,  0x0005
        .equiv IOCNE6,  0x0006
        .equiv IOCNE7,  0x0007


;----- IOCFE Bits -----------------------------------------------------
        .equiv IOCFE0,  0x0000
        .equiv IOCFE1,  0x0001
        .equiv IOCFE2,  0x0002
        .equiv IOCFE3,  0x0003
        .equiv IOCFE4,  0x0004
        .equiv IOCFE5,  0x0005
        .equiv IOCFE6,  0x0006
        .equiv IOCFE7,  0x0007


;----- CNPUE Bits -----------------------------------------------------
       ;.equiv IOCPE0,  0x0000
       ;.equiv IOCPE1,  0x0001
       ;.equiv IOCPE2,  0x0002
       ;.equiv IOCPE3,  0x0003
       ;.equiv IOCPE4,  0x0004
       ;.equiv IOCPE5,  0x0005
       ;.equiv IOCPE6,  0x0006
       ;.equiv IOCPE7,  0x0007

        .equiv IOCPUE0, 0x0000
        .equiv IOCPUE1, 0x0001
        .equiv IOCPUE2, 0x0002
        .equiv IOCPUE3, 0x0003
        .equiv IOCPUE4, 0x0004
        .equiv IOCPUE5, 0x0005
        .equiv IOCPUE6, 0x0006
        .equiv IOCPUE7, 0x0007

        .equiv CNPUE0,  0x0000
        .equiv CNPUE1,  0x0001
        .equiv CNPUE2,  0x0002
        .equiv CNPUE3,  0x0003
        .equiv CNPUE4,  0x0004
        .equiv CNPUE5,  0x0005
        .equiv CNPUE6,  0x0006
        .equiv CNPUE7,  0x0007


;----- IOCPUE Bits -----------------------------------------------------
       ;.equiv IOCPE0,  0x0000
       ;.equiv IOCPE1,  0x0001
       ;.equiv IOCPE2,  0x0002
       ;.equiv IOCPE3,  0x0003
       ;.equiv IOCPE4,  0x0004
       ;.equiv IOCPE5,  0x0005
       ;.equiv IOCPE6,  0x0006
       ;.equiv IOCPE7,  0x0007

       ;.equiv IOCPUE0, 0x0000
       ;.equiv IOCPUE1, 0x0001
       ;.equiv IOCPUE2, 0x0002
       ;.equiv IOCPUE3, 0x0003
       ;.equiv IOCPUE4, 0x0004
       ;.equiv IOCPUE5, 0x0005
       ;.equiv IOCPUE6, 0x0006
       ;.equiv IOCPUE7, 0x0007

       ;.equiv CNPUE0,  0x0000
       ;.equiv CNPUE1,  0x0001
       ;.equiv CNPUE2,  0x0002
       ;.equiv CNPUE3,  0x0003
       ;.equiv CNPUE4,  0x0004
       ;.equiv CNPUE5,  0x0005
       ;.equiv CNPUE6,  0x0006
       ;.equiv CNPUE7,  0x0007


;----- CNPDE Bits -----------------------------------------------------
        .equiv IOCPDE0, 0x0000
        .equiv IOCPDE1, 0x0001
        .equiv IOCPDE2, 0x0002
        .equiv IOCPDE3, 0x0003
        .equiv IOCPDE4, 0x0004
        .equiv IOCPDE5, 0x0005
        .equiv IOCPDE6, 0x0006
        .equiv IOCPDE7, 0x0007

        .equiv CNPDE0,  0x0000
        .equiv CNPDE1,  0x0001
        .equiv CNPDE2,  0x0002
        .equiv CNPDE3,  0x0003
        .equiv CNPDE4,  0x0004
        .equiv CNPDE5,  0x0005
        .equiv CNPDE6,  0x0006
        .equiv CNPDE7,  0x0007


;----- IOCPDE Bits -----------------------------------------------------
       ;.equiv IOCPDE0, 0x0000
       ;.equiv IOCPDE1, 0x0001
       ;.equiv IOCPDE2, 0x0002
       ;.equiv IOCPDE3, 0x0003
       ;.equiv IOCPDE4, 0x0004
       ;.equiv IOCPDE5, 0x0005
       ;.equiv IOCPDE6, 0x0006
       ;.equiv IOCPDE7, 0x0007

       ;.equiv CNPDE0,  0x0000
       ;.equiv CNPDE1,  0x0001
       ;.equiv CNPDE2,  0x0002
       ;.equiv CNPDE3,  0x0003
       ;.equiv CNPDE4,  0x0004
       ;.equiv CNPDE5,  0x0005
       ;.equiv CNPDE6,  0x0006
       ;.equiv CNPDE7,  0x0007


;----- TRISF Bits -----------------------------------------------------
        .equiv TRISF0,  0x0000
        .equiv TRISF1,  0x0001
        .equiv TRISF2,  0x0002
        .equiv TRISF3,  0x0003
        .equiv TRISF4,  0x0004
        .equiv TRISF5,  0x0005
        .equiv TRISF6,  0x0006


;----- PORTF Bits -----------------------------------------------------
        .equiv RF0,     0x0000
        .equiv RF1,     0x0001
        .equiv RF2,     0x0002
        .equiv RF3,     0x0003
        .equiv RF4,     0x0004
        .equiv RF5,     0x0005
        .equiv RF6,     0x0006


;----- LATF Bits -----------------------------------------------------
        .equiv LATF0,   0x0000
        .equiv LATF1,   0x0001
        .equiv LATF2,   0x0002
        .equiv LATF3,   0x0003
        .equiv LATF4,   0x0004
        .equiv LATF5,   0x0005
        .equiv LATF6,   0x0006


;----- ODCF Bits -----------------------------------------------------
        .equiv ODCF0,   0x0000
        .equiv ODCF1,   0x0001
        .equiv ODCF2,   0x0002
        .equiv ODCF3,   0x0003
        .equiv ODCF4,   0x0004
        .equiv ODCF5,   0x0005
        .equiv ODCF6,   0x0006


;----- IOCPF Bits -----------------------------------------------------
        .equiv IOCPF0,  0x0000
        .equiv IOCPF1,  0x0001
        .equiv IOCPF2,  0x0002
        .equiv IOCPF3,  0x0003
        .equiv IOCPF4,  0x0004
        .equiv IOCPF5,  0x0005
        .equiv IOCPF6,  0x0006


;----- IOCNF Bits -----------------------------------------------------
        .equiv IOCNF0,  0x0000
        .equiv IOCNF1,  0x0001
        .equiv IOCNF2,  0x0002
        .equiv IOCNF3,  0x0003
        .equiv IOCNF4,  0x0004
        .equiv IOCNF5,  0x0005
        .equiv IOCNF6,  0x0006


;----- IOCFF Bits -----------------------------------------------------
        .equiv IOCFF0,  0x0000
        .equiv IOCFF1,  0x0001
        .equiv IOCFF2,  0x0002
        .equiv IOCFF3,  0x0003
        .equiv IOCFF4,  0x0004
        .equiv IOCFF5,  0x0005
        .equiv IOCFF6,  0x0006


;----- CNPUF Bits -----------------------------------------------------
       ;.equiv IOCPF0,  0x0000
       ;.equiv IOCPF1,  0x0001
       ;.equiv IOCPF2,  0x0002
       ;.equiv IOCPF3,  0x0003
       ;.equiv IOCPF4,  0x0004
       ;.equiv IOCPF5,  0x0005
       ;.equiv IOCPF6,  0x0006

        .equiv IOCPUF0, 0x0000
        .equiv IOCPUF1, 0x0001
        .equiv IOCPUF2, 0x0002
        .equiv IOCPUF3, 0x0003
        .equiv IOCPUF4, 0x0004
        .equiv IOCPUF5, 0x0005
        .equiv IOCPUF6, 0x0006

        .equiv CNPUF0,  0x0000
        .equiv CNPUF1,  0x0001
        .equiv CNPUF2,  0x0002
        .equiv CNPUF3,  0x0003
        .equiv CNPUF4,  0x0004
        .equiv CNPUF5,  0x0005
        .equiv CNPUF6,  0x0006


;----- IOCPUF Bits -----------------------------------------------------
       ;.equiv IOCPF0,  0x0000
       ;.equiv IOCPF1,  0x0001
       ;.equiv IOCPF2,  0x0002
       ;.equiv IOCPF3,  0x0003
       ;.equiv IOCPF4,  0x0004
       ;.equiv IOCPF5,  0x0005
       ;.equiv IOCPF6,  0x0006

       ;.equiv IOCPUF0, 0x0000
       ;.equiv IOCPUF1, 0x0001
       ;.equiv IOCPUF2, 0x0002
       ;.equiv IOCPUF3, 0x0003
       ;.equiv IOCPUF4, 0x0004
       ;.equiv IOCPUF5, 0x0005
       ;.equiv IOCPUF6, 0x0006

       ;.equiv CNPUF0,  0x0000
       ;.equiv CNPUF1,  0x0001
       ;.equiv CNPUF2,  0x0002
       ;.equiv CNPUF3,  0x0003
       ;.equiv CNPUF4,  0x0004
       ;.equiv CNPUF5,  0x0005
       ;.equiv CNPUF6,  0x0006


;----- CNPDF Bits -----------------------------------------------------
        .equiv IOCPDF0, 0x0000
        .equiv IOCPDF1, 0x0001
        .equiv IOCPDF2, 0x0002
        .equiv IOCPDF3, 0x0003
        .equiv IOCPDF4, 0x0004
        .equiv IOCPDF5, 0x0005
        .equiv IOCPDF6, 0x0006

        .equiv CNPDF0,  0x0000
        .equiv CNPDF1,  0x0001
        .equiv CNPDF2,  0x0002
        .equiv CNPDF3,  0x0003
        .equiv CNPDF4,  0x0004
        .equiv CNPDF5,  0x0005
        .equiv CNPDF6,  0x0006


;----- IOCPDF Bits -----------------------------------------------------
       ;.equiv IOCPDF0, 0x0000
       ;.equiv IOCPDF1, 0x0001
       ;.equiv IOCPDF2, 0x0002
       ;.equiv IOCPDF3, 0x0003
       ;.equiv IOCPDF4, 0x0004
       ;.equiv IOCPDF5, 0x0005
       ;.equiv IOCPDF6, 0x0006

       ;.equiv CNPDF0,  0x0000
       ;.equiv CNPDF1,  0x0001
       ;.equiv CNPDF2,  0x0002
       ;.equiv CNPDF3,  0x0003
       ;.equiv CNPDF4,  0x0004
       ;.equiv CNPDF5,  0x0005
       ;.equiv CNPDF6,  0x0006


;----- TRISG Bits -----------------------------------------------------
        .equiv TRISG2,  0x0002
        .equiv TRISG3,  0x0003
        .equiv TRISG6,  0x0006
        .equiv TRISG7,  0x0007
        .equiv TRISG8,  0x0008
        .equiv TRISG9,  0x0009


;----- PORTG Bits -----------------------------------------------------
        .equiv RG2,     0x0002
        .equiv RG3,     0x0003
        .equiv RG6,     0x0006
        .equiv RG7,     0x0007
        .equiv RG8,     0x0008
        .equiv RG9,     0x0009


;----- LATG Bits -----------------------------------------------------
        .equiv LATG2,   0x0002
        .equiv LATG3,   0x0003
        .equiv LATG6,   0x0006
        .equiv LATG7,   0x0007
        .equiv LATG8,   0x0008
        .equiv LATG9,   0x0009


;----- ODCG Bits -----------------------------------------------------
        .equiv ODCG2,   0x0002
        .equiv ODCG3,   0x0003
        .equiv ODCG6,   0x0006
        .equiv ODCG7,   0x0007
        .equiv ODCG8,   0x0008
        .equiv ODCG9,   0x0009


;----- ANSELG Bits -----------------------------------------------------
        .equiv ANSG6,   0x0006
        .equiv ANSG7,   0x0007
        .equiv ANSG8,   0x0008
        .equiv ANSG9,   0x0009

        .equiv ANSELG6, 0x0006
        .equiv ANSELG7, 0x0007
        .equiv ANSELG8, 0x0008
        .equiv ANSELG9, 0x0009


;----- ANSG Bits -----------------------------------------------------
       ;.equiv ANSG6,   0x0006
       ;.equiv ANSG7,   0x0007
       ;.equiv ANSG8,   0x0008
       ;.equiv ANSG9,   0x0009

       ;.equiv ANSELG6, 0x0006
       ;.equiv ANSELG7, 0x0007
       ;.equiv ANSELG8, 0x0008
       ;.equiv ANSELG9, 0x0009


;----- IOCPG Bits -----------------------------------------------------
        .equiv IOCPG2,  0x0002
        .equiv IOCPG3,  0x0003
        .equiv IOCPG6,  0x0006
        .equiv IOCPG7,  0x0007
        .equiv IOCPG8,  0x0008
        .equiv IOCPG9,  0x0009


;----- IOCNG Bits -----------------------------------------------------
        .equiv IOCNG2,  0x0002
        .equiv IOCNG3,  0x0003
        .equiv IOCNG6,  0x0006
        .equiv IOCNG7,  0x0007
        .equiv IOCNG8,  0x0008
        .equiv IOCNG9,  0x0009


;----- IOCFG Bits -----------------------------------------------------
        .equiv IOCFG2,  0x0002
        .equiv IOCFG3,  0x0003
        .equiv IOCFG6,  0x0006
        .equiv IOCFG7,  0x0007
        .equiv IOCFG8,  0x0008
        .equiv IOCFG9,  0x0009


;----- CNPUG Bits -----------------------------------------------------
       ;.equiv IOCPG2,  0x0002
       ;.equiv IOCPG3,  0x0003
       ;.equiv IOCPG6,  0x0006
       ;.equiv IOCPG7,  0x0007
       ;.equiv IOCPG8,  0x0008
       ;.equiv IOCPG9,  0x0009

        .equiv IOCPUG2, 0x0002
        .equiv IOCPUG3, 0x0003
        .equiv IOCPUG6, 0x0006
        .equiv IOCPUG7, 0x0007
        .equiv IOCPUG8, 0x0008
        .equiv IOCPUG9, 0x0009

        .equiv CNPUG2,  0x0002
        .equiv CNPUG3,  0x0003
        .equiv CNPUG6,  0x0006
        .equiv CNPUG7,  0x0007
        .equiv CNPUG8,  0x0008
        .equiv CNPUG9,  0x0009


;----- IOCPUG Bits -----------------------------------------------------
       ;.equiv IOCPG2,  0x0002
       ;.equiv IOCPG3,  0x0003
       ;.equiv IOCPG6,  0x0006
       ;.equiv IOCPG7,  0x0007
       ;.equiv IOCPG8,  0x0008
       ;.equiv IOCPG9,  0x0009

       ;.equiv IOCPUG2, 0x0002
       ;.equiv IOCPUG3, 0x0003
       ;.equiv IOCPUG6, 0x0006
       ;.equiv IOCPUG7, 0x0007
       ;.equiv IOCPUG8, 0x0008
       ;.equiv IOCPUG9, 0x0009

       ;.equiv CNPUG2,  0x0002
       ;.equiv CNPUG3,  0x0003
       ;.equiv CNPUG6,  0x0006
       ;.equiv CNPUG7,  0x0007
       ;.equiv CNPUG8,  0x0008
       ;.equiv CNPUG9,  0x0009


;----- CNPDG Bits -----------------------------------------------------
        .equiv IOCPDG2, 0x0002
        .equiv IOCPDG3, 0x0003
        .equiv IOCPDG6, 0x0006
        .equiv IOCPDG7, 0x0007
        .equiv IOCPDG8, 0x0008
        .equiv IOCPDG9, 0x0009

        .equiv CNPDG2,  0x0002
        .equiv CNPDG3,  0x0003
        .equiv CNPDG6,  0x0006
        .equiv CNPDG7,  0x0007
        .equiv CNPDG8,  0x0008
        .equiv CNPDG9,  0x0009


;----- IOCPDG Bits -----------------------------------------------------
       ;.equiv IOCPDG2, 0x0002
       ;.equiv IOCPDG3, 0x0003
       ;.equiv IOCPDG6, 0x0006
       ;.equiv IOCPDG7, 0x0007
       ;.equiv IOCPDG8, 0x0008
       ;.equiv IOCPDG9, 0x0009

       ;.equiv CNPDG2,  0x0002
       ;.equiv CNPDG3,  0x0003
       ;.equiv CNPDG6,  0x0006
       ;.equiv CNPDG7,  0x0007
       ;.equiv CNPDG8,  0x0008
       ;.equiv CNPDG9,  0x0009


;----- AD1CON1 Bits -----------------------------------------------------
        .equiv DONE,    0x0000
        .equiv SAMP,    0x0001
        .equiv ASAM,    0x0002
        .equiv MODE12,  0x000A
        .equiv DMAEN_AD1CON1, 0x000B
        .equiv DMABM,   0x000C
        .equiv ADSIDL,  0x000D
        .equiv ADON,    0x000F

        .equiv SSRC0,   0x0004
        .equiv SSRC1,   0x0005
        .equiv SSRC2,   0x0006
        .equiv SSRC3,   0x0007
        .equiv FORM0,   0x0008
        .equiv FORM1,   0x0009


;----- AD1CON2 Bits -----------------------------------------------------
        .equiv ALTS,    0x0000
        .equiv BUFM,    0x0001
        .equiv BUFS,    0x0007
        .equiv CSCNA,   0x000A
        .equiv BUFREGEN, 0x000B
        .equiv NVCFG0,  0x000D

        .equiv SMPI0,   0x0002
        .equiv SMPI1,   0x0003
        .equiv SMPI2,   0x0004
        .equiv SMPI3,   0x0005
        .equiv SMPI4,   0x0006
        .equiv NVCFG,   0x000D
        .equiv PCVFG0,  0x000E
        .equiv PCVFG1,  0x000F


;----- AD1CON3 Bits -----------------------------------------------------
        .equiv PUMPEN,  0x000D
        .equiv EXTSAM,  0x000E
        .equiv ADRC,    0x000F

        .equiv ADCS0,   0x0000
        .equiv ADCS1,   0x0001
        .equiv ADCS2,   0x0002
        .equiv ADCS3,   0x0003
        .equiv ADCS4,   0x0004
        .equiv ADCS5,   0x0005
        .equiv ADCS6,   0x0006
        .equiv ADCS7,   0x0007
        .equiv SAMC0,   0x0008
        .equiv SAMC1,   0x0009
        .equiv SAMC2,   0x000A
        .equiv SAMC3,   0x000B
        .equiv SAMC4,   0x000C


;----- AD1CHS Bits -----------------------------------------------------
        .equiv CH0SA0,  0x0000
        .equiv CH0SA1,  0x0001
        .equiv CH0SA2,  0x0002
        .equiv CH0SA3,  0x0003
        .equiv CH0SA4,  0x0004
        .equiv CH0NA0,  0x0005
        .equiv CH0NA1,  0x0006
        .equiv CH0NA2,  0x0007
        .equiv CH0SB0,  0x0008
        .equiv CH0SB1,  0x0009
        .equiv CH0SB2,  0x000A
        .equiv CH0SB3,  0x000B
        .equiv CH0SB4,  0x000C
        .equiv CH0NB0,  0x000D
        .equiv CH0NB1,  0x000E
        .equiv CH0NB2,  0x000F


;----- AD1CHS0 Bits -----------------------------------------------------
       ;.equiv CH0SA0,  0x0000
       ;.equiv CH0SA1,  0x0001
       ;.equiv CH0SA2,  0x0002
       ;.equiv CH0SA3,  0x0003
       ;.equiv CH0SA4,  0x0004
       ;.equiv CH0NA0,  0x0005
       ;.equiv CH0NA1,  0x0006
       ;.equiv CH0NA2,  0x0007
       ;.equiv CH0SB0,  0x0008
       ;.equiv CH0SB1,  0x0009
       ;.equiv CH0SB2,  0x000A
       ;.equiv CH0SB3,  0x000B
       ;.equiv CH0SB4,  0x000C
       ;.equiv CH0NB0,  0x000D
       ;.equiv CH0NB1,  0x000E
       ;.equiv CH0NB2,  0x000F


;----- AD1CSSH Bits -----------------------------------------------------
        .equiv CSS16,   0x0000
        .equiv CSS17,   0x0001
        .equiv CSS18,   0x0002
        .equiv CSS19,   0x0003
        .equiv CSS20,   0x0004
        .equiv CSS21,   0x0005
        .equiv CSS22,   0x0006
        .equiv CSS23,   0x0007
        .equiv CSS24,   0x0008
        .equiv CSS25,   0x0009
        .equiv CSS28,   0x000C
        .equiv CSS29,   0x000D
        .equiv CSS30,   0x000E


;----- AD1CSSL Bits -----------------------------------------------------
        .equiv CSS0,    0x0000
        .equiv CSS1,    0x0001
        .equiv CSS2,    0x0002
        .equiv CSS3,    0x0003
        .equiv CSS4,    0x0004
        .equiv CSS5,    0x0005
        .equiv CSS6,    0x0006
        .equiv CSS7,    0x0007
        .equiv CSS8,    0x0008
        .equiv CSS9,    0x0009
        .equiv CSS10,   0x000A
        .equiv CSS11,   0x000B
        .equiv CSS12,   0x000C
        .equiv CSS13,   0x000D
        .equiv CSS14,   0x000E
        .equiv CSS15,   0x000F


;----- AD1CON4 Bits -----------------------------------------------------
        .equiv DMABL0,  0x0000
        .equiv DMABL1,  0x0001
        .equiv DMABL2,  0x0002


;----- AD1CON5 Bits -----------------------------------------------------
        .equiv BGREQ,   0x000C
        .equiv CTMREQ,  0x000D
        .equiv LPEN,    0x000E
        .equiv ASEN,    0x000F

        .equiv CM0,     0x0000
        .equiv CM1,     0x0001
        .equiv WM0,     0x0002
        .equiv WM1,     0x0003
        .equiv ASINT0,  0x0008
        .equiv ASINT1,  0x0009


;----- AD1CHITH Bits -----------------------------------------------------
        .equiv CHH16,   0x0000
        .equiv CHH17,   0x0001
        .equiv CHH18,   0x0002
        .equiv CHH19,   0x0003
        .equiv CHH20,   0x0004
        .equiv CHH21,   0x0005
        .equiv CHH22,   0x0006
        .equiv CHH23,   0x0007
        .equiv CHH24,   0x0008
        .equiv CHH25,   0x0009


;----- AD1CHITL Bits -----------------------------------------------------
        .equiv CHH0,    0x0000
        .equiv CHH1,    0x0001
        .equiv CHH2,    0x0002
        .equiv CHH3,    0x0003
        .equiv CHH4,    0x0004
        .equiv CHH5,    0x0005
        .equiv CHH6,    0x0006
        .equiv CHH7,    0x0007
        .equiv CHH8,    0x0008
        .equiv CHH9,    0x0009
        .equiv CHH10,   0x000A
        .equiv CHH11,   0x000B
        .equiv CHH12,   0x000C
        .equiv CHH13,   0x000D
        .equiv CHH14,   0x000E
        .equiv CHH15,   0x000F


;----- AD1CTMENH Bits -----------------------------------------------------
        .equiv CTMEN16, 0x0000
        .equiv CTMEN17, 0x0001
        .equiv CTMEN18, 0x0002
        .equiv CTMEN19, 0x0003
        .equiv CTMEN20, 0x0004
        .equiv CTMEN21, 0x0005
        .equiv CTMEN22, 0x0006
        .equiv CTMEN23, 0x0007
        .equiv CTMEN24, 0x0008
        .equiv CTMEN25, 0x0009


;----- AD1CTMENL Bits -----------------------------------------------------
        .equiv CTMEN0,  0x0000
        .equiv CTMEN1,  0x0001
        .equiv CTMEN2,  0x0002
        .equiv CTMEN3,  0x0003
        .equiv CTMEN4,  0x0004
        .equiv CTMEN5,  0x0005
        .equiv CTMEN6,  0x0006
        .equiv CTMEN7,  0x0007
        .equiv CTMEN8,  0x0008
        .equiv CTMEN9,  0x0009
        .equiv CTMEN10, 0x000A
        .equiv CTMEN11, 0x000B
        .equiv CTMEN12, 0x000C
        .equiv CTMEN13, 0x000D
        .equiv CTMEN14, 0x000E
        .equiv CTMEN15, 0x000F


;----- NVMCON Bits -----------------------------------------------------
        .equiv P2ACTIV, 0x000A
        .equiv SFTSWP,  0x000B
        .equiv NVMSIDL, 0x000C
        .equiv WRERR,   0x000D
        .equiv WREN,    0x000E
        .equiv WR,      0x000F

        .equiv NVMOP0,  0x0000
        .equiv NVMOP1,  0x0001
        .equiv NVMOP2,  0x0002
        .equiv NVMOP3,  0x0003




;  Some useful macros for PWRSAV instructions

   .equiv SLEEP_MODE, 0x0
   .equiv IDLE_MODE, 0x1


;==========================================================================
;
;   Configuration Words
;
;   Configuration words exist in Program Space and their locations are
;   defined in the device linker script. They can be set in source code
;   or in the MPLAB IDE. Each configuration word should be specified
;   only once (multiple settings may be combined.)
;
;==========================================================================
;
;  Setting configuration words using macros:
;
;  The following macro named 'config' can be used to set configuration words:
;==========================================================================
;  NOTICE: CONFIG MACRO IS ****D E P R E C A T E D****
;  Please use #pragma in a separate C file in the project.
;  Some features in linker and compiler cannot use values
;  set using this method. Please migrate to #pragma config method.
;==========================================================================
;
        .macro config REG, VALUE
        .pushsection \REG.sec, code
        .global \REG
        .warn "DEPRECATED. Please use #pragma config in a C file"
\REG:   .pword \VALUE
        .popsection
        .endm
;
;  For example, to set CONFIG_REG using the macro above, the following line
;  can be pasted only at the beginning of the assembly source code,
;  immediately below the '.include' directive.
;
;        config __CONFIG_REG, SETTING_A & SETTING_B
;
;  Note that the 'config' macro takes two arguments: the first is the name
;  of a configuration word (in this case, __CONFIG_REG), the second argument
;  is a boolean expression that may include multiple settings.
;  The example above would enable SETTING_A and also SETTING_B.
;
;  A description of all possible settings for each configuration word
;  appears below.
;
;==========================================================================
;
;  Setting configuration words without using macros:
;
;  To set configuration words without using the 'config' macro,
;  use the following format:
;
;           .section __CONFIG_REG.sec, code
;           .global __CONFIG_REG
;__CONFIG_REG: .pword SETTING_A & SETTING_B
;
;  This source code is equivalent to the expanded macro from the previous
;  example.
;
;==========================================================================

; Fuses that are always present


;----- FBOOT (0x801800) --------------------------------------------------
;
;  The following settings are available for FBOOT:
;
;   Boot Mode Configuration bits:
;     BTMODE_DUALPROT      Device is in Dual Boot Protected mode
;     BTMODE_DUAL          Device is in Dual Boot mode
;     BTMODE_SINGLE        Device is in Single Boot (legacy) mode
;
        .equiv BTMODE_DUALPROT, 0xFFFD ; Device is in Dual Boot Protected mode
        .equiv BTMODE_DUAL,     0xFFFE ; Device is in Dual Boot mode
        .equiv BTMODE_SINGLE,   0xFFFF ; Device is in Single Boot (legacy) mode

; Fuses that are only present in single panel mode */

.ifndef __DUAL_PARTITION

;----- FSEC (0xabf00) --------------------------------------------------
;
;  The following settings are available for FSEC:
;
;   Boot Segment Write-Protect bit:
;     BWRP_ON              Boot Segment is write protected
;     BWRP_OFF             Boot Segment may be written
;
;   Boot Segment Code-Protect Level bits:
;     BSS_HI               High Security
;     BSS_STD              Standard Security
;     BSS_OFF              No Protection (other than BWRP)
;
;   Boot Segment Control bit:
;     BSEN_ON              Boot Segment size determined by FBSLIM
;     BSEN_OFF             No Boot Segment
;
;   General Segment Write-Protect bit:
;     GWRP_ON              General Segment is write protected
;     GWRP_OFF             General Segment may be written
;
;   General Segment Code-Protect Level bits:
;     GSS_HI               High Security
;     GSS_STD              Standard Security
;     GSS_OFF              No Protection (other than GWRP)
;
;   Configuration Segment Write-Protect bit:
;     CWRP_ON              Configuration Segment is write protected
;     CWRP_OFF             Configuration Segment may be written
;
;   Configuration Segment Code-Protect Level bits:
;     CSS_HI               High Security
;     CSS_ENH              Enhanced Security
;     CSS_STD              Standard Security
;     CSS_DIS              No Protection (other than CWRP)
;
;   Alternate Interrupt Vector Table bit:
;     AIVTDIS_ENABLE       Enabled AIVT
;     AIVTDIS_DISABLE      Disabled AIVT
;
        .equiv BWRP_ON,         0xFFFE ; Boot Segment is write protected
        .equiv BWRP_OFF,        0xFFFF ; Boot Segment may be written

        .equiv BSS_HI,          0xFFF9 ; High Security
        .equiv BSS_STD,         0xFFFD ; Standard Security
        .equiv BSS_OFF,         0xFFFF ; No Protection (other than BWRP)

        .equiv BSEN_ON,         0xFFF7 ; Boot Segment size determined by FBSLIM
        .equiv BSEN_OFF,        0xFFFF ; No Boot Segment

        .equiv GWRP_ON,         0xFFDF ; General Segment is write protected
        .equiv GWRP_OFF,        0xFFFF ; General Segment may be written

        .equiv GSS_HI,          0xFF3F ; High Security
        .equiv GSS_STD,         0xFFBF ; Standard Security
        .equiv GSS_OFF,         0xFFFF ; No Protection (other than GWRP)

        .equiv CWRP_ON,         0xFEFF ; Configuration Segment is write protected
        .equiv CWRP_OFF,        0xFFFF ; Configuration Segment may be written

        .equiv CSS_HI,          0xF1FF ; High Security
        .equiv CSS_ENH,         0xF9FF ; Enhanced Security
        .equiv CSS_STD,         0xFDFF ; Standard Security
        .equiv CSS_DIS,         0xFFFF ; No Protection (other than CWRP)

        .equiv AIVTDIS_ENABLE,  0x7FFF ; Enabled AIVT
        .equiv AIVTDIS_DISABLE, 0xFFFF ; Disabled AIVT


;----- FBSLIM (0xabf10) --------------------------------------------------
;
;  The following settings are available for FBSLIM:
;
;   Boot Segment Flash Page Address Limit bits:
;     BSLIM_BSLIM          Boot Segment Flash page address  limit
;
        .equiv BSLIM_BSLIM,     0xE000 ; Boot Segment Flash page address  limit


;----- FSIGN (0xabf14) --------------------------------------------------
;
;  The following settings are available for FSIGN:
;

;----- FOSCSEL (0xabf18) --------------------------------------------------
;
;  The following settings are available for FOSCSEL:
;
;   Oscillator Source Selection:
;     FNOSC_FRC            Internal Fast RC (FRC)
;     FNOSC_FRCPLL         Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
;     FNOSC_PRI            Primary Oscillator (XT, HS, EC)
;     FNOSC_PRIPLL         Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
;     FNOSC_SOSC           Secondary Oscillator (SOSC)
;     FNOSC_LPRC           Low-Power RC Oscillator (LPRC)
;     FNOSC_DCO            Digitally-Controlled Oscillator
;     FNOSC_OSCFDIV        Oscillator with Frequency Divider
;
;   PLL Mode Selection:
;     PLLMODE_PLL96DIV1    96 MHz PLL.  (4 MHz input)
;     PLLMODE_PLL96DIV2    96 MHz PLL. (8 MHz input)
;     PLLMODE_PLL96DIV3    96 MHz PLL. (12 MHz input)
;     PLLMODE_PLL96DIV4    96 MHz PLL.  (16 MHz input)
;     PLLMODE_PLL96DIV5    96 MHz PLL. (32 MHz input)
;     PLLMODE_PLL96DIV6    96 MHz PLL.  (24 MHz input)
;     PLLMODE_PLL96DIV8    96 MHz PLL.  (32 MHz input)
;     PLLMODE_PLL96DIV12   96 MHz PLL.  (48 MHz input)
;     PLLMODE_PLL4X        4x PLL selected
;     PLLMODE_PLL6X        6x PLL selected
;     PLLMODE_PLL8X        8x PLL selected
;     PLLMODE_DISABLED     No PLL used; PLLEN bit is not available
;
;   Two-speed Oscillator Start-up Enable bit:
;     IESO_OFF             Start up with user-selected oscillator source
;     IESO_ON              Start up device with FRC, then switch to user-selected oscillator source
;
        .equiv FNOSC_FRC,       0xFFF8 ; Internal Fast RC (FRC)
        .equiv FNOSC_FRCPLL,    0xFFF9 ; Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
        .equiv FNOSC_PRI,       0xFFFA ; Primary Oscillator (XT, HS, EC)
        .equiv FNOSC_PRIPLL,    0xFFFB ; Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
        .equiv FNOSC_SOSC,      0xFFFC ; Secondary Oscillator (SOSC)
        .equiv FNOSC_LPRC,      0xFFFD ; Low-Power RC Oscillator (LPRC)
        .equiv FNOSC_DCO,       0xFFFE ; Digitally-Controlled Oscillator
        .equiv FNOSC_OSCFDIV,   0xFFFF ; Oscillator with Frequency Divider

        .equiv PLLMODE_PLL96DIV1, 0xFF87 ; 96 MHz PLL.  (4 MHz input)
        .equiv PLLMODE_PLL96DIV2, 0xFF8F ; 96 MHz PLL. (8 MHz input)
        .equiv PLLMODE_PLL96DIV3, 0xFF97 ; 96 MHz PLL. (12 MHz input)
        .equiv PLLMODE_PLL96DIV4, 0xFF9F ; 96 MHz PLL.  (16 MHz input)
        .equiv PLLMODE_PLL96DIV5, 0xFFA7 ; 96 MHz PLL. (32 MHz input)
        .equiv PLLMODE_PLL96DIV6, 0xFFAF ; 96 MHz PLL.  (24 MHz input)
        .equiv PLLMODE_PLL96DIV8, 0xFFB7 ; 96 MHz PLL.  (32 MHz input)
        .equiv PLLMODE_PLL96DIV12, 0xFFBF ; 96 MHz PLL.  (48 MHz input)
        .equiv PLLMODE_PLL4X,   0xFFE7 ; 4x PLL selected
        .equiv PLLMODE_PLL6X,   0xFFEF ; 6x PLL selected
        .equiv PLLMODE_PLL8X,   0xFFF7 ; 8x PLL selected
        .equiv PLLMODE_DISABLED, 0xFFFF ; No PLL used; PLLEN bit is not available

        .equiv IESO_OFF,        0xFF7F ; Start up with user-selected oscillator source
        .equiv IESO_ON,         0xFFFF ; Start up device with FRC, then switch to user-selected oscillator source


;----- FOSC (0xabf1c) --------------------------------------------------
;
;  The following settings are available for FOSC:
;
;   Primary Oscillator Mode Select bits:
;     POSCMD_EC            EC (External Clock) Mode
;     POSCMD_XT            XT Crystal Oscillator Mode
;     POSCMD_HS            HS Crystal Oscillator Mode
;     POSCMD_NONE          Primary Oscillator disabled
;
;   OSC2 Pin Function bit:
;     OSCIOFCN_ON          OSC2 is general purpose digital I/O pin
;     OSCIOFCN_OFF         OSC2 is clock output
;
;   SOSC Power Selection Configuration bits:
;     SOSCSEL_OFF          Digital (SCLKI) mode
;     SOSCSEL_ON           SOSC is used in crystal (SOSCI/SOSCO) mode
;
;   PLL Secondary Selection Configuration bit:
;     PLLSS_PLL_FRC        PLL is fed by the on-chip Fast RC (FRC) oscillator
;     PLLSS_PLL_PRI        PLL is fed by the Primary oscillator
;
;   Peripheral pin select configuration bit:
;     IOL1WAY_OFF          Allow multiple reconfigurations
;     IOL1WAY_ON           Allow only one reconfiguration
;
;   Clock Switching Mode bits:
;     FCKSM_CSECME         Both Clock switching and Fail-safe Clock Monitor are enabled
;     FCKSM_CSECMD         Clock switching is enabled,Fail-safe Clock Monitor is disabled
;     FCKSM_CSDCMD         Both Clock switching and Fail-safe Clock Monitor are disabled
;
        .equiv POSCMD_EC,       0xFFFC ; EC (External Clock) Mode
        .equiv POSCMD_XT,       0xFFFD ; XT Crystal Oscillator Mode
        .equiv POSCMD_HS,       0xFFFE ; HS Crystal Oscillator Mode
        .equiv POSCMD_NONE,     0xFFFF ; Primary Oscillator disabled

        .equiv OSCIOFCN_ON,     0xFFFB ; OSC2 is general purpose digital I/O pin
        .equiv OSCIOFCN_OFF,    0xFFFF ; OSC2 is clock output

        .equiv SOSCSEL_OFF,     0xFFF7 ; Digital (SCLKI) mode
        .equiv SOSCSEL_ON,      0xFFFF ; SOSC is used in crystal (SOSCI/SOSCO) mode

        .equiv PLLSS_PLL_FRC,   0xFFEF ; PLL is fed by the on-chip Fast RC (FRC) oscillator
        .equiv PLLSS_PLL_PRI,   0xFFFF ; PLL is fed by the Primary oscillator

        .equiv IOL1WAY_OFF,     0xFFDF ; Allow multiple reconfigurations
        .equiv IOL1WAY_ON,      0xFFFF ; Allow only one reconfiguration

        .equiv FCKSM_CSECME,    0xFF3F ; Both Clock switching and Fail-safe Clock Monitor are enabled
        .equiv FCKSM_CSECMD,    0xFF7F ; Clock switching is enabled,Fail-safe Clock Monitor is disabled
        .equiv FCKSM_CSDCMD,    0xFFBF ; Both Clock switching and Fail-safe Clock Monitor are disabled


;----- FWDT (0xabf20) --------------------------------------------------
;
;  The following settings are available for FWDT:
;
;   Watchdog Timer Postscaler bits:
;     WDTPS_PS1            1:1
;     WDTPS_PS2            1:2
;     WDTPS_PS4            1:4
;     WDTPS_PS8            1:8
;     WDTPS_PS16           1:16
;     WDTPS_PS32           1:32
;     WDTPS_PS64           1:64
;     WDTPS_PS128          1:128
;     WDTPS_PS256          1:256
;     WDTPS_PS512          1:512
;     WDTPS_PS1024         1:1,024
;     WDTPS_PS2048         1:2,048
;     WDTPS_PS4096         1:4,096
;     WDTPS_PS8192         1:8,192
;     WDTPS_PS16384        1:16,384
;     WDTPS_PS32768        1:32,768
;
;   Watchdog Timer Prescaler bit:
;     FWPSA_PR32           1:32
;     FWPSA_PR128          1:128
;
;   Watchdog Timer Enable bits:
;     FWDTEN_OFF           WDT and SWDTEN disabled
;     NOSLP                WDT Enabled only while device is active (disabled in sleep, SWDTEN disabled)
;     SWON                 WDT Enabled/Disabled (controlled using SWDTEN bit)
;     FWDTEN_ON            WDT Enabled
;
;   Watchdog Timer Window Enable bit:
;     WINDIS_ON            Watchdog Timer in Window mode
;     WINDIS_OFF           Watchdog Timer in Non-Window mode
;
;   Watchdog Timer Window Select bits:
;     PS75_0               WDT Windo is 75% of WDT period
;     PS50_0               WDT Window is 50% of WDT period
;     PS37_5               WDT Window is 37.5% of WDT period
;     PS25_0               WDT Window is 25% of WDT period
;
;   WDT MUX Source Select bits:
;     WDTCMX_LPRC          WDT always uses LPRC as its clock source
;     WDTCMX_WDTCLK        WDT clock source is determined by the WDTCLK Configuration bits
;
;   WDT Clock Source Select bits:
;     WDTCLK_SYSCLK        WDT uses system clock when active, LPRC while in Sleep mode
;     WDTCLK_SOSC          WDT uses SOSC inpu
;     WDTCLK_FRC           WDT uses 31 kHz source from FRC when active in Windowed WDT
;     WDTCLK_LPRC          WDT uses LPRC
;
        .equiv WDTPS_PS1,       0xFFF0 ; 1:1
        .equiv WDTPS_PS2,       0xFFF1 ; 1:2
        .equiv WDTPS_PS4,       0xFFF2 ; 1:4
        .equiv WDTPS_PS8,       0xFFF3 ; 1:8
        .equiv WDTPS_PS16,      0xFFF4 ; 1:16
        .equiv WDTPS_PS32,      0xFFF5 ; 1:32
        .equiv WDTPS_PS64,      0xFFF6 ; 1:64
        .equiv WDTPS_PS128,     0xFFF7 ; 1:128
        .equiv WDTPS_PS256,     0xFFF8 ; 1:256
        .equiv WDTPS_PS512,     0xFFF9 ; 1:512
        .equiv WDTPS_PS1024,    0xFFFA ; 1:1,024
        .equiv WDTPS_PS2048,    0xFFFB ; 1:2,048
        .equiv WDTPS_PS4096,    0xFFFC ; 1:4,096
        .equiv WDTPS_PS8192,    0xFFFD ; 1:8,192
        .equiv WDTPS_PS16384,   0xFFFE ; 1:16,384
        .equiv WDTPS_PS32768,   0xFFFF ; 1:32,768

        .equiv FWPSA_PR32,      0xFFEF ; 1:32
        .equiv FWPSA_PR128,     0xFFFF ; 1:128

        .equiv FWDTEN_OFF,      0xFF9F ; WDT and SWDTEN disabled
        .equiv NOSLP,           0xFFBF ; WDT Enabled only while device is active (disabled in sleep, SWDTEN disabled)
        .equiv SWON,            0xFFDF ; WDT Enabled/Disabled (controlled using SWDTEN bit)
        .equiv FWDTEN_ON,       0xFFFF ; WDT Enabled

        .equiv WINDIS_ON,       0xFF7F ; Watchdog Timer in Window mode
        .equiv WINDIS_OFF,      0xFFFF ; Watchdog Timer in Non-Window mode

        .equiv PS75_0,          0xFCFF ; WDT Windo is 75% of WDT period
        .equiv PS50_0,          0xFDFF ; WDT Window is 50% of WDT period
        .equiv PS37_5,          0xFEFF ; WDT Window is 37.5% of WDT period
        .equiv PS25_0,          0xFFFF ; WDT Window is 25% of WDT period

        .equiv WDTCMX_LPRC,     0xF7FF ; WDT always uses LPRC as its clock source
        .equiv WDTCMX_WDTCLK,   0xFFFF ; WDT clock source is determined by the WDTCLK Configuration bits

        .equiv WDTCLK_SYSCLK,   0x9FFF ; WDT uses system clock when active, LPRC while in Sleep mode
        .equiv WDTCLK_SOSC,     0xBFFF ; WDT uses SOSC inpu
        .equiv WDTCLK_FRC,      0xDFFF ; WDT uses 31 kHz source from FRC when active in Windowed WDT
        .equiv WDTCLK_LPRC,     0xFFFF ; WDT uses LPRC


;----- FPOR (0xabf24) --------------------------------------------------
;
;  The following settings are available for FPOR:
;
;   Brown Out Enable bit:
;     BOREN_OFF            Brown Out Disabled
;     SWC                  Controlled by SBOREN
;     BOREN_NOSLP          Disabled for Sleep Modes
;     BOREN_ON             Brown Out Enable Bit
;
;   Low power regulator control:
;     LPCFG_ON             Retention Sleep controlled by RETEN
;     LPCFG_OFF            No Retention Sleep
;
;   Downside Voltage Protection Enable bit:
;     DNVPEN_DISABLE       Downside protection disabled when BOR is inactive
;     DNVPEN_ENABLE        Downside protection enabled using ZPBOR when BOR is inactive
;
        .equiv BOREN_OFF,       0xFFFC ; Brown Out Disabled
        .equiv SWC,             0xFFFD ; Controlled by SBOREN
        .equiv BOREN_NOSLP,     0xFFFE ; Disabled for Sleep Modes
        .equiv BOREN_ON,        0xFFFF ; Brown Out Enable Bit

        .equiv LPCFG_ON,        0xFFFB ; Retention Sleep controlled by RETEN
        .equiv LPCFG_OFF,       0xFFFF ; No Retention Sleep

        .equiv DNVPEN_DISABLE,  0xFFF7 ; Downside protection disabled when BOR is inactive
        .equiv DNVPEN_ENABLE,   0xFFFF ; Downside protection enabled using ZPBOR when BOR is inactive


;----- FICD (0xabf28) --------------------------------------------------
;
;  The following settings are available for FICD:
;
;   ICD Communication Channel Select bits:
;     ICS_NONE             Reserved, do not use
;     PGX3                 Communicate on PGEC3 and PGED3
;     PGX2                 Communicate on PGEC2 and PGED2
;     PGX1                 Communicate on PGEC1 and PGED1
;     ICS_PGX1             Communicate on PGEC1 and PGED1
;
;   JTAG Enable bit:
;     JTAGEN_OFF           JTAG is disabled
;     JTAGEN_ON            JTAG is enabled
;
;   BOOTSWP Disable:
;     BTSWP_ON             BOOTSWP instruction is enabled
;     BTSWP_OFF            BOOTSWP instruction disabled
;
        .equiv ICS_NONE,        0xFFFC ; Reserved, do not use
        .equiv PGX3,            0xFFFD ; Communicate on PGEC3 and PGED3
        .equiv PGX2,            0xFFFE ; Communicate on PGEC2 and PGED2
        .equiv PGX1,            0xFFFF ; Communicate on PGEC1 and PGED1
        .equiv ICS_PGX1,        0xFFFF ; Communicate on PGEC1 and PGED1

        .equiv JTAGEN_OFF,      0xFFDF ; JTAG is disabled
        .equiv JTAGEN_ON,       0xFFFF ; JTAG is enabled

        .equiv BTSWP_ON,        0x7FFF ; BOOTSWP instruction is enabled
        .equiv BTSWP_OFF,       0xFFFF ; BOOTSWP instruction disabled


;----- FDEVOPT1 (0xabf2c) --------------------------------------------------
;
;  The following settings are available for FDEVOPT1:
;
;   Alternate Comparator Input Enable bit:
;     ALTCMPI_ENABLE       C1INC, C2INC and C3INC are on RG9
;     ALTCMPI_DISABLE      C1INC, C2INC, and C3INC are on their standard pin locations
;
;   Tamper Pin Enable bit:
;     TMPRPIN_ON           TMPRN pin function is enabled
;     TMPRPIN_OFF          TMPRN pin function is disabled
;
;   SOSC High Power Enable bit (valid only when SOSCSEL = 1:
;     SOSCHP_OFF           Enable SOSC low power mode
;     SOSCHP_ON            Enable SOSC high power mode (default)
;
;   Alternate Voltage Reference Location Enable bit:
;     ALTVREF_ALTVREFDIS   VREF+ and CVREF+ on RB0, VREF- and CVREF- on RB1
;     ALTVREF_ALTREFEN     VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9
;
        .equiv ALTCMPI_ENABLE,  0xFFFD ; C1INC, C2INC and C3INC are on RG9
        .equiv ALTCMPI_DISABLE, 0xFFFF ; C1INC, C2INC, and C3INC are on their standard pin locations

        .equiv TMPRPIN_ON,      0xFFFB ; TMPRN pin function is enabled
        .equiv TMPRPIN_OFF,     0xFFFF ; TMPRN pin function is disabled

        .equiv SOSCHP_OFF,      0xFFF7 ; Enable SOSC low power mode
        .equiv SOSCHP_ON,       0xFFFF ; Enable SOSC high power mode (default)

        .equiv ALTVREF_ALTVREFDIS, 0xFFEF ; VREF+ and CVREF+ on RB0, VREF- and CVREF- on RB1
        .equiv ALTVREF_ALTREFEN, 0xFFFF ; VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9

.endif

/* Fuses that are only present in dual panel mode */

.ifdef __DUAL_PARTITION

;----- FSEC (0x55f00) --------------------------------------------------
;
;  The following settings are available for FSEC:
;
;   Boot Segment Write-Protect bit:
;     BWRP_ON              Boot Segment is write protected
;     BWRP_OFF             Boot Segment may be written
;
;   Boot Segment Code-Protect Level bits:
;     BSS_HI               High Security
;     BSS_STD              Standard Security
;     BSS_OFF              No Protection (other than BWRP)
;
;   Boot Segment Control bit:
;     BSEN_ON              Boot Segment size determined by FBSLIM
;     BSEN_OFF             No Boot Segment
;
;   General Segment Write-Protect bit:
;     GWRP_ON              General Segment is write protected
;     GWRP_OFF             General Segment may be written
;
;   General Segment Code-Protect Level bits:
;     GSS_HI               High Security
;     GSS_STD              Standard Security
;     GSS_OFF              No Protection (other than GWRP)
;
;   Configuration Segment Write-Protect bit:
;     CWRP_ON              Configuration Segment is write protected
;     CWRP_OFF             Configuration Segment may be written
;
;   Configuration Segment Code-Protect Level bits:
;     CSS_HI               High Security
;     CSS_ENH              Enhanced Security
;     CSS_STD              Standard Security
;     CSS_DIS              No Protection (other than CWRP)
;
;   Alternate Interrupt Vector Table bit:
;     AIVTDIS_ENABLE       Enabled AIVT
;     AIVTDIS_DISABLE      Disabled AIVT
;
        .equiv BWRP_ON,         0xFFFE ; Boot Segment is write protected
        .equiv BWRP_OFF,        0xFFFF ; Boot Segment may be written

        .equiv BSS_HI,          0xFFF9 ; High Security
        .equiv BSS_STD,         0xFFFD ; Standard Security
        .equiv BSS_OFF,         0xFFFF ; No Protection (other than BWRP)

        .equiv BSEN_ON,         0xFFF7 ; Boot Segment size determined by FBSLIM
        .equiv BSEN_OFF,        0xFFFF ; No Boot Segment

        .equiv GWRP_ON,         0xFFDF ; General Segment is write protected
        .equiv GWRP_OFF,        0xFFFF ; General Segment may be written

        .equiv GSS_HI,          0xFF3F ; High Security
        .equiv GSS_STD,         0xFFBF ; Standard Security
        .equiv GSS_OFF,         0xFFFF ; No Protection (other than GWRP)

        .equiv CWRP_ON,         0xFEFF ; Configuration Segment is write protected
        .equiv CWRP_OFF,        0xFFFF ; Configuration Segment may be written

        .equiv CSS_HI,          0xF1FF ; High Security
        .equiv CSS_ENH,         0xF9FF ; Enhanced Security
        .equiv CSS_STD,         0xFDFF ; Standard Security
        .equiv CSS_DIS,         0xFFFF ; No Protection (other than CWRP)

        .equiv AIVTDIS_ENABLE,  0x7FFF ; Enabled AIVT
        .equiv AIVTDIS_DISABLE, 0xFFFF ; Disabled AIVT


;----- FBSLIM (0x55f10) --------------------------------------------------
;
;  The following settings are available for FBSLIM:
;
;   Boot Segment Flash Page Address Limit bits:
;     BSLIM_BSLIM          Boot Segment Flash page address  limit
;
        .equiv BSLIM_BSLIM,     0xE000 ; Boot Segment Flash page address  limit


;----- FSIGN (0x55f14) --------------------------------------------------
;
;  The following settings are available for FSIGN:
;

;----- FOSCSEL (0x55f18) --------------------------------------------------
;
;  The following settings are available for FOSCSEL:
;
;   Oscillator Source Selection:
;     FNOSC_FRC            Internal Fast RC (FRC)
;     FNOSC_FRCPLL         Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
;     FNOSC_PRI            Primary Oscillator (XT, HS, EC)
;     FNOSC_PRIPLL         Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
;     FNOSC_SOSC           Secondary Oscillator (SOSC)
;     FNOSC_LPRC           Low-Power RC Oscillator (LPRC)
;     FNOSC_DCO            Digitally-Controlled Oscillator
;     FNOSC_OSCFDIV        Oscillator with Frequency Divider
;
;   PLL Mode Selection:
;     PLLMODE_PLL96DIV1    96 MHz PLL.  (4 MHz input)
;     PLLMODE_PLL96DIV2    96 MHz PLL. (8 MHz input)
;     PLLMODE_PLL96DIV3    96 MHz PLL. (12 MHz input)
;     PLLMODE_PLL96DIV4    96 MHz PLL.  (16 MHz input)
;     PLLMODE_PLL96DIV5    96 MHz PLL. (32 MHz input)
;     PLLMODE_PLL96DIV6    96 MHz PLL.  (24 MHz input)
;     PLLMODE_PLL96DIV8    96 MHz PLL.  (32 MHz input)
;     PLLMODE_PLL96DIV12   96 MHz PLL.  (48 MHz input)
;     PLLMODE_PLL4X        4x PLL selected
;     PLLMODE_PLL6X        6x PLL selected
;     PLLMODE_PLL8X        8x PLL selected
;     PLLMODE_DISABLED     No PLL used; PLLEN bit is not available
;
;   Two-speed Oscillator Start-up Enable bit:
;     IESO_OFF             Start up with user-selected oscillator source
;     IESO_ON              Start up device with FRC, then switch to user-selected oscillator source
;
        .equiv FNOSC_FRC,       0xFFF8 ; Internal Fast RC (FRC)
        .equiv FNOSC_FRCPLL,    0xFFF9 ; Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) 
        .equiv FNOSC_PRI,       0xFFFA ; Primary Oscillator (XT, HS, EC)
        .equiv FNOSC_PRIPLL,    0xFFFB ; Primary Oscillator with PLL module (XT + PLL, HS + PLL, EC + PLL)
        .equiv FNOSC_SOSC,      0xFFFC ; Secondary Oscillator (SOSC)
        .equiv FNOSC_LPRC,      0xFFFD ; Low-Power RC Oscillator (LPRC)
        .equiv FNOSC_DCO,       0xFFFE ; Digitally-Controlled Oscillator
        .equiv FNOSC_OSCFDIV,   0xFFFF ; Oscillator with Frequency Divider

        .equiv PLLMODE_PLL96DIV1, 0xFF87 ; 96 MHz PLL.  (4 MHz input)
        .equiv PLLMODE_PLL96DIV2, 0xFF8F ; 96 MHz PLL. (8 MHz input)
        .equiv PLLMODE_PLL96DIV3, 0xFF97 ; 96 MHz PLL. (12 MHz input)
        .equiv PLLMODE_PLL96DIV4, 0xFF9F ; 96 MHz PLL.  (16 MHz input)
        .equiv PLLMODE_PLL96DIV5, 0xFFA7 ; 96 MHz PLL. (32 MHz input)
        .equiv PLLMODE_PLL96DIV6, 0xFFAF ; 96 MHz PLL.  (24 MHz input)
        .equiv PLLMODE_PLL96DIV8, 0xFFB7 ; 96 MHz PLL.  (32 MHz input)
        .equiv PLLMODE_PLL96DIV12, 0xFFBF ; 96 MHz PLL.  (48 MHz input)
        .equiv PLLMODE_PLL4X,   0xFFE7 ; 4x PLL selected
        .equiv PLLMODE_PLL6X,   0xFFEF ; 6x PLL selected
        .equiv PLLMODE_PLL8X,   0xFFF7 ; 8x PLL selected
        .equiv PLLMODE_DISABLED, 0xFFFF ; No PLL used; PLLEN bit is not available

        .equiv IESO_OFF,        0xFF7F ; Start up with user-selected oscillator source
        .equiv IESO_ON,         0xFFFF ; Start up device with FRC, then switch to user-selected oscillator source


;----- FOSC (0x55f1c) --------------------------------------------------
;
;  The following settings are available for FOSC:
;
;   Primary Oscillator Mode Select bits:
;     POSCMD_EC            EC (External Clock) Mode
;     POSCMD_XT            XT Crystal Oscillator Mode
;     POSCMD_HS            HS Crystal Oscillator Mode
;     POSCMD_NONE          Primary Oscillator disabled
;
;   OSC2 Pin Function bit:
;     OSCIOFCN_ON          OSC2 is general purpose digital I/O pin
;     OSCIOFCN_OFF         OSC2 is clock output
;
;   SOSC Power Selection Configuration bits:
;     SOSCSEL_OFF          Digital (SCLKI) mode
;     SOSCSEL_ON           SOSC is used in crystal (SOSCI/SOSCO) mode
;
;   PLL Secondary Selection Configuration bit:
;     PLLSS_PLL_FRC        PLL is fed by the on-chip Fast RC (FRC) oscillator
;     PLLSS_PLL_PRI        PLL is fed by the Primary oscillator
;
;   Peripheral pin select configuration bit:
;     IOL1WAY_OFF          Allow multiple reconfigurations
;     IOL1WAY_ON           Allow only one reconfiguration
;
;   Clock Switching Mode bits:
;     FCKSM_CSECME         Both Clock switching and Fail-safe Clock Monitor are enabled
;     FCKSM_CSECMD         Clock switching is enabled,Fail-safe Clock Monitor is disabled
;     FCKSM_CSDCMD         Both Clock switching and Fail-safe Clock Monitor are disabled
;
        .equiv POSCMD_EC,       0xFFFC ; EC (External Clock) Mode
        .equiv POSCMD_XT,       0xFFFD ; XT Crystal Oscillator Mode
        .equiv POSCMD_HS,       0xFFFE ; HS Crystal Oscillator Mode
        .equiv POSCMD_NONE,     0xFFFF ; Primary Oscillator disabled

        .equiv OSCIOFCN_ON,     0xFFFB ; OSC2 is general purpose digital I/O pin
        .equiv OSCIOFCN_OFF,    0xFFFF ; OSC2 is clock output

        .equiv SOSCSEL_OFF,     0xFFF7 ; Digital (SCLKI) mode
        .equiv SOSCSEL_ON,      0xFFFF ; SOSC is used in crystal (SOSCI/SOSCO) mode

        .equiv PLLSS_PLL_FRC,   0xFFEF ; PLL is fed by the on-chip Fast RC (FRC) oscillator
        .equiv PLLSS_PLL_PRI,   0xFFFF ; PLL is fed by the Primary oscillator

        .equiv IOL1WAY_OFF,     0xFFDF ; Allow multiple reconfigurations
        .equiv IOL1WAY_ON,      0xFFFF ; Allow only one reconfiguration

        .equiv FCKSM_CSECME,    0xFF3F ; Both Clock switching and Fail-safe Clock Monitor are enabled
        .equiv FCKSM_CSECMD,    0xFF7F ; Clock switching is enabled,Fail-safe Clock Monitor is disabled
        .equiv FCKSM_CSDCMD,    0xFFBF ; Both Clock switching and Fail-safe Clock Monitor are disabled


;----- FWDT (0x55f20) --------------------------------------------------
;
;  The following settings are available for FWDT:
;
;   Watchdog Timer Postscaler bits:
;     WDTPS_PS1            1:1
;     WDTPS_PS2            1:2
;     WDTPS_PS4            1:4
;     WDTPS_PS8            1:8
;     WDTPS_PS16           1:16
;     WDTPS_PS32           1:32
;     WDTPS_PS64           1:64
;     WDTPS_PS128          1:128
;     WDTPS_PS256          1:256
;     WDTPS_PS512          1:512
;     WDTPS_PS1024         1:1,024
;     WDTPS_PS2048         1:2,048
;     WDTPS_PS4096         1:4,096
;     WDTPS_PS8192         1:8,192
;     WDTPS_PS16384        1:16,384
;     WDTPS_PS32768        1:32,768
;
;   Watchdog Timer Prescaler bit:
;     FWPSA_PR32           1:32
;     FWPSA_PR128          1:128
;
;   Watchdog Timer Enable bits:
;     FWDTEN_OFF           WDT and SWDTEN disabled
;     NOSLP                WDT Enabled only while device is active (disabled in sleep, SWDTEN disabled)
;     SWON                 WDT Enabled/Disabled (controlled using SWDTEN bit)
;     FWDTEN_ON            WDT Enabled
;
;   Watchdog Timer Window Enable bit:
;     WINDIS_ON            Watchdog Timer in Window mode
;     WINDIS_OFF           Watchdog Timer in Non-Window mode
;
;   Watchdog Timer Window Select bits:
;     PS75_0               WDT Windo is 75% of WDT period
;     PS50_0               WDT Window is 50% of WDT period
;     PS37_5               WDT Window is 37.5% of WDT period
;     PS25_0               WDT Window is 25% of WDT period
;
;   WDT MUX Source Select bits:
;     WDTCMX_LPRC          WDT always uses LPRC as its clock source
;     WDTCMX_WDTCLK        WDT clock source is determined by the WDTCLK Configuration bits
;
;   WDT Clock Source Select bits:
;     WDTCLK_SYSCLK        WDT uses system clock when active, LPRC while in Sleep mode
;     WDTCLK_SOSC          WDT uses SOSC inpu
;     WDTCLK_FRC           WDT uses 31 kHz source from FRC when active in Windowed WDT
;     WDTCLK_LPRC          WDT uses LPRC
;
        .equiv WDTPS_PS1,       0xFFF0 ; 1:1
        .equiv WDTPS_PS2,       0xFFF1 ; 1:2
        .equiv WDTPS_PS4,       0xFFF2 ; 1:4
        .equiv WDTPS_PS8,       0xFFF3 ; 1:8
        .equiv WDTPS_PS16,      0xFFF4 ; 1:16
        .equiv WDTPS_PS32,      0xFFF5 ; 1:32
        .equiv WDTPS_PS64,      0xFFF6 ; 1:64
        .equiv WDTPS_PS128,     0xFFF7 ; 1:128
        .equiv WDTPS_PS256,     0xFFF8 ; 1:256
        .equiv WDTPS_PS512,     0xFFF9 ; 1:512
        .equiv WDTPS_PS1024,    0xFFFA ; 1:1,024
        .equiv WDTPS_PS2048,    0xFFFB ; 1:2,048
        .equiv WDTPS_PS4096,    0xFFFC ; 1:4,096
        .equiv WDTPS_PS8192,    0xFFFD ; 1:8,192
        .equiv WDTPS_PS16384,   0xFFFE ; 1:16,384
        .equiv WDTPS_PS32768,   0xFFFF ; 1:32,768

        .equiv FWPSA_PR32,      0xFFEF ; 1:32
        .equiv FWPSA_PR128,     0xFFFF ; 1:128

        .equiv FWDTEN_OFF,      0xFF9F ; WDT and SWDTEN disabled
        .equiv NOSLP,           0xFFBF ; WDT Enabled only while device is active (disabled in sleep, SWDTEN disabled)
        .equiv SWON,            0xFFDF ; WDT Enabled/Disabled (controlled using SWDTEN bit)
        .equiv FWDTEN_ON,       0xFFFF ; WDT Enabled

        .equiv WINDIS_ON,       0xFF7F ; Watchdog Timer in Window mode
        .equiv WINDIS_OFF,      0xFFFF ; Watchdog Timer in Non-Window mode

        .equiv PS75_0,          0xFCFF ; WDT Windo is 75% of WDT period
        .equiv PS50_0,          0xFDFF ; WDT Window is 50% of WDT period
        .equiv PS37_5,          0xFEFF ; WDT Window is 37.5% of WDT period
        .equiv PS25_0,          0xFFFF ; WDT Window is 25% of WDT period

        .equiv WDTCMX_LPRC,     0xF7FF ; WDT always uses LPRC as its clock source
        .equiv WDTCMX_WDTCLK,   0xFFFF ; WDT clock source is determined by the WDTCLK Configuration bits

        .equiv WDTCLK_SYSCLK,   0x9FFF ; WDT uses system clock when active, LPRC while in Sleep mode
        .equiv WDTCLK_SOSC,     0xBFFF ; WDT uses SOSC inpu
        .equiv WDTCLK_FRC,      0xDFFF ; WDT uses 31 kHz source from FRC when active in Windowed WDT
        .equiv WDTCLK_LPRC,     0xFFFF ; WDT uses LPRC


;----- FPOR (0x55f24) --------------------------------------------------
;
;  The following settings are available for FPOR:
;
;   Brown Out Enable bit:
;     BOREN_OFF            Brown Out Disabled
;     SWC                  Controlled by SBOREN
;     BOREN_NOSLP          Disabled for Sleep Modes
;     BOREN_ON             Brown Out Enable Bit
;
;   Low power regulator control:
;     LPCFG_ON             Retention Sleep controlled by RETEN
;     LPCFG_OFF            No Retention Sleep
;
;   Downside Voltage Protection Enable bit:
;     DNVPEN_DISABLE       Downside protection disabled when BOR is inactive
;     DNVPEN_ENABLE        Downside protection enabled using ZPBOR when BOR is inactive
;
        .equiv BOREN_OFF,       0xFFFC ; Brown Out Disabled
        .equiv SWC,             0xFFFD ; Controlled by SBOREN
        .equiv BOREN_NOSLP,     0xFFFE ; Disabled for Sleep Modes
        .equiv BOREN_ON,        0xFFFF ; Brown Out Enable Bit

        .equiv LPCFG_ON,        0xFFFB ; Retention Sleep controlled by RETEN
        .equiv LPCFG_OFF,       0xFFFF ; No Retention Sleep

        .equiv DNVPEN_DISABLE,  0xFFF7 ; Downside protection disabled when BOR is inactive
        .equiv DNVPEN_ENABLE,   0xFFFF ; Downside protection enabled using ZPBOR when BOR is inactive


;----- FICD (0x55f28) --------------------------------------------------
;
;  The following settings are available for FICD:
;
;   ICD Communication Channel Select bits:
;     ICS_NONE             Reserved, do not use
;     PGX3                 Communicate on PGEC3 and PGED3
;     PGX2                 Communicate on PGEC2 and PGED2
;     PGX1                 Communicate on PGEC1 and PGED1
;     ICS_PGX1             Communicate on PGEC1 and PGED1
;
;   JTAG Enable bit:
;     JTAGEN_OFF           JTAG is disabled
;     JTAGEN_ON            JTAG is enabled
;
;   BOOTSWP Disable:
;     BTSWP_ON             BOOTSWP instruction is enabled
;     BTSWP_OFF            BOOTSWP instruction disabled
;
        .equiv ICS_NONE,        0xFFFC ; Reserved, do not use
        .equiv PGX3,            0xFFFD ; Communicate on PGEC3 and PGED3
        .equiv PGX2,            0xFFFE ; Communicate on PGEC2 and PGED2
        .equiv PGX1,            0xFFFF ; Communicate on PGEC1 and PGED1
        .equiv ICS_PGX1,        0xFFFF ; Communicate on PGEC1 and PGED1

        .equiv JTAGEN_OFF,      0xFFDF ; JTAG is disabled
        .equiv JTAGEN_ON,       0xFFFF ; JTAG is enabled

        .equiv BTSWP_ON,        0x7FFF ; BOOTSWP instruction is enabled
        .equiv BTSWP_OFF,       0xFFFF ; BOOTSWP instruction disabled


;----- FDEVOPT1 (0x55f2c) --------------------------------------------------
;
;  The following settings are available for FDEVOPT1:
;
;   Alternate Comparator Input Enable bit:
;     ALTCMPI_ENABLE       C1INC, C2INC and C3INC are on RG9
;     ALTCMPI_DISABLE      C1INC, C2INC, and C3INC are on their standard pin locations
;
;   Tamper Pin Enable bit:
;     TMPRPIN_ON           TMPRN pin function is enabled
;     TMPRPIN_OFF          TMPRN pin function is disabled
;
;   SOSC High Power Enable bit (valid only when SOSCSEL = 1:
;     SOSCHP_OFF           Enable SOSC low power mode
;     SOSCHP_ON            Enable SOSC high power mode (default)
;
;   Alternate Voltage Reference Location Enable bit:
;     ALTVREF_ALTVREFDIS   VREF+ and CVREF+ on RB0, VREF- and CVREF- on RB1
;     ALTVREF_ALTREFEN     VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9
;
        .equiv ALTCMPI_ENABLE,  0xFFFD ; C1INC, C2INC and C3INC are on RG9
        .equiv ALTCMPI_DISABLE, 0xFFFF ; C1INC, C2INC, and C3INC are on their standard pin locations

        .equiv TMPRPIN_ON,      0xFFFB ; TMPRN pin function is enabled
        .equiv TMPRPIN_OFF,     0xFFFF ; TMPRN pin function is disabled

        .equiv SOSCHP_OFF,      0xFFF7 ; Enable SOSC low power mode
        .equiv SOSCHP_ON,       0xFFFF ; Enable SOSC high power mode (default)

        .equiv ALTVREF_ALTVREFDIS, 0xFFEF ; VREF+ and CVREF+ on RB0, VREF- and CVREF- on RB1
        .equiv ALTVREF_ALTREFEN, 0xFFFF ; VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9


;----- FBTSEQ (0x55ffc) --------------------------------------------------
;
;  The following settings are available for FBTSEQ:
;
;   Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active:
;
;   The one's complement of BSEQ; must be calculated by the user and written during device programming.:
;


.endif


.LIST
