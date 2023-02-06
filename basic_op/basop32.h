/*
  ===========================================================================
   File: BASOP32.H                                       v.2.3 - 30.Nov.2009
  ===========================================================================

            ITU-T STL  BASIC OPERATORS

            GLOBAL FUNCTION PROTOTYPES

   History:
   26.Jan.00   v1.0     Incorporated to the STL from updated G.723.1/G.729 
                        basic operator library (based on basic_op.h) and 
                        G.723.1's basop.h.
   05.Jul.00   v1.1     Added 32-bit shiftless mult/mac/msub operators

   03 Nov 04   v2.0     Incorporation of new 32-bit / 40-bit / control
                        operators for the ITU-T Standard Tool Library as 
                        described in Geneva, 20-30 January 2004 WP 3/16 Q10/16
                        TD 11 document and subsequent discussions on the
                        wp3audio@yahoogroups.com email reflector.
                        norm_s()      weight reduced from 15 to 1.
                        norm_l()      weight reduced from 30 to 1.
                        L_abs()       weight reduced from  2 to 1.
                        L_add()       weight reduced from  2 to 1.
                        L_negate()    weight reduced from  2 to 1.
                        L_shl()       weight reduced from  2 to 1.
                        L_shr()       weight reduced from  2 to 1.
                        L_sub()       weight reduced from  2 to 1.
                        mac_r()       weight reduced from  2 to 1.
                        msu_r()       weight reduced from  2 to 1.
                        mult_r()      weight reduced from  2 to 1.
                        L_deposit_h() weight reduced from  2 to 1.
                        L_deposit_l() weight reduced from  2 to 1.
                        L_mls() weight of 5.
                        div_l() weight of 32.
                        i_mult() weight of 3.

   30 Nov 09   v2.3     round() function is now round_fx().
                        saturate() is not referencable from outside application

   13 Mar 12            Add Overflow2 flag for additional overflow checking.
  ============================================================================
*/

#ifndef _BASIC_OP_H
#define _BASIC_OP_H

#include "typedef.h"

//#define BASOP_OVERFLOW2
#define HIDE_UNUSED_BASOP

/*___________________________________________________________________________
 |                                                                           |
 |   Constants and Globals                                                   |
 |___________________________________________________________________________|
*/
extern Flag Overflow, Overflow2;
extern Flag Carry;

#ifdef BASOP_OVERFLOW2
extern int overflow_count;
extern int overflow_warning_enable, overflow_warning_disable_counter;
extern int overflow_error_enable;

#ifndef HIDE_UNUSED_BASOP
#define BASOP_SATURATE_WARNING_ON  {\
    overflow_warning_disable_counter--;\
    if (overflow_warning_disable_counter < 0) overflow_warning_disable_counter = 0;\
    if (overflow_warning_disable_counter == 0) {\
        overflow_warning_enable=1;\
        Overflow = 0;\
        Overflow2 = 0;\
    }\
}
#else
#define BASOP_SATURATE_WARNING_ON  {\
    overflow_warning_disable_counter--;\
    if (overflow_warning_disable_counter < 0) overflow_warning_disable_counter = 0;\
    if (overflow_warning_disable_counter == 0) {\
        overflow_warning_enable=1;\
    }\
}
#endif


#define BASOP_SATURATE_WARNING_OFF {\
    overflow_warning_enable=0;\
    overflow_warning_disable_counter++;\
}
#define BASOP_SATURATE_ERROR_ON  {\
    overflow_error_enable=1;\
}
#define BASOP_SATURATE_ERROR_OFF {\
    overflow_error_enable=0;\
}
void BASOP_CHECK(int Overflow, int Overflow2);
#else
#define BASOP_SATURATE_WARNING_ON
#define BASOP_SATURATE_WARNING_OFF
#define BASOP_SATURATE_ERROR_ON
#define BASOP_SATURATE_ERROR_OFF
#define BASOP_CHECK(a,b)
#endif


#define MAX_32 (Word32)0x7fffffffL
#define MIN_32 (Word32)0x80000000L

#define MAX_16 (Word16)0x7fff
#define MIN_16 (Word16)0x8000

Word16 saturate_c(Word32 L_var1);

#ifdef WIN32
typedef long long	int64_t;
#define	saturate	saturate_c
#define	abs_s(x)	((x) > 0 ? (x) : -(x))
#else
#define	saturate(x)	__nds32__clips(x, 15)
#define	abs_s		__nds32__abs
#endif

#define mult64(x, y)		(int64_t)((int64_t)(x) * (y))
#define madd64(sum, x, y)	(int64_t)(sum + (int64_t)(x) * (y))
/* A list of functions that need saturation can be find below marked with an _sat */

//Word32 L_shl_sat (Word32 L_var1, Word16 var2);
//Word32 L_shr_sat (Word32 L_var1, Word16 var2);
//Word16 shl_sat (Word16 var1, Word16 var2);
//Word16 shr_sat (Word16 var1, Word16 var2);
#define L_shl_sat	L_shl
#define L_shr_sat	L_shr
#define shl_sat		shl
#define shr_sat		shr

//Word32 L_abs_sat (Word32 L_var1);
//Word16 abs_s_sat (Word16 var1);
#define	L_abs_sat	L_abs
#define	abs_s_sat	abs_s
Word32 round_fx_sat (Word32 L_var1);
//Word32 L_mac_sat (Word32 L_var1, Word16 var1, Word16 var2);
//Word32 L_msu_sat (Word32 L_var1, Word16 var1, Word16 var2);
//Word32 L_mac0_sat (Word32 L_var1, Word16 var1, Word16 var2);
#define	L_mac_sat	L_mac
#define L_msu_sat	L_msu
#define	L_mac0_sat	L_mac0

//Word32 L_add_sat (Word32 L_var1, Word32 L_var2);
//Word32 L_sub_sat (Word32 L_var1, Word32 L_var2);
//Word16 sub_sat (Word16 var1, Word16 var2);
//Word16 add_sat (Word16 var1, Word16 var2);
#define L_add_sat(a, b)	((a)+(b))
#define L_sub_sat(a, b)	((a)-(b))
#define add_sat(var1, var2)	saturate((Word32)(var1) + (var2))
#define sub_sat(var1, var2)	saturate((Word32)(var1) - (var2))
//Word16 mac_r_sat (Word32 L_var1, Word16 var1, Word16 var_2);
#define mac_r_sat	mac_r

#define L_shl_pos(x, y) (L_shl((x), (y)))
#define L_shr_pos(x, y) (L_shr((x), (y)))
#define L_shr_pos_pos(x, y) (L_shr((x), (y)))

#define shl_pos(x, y) (shl((x), (y)))
#define shr_pos(x, y) (shr((x), (y)))
#define shr_pos_pos(x, y) (shr((x), (y)))

#define lshl_pos(x, y) (lshl(x, y))
#define UL_lshr_pos(x, y) (UL_lshr(x, y))
#define UL_lshl_pos(x, y) (UL_lshl(x, y))

/*___________________________________________________________________________
 |                                                                           |
 |   Prototypes for basic arithmetic operators                               |
 |___________________________________________________________________________|
*/

//Word16 add (Word16 var1, Word16 var2);    /* Short add,           1   */
//Word16 sub (Word16 var1, Word16 var2);    /* Short sub,           1   */
#define add(var1, var2)	((Word32)(var1) + (var2))
#define sub(var1, var2)	((Word32)(var1) - (var2))
//Word16 abs_s (Word16 var1);               /* Short abs,           1   */
//Word16 shl (Word16 var1, Word16 var2);    /* Short shift left,    1   */
//Word16 shr (Word16 var1, Word16 var2);    /* Short shift right,   1   */
#define shl(var1, var2)	((var1)<<(var2))
#define shr(var1, var2)	((var1)>>(var2))
//Word16 mult (Word16 var1, Word16 var2);   /* Short mult,          1   */
//Word32 L_mult (Word16 var1, Word16 var2); /* Long mult,           1   */
#define	mult(var1, var2)	(((var1)*(var2))>>15)
#define L_mult(var1, var2)	(((var1)*(var2))<<1)
//Word16 negate (Word16 var1);              /* Short negate,        1   */
#define	negate(var1)	(-(var1))
//Word16 extract_h (Word32 L_var1);         /* Extract high,        1   */
//Word16 extract_l (Word32 L_var1);         /* Extract low,         1   */
#define extract_h(x)	((Word16)((x)>>16))
#define extract_l(x)	((Word16)(x))
//Word16 round_fx (Word32 L_var1);          /* Round,               1   */
#define round_fx(L_var1)	extract_h(L_var1 + 0x00008000L)

//Word32 L_mac (Word32 L_var3, Word16 var1, Word16 var2);   /* Mac,  1  */
//Word32 L_msu (Word32 L_var3, Word16 var1, Word16 var2);   /* Msu,  1  */
#define L_mac(L_var3, var1, var2)	((L_var3) + L_mult(var1, var2))
#define L_msu(L_var3, var1, var2)	((L_var3) - L_mult(var1, var2))

//Word32 L_macNs (Word32 L_var3, Word16 var1, Word16 var2); /* Mac without sat, 1   */
//Word32 L_msuNs (Word32 L_var3, Word16 var1, Word16 var2); /* Msu without sat, 1   */
//Word32 L_add (Word32 L_var1, Word32 L_var2);    /* Long add,        1 */
//Word32 L_sub (Word32 L_var1, Word32 L_var2);    /* Long sub,        1 */
#define L_add(a, b)	((a)+(b))
#define L_sub(a, b)	((Word32)(a)-(Word32)(b))
//Word32 L_add_c (Word32 L_var1, Word32 L_var2);  /* Long add with c, 2 */
//Word32 L_sub_c (Word32 L_var1, Word32 L_var2);  /* Long sub with c, 2 */
//Word32 L_negate (Word32 L_var1);                /* Long negate,     1 */
#define L_negate(L_var1)	(-(L_var1))
//Word16 mult_r (Word16 var1, Word16 var2);       /* Mult with round, 1 */
#define	mult_r(var1, var2)	(((var1)*(var2) + (1<<14)) >> 15)
Word32 L_shl (Word32 L_var1, Word16 var2);      /* Long shift left, 1 */
Word32 L_shr (Word32 L_var1, Word16 var2);      /* Long shift right, 1 */
Word16 shr_r (Word16 var1, Word16 var2);        /* Shift right with
                                                   round, 2           */
//Word16 mac_r (Word32 L_var3, Word16 var1, Word16 var2); /* Mac with rounding, 1 */
//Word16 msu_r (Word32 L_var3, Word16 var1, Word16 var2); /* Msu with rounding, 1 */
#define	mac_r(L_var3, var1, var2)	((L_mac(L_var3, var1, var2) + 0x00008000L)>>16)
#define	msu_r(L_var3, var1, var2)	((L_msu(L_var3, var1, var2) + 0x00008000L)>>16)

//Word32 L_deposit_h (Word16 var1);        /* 16 bit var1 -> MSB,     1 */
//Word32 L_deposit_l (Word16 var1);        /* 16 bit var1 -> LSB,     1 */
#define L_deposit_h(var1)	((var1)<<16)
#define L_deposit_l(var1)	(var1)

Word32 L_shr_r (Word32 L_var1, Word16 var2); /* Long shift right with
                                                round,             3  */
//Word32 L_abs (Word32 L_var1);            /* Long abs,              1  */
#define	L_abs	abs_s
//Word32 L_sat (Word32 L_var1);            /* Long saturation,       4  */
Word16 norm_s (Word16 var1);             /* Short norm,            1  */
//Word16 div_s (Word16 var1, Word16 var2); /* Short division,       18  */
#define div_s(a, b)		saturate(((a)<<15) / (b))
Word16 norm_l (Word32 L_var1);           /* Long norm,             1  */


/*
 * Additional G.723.1 operators
*/
//Word32 L_mls( Word32, Word16 ) ;    /* Weight FFS; currently assigned 5 */
//Word16 div_l( Word32, Word16 ) ;    /* Weight FFS; currently assigned 32 */
//Word16 i_mult(Word16 a, Word16 b);  /* Weight FFS; currently assigned 3 */
#define div_l(a, b)		((a)<<16) / (b)
#define i_mult(a, b)	((a)*(b))
/*
 *  New shiftless operators, not used in G.729/G.723.1
*/
//Word32 L_mult0(Word16 v1, Word16 v2); /* 32-bit Multiply w/o shift         1 */
//Word32 L_mac0(Word32 L_v3, Word16 v1, Word16 v2); /* 32-bit Mac w/o shift  1 */
//Word32 L_msu0(Word32 L_v3, Word16 v1, Word16 v2); /* 32-bit Msu w/o shift  1 */
#define L_mult0(v1, v2)			((v1) * (v2))
#define	L_mac0(L_v3, v1, v2)	((L_v3) + (v1) * (v2))
#define	L_msu0(L_v3, v1, v2)	((L_v3) - (v1) * (v2))

#endif /* ifndef _BASIC_OP_H */


/* end of file */
