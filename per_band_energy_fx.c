/******************************************************************************
*                        ETSI TS 103 634 V1.1.1                               *
*              Low Complexity Communication Codec Plus (LC3plus)              *
*                                                                             *
* Copyright licence is solely granted through ETSI Intellectual Property      *
* Rights Policy, 3rd April 2019. No patent licence is granted by implication, *
* estoppel or otherwise.                                                      *
******************************************************************************/

#include "functions.h"

/* #define BIT_EXACT */


void processPerBandEnergy_fx(Word32 *d2_fx, Word16 *d2_fx_exp, Word32 *d_fx, Word16 d_fx_exp,
                             const Word16 *band_offsets, Word16 fs_idx, Word16 n_bands, Word16 linear, Word16 frame_dms,
                             Word8 *scratchBuffer)
{
    Dyn_Mem_Deluxe_In(
        Counter i, k, band;
        Word16  s;
        Word16  s1;
        Word16  s2;
        Word32  nrg;
        Word16  smax;
        Word16  tmp16;
        Word16  nbands;
        Word16  maxBwBin;
        Word16  stopBand;
        Word16  bandsOffsetOne;
        Word16  bandsOffsetTwo;
        Word16 *d2_band_fx_exp;
    );


    d2_band_fx_exp = (Word16 *)scratchAlign(scratchBuffer, 0); /* Size = 2 * MAX_BANDS_NUMBER_PLC bytes */

    maxBwBin = MAX_BW; 
#if BTLLC_5MS_MODE
	maxBwBin = MAX_BW >> 1;
	bandsOffsetOne = 21;// bands_offset_with_one_max_5ms[fs_idx];
	bandsOffsetTwo = 29;// bands_offset_with_two_max_5ms[fs_idx];
	if(linear){
		bandsOffsetOne = 0;// bands_offset_with_one_max_lin_5ms[fs_idx];
		bandsOffsetTwo = 0;// bands_offset_with_two_max_lin_5ms[fs_idx];
	}
#else
    SWITCH (frame_dms)
    {
    //case 25:
    //    maxBwBin       = MAX_BW >> 2;                             
    //    bandsOffsetOne = bands_offset_with_one_max_2_5ms[fs_idx]; 
    //    bandsOffsetTwo = bands_offset_with_two_max_2_5ms[fs_idx]; 
    //    BREAK;
    case 50:
        maxBwBin       = MAX_BW >> 1;                           
        bandsOffsetOne = bands_offset_with_one_max_5ms[fs_idx]; 
        bandsOffsetTwo = bands_offset_with_two_max_5ms[fs_idx]; 
        BREAK;
    default:                                                /* 100 */
        //bandsOffsetOne = bands_offset_with_one_max[fs_idx]; 
        //bandsOffsetTwo = bands_offset_with_two_max[fs_idx]; 
        BREAK;
    }

    IF (sub(linear, 1) == 0)
    {
        SWITCH (frame_dms)
        {
        //case 25:
        //    bandsOffsetOne = bands_offset_with_one_max_lin_2_5ms[fs_idx];  
        //    bandsOffsetTwo = bands_offset_with_two_max_lin_2_5ms[fs_idx];  
        //    BREAK;
        case 50:
            bandsOffsetOne = bands_offset_with_one_max_lin_5ms[fs_idx]; 
            bandsOffsetTwo = bands_offset_with_two_max_lin_5ms[fs_idx]; 
            BREAK;
        case 100:
            //bandsOffsetOne = bands_offset_with_one_max_lin[fs_idx]; 
            //bandsOffsetTwo = bands_offset_with_two_max_lin[fs_idx]; 
            BREAK;
        }
    }
#endif
    /* start processing with band offsets == 1 */
#if (BTLLC_5MS_MODE == 2)	// low quality
	band = 0;
	i = 0;
#else
    FOR (band = 0; band < bandsOffsetOne; band++)
    {
        ASSERT((band_offsets[band + 1] - band_offsets[band]) == 1);
        ASSERT(band < maxBwBin);

        s2 = 15; 
        s  = norm_l(d_fx[band]);
        if (d_fx[band] != 0)
            s2 = s_min(s2, s);

        tmp16 = extract_h(L_shl_pos(d_fx[band], s2));

        d2_fx[band]          = L_mult0(tmp16, tmp16);  
        d2_band_fx_exp[band] = sub(1, shl_pos(s2, 1)); 
    }

    /* start processing with band offsets == 2 */
    i = bandsOffsetOne; 
    FOR (; band < bandsOffsetTwo; band++)
    {
        ASSERT((band_offsets[band + 1] - band_offsets[band]) == 2);
        IF (sub(add(i, 1), maxBwBin) >= 0)
        {
            IF (sub(i, maxBwBin) >= 0)
            {
                d2_fx[band]          = 0; 
                d2_band_fx_exp[band] = sub(1, shl_pos(15, 1));  
            }
            ELSE
            {
                s2 = 15; 
                s  = norm_l(d_fx[band]);
                if (d_fx[band] != 0)
                    s2 = s_min(s2, s);

                tmp16 = extract_h(L_shl_pos(d_fx[band], s2));

                d2_fx[band]          = L_mult0(tmp16, tmp16);  
                d2_band_fx_exp[band] = sub(1, shl_pos(s2, 1)); 
            }
        }
        ELSE
        {
            ASSERT(i + 1 < maxBwBin);

            s2 = 15; 
            s  = norm_l(d_fx[i]);
            if (d_fx[i] != 0)
                s2 = s_min(s2, s);
            s = norm_l(d_fx[i + 1]);
            if (d_fx[i + 1] != 0)
                s2 = s_min(s2, s);

            tmp16 = extract_h(L_shl_pos(d_fx[i], s2));
            nrg   = L_mult0(tmp16, tmp16);
            nrg   = L_min(nrg, 0x3FFFFFFF);
            tmp16 = extract_h(L_shl_pos(d_fx[i + 1], s2));

            d2_fx[band]          = L_shr_pos(L_mac0(nrg, tmp16, tmp16), 1); 
            d2_band_fx_exp[band] = sub(1, shl_pos(s2, 1));                  
        }
        i = add(i, 2);

    }
#endif
    /* proceed with band offsets > 2 */
    FOR (; band < n_bands; band++)
    {
        /* normalization */
        k        = i;  
        s1       = 15; 

        stopBand = s_min(band_offsets[band + 1], maxBwBin);
        FOR (; k < stopBand; k++)
        {
            s = norm_l(d_fx[k]);
            if (d_fx[k] != 0)
                s1 = s_min(s1, s);
        }

        nbands = sub(band_offsets[band + 1], band_offsets[band]);
        ASSERT(nbands < 32);
        nbands = s_min(s_max(0, nbands), 31);

        /* specify headroom, it can be reduced by one due to use of L_mac0 */
        s2 = sub(s1, bands_nrg_scale[nbands]);

        /* calculate energy per band */
        nrg = 0; 

        FOR (; i < stopBand; i++)
        {
            tmp16 = extract_h(L_shl(d_fx[i], s2));
            nrg   = L_mac0(nrg, tmp16, tmp16);
        }
        i = band_offsets[band + 1];

        /* calculate mean value of energy */
        nrg = Mpy_32_16(nrg, InvIntTable[nbands]);

        /* store normalized energy */
        s                    = norm_l(nrg);
        d2_fx[band]          = L_shl_pos(nrg, s);              
        d2_band_fx_exp[band] = sub(1, add(shl_pos(s2, 1), s)); 
    }

    /* Determine maximum exponent and rescale band energies */
    smax = -31; 
    FOR (band = 0; band < n_bands; band++)
    {
        smax = s_max(smax, d2_band_fx_exp[band]);
    }
    FOR (band = 0; band < n_bands; band++)
    {
        d2_fx[band] = L_shr_pos(d2_fx[band], s_min(sub(smax, d2_band_fx_exp[band]), 31)); 
    }

    /* Save exponent for all bands */
    *d2_fx_exp = add(shl_pos(d_fx_exp, 1), smax); 

    Dyn_Mem_Deluxe_Out();
}

