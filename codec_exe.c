/******************************************************************************
*                        ETSI TS 103 634 V1.1.1                               *
*              Low Complexity Communication Codec Plus (LC3plus)              *
*                                                                             *
* Copyright licence is solely granted through ETSI Intellectual Property      *
* Rights Policy, 3rd April 2019. No patent licence is granted by implication, *
* estoppel or otherwise.                                                      *
******************************************************************************/


#include "functions.h" /* needed for basop instrumentation */
#include "lc3.h"
#include "tinywavein_c.h"
#include "tinywaveout_c.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* struct to hold command line arguments */
typedef struct
{
    char *inputFilename;
    char *outputFilename;
    int   bitrate;
    char *bitrate_file;
    int   encoder_only;
    int   decoder_only;
    int   bipsOut;
    int   formatG192;
    char *configFilenameG192;
    float frame_ms;
    int   hide_counter;
    int   verbose;
    int   plcMeth;
    char *epf;
    int   epmode;
    char *epmode_file;
    char *edf;
    int   ept;
    int   hrmode;
    int   dc;
    char *bandwidth;
    char *channel_coder_vars_file;
} Arguments;

/* local helper functions */
static void    parseCmdl(int ac, char **av, Arguments *arg);
static FILE *  open_bitstream_reader(const char *file, uint32_t *samplerate, int *bitrate, short *channels,
                                     uint32_t *signal_len, float *frame_ms, int *epmode, int *hrmode);
static FILE *  open_bitstream_writer(const char *file, uint32_t samplerate, int bitrate, short channels,
                                     uint32_t signal_len, float frame_ms, int epmode);
static void    write_bitstream_frame(FILE *bitstream_file, uint8_t *bytes, int size);
static int     read_bitstream_frame(FILE *bitstream_file, uint8_t *bytes, int size);
static FILE *  fopen_with_ext(const char *file, const char *ext, const char *mode);
static void    cleanup(void);
static int16_t loopy_read16(FILE *f);
static int64_t loopy_read64(FILE *f);
static void    exit_if(int condition, const char *message);
static void    scale_24_to_16(const int32_t *in, int16_t *out, int n);
static void    scale_16_to_24(const int16_t *in, int32_t *out, int n);
static void    interleave(int32_t **in, int32_t *out, int n, int channels);
static void    deinterleave(int32_t *in, int32_t **out, int n, int channels);

/* needed by cleanup function */
static WAVEFILEIN * input_wav;
static WAVEFILEOUT *output_wav;
static FILE *       output_bitstream;
static FILE *       input_bitstream;
static FILE *       error_pattern_file;
static FILE *       error_detection_file;
static FILE *       bitrate_switching_file;
static FILE *       epmode_switching_file;
static FILE *bandwidth_switching_file;
static FILE *channel_decoder_debug_file_bfi;
static FILE *channel_decoder_debug_file_epmr;
static FILE *channel_decoder_debug_file_error_report;

#include "license.h" /* provides LICENSE string */

static const char *const USAGE_MESSAGE =
    /* Lines must not be longer than this! --------------------------------------->| */
    "Usage: LC3plus [OPTIONS] INPUT OUTPUT BITRATE\n"
    "\n"
    "  INPUT and OUTPUT are wav files, unless another mode is selected in OPTIONS.\n"
    "  BITRATE is specified in bits per second. Alternatively a switching file can\n"
    "  be provided.\n"
    "\nGeneral options:\n"
    "  -E                      Encode mode. INPUT is a wav file, OUTPUT is a binary file.\n"
    "  -D                      Decode mode. INPUT is a binary file, OUTPUT is a wav file.\n"
    "                          In decode mode the BITRATE parameter is ignored.\n"
    "  -bps NUM                Output bits per sample. NUM must be 16 (default) or 24.\n"
    "  -swf FILE               Use a bitrate switching file instead of fixed bitrate.\n"
    "  -dc NUM                 0: Don't use delay compensation\n"
    "                          1: Compensate delay in decoder (default)\n"
    "                          2: Split delay equally in encoder and decoder\n"
    "  -frame_ms               NUM Frame length in ms. NUM must be 10 (default), 5 or 2.5.\n"
    "  -bandwidth NUM|FILE     Select audio bandwidth limitation via value in Hz or switching file.\n"
    "                          NUM can be any integer value describing the bandwidth; max NUM=20000 Hz\n"
    "  -q                      Disable frame counter printout\n"
    "  -v                      Verbose switching commands\n"
    "\nFormat options:\n"
    "  -formatG192             Activate G192 bitstream format. A filename.cfg will be used to\n"
    "                          store/load decoder info.\n"
    "  -cfgG192 FILE           Specify a configuration file for G192 bitstream format.\n"
    "\nPLC options:\n"
    "  -epf FILE               Enable packet loss simulation using error pattern from FILE.\n"
    "  -ept                    Use together with -E -epf FILE to create bitstream triggering\n"
    "                          PLC via special value of lastnz\n"
    "  -edf FILE               Write error detection pattern to FILE.\n"
    "\nChannel coder options:\n"
    "  -epmode NUM|FILE        Error protection mode. NUM must be one of the following:\n"
    "                          0: Error protection disabled\n"
    "                          1: Minimum error protection, detection only\n"
    "                          2: Moderate error protection\n"
    "                          3: Strong error protection\n"
    "                          4: Maximum error protection\n"
    "  -ep_dbg FILE            Save variables bfi, epmr and error report to binary files\n"
    "                          FILE.bfi, FILE.epmr and FILE.error_report\n"
    ;

static const char *const MISSING_ARGUMENT_MESSAGE = "Not enough parameters! Use -h to show help.";

static const char *ERROR_MESSAGE[18] = {
    "",                                                /* LC3_OK                  */
    "Function call failed!",                           /* LC3_ERROR               */
    "Frame failed to decode and was concealed!",       /* LC3_DECODE_ERROR        */
    "Pointer argument is null!",                       /* LC3_NULL_ERROR          */
    "Invalid sampling rate!",                          /* LC3_SAMPLERATE_ERROR    */
    "Invalid number of channels!",                     /* LC3_CHANNELS_ERROR      */
    "Invalid bitrate!",                                /* LC3_BITRATE_ERROR       */
    "Invalid number of bytes!",                        /* LC3_NUMBYTES_ERROR      */
    "Invalid PLC method!",                             /* LC3_PLCMODE_ERROR       */
    "Invalid EP mode!",                                /* LC3_EPCLASS_ERROR       */
    "Invalid frame ms value!",                         /* LC3_FRAMEMS_ERROR       */
    "Unaligned pointer!",                              /* LC3_ALIGN_ERROR         */
    "Invalid channel mode request!",                   /* LC3_CMR_ERROR           */
    "Bitrate has not been set!",                       /* LC3_BITRATE_UNSET_ERROR */
    "Function can't be called after bitrate was set!", /* LC3_BITRATE_SET_ERROR   */
    "Invalid external bad frame index!",               /* LC3_BFI_EXT_ERROR       */
    "Generic Warning",                                 /* LC3_WARNING             */
    "Invalid bandwidth frequency!"                     /* LC3_BW_WARNING          */
};


int main(int ac, char **av)
{
    Arguments arg;
    uint32_t  nSamples = 0, nSamplesRead = 0, nSamplesFile = 0xffffffff, sampleRate = 0;
    short     nChannels = 0, bipsIn = 0;
    int       nBytes = 0, real_bitrate = 0, frame = 1, delay = 0;
    int       encoder_size = 0, decoder_size = 0, scratch_size = 0;
    LC3_Enc * encoder = NULL;
    LC3_Dec * decoder = NULL;
    void *    scratch = NULL;
    LC3_Error err     = LC3_OK;
    int32_t   sample_buf[LC3_MAX_CHANNELS * LC3_MAX_SAMPLES];
    int32_t   buf_24[LC3_MAX_CHANNELS * LC3_MAX_SAMPLES];
    int16_t   buf_16[LC3_MAX_CHANNELS * LC3_MAX_SAMPLES];
    uint8_t   bytes[LC3_MAX_BYTES];
    int       dc2_extra_frame = 0;

    /* Parse Command-line */
    printf(LICENSE, LC3_VERSION >> 16, (LC3_VERSION >> 8) & 255, LC3_VERSION & 255);
    parseCmdl(ac, av, &arg);

#ifdef STAMEM_COUNT
    Sta_Mem_Init();
#endif
#ifdef DYNMEM_COUNT
    Dyn_Mem_Init();
#endif

    /* exit handler to clean up resources */
    atexit(cleanup);


    if (!arg.decoder_only)
    {
        /* Open Input Wav File */
        input_wav = OpenWav(arg.inputFilename, &sampleRate, &nChannels, &nSamplesFile, &bipsIn);
        exit_if(!input_wav, "Error opening wav file!");

        /* Setup Encoder */
        encoder_size = lc3_enc_get_size(sampleRate, nChannels);
        encoder      = malloc(encoder_size);
        err          = lc3_enc_init(encoder, sampleRate, nChannels);
        exit_if(err, ERROR_MESSAGE[err]);

        err = lc3_enc_set_frame_ms(encoder, arg.frame_ms);
        exit_if(err, ERROR_MESSAGE[err]);

        err = lc3_enc_set_ep_mode(encoder, (LC3_EpMode)arg.epmode);
        exit_if(err, ERROR_MESSAGE[err]);

        err = lc3_enc_set_bitrate(encoder, arg.bitrate);
        exit_if(err, ERROR_MESSAGE[err]);

        delay        = arg.dc ? lc3_enc_get_delay(encoder) / arg.dc : 0;
		nSamples = encoder->frame_length;// lc3_enc_get_input_samples(encoder);
        real_bitrate = lc3_enc_get_real_bitrate(encoder);

        if (arg.bandwidth && atoi(arg.bandwidth) == 0)
        {
            bandwidth_switching_file = fopen(arg.bandwidth, "rb");
            exit_if(bandwidth_switching_file == NULL, "Error opening bandwidth switching file!");
            puts("Using bandwidth switching file!");
        }
    }
    else /* !arg->decoder_only */
    {
        /* Open Input Bitstream File */
        input_bitstream =
            open_bitstream_reader(arg.inputFilename, &sampleRate, &arg.bitrate, &nChannels, &nSamplesFile,
                                  &arg.frame_ms, &arg.epmode, &arg.hrmode);
        exit_if(!input_bitstream, "Error opening bitstream file!");
        exit_if(arg.hrmode, "HR bitstreams not supported!");
    }

    if (!arg.encoder_only)
    {
        /* Setup Decoder */
        decoder_size = lc3_dec_get_size(sampleRate, nChannels, (LC3_PlcMode)arg.plcMeth);
        decoder      = malloc(decoder_size);
        err          = lc3_dec_init(decoder, sampleRate, nChannels, (LC3_PlcMode)arg.plcMeth);
        exit_if(err, ERROR_MESSAGE[err]);

        err = lc3_dec_set_frame_ms(decoder, arg.frame_ms);
        exit_if(err, ERROR_MESSAGE[err]);

        err = lc3_dec_set_ep_enabled(decoder, arg.epmode != 0);
        exit_if(err, ERROR_MESSAGE[err]);

        delay    = arg.dc ? lc3_dec_get_delay(decoder) / arg.dc : 0;
		nSamples = decoder->frame_length;// lc3_dec_get_output_samples(decoder);

        /* Open Output Wav File */
        output_wav = CreateWav(arg.outputFilename, sampleRate, nChannels, arg.bipsOut);
        exit_if(!output_wav, "Error creating wav file!");
    }
    else /* !arg->encoder_only */
    {
        /* Open Output Bitstream File */
        output_bitstream = open_bitstream_writer(arg.outputFilename, sampleRate, arg.bitrate, nChannels, nSamplesFile,
                                                 arg.frame_ms, arg.epmode);
        exit_if(!output_bitstream, "Error creating bitstream file!");
    }

    /* open auxillary files */
    if (arg.epf)
    {
        error_pattern_file = fopen(arg.epf, "rb");
        exit_if(!error_pattern_file, "Error opening error pattern file!");
    }
    if (arg.bitrate_file)
    {
        bitrate_switching_file = fopen(arg.bitrate_file, "rb");
        exit_if(!bitrate_switching_file, "Error opening bitrate switching file!");
    }
    if (arg.epmode_file)
    {
        epmode_switching_file = fopen(arg.epmode_file, "rb");
        exit_if(epmode_switching_file == NULL, "Error opening epmode switching file!");
    }
    if (arg.edf)
    {
        error_detection_file = fopen(arg.edf, "wb");
        exit_if(!error_detection_file, "Error creating error detection file!");
    }
    if (arg.channel_coder_vars_file)
    {
        channel_decoder_debug_file_bfi          = fopen_with_ext(arg.channel_coder_vars_file, ".bfi", "wb");
        channel_decoder_debug_file_epmr         = fopen_with_ext(arg.channel_coder_vars_file, ".epmr", "wb");
        channel_decoder_debug_file_error_report = fopen_with_ext(arg.channel_coder_vars_file, ".error_report", "wb");
        exit_if(!channel_decoder_debug_file_bfi || !channel_decoder_debug_file_epmr ||
                    !channel_decoder_debug_file_error_report,
                "Error creating channel decoder debug files!");
    }

    scratch_size = MAX(lc3_dec_get_scratch_size(decoder), lc3_enc_get_scratch_size(encoder));
    scratch      = malloc(scratch_size);
    exit_if(!scratch, "Failed to allocate scratch memory!");

#ifdef STAMEM_COUNT
    Sta_Mem_Add("Encoder", encoder_size);
    Sta_Mem_Add("Decoder", decoder_size);
#endif

    /* Print info */
    printf("Encoder size:     %i\n", encoder_size);
    printf("Decoder size:     %i\n", decoder_size);
    printf("Scratch size:     %i\n", scratch_size);
    printf("Sample rate:      %i\n", sampleRate);
    printf("Channels:         %i\n", nChannels);
    printf("Signal length:    %u\n", nSamplesFile);
    printf("Frame length:     %i\n", nSamples);
    printf("Output format:    %i bits\n", arg.bipsOut);
    printf("Target bitrate:   %i\n", arg.bitrate);
    if (!arg.decoder_only)
    {
        printf("Real bitrate:     %i\n\n", real_bitrate);
    }
    printf("Bandwidth cutoff: %s\n", arg.bandwidth ? arg.bandwidth : "-");
	if (!arg.encoder_only) {
		printf("PLC mode:         %i\n", arg.plcMeth);
	}
    printf("\n");

    setFrameRate(sampleRate, nSamples);
    Init_WMOPS_counter();

    /* delay compensation */
    if (arg.dc == 2 && !arg.decoder_only)
    {
        ReadWavInt(input_wav, sample_buf, nChannels * delay, &nSamplesRead);
    }

    /* Encoder + Decoder loop */
    while (1)
    {
		int ch;
		uint32_t i;
        if (!arg.decoder_only)
        {
            /* Encoder */
            int32_t *input24[] = {buf_24, buf_24 + nSamples};

            /* read audio data */
            ReadWavInt(input_wav, sample_buf, nSamples * nChannels, &nSamplesRead);
            /* zero out rest of last frame */
            memset(sample_buf + nSamplesRead, 0, (nSamples * nChannels - nSamplesRead) * sizeof(sample_buf[0]));

			if (nSamplesRead == 0){
				break;
			}

            int16_t *input16[] = {buf_16, buf_16 + nSamples};
			for (ch = 0; ch < nChannels; ch++){
				for (i = 0; i < nSamples; i++){
					input16[ch][i] = sample_buf[i * nChannels + ch];
				}
			}
            //err = lc3_enc16(encoder, input16, bytes, &nBytes, scratch);
			err = LC3_OK;
			nBytes = Enc_LC3(encoder, input16, 16, bytes, scratch, 0);
			/*static LC3_Error lc3_enc(LC3_Enc * encoder, void** input_samples, int bitdepth, void* output_bytes, int* num_bytes,
				void* scratch)
			{
				RETURN_IF(!encoder || !input_samples || !output_bytes || !num_bytes || !scratch, LC3_NULL_ERROR);
				RETURN_IF(null_in_list(input_samples, encoder->channels), LC3_NULL_ERROR);
				RETURN_IF(bitdepth != 16 && bitdepth != 24, LC3_ERROR);
				RETURN_IF(!encoder->lc3_br_set, LC3_BITRATE_UNSET_ERROR);
				*num_bytes = Enc_LC3(encoder, input_samples, 16, output_bytes, scratch, *num_bytes == -1);
				assert(*num_bytes == lc3_enc_get_num_bytes(encoder));
				return LC3_OK;
			}

			LC3_Error lc3_enc16(LC3_Enc * encoder, int16_t * *input_samples, void* output_bytes, int* num_bytes, void* scratch)
			{
				return lc3_enc(encoder, (void**)input_samples, 16, output_bytes, num_bytes, scratch);
			}*/


            exit_if(err, ERROR_MESSAGE[err]);
        }
        else /* !arg.decoder_only */
        {
            /* Read bitstream */
            nBytes = read_bitstream_frame(input_bitstream, bytes, sizeof(bytes));
            if (nBytes < 0)
            {
                break;
            }
        }

        if (!arg.encoder_only)
        {	/* Decoder */
			// plc....
            //nBytes = 0; /* tell decoder packet is lost and needs to be concealed */
			if (arg.ept) {
				static int frame_pos = 0;
				frame_pos++;
				if ((frame_pos % 32) == 31) {
					nBytes = 0; // tell decoder packet is lost and needs to be concealed
				}
			}
            /* Run Decoder */
            int16_t *output16[] = {buf_16, buf_16 + nSamples};
			err = Dec_LC3(decoder, bytes, nBytes, output16, 16, scratch, 0);
            exit_if(err && err != LC3_DECODE_ERROR, ERROR_MESSAGE[err]);
			for (ch = 0; ch < nChannels; ch++) {
				for (i = 0; i < nSamples; i++) {
					sample_buf[i * nChannels + ch] = output16[ch][i];
				}
			}
            /* Write frame to file */
            WriteWavLong(output_wav, sample_buf + delay * nChannels, MIN(nSamples - delay, nSamplesFile) * nChannels);
            nSamplesFile -= nSamples - delay;
            delay = 0;
        }
        else /* !arg.encoder_only */
        {
            write_bitstream_frame(output_bitstream, bytes, nBytes);
        }


        if (!arg.hide_counter)
        {
            //printf("\rProcessing frame %i", frame++);
            fflush(stdout);
        }
        BASOP_frame_update();
    }

    if (!arg.encoder_only && nSamplesFile > 0 && nSamplesFile < nSamples)
    {
        memset(sample_buf, 0, (nSamplesFile * nChannels) * sizeof(sample_buf[0]));
        WriteWavLong(output_wav, sample_buf, nSamplesFile * nChannels);
    }

    puts("\nProcessing done!");
    if (output_wav)
    {
        printf("%i samples clipped!\n", output_wav->clipCount);
    }

    free(encoder);
    free(decoder);
    free(scratch);

#if WMOPS
    BASOP_end;
#else
    BASOP_end_noprint;
#endif
#ifdef STAMEM_COUNT
    Sta_Mem_Exit();
#endif
#ifdef DYNMEM_COUNT
    Dyn_Mem_Exit();
#endif
}

/* open file with extra extension */
static FILE *fopen_with_ext(const char *file, const char *ext, const char *mode)
{
    FILE *f   = NULL;
    char *tmp = malloc(strlen(file) + strlen(ext) + 1);
    sprintf(tmp, "%s%s", file, ext);
    f = fopen(tmp, mode);
    free(tmp);
    return f;
}

/* close file ignoring NULL pointer */
static void safe_fclose(FILE *f)
{
    if (f != NULL)
        fclose(f);
}

/* ensure clean exit so valgrind & co. don't complain */
void cleanup(void)
{
    CloseWavIn(input_wav);
    CloseWav(output_wav);
    safe_fclose(output_bitstream);
    safe_fclose(input_bitstream);
    safe_fclose(error_pattern_file);
    safe_fclose(error_detection_file);
    safe_fclose(bitrate_switching_file);
    safe_fclose(epmode_switching_file);
    safe_fclose(bandwidth_switching_file);
    safe_fclose(channel_decoder_debug_file_bfi);
    safe_fclose(channel_decoder_debug_file_epmr);
    safe_fclose(channel_decoder_debug_file_error_report);
}

static void parseCmdl(int ac, char **av, Arguments *arg)
{
    int pos = 1;
    memset(arg, 0, sizeof(*arg));
    arg->bipsOut  = 16;
    arg->frame_ms = 10;
    arg->dc       = 0;

    arg->plcMeth = LC3_PLC_ADVANCED;
    exit_if(ac <= 1, USAGE_MESSAGE);

    /* parse options in any order */
    for (; pos < ac && av[pos][0] == '-'; pos++)
    {
        if (!strcmp(av[pos], "-h"))
        {
            puts(USAGE_MESSAGE);
            exit(0);
        }
        if (!strcmp(av[pos], "-q"))
        {
            arg->hide_counter = 1;
        }
        if (!strcmp(av[pos], "-v"))
        {
            arg->verbose = 1;
        }
        if (!strcmp(av[pos], "-E"))
        {
            arg->encoder_only = 1;
            puts("Using only encoder!");
        }
        if (!strcmp(av[pos], "-D"))
        {
            arg->decoder_only = 1;
            puts("Using only decoder!");
        }
        if (!strcmp(av[pos], "-formatG192"))
        {
            arg->formatG192 = 1;
            puts("Reading/writing bitstream in G192 format!");
        }
        if (!strcmp(av[pos], "-cfgG192") && pos + 1 < ac)
        {
            arg->configFilenameG192 = av[++pos];
            puts("Using user defined configuration file for G192 bitstream format!");
        }
        /* error pattern */
        if (!strcmp(av[pos], "-epf") && pos + 1 < ac)
        {
            arg->epf = av[++pos];
            puts("Using error pattern file for frame loss simulation!");
        }
        /* trigger PLC with special decoder modes */
        if (!strcmp(av[pos], "-ept"))
        {
            arg->ept = 1;
            puts("Simulating frame loss by writing special values into lastnz variable!");
        }
        /* Bits per sample */
        if (!strcmp(av[pos], "-bps") && pos + 1 < ac)
        {
            arg->bipsOut = atoi(av[++pos]);
            exit_if(arg->bipsOut != 16 && arg->bipsOut != 24 && arg->bipsOut != 32,
                    "Only 16, 24 or 32 bits per sample are supported!");
        }
        /* delay compensation */
        if (!strcmp(av[pos], "-dc") && pos + 1 < ac)
        {
            arg->dc = atoi(av[++pos]);
            exit_if(arg->dc < 0 || arg->dc > 2, "dc musst be 0, 1 or 2!");
        }
        /* select bandwidth */
        if (!strcmp(av[pos], "-bandwidth") && pos + 1 < ac)
        {
            arg->bandwidth = av[++pos];
        }
        /* frame length in ms */
        if (!strcmp(av[pos ], "-frame_ms") && pos + 1 < ac)
        {
            arg->frame_ms = (float)atof(av[++pos]);
        }
        /* Bitrate switching file */
        if (!strcmp(av[pos], "-swf") && pos + 1 < ac)
        {
            arg->bitrate_file = av[++pos];
            puts("Using bitrate switching file!");
        }
        /* Error protection mode */
        if (!strcmp(av[pos], "-epmode") && pos + 1 < ac)
        {
            arg->epmode = atoi(av[++pos]);
            exit_if((unsigned)arg->epmode > 5, "EP mode must be in range [0-5]");
            if (arg->epmode == 0 && strcmp(av[pos], "0"))
            {
                arg->epmode      = 1;
                arg->epmode_file = av[pos];
                puts("Using epmode switching file!");
            }
            else
            {
                printf("Error protection %sabled (%i). ", arg->epmode ? "en" : "dis", arg->epmode);
            }
        }
        /* Error detection pattern */
        if (!strcmp(av[pos], "-edf") && pos + 1 < ac)
        {
            arg->edf = av[++pos];
            puts("Writing error detection file!");
        }

        /* error pattern */
        if (!strcmp(av[pos], "-ep_dbg") && pos + 1 < ac)
        {
            arg->channel_coder_vars_file = av[++pos];
            puts("Saving channel decoder debug information to files!");
        }
    }

    exit_if(arg->encoder_only && arg->decoder_only, "Enocder and decoder modes are exclusive!");
    exit_if(arg->ept && (!arg->epf && arg->encoder_only), "Use -ept only with -E -epf FILE!");
    exit_if(pos + 1 >= ac, MISSING_ARGUMENT_MESSAGE);

    arg->inputFilename  = av[pos++];
    arg->outputFilename = av[pos++];

    /* Bitrate */
    if (!arg->decoder_only)
    {
        exit_if(pos >= ac, MISSING_ARGUMENT_MESSAGE);
        arg->bitrate = atoi(av[pos]);
        if (arg->bitrate == 0)
        {
            arg->bitrate      = 64000; /* dummy value */
            arg->bitrate_file = av[pos];
            puts("Using bitrate switching file!");
        }
    }
    putchar('\n');
}

/* check condition and if it fails, exit with error message */
static void exit_if(int condition, const char *message)
{
    if (condition)
    {
        puts(message);
        if (condition < LC3_WARNING)
        {
            exit(1);
        }
    }
}

static FILE *open_bitstream_writer(const char *file, uint32_t samplerate, int bitrate, short channels,
                                   uint32_t signal_len, float frame_ms, int epmode)
{
    FILE *f     = fopen(file, "wb");
    FILE *f_use = f;
    FILE *f_cfg = NULL;

    if (f_use)
    {
        uint16_t header[9] = {0xcc1c,        sizeof(header), samplerate / 100,
                              bitrate / 100, channels,       (uint16_t)(frame_ms * 100),
                              epmode,        signal_len,     signal_len >> 16};
        fwrite(&header, sizeof(header), 1, f_use);
    }

    safe_fclose(f_cfg);
    return f;
}

static FILE *open_bitstream_reader(const char *file, unsigned int *samplerate, int *bitrate, short *channels,
                                   uint32_t *signal_len, float *frame_ms, int *epmode, int *hrmode)
{
    FILE *f     = fopen(file, "rb");
    FILE *f_use = f;
    FILE *f_cfg = NULL;

    if (f_use)
    {
        uint16_t header[10] = {0};
        fread(header, sizeof(header), 1, f_use);
        {
            assert(header[1] >= 18);
            *samplerate = header[2] * 100;
            *bitrate    = header[3] * 100;
            *channels   = header[4];
            *frame_ms   = (float)(header[5] / 100.0);
            *epmode     = header[6];
            *signal_len = (uint32_t)header[7] | ((uint32_t)header[8] << 16);
            *hrmode     = header[1] > 18 ? header[9] : 0;
            fseek(f_use, header[1], SEEK_SET);
        }
    }

    safe_fclose(f_cfg);
    return f;
}

static void write_bitstream_frame(FILE *bitstream_file, uint8_t *bytes, int size)
{
	int      i = 0;
	uint16_t nbytes = size;
	fwrite(&nbytes, sizeof(nbytes), 1, bitstream_file);
	for (i = 0; i < size; i++)
	{
		putc(bytes[i], bitstream_file);
	}
}

static int read_bitstream_frame(FILE *bitstream_file, uint8_t *bytes, int size)
{
	int      i = 0;
	uint16_t nbytes = 0;
	if (fread(&nbytes, sizeof(nbytes), 1, bitstream_file) != 1)
	{
		return -1; /* End of file reached */
	}
	for (i = 0; i < nbytes && i < size; i++)
	{
		bytes[i] = getc(bitstream_file);
	}
	return nbytes;
}


static void scale_24_to_16(const int32_t *in, int16_t *out, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        out[i] = in[i];
    }
}

static void scale_16_to_24(const int16_t *in, int32_t *out, int n)
{
    int i;
    for (i = 0; i < n; i++)
    {
        out[i] = in[i];
    }
}

static void interleave(int32_t **in, int32_t *out, int n, int channels)
{
    int ch, i;
    for (ch = 0; ch < channels; ch++)
    {
        for (i = 0; i < n; i++)
        {
            out[i * channels + ch] = in[ch][i];
        }
    }
}

static void deinterleave(int32_t *in, int32_t **out, int n, int channels)
{
    int ch, i;
    for (ch = 0; ch < channels; ch++)
    {
        for (i = 0; i < n; i++)
        {
            out[ch][i] = in[i * channels + ch];
        }
    }
}

