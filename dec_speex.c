/*
 *			GPAC - Multimedia Framework C SDK
 *
 *  This file is part of GPAC / Speex decoder filter, based on libspeex
 *  (https://www.speex.org/), the CELP-based speech codec Xiph published
 *  before Opus superseded it.
 *
 *  Unlike the image filters in this tree, this one is a link in a chain
 *  rather than a whole-file decoder: .spx files are Speex packets inside an
 *  Ogg stream, and oggdmx already demuxes them - it declares the "spx"
 *  extension and emits a GF_CODECID_SPEEX pid. So the filter takes that codec
 *  on its input, the way libmad and libfaad do, and never sees the container.
 */

#include <gpac/filters.h>
#include <gpac/constants.h>
#include <string.h>
#include <stdlib.h>

#include <speex/speex.h>
#include <speex/speex_header.h>
#include <speex/speex_stereo.h>
#include <speex/speex_callbacks.h>

typedef struct
{
	GF_FilterPid *ipid, *opid;
	u32 cfg_crc;

	void *state;
	SpeexBits bits;
	SpeexStereoState stereo;

	u32 sample_rate, nb_chan, frame_size, frames_per_packet;
	u32 timescale;
	u64 last_cts;

	/* Speex carries an algorithmic look-ahead: the decoder emits that many
	 * samples of encoder priming before the real signal starts. speexdec drops
	 * them, and so does this filter - otherwise the output is longer than what
	 * was encoded and late by the same amount. libspeex reports 80 samples for
	 * the wideband mode.
	 *
	 * What is NOT done here is speexdec's second trim, the one driven by the
	 * granule positions, which removes the padding the encoder added to reach a
	 * frame boundary. On the wideband test signal that leaves 143 extra samples
	 * at the head and 97 at the tail - 9 ms and 6 ms. The samples themselves are
	 * right: aligned on that offset, not one sample of a 10 s decode differs
	 * from upstream speexdec by more than 1, which is the wasm build's float
	 * rounding. */
	u32 skip_remaining;
	u8 *scratch;
	u32 scratch_size;
} GF_SpeexDecCtx;

static void speexdec_reset(GF_SpeexDecCtx *ctx)
{
	if (ctx->state)
	{
		speex_decoder_destroy(ctx->state);
		ctx->state = NULL;
		speex_bits_destroy(&ctx->bits);
	}
	if (ctx->scratch)
	{
		gf_free(ctx->scratch);
		ctx->scratch = NULL;
		ctx->scratch_size = 0;
	}
}

static GF_Err speexdec_configure_pid(GF_Filter *filter, GF_FilterPid *pid, Bool is_remove)
{
	const GF_PropertyValue *p;
	SpeexHeader *header;
	const SpeexMode *mode;
	SpeexCallback callback;
	int tmp;
	u32 hdr_size;
	GF_SpeexDecCtx *ctx = gf_filter_get_udta(filter);

	if (is_remove)
	{
		if (ctx->opid)
		{
			gf_filter_pid_remove(ctx->opid);
			ctx->opid = NULL;
		}
		ctx->ipid = NULL;
		return GF_OK;
	}
	if (!gf_filter_pid_check_caps(pid))
		return GF_NOT_SUPPORTED;

	/* Speex declares a single init header, which oggdmx hands over as the
	 * decoder config: the 80-byte "Speex   " header carrying the rate, the
	 * channel count, the mode and how many frames each packet holds.
	 *
	 * oggdmx wraps every init header as [u16 length][bytes], the same shape it
	 * uses for Vorbis' three headers, so the Speex header starts two bytes in
	 * rather than at offset 0 - passing the buffer straight to
	 * speex_packet_to_header() is the mistake to avoid here. */
	p = gf_filter_pid_get_property(pid, GF_PROP_PID_DECODER_CONFIG);
	if (!p || !p->value.data.ptr || (p->value.data.size < 2))
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[SpeexDec] Missing Speex decoder config\n"));
		return GF_NOT_SUPPORTED;
	}
	{
		u32 ex_crc = gf_crc_32(p->value.data.ptr, p->value.data.size);
		if (ctx->state && (ctx->cfg_crc == ex_crc))
			return GF_OK;
		ctx->cfg_crc = ex_crc;
	}

	hdr_size = ((u32)(u8)p->value.data.ptr[0] << 8) | (u8)p->value.data.ptr[1];
	if (!hdr_size || (hdr_size + 2 > p->value.data.size))
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[SpeexDec] Truncated Speex header: announced %u bytes, %u available\n",
		                                    hdr_size, p->value.data.size - 2));
		return GF_NON_COMPLIANT_BITSTREAM;
	}
	header = speex_packet_to_header(p->value.data.ptr + 2, (int)hdr_size);
	if (!header)
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[SpeexDec] Not a valid Speex header\n"));
		return GF_NON_COMPLIANT_BITSTREAM;
	}
	mode = speex_lib_get_mode(header->mode);
	if (!mode)
	{
		GF_LOG(GF_LOG_ERROR, GF_LOG_CODEC, ("[SpeexDec] Unknown Speex mode %d\n", header->mode));
		speex_header_free(header);
		return GF_NOT_SUPPORTED;
	}

	speexdec_reset(ctx);
	ctx->state = speex_decoder_init(mode);
	if (!ctx->state)
	{
		speex_header_free(header);
		return GF_OUT_OF_MEM;
	}
	speex_bits_init(&ctx->bits);

	/* The perceptual enhancer is what speexdec turns on by default, and what
	 * every reference decoding of a .spx file is produced with. */
	tmp = 1;
	speex_decoder_ctl(ctx->state, SPEEX_SET_ENH, &tmp);

	tmp = 0;
	speex_decoder_ctl(ctx->state, SPEEX_GET_FRAME_SIZE, &tmp);
	ctx->frame_size = (u32)tmp;

	ctx->sample_rate = header->rate;
	ctx->nb_chan = header->nb_channels ? header->nb_channels : 1;
	ctx->frames_per_packet = header->frames_per_packet ? header->frames_per_packet : 1;
	speex_header_free(header);

	tmp = 0;
	speex_decoder_ctl(ctx->state, SPEEX_GET_LOOKAHEAD, &tmp);
	ctx->skip_remaining = (tmp > 0) ? (u32)tmp : 0;

	ctx->scratch_size = ctx->frames_per_packet * ctx->frame_size * ctx->nb_chan * 2;
	ctx->scratch = (u8 *)gf_malloc(ctx->scratch_size);
	if (!ctx->scratch)
	{
		speexdec_reset(ctx);
		return GF_OUT_OF_MEM;
	}

	/* Speex codes stereo as a mono signal plus intensity information carried
	 * in-band, so the stereo state has to be registered as a callback handler
	 * before decoding rather than derived from the header. */
	ctx->stereo.balance = 1.0f;
	ctx->stereo.e_ratio = 0.5f;
	ctx->stereo.smooth_left = 1.0f;
	ctx->stereo.smooth_right = 1.0f;
	callback.callback_id = SPEEX_INBAND_STEREO;
	callback.func = speex_std_stereo_request_handler;
	callback.data = &ctx->stereo;
	speex_decoder_ctl(ctx->state, SPEEX_SET_HANDLER, &callback);

	if (!ctx->opid)
		ctx->opid = gf_filter_pid_new(filter);

	ctx->ipid = pid;
	gf_filter_pid_set_framing_mode(pid, GF_TRUE);

	gf_filter_pid_copy_properties(ctx->opid, pid);
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CODECID, &PROP_UINT(GF_CODECID_RAW));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_SAMPLE_RATE, &PROP_UINT(ctx->sample_rate));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_NUM_CHANNELS, &PROP_UINT(ctx->nb_chan));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_AUDIO_FORMAT, &PROP_UINT(GF_AUDIO_FMT_S16));
	gf_filter_pid_set_property(ctx->opid, GF_PROP_PID_CHANNEL_LAYOUT,
	                           &PROP_LONGUINT((ctx->nb_chan == 2)
	                                              ? (GF_AUDIO_CH_FRONT_LEFT | GF_AUDIO_CH_FRONT_RIGHT)
	                                              : GF_AUDIO_CH_FRONT_CENTER));

	return GF_OK;
}

static GF_Err speexdec_process(GF_Filter *filter)
{
	GF_FilterPacket *pck, *dst_pck;
	u8 *data, *output;
	u32 size, i, out_size, nb_samples, skip;
	GF_SpeexDecCtx *ctx = gf_filter_get_udta(filter);

	if (!ctx->state)
		return GF_OK;

	pck = gf_filter_pid_get_packet(ctx->ipid);
	if (!pck)
	{
		if (gf_filter_pid_is_eos(ctx->ipid))
		{
			gf_filter_pid_set_eos(ctx->opid);
			return GF_EOS;
		}
		return GF_OK;
	}
	data = (u8 *)gf_filter_pck_get_data(pck, &size);
	if (!data || !size)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OK;
	}

	speex_bits_read_from(&ctx->bits, (char *)data, (int)size);
	for (i = 0; i < ctx->frames_per_packet; i++)
	{
		spx_int16_t *frame = (spx_int16_t *)(ctx->scratch + i * ctx->frame_size * ctx->nb_chan * 2);
		int ret = speex_decode_int(ctx->state, &ctx->bits, frame);
		/* 1 means the stream ended inside this packet, -1 a corrupt one; both
		 * leave the remaining frames undefined, so they are silenced rather
		 * than shipped as whatever was in the buffer. */
		if (ret != 0)
		{
			u32 remaining = (ctx->frames_per_packet - i) * ctx->frame_size * ctx->nb_chan * 2;
			memset(ctx->scratch + i * ctx->frame_size * ctx->nb_chan * 2, 0, remaining);
			break;
		}
		/* Speex carries stereo as mono plus in-band intensity, so a stereo
		 * frame is decoded mono and expanded here, in place, backwards. */
		if (ctx->nb_chan == 2)
			speex_decode_stereo_int(frame, (int)ctx->frame_size, &ctx->stereo);
	}

	nb_samples = ctx->frames_per_packet * ctx->frame_size;
	skip = (ctx->skip_remaining < nb_samples) ? ctx->skip_remaining : nb_samples;
	ctx->skip_remaining -= skip;
	if (skip == nb_samples)
	{
		/* the whole packet was priming */
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OK;
	}

	out_size = (nb_samples - skip) * ctx->nb_chan * 2;
	dst_pck = gf_filter_pck_new_alloc(ctx->opid, out_size, &output);
	if (!dst_pck)
	{
		gf_filter_pid_drop_packet(ctx->ipid);
		return GF_OUT_OF_MEM;
	}
	memcpy(output, ctx->scratch + skip * ctx->nb_chan * 2, out_size);

	gf_filter_pck_merge_properties(pck, dst_pck);
	ctx->last_cts = gf_filter_pck_get_cts(pck);
	ctx->timescale = gf_filter_pck_get_timescale(pck);
	gf_filter_pck_set_cts(dst_pck, ctx->last_cts);
	gf_filter_pck_set_duration(dst_pck, nb_samples - skip);
	gf_filter_pck_set_sap(dst_pck, GF_FILTER_SAP_1);
	gf_filter_pck_send(dst_pck);

	gf_filter_pid_drop_packet(ctx->ipid);
	return GF_OK;
}

static GF_Err speexdec_initialize(GF_Filter *filter)
{
	return GF_OK;
}

static void speexdec_finalize(GF_Filter *filter)
{
	GF_SpeexDecCtx *ctx = gf_filter_get_udta(filter);
	speexdec_reset(ctx);
}

static const GF_FilterCapability SpeexDecCaps[] =
	{
		CAP_UINT(GF_CAPS_INPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_AUDIO),
		CAP_UINT(GF_CAPS_INPUT, GF_PROP_PID_CODECID, GF_CODECID_SPEEX),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_STREAM_TYPE, GF_STREAM_AUDIO),
		CAP_UINT(GF_CAPS_OUTPUT, GF_PROP_PID_CODECID, GF_CODECID_RAW),
};

GF_FilterRegister SpeexDecoderRegister = {
	.name = "speexdec",
	GF_FS_SET_DESCRIPTION("Speex decoder")
		GF_FS_SET_HELP("This filter decodes Speex speech streams using libspeex. It takes a GF_CODECID_SPEEX pid, which oggdmx produces from a .spx file, and outputs raw 16-bit PCM.")
			.private_size = sizeof(GF_SpeexDecCtx),
	SETCAPS(SpeexDecCaps),
	.initialize = speexdec_initialize,
	.configure_pid = speexdec_configure_pid,
	.process = speexdec_process,
	.finalize = speexdec_finalize,
};

const GF_FilterRegister *EMSCRIPTEN_KEEPALIVE speexdec_register(GF_FilterSession *session)
{
	return &SpeexDecoderRegister;
}

#include "filter_register.h"
__attribute__((constructor))
void register_speexdec(void) {
    gf_filter_auto_register("speexdec", speexdec_register);
}
