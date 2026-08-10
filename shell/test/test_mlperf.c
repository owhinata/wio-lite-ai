/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/*
 * Host test for the MLPerf Tiny harness (issue #55, port/mlperf/mlperf_th.cc).
 *
 * WHAT MAKES THIS WORTH HAVING: it links UPSTREAM'S OWN PARSER
 * (lib/mlperf-tiny/benchmark/api/internally_implemented.cpp), unmodified, against our
 * th_* implementation.  So the thing under test is not a reimplementation of the
 * protocol -- it is the protocol, driven exactly as the host's runner drives it:
 * `db load N%`, then `db <62 hex digits>%` over and over, then `infer N W%`.  What the
 * board would put on the wire is what this captures.
 *
 * The three things that can only go wrong QUIETLY are all checked here:
 *
 *   1. THE INPUT TRANSFORM IS PER BENCHMARK, NOT PER MODEL.  ic01/vww01 arrive as
 *      unsigned bytes and must be shifted by 128; kws01 arrives already int8 and must
 *      NOT be; ad01 arrives as float32 and must be quantized.  Get one wrong and the
 *      model still runs and still emits confident-looking scores.
 *   2. THE RESULT FORMATTING.  svc/fmt.c has no %f, so the three decimals the host
 *      parses with float() are assembled from integers.  A rounding or sign slip is
 *      invisible until the accuracy numbers come out subtly low.
 *   3. THE BENCHMARK IDENTIFICATION.  `profile` decides which test the host runs.
 *
 * The hardware is stubbed at the narrowest points the harness actually touches: six
 * nn.h functions, cli_write() and the microsecond clock.  Everything else -- the
 * parser, the formatter, the transforms, the quantizer -- is the real code.
 */
#include "nn.h"
#include "mlperf_th.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(cond, ...) do { \
	if (!(cond)) { \
		printf("test_mlperf: FAIL %s:%d: ", __FILE__, __LINE__); \
		printf(__VA_ARGS__); \
		printf("\n"); \
		exit(1); \
	} \
} while (0)

/* ---- captured console ---------------------------------------------------- */

static char   cap_buf[1 << 16];
static size_t cap_len;

static void cap_reset(void) { cap_len = 0; cap_buf[0] = '\0'; }

/* The one console primitive the harness uses.  Real signature, from cli.h. */
struct cli_instance;
int cli_write(struct cli_instance *sh, const void *data, size_t len);
int cli_write(struct cli_instance *sh, const void *data, size_t len)
{
	(void)sh;
	CHECK(cap_len + len < sizeof cap_buf, "capture buffer overflow");
	memcpy(cap_buf + cap_len, data, len);
	cap_len += len;
	cap_buf[cap_len] = '\0';
	return 0;
}

static const char *cap_find(const char *needle) { return strstr(cap_buf, needle); }

/* ---- stubbed model ------------------------------------------------------- */

/*
 * nn.h keeps `struct nn_model` opaque, which is what lets this test exist: the
 * harness only ever passes the pointer back, so a fake one costs nothing.  Same trick
 * as shell/test/test_blazeface.c.
 */
static struct nn_tensor fake_in, fake_out;
static uint8_t          in_store[96 * 96 * 3];
static int8_t           out_store[640];
static int              run_calls;
static int8_t         (*run_fill)(uint32_t idx);

static int dummy_model;
#define MODEL ((struct nn_model *)&dummy_model)

int nn_input_count(const struct nn_model *m)  { (void)m; return 1; }
int nn_output_count(const struct nn_model *m) { (void)m; return 1; }
struct nn_tensor *nn_input(struct nn_model *m, int i)  { (void)m; return i ? NULL : &fake_in; }
struct nn_tensor *nn_output(struct nn_model *m, int i) { (void)m; return i ? NULL : &fake_out; }
uint32_t nn_last_cycles(const struct nn_model *m) { (void)m; return 1000u; }

int nn_run(struct nn_model *m)
{
	uint32_t i, n = (uint32_t)fake_out.dims[0] * fake_out.dims[1];

	(void)m;
	run_calls++;
	for (i = 0u; i < n; i++)
		out_store[i] = run_fill ? run_fill(i) : 0;
	return 0;
}

uint64_t tx_glue_us64(void);
uint64_t tx_glue_us64(void) { return 1234567ull; }

/* A non-NULL console handle; mlperf_bind() refuses NULL. */
static struct cli_instance *const SH = (struct cli_instance *)(void *)&cap_len;

static void model_set(const uint16_t *dims, int ndim, uint32_t in_bytes,
                      float in_scale, int32_t in_zp,
                      uint16_t out_elems, float out_scale, int32_t out_zp)
{
	int i;

	memset(&fake_in, 0, sizeof fake_in);
	memset(&fake_out, 0, sizeof fake_out);
	memset(in_store, 0, sizeof in_store);
	memset(out_store, 0, sizeof out_store);

	fake_in.data       = in_store;
	fake_in.bytes      = in_bytes;
	fake_in.ndim       = (uint8_t)ndim;
	for (i = 0; i < ndim; i++)
		fake_in.dims[i] = dims[i];
	fake_in.dtype      = NN_DTYPE_INT8;
	fake_in.scale      = in_scale;
	fake_in.zero_point = in_zp;

	fake_out.data       = out_store;
	fake_out.bytes      = out_elems;
	fake_out.ndim       = 2;
	fake_out.dims[0]    = 1;
	fake_out.dims[1]    = out_elems;
	fake_out.dtype      = NN_DTYPE_INT8;
	fake_out.scale      = out_scale;
	fake_out.zero_point = out_zp;

	run_calls = 0;
	run_fill  = NULL;
}

/* The four benchmarks, as the table in mlperf_th.cc sees them. */
static const uint16_t DIM_IC[]  = { 1, 32, 32, 3 };
static const uint16_t DIM_KWS[] = { 1, 49, 10, 1 };
static const uint16_t DIM_VWW[] = { 1, 96, 96, 3 };
static const uint16_t DIM_AD[]  = { 1, 640 };

static void set_ic(void)  { model_set(DIM_IC,  4, 32*32*3, 1.0f,        -128,  10, 1.0f/256.0f, -128); }
static void set_kws(void) { model_set(DIM_KWS, 4, 49*10,   0.584703f,     83,  12, 1.0f/256.0f, -128); }
static void set_vww(void) { model_set(DIM_VWW, 4, 96*96*3, 1.0f/255.0f, -128,   2, 1.0f/256.0f, -128); }
static void set_ad(void)  { model_set(DIM_AD,  2, 640,     0.391015f,     89, 640, 0.364498f,     96); }

/* ---- driving the protocol exactly as the runner does --------------------- */

static void send(const char *cmd)
{
	const char *p;

	for (p = cmd; *p; p++)
		mlperf_feed(*p);
	mlperf_feed('%');           /* EE_CMD_TERMINATOR; the host sends no newline */
}

/*
 * benchmark/runner/device_under_test.py sends `db load N`, then `db ` + 31 bytes as
 * 62 hex digits per command, waiting for m-ready each time.  Same chunk size here, so
 * the test drives the same number of parser entries and the same short final chunk.
 */
#define DB_CHUNK 31

static void db_load(const uint8_t *data, size_t len)
{
	char   cmd[3 + DB_CHUNK * 2 + 1];
	size_t i, j;

	snprintf(cmd, sizeof cmd, "db load %u", (unsigned)len);
	send(cmd);

	for (i = 0; i < len; i += DB_CHUNK) {
		size_t n = (len - i < DB_CHUNK) ? len - i : DB_CHUNK;

		memcpy(cmd, "db ", 3);
		for (j = 0; j < n; j++)
			snprintf(cmd + 3 + j * 2, 3, "%02x", data[i + j]);
		send(cmd);
	}
}

/* ---- tests --------------------------------------------------------------- */

static int8_t fill_ramp(uint32_t i) { return (int8_t)(-128 + (int)(i * 7u % 256u)); }

static void test_identify(void)
{
	static const uint16_t blazeface[] = { 1, 128, 128, 3 };

	set_ic();
	CHECK(mlperf_bind(SH, MODEL) == 0, "ic01 bind");
	CHECK(strcmp(mlperf_model_id(), "ic01") == 0, "ic01 id, got %s", mlperf_model_id());

	set_kws();
	CHECK(mlperf_bind(SH, MODEL) == 0, "kws01 bind");
	CHECK(strcmp(mlperf_model_id(), "kws01") == 0, "kws01 id");

	set_vww();
	CHECK(mlperf_bind(SH, MODEL) == 0, "vww01 bind");
	CHECK(strcmp(mlperf_model_id(), "vww01") == 0, "vww01 id");

	set_ad();
	CHECK(mlperf_bind(SH, MODEL) == 0, "ad01 bind");
	CHECK(strcmp(mlperf_model_id(), "ad01") == 0, "ad01 id");

	/* BlazeFace-shaped: a real model this firmware runs, and NOT a benchmark. */
	model_set(blazeface, 4, 128 * 128 * 3, 1.0f, -128, 896, 1.0f, 0);
	CHECK(mlperf_bind(SH, MODEL) == MLPERF_ERR_SHAPE,
	      "a non-benchmark shape must be refused");
	CHECK(strcmp(mlperf_model_id(), "none") == 0,
	      "a refused bind must leave nothing bound");

	/* Right shape, wrong element type: named as a dtype problem, not run as noise. */
	set_ic();
	fake_in.dtype = NN_DTYPE_FLOAT32;
	CHECK(mlperf_bind(SH, MODEL) == MLPERF_ERR_DTYPE,
	      "float I/O must be refused as DTYPE, not SHAPE");

	mlperf_unbind();
	printf("  identify: ok\n");
}

/* `profile` must report the id the table chose, through upstream's parser. */
static void test_profile_command(void)
{
	set_kws();
	CHECK(mlperf_bind(SH, MODEL) == 0, "bind");
	cap_reset();
	mlperf_monitor_start();
	send("profile");
	CHECK(cap_find("m-model-[kws01]\r\n") != NULL,
	      "profile must name the loaded benchmark; got:\n%s", cap_buf);
	CHECK(cap_find("m-ready\r\n") != NULL, "every command ends with m-ready");
	mlperf_unbind();
	printf("  profile: ok\n");
}

/* ic01 / vww01: unsigned bytes in, int8 shifted by 128 in the tensor. */
static void test_offset_u8(const char *what, void (*setup)(void), uint32_t bytes)
{
	uint8_t *sample = malloc(bytes);
	uint32_t i;

	CHECK(sample != NULL, "alloc");
	for (i = 0u; i < bytes; i++)
		sample[i] = (uint8_t)(i * 37u + 11u);

	setup();
	CHECK(mlperf_bind(SH, MODEL) == 0, "%s bind", what);
	cap_reset();
	mlperf_monitor_start();
	db_load(sample, bytes);
	CHECK(cap_find("m-load-done\r\n") != NULL, "%s: db never completed", what);

	send("infer 1 0");
	CHECK(run_calls == 1, "%s: expected 1 inference, got %d", what, run_calls);
	for (i = 0u; i < bytes; i++)
		CHECK(in_store[i] == (uint8_t)(sample[i] ^ 0x80u),
		      "%s: byte %u: sent %u, tensor holds %d (want %d)", what, i,
		      sample[i], (int)(int8_t)in_store[i],
		      (int)(int8_t)(uint8_t)(sample[i] ^ 0x80u));

	mlperf_unbind();
	free(sample);
	printf("  %s input transform: ok (%u bytes)\n", what, bytes);
}

/* kws01: the runner already quantized; the bytes must arrive untouched. */
static void test_int8_passthrough(void)
{
	const uint32_t bytes = 49 * 10;
	uint8_t sample[49 * 10];
	uint32_t i;

	for (i = 0u; i < bytes; i++)
		sample[i] = (uint8_t)(i * 53u + 3u);

	set_kws();
	CHECK(mlperf_bind(SH, MODEL) == 0, "kws bind");
	cap_reset();
	mlperf_monitor_start();
	db_load(sample, bytes);
	send("infer 1 0");

	for (i = 0u; i < bytes; i++)
		CHECK(in_store[i] == sample[i],
		      "kws01 byte %u was transformed (%u -> %u); it must not be",
		      i, sample[i], in_store[i]);
	mlperf_unbind();
	printf("  kws01 input passthrough: ok\n");
}

/* ic01 results: ten dequantized values, three decimals, comma separated. */
static void test_results_classes(void)
{
	uint8_t sample[32 * 32 * 3];
	char    want[256];
	const char *p;
	uint32_t i;
	int      n;

	for (i = 0u; i < sizeof sample; i++)
		sample[i] = (uint8_t)i;

	set_ic();
	run_fill = fill_ramp;
	CHECK(mlperf_bind(SH, MODEL) == 0, "bind");
	cap_reset();
	mlperf_monitor_start();
	db_load(sample, sizeof sample);
	send("infer 1 0");

	p = cap_find("m-results-[");
	CHECK(p != NULL, "no results line:\n%s", cap_buf);

	/*
	 * Derive what the host will parse, independently of the formatter.  scale 1/256
	 * with zp -128 makes real = (q + 128)/256, so the milli value is
	 * round((q+128)*1000/256) -- written as integer arithmetic so this expectation
	 * does not inherit a float rounding bug from the code it is checking.
	 */
	n = snprintf(want, sizeof want, "m-results-[");
	for (i = 0u; i < 10u; i++) {
		int      q     = (int)fill_ramp(i);
		unsigned milli = (unsigned)(((q + 128) * 1000 + 128) / 256);

		n += snprintf(want + n, sizeof want - (size_t)n, "%s%u.%03u",
		              i ? "," : "", milli / 1000u, milli % 1000u);
	}
	snprintf(want + n, sizeof want - (size_t)n, "]\r\n");
	CHECK(strncmp(p, want, strlen(want)) == 0,
	      "results mismatch\n  want %s  got  %.90s", want, p);

	mlperf_unbind();
	printf("  ic01 results: ok\n");
}

/* ad01: float32 in, quantized across; one MSE value out. */
static void test_ad_mse(void)
{
	const float   in_scale = 0.391015f, out_scale = 0.364498f;
	const int32_t in_zp    = 89,        out_zp    = 96;
	float    sample[640];
	uint8_t  raw[640 * 4];
	double   sum = 0.0;
	char     want[64];
	unsigned milli;
	uint32_t i;

	/*
	 * A ramp would never leave the quantizer's linear middle, so the interesting
	 * inputs are placed explicitly first: both saturation limits, and values whose
	 * quotient is exactly +/-0.5 so the half-away-from-zero rounding is actually
	 * exercised (0.5 is a power of two, so v = 0.5 * scale divides back to exactly
	 * 0.5 in float).  The rest is the ordinary range.
	 */
	sample[0] = -1000.0f;                 /* saturates to -128 */
	sample[1] =  1000.0f;                 /* saturates to +127 */
	sample[2] =  0.5f * in_scale;         /* exactly +0.5 -> rounds away from zero */
	sample[3] = -0.5f * in_scale;         /* exactly -0.5 -> rounds away from zero */
	sample[4] =  1.5f * in_scale;         /* exactly +1.5 -> to 2, not to even      */
	sample[5] =  0.0f;                    /* the zero point itself                  */
	for (i = 6u; i < 640u; i++)
		sample[i] = -8.0f + (float)i * 0.025f;
	memcpy(raw, sample, sizeof raw);   /* little-endian float32, as the host sends */

	set_ad();
	run_fill = fill_ramp;
	CHECK(mlperf_bind(SH, MODEL) == 0, "ad bind");
	cap_reset();
	mlperf_monitor_start();
	db_load(raw, sizeof raw);
	send("infer 1 0");

	/* The quantization must match upstream's QuantizeFloatToInt8() exactly:
	 * round(v/scale) + zero_point, saturated. */
	for (i = 0u; i < 640u; i++) {
		long q = lround((double)(sample[i] / in_scale)) + in_zp;

		if (q < -128) q = -128;
		if (q >  127) q =  127;
		CHECK((int)(int8_t)in_store[i] == (int)q,
		      "ad01 element %u: quantized to %d, upstream's formula says %ld",
		      i, (int)(int8_t)in_store[i], q);
	}

	/* ...and the score must be the MSE against the ORIGINAL floats, not against the
	 * dequantized input -- that would cancel the input's own quantization error out
	 * of both sides and flatter the reconstruction. */
	for (i = 0u; i < 640u; i++) {
		double d = (double)(out_scale * (float)((int)fill_ramp(i) - out_zp))
		           - (double)sample[i];
		sum += d * d;
	}
	milli = (unsigned)(sum / 640.0 * 1000.0 + 0.5);
	snprintf(want, sizeof want, "m-results-[%u.%03u]\r\n",
	         milli / 1000u, milli % 1000u);
	CHECK(cap_find(want) != NULL, "ad01 MSE mismatch\n  want %s  got  %s",
	      want, cap_find("m-results-"));

	mlperf_unbind();
	printf("  ad01 quantize + MSE: ok\n");
}

/* A short `db` must be reported, not silently inferred on. */
static void test_short_load(void)
{
	uint8_t sample[16];

	memset(sample, 0xA5, sizeof sample);
	set_ic();
	CHECK(mlperf_bind(SH, MODEL) == 0, "bind");
	cap_reset();
	mlperf_monitor_start();
	db_load(sample, sizeof sample);
	send("infer 1 0");

	CHECK(cap_find("e-[Input db has 16 bytes, expected 3072]") != NULL,
	      "a short load must be reported on the wire:\n%s", cap_buf);
	CHECK(mlperf_get_stats()->short_loads == 1u,
	      "short_loads must count it, got %u",
	      (unsigned)mlperf_get_stats()->short_loads);
	CHECK(in_store[0] == 0u,
	      "a refused load must leave the tensor alone, found %u", in_store[0]);
	mlperf_unbind();
	printf("  short load: ok\n");
}

/*
 * The fixed-3 formatter, at the edges the protocol can actually reach.  Driven
 * through `results` rather than `infer` so the output value is ours to choose.
 */
static void test_fixed3(void)
{
	static const struct {
		int8_t      q;
		float       scale;
		int32_t     zp;
		const char *want;
	} v[] = {
		{ -128, 1.0f / 256.0f, -128, "m-results-[0.000,"    },  /* softmax floor   */
		{  127, 1.0f / 256.0f, -128, "m-results-[0.996,"    },  /* softmax ceiling */
		{ -128, 1.0f,             0, "m-results-[-128.000," },  /* negative        */
		{    1, 0.0005f,          0, "m-results-[0.001,"    },  /* rounds up       */
		{    1, 0.0004f,          0, "m-results-[0.000,"    },  /* rounds down     */
	};
	size_t k;

	for (k = 0; k < sizeof v / sizeof v[0]; k++) {
		model_set(DIM_IC, 4, 32 * 32 * 3, 1.0f, -128, 10, v[k].scale, v[k].zp);
		CHECK(mlperf_bind(SH, MODEL) == 0, "bind");
		mlperf_monitor_start();
		out_store[0] = v[k].q;
		cap_reset();
		send("results");
		CHECK(strncmp(cap_buf, v[k].want, strlen(v[k].want)) == 0,
		      "fixed3 case %zu: want prefix %s, got %.40s", k, v[k].want, cap_buf);
		mlperf_unbind();
	}
	printf("  fixed-3 formatting: ok\n");
}

int main(void)
{
	test_identify();
	test_profile_command();
	test_offset_u8("ic01",  set_ic,  32 * 32 * 3);
	test_offset_u8("vww01", set_vww, 96 * 96 * 3);
	test_int8_passthrough();
	test_results_classes();
	test_ad_mse();
	test_short_load();
	test_fixed3();

	printf("test_mlperf: all passed\n");
	return 0;
}
