/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    mlperf_th.cc
 * @brief   The submitter half of the MLPerf Tiny harness (issue #55).  See mlperf_th.h.
 *
 * Everything here implements a function upstream's api/internally_implemented.cpp
 * declares and calls.  The names are not ours; the bodies are.
 *
 * 🔴 C++, AND NOT BY PREFERENCE.  Nothing in this file wants a C++ feature -- it is C
 * with a different extension.  It has to be C++ because of LINKAGE: upstream's
 * api/submitter_implemented.h declares the th_* contract with no `extern "C"`, and
 * api/internally_implemented.h declares the ee_* entry points the same way.  Compiled
 * as C++ (which is what internally_implemented.cpp is), those names are mangled.  A C
 * implementation of th_printf would define an unmangled symbol that upstream's caller
 * never looks for, and the link would fail on every one of them.
 *
 * The alternative -- forcing C linkage onto upstream's declarations from a header
 * injected with -include -- is legal ([dcl.link]/6 keeps the first declaration's
 * linkage) and was rejected: it makes the build depend on a subtlety that is invisible
 * in either file, to save nothing.  Matching upstream's language is the honest fix.
 * The boundary to the rest of the firmware is `extern "C"` in mlperf_th.h, so
 * shell/cmds/cmd_mlperf.c stays plain C and never sees any of this.
 *
 * The gates that made issue #9 keep its buffers in C do not apply here: this file
 * places nothing in a named section, so there is no symbol for
 * check_psram_ai_residency.py or check_dtcm_residency.py to fail to recognise.  It
 * defines no object with a non-trivial constructor or destructor either, which is
 * what check_cxx_runtime.py is watching for.
 */
#include "mlperf_th.h"

#include "api/internally_implemented.h"   /* ee_* -- read-only submodule */
#include "api/submitter_implemented.h"    /* the th_* contract we implement */

#include "cli.h"
#include "fmt.h"
#include "nn.h"
#include "tx_glue.h"

#include <string.h>

/* ------------------------------------------------------------------ *
 *  Which benchmark is loaded
 * ------------------------------------------------------------------ */

/*
 * MLPerf Tiny defines each benchmark by a fixed model, so the input and output shapes
 * ARE the identity -- there is no ambiguity to resolve and nothing to configure.  The
 * table is written from the v1.4 reference models themselves (the trained_models
 * directories under lib/mlperf-tiny/benchmark/training), not from the papers:
 *
 *   ic01   1x32x32x3 int8 -> 1x10   CIFAR-10 ResNet          (ic02 is shape-identical)
 *   kws01  1x49x10x1 int8 -> 1x12   Speech Commands DS-CNN
 *   vww01  1x96x96x3 int8 -> 1x2    Visual Wake Words MobileNet
 *   ad01   1x640     int8 -> 1x640  ToyADMOS deep autoencoder
 *
 * 🔴 THE HOST BYTE FORMAT IS PART OF THE BENCHMARK DEFINITION, NOT OF THE MODEL.  The
 * runner sends ic01/vww01 as unsigned RGB bytes, kws01 as int8 MFCCs it already
 * quantized, and ad01 as little-endian float32 -- documented in upstream's
 * benchmark/evaluation/datasets/README.md.  So `load` below is chosen by BENCHMARK,
 * and deriving it from the tensor's scale/zero_point instead would be wrong in a way
 * that still produces plausible-looking scores.  (ic01's input scale happens to be
 * exactly 1.0 with zp -128, i.e. the offset-binary reading of the same bytes; vww01's
 * is 1/255.  Same byte transform, different scales -- which is the proof that the
 * scale is not what decides it.)
 */
enum mlperf_load {
	LOAD_OFFSET_U8 = 0,   /**< unsigned bytes -> int8, subtract 128       */
	LOAD_INT8,            /**< already int8, copy verbatim                */
	LOAD_F32_QUANT,       /**< float32 -> quantize with the input's s/zp  */
};

enum mlperf_report {
	REPORT_CLASSES = 0,   /**< dequantize every output element            */
	REPORT_MSE,           /**< |output - input|^2 averaged (ad01)         */
};

struct mlperf_bench {
	const char *id;
	uint16_t    dims[NN_MAX_DIMS];
	uint8_t     ndim;
	uint32_t    out_elems;
	uint8_t     load;      /**< enum mlperf_load   */
	uint8_t     report;    /**< enum mlperf_report */
	uint8_t     ready;     /**< pre/post implemented in this build */
};

static const struct mlperf_bench mlperf_table[] = {
	{ "ic01",  { 1u, 32u, 32u, 3u }, 4u,  10u, LOAD_OFFSET_U8,  REPORT_CLASSES, 1u },
	{ "kws01", { 1u, 49u, 10u, 1u }, 4u,  12u, LOAD_INT8,       REPORT_CLASSES, 1u },
	{ "vww01", { 1u, 96u, 96u, 3u }, 4u,   2u, LOAD_OFFSET_U8,  REPORT_CLASSES, 1u },
	{ "ad01",  { 1u, 640u, 1u, 1u }, 2u, 640u, LOAD_F32_QUANT,  REPORT_MSE,     1u },
};

#define MLPERF_BENCH_COUNT (sizeof mlperf_table / sizeof mlperf_table[0])

/* ------------------------------------------------------------------ *
 *  Bound state
 * ------------------------------------------------------------------ */

static struct cli_instance      *mp_sh;
static struct nn_model          *mp_model;
static const struct mlperf_bench *mp_bench;
static struct mlperf_stats       mp_stats;

/*
 * One formatted line at a time.  svc/fmt.c renders into a caller buffer, and going
 * through a buffer rather than a per-character sink keeps a protocol line to one
 * cli_write() -- which matters here because the host is waiting on `m-ready` with a
 * five-second timeout for every one of the 892 `db` commands a vww01 sample takes
 * (27,648 B at the runner's 31 bytes per command).
 *
 * 192 B is comfortably over the longest line the protocol carries (kws01's
 * `m-results-[` + twelve "0.000" + separators + "]\r\n" is about 100), and a
 * truncation is COUNTED rather than ignored: a silently shortened protocol line is
 * exactly the kind of failure that would surface as an unexplainable host-side parse
 * error hours later.  The buffer is static, not on the stack, because the CLI thread
 * runs on 4,096 B and the deepest TFLM kernel frame is already 728 B -- and this
 * layer is single-session by construction, so there is no one to share it with.
 */
static char mp_line[192];

/* Start of the current monitor session, subtracted from every timestamp -- see
 * th_timestamp() for why the protocol's clock is session-relative here. */
static uint64_t mp_epoch_us;

/*
 * ad01's input, kept as the floats the host sent.
 *
 * Anomaly detection is the one benchmark whose SCORE is computed here rather than
 * read out of the model: the deep autoencoder reconstructs its own input, and the
 * result is the mean squared error between the two.  So the float form has to
 * survive th_load_tensor() -- comparing the output against the DEQUANTIZED input
 * instead would subtract the input's quantization error from both sides and report a
 * reconstruction that is better than it was.
 *
 * 2,560 B of .bss rather than a local: the CLI thread runs on 4,096 B, and its
 * measured peak with the rest of this harness is already about half of that.  Sized
 * from the benchmark, and checked against the tensor at load time, so a model with a
 * different element count is refused rather than trusted.
 */
#define MLPERF_AD_ELEMS  640u
static float mp_ad_input[MLPERF_AD_ELEMS];

static uint32_t tensor_elems(const struct nn_tensor *t)
{
	uint32_t n = 1u;
	int i;

	for (i = 0; i < (int)t->ndim && i < NN_MAX_DIMS; i++)
		n *= (uint32_t)t->dims[i];
	return n;
}

static int shape_matches(const struct nn_tensor *t, const struct mlperf_bench *b)
{
	int i;

	if (t->ndim != b->ndim)
		return 0;
	for (i = 0; i < (int)b->ndim; i++)
		if (t->dims[i] != b->dims[i])
			return 0;
	return 1;
}

const char *mlperf_model_id(void)
{
	return (mp_bench != NULL) ? mp_bench->id : "none";
}

const char *mlperf_strerror(int rc)
{
	switch (rc) {
	case 0:                 return "ok";
	case MLPERF_ERR_ARG:    return "no model open";
	case MLPERF_ERR_SHAPE:  return "the loaded model's I/O shape is not an "
	                               "MLPerf Tiny benchmark";
	case MLPERF_ERR_DTYPE:  return "the benchmark is defined on int8 tensors and "
	                               "this model's are not";
	case MLPERF_ERR_TODO:   return "recognised, but this build has no pre/post "
	                               "processing for it yet (issue #55 P3)";
	default:                return "unknown error";
	}
}

int mlperf_bind(struct cli_instance *sh, struct nn_model *m)
{
	const struct nn_tensor *in, *out;
	size_t i;

	mlperf_unbind();

	if (sh == NULL || m == NULL)
		return MLPERF_ERR_ARG;
	if (nn_input_count(m) != 1 || nn_output_count(m) != 1)
		return MLPERF_ERR_SHAPE;

	in  = nn_input(m, 0);
	out = nn_output(m, 0);
	if (in == NULL || out == NULL)
		return MLPERF_ERR_SHAPE;

	for (i = 0u; i < MLPERF_BENCH_COUNT; i++) {
		const struct mlperf_bench *b = &mlperf_table[i];

		if (!shape_matches(in, b) || tensor_elems(out) != b->out_elems)
			continue;

		/*
		 * Shape matched, so we know WHICH benchmark this is -- and can therefore
		 * say precisely what is wrong when it is not runnable.  Every v1.4
		 * reference model has int8 in and out; a float-I/O re-export would take
		 * the offset-binary path below and quietly score noise.
		 */
		if (in->dtype != NN_DTYPE_INT8 || out->dtype != NN_DTYPE_INT8)
			return MLPERF_ERR_DTYPE;
		if (!b->ready)
			return MLPERF_ERR_TODO;

		mp_sh    = sh;
		mp_model = m;
		mp_bench = b;
		return 0;
	}
	return MLPERF_ERR_SHAPE;
}

void mlperf_unbind(void)
{
	mp_sh    = NULL;
	mp_model = NULL;
	mp_bench = NULL;
	memset(&mp_stats, 0, sizeof mp_stats);
}

const struct mlperf_stats *mlperf_get_stats(void)
{
	return &mp_stats;
}

void mlperf_monitor_start(void)
{
	/* Before ee_benchmark_initialize(), which emits a timestamp of its own -- so the
	 * host's very first reading is a small number rather than however long the board
	 * had been up. */
	mp_epoch_us = tx_glue_us64();
	ee_benchmark_initialize();
}

void mlperf_feed(char c)
{
	ee_serial_callback(c);
}

/* ------------------------------------------------------------------ *
 *  Quantization -- needed by BOTH halves of a benchmark, so it sits
 *  above them: th_load_tensor() converts the host's floats in, and
 *  th_results() converts the model's int8 back out.
 * ------------------------------------------------------------------ */

static float dequant(int8_t q, const struct nn_tensor *t)
{
	return t->scale * (float)((int32_t)q - t->zero_point);
}

/*
 * The inverse, matching upstream's QuantizeFloatToInt8()
 * (lib/mlperf-tiny/benchmark/util/quantization_helpers.h) exactly: round(v/scale)
 * plus the zero point, saturated to the int8 range.
 *
 * The rounding is written out rather than calling round(), which is a double-precision
 * libm entry.  Adding +/-0.5 before truncating toward zero IS round-half-away-from-
 * zero, which is what round() specifies -- and this stays in single precision, which
 * is the only kind the FPU does in one instruction here.
 *
 * A zero scale would be a division by zero, and it is reachable: a per-axis quantized
 * tensor reads back scale 0 through this API (issue #51 found that the hard way).
 * mlperf_bind() cannot see it -- the value is per tensor, not per shape -- so the
 * guard lives here, and it yields the zero point, i.e. "no information", rather than
 * an infinity that would propagate into every score.
 */
static int8_t quantize(float v, const struct nn_tensor *t)
{
	int32_t q;

	if (t->scale == 0.0f)
		return (int8_t)t->zero_point;

	v = v / t->scale;
	q = (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f)) + t->zero_point;
	if (q < -128)
		q = -128;
	if (q > 127)
		q = 127;
	return (int8_t)q;
}

/* ------------------------------------------------------------------ *
 *  th_* : console
 * ------------------------------------------------------------------ */

int th_vprintf(const char *format, va_list ap)
{
	int n;

	if (mp_sh == NULL)
		return 0;

	n = fmt_vsnformat(mp_line, sizeof mp_line, format, ap);
	if (n >= (int)sizeof mp_line - 1)
		mp_stats.truncated++;      /* see the note on mp_line */
	if (n > 0 && cli_write(mp_sh, mp_line, (size_t)n) < 0)
		mp_stats.tx_errors++;
	return n;
}

void th_printf(const char *p_fmt, ...)
{
	va_list args;

	va_start(args, p_fmt);
	(void)th_vprintf(p_fmt, args);
	va_end(args);
}

/*
 * Upstream's own main() loop is `while (1) ee_serial_callback(th_getchar());`.  We do
 * not use that loop -- shell/cmds/cmd_mlperf.c drives mlperf_feed() from
 * cli_read_byte(), because the console belongs to the shell and a blocking getchar
 * with no timeout and no kill path has nowhere to return to.  The symbol still has to
 * exist: it is declared in the contract header and referenced by upstream's main.cpp,
 * which we do not compile.  Returning 0 keeps a stray caller from spinning on a
 * plausible character.
 */
char th_getchar(void)
{
	return (char)0;
}

void th_serialport_initialize(void)
{
	/* The console is already up and already raw -- cmd_mlperf.c claimed it before
	 * calling mlperf_monitor_start().  Nothing to open, no baud rate to set: this is
	 * USB CDC, where the host's line settings are decoration. */
}

/* ------------------------------------------------------------------ *
 *  th_* : time
 * ------------------------------------------------------------------ */

void th_timestamp(void)
{
	/*
	 * The host brackets a batch of inferences with two of these and subtracts.  The
	 * wire format is protocol -- upstream's runner matches ^m-lap-us-([0-9]+)$ -- so
	 * the line must not gain a prefix, a suffix or a separator.
	 *
	 * tx_glue_us64() rather than DWT: a batch runs for seconds (ic02 is ~15 s of wall
	 * clock at 25 inferences) and the cycle counter wraps at 7.81 s, so the obvious
	 * high-resolution choice is the one that silently aliases.  See
	 * port/threadx/tx_glue.h for how the two clocks compose.
	 *
	 * 🔴 %llu, NOT upstream's EE_MSG_TIMESTAMP ("...%lu\r\n").  `unsigned long` is
	 * 32 bits on this target, which puts a wrap at 2^32 us = 71 min 34 s of uptime --
	 * and the host does NOT do modulo arithmetic: benchmark/runner/script.py computes
	 * `end_time - start_time` in Python, whose integers are arbitrary precision, so a
	 * batch that straddles the boundary yields a NEGATIVE elapsed time and a garbage
	 * throughput.  An accuracy sweep runs for tens of minutes, so this is a boundary
	 * a real session crosses.  The two format strings differ only in the conversion;
	 * the bytes on the wire are the same digits, and the regex takes any number of
	 * them.
	 *
	 * Rebased to the start of THIS monitor session as well.  That is belt and braces
	 * -- 64 bits would be enough on its own -- but it keeps the numbers small enough
	 * to read in the host's log, and it means anything downstream that does narrow to
	 * 32 bits needs a 71-minute single session before it can go wrong.
	 */
	th_printf("m-lap-us-%llu\r\n",
	          (unsigned long long)(tx_glue_us64() - mp_epoch_us));
}

void th_timestamp_initialize(void)
{
	/* Fixed by the protocol; the host reads it to learn which mode the DUT is in.
	 * Energy mode would drive a GPIO instead of printing, and needs an interface
	 * board this system does not have -- see the issue. */
	th_printf(EE_MSG_TIMESTAMP_MODE);
	th_timestamp();
}

/* ------------------------------------------------------------------ *
 *  th_* : inference
 * ------------------------------------------------------------------ */

void th_final_initialize(void)
{
	/* Upstream's reference builds its interpreter here.  Ours was built by
	 * `ai model load` long before the monitor started, which is the whole reason one
	 * firmware can run every benchmark. */
}

void th_pre(void)  { }
void th_post(void) { }

void th_infer(void)
{
	int rc;

	if (mp_model == NULL)
		return;

	rc = nn_run(mp_model);
	if (rc != 0) {
		mp_stats.infer_errors++;
		/* Say so on the wire.  The host would otherwise time the batch, divide by
		 * the iteration count it asked for, and publish a latency for inferences
		 * that never happened. */
		th_printf("e-[Inference failed: %d]\r\n", rc);
		return;
	}
	mp_stats.inferences++;
	mp_stats.cycles += (uint64_t)nn_last_cycles(mp_model);
}

/*
 * Does the `db` buffer hold the @p want bytes this benchmark needs?  Reports and
 * counts if not.
 *
 * 🔴 ASKED BEFORE ANY COPY, AND THAT IS THE POINT.  ee_get_buffer() copies whatever
 * it has and returns how much -- so checking its RETURN value means the short data is
 * already in the destination.  Upstream can live with that because its reference
 * copies into a scratch array and simply does not forward it; this harness writes
 * straight into the input tensor (a vww01 sample is 27,648 B and the CLI thread has a
 * 4,096 B stack), so the same mistake would leave the tensor half-overwritten with
 * untransformed bytes and the NEXT inference would quietly run on it.
 *
 * Passing NULL is upstream's own documented probe: it skips the memcpy and returns
 * the length it would have copied (api/internally_implemented.cpp).
 */
static int buffer_has(size_t want)
{
	size_t got = ee_get_buffer(NULL, want);

	if (got == want)
		return 1;
	mp_stats.short_loads++;
	th_printf("e-[Input db has %u bytes, expected %u]\r\n",
	          (unsigned)got, (unsigned)want);
	return 0;
}

void th_load_tensor(void)
{
	struct nn_tensor *in;
	size_t want, i;
	uint8_t *p;

	/* Unbound means the monitor is not running and nothing should be touching the
	 * arena.  Checked before nn_input() rather than relying on it: a benchmark that
	 * loaded into a stale tensor pointer would produce numbers, not a fault. */
	if (mp_model == NULL || mp_bench == NULL)
		return;

	in = nn_input(mp_model, 0);
	if (in == NULL || in->data == NULL)
		return;

	p = (uint8_t *)in->data;

	/*
	 * ad01 is the exception: the host sends FLOATS, four bytes per element, so the
	 * transfer is four times the tensor and cannot land in it.  It goes to the float
	 * staging array (which th_results() needs anyway) and is quantized across.
	 */
	if (mp_bench->load == LOAD_F32_QUANT) {
		want = MLPERF_AD_ELEMS * sizeof(float);

		/* The array is sized from the benchmark; this is where that assumption
		 * meets the model that was actually loaded. */
		if (tensor_elems(in) != MLPERF_AD_ELEMS) {
			th_printf("e-[Input tensor has %u elements, expected %u]\r\n",
			          (unsigned)tensor_elems(in), (unsigned)MLPERF_AD_ELEMS);
			return;
		}

		if (!buffer_has(want))
			return;
		(void)ee_get_buffer((uint8_t *)mp_ad_input, want);
		for (i = 0u; i < MLPERF_AD_ELEMS; i++)
			((int8_t *)p)[i] = quantize(mp_ad_input[i], in);
		return;
	}

	want = (size_t)in->bytes;

	/*
	 * Straight into the input tensor -- no intermediate array.  Upstream's reference
	 * stages through a local because its runner API copies out of the harness buffer
	 * anyway; here a vww01 sample would be 27,648 B of it, and the CLI thread has a
	 * 4,096 B stack.
	 *
	 * Writing the tensor directly is safe at THIS point and only at this point:
	 * ee_infer() calls th_load_tensor() before the first th_infer(), so nothing is
	 * reading the arena yet.  Issue #54 is the cautionary tale -- the input tensor's
	 * arena lifetime ENDS AT THE FIRST OPERATOR, so TFLM reuses those bytes for
	 * intermediate activations, and anything that writes them mid-inference feeds the
	 * model half a picture.  A load between two th_infer() calls would be that bug;
	 * the protocol never asks for one.
	 */
	if (!buffer_has(want))
		return;
	(void)ee_get_buffer(p, want);

	switch (mp_bench->load) {
	case LOAD_OFFSET_U8:
		/*
		 * The runner sends unsigned samples; the tensor is int8 with the same
		 * magnitude ordering shifted by 128.  XOR 0x80 IS that subtraction in
		 * two's complement (0 -> -128, 255 -> +127) and needs no signed
		 * conversion, no branch and no second buffer.
		 */
		for (i = 0u; i < want; i++)
			p[i] ^= 0x80u;
		break;
	case LOAD_INT8:
	default:
		/* kws01: the runner already quantized the MFCCs, so the bytes ARE the
		 * tensor's representation and touching them would be the bug. */
		break;
	}
}

/* ------------------------------------------------------------------ *
 *  th_* : results
 * ------------------------------------------------------------------ */

/*
 * Three decimals, built from integers.
 *
 * svc/fmt.c implements no %f and no precision flag -- a deliberate, load-bearing
 * property of this firmware's printf, not a gap to work around by pulling in newlib's
 * float formatter (which would also put an unbounded frame on a 4,096 B stack).  The
 * same idiom is used for quantization scales in shell/cmds/cmd_ai.c and for fps in
 * the camera commands.
 *
 * Saturating rather than wrapping: the protocol wants a number the host's float()
 * will parse, and a wrapped one would parse fine and mean nothing.  The !(v < limit)
 * spelling catches NaN as well, which a plain >= would let through.
 */
static void put_fixed3(float v)
{
	uint32_t milli;
	int neg = 0;

	if (v < 0.0f) {
		neg = 1;
		v = -v;
	}
	if (!(v < 1000000.0f))
		v = 999999.999f;

	milli = (uint32_t)(v * 1000.0f + 0.5f);
	th_printf("%s%lu.%03lu", neg ? "-" : "",
	          (unsigned long)(milli / 1000u), (unsigned long)(milli % 1000u));
}

void th_results(void)
{
	const struct nn_tensor *out;
	const int8_t *q;
	uint32_t n, i;

	if (mp_model == NULL || mp_bench == NULL)
		return;

	out = nn_output(mp_model, 0);
	if (out == NULL || out->data == NULL)
		return;

	/*
	 * The format is fixed: upstream's runner matches ^m-results-\[([^]]+)\]$ and
	 * splits on commas, so the brackets, the separator and the absence of spaces are
	 * all protocol.  Values are the DEQUANTIZED outputs -- the host compares them
	 * against reference float scores, so sending the raw int8 would score as noise.
	 */
	q = (const int8_t *)out->data;
	n = tensor_elems(out);

	th_printf("m-results-[");

	if (mp_bench->report == REPORT_MSE) {
		/*
		 * ad01 does not classify.  The autoencoder reconstructs its input, and
		 * the anomaly score is how badly it managed -- the mean squared error
		 * between the reconstruction and the ORIGINAL float input, which is why
		 * mp_ad_input had to survive th_load_tensor().  The host collects one
		 * such score per window and computes an AUC across the set; it never
		 * sees the 640 output elements.
		 *
		 * Accumulated in double, unlike everything else here -- and unlike
		 * upstream's reference, which uses float
		 * (reference_submissions/anomaly_detection/submitter_implemented.cpp).
		 * 640 squared residuals summed in float lose low-order bits once the
		 * running total outgrows the individual terms, and this number IS the
		 * score: there is no downstream step to absorb the error, the host feeds
		 * it straight into an AUC.  The FPU is double-capable (fpv5-d16), so it
		 * costs instructions rather than a software library.  A deliberate
		 * deviation, and one that can only move the value toward the exact answer.
		 */
		double sum = 0.0;

		if (n > MLPERF_AD_ELEMS)
			n = MLPERF_AD_ELEMS;
		for (i = 0u; i < n; i++) {
			double d = (double)dequant(q[i], out) - (double)mp_ad_input[i];

			sum += d * d;
		}
		put_fixed3((n != 0u) ? (float)(sum / (double)n) : 0.0f);
	} else {
		for (i = 0u; i < n; i++) {
			if (i != 0u)
				th_printf(",");
			put_fixed3(dequant(q[i], out));
		}
	}

	th_printf("]\r\n");
}

/* ------------------------------------------------------------------ *
 *  th_* : parser hook and libc shims
 * ------------------------------------------------------------------ */

void th_command_ready(char volatile *p_command)
{
	/*
	 * Upstream splits "a command arrived" from "run it" so a submitter whose serial
	 * ISR assembles the line can defer the work out of interrupt context.  Ours
	 * already runs in a thread -- cmd_mlperf.c calls mlperf_feed() from the CLI
	 * thread -- so dispatching straight through is correct, and it is what every
	 * reference submission does too.
	 */
	ee_serial_command_parser_callback((char *)p_command);
}

/*
 * The libc shims upstream routes its string handling through, so a platform without a
 * full libc can substitute.  This one has newlib; there is nothing to substitute.
 */
int   th_strncmp(const char *a, const char *b, size_t n) { return strncmp(a, b, n); }
char *th_strncpy(char *d, const char *s, size_t n)       { return strncpy(d, s, n); }
size_t th_strnlen(const char *s, size_t maxlen)          { return strnlen(s, maxlen); }
char *th_strcat(char *d, const char *s)                  { return strcat(d, s); }
char *th_strtok(char *s, const char *sep)                { return strtok(s, sep); }
int   th_atoi(const char *s)                             { return atoi(s); }
void *th_memset(void *b, int c, size_t len)              { return memset(b, c, len); }
void *th_memcpy(void *d, const void *s, size_t n)        { return memcpy(d, s, n); }
