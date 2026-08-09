/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    nn_tflm.cc
 * @brief   TensorFlow Lite Micro backend for the nn layer (issue #9 phase 2c).
 *
 * Bridges a TFLM MicroInterpreter to the backend-agnostic vtable in nn_backend.h.
 * Compiled only when CONFIG_NN_BACKEND=tflm, into the `tflm` static library alongside
 * the tflite-micro + CMSIS-NN tree that cmake/tflite-micro.cmake fetches and generates
 * at configure time.
 *
 * 🔴 THERE IS NO BUILT-IN MODEL, AND THAT IS THE CENTRAL DESIGN POINT OF THIS FILE.
 * The donor firmware compiles a BlazeFace flatbuffer into flash as a C array, so its
 * backend always has something to run and reload(NULL) means "go back to the built-in
 * one".  That is not available here: the app partition is 384 KB with ~109 KB free,
 * the model is 189 KB, and issue #9 decided models live in the external NOR's blob
 * region precisely so that changing one costs a transfer instead of an internal-flash
 * erase cycle (~10k lifetime, see app/blob.h).
 *
 * So open() succeeds with NO MODEL: zero inputs, zero outputs, the name "(none)".
 * Three things fall out of that, all of them wanted:
 *
 *   - `ai info` works on a board that has never been given a model, and reports the
 *     backend honestly instead of failing to open.
 *   - The nn_backend_vt contract that open() must not touch tensor bodies -- because
 *     `ai info` is called with the PSRAM down -- is satisfied structurally rather than
 *     by being careful: with no model there are no tensors to touch, and the arena is
 *     first written by AllocateTensors() inside `ai model load`, which does take the
 *     PSRAM guard.
 *   - reload(NULL, ...) is an UNLOAD rather than a revert, which is the only meaning
 *     it can have with nothing built in.
 *
 * WHAT RUNS WHEN.  Nothing in this file has a global constructor: the arena and the
 * model slots are plain C arrays (nn_tflm_bufs.c), the interpreter is placement-new'd
 * into a static buffer the first time a model is loaded, and the op resolver is a
 * function-local static.  Static constructors would run from __libc_init_array, before
 * the MPU, the caches, the PSRAM window, `dmesg` and ThreadX all exist -- one that
 * touched the carve-out would hang the board leaving nothing behind.
 * cmake/check_cxx_runtime.py fails the build if any appears.
 *
 * Clean-room bridge over a third-party runtime; the structure follows the donor
 * firmware ../stm32f746g-disco, which has been running this pin on a Cortex-M7 since
 * its issue #88.
 */
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "nn.h"
#include "nn_backend.h"
#include "nn_tflm_priv.h"
#include "nn_tflm_ops.h"

#include <new>
#include <cstddef>
#include <cstdint>

/* Compile-time canary.  CMSIS-NN's int8 kernels only take their SIMD path when the
 * compiler advertises the DSP extension; Cortex-M7 is armv7e-m, so GCC defines
 * __ARM_FEATURE_DSP.  If an -mcpu/-mfpu change ever drops it, the build must fail
 * loudly rather than quietly falling back to CMSIS-NN's plain-C path -- which would
 * look like nothing more than a disappointing benchmark, four times slower, with no
 * indication of why. */
#if defined(NN_TFLM_CMSIS_NN) && !defined(__ARM_FEATURE_DSP)
#error "CMSIS-NN build without __ARM_FEATURE_DSP -- the SIMD int8 path is not selected"
#endif

namespace {

/* The op set the resolver registers, taken from the shared list in nn_tflm_ops.h so
 * the host-side checker (scripts/verify_tflite.cc) cannot disagree with what this
 * firmware will actually run.  Sized from the list too, rather than by a hand-written
 * constant that could fall behind it.
 *
 * This is a knob and not simply "register everything" because the donor firmware
 * measured the widening from 8 ops to 23 at +97,056 B of flash.  On this partition
 * that is most of the free space -- so the default is the narrow set and widening is a
 * configure-time decision someone makes on purpose. */
using OpResolver = tflite::MicroMutableOpResolver<NN_TFLM_OPS_TOTAL>;

/* The model description handed out as the opaque backend handle.  A plain aggregate,
 * so it lands in .bss with no constructor -- see the file header. */
struct tflm_model {
	struct nn_tensor in[NN_MAX_IO];
	struct nn_tensor out[NN_MAX_IO];
	int      n_in;
	int      n_out;
	uint32_t used;      /* arena_used_bytes() after AllocateTensors()   */
	const char *name;   /* always &g_model_name[0]                      */
	bool     open;
};

tflm_model g_tm;
char       g_model_name[64];

/* The flatbuffer the live interpreter is reading, or nullptr when nothing is loaded.
 * g_active_slot is which nn_tflm_model_buf slot that is, or -1 for none. */
const void *g_active_model;
uint32_t    g_active_len;
int         g_active_slot = -1;

/* Storage for the interpreter, constructed in place on first load.  Not a pointer to
 * heap and not a global object: the first would need an allocator this firmware
 * refuses the backend, the second would need a constructor at __libc_init_array time. */
alignas(tflite::MicroInterpreter) uint8_t g_interp_buf[sizeof(tflite::MicroInterpreter)];
tflite::MicroInterpreter *g_interp = nullptr;

uint8_t map_dtype(TfLiteType t)
{
	switch (t) {
	case kTfLiteInt8:    return NN_DTYPE_INT8;
	case kTfLiteUInt8:   return NN_DTYPE_UINT8;
	case kTfLiteInt16:   return NN_DTYPE_INT16;
	case kTfLiteInt32:   return NN_DTYPE_INT32;
	case kTfLiteFloat32: return NN_DTYPE_FLOAT32;
	default:             return NN_DTYPE_NONE;
	}
}

void fill_tensor(struct nn_tensor *nt, const TfLiteTensor *tt)
{
	int nd = tt->dims ? tt->dims->size : 0;

	nt->data  = tt->data.data;
	nt->bytes = (uint32_t)tt->bytes;
	nt->ndim  = (uint8_t)(nd <= NN_MAX_DIMS ? nd : NN_MAX_DIMS);
	for (int i = 0; i < NN_MAX_DIMS; i++)
		nt->dims[i] = (i < nd) ? (uint16_t)tt->dims->data[i] : 1;
	nt->dtype      = map_dtype(tt->type);
	nt->scale      = tt->params.scale;
	nt->zero_point = tt->params.zero_point;
}

/* Copy a display name into backend-owned storage.  nullptr / "" means no model, and
 * the name is owned here rather than borrowed so `ai info` can never end up printing
 * through a pointer into a command's argv. */
void set_model_name(const char *name)
{
	const int cap = (int)sizeof(g_model_name) - 1;
	const char *src = (name && name[0]) ? name : "(none)";
	int n = 0;

	while (src[n] && n < cap) {
		g_model_name[n] = src[n];
		n++;
	}
	g_model_name[n] = '\0';
	g_tm.name = g_model_name;
}

/* No model: zero tensors, no arena, the name "(none)".  This is a legitimate steady
 * state here, not an error -- see the file header. */
void publish_empty()
{
	g_tm.n_in  = 0;
	g_tm.n_out = 0;
	g_tm.used  = 0u;
	set_model_name(nullptr);
}

/*
 * The op resolver, constructed on first use into storage that is never destroyed.
 *
 * 🔴 NOT a function-local `static OpResolver resolver;`, which is the obvious way to
 * write this and is what the donor firmware does.  MicroMutableOpResolver<N> has a
 * non-trivial destructor, so GCC emits a registration for it -- and because this build
 * uses -fno-use-cxa-atexit, that registration goes to plain `atexit()` rather than
 * `__cxa_atexit()`.  Measured on the first link: nn_tflm.cc.obj pulled newlib's
 * atexit, __register_exitproc and __call_exitprocs into the image, plus a struct
 * _atexit in AXI-SRAM, to run a destructor at a program exit that cannot happen --
 * nothing in this firmware ever returns from main().
 *
 * Placement-new into a plain array removes the destructor from the program entirely,
 * which is also the truthful description of this object's lifetime.  It matches how
 * the interpreter below is built, and it keeps the C++ runtime surface at exactly the
 * six operator new/delete forms in cxx_runtime.cc.  cmake/check_cxx_runtime.py fails
 * the build if `atexit` reappears.
 *
 * Constructed on first use, never before main(): a global would run its constructor
 * from __libc_init_array with no MPU, no cache and no usable PSRAM window.  No
 * __cxa_guard_* is needed (-fno-threadsafe-statics) because every path that reaches
 * here holds the single NN session guard -- port/nn/nn.h requires the caller to hold
 * it across a model swap, and a model swap is the only thing that calls this.
 */
alignas(OpResolver) uint8_t g_resolver_buf[sizeof(OpResolver)];
OpResolver *g_resolver = nullptr;

OpResolver *get_resolver()
{
	if (g_resolver)
		return g_resolver;

	OpResolver &resolver = *new (g_resolver_buf) OpResolver();
	bool ok = true;

	/* AddXxx() returns kTfLiteError only when the resolver's slots are exhausted, which
	 * cannot happen here -- NN_TFLM_OPS_TOTAL counts the very list being expanded -- so
	 * any failure is a build-time mistake and is fatal for the whole backend rather
	 * than for one model. */
#define NN_TFLM_REGISTER(enum_suffix, method) \
	ok = ok && (resolver.method() == kTfLiteOk);
	NN_TFLM_OPS_ALL(NN_TFLM_REGISTER)
#undef NN_TFLM_REGISTER

	if (ok) {
		g_resolver = &resolver;
		return g_resolver;
	}
	/* Slots exhausted: leave g_resolver null so this is retried rather than a
	 * half-filled resolver being handed out.  Nothing to destroy -- the object has no
	 * destructor in this program, by construction (see above). */
	return nullptr;
}

/* ~MicroInterpreter() is NOT trivial -- it calls graph_.FreeSubgraphs() -- so a
 * rebuild must destroy the previous one before placement-new reuses the storage. */
void destroy_interp()
{
	if (g_interp) {
		g_interp->~MicroInterpreter();
		g_interp = nullptr;
	}
}

/*
 * Build an interpreter for @p data (@p len bytes) and publish its I/O into g_tm.
 *
 * On failure it leaves g_interp destroyed and g_tm UNTOUCHED -- which means g_tm's
 * tensor pointers then refer to an interpreter that no longer exists.  That is why
 * every caller of this function on the failure path must rebuild something before
 * returning: see tflm_bk_reload().
 *
 * Returns 0 or one of the NN_MODEL_ERR_* codes in nn.h.
 */
int build_interp(const void *data, uint32_t len)
{
	const uint8_t *bytes = static_cast<const uint8_t *>(data);

	/* Validation, cheapest first.  The blob's CRC32 has already been checked against
	 * the copy in this buffer by the caller, which proves the bytes arrived intact --
	 * but NOT that they are a whole .tflite.  A file truncated on the PC before it was
	 * sent is stored faithfully and its CRC matches perfectly, so the structural
	 * checks below are the only thing standing between that and a fault inside
	 * GetModel(). */
	if (len < 8u)
		return NN_MODEL_ERR_EMPTY;
	/* Flatbuffer file identifier, bytes 4..7 of any .tflite. */
	if (bytes[4] != 'T' || bytes[5] != 'F' || bytes[6] != 'L' || bytes[7] != '3')
		return NN_MODEL_ERR_FORMAT;

#if NN_TFLM_VERIFY
	/* The full structural walk: every offset, vector length and table in the buffer
	 * checked against `len` before anything dereferences them.  This is what catches
	 * the truncated-on-the-PC case above, and it is a separate CMake knob only because
	 * it is the largest single item in this backend's flash cost. */
	{
		flatbuffers::Verifier verifier(bytes, (size_t)len);
		if (!tflite::VerifyModelBuffer(verifier))
			return NN_MODEL_ERR_FORMAT;
	}
#endif

	const tflite::Model *model = tflite::GetModel(data);
	if (model->version() != TFLITE_SCHEMA_VERSION)
		return NN_MODEL_ERR_VERSION;

	OpResolver *resolver = get_resolver();
	if (!resolver)
		return NN_MODEL_ERR_OPS;

	destroy_interp();
	g_interp = new (g_interp_buf) tflite::MicroInterpreter(
		model, *resolver, nn_tflm_arena, (size_t)NN_TFLM_ARENA_BYTES);

	/* Where a model that needs an unregistered op fails, and where one whose
	 * activations do not fit NN_TFLM_ARENA_BYTES fails.  TFLM does not distinguish the
	 * two in its return value; with TF_LITE_STRIP_ERROR_STRINGS there is no message
	 * either, which is the price paid for not linking MicroPrintf and stdio. */
	if (g_interp->AllocateTensors() != kTfLiteOk) {
		destroy_interp();
		return NN_MODEL_ERR_ARENA;
	}

	size_t n_in  = g_interp->inputs_size();
	size_t n_out = g_interp->outputs_size();
	if (n_in < 1u || n_in > (size_t)NN_MAX_IO ||
	    n_out < 1u || n_out > (size_t)NN_MAX_IO) {
		destroy_interp();
		return NN_MODEL_ERR_SHAPE;
	}
	/* Every tensor present, with a real buffer and a rank this API can describe.  Fail
	 * here rather than let ai_print_tensor() or a caller's indexing walk off the end
	 * of nn_tensor::dims later. */
	for (size_t i = 0; i < n_in; i++) {
		TfLiteTensor *t = g_interp->input(i);
		if (!t || !t->data.data || !t->dims || t->dims->size > NN_MAX_DIMS) {
			destroy_interp();
			return NN_MODEL_ERR_TENSOR;
		}
	}
	for (size_t i = 0; i < n_out; i++) {
		TfLiteTensor *t = g_interp->output(i);
		if (!t || !t->data.data || !t->dims || t->dims->size > NN_MAX_DIMS) {
			destroy_interp();
			return NN_MODEL_ERR_TENSOR;
		}
	}

	/*
	 * Publish counts-last, and zero them first.
	 *
	 * `ai info` reads these descriptors WITHOUT the NN session guard -- deliberately,
	 * because nn_backend.h promises open() and the descriptor accessors work with the
	 * PSRAM down, and making an inspector wait on a running benchmark would be a poor
	 * trade.  So a second console can be walking g_tm while this runs.  Filling the
	 * arrays with the counts still describing the OLD model would let it print one
	 * model's shapes under another's name.  Zeroed first, the only states it can
	 * observe are "no model" and the complete new one.
	 *
	 * This is ordering, not atomicity: it is worth exactly what it claims, and the
	 * reason it is enough is that nothing here is dereferenced.  ai_print_tensor()
	 * reads dims/dtype/scale out of the descriptor and never follows ->data, so a
	 * torn read costs a confusing line, not a fault.
	 */
	g_tm.n_in  = 0;
	g_tm.n_out = 0;
	for (size_t i = 0; i < n_in; i++)
		fill_tensor(&g_tm.in[i], g_interp->input(i));
	for (size_t i = 0; i < n_out; i++)
		fill_tensor(&g_tm.out[i], g_interp->output(i));
	g_tm.used  = (uint32_t)g_interp->arena_used_bytes();
	g_tm.n_in  = (int)n_in;
	g_tm.n_out = (int)n_out;
	return 0;
}

}  /* namespace */

/*
 * The vtable callbacks.  Internal linkage plus C language linkage, so their addresses
 * have C function-pointer type and match struct nn_backend_vt exactly.
 */
extern "C" {

static int tflm_bk_init(void)
{
	return 0;   /* TFLM has no global runtime to bring up */
}

/* Opens with no model -- see the file header for why that is the normal state and not
 * a degraded one.  Touches no tensor body, so `ai info` works with the PSRAM down. */
static int tflm_bk_open(void **impl_out)
{
	if (!g_tm.open) {
		publish_empty();
		g_tm.open = true;
	}
	*impl_out = &g_tm;
	return 0;
}

static void tflm_bk_close(void *impl)
{
	(void)impl;   /* singleton: the interpreter and its arena outlive any one command */
}

/* Hand out the slot that is NOT live, so filling it cannot touch the flatbuffer the
 * running interpreter still holds pointers into.  With one slot a corrupt download
 * would take the working model down with it; with two, the worst case is a message. */
static int tflm_bk_load_region(void **buf, uint32_t *cap)
{
	int inactive = (g_active_slot == 0) ? 1 : 0;

	*buf = nn_tflm_model_buf[inactive];
	*cap = NN_TFLM_MODEL_MAX;
	return 0;
}

/*
 * Adopt a model, or unload.  Transactional in both directions: on any rejection the
 * previously active model is rebuilt and reported, so a bad file costs a message and
 * nothing else.
 *
 * *impl_out is ALWAYS the resulting active handle -- the new model, or the restored
 * previous one -- and NULL only if even the restore failed.
 */
static int tflm_bk_reload(const void *data, uint32_t len, const char *name,
                          void **impl_out)
{
	const void *old_model = g_active_model;
	uint32_t    old_len   = g_active_len;
	int         old_slot  = g_active_slot;
	int         new_slot;
	int         rc;

	if (data == nullptr) {                    /* unload */
		destroy_interp();
		g_active_model = nullptr;
		g_active_len   = 0u;
		g_active_slot  = -1;
		publish_empty();
		g_tm.open = true;
		*impl_out = &g_tm;
		return 0;
	}

	/* Only our own staging slots are accepted.  The alternative -- interpreting from
	 * wherever the caller points -- would put the flatbuffer's lifetime and alignment
	 * in the caller's hands, and TFLM keeps pointers INTO the model for as long as the
	 * interpreter lives.  Refusing here is what makes the double-buffering above mean
	 * anything. */
	if (data == nn_tflm_model_buf[0])
		new_slot = 0;
	else if (data == nn_tflm_model_buf[1])
		new_slot = 1;
	else {
		*impl_out = &g_tm;                /* nothing changed */
		return NN_MODEL_ERR_SLOT;
	}
	/* Never adopt the slot the LIVE model is in.  load_region() hands out the inactive
	 * one, so reaching here means the caller used an answer from before some other
	 * swap -- and by now the caller has already written its download over the
	 * flatbuffer this interpreter is reading, so the model is gone either way.  Say so
	 * rather than rebuild from bytes that are half one model and half another; the
	 * transactional restore below would otherwise "succeed" on corrupt input.  The
	 * caller-side rule that prevents this is stated in port/nn/nn.h: hold the NN
	 * session across load_region() AND reload(). */
	if (old_model != nullptr && new_slot == old_slot) {
		*impl_out = &g_tm;
		return NN_MODEL_ERR_SLOT;
	}
	if (len == 0u || len > NN_TFLM_MODEL_MAX) {
		*impl_out = &g_tm;
		return NN_MODEL_ERR_EMPTY;
	}

	rc = build_interp(data, len);
	if (rc == 0) {
		/* Published only after the interpreter exists AND its I/O was accepted. */
		g_active_model = data;
		g_active_len   = len;
		g_active_slot  = new_slot;
		set_model_name(name);
		g_tm.open = true;
		*impl_out = &g_tm;
		return 0;
	}

	/* Rejected.  build_interp() has destroyed the interpreter, so g_tm currently
	 * describes tensors that no longer exist -- something must be rebuilt before
	 * returning, whatever the outcome. */
	if (old_model == nullptr) {           /* nothing was loaded before: stay empty */
		g_active_len  = 0u;
		g_active_slot = -1;
		publish_empty();
		g_tm.open = true;
		*impl_out = &g_tm;
		return rc;
	}

	/* The previous model's bytes are untouched (it lives in the OTHER slot), so this
	 * rebuild should not fail.  g_model_name still holds its name -- only success
	 * paths overwrite it -- so re-pointing g_tm.name is all the naming needed. */
	if (build_interp(old_model, old_len) == 0) {
		g_active_model = old_model;
		g_active_len   = old_len;
		g_active_slot  = old_slot;
		g_tm.name      = g_model_name;
		g_tm.open      = true;
		*impl_out      = &g_tm;
		return rc;                    /* report why the NEW model was refused */
	}

	/* Even the known-good model could not be rebuilt.  Leave the handle closed so a
	 * later nn_model_open() retries from scratch rather than handing out descriptors
	 * for an interpreter that is gone. */
	g_active_model = nullptr;
	g_active_len   = 0u;
	g_active_slot  = -1;
	g_tm.open      = false;
	*impl_out      = nullptr;
	return rc;
}

static const char *tflm_bk_name(void *impl)
{
	return ((struct tflm_model *)impl)->name;
}

static int tflm_bk_in_count(void *impl)  { return ((struct tflm_model *)impl)->n_in; }
static int tflm_bk_out_count(void *impl) { return ((struct tflm_model *)impl)->n_out; }

static struct nn_tensor *tflm_bk_input(void *impl, int idx)
{
	struct tflm_model *m = (struct tflm_model *)impl;

	return (idx >= 0 && idx < m->n_in) ? &m->in[idx] : nullptr;
}

static struct nn_tensor *tflm_bk_output(void *impl, int idx)
{
	struct tflm_model *m = (struct tflm_model *)impl;

	return (idx >= 0 && idx < m->n_out) ? &m->out[idx] : nullptr;
}

/* The arena TFLM actually PLANNED, not the NN_TFLM_ARENA_BYTES reservation.  The
 * reservation is a build-time guess; this is the number that says whether it was a
 * good one, and the only one worth printing. */
static uint32_t tflm_bk_acts_bytes(void *impl)
{
	return ((struct tflm_model *)impl)->used;
}

static int tflm_bk_run(void *impl)
{
	(void)impl;

	if (!g_interp)
		return NN_ERR_STATE;   /* open, but no model loaded */
	return (g_interp->Invoke() == kTfLiteOk) ? 0 : -1;
}

static uint32_t tflm_bk_heap_allocs(void)
{
	return nn_tflm_cxx_new_calls;
}

/* The version string carries the build's POSTURE, not just its identity.  Whether the
 * flatbuffer verifier is compiled in changes what a failed `ai model load` can mean, so
 * `ai info` should not require reading CMakeCache.txt to find out.  With it off, the
 * equivalent check lives on the PC (scripts/verify_tflite.cc) and someone has to have
 * run it -- which is exactly the kind of assumption worth printing. */
#if defined(NN_TFLM_CMSIS_NN)
#  if NN_TFLM_VERIFY
static const struct nn_backend_info g_info = { "tflm", "tflite-micro + CMSIS-NN" };
#  else
static const struct nn_backend_info g_info = {
	"tflm", "tflite-micro + CMSIS-NN, no on-board model verify" };
#  endif
#else
#  if NN_TFLM_VERIFY
static const struct nn_backend_info g_info = { "tflm", "tflite-micro (reference kernels)" };
#  else
static const struct nn_backend_info g_info = {
	"tflm", "tflite-micro (reference kernels), no on-board model verify" };
#  endif
#endif

/*
 * The one definition of the selected backend.  Inside extern "C" this is a DEFINITION
 * with external linkage even though it is const: a declaration directly contained in a
 * linkage specification is treated as if it carried `extern` ([dcl.link]), and the
 * initializer makes it a definition.  (The same rule is what makes `extern "C"
 * uint8_t buf[N];` a mere declaration -- which is why the carve-out buffers are
 * defined in C.  See nn_tflm_priv.h.)
 *
 * The designated initializers are a GCC extension in C++17 (they are standard only
 * from C++20) and are used knowingly, despite CXX_EXTENSIONS OFF asking for strict
 * -std=c++17.  The alternative is positional initialization of fourteen function
 * pointers whose types are largely interchangeable -- where inserting a field in
 * nn_backend.h silently shifts every entry after it and the build stays green.  A
 * documented extension that the compiler checks beats a correct-looking list nobody
 * can verify by eye.  GCC accepts in-declaration-order designators; it rejects
 * out-of-order ones, which is the case this relies on.
 */
const struct nn_backend_vt nn_backend_vt_selected = {
	.info              = &g_info,
	.init              = tflm_bk_init,
	.open              = tflm_bk_open,
	.close             = tflm_bk_close,
	.model_name        = tflm_bk_name,
	.in_count          = tflm_bk_in_count,
	.out_count         = tflm_bk_out_count,
	.input             = tflm_bk_input,
	.output            = tflm_bk_output,
	.activations_bytes = tflm_bk_acts_bytes,
	.run               = tflm_bk_run,
	.load_region       = tflm_bk_load_region,
	.reload            = tflm_bk_reload,
	.heap_allocs       = tflm_bk_heap_allocs,
};

}  /* extern "C" */
