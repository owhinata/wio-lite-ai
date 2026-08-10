/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    verify_tflite.cc
 * @brief   Check a .tflite on the PC before sending it to the board (issue #9 P2c).
 *
 * A HOST tool.  It is never compiled into any firmware; it is built with the host
 * compiler against the SAME tflite-micro tree, at the SAME pinned SHA, that
 * cmake/tflite-micro.cmake already fetched for the `tflm` build.  That matters more
 * than anything else about it:
 *
 *   🔴 IT RUNS tflite::VerifyModelBuffer() ITSELF, not an approximation of it.
 *
 * The firmware can be built with NN_TFLM_VERIFY=ON and pay 22,936 B of a 384 KB
 * partition to run that function on the board.  Running it here instead costs the
 * board nothing and catches the problem EARLIER -- before `blob write` erases a slot
 * and before a ~90 s YMODEM transfer -- with a real error message on a machine that
 * has a screen.  The on-board knob remains for models that arrive from somewhere other
 * than a PC you are sitting at.
 *
 * A hand-written "sanity check" would not have been an acceptable substitute, on the
 * PC any more than on the board.  Partial flatbuffer validation is worse than none: it
 * passes the malformed files it does not happen to cover while reporting that the
 * model was checked.  The only version of this worth having is the identical code.
 *
 * WHAT IT CHECKS, in the order the board would:
 *   1. length, and that it fits a blob slot payload AND the backend's staging buffer
 *   2. the "TFL3" file identifier at offset 4
 *   3. tflite::VerifyModelBuffer() -- the full structural walk
 *   4. the schema version the runtime understands
 *   5. 🔴 that every operator the model uses is one this FIRMWARE registered
 *
 * Step 5 is the one the board cannot do well.  TFLM's AllocateTensors() returns a
 * single kTfLiteError for "an operator is missing" and "the activations do not fit",
 * and with TF_LITE_STRIP_ERROR_STRINGS there is no message to tell them apart -- so
 * `ai model load` has to report both possibilities and let you guess.  Here the
 * missing operator is named.  The list comes from port/nn/tflm/nn_tflm_ops.h, the same
 * header the firmware registers from, so the two cannot drift apart.
 *
 * It also prints the CRC-32/ISO-HDLC of the file, which is what `blob list` and
 * `ai model load` display, so the "is this the file I sent?" question is answered by
 * the same command.
 *
 * Build + run:  cmake --build build-tflm --target verify-model
 *               ./build-tflm/verify_tflite <model.tflite>
 */
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/schema/schema_utils.h"   /* GetBuiltinCode()          */
/* For TFLITE_SCHEMA_VERSION, taken from the runtime's own header rather than restated
 * here.  A restated constant is a constant that can fall out of step with the pinned
 * tree, and this tool's only reason to exist is that it cannot. */
#include "tensorflow/lite/micro/micro_interpreter.h"

#include "nn_tflm_ops.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

/* Mirrors app/blob.h (BLOB_SLOT_SIZE - BLOB_HDR_SIZE) and
 * port/nn/tflm/nn_tflm_priv.h (NN_TFLM_MODEL_MAX).  Restated rather than included
 * because those headers pull in the firmware's world; the numbers are checked against
 * their sources by the message this prints when a model is too big. */
static const uint32_t kBlobPayloadMax = 520192u;
static const uint32_t kStagingMax     = 512u * 1024u;

/* CRC-32/ISO-HDLC, bit by bit -- the same value FlashDB's fdb_calc_crc32() produces on
 * the board and Python's zlib.crc32() produces on this machine.  Written out rather
 * than linked from zlib so this tool builds with nothing but a C++ compiler. */
static uint32_t crc32_iso_hdlc(const uint8_t *p, size_t n)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (size_t i = 0; i < n; i++) {
		crc ^= p[i];
		for (int b = 0; b < 8; b++)
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
	}
	return ~crc;
}

static bool read_file(const char *path, std::vector<uint8_t> &out)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "verify_tflite: cannot open %s\n", path);
		return false;
	}
	uint8_t buf[65536];
	size_t n;
	while ((n = fread(buf, 1, sizeof buf, f)) > 0)
		out.insert(out.end(), buf, buf + n);
	bool ok = (ferror(f) == 0);
	fclose(f);
	if (!ok)
		fprintf(stderr, "verify_tflite: read error on %s\n", path);
	return ok;
}

/* The operators this firmware configuration registers, from the shared X-macro list. */
static const tflite::BuiltinOperator kRegistered[] = {
#define NN_TFLM_OP_ENUM(enum_suffix, method) tflite::BuiltinOperator_##enum_suffix,
	NN_TFLM_OPS_ALL(NN_TFLM_OP_ENUM)
#undef NN_TFLM_OP_ENUM
};

/*
 * Which NN_TFLM_OPS profile this checker was built for.  It is printed beside the
 * operator count, so it has to be derived from the SAME defines that select the list
 * above -- a label that says "blazeface" next to eleven registered operators is the
 * kind of confidently-wrong output this tool exists to prevent elsewhere.
 */
static const char *const kProfileName =
#if defined(NN_TFLM_OPS_EXTENDED)
	"extended";
#elif defined(NN_TFLM_OPS_MLPERF_PROFILE)
	"mlperf";
#else
	"blazeface";
#endif

/*
 * What to tell the operator when a model needs an operator this build did not
 * register: the next profile up, or -- when there is no next profile -- that widening
 * is not the answer.  `extended` is the widest one there is, so advising it from an
 * `extended` build would send someone to rebuild with the settings they already have.
 *
 * NULL means "no wider profile exists"; the REJECT message branches on it.
 */
static const char *const kNextProfile =
#if defined(NN_TFLM_OPS_EXTENDED)
	nullptr;
#elif defined(NN_TFLM_OPS_MLPERF_PROFILE)
	"extended";
#else
	"mlperf";
#endif

static bool is_registered(tflite::BuiltinOperator op)
{
	for (size_t i = 0; i < sizeof kRegistered / sizeof kRegistered[0]; i++)
		if (kRegistered[i] == op)
			return true;
	return false;
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		fprintf(stderr,
		        "usage: verify_tflite <model.tflite>\n"
		        "  Checks a model against THIS firmware configuration before you send\n"
		        "  it with `blob write <slot>` + `sb <model.tflite>`.\n");
		return 2;
	}

	std::vector<uint8_t> buf;
	if (!read_file(argv[1], buf))
		return 2;

	const uint32_t len = (uint32_t)buf.size();
	printf("file    : %s\n", argv[1]);
	printf("size    : %u B\n", len);
	printf("crc32   : %08X   (compare with `blob list` / `ai model load`)\n",
	       crc32_iso_hdlc(buf.data(), buf.size()));

	/* 1. length ---------------------------------------------------------- */
	if (len < 8u) {
		printf("RESULT  : REJECT -- too short to be a flatbuffer (%u B)\n", len);
		return 1;
	}
	if (len > kBlobPayloadMax) {
		printf("RESULT  : REJECT -- %u B exceeds a blob slot payload (%u B).\n"
		       "          Widen BLOB_SLOT_SIZE in app/blob.h, or use a smaller model.\n",
		       len, kBlobPayloadMax);
		return 1;
	}
	if (len > kStagingMax) {
		printf("RESULT  : REJECT -- %u B exceeds the backend staging slot (%u B).\n"
		       "          Widen NN_TFLM_MODEL_MAX in port/nn/tflm/nn_tflm_priv.h.\n",
		       len, kStagingMax);
		return 1;
	}

	/* 2. file identifier -------------------------------------------------- */
	if (!tflite::ModelBufferHasIdentifier(buf.data())) {
		printf("RESULT  : REJECT -- no \"TFL3\" identifier at offset 4; this is not a\n"
		       "          .tflite flatbuffer.\n");
		return 1;
	}

	/* 3. the structural walk ---------------------------------------------- */
	flatbuffers::Verifier verifier(buf.data(), (size_t)len);
	if (!tflite::VerifyModelBuffer(verifier)) {
		printf("RESULT  : REJECT -- the flatbuffer is malformed.\n"
		       "          This is what a file truncated on THIS machine looks like: the\n"
		       "          CRC32 above would still have matched on the board, because the\n"
		       "          transfer would have carried the damage faithfully.\n"
		       "          Check the source of the file (interrupted download, a\n"
		       "          converter still writing, a git-lfs pointer).\n");
		return 1;
	}

	const tflite::Model *model = tflite::GetModel(buf.data());

	/* 4. schema version --------------------------------------------------- */
	if (model->version() != TFLITE_SCHEMA_VERSION) {
		printf("RESULT  : REJECT -- schema version %u, this runtime reads %d.\n",
		       (unsigned)model->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}

	/* 5. operator coverage ------------------------------------------------ */
	std::vector<std::string> missing;
	const auto *codes = model->operator_codes();
	if (codes) {
		for (uint32_t i = 0; i < codes->size(); i++) {
			const tflite::OperatorCode *c = codes->Get(i);
			if (!c)
				continue;
			tflite::BuiltinOperator op = tflite::GetBuiltinCode(c);
			if (op == tflite::BuiltinOperator_CUSTOM) {
				missing.push_back(std::string("CUSTOM:") +
				                  (c->custom_code() ? c->custom_code()->str() : "?"));
			} else if (!is_registered(op)) {
				missing.push_back(tflite::EnumNameBuiltinOperator(op));
			}
		}
	}

	const auto *subgraphs = model->subgraphs();
	printf("schema  : version %u, %u subgraph(s), %u distinct operator(s)\n",
	       (unsigned)model->version(),
	       subgraphs ? (unsigned)subgraphs->size() : 0u,
	       codes ? (unsigned)codes->size() : 0u);
	printf("ops set : %d registered by this build (NN_TFLM_OPS=%s)\n",
	       (int)(sizeof kRegistered / sizeof kRegistered[0]), kProfileName);

	if (!missing.empty()) {
		printf("RESULT  : REJECT -- the model uses %zu operator(s) this build did not\n"
		       "          register:\n", missing.size());
		for (const std::string &m : missing)
			printf("            %s\n", m.c_str());
		printf("          On the board this would fail as NN_MODEL_ERR_ARENA, whose\n"
		       "          message cannot tell a missing operator from an arena that is\n"
		       "          too small -- which is exactly why this check is here.\n");
		if (kNextProfile != nullptr)
			printf("          Rebuild with -DNN_TFLM_OPS=%s, or re-export the model.\n",
			       kNextProfile);
		else
			printf("          This is already the widest NN_TFLM_OPS profile, so the\n"
			       "          operator has to be added to nn_tflm_ops.h or the model\n"
			       "          re-exported without it.\n");
		return 1;
	}

	/*
	 * Deliberately narrow wording.  What was established is: the flatbuffer is
	 * structurally sound, the schema version is one this runtime reads, and every
	 * BUILTIN OPERATOR CODE it references is in the registered set.  What was NOT
	 * established, and cannot be from here, is that each kernel's Prepare() accepts
	 * this model's particular tensor shapes and types, that the operator VERSIONS are
	 * ones these kernels implement, or that the activations fit the arena -- all three
	 * are settled by AllocateTensors() on the board, against the real arena size.
	 *
	 * Saying "safe to send" would overstate that, and an overstated pass is the same
	 * failure this whole tool exists to avoid: a check that reports more confidence
	 * than it earned.
	 */
	printf("RESULT  : PASS -- structurally valid, schema version %d, and no unregistered\n"
	       "          builtin operators.  Remaining checks happen on the board when\n"
	       "          `ai model load` calls AllocateTensors(): operator versions, each\n"
	       "          kernel's shape/type constraints, and whether the activations fit\n"
	       "          the arena.\n", TFLITE_SCHEMA_VERSION);
	return 0;
}
