/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wio Lite AI ThreadX Shell Project
 */
/**
 * @file    tflite_int8_input.cc
 * @brief   Turn a float-input .tflite into an int8-input one (issue #51).
 *
 * A HOST tool, built the same way scripts/verify_tflite.cc is: with the host compiler,
 * against the SAME tflite-micro tree, at the SAME pinned SHA, that cmake/tflite-micro
 * .cmake already fetched for the `tflm` firmware.  Nothing here is ever compiled into
 * a firmware image.
 *
 * WHY THIS EXISTS
 *
 * app/nn_camera.c fills the model's input tensor from the camera.  When that tensor is
 * quantized it does the quantization itself, using the TENSOR'S OWN scale/zero_point --
 * deliberately, because this board's struct nn_tensor carries those parameters and the
 * donor firmware's did not, so the donor had to hardcode (1/128, 0).  That int8 branch
 * had never executed on hardware, which is issue #51.
 *
 * 🔴 It had never executed because "an int8 model" does not mean "an int8 INPUT".  The
 * model this firmware runs -- ST model zoo's blazeface_front_128_int8.tflite -- has
 * int8 WEIGHTS and float32 I/O: its first operator is a QUANTIZE that converts the
 * float input to int8 (scale 1/255, zero point -128), and four DEQUANTIZEs convert the
 * outputs back.  So the graph input is float32 and the int8 path is unreachable, no
 * matter how the board is driven.  That is a property of the FILE, not of the firmware.
 *
 * This tool removes that leading QUANTIZE and makes its output the graph input.  The
 * result has the same weights, the same arithmetic, and the same detections -- the only
 * difference is WHO quantizes: the model, or nncam_rows().  That makes the on-board
 * check a differential one (load slot 0, load slot 1, the same face must appear) rather
 * than "the numbers look plausible", which is the strongest form the check can take.
 *
 * WHAT IT REFUSES, AND WHY IT REFUSES RATHER THAN COPES
 *
 * Removing a tensor means renumbering every tensor index in the file.  The reference
 * sites are enumerable and this tool updates all of them (subgraph inputs/outputs,
 * operator inputs/outputs/intermediates, SignatureDef tensor maps).  What is NOT
 * enumerable is what a METADATA buffer means:
 *
 *   🔴 tflite-micro reads an "OfflineMemoryAllocation" metadata buffer as a tensor
 *   COUNT followed by one offset PER TENSOR INDEX, and fails AllocateTensors() when
 *   the count disagrees with the graph (micro_allocation_info.cc,
 *   GetOfflinePlannedOffsets).  A model carrying it would pass the flatbuffer verifier
 *   after this transform and then fail -- or worse, use another tensor's offset -- on
 *   the board.
 *
 * The flatbuffer verifier checks structure, never meaning, so there is no general way
 * to know whether an unfamiliar metadata buffer is index-sensitive.  Hence an ALLOWLIST
 * of names known to be index-free, and a refusal that names anything else.  Refusing to
 * convert costs a message; converting a file whose metadata now lies costs a debugging
 * session on the board, where the symptom is a single kTfLiteError with no string.
 *
 * The same reasoning applies to debug_metadata_index: this tool does not parse debug
 * metadata, so it will not renumber a graph that references it.
 *
 * 🔴 THE FLATBUFFER BUILDER NEEDS AN EXPLICIT ALLOCATOR HERE.
 * Upstream flatbuffers lets a null allocator mean "use the default one".  The copy
 * vendored into the pinned tflite-micro tree does not -- its Allocate() dereferences
 * the pointer unconditionally (third_party/flatbuffers/include/flatbuffers/
 * default_allocator.h), because tflite-micro does not want a new/delete-based default.
 * A default-constructed FlatBufferBuilder therefore SEGFAULTS on the first Pack(), with
 * a stack that points at the schema and not at the mistake.  Construct it with a
 * DefaultAllocator, as below.
 *
 * WHAT IT DOES NOT DO: it prints no CRC-32.  scripts/verify_tflite.cc prints the value
 * that `blob list` and `ai model load` display, and one number with one source cannot
 * disagree with itself.  Run that on the output -- the closing message says so.
 *
 * Build + run:  cmake --build build-tflm --target int8-input-model
 *               ./build-tflm/tflite_int8_input in.tflite out.tflite
 *               ./build-tflm/verify_tflite out.tflite       # then blob write + sb
 */
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/schema/schema_utils.h"   /* GetBuiltinCode()          */
/*
 * For TFLITE_SCHEMA_VERSION only -- this tool never builds an interpreter.  Taken from
 * the runtime's own header rather than restated, exactly as verify_tflite.cc does.
 * The check matters more here than in a read-only checker: unpacking and repacking
 * sends the model THROUGH this tree's generated schema, and anything the tree does not
 * model is dropped on the way out.
 */
#include "tensorflow/lite/micro/micro_interpreter.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

/*
 * Metadata names known to carry nothing that is indexed by tensor.  An ALLOWLIST and
 * not a denylist of "OfflineMemoryAllocation": the point is that an unrecognised name
 * is exactly the case we cannot reason about, so it must stop the tool rather than
 * slip through it.
 *
 *   min_runtime_version   a version string.  (It may now overstate what the model
 *                         needs, since an operator was removed; that is not a
 *                         correctness problem for any runtime.)
 *   CONVERSION_METADATA   provenance written by the converter.
 *   TFLITE_METADATA       the tflite-support ModelMetadata.  Its input/output tensor
 *                         metadata corresponds to SubGraph.inputs/outputs BY POSITION
 *                         and holds no tensor indices, and this transform changes
 *                         neither the count nor the order of the graph inputs.
 */
static const char *const kIndexFreeMetadata[] = {
	"min_runtime_version",
	"CONVERSION_METADATA",
	"TFLITE_METADATA",
};

static const char *type_name(tflite::TensorType t)
{
	const char *n = tflite::EnumNameTensorType(t);

	return n ? n : "?";
}

static bool read_file(const char *path, std::vector<uint8_t> &out)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "tflite_int8_input: cannot open %s\n", path);
		return false;
	}
	uint8_t buf[65536];
	size_t n;
	while ((n = fread(buf, 1, sizeof buf, f)) > 0)
		out.insert(out.end(), buf, buf + n);
	bool ok = (ferror(f) == 0);
	fclose(f);
	if (!ok)
		fprintf(stderr, "tflite_int8_input: read error on %s\n", path);
	return ok;
}

static bool write_file(const char *path, const uint8_t *p, size_t n)
{
	FILE *f = fopen(path, "wb");
	if (!f) {
		fprintf(stderr, "tflite_int8_input: cannot create %s\n", path);
		return false;
	}
	bool ok = (fwrite(p, 1, n, f) == n);
	if (fclose(f) != 0)
		ok = false;
	if (!ok)
		fprintf(stderr, "tflite_int8_input: write error on %s\n", path);
	return ok;
}

/* "  in[0]  1x128x128x3 FLOAT32" (+ " q(s=... zp=...)" when per-tensor quantized). */
static void print_tensor(const tflite::SubGraphT *sg, int idx, const char *tag)
{
	if (idx < 0 || (size_t)idx >= sg->tensors.size()) {
		printf("  %s  <tensor %d out of range>\n", tag, idx);
		return;
	}

	const tflite::TensorT *t = sg->tensors[(size_t)idx].get();

	printf("  %s  ", tag);
	for (size_t i = 0; i < t->shape.size(); i++)
		printf("%s%d", i ? "x" : "", t->shape[i]);
	printf(" %s", type_name(t->type));

	const tflite::QuantizationParametersT *q = t->quantization.get();
	if (q && q->scale.size() == 1 && q->zero_point.size() == 1)
		printf("  q(s=%.9g zp=%d)", (double)q->scale[0], (int)q->zero_point[0]);
	else if (q && q->scale.size() > 1)
		printf("  q(per-axis, %zu scales)", q->scale.size());
	printf("   tensor %d \"%s\"\n", idx, t->name.c_str());
}

/* Every tensor index above @p removed shifts down by one.  -1 is an omitted optional
 * operator input and must survive as -1. */
static inline int32_t remap(int32_t idx, int32_t removed)
{
	return (idx > removed) ? idx - 1 : idx;
}

/* Does anything in the graph still name tensor @p idx?  Asked after the QUANTIZE is
 * gone, because the deletion that follows renumbers around its absence. */
static bool tensor_referenced(const tflite::ModelT &m, const tflite::SubGraphT *sg,
                              int32_t idx)
{
	for (int32_t t : sg->inputs)
		if (t == idx)
			return true;
	for (int32_t t : sg->outputs)
		if (t == idx)
			return true;
	for (const auto &op : sg->operators) {
		for (int32_t t : op->inputs)
			if (t == idx)
				return true;
		for (int32_t t : op->outputs)
			if (t == idx)
				return true;
		for (int32_t t : op->intermediates)
			if (t == idx)
				return true;
	}
	for (const auto &sd : m.signature_defs) {
		if (sd->subgraph_index != 0u)
			continue;
		for (const auto &tm : sd->inputs)
			if ((int32_t)tm->tensor_index == idx)
				return true;
		for (const auto &tm : sd->outputs)
			if ((int32_t)tm->tensor_index == idx)
				return true;
	}
	return false;
}

int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr,
		        "usage: tflite_int8_input <in.tflite> <out.tflite>\n"
		        "  Rewrites a model whose graph input is float32 followed by a\n"
		        "  QUANTIZE so that the quantized tensor IS the graph input.  Same\n"
		        "  weights, same arithmetic -- the caller quantizes instead of the\n"
		        "  model.  Run verify_tflite on the result before sending it.\n");
		return 2;
	}

	std::vector<uint8_t> buf;
	if (!read_file(argv[1], buf))
		return 2;

	printf("in      : %s (%zu B)\n", argv[1], buf.size());

	/* Do not transform a file we cannot trust.  Same identical VerifyModelBuffer()
	 * verify_tflite runs, and for the same reason: a partial check is worse than
	 * none, because it reports that the file was examined. */
	if (buf.size() < 8u || !tflite::ModelBufferHasIdentifier(buf.data())) {
		printf("RESULT  : REJECT -- not a .tflite flatbuffer (no \"TFL3\" identifier).\n");
		return 1;
	}
	{
		flatbuffers::Verifier v(buf.data(), buf.size());
		if (!tflite::VerifyModelBuffer(v)) {
			printf("RESULT  : REJECT -- the input flatbuffer is malformed.\n"
			       "          Check where the file came from (interrupted download,\n"
			       "          a converter still writing, a git-lfs pointer).\n");
			return 1;
		}
	}

	const tflite::Model *fb = tflite::GetModel(buf.data());
	if (fb->version() != TFLITE_SCHEMA_VERSION) {
		printf("RESULT  : REJECT -- schema version %u, this tree reads %d.\n",
		       (unsigned)fb->version(), TFLITE_SCHEMA_VERSION);
		return 1;
	}

	tflite::ModelT m;
	fb->UnPackTo(&m);

	/* --- refusals, all before anything is modified ----------------------- */

	for (const auto &md : m.metadata) {
		bool known = false;
		for (const char *ok : kIndexFreeMetadata)
			if (md->name == ok)
				known = true;
		if (!known) {
			printf("RESULT  : REJECT -- metadata \"%s\": tensor deletion/remap is not\n"
			       "          supported for this metadata.  This transform renumbers\n"
			       "          tensors, and the flatbuffer verifier checks structure but\n"
			       "          never meaning, so a metadata buffer this tool does not\n"
			       "          recognise could silently start describing the wrong\n"
			       "          tensors.  \"OfflineMemoryAllocation\" is the known case:\n"
			       "          tflite-micro reads it as a tensor count plus one offset\n"
			       "          per tensor index and fails AllocateTensors() on the board.\n",
			       md->name.empty() ? "<unnamed>" : md->name.c_str());
			return 1;
		}
	}

	/* 🔴 A CUSTOM operator's options are an opaque blob.  Nothing in the schema stops
	 * one from holding tensor indices, and nothing in this tool could renumber them --
	 * the same argument as the metadata allowlist above, applied to the other opaque
	 * field in the file.  (verify_tflite rejects CUSTOM anyway, because no custom
	 * kernel is registered; refusing here keeps this tool honest on its own terms
	 * rather than borrowing that guarantee.) */
	if (fb->operator_codes() == nullptr || m.operator_codes.empty()) {
		printf("RESULT  : REJECT -- the model declares no operator codes.\n");
		return 1;
	}
	for (uint32_t i = 0; i < fb->operator_codes()->size(); i++) {
		const tflite::OperatorCode *c = fb->operator_codes()->Get(i);
		if (c && tflite::GetBuiltinCode(c) == tflite::BuiltinOperator_CUSTOM) {
			printf("RESULT  : REJECT -- operator code %u is CUSTOM (\"%s\").  Its\n"
			       "          options are opaque, so this tool cannot know whether\n"
			       "          they carry tensor indices that renumbering would\n"
			       "          invalidate.\n", i,
			       c->custom_code() ? c->custom_code()->c_str() : "?");
			return 1;
		}
	}

	if (m.subgraphs.size() != 1u) {
		printf("RESULT  : REJECT -- %zu subgraphs; this tool rewrites single-subgraph\n"
		       "          models only (indices would have to be remapped per subgraph,\n"
		       "          and control flow operators carry subgraph indices too).\n",
		       m.subgraphs.size());
		return 1;
	}

	tflite::SubGraphT *sg = m.subgraphs[0].get();

	if (sg->debug_metadata_index != -1) {
		printf("RESULT  : REJECT -- the subgraph references debug metadata, which this\n"
		       "          tool does not parse and therefore will not renumber.\n");
		return 1;
	}
	for (const auto &op : sg->operators) {
		if (op->debug_metadata_index != -1) {
			printf("RESULT  : REJECT -- an operator references debug metadata, which\n"
			       "          this tool does not parse and therefore will not renumber.\n");
			return 1;
		}
	}

	if (sg->inputs.size() != 1u) {
		printf("RESULT  : REJECT -- %zu graph inputs; expected exactly 1.\n",
		       sg->inputs.size());
		return 1;
	}

	const int32_t in_idx = sg->inputs[0];
	if (in_idx < 0 || (size_t)in_idx >= sg->tensors.size()) {
		printf("RESULT  : REJECT -- graph input tensor index %d is out of range.\n",
		       (int)in_idx);
		return 1;
	}

	const tflite::TensorT *in_t = sg->tensors[(size_t)in_idx].get();
	if (in_t->type == tflite::TensorType_INT8) {
		printf("RESULT  : REJECT -- the graph input is already INT8; nothing to do.\n");
		return 1;
	}
	if (in_t->type != tflite::TensorType_FLOAT32) {
		printf("RESULT  : REJECT -- the graph input is %s; this transform starts from\n"
		       "          a FLOAT32 input feeding a QUANTIZE.\n", type_name(in_t->type));
		return 1;
	}

	/* The input must be consumed by exactly one operator, and that operator must be
	 * the QUANTIZE whose job we are taking over.  Anything else -- two consumers, a
	 * graph that also exports the float tensor -- means the float tensor is still
	 * needed and this rewrite would change what the model computes. */
	for (int32_t o : sg->outputs) {
		if (o == in_idx) {
			printf("RESULT  : REJECT -- the graph input is also a graph output.\n");
			return 1;
		}
	}

	size_t  q_op    = 0u;
	size_t  n_users = 0u;
	for (size_t i = 0; i < sg->operators.size(); i++)
		for (int32_t t : sg->operators[i]->inputs)
			if (t == in_idx) {
				q_op = i;
				n_users++;
			}
	if (n_users != 1u) {
		printf("RESULT  : REJECT -- the graph input feeds %zu operator inputs; expected\n"
		       "          exactly 1 (a single QUANTIZE).\n", n_users);
		return 1;
	}

	tflite::OperatorT *q = sg->operators[q_op].get();
	/* Bounds-checked before the lookup: the flatbuffer verifier validates structure,
	 * not that an operator's opcode_index names a code the model declares. */
	if (q->opcode_index >= fb->operator_codes()->size()) {
		printf("RESULT  : REJECT -- the first operator's opcode_index (%u) is outside\n"
		       "          the %u declared operator codes.\n",
		       (unsigned)q->opcode_index, (unsigned)fb->operator_codes()->size());
		return 1;
	}
	/* Read the builtin code through the flatbuffer accessor rather than off the
	 * unpacked object: GetBuiltinCode() is what reads the schema's deprecated/current
	 * opcode field pair correctly, and the indices still match because nothing has
	 * been modified yet. */
	const tflite::BuiltinOperator q_code =
	        tflite::GetBuiltinCode(fb->operator_codes()->Get(q->opcode_index));
	if (q_code != tflite::BuiltinOperator_QUANTIZE) {
		printf("RESULT  : REJECT -- the graph input feeds %s, not QUANTIZE.  A model\n"
		       "          with float I/O quantizes at the very first operator; this one\n"
		       "          does something else with the input first.\n",
		       tflite::EnumNameBuiltinOperator(q_code));
		return 1;
	}
	if (q->inputs.size() != 1u || q->outputs.size() != 1u) {
		printf("RESULT  : REJECT -- the QUANTIZE has %zu inputs / %zu outputs; expected\n"
		       "          1 and 1.\n", q->inputs.size(), q->outputs.size());
		return 1;
	}

	const int32_t q_out = q->outputs[0];
	if (q_out < 0 || (size_t)q_out >= sg->tensors.size()) {
		printf("RESULT  : REJECT -- the QUANTIZE output index %d is out of range.\n",
		       (int)q_out);
		return 1;
	}

	const tflite::TensorT *q_t = sg->tensors[(size_t)q_out].get();
	if (q_t->type != tflite::TensorType_INT8) {
		printf("RESULT  : REJECT -- the QUANTIZE produces %s, not INT8.\n",
		       type_name(q_t->type));
		return 1;
	}
	/* 🔴 Per-tensor quantization specifically.  A per-axis input would arrive on the
	 * board with TfLiteTensor::params.scale == 0 (the real parameters live in the
	 * affine quantization struct, which port/nn/nn.h does not expose), and
	 * nn_camera_start() rejects exactly that.  Producing such a model here would be
	 * producing one the firmware is designed to refuse. */
	if (!q_t->quantization || q_t->quantization->scale.size() != 1u ||
	    q_t->quantization->zero_point.size() != 1u) {
		printf("RESULT  : REJECT -- the QUANTIZE output is not per-tensor quantized.\n"
		       "          The firmware needs one scale and one zero point on the input\n"
		       "          (app/nn_camera.c), and a per-axis input reaches it as\n"
		       "          scale 0.\n");
		return 1;
	}
	/* The contract of this tool is "produce a model the firmware will accept", and
	 * nn_camera_start() rejects an int8 input whose scale is not positive.  Checking
	 * it here means that refusal is never the way you find out. */
	if (!(q_t->quantization->scale[0] > 0.0f)) {
		printf("RESULT  : REJECT -- the QUANTIZE output's scale is %.9g; the firmware\n"
		       "          needs a positive per-tensor scale to quantize into it.\n",
		       (double)q_t->quantization->scale[0]);
		return 1;
	}

	printf("before  : %zu tensors, %zu operators, %zu distinct operator code(s)\n",
	       sg->tensors.size(), sg->operators.size(), m.operator_codes.size());
	print_tensor(sg, in_idx, "in [0]");
	for (size_t i = 0; i < sg->outputs.size(); i++) {
		char tag[16];
		snprintf(tag, sizeof tag, "out[%zu]", i);
		print_tensor(sg, sg->outputs[i], tag);
	}

	/* --- the transform ---------------------------------------------------- */

	/* 🔴 Read off the operator BEFORE erasing it: `q` dangles from the next line on,
	 *    and the operator code is needed at the end. */
	const uint32_t q_code_idx = q->opcode_index;

	/* 1. the quantized tensor becomes the graph input, and the QUANTIZE goes away. */
	sg->inputs[0] = q_out;
	sg->operators.erase(sg->operators.begin() + (long)q_op);
	q = nullptr;

	/* 2. a SignatureDef naming the float tensor is expected -- that name is the
	 *    model's public input name.  Point it at the tensor that IS the input now;
	 *    the name is unchanged, so anything driving the model by signature still
	 *    addresses the same thing (it just hands over int8). */
	for (const auto &sd : m.signature_defs) {
		if (sd->subgraph_index != 0u)
			continue;
		for (const auto &tm : sd->inputs)
			if ((int32_t)tm->tensor_index == in_idx)
				tm->tensor_index = (uint32_t)q_out;
	}

	/* 3. the float tensor must now be referenced by nothing.  Checked rather than
	 *    assumed, because everything below renumbers around its absence. */
	if (tensor_referenced(m, sg, in_idx)) {
		printf("RESULT  : REJECT -- the float input tensor is still referenced after\n"
		       "          the QUANTIZE was removed; refusing to renumber around it.\n");
		return 1;
	}

	/* 4. delete the orphan and renumber every tensor index in the file.  The
	 *    reference sites are: subgraph inputs/outputs, operator inputs/outputs/
	 *    intermediates, and SignatureDef tensor maps.  TensorT::buffer is a BUFFER
	 *    index, not a tensor index, so buffers are deliberately left alone -- an
	 *    empty activation buffer costs nothing and pruning it would add a third
	 *    renumbering surface for no gain. */
	sg->tensors.erase(sg->tensors.begin() + (long)in_idx);

	for (int32_t &t : sg->inputs)
		t = remap(t, in_idx);
	for (int32_t &t : sg->outputs)
		t = remap(t, in_idx);
	for (const auto &op : sg->operators) {
		for (int32_t &t : op->inputs)
			t = remap(t, in_idx);
		for (int32_t &t : op->outputs)
			t = remap(t, in_idx);
		for (int32_t &t : op->intermediates)
			t = remap(t, in_idx);
	}
	for (const auto &sd : m.signature_defs) {
		if (sd->subgraph_index != 0u)
			continue;
		for (const auto &tm : sd->inputs)
			tm->tensor_index = (uint32_t)remap((int32_t)tm->tensor_index, in_idx);
		for (const auto &tm : sd->outputs)
			tm->tensor_index = (uint32_t)remap((int32_t)tm->tensor_index, in_idx);
	}

	/* 5. QUANTIZE may now be unused.  Drop its operator code and renumber, so the
	 *    file states the operator set the graph actually needs -- which is what
	 *    verify_tflite reports and what decides whether a firmware build can run it. */
	{
		bool still_used_code = false;

		for (const auto &op : sg->operators)
			if (op->opcode_index == q_code_idx)
				still_used_code = true;

		if (!still_used_code) {
			m.operator_codes.erase(m.operator_codes.begin() + (long)q_code_idx);
			for (const auto &op : sg->operators)
				if (op->opcode_index > q_code_idx)
					op->opcode_index--;
		}
	}

	/* --- repack ----------------------------------------------------------- */

	/* 🔴 The allocator is not optional here -- see the file header. */
	flatbuffers::DefaultAllocator alloc;
	flatbuffers::FlatBufferBuilder fbb(1024, &alloc);
	tflite::FinishModelBuffer(fbb, tflite::Model::Pack(fbb, &m));

	/* Verify what we are about to write, not what we meant to write. */
	{
		flatbuffers::Verifier v(fbb.GetBufferPointer(), fbb.GetSize());
		if (!tflite::VerifyModelBuffer(v)) {
			printf("RESULT  : REJECT -- the rewritten flatbuffer does not verify; "
			       "nothing was written.\n");
			return 1;
		}
	}

	{
		const tflite::Model *out_fb = tflite::GetModel(fbb.GetBufferPointer());
		tflite::ModelT out;
		out_fb->UnPackTo(&out);
		const tflite::SubGraphT *osg = out.subgraphs[0].get();

		printf("after   : %zu tensors, %zu operators, %zu distinct operator code(s)\n",
		       osg->tensors.size(), osg->operators.size(), out.operator_codes.size());
		print_tensor(osg, osg->inputs[0], "in [0]");
		for (size_t i = 0; i < osg->outputs.size(); i++) {
			char tag[16];
			snprintf(tag, sizeof tag, "out[%zu]", i);
			print_tensor(osg, osg->outputs[i], tag);
		}
	}

	if (!write_file(argv[2], fbb.GetBufferPointer(), fbb.GetSize()))
		return 2;

	printf("out     : %s (%u B)\n", argv[2], fbb.GetSize());
	printf("RESULT  : PASS -- the graph input is now the quantized tensor.  The caller\n"
	       "          must apply q = round(real / scale) + zero_point itself; that is\n"
	       "          what app/nn_camera.c does, from the tensor's own parameters.\n"
	       "          Next, for the structural check and the CRC-32 that `blob list`\n"
	       "          and `ai model load` display:\n"
	       "            verify_tflite %s\n", argv[2]);
	return 0;
}
