// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: voxblox/Layer.proto

#include "voxblox/Layer.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)

namespace voxblox {
class LayerProtoDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<LayerProto>
      _instance;
} _LayerProto_default_instance_;
}  // namespace voxblox
namespace protobuf_voxblox_2fLayer_2eproto {
static void InitDefaultsLayerProto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::voxblox::_LayerProto_default_instance_;
    new (ptr) ::voxblox::LayerProto();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::voxblox::LayerProto::InitAsDefaultInstance();
}

::google::protobuf::internal::SCCInfo<0> scc_info_LayerProto =
    {{ATOMIC_VAR_INIT(::google::protobuf::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsLayerProto}, {}};

void InitDefaults() {
  ::google::protobuf::internal::InitSCC(&scc_info_LayerProto.base);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::voxblox::LayerProto, _has_bits_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::voxblox::LayerProto, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::voxblox::LayerProto, voxel_size_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::voxblox::LayerProto, voxels_per_side_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::voxblox::LayerProto, type_),
  1,
  2,
  0,
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::voxblox::LayerProto)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::voxblox::_LayerProto_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  AssignDescriptors(
      "voxblox/Layer.proto", schemas, file_default_instances, TableStruct::offsets,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\023voxblox/Layer.proto\022\007voxblox\"G\n\nLayerP"
      "roto\022\022\n\nvoxel_size\030\001 \001(\001\022\027\n\017voxels_per_s"
      "ide\030\002 \001(\r\022\014\n\004type\030\003 \001(\t"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 103);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "voxblox/Layer.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static ::google::protobuf::internal::once_flag once;
  ::google::protobuf::internal::call_once(once, AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_voxblox_2fLayer_2eproto
namespace voxblox {

// ===================================================================

void LayerProto::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LayerProto::kVoxelSizeFieldNumber;
const int LayerProto::kVoxelsPerSideFieldNumber;
const int LayerProto::kTypeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LayerProto::LayerProto()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  ::google::protobuf::internal::InitSCC(
      &protobuf_voxblox_2fLayer_2eproto::scc_info_LayerProto.base);
  SharedCtor();
  // @@protoc_insertion_point(constructor:voxblox.LayerProto)
}
LayerProto::LayerProto(const LayerProto& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  type_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  if (from.has_type()) {
    type_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.type_);
  }
  ::memcpy(&voxel_size_, &from.voxel_size_,
    static_cast<size_t>(reinterpret_cast<char*>(&voxels_per_side_) -
    reinterpret_cast<char*>(&voxel_size_)) + sizeof(voxels_per_side_));
  // @@protoc_insertion_point(copy_constructor:voxblox.LayerProto)
}

void LayerProto::SharedCtor() {
  type_.UnsafeSetDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  ::memset(&voxel_size_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&voxels_per_side_) -
      reinterpret_cast<char*>(&voxel_size_)) + sizeof(voxels_per_side_));
}

LayerProto::~LayerProto() {
  // @@protoc_insertion_point(destructor:voxblox.LayerProto)
  SharedDtor();
}

void LayerProto::SharedDtor() {
  type_.DestroyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}

void LayerProto::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ::google::protobuf::Descriptor* LayerProto::descriptor() {
  ::protobuf_voxblox_2fLayer_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_voxblox_2fLayer_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LayerProto& LayerProto::default_instance() {
  ::google::protobuf::internal::InitSCC(&protobuf_voxblox_2fLayer_2eproto::scc_info_LayerProto.base);
  return *internal_default_instance();
}


void LayerProto::Clear() {
// @@protoc_insertion_point(message_clear_start:voxblox.LayerProto)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    type_.ClearNonDefaultToEmptyNoArena();
  }
  if (cached_has_bits & 6u) {
    ::memset(&voxel_size_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&voxels_per_side_) -
        reinterpret_cast<char*>(&voxel_size_)) + sizeof(voxels_per_side_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

bool LayerProto::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:voxblox.LayerProto)
  for (;;) {
    ::std::pair<::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional double voxel_size = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {
          set_has_voxel_size();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &voxel_size_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional uint32 voxels_per_side = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {
          set_has_voxels_per_side();
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::uint32, ::google::protobuf::internal::WireFormatLite::TYPE_UINT32>(
                 input, &voxels_per_side_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string type = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadString(
                input, this->mutable_type()));
          ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
            this->type().data(), static_cast<int>(this->type().length()),
            ::google::protobuf::internal::WireFormat::PARSE,
            "voxblox.LayerProto.type");
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:voxblox.LayerProto)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:voxblox.LayerProto)
  return false;
#undef DO_
}

void LayerProto::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:voxblox.LayerProto)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double voxel_size = 1;
  if (cached_has_bits & 0x00000002u) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->voxel_size(), output);
  }

  // optional uint32 voxels_per_side = 2;
  if (cached_has_bits & 0x00000004u) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt32(2, this->voxels_per_side(), output);
  }

  // optional string type = 3;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->type().data(), static_cast<int>(this->type().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "voxblox.LayerProto.type");
    ::google::protobuf::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->type(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:voxblox.LayerProto)
}

::google::protobuf::uint8* LayerProto::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:voxblox.LayerProto)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional double voxel_size = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->voxel_size(), target);
  }

  // optional uint32 voxels_per_side = 2;
  if (cached_has_bits & 0x00000004u) {
    target = ::google::protobuf::internal::WireFormatLite::WriteUInt32ToArray(2, this->voxels_per_side(), target);
  }

  // optional string type = 3;
  if (cached_has_bits & 0x00000001u) {
    ::google::protobuf::internal::WireFormat::VerifyUTF8StringNamedField(
      this->type().data(), static_cast<int>(this->type().length()),
      ::google::protobuf::internal::WireFormat::SERIALIZE,
      "voxblox.LayerProto.type");
    target =
      ::google::protobuf::internal::WireFormatLite::WriteStringToArray(
        3, this->type(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:voxblox.LayerProto)
  return target;
}

size_t LayerProto::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:voxblox.LayerProto)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  if (_has_bits_[0 / 32] & 7u) {
    // optional string type = 3;
    if (has_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::StringSize(
          this->type());
    }

    // optional double voxel_size = 1;
    if (has_voxel_size()) {
      total_size += 1 + 8;
    }

    // optional uint32 voxels_per_side = 2;
    if (has_voxels_per_side()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::UInt32Size(
          this->voxels_per_side());
    }

  }
  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void LayerProto::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:voxblox.LayerProto)
  GOOGLE_DCHECK_NE(&from, this);
  const LayerProto* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LayerProto>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:voxblox.LayerProto)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:voxblox.LayerProto)
    MergeFrom(*source);
  }
}

void LayerProto::MergeFrom(const LayerProto& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:voxblox.LayerProto)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 7u) {
    if (cached_has_bits & 0x00000001u) {
      set_has_type();
      type_.AssignWithDefault(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), from.type_);
    }
    if (cached_has_bits & 0x00000002u) {
      voxel_size_ = from.voxel_size_;
    }
    if (cached_has_bits & 0x00000004u) {
      voxels_per_side_ = from.voxels_per_side_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void LayerProto::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:voxblox.LayerProto)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LayerProto::CopyFrom(const LayerProto& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:voxblox.LayerProto)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LayerProto::IsInitialized() const {
  return true;
}

void LayerProto::Swap(LayerProto* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LayerProto::InternalSwap(LayerProto* other) {
  using std::swap;
  type_.Swap(&other->type_, &::google::protobuf::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(voxel_size_, other->voxel_size_);
  swap(voxels_per_side_, other->voxels_per_side_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
}

::google::protobuf::Metadata LayerProto::GetMetadata() const {
  protobuf_voxblox_2fLayer_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_voxblox_2fLayer_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace voxblox
namespace google {
namespace protobuf {
template<> GOOGLE_PROTOBUF_ATTRIBUTE_NOINLINE ::voxblox::LayerProto* Arena::CreateMaybeMessage< ::voxblox::LayerProto >(Arena* arena) {
  return Arena::CreateInternal< ::voxblox::LayerProto >(arena);
}
}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)
