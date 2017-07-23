// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: tactile_pps22_ec.proto

#ifndef PROTOBUF_tactile_5fpps22_5fec_2eproto__INCLUDED
#define PROTOBUF_tactile_5fpps22_5fec_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
#include "component_base.pb.h"
// @@protoc_insertion_point(includes)

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_tactile_5fpps22_5fec_2eproto();
void protobuf_AssignDesc_tactile_5fpps22_5fec_2eproto();
void protobuf_ShutdownFile_tactile_5fpps22_5fec_2eproto();

class M3TactilePPS22EcStatus;
class M3TactilePPS22EcCommand;
class M3TactilePPS22EcParam;

// ===================================================================

class M3TactilePPS22EcStatus : public ::google::protobuf::Message {
 public:
  M3TactilePPS22EcStatus();
  virtual ~M3TactilePPS22EcStatus();

  M3TactilePPS22EcStatus(const M3TactilePPS22EcStatus& from);

  inline M3TactilePPS22EcStatus& operator=(const M3TactilePPS22EcStatus& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const M3TactilePPS22EcStatus& default_instance();

  void Swap(M3TactilePPS22EcStatus* other);

  // implements Message ----------------------------------------------

  M3TactilePPS22EcStatus* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3TactilePPS22EcStatus& from);
  void MergeFrom(const M3TactilePPS22EcStatus& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional .M3BaseStatus base = 1;
  inline bool has_base() const;
  inline void clear_base();
  static const int kBaseFieldNumber = 1;
  inline const ::M3BaseStatus& base() const;
  inline ::M3BaseStatus* mutable_base();
  inline ::M3BaseStatus* release_base();
  inline void set_allocated_base(::M3BaseStatus* base);

  // optional .M3EtherCATStatus ethercat = 2;
  inline bool has_ethercat() const;
  inline void clear_ethercat();
  static const int kEthercatFieldNumber = 2;
  inline const ::M3EtherCATStatus& ethercat() const;
  inline ::M3EtherCATStatus* mutable_ethercat();
  inline ::M3EtherCATStatus* release_ethercat();
  inline void set_allocated_ethercat(::M3EtherCATStatus* ethercat);

  // optional uint64 timestamp = 3;
  inline bool has_timestamp() const;
  inline void clear_timestamp();
  static const int kTimestampFieldNumber = 3;
  inline ::google::protobuf::uint64 timestamp() const;
  inline void set_timestamp(::google::protobuf::uint64 value);

  // repeated int32 taxels = 4;
  inline int taxels_size() const;
  inline void clear_taxels();
  static const int kTaxelsFieldNumber = 4;
  inline ::google::protobuf::int32 taxels(int index) const;
  inline void set_taxels(int index, ::google::protobuf::int32 value);
  inline void add_taxels(::google::protobuf::int32 value);
  inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      taxels() const;
  inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_taxels();

  // @@protoc_insertion_point(class_scope:M3TactilePPS22EcStatus)
 private:
  inline void set_has_base();
  inline void clear_has_base();
  inline void set_has_ethercat();
  inline void clear_has_ethercat();
  inline void set_has_timestamp();
  inline void clear_has_timestamp();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::M3BaseStatus* base_;
  ::M3EtherCATStatus* ethercat_;
  ::google::protobuf::uint64 timestamp_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > taxels_;
  friend void  protobuf_AddDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_AssignDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_ShutdownFile_tactile_5fpps22_5fec_2eproto();

  void InitAsDefaultInstance();
  static M3TactilePPS22EcStatus* default_instance_;
};
// -------------------------------------------------------------------

class M3TactilePPS22EcCommand : public ::google::protobuf::Message {
 public:
  M3TactilePPS22EcCommand();
  virtual ~M3TactilePPS22EcCommand();

  M3TactilePPS22EcCommand(const M3TactilePPS22EcCommand& from);

  inline M3TactilePPS22EcCommand& operator=(const M3TactilePPS22EcCommand& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const M3TactilePPS22EcCommand& default_instance();

  void Swap(M3TactilePPS22EcCommand* other);

  // implements Message ----------------------------------------------

  M3TactilePPS22EcCommand* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3TactilePPS22EcCommand& from);
  void MergeFrom(const M3TactilePPS22EcCommand& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 dummy = 1;
  inline bool has_dummy() const;
  inline void clear_dummy();
  static const int kDummyFieldNumber = 1;
  inline ::google::protobuf::int32 dummy() const;
  inline void set_dummy(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:M3TactilePPS22EcCommand)
 private:
  inline void set_has_dummy();
  inline void clear_has_dummy();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 dummy_;
  friend void  protobuf_AddDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_AssignDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_ShutdownFile_tactile_5fpps22_5fec_2eproto();

  void InitAsDefaultInstance();
  static M3TactilePPS22EcCommand* default_instance_;
};
// -------------------------------------------------------------------

class M3TactilePPS22EcParam : public ::google::protobuf::Message {
 public:
  M3TactilePPS22EcParam();
  virtual ~M3TactilePPS22EcParam();

  M3TactilePPS22EcParam(const M3TactilePPS22EcParam& from);

  inline M3TactilePPS22EcParam& operator=(const M3TactilePPS22EcParam& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const M3TactilePPS22EcParam& default_instance();

  void Swap(M3TactilePPS22EcParam* other);

  // implements Message ----------------------------------------------

  M3TactilePPS22EcParam* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3TactilePPS22EcParam& from);
  void MergeFrom(const M3TactilePPS22EcParam& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional int32 config = 1;
  inline bool has_config() const;
  inline void clear_config();
  static const int kConfigFieldNumber = 1;
  inline ::google::protobuf::int32 config() const;
  inline void set_config(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:M3TactilePPS22EcParam)
 private:
  inline void set_has_config();
  inline void clear_has_config();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 config_;
  friend void  protobuf_AddDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_AssignDesc_tactile_5fpps22_5fec_2eproto();
  friend void protobuf_ShutdownFile_tactile_5fpps22_5fec_2eproto();

  void InitAsDefaultInstance();
  static M3TactilePPS22EcParam* default_instance_;
};
// ===================================================================


// ===================================================================

// M3TactilePPS22EcStatus

// optional .M3BaseStatus base = 1;
inline bool M3TactilePPS22EcStatus::has_base() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3TactilePPS22EcStatus::set_has_base() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3TactilePPS22EcStatus::clear_has_base() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3TactilePPS22EcStatus::clear_base() {
  if (base_ != NULL) base_->::M3BaseStatus::Clear();
  clear_has_base();
}
inline const ::M3BaseStatus& M3TactilePPS22EcStatus::base() const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcStatus.base)
  return base_ != NULL ? *base_ : *default_instance_->base_;
}
inline ::M3BaseStatus* M3TactilePPS22EcStatus::mutable_base() {
  set_has_base();
  if (base_ == NULL) base_ = new ::M3BaseStatus;
  // @@protoc_insertion_point(field_mutable:M3TactilePPS22EcStatus.base)
  return base_;
}
inline ::M3BaseStatus* M3TactilePPS22EcStatus::release_base() {
  clear_has_base();
  ::M3BaseStatus* temp = base_;
  base_ = NULL;
  return temp;
}
inline void M3TactilePPS22EcStatus::set_allocated_base(::M3BaseStatus* base) {
  delete base_;
  base_ = base;
  if (base) {
    set_has_base();
  } else {
    clear_has_base();
  }
  // @@protoc_insertion_point(field_set_allocated:M3TactilePPS22EcStatus.base)
}

// optional .M3EtherCATStatus ethercat = 2;
inline bool M3TactilePPS22EcStatus::has_ethercat() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void M3TactilePPS22EcStatus::set_has_ethercat() {
  _has_bits_[0] |= 0x00000002u;
}
inline void M3TactilePPS22EcStatus::clear_has_ethercat() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void M3TactilePPS22EcStatus::clear_ethercat() {
  if (ethercat_ != NULL) ethercat_->::M3EtherCATStatus::Clear();
  clear_has_ethercat();
}
inline const ::M3EtherCATStatus& M3TactilePPS22EcStatus::ethercat() const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcStatus.ethercat)
  return ethercat_ != NULL ? *ethercat_ : *default_instance_->ethercat_;
}
inline ::M3EtherCATStatus* M3TactilePPS22EcStatus::mutable_ethercat() {
  set_has_ethercat();
  if (ethercat_ == NULL) ethercat_ = new ::M3EtherCATStatus;
  // @@protoc_insertion_point(field_mutable:M3TactilePPS22EcStatus.ethercat)
  return ethercat_;
}
inline ::M3EtherCATStatus* M3TactilePPS22EcStatus::release_ethercat() {
  clear_has_ethercat();
  ::M3EtherCATStatus* temp = ethercat_;
  ethercat_ = NULL;
  return temp;
}
inline void M3TactilePPS22EcStatus::set_allocated_ethercat(::M3EtherCATStatus* ethercat) {
  delete ethercat_;
  ethercat_ = ethercat;
  if (ethercat) {
    set_has_ethercat();
  } else {
    clear_has_ethercat();
  }
  // @@protoc_insertion_point(field_set_allocated:M3TactilePPS22EcStatus.ethercat)
}

// optional uint64 timestamp = 3;
inline bool M3TactilePPS22EcStatus::has_timestamp() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void M3TactilePPS22EcStatus::set_has_timestamp() {
  _has_bits_[0] |= 0x00000004u;
}
inline void M3TactilePPS22EcStatus::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void M3TactilePPS22EcStatus::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
  clear_has_timestamp();
}
inline ::google::protobuf::uint64 M3TactilePPS22EcStatus::timestamp() const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcStatus.timestamp)
  return timestamp_;
}
inline void M3TactilePPS22EcStatus::set_timestamp(::google::protobuf::uint64 value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:M3TactilePPS22EcStatus.timestamp)
}

// repeated int32 taxels = 4;
inline int M3TactilePPS22EcStatus::taxels_size() const {
  return taxels_.size();
}
inline void M3TactilePPS22EcStatus::clear_taxels() {
  taxels_.Clear();
}
inline ::google::protobuf::int32 M3TactilePPS22EcStatus::taxels(int index) const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcStatus.taxels)
  return taxels_.Get(index);
}
inline void M3TactilePPS22EcStatus::set_taxels(int index, ::google::protobuf::int32 value) {
  taxels_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3TactilePPS22EcStatus.taxels)
}
inline void M3TactilePPS22EcStatus::add_taxels(::google::protobuf::int32 value) {
  taxels_.Add(value);
  // @@protoc_insertion_point(field_add:M3TactilePPS22EcStatus.taxels)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
M3TactilePPS22EcStatus::taxels() const {
  // @@protoc_insertion_point(field_list:M3TactilePPS22EcStatus.taxels)
  return taxels_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
M3TactilePPS22EcStatus::mutable_taxels() {
  // @@protoc_insertion_point(field_mutable_list:M3TactilePPS22EcStatus.taxels)
  return &taxels_;
}

// -------------------------------------------------------------------

// M3TactilePPS22EcCommand

// optional int32 dummy = 1;
inline bool M3TactilePPS22EcCommand::has_dummy() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3TactilePPS22EcCommand::set_has_dummy() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3TactilePPS22EcCommand::clear_has_dummy() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3TactilePPS22EcCommand::clear_dummy() {
  dummy_ = 0;
  clear_has_dummy();
}
inline ::google::protobuf::int32 M3TactilePPS22EcCommand::dummy() const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcCommand.dummy)
  return dummy_;
}
inline void M3TactilePPS22EcCommand::set_dummy(::google::protobuf::int32 value) {
  set_has_dummy();
  dummy_ = value;
  // @@protoc_insertion_point(field_set:M3TactilePPS22EcCommand.dummy)
}

// -------------------------------------------------------------------

// M3TactilePPS22EcParam

// optional int32 config = 1;
inline bool M3TactilePPS22EcParam::has_config() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3TactilePPS22EcParam::set_has_config() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3TactilePPS22EcParam::clear_has_config() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3TactilePPS22EcParam::clear_config() {
  config_ = 0;
  clear_has_config();
}
inline ::google::protobuf::int32 M3TactilePPS22EcParam::config() const {
  // @@protoc_insertion_point(field_get:M3TactilePPS22EcParam.config)
  return config_;
}
inline void M3TactilePPS22EcParam::set_config(::google::protobuf::int32 value) {
  set_has_config();
  config_ = value;
  // @@protoc_insertion_point(field_set:M3TactilePPS22EcParam.config)
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_tactile_5fpps22_5fec_2eproto__INCLUDED
