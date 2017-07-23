// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: led_matrix_ec_shm.proto

#ifndef PROTOBUF_led_5fmatrix_5fec_5fshm_2eproto__INCLUDED
#define PROTOBUF_led_5fmatrix_5fec_5fshm_2eproto__INCLUDED

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
void  protobuf_AddDesc_led_5fmatrix_5fec_5fshm_2eproto();
void protobuf_AssignDesc_led_5fmatrix_5fec_5fshm_2eproto();
void protobuf_ShutdownFile_led_5fmatrix_5fec_5fshm_2eproto();

class M3LedMatrixEcShmStatus;
class M3LedMatrixEcShmCommand;
class M3LedMatrixEcShmParam;

// ===================================================================

class M3LedMatrixEcShmStatus : public ::google::protobuf::Message {
 public:
  M3LedMatrixEcShmStatus();
  virtual ~M3LedMatrixEcShmStatus();

  M3LedMatrixEcShmStatus(const M3LedMatrixEcShmStatus& from);

  inline M3LedMatrixEcShmStatus& operator=(const M3LedMatrixEcShmStatus& from) {
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
  static const M3LedMatrixEcShmStatus& default_instance();

  void Swap(M3LedMatrixEcShmStatus* other);

  // implements Message ----------------------------------------------

  M3LedMatrixEcShmStatus* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3LedMatrixEcShmStatus& from);
  void MergeFrom(const M3LedMatrixEcShmStatus& from);
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

  // optional double test = 2;
  inline bool has_test() const;
  inline void clear_test();
  static const int kTestFieldNumber = 2;
  inline double test() const;
  inline void set_test(double value);

  // @@protoc_insertion_point(class_scope:M3LedMatrixEcShmStatus)
 private:
  inline void set_has_base();
  inline void clear_has_base();
  inline void set_has_test();
  inline void clear_has_test();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::M3BaseStatus* base_;
  double test_;
  friend void  protobuf_AddDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_AssignDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_ShutdownFile_led_5fmatrix_5fec_5fshm_2eproto();

  void InitAsDefaultInstance();
  static M3LedMatrixEcShmStatus* default_instance_;
};
// -------------------------------------------------------------------

class M3LedMatrixEcShmCommand : public ::google::protobuf::Message {
 public:
  M3LedMatrixEcShmCommand();
  virtual ~M3LedMatrixEcShmCommand();

  M3LedMatrixEcShmCommand(const M3LedMatrixEcShmCommand& from);

  inline M3LedMatrixEcShmCommand& operator=(const M3LedMatrixEcShmCommand& from) {
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
  static const M3LedMatrixEcShmCommand& default_instance();

  void Swap(M3LedMatrixEcShmCommand* other);

  // implements Message ----------------------------------------------

  M3LedMatrixEcShmCommand* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3LedMatrixEcShmCommand& from);
  void MergeFrom(const M3LedMatrixEcShmCommand& from);
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

  // optional double test = 1;
  inline bool has_test() const;
  inline void clear_test();
  static const int kTestFieldNumber = 1;
  inline double test() const;
  inline void set_test(double value);

  // @@protoc_insertion_point(class_scope:M3LedMatrixEcShmCommand)
 private:
  inline void set_has_test();
  inline void clear_has_test();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double test_;
  friend void  protobuf_AddDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_AssignDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_ShutdownFile_led_5fmatrix_5fec_5fshm_2eproto();

  void InitAsDefaultInstance();
  static M3LedMatrixEcShmCommand* default_instance_;
};
// -------------------------------------------------------------------

class M3LedMatrixEcShmParam : public ::google::protobuf::Message {
 public:
  M3LedMatrixEcShmParam();
  virtual ~M3LedMatrixEcShmParam();

  M3LedMatrixEcShmParam(const M3LedMatrixEcShmParam& from);

  inline M3LedMatrixEcShmParam& operator=(const M3LedMatrixEcShmParam& from) {
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
  static const M3LedMatrixEcShmParam& default_instance();

  void Swap(M3LedMatrixEcShmParam* other);

  // implements Message ----------------------------------------------

  M3LedMatrixEcShmParam* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3LedMatrixEcShmParam& from);
  void MergeFrom(const M3LedMatrixEcShmParam& from);
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

  // optional double test = 1;
  inline bool has_test() const;
  inline void clear_test();
  static const int kTestFieldNumber = 1;
  inline double test() const;
  inline void set_test(double value);

  // @@protoc_insertion_point(class_scope:M3LedMatrixEcShmParam)
 private:
  inline void set_has_test();
  inline void clear_has_test();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double test_;
  friend void  protobuf_AddDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_AssignDesc_led_5fmatrix_5fec_5fshm_2eproto();
  friend void protobuf_ShutdownFile_led_5fmatrix_5fec_5fshm_2eproto();

  void InitAsDefaultInstance();
  static M3LedMatrixEcShmParam* default_instance_;
};
// ===================================================================


// ===================================================================

// M3LedMatrixEcShmStatus

// optional .M3BaseStatus base = 1;
inline bool M3LedMatrixEcShmStatus::has_base() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3LedMatrixEcShmStatus::set_has_base() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3LedMatrixEcShmStatus::clear_has_base() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3LedMatrixEcShmStatus::clear_base() {
  if (base_ != NULL) base_->::M3BaseStatus::Clear();
  clear_has_base();
}
inline const ::M3BaseStatus& M3LedMatrixEcShmStatus::base() const {
  // @@protoc_insertion_point(field_get:M3LedMatrixEcShmStatus.base)
  return base_ != NULL ? *base_ : *default_instance_->base_;
}
inline ::M3BaseStatus* M3LedMatrixEcShmStatus::mutable_base() {
  set_has_base();
  if (base_ == NULL) base_ = new ::M3BaseStatus;
  // @@protoc_insertion_point(field_mutable:M3LedMatrixEcShmStatus.base)
  return base_;
}
inline ::M3BaseStatus* M3LedMatrixEcShmStatus::release_base() {
  clear_has_base();
  ::M3BaseStatus* temp = base_;
  base_ = NULL;
  return temp;
}
inline void M3LedMatrixEcShmStatus::set_allocated_base(::M3BaseStatus* base) {
  delete base_;
  base_ = base;
  if (base) {
    set_has_base();
  } else {
    clear_has_base();
  }
  // @@protoc_insertion_point(field_set_allocated:M3LedMatrixEcShmStatus.base)
}

// optional double test = 2;
inline bool M3LedMatrixEcShmStatus::has_test() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void M3LedMatrixEcShmStatus::set_has_test() {
  _has_bits_[0] |= 0x00000002u;
}
inline void M3LedMatrixEcShmStatus::clear_has_test() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void M3LedMatrixEcShmStatus::clear_test() {
  test_ = 0;
  clear_has_test();
}
inline double M3LedMatrixEcShmStatus::test() const {
  // @@protoc_insertion_point(field_get:M3LedMatrixEcShmStatus.test)
  return test_;
}
inline void M3LedMatrixEcShmStatus::set_test(double value) {
  set_has_test();
  test_ = value;
  // @@protoc_insertion_point(field_set:M3LedMatrixEcShmStatus.test)
}

// -------------------------------------------------------------------

// M3LedMatrixEcShmCommand

// optional double test = 1;
inline bool M3LedMatrixEcShmCommand::has_test() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3LedMatrixEcShmCommand::set_has_test() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3LedMatrixEcShmCommand::clear_has_test() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3LedMatrixEcShmCommand::clear_test() {
  test_ = 0;
  clear_has_test();
}
inline double M3LedMatrixEcShmCommand::test() const {
  // @@protoc_insertion_point(field_get:M3LedMatrixEcShmCommand.test)
  return test_;
}
inline void M3LedMatrixEcShmCommand::set_test(double value) {
  set_has_test();
  test_ = value;
  // @@protoc_insertion_point(field_set:M3LedMatrixEcShmCommand.test)
}

// -------------------------------------------------------------------

// M3LedMatrixEcShmParam

// optional double test = 1;
inline bool M3LedMatrixEcShmParam::has_test() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3LedMatrixEcShmParam::set_has_test() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3LedMatrixEcShmParam::clear_has_test() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3LedMatrixEcShmParam::clear_test() {
  test_ = 0;
  clear_has_test();
}
inline double M3LedMatrixEcShmParam::test() const {
  // @@protoc_insertion_point(field_get:M3LedMatrixEcShmParam.test)
  return test_;
}
inline void M3LedMatrixEcShmParam::set_test(double value) {
  set_has_test();
  test_ = value;
  // @@protoc_insertion_point(field_set:M3LedMatrixEcShmParam.test)
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_led_5fmatrix_5fec_5fshm_2eproto__INCLUDED
