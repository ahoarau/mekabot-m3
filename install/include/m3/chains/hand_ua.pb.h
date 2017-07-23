// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: hand_ua.proto

#ifndef PROTOBUF_hand_5fua_2eproto__INCLUDED
#define PROTOBUF_hand_5fua_2eproto__INCLUDED

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
void  protobuf_AddDesc_hand_5fua_2eproto();
void protobuf_AssignDesc_hand_5fua_2eproto();
void protobuf_ShutdownFile_hand_5fua_2eproto();

class M3HandUAStatus;
class M3HandUACommand;
class M3HandUAParam;

// ===================================================================

class M3HandUAStatus : public ::google::protobuf::Message {
 public:
  M3HandUAStatus();
  virtual ~M3HandUAStatus();

  M3HandUAStatus(const M3HandUAStatus& from);

  inline M3HandUAStatus& operator=(const M3HandUAStatus& from) {
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
  static const M3HandUAStatus& default_instance();

  void Swap(M3HandUAStatus* other);

  // implements Message ----------------------------------------------

  M3HandUAStatus* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HandUAStatus& from);
  void MergeFrom(const M3HandUAStatus& from);
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

  // repeated double theta_thumb = 2;
  inline int theta_thumb_size() const;
  inline void clear_theta_thumb();
  static const int kThetaThumbFieldNumber = 2;
  inline double theta_thumb(int index) const;
  inline void set_theta_thumb(int index, double value);
  inline void add_theta_thumb(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_thumb() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_thumb();

  // repeated double theta_index = 3;
  inline int theta_index_size() const;
  inline void clear_theta_index();
  static const int kThetaIndexFieldNumber = 3;
  inline double theta_index(int index) const;
  inline void set_theta_index(int index, double value);
  inline void add_theta_index(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_index() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_index();

  // repeated double theta_ring = 4;
  inline int theta_ring_size() const;
  inline void clear_theta_ring();
  static const int kThetaRingFieldNumber = 4;
  inline double theta_ring(int index) const;
  inline void set_theta_ring(int index, double value);
  inline void add_theta_ring(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_ring() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_ring();

  // repeated double theta_pinky = 5;
  inline int theta_pinky_size() const;
  inline void clear_theta_pinky();
  static const int kThetaPinkyFieldNumber = 5;
  inline double theta_pinky(int index) const;
  inline void set_theta_pinky(int index, double value);
  inline void add_theta_pinky(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      theta_pinky() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_theta_pinky();

  // @@protoc_insertion_point(class_scope:M3HandUAStatus)
 private:
  inline void set_has_base();
  inline void clear_has_base();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::M3BaseStatus* base_;
  ::google::protobuf::RepeatedField< double > theta_thumb_;
  ::google::protobuf::RepeatedField< double > theta_index_;
  ::google::protobuf::RepeatedField< double > theta_ring_;
  ::google::protobuf::RepeatedField< double > theta_pinky_;
  friend void  protobuf_AddDesc_hand_5fua_2eproto();
  friend void protobuf_AssignDesc_hand_5fua_2eproto();
  friend void protobuf_ShutdownFile_hand_5fua_2eproto();

  void InitAsDefaultInstance();
  static M3HandUAStatus* default_instance_;
};
// -------------------------------------------------------------------

class M3HandUACommand : public ::google::protobuf::Message {
 public:
  M3HandUACommand();
  virtual ~M3HandUACommand();

  M3HandUACommand(const M3HandUACommand& from);

  inline M3HandUACommand& operator=(const M3HandUACommand& from) {
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
  static const M3HandUACommand& default_instance();

  void Swap(M3HandUACommand* other);

  // implements Message ----------------------------------------------

  M3HandUACommand* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HandUACommand& from);
  void MergeFrom(const M3HandUACommand& from);
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

  // @@protoc_insertion_point(class_scope:M3HandUACommand)
 private:
  inline void set_has_test();
  inline void clear_has_test();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  double test_;
  friend void  protobuf_AddDesc_hand_5fua_2eproto();
  friend void protobuf_AssignDesc_hand_5fua_2eproto();
  friend void protobuf_ShutdownFile_hand_5fua_2eproto();

  void InitAsDefaultInstance();
  static M3HandUACommand* default_instance_;
};
// -------------------------------------------------------------------

class M3HandUAParam : public ::google::protobuf::Message {
 public:
  M3HandUAParam();
  virtual ~M3HandUAParam();

  M3HandUAParam(const M3HandUAParam& from);

  inline M3HandUAParam& operator=(const M3HandUAParam& from) {
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
  static const M3HandUAParam& default_instance();

  void Swap(M3HandUAParam* other);

  // implements Message ----------------------------------------------

  M3HandUAParam* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const M3HandUAParam& from);
  void MergeFrom(const M3HandUAParam& from);
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

  // repeated double flex_factor_thumb = 1;
  inline int flex_factor_thumb_size() const;
  inline void clear_flex_factor_thumb();
  static const int kFlexFactorThumbFieldNumber = 1;
  inline double flex_factor_thumb(int index) const;
  inline void set_flex_factor_thumb(int index, double value);
  inline void add_flex_factor_thumb(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      flex_factor_thumb() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_flex_factor_thumb();

  // repeated double flex_factor_index = 2;
  inline int flex_factor_index_size() const;
  inline void clear_flex_factor_index();
  static const int kFlexFactorIndexFieldNumber = 2;
  inline double flex_factor_index(int index) const;
  inline void set_flex_factor_index(int index, double value);
  inline void add_flex_factor_index(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      flex_factor_index() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_flex_factor_index();

  // repeated double flex_factor_ring = 3;
  inline int flex_factor_ring_size() const;
  inline void clear_flex_factor_ring();
  static const int kFlexFactorRingFieldNumber = 3;
  inline double flex_factor_ring(int index) const;
  inline void set_flex_factor_ring(int index, double value);
  inline void add_flex_factor_ring(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      flex_factor_ring() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_flex_factor_ring();

  // repeated double flex_factor_pinky = 4;
  inline int flex_factor_pinky_size() const;
  inline void clear_flex_factor_pinky();
  static const int kFlexFactorPinkyFieldNumber = 4;
  inline double flex_factor_pinky(int index) const;
  inline void set_flex_factor_pinky(int index, double value);
  inline void add_flex_factor_pinky(double value);
  inline const ::google::protobuf::RepeatedField< double >&
      flex_factor_pinky() const;
  inline ::google::protobuf::RepeatedField< double >*
      mutable_flex_factor_pinky();

  // @@protoc_insertion_point(class_scope:M3HandUAParam)
 private:

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< double > flex_factor_thumb_;
  ::google::protobuf::RepeatedField< double > flex_factor_index_;
  ::google::protobuf::RepeatedField< double > flex_factor_ring_;
  ::google::protobuf::RepeatedField< double > flex_factor_pinky_;
  friend void  protobuf_AddDesc_hand_5fua_2eproto();
  friend void protobuf_AssignDesc_hand_5fua_2eproto();
  friend void protobuf_ShutdownFile_hand_5fua_2eproto();

  void InitAsDefaultInstance();
  static M3HandUAParam* default_instance_;
};
// ===================================================================


// ===================================================================

// M3HandUAStatus

// optional .M3BaseStatus base = 1;
inline bool M3HandUAStatus::has_base() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3HandUAStatus::set_has_base() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3HandUAStatus::clear_has_base() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3HandUAStatus::clear_base() {
  if (base_ != NULL) base_->::M3BaseStatus::Clear();
  clear_has_base();
}
inline const ::M3BaseStatus& M3HandUAStatus::base() const {
  // @@protoc_insertion_point(field_get:M3HandUAStatus.base)
  return base_ != NULL ? *base_ : *default_instance_->base_;
}
inline ::M3BaseStatus* M3HandUAStatus::mutable_base() {
  set_has_base();
  if (base_ == NULL) base_ = new ::M3BaseStatus;
  // @@protoc_insertion_point(field_mutable:M3HandUAStatus.base)
  return base_;
}
inline ::M3BaseStatus* M3HandUAStatus::release_base() {
  clear_has_base();
  ::M3BaseStatus* temp = base_;
  base_ = NULL;
  return temp;
}
inline void M3HandUAStatus::set_allocated_base(::M3BaseStatus* base) {
  delete base_;
  base_ = base;
  if (base) {
    set_has_base();
  } else {
    clear_has_base();
  }
  // @@protoc_insertion_point(field_set_allocated:M3HandUAStatus.base)
}

// repeated double theta_thumb = 2;
inline int M3HandUAStatus::theta_thumb_size() const {
  return theta_thumb_.size();
}
inline void M3HandUAStatus::clear_theta_thumb() {
  theta_thumb_.Clear();
}
inline double M3HandUAStatus::theta_thumb(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAStatus.theta_thumb)
  return theta_thumb_.Get(index);
}
inline void M3HandUAStatus::set_theta_thumb(int index, double value) {
  theta_thumb_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAStatus.theta_thumb)
}
inline void M3HandUAStatus::add_theta_thumb(double value) {
  theta_thumb_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAStatus.theta_thumb)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAStatus::theta_thumb() const {
  // @@protoc_insertion_point(field_list:M3HandUAStatus.theta_thumb)
  return theta_thumb_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAStatus::mutable_theta_thumb() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAStatus.theta_thumb)
  return &theta_thumb_;
}

// repeated double theta_index = 3;
inline int M3HandUAStatus::theta_index_size() const {
  return theta_index_.size();
}
inline void M3HandUAStatus::clear_theta_index() {
  theta_index_.Clear();
}
inline double M3HandUAStatus::theta_index(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAStatus.theta_index)
  return theta_index_.Get(index);
}
inline void M3HandUAStatus::set_theta_index(int index, double value) {
  theta_index_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAStatus.theta_index)
}
inline void M3HandUAStatus::add_theta_index(double value) {
  theta_index_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAStatus.theta_index)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAStatus::theta_index() const {
  // @@protoc_insertion_point(field_list:M3HandUAStatus.theta_index)
  return theta_index_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAStatus::mutable_theta_index() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAStatus.theta_index)
  return &theta_index_;
}

// repeated double theta_ring = 4;
inline int M3HandUAStatus::theta_ring_size() const {
  return theta_ring_.size();
}
inline void M3HandUAStatus::clear_theta_ring() {
  theta_ring_.Clear();
}
inline double M3HandUAStatus::theta_ring(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAStatus.theta_ring)
  return theta_ring_.Get(index);
}
inline void M3HandUAStatus::set_theta_ring(int index, double value) {
  theta_ring_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAStatus.theta_ring)
}
inline void M3HandUAStatus::add_theta_ring(double value) {
  theta_ring_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAStatus.theta_ring)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAStatus::theta_ring() const {
  // @@protoc_insertion_point(field_list:M3HandUAStatus.theta_ring)
  return theta_ring_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAStatus::mutable_theta_ring() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAStatus.theta_ring)
  return &theta_ring_;
}

// repeated double theta_pinky = 5;
inline int M3HandUAStatus::theta_pinky_size() const {
  return theta_pinky_.size();
}
inline void M3HandUAStatus::clear_theta_pinky() {
  theta_pinky_.Clear();
}
inline double M3HandUAStatus::theta_pinky(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAStatus.theta_pinky)
  return theta_pinky_.Get(index);
}
inline void M3HandUAStatus::set_theta_pinky(int index, double value) {
  theta_pinky_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAStatus.theta_pinky)
}
inline void M3HandUAStatus::add_theta_pinky(double value) {
  theta_pinky_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAStatus.theta_pinky)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAStatus::theta_pinky() const {
  // @@protoc_insertion_point(field_list:M3HandUAStatus.theta_pinky)
  return theta_pinky_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAStatus::mutable_theta_pinky() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAStatus.theta_pinky)
  return &theta_pinky_;
}

// -------------------------------------------------------------------

// M3HandUACommand

// optional double test = 1;
inline bool M3HandUACommand::has_test() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void M3HandUACommand::set_has_test() {
  _has_bits_[0] |= 0x00000001u;
}
inline void M3HandUACommand::clear_has_test() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void M3HandUACommand::clear_test() {
  test_ = 0;
  clear_has_test();
}
inline double M3HandUACommand::test() const {
  // @@protoc_insertion_point(field_get:M3HandUACommand.test)
  return test_;
}
inline void M3HandUACommand::set_test(double value) {
  set_has_test();
  test_ = value;
  // @@protoc_insertion_point(field_set:M3HandUACommand.test)
}

// -------------------------------------------------------------------

// M3HandUAParam

// repeated double flex_factor_thumb = 1;
inline int M3HandUAParam::flex_factor_thumb_size() const {
  return flex_factor_thumb_.size();
}
inline void M3HandUAParam::clear_flex_factor_thumb() {
  flex_factor_thumb_.Clear();
}
inline double M3HandUAParam::flex_factor_thumb(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAParam.flex_factor_thumb)
  return flex_factor_thumb_.Get(index);
}
inline void M3HandUAParam::set_flex_factor_thumb(int index, double value) {
  flex_factor_thumb_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAParam.flex_factor_thumb)
}
inline void M3HandUAParam::add_flex_factor_thumb(double value) {
  flex_factor_thumb_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAParam.flex_factor_thumb)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAParam::flex_factor_thumb() const {
  // @@protoc_insertion_point(field_list:M3HandUAParam.flex_factor_thumb)
  return flex_factor_thumb_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAParam::mutable_flex_factor_thumb() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAParam.flex_factor_thumb)
  return &flex_factor_thumb_;
}

// repeated double flex_factor_index = 2;
inline int M3HandUAParam::flex_factor_index_size() const {
  return flex_factor_index_.size();
}
inline void M3HandUAParam::clear_flex_factor_index() {
  flex_factor_index_.Clear();
}
inline double M3HandUAParam::flex_factor_index(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAParam.flex_factor_index)
  return flex_factor_index_.Get(index);
}
inline void M3HandUAParam::set_flex_factor_index(int index, double value) {
  flex_factor_index_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAParam.flex_factor_index)
}
inline void M3HandUAParam::add_flex_factor_index(double value) {
  flex_factor_index_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAParam.flex_factor_index)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAParam::flex_factor_index() const {
  // @@protoc_insertion_point(field_list:M3HandUAParam.flex_factor_index)
  return flex_factor_index_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAParam::mutable_flex_factor_index() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAParam.flex_factor_index)
  return &flex_factor_index_;
}

// repeated double flex_factor_ring = 3;
inline int M3HandUAParam::flex_factor_ring_size() const {
  return flex_factor_ring_.size();
}
inline void M3HandUAParam::clear_flex_factor_ring() {
  flex_factor_ring_.Clear();
}
inline double M3HandUAParam::flex_factor_ring(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAParam.flex_factor_ring)
  return flex_factor_ring_.Get(index);
}
inline void M3HandUAParam::set_flex_factor_ring(int index, double value) {
  flex_factor_ring_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAParam.flex_factor_ring)
}
inline void M3HandUAParam::add_flex_factor_ring(double value) {
  flex_factor_ring_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAParam.flex_factor_ring)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAParam::flex_factor_ring() const {
  // @@protoc_insertion_point(field_list:M3HandUAParam.flex_factor_ring)
  return flex_factor_ring_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAParam::mutable_flex_factor_ring() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAParam.flex_factor_ring)
  return &flex_factor_ring_;
}

// repeated double flex_factor_pinky = 4;
inline int M3HandUAParam::flex_factor_pinky_size() const {
  return flex_factor_pinky_.size();
}
inline void M3HandUAParam::clear_flex_factor_pinky() {
  flex_factor_pinky_.Clear();
}
inline double M3HandUAParam::flex_factor_pinky(int index) const {
  // @@protoc_insertion_point(field_get:M3HandUAParam.flex_factor_pinky)
  return flex_factor_pinky_.Get(index);
}
inline void M3HandUAParam::set_flex_factor_pinky(int index, double value) {
  flex_factor_pinky_.Set(index, value);
  // @@protoc_insertion_point(field_set:M3HandUAParam.flex_factor_pinky)
}
inline void M3HandUAParam::add_flex_factor_pinky(double value) {
  flex_factor_pinky_.Add(value);
  // @@protoc_insertion_point(field_add:M3HandUAParam.flex_factor_pinky)
}
inline const ::google::protobuf::RepeatedField< double >&
M3HandUAParam::flex_factor_pinky() const {
  // @@protoc_insertion_point(field_list:M3HandUAParam.flex_factor_pinky)
  return flex_factor_pinky_;
}
inline ::google::protobuf::RepeatedField< double >*
M3HandUAParam::mutable_flex_factor_pinky() {
  // @@protoc_insertion_point(field_mutable_list:M3HandUAParam.flex_factor_pinky)
  return &flex_factor_pinky_;
}


// @@protoc_insertion_point(namespace_scope)

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_hand_5fua_2eproto__INCLUDED
