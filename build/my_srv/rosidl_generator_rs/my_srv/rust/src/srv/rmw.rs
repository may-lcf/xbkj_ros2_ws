#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "my_srv__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_srv__srv__Add_Request() -> *const std::ffi::c_void;
}

#[link(name = "my_srv__rosidl_generator_c")]
extern "C" {
    fn my_srv__srv__Add_Request__init(msg: *mut Add_Request) -> bool;
    fn my_srv__srv__Add_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Add_Request>, size: usize) -> bool;
    fn my_srv__srv__Add_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Add_Request>);
    fn my_srv__srv__Add_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Add_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<Add_Request>) -> bool;
}

// Corresponds to my_srv__srv__Add_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Add_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub color: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub low_h: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub low_s: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub low_v: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub high_h: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub high_s: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub high_v: i32,

}



impl Default for Add_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_srv__srv__Add_Request__init(&mut msg as *mut _) {
        panic!("Call to my_srv__srv__Add_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Add_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Add_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Add_Request where Self: Sized {
  const TYPE_NAME: &'static str = "my_srv/srv/Add_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_srv__srv__Add_Request() }
  }
}


#[link(name = "my_srv__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__my_srv__srv__Add_Response() -> *const std::ffi::c_void;
}

#[link(name = "my_srv__rosidl_generator_c")]
extern "C" {
    fn my_srv__srv__Add_Response__init(msg: *mut Add_Response) -> bool;
    fn my_srv__srv__Add_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<Add_Response>, size: usize) -> bool;
    fn my_srv__srv__Add_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<Add_Response>);
    fn my_srv__srv__Add_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<Add_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<Add_Response>) -> bool;
}

// Corresponds to my_srv__srv__Add_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Add_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for Add_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !my_srv__srv__Add_Response__init(&mut msg as *mut _) {
        panic!("Call to my_srv__srv__Add_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for Add_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { my_srv__srv__Add_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for Add_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for Add_Response where Self: Sized {
  const TYPE_NAME: &'static str = "my_srv/srv/Add_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__my_srv__srv__Add_Response() }
  }
}






#[link(name = "my_srv__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__my_srv__srv__Add() -> *const std::ffi::c_void;
}

// Corresponds to my_srv__srv__Add
#[allow(missing_docs, non_camel_case_types)]
pub struct Add;

impl rosidl_runtime_rs::Service for Add {
    type Request = Add_Request;
    type Response = Add_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__my_srv__srv__Add() }
    }
}


