#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to my_srv__srv__Add_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Add_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub color: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Add_Request::default())
  }
}

impl rosidl_runtime_rs::Message for Add_Request {
  type RmwMsg = super::srv::rmw::Add_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        color: msg.color.as_str().into(),
        low_h: msg.low_h,
        low_s: msg.low_s,
        low_v: msg.low_v,
        high_h: msg.high_h,
        high_s: msg.high_s,
        high_v: msg.high_v,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        color: msg.color.as_str().into(),
      low_h: msg.low_h,
      low_s: msg.low_s,
      low_v: msg.low_v,
      high_h: msg.high_h,
      high_s: msg.high_s,
      high_v: msg.high_v,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      color: msg.color.to_string(),
      low_h: msg.low_h,
      low_s: msg.low_s,
      low_v: msg.low_v,
      high_h: msg.high_h,
      high_s: msg.high_s,
      high_v: msg.high_v,
    }
  }
}


// Corresponds to my_srv__srv__Add_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Add_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub success: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for Add_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::Add_Response::default())
  }
}

impl rosidl_runtime_rs::Message for Add_Response {
  type RmwMsg = super::srv::rmw::Add_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        success: msg.success,
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      success: msg.success,
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      success: msg.success,
      message: msg.message.to_string(),
    }
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


