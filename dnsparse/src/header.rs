use core::fmt;
use core::mem::{size_of, transmute};

#[derive(Clone)]
#[repr(C)]
pub struct DnsHeader {
  id: [u8; 2],
  flags: [u8; 2],
  question_count: [u8; 2],
  answer_count: [u8; 2],
  name_server_count: [u8; 2],
  additional_records_count: [u8; 2],
}

#[derive(Debug, PartialEq)]
pub enum HeaderKind {
  Query,
  Response,
}

#[derive(Debug, PartialEq)]
pub enum OpCode {
  Query,
  InverseQuery,
  Status,
  Notify,
  Update,
  Reserved(u8)
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ResponseCode {
  NoError,
  FormatError,
  ServerFailure,
  NonExistentDomain,
  NotImplemented,
  Refused,
  ExistentDomain,
  ExistentRrSet,
  NonExistentRrSet,
  NotAuthoritative,
  NotZone,
  BadOptVersion,
  BadSignature,
  BadKey,
  BadTime,
  BadMode,
  BadName,
  BadAlg,
  Reserved(u16),
}

impl From<ResponseCode> for u16 {
  fn from(r: ResponseCode) -> Self {
    match r {
      ResponseCode::NoError => 0,
      ResponseCode::FormatError => 1,
      ResponseCode::ServerFailure => 2,
      ResponseCode::NonExistentDomain => 3,
      ResponseCode::NotImplemented => 4,
      ResponseCode::Refused => 5,
      ResponseCode::ExistentDomain => 6,
      ResponseCode::ExistentRrSet => 7,
      ResponseCode::NonExistentRrSet => 8,
      ResponseCode::NotAuthoritative => 9,
      ResponseCode::NotZone => 10,
      ResponseCode::BadOptVersion => 16,
      ResponseCode::BadSignature => 16,
      ResponseCode::BadKey => 17,
      ResponseCode::BadTime => 18,
      ResponseCode::BadMode => 19,
      ResponseCode::BadName => 20,
      ResponseCode::BadAlg => 21,
      ResponseCode::Reserved(n) => n
    }
  }
}

impl From<u16> for ResponseCode {
  fn from(n: u16) -> Self {
    match n {
      0 => ResponseCode::NoError,
      1 => ResponseCode::FormatError,
      2 => ResponseCode::ServerFailure,
      3 => ResponseCode::NonExistentDomain,
      4 => ResponseCode::NotImplemented,
      5 => ResponseCode::Refused,
      6 => ResponseCode::ExistentDomain,
      7 => ResponseCode::ExistentRrSet,
      8 => ResponseCode::NonExistentRrSet,
      9 => ResponseCode::NotAuthoritative,
      10 => ResponseCode::NotZone,
      16 => ResponseCode::BadOptVersion,
      16 => ResponseCode::BadSignature,
      17 => ResponseCode::BadKey,
      18 => ResponseCode::BadTime,
      19 => ResponseCode::BadMode,
      20 => ResponseCode::BadName,
      21 => ResponseCode::BadAlg,
      n => ResponseCode::Reserved(n),
    }
  }
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum QueryKind {
  A = 1,
  NS = 2,
  MD = 3,
  MF = 4,
  CNAME = 5,
  SOA = 6,
  MB = 7,
  MG = 8,
  MR = 9,
  NULL = 10,
  WKS = 11,
  PTR = 12,
  HINFO = 13,
  MINFO = 14,
  MX = 15,
  TXT = 16,
  AXFR = 252,
  MAILA = 253,
  MAILB = 254,
  ALL = 255,
  Reserved,
}

impl From<u16> for QueryKind {
  fn from(n: u16) -> Self {
    unsafe { transmute(n) }
  }
}

impl QueryKind {
  fn to_be_bytes(&self) -> [u8; 2] {
    (*self as u16).to_be_bytes()
  }
}

/// https://tools.ietf.org/rfc/rfc1035.txt
#[derive(Copy, Clone, Debug, PartialEq)]
#[repr(u16)]
pub enum QueryClass {
  IN = 1,
  CS = 2,
  CH = 3,
  HS = 4,
  Reserved,
}

impl QueryClass {
  fn to_be_bytes(&self) -> [u8; 2] {
    (*self as u16).to_be_bytes()
  }
}

impl From<u16> for QueryClass {
  fn from(n: u16) -> Self {
    unsafe { transmute(n) }
  }
}

impl fmt::Debug for DnsHeader {
  fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
    fmt.debug_struct("DnsHeader")
      .field("id", &self.id())
      .field("kind", &self.kind())
      .field("opcode", &self.opcode())
      .field("authoritative_answer", &self.authoritative_answer())
      .field("truncated", &self.truncated())
      .field("recursion_desired", &self.recursion_desired())
      .field("recursion_available", &self.recursion_available())
      .field("response_code", &self.response_code())
      .field("question_count", &self.question_count())
      .field("answer_count", &self.answer_count())
      .field("name_server_count", &self.name_server_count())
      .field("additional_records_count", &self.additional_records_count())
      .finish()
  }
}

impl DnsHeader {
  pub fn id(&self) -> u16 {
    u16::from_be_bytes(self.id)
  }

  pub fn set_id(&mut self, id: u16) {
    self.id = id.to_be_bytes()
  }

  pub fn kind(&self) -> HeaderKind {
    if (self.flags[0] & 0b10000000) == 0 {
      HeaderKind::Query
    } else {
      HeaderKind::Response
    }
  }

  pub fn set_kind(&mut self, kind: HeaderKind) {
    match kind {
      HeaderKind::Query    => self.flags[0] &= 0b01111111,
      HeaderKind::Response => self.flags[0] |= 0b10000000,
    }
  }

  pub fn opcode(&self) -> OpCode {
    match (self.flags[0] & 0b01111000) >> 3 {
      0 => OpCode::Query,
      1 => OpCode::InverseQuery,
      2 => OpCode::Status,
      4 => OpCode::Notify,
      5 => OpCode::Update,
      n => OpCode::Reserved(n),
    }
  }

  pub fn set_opcode(&mut self, opcode: OpCode) {
    self.flags[0] = (self.flags[0] & 0b10000111) | (match opcode {
      OpCode::Query => 0,
      OpCode::InverseQuery => 1,
      OpCode::Status => 2,
      OpCode::Notify => 4,
      OpCode::Update => 5,
      OpCode::Reserved(n) => n,
    } << 3);
  }

  pub fn authoritative_answer(&self) -> bool {
    (self.flags[0] & 0b00000100) != 0
  }

  pub fn truncated(&self) -> bool {
    (self.flags[0] & 0b00000010) != 0
  }

  pub fn recursion_desired(&self) -> bool {
    (self.flags[0] & 0b00000001) != 0
  }

  pub fn set_recursion_desired(&mut self, recursion_desired: bool) {
    if recursion_desired {
      self.flags[0] |= 0b00000001;
    } else {
      self.flags[0] &= 0b11111110;
    }
  }

  pub fn recursion_available(&self) -> bool {
    (self.flags[1] & 0b10000000) != 0
  }

  pub fn set_recursion_available(&mut self, recursion_available: bool) {
    if recursion_available {
      self.flags[1] |= 0b10000000;
    } else {
      self.flags[1] &= 0b01111111;
    }
  }

  pub fn response_code(&self) -> ResponseCode {
    u16::from(self.flags[1] & 0b00001111).into()
  }

  pub fn set_response_code(&mut self, response_code: ResponseCode) {
    self.flags[1] = (self.flags[1] & 0b11110000) | u16::from(response_code) as u8;
  }

  pub fn question_count(&self) -> u16 {
    u16::from_be_bytes(self.question_count)
  }

  pub fn set_question_count(&mut self, question_count: u16) {
    self.question_count = question_count.to_be_bytes();
  }

  pub fn answer_count(&self) -> u16 {
    u16::from_be_bytes(self.answer_count)
  }

  pub fn set_answer_count(&mut self, answer_count: u16) {
    self.answer_count = answer_count.to_be_bytes();
  }

  pub fn name_server_count(&self) -> u16 {
    u16::from_be_bytes(self.name_server_count)
  }

  pub fn set_name_server_count(&mut self, name_server_count: u16) {
    self.name_server_count = name_server_count.to_be_bytes();
  }

  pub fn additional_records_count(&self) -> u16 {
    u16::from_be_bytes(self.additional_records_count)
  }

  pub fn set_additional_records_count(&mut self, additional_records_count: u16) {
    self.additional_records_count = additional_records_count.to_be_bytes();
  }

  pub fn as_bytes(&self) -> &[u8] {
    unsafe { &*(self as *const _ as *const [u8; size_of::<Self>()]) }
  }
}
