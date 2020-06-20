use std::mem::MaybeUninit;
use std::net::Ipv4Addr;

use esp_idf_bindgen::{esp_netif_get_ip_info, esp_netif_get_handle_from_ifkey, esp_netif_ip_info_t};

use crate::assert_esp_ok;

#[derive(Debug)]
pub struct IpInfo {
  ip: Ipv4Addr,
  netmask: Ipv4Addr,
  gateway: Ipv4Addr,
}

impl IpInfo {
  pub fn ip(&self) -> &Ipv4Addr {
    &self.ip
  }

  pub fn netmask(&self) -> &Ipv4Addr {
    &self.netmask
  }

  pub fn gateway(&self) -> &Ipv4Addr {
    &self.gateway
  }
}

impl IpInfo {
  pub fn sta() -> Option<Self> {
    Self::get_ip_info(b"WIFI_STA_DEF\0")
  }

  pub fn ap() -> Option<Self> {
    Self::get_ip_info(b"WIFI_AP_DEF\0")
  }

  pub(crate) unsafe fn from_native_unchecked(ip_info: esp_netif_ip_info_t) -> Self {
    IpInfo {
      ip: u32::from_be(ip_info.ip.addr).into(),
      netmask: u32::from_be(ip_info.netmask.addr).into(),
      gateway: u32::from_be(ip_info.gw.addr).into(),
    }
  }

  pub(crate) fn from_native(ip_info: esp_netif_ip_info_t) -> Option<Self> {
    if ip_info.ip.addr == 0 && ip_info.netmask.addr == 0 && ip_info.gw.addr == 0 {
      return None;
    }

    let ip = u32::from_be(ip_info.ip.addr);
    let netmask = u32::from_be(ip_info.netmask.addr);
    let gateway = u32::from_be(ip_info.gw.addr);

    if ip == 0 && netmask == 0 && gateway == 0 {
      return None;
    }

    Some(unsafe { Self::from_native_unchecked(ip_info) })
  }

  fn get_ip_info(key: &[u8]) -> Option<Self> {
    let mut ip_info = MaybeUninit::<esp_netif_ip_info_t>::uninit();
    let interface = unsafe { esp_netif_get_handle_from_ifkey(key.as_ptr() as *const _) };
    assert_esp_ok!(esp_netif_get_ip_info(interface, ip_info.as_mut_ptr()));
    Self::from_native(unsafe { ip_info.assume_init() })
  }
}
