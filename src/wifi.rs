//! Credentials and outcomes for the Scout's router Wi-Fi setup service.
//!
//! The released firmware interpolates credentials into a shell command and
//! quoted configuration strings. The accepted subset intentionally excludes
//! characters that cannot safely round-trip through those implementations.

use std::fmt;

/// Validated WPA/WPA2 personal credentials. Debug output never shows either field.
///
/// Open networks and raw 64-digit PSKs are not supported by this API. Passwords
/// are passphrases, and the firmware saves credentials using line-based storage.
#[derive(Clone)]
pub struct WifiCredentials {
    ssid: String,
    password: String,
}

impl WifiCredentials {
    /// Accepts an SSID of 1–32 bytes and a passphrase of 8–63 bytes.
    /// Both must be printable ASCII, excluding quotes and backslashes.
    pub fn new(
        ssid: impl Into<String>,
        password: impl Into<String>,
    ) -> Result<Self, WifiValidationError> {
        let ssid = ssid.into();
        let password = password.into();
        if !(1..=32).contains(&ssid.len()) {
            return Err(WifiValidationError("SSID must contain 1–32 bytes"));
        }
        if !(8..=63).contains(&password.len()) {
            return Err(WifiValidationError(
                "Wi-Fi passphrase must contain 8–63 bytes",
            ));
        }
        if !ssid.bytes().all(safe_firmware_byte) {
            return Err(WifiValidationError(
                "SSID must use printable ASCII without quotes or backslashes",
            ));
        }
        if !password.bytes().all(safe_firmware_byte) {
            return Err(WifiValidationError(
                "Wi-Fi passphrase must use printable ASCII without quotes or backslashes",
            ));
        }
        Ok(Self { ssid, password })
    }

    #[cfg(feature = "ros1")]
    pub(crate) fn encode_request(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(9 + self.ssid.len() + self.password.len());
        // wifi_switch_ssid_key.srv: int8 mode, string ssid, string key.
        // The public firmware enum uses 1 for station/router mode.
        bytes.push(1);
        crate::codec::write_string(&mut bytes, &self.ssid);
        crate::codec::write_string(&mut bytes, &self.password);
        bytes
    }
}

fn safe_firmware_byte(byte: u8) -> bool {
    (b' '..=b'~').contains(&byte) && !matches!(byte, b'\'' | b'"' | b'\\')
}

impl fmt::Debug for WifiCredentials {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Read the fields so this implementation also remains warning-free in
        // protocol-only builds, where no request can be encoded.
        let _ = (&self.ssid, &self.password);
        f.debug_struct("WifiCredentials")
            .field("ssid", &"[REDACTED]")
            .field("password", &"[REDACTED]")
            .finish()
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct WifiValidationError(&'static str);

impl fmt::Display for WifiValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.0)
    }
}

impl std::error::Error for WifiValidationError {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WifiProvisioningResult {
    /// The robot accepted the request. Router association is asynchronous and
    /// still needs to be checked after reconnecting this computer to the router.
    Accepted,
    /// Sending started, but no valid acknowledgement arrived. The robot may
    /// have switched networks; inspect connectivity before trying again.
    DeliveryUncertain,
}

/// Provisioning failures deliberately omit remote error strings, which could
/// contain credentials echoed by firmware.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WifiProvisioningError {
    InvalidConfiguration(&'static str),
    Timeout,
    Transport(std::io::ErrorKind),
    Protocol(&'static str),
    ServiceUnavailable,
    ServiceFailed,
    Rejected(i8),
}

impl fmt::Display for WifiProvisioningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidConfiguration(message) | Self::Protocol(message) => f.write_str(message),
            Self::Timeout => f.write_str("Wi-Fi setup timed out before sending credentials"),
            Self::Transport(kind) => write!(f, "Wi-Fi setup connection failed ({kind})"),
            Self::ServiceUnavailable => f.write_str("the Scout Wi-Fi setup service is unavailable"),
            Self::ServiceFailed => f.write_str("the Scout Wi-Fi setup service reported failure"),
            Self::Rejected(status) => write!(f, "the Scout rejected Wi-Fi setup (status {status})"),
        }
    }
}

impl std::error::Error for WifiProvisioningError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validates_firmware_safe_boundaries() {
        for length in [1, 32] {
            for password_length in [8, 63] {
                assert!(
                    WifiCredentials::new("s".repeat(length), "p".repeat(password_length)).is_ok()
                );
            }
        }
        for ssid in ["".to_owned(), "s".repeat(33)] {
            assert!(WifiCredentials::new(ssid, "password").is_err());
        }
        for password in ["".to_owned(), "p".repeat(7), "p".repeat(64)] {
            assert!(WifiCredentials::new("router", password).is_err());
        }
        for unsafe_char in ['\'', '"', '\\', '\n', '\r', '\0', '\t', '\u{7f}', 'é'] {
            assert!(WifiCredentials::new(format!("a{unsafe_char}b"), "password").is_err());
            assert!(WifiCredentials::new("router", format!("pass{unsafe_char}word")).is_err());
        }
        // These are safe inside the firmware's single-quoted shell arguments.
        assert!(WifiCredentials::new("Living Room", "$value;`text`!#()&").is_ok());
    }

    #[test]
    fn debug_and_validation_errors_redact_credentials() {
        let credentials = WifiCredentials::new("private-ssid", "secret-password").unwrap();
        let debug = format!("{credentials:?}");
        assert!(!debug.contains("private-ssid"));
        assert!(!debug.contains("secret-password"));
        let error = WifiCredentials::new("private-ssid", "secret'password").unwrap_err();
        assert!(!format!("{error:?}: {error}").contains("secret"));
    }

    #[cfg(feature = "ros1")]
    #[test]
    fn request_matches_the_firmware_service_field_order() {
        let bytes = WifiCredentials::new("router", "password")
            .unwrap()
            .encode_request();
        assert_eq!(bytes, b"\x01\x06\0\0\0router\x08\0\0\0password");
    }
}
