//! Names used for the classroom Scout fleet.

/// A Scout whose network identifier is known to this project.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct KnownScout {
    pub name: &'static str,
    pub mac_address: &'static str,
}

/// The robot names and MAC addresses supplied with the classroom materials.
pub const KNOWN_SCOUTS: &[KnownScout] = &[
    KnownScout {
        name: "Bombur",
        mac_address: "d4:9c:dd:e9:ea:a0",
    },
    KnownScout {
        name: "Bofur",
        mac_address: "d4:9c:dd:ea:dc:3a",
    },
    KnownScout {
        name: "Bifur",
        mac_address: "d4:9c:dd:ea:fd:a4",
    },
    KnownScout {
        name: "Glóin",
        mac_address: "d4:9c:dd:ea:3b:16",
    },
    KnownScout {
        name: "Thráin",
        mac_address: "d4:9c:dd:e9:ea:60",
    },
    KnownScout {
        name: "Thrór",
        mac_address: "d4:9c:dd:ea:dc:a2",
    },
    KnownScout {
        name: "Thorin",
        mac_address: "b8:2d:28:56:92:0e",
    },
    KnownScout {
        name: "Balin",
        mac_address: "b8:2d:28:56:87:da",
    },
    KnownScout {
        name: "Dwalin",
        mac_address: "b8:2d:28:56:91:60",
    },
    KnownScout {
        name: "Fíli",
        mac_address: "d4:9c:dd:ea:fd:9c",
    },
    KnownScout {
        name: "Kíli",
        mac_address: "d4:9c:dd:eb:0c:f6",
    },
    KnownScout {
        name: "Dori",
        mac_address: "d4:9c:dd:e9:fa:2e",
    },
    KnownScout {
        name: "Nori",
        mac_address: "d4:9c:dd:ea:dc:8c",
    },
    KnownScout {
        name: "Ori",
        mac_address: "d4:9c:dd:ea:dc:56",
    },
    KnownScout {
        name: "Óin",
        mac_address: "d4:9c:dd:e9:b9:da",
    },
    KnownScout {
        name: "Dain",
        mac_address: "d4:9c:dd:ea:5a:d8",
    },
];

/// Finds a known Scout from a full MAC address or `robot_scout_XXXXXX` SSID.
pub fn known_scout(identifier: &str) -> Option<KnownScout> {
    let identifier = identifier.trim().to_ascii_lowercase();
    if let Some(mac) = normalize_mac(&identifier) {
        return KNOWN_SCOUTS
            .iter()
            .copied()
            .find(|scout| compact_mac(scout.mac_address) == mac);
    }

    let suffix = identifier.strip_prefix("robot_scout_")?;
    if suffix.len() != 6 || !suffix.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return None;
    }
    KNOWN_SCOUTS
        .iter()
        .copied()
        .find(|scout| compact_mac(scout.mac_address).ends_with(suffix))
}

/// Finds a known full MAC address or Scout SSID embedded in command output.
pub fn known_scout_in_text(text: &str) -> Option<KnownScout> {
    let text = text.to_ascii_lowercase();
    KNOWN_SCOUTS.iter().copied().find(|scout| {
        let colon = scout.mac_address;
        let hyphen = colon.replace(':', "-");
        let compact = compact_mac(colon);
        let ssid = format!("robot_scout_{}", &compact[compact.len() - 6..]);
        text.contains(colon)
            || text.contains(&hyphen)
            || text
                .split(|character: char| !character.is_ascii_hexdigit())
                .any(|token| token == compact)
            || text.contains(&ssid)
    })
}

fn compact_mac(mac: &str) -> String {
    mac.chars()
        .filter(|character| character.is_ascii_hexdigit())
        .collect()
}

fn normalize_mac(value: &str) -> Option<String> {
    if value.len() == 12 && value.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        return Some(value.to_owned());
    }
    if value.len() != 17 {
        return None;
    }
    let bytes = value.as_bytes();
    let separator = bytes[2];
    if !matches!(separator, b':' | b'-') {
        return None;
    }
    for index in [2, 5, 8, 11, 14] {
        if bytes[index] != separator {
            return None;
        }
    }
    if bytes
        .iter()
        .enumerate()
        .any(|(index, byte)| ![2, 5, 8, 11, 14].contains(&index) && !byte.is_ascii_hexdigit())
    {
        return None;
    }
    Some(compact_mac(value))
}

#[cfg(test)]
mod tests {
    use super::{known_scout, known_scout_in_text};

    #[test]
    fn recognizes_full_mac_addresses_and_ssids() {
        assert_eq!(known_scout("d4:9c:dd:eb:0c:f6").unwrap().name, "Kíli");
        assert_eq!(known_scout("D4-9C-DD-EB-0C-F6").unwrap().name, "Kíli");
        assert_eq!(known_scout("d49cddeb0cf6").unwrap().name, "Kíli");
        assert_eq!(known_scout("robot_scout_EB0CF6").unwrap().name, "Kíli");
        assert_eq!(known_scout("robot_scout_EA5AD8").unwrap().name, "Dain");
    }

    #[test]
    fn recognizes_identifiers_inside_system_command_output() {
        assert_eq!(
            known_scout_in_text("BSSID : d4-9c-dd-e9-ea-a0")
                .unwrap()
                .name,
            "Bombur"
        );
        assert_eq!(
            known_scout_in_text("SSID : robot_scout_5687DA")
                .unwrap()
                .name,
            "Balin"
        );
    }

    #[test]
    fn rejects_partial_or_malformed_identifiers() {
        assert!(known_scout("eb0cf6").is_none());
        assert!(known_scout("robot_scout_0cf6").is_none());
        assert!(known_scout("d4:9c:dd:eb:0c:f6:00").is_none());
        assert!(known_scout_in_text("unrelated deadbeef text").is_none());
    }
}
