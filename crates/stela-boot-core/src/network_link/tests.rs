use super::*;
use alloc::string::ToString;
use stela_runtime::net::NetError;

#[test]
fn hide_ssid_hides_every_occurrence_of_the_name() {
    // The connection keeper's failures name the network after the radio.
    assert_eq!(
        hide_ssid("rtw89: test not found", b"test"),
        "rtw89: <ssid> not found"
    );
    assert_eq!(
        hide_ssid(
            "rtw89: test offers no supported security: Akm; test",
            b"test"
        ),
        "rtw89: <ssid> offers no supported security: Akm; <ssid>"
    );
    // Byte for byte: another case, or part of the name, is not the name.
    for text in ["rtw89: TEST not found", "rtw89: tes not found", ""] {
        assert_eq!(hide_ssid(text, b"test"), text);
    }
    // A name of several-byte characters, and one that is not UTF-8, which
    // the keeper writes as from_utf8_lossy does.
    assert_eq!(
        hide_ssid("rtw89: café ü not found", "café ü".as_bytes()),
        "rtw89: <ssid> not found"
    );
    assert_eq!(
        hide_ssid("rtw89: caf\u{fffd} not found", b"caf\xe9"),
        "rtw89: <ssid> not found"
    );
    // An empty name, a wired adapter's, hides nothing.
    assert_eq!(hide_ssid("rtw89:  not found", b""), "rtw89:  not found");
}

#[test]
fn hide_ssid_hides_a_name_inside_other_words_too() {
    // The line gives up legibility, never the name.
    assert_eq!(
        hide_ssid("eapol rx 2; scan rx", b"rx"),
        "eapol <ssid> 2; scan <ssid>"
    );
    assert_eq!(
        hide_ssid("scans 1, assoc", b"s"),
        "<ssid>can<ssid> 1, a<ssid><ssid>oc"
    );
    // Overlapping occurrences are hidden as one, touching ones one each.
    assert_eq!(hide_ssid("x aaa y", b"aa"), "x <ssid> y");
    assert_eq!(hide_ssid("ababa", b"aba"), "<ssid>");
    assert_eq!(hide_ssid("abab", b"ab"), "<ssid><ssid>");
}

#[test]
fn hide_ssid_leaves_a_hidden_name_hidden() {
    // A `<ssid>` already written stays as it is, so hiding twice changes
    // nothing, even for a name that is part of it or is it.
    let text = "rtw89: test not found; <ssid>";
    for name in ["test", "s", "ssid", "id>", "<", "<ssid>"] {
        let once = hide_ssid(text, name.as_bytes());
        assert!(once.ends_with("; <ssid>"), "{name}: {once}");
        assert_eq!(hide_ssid(&once, name.as_bytes()), once, "{name}");
    }
    // A name longer than the placeholder is still hidden over it.
    assert_eq!(hide_ssid("<ssid> <ssid>x", b"<ssid>x"), "<ssid> <ssid>");
}

/// The boot's `NETWORK adapter … failed` line and its account on the
/// screen write the network stack's error, which carries the keeper's
/// failure as the link's.
#[test]
fn a_keeper_failure_reaches_the_boot_with_the_name_hidden() {
    let failure = hide_ssid("rtw89: test not found", b"test");
    let error = NetError::Link(failure);
    assert_eq!(error.to_string(), "network link: rtw89: <ssid> not found");
    assert_eq!(
        alloc::format!("0000:03:00.0: no address ({error})."),
        "0000:03:00.0: no address (network link: rtw89: <ssid> not found)."
    );
    assert!(!error.to_string().contains("test"));
}
