//! What a normal boot's network (`engine::network`) does with a host
//! adapter's link that needs no hardware, so it is tested here.
//!
//! A Wi-Fi adapter's link is a connection keeper's, whose failures name the
//! network (`rtw89: <name> not found`, `… offers no supported security`)
//! and reach the boot as the link's own errors: the network stack's, the
//! `NETWORK adapter … failed` line and the screen's account, a later
//! download's and the guest network's. [`hide_ssid`] takes the configured
//! name out of them where the boot receives them.
use alloc::string::String;

/// What a hidden network name reads as.
const HIDDEN: &str = "<ssid>";

/// `text` with the network's name `ssid` written `<ssid>` wherever it
/// occurs, byte for byte, as `String::from_utf8_lossy` writes it (the
/// connection keeper's failures name the network so): occurrences that
/// overlap are hidden as one, and a name that is also part of other words
/// is hidden there too, so the line gives up legibility, never the name. A
/// `<ssid>` already in `text` stays as it is, even for a name that is part
/// of it, so hiding twice changes nothing; an empty name hides nothing.
pub fn hide_ssid(text: &str, ssid: &[u8]) -> String {
    let name = String::from_utf8_lossy(ssid);
    let (name, bytes) = (name.as_bytes(), text.as_bytes());
    if name.is_empty() || name.len() > bytes.len() {
        return String::from(text);
    }
    // Whether `bytes[start..end]` lies inside a `<ssid>` of `text`.
    let placed = |start: usize, end: usize| {
        (start.saturating_sub(HIDDEN.len())..=start)
            .any(|at| at + HIDDEN.len() >= end && bytes[at..].starts_with(HIDDEN.as_bytes()))
    };
    let mut hidden = String::with_capacity(text.len());
    // `text[..copied]` is in `hidden`, written out or hidden.
    let mut copied = 0;
    // The occurrences being hidden as one: `text[start..end]`.
    let mut run: Option<(usize, usize)> = None;
    let mut flush = |hidden: &mut String, (start, end): (usize, usize)| {
        // An occurrence of UTF-8 in UTF-8 starts and ends on characters.
        hidden.push_str(&text[copied..start]);
        hidden.push_str(HIDDEN);
        copied = end;
    };
    for start in 0..=bytes.len() - name.len() {
        let end = start + name.len();
        if !bytes[start..].starts_with(name) || placed(start, end) {
            continue;
        }
        match &mut run {
            Some((_, until)) if start < *until => *until = end,
            _ => {
                if let Some(done) = run.replace((start, end)) {
                    flush(&mut hidden, done);
                }
            }
        }
    }
    if let Some(done) = run {
        flush(&mut hidden, done);
    }
    hidden.push_str(&text[copied..]);
    hidden
}

#[cfg(test)]
mod tests;
