// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! C2H parser: the header of `rtw89_fw_c2h_parse_attr` (`fw.c:9043`,
//! `fw.h:3926`), the class 0 REC_ACK, DONE_ACK and log handlers
//! (`mac.c:5459`, `:5495`, `:5559`) and the formatted-log test of
//! `rtw89_fw_log_dump` (`fw.c:2249`). Formatted logs are not rendered (the
//! firmware's format table is not ported). It grants no I/O.

const C2H_HEADER_LEN: usize = 8;
const ACK_LEN: usize = 12;
const C2H_CAT_MAC: u8 = 1;
const C2H_CLASS_INFO: u8 = 0;
const FUNC_REC_ACK: u8 = 0;
const FUNC_DONE_ACK: u8 = 1;
const FUNC_LOG: u8 = 2;
/// `RTW89_C2H_FW_LOG_SIGNATURE` and `RTW89_C2H_FW_FORMATTED_LOG_MIN_LEN`
/// (`fw.h:3993`, `:3990`).
const LOG_SIGNATURE: [u8; 2] = [0xA5, 0xA5];
const FORMATTED_LOG_MIN_LEN: usize = 11;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum C2h<'a> {
    RecAck {
        cat: u8,
        class: u8,
        func: u8,
        seq: u8,
    },
    DoneAck {
        cat: u8,
        class: u8,
        func: u8,
        ret: u8,
        seq: u8,
    },
    Log(&'a [u8]),
    FormattedLog,
    Other {
        cat: u8,
        class: u8,
        func: u8,
    },
}

/// `None` when the payload is shorter than the header, or than an ack's
/// 12 bytes.
pub fn parse(payload: &[u8]) -> Option<C2h<'_>> {
    let w0 = u32::from_le_bytes(payload.get(..4)?.try_into().ok()?);
    payload.get(4..C2H_HEADER_LEN)?;
    let cat = (w0 & 0x3) as u8;
    let class = ((w0 >> 2) & 0x3F) as u8;
    let func = ((w0 >> 8) & 0xFF) as u8;
    if cat != C2H_CAT_MAC || class != C2H_CLASS_INFO {
        return Some(C2h::Other { cat, class, func });
    }
    match func {
        FUNC_REC_ACK | FUNC_DONE_ACK => {
            let w2 = u32::from_le_bytes(payload.get(8..ACK_LEN)?.try_into().ok()?);
            let (cat, class, f) = (
                (w2 & 0x3) as u8,
                ((w2 >> 2) & 0x3F) as u8,
                ((w2 >> 8) & 0xFF) as u8,
            );
            Some(if func == FUNC_REC_ACK {
                // fw.h:3965-3974: H2C_SEQ is [23:16].
                C2h::RecAck {
                    cat,
                    class,
                    func: f,
                    seq: (w2 >> 16) as u8,
                }
            } else {
                // fw.h:3952-3963: H2C_RETURN [23:16], H2C_SEQ [31:24].
                C2h::DoneAck {
                    cat,
                    class,
                    func: f,
                    ret: (w2 >> 16) as u8,
                    seq: (w2 >> 24) as u8,
                }
            })
        }
        FUNC_LOG => {
            let body = &payload[C2H_HEADER_LEN..];
            if body.len() >= FORMATTED_LOG_MIN_LEN && body[..2] == LOG_SIGNATURE {
                Some(C2h::FormattedLog)
            } else {
                Some(C2h::Log(body))
            }
        }
        _ => Some(C2h::Other { cat, class, func }),
    }
}

#[cfg(test)]
mod tests;
