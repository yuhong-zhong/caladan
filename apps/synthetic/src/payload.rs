extern crate clap;

use crate::Buffer;
use crate::Connection;
use crate::LoadgenProtocol;
use crate::Packet;
use crate::Transport;
use clap::{App, Arg};

use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use std::io;
use std::io::Read;

pub struct Payload {
    pub work_iterations: u64,
    pub index: u64,
    pub randomness: u64,
    pub extra_payload_size: u64,
}

pub const PAYLOAD_SIZE: usize = 32;
pub const MAX_EXTRA_PAYLOAD_SIZE: usize = 8925;
pub const MAX_TOTAL_SIZE: usize = PAYLOAD_SIZE + MAX_EXTRA_PAYLOAD_SIZE;

#[derive(Clone, Copy)]
pub struct SyntheticProtocol {
    pub extra_payload_size: usize,
}

impl LoadgenProtocol for SyntheticProtocol {
    fn uses_ordered_requests(&self) -> bool {
        false
    }

    fn gen_req(&self, i: usize, p: &Packet, buf: &mut Vec<u8>) {
        Payload {
            work_iterations: p.work_iterations,
            index: i as u64,
            randomness: p.randomness,
            extra_payload_size: self.extra_payload_size as u64,
        }
        .serialize_into(buf)
        .unwrap();
    }

    fn read_response(&self, mut sock: &Connection, buf: &mut Buffer) -> io::Result<(usize, u64)> {
        let scratch = buf.get_empty_buf();
        sock.read_exact(&mut scratch[..PAYLOAD_SIZE])?;
        let payload = Payload::deserialize(&mut &scratch[..])?;
        Ok((payload.index as usize, payload.randomness))
    }
}

impl SyntheticProtocol {
    pub fn with_args(_matches: &clap::ArgMatches, _tport: Transport) -> Self {
        let extra_payload_size = value_t_or_exit!(_matches, "extra_payload_size", usize);
        assert!(extra_payload_size <= MAX_EXTRA_PAYLOAD_SIZE);
        SyntheticProtocol {
            extra_payload_size: extra_payload_size,
        }
    }

    pub fn args<'a, 'b>() -> Vec<clap::Arg<'a, 'b>> {
        vec![
            Arg::with_name("extra_payload_size")
                .long("extra_payload_size")
                .takes_value(true)
                .default_value("0")
                .help("Synthetic: size of extra payload"),
        ]
    }
}

impl Payload {
    pub fn serialize_into<W: io::Write>(&self, writer: &mut W) -> io::Result<()> {
        writer.write_u64::<BigEndian>(self.work_iterations)?;
        writer.write_u64::<BigEndian>(self.index)?;
        writer.write_u64::<BigEndian>(self.randomness)?;
        writer.write_u64::<BigEndian>(self.extra_payload_size)?;
        writer.write_all(&vec![0; self.extra_payload_size as usize])?;
        Ok(())
    }

    pub fn deserialize<R: io::Read>(reader: &mut R) -> io::Result<Payload> {
        let p = Payload {
            work_iterations: reader.read_u64::<BigEndian>()?,
            index: reader.read_u64::<BigEndian>()?,
            randomness: reader.read_u64::<BigEndian>()?,
            extra_payload_size: reader.read_u64::<BigEndian>()?,
        };
        return Ok(p);
    }
}
